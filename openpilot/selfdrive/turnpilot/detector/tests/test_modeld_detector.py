import time
from collections import deque
from contextlib import contextmanager

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.turnpilot.detector.decoder import PERSON, TRAFFIC_LIGHT
from openpilot.selfdrive.turnpilot.detector.graph import letterbox
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT, LightHead
from openpilot.selfdrive.turnpilot.detector.modeld_detector import (BUDGET_WARN_S, CYCLE_S, DEBT_CAP_S, LIGHT_BUDGET_S, PAINT_BUDGET_S,
                                                                    RETRY_S, RUN_EVERY, STEP_BUDGET_S, TIME_WINDOW, WIDE_HOLD_S,
                                                                    LanePaintStatus, LightStatus, ModeldDetector, camera_lights_mode)
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint


class FakeOutput:
  def __init__(self, raw):
    self.raw = raw

  def numpy(self):
    return self.raw


class FakeRun:
  """Stands in for the compiled jit: records which frame it got, returns one person."""
  def __init__(self, lb):
    self.calls, self.fail = [], False
    self.raw = np.zeros((1, 300, 6), np.float32)
    self.raw[0, 0] = [*lb.to_input(100, 200), *lb.to_input(160, 400), 0.9, PERSON]
    self.raw[0, 1] = [*lb.to_input(700, 100), *lb.to_input(715, 145), 0.8, TRAFFIC_LIGHT]

  def __call__(self, input_frame):
    if self.fail:
      raise RuntimeError("gpu fell over")
    self.calls.append(input_frame)
    return FakeOutput(self.raw)


class Meta:
  def __init__(self, frame_id):
    self.frame_id, self.timestamp_eof = frame_id, frame_id * 50_000_000


class FakeLanePaint:
  """Stands in for LanePaintReader: records the frame and boxes it got."""
  def __init__(self):
    self.calls, self.fail, self.delay = [], False, 0.

  def read(self, frame, meta, boxes):
    if self.fail:
      raise RuntimeError("bad frame")
    if self.delay:
      time.sleep(self.delay)
    self.calls.append((frame, meta.frame_id, None if boxes is None else len(boxes)))
    return [UNKNOWN, LinePaint('broken', 'white', 0.9, 0.3), LinePaint('solid', 'yellow', 0.8, 0.95), UNKNOWN], LanePaintStatus.read


class FakeLights:
  """Stands in for CameraLightReader: records the frame and light boxes it got, returns one governing red head (heads: what
  it returns instead)."""
  def __init__(self, heads=None):
    self.calls, self.fail, self.delay = [], False, 0.
    self.heads = heads or [LightHead(3, (0.52, 0.13, 0.53, 0.19), 0.8, 'red', 4, 'red', 'read', 1.5, 32., True)]

  def read(self, frame, meta, boxes):
    if self.fail:
      raise RuntimeError("bad frame")
    if self.delay:
      time.sleep(self.delay)
    self.calls.append((frame, meta.frame_id, boxes[:, 5].tolist()))
    return list(self.heads), LightStatus.read


class FakeModelState:
  frames = ['host0', 'host1']
  chestnut = False  # the small model's device: no native pkl, so a native load is the far crop


class PM:
  def __init__(self):
    self.sent = []

  def send(self, name, msg):
    self.sent.append(msg.as_reader().turnpilotDetections)


def make_detector(cameras=None, det_time=0.005):
  det = ModeldDetector.__new__(ModeldDetector)  # skips loading a pkl
  det.lb = letterbox(1344, 760)
  det.run = FakeRun(det.lb)
  det.model_state = FakeModelState()
  det.cameras = cameras or {'road': 0, 'wide': 1}
  det.lane_paint = FakeLanePaint() if 'road' in det.cameras else None
  det.frames = {cam: f'frame{i}' for cam, i in det.cameras.items()}
  det.times = deque([det_time])
  det.paint_times = deque(maxlen=TIME_WINDOW)
  det.light_times = deque(maxlen=TIME_WINDOW)
  det.since_run, det.wide_until = RUN_EVERY, -1.0
  det.bus = None  # __init__'s frame_bus, which the far crop's, the side views' and the native loads take
  return det


def with_lights(det, mode='shadow'):
  det.light_mode, det.lights = mode, FakeLights()
  return det


class SimClock:
  """modeld_detector's time module stood in: monotonic and perf_counter both read t, which only the test moves (tick), so a
  budget test's runs take no host time and its room is its estimates' arithmetic, whatever the host's load (11-06 review:
  a 5 ms stall on a loaded host ate the 4 ms test_a_paint_skipped_for_room_is_measured_again left, and its run never came)."""
  def __init__(self, t: float = 1000.):
    self.t = t

  def monotonic(self) -> float:
    return self.t

  def perf_counter(self) -> float:
    return self.t

  def tick(self, s: float = CYCLE_S) -> float:
    self.t += s
    return self.t


def costed(det, clock: SimClock, det_s: float, paint_s: float | None = None) -> None:
  """det's runs (and its lane paint reads) take det_s (paint_s) on the simulated clock, so their measured times are those."""
  def costs(fn, s):
    def run(*args, **kwargs):
      clock.tick(s)
      return fn(*args, **kwargs)
    return run
  det.run = costs(det.run, det_s)
  if paint_s is not None:
    det.lane_paint.read = costs(det.lane_paint.read, paint_s)


@contextmanager
def sim_clock(t: float = 1000.):
  """A SimClock as modeld_detector's time for the block (the detectors and far crops made in it take their load times off it)."""
  from unittest import mock
  from openpilot.selfdrive.turnpilot.detector import modeld_detector
  clock = SimClock(t)
  with mock.patch.object(modeld_detector, 'time', clock):
    yield clock


class TestModeldDetector(OpenpilotTestCase):
  def test_camera_choice(self):
    det = make_detector()
    assert det.choose_camera(0.001, log.Desire.none, 0.0) == 'road'
    assert det.choose_camera(-0.02, log.Desire.none, 1.0) == 'wide'
    assert det.choose_camera(0.0, log.Desire.none, 1.0 + WIDE_HOLD_S - 0.1) == 'wide'  # held after the turn
    assert det.choose_camera(0.0, log.Desire.none, 1.0 + WIDE_HOLD_S + 0.1) == 'road'
    # a turn desire before the car turns alternates the cameras 1:1, the wide first: it no longer holds the wide camera (11-06,
    # 1e 8+16: a signalized right's approach was wide only, no light read)
    assert [det.choose_camera(0.0, log.Desire.turnRight, 10.0 + 0.1 * k) for k in range(4)] == ['wide', 'road'] * 2
    assert det.choose_camera(0.0, log.Desire.laneChangeLeft, 20.0) == 'road'
    assert make_detector({'wide': 0}).choose_camera(0.0, log.Desire.none, 0.0) == 'wide'
    assert make_detector({'road': 0}).choose_camera(0.05, log.Desire.turnLeft, 0.0) == 'road'

  def test_the_plans_view_overrides_the_turns_wide_hold(self):
    # CRITIQUE REQUIRED 3: from an unprotected left's go to its commit the road camera only, whatever the turn desire or the
    # wide hold after it; held at its line, the road camera with the wide camera one run in WIDE_EVERY (its exit crosswalk)
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import WIDE_EVERY
    det = make_detector()
    assert det.choose_camera(0.05, log.Desire.turnLeft, 0.0, 'road') == 'road'
    assert det.choose_camera(0.0, log.Desire.none, 0.5, 'road') == 'road'  # inside the turn desire's wide hold
    picks = [det.choose_camera(0.05, log.Desire.turnLeft, 1.0 + 0.1 * k, 'roadAndWide') for k in range(3 * WIDE_EVERY)]
    assert picks.count('wide') == 3 and picks.count('road') == 3 * (WIDE_EVERY - 1) and WIDE_EVERY >= 3
    assert 'wide' not in picks[:WIDE_EVERY - 1]  # the road camera first
    # auto: as before
    assert det.choose_camera(0.05, log.Desire.turnLeft, 10.0, 'auto') == 'wide'
    # a device with one camera runs it, whatever the view
    assert make_detector({'wide': 0}).choose_camera(0.0, log.Desire.none, 0.0, 'road') == 'wide'

  def test_at_a_standstill_and_a_crawl_the_full_pass_keeps_the_wide_camera(self):
    # SAT_VALIDATION rows 6 and X3: at a standstill the full pass went to the road camera and a walker standing 2.8 m ahead,
    # below its view, was lost. At or under WIDE_SLOW_V the wide camera is the full pass's, but for one run in
    # WIDE_SLOW_ROAD_EVERY on the road camera (its light and lane paint reads, which turnpilotd ages out at CAMERA_STALE_S);
    # at speed the choice is as before; an unknown speed is as before
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import WIDE_SLOW_ROAD_EVERY, WIDE_SLOW_V
    from openpilot.selfdrive.turnpilot.map_controls import CAMERA_STALE_S
    assert WIDE_SLOW_V >= 3. - 1e-6 and WIDE_SLOW_ROAD_EVERY >= 3
    for v in (0., 1.5, WIDE_SLOW_V):
      det = make_detector()
      picks = [det.choose_camera(0.0, log.Desire.none, 0.1 * k, v_ego=v) for k in range(3 * WIDE_SLOW_ROAD_EVERY)]
      assert picks[0] == 'wide' and picks.count('road') == 3 and picks.count('wide') == 3 * (WIDE_SLOW_ROAD_EVERY - 1), v
      assert WIDE_SLOW_ROAD_EVERY * RUN_EVERY * 0.05 < CAMERA_STALE_S  # the camera's light read never ages out between
    det = make_detector()
    assert [det.choose_camera(0.0, log.Desire.none, 0.1 * k, v_ego=WIDE_SLOW_V + 0.5) for k in range(6)] == ['road'] * 6
    assert det.choose_camera(-0.02, log.Desire.none, 1.0, v_ego=10.) == 'wide'  # a curve at speed: as before
    det = make_detector()
    assert [det.choose_camera(0.0, log.Desire.none, 0.1 * k) for k in range(3)] == ['road'] * 3
    assert det.choose_camera(0.0, log.Desire.none, 1.0, 'road', v_ego=0.) == 'road'  # the plan's view still decides
    det, pm = make_detector(), PM()
    for k in range(2 * RUN_EVERY):
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(100 + k), Meta(200 + k)], v_ego=0.)
    assert [str(td.camera) for td in pm.sent] == ['wide', 'wide']

  def test_a_standstill_curvature_never_holds_the_wide_camera(self):
    # 11-06, 1e 3+10 to 3+36: standing 20 m from a signal's line, modeld's desiredCurvature frozen at 0.0117 (below
    # MIN_LAT_CONTROL_SPEED it holds its last value), over WIDE_CURVATURE: 0 road runs against 259 wide, no light read
    # through the wait. Below MOVING_V the curvature starts no wide hold: the slow cycle's one road run in 3
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import MOVING_V, WIDE_SLOW_ROAD_EVERY
    assert MOVING_V == 0.3 and WIDE_SLOW_ROAD_EVERY == 3
    det = make_detector()
    picks = [det.choose_camera(0.0117, log.Desire.none, 0.1 * k, v_ego=0.) for k in range(9)]
    assert picks == ['wide', 'wide', 'road'] * 3
    # a real turn (moving) holds the wide camera; standing after it with its curvature frozen, the hold runs out at
    # WIDE_HOLD_S and the slow cycle is back
    det = make_detector()
    assert [det.choose_camera(0.05, log.Desire.none, 0.1 * k, v_ego=4.) for k in range(5)] == ['wide'] * 5
    held = [det.choose_camera(0.05, log.Desire.none, 0.5 + 0.1 * k, v_ego=0.) for k in range(int(WIDE_HOLD_S / 0.1) + 6)]
    assert held[:int(WIDE_HOLD_S / 0.1) - 1] == ['wide'] * (int(WIDE_HOLD_S / 0.1) - 1) and 'road' in held[-6:]
    # speed unknown: a curvature is a turn, as before
    assert make_detector().choose_camera(0.0117, log.Desire.none, 0.) == 'wide'

  def test_near_a_signal_the_light_reader_gets_every_other_run(self):
    # the table's lights_near rows (choose_camera; a signal mapped near with the light reader reading): standing or at a
    # crawl, and in a turn, the road and wide cameras 1:1 (the wide first), so the light reader reads 5 Hz where a red must
    # stop the car and the wide camera keeps every other run for walkers at the corner; at speed the road camera as before
    det = make_detector()
    det.lights_near = True
    assert [det.choose_camera(0.0117, log.Desire.none, 0.1 * k, v_ego=0.) for k in range(6)] == ['wide', 'road'] * 3
    det = make_detector()
    det.lights_near = True
    assert [det.choose_camera(0.05, log.Desire.none, 0.1 * k, v_ego=4.) for k in range(6)] == ['wide', 'road'] * 3
    det = make_detector()  # no signal: a turn is the wide camera's
    assert [det.choose_camera(0.05, log.Desire.none, 0.1 * k, v_ego=4.) for k in range(6)] == ['wide'] * 6
    det = make_detector()
    det.lights_near = True
    assert [det.choose_camera(0.0, log.Desire.none, 0.1 * k, v_ego=12.) for k in range(4)] == ['road'] * 4
    # the plan's views and the sweeps still decide
    assert det.choose_camera(0.05, log.Desire.turnLeft, 1., 'road', v_ego=4.) == 'road'
    assert [det.choose_camera(0., log.Desire.none, 2. + 0.1 * k, 'stopSweepGo', v_ego=0.) for k in range(3)] == ['wide'] * 3

  def test_a_turn_desire_before_the_turn_alternates_the_cameras(self):
    # 1e 8+16: the turnRight desire went out at 39 m with a signal 32 m ahead, and every run to the end of the stand was
    # wide, no light read. A turn desire alone no longer holds the wide camera: 1:1 until the car turns (its curvature,
    # moving), then the turn's wide hold
    det = make_detector()
    picks = [det.choose_camera(0.002, log.Desire.turnRight, 0.1 * k, v_ego=7.) for k in range(6)]
    assert picks == ['wide', 'road'] * 3
    assert [det.choose_camera(0.04, log.Desire.turnRight, 1. + 0.1 * k, v_ego=5.) for k in range(4)] == ['wide'] * 4
    det = make_detector()  # standing at the line with the desire out and the curvature frozen: 1:1 too
    assert [det.choose_camera(0.013, log.Desire.turnLeft, 0.1 * k, v_ego=0.) for k in range(4)] == ['wide', 'road'] * 2

  def test_a_peek_never_advances_the_cycles(self):
    for v, near, curv, desire in ((0., False, 0.0117, log.Desire.none), (0., True, 0.0117, log.Desire.none),
                                  (7., False, 0.002, log.Desire.turnRight), (4., True, 0.05, log.Desire.none)):
      det = make_detector()
      det.lights_near = near
      picks = []
      for k in range(6):
        state = (det.slow_runs, det.alt_runs, det.wide_until, det.view_runs, det.sweep_runs)
        peeks = {det.choose_camera(curv, desire, 0.1 * k, v_ego=v, peek=True) for _ in range(3)}
        assert (det.slow_runs, det.alt_runs, det.wide_until, det.view_runs, det.sweep_runs) == state
        picks.append(det.choose_camera(curv, desire, 0.1 * k, v_ego=v))
        assert peeks == {picks[-1]}, (v, near, k)  # the peek is the pick

  def test_the_peek_takes_this_steps_signal_near(self):
    # signal_near (and lights_near with it) is set before the peek (the camera the time budget estimates for): standing on
    # the slow cycle two runs in, the step a signal comes near takes the 1:1 alternation's wide run, and the peek says so,
    # not the slow cycle's road run with the step before's
    det, pm = with_lights(make_detector(), 'addOnly'), PM()
    for k in range(2 * RUN_EVERY):
      det.step(pm, det.model_state, 0.010, 0.0117, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=0.)
    assert [str(td.camera) for td in pm.sent] == ['wide', 'wide'] and det.slow_runs == 2 and not det.lights_near
    det.step(pm, det.model_state, 0.010, 0.0117, log.Desire.none, [Meta(50), Meta(51)], v_ego=0., signal_near=True)
    assert det.next_cam == str(pm.sent[-1].camera) == 'wide' and det.signal_near and det.lights_near

  def test_a_test_stand_in_without_the_new_cycles_still_picks_at_speed(self):
    # test_turnpilotd's stand-in (frames, wide_until, sweep_runs, view_runs) at road speed, straight: no attribute it lacks
    # is read on that row
    from types import SimpleNamespace
    det = SimpleNamespace(frames={'road': 0, 'wide': 1}, wide_until=-1., sweep_runs=0, view_runs=0)
    assert [ModeldDetector.choose_camera(det, 0., log.Desire.none, 0.1 * k, v_ego=9.) for k in range(4)] == ['road'] * 4

  def test_step_takes_the_view(self):
    det, pm = make_detector(), PM()
    for k in range(3 * RUN_EVERY):
      det.step(pm, det.model_state, 0.010, 0.05, log.Desire.turnLeft, [Meta(100 + k), Meta(200 + k)], view='road')
    assert det.run.calls == ['frame0'] * 3 and all(str(td.camera) == 'road' for td in pm.sent)

  def test_every_nth_frame_and_message(self):
    det, pm = make_detector(), PM()
    for k in range(3 * RUN_EVERY):
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(100 + k), Meta(200 + k)])
    assert len(pm.sent) == 3 and det.run.calls == ['frame0'] * 3
    td = pm.sent[-1]
    assert str(td.camera) == 'road' and td.frameId == 100 + 2 * RUN_EVERY and len(td.detections) == 1
    np.testing.assert_allclose([td.detections[0].x0, td.detections[0].y1], [100, 400], atol=1e-3)

  def test_wide_uses_extra_frame(self):
    det, pm = make_detector(), PM()
    det.step(pm, det.model_state, 0.010, 0.03, log.Desire.none, [Meta(1), Meta(2)])
    assert det.run.calls == ['frame1'] and str(pm.sent[0].camera) == 'wide' and pm.sent[0].frameId == 2

  def test_budget(self):
    # a run only where the debt it leaves (the step past STEP_BUDGET_S) stays within DEBT_CAP_S: a 10 ms detector beside a
    # model 1 ms under the budget leaves 9 ms; 3 ms under, 7. The model alone fits, so no debt builds while it waits. On the
    # simulated clock (SimClock), so the host's own time never decides it
    with sim_clock() as clock:
      det, pm = make_detector(det_time=0.010), PM()
      for _ in range(10):
        clock.tick()
        det.step(pm, det.model_state, STEP_BUDGET_S - 0.001, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert pm.sent == [] and det.debt == 0.  # no room this step, every step
      clock.tick()
      det.step(pm, det.model_state, STEP_BUDGET_S - 0.003, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert len(pm.sent) == 1  # the first frame with room runs, even if it's not on the N-frame grid
    assert 0.010 - 0.003 <= DEBT_CAP_S < 0.010 - 0.001

  def test_failure_retries_and_a_wrong_model_raises(self):
    det, pm = make_detector(), PM()
    det.run.fail = True
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert det.failures == 1 and det.retry_at > 0. and pm.sent == []  # test_detector_recovery.py has the backoff
    det = make_detector()
    with pytest.raises(ValueError):  # no quiet off: DetectorHost logs it and retries
      det.step(pm, object(), 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert pm.sent == []

  def test_lane_paint_rides_along_on_the_road_frame(self):
    det, pm = make_detector(), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(7), Meta(8)])
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(9), Meta(10)])  # not on the N-frame grid
    det.since_run = RUN_EVERY
    det.step(pm, det.model_state, 0.010, 0.03, log.Desire.none, [Meta(11), Meta(12)])  # the detector looks through the wide
    # the road camera's host frame, behind the detector's boxes on it; nothing while it looks through the wide camera, where
    # there are no boxes for the road frame
    assert det.lane_paint.calls == [('host0', 7, 1)]
    assert len(pm.sent) == 2 and str(pm.sent[1].camera) == 'wide' and len(pm.sent[1].lanePaint) == 0
    paint = pm.sent[0].lanePaint
    assert [(str(p.kind), str(p.color)) for p in paint] == [('unknown', 'unknown'), ('broken', 'white'), ('solid', 'yellow'),
                                                           ('unknown', 'unknown')]
    assert abs(paint[1].confidence - 0.9) < 1e-6
    assert pm.sent[0].lanePaintStatus == LanePaintStatus.read and pm.sent[1].lanePaintStatus == LanePaintStatus.wideCamera
    assert pm.sent[0].executionTime > 0. and pm.sent[0].lanePaintTime >= 0.

  def test_lane_paint_failure_is_retried_and_the_detector_keeps_going(self):
    # never off for the drive (the no-features-off rule): a failed read is logged, the runs inside its backoff say retrying
    # (unknown paint: turnpilotd plans no lane change on it), and after RETRY_S, doubling, it is read again
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    det, pm = make_detector(), PM()
    det.lane_paint.fail = True
    with mock.patch.object(modeld_detector.cloudlog, 'exception') as logged:
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert det.lane_paint is not None and len(pm.sent) == 1 and len(pm.sent[0].detections) == 1
      assert len(pm.sent[0].lanePaint) == 0 and pm.sent[0].lanePaintStatus == LanePaintStatus.retrying
      assert det.paint_failures == 1 and RETRY_S - 0.1 < det.paint_retry_at - time.monotonic() <= RETRY_S
      det.lane_paint.fail = False
      det.since_run = RUN_EVERY
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(3), Meta(4)])
      assert pm.sent[1].lanePaintStatus == LanePaintStatus.retrying and det.lane_paint.calls == []  # inside the backoff
      det.paint_retry_at = 0.  # the backoff over
      det.since_run = RUN_EVERY
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(5), Meta(6)])
      assert pm.sent[2].lanePaintStatus == LanePaintStatus.read and len(pm.sent[2].lanePaint) == 4 and det.paint_failures == 0
      # failures in a row double the wait, each logged
      det.lane_paint.fail = True
      for k in range(2):
        det.paint_retry_at, det.since_run = 0., RUN_EVERY
        det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(7 + 2 * k), Meta(8 + 2 * k)])
      assert det.paint_failures == 2 and det.paint_retry_at - time.monotonic() > 2 * RETRY_S - 0.1
    assert sum('lane paint failed' in c.args[0] for c in logged.call_args_list) == 3
    # no road camera, no paint
    det, pm = make_detector({'wide': 0}), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(1)])
    assert len(pm.sent) == 1 and len(pm.sent[0].lanePaint) == 0 and pm.sent[0].lanePaintStatus == LanePaintStatus.disabled

  def test_slow_lane_paint_stays_on_and_is_warned_once_a_minute(self):
    # 11-06: paint medians peaked at 4.29-4.59 ms against the 5.0 ms that turned it off for the drive. A median over
    # PAINT_BUDGET_S is warned at most once in BUDGET_WARN_S and the paint stays on, its time in the estimate
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    det, pm = make_detector(), PM()
    det.lane_paint.delay = PAINT_BUDGET_S * 1.5
    with mock.patch.object(modeld_detector.cloudlog, 'warning') as warned:
      for k in range(TIME_WINDOW + 3):
        det.since_run, det.debt = RUN_EVERY, 0.
        det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)])
      assert sum('lane paint over budget' in c.args[0] for c in warned.call_args_list) == 1
      det.paint_warn_t -= BUDGET_WARN_S  # a minute on: warned again
      det.since_run, det.debt = RUN_EVERY, 0.
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(100), Meta(101)])
      assert sum('lane paint over budget' in c.args[0] for c in warned.call_args_list) == 2
    assert det.lane_paint is not None and len(pm.sent) == TIME_WINDOW + 4
    assert all(m.lanePaintStatus == LanePaintStatus.read and m.lanePaintTime > PAINT_BUDGET_S for m in pm.sent)

  def test_paint_time_counts_toward_the_step_budget_and_the_lights_go_first(self):
    # on the simulated clock (SimClock), the runs taking their stated times: the host's own time never decides a step's room
    with sim_clock() as clock:
      det, pm = make_detector(det_time=0.006), PM()
      costed(det, clock, 0.006, 0.003)
      det.paint_times.extend([0.003] * 3)
      # 6 ms detector + 3 ms paint past the budget: a 9 ms debt, past DEBT_CAP_S (8 ms); without the paint, 6: the run goes,
      # its paint skipped for room (noRoom), and the paint's read time stays as it was
      clock.tick()
      det.step(pm, det.model_state, STEP_BUDGET_S, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert len(pm.sent) == 1 and pm.sent[0].lanePaintStatus == LanePaintStatus.noRoom and det.lane_paint.calls == []
      assert len(pm.sent[0].detections) == 1 and len(pm.sent[0].lanePaint) == 0 and list(det.paint_times) == [0.003] * 3
      assert abs(det.debt - 0.006) < 1e-9  # the run's 6 ms past the budget, carried
      # with room for both, both: the frames between paid the debt back, and 2 ms under the budget the detector and the
      # paint leave 7 ms
      det.since_run, det.debt = RUN_EVERY, 0.
      clock.tick()
      det.step(pm, det.model_state, STEP_BUDGET_S - 0.002, 0.0, log.Desire.none, [Meta(3), Meta(4)])
      assert len(pm.sent) == 2 and pm.sent[1].lanePaintStatus == LanePaintStatus.read and len(det.lane_paint.calls) == 1
      assert abs(det.debt - 0.007) < 1e-9
      # a step that starts 1.5 ms late (the debt carried) has room for the run without the paint only: 8.5 ms with it
      det.since_run, det.debt = RUN_EVERY, 0.0015
      clock.tick()
      det.step(pm, det.model_state, STEP_BUDGET_S - 0.002, 0.0, log.Desire.none, [Meta(5), Meta(6)])
      assert len(pm.sent) == 3 and pm.sent[2].lanePaintStatus == LanePaintStatus.noRoom and len(det.lane_paint.calls) == 1
      # with room for neither: no run, as before (the forced run measures again, with the paint)
      det, pm = make_detector(det_time=0.009), PM()
      det.paint_times.extend([0.003] * 3)
      clock.tick()
      det.step(pm, det.model_state, STEP_BUDGET_S, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert pm.sent == []

  def test_a_paint_skipped_for_room_is_measured_again(self):
    # a burst of slow reads can't keep the paint skipped: PAINT_FORCE_S skipped for room, the next road run reads it anyway,
    # its estimate starting over from that read, which then fits. Driven at 20 Hz on the simulated clock (SimClock): a 6 ms
    # detector beside a model 2 ms under the budget leaves 4 ms with no paint and 16 with the burst's, whatever the host's load
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import PAINT_FORCE_S
    with sim_clock() as clock:
      det, pm = make_detector(det_time=0.006), PM()
      costed(det, clock, 0.006, 0.003)  # the burst is over: a read now takes 3 ms
      det.paint_times.extend([0.012] * TIME_WINDOW)  # the burst
      t0, k, first_skip = clock.t, 0, None
      while not det.lane_paint.calls and clock.t - t0 < 2 * PAINT_FORCE_S:
        det.since_run, det.debt = RUN_EVERY, 0.  # a full run every RUN_EVERY frames, the frames between paying back its debt
        start = clock.tick(RUN_EVERY * CYCLE_S)
        det.step(pm, det.model_state, STEP_BUDGET_S - 0.002, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=10.)
        first_skip = det.paint_skipped_t if first_skip is None else first_skip
        k += 1
      # every road run until then skipped it for room, its estimate kept; the read came PAINT_FORCE_S after the first skip
      assert [td.lanePaintStatus for td in pm.sent[:-1]] == [LanePaintStatus.noRoom] * (len(pm.sent) - 1) and len(pm.sent) > 10
      assert PAINT_FORCE_S <= start - first_skip <= PAINT_FORCE_S + RUN_EVERY * CYCLE_S + 0.006 + 1e-9
      assert pm.sent[-1].lanePaintStatus == LanePaintStatus.read and len(det.lane_paint.calls) == 1 and len(det.paint_times) == 1
      assert abs(det.paint_times[0] - 0.003) < 1e-9
      assert det.paint_skipped_t is None  # read: the next skip for room starts its clock over
      det.since_run, det.debt = RUN_EVERY, 0.
      clock.tick(RUN_EVERY * CYCLE_S)
      det.step(pm, det.model_state, STEP_BUDGET_S - 0.002, 0.0, log.Desire.none, [Meta(500), Meta(501)], v_ego=10.)
      assert pm.sent[-1].lanePaintStatus == LanePaintStatus.read and len(det.lane_paint.calls) == 2  # the fast read's estimate fits


# the car at a junction (11-06, 1e 11+1.7 to 11+9.9, standing): the big model 39 ms, the full pass 10.4 ms, the lane paint
# 3.4 ms, the lights 3.5 ms a read at a junction's heads. Together they had no room (a 9.3 ms debt), so the road runs went
# forced only, 0.8-1.0 s apart, and the camera's light read aged out between them (CAMERA_STALE_S, 1 s)
JUNCTION_MODEL_S, JUNCTION_DET_S, JUNCTION_PAINT_S, JUNCTION_LIGHT_S = 0.039, 0.0104, 0.0034, 0.0035


def junction_drive(monkeypatch, frames, v_ego, curvature, signal_near, desire=log.Desire.none):
  """A drive on a simulated clock (modeld_detector.time) with every run taking the junction's times: the messages and, per
  frame, the camera of a full run on it (None: none)."""
  from types import SimpleNamespace
  from openpilot.selfdrive.turnpilot.detector import modeld_detector
  clock = SimpleNamespace(t=1000.)
  monkeypatch.setattr(modeld_detector, 'time', SimpleNamespace(monotonic=lambda: clock.t, perf_counter=lambda: clock.t))

  def costs(fn, s):
    def run(*args, **kwargs):
      clock.t += s
      return fn(*args, **kwargs)
    return run
  det, pm = with_lights(make_detector(det_time=JUNCTION_DET_S), 'addOnly'), PM()
  det.paint_times.extend([JUNCTION_PAINT_S] * TIME_WINDOW)
  det.light_times.extend([JUNCTION_LIGHT_S] * TIME_WINDOW)
  det.times = deque([JUNCTION_DET_S] * TIME_WINDOW, maxlen=TIME_WINDOW)
  det.run = costs(det.run, JUNCTION_DET_S)
  det.lane_paint.read = costs(det.lane_paint.read, JUNCTION_PAINT_S)
  det.lights.read = costs(det.lights.read, JUNCTION_LIGHT_S)
  cams = []
  t0 = clock.t
  for k in range(frames):
    clock.t = max(clock.t + modeld_detector.REST_S, t0 + k * modeld_detector.CYCLE_S) + JUNCTION_MODEL_S
    n = len(pm.sent)
    det.step(pm, det.model_state, JUNCTION_MODEL_S, curvature, desire, [Meta(2 * k), Meta(2 * k + 1)], 'auto', 0., v_ego, signal_near)
    cams.append(str(pm.sent[-1].camera) if len(pm.sent) > n else None)
  return det, pm, cams


def road_gaps(cams):
  at = [k for k, c in enumerate(cams) if c == 'road']
  return [b - a for a, b in zip(at, at[1:], strict=False)]


def test_standing_at_a_signal_the_lights_are_read_every_other_run(monkeypatch):
  # 1e 3+10 to 3+36 and 11+1.7 to 11+9.9: standing near a mapped signal with the curvature frozen at 0.0117. The paint is
  # skipped outright there (noRoom: the lane changes and the wrong-side guard need it moving), so the road runs fit beside
  # the lights: the full pass every RUN_EVERY frames, the road camera every other run (5 Hz, 0.2 s apart), each with its
  # light read, never near the camera light's 1 s age-out
  from openpilot.selfdrive.turnpilot.map_controls import CAMERA_STALE_S
  det, pm, cams = junction_drive(monkeypatch, 200, 0., 0.0117, True)
  runs = [c for c in cams if c is not None]
  assert len(runs) == 100 and runs.count('road') == 50 and set(road_gaps(cams)) == {2 * RUN_EVERY}
  assert 2 * RUN_EVERY * 0.05 < CAMERA_STALE_S / 4
  road = [td for td in pm.sent if str(td.camera) == 'road']
  assert all(td.lanePaintStatus == LanePaintStatus.noRoom and td.lightStatus == LightStatus.read for td in road)
  assert len(det.lights.calls) == 50 and det.lane_paint.calls == [] and max(td.lightTime for td in road) > 0.
  assert det.debt <= DEBT_CAP_S
  # with no signal mapped near, the slow cycle's one road run in 3: the paint read where it fits (here it doesn't beside
  # the lights: skipped for room, read anyway every PAINT_FORCE_S), the road runs never forced only
  det, pm, cams = junction_drive(monkeypatch, 200, 0., 0.0117, False)
  assert [c for c in cams if c is not None].count('road') >= 100 // 3 and max(road_gaps(cams)) <= 3 * RUN_EVERY
  road = [td for td in pm.sent if str(td.camera) == 'road']
  assert {str(td.lanePaintStatus) for td in road} == {'noRoom', 'read'}


def test_the_paint_is_skipped_outright_only_under_its_readers_floors():
  # standing near a signal the paint is skipped for the lights' room, but never where something reads it: the wrong-side
  # guard (WRONG_SIDE_MIN_SPEED) and the lane planner's changes (LC_SPEED_MIN) read it from their floors up
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import PAINT_SKIP_V
  from openpilot.selfdrive.turnpilot.lane_planner import LC_SPEED_MIN
  from openpilot.selfdrive.turnpilot.turnpilotd import WRONG_SIDE_MIN_SPEED
  assert PAINT_SKIP_V <= min(WRONG_SIDE_MIN_SPEED, LC_SPEED_MIN)
  for v, near, want in ((0., True, LanePaintStatus.noRoom), (PAINT_SKIP_V - 0.1, True, LanePaintStatus.noRoom),
                        (PAINT_SKIP_V, True, LanePaintStatus.read), (2.8, True, LanePaintStatus.read), (0., False, LanePaintStatus.read)):
    det, pm = with_lights(make_detector(), 'addOnly'), PM()  # the light reader reading (lights_near)
    det.slow_runs = 2  # the slow cycle's road run
    det.alt_runs = 1   # the alternation's
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)], v_ego=v, signal_near=near)
    assert str(pm.sent[0].camera) == 'road' and pm.sent[0].lanePaintStatus == want, (v, near)


def test_approaching_a_signal_the_paint_goes_before_the_lights(monkeypatch):
  # moving straight toward a signal at a junction's light time: the road camera every run, at least every 3rd frame, the
  # lights read on each, the paint skipped where it has no room beside them (noRoom) and read anyway every PAINT_FORCE_S
  # (its estimate measured again); never a feature off for the drive
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import PAINT_FORCE_S
  det, pm, cams = junction_drive(monkeypatch, 200, 8., 0.0, True)
  assert all(c in (None, 'road') for c in cams) and max(road_gaps(cams)) <= 3 and len(road_gaps(cams)) >= 90
  statuses = [td.lanePaintStatus for td in pm.sent]
  assert statuses.count(LanePaintStatus.noRoom) >= 80 and 2 <= statuses.count(LanePaintStatus.read) <= 10 / PAINT_FORCE_S + 1
  assert all(td.lightStatus == LightStatus.read for td in pm.sent) and det.lane_paint is not None and det.lights is not None
  # the same frames before 11-06's fix: paint and lights together (a 9.3 ms debt) had room only on the forced runs
  assert JUNCTION_MODEL_S + JUNCTION_DET_S + JUNCTION_PAINT_S + JUNCTION_LIGHT_S - STEP_BUDGET_S > DEBT_CAP_S


def test_a_signalized_turn_reads_its_lights_to_the_line(monkeypatch):
  # 1e 8+16 to 8+25: a right at a signal, its desire out from 39 m, then the turn: every run was wide. Now the approach
  # (the desire, not yet turning) and the turn near the signal alternate 1:1, each road run with its light read
  for curvature, v in ((0.002, 6.), (0.05, 4.)):
    det, pm, cams = junction_drive(monkeypatch, 100, v, curvature, True, log.Desire.turnRight)
    runs = [c for c in cams if c is not None]
    assert len(runs) == 50 and runs.count('road') == 25, (curvature, v)
    assert all(td.lightStatus == LightStatus.read for td in pm.sent if str(td.camera) == 'road')


def test_the_lights_take_the_road_runs_only_while_their_reader_reads():
  # 11-06 review: the outright paint skip and a signal's 1:1 alternation went with signal_near whatever the light reader
  # did, so with TurnPilotCameraLights off, the reader in its retry backoff, or no road camera's reader, a stand near a
  # signal skipped the paint and gave every other wide run to a road camera nothing read the lights on. They go with the
  # reader reading (lights_near): without it the stand is the slow cycle's (one road run in 3, the paint read on it); the
  # run its retry is due on alternates again, and reads
  with sim_clock() as clock:
    for setup in ('off', 'backoff', 'no reader'):
      det, pm = with_lights(make_detector(), 'addOnly'), PM()
      if setup == 'off':
        det.light_mode = 'off'
      elif setup == 'backoff':
        det.light_failures, det.light_retry_at = 1, clock.t + 60.
      else:
        det.lights = None
      for k in range(6 * RUN_EVERY):
        clock.tick()
        det.step(pm, det.model_state, 0.010, 0.0117, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=0., signal_near=True)
      assert [str(td.camera) for td in pm.sent] == ['wide', 'wide', 'road'] * 2, setup
      assert det.signal_near and not det.lights_near
      road = [td for td in pm.sent if str(td.camera) == 'road']
      assert all(td.lanePaintStatus == LanePaintStatus.read for td in road) and len(det.lane_paint.calls) == 2, setup
      assert det.lights is None or det.lights.calls == []
    # the reader reading: 1:1, the paint skipped outright, the lights read on every road run
    det, pm = with_lights(make_detector(), 'addOnly'), PM()
    for k in range(6 * RUN_EVERY):
      clock.tick()
      det.step(pm, det.model_state, 0.010, 0.0117, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=0., signal_near=True)
    assert [str(td.camera) for td in pm.sent] == ['wide', 'road'] * 3 and det.lights_near
    road = [td for td in pm.sent if str(td.camera) == 'road']
    assert all(td.lanePaintStatus == LanePaintStatus.noRoom and td.lightStatus == LightStatus.read for td in road)
    # in a backoff, then the step its retry is due: the slow cycle, then 1:1 again, its road run the retry's read
    det, pm = with_lights(make_detector(), 'addOnly'), PM()
    det.light_failures, det.light_retry_at = 1, clock.t + 0.5
    cams = []
    for k in range(10 * RUN_EVERY):
      clock.tick()
      n = len(pm.sent)
      det.step(pm, det.model_state, 0.010, 0.0117, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=0., signal_near=True)
      if len(pm.sent) > n:
        cams.append((clock.t >= det.light_retry_at, str(pm.sent[-1].camera), pm.sent[-1].lightStatus))
    assert [c for due, c, _ in cams if not due] == ['wide', 'wide', 'road', 'wide', 'wide']
    assert [(c, st) for due, c, st in cams if due][:2] == [('wide', LightStatus.wideCamera), ('road', LightStatus.read)]
    assert det.light_failures == 0 and len(det.lights.calls) >= 2


def test_the_light_gates_are_logged_once_a_minute():
  # 11-06 review: Light.reason carries only the heads', and the boxes the size and horizon gates stop are never heads, so
  # the night tuning couldn't see how many class-9 boxes die there. The reader's funnel (frames, gated boxes, heads by
  # reason and colour) is logged at most once in LIGHT_GATES_LOG_S, for a window that saw a box, the counts starting over
  from unittest import mock
  from openpilot.selfdrive.turnpilot.detector import modeld_detector
  from openpilot.selfdrive.turnpilot.detector.light_state import LightReader
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import LIGHT_GATES_LOG_S
  from openpilot.selfdrive.turnpilot.detector.tests.test_light_state import LENS, LEVEL, STRIDE, Y_HEIGHT, head, scene
  frame, boxes = scene([head(0, 'red'), head(0, 'red', x0=900, w=6, h=18), head(0, 'red', x0=300, y0=500)])  # read, small, low
  none = np.zeros((0, 6), np.float32)

  class GateLights(FakeLights):
    """FakeLights with CameraLightReader's real LightReader inside, reading the junction's three boxes, or none."""
    def __init__(self):
      super().__init__()
      self.reader, self.boxes = LightReader(LENS), boxes

    def read(self, f, meta, b):
      self.reader.read(meta.timestamp_eof * 1e-9, frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, self.boxes)
      return super().read(f, meta, b)

  assert LIGHT_GATES_LOG_S == 60.
  with sim_clock() as clock, mock.patch.object(modeld_detector.cloudlog, 'event') as event:
    det, pm = with_lights(make_detector(), 'addOnly'), PM()
    det.lights = GateLights()
    for k in range(int(130 / CYCLE_S)):  # 130 s at road speed: the road camera every full run, 10 Hz
      clock.tick()
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=12.)
    logs = [c.kwargs for c in event.call_args_list if c.args[0] == 'turnpilot.light_gates']
    assert len(logs) == 2
    for kw in logs:
      n = kw['frames']
      assert 590 <= n <= 610 and abs(kw['window_s'] - LIGHT_GATES_LOG_S) <= 0.11 and kw['source'] == 'road' and not kw['signal_near']
      assert kw['gates'] == {'small': n, 'belowHorizon': n} and kw['heads'] == {'read': n} and kw['colours'] == {'red': n}
      assert kw['boxes'] == 3 * n
    # the boxes stop: the window they were in is logged once, a minute after it opened; one with none logs nothing, however long
    det.lights.boxes = none
    for k in range(int(130 / CYCLE_S)):
      clock.tick()
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=12.)
    logs = [c.kwargs for c in event.call_args_list if c.args[0] == 'turnpilot.light_gates']
    assert len(logs) == 3 and 590 <= logs[2]['frames'] <= 610 and 90 <= logs[2]['heads']['read'] <= 110


class TestCameraLights(OpenpilotTestCase):
  def test_lights_ride_along_on_the_road_frame(self):
    det, pm = with_lights(make_detector()), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(7), Meta(8)])
    det.since_run = RUN_EVERY
    det.step(pm, det.model_state, 0.010, 0.03, log.Desire.none, [Meta(11), Meta(12)])  # through the wide camera
    # the road camera's host frame and its class-9 boxes alone; the detections and the lane paint's mask never see a light
    assert det.lights.calls == [('host0', 7, [TRAFFIC_LIGHT])]
    assert det.lane_paint.calls == [('host0', 7, 1)]
    td = pm.sent[0]
    assert [d.cls for d in td.detections] == [PERSON]
    assert td.lightStatus == LightStatus.read and td.lightTime >= 0.
    lights = td.lights
    assert str(lights.source) == 'camera' and lights.shadow
    (light,) = lights.lights
    assert str(light.state) == 'red' and abs(light.distance - 32.) < 1e-4 and light.votes == 4 and light.governs
    assert abs(light.conf - 0.8) < 1e-6 and abs(light.bearing - 1.5) < 1e-6 and len(light.box) == 4
    assert str(light.read) == 'red' and light.reason == 'read'  # this frame's read and its reason
    assert pm.sent[1].lightStatus == LightStatus.wideCamera and len(pm.sent[1].lights.lights) == 0

  def test_the_reader_reads_boxes_under_the_detectors_threshold_and_light_boxes_dont_carry_them(self):
    # route 29 (2026-11-08): a red read only at its line, its heads first boxed at MIN_CONF 40 m out while the full pass had
    # them at 0.2-0.33 from about 60 m. The reader takes class 9 down to LIGHT_READ_CONF; lightBoxes (the signal-head checks)
    # stay at MIN_CONF
    from openpilot.selfdrive.turnpilot.detector.decoder import LIGHT_READ_CONF, MIN_CONF
    assert LIGHT_READ_CONF < 0.25 < MIN_CONF
    det, pm = with_lights(make_detector()), PM()
    det.run.raw[0, 2] = [*det.lb.to_input(800, 120), *det.lb.to_input(815, 165), 0.25, TRAFFIC_LIGHT]
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(7), Meta(8)])
    assert det.lights.calls == [('host0', 7, [TRAFFIC_LIGHT, TRAFFIC_LIGHT])]
    assert [round(b.conf, 2) for b in pm.sent[0].lightBoxes] == [0.8]

  def test_add_only_mode_is_not_shadow(self):
    det, pm = with_lights(make_detector(), 'addOnly'), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert str(pm.sent[0].lights.source) == 'camera' and not pm.sent[0].lights.shadow

  def test_off_reads_nothing(self):
    det, pm = with_lights(make_detector(), 'off'), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert det.lights.calls == [] and pm.sent[0].lightStatus == LightStatus.off and str(pm.sent[0].lights.source) == 'none'
    det, pm = make_detector({'wide': 0}), PM()  # no road camera
    det.light_mode = 'shadow'
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(1)])
    assert pm.sent[0].lightStatus == LightStatus.off

  def test_a_reads_reason_and_its_frames_read_are_published(self):
    # 11-06: 667 night reads, 0 greens, 85% of plausible heads with no vote, and the log couldn't say why: each head now
    # carries this frame's read and the gate that stopped it beside its vote
    head = LightHead(5, (0.4, 0.1, 0.41, 0.15), 0.6, 'red', 3, 'unknown', 'twoLit', -18., 17., False, 'h40 fr0.43/0.47/0.10')
    det, pm = with_lights(make_detector()), PM()
    det.lights = FakeLights([head])
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    (light,) = pm.sent[0].lights.lights
    assert str(light.state) == 'red' and str(light.read) == 'unknown' and light.reason == 'twoLit' and not light.governs
    assert light.instance == 'head5 h40 fr0.43/0.47/0.10'  # the read's detail on every run (2026-11-07), not only the samples
    from openpilot.cereal import messaging
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import fill_lights
    td = messaging.new_message('turnpilotDetections').turnpilotDetections
    fill_lights(td.lights, [LightHead(6, (0.4, 0.1, 0.41, 0.15), 0.6, 'red', 3, 'red', 'read', 1., 30., True)], False)
    assert td.lights.lights[0].instance == 'head6'  # no detail: the id alone, as before

  def test_light_failure_is_retried_and_the_detector_keeps_going(self):
    # never off for the drive: a failed read is logged, the road runs inside its backoff say retrying (no read: never an
    # empty read that says no red), and after RETRY_S, doubling, the lights are read again. On the simulated clock
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import LIGHT_DISABLED_FAILS
    from openpilot.selfdrive.turnpilot.map_controls import camera_light_input
    assert LIGHT_DISABLED_FAILS == 3
    with sim_clock() as clock, mock.patch.object(modeld_detector.cloudlog, 'exception') as logged:
      det, pm = with_lights(make_detector()), PM()
      det.lights.fail = True
      clock.tick()
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert det.lights is not None and det.light_failures == 1 and len(pm.sent) == 1 and len(pm.sent[0].detections) == 1
      assert pm.sent[0].lightStatus == LightStatus.retrying and len(pm.sent[0].lights.lights) == 0
      assert sum('lights failed' in c.args[0] for c in logged.call_args_list) == 1
      assert len(pm.sent[0].lanePaint) == 4  # the paint is its own
      assert det.light_retry_at == clock.t + RETRY_S
      det.lights.fail = False
      det.since_run = RUN_EVERY
      clock.tick()
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(3), Meta(4)])
      assert pm.sent[1].lightStatus == LightStatus.retrying and det.lights.calls == []  # inside the backoff
      det.since_run = RUN_EVERY
      clock.tick(RETRY_S)
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(5), Meta(6)])
      assert pm.sent[2].lightStatus == LightStatus.read and len(pm.sent[2].lights.lights) == 1 and det.light_failures == 0
      det.lights.fail = True
      for k in range(2):
        det.light_retry_at, det.since_run = 0., RUN_EVERY
        clock.tick()
        det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(7 + 2 * k), Meta(8 + 2 * k)])
      assert det.light_failures == 2 and det.light_retry_at == clock.t + 2 * RETRY_S  # doubling
      # two in a row are still a retry: turnpilotd's camera input takes retrying as no read at all, never a 'no red seen'
      assert pm.sent[-1].lightStatus == LightStatus.retrying
      assert camera_light_input(pm.sent[-1].lights, str(pm.sent[-1].lightStatus), 0., 'addOnly') is None

  def test_a_reader_failing_every_retry_says_disabled_until_a_read(self):
    # 11-06 review: since the reader stopped turning off for the drive, one failing on every retry all night said only
    # retrying, which turnpilotd takes for no read and nothing says: the red-light rail's sensor gone and the driver not
    # told. From LIGHT_DISABLED_FAILS failures in a row (3 s of nothing but failures) its road runs say disabled, logged once,
    # still retried on the backoff to RETRY_MAX_S; the first read that succeeds is read again, said
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import LIGHT_DISABLED_FAILS, RETRY_MAX_S, backoff_s
    cl = modeld_detector.cloudlog
    with sim_clock() as clock, mock.patch.object(cl, 'exception') as failed, mock.patch.object(cl, 'error') as error, \
         mock.patch.object(cl, 'warning') as warning:
      det, pm = with_lights(make_detector(), 'addOnly'), PM()
      det.lights.fail = True
      tries, statuses, t0 = [], [], clock.t
      for k in range(int(240 / CYCLE_S)):  # four minutes at road speed: every full run on the road camera, 10 Hz
        clock.tick()
        n, f = len(pm.sent), det.light_failures
        det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=12.)
        if len(pm.sent) > n:
          statuses.append((round(clock.t - t0, 2), str(pm.sent[-1].lightStatus)))
        if det.light_failures > f:
          tries.append(round(clock.t - t0, 2))
      # retried on the full pass's backoff (1, 2, 4 ... 60 s), every try logged, and only the third one onward says disabled
      waits = [b - a for a, b in zip(tries, tries[1:], strict=False)]  # each on the first road run its backoff allows
      assert len(tries) >= 8 and all(backoff_s(i + 1) - 1e-6 <= w <= backoff_s(i + 1) + RUN_EVERY * CYCLE_S + 1e-6 for i, w in enumerate(waits))
      assert backoff_s(len(tries)) == RETRY_MAX_S and failed.call_count == len(tries)
      third = tries[LIGHT_DISABLED_FAILS - 1]
      assert {st for t, st in statuses if t < third} == {'retrying'}
      assert {st for t, st in statuses if t >= third} == {'disabled'}
      assert third - tries[0] <= 3 * RETRY_S + 2 * RUN_EVERY * CYCLE_S + 1e-6  # 3 s of nothing but failures
      assert [c.args[0] for c in error.call_args_list] == [
        "turnpilot lights: 3 failed reads in a row, lightStatus disabled until a read succeeds (still retried)"]
      # it reads again on the next retry, said
      det.lights.fail = False
      while not det.lights.calls:
        clock.tick()
        det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(9000), Meta(9001)], v_ego=12.)
      assert pm.sent[-1].lightStatus == LightStatus.read and len(pm.sent[-1].lights.lights) == 1 and det.light_failures == 0
      assert sum('read again' in c.args[0] for c in warning.call_args_list) == 1 and error.call_count == 1

  def test_a_disabled_reader_reaches_the_driver_once(self):
    # the chain past modeld: turnpilotd's camera input (TurnPilot.camera_lights) keeps a disabled read past CAMERA_STALE_S,
    # through the wide runs and the backoff, as no read that names no red (shadow, no heads, unsignalized_why's 'camera');
    # map_controls says it once (camera_lights_off, 'Camera light reader off')
    from types import SimpleNamespace
    from unittest import mock
    from openpilot.common.realtime import DT_MDL
    from openpilot.selfdrive.turnpilot import map_controls
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    from openpilot.selfdrive.turnpilot.map_controls import CAMERA_OFF_REASON, CAMERA_STALE_S, Lights
    from openpilot.selfdrive.turnpilot.tests.test_map_controls import Car
    from openpilot.selfdrive.turnpilot.turnpilotd import TurnPilot
    assert DT_MDL == CYCLE_S

    class SM:
      def __init__(self):
        self.updated, self.msg = {'turnpilotDetections': False}, None

      def __getitem__(self, name):
        return self.msg

    tp, sm = SimpleNamespace(camera_age=float('inf'), camera_read=None, camera_mode='addOnly'), SM()
    car = Car([(2000., 'signal')])
    cl = modeld_detector.cloudlog
    with sim_clock() as clock, mock.patch.object(cl, 'exception'), mock.patch.object(cl, 'error'), \
         mock.patch.object(map_controls.cloudlog, 'event') as events:
      det, pm = with_lights(make_detector(), 'addOnly'), PM()
      det.lights.fail = True
      cams, alerts = [], []
      for k in range(int(30 / CYCLE_S)):
        clock.tick()
        n = len(pm.sent)
        det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], v_ego=0.)  # standing: 1 road run in 3
        sm.updated['turnpilotDetections'] = len(pm.sent) > n
        sm.msg = pm.sent[-1] if pm.sent else None
        cam = TurnPilot.camera_lights(tp, sm, {'turnpilotDetections': sm.msg})
        cams.append(cam)
        out, _, _ = car.step(lights=Lights(camera=cam))
        alerts.append((out.alert, out.reason))
    first = next(i for i, c in enumerate(cams) if c is not None and c.status == 'disabled')
    assert first * CYCLE_S <= 3.5 and all(c is not None and c.status == 'disabled' and c.heads == [] and c.shadow for c in cams[first:])
    assert (len(cams) - first) * CYCLE_S > 20 * CAMERA_STALE_S  # kept, never aged out
    said = [i for i, (a, r) in enumerate(alerts) if a == 'lightSourceRejected' and r == CAMERA_OFF_REASON]
    assert said and said[0] >= first and said == list(range(said[0], said[-1] + 1))  # one stretch: said once
    assert sum(c.args[0] == 'turnpilot.camera_lights_off' for c in events.call_args_list) == 1

  def test_slow_lights_stay_on_and_are_warned_once_a_minute(self):
    # 11-06: light medians to 3.74 ms against the 4.0 ms that turned them off for the drive, at the junctions where a red
    # must stop the car. Over LIGHT_BUDGET_S they stay on, warned at most once in BUDGET_WARN_S
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    det, pm = with_lights(make_detector()), PM()
    det.lights.delay = LIGHT_BUDGET_S * 1.5
    with mock.patch.object(modeld_detector.cloudlog, 'warning') as warned:
      for k in range(TIME_WINDOW + 3):
        det.since_run, det.debt = RUN_EVERY, 0.
        det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)])
    assert sum('lights over budget' in c.args[0] for c in warned.call_args_list) == 1
    assert det.lights is not None and det.light_failures == 0 and all(m.lightTime > LIGHT_BUDGET_S for m in pm.sent)
    assert all(m.lightStatus == LightStatus.read and len(m.lights.lights) == 1 for m in pm.sent)

  def test_light_time_counts_toward_the_step_budget(self):
    # 6 ms detector + 3 ms lights at the budget: 9 ms, past DEBT_CAP_S; off, 6 (on the simulated clock: SimClock)
    with sim_clock() as clock:
      det, pm = with_lights(make_detector(det_time=0.006)), PM()
      det.lane_paint = None
      det.light_times.extend([0.003] * 3)
      clock.tick()
      det.step(pm, det.model_state, STEP_BUDGET_S, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert pm.sent == []
      det.light_mode = 'off'  # off: its time no longer counts
      clock.tick()
      det.step(pm, det.model_state, STEP_BUDGET_S, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert len(pm.sent) == 1
      # in its retry backoff it reads nothing, so its time doesn't count either (estimate, light_reading)
      det, pm = with_lights(make_detector(det_time=0.006)), PM()
      det.lane_paint = None
      det.light_times.extend([0.003] * 3)
      det.light_retry_at = clock.tick() + 1.
      det.step(pm, det.model_state, STEP_BUDGET_S, 0.0, log.Desire.none, [Meta(1), Meta(2)])
      assert len(pm.sent) == 1 and pm.sent[0].lightStatus == LightStatus.retrying


class LightParams:
  def __init__(self, value=None, unknown=False):
    self.value, self.unknown = value, unknown

  def get(self, key):
    assert key == "TurnPilotCameraLights"
    if self.unknown:
      raise UnknownKeyName(key)
    return self.value


def test_camera_lights_mode():
  # the default is add-only since 5dcb363 (camera reds stop the car); this test still said shadow
  assert LIGHT_MODE_DEFAULT == 'addOnly'
  assert camera_lights_mode(LightParams(None)) == LIGHT_MODE_DEFAULT
  assert camera_lights_mode(LightParams('off')) == 'off'
  assert camera_lights_mode(LightParams('shadow')) == 'shadow'
  assert camera_lights_mode(LightParams('addOnly')) == 'addOnly'
  assert camera_lights_mode(LightParams('loud')) == LIGHT_MODE_DEFAULT
  assert camera_lights_mode(LightParams(unknown=True)) == LIGHT_MODE_DEFAULT  # a build whose params predate the key


class TestLanePaintReader(OpenpilotTestCase):
  def test_reads_this_frames_lines_off_the_bus(self):
    from openpilot.cereal import messaging
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import LanePaintReader
    from openpilot.selfdrive.turnpilot.scene.tests.test_lane_paint import DT, HEIGHT, LINE_YS, RPY, V_EGO, dashed, render, solid, WHITE
    pm = messaging.PubMaster(['modelV2', 'extrinsicsCalibration', 'deviceState', 'narrowRoadCameraState'])
    reader = LanePaintReader(1344, 760)
    msg = messaging.new_message('deviceState')
    msg.deviceState.deviceType = 'mici'
    pm.send('deviceState', msg)
    msg = messaging.new_message('narrowRoadCameraState')
    msg.narrowRoadCameraState.sensor = 'os04c10'
    pm.send('narrowRoadCameraState', msg)
    msg = messaging.new_message('extrinsicsCalibration')
    msg.extrinsicsCalibration.rpyCalib, msg.extrinsicsCalibration.calStatus = RPY, 'calibrated'
    pm.send('extrinsicsCalibration', msg)
    x = np.linspace(0., 100., 33)
    paint = None
    for k in range(20):
      msg = messaging.new_message('modelV2')
      msg.modelV2.frameId = 100 + k
      for line, y in zip(msg.modelV2.init('laneLines', 4), LINE_YS, strict=True):
        line.x, line.y, line.z = x.tolist(), [y] * len(x), [HEIGHT] * len(x)
      msg.modelV2.laneLineProbs = [0.9] * 4
      pm.send('modelV2', msg)
      frame, _ = render([None, dashed(WHITE), solid(WHITE), None], V_EGO * DT * k, seed=k)
      meta = Meta(100 + k)
      meta.timestamp_eof = int(k * DT * 1e9)
      if k == 19:
        assert reader.read(frame, Meta(99), None) == ([UNKNOWN] * 4, LanePaintStatus.noModel)  # another frame's lines
      paint, status = reader.read(frame, meta, None)
      assert status == LanePaintStatus.read
    assert [(p.kind, p.color) for p in paint[1:3]] == [('broken', 'white'), ('solid', 'white')], paint


class TestCameraLightReader(OpenpilotTestCase):
  def test_reads_on_the_calibrated_road_frame(self):
    from openpilot.cereal import messaging
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import CameraLightReader, frame_bus
    from openpilot.selfdrive.turnpilot.detector.light_state import VOTE_N
    from openpilot.selfdrive.turnpilot.detector.tests.test_light_state import head, scene
    pm = messaging.PubMaster(['modelV2', 'extrinsicsCalibration', 'deviceState', 'narrowRoadCameraState'])
    reader = CameraLightReader(frame_bus())
    frame, boxes = scene([head(0, 'red')])
    assert reader.read(frame, Meta(1), boxes) == ([], LightStatus.noCalibration)  # no device yet
    msg = messaging.new_message('deviceState')
    msg.deviceState.deviceType = 'mici'
    pm.send('deviceState', msg)
    msg = messaging.new_message('narrowRoadCameraState')
    msg.narrowRoadCameraState.sensor = 'os04c10'
    pm.send('narrowRoadCameraState', msg)
    msg = messaging.new_message('extrinsicsCalibration')
    msg.extrinsicsCalibration.rpyCalib, msg.extrinsicsCalibration.calStatus = [0., 0., 0.], 'uncalibrated'
    pm.send('extrinsicsCalibration', msg)
    assert reader.read(frame, Meta(2), boxes) == ([], LightStatus.noCalibration)  # no horizon to gate by
    msg = messaging.new_message('extrinsicsCalibration')
    msg.extrinsicsCalibration.rpyCalib, msg.extrinsicsCalibration.calStatus = [0., 0., 0.], 'calibrated'
    pm.send('extrinsicsCalibration', msg)
    for k in range(VOTE_N):
      heads, status = reader.read(frame, Meta(10 + k), boxes)
    assert status == LightStatus.read and [(h.state, h.governs) for h in heads] == [('red', True)]


# --- the stop sweep and the side views (STOP_SIGNS.md sections 1.1, 1.3 and 6; catalogue rows G17, E7-E11 and E14) ---
CAL_RPY, CAL_EULER = (-0.0004, 0.16485, 0.004535), (-2.1e-05, 0.00551, -0.00276)  # the demo route's comma four


class SyncExecutor:
  """Runs a submitted recompute at once (the side views' maps)."""
  def submit(self, fn, *args):
    from concurrent.futures import Future
    f = Future()
    f.set_result(fn(*args))
    return f


class HeldExecutor:
  """Holds a submitted recompute until finish(): the side views stay invalid until then."""
  def __init__(self):
    self.pending = []

  def submit(self, fn, *args):
    from concurrent.futures import Future
    f = Future()
    self.pending.append((f, fn, args))
    return f

  def finish(self):
    for f, fn, args in self.pending:
      f.set_result(fn(*args))
    self.pending = []


class FakeSideRun:
  """Stands in for the side-view jit: records which view's maps it got, returns a car and our own dash in view pixels."""
  def __init__(self):
    self.calls, self.fail, self.delay = [], False, 0.
    self.raw = np.zeros((1, 300, 6), np.float32)
    self.raw[0, 0] = [300, 160, 380, 215, 0.7, 2]   # a car on the horizon
    self.raw[0, 1] = [20, 380, 760, 512, 0.9, 2]    # the dash: wider than 0.9 of the view, at its bottom

  def __call__(self, input_frame, luma_idx, chroma_idx, weights):
    if self.fail:
      raise RuntimeError("gpu fell over")
    if self.delay:
      time.sleep(self.delay)
    self.calls.append((input_frame, luma_idx.split('.')[0]))
    return FakeOutput(self.raw)


def fake_upload(name, side_map):
  return f'{name}.luma', f'{name}.chroma', f'{name}.weights'


class SweepPM:
  def __init__(self):
    self.msgs, self.far = [], []

  def send(self, name, msg):
    r = msg.as_reader()
    (self.msgs if name == 'turnpilotDetections' else self.far).append(getattr(r, name))

  @property
  def runs(self):
    return [(str(m.camera), str(m.view)) for m in self.msgs]


def side_detector(rpy=CAL_RPY, side_time=0.007, det_time=0.005, executor=None, ready=True):
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import SideViews
  from openpilot.selfdrive.turnpilot.detector.sideview import SIDE_VIEWS
  from openpilot.selfdrive.turnpilot.scene.ranging import wide_lens
  det = make_detector(det_time=det_time)
  det.model_state.frames = [np.zeros(1, np.uint8), np.zeros(1, np.uint8)]  # sector quality reads nothing here
  side = SideViews(FakeSideRun(), 1344, 760, 'frame1', executor=executor or SyncExecutor(), upload=fake_upload)
  side.lens = wide_lens(DEVICE_CAMERAS[('mici', 'os04c10')].wide_road, 'mici')
  side.calibration = lambda: None if rpy is None else (list(rpy), list(CAL_EULER))
  side.quality = None  # E11 tests it on a real frame
  side.warmup_s = side_time
  det.side = side
  if ready:
    for _ in range(len(SIDE_VIEWS)):  # the maps for this calibration, on the device a view at a time
      side.prepare(True)
    assert side.geometry is not None and set(side.device) == {'sideLeft', 'sideRight'}
  return det


def sweep_steps(det, pm, n, view='stopSweep', v_ego=0., model_time=0.010, start=0, desire=log.Desire.none, curvature=0.):
  for k in range(n):
    det.step(pm, det.model_state, model_time, curvature, desire, [Meta(start + 2 * k), Meta(start + 2 * k + 1)], view, 0., v_ego)


HELD = [('wide', 'full'), ('wide', 'sideLeft'), ('road', 'full'), ('wide', 'sideRight')]
CROSSING = [('wide', 'full'), ('wide', 'sideLeft'), ('wide', 'full'), ('wide', 'sideRight')]


class TestStopSweep(OpenpilotTestCase):
  def test_E10_a_car_standing_straight_at_a_stop_runs_the_held_cycle(self):
    # today a car standing straight at a stop line (no curvature, no turn desire) runs the road camera alone, which can't see
    # cross traffic at 60-85 deg; under stopSweep the even steps alternate the full wide pass and the road camera
    det = make_detector()
    assert [det.choose_camera(0.0, log.Desire.none, 0.1 * k, 'stopSweep') for k in range(4)] == ['wide', 'road'] * 2
    assert [det.choose_camera(0.0, log.Desire.none, 1. + 0.1 * k, 'stopSweepGo') for k in range(3)] == ['wide'] * 3
    # and the odd steps alternate the side views: [full wide, left, road, right], 5 Hz a view
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SideViewStatus
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 8)
    assert pm.runs == HELD * 2
    assert [c[1] for c in det.side.run.calls] == ['sideLeft', 'sideRight'] * 2 and all(c[0] == 'frame1' for c in det.side.run.calls)
    assert all(m.sideViewStatus == SideViewStatus.running for m in pm.msgs)
    # the crossing cycle: the full wide pass on every even step, the sides on the odd ones
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 8, view='stopSweepGo')
    assert pm.runs == CROSSING * 2
    # out of the sweep, as before: no side view; standing, the full pass's wide camera (WIDE_SLOW_V), at speed the road camera
    sweep_steps(det, pm, 4, view='auto', start=100)
    assert pm.runs[8:] == [('wide', 'full')] * 2 and len(det.side.run.calls) == 4
    sweep_steps(det, pm, 4, view='auto', start=200, v_ego=10.)
    assert pm.runs[10:] == [('road', 'full')] * 2 and len(det.side.run.calls) == 4

  def test_E10_side_boxes_go_back_to_fisheye_pixels_on_the_wide_camera_without_the_dash(self):
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 2)
    td = pm.msgs[1]
    assert str(td.camera) == 'wide' and str(td.view) == 'sideLeft' and td.frameId == 3  # the wide camera's frame of the step
    (d,) = td.detections  # the dash's box is dropped
    geom = det.side.geometry
    u, v = geom.to_fisheye('sideLeft', np.array([300., 380., 340.]), np.array([160., 215., 215.]))
    assert d.x0 <= min(u) + 0.5 and d.x1 >= max(u) - 0.5 and d.y0 <= min(v) + 0.5 and d.y1 >= max(v) - 0.5
    assert d.x1 < 1344 / 2  # left of the frame's centre: the left view
    assert td.lens == det.side.lens.spec  # the lens the views were mapped with, for SG4a's mismatch check

  def test_E14_no_side_view_at_road_speed(self):
    # the speed gate: above SIDE_VIEW_V_MAX the odd steps stay idle, so the approach costs what the detector costs today
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SIDE_VIEW_V_MAX, SideViewStatus
    assert SIDE_VIEW_V_MAX == 2.0
    for view in ('stopSweep', 'stopSweepGo'):
      det, pm = side_detector(), SweepPM()
      sweep_steps(det, pm, 8, view=view, v_ego=5.)
      assert det.side.run.calls == [] and all(m.view == 'full' for m in pm.msgs) and len(pm.msgs) == 4
      assert all(m.sideViewStatus == SideViewStatus.speedGated for m in pm.msgs)  # the decider counts only views the gate runs
      sweep_steps(det, pm, 8, view=view, v_ego=1.5, start=100)
      assert [c[1] for c in det.side.run.calls] == ['sideLeft', 'sideRight'] * 2
      assert pm.msgs[-1].sideViewStatus == SideViewStatus.running
    # no speed known: no side view
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 4, v_ego=None)
    assert det.side.run.calls == []

  def test_G17_a_stop_sign_just_after_a_turn_still_gets_the_sweep(self):
    # a stop 30 m after a turn: the turn's wide hold has run out and its desire is off, or its desire is still out. Either
    # way the sweep runs its own cycle: the wide camera at the hold, whatever the turn-desire state
    for desire, curvature in ((log.Desire.none, 0.0), (log.Desire.turnRight, 0.05)):
      det, pm = side_detector(), SweepPM()
      det.wide_until = -1.0 if desire == log.Desire.none else 1e9
      sweep_steps(det, pm, 8, desire=desire, curvature=curvature)
      assert pm.runs == HELD * 2, desire

  def test_a_shadow_plan_still_sweeps(self):
    # modeld reads the view through fresh_plan, whatever the plan's state: a shadow drive (and step S3's dwell with
    # openpilot off) sweeps too; a signal mapped within SWEEP_SIGNAL_M is what lets the light reader read during a sweep
    from openpilot.cereal import messaging
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SWEEP_SIGNAL_M, plan_inputs
    from openpilot.selfdrive.turnpilot.hooks import fresh_plan
    msg = messaging.new_message('turnpilotPlan')
    p = msg.turnpilotPlan
    p.shadow, p.state, p.detectorView, p.farCropBearing = True, 'disabled', 'stopSweep', 3.
    p.mapControl.kind, p.mapControl.distance = 'signal', SWEEP_SIGNAL_M - 1.

    class SM:
      recv_time, valid = {'turnpilotPlan': time.monotonic()}, {'turnpilotPlan': True}

      def __getitem__(self, s):
        return msg.as_reader().turnpilotPlan
    assert plan_inputs(fresh_plan(SM())) == ('stopSweep', 3., True, False)
    p.mapControl.distance = SWEEP_SIGNAL_M + 1.
    assert plan_inputs(fresh_plan(SM())) == ('stopSweep', 3., False, False)
    p.mapControl.kind, p.mapControl.distance = 'stop', 10.
    assert plan_inputs(fresh_plan(SM()))[2] is False
    assert plan_inputs(None) == ('auto', 0., False, False)

  def test_far_first_is_an_unprotected_lefts_view_with_its_band_past_the_full_pass(self):
    # plan_inputs' far_first (left_pick): the left's views (road from VIEW_FROM_M, roadAndWide at its line) with R_req past
    # the full pass's range (rangeM), from those two alone: farNeeded goes false while the far slot is stale, and a stale
    # far view would then never get the frames to read again
    from openpilot.cereal import messaging
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import plan_inputs
    from openpilot.selfdrive.turnpilot.hooks import fresh_plan
    msg = messaging.new_message('turnpilotPlan')
    p = msg.turnpilotPlan
    p.unprotectedLeft.rReq, p.unprotectedLeft.rangeM, p.unprotectedLeft.farNeeded = 134., 98., False

    class SM:
      recv_time, valid = {'turnpilotPlan': time.monotonic()}, {'turnpilotPlan': True}

      def __getitem__(self, s):
        return msg.as_reader().turnpilotPlan
    for view, want in (('road', True), ('roadAndWide', True), ('auto', False), ('stopSweep', False), ('stopSweepGo', False)):
      p.detectorView = view
      assert plan_inputs(fresh_plan(SM()))[3] is want, view
    p.detectorView = 'roadAndWide'
    p.unprotectedLeft.rReq = 85.  # a 15 mph road's: inside the full pass's reach, nothing for the far slot
    assert plan_inputs(fresh_plan(SM()))[3] is False

  def test_a_sweep_suspends_the_far_crop_lane_paint_and_unmapped_lights(self):
    # G6's far crop never runs during a sweep (the odd steps are the side views'), and says so; lane paint is off; the
    # light reader's colour read waits for a signal mapped within SWEEP_SIGNAL_M, but every forward run's class-9 boxes
    # are published
    from openpilot.selfdrive.turnpilot.detector.tests.test_far_crop import FakeFarRun
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import FarCrop
    from openpilot.selfdrive.turnpilot.scene.ranging import road_lens
    from openpilot.common.transformations.camera import DEVICE_CAMERAS
    FarStatus = __import__('openpilot.cereal', fromlist=['custom']).custom.TurnpilotDetections.FarStatus
    det, pm = with_lights(side_detector()), SweepPM()
    far = FarCrop(FakeFarRun(), 1344, 760, lambda crop: ('luma', 'chroma'))
    far.lens, far.calibration, far.warmup_s = road_lens(DEVICE_CAMERAS[('mici', 'os04c10')].narrow_road), lambda: [0., 0., 0.], 0.006
    det.far = far
    sweep_steps(det, pm, 8, v_ego=5.)  # odd steps idle at speed: still no far crop
    sweep_steps(det, pm, 8, start=100)
    assert far.run.calls == [] and pm.far == []
    assert all(m.farStatus == FarStatus.sweep for m in pm.msgs)
    road = [m for m in pm.msgs if str(m.camera) == 'road']
    assert road and det.lane_paint.calls == [] and all(m.lanePaintStatus == LanePaintStatus.sweep for m in road)
    assert det.lights.calls == [] and all(m.lightStatus == LightStatus.sweep for m in road)
    forward = [m for m in pm.msgs if str(m.view) == 'full']
    assert all([b.cls for b in m.lightBoxes] == [TRAFFIC_LIGHT] for m in forward)  # wide runs' too
    # a signal mapped within 60 m: the colour read runs on the road runs
    for k in range(4):
      det.step(pm, det.model_state, 0.010, 0., log.Desire.none, [Meta(300 + k), Meta(400 + k)], 'stopSweep', 0., 0., True)
    assert len(det.lights.calls) == 1 and det.lane_paint.calls == []
    # out of the sweep the far crop comes back
    sweep_steps(det, pm, 4, view='auto', start=500)
    assert len(far.run.calls) == 2 and pm.msgs[-1].farStatus == FarStatus.read

  def test_E7_a_side_view_without_room_is_skipped_and_said(self):
    # the time budget governs each side-view run on its own estimate: one that doesn't fit is skipped (its view goes stale,
    # which the decider counts against a go) and the messages say noRoom; the full pass's runs are untouched
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SIDE_STATUS_S, SideViewStatus
    det, pm = side_detector(side_time=0.012), SweepPM()
    # 1 ms under the budget: the full pass (5 ms) leaves 4 ms of debt; a side view (12 ms) would leave 11 with the next full
    # pass's 4 still to come, a 4 ms one 3 + 4
    sweep_steps(det, pm, 8, model_time=STEP_BUDGET_S - 0.001)
    assert det.side.run.calls == [] and pm.runs == [('wide', 'full'), ('road', 'full')] * 2
    det.side.started_t -= SIDE_STATUS_S + 0.01
    sweep_steps(det, pm, 2, model_time=STEP_BUDGET_S - 0.001, start=100)
    assert pm.msgs[-1].sideViewStatus == SideViewStatus.noRoom
    det.side.times['sideLeft'].extend([0.004] * TIME_WINDOW)  # its own estimate: the left fits, the right doesn't
    sweep_steps(det, pm, 4, model_time=STEP_BUDGET_S - 0.001, start=200)
    assert [c[1] for c in det.side.run.calls] == ['sideLeft']

  def test_E9_an_overrunning_side_view_is_off_for_the_drive_loudly(self):
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SIDE_BUDGET_S, SideViewStatus
    det, pm = side_detector(side_time=0.001), SweepPM()
    det.side.run.delay = SIDE_BUDGET_S * 1.3
    with mock.patch.object(modeld_detector.cloudlog, 'error') as error:
      sweep_steps(det, pm, 4 * TIME_WINDOW, model_time=0.002)
    assert det.side.disabled and len(det.side.run.calls) == TIME_WINDOW
    assert sum('side views' in c.args[0] for c in error.call_args_list) == 1  # said once
    assert pm.msgs[-1].sideViewStatus == SideViewStatus.disabled
    # both cycles fall back to the full wide pass and the road camera, the odd steps idle, for the drive
    det.side.run.delay = 0.
    n = len(pm.msgs)
    sweep_steps(det, pm, 8, start=1000)
    assert pm.runs[n:] == [('wide', 'full'), ('road', 'full')] * 2 and len(det.side.run.calls) == TIME_WINDOW
    sweep_steps(det, pm, 4, view='stopSweepGo', start=2000)
    assert pm.runs[n + 4:] == [('wide', 'full')] * 2 and all(m.sideViewStatus == SideViewStatus.disabled for m in pm.msgs[n:])
    # a failure is the same, and the full pass's own backoff never moves
    det, pm = side_detector(), SweepPM()
    det.side.run.fail = True
    sweep_steps(det, pm, 4)
    assert det.side.disabled and det.failures == 0 and det.retry_at == 0. and len(pm.msgs) == 2

  def test_E9_step_overruns_turn_the_side_views_off(self):
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SIDE_OVERRUNS
    det, pm = side_detector(side_time=0.001), SweepPM()
    det.side.run.delay = 0.009
    for k in range(4 * TIME_WINDOW):
      for t in det.side.times.values():
        t.clear()  # each run fits its (low) estimate from a paid-back cycle, but beside a model 1 ms over the budget it
      det.debt = 0.  # leaves 10 ms of debt, past DEBT_CAP_S
      sweep_steps(det, pm, 1, model_time=STEP_BUDGET_S + 0.001, start=10 * k)
    assert det.side.disabled and len(det.side.run.calls) == SIDE_OVERRUNS

  def test_E8_a_calibration_change_invalidates_the_side_views_until_their_maps_are_recomputed(self):
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SideViewStatus
    from openpilot.selfdrive.turnpilot.detector.sideview import RECALIB_DEG
    held = HeldExecutor()
    det, pm = side_detector(executor=held, ready=False), SweepPM()
    sweep_steps(det, pm, 4)  # no maps yet: computing, nothing runs
    assert det.side.run.calls == [] and len(held.pending) == 1 and pm.msgs[-1].sideViewStatus == SideViewStatus.mapsStale
    held.finish()
    sweep_steps(det, pm, 2, start=50)  # one odd step: one view's maps up (27 ms for both on the chestnut), no view run
    assert list(det.side.staged) == ['sideLeft'] and det.side.geometry is None and det.side.run.calls == []
    sweep_steps(det, pm, 8, start=100)
    # the steps after the recompute upload the maps a view at a time (and run no view), then the views run
    assert {c[1] for c in det.side.run.calls} == {'sideLeft', 'sideRight'} and pm.msgs[-1].sideViewStatus == SideViewStatus.running
    first = det.side.geometry
    # under RECALIB_DEG of movement: the same maps
    rpy = np.array(CAL_RPY) + [0., np.radians(RECALIB_DEG - 0.1), 0.]
    det.side.calibration = lambda: (list(rpy), list(CAL_EULER))
    sweep_steps(det, pm, 4, start=200)
    assert held.pending == [] and det.side.geometry is first and pm.msgs[-1].sideViewStatus == SideViewStatus.running
    # past it: invalid at once, recomputed off the step, then back on the new maps
    rpy = np.array(CAL_RPY) + [0., np.radians(RECALIB_DEG + 0.1), 0.]
    det.side.calibration = lambda: (list(rpy), list(CAL_EULER))
    n = len(det.side.run.calls)
    sweep_steps(det, pm, 4, start=300)
    assert len(det.side.run.calls) == n and len(held.pending) == 1 and pm.msgs[-1].sideViewStatus == SideViewStatus.mapsStale
    held.finish()
    sweep_steps(det, pm, 8, start=400)
    assert len(det.side.run.calls) > n and det.side.geometry is not first
    np.testing.assert_allclose(det.side.geometry.rpy, rpy)
    # no calibration: none
    det, pm = side_detector(rpy=None, ready=False), SweepPM()
    sweep_steps(det, pm, 4)
    assert det.side.run.calls == [] and pm.msgs[-1].sideViewStatus == SideViewStatus.noCalibration

  def test_E11_dark_frozen_and_saturated_sectors_are_not_ok(self):
    # sector quality per 5 deg of bearing on each side view, from the host copy of the wide frame: a black sector reads
    # dark, a stuck one frozen, a saturated one washed, and textured ones ok; within about 1 ms
    from openpilot.selfdrive.turnpilot.detector.sideview import SectorQuality
    from openpilot.selfdrive.turnpilot.detector.tests.test_sideview import geometry, textured_frame
    geom = geometry()
    q = SectorQuality()
    rng = np.random.default_rng(5)
    left = geom.maps['sideLeft'].sectors
    dark, frozen, sat = left[2], left[4], geom.maps['sideRight'].sectors[3]
    stuck = None
    times = []
    for k in range(6):  # 5 Hz a side, 1.2 s
      frame = textured_frame(geom, rng)
      frame[dark.idx] = rng.integers(16, 30, len(dark.idx))
      frame[sat.idx] = 235  # full range 255
      if stuck is None:
        stuck = frame[frozen.idx].copy()
      frame[frozen.idx] = stuck
      t0 = time.perf_counter()
      got = {name: q.update(geom, name, frame, 0.2 * k) for name in ('sideLeft', 'sideRight')}
      times.append((time.perf_counter() - t0) / 2)
      quality = {(name, s[0]): s[2] for name, ss in got.items() for s in ss}
      if k == 0:
        assert quality[('sideLeft', dark.inner)] == 'dark' and quality[('sideRight', sat.inner)] == 'washed'
        assert quality[('sideLeft', frozen.inner)] == 'ok'
    assert quality[('sideLeft', frozen.inner)] == 'frozen' and quality[('sideRight', sat.inner)] == 'washed'
    assert quality[('sideLeft', dark.inner)] in ('dark', 'blocked')
    others = [v for key, v in quality.items() if key not in (('sideLeft', dark.inner), ('sideLeft', frozen.inner), ('sideRight', sat.inner))]
    assert others and all(v == 'ok' for v in others), quality
    print(f"sector quality: {1e3 * np.median(times):.2f} ms a side (median)")
    assert np.median(times) < 0.003

  def test_E11_a_side_run_carries_its_sectors(self):
    from openpilot.selfdrive.turnpilot.detector.sideview import SectorQuality
    from openpilot.selfdrive.turnpilot.detector.tests.test_sideview import textured_frame
    det, pm = side_detector(), SweepPM()
    det.side.quality = SectorQuality()
    frame = textured_frame(det.side.geometry, np.random.default_rng(1))
    det.model_state.frames = [frame, frame]
    sweep_steps(det, pm, 4)
    left, right = pm.msgs[1], pm.msgs[3]
    assert str(left.view) == 'sideLeft' and len(left.sectorQuality) >= 6 and all(s.inner > 45. for s in left.sectorQuality)
    assert all(s.outer < -45. for s in right.sectorQuality) and all(str(s.quality) == 'ok' for s in left.sectorQuality)
    assert left.sectorTime > 0. and len(pm.msgs[0].sectorQuality) == 0  # forward runs carry none

  def test_side_views_off_without_a_side_pkl_or_a_wide_camera(self):
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SideViewStatus
    det, pm = make_detector(), SweepPM()
    det.side = None
    sweep_steps(det, pm, 8)
    assert pm.runs == [('wide', 'full'), ('road', 'full')] * 2 and pm.msgs[-1].sideViewStatus == SideViewStatus.off
    sweep_steps(det, pm, 2, view='auto', start=100)
    assert pm.msgs[-1].sideViewStatus == SideViewStatus.off
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 2, view='auto')
    assert pm.msgs[-1].sideViewStatus == SideViewStatus.idle

  def test_stop_signs_are_decoded_apart_in_shadow_with_their_plaque(self):
    # COCO 11 never reaches the detections (nothing stops, caps or releases on it); under TurnPilotCameraStopSigns shadow
    # (the default) each sign rides in signs with the luma below it for plaque labelling; off decodes none
    from openpilot.selfdrive.turnpilot.detector.decoder import STOP_SIGN
    det, pm = make_detector(), SweepPM()
    lb = det.lb
    det.run.raw[0, 2] = [*lb.to_input(600, 300), *lb.to_input(640, 340), 0.8, STOP_SIGN]
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    stride, y_height, uv_height, _ = get_nv12_info(1344, 760)
    frame = np.zeros(stride * (y_height + uv_height), np.uint8)
    frame.reshape(-1, stride)[330:390, 570:670] = 200  # a plaque below the sign
    det.model_state.frames = [frame, frame]
    det.step(pm, det.model_state, 0.010, 0., log.Desire.none, [Meta(1), Meta(2)])
    td = pm.msgs[0]
    assert [d.cls for d in td.detections] == [PERSON]
    (s,) = td.signs
    assert s.box.cls == STOP_SIGN and abs(s.box.x0 - 600) < 1 and abs(s.box.y1 - 340) < 1
    x0, y0, x1, y1 = s.plaqueRect
    assert y0 >= 339 and x0 < 600 < 640 < x1 and s.plaqueW * s.plaqueH == len(s.plaque) and min(s.plaque) == 200
    det.signs_mode = 'off'
    det.step(pm, det.model_state, 0.010, 0., log.Desire.none, [Meta(3), Meta(4)])
    det.step(pm, det.model_state, 0.010, 0., log.Desire.none, [Meta(5), Meta(6)])
    assert len(pm.msgs[-1].signs) == 0 and [d.cls for d in pm.msgs[-1].detections] == [PERSON]


class StopSignParams(LightParams):
  def get(self, key):
    assert key == "TurnPilotCameraStopSigns"
    if self.unknown:
      raise UnknownKeyName(key)
    return self.value


def test_camera_stop_signs_mode():
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import camera_stop_signs_mode
  assert camera_stop_signs_mode(StopSignParams(None)) == 'shadow'
  assert camera_stop_signs_mode(StopSignParams('off')) == 'off'
  assert camera_stop_signs_mode(StopSignParams(b'shadow')) == 'shadow'
  assert camera_stop_signs_mode(StopSignParams('auto')) == 'shadow'  # nothing acts on a stop sign seen this week
  assert camera_stop_signs_mode(StopSignParams(unknown=True)) == 'shadow'


def test_detections_are_published_at_20_hz():
  # the sweep runs a view on every model frame while it holds the car (STOP_SIGNS.md section 1.3)
  from openpilot.cereal.services import SERVICE_LIST
  assert SERVICE_LIST['turnpilotDetections'].frequency == 20.


def test_a_side_pkl_is_checked_at_load():
  # modeld loads the side pkl only if its input_specs and metadata are what it gives it: the wide frame's view and a
  # 768x512 view's maps (graph.side_input_specs, which compile_detector writes)
  from openpilot.selfdrive.turnpilot.detector.graph import side_input_specs
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import check_side_specs
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride, y_height, uv_height, _ = get_nv12_info(1344, 760)
  size = stride * (y_height + uv_height)
  meta = {'kind': 'side', 'camera_size': (1344, 760), 'input_size': (768, 512)}
  check_side_specs({'metadata': meta, 'input_specs': side_input_specs(1344, 760, 'AMD')}, size)
  bad = side_input_specs(1344, 760, 'AMD')
  bad['weights'] = ((2, 768 * 512), bad['weights'][1], 'AMD')
  for art, frame_size in (({'metadata': meta, 'input_specs': bad}, size),
                          ({'metadata': {**meta, 'kind': 'far'}, 'input_specs': side_input_specs(1344, 760, 'AMD')}, size),
                          ({'metadata': {**meta, 'input_size': (640, 384)}, 'input_specs': side_input_specs(1344, 760, 'AMD')}, size),
                          ({'metadata': meta, 'input_specs': side_input_specs(1344, 760, 'AMD')}, size + 128)):
    with pytest.raises(ValueError):
      check_side_specs(art, frame_size)


def test_the_side_views_maps_and_warm_up_are_on_the_frames_device(monkeypatch, tmp_path):
  """BUILD_FINDINGS N4: every diag run's modeld logged 'turnpilot side views failed to load', a tinygrad JitError: the compiled
  side pkl's inputs and the wide frame on the sim host's AMD eGPU, the warm-up's blank maps made on tinygrad's default device
  (METAL). With no side views every sim stop sign ended 'Stop-sign go off: no camera measurements'. The warm-up's blanks and
  every view's maps (upload_maps) go on the frame's device, and a pkl compiled for another device is refused at load, said."""
  from types import SimpleNamespace
  from tinygrad import Device, Tensor
  import tinygrad.engine.realize as realize
  from openpilot.selfdrive.turnpilot.detector import modeld_detector as md
  from openpilot.selfdrive.turnpilot.detector.graph import side_input_specs
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  frame_dev = next(d for d in ('CPU', 'PYTHON') if d != Device.DEFAULT)  # never tinygrad's default here
  stride, y_height, uv_height, _ = get_nv12_info(1344, 760)
  size = stride * (y_height + uv_height)
  frame = Tensor(np.zeros(size, np.uint8), device=frame_dev).realize()

  class DeviceRecordingRun:
    def __init__(self):
      self.captured, self.devices = SimpleNamespace(_linear=[]), []

    def __call__(self, input_frame, luma_idx, chroma_idx, weights):
      self.devices.append({Device.canonicalize(t.device) for t in (input_frame, luma_idx, chroma_idx, weights)})
      return FakeOutput(np.zeros((1, 300, 6), np.float32))

  def art(device):
    return {'metadata': {'kind': 'side', 'camera_size': (1344, 760), 'input_size': (768, 512)},
            'input_specs': side_input_specs(1344, 760, device), 'run': DeviceRecordingRun()}

  pkl = tmp_path / 'side.pkl'
  pkl.write_bytes(b'')
  loaded = {}
  monkeypatch.setattr(md, 'side_detector_pkl_path', lambda *a: pkl)
  monkeypatch.setattr(md, 'frame_view', lambda state, index: frame)
  monkeypatch.setattr(realize, 'lower_and_compile', lambda linear: linear)
  import openpilot.selfdrive.modeld.helpers as helpers
  monkeypatch.setattr(helpers, 'load_oob', lambda path, chestnut: loaded['art'])
  state = SimpleNamespace(chestnut=True, frame_copy_size=size)
  loaded['art'] = art(frame_dev)
  side = md.load_side(state, 1344, 760, 1, None)
  assert side is not None, "loaded"
  assert loaded['art']['run'].devices and all(d == {frame_dev} for d in loaded['art']['run'].devices), loaded['art']['run'].devices
  maps = side.upload('sideLeft', SimpleNamespace(luma_idx=np.zeros(4, np.int32), chroma_idx=np.zeros(4, np.int32),
                                                 weights=np.zeros((3, 4), np.uint8)))
  assert all(Device.canonicalize(t.device) == frame_dev for t in maps)
  # compiled for another device than the frame's: refused at load, logged, never run
  loaded['art'] = art(Device.DEFAULT)
  assert md.load_side(state, 1344, 760, 1, None) is None and not loaded['art']['run'].devices


def test_side_maps_run_off_modelds_realtime_core(monkeypatch):
  """The side maps' worker drops modeld's SCHED_FIFO 54 and core 7 (dmonitoringmodeld's core) before its first recompute, in
  its own thread."""
  import threading
  import openpilot.common.realtime as realtime
  import openpilot.selfdrive.turnpilot.detector.modeld_detector as md
  calls = []
  monkeypatch.setattr(realtime, 'drop_realtime', lambda: calls.append(('drop', threading.current_thread().name)))
  monkeypatch.setattr(realtime, 'set_core_affinity', lambda cores: calls.append((tuple(cores), threading.current_thread().name)))
  executor = md.side_maps_executor()
  try:
    worker = executor.submit(lambda: threading.current_thread().name).result(timeout=5)
  finally:
    executor.shutdown()
  assert worker.startswith('side_maps') and worker != threading.current_thread().name
  assert calls == [('drop', worker), (tuple(md.SIDE_MAPS_CORES), worker)]
  assert 7 not in md.SIDE_MAPS_CORES
