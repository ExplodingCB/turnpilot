"""The far reading at an unprotected left on the car's budget (2026-11-07), replayed on a simulated clock.

On both 11-06 night drives (build 7c37f664a) the far slot read nothing at a left. The native schedule never ran: a 1344x512
strip took 14.9-15.4 ms warm, which beside the big model (38.4-38.6 ms median, p90 39.5, p99 40.6) leaves 6.4 ms or more of
debt from a paid-back step before its step's own 2.4 ms around the run, and the frame between the full runs had to leave the
next full pass its room too (1.7 ms after a wide run, 5.8 after a road run on the estimates): no frame could ever hold it.
The far crop (10.5 ms median, its lights 0.7) ran only between two wide runs (all 181 of its runs, 28 on 1d, 153 on 1e), in
the drives' first minutes: its room check took its run alone while its step cost 2.4 ms more (decode, the message, the
send), so runs it fit left 6.5-7.1 ms of debt (median; p90 7.4-8.5, a 13-15 ms run 10-11 ms), and three of five past
DEBT_CAP_S turned it off for the drive (79.5 s into 1d's log, 190.6 s into 1e's). From then on every full pass message said farStatus noRoom.

The car's numbers here, from both drives' rlogs: the model's time by its quantiles; the full pass's run 10.2 ms, the paint
3.4, the lights 0.7; the far crop's run 10.5 and its lights 0.7; each step's time past them (decode, messages, sends: a wide
run's 1.3 ms, a road run's 1.6 more with the paint 0.6 on top, a far run's 2.4), measured as each step's next frame's lateness
less the model and the runs (modelV2's publish against its frame's end). REST_S is the rest of modeld's step, as the time
budget takes it, so the replay's lateness is the debt the detector counts. No frame was dropped on either drive at
lateness up to 26.8 ms (1d) and 47.9 (1e) but in 1e's 398 ms stall at the turn-desire clear (95.2 s into 1e's log, since fixed:
5044fe632), the one rise of frameDropPerc in either drive (modeldLagging soft-disables past 1%), and in the detector's
own load before the drive (50.1 and 56.2 s into the logs: 103 and 152 ms, the native schedule's warm-up runs, disengaged)."""
import sys
from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import custom, log
from openpilot.selfdrive.turnpilot.detector import modeld_detector
from openpilot.selfdrive.turnpilot.detector.modeld_detector import (CYCLE_S, DEBT_CAP_S, FORCE_RUN_FRAMES, REST_S, TIME_WINDOW,
                                                                    FarCrop, LanePaintStatus)
from openpilot.selfdrive.turnpilot.detector.tests.test_far_crop import ROAD, FarPM, FakeFarRun
from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import FakeLights, Meta, make_detector
from openpilot.selfdrive.turnpilot.scene.ranging import road_lens

FarStatus = custom.TurnpilotDetections.FarStatus

# the big model's time on the car, 11-06 (both drives' modelV2.modelExecutionTime, 30563 frames): (quantile, ms)
MODEL_Q = ((0., 37.5), (.05, 37.85), (.10, 37.96), (.25, 38.17), (.50, 38.5), (.75, 38.99), (.90, 39.53), (.95, 39.84),
           (.99, 40.55), (.999, 42.3), (1., 44.5))
FULL_S, PAINT_S, LIGHT_S = 0.0102, 0.0034, 0.0007   # the full pass's run, its paint read, its lights' (medians)
FAR_S, FAR_LIGHT_S = 0.0105, 0.0007                # the far crop's run and its lights'
# each step's time past them: a wide run's, a road run's (and its paint read's on top), a far run's
WIDE_EXTRA_S, ROAD_EXTRA_S, PAINT_EXTRA_S, FAR_EXTRA_S = 0.0013, 0.0016, 0.0006, 0.0024
ROAD_FRESH_FRAMES = 6  # unprotected_left's ROAD_FRESH_S and FAR_FRESH_S, 0.3 s: a stream's gap past this starts its window over


def model_times(n: int, seed: int = 0, shift: float = 0.) -> np.ndarray:
  q, ms = zip(*MODEL_Q, strict=True)
  return np.interp(np.random.default_rng(seed).random(n), q, ms) * 1e-3 + shift


# noise=True (car_drive): each run and each step's extra spread as on 11-06 (overhead inference over both drives' rlogs): the
# full pass's run p10-p90 10.0-10.4 ms, the far crop's 10.35-11.0, each step's extra past its runs p10-p90 about 1.6 ms wide
# (wide 3.4-5.2, road 4.1-6.7, far 4.9-6.4 with REST_S), and 1d's far p99 13.6 ms as a 1% tail of +3 ms on every timing
RUN_SD_S, EXTRA_SD_S, PAINT_SD_S, LIGHT_SD_S, TAIL_S, TAIL_P = 0.0002, 0.0008, 0.0003, 0.0001, 0.003, 0.01


def car_drive(monkeypatch, frames: int, view: str, v_ego: float, far_first: bool, shift: float = 0., seed: int = 0,
              curvature: float = 0., noise: bool = False):
  """A drive of frames steps on a simulated clock (modeld_detector.time), every run and read taking the car's time and each
  message its step's extra (the module docstring), spread as on 11-06 with noise (RUN_SD_S...): per frame what ran ('road',
  'wide', 'far' or None) and the lateness the next frame starts with (the step's time past STEP_BUDGET_S, as the time budget
  counts it)."""
  clock = SimpleNamespace(t=1000.)
  monkeypatch.setattr(modeld_detector, 'time', SimpleNamespace(monotonic=lambda: clock.t, perf_counter=lambda: clock.t))
  rng = np.random.default_rng(seed + 100)

  def spread(s, sd):
    if not noise:
      return s
    return max(0., s + rng.normal(0., sd)) + (TAIL_S if rng.random() < TAIL_P else 0.)

  def costs(fn, s, sd=RUN_SD_S):
    def run(*args, **kwargs):
      clock.t += spread(s, sd)
      return fn(*args, **kwargs)
    return run
  det = make_detector(det_time=FULL_S)
  det.light_mode, det.lights = 'addOnly', FakeLights()
  det.times = deque([FULL_S] * TIME_WINDOW, maxlen=TIME_WINDOW)
  det.paint_times.extend([PAINT_S] * TIME_WINDOW)
  det.light_times.extend([LIGHT_S] * TIME_WINDOW)
  det.run = costs(det.run, FULL_S)
  det.lane_paint.read = costs(det.lane_paint.read, PAINT_S, PAINT_SD_S)
  det.lights.read = costs(det.lights.read, LIGHT_S, LIGHT_SD_S)
  far = FarCrop(FakeFarRun(), 1344, 760, lambda crop: (f'luma{crop.x0},{crop.y0}', f'chroma{crop.x0},{crop.y0}'))
  far.lens, far.calibration, far.warmup_s = road_lens(ROAD), lambda: [0., 0., 0.], FAR_S
  far.lights = FakeLights()
  far.light_times.extend([FAR_LIGHT_S] * TIME_WINDOW)
  far.run, far.lights.read = costs(far.run, FAR_S), costs(far.lights.read, FAR_LIGHT_S, LIGHT_SD_S)
  det.far = far
  pm, ran = FarPM(), []
  send = pm.send

  def costed_send(name, msg):
    r = msg.as_reader()
    if name == 'turnpilotFarDetections':
      clock.t += spread(FAR_EXTRA_S, EXTRA_SD_S)
      ran.append('far')
    else:
      td = r.turnpilotDetections
      road = str(td.camera) == 'road'
      extra = (ROAD_EXTRA_S + (PAINT_EXTRA_S if td.lanePaintStatus == LanePaintStatus.read else 0.)) if road else WIDE_EXTRA_S
      clock.t += spread(extra, EXTRA_SD_S)
      ran.append(str(td.camera))
    send(name, msg)
  pm.send = costed_send
  model = model_times(frames, seed, shift)
  t0, late, what = clock.t + REST_S, [], []  # the first frame's on time: the last step's rest ends as it arrives
  for k in range(frames):
    start = max(clock.t + REST_S, t0 + k * CYCLE_S)  # a step late by the last one's debt starts at once on a waiting frame
    clock.t = start + float(model[k])
    n = len(ran)
    det.step(pm, det.model_state, float(model[k]), curvature, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], view, 4., v_ego, False,
             far_first)
    what.append(ran[n] if len(ran) > n else None)
    late.append(max(0., clock.t + REST_S - (t0 + (k + 1) * CYCLE_S)))
  return det, pm, what, np.array(late)


def gaps(what, kind) -> list[int]:
  at = [k for k, w in enumerate(what) if w == kind]
  return [b - a for a, b in zip(at, at[1:], strict=False)]


def test_on_the_old_schedule_a_left_had_no_far_reading(monkeypatch):
  # the schedule before far first (auto's rule at the left's views: the full pass every RUN_EVERY frames, the far crop only
  # where it leaves the next full pass its room), on the car's numbers: held at a left's line (roadAndWide), standing, no far
  # run at all; nor on its approach (road, moving, the paint read)
  for view, v in (('roadAndWide', 0.), ('road', 8.)):
    _, pm, what, late = car_drive(monkeypatch, 600, view, v, far_first=False)
    assert 'far' not in what and pm.far == [], view
    assert late.max() < CYCLE_S  # never a frame dropped


@pytest.mark.parametrize("seed", [0, 1])
def test_held_at_a_lefts_line_the_far_crop_reads_and_the_driving_model_stays_real_time(monkeypatch, seed):
  """Far first at a left's line (roadAndWide, standing: the paint skipped, nothing reads it there), 60 s on the car's numbers:
  the far crop reads about 4 times a second, its runs and the road camera's never more than ROAD_FRESH_FRAMES apart but
  rarely (unprotected_left starts a stream's window over past it), the wide camera's runs on (its exit crosswalk), every
  step's debt within DEBT_CAP_S, so modeld never drops a frame (stock 'Driving Model Lagging' soft-disables at 1%)."""
  det, pm, what, late = car_drive(monkeypatch, 1200, 'roadAndWide', 0., far_first=True, seed=seed)
  s = len(what) * CYCLE_S
  far, road, wide = gaps(what, 'far'), gaps(what, 'road'), gaps(what, 'wide')
  print(f"far {len(far) / s:.1f}/s max {max(far)}, road {len(road) / s:.1f}/s max {max(road)}, wide {len(wide) / s:.1f}/s max {max(wide)}")
  assert len(far) / s > 3.5 and len(road) / s > 3.5 and len(wide) / s > 1.5
  assert sum(g > ROAD_FRESH_FRAMES for g in far) <= 0.01 * len(far) and sum(g > ROAD_FRESH_FRAMES for g in road) <= 0.01 * len(road)
  assert max(far) <= ROAD_FRESH_FRAMES + 2 and max(road) <= ROAD_FRESH_FRAMES + 2 and max(wide) < FORCE_RUN_FRAMES
  assert within_cap(late)
  assert all(str(td.lanePaintStatus) == 'noRoom' for td in pm.full if str(td.camera) == 'road')
  assert not det.far.disabled and det.far.failures == 0
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import LEFT_ROAD_RUNS_HZ
  assert len(road) / s >= LEFT_ROAD_RUNS_HZ  # what tools/turnpilot/left_range.py takes a left's road runs to be


def road_hits_per_window(what) -> list[int]:
  """Road camera runs in each CONFIRM_WINDOW_S ending on each frame (scene_policy's hard tier counts a road-only track's hits
  there: CONFIRM_RECENT_HITS), from the first full window on."""
  from openpilot.selfdrive.turnpilot.scene_policy import CONFIRM_WINDOW_S
  n = round(CONFIRM_WINDOW_S / CYCLE_S)
  road = np.cumsum([w == 'road' for w in what])
  return [int(road[k] - road[k - n]) for k in range(n, len(what))]


@pytest.mark.parametrize("v", [8., 4., 2.5])
def test_moving_far_first_keeps_the_full_passs_cadence(monkeypatch, v):
  """Far first moving (a left's approach from VIEW_FROM_M, its go until the commit) is auto's schedule: the road camera's runs
  as many as without it, 10 a second, so a person only the road camera sees keeps the hard brake's CONFIRM_RECENT_HITS in
  every CONFIRM_WINDOW_S with room for missed boxes, the light vote its 4 of 5 at 10 Hz, and the paint read as before
  (PAINT_FORCE_S). Until the 11-06 review (2026-11-07) far first took the full pass's frames moving too: the road camera 5 a
  second and the far crop 5, a road-only track's hits 5 a window (the hard tier steady on 57.5% of frames at a 0.9 recall),
  for far runs the left throws away (the latch restarts the window and far_stale)."""
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import PAINT_FORCE_S
  from openpilot.selfdrive.turnpilot.scene_policy import CONFIRM_RECENT_HITS
  _, pm_old, what_old, late_old = car_drive(monkeypatch, 1200, 'road', v, far_first=False)
  det, pm, what, late = car_drive(monkeypatch, 1200, 'road', v, far_first=True)
  s = len(what) * CYCLE_S
  road, road_old = what.count('road'), what_old.count('road')
  assert road / s > 9.5 and abs(road - road_old) <= 0.01 * road_old, (road, road_old)
  assert min(road_hits_per_window(what)) >= 2 * CONFIRM_RECENT_HITS - 1  # 5 a window, before: no room for one missed box
  assert not det.far_first
  painted = [k for k, td in enumerate(pm.full) if td.lanePaintStatus == LanePaintStatus.read]
  assert painted and max(b - a for a, b in zip(painted, painted[1:], strict=False)) * 0.1 <= PAINT_FORCE_S + 0.5
  assert (late > DEBT_CAP_S + 1e-9).sum() <= (late_old > DEBT_CAP_S + 1e-9).sum() and late.max() < CYCLE_S / 4
  # with the 11-06 spread on every timing and a 0.9 per-run recall, the hard tier steady on every frame but a rare few
  _, _, what, _ = car_drive(monkeypatch, 2400, 'road', v, far_first=True, noise=True)
  rng = np.random.default_rng(7)
  seen = ['road' if w == 'road' and rng.random() < 0.9 else None for w in what]
  assert np.mean([n >= CONFIRM_RECENT_HITS for n in road_hits_per_window(seen)]) > 0.99


def left_waits(what, starts: int = 200, horizon_s: float = 60.) -> list[tuple[float, bool]]:
  """unprotected_left's clear window (UnprotectedLeft.update's road_age, clear_s, far_age, far_stale) on the detector's
  stream what, each run seen on the frame it ran (1:1, as the closed-loop tests take it), an empty band, R_req past the full
  pass's range (far needed), nothing else holding the car: from evenly spaced latch frames (the hold, where clear_s and
  far_stale start over), (the s it waits at its line until CLEAR_WINDOW_S certifies, horizon_s if never; whether any of
  that window went with the far slot stale past STALE_UNDECIDED_S: far_needed off, the full pass's band alone)."""
  from openpilot.selfdrive.turnpilot.unprotected_left import CLEAR_WINDOW_S, FAR_FRESH_S, ROAD_FRESH_S, STALE_UNDECIDED_S
  dt, horizon, out = CYCLE_S, round(horizon_s / CYCLE_S), []
  for s in np.linspace(40, len(what) - horizon - 1, starts).astype(int):
    road_age = far_age = far_stale = clear = 0.
    went, blind = (horizon_s, False), False
    for k in range(s - 40, s + horizon):
      road_age, far_age, far_stale = road_age + dt, far_age + dt, far_stale + dt
      if what[k] == 'road':
        clear, road_age = clear + dt if road_age <= ROAD_FRESH_S else 0., 0.
      else:
        clear = 0. if road_age > ROAD_FRESH_S else clear + dt
      if what[k] == 'far':
        far_age = far_stale = 0.
      needed = far_stale <= STALE_UNDECIDED_S
      if needed and far_age > FAR_FRESH_S:
        clear = 0.
      if k == s:
        clear = far_stale = 0.
      blind = blind or (k >= s and not needed)
      if k >= s and clear >= CLEAR_WINDOW_S - 1e-6:
        went = ((k - s) * dt, blind)
        break
    out.append(went)
  return out


def test_far_first_at_the_line_with_the_11_06_spread(monkeypatch):
  """Held at a left's line on the car's budget with every timing spread as on 11-06 (noise: RUN_SD_S...), unprotected_left's
  2.5 s window on the far-first cadence (left_waits): with an empty road it goes at once on most holds and within a few
  seconds on nearly all, at 11-06's model and at one 1 ms slower; modeld never drops a frame. At a model 2 ms slower the
  budget is past what 0.3 s ROAD_FRESH_S and FAR_FRESH_S leave room for (road and far 4 a second each and the wide camera 2
  need 10 runs a second of the 10 the frames hold): the left waits longer there (the 11-06 review, 2026-11-07: a median of
  15-19 s), never shorter; but the far slot's gaps stay within STALE_UNDECIDED_S (FAR_FORCE_S), so no left is judged blind
  past the full pass's ~98 m for the far crop's own wait to measure again (at FAR_FORCE_S 2 s: gaps of 2.3-2.4 s)."""
  from openpilot.selfdrive.turnpilot.unprotected_left import STALE_UNDECIDED_S
  for shift in (0., 0.001):
    _, _, what, late = car_drive(monkeypatch, 6000, 'roadAndWide', 0., far_first=True, shift=shift, noise=True)
    waits = left_waits(what)
    w = np.array([x for x, _ in waits])
    print(f"+{shift * 1e3:.0f} ms: wait p50 {np.median(w):.1f} p90 {np.percentile(w, 90):.1f} max {w.max():.1f}")
    assert np.median(w) <= 2.6 and np.percentile(w, 90) <= 6. and w.max() <= 15., shift
    # a road run due judged as auto judges it (R1, review 2026-11-07) may run a step's extra past the cap: 13 ms at most here
    assert not any(blind for _, blind in waits) and late.max() < CYCLE_S / 3, shift
  for seed in (0, 1):
    _, _, what, late = car_drive(monkeypatch, 6000, 'roadAndWide', 0., far_first=True, shift=0.002, seed=seed, noise=True)
    far = gaps(what, 'far')
    assert len(far) / (len(what) * CYCLE_S) > 2.5 and max(far) * CYCLE_S < STALE_UNDECIDED_S, (seed, max(far))
    assert not any(blind for _, blind in left_waits(what)) and late.max() < CYCLE_S / 3, seed


def test_a_warm_chestnut_spaces_the_far_runs_and_never_drops_a_frame(monkeypatch):
  # the model 1-2 ms slower than 11-06's (a warm eGPU): at the line the far crop still reads, its runs spaced further where
  # the budget is short (the left waits longer: a far gap past FAR_FRESH_S starts its window over), every step's debt
  # within DEBT_CAP_S
  for shift, far_hz in ((0.001, 3.5), (0.002, 3.0)):
    _, _, what, late = car_drive(monkeypatch, 1200, 'roadAndWide', 0., far_first=True, shift=shift)
    s = len(what) * CYCLE_S
    assert len(gaps(what, 'far')) / s > far_hz and len(gaps(what, 'road')) / s > 3.5, shift
    assert within_cap(late), shift


def within_cap(late) -> bool:
  """Every step's debt within DEBT_CAP_S, but a road run due (modeld_detector.road_due: its gap at ROAD_DUE_S) judged as auto
  judges it, without its step's extra (review 2026-11-07, R1): such a step past the cap by the extra at most
  (FULL_STEP_OVERHEAD_S), on 1% of steps at most. Never near a dropped frame (a whole CYCLE_S)."""
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import FULL_STEP_OVERHEAD_S
  return late.max() <= DEBT_CAP_S + FULL_STEP_OVERHEAD_S + 1e-9 and (late > DEBT_CAP_S + 1e-9).mean() <= 0.01


# the light read's cost on the car by the heads in a run (11-06 rlogs, HEAD's reader: 0, 1, 2, 3, 4 heads), ms
LIGHT_HEADS_MS = {0: 0.73, 1: 2.02, 2: 3.29, 3: 4.14, 4: 4.64}


def light_cost(heads: int, factor: float = 1.) -> float:
  """s a light read with heads heads costs, the per-head part factor times the car's (this tree's reader ~1.4x HEAD's on
  a Mac, unmeasured on the comma)."""
  return (LIGHT_HEADS_MS[0] + factor * (LIGHT_HEADS_MS[heads] - LIGHT_HEADS_MS[0])) * 1e-3


@pytest.mark.parametrize("road_heads,far_heads,factor", [(1, 2, 1.), (2, 2, 1.), (1, 2, 1.4), (2, 2, 1.4)])
def test_at_a_signalized_lefts_line_the_light_reads_never_starve_the_road_camera(monkeypatch, road_heads, far_heads, factor):
  """Review 2026-11-07 (R1, far first): at a signalized left's line with heads in view (the 11-06 stands: 0-2 in the road
  camera's runs; the far-side heads by geometry in the far crop's window by day, at twice the full pass's pixel density) the
  far crop's shadow light reads (nothing acts on them at a left) and the full pass's far-first room rule (its step's extra
  on top of its lights) starved both streams: an empty road's window waited p50 8-19 s, 0.5-2.7 far runs a second, or (3
  road heads) never filled. Far first now reads no far lights, and a road run due is judged as auto judges it: the window
  fills as on an unsignalized road's, its streams within ROAD_FRESH_S, every step within the cap but a rare extra's."""
  monkeypatch.setattr(sys.modules[__name__], 'LIGHT_S', light_cost(road_heads, factor))
  monkeypatch.setattr(sys.modules[__name__], 'FAR_LIGHT_S', light_cost(far_heads, factor))
  _, pm, what, late = car_drive(monkeypatch, 6000, 'roadAndWide', 0., far_first=True, noise=True)
  w = np.array([x for x, _ in left_waits(what)])
  s = len(what) * CYCLE_S
  assert np.median(w) <= 2.6 and np.percentile(w, 90) <= 6. and w.max() < 60., (np.median(w), np.percentile(w, 90), w.max())
  assert what.count('road') / s > 3.8 and what.count('far') / s > 3.8
  assert pm.far and all(str(fd.run.lightStatus) == 'off' for fd in pm.far)  # far first: no far light read
  assert late.max() < CYCLE_S / 3  # never near a dropped frame


def real_left_waits(what, starts: int = 100, horizon_s: float = 60.) -> list[float]:
  """unprotected_left.UnprotectedLeft itself (test_unprotected_left's Junction) on the detector's stream what: a 35 mph left
  (R_req past the full pass's ~98 m), the far crop live at its provisional FAR_CROP_LEFT_RANGE_M, an empty band, each run seen
  on the frame it ran, 40 frames of the approach's stream first: from evenly spaced latch frames the s it waits at its line
  until its go (horizon_s if never)."""
  from openpilot.selfdrive.turnpilot.maps.defaults import FAR_CROP_LEFT_RANGE_M
  from openpilot.selfdrive.turnpilot.tests.test_unprotected_left import Junction
  from openpilot.selfdrive.turnpilot.unprotected_left import FULL_PASS_RANGE_M
  horizon, out = round(horizon_s / CYCLE_S), []
  for s in np.linspace(40, len(what) - horizon - 1, starts).astype(int):
    j = Junction(v_limit=15.65, lanes=1, rng=FULL_PASS_RANGE_M)
    went = horizon_s
    for k in range(s - 40, s + horizon):
      j.held = k >= s
      w = what[k]
      d = j.step(raw_new=w in ('road', 'wide'), road=w == 'road', far=[] if w == 'far' else None, far_range_m=FAR_CROP_LEFT_RANGE_M,
                 far_live=True)
      if k >= s and d.go:
        went = (k - s) * CYCLE_S
        break
    out.append(went)
  return out


@pytest.mark.parametrize("shift_ms", [2.5, 3.])
def test_a_far_crop_running_too_slowly_to_hold_its_freshness_never_holds_an_empty_roads_left_forever(monkeypatch, shift_ms):
  """Review 2026-11-07 (R2): with the chestnut's model 2.5-3 ms slower than 11-06's (a warm cabin by day: not seen on the
  three chestnut drives, whose segment medians stayed 38.1-39.2 ms), the far crop runs 1.4-2.1 times a second, its gaps
  0.3-2.6 s: far_needed dropped only past one gap over STALE_UNDECIDED_S and came back on the next run, and every far gap
  past FAR_FRESH_S started the window over, so the real UnprotectedLeft held an empty road's left at its line on 100 of 100
  holds over 60 s, saying "Waiting for a gap" (a far crop that didn't run went at once on the full pass's band). Now, the
  full pass's band clear a whole window with the far slot's age alone holding it, the left is judged on that band (far_behind)
  and goes, a far box still holding it."""
  from openpilot.selfdrive.turnpilot import unprotected_left as ul
  monkeypatch.setattr(ul.cloudlog, 'event', lambda *a, **kw: None)
  _, _, what, _ = car_drive(monkeypatch, 6000, 'roadAndWide', 0., far_first=True, shift=shift_ms * 1e-3, noise=True)
  w = np.array(real_left_waits(what))
  assert np.median(w) <= 6. and np.percentile(w, 90) <= 15. and (w >= 60.).sum() == 0, (np.median(w), np.percentile(w, 90), int((w >= 60.).sum()))


def far_first_detector(monkeypatch, model_s=0.0385):
  """car_drive's detector on its clock, stepped by hand: (det, pm, step(view, v, far_first) -> what ran)."""
  det, pm, _, _ = car_drive(monkeypatch, 0, 'roadAndWide', 0., far_first=True)
  clock = modeld_detector.time
  k = [0]

  def step(view='roadAndWide', v=0., far_first=True):
    k[0] += 1
    clock_t = max(clock.monotonic() + REST_S, 1000. + REST_S + k[0] * CYCLE_S) + model_s
    monkeypatch.setattr(modeld_detector, 'time', SimpleNamespace(monotonic=lambda: clock_t, perf_counter=lambda: clock_t))
    n, nf = len(pm.full), len(pm.far)
    det.step(pm, det.model_state, model_s, 0.0, log.Desire.none, [Meta(2 * k[0]), Meta(2 * k[0] + 1)], view, 4., v, False, far_first)
    return 'far' if len(pm.far) > nf else str(pm.full[-1].camera) if len(pm.full) > n else None
  return det, pm, step


def test_a_far_run_due_first_takes_the_full_passs_frame(monkeypatch):
  # far first the frame goes to whichever pass is due first that fits on its own step cost: the far crop, its last run
  # FAR_DUE_S ago, takes a frame the full pass is due on when the road camera's last run is more recent; in auto the full
  # pass always wins its frames (the far crop only between them, leaving the next full run its room)
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import FAR_DUE_S, ROAD_DUE_S
  det, pm, step = far_first_detector(monkeypatch)
  det.run_t, det.full_extra = {}, deque(maxlen=TIME_WINDOW)
  now = modeld_detector.time.monotonic()
  det.since_run, det.debt = 5, 0.
  det.run_t['road'] = now - 0.05      # the road camera ran a frame ago: next due ROAD_DUE_S after
  det.far.last_run_t = now - 0.2      # the far crop's due FAR_DUE_S after: sooner
  assert det.far.last_run_t + FAR_DUE_S < det.run_t['road'] + ROAD_DUE_S
  assert step(view='road') == 'far' and det.since_run == 6  # the full pass waits for the next frame it fits
  det.far.last_run_t, det.debt = modeld_detector.time.monotonic() - 0.2, 0.
  assert step(view='road', far_first=False) == 'road'  # auto: the full pass's frame


def test_far_first_skips_the_paint_standing_and_reads_it_moving(monkeypatch):
  # under PAINT_SKIP_V nothing reads the paint (the wrong-side guard and the lane changes from 2.0 and 2.74 m/s): far first
  # it is skipped outright there (noRoom), the far crop's time; from PAINT_SKIP_V up it is read as before
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import PAINT_SKIP_V
  for v, want in ((0., LanePaintStatus.noRoom), (PAINT_SKIP_V - 0.1, LanePaintStatus.noRoom), (PAINT_SKIP_V + 0.5, LanePaintStatus.read)):
    det, pm, step = far_first_detector(monkeypatch, model_s=0.020)  # room for everything
    det.far.last_run_t = modeld_detector.time.monotonic() + 10.  # the far crop not due: the full pass's frames
    ran = [step(view='road', v=v) for _ in range(4)]
    road = [td for td in pm.full if str(td.camera) == 'road']
    assert road and all(td.lanePaintStatus == want for td in road), (v, ran)
  # auto standing with no signal mapped near: the paint read as before
  det, pm, step = far_first_detector(monkeypatch, model_s=0.020)
  for _ in range(4):
    step(view='road', v=0., far_first=False)
  assert all(td.lanePaintStatus == LanePaintStatus.read for td in pm.full if str(td.camera) == 'road')


def test_far_first_is_the_far_crops_slot_never_the_natives(monkeypatch):
  # the native strips (15.4 ms warm on the car) need a paid-back step and another after: far first the slot is the far
  # crop's (aimed up the oncoming approach, the strips' pixel density), the native's only with no far crop; its no-run clock
  # held meanwhile (native_hold farFirst), so a left's wait never says it never ran
  from openpilot.selfdrive.turnpilot.detector.tests.test_far_crop import native_slot
  det, pm, step = far_first_detector(monkeypatch, model_s=0.020)
  native = native_slot(det, native_time=0.0154)
  assert det.left_far() is det.far
  ran = [step(view='roadAndWide') for _ in range(8)]
  assert 'far' in ran and native.run.calls == [] and len(pm.far) == ran.count('far')
  assert all(tuple(fd.crop)[2] - tuple(fd.crop)[0] == 640 for fd in pm.far)  # the far crop's window, never a strip's
  assert 'farFirst' in det.native_holds
  step(view='auto', far_first=False)
  assert 'farFirst' not in det.native_holds
  det.far = None  # no far crop: the native's
  assert det.left_far() is native


def test_a_failing_far_crop_stays_disabled_through_a_sweep(monkeypatch):
  # FAR_DISABLED_FAILS in a row say disabled (turnpilotd's farViewOff, once); a stop sweep suspends its runs, so its retries
  # wait, and saying sweep (on, to turnpilotd) through it and disabled after would chime Far View Off after every hold
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import FAR_DISABLED_FAILS
  det, pm, step = far_first_detector(monkeypatch, model_s=0.020)
  det.far.failures = FAR_DISABLED_FAILS
  assert det.far_status() == FarStatus.disabled
  det.sweep = 'stopSweep'
  assert det.far_status() == FarStatus.disabled
  det.far.failures = 1  # one failure: on, its retry waiting
  assert det.far_status() == FarStatus.sweep


def test_the_host_passes_far_first(monkeypatch):
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import DetectorHost
  from openpilot.selfdrive.turnpilot.detector.tests.test_detector_recovery import FakeLoad, FakeParams
  from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import PM
  model, pm = object(), PM()
  host = DetectorHost(FakeParams(True), 1344, 760, False, True, [model], load_fn=FakeLoad({model}))
  host.load()
  (_, det), = host.detectors
  host.step(pm, model, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)], True, 'roadAndWide', 4., 0., False, True)
  assert det.far_first
  host.step(pm, model, 0.010, 0.0, log.Desire.none, [Meta(3), Meta(4)], True, 'roadAndWide', 4., 0., False)
  assert not det.far_first
