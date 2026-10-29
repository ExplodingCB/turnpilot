"""stop_evidence.py (STOP_SIGNS.md 1.7, 2.3; SG3b): each cross track's history on camera time, its lane from its bearing ray,
its distance at sLaneMin with the bearing toward the axis and its speed at sLaneMax toward the edge, TTA to our conflict
point and the bearing-rate cross-check, trust in time, U2, stopped-at-line, the shield and through-at-speed. Expected values
are computed here from the geometry (repo tests can't read ~/turnpilot-artifacts)."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.stop_evidence import SIDE_PIXEL_RAD, StopEvidence, junction_lanes

MPH = sr.MPH
CAR, BICYCLE = 2, 1


def junction(limit_mph=25., tier='surveyed'):
  """A surveyed 4-way all-way, 2-lane roads, our heading 0 (north), at its default stop point 13 m before the node."""
  v = limit_mph * MPH
  legs = [sr.Approach(270., -90., 'left', 'residential', 'road', True, True, 1, 1, 'stop', tier, v, 11),
          sr.Approach(90., 90., 'right', 'residential', 'road', True, True, 1, 1, 'stop', tier, v, 12),
          sr.Approach(0., 0., 'opposite', 'residential', 'road', True, True, 1, 1, 'stop', tier, v, 13)]
  return sr.Junction(osm_node_id=99, lat=39.8, lon=-86.15, stop_point_distance=13., heading_in=0., maneuver='straight',
                     rail_distance=-1., sidecar=True, surveyed=True, approaches=legs)


def lanes(setback=1.9):
  return junction_lanes(junction(), 'straight', setback)


def lane(role):
  return next(ln for ln in lanes() if ln.role == role)


def track(tid, t, d, ln, view='sideLeft', cls=CAR, jitter_px=0., rng=None, s=None):
  """A tracker Track's stop-evidence fields for a car d m down lane ln (from where it crosses our path) at camera time t,
  seen from the stop point: the bearing of its bottom centre (left positive), its box width (1.8 m at the side view's
  904 px focal length), the view, its class and its ground point (tracker frame, y left)."""
  s = (ln.s_min + ln.s_max) / 2. if s is None else s
  left = ln.role != 'right'
  psi = math.atan2(d, s)
  if jitter_px:
    psi += rng.normal(0., jitter_px * SIDE_PIXEL_RAD)
  bearing = math.degrees(psi) * (1. if left else -1.)
  return SimpleNamespace(track_id=tid, t_seen=t, bearing_seen=bearing, width_seen=1.8 * 904. / math.hypot(d, s), view=view, cls=cls,
                         x=np.array([s, d if left else -d, 0., 0.]))


def drive(ev, tid, ln, d_at, t_end, hz=5., t0=0., view='sideLeft', cls=CAR, jitter_px=0., seed=0, each=None):
  """Feed ev a car whose distance down ln at time t is d_at(t), at hz, from t0 to t_end; each(t, out) after every frame."""
  rng = np.random.default_rng(seed)
  n = int(round((t_end - t0) * hz))
  out = None
  for k in range(n + 1):
    t = t0 + k / hz
    ev.observe([track(tid, t, d_at(t), ln, view, cls, jitter_px, rng)], d_stop=0.)
    out = ev.evaluate()
    if each is not None:
      each(t, out[tid])
  return out[tid]


def evidence(lns=None):
  ev = StopEvidence()
  ev.set_junction(lns or lanes(), sr.angle_bound(True), lines={'left': 9., 'right': 9., 'opposite': 9.})
  return ev


def braking(d0, v0, a):
  def d_at(t):
    tt = min(t, v0 / a) if a > 0 else t
    return d0 - (v0 * tt - 0.5 * a * tt * tt)
  return d_at


def test_lane_assignment_by_bearing_side_and_range():
  ev = evidence()
  ln_l, ln_r = lane('left'), lane('right')
  ev.observe([track(1, 0., 30., ln_l), track(2, 0., 30., ln_r, view='sideRight')], d_stop=0.)
  out = ev.evaluate()
  assert (out[1].role, out[1].index) == ('left', 1) and (out[2].role, out[2].index) == ('right', 1)


def test_distance_at_s_lane_min_toward_the_axis_and_speed_at_s_lane_max_toward_the_edge():
  ln = lane('left')
  ev = evidence()
  e = drive(ev, 1, ln, braking(40., 10., 0.), 1.4)
  d_true = 40. - 10. * 1.4
  bound = sr.angle_bound(True)
  psi = math.degrees(math.atan2(d_true, (ln.s_min + ln.s_max) / 2.))
  # the pessimistic distance: sLaneMin, the bearing rotated toward the axis (before 2 sigma)
  assert e.d_lo <= ln.s_min * math.tan(math.radians(psi - bound)) + 1e-6 < d_true
  assert e.v_hi > 10. and e.d_nom == pytest.approx(d_true, abs=0.3) and e.v_nom == pytest.approx(10., abs=0.3)


def test_a_car_at_12_m_s_from_40_m_tta_within_10_percent_and_untrusted_before_1_s():
  ln = lane('left')
  ev = evidence()
  seen = []

  def each(t, e):
    seen.append((t, e))
  ln = lane('left')
  drive(ev, 1, ln, braking(40., 12., 0.), 2.0, each=each)
  for t, e in seen:
    truth = (40. - 12. * t) / 12.
    if t < 1.0 - 1e-9:
      assert not e.v_trusted and e.u2 == 'notStopping'
      continue
    assert e.v_trusted
    assert abs(e.tta - truth) <= 0.1 * truth                 # to our conflict point (0 m along this lane)
    assert abs(e.tta_foot - truth) <= 0.1 * truth            # the s-free bearing-rate form, perpendicular lane
    assert e.tta_lo <= truth + 1e-6                           # the pessimistic ends arrive no later than the truth
    # the box expansion's time to contact is the camera range's r / -dr/dt, (s^2 + D^2) / (D v), not the lane's TTA
    d, s = 40. - 12. * t, (ln.s_min + ln.s_max) / 2.
    ttc = (s * s + d * d) / (d * 12.)
    assert e.ttc_expansion is not None and abs(e.ttc_expansion - ttc) <= 0.1 * ttc


def test_U2_braking_at_1_5_from_40_m_at_12_85_m_s_is_not_stopping():
  # 12.85 m/s (about 28.7 mph) needs 55 m at 1.5 m/s^2: it can't stop before our path from 40 m
  ln = lane('left')
  ev = evidence()
  states = []
  drive(ev, 1, ln, braking(40., 12.85, 1.5), 2.6, each=lambda t, e: states.append(e.u2))
  assert 'stopping' not in states and 'stopped' not in states and states[-1] == 'notStopping'


def test_U2_two_pixels_of_bearing_jitter_never_reads_as_stopping():
  ln = lane('left')
  for seed in range(20):
    ev = evidence()
    states: list[str] = []
    drive(ev, 1, ln, braking(45., 11., 0.), 3.0, jitter_px=2., seed=seed, each=lambda t, e, out=states: out.append(e.u2))
    assert 'stopping' not in states and 'stopped' not in states


def test_V2_a_roller_at_an_all_way_is_never_stopping_and_one_through_at_speed_is_flagged():
  ln = lane('left')
  # a rolling stop: slowing at 1.0 m/s^2 from 7 m/s to 3 m/s by its line and on through it
  def roll(t):
    tt = min(t, 4.0)
    d = 30. - (7. * tt - 0.5 * tt * tt)
    return d - 3. * max(t - 4.0, 0.)
  ev = evidence()
  states, stopped = [], []
  e = drive(ev, 1, ln, roll, 6.0, each=lambda t, e: (states.append(e.u2), stopped.append(e.stopped_at_line)))
  assert 'stopping' not in states and 'stopped' not in states and not any(stopped)
  assert not e.through_at_speed  # 3 m/s, under DOWNGRADE_V
  # a car through its line at 8 m/s: through at speed once past its line, never stopped
  ev = evidence()
  e = drive(ev, 2, ln, braking(35., 8., 0.), 4.0)
  assert e.through_at_speed and not e.stopped_at_line
  # a bicycle through at speed is never a downgrade (motor vehicles only)
  ev = evidence()
  assert not drive(ev, 3, ln, braking(35., 8., 0.), 4.0, cls=BICYCLE).through_at_speed


def test_stopped_at_its_line_and_the_shield():
  ln = lane('left')
  ev = evidence()
  # braking at 2.5 m/s^2 from 8 m/s 23.8 m out: stands with its front at its line (9 m down the lane; its box's bottom
  # centre CAR_HALF_LEN_M further), then waits
  e = drive(ev, 1, ln, braking(23.8, 8., 2.5), 6.0)
  assert e.stopped_at_line and e.u2 == 'stopped' and e.stopped_s >= 1.0 and e.shield
  # it starts moving: no shield at once
  e = drive(ev, 1, ln, lambda t: 11. - 1.5 * (t - 6.0), 6.4, t0=6.2)
  assert not e.shield
  # a car waiting 10 m back from its line (the parked case) is no shield
  ev = evidence()
  e = drive(ev, 2, ln, lambda t: 21.3, 3.0)
  assert not e.stopped_at_line and not e.shield
  # a bicycle at its line is never a shield
  ev = evidence()
  b = drive(ev, 3, ln, lambda t: 11.3, 3.0, cls=BICYCLE)
  assert b.stopped_at_line and not b.shield  # it holds a rank, never shields
  # missed runs of its view (a hit every 0.6 s): not seen on every cycle, no shield
  ev = evidence()
  assert not drive(ev, 4, ln, lambda t: 11.3, 4.2, hz=1. / 0.6).shield


def test_history_is_on_camera_time_and_restarts_when_our_car_moves():
  ln = lane('left')
  ev = evidence()
  drive(ev, 1, ln, braking(40., 12., 0.), 1.2)
  assert ev.evaluate()[1].v_trusted
  # the same track seen again with our car 1 m further on: its old bearings were taken from elsewhere, so trust restarts
  ev.observe([track(1, 1.4, 40. - 12. * 1.4, ln, s=(ln.s_min + ln.s_max) / 2. - 1.)], d_stop=-1.)
  assert not ev.evaluate()[1].v_trusted
  # a repeat of the last match (no new camera time) adds nothing
  n = len(ev.hist[1])
  ev.observe([track(1, 1.4, 40. - 12. * 1.4, ln)], d_stop=-1.)
  assert len(ev.hist[1]) == n


def test_first_track_range_is_the_pessimistic_distance_end():
  ln = lane('left')
  ev = evidence()
  drive(ev, 1, ln, braking(38., 10., 0.), 1.0)
  first = ev.first[1]
  psi = math.degrees(math.atan2(38., (ln.s_min + ln.s_max) / 2.))
  assert first.range_m == pytest.approx(ln.s_min * math.tan(math.radians(psi - sr.angle_bound(True))), abs=1e-6)
  assert first.range_m < 38. and first.view == 'sideLeft' and first.t == 0.


def test_through_the_tracker_its_matches_bearings_and_widths():
  # the plumbing: a car crossing the left side view's field, detected as modeld publishes a side view's boxes (the wide
  # camera's fisheye pixels, view sideLeft), tracked, its matches' bearings, widths and camera times into the evidence
  from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import CAMS, calibration, car_state
  from openpilot.selfdrive.turnpilot.scene.tests.test_tracker_views import run
  from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  ln = lane('left')
  s = (ln.s_min + ln.s_max) / 2.
  ev = evidence()
  for k in range(9):
    t = 0.1 + 0.2 * k
    trk.update(run(trk, 'wide', 'sideLeft', t, [(CAR, s, 30. - 5. * t, 0.7)], k), cal, car_state(), now=t)
    ev.observe(trk.tracks, d_stop=0.)
  (e,) = ev.evaluate().values()
  assert (e.role, e.index, e.view) == ('left', 1, 'sideLeft') and e.v_trusted and e.samples == 9  # every match in the fit
  assert e.d_nom == pytest.approx(30. - 5. * 1.7, abs=0.5) and e.v_nom == pytest.approx(5., abs=0.5)
  # its first track is its second match, where the tracker publishes it (MIN_HITS): two hits make a track (1.4)
  assert abs(e.t - 1.7) < 1e-6 and abs(ev.first[e.track_id].t - 0.3) < 1e-6
