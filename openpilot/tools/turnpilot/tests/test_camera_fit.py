"""camera_fit.py (STOP_SIGNS_SATURDAY.md T1, step S1): the lens fit from a person standing on taped marks, the frame edge,
the static-dark mask, darkThreshold, every refusal, and the copy of visibility.json it writes. Pure functions on synthetic
data: no YOLO, no video."""
import json
import math

import numpy as np
import pytest

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table as vt
from openpilot.selfdrive.turnpilot.detector import sideview
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view
from openpilot.tools.turnpilot import camera_fit as cf

W, H = cf.CAM_W, cf.CAM_H
CAL = cf.Calib((0.0, 0.012, -0.006), (0.0, 0.004, 0.003))  # rpyCalib, wideFromDeviceEuler: rad
TAPE = cf.Tape(1.92, 0.05, 1.31, '2026-10-31')
ORDER = [cf.Mark(s, b) for s in ('left', 'right') for b in cf.MARK_BEARINGS]


def true_lens(f=452., k=0.3):
  return Lens(W, H, f, fisheye=True, k=k)


def dwells_for(lens, delta, marks=ORDER, noise_px=0.3, seed=0, height=TAPE.height, skip=()):
  """A dwell per mark the lens puts in the frame (1.5 px inside), in standing order, 10 s apart."""
  rng = np.random.default_rng(seed)
  r = calib_from_view(list(CAL.rpy), list(CAL.euler))
  out = []
  for i, m in enumerate(marks):
    if (m.side, m.bearing) in skip:
      continue
    u, v = cf.mark_pixel(lens, r, m.side, m.bearing, height, cf.RANGE_M, delta)
    if not (1.5 <= u <= W - 1.5 and 1.5 <= v <= H - 1.5):
      continue
    out.append(cf.Dwell(10. * i, 10. * i + 6., float(u + rng.normal(0, noise_px)), float(v + rng.normal(0, noise_px)), 'sideLeft'
                        if m.side == 'left' else 'sideRight', 30))
  return out


def test_a_synthetic_lens_is_recovered_within_0_2_deg():
  lens = true_lens()
  dw = dwells_for(lens, 0.2)
  fit = cf.solve(dw, ORDER, CAL, TAPE.height)
  assert abs(fit.delta - 0.2) < 0.2 and abs(fit.f - 452.) < 5. and abs(fit.k - 0.3) < 0.05
  assert fit.rms_b < 0.2 and max(abs(x) for x in fit.resid_b + fit.resid_e) < 0.3
  truth = cf.edge_deg(lens, calib_from_view(list(CAL.rpy), list(CAL.euler)))
  for side in ('left', 'right'):
    assert abs(fit.edge[side] - truth[side]) < 0.2
  # the order resolves the outer marks, whose pixel brackets overlap: every dwell got its own mark
  assert [(m.side, m.bearing) for _, m in fit.assignment] == [(m.side, m.bearing) for m in ORDER
                                                             if any(abs(d.t0 - 10. * ORDER.index(m)) < 1e-9 for d in dw)]


def test_a_1_deg_yaw_offset_is_refused():
  with pytest.raises(cf.Refusal, match='yaw offset'):
    cf.solve(dwells_for(true_lens(), 1.0), ORDER, CAL, TAPE.height)


def test_a_mark_the_fit_puts_inside_the_frame_but_unseen_is_refused():
  dw = dwells_for(true_lens(), 0.1, skip={('left', 85)})
  with pytest.raises(cf.Refusal, match='unseen'):
    cf.solve(dw, ORDER, CAL, TAPE.height)


def test_too_few_inner_marks_seen_on_a_side_is_refused():
  marks = [m for m in ORDER if not (m.side == 'right' and m.bearing in (55, 70))]
  with pytest.raises(cf.Refusal, match='40-80 deg marks'):
    cf.solve(dwells_for(true_lens(), 0.1, marks=marks), marks, CAL, TAPE.height)


def test_a_dwell_that_matches_no_mark_is_refused():
  dw = dwells_for(true_lens(), 0.1)
  dw.append(cf.Dwell(500., 506., 672., 600., 'full', 30))  # someone standing ahead of the car, low in the frame
  with pytest.raises(cf.Refusal, match='matches no mark'):
    cf.solve(dw, ORDER, CAL, TAPE.height)


def test_a_wrong_order_is_refused():
  dw = dwells_for(true_lens(), 0.1)
  dw[0], dw[1] = cf.Dwell(dw[1].t0 + 50., dw[1].t1 + 50., dw[0].u, dw[0].v, dw[0].view, 30), dw[1]
  with pytest.raises(cf.Refusal):
    cf.solve(sorted(dw, key=lambda d: d.t0), ORDER, CAL, TAPE.height)


def test_a_noisy_or_misplaced_person_is_refused_by_the_residuals():
  dw = dwells_for(true_lens(), 0.1)
  dw[2] = cf.Dwell(dw[2].t0, dw[2].t1, dw[2].u - 20., dw[2].v, dw[2].view, 30)  # 70 deg left, about 2.5 deg out
  with pytest.raises(cf.Refusal, match='residual|RMS'):
    cf.solve(dw, ORDER, CAL, TAPE.height)


def test_a_lens_outside_the_bracket_is_refused():
  with pytest.raises(cf.Refusal, match='outside|matches no mark'):
    cf.solve(dwells_for(true_lens(f=500.), 0.1), ORDER, CAL, TAPE.height)
  with pytest.raises(cf.Refusal, match='outside'):
    cf.check_lens_bracket(480., 0.3)
  with pytest.raises(cf.Refusal, match='outside'):
    cf.check_lens_bracket(452., 0.65)


def test_dwells_are_a_still_feet_point_for_4_s():
  dets = []
  for i in range(60):  # 15 s at 4 Hz: walking in, still 6 s, walking off
    t = i * 0.25
    u = 100. + (t - 5.) * 40. if t < 5. else 100. + (0.5 if i % 2 else -0.5) if t < 11. else 100. + (t - 11.) * 40.
    dets.append((t, 'sideLeft', u, 400.))
  dets += [(t * 0.25, 'full', 101., 400.5) for t in range(20, 44)]  # the full pass saw the same person
  out = cf.merge_dwells(cf.find_dwells(dets))
  assert len(out) == 1 and out[0].view == 'sideLeft' and abs(out[0].u - 100.) < 1. and out[0].t1 - out[0].t0 >= 4.
  assert cf.find_dwells([(i * 0.25, 'full', 300., 300.) for i in range(14)]) == []  # 3.25 s: too short


def test_the_side_view_feet_point_maps_back_through_its_own_index_map():
  geom = sideview.compute_geometry(Lens(W, H, 452., fisheye=True), CAL.rpy, CAL.euler, W, H)
  for name, (x, y) in (('sideLeft', (300.2, 260.7)), ('sideRight', (500.6, 240.3))):
    got = cf.feet_from_map(geom.maps[name], geom.stride, x, y)
    want = geom.to_fisheye(name, np.array([x]), np.array([y]))
    assert got is not None and abs(got[0] - float(want[0][0])) < 0.05 and abs(got[1] - float(want[1][0])) < 0.05


def grids(lens):
  return cf.pixel_grid(lens, calib_from_view(list(CAL.rpy), list(CAL.euler)))


def test_a_static_dark_pillar_at_60_66_deg_removes_those_rows():
  lens = true_lens()
  bearing, elev = grids(lens)
  r = calib_from_view(list(CAL.rpy), list(CAL.euler))
  edge = cf.edge_deg(lens, r)
  blocked = (bearing <= -60.) & (bearing >= -66.)
  mask = cf.mask_rows(blocked, bearing, elev, edge, lens, r)
  left = {int(b): (lo, hi) for b, lo, hi in mask['left']}
  right = {int(b): (lo, hi) for b, lo, hi in mask['right']}
  assert not any(b in left for b in range(60, 66)) and 58 in left and 68 in left and 62 in right
  assert max(left) <= edge['left'] - sr.EDGE_MARGIN_DEG and max(right) <= edge['right'] - sr.EDGE_MARGIN_DEG
  assert left[30] == (cf.ELEV_LO, cf.ELEV_HI)
  # a frit above 1 deg narrows the rows it covers; a car's footprint there no longer fits at 10 m (its roof is at +1 deg)
  blocked = (bearing <= -40.) & (bearing >= -50.) & (elev > 1.)
  left = {int(b): (lo, hi) for b, lo, hi in cf.mask_rows(blocked, bearing, elev, edge, lens, r)['left']}
  assert left[45][1] <= 1.25 and left[45][0] == cf.ELEV_LO and left[55][1] == cf.ELEV_HI
  assert not sr.footprint_ok({'left': [[b, lo, hi] for b, (lo, hi) in left.items()], 'right': []}, 'left', 45., 6., TAPE.height)
  assert sr.footprint_ok({'left': [[b, lo, hi] for b, (lo, hi) in left.items()], 'right': []}, 'left', 56., 10., TAPE.height)


def test_the_static_dark_test_and_dark_threshold():
  n = 300
  rng = np.random.default_rng(1)
  frames = np.concatenate([rng.normal(30., 1.0, (n, 1, 50)), rng.normal(100., 30., (n, 1, 50))], axis=2)  # 50 dark static, 50 scene
  acc = cf.Accumulator((1, 100))
  for fr in frames:
    acc.add(fr)
  std, mean = acc.stats()
  blocked = cf.static_dark(std, mean)
  assert blocked[0, :50].all() and not blocked[0, 50:].any()
  assert cf.dark_threshold(mean, blocked) == pytest.approx(max(45., float(np.percentile(mean[blocked], 99)) + 5.))
  assert cf.dark_threshold(np.full((1, 4), 10.), np.ones((1, 4), bool)) == 45.  # never under 45 DN
  assert cf.dark_threshold(mean, np.zeros_like(blocked)) == 45.


S1_CAR = [(t * 0.1, 0.0, 'park', True) for t in range(600)]
S1_CAL = [(t * 1., 'calibrated', CAL.rpy, CAL.euler) for t in range(60)]


def test_s1_must_be_parked_still_calibrated_and_a_comma_four():
  assert cf.check_s1('mici', 'os04c10', (W, H), S1_CAR, S1_CAL) == CAL
  with pytest.raises(cf.Refusal, match='moved'):
    cf.check_s1('mici', 'os04c10', (W, H), S1_CAR[:-1] + [(60., 0.4, 'park', False)], S1_CAL)
  with pytest.raises(cf.Refusal, match='Park'):
    cf.check_s1('mici', 'os04c10', (W, H), S1_CAR[:-1] + [(60., 0.0, 'drive', True)], S1_CAL)
  with pytest.raises(cf.Refusal, match='calibrat'):
    cf.check_s1('mici', 'os04c10', (W, H), S1_CAR, S1_CAL[:-1] + [(60., 'recalibrating', CAL.rpy, CAL.euler)])
  with pytest.raises(cf.Refusal, match='calibration moved'):
    cf.check_s1('mici', 'os04c10', (W, H), S1_CAR, S1_CAL[:-1] + [(60., 'calibrated', (0., 0.012 + math.radians(0.4), -0.006), CAL.euler)])
  with pytest.raises(cf.Refusal, match='comma four'):
    cf.check_s1('tizi', 'os04c10', (W, H), S1_CAR, S1_CAL)
  with pytest.raises(cf.Refusal, match='1344x760'):
    cf.check_s1('mici', 'os04c10', (1928, 1208), S1_CAR, S1_CAL)


def test_the_tape_is_checked():
  assert cf.check_tape({'cameraSetbackM': 1.9, 'lateralOffsetM': -0.1, 'heightM': 1.3, 'date': '2026-10-31'}).height == 1.3
  for bad in ({'cameraSetbackM': 1.9, 'lateralOffsetM': 0.0, 'date': '2026-10-31'}, {'cameraSetbackM': 3.5, 'lateralOffsetM': 0., 'heightM': 1.3, 'date': 'x'},
              {'cameraSetbackM': 1.9, 'lateralOffsetM': 0.7, 'heightM': 1.3, 'date': '2026-10-31'},
              {'cameraSetbackM': 1.9, 'lateralOffsetM': 0.0, 'heightM': 2.0, 'date': '2026-10-31'}):
    with pytest.raises(cf.Refusal):
      cf.check_tape(bad)


def drive_frames(n=300, headings=(0., 90.)):
  return [cf.FrameMeta(t=i * 0.5, v=10., heading=headings[i * len(headings) // n], sun_el=30., exposure=1000., calibrated=True,
                       calib=CAL) for i in range(n)]


def n_frames(groups):
  return sum(len(g) for g in groups.values())


def test_the_mask_drive_needs_120_s_of_moving_daylight_on_two_headings():
  assert n_frames(cf.select_mask_frames(drive_frames(), CAL)) == 300
  with pytest.raises(cf.Refusal, match='120 s'):
    cf.select_mask_frames(drive_frames(200), CAL)
  with pytest.raises(cf.Refusal, match='within 30 deg'):
    cf.select_mask_frames(drive_frames(300, headings=(10., 40.)), CAL)
  night = [cf.FrameMeta(**{**f.__dict__, 'sun_el': -3.}) for f in drive_frames()]
  with pytest.raises(cf.Refusal, match='120 s'):
    cf.select_mask_frames(night, CAL)
  moved = [cf.FrameMeta(**{**f.__dict__, 'calib': cf.Calib((0., 0.012 + math.radians(0.5), -0.006), CAL.euler)}) for f in drive_frames()]
  with pytest.raises(cf.Refusal, match='calibration moved'):
    cf.select_mask_frames(moved, CAL)


def test_frames_are_grouped_by_exposure_and_short_groups_left_out():
  """In daylight the wide camera integrates 4-8 lines: one line is a 20% step, so frames are judged one exposure at a time."""
  mixed = [cf.FrameMeta(**{**f.__dict__, 'exposure': (4., 5., 6.)[i % 3] if i < 370 else 9.}) for i, f in enumerate(drive_frames(400))]
  groups = cf.select_mask_frames(mixed, CAL)
  assert sorted(len(g) for g in groups.values()) == [123, 123, 124]  # the 30 frames at 9 lines (15 s) are left out
  # a static dark pixel whose luma follows a 4/5-line flicker: pooled its std is over 4 DN, in each group it is still
  rng = np.random.default_rng(3)
  accs = {cf.exposure_key(4.): cf.Accumulator((1, 2)), cf.exposure_key(5.): cf.Accumulator((1, 2))}
  pooled = cf.Accumulator((1, 2))
  for i in range(200):
    e = 4. if i % 2 else 5.
    fr = np.array([[8. * e, 0.]]) + np.array([[rng.normal(0, 0.8), rng.normal(120., 40.)]])
    accs[cf.exposure_key(e)].add(fr)
    pooled.add(fr)
  assert not cf.static_dark(*pooled.stats())[0, 0]
  blocked, dark = cf.group_blocked(accs)
  assert blocked[0, 0] and not blocked[0, 1] and dark == 45.


def test_the_parked_noise_floor_is_checked():
  rng = np.random.default_rng(2)
  quiet = cf.Accumulator((1, 20))
  loud = cf.Accumulator((1, 20))
  for _ in range(60):
    quiet.add(rng.normal(30., 1.0, (1, 20)))
    loud.add(rng.normal(30., 3.5, (1, 20)))
  blocked = np.ones((1, 20), bool)
  assert cf.noise_floor(quiet, blocked) < 1.5
  with pytest.raises(cf.Refusal, match='noise'):
    cf.check_noise(cf.noise_floor(loud, blocked))


def test_the_output_is_a_copy_that_passes_the_check_and_camera_ready():
  lens = true_lens()
  r = calib_from_view(list(CAL.rpy), list(CAL.euler))
  fit = cf.solve(dwells_for(lens, 0.1), ORDER, CAL, TAPE.height)
  bearing, elev = grids(cf.fitted_lens(fit))
  mask = cf.mask_rows(np.zeros_like(bearing, bool), bearing, elev, fit.edge, cf.fitted_lens(fit), r)
  shipped = json.loads(vt.SHIPPED.read_text())
  out = cf.output(shipped, TAPE, fit, mask, 47.5)
  vt.validate(out)
  ready, missing = vt.Visibility(out).camera_ready()
  assert ready and missing == []
  assert out['veil'] == 'unmeasured' and out['darkThreshold'] == 47.5 and out['lens']['model'] == 'equisolid'
  assert out['tables'] == shipped['tables'] and out['allowProvisionalFor'] == shipped['allowProvisionalFor']
  assert {k for k in out if out[k] != shipped.get(k)} <= set(vt.CAMERA)
  assert json.loads(vt.SHIPPED.read_text()) == shipped  # the shipped file untouched


def test_main_refuses_to_overwrite_its_inputs_or_the_shipped_file(tmp_path, capsys):
  shipped_bytes = vt.SHIPPED.read_bytes()
  vis = tmp_path / 'vis.json'
  vis.write_text(vt.SHIPPED.read_text())
  tape, marks = tmp_path / 'tape.json', tmp_path / 'marks.json'
  tape.write_text(json.dumps({'cameraSetbackM': 1.9, 'lateralOffsetM': 0., 'heightM': 1.3, 'date': '2026-10-31'}))
  marks.write_text(json.dumps({'rangeM': 10, 'marks': [{'side': m.side, 'bearingDeg': m.bearing} for m in ORDER]}))
  base = ['--tape', str(tape), '--marks', str(marks), '--s1', str(tmp_path), '--drive', str(tmp_path)]
  for out in (vis, vt.SHIPPED, tape, marks):
    before = out.read_bytes()
    assert cf.main(base + ['--vis', str(vis), '--out', str(out)]) == 1
    assert out.read_bytes() == before
  assert 'refused' in capsys.readouterr().err
  assert vt.SHIPPED.read_bytes() == shipped_bytes


def test_main_writes_nothing_when_it_refuses(tmp_path, monkeypatch):
  tape, marks = tmp_path / 'tape.json', tmp_path / 'marks.json'
  tape.write_text(json.dumps({'cameraSetbackM': 1.9, 'lateralOffsetM': 0., 'heightM': 1.3, 'date': '2026-10-31'}))
  marks.write_text(json.dumps({'rangeM': 10, 'marks': [{'side': m.side, 'bearingDeg': m.bearing} for m in ORDER]}))
  out = tmp_path / 'out' / 'visibility.json'
  out.parent.mkdir()
  assert cf.main(['--tape', str(tape), '--marks', str(marks), '--s1', str(tmp_path / 'none'), '--drive', str(tmp_path / 'none'),
                  '--out', str(out)]) == 1
  assert list(out.parent.iterdir()) == []


def test_main_end_to_end_on_synthetic_logs_writes_the_three_files(tmp_path, monkeypatch, capsys):
  """run()'s orchestration with the I/O replaced: S1's logs and detections, the drive's frames and their luma."""
  lens = true_lens()
  dw = dwells_for(lens, 0.15)
  dets = [(d.t0 + 0.25 * i, d.view, d.u, d.v, 0.8) for d in dw for i in range(int((d.t1 - d.t0) / 0.25) + 1)]
  s1_seg, drive_seg = tmp_path / 's1' / 'r--0', tmp_path / 'drive' / 'r--1'
  monkeypatch.setattr(cf, 'segments', lambda paths: [s1_seg] if any('s1' in p for p in paths) else [drive_seg])

  def read(p):
    lg = cf.SegmentLog(p, 'mici', 'os04c10', (W, H))
    if p == s1_seg:
      lg.car, lg.cal = list(S1_CAR), list(S1_CAL)
    return lg
  monkeypatch.setattr(cf, 'read_segment', read)
  monkeypatch.setattr(cf, 'detect', lambda paths, logs, cal: dets)
  parked = [cf.FrameMeta(t=i * 0.5, v=0., heading=None, sun_el=30., exposure=5., calibrated=True, calib=CAL) for i in range(60)]
  monkeypatch.setattr(cf, 'frame_metas', lambda lg, i, step: drive_frames() if lg.path == drive_seg else parked)
  rng = np.random.default_rng(4)

  def accumulate(paths, groups, step):
    accs = {}
    for k, g in groups.items():
      acc = cf.Accumulator((H, W))
      for _ in range(min(len(g), 12)):
        fr = rng.normal(120., 40., (H, W)) if paths == [drive_seg] else rng.normal(30., 1.0, (H, W))
        if paths == [drive_seg]:
          fr[:, :100] = 25. + rng.normal(0., 0.5, (H, 100))  # a dark static strip down the left edge, from about 78 deg
        acc.add(fr)
      accs[k] = acc
    return accs
  monkeypatch.setattr(cf, 'accumulate', accumulate)
  tape, marks, out = tmp_path / 'tape.json', tmp_path / 'marks.json', tmp_path / 'out' / 'visibility.json'
  out.parent.mkdir()
  tape.write_text(json.dumps({'cameraSetbackM': TAPE.setback, 'lateralOffsetM': TAPE.lateral, 'heightM': TAPE.height, 'date': TAPE.date}))
  marks.write_text(json.dumps({'rangeM': 10, 'marks': [{'side': m.side, 'bearingDeg': m.bearing} for m in ORDER]}))
  assert cf.main(['--tape', str(tape), '--marks', str(marks), '--s1', str(tmp_path / 's1'), '--drive', str(tmp_path / 'drive'),
                  '--out', str(out)]) == 0
  assert sorted(p.name for p in out.parent.iterdir()) == ['visibility.json', 'visibility.provenance.json', 'visibility.report.txt']
  got = json.loads(out.read_text())
  v = vt.load(out)
  assert v.camera_ready() == (True, [])
  assert got['cameraSetbackM'] == TAPE.setback and got['heightM'] == TAPE.height and got['veil'] == 'unmeasured'
  assert abs(got['lens']['k'] - 0.3) < 0.05 and got['darkThreshold'] >= 45.
  left = {b for b, _, _ in got['mask']['left']}
  assert 70 in left and not any(b in left for b in range(80, 90))  # the strip masks the left edge
  assert max(b for b, _, _ in got['mask']['right']) <= got['lens']['edgeDeg']['right'] - sr.EDGE_MARGIN_DEG
  assert 'camera measurements ready' in capsys.readouterr().out
