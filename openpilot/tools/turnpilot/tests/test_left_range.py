"""left_range (LEFT_RANGE.md): TurnPilotLeftRangeM and TurnPilotLeftFarRangeM measured on a shadow drive's rlogs, on
synthetic logs built with cereal's messaging. Oncoming cars are boxed by the road camera's full pass and the far crop from
known ranges, through the fork's own lens and ranging (a box is the projection of the car's ground point, so it ranges back
to where it was), and tracked in turnpilotScene as the fork's tracker publishes them. Each case states the recommendation,
or its refusal, and the exit status: enough cars, too few, a car boxed late or lost for a while, the far crop on and off, a
car parked by the road (not oncoming), a stretch with no road runs (unknown is no range), an uncalibrated drive and a
sunnypilot log. The tool only reads, and never prints a position."""
import ast
import math
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.scene.ranging import calib_from_view
from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
from openpilot.selfdrive.turnpilot.unprotected_left import CLEAR_WINDOW_S, ROUTE_REACH_MARGIN_M, reach_at_line
from openpilot.tools.lib.logreader import save_log
from openpilot.tools.turnpilot import left_range as lr

TOOL = Path(lr.__file__)
DEVICE, SENSOR = 'mici', 'os04c10'
CAM = DEVICE_CAMERAS[(DEVICE, SENSOR)]
WIDTH, HEIGHT_PX = CAM.narrow_road.width, CAM.narrow_road.height
RPY = [0., 0., 0.]
CAM_HEIGHT = 1.22
ROT = calib_from_view(RPY)
TRACKER = SceneTracker(CAM, DEVICE)
LENS = TRACKER.lenses['road']
FRAME_DT = 0.05             # model frames at 20 Hz: the full pass on the even ones, the far crop on the odd ones
OFFSET_S = 100.             # logMonoTime and timestampEof of t 0
SPACING_S = 4.              # s between oncoming cars: 100 m apart at 25 m/s closing
EGO_V = 10.                 # m/s
FAR_CROP = [352., 124., 992., 508.]
LAT, LON = 39.765432, -86.123456  # a GPS position in every log: never printed
TURNPILOT = "https://github.com/kaileh57/turnpilot.git"
SUNNYPILOT = "https://github.com/sunnypilot/openpilot.git"
MPH = 0.44704


@dataclass
class Car:
  t0: float                 # s it is first in the world, x0 ahead
  x0: float = 300.          # m ahead, over the ground
  y: float = 3.6            # m left: the first oncoming lane
  speed: float = 15.        # m/s over the ground toward the car; 0 parked
  road_from: float = 150.   # the road camera's full pass boxes it from this x in
  far_from: float = 250.    # the far crop boxes it from this x in (0 never)
  gaps: tuple = ()          # (t0, t1) spells with no box of it on any pass
  cls: str = 'car'
  split_at: float = 0.      # the tracker loses it here and starts a new track (id 1000 more), its hits from 1


def pixel(x, y, up=0.):
  """Full-frame pixel of a point x forward, y left, up m above the ground, in the calibrated frame (z down)."""
  u, v = LENS.project(ROT.T @ np.array([x, -y, CAM_HEIGHT - up]))
  return float(u), float(v)


def box_at(x, y):
  x0, y1 = pixel(x, y + 0.9)
  x1, _ = pixel(x, y - 0.9)
  _, y0 = pixel(x, y, 1.5)
  return x0, y0, x1, y1


CALIBRATED = SimpleNamespace(rpyCalib=RPY, calStatus='calibrated', height=[CAM_HEIGHT], wideFromDeviceEuler=[])


def near_of(box, view='full'):
  """The near_m the fork's own raw_frame gives this box: what the left's gap rule compares with R_req."""
  det = SimpleNamespace(cls=2, conf=0.6, x0=box[0], y0=box[1], x1=box[2], y1=box[3])
  frame = TRACKER.raw_frame(SimpleNamespace(detections=[det]), 'road', CALIBRATED, 0., view, FAR_CROP if view == 'far' else None)
  return frame.boxes[0].near_m


def stamped(msg, t):
  msg.logMonoTime = int(round((t + OFFSET_S) * 1e9))
  return msg.as_reader()


def fill_dets(dets, boxes):
  for d, (cls, b) in zip(dets, boxes, strict=True):
    d.cls, d.conf, d.x0, d.y0, d.x1, d.y1 = cls, 0.6, *b


COCO = {'car': 2, 'truck': 7, 'bus': 5, 'motorcycle': 3}


def drive(cars, end=None, far=True, calibrated=True, remote=TURNPILOT, no_road=(), recalibrating=()):
  """(messages, truth) for a drive past these cars: truth[i] the near_m of car i's first road box and first far box. no_road:
  (t0, t1) spells where the full pass runs on the wide camera (a turn), so no road run; recalibrating: spells where
  extrinsicsCalibration reads recalibrating."""
  end = end if end is not None else max(c.t0 for c in cars) + 14.
  msgs, truth = [], [{'road': None, 'far': None} for _ in cars]
  init = messaging.new_message('initData')
  init.initData.gitRemote, init.initData.gitBranch = remote, 'tp/chase-range'
  msgs.append(stamped(init, 0.))
  last = {}  # car index -> (hits, camera time of its last road box, its normalized box)
  for k in range(int(round(end / FRAME_DT))):
    t = round(k * FRAME_DT, 6)
    cs = messaging.new_message('carState')
    cs.carState.vEgo = EGO_V
    msgs.append(stamped(cs, t))
    if k % 20 == 0:
      ds = messaging.new_message('deviceState')
      ds.deviceState.deviceType = DEVICE
      cam = messaging.new_message('narrowRoadCameraState')
      cam.narrowRoadCameraState.sensor = SENSOR
      gps = messaging.new_message('gpsLocationExternal')
      gps.gpsLocationExternal.latitude, gps.gpsLocationExternal.longitude = LAT, LON
      msgs += [stamped(ds, t), stamped(cam, t), stamped(gps, t)]
    if k % 5 == 0:
      cal = messaging.new_message('extrinsicsCalibration')
      cal.extrinsicsCalibration.calStatus = 'recalibrating' if any(a <= t < b for a, b in recalibrating) else \
        'calibrated' if calibrated else 'uncalibrated'
      cal.extrinsicsCalibration.rpyCalib, cal.extrinsicsCalibration.height = RPY, [CAM_HEIGHT]
      msgs.append(stamped(cal, t))
    road_pass = k % 2 == 0
    wide = road_pass and any(a <= t < b for a, b in no_road)
    if not road_pass and not far:
      continue
    boxes = []
    for i, c in enumerate(cars):
      if t < c.t0:
        continue
      x = c.x0 - (c.speed + EGO_V) * (t - c.t0)
      if x < 6. or wide or any(a <= t < b for a, b in c.gaps):
        continue
      if road_pass and x <= c.road_from:
        b = box_at(x, c.y)
        boxes.append((COCO[c.cls], b))
        if truth[i]['road'] is None:
          truth[i]['road'] = near_of(b)
        key = i + 1000 if c.split_at and x < c.split_at else i
        hits = last.get(key, (0, 0., None))[0] + 1
        last[key] = (hits, t + OFFSET_S, [b[0] / WIDTH, b[1] / HEIGHT_PX, b[2] / WIDTH, b[3] / HEIGHT_PX])
      elif not road_pass and c.far_from and x <= c.far_from and x >= 30.:
        b = box_at(x, c.y)
        boxes.append((COCO[c.cls], b))
        if truth[i]['far'] is None:
          truth[i]['far'] = near_of(b, 'far')
    if road_pass:
      det = messaging.new_message('turnpilotDetections')
      d = det.turnpilotDetections
      d.frameId, d.timestampEof = k, int(round((t + OFFSET_S) * 1e9))
      d.camera, d.view, d.farStatus = 'wide' if wide else 'road', 'full', 'read' if far else 'off'
      fill_dets(d.init('detections', len(boxes)), boxes)
      msgs.append(stamped(det, t))
    else:
      fd = messaging.new_message('turnpilotFarDetections')
      f = fd.turnpilotFarDetections
      f.crop, f.bearing = FAR_CROP, 0.
      f.run.frameId, f.run.timestampEof = k, int(round((t + OFFSET_S) * 1e9))
      f.run.camera, f.run.view = 'road', 'far'
      fill_dets(f.run.init('detections', len(boxes)), boxes)
      msgs.append(stamped(fd, t))
    # turnpilotScene as the tracker publishes it: a track per car the full pass boxed, coasting MAX_COAST_S after its last box
    live = [(i, h, ts, nb) for i, (h, ts, nb) in last.items() if h >= 2 and t + OFFSET_S - ts <= 0.5]
    sc = messaging.new_message('turnpilotScene')
    sc.turnpilotScene.available = True
    objs = sc.turnpilotScene.init('objects', len(live))
    for o, (i, hits, ts, nb) in zip(objs, live, strict=True):
      c = cars[i % 1000]
      o.trackId, o.cls, o.conf, o.age = i + 1, c.cls, 0.6, hits
      o.x, o.y, o.vx, o.vy = c.x0 - (c.speed + EGO_V) * (t - c.t0), c.y, -c.speed, 0.
      o.camera, o.view, o.tSeen, o.box = 'road', 'full', ts, nb
    msgs.append(stamped(sc, t))
  return sorted(msgs, key=lambda m: m.logMonoTime), truth


def first_road_run(t):
  """The first full-pass run at or after t, as drive() times them."""
  return next(round(k * FRAME_DT, 6) for k in range(0, 10 ** 6, 2) if round(k * FRAME_DT, 6) >= t)


def x_at(c, t):
  return c.x0 - (c.speed + EGO_V) * (t - c.t0)


def stream(n, spacing=SPACING_S, **kw):
  return [Car(t0=spacing * i, **kw) for i in range(n)]


def floor10(m):
  return int(math.floor(m / 10.) * 10)


def drives_of(msgs):
  """The messages as main reads them: split into a Drive per stretch of continuous log time."""
  drives = lr.Drives()
  for m in msgs:
    drives.feed(m)
  return drives.drives


def run(msgs, **kw):
  return lr.measure(drives_of(msgs), **kw)


def shifted(msgs, dt):
  """The messages dt s later in log and camera time (another route in the same boot)."""
  out = []
  for m in msgs:
    b = m.as_builder()
    b.logMonoTime = m.logMonoTime + int(round(dt * 1e9))
    which = b.which()
    if which == 'turnpilotDetections':
      b.turnpilotDetections.timestampEof += int(round(dt * 1e9))
    elif which == 'turnpilotFarDetections':
      b.turnpilotFarDetections.run.timestampEof += int(round(dt * 1e9))
    elif which == 'turnpilotScene':
      for o in b.turnpilotScene.objects:
        o.tSeen += dt
    out.append(b.as_reader())
  return out


def text(result):
  return '\n'.join(lr.render(result))


def test_wilks_order_statistic():
  """The k-th shortest of n is a one-sided 95% lower bound on the 5th percentile when P(Bin(n, 0.05) >= k) >= 0.95: never
  with 58 cars, the shortest from 59, the second shortest from 93."""
  assert [lr.wilks_k(n) for n in (0, 30, 58, 59, 92, 93, 100)] == [0, 0, 0, 1, 1, 2, 2]
  assert lr.MIN_VEHICLES == 59
  assert lr.MAX_DROPOUT_S == pytest.approx(CLEAR_WINDOW_S / 3.)


def test_enough_oncoming_cars_recommend_the_shortest_confirmed_range_and_print_the_set_command(tmp_path, capsys):
  cars = stream(60, far_from=0.)
  msgs, truth = drive(cars, far=False)
  result = run(msgs)
  assert len(result.samples) == 60
  shortest = min(t['road'] for t in truth)
  assert all(s.road_m == pytest.approx(t['road'], abs=0.01) for s, t in zip(sorted(result.samples, key=lambda s: s.track_id), truth,
                                                                            strict=True))
  assert result.road.range_m == floor10(shortest) and result.road.k == 1 and result.road.n == 60
  # near_m is the nearest plausible range: a car first boxed 150 m out reads about 113 m, its box's ground range about 150 m
  assert all(s.road_ground_m == pytest.approx(150., abs=1.) for s in result.samples) and 110. < shortest < 116.
  assert result.road.range_m <= max(s.road_m for s in result.samples)  # never past what the samples reached
  out = text(result)
  assert f"--set TurnPilotLeftRangeM={floor10(shortest)}" in out and "TurnPilotLeftFarRangeM=" not in out
  assert "5th percentile" in out and "60 oncoming" in out
  # from an rlog on disk, through main: exit 0, the same command
  path = tmp_path / "rlog.zst"
  save_log(str(path), msgs)
  assert lr.main([str(path)]) == 0
  printed = capsys.readouterr().out
  assert f"python3 -m openpilot.selfdrive.turnpilot.preflight --set TurnPilotLeftRangeM={floor10(shortest)}" in printed
  assert str(LAT)[:6] not in printed and str(LON)[:7] not in printed


def test_too_few_oncoming_cars_recommend_nothing(tmp_path, capsys):
  msgs, _ = drive(stream(58, far_from=0.), far=False)
  result = run(msgs)
  assert len(result.samples) == 58 and result.road.range_m is None and result.far.range_m is None
  out = text(result)
  assert "58 oncoming" in out and "59" in out and "--set" not in out
  path = tmp_path / "rlog.zst"
  save_log(str(path), msgs)
  assert lr.main([str(path)]) == 2
  assert "--set" not in capsys.readouterr().out


def test_a_car_boxed_late_or_lost_for_a_while_sets_the_range_and_is_listed():
  """A car first boxed at 100 m, and one lost for 1.2 s (over CLEAR_WINDOW_S / 3) around 115 m: each is confirmed only from
  where its boxes stop breaking off, and the shortest of them is the range."""
  cars = stream(60, far_from=0.)
  cars[10].road_from = 100.
  lost = cars[30]
  t_mid = lost.t0 + (lost.x0 - 115.) / (lost.speed + EGO_V)
  lost.gaps = ((t_mid - 0.6, t_mid + 0.6),)
  msgs, truth = drive(cars, far=False)
  result = run(msgs)
  by_id = {s.track_id: s for s in result.samples}
  assert by_id[11].road_m == pytest.approx(truth[10]['road'], abs=0.01) and by_id[11].road_why == 'unboxed'
  after = near_of(box_at(x_at(lost, first_road_run(lost.gaps[0][1])), lost.y))
  assert by_id[31].road_m == pytest.approx(after, abs=0.01) and by_id[31].road_why == 'unboxed'
  assert result.road.range_m == floor10(min(by_id[11].road_m, by_id[31].road_m))
  out = text(result)
  assert "track 11" in out and "track 31" in out


def test_allow_misses_takes_the_wilks_order_statistic_and_lists_what_fell_short():
  """Off by default: with 100 cars the second shortest is still a 95/95 bound, and the one car short of it is a listed miss."""
  cars = stream(100, far_from=0.)
  cars[50].road_from = 90.
  msgs, truth = drive(cars, far=False)
  strict = run(msgs)
  assert strict.road.k == 1 and strict.road.range_m == floor10(truth[50]['road'])
  loose = run(msgs, allow_misses=True)
  second = sorted(t['road'] for t in truth)[1]
  assert loose.road.k == 2 and loose.road.range_m == floor10(second)
  assert [s.track_id for s in loose.road.misses] == [51]
  assert "allow-misses" in text(loose) and "track 51" in text(loose)


def test_the_far_crop_on_extends_the_range_and_both_params_are_set():
  msgs, truth = drive(stream(60))
  result = run(msgs)
  road, far = min(t['road'] for t in truth), min(t['far'] for t in truth)
  assert result.road.range_m == floor10(road)
  assert result.far.range_m == floor10(far) and result.far.range_m > result.road.range_m
  out = text(result)
  assert f"--set TurnPilotLeftRangeM={floor10(road)} --set TurnPilotLeftFarRangeM={floor10(far)}" in out


def test_the_far_crop_off_sets_no_far_range():
  msgs, truth = drive(stream(60, far_from=0.), far=False)
  result = run(msgs)
  assert result.road.range_m == floor10(min(t['road'] for t in truth)) and result.far.range_m is None
  assert "far crop off" in result.far.why
  assert "TurnPilotLeftFarRangeM=" not in text(result)


def test_far_runs_missing_from_the_logs_are_not_called_a_far_crop_off():
  """The full pass's farStatus reads but no turnpilotFarDetections were logged (the BeamNG runs' rlogs on this Mac): no far
  range, and the reason says the far runs are missing, not that the far crop was off."""
  msgs, _ = drive(stream(60))
  result = run([m for m in msgs if m.which() != 'turnpilotFarDetections'])
  assert result.road.range_m is not None and result.far.range_m is None
  assert "far crop off" not in result.far.why and "not logged" in result.far.why


def test_a_far_crop_that_adds_no_reach_sets_no_far_range():
  msgs, _ = drive(stream(60, far_from=150.))
  result = run(msgs)
  assert result.road.range_m is not None and result.far.range_m is None and "no reach" in result.far.why


def test_no_far_range_without_a_road_range():
  """turnpilotd extends the reach by the far crop only with TurnPilotLeftRangeM measured (range_eff, left_reach_m)."""
  msgs, _ = drive(stream(58))
  result = run(msgs)
  assert result.road.range_m is None and result.far.range_m is None and "--set" not in text(result)


def test_a_car_parked_by_the_road_is_not_an_oncoming_sample():
  """Parked 12 m left of the car's lane, inside the band, boxed all the way in and tracked, standing still over the ground:
  never a sample, so 58 oncoming cars and it are too few, and 59 and it are enough. (At 9 m it is inside the ambiguity gate
  of the oncoming car that passes it about 140 m out, which is then confirmed only to there: the tests below.)"""
  parked = Car(t0=0., x0=300., y=12., speed=0., far_from=0.)
  msgs, _ = drive([parked] + stream(58, far_from=0.), far=False)
  result = run(msgs)
  assert 1 not in [s.track_id for s in result.samples]
  assert len(result.samples) == 58 and result.road.range_m is None
  msgs, truth = drive([parked] + stream(59, far_from=0.), far=False)
  result = run(msgs)
  assert len(result.samples) == 59 and 1 not in [s.track_id for s in result.samples]
  assert result.road.range_m == floor10(min(t['road'] for t in truth[1:]))


def test_one_car_tracked_twice_is_one_sample():
  """The tracker loses car 1 at 75 m and starts a new track for it: both tracks are samples by themselves, but their chains
  run through the same boxes, so it counts once (a count of 59 must be 59 cars)."""
  cars = stream(59, far_from=0.)
  cars[0].split_at = 75.
  msgs, _ = drive(cars, far=False)
  d = lr.Drive()
  for m in msgs:
    d.feed(m)
  assert {1, 1001} <= {o.trackId for o, _ in lr.seeds(d)}
  result = lr.measure(d)
  assert len(result.samples) == 59 and result.road.range_m is not None
  msgs, _ = drive(cars[:58], far=False)
  assert len(run(msgs).samples) == 58


def test_one_cars_tracks_merge_to_its_shorter_road_and_its_shorter_far_range():
  """Chains through a shared box are one car (through a third too: 1 shares with 2, 2 with 3), and it keeps the shortest of
  each range on its own, with that chain's reason: never the longer of two readings of one car."""
  def sample(track, road, far, boxes):
    return lr.Sample(track, 'car', float(track), 15., road_m=road, road_why=f"r{track}", far_m=far, far_why=f"f{track}",
                     boxes=frozenset(boxes))
  merged = lr.merge([sample(1, 100., 200., {(5, 0)}), sample(2, 110., 150., {(5, 0), (6, 0)}), sample(3, 90., 300., {(6, 0)}),
                     sample(4, 50., 60., {(9, 1)})])
  assert len(merged) == 2
  one = next(s for s in merged if s.track_id != 4)
  assert (one.road_m, one.road_why, one.far_m, one.far_why) == (90., 'r3', 150., 'f2')


def test_two_boxes_where_the_car_could_be_end_its_chain():
  """Two cars side by side in one lane's width: neither chain can tell which box was its car's, so each ends at its own
  seed's box (ambiguous), and the range is that short."""
  cars = stream(60, far_from=0.)
  cars.append(Car(t0=cars[20].t0, y=4.4, far_from=0.))
  msgs, _ = drive(cars, far=False)
  result = run(msgs)
  pair = [s for s in result.samples if s.road_why == 'ambiguous']
  assert pair and all(s.road_m < 90. for s in pair)
  assert result.road.range_m == floor10(min(s.road_m for s in result.samples))


def late_car_stream(spacing=SPACING_S):
  """60 oncoming cars, car 10 (track 11) boxed only from 100 m: the true shortest, its first box's near_m about 82 m."""
  cars = stream(60, spacing=spacing, far_from=0.)
  cars[10].road_from = 100.
  return cars


def no_credit_past_its_own_boxes(result, truth):
  """Car 10 never confirmed past its own first box, nor the recommendation past it: every sampled car's chain is its own."""
  worst = truth[10]['road']
  assert 80. < worst < 90.
  s = {s.track_id: s for s in result.samples}.get(11)
  assert s is not None and s.road_m <= worst + 0.01
  assert result.road.range_m is not None and result.road.range_m <= floor10(worst)
  assert all(s.road_why != 'logStart' for s in result.samples)


@pytest.mark.parametrize("y", (5.0, 6.0, 6.6))
@pytest.mark.parametrize("x", (100., 110., 120.))
def test_a_car_parked_by_the_oncoming_lane_gives_no_oncoming_car_credit(x, y):
  """Parked y m left, x m ahead when car 10 is first boxed (100 m): when car 10 has no box, the parked car's box where it
  could be is not its, so car 10 is confirmed no farther than its own first box and the range no longer than that."""
  cars = late_car_stream()
  t_first = cars[10].t0 + (cars[10].x0 - 100.) / (cars[10].speed + EGO_V)
  parked = Car(t0=t_first - 15., x0=x + EGO_V * 15., y=y, speed=0., far_from=0.)
  msgs, truth = drive(cars + [parked], far=False)
  result = run(msgs)
  no_credit_past_its_own_boxes(result, truth[:60])
  assert len(result.samples) == 60


@pytest.mark.parametrize("gap_s", (0.5, 0.8, 1.0))
def test_a_car_close_behind_in_the_same_lane_gives_no_credit(gap_s):
  """A car gap_s behind car 10 in its lane, boxed from 150 m: its boxes are never car 10's."""
  cars = late_car_stream()
  msgs, truth = drive(cars + [Car(t0=cars[10].t0 + gap_s, far_from=0.)], far=False)
  result = run(msgs)
  no_credit_past_its_own_boxes(result, truth[:60])


@pytest.mark.parametrize("x", (110., 120., 130., 140.))
def test_a_lead_in_our_lane_gives_no_credit(x):
  """A lead x m ahead at our speed, boxed all drive: no oncoming car's chain follows it back (logStart), none is credited its
  boxes, and no two cars' chains merge through it."""
  lead = Car(t0=0., x0=x, y=0., speed=-EGO_V, road_from=400., far_from=0.)
  msgs, truth = drive(late_car_stream() + [lead], far=False)
  result = run(msgs)
  no_credit_past_its_own_boxes(result, truth[:60])
  assert len(result.samples) == 60


def test_a_box_that_closes_at_another_rate_is_not_followed_back():
  """A car stopped in the oncoming lane, boxed from 150 m until car 10 passes it about 105 m out and hides it, so going back
  its box is the one where car 10 would be the run after car 10's first: never inside the ambiguity gate beside car 10's.
  Re-centred on each box, car 10's chain would follow it back to its own first box (150 m). The seed's own motion closes at
  25 m/s and it at 10: once they part by the gate, the chain ends there (ambiguous). Bounded, not prevented: the gate is the
  tracker's (2 m + 15% of range) wide. Cars 8 s apart, so no other ends car 10's chain first."""
  cars = late_car_stream(spacing=8.)
  t1 = cars[10].t0 + (cars[10].x0 - 100.) / (cars[10].speed + EGO_V)  # car 10's first box, 100 m out
  t1 = first_road_run(t1)
  t_hidden = round(t1 - 0.15, 6)
  stopped = Car(t0=t1 - 15., x0=x_at(cars[10], t1 - 0.2) + EGO_V * 14.8, speed=0., far_from=0., gaps=((t_hidden, 1e6),))
  msgs, truth = drive(cars + [stopped], far=False)
  s = {s.track_id: s for s in run(msgs).samples}[11]
  assert s.road_why == 'ambiguous' and s.road_m < near_of(box_at(150., 3.6)) - 5.


@pytest.mark.parametrize("dt", (3600., 0.), ids=("same boot an hour later", "after a reboot"))
def test_two_routes_are_each_measured_on_their_own(dt, tmp_path, capsys):
  """Two routes of 35 oncoming cars each, the second an hour later in the same boot or after a reboot (its log and camera
  times start over): each turnpilotd's tracker numbers its tracks from 1, so the routes' track ids are the same ones. Each
  route is measured on its own time base, so all 70 cars are samples and route 1's car 10 (boxed only from 100 m) is
  confirmed from its own first box, never from a box of route 2's; through main too, read as two files."""
  a_cars = stream(35, far_from=0.)
  a_cars[10].road_from = 100.
  a, a_truth = drive(a_cars, far=False)
  b, _ = drive(stream(35, far_from=0.), far=False)
  second = shifted(b, dt)
  drives = drives_of(a + second)
  assert len(drives) == 2
  result = lr.measure(drives)
  assert len(result.samples) == 70
  worst = [s for s in result.samples if s.drive == 0 and s.track_id == 11]
  assert len(worst) == 1 and worst[0].road_m == pytest.approx(a_truth[10]['road'], abs=0.01) and worst[0].road_why == 'unboxed'
  assert result.road.range_m == floor10(a_truth[10]['road'])
  out = text(result)
  assert "2 drives" in out and "drive 1" in out
  for name, msgs in (("a", a), ("b", second)):
    save_log(str(tmp_path / f"{name}--0--rlog.zst"), msgs)
  assert lr.main([str(tmp_path / "a--0--rlog.zst"), str(tmp_path / "b--0--rlog.zst")]) == 0
  assert f"--set TurnPilotLeftRangeM={floor10(a_truth[10]['road'])}" in capsys.readouterr().out


@pytest.mark.parametrize("how", ("unavailable", "gap"))
def test_turnpilotd_starting_again_mid_drive_drops_no_car(how):
  """turnpilotd's tracker starts again mid-drive, and its track ids with it: its scene failed (one unavailable scene, then a
  new tracker) or the process restarted (no scene for 1.5 s). Car 35, not yet a sample, takes the id car 34's track had 0.15
  s before: it is another car, and all 70 are samples."""
  cars = stream(70, far_from=0.)
  msgs, _ = drive(cars, far=False)
  t_restart = round(4. * 35 + 8.4, 6)
  out, first = [], True
  for m in msgs:
    t = round(m.logMonoTime * 1e-9 - OFFSET_S, 6)
    if m.which() != 'turnpilotScene' or t < t_restart:
      out.append(m)
      continue
    if how == 'gap' and t < t_restart + 1.5:
      continue
    b = m.as_builder()
    if how == 'unavailable' and first:
      b.turnpilotScene.available = False
      b.turnpilotScene.init('objects', 0)
    for o in b.turnpilotScene.objects:
      o.trackId = o.trackId - 1 if o.trackId >= 36 else o.trackId
    first = False
    out.append(b.as_reader())
  result = run(out)
  assert len(result.samples) == 70


def test_a_spell_with_no_road_runs_is_no_range_not_a_clear_road():
  """The full pass on the wide camera for 1.2 s (a turn) while car 31 closes from about 120 m: nothing says it was boxed
  there, so it is confirmed only from its first road box after the spell (unknown is no range), said as noRuns."""
  cars = stream(60, far_from=0.)
  c = cars[30]
  t_mid = c.t0 + (c.x0 - 120.) / (c.speed + EGO_V)
  spell = (t_mid - 0.6, t_mid + 0.6)
  msgs, _ = drive(cars, far=False, no_road=(spell,))
  result = run(msgs)
  s = {s.track_id: s for s in result.samples}[31]
  assert s.road_m == pytest.approx(near_of(box_at(x_at(c, first_road_run(spell[1])), c.y)), abs=0.01)
  assert s.road_why == 'noRuns' and result.road.range_m == floor10(s.road_m)


def test_a_recalibrating_spell_is_no_range_either():
  """Calibration lost for 1.5 s while car 31 closes from about 120 m: those runs can't be ranged, so they never bridge its
  boxes (said as uncalibrated)."""
  cars = stream(60, far_from=0.)
  c = cars[30]
  t_mid = c.t0 + (c.x0 - 120.) / (c.speed + EGO_V)
  spell = (round(t_mid - 0.75, 2), round(t_mid + 0.75, 2))
  msgs, _ = drive(cars, far=False, recalibrating=(spell,))
  s = {s.track_id: s for s in run(msgs).samples}[31]
  assert s.road_why == 'uncalibrated' and s.road_m < near_of(box_at(x_at(c, spell[0]), c.y)) - 5.


def test_an_uncalibrated_drive_has_no_road_runs_and_no_range():
  msgs, _ = drive(stream(60, far_from=0.), far=False, calibrated=False)
  result = run(msgs)
  assert result.road.range_m is None and "calibrat" in text(result) and "--set" not in text(result)


def test_a_sunnypilot_log_is_refused(tmp_path, capsys):
  """Sunnypilot puts its own messages on turnpilotDetections' union slot (CAR_PREFLIGHT.md), so its log is never read."""
  msgs, _ = drive(stream(60, far_from=0.), far=False, remote=SUNNYPILOT)
  result = run(msgs)
  assert result.refused and "sunnypilot" in result.refused and result.road.range_m is None
  path = tmp_path / "rlog.zst"
  save_log(str(path), msgs)
  assert lr.main([str(path)]) == 1
  assert "--set" not in capsys.readouterr().out


def test_the_supported_opposing_speed_is_route_times_own_reach():
  for range_m in (120, 150, 230):
    mph = lr.supported_mph(range_m)
    assert reach_at_line(mph * MPH, None, ROUTE_REACH_MARGIN_M) <= range_m < reach_at_line((mph + 1) * MPH, None, ROUTE_REACH_MARGIN_M)
  assert lr.supported_mph(30) == 0


def test_it_only_reads():
  """No param is written, no socket opened, no location service read."""
  tree = ast.parse(TOOL.read_text())
  calls = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
  assert not calls & {'put', 'put_bool', 'put_nonblocking', 'remove', 'clear_all', 'pub_sock', 'send'}
  source = TOOL.read_text()
  assert 'gpsLocation' not in source and 'liveLocation' not in source and 'Params(' not in source
