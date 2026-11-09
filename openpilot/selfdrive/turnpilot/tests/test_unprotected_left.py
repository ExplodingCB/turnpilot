"""The autonomous unprotected left (unprotected_left.py): its gap rule on raw detections, the clear window, the occluder and
the view, the exit crosswalk, creeping, the go, its re-certification at first motion, the commit, and its faults."""
import dataclasses
import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import unprotected_left as ul
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.route_follower import LEFT
from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M, VEL_TRUST_HITS
from openpilot.selfdrive.turnpilot.tests.synthetic import scene
from openpilot.selfdrive.turnpilot.unprotected_left import (BLOCKED_UNDECIDED_S, CLEAR_WINDOW_S, CREEP_MAX_M, CREEP_SHORT_M, CREEP_STALL_S, GO_LATENCY_S,
                                                           HELD_MIN_M,
                                                           LIGHT_RED_MAX_S, LIGHT_UNKNOWN_S, LONG_WAIT_REASON, PET_S, RAW_CONF,
                                                           FAR_FRESH_S, ROAD_FRESH_S, SPEED_MARGIN, STALE_UNDECIDED_S, STAND_UNDECIDED_S, LeftInput, RawBox,
                                                           RawFrame, UnprotectedLeft, WIDTH_FLOOR_M, band_blockers, nearest_range, r_req,
                                                           t_clear, t_req)

HOLD_D = 9.     # m from the hold line (map_controls: HOLD_LINE_M + MAP_MARGIN_M) to the maneuver point
V_25 = 11.18    # m/s, 25 mph
RANGE = 150.


@pytest.fixture
def log_events(monkeypatch):
  """The left's cloudlog events, (name, fields), in order: a converted wait is said once, never a frame's log."""
  out = []
  monkeypatch.setattr(ul.cloudlog, 'event', lambda name, *a, **kw: out.append((name, kw)))
  return out


def raw(x, y, cls='car', conf=0.6, camera='road', near=None, ranged=True, width=1.8):
  """A raw detection of a vehicle whose ground point is (x, y) in the car's frame; near: its nearest plausible range."""
  bearing = math.degrees(math.atan2(y, x))
  half = math.degrees(math.atan2(width / 2., math.hypot(x, y)))
  return RawBox(camera, cls, conf, bearing, (bearing - half, bearing + half), (x, y) if ranged else None,
                (near if near is not None else math.hypot(x, y)) if ranged else None, (0., 0., 0., 0.))


def frame(boxes=(), camera='road', t=0.):
  return RawFrame(list(boxes), camera, t)


FULL_COVER = (-15.6, 15.6)  # the far crop straight ahead: its bearings at the horizon, right and left


def far_frame(boxes=(), cover=FULL_COVER, t=0.):
  """A far crop run (tracker.update_far): its boxes view far, on the road camera."""
  return RawFrame([dataclasses.replace(b, view='far') for b in boxes], 'road', t, 'far', cover)


def oncoming(x, y=3.6, vx=-11., age=VEL_TRUST_HITS, **fields):
  return {'cls': 'car', 'x': x, 'y': y, 'vx': vx, 'vy': 0., 'inPath': False, 'age': age, **fields}


def ped(x, y, **fields):
  return {'cls': 'pedestrian', 'x': x, 'y': y, 'vx': 0., 'vy': 0., 'inPath': False, 'age': 20, 'conf': 0.8, **fields}


def test_t_req_and_r_req():
  # the left's post-go profile (1.5 m/s^2 to 4.3 m/s) over the path from the hold line through the oncoming lanes, this car's
  # own launch from go to moving (GO_LATENCY_S: 0.92-1.6 s on 11-06, review 2026-11-07 L4; turnpilotd's LEFT_NO_MOVE_S
  # withdraws a go not moving by then) and a 2 s post-encroachment margin
  assert 5.0 < t_clear(HOLD_D + 8.) < 5.8
  assert 1.6 <= GO_LATENCY_S < 2.
  t1, t2 = t_req(HOLD_D, 1), t_req(HOLD_D, 2)
  assert 8.7 < t1 < 9.7 and abs(t2 - t1 - 0.5) < 1e-6
  assert abs(t_req(HOLD_D, None) - t2) < 1e-6  # unknown opposing lanes: two
  assert abs(t_req(HOLD_D, 1, latency=False) - (t1 - GO_LATENCY_S)) < 1e-6
  # the range to see clear is the mapped limit's 1.3 x over it, from the conflict point
  assert SPEED_MARGIN >= 1.3 and PET_S >= 2.
  r = r_req(V_25, t1, HOLD_D)
  assert abs(r - (V_25 * SPEED_MARGIN * t1 + HOLD_D + 3.)) < 1e-6 and 130. < r < 150.
  assert r_req(20.1, t1, HOLD_D) > 220.  # 45 mph: beyond the road camera's provisional range


def test_band_counts_raw_boxes_ranged_or_not_never_tracks():
  r = 140.
  # in the oncoming lanes within reach, at the decoder's floor confidence, never needing a track
  assert band_blockers([raw(100., 3.6, conf=RAW_CONF)], 0., LEFT, r, HOLD_D)
  assert not band_blockers([raw(100., 3.6, conf=RAW_CONF - 0.01)], 0., LEFT, r, HOLD_D)
  # beyond reach, by its nearest plausible range
  assert not band_blockers([raw(200., 3.6)], 0., LEFT, r, HOLD_D)
  assert band_blockers([raw(200., 3.6, near=r - 1.)], 0., LEFT, r, HOLD_D)
  # unranged (the box bottom above the horizon: a far car, or one on a road that climbs): counted on its bearing
  assert band_blockers([raw(300., 4., ranged=False)], 0., LEFT, r, HOLD_D)
  # trucks, buses, motorcycles too; a pedestrian isn't traffic in the band (the crosswalk's)
  for cls in ('truck', 'bus', 'motorcycle', 'cyclist'):
    assert band_blockers([raw(100., 3.6, cls=cls)], 0., LEFT, r, HOLD_D), cls
  assert not band_blockers([raw(100., 3.6, cls='pedestrian')], 0., LEFT, r, HOLD_D)
  # the far side of the car's own road, and a car past the junction beside the car, aren't in it
  assert not band_blockers([raw(60., -4.)], 0., LEFT, r, HOLD_D)
  assert not band_blockers([raw(2., 3.6)], 0., LEFT, r, HOLD_D)
  # the car's own lane far up the road: within the band's heading tolerance there, counted (velocity at range is noise)
  assert band_blockers([raw(120., 0.)], 0., LEFT, r, HOLD_D)
  # the wide camera's boxes don't certify or break the band (its range is a fraction of the road camera's)
  assert not band_blockers([raw(100., 3.6, camera='wide')], 0., LEFT, r, HOLD_D)
  # in the road frame: yawed 30 deg into the left, a car 80 m up the oncoming lane is to the car's right
  a = math.radians(30.)
  x, y = 80. * math.cos(a) + 3.6 * math.sin(a), -80. * math.sin(a) + 3.6 * math.cos(a)
  assert band_blockers([raw(x, y)], 30., LEFT, r, HOLD_D)
  assert not band_blockers([raw(x, y)], 0., LEFT, r, HOLD_D)


def test_band_reach_floored_by_box_width():
  r = 140.
  # a car truly 120 m up the oncoming lane whose box bottom reads 250 m (the road climbing, the car pitched): its 1.8 m wide
  # box floors its nearest range to 1.5 m's, 100 m, inside reach, so it blocks
  car = raw(120., 3.6)
  far_read = dataclasses.replace(car, xy=(250., 7.5), near_m=240.)
  assert car.near_m <= r and far_read.near_m > r
  assert band_blockers([far_read], 0., LEFT, r, HOLD_D)
  assert abs(nearest_range(far_read) - 1.5 / 1.8 * math.hypot(120., 3.6)) < 0.5
  # the floor never lifts a box's range: one nearer by near_m keeps it
  assert nearest_range(dataclasses.replace(car, near_m=50.)) == 50.
  # a car wide enough to be 1.5 m only past reach doesn't block
  assert not band_blockers([raw(200., 3.6)], 0., LEFT, r, HOLD_D)
  # a motorcycle's box is narrow: its floor is its own class's (0.6 m), so it blocks no further out than a car's would
  moto = raw(120., 3.6, cls='motorcycle', width=0.8)
  assert band_blockers([dataclasses.replace(moto, xy=(250., 7.5), near_m=240.)], 0., LEFT, r, HOLD_D)
  assert not band_blockers([dataclasses.replace(raw(400., 3.6, cls='motorcycle', width=0.8), near_m=390.)], 0., LEFT, r, HOLD_D)
  # an unranged box is as before: on the turn side it blocks on its bearing
  assert band_blockers([raw(300., 4., ranged=False)], 0., LEFT, r, HOLD_D)


class Junction:
  """A held unprotected left: the car at the hold line, frames of raw road detections, scene tracks."""
  def __init__(self, v_limit=V_25, lanes=1, mode='auto', rng=RANGE):
    self.left = UnprotectedLeft()
    self.mode, self.v_limit, self.lanes, self.range = mode, v_limit, lanes, rng
    self.distance, self.v, self.t = HOLD_D, 0., 0.
    self.held = True
    self.decisions = []
    self.wide_every = 0  # every this many frames the run is the wide camera's (roadAndWide), 0 never

  def step(self, boxes=(), objects=(), road=True, raw_new=True, v=None, distance=None, far=None, cover=FULL_COVER, **kw):
    """far: the far crop's boxes on this frame (None: no far run), its run covering cover (deg, right and left)."""
    self.t += DT_MDL
    if v is not None:
      self.v = v
    if distance is not None:
      self.distance = distance
    if self.wide_every and len(self.decisions) % self.wide_every == self.wide_every - 1:
      road = False
    fr = frame(boxes, 'road' if road else 'wide', self.t) if raw_new else None
    if far is not None:
      kw['far'] = far_frame(far, cover, self.t)
    inp = LeftInput(mode=self.mode, maneuver_id=1, side=LEFT, distance=self.distance, v_ego=self.v, held=self.held,
                    raw=fr, objects=scene(objects).objects, heading_deg=kw.pop('heading_deg', 0.), v_limit=self.v_limit,
                    opposing_lanes=self.lanes, range_m=self.range, **kw)
    d = self.left.update(inp, DT_MDL)
    self.decisions.append(d)
    return d

  def wait(self, seconds, **kw):
    d = None
    for _ in range(round(seconds / DT_MDL)):
      d = self.step(**kw)
    return d


def runs_every(j, frames, seconds, far=False):
  """j's runs every frames frames for seconds (a far crop run on the road run's frame with far)."""
  for _ in range(round(seconds / (frames * DT_MDL))):
    j.step(far=[] if far else None, far_range_m=300., far_live=far)
    for _ in range(frames - 1):
      j.step(raw_new=False, far_range_m=300., far_live=far)


def test_the_streams_freshness_is_what_the_gap_rule_can_spare_never_the_schedules_edge():
  """Review 2026-11-07 (L2): ROAD_FRESH_S and FAR_FRESH_S were 0.3 s, where far first's deadlines sit one frame inside them
  (ROAD_DUE_S 0.25): a run a frame late started the window over, so with the chestnut's model 2 ms slower an empty road's
  35 mph left at a signalized line waited p50 28-35 s, 20-26 of 100 holds a minute or more (test_far_first's real left, 2
  road heads at 1.4x, without CS-1's overdue run). The gap rule's margin is what a gap between runs may spend: a car at
  SPEED_MARGIN x the limit just past R_req on the last run is inside it by that travel at the go, out of PET_S. So the
  freshness is a quarter of PET_S (half a second): runs 0.35 s apart now fill the window, 0.55 s apart still start it over."""
  assert ROAD_FRESH_S == FAR_FRESH_S == PET_S / 4.
  j = Junction()
  runs_every(j, 7, CLEAR_WINDOW_S + 1.)  # 0.35 s apart
  assert any(x.go for x in j.decisions)
  j = Junction()
  runs_every(j, 11, CLEAR_WINDOW_S + 3.)  # 0.55 s apart
  assert not any(x.go for x in j.decisions)
  # the far crop's runs (a 45 mph left, far needed) the same
  j = Junction(v_limit=V_45)
  runs_every(j, 7, CLEAR_WINDOW_S + 1., far=True)
  assert any(x.go and x.far_needed for x in j.decisions)


def test_goes_only_after_the_band_is_clear_on_road_frames_for_the_window():
  j = Junction()
  d = j.step([raw(100., 3.6)])
  assert not d.go and d.state == 'waitGap' and d.prompt == 'waitingForGap' and d.view == 'roadAndWide'
  assert CLEAR_WINDOW_S >= 2.5
  d = j.wait(CLEAR_WINDOW_S - 0.2)
  assert not d.go and 0. < d.clear_s < CLEAR_WINDOW_S
  j.wait(0.3)
  gos = [x for x in j.decisions if x.go]
  assert len(gos) == 1 and gos[0].state == 'go' and gos[0].view == 'road' and gos[0].prompt == 'leftGo'
  assert gos[0].clear_s >= CLEAR_WINDOW_S - 1e-6
  # one road frame with a car in the band anywhere in the window starts it over
  j = Junction()
  j.wait(CLEAR_WINDOW_S - 0.5)
  d = j.step([raw(130., 3.6, conf=0.36)])
  assert d.clear_s == 0.
  assert not j.wait(CLEAR_WINDOW_S - 0.2).go


def test_a_missing_frame_is_never_clear():
  # no road camera run (the detector on the wide camera, or not running): the window doesn't count, and stops
  j = Junction()
  d = j.wait(CLEAR_WINDOW_S + 1., road=False)
  assert not d.go and d.clear_s == 0.
  j = Junction()
  d = j.wait(CLEAR_WINDOW_S + 1., raw_new=False)
  assert not d.go and d.clear_s == 0.
  # a road frame every 0.1 s is fresh; a gap over ROAD_FRESH_S starts the window over
  j = Junction()
  for _ in range(round((CLEAR_WINDOW_S + 1.) / 0.1)):
    j.step()
    d = j.step(raw_new=False)
  assert d.go or any(x.go for x in j.decisions)
  j = Junction()
  gap = math.ceil(ROAD_FRESH_S / DT_MDL) + 1
  for _ in range(round((CLEAR_WINDOW_S + 1.) / (gap * DT_MDL))):
    j.step()
    for _ in range(gap - 1):
      d = j.step(raw_new=False)
  assert not any(x.go for x in j.decisions)


def test_stale_road_frames_are_a_fault_never_a_wait():
  j = Junction()
  d = j.wait(STALE_UNDECIDED_S + 0.2, raw_new=False)
  assert d.state == 'undecided' and d.prompt == 'leftUndecidedTakeControl' and not d.go


def test_a_gap_stream_goes_on_a_long_gap_and_waits_through_short_ones():
  # oncoming cars at the limit, headway h, the car standing at the hold line: the road camera sees each from far out
  def stream(headway, n=4, first_at=100.):
    cars = [first_at + k * headway * V_25 for k in range(n)]  # m from the camera at t 0
    j = Junction()
    went = None
    for k in range(round(60. / DT_MDL)):
      t = k * DT_MDL
      xs = [c - V_25 * t for c in cars]
      boxes = [raw(x, 3.6) for x in xs if HOLD_D - 5. < x < 400.]
      d = j.step(boxes)
      if d.go:
        went = t
        break
    return went, [c / V_25 for c in cars], [(c - HOLD_D - 3.) / V_25 for c in cars]

  for headway in (4., 6., 8., 10.):
    went, arrive, _ = stream(headway)
    assert went is not None, headway  # after the last car
    after_last = went - (arrive[-1] - HOLD_D / V_25)
    assert after_last > 0., headway   # never between two of them
  # a gap of the rule's own need goes between two cars, leaving the next one at least T_req + PET out. A 1.8 m car's box
  # reads its width floor (WIDTH_FLOOR_M, a 1.5 m car) at 1.5/1.8 of its range, so it blocks from 1.8/1.5 x R_req out
  reach = r_req(V_25, t_req(HOLD_D, 1), HOLD_D) * 1.8 / WIDTH_FLOOR_M['car']
  need = (reach - HOLD_D) / V_25 + CLEAR_WINDOW_S + 1.
  went, arrive, conflict = stream(need, n=2)
  assert arrive[0] < went < arrive[1]
  assert conflict[1] - went >= SPEED_MARGIN * t_req(HOLD_D, 1) - 0.1  # the next car, at the limit, at the conflict point


def test_beyond_the_proven_range_it_goes_on_the_band_it_sees():
  # 45 mph oncoming: its reach is past the road camera's proven range. Kellen, 2026-11-07 ("no, no handbacks"; a safety driver on
  # every drive): judged on the band the camera covers, it goes when that band is clear, never a handback for what it can't see
  j = Junction(v_limit=20.1)
  j.wait(CLEAR_WINDOW_S + 1.)
  assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions)
  # unknown speed is taken as 45 mph: the same
  j = Junction(v_limit=None)
  j.wait(CLEAR_WINDOW_S + 1.)
  assert any(x.go for x in j.decisions)
  # a car it sees in the band still holds it
  j = Junction(v_limit=20.1)
  d = j.wait(CLEAR_WINDOW_S + 1., boxes=[raw(80., 3.6)])
  assert not any(x.go for x in j.decisions) and d.state == 'waitGap'


def test_an_occluder_is_no_traffic_but_blocks_the_view_and_the_car_creeps():
  # an opposing left-turner standing in its turn lane across the junction: a stationary, velocity-trusted track. Its raw
  # box isn't traffic (it would never clear), but it hides the oncoming lanes behind it: the car creeps for a view
  turner = oncoming(HOLD_D + 12., 3.6, vx=0., trackId=7, box=[0.5, 0.4, 0.6, 0.5], camera='road')
  box = raw(HOLD_D + 12., 3.6)
  box = RawBox(box.camera, box.cls, box.conf, box.bearing_deg, box.span_deg, box.xy, box.near_m, (0.5, 0.4, 0.6, 0.5))
  j = Junction()
  d = j.wait(1., boxes=[box], objects=[turner])
  assert d.view_blocked and d.state == 'creep' and d.prompt == 'creepingForView' and d.creep_m == CREEP_MAX_M and not d.go
  assert CREEP_MAX_M <= 4.
  # never closer than CREEP_SHORT_M short of the oncoming lanes: from nearer the line, less
  assert CREEP_SHORT_M >= 1.
  j = Junction()
  d = j.wait(0.5, boxes=[box], objects=[turner], distance=ONCOMING_ENTRY_M + CREEP_SHORT_M + 1.5)
  assert d.state == 'creep' and abs(d.creep_m - 1.5) < 1e-6
  # creeping stops where a car enters the band: held where it is
  j = Junction()
  j.wait(0.5, boxes=[box], objects=[turner])
  d = j.wait(0.2, boxes=[box, raw(90., 6.)], objects=[turner], v=0.7, distance=HOLD_D - 0.3)
  assert d.state == 'waitGap' and not d.go and d.creep_m <= 0.3 + 0.7 * 0.2 + 1e-6
  # the view still blocked BLOCKED_UNDECIDED_S after the full creep: said once, and waited through where it crept to (until
  # 2026-11-07 the 'blocked' handback; Kellen: "no, no handbacks"), never a go on the hidden lanes
  j = Junction()
  j.wait(0.5, boxes=[box], objects=[turner])
  j.wait(BLOCKED_UNDECIDED_S - 0.5, boxes=[box], objects=[turner], v=0., distance=HOLD_D - CREEP_MAX_M)
  assert j.decisions[-1].prompt == 'creepingForView'
  d = j.wait(60., boxes=[box], objects=[turner], v=0., distance=HOLD_D - CREEP_MAX_M)
  assert d.state == 'creep' and d.prompt == 'waitingForGap' and d.reason == LONG_WAIT_REASON['view'] == "Waiting for the view to clear"
  assert d.creep_m == CREEP_MAX_M and not any(x.go or x.state == 'undecided' for x in j.decisions)
  # ...and the turner gone, the car goes from where it crept to on the band it now sees
  d = j.wait(CLEAR_WINDOW_S + 0.3, v=0., distance=HOLD_D - CREEP_MAX_M)
  assert any(x.go for x in j.decisions[-round((CLEAR_WINDOW_S + 0.3) / DT_MDL):])
  # the turner goes: the view clears, the band is clear, and the car goes from where it crept to
  j = Junction()
  j.wait(0.5, boxes=[box], objects=[turner])
  j.wait(2., boxes=[box], objects=[turner], v=0., distance=HOLD_D - 2.)
  d = j.wait(CLEAR_WINDOW_S + 0.2)
  assert any(x.go for x in j.decisions[-round((CLEAR_WINDOW_S + 0.2) / DT_MDL):])


def test_a_moving_or_new_track_is_traffic_not_an_occluder():
  for fields in ({'vx': -3.}, {'age': VEL_TRUST_HITS - 1}):
    turner = {**oncoming(HOLD_D + 12., 3.6, vx=0.), **fields, 'trackId': 7, 'box': [0.5, 0.4, 0.6, 0.5], 'camera': 'road'}
    box = raw(HOLD_D + 12., 3.6)
    box = RawBox(box.camera, box.cls, box.conf, box.bearing_deg, box.span_deg, box.xy, box.near_m, (0.5, 0.4, 0.6, 0.5))
    j = Junction()
    d = j.wait(CLEAR_WINDOW_S + 1., boxes=[box], objects=[turner])
    assert not d.view_blocked and d.state == 'waitGap' and not any(x.go for x in j.decisions), fields


def test_the_exit_crosswalk_and_exit_lane_must_be_clear():
  j = Junction()
  d = j.wait(CLEAR_WINDOW_S + 1., objects=[ped(HOLD_D + 2., 8.)])
  assert not any(x.go for x in j.decisions) and d.state == 'waitGap' and not d.crosswalk_clear
  # a car standing in the exit lane: the car would stop in the oncoming lanes behind it
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 1., objects=[oncoming(HOLD_D + 2., 7., vx=0.)])
  assert not any(x.go for x in j.decisions)
  # nor while the scene holds for anything
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 1., scene_clear=False)
  assert not any(x.go for x in j.decisions)


def go(j):
  for _ in range(round((CLEAR_WINDOW_S + 0.5) / DT_MDL)):
    if j.step().go:
      j.held = False
      return
  raise AssertionError("no go")


def test_recertified_at_first_motion_and_aborted_short_of_the_oncoming_lanes():
  # the go is certified with a second's latency in T_req; at the car's first motion (0.3 m/s) the band is checked again for
  # the remaining clear time. A car that came into the band since: the go is withdrawn while the car can still stop short
  # of the oncoming lanes on the comfortable curve, and it waits again
  j = Junction()
  go(j)
  j.wait(0.8, v=0.)
  d = j.step([raw(110., 3.6)], v=0.35, distance=HOLD_D - 0.05)
  assert d.state == 'waitGap' and d.alert == 'oncomingHolding' and d.stops and not d.go
  stop = d.stops[0]
  assert stop.speed_cap < 0.35 and stop.accel_cap >= -2.
  # nothing in the band at first motion: on, and committed once it can't stop short of the oncoming lanes
  j = Junction()
  go(j)
  d = j.step(v=0.35, distance=HOLD_D - 0.05)
  assert d.state == 'go' and not d.stops
  d = j.step([raw(110., 3.6)], v=3.5, distance=ONCOMING_ENTRY_M + 1.)
  assert d.state == 'committed' and not d.stops and not d.go
  # committed: never cancelled, whatever comes into the band
  for _ in range(20):
    d = j.step([raw(30., 3.6)], v=3.5, distance=ONCOMING_ENTRY_M - 2.)
    assert d.state == 'committed' and not d.stops


def test_a_go_withdrawn_for_not_moving_is_left_to_its_handback():
  j = Junction()
  go(j)
  d = j.step(go_withdrawn=True)
  assert not d.go and not d.stops and d.state == 'go'


def test_unknown_junction_position_is_a_fault():
  j = Junction()
  d = j.wait(0.5, junction_uncertain=True)
  assert d.state == 'undecided' and d.prompt == 'leftUndecidedTakeControl'


def test_announce_says_what_it_would_do_and_never_goes():
  j = Junction(mode='announce')
  d = j.step([raw(100., 3.6)])
  assert d.state == 'announce' and not d.go and d.verdict == 'wait'
  d = j.wait(CLEAR_WINDOW_S + 0.5)
  assert not any(x.go for x in j.decisions) and d.verdict == 'go' and d.creep_m == 0.


def test_off_does_nothing():
  j = Junction(mode='off')
  d = j.wait(CLEAR_WINDOW_S + 0.5)
  assert d.state == 'off' and not d.go and d.view == 'none' and d.prompt == 'none'


def test_its_geometry_takes_the_furthest_reading_of_the_junction():
  # BeamNG unprotected_left_gap6 (2026-10-28): the hold latched at the nearest of navd's readings of the junction (route
  # follower: 9 m), with navd's own reading and the junction itself 22 m away, and the left certified for a 17 m path that
  # was 30 m. Its clear time and reach take the furthest reading (the follower's spread), never the nearest
  near = t_req(HOLD_D, 1)
  j = Junction()
  d = j.step(spread_m=13.)
  assert d.t_req > near + 2.5 and abs(d.t_req - t_req(HOLD_D + 13., 1)) < 1e-6
  assert abs(d.r_req - r_req(V_25, t_req(HOLD_D + 13., 1), HOLD_D + 13.)) < 1e-6
  # a walker in the exit crosswalk of the junction where it really is, 13 m further than the nearest reading put it
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 1., objects=[ped(HOLD_D + 13. + 10., 8.)], spread_m=13.)
  assert not any(x.go for x in j.decisions)


def test_held_short_of_its_line_it_rolls_up_before_judging_the_reach():
  # BeamNG unprotected_left_gap4 (2026-10-28): the latch held the car 16 m out, 7 m short of its 9 m line, where the longer
  # crossing put R_req at 164 m, past the 150 m range: undecided at once. It rolls up to its line first (the latch's creep,
  # under 0.7 m/s), where R_req is 134 m, and judges the gap there
  from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, ROLL_UP_STALL_S
  j = Junction()
  d = j.step(distance=16.)
  assert d.r_req > RANGE and d.state == 'creep' and abs(d.creep_m - (16. - LINE_D)) < 1e-6 and d.reason == "Rolling up to the line"
  for dist in (14., 12., 10., LINE_D + 0.3):
    d = j.step(distance=dist, v=0.6)
    assert d.state in ('creep', 'waitGap') and not d.go
  d = j.wait(CLEAR_WINDOW_S + 0.5, distance=LINE_D + 0.3, v=0.)
  assert any(x.go for x in j.decisions) and all(x.state != 'undecided' for x in j.decisions)
  # a roll-up the car never makes (the model holding it): after ROLL_UP_STALL_S judged where it stands, on the band it sees
  # (2026-11-07: no reach handback), and gone on it when clear
  j = Junction()
  j.wait(ROLL_UP_STALL_S + CLEAR_WINDOW_S + 0.5, distance=16., v=0.)
  assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions)


def test_the_window_starts_at_the_hold_line():
  # DESIGN's flow is hold, then judge: the clear window the approach ran up never carries into the hold, so the car stands
  # at its line CLEAR_WINDOW_S at least before a go, never going on the frame it latches
  j = Junction()
  j.held = False
  j.wait(CLEAR_WINDOW_S + 1., distance=30., v=5.)
  j.held = True
  d = j.step(distance=HOLD_D, v=0.)
  assert not d.go and d.clear_s < 0.1
  j.wait(CLEAR_WINDOW_S - 0.2, distance=HOLD_D)
  assert not any(x.go for x in j.decisions)
  j.wait(0.4, distance=HOLD_D)
  assert any(x.go for x in j.decisions)


def test_only_an_occluders_own_box_is_exempt():
  # a car passing in the through lane beside a standing opposing left-turner is traffic, not the turner's box: the ground
  # gate alone (2 m + 15% of range) would take one 3.6 m across for it
  turner = oncoming(HOLD_D + 12., 3.6, vx=0., trackId=7, box=[0.5, 0.4, 0.6, 0.5], camera='road')
  passing = raw(HOLD_D + 12., 7.2)
  from openpilot.selfdrive.turnpilot.unprotected_left import matches
  assert not matches(passing, scene([turner]).objects[0])
  same = raw(HOLD_D + 12.5, 3.9)
  assert matches(same, scene([turner]).objects[0])


def test_a_creep_the_car_never_makes_is_said_and_waited_through_never_handed_back(log_events):
  # skeptic G4 (blocker 4): the view blocked, the band clear, the creep asked for, and the car never moves (the model holding
  # a standstill: G0's tapless stand). The full-creep timer never started (blocked_s counts only once crept), so it stood in
  # 'creep' for minutes with only the visual creepingForView. Standing CREEP_STALL_S in a creep it hasn't made was the
  # creepStall handback until 2026-11-07 (Kellen: "no, no handbacks"): now its line says the wait, it is logged once, the
  # creep stays asked (the model may yet take it), and the view clearing lets the car go from where it stands
  turner = oncoming(HOLD_D + 12., 3.6, vx=0., trackId=7, box=[0.5, 0.4, 0.6, 0.5], camera='road')
  box = raw(HOLD_D + 12., 3.6)
  box = RawBox(box.camera, box.cls, box.conf, box.bearing_deg, box.span_deg, box.xy, box.near_m, (0.5, 0.4, 0.6, 0.5))
  j = Junction()
  d = j.wait(CREEP_STALL_S - 0.5, boxes=[box], objects=[turner], v=0.)
  assert d.state == 'creep' and d.prompt == 'creepingForView'
  d = j.wait(120., boxes=[box], objects=[turner], v=0.)
  assert d.state == 'creep' and d.prompt == 'waitingForGap' and d.reason == LONG_WAIT_REASON['view'] and d.creep_m == CREEP_MAX_M
  assert not any(x.go or x.state == 'undecided' or x.prompt in TAKE_CONTROL_PROMPTS for x in j.decisions)
  said = [kw for name, kw in log_events if name == 'turnpilot.left_view_wait']
  assert len(said) == 1 and said[0]['why'] == 'creepStall', said
  j.wait(CLEAR_WINDOW_S + 0.3, v=0.)  # the turner gone: the view open from where the car stands
  assert any(x.go for x in j.decisions[-round((CLEAR_WINDOW_S + 0.3) / DT_MDL):])
  # a creep that moves isn't stalled; nor one held by something else (a walker crossing in front)
  j = Junction()
  d = j.wait(CREEP_STALL_S + 2., boxes=[box], objects=[turner], v=0.5, distance=HOLD_D - 1.)
  assert d.state == 'creep'
  j = Junction()
  d = j.wait(CREEP_STALL_S + 2., boxes=[box], objects=[turner], v=0., scene_clear=False)
  assert d.state == 'creep'


def test_a_commit_while_standing_held_before_any_go_is_a_fault_never_a_freeze():
  # skeptic G4 (blocker 3): the scene committed to the junction (a reading put the car past its line) while the latch held it
  # and the left had never gone: the left took it as committed, sent nothing, and the car stood forever, silently. Before
  # the left's own go a commit is no commit; held standing, it is a contradiction: undecided, out loud
  j = Junction()
  j.wait(1.)
  d = j.step(committed=True)
  assert d.state == 'undecided' and d.why == 'position' and d.prompt == 'leftUndecidedTakeControl' and not d.go
  # after its go the car must have moved before anything commits it: at the go, a scene commit leaves its abort window open
  # until it pulls away
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 0.2)
  assert any(x.go for x in j.decisions)
  j.held = False
  d = j.step(committed=True)
  assert d.state == 'go'
  d = j.step(boxes=[raw(80., 3.6)], committed=True)
  # standing where it was held, it can always stay: a car in the band withdraws the go
  assert d.state == 'waitGap' and d.alert == 'oncomingHolding' and d.stops and d.stops[0].should_stop
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 0.2)
  j.held = False
  d = j.step(v=0.5, distance=HOLD_D - 0.1, committed=True)  # its first motion: the re-check, then committed
  assert d.state == 'go'
  d = j.step(v=0.6, distance=HOLD_D - 0.2, committed=True)
  assert d.state == 'committed'
  # a reading too near for the first-motion re-check (under HELD_MIN_M: skeptic SIMFIX P2), standing at its line or on the
  # go before it moves, a scene commit with it or not: the position fault, the go withdrawn where it stands, held
  for committed in (False, True):
    j = Junction()
    j.wait(1.)
    d = j.step(distance=HELD_MIN_M - 0.1, committed=committed)
    assert d.state == 'undecided' and d.why == 'position' and not d.go, committed
    for v in (0., 0.5):  # the go's frame on, and its first motion
      j = Junction()
      j.wait(CLEAR_WINDOW_S + 0.2)
      j.held = False
      d = j.step(v=v, distance=ONCOMING_ENTRY_M - 0.5, committed=committed)
      assert d.state == 'undecided' and d.why == 'position' and d.prompt == 'leftUndecidedTakeControl', (committed, v)
      assert d.stops and d.stops[0].should_stop == (v < 0.3), (committed, v)
      assert j.step(v=0., distance=ONCOMING_ENTRY_M - 0.5, committed=committed).state == 'undecided'


def test_a_scene_commit_before_any_go_is_a_fault_moving_or_standing_and_only_the_drivers_release_is_a_commit():
  # skeptic G4 (residual): a nearer reading arriving while the car rolled up to its line at 0.9-2.2 m/s committed the moving
  # car's scene (a2d2ebe's guard needs it standing), the left took that as its commit with no go of its own and stopped
  # judging, and the latch held the car at the line for good, saying nothing (121 s of 150). The left is committed only by its
  # own go and motion: a scene commit before its go is 'position', out loud, whether the car moves or stands
  for v in (0.93, 2.22):
    j = Junction()
    j.held = False
    d = j.step(v=v, distance=HOLD_D + 2., committed=True)
    assert d.state == 'undecided' and d.why == 'position' and d.prompt == 'leftUndecidedTakeControl' and not d.go, v
    d = j.step(v=0., distance=HOLD_D, committed=True)
    j.held = True
    assert j.step(committed=True).state == 'undecided'
  # the driver releasing the junction (a gas press at the line: scene_policy's released_id) and driving on is theirs: committed,
  # nothing said
  j = Junction()
  j.held = False
  d = j.step(v=2., distance=HOLD_D - 1., released=True)
  assert d.state == 'committed' and d.prompt == 'none' and not d.stops


@pytest.mark.parametrize('how', ['released', 'disengaged'])
def test_a_left_the_driver_takes_from_its_line_is_driven_never_the_position_fault(how):
  # drive 29 (2026-11-08, cause 4): every left the driver took at its line said 'Junction position unsure: take control'
  # (leftUndecidedTakeControl, the handback chime) on the frame the scene released the hold under their gas (467.0 s), the
  # latch letting go as driverWent 0.9 s later. The driver taking it is no fault of TurnPilot's: driven, said as the crossing
  # it is (its own words, never a take-control of the left's), and the left is done
  j = Junction()
  j.wait(1.)
  assert j.decisions[-1].state == 'waitGap'
  kw = {'released': True} if how == 'released' else {'committed': True, 'enabled': False, 'v': 1.5}
  d = j.step(**kw)
  assert d.state == 'undecided' and d.why == ul.DRIVEN and d.why != 'position'
  assert d.prompt not in TAKE_CONTROL_PROMPTS and d.reason == ul.DRIVEN_REASON[LEFT] and not d.go and not d.stops
  d = j.step(v=3., distance=HOLD_D - 2., driven=True)
  assert d.why == ul.DRIVEN and d.prompt not in TAKE_CONTROL_PROMPTS and not d.go
  # engaged, a scene commit by a reading before any go is still the position fault (the test above)
  j = Junction()
  j.wait(1.)
  assert j.step(committed=True).why == 'position'


def test_a_left_lc03_placed_in_the_wrong_lane_is_driven_from_its_lane_never_handed_back():
  # review 2026-11-07 (LN-5): the lane planner's LC-03 verdict (its change blocked to the window's end) never skips the left;
  # Kellen, 2026-11-07 ("no, no handbacks"): it is driven from the lane it is in, gone on the gap rule as any other, after dark
  # too, and a car seen in the band still holds it
  j = Junction()
  j.held = False
  d = j.step(wrong_lane=True)
  assert d.state == 'approach' and d.prompt == 'none'
  j.held = True
  j.wait(CLEAR_WINDOW_S + 1., wrong_lane=True, dark=True)
  assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions)
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 1., wrong_lane=True, boxes=[raw(60., 3.6)])
  assert not any(x.go for x in j.decisions)


def test_an_opposing_road_not_known_straight_over_r_req_is_judged_on_its_straight_band():
  # skeptic G4 (residual): the band is straight on from our approach, so on a road curving away from the turn a car up the bend
  # is outside it. Kellen, 2026-11-07 ("no, no handbacks"; the safety driver supervises): never 'curve' any more, judged on the
  # band the camera covers, gone when clear; a car seen in it holds
  for straight in (0., 60., 120.):
    j = Junction()
    j.wait(CLEAR_WINDOW_S + 1., straight_m=straight)
    assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions), straight
    j = Junction()
    j.wait(CLEAR_WINDOW_S + 1., straight_m=straight, boxes=[raw(40., 3.6)])
    assert not any(x.go for x in j.decisions), straight
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 0.2, straight_m=250.)
  assert any(x.go for x in j.decisions)


def test_never_a_go_while_anything_else_holds_the_car_or_its_light_isnt_green():
  # skeptic G4 (blocker 2), DESIGN G4 go condition 5: with a red at the junction (accepted: the map's signal stop, other_hold;
  # or any source, the camera in shadow too: light) the band empty, it never goes; the window counts only from the green
  for kw in ({'other_hold': True}, {'light': 'red'}, {'light': 'yellow'}, {'light': 'unknown'}):
    j = Junction()
    d = j.wait(LIGHT_UNKNOWN_S - 0.5, **kw)
    assert not any(x.go for x in j.decisions) and d.state == 'waitGap', kw
    d = j.wait(CLEAR_WINDOW_S - 0.3, light='green')
    assert not any(x.go for x in j.decisions), kw  # the window started at the green
    j.wait(0.5, light='green')
    assert any(x.go for x in j.decisions), kw
  # no signal at the junction: as before
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 0.2, light='none')
  assert any(x.go for x in j.decisions)
  # a blocked view under a red: no creep past the line into the junction, it waits
  turner = oncoming(HOLD_D + 12., 3.6, vx=0., trackId=7, box=[0.5, 0.4, 0.6, 0.5], camera='road')
  box = raw(HOLD_D + 12., 3.6)
  box = RawBox(box.camera, box.cls, box.conf, box.bearing_deg, box.span_deg, box.xy, box.near_m, (0.5, 0.4, 0.6, 0.5))
  j = Junction()
  d = j.wait(2., boxes=[box], objects=[turner], light='red')
  assert d.state == 'waitGap' and d.creep_m == 0.


def test_a_light_it_cant_read_or_that_changes_while_it_waits_is_waited_through_never_handed_back(log_events):
  """Until 2026-11-07 these handed back, the car held: no read ('unknown': navd's controls failing, whether a signal is there
  not known) for LIGHT_UNKNOWN_S, a red past LIGHT_RED_MAX_S ('light'), and the light going red or yellow while it waited on
  a green (lightChange). Kellen: "no, no handbacks". Each is now a wait at its line saying so, logged once, never a go under
  the red or yellow read (the hard rail): gone on the next green, or on the gap once turn_light gives no red or yellow. This
  harness feeds turn_light's value as given; in the closed loop a red that raised TurnPilot's own stop stays 'red' while the
  stop is in force, read or not (test_turnpilotd_left: a camera red lost at the line), so 'none' after a red is only the
  camera's shadow red, which raised none."""
  def no_handback(j):
    return not any(x.go or x.state == 'undecided' or x.prompt in TAKE_CONTROL_PROMPTS for x in j.decisions)
  # whether there is a light isn't known (SIGNAL_NEEDS_GREEN): waited on, said once, its line plain
  j = Junction()
  d = j.wait(LIGHT_UNKNOWN_S + 60., light='unknown')
  assert d.state == 'waitGap' and d.prompt == 'waitingForGap' and d.reason == "Waiting: can't tell if there's a light" and no_handback(j)
  assert [kw['light'] for name, kw in log_events if name == 'turnpilot.left_light_wait'] == ['unknown']
  j.wait(CLEAR_WINDOW_S + 0.2, light='green')
  assert any(x.go for x in j.decisions)
  # a red is waited through, a red read that never ends too (past LIGHT_RED_MAX_S: said once more, never a handback)
  log_events.clear()
  j = Junction()
  d = j.wait(LIGHT_RED_MAX_S + 30., light='red')
  assert d.state == 'waitGap' and d.reason == "Waiting for the light" and no_handback(j)
  assert [kw['light'] for name, kw in log_events if name == 'turnpilot.left_light_wait'] == ['red']
  # waiting at the line on a green, the light goes yellow then red: held where it stands, short of the oncoming lanes, said
  # once for the change; the next green is a fresh window, and it goes on its gap
  log_events.clear()
  j = Junction()
  j.wait(1., boxes=[raw(90., 3.6)], light='green')
  d = j.step(boxes=[raw(90., 3.6)], light='yellow')
  assert d.state == 'waitGap' and d.reason == "Waiting for the light" and not d.stops and d.creep_m == 0.
  d = j.wait(40., light='red')
  assert d.state == 'waitGap' and no_handback(j)
  assert [kw['light'] for name, kw in log_events if name == 'turnpilot.left_light_changed'] == ['yellow']
  n = len(j.decisions)
  j.wait(CLEAR_WINDOW_S - 0.3, light='green')
  assert not any(x.go for x in j.decisions[n:])  # a fresh window from the green: never a window counted under the red
  d = j.wait(0.5, light='green')
  assert any(x.go for x in j.decisions[n:])
  # ...or, the red no longer read and no stop of TurnPilot's in force for it (turn_light's 'none': the camera's shadow red,
  # which raised none; an unread light is no reason to wait), on the gap, a fresh window
  j = Junction()
  j.wait(1., light='green')
  j.wait(5., light='red')
  n = len(j.decisions)
  j.wait(CLEAR_WINDOW_S - 0.3, light='none')
  assert not any(x.go for x in j.decisions[n:])
  j.wait(0.5, light='none')
  assert any(x.go for x in j.decisions[n:])
  # another change on the same hold is another event: the next green then red is said again, once
  log_events.clear()
  j = Junction()
  for _ in range(3):
    j.wait(1., boxes=[raw(90., 3.6)], light='green')
    j.wait(5., boxes=[raw(90., 3.6)], light='red')
  assert len([1 for name, _ in log_events if name == 'turnpilot.left_light_changed']) == 3


def test_a_go_never_outlives_another_hold_or_a_change_before_its_commit():
  # skeptic G4 (blocker 2): the go stayed 'go' through a red that held the car 14 s, and through a walker who came into the
  # exit crosswalk, and ran on the stale go when that hold let go. Before its commit, another hold, a light gone red or
  # yellow, the exit crosswalk or the view no longer clear withdraw the go: its own hold, waiting again, a fresh window
  walker = ped(HOLD_D + 4., 9.)
  turner = oncoming(HOLD_D + 12., 3.6, vx=0., trackId=7, box=[0.5, 0.4, 0.6, 0.5], camera='road')
  for kw in ({'other_hold': True}, {'scene_clear': False}, {'light': 'red'}, {'objects': [walker]}, {'objects': [turner]}):
    j = Junction()
    j.wait(CLEAR_WINDOW_S + 0.2, light='green')
    assert any(x.go for x in j.decisions)
    j.held = False
    d = j.step(**{'light': 'green', **kw})
    assert d.state == 'waitGap' and not d.go and d.stops and d.stops[0].should_stop, kw
    n = len(j.decisions)
    d = j.wait(CLEAR_WINDOW_S - 0.3, light='green')
    assert not any(x.go for x in j.decisions[n:]), kw  # a fresh window from where the hold let go
    j.wait(0.5, light='green')
    assert any(x.go for x in j.decisions[n:]), kw
  # rolling, it withdraws only where it can stop short of the oncoming lanes; past that it's on (never stopped in them)
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 0.2)
  j.held = False
  j.step(v=0.5, distance=HOLD_D - 0.1)
  d = j.step(v=1.5, distance=HOLD_D - 0.5, objects=[walker])
  assert d.state == 'waitGap' and d.stops and d.stops[0].accel_cap >= -2.0
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 0.2)
  j.held = False
  j.step(v=0.5, distance=HOLD_D - 0.1)
  d = j.step(v=3.5, distance=ONCOMING_ENTRY_M + 2., objects=[walker])
  assert d.state in ('go', 'committed') and not d.stops


def test_held_short_of_its_line_under_a_red_it_waits_where_it_is():
  # held short of its line where R_req from there is past the range, under a red (or anything else holding the car): no roll-up
  # toward the junction under it, and no 'range' fault for a reach it will judge from the line once it has rolled up
  from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, ROLL_UP_STALL_S
  for kw in ({'light': 'red'}, {'other_hold': True}):
    j = Junction()
    d = j.wait(ROLL_UP_STALL_S + 2., distance=16., v=0., **kw)
    assert d.r_req > RANGE and d.state == 'waitGap' and d.creep_m == 0. and not any(x.go for x in j.decisions), kw
    d = j.step(distance=16., v=0., light='green')
    assert d.state == 'creep' and abs(d.creep_m - (16. - LINE_D)) < 1e-6 and d.reason == "Rolling up to the line", kw


# ------------------------------------------------------------------- the turn signal (car_signals.py, DESIGN phase4/blinker 6.7)

def test_no_go_and_no_creep_without_the_signal_and_it_waits_for_it_never_handed_back(log_events):
  """The left goes only with its side lit (LEFT_GO_NEEDS_SIGNAL: IC 9-21-8-25, and a car standing dark in a left-turn position
  invites a pass into its path). On this car only the driver can light it, prompted per side on repeat (car_signals.py).
  Until 2026-11-07 the left handed back after LEFT_SIGNAL_WAIT_S dark ('unsignalled'); now it waits at its line saying so,
  logged once, never gone unsignalled however long, and goes on a gap once lit."""
  from openpilot.selfdrive.turnpilot.unprotected_left import LEFT_SIGNAL_WAIT_S, NO_SIGNAL_REASON
  j = Junction()
  d = j.wait(CLEAR_WINDOW_S + 1., signal_lit=False)
  assert not any(x.go for x in j.decisions) and d.state == 'waitGap' and d.reason == NO_SIGNAL_REASON and d.creep_m == 0.
  d = j.wait(LEFT_SIGNAL_WAIT_S + 120., signal_lit=False)
  # its own prompt stands aside for car_signals' "Signal LEFT Now" and its chime (the module's no_signal comment)
  assert d.state == 'waitGap' and d.prompt == 'none' and d.reason == NO_SIGNAL_REASON
  assert not any(x.go or x.state == 'undecided' or x.prompt in TAKE_CONTROL_PROMPTS for x in j.decisions)
  said = [kw for name, kw in log_events if name == 'turnpilot.left_unsignalled']
  assert len(said) == 1 and abs(said[0]['unsignalled_s'] - LEFT_SIGNAL_WAIT_S) < 0.1, said
  # lit at last: the window counts from the lamp, then it goes
  n = len(j.decisions)
  j.wait(CLEAR_WINDOW_S - 0.3, signal_lit=True)
  assert not any(x.go for x in j.decisions[n:])
  j.wait(0.5, signal_lit=True)
  assert any(x.go for x in j.decisions[n:])
  # lit: the window counts from the lamp, then it goes
  j = Junction()
  j.wait(2., signal_lit=False)
  j.wait(CLEAR_WINDOW_S + 0.2, signal_lit=True)
  assert any(x.go for x in j.decisions)


def test_each_long_wait_is_logged_once_per_spell_never_per_frame(log_events):
  # a converted wait is said once as its old bound passes, and again only once what held it let go and it came round again:
  # the lamp lit then dark again, a red after a green, the creep left and stalled again
  from openpilot.selfdrive.turnpilot.unprotected_left import LEFT_SIGNAL_WAIT_S
  j = Junction()
  car = [raw(90., 3.6)]  # a car in the band: no go meanwhile
  j.wait(LEFT_SIGNAL_WAIT_S + 20., boxes=car, signal_lit=False)
  j.wait(1., boxes=car, signal_lit=True)
  j.wait(LEFT_SIGNAL_WAIT_S + 20., boxes=car, signal_lit=False)
  assert len([1 for name, _ in log_events if name == 'turnpilot.left_unsignalled']) == 2
  log_events.clear()
  for _ in range(2):
    j.wait(LIGHT_RED_MAX_S + 5., boxes=car, light='red')
    j.wait(1., boxes=car, light='green')
  assert len([1 for name, _ in log_events if name == 'turnpilot.left_light_wait']) == 2
  assert not any(x.go or x.state == 'undecided' for x in j.decisions)


def test_signal_gate_off_is_todays_rule(monkeypatch):
  import openpilot.selfdrive.turnpilot.unprotected_left as ul
  monkeypatch.setattr(ul, 'LEFT_GO_NEEDS_SIGNAL', False)
  a, b = Junction(), Junction()
  for _ in range(round((CLEAR_WINDOW_S + 1.) / DT_MDL)):
    da, db = a.step(signal_lit=False), b.step()
    assert (da.state, da.go, da.reason) == (db.state, db.go, db.reason)


def test_a_veto_at_the_line_hands_back():
  j = Junction()
  j.wait(1.)
  d = j.step(signal_veto='driverSignal')
  assert d.state == 'undecided' and d.why == 'driverSignal' and not d.go
  assert j.wait(CLEAR_WINDOW_S + 1.).state == 'undecided'


def far_wait(j, seconds, far=(), every=2, **kw):
  """Full runs on every other frame, the far crop's on the frames between (modeld), one in every/2 of them; far: its boxes."""
  d = None
  for k in range(round(seconds / DT_MDL)):
    args = {'raw_new': k % 2 == 0}
    if k % every == 1:
      args['far'] = far
    d = j.step(**args, **kw)
    if d.go:
      return d
  return d


V_45, V_55 = 20.1, 24.6  # m/s


def test_route_times_reach_is_taken_where_the_car_may_stand():
  """BUILD_FINDINGS D4: route time checked R_req at LINE_D, a best case the live check at the line rarely meets (it takes
  where the car stands and the furthest reading). reach_at_line's setback is how much further back the car may stand: R_req
  from LINE_D + setback, rising past the 6.2 m ramp by about 4.4 m a metre at 25 mph."""
  from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, r_req, reach_at_line, t_req
  assert reach_at_line(11.18, 1) == reach_at_line(11.18, 1, 0.) == r_req(11.18, t_req(LINE_D, 1), LINE_D)
  for setback in (2., 15.):
    assert math.isclose(reach_at_line(11.18, 1, setback), r_req(11.18, t_req(LINE_D + setback, 1), LINE_D + setback))
  assert 4. < (reach_at_line(11.18, 1, 15.) - reach_at_line(11.18, 1, 10.)) / 5. < 4.8
  assert reach_at_line(None, None, 15.) == reach_at_line(20.1, 2, 15.)  # unmapped: 45 mph, two lanes


def test_45_and_55_mph_reach_needs_the_far_crop_and_its_measured_range():
  # DESIGN phase 4 G5, the owner's "support both": R_req from the line is ~249-263 m at 45 mph and ~303-319 m at 55 mph,
  # past the full pass's range; the far crop's runs judge the band out there, but only under a measured far range
  # (TurnPilotLeftFarRangeM), with the far crop reading
  from openpilot.selfdrive.turnpilot.unprotected_left import reach_at_line
  assert 240. < reach_at_line(V_45, 1) < reach_at_line(V_45, 2) < 270. and 295. < reach_at_line(V_55, 1) < reach_at_line(V_55, 2) < 325.
  for v, lanes in ((V_45, 1), (V_45, 2), (V_55, 1), (V_55, 2)):
    j = Junction(v_limit=v, lanes=lanes)
    d = far_wait(j, CLEAR_WINDOW_S + 1., far_range_m=300., far_live=True)
    assert d.go and d.far_needed and d.r_req > RANGE, (v, lanes)
    assert d.clear_s >= CLEAR_WINDOW_S - 1e-6
  # unmeasured far range (the default), past even the far crop's measured range, or the far crop off: judged on the band the
  # camera covers and gone when clear (Kellen, 2026-11-07: no handbacks), never undecided for the reach
  for v, lanes, kw in ((V_45, 1, {'far_live': True}), (V_55, 2, {'far_range_m': 280., 'far_live': True}),
                       (V_45, 1, {'far_range_m': 300., 'far_live': False})):
    j = Junction(v_limit=v, lanes=lanes)
    far_wait(j, CLEAR_WINDOW_S + 1., **kw)
    assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions), kw
  # 25 mph needs no far crop
  j = Junction()
  d = far_wait(j, CLEAR_WINDOW_S + 1., far_live=False)
  assert d.go and not d.far_needed
  # the full pass's range unmeasured: the far crop covers the band only past it, so it extends nothing and its runs gate
  # nothing (never a stale far view): judged on what the camera boxes
  j = Junction(v_limit=V_45, rng=0.)
  far_wait(j, CLEAR_WINDOW_S + 1., far_range_m=300., far_live=True)
  assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions)


def test_a_far_box_in_the_band_blocks_at_any_range_when_the_far_crop_is_needed():
  # at 230-280 m one pixel of box bottom is 40-56 m and 0.15 deg of pitch is the whole NEAR_PX margin: a far run never clears
  # a car in the band by its range
  for box in (raw(200., 3.6), raw(400., 6.), raw(600., 3.6, near=500.), raw(300., 4., ranged=False)):
    j = Junction(v_limit=V_45)
    d = far_wait(j, CLEAR_WINDOW_S + 1., far=[box], far_range_m=300., far_live=True)
    assert not d.go and d.clear_s == 0. and d.far_blockers == 1, box
  # outside the band (the car's own side of the road, beside the car) it doesn't
  j = Junction(v_limit=V_45)
  assert far_wait(j, CLEAR_WINDOW_S + 1., far=[raw(150., -12.), raw(2., 3.6)], far_range_m=300., far_live=True).go
  # not needed (25 mph): a far box blocks as a full pass box does, within R_req by its nearest plausible range
  j = Junction()
  assert far_wait(j, CLEAR_WINDOW_S + 1., far=[raw(250., 3.6)]).go
  j = Junction()
  assert not far_wait(j, CLEAR_WINDOW_S + 1., far=[raw(100., 3.6)]).go
  # a full pass box still blocks with the far runs clear
  j = Junction(v_limit=V_45)
  d = None
  for k in range(round((CLEAR_WINDOW_S + 1.) / DT_MDL)):
    d = j.step([raw(120., 3.6)], **({'far': []} if k % 2 else {}), far_range_m=300., far_live=True)
  assert not d.go and d.clear_s == 0.


def test_needed_far_runs_must_be_fresh_and_cover_the_band():
  from openpilot.selfdrive.turnpilot.unprotected_left import FAR_FRESH_S
  # full runs clear, no far run: no go while the far slot might still read; once it has seen nothing of the band for
  # STALE_UNDECIDED_S, judged on the full pass's band and gone when clear, never a farStale handback (2026-11-07: no handbacks)
  j = Junction(v_limit=V_45)
  d = j.wait(STALE_UNDECIDED_S - 0.2, far_range_m=300., far_live=True)
  assert not any(x.go for x in j.decisions)
  d = j.wait(CLEAR_WINDOW_S + 1., far_range_m=300., far_live=True)
  assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions)
  # far runs further apart than FAR_FRESH_S: no go on them while the full pass's band hasn't been clear a whole window. Review
  # 2026-11-07 (R2): that held an empty road's left forever while the far crop ran 1-3 times a second (far_needed dropped
  # only past one far gap over STALE_UNDECIDED_S), where one that didn't run went on the full pass's band. Once that band has
  # been clear a whole window with the far slot's age alone holding it, the left is judged on it (far_behind) and goes; a far
  # box in the band within FAR_BLOCK_RECENT_S still holds it
  for every in (math.ceil(FAR_FRESH_S / DT_MDL) + 2, 30, 38):  # 1.67, 0.67 and 0.53 far runs a second, gaps under 2 s
    j = Junction(v_limit=V_45)
    d = far_wait(j, CLEAR_WINDOW_S - 0.2, every=every, far_range_m=300., far_live=True)
    assert not any(x.go for x in j.decisions), every
    d = far_wait(j, 1., every=every, far_range_m=300., far_live=True)
    assert d.go and not d.far_needed and not any(x.state == 'undecided' for x in j.decisions), every
    j = Junction(v_limit=V_45)
    d = far_wait(j, 20., far=[raw(250., 3.6)], every=every, far_range_m=300., far_live=True)
    assert not any(x.go for x in j.decisions), every  # the far crop's car holds it, however slow its runs
  # ...and once its runs hold FAR_FRESH_S a window again it is needed again (disengaged here: the rule read with no go taken)
  from openpilot.selfdrive.turnpilot.unprotected_left import FAR_BEHIND_CLEAR_S
  j = Junction(v_limit=V_45)
  far_wait(j, CLEAR_WINDOW_S + 0.5, every=math.ceil(FAR_FRESH_S / DT_MDL) + 2, far_range_m=300., far_live=True, enabled=False)
  assert j.left.far_behind and not j.decisions[-1].far_needed
  far_wait(j, FAR_BEHIND_CLEAR_S + 0.2, every=2, far_range_m=300., far_live=True, enabled=False)
  assert not j.left.far_behind and j.decisions[-1].far_needed
  # far runs that don't cover the band's bearings past the full pass's range (aimed off, clamped at the frame's edge, the
  # horizon out of the crop): seen nothing there, judged on the full pass's band once stale and gone when clear
  for cover in ((-20., 3.), (5., 30.), None):
    j = Junction(v_limit=V_45)
    d = far_wait(j, STALE_UNDECIDED_S - 0.2, cover=cover, far_range_m=300., far_live=True)
    assert not any(x.go for x in j.decisions), cover
    d = far_wait(j, CLEAR_WINDOW_S + 1., cover=cover, far_range_m=300., far_live=True)
    assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions), cover
  # ...and a car the full pass boxes in the band still holds it there
  j = Junction(v_limit=V_45)
  far_wait(j, STALE_UNDECIDED_S + 0.2, cover=(-20., 3.), far_range_m=300., far_live=True)
  for _ in range(round((CLEAR_WINDOW_S + 1.) / DT_MDL)):
    d = j.step([raw(60., 3.6)], far_range_m=300., far_live=True)
  assert not any(x.go for x in j.decisions) and d.clear_s == 0.
  # yawed 10 deg toward the turn (after a withdrawn go), the band is 10 deg to the right in the car's frame
  j = Junction(v_limit=V_45)
  d = far_wait(j, CLEAR_WINDOW_S + 1., cover=(-25.6, 5.6), heading_deg=10., far_range_m=300., far_live=True)
  assert d.go
  j = Junction(v_limit=V_45)
  d = far_wait(j, STALE_UNDECIDED_S + 0.5, cover=FULL_COVER, heading_deg=-14., far_range_m=300., far_live=True)
  assert not any(x.go for x in j.decisions)


def test_the_far_crop_is_aimed_up_the_oncoming_approach():
  from openpilot.selfdrive.turnpilot.route_follower import RIGHT
  from openpilot.selfdrive.turnpilot.unprotected_left import far_aim
  # the bearings it must cover past the full pass's range, toward the turn: centred on them, in the car's frame
  lo, hi = -2., math.degrees(math.atan2(20., RANGE)) + 2.
  assert abs(far_aim(0., LEFT, RANGE) - (lo + hi) / 2) < 1e-6 and abs(far_aim(0., RIGHT, RANGE) + (lo + hi) / 2) < 1e-6
  assert abs(far_aim(10., LEFT, RANGE) - ((lo + hi) / 2 - 10.)) < 1e-6
  assert abs(far_aim(0., LEFT, 0.)) <= 15.  # unmeasured full range: never off the frame
  j = Junction(v_limit=V_45)
  d = far_wait(j, 0.5, far_range_m=300., far_live=True)
  assert abs(d.far_bearing - far_aim(0., LEFT, RANGE)) < 1e-6


def test_a_far_box_withdraws_a_go_before_its_commit():
  j = Junction(v_limit=V_45)
  d = far_wait(j, CLEAR_WINDOW_S + 1., far_range_m=300., far_live=True)
  assert d.go
  d = j.step(far=[raw(260., 3.6)], far_range_m=300., far_live=True)
  assert not d.go and d.state == 'waitGap' and d.alert == 'oncomingHolding' and d.stops


def far_blocked_wait(j, seconds, x, y, seen_every=1, light='none', **kw):
  """Held at a far-needed left's line: full runs on every other frame, the far crop's on the frames between with a car
  standing at (x, y) found on one far run in seen_every. Stops at a go or undecided."""
  d, far_n = None, 0
  for k in range(round(seconds / DT_MDL)):
    args = {'raw_new': k % 2 == 0, 'far_range_m': 300., 'far_live': True, 'light': light, **kw}
    if k % 2 == 1:
      far_n += 1
      args['far'] = [raw(x, y, conf=0.5)] if far_n % seen_every == 0 else []
    d = j.step(**args)
    if d.go or d.state == 'undecided':
      break
  return d


def test_a_far_car_standing_in_the_band_is_said_and_holds_the_left_never_an_endless_silent_wait(log_events):
  # skeptic G6: any far box in the band blocks at any range (math.inf: its range can't clear it), so a car standing far up
  # the road in the band (parked on the oncoming side, queued at the next light in the car's own lanes, or just far up
  # them) held a 45-55 mph left at its line for good, silently (waitingForGap). Moving traffic reaches the full pass's range
  # within FAR_BLOCKED_UNDECIDED_S; waiting on far boxes alone for that long, the car can't tell standing from coming. Until
  # 2026-11-07 that was the farBlocked handback; now (Kellen: "no, no handbacks") the car the detector sees still holds the
  # left, and its line says what it waits for from FAR_BLOCKED_UNDECIDED_S, logged once: never a go, never silent
  from openpilot.selfdrive.turnpilot.unprotected_left import FAR_BLOCKED_UNDECIDED_S
  assert 30. <= FAR_BLOCKED_UNDECIDED_S <= 45.  # DESIGN's no gap in 45 s, scoped to the far crop's boxes alone
  for v in (V_45, V_55):
    for x, y in ((280., 5.4), (300., 0.), (400., 0.)):
      for seen_every in (1, 2, 10):  # every far run, half of them (the far recall G6 is built for), one a second
        log_events.clear()
        j = Junction(v_limit=v)
        d = far_blocked_wait(j, FAR_BLOCKED_UNDECIDED_S + 30., x, y, seen_every)
        case = (v, x, y, seen_every, d.state, d.why, round(j.t, 1))
        assert not any(x.go or x.state == 'undecided' or x.prompt in TAKE_CONTROL_PROMPTS for x in j.decisions), case
        assert all(x.state == 'waitGap' and x.prompt == 'waitingForGap' for x in j.decisions), case
        said = [k for k, x in enumerate(j.decisions) if x.reason == LONG_WAIT_REASON['farBlocked']]
        assert said and FAR_BLOCKED_UNDECIDED_S - 0.1 <= said[0] * DT_MDL <= FAR_BLOCKED_UNDECIDED_S + 2 * seen_every * DT_MDL + 0.1, case
        assert len([1 for name, _ in log_events if name == 'turnpilot.left_far_wait']) == 1, case


def test_moving_traffic_and_other_holds_never_make_a_far_car_a_fault():
  from openpilot.selfdrive.turnpilot.unprotected_left import FAR_BLOCKED_UNDECIDED_S

  def stream(v_car, headway, first_x, seconds):
    """Cars coming down the oncoming lane at v_car, headway s apart, the first first_x m out: the far crop finds every one
    in the band at any range, the full pass those within its range."""
    j = Junction(v_limit=V_45)
    for k in range(round(seconds / DT_MDL)):
      t = k * DT_MDL
      xs = [first_x + n * headway * v_car - v_car * t for n in range(40)]
      xs = [x for x in xs if HOLD_D - 5. < x < 450.]
      args = {'raw_new': k % 2 == 0, 'far_range_m': 300., 'far_live': True}
      if k % 2 == 0:
        args['boxes'] = [raw(x, 3.6) for x in xs if x < RANGE]
      else:
        args['far'] = [raw(x, 3.6) for x in xs if x >= RANGE]
      d = j.step(**args)
      if d.go:
        break
    return j
  # a 45 mph stream at an 8 s headway (no gap of T_req): waited through, never the far wait's line (traffic reaches the full
  # pass's range); no gap in STAND_UNDECIDED_S, said (owner decision (ii)'s bound) and waited through (2026-11-07: no handback)
  j = stream(V_45, 8., 400., 3. * FAR_BLOCKED_UNDECIDED_S)
  assert not any(x.state == 'undecided' or x.reason == LONG_WAIT_REASON['farBlocked'] for x in j.decisions)
  assert all(x.state == 'waitGap' for x in j.decisions) and not any(x.go for x in j.decisions)
  # one slow car (8 m/s) from 300 m: in the full pass's range within FAR_BLOCKED_UNDECIDED_S, then past: no fault, and a go
  # once it has passed, inside STAND_UNDECIDED_S (from 400 m, 56 s: noGap)
  j = stream(8., 1000., 300., (300. + 20.) / 8. + CLEAR_WINDOW_S + 1.)
  assert not any(x.state == 'undecided' for x in j.decisions) and j.decisions[-1].go
  # another hold keeps the car where it is (a red: LIGHT_RED_MAX_S): the far car isn't what holds it, never a fault under
  # it; at the green the wait on the far car alone starts from there
  j = Junction(v_limit=V_45)
  d = far_blocked_wait(j, 2. * FAR_BLOCKED_UNDECIDED_S, 280., 5.4, light='red')
  assert d.state == 'waitGap' and d.reason == "Waiting for the light" and not any(x.state == 'undecided' for x in j.decisions)
  n = len(j.decisions)
  d = far_blocked_wait(j, FAR_BLOCKED_UNDECIDED_S + 5., 280., 5.4, light='green')
  said = [k for k, x in enumerate(j.decisions[n:]) if x.reason == LONG_WAIT_REASON['farBlocked']]
  assert d.state == 'waitGap' and said and said[0] * DT_MDL >= FAR_BLOCKED_UNDECIDED_S - 0.1
  # the far crop not needed (25 mph): a far box blocks only within R_req, as the full pass's do; beyond it, a go
  j = Junction()
  d = far_blocked_wait(j, CLEAR_WINDOW_S + 1., 280., 5.4)
  assert d.go


def roll(j, seconds, v, enabled=True, boxes=lambda: [raw(100., 3.6)]):
  """The car at v m/s (a number, or a function of the step) for seconds, its junction coming nearer by it; enabled: openpilot
  engaged (False: the driver's pedals). A car stays in the band: no fresh go meanwhile."""
  out = []
  for k in range(round(seconds / DT_MDL)):
    vk = v(k) if callable(v) else v
    out.append(j.step(boxes(), v=vk, distance=j.distance - vk * DT_MDL, enabled=enabled))
  return out


def withdrawn(j, v):
  """The go certified, then withdrawn at its first motion at v by a car in the band: the left's own hold (hold_here)."""
  go(j)
  d = j.step([raw(110., 3.6)], v=v, distance=j.distance - v * DT_MDL)
  assert d.state == 'waitGap' and d.stops and j.left.hold_here and not d.crossed
  return d


def car_seen(x, y, heading_deg):
  """raw() of a vehicle at road-frame (x along the approach, y toward the turn) from a car yawed heading_deg toward the turn."""
  a = math.radians(heading_deg)
  return raw(x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a))


@pytest.mark.parametrize('heading_deg', [20., 35., 50.])
def test_a_stand_yawed_into_the_turn_is_judged_at_its_heading_never_handed_back(log_events, heading_deg):
  """Review 2026-11-08 (L3, round 2): standing on its own hold after a withdrawn go yawed past 15 deg (YAW_UNDECIDED_DEG)
  was the 'yaw' handback (leftUndecidedTakeControl, the car held). A go withdrawn short of the oncoming lanes stands yawed
  into the turn (the Sim 25-29 deg; this car's own lefts from a stand 0-39 deg 4 m from first motion). Now judged at its
  measured heading: a car in the band seen from there holds it, and on an empty band it goes. Past YAW_VIEW_DEG (the band's
  far edge out of the road camera's frame) it is judged on what the camera frames, said once with a chime, its line saying
  so; never a handback."""
  from openpilot.selfdrive.turnpilot.unprotected_left import ROAD_HALF_FOV_DEG, YAW_VIEW_DEG, yaw_view_m
  j = Junction()
  withdrawn(j, 1.)
  roll(j, 1.5, lambda k: max(1. - 0.67 * k * DT_MDL, 0.))
  seen = [j.step([car_seen(100., 3.6, heading_deg)], v=0., heading_deg=heading_deg) for _ in range(round((CLEAR_WINDOW_S + 1.) / DT_MDL))]
  assert not any(d.go or d.state == 'undecided' or d.prompt in TAKE_CONTROL_PROMPTS for d in seen)
  assert all(d.blockers for d in seen[::2])  # the car seen from the car's heading is in the band
  ds = [j.step(v=0., heading_deg=heading_deg) for _ in range(round((CLEAR_WINDOW_S + 1.) / DT_MDL))]
  assert any(d.go for d in ds) and not any(d.state == 'undecided' or d.prompt in TAKE_CONTROL_PROMPTS for d in seen + ds)
  views = [kw for name, kw in log_events if name == 'turnpilot.left_yaw_view']
  past = heading_deg > YAW_VIEW_DEG
  assert len(views) == (1 if past else 0), views
  if past:
    assert any(d.chime for d in seen) and any(d.reason == LONG_WAIT_REASON['yaw'] for d in ds)
    assert abs(views[0]['view_m'] - min(yaw_view_m(heading_deg), 999.)) < 0.1 and views[0]['hold']
  assert not any(name == 'turnpilot.left_undecided' for name, _ in log_events)
  assert 30. < ROAD_HALF_FOV_DEG < 31. and abs(YAW_VIEW_DEG - (ROAD_HALF_FOV_DEG - ul.BAND_TOL_DEG)) < 1e-9
  assert yaw_view_m(30.) == math.inf and 50. < yaw_view_m(32.) < 60. and yaw_view_m(40.) < 10.


def test_a_fault_on_the_lefts_own_hold_says_the_faults_words_on_its_stop():
  """A fault raised on the left's own hold after a withdrawn go ('stale': no road run since) held the car with the
  withdrawal's words on its stop ("Oncoming, holding"), and turnpilotd's line is the stop's reason first: under
  leftUndecidedTakeControl the driver read the withdrawal (the 'yaw' handback read 'Car started too slowly, holding' until
  2026-11-08). The stop says the fault's words."""
  from openpilot.selfdrive.turnpilot.unprotected_left import UNDECIDED_REASON, WITHDRAWN_REASON
  j = Junction()
  withdrawn(j, 1.)
  roll(j, 1.5, lambda k: max(1. - 0.67 * k * DT_MDL, 0.))
  assert j.decisions[-1].stops[0].reason == WITHDRAWN_REASON['band']
  d = j.wait(STALE_UNDECIDED_S + 0.5, raw_new=False, v=0.)
  assert d.state == 'undecided' and d.why == 'stale' and d.prompt == 'leftUndecidedTakeControl'
  assert d.reason == UNDECIDED_REASON['stale'] and d.stops and all(s.reason == UNDECIDED_REASON['stale'] for s in d.stops)


def test_the_lefts_own_hold_is_let_go_only_past_it_by_the_drivers_own_metres():
  """Skeptic SIMFIX round 2: the left's own hold after a withdrawn go (hold_here) had no release rule: whatever the driver did,
  a re-engage stood the car where its comfortable curve stopped it, in the oncoming lanes too. Owner decision (a), as the
  latch's: let go as the driver's only past where TurnPilot stops it by the driver's own movement (disengaged), beyond
  LATCH_PAST_M; however far TurnPilot's own stop carries the car (engaged) is TurnPilot's, never the driver's; short of it
  the hold stands (c). Let go, it is said once (crossed: map_controls raises the crossing, Driven), and the left neither holds
  nor goes again: undecided ('driven'), its prompt the crossing's."""
  from openpilot.selfdrive.turnpilot.map_controls import CROSSED_CAUSES, LATCH_PAST_M
  j = Junction()
  withdrawn(j, 1.)
  # TurnPilot's stop from 1 m/s carries the car 1 m (its curve asked 0.33): TurnPilot's own metres
  ds = roll(j, 1.5, lambda k: max(1. - 0.67 * k * DT_MDL, 0.)) + roll(j, 2., 0.)
  assert all(d.stops and not d.crossed for d in ds) and j.left.hold_here and ds[-1].state == 'waitGap'
  stood = j.distance
  # the driver moves it on 1.5 m (disengaged) and stops: under the reach, still the left's, held on a re-engage
  ds = roll(j, 1.5 / 0.6, 0.6, enabled=False) + roll(j, 1., 0., enabled=False) + roll(j, 2., 0.)
  assert not any(d.crossed for d in ds) and j.left.hold_here and ds[-1].stops and ds[-1].state == 'waitGap'
  # on: past it by more than the reach by the driver's own metres, theirs: said once, never held again, never a go
  ds = roll(j, 1.5, 0.6, enabled=False)
  crossed = [k for k, d in enumerate(ds) if d.crossed]
  assert len(crossed) == 1 and ds[crossed[0]].crossed in CROSSED_CAUSES and not j.left.hold_here
  moved = stood - (j.distance + 0.6 * DT_MDL * (len(ds) - 1 - crossed[0]))
  assert LATCH_PAST_M < moved < LATCH_PAST_M + 0.1, moved
  ds += roll(j, 3., 1.) + roll(j, 3., 1., boxes=list)
  after = ds[crossed[0] + 1:]
  assert not any(d.stops or d.crossed or d.go for d in after)
  assert all(d.state == 'undecided' and d.why == 'driven' and d.prompt == 'none' for d in after)


def test_the_lefts_own_hold_short_of_the_oncoming_lanes_is_never_held_in_them_on_a_re_engage():
  """The invariant: never a TurnPilot hold that stands the car in the oncoming lanes on a re-engage. A go from a creep
  (ONCOMING_ENTRY_M + CREEP_SHORT_M out) withdrawn at first motion holds the car 1.3 m short of the lanes, under LATCH_PAST_M.
  The driver's nudge on (disengaged) into the lanes is theirs as the car enters them; re-engaged short of them too fast for
  the hold to stand it short (from the re-engage's speed as its own braking takes it: hold_caps, map_controls.reengage_stop_m),
  theirs at the re-engage, never held inside; braked to a stand short of them first, held there (c). Standing, a reading that
  jumps nearer is no movement of the driver's (position-free): the position fault, held, never let go."""
  from openpilot.selfdrive.turnpilot.map_controls import LATCH_PAST_M, reengage_stop_m
  from openpilot.selfdrive.turnpilot.unprotected_left import hold_caps

  def nudged(nudge_m):
    j = Junction()
    j.distance = ONCOMING_ENTRY_M + CREEP_SHORT_M + 0.6
    withdrawn(j, 0.5)
    roll(j, 1., lambda k: max(0.5 - k * DT_MDL, 0.))
    roll(j, 1., 0.)
    assert j.left.hold_here and j.distance - ONCOMING_ENTRY_M > 1.2
    return j, roll(j, nudge_m / 0.8, 0.8, enabled=False)
  # on into the lanes, disengaged: theirs as the car enters them, short of the reach
  j, ds = nudged(1.6)
  crossed = [k for k, d in enumerate(ds) if d.crossed]
  assert len(crossed) == 1 and not j.left.hold_here
  at = j.distance + 0.8 * DT_MDL * (len(ds) - 1 - crossed[0])
  assert ONCOMING_ENTRY_M - 0.8 * DT_MDL <= at < ONCOMING_ENTRY_M and j.left.hold_drv_past < LATCH_PAST_M - 0.5, at
  # 0.8 m on and re-engaged rolling, short of the lanes but too fast for the hold to stand it short: theirs at the re-engage
  j, ds = nudged(0.8)
  assert not any(d.crossed for d in ds) and j.distance > ONCOMING_ENTRY_M + 0.3
  room = j.distance - ONCOMING_ENTRY_M
  d = j.step([raw(100., 3.6)], v=0.8, distance=j.distance - 0.8 * DT_MDL)  # re-engaged
  assert reengage_stop_m(0.8, hold_caps) > room - 0.8 * DT_MDL
  assert d.crossed and not d.stops and not j.left.hold_here
  # the same nudge braked to a stand short of the lanes, then re-engaged: held there
  j, ds = nudged(0.8)
  ds += roll(j, 0.85, lambda k: max(0.8 - (k + 1) * DT_MDL, 0.), enabled=False)
  assert j.v == 0. and j.distance > ONCOMING_ENTRY_M
  ds += roll(j, 2., 0.)
  assert not any(d.crossed for d in ds) and j.left.hold_here and ds[-1].stops
  assert (ds[-1].state, ds[-1].why) == ('undecided', 'position')  # under HELD_MIN_M: the position handback, held
  # standing on the hold, a reading 4 m nearer: the position fault, held, never the driver's
  j = Junction()
  withdrawn(j, 0.5)
  roll(j, 1., lambda k: max(0.5 - k * DT_MDL, 0.))
  ds = [j.step([raw(100., 3.6)], v=0., distance=j.distance - 4.) for _ in range(40)]
  assert all(d.stops and not d.crossed for d in ds) and ds[-1].state == 'undecided' and ds[-1].why == 'position'


def test_every_wait_at_the_line_for_a_gap_is_said_at_45_s_and_waited_through_never_a_go_nor_a_handback(log_events):
  """Owner decision (ii), 2026-10-30, Kellen's delegate (DESIGN's UNDECIDED row: no gap in 45 s): a car parked in the band
  seen only as raw boxes, and a stream with no gap, each waited on silently with no limit (154bdaa bounded only what the
  tracker says stands). Until 2026-11-07 every wait at its line for a gap ended in the noGap handback at STAND_UNDECIDED_S
  ('standing' where the tracker says so). Kellen, 2026-11-07: "no, no handbacks". It is now said there, once a hold (the
  log; its line 'standing' where the tracker says so), and waited through, the car held, a car it sees holding it, never a
  go. Another hold keeps the car where it is (a red, no lamp lit): that is no wait for a gap, so the count pauses, and a
  go starts it over."""
  for boxes in (lambda t: [raw(50., 7.5)], lambda t: [raw(x, 3.6) for x in (120. - 11. * (t % 3.), 150. - 11. * (t % 3.))]):
    log_events.clear()
    j = Junction()
    for k in range(round((STAND_UNDECIDED_S + 120.) / DT_MDL)):
      d = j.step(boxes(k * DT_MDL))
    assert d.state == 'waitGap' and d.prompt == 'waitingForGap' and d.reason == "Waiting for a gap" and not d.stops
    assert not any(x.go or x.state == 'undecided' or x.prompt in TAKE_CONTROL_PROMPTS for x in j.decisions)
    said = [kw for name, kw in log_events if name == 'turnpilot.left_no_gap']
    assert len(said) == 1 and said[0]['why'] == 'noGap' and abs(said[0]['wait_s'] - STAND_UNDECIDED_S) < 0.1, said
  # under a red for 60 s, then 40 s on the green with the band blocked: no wait for a gap under the red, so nothing said
  log_events.clear()
  j = Junction()
  j.wait(60., boxes=[raw(50., 7.5)], light='red')
  d = j.wait(STAND_UNDECIDED_S - 5., boxes=[raw(50., 7.5)], light='green')
  assert d.state == 'waitGap' and not any(name == 'turnpilot.left_no_gap' for name, _ in log_events)
  d = j.wait(6., boxes=[raw(50., 7.5)], light='green')
  assert d.state == 'waitGap' and [name for name, _ in log_events if name == 'turnpilot.left_no_gap'] == ['turnpilot.left_no_gap']
  # a go after 40 s, withdrawn at its first motion: the wait on its own hold is counted afresh from there, and said afresh
  log_events.clear()
  j = Junction()
  j.wait(40., boxes=[raw(50., 7.5)])
  go(j)
  d = j.step([raw(110., 3.6)], v=0.5, distance=HOLD_D - 0.05)
  assert d.state == 'waitGap' and j.left.hold_here
  d = j.wait(STAND_UNDECIDED_S - 5., boxes=[raw(100., 3.6)], v=0.)
  assert d.state == 'waitGap' and d.stops and not any(name == 'turnpilot.left_no_gap' for name, _ in log_events)
  d = j.wait(60., boxes=[raw(100., 3.6)], v=0.)
  assert d.state == 'waitGap' and d.stops and d.stops[0].should_stop  # held, its own hold
  assert [kw['hold'] for name, kw in log_events if name == 'turnpilot.left_no_gap'] == [True]


def test_a_car_standing_in_the_way_is_said_for_what_it_is_and_waited_on(log_events):
  # the tracker says the band's car stands (a velocity-trusted track under STAND_SPEED matched to its box): past
  # STAND_UNDECIDED_S the line says so, in plain words (until 2026-11-07: 'standing', "Blocked by something standing: take
  # control"), the car held, and it goes once the car has gone
  parked = oncoming(50., 7.5, vx=0., trackId=3, box=[0.5, 0.4, 0.6, 0.5], camera='road')
  j = Junction()
  d = j.wait(STAND_UNDECIDED_S + 30., boxes=[raw(50., 7.5)], objects=[parked])
  assert d.state == 'waitGap' and d.reason == LONG_WAIT_REASON['standing'] == "Waiting: something stopped is in the way"
  assert not any(x.go or x.state == 'undecided' or x.prompt in TAKE_CONTROL_PROMPTS for x in j.decisions)
  assert [kw['why'] for name, kw in log_events if name == 'turnpilot.left_no_gap'] == ['standing']
  d = j.wait(CLEAR_WINDOW_S + 0.3)
  assert any(x.go for x in j.decisions[-round((CLEAR_WINDOW_S + 0.3) / DT_MDL):])


def under_the_hold(j, seconds):
  """openpilot engaged for seconds, the car braking as the left's decisions ask (their stops through plan_accel, as the sims
  apply them: shouldStop's -1 m/s^2 floor), a car in the band (no fresh go)."""
  from openpilot.selfdrive.turnpilot.hooks import plan_accel
  from openpilot.selfdrive.turnpilot.scene_policy import decide
  out, d = [], j.decisions[-1]
  for _ in range(round(seconds / DT_MDL)):
    a = 0.
    if d.stops:
      dec = decide(d.stops, j.v)
      a = plan_accel(dec.accel_cap, dec.speed_cap, j.v)
      a = min(a, -1.) if dec.should_stop else a
    v = max(j.v + a * DT_MDL, 0.)
    d = j.step([raw(100., 3.6)], v=v, distance=j.distance - v * DT_MDL)
    out.append(d)
  return out


def test_the_lefts_own_hold_never_counts_turnpilots_stop_past_its_ask_as_the_drivers_whatever_the_order():
  """Skeptic round 1 (tp/left-hold): past = min(hold_m - hold_asked, hold_drv_m) took the driver's metres inside TurnPilot's
  asked stop for metres past it once TurnPilot's own braking (engaged) had carried the car past the ask: the driver keeps the
  speed 0.6 m inside a 0.75 m ask, re-engages, the hold stands the car 1.1 m past the ask, and a 1.5 m nudge on (under the
  reach) let it go. Owner decision (a): the driver's metres are only those they move the car past where TurnPilot's stop put
  it; inside the ask they are TurnPilot's, and whatever TurnPilot's stop carries, engaged, is never theirs. Held for the
  full check (c), let go only past the reach by the driver's own metres beyond the ask."""
  from openpilot.selfdrive.turnpilot.map_controls import LATCH_PAST_M
  j = Junction()
  withdrawn(j, 1.5)
  asked = j.left.hold_asked
  assert abs(asked - 0.75) < 0.01
  ds = roll(j, 0.6 / 1.5, 1.5, enabled=False)  # the driver keeps the speed inside the ask, disengaged
  ds += under_the_hold(j, 4.)  # re-engaged rolling: TurnPilot's hold stands the car past its ask
  assert j.v == 0. and j.left.hold_m - asked > 1., j.left.hold_m
  stood = j.distance
  ds += roll(j, 1.5 / 0.5, 0.5, enabled=False) + roll(j, 1., 0., enabled=False) + under_the_hold(j, 2.)  # nudged on 1.5 m
  assert not any(d.crossed for d in ds) and j.left.hold_here and ds[-1].stops and ds[-1].state == 'waitGap'
  # on: past it by more than the reach of the driver's own metres beyond where TurnPilot's stop put it, theirs
  ds = roll(j, 1.5, 0.5, enabled=False)
  crossed = [k for k, d in enumerate(ds) if d.crossed]
  assert len(crossed) == 1 and not j.left.hold_here
  at = stood - (j.distance + 0.5 * DT_MDL * (len(ds) - 1 - crossed[0]))
  assert LATCH_PAST_M < at < LATCH_PAST_M + 0.1, at


def test_the_lefts_own_hold_is_let_go_for_the_lanes_at_the_re_engage_never_on_turnpilots_own_metres():
  """Skeptic round 1 (tp/left-hold): the go withdrawn later in its launch (2.38 m/s, 7.03 m from the junction: TurnPilot's
  stop asks 1.89 m), the driver keeps the speed disengaged inside the ask and re-engages: the lanes test was reached only once
  TurnPilot's own braking had carried the car past the ask, so the hold was let go a frame or two into the re-engage,
  engaged, on TurnPilot's metres (and so too where those metres would have stood the car short of the lanes). Judged at the
  re-engage itself, from its frame's speed and where the car is, before TurnPilot's hold has braked it (skeptic round 2):
  let go there where a hold could no longer stand the car short of the oncoming lanes (the invariant), its decision asking
  no stop; a re-engage short of that is held, never let go engaged later."""
  for keep_m, theirs in ((1.1, True), (1.7, True), (0.3, False)):
    j = Junction()
    j.distance = 7.6
    withdrawn(j, 2.38)
    entry = j.left.hold_entry
    assert j.left.hold_asked > 1.8 and abs(entry - 3.5) < 0.1
    ds = roll(j, keep_m / 2.38, 2.38, enabled=False)
    n = len(ds)
    ds += under_the_hold(j, 6.)
    crossed = [k for k, d in enumerate(ds) if d.crossed]
    if theirs:
      assert crossed == [n], (keep_m, crossed, n)  # the re-engage, the first engaged frame
      assert not any(d.stops for d in ds[n:]) and j.left.hold_m < j.left.hold_asked + 1.  # never braked by the hold
    else:
      assert not crossed and j.left.hold_here and j.v == 0. and j.left.hold_m < entry, (keep_m, j.left.hold_m, entry)
      assert ds[-1].state == 'waitGap' and ds[-1].stops


@pytest.mark.parametrize("then_v", [0., 0.6])
@pytest.mark.parametrize("keep_m", [1.3, 1.7])
def test_the_lefts_own_hold_is_judged_for_the_lanes_at_the_re_engage_so_a_car_braked_short_of_them_is_held(keep_m, then_v):
  """Skeptic round 2 (tp/left-hold): the lanes test (a hold taken up by a re-engage could no longer stand the car short of the
  oncoming lanes) was judged on the driver's own frames from that frame's speed, and its let-go was for good. The go withdrawn
  later in its launch (2.38 m/s, 7.6 m out), the driver keeps the speed 1.3-1.7 m disengaged (let go there), then brakes,
  short of the lanes, and re-engages, standing or at 0.6 m/s: no hold, and openpilot drove the car on across the lanes the
  go was withdrawn for, uncertified (e93750b held it). Owner decision (c), the invariant: judged at the re-engage, from that
  frame's speed and where the car is. A hold there stands the car short of the lanes: held for the full check (waitGap, its
  stop; the position handback, held, under HELD_MIN_M), never let go."""
  from openpilot.selfdrive.turnpilot.map_controls import reengage_stop_m
  from openpilot.selfdrive.turnpilot.unprotected_left import hold_caps
  j = Junction()
  j.distance = 7.6
  withdrawn(j, 2.38)
  entry = j.left.hold_entry
  ds = roll(j, keep_m / 2.38, 2.38, enabled=False)  # the driver keeps the speed, disengaged
  assert j.left.hold_m + reengage_stop_m(2.38, hold_caps) > entry  # re-engaged here, no hold could stand it short
  ds += roll(j, (2.38 - then_v) / 2.5 + DT_MDL, lambda k: max(2.38 - 2.5 * (k + 1) * DT_MDL, then_v), enabled=False)  # brakes
  assert abs(j.v - then_v) < 1e-6 and j.left.hold_m + reengage_stop_m(then_v, hold_caps) < entry - 0.05
  n = len(ds)
  ds += under_the_hold(j, 4.)  # re-engaged
  assert not any(d.crossed for d in ds), f"let go on frame {next(k for k, d in enumerate(ds) if d.crossed)} of {n} disengaged"
  assert j.left.hold_here and j.v == 0. and j.left.hold_m < entry
  # held, waiting for its gap, or standing under HELD_MIN_M its position handback, held
  assert all(d.stops and (d.state, d.why) in (('waitGap', ''), ('undecided', 'position')) for d in ds[n:])


@pytest.mark.parametrize("v", [1.5, 2.38])
def test_a_re_engage_the_lefts_own_hold_takes_up_is_never_let_go_on_a_later_frame(v):
  """The lanes test is the re-engage's alone (skeptic rounds 1, 2): the go withdrawn in its launch at v, the driver keeps the
  speed disengaged over a sweep of distances and re-engages rolling, the car keeping its speed REENGAGE_LAG_S (the plan's
  frame, the actuator's delay) before the hold brakes it. Let go at the re-engage itself or held: a hold taken up there
  stands the car short of the oncoming lanes, and the frames after it never let it go, however narrow the room it had
  (reengage_stop_m judged again on them takes the lag again, the car still at its speed, and lets go on TurnPilot's metres)."""
  from openpilot.selfdrive.turnpilot.map_controls import REENGAGE_LAG_S
  for k in range(int(round(1.5 / 0.05)) + 1):
    keep_m = 0.05 * k
    j = Junction()
    j.distance = 7.6
    withdrawn(j, v)
    entry = j.left.hold_entry
    ds = roll(j, keep_m / v, v, enabled=False)
    n = len(ds)
    ds += roll(j, REENGAGE_LAG_S, v) + under_the_hold(j, 6.)
    crossed = [i for i, d in enumerate(ds) if d.crossed]
    assert crossed in ([], [n]), (keep_m, crossed, n)
    if not crossed:
      assert j.left.hold_here and j.v == 0. and j.left.hold_m < entry and ds[-1].stops, (keep_m, j.left.hold_m, entry)


# ------------------------------------------------------------------- after dark (2026-11-06: NIGHT_LEFT_RANGE_M, darkness.py)

def test_after_dark_a_left_past_the_night_range_goes_on_the_band_it_sees():
  """The 11-06 night drive (1e): the full pass's raw vehicle boxes reached p99 50 m, max 78 m. Until 2026-11-07 the reach was
  capped there after dark and every such left handed back ('night'). Kellen, 2026-11-07 ("no, no handbacks ... we are pushing
  bleeding edge here, not maximal caution"; a safety driver supervises): no cap, judged on the band the camera covers, gone
  when it is clear; a car it sees there holds it."""
  from openpilot.selfdrive.turnpilot.unprotected_left import NIGHT_LEFT_RANGE_M
  for v_limit, lanes in ((V_25, 1), (V_25, 2), (8.94, 1), (None, None)):  # 25 mph, two lanes, 20 mph, unmapped (45 mph)
    j = Junction(v_limit=v_limit, lanes=lanes)
    d = j.step(dark=True)
    assert d.r_req > NIGHT_LEFT_RANGE_M and d.state != 'undecided', (v_limit, d.state, d.why)
    j.wait(CLEAR_WINDOW_S + 1., dark=True)
    assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions), v_limit
    j = Junction(v_limit=v_limit, lanes=lanes)
    j.wait(CLEAR_WINDOW_S + 1., dark=True, boxes=[raw(40., 3.6)])
    assert not any(x.go for x in j.decisions), v_limit
  # the far crop and its measured range as by day
  j = Junction(v_limit=20.1)
  far_wait(j, CLEAR_WINDOW_S + 1., far_range_m=300., far_live=True, dark=True)
  assert any(x.go for x in j.decisions)


def test_after_dark_a_left_held_short_of_its_line_rolls_up_to_it_then_judges_the_gap():
  # held short of its line (the latch reaches LATCH_NEAR_M short), dark: rolled up to the line as by day for a reach past the
  # range, then judged there on the band it sees (2026-11-07: no night handback), never a go from where it stood
  from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, ROLL_UP_M
  j = Junction()
  d = j.step(distance=16., dark=True)
  assert d.state == 'creep' and abs(d.creep_m - (16. - LINE_D)) < 1e-6 and d.reason == "Rolling up to the line"
  for dist in (14., 12., LINE_D + ROLL_UP_M + 0.1):  # rolled up as by day (no night cap: R_req within the range nearer)
    d = j.step(distance=dist, v=0.6, dark=True)
    assert d.state in ('creep', 'waitGap') and not d.go, dist
  d = j.step(distance=LINE_D + ROLL_UP_M - 0.1, v=0.3, dark=True)  # within ROLL_UP_M of its line: judged there
  assert d.state != 'undecided' and not d.go
  j.wait(CLEAR_WINDOW_S + 0.5, distance=LINE_D, v=0., dark=True)
  assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions)


def test_a_go_under_way_when_it_gets_dark_is_never_taken_back_for_the_night():
  # the go certified by day and moving: the reach isn't judged again in the go (only the band, the crosswalk, the view and the
  # holds are), so the filter turning dark mid-launch never stops the car in the junction
  j = Junction()
  j.wait(CLEAR_WINDOW_S + 0.2)
  assert j.decisions[-1].go or j.decisions[-1].state == 'go'
  d = j.step(v=0.8, distance=HOLD_D - 0.2, dark=True)
  assert d.state == 'go' and d.why == '' and not d.stops


def test_the_reach_handbacks_are_gone_their_words_kept():
  # 11-06: every reach handback read 'Far camera range not measured'; they said the car can't see oncoming far enough. Since
  # 2026-11-07 none is raised (no handbacks): the same lefts go on the band they see; the words stay for the logs' readers
  from openpilot.selfdrive.turnpilot.unprotected_left import UNDECIDED_REASON
  for why in ('range', 'farUnmeasured', 'farOff'):
    assert UNDECIDED_REASON[why] == "Can't see oncoming far enough: take control", why
  for kw in ({}, {'far_range_m': 300.}, {'dark': True}):
    j = Junction(v_limit=20.1)
    far_wait(j, CLEAR_WINDOW_S + 1., far_live='far_range_m' not in kw, **kw)
    assert any(x.go for x in j.decisions) and not any(x.state == 'undecided' for x in j.decisions), kw


def test_route_range_refuses_no_left_and_still_says_its_reach_and_bend():
  """2026-11-06 (Kellen: never route around or skip a left for range): route_range was the predicate turnpilotd skipped lefts
  by (unprotectedRange) and navd routed around, and it refused every left of the 11-06 drives 58-394 m out. It refuses none
  now, in any mode, whatever the range; its reach (R_req where the car may stand) and bent (the opposing road not known
  straight out to it) are as before, for the logs."""
  from openpilot.selfdrive.turnpilot.route_follower import TURN, Maneuver
  from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, ROUTE_REACH_MARGIN_M, reach_at_line, route_range

  def left(**kw):
    return Maneuver(**{'kind': TURN, 'side': LEFT, 'type': 'turn', 'modifier': 'left', 'angle_deg': 90., 'geometry': True,
                       'opposing': 'present', 'opposing_speed': V_25, 'opposing_lanes': 1, 'opposing_straight_m': 400.} | kw)
  for maneuver in (left(), left(opposing_speed=0., opposing_lanes=0, opposing_straight_m=0.), left(opposing='unknown'),
                   left(opposing_straight_m=60.)):
    for reach_m in (0., 150., 210., 400.):
      for setback in (None, 0., 11.):
        rc = route_range(maneuver, setback, reach_m)
        margin = ROUTE_REACH_MARGIN_M if setback is None else setback
        reach = reach_at_line(maneuver.opposing_speed, maneuver.opposing_lanes or None, margin)
        assert rc.unprotected and not rc.out_of_range, (maneuver, reach_m, setback)
        assert rc.reach == pytest.approx(reach) and rc.margin == margin
        assert rc.bent == (reach > LINE_D + margin + maneuver.opposing_straight_m)
  # the 11-06 lefts' route-time numbers (scratchpad reach.py): 1e L1 267.8 m and L3 198.4 m bent at GO_LATENCY_S 1 s, 282.0
  # and 208.5 at this car's measured launch (L4), 1e L4/L5 (all unknown) bent
  assert route_range(left(opposing_speed=15.6, opposing_straight_m=400.), None, 210.).reach == pytest.approx(282.0, abs=0.5)
  rc = route_range(left(opposing_speed=11.1, opposing_straight_m=125.), None, 210.)
  assert rc.reach == pytest.approx(208.5, abs=0.5) and rc.bent and not rc.out_of_range
  rc = route_range(left(opposing_speed=0., opposing_lanes=0, opposing_straight_m=0.), None, 210.)
  assert rc.bent and not rc.out_of_range


def launch_from_the_line(j, a, v_max=math.inf, boxes=lambda d: [], seconds=15.):
  """j's go, then the car launching evenly at a m/s^2 from its line up to v_max (a coast at the lift's speed: v_max 0.78),
  armed for its launch (turnpilotd's left_launch_armed), road runs every frame of boxes(distance): (the decisions, the left's
  events)."""
  go(j)
  v, out, stopping = 0., [], False
  for _ in range(round(seconds / DT_MDL)):
    stopping = stopping or bool(out and out[-1].stops)  # withdrawn: the car stops where its hold asks (STOP_DECEL)
    v = max(v - ul.STOP_DECEL * DT_MDL, 0.) if stopping else min(v + a * DT_MDL, v_max)
    d = j.step(boxes(j.distance), v=v, distance=j.distance - v * DT_MDL, launch_armed=j.left.state == 'go')
    out.append(d)
    if d.state == 'committed' and j.distance < -10.:
      break
  return out


@pytest.mark.parametrize('a,v_max,carried', [(0.5, 0.78, True), (0.3, math.inf, True), (0.5, math.inf, True),
                                             (1.08, math.inf, False), (1.3, math.inf, False), (1.5, math.inf, False),
                                             (1.9, math.inf, False)])
def test_a_slow_launch_judged_on_is_carried_only_where_it_commits_short_of_a_launch(log_events, a, v_max, carried):
  """Review 2026-11-08 (round 2's trade, 'G5's coast'): a slow launch judged on (L3: the band it needs clear) crossed the
  oncoming lanes at the car's own pace, a coast on the lift's 0.78 m/s for about 15 s. Committing short of LEFT_LAUNCH_SPEED
  after that judgement it is now carried (LeftDecision.carry: turnpilotd's left_carry, go_override's carry). This car's own
  launches never are: 1d's gentlest (1.08 m/s^2 equivalent, 2.0 m/s in 1.85 m) is judged on from LINE_D at 1.94 m/s and
  commits at about 2.5; the others (1.3-1.9) are never judged; the left's own A_GO never is."""
  j = Junction()
  ds = launch_from_the_line(j, a, v_max)
  slow = [kw for name, kw in log_events if name == 'turnpilot.left_not_launched']
  committed = [kw for name, kw in log_events if name == 'turnpilot.left_committed']
  assert len(committed) == 1 and committed[0]['carry'] == carried, committed
  assert (committed[0]['v'] < ul.LEFT_LAUNCH_SPEED) == carried
  assert bool(slow) == (a < 1.2) and all(kw['on'] for kw in slow), slow
  after = [d for d in ds if d.state == 'committed']
  assert after and all(d.carry == carried for d in after) and not any(d.carry for d in ds if d.state != 'committed')
  assert not any(d.stops for d in after) and not any(str(d.prompt) in TAKE_CONTROL_PROMPTS for d in ds)


def test_a_slow_launch_with_a_car_seen_in_its_band_is_withdrawn_and_never_carried(log_events):
  """L3's rail: a car seen in the band the slow launch needs (165 m up, its width's floor 137 m: outside the A_GO launch's
  band from its first motion on, inside the coast's out to the camera's 150 m) withdraws the go to the left's own wait while
  the car can still stop short of the oncoming lanes. Nothing is carried; it is never committed."""
  j = Junction()
  ds = launch_from_the_line(j, 0.5, 0.78, boxes=lambda d: [raw(165., 3.6)] if d < HOLD_D - 0.5 else [], seconds=8.)  # once moving
  slow = [kw for name, kw in log_events if name == 'turnpilot.left_not_launched']
  assert len(slow) == 1 and not slow[0]['on'] and slow[0]['seen'], slow
  assert not any(d.state == 'committed' or d.carry for d in ds)
  assert any(d.state == 'waitGap' and d.stops for d in ds) and j.left.hold_here


def ranged_box(x, y, box):
  b = raw(x, y)
  return RawBox(b.camera, b.cls, b.conf, b.bearing_deg, b.span_deg, b.xy, b.near_m, box)


def test_a_car_parked_past_the_junction_hides_no_lane_of_a_one_lane_road():
  # route 29 (2026-11-08) seg 7: a car parked at the far curb 26 m past the maneuver point, 7.2 m to the turn side, standing
  # and trusted, was an occluder whose bearings LANE_FAR_M (9 m, two lanes) took for the oncoming lanes': view blocked 25 s
  # at an empty junction, the window never started. One opposing lane (1.8-5.4 m): it hides only the curb behind it
  parked = oncoming(HOLD_D + 26., 7.2, vx=0., trackId=69, box=[0.32, 0.55, 0.38, 0.63], camera='road')
  box = ranged_box(HOLD_D + 26., 7.2, (0.32, 0.55, 0.38, 0.63))
  j = Junction(lanes=1)
  j.wait(CLEAR_WINDOW_S + 0.5, boxes=[box], objects=[parked])
  assert not any(x.view_blocked for x in j.decisions) and any(x.go for x in j.decisions)
  # two opposing lanes (to 9 m): the same car stands in the outer lane, and hides it
  j = Junction(lanes=2)
  d = j.wait(CLEAR_WINDOW_S + 0.5, boxes=[box], objects=[parked])
  assert d.view_blocked and not any(x.go for x in j.decisions)
  # the opposing left-turner in its lane by the junction still blocks a one-lane road's view
  turner = oncoming(HOLD_D + 12., 3.6, vx=0., trackId=7, box=[0.5, 0.4, 0.6, 0.5], camera='road')
  j = Junction(lanes=1)
  d = j.wait(1., boxes=[ranged_box(HOLD_D + 12., 3.6, (0.5, 0.4, 0.6, 0.5))], objects=[turner])
  assert d.view_blocked and not d.go


def test_a_red_read_in_the_junctions_mouth_clears_the_junction_on_the_gap():
  # Kellen, 2026-11-08: a left waiting in a signal's junction mouth (past its stop line, SIGNAL_LEFT_HOLD_M short of the
  # maneuver point) when the light goes red or yellow clears the junction like a driver: on the gap, an oncoming car seen
  # holding it until it passes, no handback. Until then it held there through the red ('Light changed: holding in the
  # junction', LS-4 option b)
  mouth = ONCOMING_ENTRY_M + 2.
  j = Junction()
  j.distance, j.wide_every = mouth, 3  # the wide camera's runs: the cross road's view (CROSS_VIEW_FRESH_S)
  j.wait(2., line_m=mouth, light='green')  # a steady green read standing in the mouth (CAMERA_GREEN_STEADY_S)
  approaching = [raw(120., 3.6)]
  j.wait(2., boxes=approaching, line_m=mouth, light='yellow')
  d = j.wait(2., boxes=approaching, line_m=mouth, light='red')
  assert d.clearing and not any(x.go for x in j.decisions) and d.state == 'waitGap'  # the car seen holds it
  assert d.reason == ul.LIGHT_CHANGED_REASON
  ds = [j.step(line_m=mouth, light='red') for _ in range(round((CLEAR_WINDOW_S + 0.5) / DT_MDL))]
  go = next(i for i, x in enumerate(ds) if x.go)
  assert CLEAR_WINDOW_S - 0.1 < go * DT_MDL < CLEAR_WINDOW_S + 0.3
  assert not any(x.state == 'undecided' or x.prompt in TAKE_CONTROL_PROMPTS for x in j.decisions)
  # the model's hold for the red (left_lift_refused 'light', the frame before turnpilotd lifts it) restarts no window; the
  # go override off ('param': nothing lifts it) still holds it
  j = Junction()
  j.distance, j.wide_every = mouth, 3
  j.wait(2., line_m=mouth, light='green')
  ds = [j.step(line_m=mouth, light='red', model_holds='light') for _ in range(round((CLEAR_WINDOW_S + 0.5) / DT_MDL))]
  assert any(x.go for x in ds)
  j = Junction()
  j.distance, j.wide_every = mouth, 3
  j.wait(2., line_m=mouth, light='green')
  ds = [j.step(line_m=mouth, light='red', model_holds='param') for _ in range(round((CLEAR_WINDOW_S + 0.5) / DT_MDL))]
  assert not any(x.go for x in ds)


def test_a_red_read_rolling_into_the_mouth_holds_and_one_after_a_stand_there_clears_even_mid_creep():
  """The review of 2026-11-08: the clearing only for a car that stood past its line on a light not red before the spell. A red
  first read rolling into the mouth (never stood there) holds for the green; one read after a steady green there clears, the
  car creeping for a view when it came (LS-4: a creep into the near cross lanes); one read after CLEAR_STAND_S of an unread
  stand there clears, and a shorter stand doesn't."""
  mouth = ONCOMING_ENTRY_M + 2.
  j = Junction()
  j.distance, j.wide_every = mouth, 3
  d = j.step(line_m=mouth, light='red', v=2., signal_line_m=mouth + 3.)
  d = j.wait(2., line_m=mouth, light='red', v=0., signal_line_m=mouth + 3.)
  assert not d.clearing and d.reason == ul.LIGHT_WAIT_REASON['red'] and not any(x.go for x in j.decisions)
  j = Junction()
  j.distance, j.wide_every = mouth, 3
  j.wait(2., line_m=mouth, light='green', signal_line_m=mouth + 3.)
  d = j.step(line_m=mouth, light='yellow', v=0.8, signal_line_m=mouth + 3.)
  assert d.clearing
  for stand, clears in ((ul.CLEAR_STAND_S + 0.5, True), (ul.CLEAR_STAND_S - 1., False)):
    j = Junction()
    j.distance, j.wide_every = mouth, 3
    j.wait(stand, line_m=mouth, light='none', signal_line_m=mouth + 3.)
    d = j.step(line_m=mouth, light='red', signal_line_m=mouth + 3.)
    assert d.clearing == clears, stand


def test_a_red_read_with_the_car_at_its_line_still_holds_it_there():
  # the hard rail: a red read before the car passed its stop line (held at the line, LINE_D) keeps it there, never cleared
  j = Junction()
  j.wait(1., light='green')
  d = j.wait(10., light='red')
  assert not d.clearing and not any(x.go for x in j.decisions) and d.reason == ul.LIGHT_WAIT_REASON['red']
  # read red while still short of the mouth's stop line (rolling up at a mapped signal): no clearing for that spell either
  j = Junction()
  j.distance = HOLD_D + 3.
  j.step(line_m=ONCOMING_ENTRY_M + 2., light='red')
  d = j.wait(10., distance=ONCOMING_ENTRY_M + 2., line_m=ONCOMING_ENTRY_M + 2., light='red')
  assert not d.clearing and not any(x.go for x in j.decisions)
