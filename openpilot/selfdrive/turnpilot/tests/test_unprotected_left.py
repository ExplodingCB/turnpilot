"""The autonomous unprotected left (unprotected_left.py): its gap rule on raw detections, the clear window, the occluder and
the view, the exit crosswalk, creeping, the go, its re-certification at first motion, the commit, and its faults."""
import math

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.route_follower import LEFT
from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M, VEL_TRUST_HITS
from openpilot.selfdrive.turnpilot.tests.synthetic import scene
from openpilot.selfdrive.turnpilot.unprotected_left import (BLOCKED_UNDECIDED_S, CLEAR_WINDOW_S, CREEP_MAX_M, CREEP_SHORT_M, PET_S, RAW_CONF,
                                                           ROAD_FRESH_S, SPEED_MARGIN, STALE_UNDECIDED_S, LeftInput, RawBox, RawFrame,
                                                           UnprotectedLeft, band_blockers, r_req, t_clear, t_req)

HOLD_D = 9.     # m from the hold line (map_controls: HOLD_LINE_M + MAP_MARGIN_M) to the maneuver point
V_25 = 11.18    # m/s, 25 mph
RANGE = 150.


def raw(x, y, cls='car', conf=0.6, camera='road', near=None, ranged=True, width=1.8):
  """A raw detection of a vehicle whose ground point is (x, y) in the car's frame; near: its nearest plausible range."""
  bearing = math.degrees(math.atan2(y, x))
  half = math.degrees(math.atan2(width / 2., math.hypot(x, y)))
  return RawBox(camera, cls, conf, bearing, (bearing - half, bearing + half), (x, y) if ranged else None,
                (near if near is not None else math.hypot(x, y)) if ranged else None, (0., 0., 0., 0.))


def frame(boxes=(), camera='road', t=0.):
  return RawFrame(list(boxes), camera, t)


def oncoming(x, y=3.6, vx=-11., age=VEL_TRUST_HITS, **fields):
  return {'cls': 'car', 'x': x, 'y': y, 'vx': vx, 'vy': 0., 'inPath': False, 'age': age, **fields}


def ped(x, y, **fields):
  return {'cls': 'pedestrian', 'x': x, 'y': y, 'vx': 0., 'vy': 0., 'inPath': False, 'age': 20, 'conf': 0.8, **fields}


def test_t_req_and_r_req():
  # the left's post-go profile (1.5 m/s^2 to 4.3 m/s) over the path from the hold line through the oncoming lanes, a second
  # from go to moving (LEFT_NO_MOVE_S withdraws a go not moving by then) and a 2 s post-encroachment margin
  assert 5.0 < t_clear(HOLD_D + 8.) < 5.8
  t1, t2 = t_req(HOLD_D, 1), t_req(HOLD_D, 2)
  assert 8.0 < t1 < 9.0 and abs(t2 - t1 - 0.5) < 1e-6
  assert abs(t_req(HOLD_D, None) - t2) < 1e-6  # unknown opposing lanes: two
  assert abs(t_req(HOLD_D, 1, latency=False) - (t1 - 1.)) < 1e-6
  # the range to see clear is the mapped limit's 1.3 x over it, from the conflict point
  assert SPEED_MARGIN >= 1.3 and PET_S >= 2.
  r = r_req(V_25, t1, HOLD_D)
  assert abs(r - (V_25 * SPEED_MARGIN * t1 + HOLD_D + 3.)) < 1e-6 and 125. < r < 145.
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


class Junction:
  """A held unprotected left: the car at the hold line, frames of raw road detections, scene tracks."""
  def __init__(self, v_limit=V_25, lanes=1, mode='auto', rng=RANGE):
    self.left = UnprotectedLeft()
    self.mode, self.v_limit, self.lanes, self.range = mode, v_limit, lanes, rng
    self.distance, self.v, self.t = HOLD_D, 0., 0.
    self.held = True
    self.decisions = []

  def step(self, boxes=(), objects=(), road=True, raw_new=True, v=None, distance=None, **kw):
    self.t += DT_MDL
    if v is not None:
      self.v = v
    if distance is not None:
      self.distance = distance
    fr = frame(boxes, 'road' if road else 'wide', self.t) if raw_new else None
    inp = LeftInput(mode=self.mode, maneuver_id=1, side=LEFT, distance=self.distance, v_ego=self.v, held=self.held,
                    raw=fr, objects=scene(objects).objects, heading_deg=0., v_limit=self.v_limit, opposing_lanes=self.lanes,
                    range_m=self.range, **kw)
    d = self.left.update(inp, DT_MDL)
    self.decisions.append(d)
    return d

  def wait(self, seconds, **kw):
    d = None
    for _ in range(round(seconds / DT_MDL)):
      d = self.step(**kw)
    return d


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
  # a gap of the rule's own need goes between two cars, leaving the next one at least T_req + PET out
  need = (r_req(V_25, t_req(HOLD_D, 1), HOLD_D) - HOLD_D) / V_25 + CLEAR_WINDOW_S + 1.
  went, arrive, conflict = stream(need, n=2)
  assert arrive[0] < went < arrive[1]
  assert conflict[1] - went >= SPEED_MARGIN * t_req(HOLD_D, 1) - 0.1  # the next car, at the limit, at the conflict point


def test_beyond_the_proven_range_never_goes():
  # 45 mph oncoming: its reach is past the road camera's proven range: never a go on a gap it can't see, said loudly
  j = Junction(v_limit=20.1)
  d = j.wait(CLEAR_WINDOW_S + 1.)
  assert not any(x.go for x in j.decisions) and d.state == 'undecided' and d.prompt == 'leftUndecidedTakeControl'
  # unknown speed is taken as 45 mph
  j = Junction(v_limit=None)
  assert j.wait(1.).state == 'undecided'
  # a range measured further (TurnPilotLeftRangeM) takes it
  j = Junction(v_limit=20.1, rng=300.)
  assert any(x.go for x in [j.wait(CLEAR_WINDOW_S + 0.5)] + j.decisions)


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
  # the view still blocked BLOCKED_UNDECIDED_S after the full creep: can't judge, said and handed back
  j = Junction()
  j.wait(0.5, boxes=[box], objects=[turner])
  j.wait(BLOCKED_UNDECIDED_S - 0.5, boxes=[box], objects=[turner], v=0., distance=HOLD_D - CREEP_MAX_M)
  d = j.wait(1., boxes=[box], objects=[turner], v=0., distance=HOLD_D - CREEP_MAX_M)
  assert d.state == 'undecided' and d.prompt == 'leftUndecidedTakeControl' and not any(x.go for x in j.decisions)
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
  # a roll-up the car never makes (the model holding it): undecided on the reach after ROLL_UP_STALL_S, said
  j = Junction()
  d = j.wait(ROLL_UP_STALL_S + 0.5, distance=16., v=0.)
  assert d.state == 'undecided' and d.why == 'range'


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
