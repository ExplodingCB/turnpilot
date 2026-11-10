"""The stop-sign decider under TurnPilotStopSignGo=vision when it can't see as far as it would like (test_stop_go_vision's World
and closed-loop Drive: the shipped visibility file, no surveys, the map's tiers, the side views running), by day (15:00 EDT)
and at night (23:00 EDT).

Kellen, 2026-11-07: test builds are "bleeding edge, not maximal caution"; "no, no handbacks": where TurnPilot can't judge
something it acts on its best judgement of what it can see, a safety driver supervising; stop signs a vision-based go like
early Autopilot, imperfect OK, not aggressive. Until then a stop whose walk fell short of a lane's vision floor waited, tried a
creep only where the walk predicted one would help, and handed back 15 s after its full stop ("Stop sign: can't see far
enough from the left (...): take control", H3), by day and at night. The commonest cause was a far side-view sector reading
'degraded' (a contrast ratio under 0.5 against the 10-35 deg band, which ordinary sunny footage reads at 70-85 deg: wf7
verify:LS-3-R1 point 4). Now:

- a degraded sector is judged on the boxes and tracks it still yields, as a dark or blocked one is (low texture is what an
  empty far road looks like; the thresholds were measured on moving frames, sideview's docstring);
- a lane still short of its floor is crept for (Creeping for view, VIEW_CREEP_M at a time at VIEW_CREEP_V, never closer to the
  cross road's near edge than a go needs to stop short of it), judged again from each stand, and once the creep has reached
  its limit the car goes on what it sees: the long look's window ("Look both ways"), every car or box seen in a lane still
  holding the go, and the launch's re-check still withdrawing it for one;
- a washed-out or frozen sector (the camera can't see there at all) never rides that go: it waits, said, with no bound and no
  handback, and goes once the sector reads again; under glare too, where it lies inside the sun's zone short of a lane's floor;
- a side view whose every sector reads frozen is its feed stuck: the camera fault, said, then H1 (the review of 2026-11-08);
- the creep's steps leave a step's run-on to spare, so its last stand's go can still be withdrawn with an actuator slower than
  the decider knows."""
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_go as sg
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, lanes_junction, two_way_junction, utc, vision_junction
from openpilot.selfdrive.turnpilot.tests import test_stop_go as tsg
from openpilot.selfdrive.turnpilot.tests.test_stop_go import arrived_at, first_go, lane_box
from openpilot.selfdrive.turnpilot.tests.test_stop_go_vision import forward_cars, standing, vcars, vdrive, vworld, watch

NIGHT = ETA + 8 * 3600.  # 23:00 EDT, the sun -34 deg
WHEN = {'day': ETA, 'night': NIGHT}
TAKE_CONTROL = 'stopSignTakeControl'


def far_sectors(quality, from_deg):
  """A side view's sectorQuality: ok from its inner edge, quality from from_deg out past the frame edge."""
  def fn(view):
    sign = 1. if view == 'sideLeft' else -1.
    return [(sign * lo, sign * (lo + 5.), quality if lo >= from_deg - 1e-9 else 'ok') for lo in np.arange(sr.INNER_BAND_HI_DEG, 90., 5.)]
  return fn


def no_handback(out):
  return not any(d.fault or d.prompt == TAKE_CONTROL or d.state == 'ladder' for d in out)


def crossed(out):
  return any(d.state in ('inLane', 'done') for d in out)


def creeps(out):
  return [k for k, d in enumerate(out) if d.go_cause == 'creep']


def look_said(out, i):
  """The go at frame i came after its "Look both ways" line (the long look's window), said VISION_ANNOUNCE_S before it."""
  said = [k for k in range(i) if out[k].prompt == 'stopGo' and out[k].state == 'observe']
  return bool(said) and all(out[k].reason == sg.LOOK_REASON for k in said[-round(sr.VISION_ANNOUNCE_S / DT_MDL) + 1:])


# ------------------------------------------------------------------------------------------- degraded: judged on its boxes

@pytest.mark.parametrize('from_deg', [82.5, 72.5, sr.INNER_BAND_HI_DEG])
@pytest.mark.parametrize('when', list(WHEN))
def test_degraded_far_sectors_and_an_empty_cross_road_go_after_the_window_never_handed_back(tmp_path, monkeypatch, when, from_deg):
  # wf7 verify:LS-3-R1 point 4: the far sectors 'degraded' (82.5 deg out: the left lane 37.0 of 37.6 m) waited "can't see far
  # enough from the left (side camera unclear)" from 1 s and handed back H3 at 16 s, by day and at night, at an empty junction
  monkeypatch.setattr(tsg, 'sectors', far_sectors('degraded', from_deg))
  w = vdrive(tmp_path, wall_t=WHEN[when])
  out = w.run(16.)
  a, i = arrived_at(out), first_go(out)
  assert i is not None, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][a::40]
  assert (i - a) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11) and out[i].reason == sg.GOING_REASON
  assert all(x.r_seen >= x.r_req for x in out[i - 1].lanes) and not any(x.binding == 'sector' for x in out[i - 1].lanes)
  assert no_handback(out) and crossed(out)


@pytest.mark.parametrize('when', list(WHEN))
def test_degraded_far_sectors_a_crossing_car_holds_the_go_until_it_has_passed_then_the_car_goes(tmp_path, monkeypatch, when):
  # the boxes and tracks a degraded sector still yields judge the lane: a car coming down the left lane at 8 m/s from 45 m,
  # 2 s in, holds the go while it comes; the go comes VISION_CLEAR_S after it has gone, never a handback
  monkeypatch.setattr(tsg, 'sectors', far_sectors('degraded', 72.5))
  w = vdrive(tmp_path, wall_t=WHEN[when])
  w.tracks = vcars((41, 'left', lambda t: None if t < 2. else 45. - 8. * (t - 2.)))
  out = w.run(24.)
  a, i = arrived_at(out), first_go(out)
  gone_t = 2. + 57. / 8.  # s of camera time from the start: 12 m past our path, out of the tracks
  assert i is not None and (i + 1) * DT_MDL >= gone_t + sr.VISION_CLEAR_S - 0.11
  assert any(h.key == 'tracked:41' for d in out[a:i] for h in d.holds)
  assert no_handback(out) and crossed(out)


# --------------------------------------------------------------------- short of reach: creep to the limit, go on what it sees

def corner_car(tid=3):
  """A car standing at the corner 10 m down a two-way's through road, at its lane band's near edge: a bearing-only occluder
  of the lane behind it (no shield: that approach doesn't stop), never moving."""
  return vcars((tid, 'left', standing(10.), 2, 0.05))


@pytest.mark.parametrize('when', list(WHEN))
def test_a_lane_hidden_behind_a_corner_car_is_crept_for_to_the_creep_limit_and_then_the_car_goes_on_what_it_sees(tmp_path, when):
  # before: the walk predicted no step would help a bearing-only occluder, so there was no creep, and H3 15 s after the stop
  # ("can't see far enough from the left (view blocked)"). Now the car creeps for a view in VIEW_CREEP_M steps at
  # VIEW_CREEP_V, judged again from each stand, never closer to the closest plausible edge than a go needs, and at the limit
  # goes on what it sees: the long look's window and its "Look both ways" line, then across, never a handback
  w = vdrive(tmp_path, j=two_way_junction(20), wall_t=WHEN[when])
  w.tracks = corner_car()
  out = w.run(45.)
  a, i = arrived_at(out), first_go(out)
  c = creeps(out)
  assert c and i is not None and c[0] < i, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][a::40]
  assert all(out[k].reason == sg.VIEW_CREEP_REASON and out[k].action == 'creeping' for k in c)
  assert all(d.creep is None or d.creep[1] <= sg.VIEW_CREEP_V + 1e-9 for d in out)
  # crept: further than one step, and never inside the room a go needs to stop short of the closest plausible edge
  assert out[i].position_m - out[a].position_m > sg.VIEW_CREEP_M
  assert w.sg.edges.d_min - out[i].position_m >= sr.go_room_vision_m(w.kw['t_abort']) - 1e-6
  # the view still short at the go (the corner car still hides the lane behind it): the go on what it sees, said first
  assert any(x.role == 'left' and x.r_seen < x.r_req for x in out[i - 1].lanes)
  assert look_said(out, i) and out[i].reason == sg.GOING_REASON and out[i].log['viewLimited']
  assert no_handback(out) and crossed(out)
  assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)


@pytest.mark.parametrize('when', list(WHEN))
def test_on_a_limited_view_a_crossing_car_seen_holds_the_go_until_it_has_passed_then_the_car_goes(tmp_path, when):
  # the same corner, and once the creep has reached its limit a car comes down the right lane (seen: a track) at 8 m/s from
  # 45 m: the go waits for it, the window runs again once it has gone, and the car goes; never a handback
  w = vdrive(tmp_path, j=two_way_junction(20), wall_t=WHEN[when], behaviour='goes')
  parked = corner_car()
  start = {}

  def tracks(ww):
    out = list(parked(ww))
    if ww.sg.view_limited and 't' not in start:
      start['t'] = ww.t_cam - 100.
    if 't' in start:
      out += vcars((41, 'right', lambda t: 45. - 8. * (t - start['t'])))(ww)
    return out
  w.tracks = tracks
  out = w.run(50.)
  i = first_go(out)
  assert 't' in start and i is not None, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][::40]
  gone_t = start['t'] + 57. / 8.
  assert (i + 1) * DT_MDL >= gone_t + sr.VISION_LONG_CLEAR_S - 0.11
  limited = round(start['t'] / DT_MDL)
  assert any(h.key == 'tracked:41' for d in out[limited:i] for h in d.holds)
  assert any(d.reason == "Waiting: a car from the right" for d in out[limited:i]), {d.reason for d in out[limited:i]}
  assert look_said(out, i) and no_handback(out) and crossed(out)


def test_a_limited_views_go_is_withdrawn_for_a_car_seen_during_its_launch_and_goes_again_once_it_has_gone(tmp_path):
  # the launch's re-check judges the limited go as any: a lane short of its floor withdraws nothing (it was gone on as it was),
  # a box in a lane does, short of the closest plausible edge, and the next go comes on a fresh window
  w = vdrive(tmp_path, j=two_way_junction(20))
  w.tracks = corner_car()
  seen = {'on': None}

  def boxes(ww):
    if seen['on'] is None and ww.sg.state == 'going' and ww.v > 0.5:
      seen['on'] = ww.t_cam
    if seen['on'] is not None and ww.t_cam - seen['on'] < 3.:
      return lane_box(ww, 'right', 25., views=('side',))
    return {}
  w.boxes = boxes
  out = w.run(55.)
  k = next((n for n, d in enumerate(out) if d.withdraw), None)
  assert k is not None and out[k].withdraw.startswith('raw:'), [d.withdraw for d in out if d.withdraw]
  assert max(d.position_m for d in out[k:k + 60]) <= w.sg.edges.d_min + 1e-6
  gos = [n for n, d in enumerate(out) if d.go_cause in ('stopSign', 'clearBox')]  # from the abort's rest: committed (no room)
  assert len(gos) >= 2 and no_handback(out) and crossed(out)


def test_an_earlier_arrival_standing_at_its_line_while_a_lane_is_hidden_is_crept_past_and_waived_then_the_car_goes(tmp_path):
  # an all-way: a car stood at the left line before we arrived (an earlier arrival, its lane's shield) and never goes, and a car
  # parked 18 m down the right lane hides it behind itself. With a lane short of its reach no deadlock creep could start (it
  # needs every other go condition met), so the turn order was a static cause beside the reach: no creep for a view either
  # (the wait wasn't on reach alone), H3 15 s after the stop. Now the two together are the creep for a view's wait: crept to
  # its limit, the right lane judged on what is seen, the turn passes to us (no room left for the deadlock's creep) and the car
  # goes, never a handback
  w = vdrive(tmp_path, 20)
  hidden, earlier = vcars((3, 'right', standing(18.), 2, 0.05)), vcars((1, 'left', standing()))
  w.tracks = lambda ww: list(hidden(ww)) + list(earlier(ww))
  events = []
  real = sg.cloudlog.event
  sg.cloudlog.event = lambda name, **kw: (events.append(name), real(name, **kw))
  try:
    watch(w)
    out = w.run(45.)
  finally:
    sg.cloudlog.event = real
  a, i = arrived_at(out), first_go(out)
  assert out[a + 1].rank == 1
  assert any({h.key for h in d.holds} == {'reach:right1', 'turn'} and all(h.kind == 'static' for h in d.holds) for d in out[a:i])
  assert creeps(out) and i is not None and creeps(out)[0] < i, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][a::40]
  assert 'turnpilot.stop_go_view_limited' in events and 'turnpilot.stop_go_order_waived' in events
  assert look_said(out, i) and out[i].log['viewLimited'] and no_handback(out) and crossed(out)


@pytest.mark.parametrize('when', list(WHEN))
def test_a_cross_road_of_two_lanes_each_way_with_no_room_to_creep_goes_on_what_it_sees_from_its_stand(tmp_path, when):
  # two lanes each way: the near lane runs out of the side camera's frame 20.2 m down it (of 37.6 m at 25 mph), and the stop
  # point already stands inside the room a go needs from the closest plausible edge (dEdgeMin -0.4 m: the cross road's
  # unsurveyed band), so no creep can start and no go can be withdrawn short of it. Before: H3 15 s after the stop. Now the
  # creep's limit is where it stands: the long look's window, "Look both ways", and the go committed from there (the
  # in-path rail and the band's threat test after it), never a handback
  w = vdrive(tmp_path, j=lanes_junction(25, left=2, right=2), wall_t=WHEN[when])
  out = w.run(30.)
  a, i = arrived_at(out), next((k for k, d in enumerate(out) if d.go_cause in ('stopSign', 'clearBox') and d.state != 'inLane'), None)
  assert i is not None, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][a::40]
  assert not creeps(out) and out[i].log['viewLimited']
  assert any(x.role == 'left' and x.binding == 'frameEdge' and x.r_seen < x.r_req for x in out[i - 1].lanes)
  assert look_said(out, i) and (i - a) * DT_MDL <= sr.VISION_CREEP_AFTER_S + sr.VISION_LONG_CLEAR_S + 0.6
  assert no_handback(out) and crossed(out)


# ---------------------------------------------------------------------- washed out or frozen: the camera can't see there

def goes_on_what_it_sees(out, words=None):
  """The stand ends in the go on what the cameras see: no creep or wait for a view that won't come, the lane's words said,
  "Look both ways" before the go, the long look's window from the stand's first second, never a handback. Returns the go."""
  a, i = arrived_at(out), first_go(out)
  assert i is not None, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][a::40]
  assert (i - a) * DT_MDL <= sr.VISION_CREEP_AFTER_S + sr.VISION_LONG_CLEAR_S + 0.6
  assert words is None or any(words in d.reason for d in out[a:i]), {d.reason for d in out[a:i]}
  assert out[i].log['viewLimited'] and look_said(out, i) and no_handback(out) and crossed(out)
  return i


def held_for_a_car_coming(w, seconds=14.):
  """The rail on a limited view: a car the forward cameras track coming down the left lane (8 m/s, from 45 m, 2 s into the
  stand) holds the go until it is by."""
  w.tracks = forward_cars(w.j, (41, 'left', lambda t: None if t < 2. else 45. - 8. * (t - 2.)))
  out = w.run(seconds)
  i = first_go(out)
  held = [k for k, d in enumerate(out) if any(h.key == 'tracked:41' for h in d.holds)]
  assert held and i is not None and i > max(held) and (i + 1) * DT_MDL >= 2. + 45. / 8., (i, held[:1], held[-1:])
  return out


@pytest.mark.parametrize('quality', ['washed', 'frozen'])
@pytest.mark.parametrize('when', list(WHEN))
def test_a_washed_out_or_frozen_side_view_is_said_and_the_car_goes_on_what_its_cameras_see(tmp_path, monkeypatch, when, quality):
  # a washed sector (the sun, a streetlight or headlights in the lens) or a frozen one is a bearing that side camera can't
  # see, for as long as the light shines. The lane is short as behind any occluder: said in plain words, no creep (a step
  # forward shows no more), then the long look and the go on what is seen. A car seen coming still holds the go
  monkeypatch.setattr(tsg, 'sectors', far_sectors(quality, 62.5))
  out = vdrive(tmp_path, wall_t=WHEN[when]).run(20.)
  goes_on_what_it_sees(out, sg.REACH_WORDS[quality])
  assert not creeps(out)
  held_for_a_car_coming(vworld(tmp_path, wall_t=WHEN[when]))


# --------------------------------------------------------------------------------- the light at the line: glare, no gate

LOW_SUN = utc(2026, 10, 28, 22, 0)  # 18:00 EDT: the sun low in the west-south-west, along the left approach


def test_low_sun_along_an_approach_is_judged_by_the_walk_never_the_gates_handback(tmp_path):
  # before: vision's gate failed the light at the line (low sun) and handed back 1 s after the full stop (H9, "Stop sign: low
  # sun ...: take control"). Glare is a view the walk judges (its low-sun and sun-ahead zeroes, the sectors' washed test): the
  # car waits on reach, creeps for a view, and goes on what it sees
  el, _az = sr.sun_position(39.7684, -86.1581, LOW_SUN)
  assert 0. < el < sr.SUN_EL_MAX
  w = vdrive(tmp_path, wall_t=LOW_SUN)
  out = w.run(45.)
  assert not any(d.latch == 'shadow' for d in out) and w.sg.gate is None
  i = first_go(out)
  assert i is not None, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][::40]
  assert any(x.binding == 'lowSun' for d in out[:i] for x in d.lanes)
  assert no_handback(out) and crossed(out)


# --------------------------------------------------------------------------- other judgement bounds: waits, not handbacks

def test_a_car_stopped_in_the_junction_holds_the_go_with_no_bound_and_the_car_goes_once_it_has_left(tmp_path):
  # before: a vehicle standing in the box was a static wait, H3 at 15 s; under vision it holds the go for as long as it stands
  w = vworld(tmp_path, 20)
  box_car = SimpleNamespace(trackId=60, cls='car', x=1.9 + 3.3 + 4., y=0.5, vx=0., vy=0.)
  w.objects = lambda ww: (box_car,) if ww.t_cam - 100. < 30. else ()
  out = w.run(36.)
  held = [d for d in out if any(h.key == 'box:60' for h in d.holds)]
  assert held and all(h.kind == 'dynamic' for d in held for h in d.holds if h.key == 'box:60')
  i = first_go(out)
  assert i is not None and i * DT_MDL > 30. and no_handback(out)


def test_a_car_yawed_past_8_deg_at_its_line_is_no_hold_and_the_car_goes(tmp_path):
  # a standing car's heading doesn't change by waiting, so its yaw is no hold. It is the walk's rotation bound: the lanes
  # read shorter (the road opposite 4.8 m), and no step forward can show enough of them (0.25 m a step), so there is no
  # creep: the long look and the go on what is seen. A car seen coming still holds the go
  w = vdrive(tmp_path, 15)
  out = w.run(30., yaw20_deg=12.)
  goes_on_what_it_sees(out)
  assert not creeps(out) and not any(h.key == 'yaw' for d in out for h in d.holds)
  w = vworld(tmp_path, 15)
  w.kw['yaw20_deg'] = 12.
  held_for_a_car_coming(w, 30.)


def test_a_person_in_front_still_holds_and_is_still_handed_back_at_30_s(tmp_path):
  # the hard rail is unchanged on a limited view: a person TurnPilot lost in front of the car holds the go (P6), H7 at
  # FRONT_ZONE_MAX_S, never a creep toward them
  w = vdrive(tmp_path, j=two_way_junction(20))
  w.tracks = corner_car()
  out = w.run(31., person_front=True)
  assert first_go(out) is None and not creeps(out)
  assert any(d.fault == 'H7' for d in out)


# ------------------------------------------------------------------- the review of 2026-11-08: glare, the creep's run-on, stuck

def side_sectors(left, right):
  """A side view's sectorQuality per side: left and right are (quality, from_deg) (ok from the inner edge to from_deg)."""
  def fn(view):
    return far_sectors(*(left if view == 'sideLeft' else right))(view)
  return fn


SUN_AHEAD = utc(2026, 10, 28, 17, 0)  # 13:00 EDT: the sun ahead of a car heading south
GLARE = {'lowSun': (LOW_SUN, {}, (('washed', 62.5), ('ok', 90.))),
         'sunAhead': (SUN_AHEAD, {'heading_in': 180.}, (('washed', 72.5), ('washed', 72.5)))}


@pytest.mark.parametrize('glare', list(GLARE))
def test_glare_with_a_side_camera_washed_out_inside_the_suns_zone_goes_on_what_it_sees_and_a_car_coming_holds(tmp_path, monkeypatch,
                                                                                                              glare):
  # the sun in a side lens: the lanes read short (the glare's zero, the washed sectors), which is a view, never a wait with
  # no bound: the long look and the go on what is seen, and a car the forward cameras see coming holds it
  wall_t, jkw, (left, right) = GLARE[glare]
  monkeypatch.setattr(tsg, 'sectors', side_sectors(left, right))
  out = vdrive(tmp_path, wall_t=wall_t, j=vision_junction(**jkw)).run(30.)
  i = first_go(out)
  assert i is not None and any(x.r_seen < x.r_req for x in out[i - 1].lanes)
  assert out[i].log['viewLimited'] and look_said(out, i) and no_handback(out) and crossed(out)
  held_for_a_car_coming(vworld(tmp_path, wall_t=wall_t, j=vision_junction(**jkw)), 40.)


def test_low_sun_with_a_washed_sector_the_lanes_walk_never_reaches_on_a_plain_day_still_goes_on_what_it_sees(tmp_path, monkeypatch):
  # the left side washed only from 87.5 deg (the sun's disc at the horizon): a plain day's walk ends on the views' range 40.2 m
  # down the near lane, past its 37.6 m floor, and goes; under low sun the lane stays the glare's: crept for, then the go on
  # what is seen, never a wait with no bound for a sector no plain-day walk would read
  monkeypatch.setattr(tsg, 'sectors', side_sectors(('washed', 87.5), ('ok', 90.)))
  w = vdrive(tmp_path, wall_t=LOW_SUN)
  out = w.run(45.)
  a, i = arrived_at(out), first_go(out)
  assert i is not None, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][a::80]
  assert any(x.role == 'left' and x.binding == 'lowSun' and x.r_seen < x.r_req for x in out[i - 1].lanes)
  assert out[i].log['viewLimited'] and look_said(out, i) and no_handback(out) and crossed(out)


@pytest.mark.parametrize('d0', [0.35, -0.3])
def test_with_an_actuator_slower_than_the_decider_knows_the_creeps_last_stand_still_leaves_a_go_that_can_be_withdrawn(tmp_path, d0):
  # each step's target is shortened by the run-on the known delay gives; the actuator twice as slow (0.6 s, the decider told
  # 0.3 s) ran the last step on 0.02 m inside the go's room, and the limited view's go went committed from there (clearBox: no
  # launch re-check, nothing withdrawn short of the closest plausible edge). The steps now stop one delay's run-on short
  w = vdrive(tmp_path, j=two_way_junction(25), lag_s=0.6, d0=d0)
  w.kw['t_abort'] = sr.t_abort(0.3)
  w.tracks = corner_car()
  out = w.run(50.)
  gos = [k for k, d in enumerate(out) if d.go_cause in ('stopSign', 'clearBox')]
  assert creeps(out) and gos, [(round(k * DT_MDL, 1), d.state, d.reason) for k, d in enumerate(out)][::80]
  assert out[gos[0]].go_cause == 'stopSign' and out[gos[0]].log['viewLimited']
  assert w.sg.edges.d_min - out[gos[0]].position_m >= sr.go_room_vision_m(w.kw['t_abort']) - 1e-6
  assert no_handback(out) and crossed(out)


def stuck(view):
  """The left side view's feed stuck (sideview: its luma repeated bit for bit, every sector frozen); the right as ever."""
  return far_sectors('frozen', 0.)(view) if view == 'sideLeft' else far_sectors('ok', 90.)(view)


@pytest.mark.parametrize('when', list(WHEN))
def test_a_stuck_side_camera_feed_is_no_fault_and_the_car_goes_on_its_other_cameras(tmp_path, monkeypatch, when):
  # a side feed stuck (every sector frozen) is no camera fault: the side views are used when they run and never needed. The
  # left lane is short (said: side camera frozen), the go comes on what the other cameras see, and the stuck feed is in the
  # log (stop_go_side_views) once it has been out DET_FAULT_S
  events = []
  monkeypatch.setattr(sg.cloudlog, 'event', lambda name, **kw: events.append((name, kw)))
  monkeypatch.setattr(tsg, 'sectors', stuck)
  w = vdrive(tmp_path, wall_t=WHEN[when])
  out = w.run(16.)
  goes_on_what_it_sees(out, sg.REACH_WORDS['frozen'])
  assert not any(d.alert == 'cameraPaused' for d in out)
  w = vworld(tmp_path, wall_t=WHEN[when])
  out = held_for_a_car_coming(w, 16.)
  said = [kw for name, kw in events if name == 'turnpilot.stop_go_side_views']
  assert said and said[-1]['out'] and said[-1]['dead'] == ['sideLeft'] and not any(d.fault for d in out)


def test_a_side_feed_stuck_for_a_moment_then_live_again_is_no_fault_and_the_car_goes(tmp_path, monkeypatch):
  # a frozen spell that clears: said while it lasts, no fault, and the car goes
  world = []

  def fn(view):
    t = world[0].t_cam - 100.
    return stuck(view) if 1.5 <= t < 5.5 else far_sectors('ok', 90.)(view)
  monkeypatch.setattr(tsg, 'sectors', fn)
  w = vdrive(tmp_path)
  world.append(w)
  out = w.run(24.)
  i = first_go(out)
  assert i is not None, [(round(n * DT_MDL, 1), d.state, d.reason) for n, d in enumerate(out)][::40]
  assert any(sg.REACH_WORDS['frozen'] in d.reason for d in out[:i])
  assert not any(d.fault for d in out) and no_handback(out) and crossed(out)
