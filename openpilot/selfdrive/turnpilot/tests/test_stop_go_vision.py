"""The stop-sign decider under TurnPilotStopSignGo=vision (stop_go.py's vision branch; tp/vision-go's spec and ALWAYS_DRIVE's
amendments V1-V10): decider frames on test_stop_go's World and closed-loop Drive, with Saturday's inputs: the shipped
visibility file (PV1 tables, camera fields and mask null, the veil unmeasured), no surveys, the map's tiers (mapStrong at an
all-way, no on a two-way's through road), 15:00 EDT with the sun behind a northbound car, the side views running."""
import math
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_go as sg
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table as vt
from openpilot.selfdrive.turnpilot.routing import certify as ce
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, two_way_junction, utc, vision_junction, visibility
from openpilot.selfdrive.turnpilot.stop_evidence import CAR_HALF_LEN_M, DEFAULT_LINE_M, junction_lanes
from openpilot.selfdrive.turnpilot.tests.test_stop_evidence import track as ev_track
from openpilot.selfdrive.turnpilot.tests.test_stop_go import Drive, World, arrived_at, first_go, raw_box, rolling

MPH = sr.MPH
SHIPPED = vt.load(vt.SHIPPED)
LINE_V = DEFAULT_LINE_M + CAR_HALF_LEN_M  # m down its lane a side-on car's box centre sits with its front at its line (no survey)


def vworld(tmp_path, mph=25, j=None, cls=World, **kw):
  kw.setdefault('vis', SHIPPED)
  kw.setdefault('entries', [])
  kw.setdefault('mode', 'vision')
  return cls(tmp_path, j=j if j is not None else vision_junction(mph, maneuver=kw.get('maneuver', 'straight')), **kw)


def vdrive(tmp_path, mph=25, behaviour='goes', **kw):
  return vworld(tmp_path, mph, cls=Drive, behaviour=lambda w, b=behaviour: b, **kw)


def vlanes(j, maneuver='unknown'):
  return {ln.role: ln for ln in junction_lanes(j, maneuver, 1.9, None) if ln.index == 1}


def vcars(*specs):
  """World.tracks for scripted cars on vision's geometry (no survey: the unsurveyed edge band): (trackId, role, d(t) [, cls
  [, frac across the lane band]])."""
  def tracks(w):
    t = w.t_cam - 100.
    pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref) if w.sg.token is not None else -w.control.d
    out = []
    for spec in specs:
      tid, role, d_fn = spec[:3]
      cls = spec[3] if len(spec) > 3 else 2
      dd = d_fn(t)
      if dd is None or dd < -12.:
        continue
      ln = vlanes(w.j)[role]
      frac = spec[4] if len(spec) > 4 else 0.5  # where across its lane's plausible band (0: sLaneMin, 1: sLaneMax)
      s = ln.s_min + frac * (ln.s_max - ln.s_min) - pos
      view = 'sideLeft' if role == 'left' else 'sideRight'
      trk = ev_track(tid, w.t_cam, max(dd, 0.3), ln, view=view, cls=cls, s=s)
      half = math.degrees(math.atan2(0.9, math.hypot(s, dd)))
      trk.span_seen = tuple(sorted((trk.bearing_seen - half, trk.bearing_seen + half)))
      trk.hits = 5
      out.append(trk)
    return out
  return tracks


def standing(d=LINE_V, t0=-10.):
  return lambda t: None if t < t0 else d


def watch(w, seconds=2.):
  w.control.held = False
  out = w.run(seconds, approach=w.control)
  w.control.held = True
  return out


def gate_fault(out):
  return next((d for d in out if d.latch == 'shadow'), None)


# --------------------------------------------------------------------------------------------------- acting, the window

@pytest.mark.parametrize('mph', [15, 20, 25])
def test_vision_acts_on_the_shipped_file_with_no_survey_after_the_full_stop_and_2_5_s(tmp_path, mph):
  w = vworld(tmp_path, mph)
  out = w.run(6.)
  a = arrived_at(out)
  i = first_go(out)
  assert i is not None, [(d.state, d.reason, [h.key for h in d.holds]) for d in out[::10]]
  assert (i - a) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11) and out[i].acting and out[i].mode == 'vision'
  d = out[i]
  assert d.release == ((0, 1), 'clear') and d.prompt == 'stopGo' and d.reason == 'Going' and d.speed_cap == sr.STOP_GO_V_MAX
  lanes = out[i - 1].lanes
  assert {x.binding for x in lanes} <= {'frameEdge', 'range'} and {x.role for x in lanes} == {'left', 'right', 'opposite'}
  assert all(x.r_req == pytest.approx(sr.reach_needed_vision(mph * MPH)) and x.r_seen >= x.r_req for x in lanes)
  # the line and its chime VISION_ANNOUNCE_S before the go, the wait's words before that
  said = [k for k in range(a, i) if out[k].prompt == 'stopGo']
  assert said and (i - said[0]) * DT_MDL == pytest.approx(sr.VISION_ANNOUNCE_S, abs=0.11) and all(out[k].reason == 'Going' for k in said)
  assert all(out[k].prompt == 'waitingForCross' for k in range(a + 1, said[0]))
  assert all(out[k].release is None for k in said)  # the latch holds until the go


def test_vision_on_the_same_frames_goes_at_2_5_s_where_auto_goes_at_2_0(tmp_path):
  gos = {}
  for mode in ('auto', 'vision'):
    w = World(tmp_path, mode=mode)  # test_certify's filled file and survey: auto certifies, vision ignores the survey and stamp
    out = w.run(6.)
    gos[mode] = (first_go(out) - arrived_at(out)) * DT_MDL
  assert gos['auto'] == pytest.approx(sr.CLEAR_S, abs=0.11) and gos['vision'] == pytest.approx(sr.VISION_CLEAR_S, abs=0.11)


def test_vision_an_empty_two_way_goes_under_option_a(tmp_path):
  w = vworld(tmp_path, j=two_way_junction(20))
  out = w.run(6.)
  assert first_go(out) is not None and not any(d.fault for d in out)


def test_vision_the_noMask_and_notSurveyed_zeroes_never_bind(tmp_path):
  w = vworld(tmp_path, j=two_way_junction(25))
  out = w.run(3.)
  assert all(x.binding not in ('noMask', 'notSurveyed') and x.r_seen > 30. for d in out[-5:] for x in d.lanes)


# ------------------------------------------------------------------------------------------------------ the long look

@pytest.mark.parametrize('mph', [30, 35, 0])
def test_vision_a_cross_road_over_25_mph_or_unknown_goes_after_the_4_s_long_look_saying_look_both_ways(tmp_path, mph):
  w = vworld(tmp_path, mph)
  out = w.run(8.)
  a, i = arrived_at(out), first_go(out)
  assert i is not None and not any(d.fault or d.latch == 'shadow' for d in out)
  assert (i - a) * DT_MDL == pytest.approx(sr.VISION_LONG_CLEAR_S, abs=0.11)
  said = [k for k in range(a, i) if out[k].prompt == 'stopGo']
  assert said and (i - said[0]) * DT_MDL == pytest.approx(sr.VISION_ANNOUNCE_S, abs=0.11)
  assert all(out[k].reason == sg.LOOK_REASON for k in said) and out[i].reason == 'Going'
  assert all(x.r_req == pytest.approx(sr.reach_needed_vision(25 * MPH)) for x in out[i - 1].lanes)  # judged at 25 mph
  assert w.sg.long_look and out[i].log['longLook']


# ------------------------------------------------------------------------------------- re-engaged at the line, said first

def announced(events):
  return [kw for name, kw in events if name == 'turnpilot.stop_go_vision_announce']


@pytest.mark.parametrize('mph, reason', [(25, sg.GOING_REASON), (35, sg.LOOK_REASON), (0, sg.LOOK_REASON)])
def test_vision_re_engaged_at_the_line_with_the_window_run_says_it_1_s_before_the_go(tmp_path, monkeypatch, mph, reason):
  # the driver's brake or gas at the line disengages openpilot for the whole window, which counts on meanwhile; re-engaged,
  # the go's line and chime come first ("Look both ways" at a long look), 1 s of engaged frames, then the go: never a go on
  # the re-engaged frame with nothing said
  events = []
  monkeypatch.setattr(sg.cloudlog, 'event', lambda name, **kw: events.append((name, kw)))
  w = vworld(tmp_path, mph)
  w.run(1.5)
  off = w.run(6., enabled=False)
  assert first_go(off) is None and not any(d.prompt == 'stopGo' or d.release is not None for d in off)
  n = len(announced(events))
  out = w.run(3.)
  i = first_go(out)
  assert i is not None and out[i].reason == 'Going' and out[i].release == ((0, 1), 'clear')
  assert i * DT_MDL == pytest.approx(1.0, abs=0.06)  # VISION_ANNOUNCE_S: the line said on engaged frames this long first
  assert all(out[k].prompt == 'stopGo' and out[k].reason == reason and out[k].release is None for k in range(i))
  assert len(announced(events)) == n + 1 and announced(events)[-1]['reason'] == reason  # the re-engage's own line, logged


def test_vision_disengaged_inside_the_announcement_says_it_again_for_1_s_before_the_go(tmp_path, monkeypatch):
  # a brake tap 0.5 s into "Going": the go is not 0.2 s after the re-engage on the old line, it is said again for 1 s
  events = []
  monkeypatch.setattr(sg.cloudlog, 'event', lambda name, **kw: events.append((name, kw)))
  w = vworld(tmp_path, 25)
  pre = [w.step()]
  while pre[-1].prompt != 'stopGo':
    pre.append(w.step())
    assert len(pre) < round(6. / DT_MDL)
  pre += w.run(0.5)
  assert first_go(pre) is None and len(announced(events)) == 1
  w.run(0.3, enabled=False)
  out = w.run(3.)
  i = first_go(out)
  assert i is not None and i * DT_MDL == pytest.approx(1.0, abs=0.06)
  assert all(out[k].prompt == 'stopGo' and out[k].reason == sg.GOING_REASON for k in range(i))
  assert len(announced(events)) == 2


def test_vision_disengaged_short_of_the_announcement_goes_on_the_window_as_before(tmp_path):
  # re-engaged with the window not yet at its line: nothing changes, the line 1 s before, the go 2.5 s after the arrival
  w = vworld(tmp_path, 25)
  out = w.run(1.5)
  a = arrived_at(out)
  out += w.run(0.5, enabled=False)
  out += w.run(4.)
  i = first_go(out)
  assert i is not None and (i - a) * DT_MDL == pytest.approx(2.5, abs=0.11)
  said = [k for k in range(a, i) if out[k].prompt == 'stopGo']
  assert said and (i - said[0]) * DT_MDL == pytest.approx(1.0, abs=0.06)


def test_vision_an_unknown_speed_lane_is_watched_as_a_fast_one(tmp_path):
  # assumedLimit 0 (unmatched): its floor is the long look's 25 mph, but a raw box down the lane, an untrusted track's speed
  # and the band's threat take it as UNKNOWN_CLASS_MPH, never 0 (a raw car 20 m down the near lane, then a car seen 30 m down
  # it whose speed isn't trusted yet, each hold the go; it comes after both are gone)
  w = vworld(tmp_path, 0)
  ln = vlanes(w.j)['left']
  x0, y0 = ln.point(20., 'min')
  x1, y1 = ln.point(20., 'max')
  x, y = (x0 + x1) / 2. + 0.5, -(y0 + y1) / 2.

  def boxes(w):
    t = w.t_cam - 100.
    if 1.2 <= t < 3.:
      b = raw_box(math.degrees(math.atan2(y, x)), conf=0.5, xy=(x, y), view='sideLeft')
      return {'full': [b], 'sideLeft': [b]}
    return {}
  w.boxes = boxes
  w.tracks = vcars((8, 'left', lambda t: 30. if 4. <= t < 4.6 else None))
  out = w.run(12.)
  assert all(x.r_req == pytest.approx(sr.reach_needed_vision(25 * MPH)) for x in out[-1].lanes if x.role == 'left')
  assert all(ln.limit == pytest.approx(sr.UNKNOWN_CLASS_MPH * MPH) for ln in w.sg.junction_lanes(w.frame()))
  holds = {h.key.split(':')[0] for d in out for h in d.holds}
  assert 'raw' in holds and 'tracked' in holds, holds
  i = first_go(out)
  last = max(k for k in range(len(out) if i is None else i) if out[k].holds)
  assert i is not None and (i - last) * DT_MDL >= sr.VISION_LONG_CLEAR_S - 0.06


def test_vision_the_long_look_applies_to_the_maneuvers_union_off_the_route(tmp_path):
  j = vision_junction(15)
  j.approaches[2].assumed_limit = 35 * MPH  # the opposite approach (our own road's far side) 35 mph
  w = vworld(tmp_path, j=j, maneuver='unknown')
  out = w.run(8.)
  i = first_go(out)
  assert [r for r, _ in w.sg.long_look] == ['opposite'] and i is not None
  assert (i - arrived_at(out)) * DT_MDL == pytest.approx(sr.VISION_LONG_CLEAR_S, abs=0.11)  # straight ahead: no creep helps


def test_vision_the_long_look_creeps_for_a_fast_far_lane_while_the_near_one_keeps_its_floor(tmp_path):
  # the right road 35 mph, the left 15: a step forward gains the range-bound right lane and costs the near lane frame-edge
  # reach it can spare, so the long look creeps first (cause creep), then its 4 s window from where it stands
  j = vision_junction(15)
  j.approaches[1].assumed_limit = 35 * MPH
  w = vdrive(tmp_path, j=j, behaviour='holds')
  out = w.run(14.)
  creeps = [k for k, d in enumerate(out) if d.go_cause == 'creep']
  i = first_go(out)
  assert creeps and i is not None and creeps[0] < i and out[i - 1].prompt == 'stopGo' and out[i - 1].reason == sg.LOOK_REASON
  assert all(d.creep is None or d.creep[1] <= sg.VIEW_CREEP_V + 1e-9 for d in out)


# ------------------------------------------------------------------------------------------------------------ the gate

@pytest.mark.parametrize('status', ['off', 'disabled', 'unset'])
def test_vision_side_views_off_for_the_drive_is_the_1_s_handback_saying_so(tmp_path, status):
  w = vworld(tmp_path, side=status)
  out = w.run(4.)
  d = gate_fault(out)
  assert d is not None and first_go(out) is None
  assert d.reason == "Stop sign: side cameras off: take control" and d.fault == 'H9'


@pytest.mark.parametrize('wall_t, heading, words', [
  (ETA + 8 * 3600., 0., 'night at ETA +0 min'), (utc(2026, 10, 28, 23, 0), 0., 'dusk at ETA +0 min'),
  (utc(2026, 10, 28, 22, 0), 0., 'low sun along an approach at ETA +0 min'),
  (utc(2026, 10, 28, 17, 0), 180., 'sun ahead at ETA +0 min (veil unmeasured)'), (None, 0., "the clock isn't set")])
def test_vision_night_dusk_low_sun_the_sun_ahead_and_no_clock_hand_back_saying_which(tmp_path, wall_t, heading, words):
  w = vworld(tmp_path, j=vision_junction(heading_in=heading), wall_t=wall_t)
  out = w.run(4.)
  d = gate_fault(out)
  assert d is not None and first_go(out) is None and d.reason.startswith('Stop sign: ') and words in d.reason, d.reason


def test_vision_without_the_sidecar_is_h8(tmp_path):
  w = vworld(tmp_path, j=vision_junction(sidecar=False))
  d = gate_fault(w.run(4.))
  assert d is not None and d.reason == sg.REASONS['H8'] and d.fault == 'H8'


@pytest.mark.parametrize('changes, words', [({'divided': True}, 'a divided cross road'), ({'roundabout': True}, 'a roundabout'),
                                            ({'rail_distance': 20.}, 'a rail crossing 20 m away')])
def test_vision_geometry_hands_back_saying_which(tmp_path, changes, words):
  d = gate_fault(vworld(tmp_path, j=vision_junction(**changes)).run(4.))
  assert d is not None and d.reason == f"Stop sign: {words}: take control"


def test_vision_a_yield_says_yield_v10(tmp_path):
  from openpilot.selfdrive.turnpilot.map_controls import YIELD_STOP
  w = vworld(tmp_path, side='off')
  w.control.kind, w.state.mode = 'yield', YIELD_STOP  # a yield that became a stop (yieldStop): the decider's
  d = gate_fault(w.run(4.))
  assert d is not None and d.reason == "Yield: side cameras off: take control"


def test_vision_the_lens_check_runs_only_with_a_fitted_file(tmp_path):
  lenses = {'tracker': 'equidistant f=452.0 k=0.0 something else'}
  w = vworld(tmp_path)  # the shipped file has no lens: nothing to compare
  w.kw['lenses'] = lenses
  assert first_go(w.run(6.)) is not None
  w = vworld(tmp_path, vis=visibility(tmp_path))
  w.kw['lenses'] = lenses
  d = gate_fault(w.run(4.))
  assert d is not None and d.fault == 'H4' and d.reason == "Stop sign: camera lens mismatch: take control"


def test_vision_a_route_that_couldnt_avoid_the_stop_is_judged_at_the_line_v1(tmp_path):
  # V1: navd's 'unavoidable' (or 'uncertifiable') is no handback under vision: the line's own gate decides
  for cert in ('unavoidable', 'uncertifiable'):
    w = vworld(tmp_path)
    out = w.run(6., route_cert=cert)
    assert first_go(out) is not None and not any(d.fault for d in out), cert


def test_vision_the_gate_failing_after_the_first_go_is_a_held_h9_never_shadow_c14(tmp_path):
  w = vdrive(tmp_path, behaviour='holds')
  out = w.run(6.)
  i = first_go(out)
  assert i is not None
  # replay to just after the go, then the side views fail during the launch
  w = vdrive(tmp_path, behaviour='holds')
  w.run((i + 4) * DT_MDL)
  assert w.sg.state == 'going'
  w.side = 'disabled'
  out = w.run(4.)
  assert any(d.withdraw for d in out) and out[-1].fault == 'H9' and out[-1].state == 'ladder'
  assert out[-1].reason == "Stop sign: side cameras off: take control" and out[-1].prompt == 'stopSignTakeControl'
  assert not any(d.latch == 'shadow' for d in out)
  assert out[-1].stops and out[-1].stops[0].should_stop  # the decider's own hold, the latch long gone
  assert all(s.accel_cap >= -2. - 1e-9 for d in w.trace for s in d.stops)


def test_vision_maps_stale_waits_its_calibration_bound_not_h1_at_10_s(tmp_path):
  w = vworld(tmp_path, side='mapsStale')
  out = w.run(sg.CAL_WAIT_S + 2.)
  assert first_go(out) is None and not any(d.fault == 'H1' for d in out)
  assert any(h.key == 'calibration' for h in out[round(5. / DT_MDL)].holds)


# ----------------------------------------------------------------------------------------- traffic, the order, the shield

def test_vision_a_car_approaching_is_waited_for_then_the_go_comes_2_5_s_after_its_last_hold(tmp_path):
  w = vdrive(tmp_path, 20)
  w.tracks = vcars((5, 'left', rolling(1.5, d0=35., v0=11.)))
  out = w.run(14.)
  i = first_go(out)
  assert i is not None
  last = max(k for k in range(i) if out[k].holds)
  assert (i - last) * DT_MDL >= sr.VISION_CLEAR_S - 0.06
  assert any(h.key.startswith(('tracked:', 'raw:', 'reach:')) for k in range(last - 5, last + 1) for h in out[k].holds)


def test_vision_a_car_u2_would_call_far_still_meets_the_tracked_test(tmp_path):
  # every approach possibly not stopping: a car 30 m down an all-way's lane at 8 m/s holds the go under vision
  w = vworld(tmp_path, 20)
  w.tracks = vcars((5, 'left', lambda t: None if t < 0.5 else max(30. - 8. * (t - 0.5), LINE_V)))
  out = w.run(3.)
  assert any(h.key.startswith('tracked:5') for d in out for h in d.holds)


def test_vision_a_car_through_its_line_at_speed_is_no_downgrade(tmp_path):
  w = vdrive(tmp_path, 20, behaviour='holds')
  calls = []
  w.sg.on_downgrade = lambda j, a, ev: calls.append(ev)
  w.tracks = vcars((41, 'left', rolling(1.0, d0=45., v0=8.)))
  w.run(14.)
  assert calls == [] and not w.sg.downgraded


def test_vision_an_earlier_arrival_goes_first_then_we_go(tmp_path):
  w = vdrive(tmp_path, 20)
  w.tracks = vcars((7, 'left', lambda t: None if t < -10. else LINE_V if t < 6. else LINE_V - 6. * (t - 6.)))
  watch(w)
  out = w.run(16.)
  a = arrived_at(out)
  i = first_go(out)
  assert out[a + 1].rank == 1 and i is not None and (i - a) * DT_MDL > 6. - 2.
  assert not any(d.fault for d in out)


def test_vision_a_later_arrival_at_its_line_on_a_stop_mapped_approach_shields_its_lane_and_we_go(tmp_path):
  w = vdrive(tmp_path, 20)
  t_arrive = 1.2  # it stops at its line just after our full stop: later, it waits for us
  w.tracks = vcars((9, 'left', lambda t: None if t < t_arrive - 3. else max(LINE_V + 18. - 6. * (t - t_arrive + 3.), LINE_V)))
  out = w.run(14.)
  i = first_go(out)
  assert i is not None and not any(d.fault for d in out)
  assert any(x.role == 'left' and x.shielded for x in out[i - 1].lanes) or all(x.r_seen >= x.r_req for x in out[i - 1].lanes)


def test_vision_a_car_standing_down_a_two_ways_through_road_is_no_shield(tmp_path):
  # on an approach the map doesn't say stops, a car standing 6 m down the lane at the corner is likelier parked: never a shield;
  # its box caps the lane behind it (the static wait, the creep for a view, H3 at the bound), never a go
  w = vdrive(tmp_path, j=two_way_junction(20), behaviour='holds')
  w.tracks = vcars((3, 'left', standing(10.), 2, 0.05))  # at the near edge of the lane band: a bearing-only occluder
  out = w.run(sg.STATIC_WAIT_S + 8.)
  assert first_go(out) is None and not any(x.shielded for d in out for x in d.lanes)
  assert out[-1].fault in ('H3',) and "can't see far enough" in out[-1].reason


# ------------------------------------------------------------------------------------------------ ALWAYS_DRIVE V2-V9

def test_vision_v2_off_the_route_the_union_is_judged_at_once(tmp_path):
  w = vworld(tmp_path, 15, maneuver='unknown')
  out = w.run(6.)
  i = first_go(out)
  assert i is not None and (i - arrived_at(out)) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11)
  assert not any(h.key == 'route' for d in out for h in d.holds)


def test_vision_v3_goes_withdrawn_for_traffic_never_hand_back_h5(tmp_path):
  # a car's box comes into the near lane each time we start (the first three goes): every go withdrawn with the capped abort,
  # each next go after a fresh window, never H5 for them (auto hands back H5 after GO_RETRIES); the fourth goes across
  w = vdrive(tmp_path, 20)
  seen = {'n': 0, 'on': False}

  def boxes(w):
    if w.sg.state == 'going' and seen['n'] < 3 and not seen['on']:
      seen['n'] += 1
      seen['on'] = True
    if seen['on'] and w.sg.state != 'going' and w.v < 0.05:
      seen['on'] = False
    return {v: [raw_box(65., conf=0.5, xy=(8., 17.), view='sideLeft')] for v in ('full', 'sideLeft', 'road', 'sideRight')} if seen['on'] else {}
  w.boxes = boxes
  out = w.run(40.)
  withdrawn = [d.withdraw for d in out if d.withdraw]
  assert len(withdrawn) >= 3 and not any(d.fault for d in out), withdrawn
  assert any(d.state in ('inLane', 'done') for d in out)
  assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)


def test_vision_v3_a_model_that_never_takes_over_still_meets_go_retries(tmp_path):
  w = vdrive(tmp_path, 20, behaviour='holds')
  out = w.run(40.)
  assert out[-1].fault == 'H5' and all(d.takeover_s < 0. for d in out)


def test_vision_v4_stalled_in_a_lane_lifts_from_a_standstill_inside_2_25_m_and_hands_back_at_30_s_with_no_hold(tmp_path):
  w = vdrive(tmp_path, 20)
  k = 0
  while w.step().state != 'inLane':
    k += 1
    assert k < 600
  w.behaviour = lambda w: 'holds'
  w.v, w.queue = 0., type(w.queue)([0.] * len(w.queue))
  out = w.run(50.)
  boxes = [k for k, d in enumerate(out) if d.go_cause == 'clearBox']
  starts = [w.cmds[len(w.cmds) - len(out) + k][1] for k in boxes]
  assert boxes and all(v < sg.STAND_V for v in starts)  # each from a standstill
  assert w.sg.box_lift_m <= sg.VISION_BOX_LIFT_M + 0.1 and any(d.withdraw == 'budget' for d in out)  # 2.25 m lifted in all
  assert sum(1 for d in out if d.alert == 'stalledInJunction') > 1
  f = next(d for d in out if d.fault == 'H5')
  k = out.index(f)
  stood = sum(DT_MDL for c in w.cmds[len(w.cmds) - len(out):len(w.cmds) - len(out) + k] if c[1] < sg.STAND_V)
  assert stood >= sg.VISION_STALL_H5_S - 0.5 and f.prompt == 'stopSignTakeControl'
  assert all(not d.stops for d in out[k:])  # no hold added in the lane


def test_vision_v5_an_earlier_arrival_that_never_goes_is_waived_after_the_creeps(tmp_path):
  w = vdrive(tmp_path, 20)
  w.tracks = vcars((1, 'left', standing()))
  watch(w)
  out = w.run(25.)
  assert first_go(out) is not None and not any(d.fault for d in out)


def test_vision_v6_a_car_standing_in_the_exit_is_a_dynamic_wait_never_h3(tmp_path):
  w = vworld(tmp_path, 20)
  parked = SimpleNamespace(trackId=50, cls='car', x=1.9 + 13. + 7.4 + 3., y=0., vx=0., vy=0.)
  w.objects = lambda w: (parked,) if w.t_cam - 100. < 22. else ()
  out = w.run(26.)
  assert not any(d.fault for d in out)
  held = [d for d in out if any(h.key.startswith('exit:') for h in d.holds)]
  assert held and all(h.kind == 'dynamic' for d in held for h in d.holds if h.key.startswith('exit:'))
  i = first_go(out)
  assert i is not None and i * DT_MDL > 22.


def test_vision_v7_a_signal_head_releases_the_latch_with_no_lift_and_never_hands_back(tmp_path):
  w = vdrive(tmp_path, 20, behaviour='holds')
  w.run(0.05)
  w.sg.signal_seen = True  # as _feed sets it after SIGNAL_HEAD_RUNS head sightings (a flashing red, a mis-mapped signal)
  out = w.run(24.)
  gos = [d for d in out if d.release is not None or (d.state == 'going' and d.go_t == 0.)]
  assert gos and all(d.go_cause in ('none', 'creep') for d in out)
  assert not any(d.fault for d in out) and not any(h.key == 'signal' for d in out for h in d.holds)
  first_out = next(k for k, d in enumerate(out) if d.withdraw == 'window')
  assert sum(1 for d in out if d.withdraw == 'window') >= 2  # re-tried, silently after the first
  loud = [k for k, d in enumerate(out) if d.prompt == 'stopGo']
  assert loud and max(loud) < first_out


@pytest.mark.parametrize('yaw, goes', [(2., True), (5., True), (8., False)])
def test_vision_v8_the_car_yawed_up_to_8_deg_is_straight_enough_reach_deciding(tmp_path, yaw, goes):
  w = vworld(tmp_path, 15)
  out = w.run(6., yaw20_deg=yaw, steer_deg=20.)
  assert not any(h.key == 'yaw' for d in out for h in d.holds)
  assert (first_go(out) is not None) == goes
  if not goes:
    assert any(h.key.startswith('reach:') for h in out[-1].holds)  # honest: the walk's reach, never the yaw


def test_vision_v9_a_corner_occluder_starts_the_creep_for_a_view_after_2_s(tmp_path):
  # a car standing at the corner 10 m down the near lane hides the lane behind it: a static wait on reach alone, and the
  # creep for a view's rung is tried VISION_CREEP_AFTER_S into it (STATIC_WAIT_S under the other modes); a bearing-only
  # occluder a step never helps, so the walk says no step (logged) and the wait goes on to its bound
  w = vdrive(tmp_path, j=two_way_junction(20), behaviour='holds')
  w.tracks = vcars((3, 'left', standing(10.), 2, 0.05))
  events = []
  real = sg.cloudlog.event
  sg.cloudlog.event = lambda name, **kw: (events.append((name, w.sg.t, kw)), real(name, **kw))
  try:
    out = w.run(8.)
  finally:
    sg.cloudlog.event = real
  static = next(k for k, d in enumerate(out) if d.state == 'observe' and d.ledger == 'static')
  assert all(h.key.startswith('reach:') for h in out[static].holds)
  tried = [t for name, t, kw in events if name in ('turnpilot.stop_go_view_creep', 'turnpilot.stop_go_view_creep_no')]
  assert w.sg.vision_creep_tried and tried and tried[0] - static * DT_MDL <= sr.VISION_CREEP_AFTER_S + 0.3
  assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)


# ----------------------------------------------------------------------------------------------- the property, the log

def test_vision_every_decider_stop_is_minus_2_or_gentler_and_every_held_frame_says_why(tmp_path):
  for mph in (15, 25, 35):
    for behaviour in ('goes', 'holds'):
      w = vdrive(tmp_path, mph, behaviour=behaviour)
      w.tracks = vcars((5, 'left', rolling(3.4, d0=36., v0=10.)), (6, 'right', rolling(9., d0=30., v0=8.)))
      out = w.run(24.)
      assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)
      assert all(d.reason for d in out if d.state in ('stopped', 'observe', 'ladder'))


def test_vision_logs_its_gate_its_waits_its_announce_and_its_go(tmp_path, monkeypatch):
  events = []
  monkeypatch.setattr(sg.cloudlog, 'event', lambda name, **kw: events.append((name, kw)))
  w = vworld(tmp_path, 35)
  w.run(8.)
  names = [n for n, _ in events]
  gate = next(kw for n, kw in events if n == 'turnpilot.stop_go_vision_gate')
  assert gate['verdict'] == 'pass' and gate['longLook'] and ('left', 35.0) in gate['approaches']
  ann = next(kw for n, kw in events if n == 'turnpilot.stop_go_vision_announce')
  assert ann['reason'] == sg.LOOK_REASON and ann['need_s'] == sr.VISION_LONG_CLEAR_S
  go = next(kw for n, kw in events if n == 'turnpilot.stop_go_go')
  assert go['vision'] and go['need_s'] == sr.VISION_LONG_CLEAR_S and go['margins']
  assert 'turnpilot.stop_go_vision_wait' in names or 'turnpilot.stop_go_vision_announce' in names
  w = vworld(tmp_path, side='off')
  w.run(3.)
  fail = [kw for n, kw in events if n == 'turnpilot.stop_go_vision_gate' and kw['verdict'] == 'fail']
  assert fail and fail[-1]['reason'] == 'sideViews' and fail[-1]['sideViewStatus'] == 'off'


def test_vision_the_other_modes_are_unchanged_by_its_branch(tmp_path):
  # the four modes' traces are pinned by test_stop_go_modes_golden; here: auto on the shipped file is still H4camera
  w = World(tmp_path, mode='auto', vis=SHIPPED, entries=[], j=vision_junction())
  out = w.run(4.)
  assert first_go(out) is None and gate_fault(out).reason == sg.REASONS['H4camera']
  assert not any(d.mode == 'vision' for d in out) and 'vision' in ce.MODES


def test_vision_a_pedestrian_in_the_crosswalk_is_waited_for_then_the_go_after_the_window(tmp_path):
  from openpilot.selfdrive.turnpilot.tests.test_stop_go import person
  w = vworld(tmp_path, 20)
  # a person crossing the near crosswalk left to right at 1.2 m/s from 1 s, out past its right end about 5 s later
  def walker(w):
    t = w.t_cam - 100.
    return (person(70, 4.9, 2.0 - 1.2 * max(t - 1., 0.), 0., -1.2),) if 1. <= t < 7.5 else ()
  w.objects = walker
  out = w.run(12.)
  peds = [k for k, d in enumerate(out) if any(h.key.startswith(('ped:', 'pedArrive')) for h in d.holds)]
  i = first_go(out)
  assert peds and i is not None and i > max(peds) and (i - max(peds)) * DT_MDL >= sr.VISION_CLEAR_S - 0.06
