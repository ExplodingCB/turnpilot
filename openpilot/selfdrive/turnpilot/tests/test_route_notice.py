"""route_notice.py: what the driver hears about the route's stop signs (STOP_SIGNS.md v3 section 3.4, SG4c): at route start the
stops the avoidance routed around and the time it added (a chime past DETOUR_WARN), the stops that need the driver, "First
stop needs you" before engagement when the first is on the origin street, a failed avoidance; and "Stop sign ahead needs you"
300 m before each stop the route couldn't avoid. navInstruction readers built here as navd sends them."""
import pytest

from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.route_notice import AHEAD_M, DETOUR_WARN_MIN_S, NOTICE_S, RouteNotices, detour_warn_s


def nav(route=1, status='clean', avoided=0, added=0., remaining=600., controls=(), valid=True, unavoidable=None, origin=None):
  """A navInstruction reader: controls [(id, kind, m, certification[, osmNodeId])] near the car; unavoidable: the whole route's
  count (default: the unavoidable ones in controls), origin: the first of them on the route's first edge (default: the
  first unavoidable in controls under 200 m)."""
  msg = messaging.new_message('navInstruction')
  ni = msg.navInstruction
  ni.routeCount, ni.avoidStatus, ni.avoidedControls, ni.avoidAddedS = route, status, avoided, added
  ni.timeRemaining = remaining
  mine = sorted((c for c in controls if c[3] == 'unavoidable'), key=lambda c: c[2])
  ni.unavoidableControls = len(mine) if unavoidable is None else unavoidable
  ni.firstUnavoidableOnOrigin = (bool(mine) and mine[0][2] < 200.) if origin is None else origin
  cs = ni.init('controls', len(controls))
  for c, spec in zip(cs, controls, strict=True):
    c.id, c.kind, c.distance = spec[0], spec[1], spec[2]
    c.junctionInfo.status, c.junctionInfo.certification = 'ok', spec[3]
    c.junctionInfo.osmNodeId = spec[4] if len(spec) > 4 else 1000 + spec[0]
  return msg.as_reader().navInstruction, valid


def send(rn, n, v, engaged):
  """One frame with navd's message n ((reader, valid)) arriving."""
  return rn.update(n[0], True, n[1], v, engaged)


def said(rn, frames, n, v=0., engaged=False, period=20):
  """The notices said over frames, navd's message every period frames: [(alert, text)] as each starts."""
  out, last = [], ('none', '')
  for k in range(frames):
    msg, valid = n(k) if callable(n) else n
    got = rn.update(msg if k % period == 0 else None, k % period == 0, valid, v, engaged)
    if got[0] != 'none' and got[:2] != last:
      out.append(got[:2])
    last = got[:2]
  return out


def test_detour_warn_is_the_larger_of_half_the_trip_and_5_minutes():
  assert detour_warn_s(1200.) == pytest.approx(600.) and detour_warn_s(300.) == pytest.approx(DETOUR_WARN_MIN_S)


@pytest.mark.parametrize('added, base, kind', [(120., 600., 'routeAvoidsStops'), (400., 400., 'routeDetour')])
def test_route_start_says_how_many_stops_it_avoids_and_the_time_added_with_a_chime_past_detour_warn(added, base, kind):
  rn = RouteNotices()
  out = said(rn, round(4. / DT_MDL), nav(avoided=3, added=added, remaining=base + added))
  assert out[0] == (kind, f"Route avoids 3 stop signs (+{added / 60.:.0f} min)")
  assert rn.chime(kind) == (kind == 'routeDetour')


def test_route_start_says_the_stops_that_need_you_and_the_first_on_the_origin_street_before_engagement():
  # 2 unavoidable stops; the first 120 m ahead on the route's first edge: the origin street
  ctl = [(0, 'stop', 120., 'unavoidable'), (1, 'stop', 900., 'unavoidable'), (2, 'stop', 300., 'certifiable')]
  rn = RouteNotices()
  out = said(rn, round(10. / DT_MDL), nav(status='unavoidable', controls=ctl))
  kinds = [k for k, _ in out]
  assert kinds[:2] == ['stopsNeedYou', 'firstStopNeedsYou']
  assert out[0][1] == "2 stop signs on this route need you" and out[1][1] == "First stop needs you"
  assert all(rn.chime(k) for k in kinds)
  # engaged already (a reroute while driving), or the first beyond the first maneuver: no "First stop needs you"
  rn = RouteNotices()
  assert 'firstStopNeedsYou' not in [k for k, _ in said(rn, round(10. / DT_MDL), nav(status='unavoidable', controls=ctl), engaged=True)]
  far = [(0, 'stop', 500., 'unavoidable')]
  rn = RouteNotices()
  out = said(rn, round(10. / DT_MDL), nav(status='unavoidable', controls=far))
  assert [k for k, _ in out][:1] == ['stopsNeedYou'] and out[0][1] == "1 stop sign on this route needs you"
  assert 'firstStopNeedsYou' not in [k for k, _ in out]


def test_a_failed_avoidance_is_said_as_a_fault_at_route_start():
  rn = RouteNotices()
  out = said(rn, round(4. / DT_MDL), nav(status='failed'))
  assert out[0] == ('stopRoutingFailed', "Stop-sign routing failed: stops may need you") and rn.chime('stopRoutingFailed')


def test_each_notice_is_held_notice_s_then_the_next_and_a_clean_route_says_nothing():
  rn = RouteNotices()
  seq = [send(rn, nav(avoided=2, added=60., status='unavoidable', controls=[(0, 'stop', 900., 'unavoidable')]), 0., False)[0]]
  seq += [rn.update(None, False, True, 0., False)[0] for _ in range(round(3. * NOTICE_S / DT_MDL))]
  first = seq.index('stopsNeedYou')
  assert seq[:first] and all(k == 'routeAvoidsStops' for k in seq[:first]) and abs(first * DT_MDL - NOTICE_S) <= DT_MDL + 1e-6
  assert seq[-1] == 'none'
  rn = RouteNotices()
  assert said(rn, round(6. / DT_MDL), nav()) == []


def test_stop_sign_ahead_needs_you_300_m_before_each_unavoidable_stop_once_or_at_once_if_closer():
  # driving on (v 13 m/s, engaged), a route that started earlier: an unavoidable stop coming from 600 m, said once at AHEAD_M;
  # one first seen 150 m ahead (a reroute put it there) said at once; a certifiable one never
  def moving(k):
    d = 600. - 13. * k * DT_MDL
    return nav(route=1, status='unavoidable', controls=[(0, 'stop', d, 'unavoidable', 77), (1, 'stop', d + 50., 'certifiable', 78)])
  rn = RouteNotices()
  send(rn, nav(route=1, status='unavoidable'), 13., True)  # the route start, said and done
  rn.seen_start = True
  out, at = [], None
  for k in range(round(40. / DT_MDL)):
    msg, valid = moving(k)
    got = rn.update(msg if k % 20 == 0 else None, k % 20 == 0, valid, 13., True)
    if got[0] == 'stopAheadNeedsYou' and at is None:
      at = 600. - 13. * k * DT_MDL
    out.append(got[0])
  assert at is not None and AHEAD_M - 15. <= at <= AHEAD_M + 1e-6
  starts = [k for k in range(1, len(out)) if out[k] == 'stopAheadNeedsYou' and out[k - 1] != 'stopAheadNeedsYou']
  assert len(starts) == 1 and rn.chime('stopAheadNeedsYou')
  rn = RouteNotices()
  send(rn, nav(route=2, status='unavoidable'), 13., True)
  got = [send(rn, nav(route=2, status='unavoidable', controls=[(0, 'stop', 150., 'unavoidable', 90)]), 13., True)[0] for _ in range(3)]
  assert 'stopAheadNeedsYou' in got


def test_a_reroute_while_driving_doesnt_say_the_route_start_again():
  rn = RouteNotices()
  out = said(rn, round(8. / DT_MDL), nav(route=1, avoided=2, added=60.))
  assert out and out[0][0] == 'routeAvoidsStops'
  out = said(rn, round(8. / DT_MDL), nav(route=2, avoided=3, added=90.), v=12., engaged=True)
  assert out == []
  # off the route 20 s while driving (navd's messages invalid), then a reroute: still no route start
  rn.update(*nav(route=2, valid=False)[:1], True, False, 12., True)
  for _ in range(round(20. / DT_MDL)):
    rn.update(None, False, True, 12., True)
  assert said(rn, round(8. / DT_MDL), nav(route=3, avoided=3, added=90.), v=12., engaged=True) == []


def test_a_notice_held_back_behind_another_alert_is_said_in_full_once_free_and_one_for_a_stop_passed_meanwhile_is_dropped():
  # turnpilotd puts a notice on the plan only where no other alert is up (free): the queue had popped and counted down whether
  # or not it was shown, so a notice due behind another alert was used up unseen. Now its time runs only on frames it is said
  rn = RouteNotices()
  start = nav(avoided=2, added=60., status='unavoidable', controls=[(0, 'stop', 900., 'unavoidable')])
  got = [rn.update(start[0], True, True, 0., False, free=False)[0]]
  got += [rn.update(None, False, True, 0., False, free=False)[0] for _ in range(round(4. / DT_MDL))]
  assert set(got) == {'none'}
  seq = []
  for k in range(round(3. * NOTICE_S / DT_MDL)):
    free = not (round(1. / DT_MDL) <= k < round(3. / DT_MDL))  # another alert up for 2 s, 1 s into the first notice
    kind = rn.update(None, False, True, 0., False, free=free)[0]
    seq.append(kind if free else 'held')
  shown = [k for k in seq if k != 'held']
  first = shown.index('stopsNeedYou')
  assert all(k == 'routeAvoidsStops' for k in shown[:first]) and abs(first * DT_MDL - NOTICE_S) <= DT_MDL + 1e-6
  assert shown[first:first + round(NOTICE_S / DT_MDL)] == ['stopsNeedYou'] * round(NOTICE_S / DT_MDL)
  # "Stop sign ahead needs you" due at 300 m while another alert is up: said once the slot is free, and dropped (never said)
  # if the stop is passed first
  for passed in (False, True):
    rn = RouteNotices()
    send(rn, nav(route=1, status='unavoidable'), 13., True)
    out = []
    for k in range(round(40. / DT_MDL)):
      d = 320. - 13. * k * DT_MDL
      ctl = [(0, 'stop', d, 'unavoidable', 77)] if d > -15. else []
      msg, valid = nav(route=1, status='unavoidable', controls=ctl)
      if k % 20 == 0:
        b = msg.as_builder()
        b.controlsStatus = 'ok'
        msg = b.as_reader()
      free = d < (-20. if passed else 200.)
      out.append(rn.update(msg if k % 20 == 0 else None, k % 20 == 0, valid, 13., True, free=free)[0])
    assert ('stopAheadNeedsYou' in out) == (not passed), passed
    if passed:
      # dropped unsaid: a reroute that brings that stop back ahead says it then
      got = [send(rn, nav(route=1, status='unavoidable', controls=[(0, 'stop', 150., 'unavoidable', 77)]), 13., True)[0]
             for _ in range(3)]
      assert 'stopAheadNeedsYou' in got
