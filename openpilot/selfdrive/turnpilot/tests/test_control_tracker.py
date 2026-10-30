"""ControlTracker: the route's stop signs, yields and signals between navd's 1 Hz fixes (control_tracker.py)."""
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.control_tracker import (CORRECTION_M, DROP_PAST_M, FIX_SPREAD_M, GAP_MAX_M, JUMP_M, OFF_ROUTE,
                                                            OFF_ROUTE_GRACE_S, ControlTracker)
from openpilot.selfdrive.turnpilot.tests.synthetic import nav_instruction


def nav(controls, route_count=1, status='ok', valid=True, off=(), off_status=None):
  return nav_instruction(500., route_count=route_count, controls=controls, controls_status=status, valid=valid, off_route=off,
                         off_route_status=off_status)


def step(tr, msg=None, v=10., valid=True, n=1):
  """n frames, msg arriving on the first; the controls after the last. tr.lost_all: every control lost on the way."""
  out, tr.lost_all = None, []
  for i in range(n):
    out = tr.update(msg if i == 0 else None, msg is not None and i == 0, valid, v, DT_MDL)
    tr.lost_all += tr.lost
  return out


def test_dead_reckons_between_fixes():
  tr = ControlTracker()
  c, = step(tr, nav([(3, 'stop', 100.)]), v=0.)
  assert c.key == (1, 3) and c.kind == 'stop' and c.d == 100.
  step(tr, v=10., n=20)  # a second at 10 m/s, no fix
  assert abs(tr.controls[(1, 3)].d - 90.) < 1e-6


def test_fix_pulls_at_most_half_a_meter():
  tr = ControlTracker()
  step(tr, nav([(0, 'signal', 100.)]), v=0.)
  step(tr, nav([(0, 'signal', 104.)]), v=10.)  # dead-reckoned 99.5, measured 104: +0.5 only
  assert abs(tr.controls[(1, 0)].d - (100. - 10. * DT_MDL + CORRECTION_M)) < 1e-6
  step(tr, nav([(0, 'signal', 80.)]), v=10.)
  assert abs(tr.controls[(1, 0)].d - (100. - 20. * DT_MDL + CORRECTION_M - CORRECTION_M)) < 1e-6


def test_no_correction_while_slow():
  tr = ControlTracker()
  step(tr, nav([(0, 'stop', 4.)]), v=0.)
  step(tr, nav([(0, 'stop', 7.)]), v=0.2)  # standstill GPS noise moves nothing
  assert abs(tr.controls[(1, 0)].d - (4. - 0.2 * DT_MDL)) < 1e-6


def test_jump_reinitialises_and_flags():
  tr = ControlTracker()
  step(tr, nav([(0, 'stop', 100.)]), v=0.)
  c, = step(tr, nav([(0, 'stop', 100. + JUMP_M + 5.)]), v=10.)
  assert c.jumped and c.d == 100. + JUMP_M + 5.
  c, = step(tr, v=10.)
  assert not c.jumped


def test_held_ignores_fixes():
  tr = ControlTracker()
  step(tr, nav([(0, 'stop', 2.)]), v=0.)
  tr.controls[(1, 0)].held = True
  step(tr, nav([(0, 'stop', 60.)]), v=1.)
  assert tr.controls[(1, 0)].d == 2.  # frozen: the latch is position-free


def test_fix_spread_is_how_far_navds_moving_fixes_disagree_over_the_last_20_m():
  # STOP_SIGNS.md 7.1 H2: the stop-sign decider's position fault reads how far navd's fixes of the control disagree, the
  # car's own travel taken out, over the last FIX_SPREAD_M (20 m) driven; fixes while slow, and a held control's, are noise
  tr = ControlTracker()
  step(tr, nav([(0, 'stop', 100.)]), v=0.)
  d = 100.
  for _ in range(3):  # fixes 1 s apart at 10 m/s that agree with the odometer
    d -= 10. * 20 * DT_MDL
    step(tr, nav([(0, 'stop', d)]), v=10., n=20)
  assert tr.controls[(1, 0)].fix_spread < 0.01
  step(tr, nav([(0, 'stop', d - 10. * 20 * DT_MDL + 4.)]), v=10., n=20)  # one fix 4 m behind the others
  assert abs(tr.controls[(1, 0)].fix_spread - 4.) < 0.01
  d -= 10. * 20 * DT_MDL
  for _ in range(int(FIX_SPREAD_M / 10.) + 1):  # over 20 m on, the odd fix is gone
    d -= 10. * 20 * DT_MDL
    step(tr, nav([(0, 'stop', d)]), v=10., n=20)
  assert tr.controls[(1, 0)].fix_spread < 0.01
  step(tr, nav([(0, 'stop', d + 9.)]), v=0.3, n=1)  # creeping: GPS noise, not a fix
  tr.controls[(1, 0)].held = True
  step(tr, nav([(0, 'stop', d + 15.)]), v=1., n=1)  # held: the latch is position-free
  assert tr.controls[(1, 0)].fix_spread < 0.01


def test_new_route_drops_unengaged_and_carries_engaged():
  tr = ControlTracker()
  step(tr, nav([(0, 'stop', 50.), (1, 'signal', 300.)]), v=0.)
  tr.controls[(1, 0)].engaged = True
  tr.controls[(1, 0)].state = 'mine'
  # a reroute: the same stop sign 12 m off (adopted, keeps the old state and distance), the signal gone
  out = step(tr, nav([(4, 'stop', 62.), (5, 'yield', 200.)], route_count=2), v=0.)
  assert [c.key for c in out] == [(2, 4), (2, 5)]
  assert out[0].engaged and out[0].state == 'mine' and out[0].d == 50.
  # a reroute that doesn't list it: carried under its old key
  out = step(tr, nav([(0, 'yield', 200.)], route_count=3), v=0.)
  assert [c.key for c in out] == [(2, 4), (3, 0)]


def test_junction_info_rides_with_the_control_and_a_reroute_brings_its_own():
  """A stop or yield's junction (MapControl.junctionInfo) is copied to the tracked control as a plain dict (the message's
  buffer isn't kept); a signal has none. A reroute's same control takes over an engaged one's state, and the junction
  it carries is the new route's own (its maneuver may have changed); the lookups ahead off the route carry theirs."""
  tr = ControlTracker()
  left = {'status': 'ok', 'osmNodeId': 900, 'maneuver': 'left', 'approaches': [{'role': 'exit', 'tier': 'no', 'wayId': 812}]}
  stop, signal = step(tr, nav([(0, 'stop', 50., False, True, False, left), (1, 'signal', 300.)]), v=0.)
  assert isinstance(stop.junction_info, dict) and stop.junction_info['osmNodeId'] == 900 and signal.junction_info is None
  a = stop.junction_info['approaches'][0]
  assert (a['role'], a['tier'], a['wayId']) == ('exit', 'no', 812)
  stop.engaged, stop.state = True, 'mine'
  straight = left | {'maneuver': 'straight'}
  out = step(tr, nav([(4, 'stop', 58., False, True, False, straight)], route_count=2), v=0.)
  assert out[0].key == (2, 4) and out[0].state == 'mine' and out[0].junction_info['maneuver'] == 'straight'
  ahead = {'status': 'ok', 'osmNodeId': 901, 'maneuver': 'unknown'}
  out = step(tr, nav([], valid=False, off=[(7, 'stop', 40., False, True, False, ahead)]), valid=False, v=0.)
  assert [c.junction_info['osmNodeId'] for c in out if c.key[0] == OFF_ROUTE] == [901]


def test_dropped_once_past():
  tr = ControlTracker()
  step(tr, nav([(0, 'yield', 1.)]), v=0.)
  step(tr, v=10., n=int((DROP_PAST_M + 1.) / 10. / DT_MDL) + 1)
  assert not tr.controls


def test_unengaged_out_of_window_dropped_engaged_kept():
  tr = ControlTracker()
  step(tr, nav([(0, 'stop', 50.), (1, 'signal', 90.)]), v=0.)
  tr.controls[(1, 0)].engaged = True
  step(tr, nav([]), v=0.)
  assert list(tr.controls) == [(1, 0)]


def test_gap_dead_reckons_then_drops_but_never_the_held():
  tr = ControlTracker()
  step(tr, nav([(0, 'stop', 100.), (1, 'signal', 140.), (2, 'stop', 300.)]), v=0.)
  tr.controls[(1, 0)].held = True
  step(tr, nav([], valid=False), v=10., valid=False)
  step(tr, v=10., n=int(GAP_MAX_M / 10. / DT_MDL) - 5)
  assert len(tr.controls) == 3 and not tr.lost_all
  step(tr, v=10., n=10)
  assert list(tr.controls) == [(1, 0)] and {c.key for c in tr.lost_all} == {(1, 1), (1, 2)}


def test_stale_nav_is_a_gap():
  tr = ControlTracker()
  step(tr, nav([(0, 'stop', 300.)]), v=0.)
  step(tr, v=10., n=int(2.9 / DT_MDL))  # navd quiet for under 3 s: dead reckoning, not yet a gap
  assert tr.gap_m == 0.
  step(tr, v=10., n=int(GAP_MAX_M / 10. / DT_MDL) + 5)
  assert not tr.controls and [c.key for c in tr.lost_all] == [(1, 0)]


def test_status():
  tr = ControlTracker()
  assert tr.status == 'none' and not tr.route
  step(tr, nav([], status='unsupported'), v=0.)
  assert tr.status == 'unsupported' and tr.route
  step(tr, nav([], valid=False), v=0., valid=False)
  assert not tr.route  # off the route: no status shown for it


def test_off_route_the_lookup_ahead_replaces_the_stale_routes_controls():
  """BeamNG route_demo, 2026-10-28: off the route the tracker dead-reckoned the old route's signal 106 m out while the car
  drove through the one on its own road. Once navd's lookup ahead of the car is in, the stale route's controls go (a held
  one stays: the latch is position-free), and the car's own road's are tracked under the off-route key."""
  tr = ControlTracker()
  step(tr, nav([(0, 'signal', 106.), (1, 'stop', 3.)]), v=0.)
  tr.controls[(1, 1)].held = True
  step(tr, nav([], valid=False, off_status='pending'), valid=False, v=10.)
  assert (1, 0) in tr.controls  # nothing better known yet: dead-reckoned as before
  out = step(tr, nav([], valid=False, off=[(7, 'signal', 60.)]), valid=False, v=10.)
  assert [c.key for c in out] == [(1, 1), (OFF_ROUTE, 7)]
  assert out[1].kind == 'signal' and abs(out[1].d - 60.) < 1e-6
  step(tr, v=10., valid=False, n=10)  # dead-reckoned between lookups
  assert abs(tr.controls[(OFF_ROUTE, 7)].d - (60. - 10 * 10. * DT_MDL)) < 1e-6
  step(tr, nav([], valid=False, off=[(7, 'signal', 55.)]), valid=False, v=10.)  # a lookup is a fix like navd's on a route
  assert abs(tr.controls[(OFF_ROUTE, 7)].d - (60. - 11 * 10. * DT_MDL + CORRECTION_M)) < 1e-6


def test_off_route_the_lookup_that_takes_over_an_engaged_control_brings_its_own_distance():
  """The stale route's engaged signal handed to the lookup's within ADOPT_M keeps its state, at the lookup's distance (the
  road the car is on), marked jumped so the policy re-checks its stop."""
  tr = ControlTracker()
  step(tr, nav([(0, 'signal', 87.)]), v=10.)
  c = tr.controls[(1, 0)]
  c.engaged, c.state = True, 'state'
  out = step(tr, nav([], valid=False, off=[(7, 'signal', 72.)]), valid=False, v=10.)
  assert [o.key for o in out] == [(OFF_ROUTE, 7)]
  assert out[0].engaged and out[0].state == 'state' and abs(out[0].d - 72.) < 1e-6 and out[0].jumped


def test_off_route_an_engaged_control_takes_a_miss_then_goes():
  tr = ControlTracker()
  step(tr, nav([], valid=False, off=[(2, 'stop', 80.)]), valid=False)
  tr.controls[(OFF_ROUTE, 2)].engaged = True
  step(tr, nav([], valid=False, off=[], off_status='ok'), valid=False)  # one lookup misses it: a GPS hiccup
  assert (OFF_ROUTE, 2) in tr.controls
  step(tr, nav([], valid=False, off=[], off_status='ok'), valid=False)  # two: the car turned off that road
  assert (OFF_ROUTE, 2) not in tr.controls
  step(tr, nav([], valid=False, off=[(3, 'stop', 80.)]), valid=False)
  step(tr, nav([], valid=False, off=[], off_status='ok'), valid=False)  # not engaged: gone at once
  assert (OFF_ROUTE, 3) not in tr.controls


def test_back_on_a_route_its_control_takes_over_the_engaged_off_route_one():
  tr = ControlTracker()
  step(tr, nav([], valid=False, off=[(4, 'signal', 50.)]), valid=False)
  c = tr.controls[(OFF_ROUTE, 4)]
  c.engaged, c.state = True, 'state'
  out = step(tr, nav([(0, 'signal', 52.), (1, 'stop', 200.)], route_count=2))
  assert [o.key for o in out] == [(2, 0), (2, 1)]
  assert out[0].engaged and out[0].state == 'state' and abs(out[0].d - (50. - 10. * DT_MDL)) < 1e-6  # its own distance


def test_off_route_stops_unknown_only_after_the_grace_and_with_a_destination():
  tr = ControlTracker()
  step(tr, nav([(0, 'signal', 300.)]))
  assert not tr.off_route_unknown
  step(tr, nav([], valid=False, off_status='pending'), valid=False, n=int(OFF_ROUTE_GRACE_S / DT_MDL) - 2)
  assert not tr.off_route_unknown  # navd's one invalid message between two routes, a lookup on its way
  step(tr, valid=False, n=4)
  assert tr.off_route_unknown
  step(tr, nav([], valid=False, off_status='failed'), valid=False)
  assert tr.off_route_unknown
  step(tr, nav([], valid=False, off=[(0, 'stop', 90.)]), valid=False)
  assert not tr.off_route_unknown  # looked up: those are the stops ahead
  step(tr, nav([], valid=False, off=[], off_status='ok'), valid=False)
  assert not tr.off_route_unknown  # none ahead, and that is known
  step(tr, nav([(0, 'signal', 300.)]))
  assert not tr.off_route_unknown
  no_dest = ControlTracker()
  step(no_dest, nav([], valid=False, off_status='none'), valid=False, n=100)
  assert not no_dest.off_route_unknown  # no destination: navd isn't on a route nor trying to be
  first = ControlTracker()  # a destination, its first route not in yet (a corridor downloading), nothing looked up ahead
  step(first, nav([], route_count=0, valid=False, off_status='failed'), valid=False, n=int(OFF_ROUTE_GRACE_S / DT_MDL) + 2)
  assert first.off_route_unknown


def test_off_route_navd_gone_quiet_is_unknown():
  tr = ControlTracker()
  step(tr, nav([(0, 'signal', 300.)]))
  step(tr, nav([], valid=False, off=[(0, 'stop', 90.)]), valid=False)
  step(tr, valid=False, n=int((OFF_ROUTE_GRACE_S + 2.) / DT_MDL))
  assert tr.off_route_unknown  # no lookup for seconds: what is ahead now isn't known
