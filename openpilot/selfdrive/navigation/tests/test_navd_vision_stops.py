"""navd's stop-sign avoidance under TurnPilotStopSignGo=vision with the real certifier on the shipped files (test_navigation's
TestStopAvoidance harness and its vision engine: turnpilotd publishing mode vision and its side views): what navd keeps on the
route after dark, a right whose far lane the line walks as its cross movement (certify.vision_lanes), and a stop the line is
short of reach at too (a cross road of 2 lanes each way), which is still routed round and marked unavoidable when it can't be.

Kellen, 2026-11-07: the problems are "our nav system", and "why the hell do we care about dark?". Before, every stop on the
11-06 night drives failed the light check at route time, was routed round where the 30 s budget allowed (1 of 7 tries on 1e)
and otherwise marked unavoidable, said "Stop sign ahead needs you" (6 times on 1e) and handed back at its line."""
from openpilot.selfdrive.navigation.tests import test_navigation as tn
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA

NIGHT = ETA + 8 * 3600.  # 23:00 EDT


class TestVisionStops(tn.TestStopAvoidance):
  def right_stop(self, n, route_m, mph=25):
    """vision_stop's all-way with the route turning right there (the right leg the exit)."""
    c = self.vision_stop(n, route_m, mph=mph)
    c.junction_info.maneuver = 'right'
    for a in c.junction_info.approaches:
      if a.role == 'right':
        a.role = 'exit'
    return c

  def wide_stop(self, n, route_m):
    """vision_stop's all-way on a cross road of 2 lanes each way."""
    c = self.vision_stop(n, route_m)
    for a in c.junction_info.approaches:
      if a.role in ('left', 'right'):
        a.lanes_in = a.lanes_out = 2
    return c

  def test_vision_at_night_keeps_the_stop_and_yield_certified_and_unannounced(self):
    self.vision_engine()
    self.now = NIGHT - 30.
    a, y = self.vision_stop(1, 60.), self.vision_stop(2, 150., kind='yield')
    router = self.start([a, y], reroutes=[[]])
    self.assertEqual(router.reroute_calls, [])
    self.assertEqual([(c.certification, c.cert_reason) for c in (a, y)], [('certifiable', 'none')] * 2, a.junction_info.cert_detail)
    ni = self.instruction()
    self.assertEqual((str(ni.avoidStatus), ni.avoidedControls, ni.unavoidableControls), ('clean', 0, 0))

  def test_vision_certifies_a_right_whose_far_lane_the_line_walks_as_its_cross_movement_day_or_night(self):
    # the route-time walk judged the far lane's swing zone too ('reach': 23.8 of 37.6 m), which the line doesn't walk under
    # 15 deg of yaw: navd excluded a right the line drives, tried the reroute, and marked it unavoidable
    for now in (ETA, NIGHT):
      self.vision_engine()
      self.now = now - 30.
      r = self.right_stop(1, 60.)
      router = self.start([r], reroutes=[[]])
      self.assertEqual((r.certification, r.cert_reason), ('certifiable', 'none'), r.junction_info.cert_detail)
      self.assertEqual(router.reroute_calls, [])
      ni = self.instruction()
      self.assertEqual((str(ni.avoidStatus), ni.unavoidableControls), ('clean', 0))

  def test_vision_routes_round_a_stop_the_line_is_short_at_too_and_marks_it_unavoidable_when_it_cant_day_or_night(self):
    # review of the 11-06 repairs: kept unannounced, such a stop waited, crept and handed back 15 s after its full stop
    # (tests/test_turnpilotd_stop_night.py) with the driver never told
    for now in (ETA, NIGHT):
      self.vision_engine()
      self.now = now - 30.
      keep, wide = self.vision_stop(1, 60.), self.wide_stop(2, 150.)
      router = self.start([keep, wide], reroutes=[[keep]])
      self.assertEqual([(c.certification, c.cert_reason) for c in (keep, wide)], [('certifiable', 'none'), ('uncertifiable', 'reach')])
      self.assertEqual([len(c[4]) for c in router.reroute_calls], [1])
      ni = self.instruction()
      self.assertEqual((str(ni.avoidStatus), ni.avoidedControls, ni.unavoidableControls), ('clean', 1, 0))
      self.vision_engine()
      self.now = now - 30.
      wide = self.wide_stop(1, 60.)
      router = self.start([wide], reroutes=[None])
      self.assertEqual(wide.certification, 'unavoidable')
      self.assertEqual(self.instruction().unavoidableControls, 1)

  def test_vision_at_night_still_routes_round_what_its_line_hands_back(self):
    self.vision_engine()
    self.now = NIGHT - 30.
    keep, divided = self.vision_stop(1, 60.), self.vision_stop(2, 150., divided=True)
    router = self.start([keep, divided], reroutes=[[keep]])
    self.assertEqual((keep.certification, divided.cert_reason), ('certifiable', 'geometry'))
    self.assertEqual([len(c[4]) for c in router.reroute_calls], [1])


# the harness's own tests run in test_navigation.py, not again here
for _name in [n for n in dir(tn.TestStopAvoidance) if n.startswith('test')]:
  setattr(TestVisionStops, _name, None)
