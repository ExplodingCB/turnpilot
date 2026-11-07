from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.route_follower import ARRIVE, KEEP, LEFT, NONE, RIGHT, TURN, UTURN, RouteFollower, classify
from openpilot.selfdrive.turnpilot.tests.synthetic import FakeNavd, nav_instruction


def test_classify():
  assert classify("turn", "left") == (TURN, LEFT)
  assert classify("end of road", "sharp right") == (TURN, RIGHT)
  assert classify("on ramp", "right") == (TURN, RIGHT)
  assert classify("turn", "slight right") == (KEEP, RIGHT)
  # the road bending: no keep desire, which at highway speed swerves the car
  assert classify("continue", "slight right") == (NONE, 0)
  assert classify("new name", "slight left") == (NONE, 0)
  assert classify("continue", "left") == (TURN, LEFT)
  assert classify("off ramp", "right") == (KEEP, RIGHT)
  assert classify("fork", "left") == (KEEP, LEFT)
  assert classify("turn", "uturn") == (UTURN, 0)
  assert classify("turn", "straight") == (NONE, 0)
  assert classify("roundabout", "left") == (NONE, 0)
  assert classify("arrive", "right") == (ARRIVE, RIGHT)


def test_dead_reckoning_between_updates():
  rf = RouteFollower()
  rf.update(nav_instruction(200.), True, True, 10., DT_MDL)
  assert rf.valid and rf.distance == 200.
  for _ in range(10):
    rf.update(None, False, True, 10., DT_MDL)
  assert abs(rf.distance - 195.) < 1e-6
  # the next message wins over the dead-reckoned estimate
  rf.update(nav_instruction(193.), True, True, 10., DT_MDL)
  assert rf.distance == 193.
  assert rf.maneuver.kind == TURN and rf.maneuver.side == LEFT


def test_passed_when_navd_moves_on():
  navd = FakeNavd([(100., "turn", "left"), (400., "turn", "right")])
  rf = RouteFollower()
  s, v, passed_at, ids = 0., 10., [], set()
  for _ in range(int(30 / DT_MDL)):
    instr = navd.update(s)
    rf.update(instr, instr is not None, True, v, DT_MDL)
    ids.add(rf.maneuver_id)
    if rf.passed:
      passed_at.append(s)
    s += v * DT_MDL
  assert len(passed_at) == 1 and 100. < passed_at[0] < 125.
  assert len(ids) == 2
  assert rf.maneuver.side == RIGHT


def test_gps_jump_is_not_a_new_maneuver():
  rf = RouteFollower()
  rf.update(nav_instruction(100.), True, True, 10., DT_MDL)
  first = rf.maneuver_id
  rf.update(nav_instruction(115.), True, True, 10., DT_MDL)
  # nor is it taken up: the junction stays at the nearest navd's readings put it (route_follower.py: JunctionSpan)
  assert rf.maneuver_id == first and not rf.passed and abs(rf.distance - 99.5) < 1e-6


def test_passed_by_dead_reckoning_when_navd_stalls():
  rf = RouteFollower()
  rf.update(nav_instruction(5.), True, True, 10., DT_MDL)
  passed = 0
  for _ in range(int(2.5 / DT_MDL)):
    rf.update(None, False, True, 10., DT_MDL)
    passed += rf.passed
  assert passed == 1 and rf.upcoming is None
  # navd catching up with the next step doesn't count the same maneuver twice
  rf.update(nav_instruction(300., modifier="right"), True, True, 10., DT_MDL)
  assert not rf.passed and rf.upcoming is not None


def test_route_lost():
  rf = RouteFollower()
  rf.update(nav_instruction(100.), True, True, 10., DT_MDL)
  rf.update(nav_instruction(100., valid=False), True, False, 10., DT_MDL)
  assert not rf.valid
  rf.update(nav_instruction(100.), True, True, 10., DT_MDL)
  for _ in range(int(4 / DT_MDL)):
    rf.update(None, False, True, 0., DT_MDL)
  assert not rf.valid


def test_arrived():
  rf = RouteFollower()
  rf.update(nav_instruction(40., "arrive", "right"), True, True, 5., DT_MDL)
  assert not rf.arrived
  rf.update(nav_instruction(10., "arrive", "right"), True, True, 5., DT_MDL)
  assert rf.arrived


def test_new_route_with_the_same_maneuver_is_a_new_maneuver():
  # a destination set from the phone 30 m before a left, whose route starts with a left 284 m on
  rf = RouteFollower()
  rf.update(nav_instruction(30.), True, True, 7., DT_MDL)
  first = rf.maneuver_id
  rf.update(nav_instruction(284.), True, True, 7., DT_MDL)
  assert rf.maneuver_id != first and rf.replaced and not rf.passed
  rf.update(None, False, True, 7., DT_MDL)
  assert not rf.replaced  # for the one update
  # navd marking the new route: one invalid instruction between the two
  rf = RouteFollower()
  rf.update(nav_instruction(30.), True, True, 7., DT_MDL)
  first = rf.maneuver_id
  rf.update(nav_instruction(30., valid=False), True, False, 7., DT_MDL)
  assert rf.maneuver_id != first and rf.replaced and rf.upcoming is None
  rf.update(nav_instruction(35.), True, True, 7., DT_MDL)
  assert rf.upcoming is not None and not rf.replaced and not rf.passed


def test_moving_on_from_a_sharp_turn_is_not_a_new_route():
  # navd moves on to the next step's geometry up to 10 m before a sharp turn's end: the left was passed
  rf = RouteFollower()
  rf.update(nav_instruction(8.), True, True, 4., DT_MDL)
  first = rf.maneuver_id
  rf.update(nav_instruction(300.), True, True, 4., DT_MDL)
  assert rf.maneuver_id != first and rf.passed and not rf.replaced


def test_committed_turn_is_held_through_a_gap_in_the_route():
  # the turn desire is out 14 m before a left when one navd update has a fix 20 m off the step (valid False, the same
  # route): the maneuver stays, dead-reckoned, under the same id, and navd's next update carries on with it
  rf = RouteFollower()
  rf.update(nav_instruction(14., route_count=1), True, True, 6., DT_MDL, committed=True)
  first = rf.maneuver_id
  rf.update(nav_instruction(12., valid=False, route_count=1), True, False, 6., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.replaced and rf.upcoming is not None and abs(rf.distance - 13.7) < 1e-4
  for _ in range(19):
    rf.update(None, False, True, 6., DT_MDL, committed=True)
  rf.update(nav_instruction(8., route_count=1), True, True, 6., DT_MDL, committed=True)
  # ...still dead-reckoned while the turn is committed: navd's reading of the same junction supports its track, but
  # doesn't move it
  assert rf.maneuver_id == first and not rf.replaced and not rf.passed and abs(rf.distance - 7.7) < 1e-4
  # not committed, the maneuver is dropped as before
  rf = RouteFollower()
  rf.update(nav_instruction(60., route_count=1), True, True, 6., DT_MDL)
  rf.update(nav_instruction(60., valid=False, route_count=1), True, False, 6., DT_MDL)
  assert rf.upcoming is None and not rf.valid


def test_committed_turn_is_held_while_navd_is_quiet_or_rerouting():
  # navd quiet (or rerouting, invalid) for longer than NAV_STALE_S with the turn desire out: the turn is dead-reckoned
  # to its pass, and the route is dropped once the turn is over
  for gap in ('quiet', 'invalid'):
    rf = RouteFollower()
    rf.update(nav_instruction(25., route_count=1), True, True, 5., DT_MDL, committed=True)
    first, passed = rf.maneuver_id, 0
    for i in range(int(10. / DT_MDL)):
      instr = nav_instruction(0., valid=False, route_count=1) if gap == 'invalid' and i % 20 == 0 else None
      rf.update(instr, instr is not None, False, 5., DT_MDL, committed=not passed)
      passed += rf.passed
      if not passed:
        assert rf.maneuver_id == first and rf.upcoming is not None and not rf.replaced, gap
    assert passed == 1, gap
    assert not rf.valid and rf.maneuver_id != first, gap  # lost once nothing holds it


def test_committed_turn_survives_a_distance_jump_and_a_reroute_onto_the_same_turn():
  # a 60 m jump in navd's distance from 30 m out (a GPS jump back along the road) isn't a new route for a committed turn
  rf = RouteFollower()
  rf.update(nav_instruction(30., route_count=1), True, True, 5., DT_MDL, committed=True)
  first = rf.maneuver_id
  rf.update(nav_instruction(90., route_count=1), True, True, 5., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.replaced and abs(rf.distance - 29.75) < 1e-4
  # a reroute (a new routeCount) that starts with the same turn where the car is: the same maneuver, at the nearest
  # any reading put it (2.5 m nearer than dead-reckoned: the cap and hold line early, never late)
  rf.update(nav_instruction(27., route_count=2), True, True, 5., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.replaced and abs(rf.distance - 27.) < 1e-4
  # a new route that doesn't turn there (a destination changed from the phone): the committed turn is held to its end
  rf.update(nav_instruction(284., route_count=3), True, True, 5., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.replaced and rf.upcoming is not None and abs(rf.distance - 26.75) < 1e-4
  # ...and once it is over, the new route is taken up: its turn a new maneuver
  rf.update(nav_instruction(283., route_count=3), True, True, 5., DT_MDL, committed=False)
  assert rf.maneuver_id != first and rf.distance == 283. and rf.upcoming is not None


LEFT_90 = (0., 270., 2)


def committed_approach(rf, distance=14., route_count=1):
  rf.update(nav_instruction(distance, junction=LEFT_90, route_count=route_count), True, True, 6., DT_MDL, committed=True)
  return rf.maneuver_id


def test_committed_turn_survives_a_jump_close_in():
  # a 60 m GPS jump 12 m before a committed left: the same junction (its bearings), so noise, however close in
  rf = RouteFollower()
  first = committed_approach(rf, 12.)
  rf.update(nav_instruction(72., junction=LEFT_90, route_count=1), True, True, 6., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.passed and not rf.replaced and abs(rf.distance - 11.7) < 1e-4
  # navd moving on early to the next step, a left from the heading this one ends on: the left is held, dead-reckoned,
  # until it is over (the car turned it, or dead-reckoned past it), and the next step taken up after
  for close, then in ((4., 'turned'), (15., 'turned'), (4., 'past')):
    rf = RouteFollower()
    first = committed_approach(rf, close)
    next_left = nav_instruction(200., junction=(270., 180., 2), route_count=1)
    rf.update(next_left, True, True, 6., DT_MDL, committed=True)
    assert rf.maneuver_id == first and not rf.passed and not rf.replaced and abs(rf.distance - (close - 0.3)) < 1e-4
    assert rf.upcoming is not None
    passed = 0
    for i in range(int(6. / DT_MDL)):
      committed = then == 'past' and not passed
      next_left = nav_instruction(200. - 0.3 * (i + 1), junction=(270., 180., 2), route_count=1)
      rf.update(next_left if i % 20 == 19 else None, i % 20 == 19, True, 6., DT_MDL, committed=committed)
      passed += rf.passed
      if committed:
        assert rf.maneuver_id == first and not rf.replaced, (close, then)
    assert passed == 1 and rf.maneuver_id != first and abs(rf.distance - 164.) < 1e-6 and not rf.replaced, (close, then)


def test_an_outlier_that_committed_the_turn_holds_it_early_and_never_passes_it_short():
  # navd's distance to a left reads 20 m short in one update 64 m out, and that reading brings the turn's first pulse
  # out (the turn is committed from the next frame). Which reading is right can't be told: the junction is held at the
  # nearest any of them put it (the cap and hold line early, never late) and the turn passed only once the car is
  # PASSED_DIST_M past the furthest, so never 20 m short with its cap lifted
  def second_on(rf, committed):  # navd's next update a second (6 m) on
    for _ in range(19):
      rf.update(None, False, True, 6., DT_MDL, committed=committed)
  rf = RouteFollower()
  rf.update(nav_instruction(70., junction=LEFT_90, route_count=1), True, True, 6., DT_MDL)
  first = rf.maneuver_id
  second_on(rf, False)
  rf.update(nav_instruction(64. - 20., junction=LEFT_90, route_count=1), True, True, 6., DT_MDL)  # taken up
  assert abs(rf.distance - 44.) < 1e-3
  second_on(rf, True)
  rf.update(nav_instruction(57., junction=LEFT_90, route_count=1), True, True, 6., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.passed and not rf.replaced and abs(rf.distance - 38.) < 1e-3
  assert abs(rf.junction_spread - 20.) < 1e-3
  # a reroute reading 25 m off widens the span the other way: the junction no further, the pass later
  rf.update(nav_instruction(82.7, junction=LEFT_90, route_count=2), True, True, 6., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.replaced and abs(rf.distance - 37.7) < 1e-3
  truth, passed_at = 57.7, None
  for _ in range(int(20. / DT_MDL)):
    truth -= 0.3
    rf.update(None, False, True, 6., DT_MDL, committed=passed_at is None)
    if passed_at is None and rf.passed:
      passed_at = truth
    if passed_at is None:
      assert rf.maneuver_id == first and rf.distance < truth
  assert passed_at is not None and passed_at < -39.


def test_a_committed_turns_junction_is_held_at_the_nearest_of_its_readings_and_passed_past_the_furthest():
  def second_on(rf, committed):  # navd's next update a second (6 m) on
    for _ in range(19):
      rf.update(None, False, True, 6., DT_MDL, committed=committed)
  def approach(rf, readings, committed_from):
    for i, (truth, off) in enumerate(readings):
      rf.update(nav_instruction(truth + off, junction=LEFT_90, route_count=1), True, True, 6., DT_MDL,
                committed=i > committed_from)
      second_on(rf, i >= committed_from)
  # the reading before the outlier 5 m off too, the outlier lasting two fixes and decaying: no vote decides it, the
  # junction held at the nearest any reading put it, not passed
  for offs in ((-5., -20., 0.), (12., -20., 0.), (-20., -20., 0.), (-20., -15., 0.), (-10., -20., -30., 0.)):
    rf = RouteFollower()
    truths = [100. - 6. * i for i in range(8)]
    readings = [(t, 0.) for t in truths[:4]] + list(zip(truths[4:], offs, strict=False))
    approach(rf, readings, 4 + offs.index(-20.))
    assert not rf.passed and abs(rf.distance - (truths[3 + len(offs)] - 5.7 + min(offs))) < 0.5, (offs, rf.distance)
  # an offset that lasts from after the commit, the readings before it many: the junction held at its nearest, 20 m
  # short, the turn passed only past the truth
  rf = RouteFollower()
  approach(rf, [(100. - 6. * i, 0.) for i in range(6)], 5)
  first, passed_at, truth = rf.maneuver_id, None, 70. - 5.7
  for i in range(int(14. / DT_MDL)):
    truth -= 0.3
    instr = nav_instruction(truth - 20., junction=LEFT_90, route_count=1) if i % 20 == 19 else None
    rf.update(instr, instr is not None, True, 6., DT_MDL, committed=passed_at is None)
    if passed_at is None and rf.passed:
      passed_at = truth
    if passed_at is None:
      assert rf.maneuver_id == first and abs(rf.distance - (truth if i < 19 else truth - 20.)) < 1e-3
  assert passed_at is not None and passed_at < -14.
  # one reading each way (a step navd has just moved on to, its first reading the outlier): the junction stays on the
  # track it is on, and isn't passed until the other track is too, never on the one reading
  rf = RouteFollower()
  rf.update(nav_instruction(40., junction=LEFT_90, route_count=1), True, True, 6., DT_MDL, committed=True)
  second_on(rf, True)
  rf.update(nav_instruction(54., junction=LEFT_90, route_count=1), True, True, 6., DT_MDL, committed=True)
  assert abs(rf.distance - 34.) < 1e-3
  passed_at, truth = None, 54.
  for _ in range(int(14. / DT_MDL)):
    truth -= 0.3
    rf.update(None, False, True, 6., DT_MDL, committed=passed_at is None)
    if passed_at is None and rf.passed:
      passed_at = truth
  assert passed_at is not None and passed_at < -14.


def test_reroute_onto_the_same_committed_turn_from_further_off():
  # navd reroutes 25 m off the route: the new route's first left reads up to that much off where it was dead-reckoned
  rf = RouteFollower()
  first = committed_approach(rf, 14.)
  rf.update(nav_instruction(39., junction=LEFT_90, route_count=2), True, True, 6., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.replaced and abs(rf.distance - 13.7) < 1e-4
  # and the new route's updates after it, still that far off, never put the junction further (the turn's cap would lift
  # or the turn be read as passed before the car gets there): short, it is held at their nearest (the cap early), and
  # long, the turn is passed only past them; taken from navd once the turn is over
  for off in (25., -20.):
    rf = RouteFollower()
    first = committed_approach(rf, 14.)
    rf.update(nav_instruction(14. + off, junction=LEFT_90, route_count=2), True, True, 6., DT_MDL, committed=True)
    truth, passed_at = 13.7, None
    for i in range(int(10. / DT_MDL)):
      truth -= 0.3
      instr = nav_instruction(truth + off, junction=LEFT_90, route_count=2) if i % 20 == 19 else None
      rf.update(instr, instr is not None, True, 6., DT_MDL, committed=passed_at is None)
      if passed_at is None and rf.passed:
        passed_at = truth
      if passed_at is None:
        assert rf.maneuver_id == first and not rf.replaced and abs(rf.distance - (truth + min(off, 0.))) < 0.31, off  # its first reading 0.3 m nearer
    assert passed_at is not None and passed_at < -14., off
    assert rf.maneuver_id == first, off  # never the same junction again under a new id
  # the next left on, the same way from the same road (a block on): a new route that doesn't turn here
  rf = RouteFollower()
  first = committed_approach(rf, 14.)
  rf.update(nav_instruction(110., junction=LEFT_90, route_count=2), True, True, 6., DT_MDL, committed=True)
  assert rf.maneuver_id == first and not rf.replaced and abs(rf.distance - 13.7) < 1e-4


def test_reroute_past_a_committed_turn_holds_it_to_its_end():
  # a reroute whose origin snapped past the junction (it starts with a straight-on step): the committed left is held,
  # dead-reckoned, to its pass, and the new route taken up after, not as a replacement
  rf = RouteFollower()
  first = committed_approach(rf, 10.)
  passed = 0
  for i in range(int(6. / DT_MDL)):
    instr = nav_instruction(200., "new name", "straight", junction=(0., 0., 2), route_count=2) if i % 20 == 0 else None
    rf.update(instr, instr is not None, True, 6., DT_MDL, committed=not passed)
    passed += rf.passed
    if not passed:
      assert rf.maneuver_id == first and rf.upcoming is not None and not rf.replaced
  assert passed == 1 and rf.maneuver_id != first and rf.maneuver.type == "new name"


def test_moves_on_past_a_straight_on_step_where_navd_does():
  # past a bend or a straight-on step by NAVD_ADVANCE_M, the turn after it is the maneuver; navd's updates still on the
  # step bring its distance, and navd moving on to it keeps its id
  rf = RouteFollower()
  ahead = [(40., "turn", "left", LEFT_90)]
  rf.update(nav_instruction(0., "new name", "straight", junction=(0., 0., 2), ahead=ahead), True, True, 5., DT_MDL)
  bend = rf.maneuver_id
  assert rf.maneuver.kind == NONE and rf.ahead is not None
  for _ in range(int(2.1 / DT_MDL)):  # 10.5 m on
    rf.update(None, False, True, 5., DT_MDL)
  assert rf.maneuver_id == bend + 1 and rf.upcoming.kind == TURN and abs(rf.distance - 29.5) < 0.3
  turn = rf.maneuver_id
  rf.update(nav_instruction(-11., "new name", "straight", junction=(0., 0., 2), ahead=[(29., "turn", "left", LEFT_90)]),
            True, True, 5., DT_MDL)
  assert rf.maneuver_id == turn and rf.distance == 29.
  rf.update(nav_instruction(28., junction=LEFT_90), True, True, 5., DT_MDL)
  assert rf.maneuver_id == turn and rf.distance == 28. and not rf.passed and not rf.replaced


def drive_readings(readings, v=6., commit_at=40., start=None, after_s=12.):
  """navd's readings of a left at 1 Hz, each (offset, route_count) on the truth, the car at v (m/s; 0 stands it), then
  true readings for after_s. The turn commits once the follower reads it under commit_at (its first pulse). Returns
  per frame (truth, follower distance, committed) and where the committed turn was first read as passed (truth): its
  id bumped, or replaced, counts too."""
  rf, truth, committed, rows, passed_at = RouteFollower(), start, False, [], None
  first = None
  def frame(instr, speed):
    nonlocal truth, committed, passed_at, first
    truth -= speed * DT_MDL
    rf.update(instr, instr is not None, True, speed, DT_MDL, committed=committed and passed_at is None)
    if committed and passed_at is None and (rf.passed or rf.maneuver_id != first or rf.replaced):
      passed_at = truth
    rows.append((truth, rf.distance, committed and passed_at is None))
    if not committed and rf.distance < commit_at:
      committed, first = True, rf.maneuver_id
  for off, route_count, speed in readings:
    frame(nav_instruction(truth - speed * DT_MDL + off, junction=LEFT_90, route_count=route_count), speed)
    for _ in range(19):
      frame(None, speed)
  for _ in range(int(after_s)):
    frame(nav_instruction(truth - v * DT_MDL, junction=LEFT_90, route_count=readings[-1][1]), v)
    for _ in range(19):
      frame(None, v)
  return rows, passed_at, rf


def assert_conservative(rows, passed_at, case):
  """The committed turn is never read as passed (its cap lifted, its id bumped) before the car is PASSED_DIST_M past
  the true junction, and its junction, for the cap and the hold line, is never further than the true one."""
  assert passed_at is not None and passed_at <= -15. + 0.1, (case, passed_at)
  late = [(round(t, 1), round(d, 1)) for t, d, c in rows if c and d > t + 0.5]
  assert not late, (case, late[:3])


def test_a_long_early_bias_never_outvotes_the_truth_at_the_commit():
  # p_seq2: navd's readings 20 m short (or long) for 16 s from far out, then true for the last 100 m. The readings
  # the bias won don't decide where the junction is: it is the nearest any reading put it, the turn passed only past
  # the furthest
  for off in (-20., 20., -12., -10.):
    rows, passed_at, _ = drive_readings([(off, 1, 6.)] * 16 + [(0., 1, 6.)] * 4, start=170., after_s=16.)
    assert_conservative(rows, passed_at, off)


def test_a_bias_beyond_same_turn_already_there_from_far_out_never_passes_the_turn_short():
  # navd true out to 230 m, then 45 m short (more than SAME_TURN_M) from before the car is FROM_M out, past the commit:
  # the true readings from further out are some of the junction's too, so it is passed only past them
  for off in (-45., -60., -41.):
    # the bias clears after the commit (true from 80 m in)
    rows, passed_at, _ = drive_readings([(0., 1, 6.)] * 5 + [(off, 1, 6.)] * 25, start=260., after_s=20.)
    assert_conservative(rows, passed_at, ('clears', off))
    # ...or never clears, to past the junction
    rows, passed_at, _ = drive_readings([(0., 1, 6.)] * 5 + [(off, 1, 6.)] * 50, start=260., after_s=0.)
    assert_conservative(rows, passed_at, ('lasts', off))


def test_a_bias_while_the_car_stands_in_a_queue_never_outvotes_the_truth():
  # stood 60 s at a light 150 m out with the fix 15 m off, every reading the same: then true readings to the turn
  for off in (-15., -10., 15.):
    rows, passed_at, _ = drive_readings([(off, 1, 0.)] * 60 + [(0., 1, 6.)] * 18, start=150.)
    assert_conservative(rows, passed_at, off)


def test_outliers_around_the_commit_never_pass_the_turn_short():
  # the reading that brings the first pulse out is an outlier, the ones either side of it off too, lasting two fixes,
  # or the first reading of a reroute
  for offs in ((-20.,), (-5., -20.), (12., -20.), (-20., -20.), (-20., -15.), (-10., -20., -30.), (-20., 60.),
               (-20., -17., -12., -6.)):
    readings = [(0., 1, 6.)] * 4 + [(o, 1, 6.) for o in offs] + [(0., 1, 6.)] * 2
    rows, passed_at, _ = drive_readings(readings, start=70. + 6. * len(offs))
    assert_conservative(rows, passed_at, offs)
  # a reroute whose first reading, which brings the pulse out, is snapped short, and its readings after it too
  for off in (-20., -25., -12.):
    rows, passed_at, _ = drive_readings([(0., 1, 6.)] * 4 + [(off, 2, 6.)] * 6, start=75.)
    assert_conservative(rows, passed_at, ('reroute', off))
  # the same, with the readings before the commit off too (ordinary noise, an earlier stretch of the bias): those are
  # no truth to vote the outlier down with
  for case, readings, start in (('first -5,-20', [(-5., 1, 6.), (-20., 1, 6.), (0., 1, 6.)], 46.),
                                ('-8 x3, -20', [(-8., 1, 6.)] * 3 + [(-20., 1, 6.)] + [(0., 1, 6.)] * 3, 70.),
                                ('-20 x3, 0 x2, -20', [(-20., 1, 6.)] * 3 + [(0., 1, 6.)] * 2 + [(-20., 1, 6.)] + [(0., 1, 6.)] * 3, 80.),
                                ('-6 x4, reroute -20 x4', [(-6., 1, 6.)] * 4 + [(-20., 2, 6.)] * 4, 75.)):
    rows, passed_at, _ = drive_readings(readings, start=start)
    assert_conservative(rows, passed_at, case)


def test_a_bias_that_starts_at_the_commit_and_lasts_never_passes_the_turn_short():
  # the turn's first reading (a step navd has just moved on to) brings the pulse out, and every reading after it is
  # off the same way: whichever way it is off, the junction is never further than the truth, nor passed short of it
  for off in (20., -20., 12.):
    rows, passed_at, _ = drive_readings([(0., 1, 6.)] + [(off, 1, 6.)] * 11, start=38.)
    assert_conservative(rows, passed_at, off)


def test_the_opposing_roads_mapped_speed_and_lanes_come_with_the_maneuver():
  # the unprotected left's R_req is from the opposing road's mapped limit (navInstruction maneuverOpposingSpeed, m/s, 0
  # unknown) and its lanes (0 unknown), for the maneuver and for each of allManeuvers
  from openpilot.selfdrive.turnpilot.route_follower import maneuver_from_ahead, maneuver_from_instruction
  instr = nav_instruction(100., "turn", "left", junction=(0., 270., 2, 'present', 11.18, 2),
                          ahead=[(300., "turn", "right", (270., 0., 1, 'present', 15.6, 1))])
  m = maneuver_from_instruction(instr)
  assert abs(m.opposing_speed - 11.18) < 1e-3 and m.opposing_lanes == 2
  a = maneuver_from_ahead(instr.allManeuvers[1])
  assert abs(a.opposing_speed - 15.6) < 1e-3 and a.opposing_lanes == 1
  m = maneuver_from_instruction(nav_instruction(100., "turn", "left", junction=(0., 270., 2, 'present')))
  assert m.opposing_speed == 0. and m.opposing_lanes == 0
  # and its straight span from our approach (maneuverOpposingStraightM, 0 unknown: skeptic G4's curve check)
  assert m.opposing_straight_m == 0.
  instr = nav_instruction(100., "turn", "left", junction=(0., 270., 2, 'present', 11.18, 2, 175.),
                          ahead=[(300., "turn", "right", (270., 0., 1, 'present', 15.6, 1, 50.))])
  assert maneuver_from_instruction(instr).opposing_straight_m == 175. and maneuver_from_ahead(instr.allManeuvers[1]).opposing_straight_m == 50.


# 1e seg 8, 26-38 s (2026-11-06, after dark; verified in the log): navd's readings of a right (bearings 91 deg, 3 branches,
# an opposing approach) at 1 Hz, routeCount 10 throughout, and the car's speed at each; None is the one invalid message
RIGHT_91 = (0., 91., 3, 'present')
BLIP_1E = [(26.39, 17.2, 0.22), (27.39, 16.3, 1.18), (28.39, 14.7, 2.32), (29.39, 12.2, 2.73), (30.39, 5.4, 2.83),
           (31.39, 2.4, 2.96), (32.39, -0.6, 3.00), (33.39, -3.5, 3.03), (34.39, -6.3, 2.97), (35.39, None, 3.01),
           (36.40, -12.1, 3.81), (37.40, 927.4, 5.17), (38.39, 922.3, 6.23)]
CLEARED_1E = 32.61  # the right's desire cleared at its angle: no longer committed from here
NEXT_1E = (91., 167., 1, 'present')  # the step after it: a right of 76 deg off the exit road


def replay_blip(readings=BLIP_1E, committed_until=CLEARED_1E, route_count=10, pulsed_at=26.26):
  """The readings through RouteFollower and TurnSkip as turnpilotd runs them, 20 Hz between them (speed interpolated), the
  turn committed until committed_until, its desire out (DesireTiming.started, latched past its clear) from pulsed_at.
  Returns [(t, distance, maneuver_id, passed, upcoming, skipped, reason)]."""
  from openpilot.selfdrive.turnpilot.turn_skip import TurnSkip
  rf, skip, out = RouteFollower(), TurnSkip(), []
  t, right_id = readings[0][0], rf.maneuver_id + 1  # the right: the first maneuver
  for (t0, d0, v0), (t1, _, v1) in zip(readings, readings[1:] + [(readings[-1][0] + 1., None, readings[-1][2])], strict=True):
    first = True
    while t < t1 - 1e-6:
      v = v0 + (v1 - v0) * (t - t0) / (t1 - t0)
      instr = None
      if first:
        instr = nav_instruction(0. if d0 is None else d0, "turn", "right", valid=d0 is not None, route_count=route_count,
                                junction=None if d0 is None else (RIGHT_91 if d0 < 100. else NEXT_1E))
      rf.update(instr, first, d0 is not None, v, DT_MDL, committed=t < committed_until)
      state = skip.update(rf.upcoming, rf.distance, rf.maneuver_id, v, desire_out=t >= pulsed_at and rf.maneuver_id == right_id)
      out.append((t, rf.distance, rf.maneuver_id, rf.passed, rf.upcoming, state.skipped, skip.reason))
      first = False
      t += DT_MDL
  return out


def test_a_navd_blip_after_a_finished_right_never_makes_it_a_new_maneuver():
  """1e 8:36.47: the right was done (its desire cleared at 32.61 s, so no longer committed) when navd sent one invalid
  message for the same route (35.39 s, off its step for a fix with the car on the exit road); the follower reset on it, the
  valid one after it for the same step at -12.1 m became a new maneuver behind the car, and it was skipped tooLate: 'Too Fast
  to Turn / Rerouting' for a turn just made. An invalid message for the same route with the car at or past the junction is a
  gap: dead-reckoned under the same id, the -12.1 m reading one more of the junction's, the right passed at PASSED_DIST_M,
  and the 927 m step after it a new maneuver, passed once."""
  out = replay_blip()
  right_id = next(mid for t, d, mid, *_ in out if t > 27.)
  on_right = [row for row in out if row[0] < 37.39]
  assert all(mid == right_id for _, _, mid, *_ in on_right), "the right's id bumped"
  assert not any(skipped for *_, skipped, _ in out), [row for row in out if row[5]][:1]
  # passed once: past the -12.1 m reading, by dead reckoning PASSED_DIST_M past its furthest reading or at navd's move on
  passes = [(t, d) for t, d, _, passed, *_ in out if passed]
  assert len(passes) == 1 and passes[0][0] <= 37.45, passes
  after = [row for row in out if row[0] > 37.45]
  assert all(mid == right_id + 1 and up is not None and d > 900. for _, d, mid, _, up, *_ in after)
  # the blip at the same place on the approach (12 m short, not committed): a drop, as before (the route may be gone)
  early = [(t, d, v) for t, d, v in BLIP_1E[:3]] + [(29.39, None, 2.73), (30.39, 5.4, 2.83), (31.39, 2.4, 2.96)]
  out = replay_blip(early, committed_until=0., pulsed_at=100.)
  assert any(up is None for t, _, _, _, up, *_ in out if 29.39 <= t < 30.39)


PASSED_M = 15.  # route_follower.PASSED_DIST_M


def test_a_new_route_whose_first_step_is_behind_the_car_is_passed_from_the_start():
  """1e 9:43.5: a reroute arrived 4-7 s late and its first step, a right, read 4.8 m behind the car; it was skipped tooLate.
  A turn or keep whose first reading is behind the car is passed from the start: upcoming None (no pulse, no skip, no skip
  mark for navd), and navd's next step is the next maneuver."""
  from openpilot.selfdrive.turnpilot.turn_skip import TurnSkip
  rf, skip = RouteFollower(), TurnSkip()
  rf.update(nav_instruction(300., "turn", "left", junction=(0., 270., 2, 'present'), route_count=3), True, True, 8., DT_MDL)
  rf.update(nav_instruction(0., valid=False, route_count=4), True, False, 8., DT_MDL)  # navd's separator before the reroute
  rf.update(nav_instruction(-4.8, "turn", "right", junction=RIGHT_91, route_count=4), True, True, 8., DT_MDL)
  first = rf.maneuver_id
  assert rf.valid and rf.upcoming is None and not rf.passed
  for i in range(int(2. / DT_MDL)):
    instr = nav_instruction(-4.8 - 8. * (i + 1) * DT_MDL, "turn", "right", junction=RIGHT_91, route_count=4) if i % 20 == 19 else None
    rf.update(instr, instr is not None, True, 8., DT_MDL)
    assert rf.upcoming is None and not rf.passed and rf.maneuver_id == first
    assert not skip.update(rf.upcoming, rf.distance, rf.maneuver_id, 8., False).skipped
  # navd moves on to the next step: a maneuver as ever, upcoming
  rf.update(nav_instruction(400., "turn", "left", junction=(91., 0., 2, 'present'), route_count=4), True, True, 8., DT_MDL)
  assert rf.maneuver_id == first + 1 and rf.upcoming is not None and not rf.passed
  # a NONE step behind the car (a road going on) isn't held back: the turn after it is looked ahead to as ever
  rf = RouteFollower()
  rf.update(nav_instruction(-3., "continue", "straight", ahead=[(40., "turn", "left", (0., 270., 2, 'present'))]), True, True, 8., DT_MDL)
  assert rf.upcoming is not None and rf.ahead is not None


def test_an_invalid_message_with_a_new_route_count_still_resets():
  """The gap rule is for this route's messages only: navd's separator between two routes carries the new routeCount, so a
  turn past its junction still gives way to the new route (its maneuver replaced), as does one not yet reached."""
  for d in (-6.3, 40.):
    rf = RouteFollower()
    rf.update(nav_instruction(d + 3., "turn", "right", junction=RIGHT_91, route_count=10), True, True, 3., DT_MDL)
    for _ in range(19):
      rf.update(None, False, True, 3., DT_MDL)
    first = rf.maneuver_id
    rf.update(nav_instruction(0., valid=False, route_count=11), True, False, 3., DT_MDL)
    assert rf.maneuver_id != first and rf.replaced and rf.upcoming is None and not rf.valid, d
  # the same route's invalid messages past the junction: a gap until NAV_STALE_S, then the route is lost as before
  from openpilot.selfdrive.turnpilot.route_follower import NAV_STALE_S
  rf = RouteFollower()
  rf.update(nav_instruction(2., "turn", "right", junction=RIGHT_91, route_count=10), True, True, 3., DT_MDL)
  for _ in range(20):  # dead-reckoned to -1 m: at the junction, its turn made (not committed: its desire cleared)
    rf.update(None, False, True, 3., DT_MDL)
  first, lost_at = rf.maneuver_id, None
  for i in range(int(6. / DT_MDL)):
    rf.update(nav_instruction(0., valid=False, route_count=10) if i % 20 == 0 else None, i % 20 == 0, False, 0.5, DT_MDL)
    if not rf.valid and lost_at is None:
      lost_at = 1. + (i + 1) * DT_MDL  # since the last valid message
    if lost_at is None:
      assert rf.maneuver_id == first and rf.upcoming is not None
  assert lost_at is not None and abs(lost_at - NAV_STALE_S) < 2 * DT_MDL, lost_at


def test_a_slight_left_across_oncoming_traffic_is_a_turn():
  """1e seg 5 (~23.4 s: 29 deg, an opposing approach, 11.1 m/s, 3 lanes) and 1d seg 9 (~6.5 s: 38 deg, opposing present):
  "slight left"s at junctions were keeps, refused at night (keepNoLine), driven straight, 'Fork Missed' and a reroute into a
  left; a keepLeft there would have crossed the oncoming lanes with no hold and no gap check. A slight turn to the far side
  of SLIGHT_TURN_MIN_DEG or more at a junction with an opposing approach present is a TURN, an unprotected left
  (map_controls.unprotected_turn); a true fork under 25 deg, a slight right, an off-ramp slight left, and a slight left with
  no opposing approach (or one not known) stay keeps. Mirrored where traffic keeps left."""
  from openpilot.selfdrive.turnpilot.map_controls import unprotected_turn
  from openpilot.selfdrive.turnpilot.route_follower import SLIGHT_TURN_MIN_DEG, make_maneuver, maneuver_from_ahead, \
    maneuver_from_instruction

  def m(mtype, modifier, after, opposing='present', branches=2, rht=True, geometry=True):
    return make_maneuver(mtype, modifier, (), geometry, 0., after, branches, opposing, rht)
  for angle in (29., 38., SLIGHT_TURN_MIN_DEG, 44.):
    left = m("turn", "slight left", 360. - angle)
    assert (left.kind, left.side, left.angle_deg) == (TURN, LEFT, angle) and unprotected_turn(left, True), angle
    assert m("end of road", "slight left", 360. - angle).kind == TURN, angle
  for keep in (m("turn", "slight left", 345.), m("turn", "slight left", 360. - SLIGHT_TURN_MIN_DEG + 1.),  # under 25 deg
               m("turn", "slight right", 30.), m("turn", "slight right", 38.),                              # the curb side
               m("off ramp", "slight left", 330.), m("fork", "slight left", 320.), m("on ramp", "slight left", 330.),
               m("turn", "slight left", 330., opposing='none'), m("turn", "slight left", 330., opposing='unknown'),
               m("turn", "slight left", 30.),                       # its bearings turn the other way: no far-side turn
               m("turn", "slight left", 330., geometry=False)):     # no junction data: the modifier alone
    assert keep.kind == KEEP and not unprotected_turn(keep, True), keep
  assert m("turn", "slight left", 330., branches=0).kind == NONE  # a bend, as ever
  # traffic keeping left: the slight right is the far side's
  assert m("turn", "slight right", 30., rht=False).kind == TURN and m("turn", "slight left", 330., rht=False).kind == KEEP
  # the same from navd's message and its allManeuvers, and through the follower (right_hand_traffic as it is given)
  instr = nav_instruction(100., "turn", "slight left", junction=(0., 331., 2, 'present'),
                          ahead=[(300., "turn", "slight left", (331., 293., 1, 'present'))])
  assert maneuver_from_instruction(instr).kind == TURN and maneuver_from_ahead(instr.allManeuvers[1]).kind == TURN
  assert maneuver_from_instruction(instr, right_hand_traffic=False).kind == KEEP
  rf = RouteFollower()
  rf.update(instr, True, True, 11.1, DT_MDL)
  assert rf.upcoming.kind == TURN
  rf = RouteFollower()
  rf.update(instr, True, True, 11.1, DT_MDL, right_hand_traffic=False)
  assert rf.upcoming.kind == KEEP


def test_the_latest_reading_of_the_junction_runs_down_with_the_car_between_the_nearest_and_the_furthest():
  """RouteFollower.junction_latest (2026-11-06): navd's latest CONFIRMED reading of the maneuver's junction, dead-reckoned, the
  deadline a curb-side turn's pulse goes by at the latest (desire_timing.pulse_distance) when the furthest holds a reading from
  far out long past the junction (1e 8:32: the furthest 8.8 m out with the nearest at the junction). A reading counts once it
  agrees with the one before it (dead-reckoned) within LATEST_AGREE_M; one that doesn't leaves the last confirmed one,
  dead-reckoned, until the next agrees with it: one outlier 9 m short about 20 m out was the latest until navd's next true
  reading and brought a right's pulse out 14.8 m from its junction. Always between the nearest and the furthest; it follows a
  committed turn through a gap, takes no reading the held span rejects, and starts again with a new maneuver's readings."""
  from openpilot.selfdrive.turnpilot.route_follower import LATEST_AGREE_M
  J = 'turn', 'right'
  step = 3. * DT_MDL
  rf = RouteFollower()
  rf.update(nav_instruction(70., *J), True, True, 3., DT_MDL)  # a reading 9 m long from far out: a span's first is its own
  assert rf.junction_latest == rf.distance == 70.
  for _ in range(20):
    rf.update(None, False, True, 3., DT_MDL)
  assert abs(rf.junction_latest - 67.) < 1e-6  # ...dead-reckoned on
  rf.update(nav_instruction(58., *J), True, True, 3., DT_MDL)  # the next one true, 9 m in from it: not confirmed yet
  assert rf.distance == 58. and abs(rf.junction_latest - (70. - 21 * step)) < 1e-6
  for _ in range(20):
    rf.update(None, False, True, 3., DT_MDL)
  rf.update(nav_instruction(55., *J), True, True, 3., DT_MDL)  # the next agrees with it: confirmed
  assert rf.junction_latest == 55. and abs(rf.distance + rf.junction_spread - (70. - 42 * step)) < 1e-6
  for _ in range(20):
    rf.update(None, False, True, 3., DT_MDL)
  rf.update(nav_instruction(56., *J), True, True, 3., DT_MDL)  # GPS noise long, within LATEST_AGREE_M: the latest, not the nearest
  assert 56. - (55. - 21 * step) < LATEST_AGREE_M
  assert rf.distance < rf.junction_latest == 56. < rf.distance + rf.junction_spread
  # committed: a gap (an invalid message) leaves it dead-reckoned on, under the same id
  mid = rf.maneuver_id
  rf.update(nav_instruction(0., valid=False), True, False, 3., DT_MDL, committed=True)
  assert rf.maneuver_id == mid and abs(rf.junction_latest - (56. - step)) < 1e-6
  # a reading the held span takes no notice of (another junction, beyond SAME_TURN_M of its readings) is not the latest
  rf.update(nav_instruction(140., *J), True, True, 3., DT_MDL, committed=True)
  assert rf.maneuver_id == mid and rf.junction_latest < 56.
  # one outlier either way never moves it: 9 m short, then the truth again (9 m from the outlier's own dead reckoning)
  for off in (-9., 9.):
    rf = RouteFollower()
    rf.update(nav_instruction(30., *J), True, True, 3., DT_MDL)
    for _ in range(20):
      rf.update(None, False, True, 3., DT_MDL)
    rf.update(nav_instruction(27. - step + off, *J), True, True, 3., DT_MDL)  # the outlier
    assert abs(rf.junction_latest - (30. - 21 * step)) < 1e-6, off  # the last confirmed one, dead-reckoned
    for _ in range(20):
      rf.update(None, False, True, 3., DT_MDL)
    true = 30. - 42 * step
    rf.update(nav_instruction(true, *J), True, True, 3., DT_MDL)  # the truth: disagrees with the outlier's dead reckoning
    assert abs(rf.junction_latest - true) < 1e-4, off  # ...and the confirmed one was the truth all along
    for _ in range(20):
      rf.update(None, False, True, 3., DT_MDL)
    rf.update(nav_instruction(true - 21 * step, *J), True, True, 3., DT_MDL)
    assert abs(rf.junction_latest - (true - 21 * step)) < 1e-4, off  # confirmed again (navd's distance is a Float32)
  # two that agree on a junction 9 m short (a bias that lasts) are the latest
  rf = RouteFollower()
  rf.update(nav_instruction(30., *J), True, True, 3., DT_MDL)
  for k in (1, 2):
    for _ in range(20):
      rf.update(None, False, True, 3., DT_MDL)
    rf.update(nav_instruction(30. - 21 * k * step - 9., *J), True, True, 3., DT_MDL)
  assert abs(rf.junction_latest - (30. - 42 * step - 9.)) < 1e-4
  # the next maneuver's first reading is the latest again
  rf = RouteFollower()
  rf.update(nav_instruction(20., *J), True, True, 3., DT_MDL)
  mid0 = rf.maneuver_id
  rf.update(nav_instruction(300., 'turn', 'left'), True, True, 3., DT_MDL)
  assert rf.maneuver_id == mid0 + 1 and rf.junction_latest == rf.distance == 300. and rf.junction_spread == 0.
  # no route: the distance (inf)
  assert RouteFollower().junction_latest == RouteFollower().distance


def in_hand_at(d=30., route_count=5, junction=RIGHT_91):
  """A follower on a right d m out (navd's route route_count), as turnpilotd has it in hand (junction_hold)."""
  rf = RouteFollower()
  rf.update(nav_instruction(d, "turn", "right", junction=junction, route_count=route_count), True, True, 3., DT_MDL,
            junction_hold=True)
  return rf


def hold_frames(rf, seconds, instr=None, valid=True, v=3., every=1., hold=True):
  """seconds of frames at v m/s, navd sending instr(rf) (None: nothing) every `every` s, junction_hold as given."""
  n = int(round(every / DT_MDL))
  for i in range(int(round(seconds / DT_MDL))):
    send = instr is not None and i % n == n - 1
    rf.update(instr(rf) if send else None, send, valid, v, DT_MDL, junction_hold=hold)


def test_a_turn_in_hand_is_held_through_gaps_in_its_route():
  """route_follower's junction_hold (2026-11-06): a curb-side turn turnpilotd has in hand, its one pulse not out until 8 m
  (where at HEAD its 40 m pulse had it committed), is held as a committed turn through an invalid run of its route and navd
  going quiet past NAV_STALE_S, dead-reckoned under the same id; without the hold it is dropped as before."""
  from openpilot.selfdrive.turnpilot.route_follower import NAV_STALE_S
  for hold in (True, False):
    rf = in_hand_at()
    mid = rf.maneuver_id
    # navd off its step for a fix, rerouting for the same destination: the same route's invalid messages for 4 s
    hold_frames(rf, NAV_STALE_S + 1., lambda rf: nav_instruction(0., valid=False, route_count=5), valid=False, hold=hold)
    if hold:
      assert rf.maneuver_id == mid and rf.upcoming is not None and abs(rf.distance - (30. - 3. * (NAV_STALE_S + 1.))) < 0.1
    else:
      assert rf.maneuver_id != mid and rf.upcoming is None and rf.replaced is False
    # navd quiet past NAV_STALE_S
    rf = in_hand_at()
    hold_frames(rf, NAV_STALE_S + 1., hold=hold)
    assert (rf.maneuver_id == mid and rf.upcoming is not None) is hold, hold
  # held to its pass, then navd's next update taken up
  rf = in_hand_at(10.)
  mid = rf.maneuver_id
  hold_frames(rf, 9., lambda rf: nav_instruction(0., valid=False, route_count=5), valid=False)
  assert rf.upcoming is None and rf.maneuver_id == mid  # passed: 15 m past its furthest reading
  rf.update(nav_instruction(300., "turn", "left", junction=(90., 0., 2, 'present'), route_count=5), True, True, 3., DT_MDL,
            junction_hold=False)
  assert rf.maneuver_id == mid + 1 and rf.upcoming.side == LEFT


def test_a_turn_in_hand_is_held_when_navd_moves_on_early_or_jumps():
  """navd on another step of the same route while a curb-side turn is in hand: moved on early to the next step (other
  bearings: a sharp turn's next step up to 10 m before its end; without bearings, the next right 50 m on, beyond SAME_TURN_M of
  its readings), or a jump up from close in to beyond every reading of it: a gap, the turn held under its id, its junction where
  it was, as a committed one is. Not held, the same moves dropped it (2026-11-06 review: a right with no junction data passed
  2.9 m short of it, unpulsed, nothing said)."""
  # the next step, a left 60 m on (other bearings), 12 m before the right
  for hold in (True, False):
    rf = in_hand_at(12.)
    mid = rf.maneuver_id
    hold_frames(rf, 2., lambda rf: nav_instruction(rf.distance + 60., "turn", "left", junction=(91., 1., 2, 'present'), route_count=5),
                hold=hold)
    assert (rf.maneuver_id == mid and rf.upcoming.side == RIGHT and abs(rf.distance - 6.) < 0.1) is hold, hold
  # no junction data: the next right 50 m on, from 14 m before the first (the 2026-11-06 review's red case)
  for hold in (True, False):
    rf = in_hand_at(14., junction=None)
    mid = rf.maneuver_id
    hold_frames(rf, 3.5, lambda rf: nav_instruction(rf.distance + 50., "turn", "right", route_count=5), hold=hold)
    if hold:
      assert rf.maneuver_id == mid and abs(rf.distance - 3.5) < 0.1
  # a jump up from inside ADVANCE_NEAR_M to beyond every reading by more than NEW_MANEUVER_JUMP_M, the same step: held
  rf = in_hand_at(10.)
  mid = rf.maneuver_id
  hold_frames(rf, 2.)
  hold_frames(rf, 1., lambda rf: nav_instruction(rf.distance + 30., "turn", "right", junction=RIGHT_91, route_count=5))
  assert rf.maneuver_id == mid and abs(rf.distance - 1.) < 0.1


def test_a_turn_in_hand_is_dropped_for_a_new_route_that_doesnt_turn_there_and_kept_for_one_that_does():
  """Not as a committed turn (2026-11-06, why committed=True was refused for a turn whose pulse hasn't gone): a new route
  (a reroute, a destination changed from the phone) that doesn't turn at its junction is taken, the turn dropped (replaced) and
  its cap with it, never a gap held to the pass that would cap and pulse a right 35 m out on a dead route; one that does turn
  there carries the junction's readings over (a new id, as for any turn). navd's separator between the two routes (an invalid
  message with the new routeCount) is held through, the new route's first valid update deciding."""
  for separator in (True, False):
    rf = in_hand_at(35.)
    mid = rf.maneuver_id
    if separator:
      rf.update(nav_instruction(0., valid=False, route_count=6), True, False, 3., DT_MDL, junction_hold=True)
      assert rf.maneuver_id == mid and rf.upcoming is not None and not rf.replaced, separator  # held through it
    rf.update(nav_instruction(400., "turn", "left", junction=(0., 270., 2, 'present'), route_count=6), True, True, 3., DT_MDL,
              junction_hold=True)
    assert rf.maneuver_id == mid + 1 and rf.replaced and rf.upcoming.side == LEFT and rf.distance == 400., separator
    # one that turns there: the same junction, its readings carried over
    rf = in_hand_at(35.)
    hold_frames(rf, 1., lambda rf: nav_instruction(rf.distance + 9., "turn", "right", junction=RIGHT_91, route_count=5))
    mid, far = rf.maneuver_id, rf.distance + rf.junction_spread
    if separator:
      rf.update(nav_instruction(0., valid=False, route_count=6), True, False, 3., DT_MDL, junction_hold=True)
    rf.update(nav_instruction(rf.distance - 4., "turn", "right", junction=RIGHT_91, route_count=6), True, True, 3., DT_MDL,
              junction_hold=True)
    assert rf.maneuver_id == mid + 1 and rf.replaced and rf.upcoming.side == RIGHT, separator
    assert abs(rf.distance + rf.junction_spread - (far - (2 if separator else 1) * 3. * DT_MDL)) < 1e-6, separator  # carried


def test_a_turn_in_hand_is_still_rebaselined_past_one_short_outlier():
  """The follower's ADVANCE_NEAR_M re-baselining is kept for a turn in hand (2026-11-06: a held span forces it off, and the
  rights fix relies on it): one reading 17 m short has the nearest inside ADVANCE_NEAR_M with the junction 21 m out, and the
  next true reading, a jump up its other readings bear out (within NEW_MANEUVER_JUMP_M of the furthest), is a new maneuver at
  the same junction, its readings fresh: never the short reading held as the junction to the end."""
  rf = in_hand_at(24.)
  mid = rf.maneuver_id
  hold_frames(rf, 1., lambda rf: nav_instruction(rf.distance + rf.junction_spread - 3. * DT_MDL - 17., "turn", "right",
                                                 junction=RIGHT_91, route_count=5))
  assert rf.maneuver_id == mid and rf.distance < 5. and rf.distance + rf.junction_spread > 20.
  true = rf.distance + rf.junction_spread
  hold_frames(rf, 1., lambda rf: nav_instruction(true - 3., "turn", "right", junction=RIGHT_91, route_count=5))
  assert rf.maneuver_id == mid + 1 and rf.upcoming.side == RIGHT and abs(rf.distance - (true - 3.)) < 1e-4 and rf.junction_spread == 0.


def test_a_reroutes_same_turn_carrying_a_short_outlier_as_its_nearest_is_not_passed_in_silence():
  """A new route whose first maneuver is the same junction (its bearings, within SAME_TURN_M of the dead-reckoned one) carries
  the old span over, and one short outlier the nearest kept below zero then made route_follower pass that turn from the start
  (maneuver_behind): uncommitted (a right is until its 8 m pulse), it got no pulse, skip, alert or skip mark. Behind is by the
  new maneuver's own reading and the furthest of its span: here neither is, so it stays the maneuver. Its readings split across
  the junction by the nearest, one short outlier or not (the reroute's own reading ahead or behind), no first pulse can go
  (desire_timing's guard), and turn_skip skips it out loud at once (2026-11-06 review: a curb-side turn is past its junction by
  any one reading, since a lone short reading may be the true one inside a lasting long bias; route_follower module
  docstring). A fresh route's first step behind by every reading (1e 9:43) is still passed from the start
  (test_a_new_route_whose_first_step_is_behind_the_car_is_passed_from_the_start)."""
  from openpilot.selfdrive.turnpilot.turn_skip import TurnSkip
  for reroute_reading in (5., -2.):
    rf = RouteFollower()
    rf.update(nav_instruction(14., "turn", "right", junction=RIGHT_91, route_count=3), True, True, 3., DT_MDL)
    for _ in range(19):
      rf.update(None, False, True, 3., DT_MDL)
    rf.update(nav_instruction(-1., "turn", "right", junction=RIGHT_91, route_count=3), True, True, 3., DT_MDL)  # 12 m short
    for _ in range(19):
      rf.update(None, False, True, 3., DT_MDL)
    old = rf.maneuver_id
    assert rf._span.near < 0. < rf._span.far
    # navd's reroute onto the road the car is on, its first maneuver this right (no separator seen: conflated)
    rf.update(nav_instruction(reroute_reading, "turn", "right", junction=RIGHT_91, route_count=4), True, True, 3., DT_MDL)
    assert rf.maneuver_id == old + 1 and rf.replaced and rf._span.near < 0. < rf._span.far, reroute_reading  # carried over
    assert rf.upcoming is not None and not rf.passed, reroute_reading  # not passed from the start
    skip = TurnSkip()
    state = skip.update(rf.upcoming, rf.distance, rf.maneuver_id, 3., False, far=rf.distance + rf.junction_spread,
                        last=rf.junction_latest)
    assert state.skipped and skip.reason == 'tooLate', reroute_reading  # by the nearest: said, never in silence
  # first seen behind by every reading, its span fresh: passed from the start as ever, never skipped (the 9:43 phantom)
  rf = RouteFollower()
  rf.update(nav_instruction(300., "turn", "left", junction=(0., 270., 2, 'present'), route_count=3), True, True, 8., DT_MDL)
  rf.update(nav_instruction(-4.8, "turn", "right", junction=RIGHT_91, route_count=4), True, True, 8., DT_MDL)
  assert rf.upcoming is None and rf._span.far < 0.
  # a turn first seen split that way with openpilot not steering (the driver's), or with its desire out: nothing, as before
  from openpilot.selfdrive.turnpilot.route_follower import maneuver_from_instruction
  right = maneuver_from_instruction(nav_instruction(40., "turn", "right", junction=RIGHT_91))
  assert not TurnSkip().update(right, -4., 1, 3., False, lat_active=False, far=8.).skipped
  assert not TurnSkip().update(right, -4., 1, 3., True, far=8.).skipped


def test_an_invalid_message_past_a_bend_or_the_arrival_is_a_drop_not_a_gap():
  """The same route's invalid message with the car at or past its maneuver's junction is a gap for a TURN or a KEEP (1e 8:36:
  a right just made, navd off its step for a fix). Past a bend step (NONE) or at the arrival it is a drop, as before the gap:
  dead-reckoned on through a route the user cancelled just past a bend, the follower would move on to the turn after the bend
  (_move_on) NAVD_ADVANCE_M past it, and cap and pulse that turn for a route that no longer exists."""
  from openpilot.selfdrive.turnpilot.route_follower import NAV_STALE_S
  # a bend step with a right 40 m past it, the car 3 m past the bend's point when the route is cancelled
  rf = RouteFollower()
  rf.update(nav_instruction(3., "continue", "straight", route_count=7, ahead=[(43., "turn", "right", RIGHT_91)]), True, True, 3., DT_MDL)
  assert rf.maneuver.kind == NONE and rf.ahead is not None and rf.ahead.kind == TURN
  for _ in range(40):
    rf.update(None, False, True, 3., DT_MDL)
  assert rf.distance < 0. and rf.valid
  first = rf.maneuver_id
  rf.update(nav_instruction(0., valid=False, route_count=7), True, False, 3., DT_MDL)
  assert not rf.valid and rf.upcoming is None and rf.ahead is None and rf.replaced and rf.maneuver_id == first + 1
  for _ in range(int((NAV_STALE_S + 1.) / DT_MDL)):  # nothing promoted or capped while navd says nothing
    rf.update(None, False, True, 3., DT_MDL)
    assert rf.upcoming is None and rf.ahead is None
  # the arrival, the car 2 m past it
  rf = RouteFollower()
  rf.update(nav_instruction(4., "arrive", "", route_count=7), True, True, 3., DT_MDL)
  assert rf.maneuver.kind == ARRIVE
  for _ in range(40):
    rf.update(None, False, True, 3., DT_MDL)
  rf.update(nav_instruction(0., valid=False, route_count=7), True, False, 3., DT_MDL)
  assert not rf.valid and rf.upcoming is None
  # a keep and a turn just passed: a gap, the same maneuver dead-reckoned on (test_a_navd_blip_after_a_finished_right...)
  for instr in (nav_instruction(3., "fork", "slight left", junction=(0., 340., 1, 'none'), route_count=7),
                nav_instruction(3., "turn", "right", junction=RIGHT_91, route_count=7)):
    rf = RouteFollower()
    rf.update(instr, True, True, 3., DT_MDL)
    assert rf.maneuver.kind in (KEEP, TURN)
    for _ in range(40):
      rf.update(None, False, True, 3., DT_MDL)
    first = rf.maneuver_id
    rf.update(nav_instruction(0., valid=False, route_count=7), True, False, 3., DT_MDL)
    assert rf.valid and rf.maneuver_id == first and not rf.replaced, instr.maneuverType


def test_a_slight_left_across_a_present_approach_is_an_unprotected_turn_whatever_that_roads_bearing():
  """The 11-06 rights review put a cone on the opposing road's bearing off straight on: a road further than 25 deg off the
  other side was taken for a side road past a main road bending left, and the slight left kept a keep. navd sends the
  bearing since (maneuverOpposingRelBearing, opposingRelBearing), and on the local tiles around the 11-06 drives the cone
  made keeps of 6 of 66 such TURNs, 5 of them across the approach's own road going on 26-43 deg off the far side at the
  approach's own mapped speed: a keep across its oncoming lanes, with no hold line and no gap check. None was the bend,
  which navd gives as 'continue' (NONE). So the bearing decides nothing (2026-11-07): the slight left 30-50 deg off a through
  road whose continuation leaves 30-40 deg right is a TURN at the approach's speed and with no speed known, on navd's message
  and on the look-ahead past a bend step (allManeuvers[1]); the bend itself stays the model's. Mirrored where traffic keeps
  left. Under SLIGHT_TURN_MIN_DEG, or with nothing present, it stays a keep as before."""
  import math
  from openpilot.selfdrive.turnpilot.route_follower import make_maneuver, maneuver_from_ahead, maneuver_from_instruction

  def m(rel, after=332., modifier="slight left", rht=True, mtype="turn", opposing='present'):
    return make_maneuver(mtype, modifier, (), True, 0., after, 2, opposing, rht, rel)
  for rel in (None, 0., 10., 25., -25., -35., -44., 26., 30., 35., 40., 44.):  # straight on, either side, the old cone's out
    left = m(rel)
    assert (left.kind, left.side, left.opposing_rel_deg) == (TURN, LEFT, rel), rel
    assert m(rel, mtype="end of road").kind == TURN, rel  # no road of the car's own goes on: never a keep either
  assert m(None, after=338.).kind == KEEP and m(40., after=338.).kind == KEEP  # 22 deg: under SLIGHT_TURN_MIN_DEG
  assert m(40., opposing='none').kind == KEEP and m(40., opposing='unknown').kind == KEEP  # nothing oncoming to cross
  assert m(40., mtype="continue").kind == NONE and m(40., mtype="fork").kind == KEEP  # the bend the model's, a fork a keep
  # traffic keeping left: the slight right, its far-side road off to the left whatever its bearing
  for rel in (-40., 0., 35., None):
    assert m(rel, after=28., modifier="slight right", rht=False).kind == TURN, rel
  assert m(-40., after=28., modifier="slight right").kind == KEEP  # where traffic keeps right a slight right is the curb side
  limit = 13.41  # m/s, 30 mph: a through road's, the approach's and the far-side road's alike
  for angle, rel, speed in ((35., 35., limit), (50., 30., limit), (30., 40., limit), (35., 35., 0.), (50., 40., 0.)):
    junction = (0., 360. - angle, 2, 'present', speed, 2, 0., rel)
    ni = nav_instruction(100., "turn", "slight left", junction=junction, speed_limit=limit if speed else 0.)
    here = maneuver_from_instruction(ni)
    assert (here.kind, here.side, here.opposing_rel_deg) == (TURN, LEFT, rel) and abs(here.opposing_speed - speed) < 1e-3, \
      (angle, rel, speed)
    rf = RouteFollower()
    rf.update(ni, True, True, 11.1, DT_MDL)
    assert rf.upcoming.kind == TURN and rf.upcoming.opposing_rel_deg == rel, (angle, rel, speed)
    # the same junction past a bend step first (no other road there: NONE), the turn RouteFollower looks ahead to
    bend = nav_instruction(60., "turn", "slight left", junction=(0., 330., 0), speed_limit=limit if speed else 0.,
                           ahead=[(160., "turn", "slight left", junction)])
    assert maneuver_from_ahead(bend.allManeuvers[1]).kind == TURN, (angle, rel, speed)
    rf = RouteFollower()
    rf.update(bend, True, True, 11.1, DT_MDL)
    assert rf.upcoming.kind == NONE and rf.upcoming.bend, (angle, rel, speed)
    assert rf.ahead is not None and (rf.ahead.kind, rf.ahead.side, rf.ahead.opposing_rel_deg) == (TURN, LEFT, rel), (angle, rel, speed)
  # not known (NaN: no junction data, an invalid message, a navd before the field): read None, and present alone decides
  ni = nav_instruction(100., "turn", "slight left", junction=(0., 325., 2, 'present'))
  assert math.isnan(ni.maneuverOpposingRelBearing) and maneuver_from_instruction(ni).opposing_rel_deg is None
  assert maneuver_from_instruction(ni).kind == TURN
