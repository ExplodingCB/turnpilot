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
