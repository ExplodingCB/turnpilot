"""A curb-side turn's go from a stand, closed loop under navd's readings as the 11-06 drives had them (2026-11-07 review, rights
R1). The go is the right's one pulse only within desire_timing.CURB_GO_SLACK_M of its pulse point by the reading that pulse
goes by (pulse_distance: the furthest, at the latest by navd's latest), never by the nearest alone. By the nearest, which
keeps one reading 3-5 m short to the junction (1d 5:02, 1e 7:15, measured against the driven arc), a go from a red line 10-12
m out, or from the walker's hold line (placed by the nearest: 8.25 by it), sent the right's only edge standing 10.3-13.0 m
from the junction: the early turn-in that clipped 1e's curbs. Now such a go sends nothing, the car pulls away on the model
(the Sim's car does), and the one pulse goes rolling about 8 m out. The cost, a stand the readings disagree about let go with
no desire: a model that holds the car without a fresh edge (BeamNG ped_crosswalk_right's) is handed back loudly by
turnpilotd's no-move clock for a curb-side go (curb_release), never left to turn in from 13 m. Every distance asserted here is
the TRUE one (the junction at J), never navd's reading."""
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import CURB_GO_SLACK_M, CURB_TURN_PULSE_M
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import T_LEFT, Sim, oracle_lights, plan_accel
from openpilot.selfdrive.turnpilot.turnpilotd import NO_MOVE_MAX_S, NO_MOVE_REASON, NO_MOVE_S

Desire = log.Desire
J = 400.  # the right's junction (true), s m
ROUTE = [(J, "turn", "right", junction_of("right")), (J + 1200., "turn", "left", T_LEFT)]
GO_OUT_M = CURB_TURN_PULSE_M + CURB_GO_SLACK_M  # no edge standing further out than this, by the true distance


def readings_1d():
  """navd's error (m, + long) on every reading as 1d 5:02's against its driven arc (the round-3 replay): 2 m long from more
  than 60 m out (the furthest keeps that), one reading 4.7 m short at 28 m or less (the nearest keeps it to the junction), the
  rest 2.5 m short (the latest). A function of the true distance to the junction."""
  hit = []

  def off(d):
    if d > 60.:
      return 2.
    if d <= 28. and not hit:
      hit.append(d)
      return -4.7
    return -2.5
  return off


def shifted(off, controls=False):
  """navd's whole message shifted by off(true distance), as a GPS position error along the road shifts every maneuver's
  distance; the stop line's too when controls (else the map's line stays where it is)."""
  def filt(sim, instr):
    o = off(J - sim.s)
    b = instr.as_builder()
    b.maneuverDistance = b.maneuverDistance + o
    b.distanceRemaining = b.distanceRemaining + o
    for m in b.allManeuvers:
      m.distance = m.distance + o
    if controls:
      for c in b.controls:
        c.distance = c.distance + o
    return b.as_reader(), True
  return filt


def holds_without_an_edge(stood):
  """The car's accel when the model holds a stand until a fresh turnRight edge (BeamNG ped_crosswalk_right: it didn't pull
  away on its own with no fresh edge), the plan's caps followed otherwise."""
  edge = []

  def accel(sim, plan):
    if stood and not edge:
      if plan.desire == Desire.turnRight:
        edge.append(sim.t)
      else:
        sim.model_holds = True
        return -1.
    sim.model_holds = False
    a = min(plan_accel(plan.accelCap, plan.speedCap, sim.v), (sim.cruise - sim.v) / 2., 1.5)
    return min(a, -1.) if plan.shouldStop else a
  return accel


def drive(sim, each, objects=None, until_edge=True, seconds=None):
  """Drive sim engaged, each(sim) before every frame, until the right's first turnRight edge (until_edge) or for seconds from
  the stand. Returns [(true distance at the frame, v, plan)]."""
  rows = []
  while J - sim.s > 2.:
    each(sim)
    d = J - sim.s
    p = sim.step(objects(sim) if objects else (), engaged=True)
    rows.append((d, sim.v, p))
    if until_edge and p.desire == Desire.turnRight:
      break
    if seconds is not None and sim.stood.get('t') is not None and sim.t - sim.stood['t'] > seconds:
      break
    assert sim.t < 150., (round(d, 2), round(sim.v, 2), p.reason)
  return rows


def at_a_red(x, off=None, controls=False, accel=None):
  """The right with a mapped signal x m before its junction, red in the sim's light oracle until the car has stood 3 s, then
  green. Returns the sim (sim.stood: where it stood, true and by the nearest)."""
  sim = Sim(ROUTE, controls=[(J - x, 'signal')])
  sim.go_taps, sim.tp.oracle_ok, sim.stood = False, True, {}
  light = {'state': 'red'}
  sim.lights = lambda sim: oracle_lights(light['state'], J - x)(sim)
  if off is not None:
    sim.nav_filter = shifted(off, controls)
  if accel is not None:
    sim.accel = accel(sim.stood)

  def each(sim):
    if sim.v < 0.05 and J - sim.s < x + 40. and 't' not in sim.stood:
      sim.stood.update(t=sim.t, d=J - sim.s, nearest=sim.tp.follower.distance)
    if 't' in sim.stood and sim.t - sim.stood['t'] > 3.:
      light['state'] = 'green'
  sim.each = each
  return sim


def at_the_walker(off=None, accel=None):
  """The right with a walker on its crosswalk (just past the junction, on the turn's side) seen on every detector run until
  the car has stood 4 s at its hold line (scene_policy.HOLD_LINE_M, placed by the nearest), then gone. Returns the sim."""
  sim = Sim(ROUTE)
  sim.go_taps, sim.stood, walker = False, {}, {'on': True}
  if off is not None:
    sim.nav_filter = shifted(off)
  if accel is not None:
    sim.accel = accel(sim.stood)

  def objects(sim):
    return ({'cls': 'pedestrian', 'x': J - sim.s + 3., 'y': -5., 'age': 5 + int(sim.t * 10.)},) if walker['on'] else ()

  def each(sim):
    if sim.v < 0.05 and J - sim.s < 40. and 't' not in sim.stood:
      sim.stood.update(t=sim.t, d=J - sim.s, nearest=sim.tp.follower.distance)
    if 't' in sim.stood and sim.t - sim.stood['t'] > 4. and walker['on']:
      walker['on'], sim.stood['go_t'] = False, sim.t  # gone: the hold's go is this frame's
  sim.each, sim.objects = each, objects
  return sim


def edges(rows):
  return [(d, v) for k, (d, v, p) in enumerate(rows) if p.desire == Desire.turnRight and (k == 0 or rows[k - 1][2].desire != Desire.turnRight)]


def assert_pulsed_rolling_at_8_m(sim, rows, case):
  """The stand further out than GO_OUT_M by the true distance with the nearest inside it (the finding's case), no edge
  standing there, and the right's one pulse rolling about CURB_TURN_PULSE_M out, nothing said."""
  assert sim.stood['d'] > GO_OUT_M and sim.stood['nearest'] < GO_OUT_M, (case, sim.stood)
  e = edges(rows)
  assert e and all(d <= GO_OUT_M for d, _ in e), (case, e)
  d, v = e[0]
  assert abs(d - CURB_TURN_PULSE_M) < 0.6 and v > 0.3, (case, e)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or p.tooFastToTurn for _, _, p in rows), case


def test_a_go_from_a_red_line_10_12_m_out_with_a_short_nearest_pulses_rolling_at_8_m():
  """1d 5:02's readings, the red line 10 and 12 m before the junction: the car stood 10.3 / 12.3 m out with the nearest at 5.6
  / 7.6 and pulsed standing there by it (2026-11-07 review); now the go sends nothing and the pulse goes rolling at 8.4-8.5
  m. The line shifted by the same GPS error as the readings (the stop 12.8 m out for a 10 m line) is the same."""
  for x, controls in ((10., False), (12., False), (10., True)):
    sim = at_a_red(x, readings_1d(), controls)
    rows = drive(sim, sim.each)
    assert_pulsed_rolling_at_8_m(sim, rows, (x, controls))


def test_a_go_from_the_walker_hold_with_a_short_nearest_pulses_rolling_at_8_m():
  """1d 5:02's readings: the walker's hold line, placed by the nearest (8.25 by it), stood the car 12.95 m from the junction,
  and its go pulsed the right there by the nearest (2026-11-07 review); now it sends nothing, the car pulls away on the model,
  and the one pulse goes rolling at 8.4 m."""
  sim = at_the_walker(readings_1d())
  rows = drive(sim, sim.each, sim.objects)
  assert_pulsed_rolling_at_8_m(sim, rows, 'walker')


def test_with_the_readings_agreeing_the_walker_holds_go_is_still_its_pulse():
  """The control (ped_crosswalk_right's premise): steady readings, the car stood at the walker's hold line 8.25 m out, and its
  go is the right's one pulse, at once, standing; and at a red line 10 m out the go sends nothing, the pulse rolling at 8 m."""
  sim = at_the_walker()
  rows = drive(sim, sim.each, sim.objects)
  e = edges(rows)
  assert abs(sim.stood['d'] - sim.stood['nearest']) < 0.1 and sim.stood['d'] < GO_OUT_M, sim.stood
  assert len(e) == 1 and e[0][1] < 0.3 and abs(e[0][0] - sim.stood['d']) < 0.1, (sim.stood, e)
  sim = at_a_red(10.)
  rows = drive(sim, sim.each)
  e = edges(rows)
  assert sim.stood['d'] > GO_OUT_M and e and abs(e[0][0] - CURB_TURN_PULSE_M) < 0.3 and e[0][1] > 0.3, (sim.stood, e)


def test_a_walker_holds_go_with_no_desire_the_model_doesnt_follow_is_handed_back_loudly():
  """The cost of the go by pulse_distance: the readings disagree at the walker's stand (1d 5:02's), so its go sends no desire,
  and a model that holds the car without a fresh edge (BeamNG ped_crosswalk_right's) leaves it standing. turnpilotd's no-move
  clock for a curb-side go from further out than its pulse (curb_release) hands back loudly, "Can't start the turn: take
  control" (cantStartTakeControl), at NO_MOVE_S, the car held; under TurnPilotAlwaysDrive (the car ships it on), with no
  hold, at NO_MOVE_MAX_S. No turnRight edge goes standing 13 m out, at any point."""
  for always_drive, limit in ((False, NO_MOVE_S), (True, NO_MOVE_MAX_S)):
    sim = at_the_walker(readings_1d(), accel=holds_without_an_edge)
    sim.tp.always_drive = always_drive
    times = []
    rows = drive(sim, lambda sim, times=times: (sim.each(sim), times.append(sim.t)), sim.objects, until_edge=False,
                 seconds=4. + limit + 1.)
    assert sim.stood['d'] > GO_OUT_M + 3. and sim.stood['nearest'] < GO_OUT_M, sim.stood
    go_k = next(k for k, t in enumerate(times) if t >= sim.stood['go_t'])
    assert not edges(rows) and max(v for _, v, _ in rows[go_k:]) < 0.1, always_drive
    handed = next(k for k, (_, _, p) in enumerate(rows) if str(p.controlPrompt) == 'cantStartTakeControl')
    assert abs((handed - go_k) * DT_MDL - limit) < 0.2 and rows[handed][2].reason == NO_MOVE_REASON, (always_drive, handed - go_k)
    assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for _, _, p in rows[:handed]), always_drive
