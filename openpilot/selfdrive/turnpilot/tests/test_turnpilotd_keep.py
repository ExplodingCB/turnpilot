"""turnpilotd's keeps (route_follower.KEEP: a slight turn at a junction, a fork, an off ramp) in closed loop, on the synthetic
route, model and scene of test_turnpilotd: a keep the model can't be sent is said and rerouted, never left to the model
in silence (BUILD_FINDINGS D2-keep), and none goes toward the oncoming side of the road (D5)."""
from unittest import mock

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import keep_lead, keep_last
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP

Desire = log.Desire
KEEP_J = 300.
BROKEN_WHITE, SOLID_YELLOW = LinePaint('broken', 'white', 0.9, 0.3), LinePaint('solid', 'yellow', 0.9, 0.95)
# S23's first step: turn, slight right, 22 deg, 1 branch (navd on twoway_route_pair), a keep (route_follower.classify)
SLIGHT_RIGHT = [(KEEP_J, "turn", "slight right", junction_of("slight right", branches=1, angle=22.)),
                (1500., "turn", "right", junction_of("right"))]


def keep_sim(maneuvers=SLIGHT_RIGHT, cruise=13., model_lanes=(0, 1), **road):
  sim = Sim(maneuvers, cruise=cruise, model_lanes=model_lanes)
  sim.model_kw = road
  return sim


def keep_events(event):
  return [c for c in event.call_args_list if c.args[0] == 'turnpilot.keep_not_sent']


def test_a_keep_the_model_sees_no_lane_line_for_is_said_and_rerouted():
  """BUILD_FINDINGS D2-keep (smoke S23 twoway_route_pair, both modes, and diag s23_twoway_avoid): laneLineProbs 0.01-0.11 the
  whole approach to the slight right at DR677_1, so no keepRight could go (the gate is right: a keep with no line beside the
  car is a swerve), and the route's maneuver was left to the model with no alert and no log. A keep still unsent at
  keep_last before its point is skipped out loud (tooFastToTurn, its reason in skipReason: selfdrived's skip chime, "Can't
  See Lanes", "Rerouting"; navd reroutes on it, or moves on if the model took the route's branch), logged once with the
  side and the line probs, and its cap kept to its pass: the car slowing for it never speeds up into it."""
  sim = keep_sim(line_prob=0.1)
  with mock.patch('openpilot.selfdrive.turnpilot.turnpilotd.cloudlog.event') as event:
    for _ in range(int(30. / DT_MDL)):
      sim.step()
      if sim.s > KEEP_J + 30.:
        break
  assert not any(p.desire == Desire.keepRight for _, _, p in sim.trace), "no keep with no line beside the car"
  said = [i for i, (_, _, p) in enumerate(sim.trace) if p.tooFastToTurn]
  assert said, "the keep that can't be sent is said"
  first_s, first_v, first = sim.trace[said[0]]
  # said at keep_last before the point (half of keep_lead), not as soon as it is inside keep_lead (a line may yet be seen)
  assert abs(first.maneuverDistance - keep_last(first_v)) < 1., (first.maneuverDistance, first_v)
  assert KEEP_J - first_s > 0. and str(first.skipReason) == 'keepNoLine'
  on_keep = [p for s, _, p in sim.trace if s >= first_s and str(p.maneuverModifier) == 'slight right' and p.routeValid]
  assert on_keep and all(p.tooFastToTurn and str(p.skipReason) == 'keepNoLine' for p in on_keep), "said until it is passed"
  # its cap (a town keep under KEEP_CAP_SPEED_MAX) stays to its pass: never lifted by the skip
  assert all(p.speedCap < NO_SPEED_CAP for p in on_keep if p.maneuverDistance > 0.)
  logged = keep_events(event)
  assert len(logged) == 1, logged
  kw = logged[0].kwargs
  assert kw['side'] == 'right' and kw['reason'] == 'keepNoLine' and kw['probs'] == [0.05, 0.1, 0.1, 0.1], kw  # synthetic.road: no lane beyond on the left


def test_a_keep_with_its_lane_line_goes_and_is_never_skipped():
  # the control: the right ego line seen with room beyond it, keepRight goes inside keep_lead, nothing said
  sim = keep_sim()
  for _ in range(int(30. / DT_MDL)):
    sim.step()
    if sim.s > KEEP_J + 30.:
      break
  edges = [s for s, _, p in sim.trace if p.desire == Desire.keepRight]
  assert edges and KEEP_J - keep_lead(sim.cruise) - 1. < edges[0] < KEEP_J - keep_last(sim.cruise)
  assert not any(p.tooFastToTurn for _, _, p in sim.trace)
  assert all(str(p.skipReason) == '' for _, _, p in sim.trace)


def test_a_line_seen_after_the_keep_was_said_never_sends_it_late():
  # said at keep_last, the keep is over for this maneuver: a line seen closer in never sends a keep after the alert said
  # it wouldn't (a late swerve)
  sim = keep_sim(line_prob=0.1)
  said_at = None
  for _ in range(int(30. / DT_MDL)):
    if said_at is not None and sim.s > said_at + 3.:
      sim.model_kw = {}  # the right line seen from here, room beyond it
    plan = sim.step()
    if plan.tooFastToTurn and said_at is None:
      said_at = sim.s
    if sim.s > KEEP_J + 30.:
      break
  assert said_at is not None
  assert not any(p.desire == Desire.keepRight for _, _, p in sim.trace)


def test_a_keep_driven_with_openpilot_not_steering_is_never_said():
  # no desire goes out with openpilot not steering (DesireTiming), so a keep the driver steers isn't skipped under them
  sim = keep_sim(line_prob=0.1)
  for _ in range(int(30. / DT_MDL)):
    sim.step(lat_active=False)
    if sim.s > KEEP_J + 30.:
      break
  assert not any(p.tooFastToTurn for _, _, p in sim.trace)


def test_an_off_ramp_with_no_line_is_said_too():
  # every keep, not only a city 'slight' turn: an off ramp right at 25 m/s with no right line seen is said the same way
  ramp = [(600., "off ramp", "slight right", junction_of("slight right", branches=1, angle=25.)), (3000., "turn", "right", junction_of("right"))]
  sim = keep_sim(maneuvers=ramp, cruise=25., line_prob=0.1)
  for _ in range(int(40. / DT_MDL)):
    sim.step()
    if sim.s > 630.:
      break
  assert not any(p.desire == Desire.keepRight for _, _, p in sim.trace)
  said = [p for _, _, p in sim.trace if p.tooFastToTurn]
  assert said and str(said[0].skipReason) == 'keepNoLine' and said[0].maneuverDistance > 0.


# S19's reroute step: turn, slight left, 14 deg (bearings 221 to 207), 1 branch, at a bend on a two-way road
SLIGHT_LEFT = [(KEEP_J, "turn", "slight left", junction_of("slight left", branches=1, angle=14.)),
               (1500., "turn", "right", junction_of("right"))]


def run_keep(sim, until):
  for _ in range(int(40. / DT_MDL)):
    sim.step()
    if sim.s > until:
      break
  return sim


def test_no_keep_toward_the_oncoming_side_of_an_undivided_road():
  """BUILD_FINDINGS D5 (smoke S19): the reroute's slight left on a two-way road went out as keepLeft at 19 m/s, the left ego
  line read with room beyond it (the oncoming lane: no line beyond, the far edge about 4.5 m out), and the car crossed the
  centre line by up to 2.5 m for 52 m. Toward the far side a keep needs a same-direction lane counted beyond the ego line:
  none here, so no keepLeft, and the keep is said (skipReason keepFarSide) and logged as D2-keep's."""
  sim = Sim(SLIGHT_LEFT, cruise=19., model_lanes=(0, 1))
  sim.model_kw = {'shoulder': 4.5}
  with mock.patch('openpilot.selfdrive.turnpilot.turnpilotd.cloudlog.event') as event:
    run_keep(sim, KEEP_J + 20.)
  assert not any(p.desire == Desire.keepLeft for _, _, p in sim.trace), "no keep over the centre line"
  said = [p for _, _, p in sim.trace if p.tooFastToTurn]
  assert said and str(said[0].skipReason) == 'keepFarSide' and said[0].maneuverDistance > 0.
  logged = keep_events(event)
  assert len(logged) == 1 and logged[0].kwargs['side'] == 'left' and logged[0].kwargs['reason'] == 'keepFarSide', logged


def test_a_keep_left_with_a_lane_of_ours_beyond_goes():
  # the control: a lane counted beyond the left ego line (its next line out painted), paint unread or white: keepLeft goes
  for pnt in ((), (UNKNOWN, BROKEN_WHITE, UNKNOWN, UNKNOWN)):
    sim = Sim(SLIGHT_LEFT, cruise=19., model_lanes=(1, 0), paint=pnt)
    run_keep(sim, KEEP_J + 20.)
    assert any(p.desire == Desire.keepLeft for _, _, p in sim.trace), pnt
    assert not any(p.tooFastToTurn for _, _, p in sim.trace), pnt


def test_no_keep_left_across_a_yellow_line():
  # a lane counted beyond the left ego line, but that line read yellow: oncoming traffic beyond it (MUTCD). No keepLeft, said
  sim = Sim(SLIGHT_LEFT, cruise=19., model_lanes=(1, 0), paint=(UNKNOWN, SOLID_YELLOW, UNKNOWN, UNKNOWN))
  run_keep(sim, KEEP_J + 20.)
  assert not any(p.desire == Desire.keepLeft for _, _, p in sim.trace)
  said = [p for _, _, p in sim.trace if p.tooFastToTurn]
  assert said and str(said[0].skipReason) == 'keepFarSide'
