"""turnpilotd's keeps (route_follower.KEEP: a slight turn at a junction, a fork, an off ramp) in closed loop, on the synthetic
route, model and scene of test_turnpilotd: a keep the model can't be sent is said and rerouted, never left to the model
in silence (BUILD_FINDINGS D2-keep), and none goes toward the oncoming side of the road (D5)."""
from unittest import mock

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import keep_lead, keep_last
from openpilot.selfdrive.turnpilot.hooks import KEEP_FOLLOW, KEEP_REROUTED
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
  # the control: a lane counted beyond the left ego line (its next line out painted), that line read broken white: keepLeft goes
  sim = Sim(SLIGHT_LEFT, cruise=19., model_lanes=(1, 0), paint=(UNKNOWN, BROKEN_WHITE, UNKNOWN, UNKNOWN))
  run_keep(sim, KEEP_J + 20.)
  assert any(p.desire == Desire.keepLeft for _, _, p in sim.trace)
  assert not any(p.tooFastToTurn for _, _, p in sim.trace)


def test_no_keep_left_on_the_lane_count_alone_with_the_paint_unread():
  """Skeptic SIMFIX D5 (8c0f4e7): with the lane paint unread a far-side keep went on the model's lane count alone, and an
  undivided two-way road whose oncoming lane has its outer edge line painted counts a lane to the left (road(1, 0)): an
  uncapped keepLeft over the centre line whenever the paint is unread (night, worn paint, frames the reader skips). S19's keep
  was 14 deg, under KEEP_CORNER_MIN_DEG, so no corner cap either, and the wrong-side handback needs WRONG_SIDE_S of yellow
  reads, after the crossing. A far-side keep needs the ego line read broken white: unread or unknown it is said
  (skipReason keepFarSide, 'Rerouting') and never sent."""
  for pnt in ((), (UNKNOWN,) * 4):
    sim = Sim(SLIGHT_LEFT, cruise=19., model_lanes=(1, 0), paint=pnt)
    run_keep(sim, KEEP_J + 20.)
    assert not any(p.desire == Desire.keepLeft for _, _, p in sim.trace), f"keepLeft on the lane count alone, paint {pnt}"
    said = [p for _, _, p in sim.trace if p.tooFastToTurn]
    assert said and str(said[0].skipReason) == 'keepFarSide' and said[0].maneuverDistance > 0., pnt


def test_no_keep_left_across_a_yellow_line():
  # a lane counted beyond the left ego line, but that line read yellow: oncoming traffic beyond it (MUTCD). No keepLeft, said
  sim = Sim(SLIGHT_LEFT, cruise=19., model_lanes=(1, 0), paint=(UNKNOWN, SOLID_YELLOW, UNKNOWN, UNKNOWN))
  run_keep(sim, KEEP_J + 20.)
  assert not any(p.desire == Desire.keepLeft for _, _, p in sim.trace)
  said = [p for _, _, p in sim.trace if p.tooFastToTurn]
  assert said and str(said[0].skipReason) == 'keepFarSide'


def follow_sim(**road):
  sim = keep_sim(**road)
  sim.tp.always_drive = True
  return sim


def test_a_keep_that_cant_be_sent_under_always_drive_is_a_status_line_while_the_model_drives_the_fork():
  """ALWAYS_DRIVE.md R7 (S3), under TurnPilotAlwaysDrive: the keep the model sees no lane line for is still never sent, and
  still navd's skip at keep_last (tooFastToTurn: past the fork navd moves on if the model took the route's branch, or
  reroutes), logged once with its reason; but it is said as a status line (skipReason keepFollow), not the skip's chime and
  'Rerouting'. Its corner cap slows the car on the approach up to the skip and is lifted there: the model drives the fork
  at its own pace (comma first, turnpilotd.keep_unsent; on 2026-11-06 the cap kept to the pass bound 8.1 s of 1e and 16.7 s
  of 1d past the skip, the model asking +0.3 to +1.0 m/s^2 over it on a road it was driving straight on). The plain skip,
  with the switch off, keeps it to its pass (test_a_keep_the_model_sees_no_lane_line_for_is_said_and_rerouted). The model
  taking the branch (navd moving on to the next step) says nothing more."""
  sim = follow_sim(line_prob=0.1)
  with mock.patch('openpilot.selfdrive.turnpilot.turnpilotd.cloudlog.event') as event:
    run_keep(sim, KEEP_J + 40.)
  assert not any(p.desire == Desire.keepRight for _, _, p in sim.trace)
  said = [i for i, (_, _, p) in enumerate(sim.trace) if p.tooFastToTurn]
  assert said
  first_s, first_v, first = sim.trace[said[0]]
  assert abs(first.maneuverDistance - keep_last(first_v)) < 1. and str(first.skipReason) == KEEP_FOLLOW
  on_keep = [p for s, _, p in sim.trace if s >= first_s and str(p.maneuverModifier) == 'slight right' and p.routeValid]
  assert on_keep and all(p.tooFastToTurn and str(p.skipReason) == KEEP_FOLLOW for p in on_keep)
  approach = [p for s, _, p in sim.trace if s <= first_s and 0. < p.maneuverDistance < keep_lead(first_v)]
  assert approach and all(p.speedCap < NO_SPEED_CAP for p in approach)  # the corner cap on its approach, to the skip
  assert len(on_keep) > 1 and all(p.speedCap == NO_SPEED_CAP for p in on_keep[1:])  # lifted from the skip: the model's pace
  assert not any(str(p.skipReason) == KEEP_REROUTED or p.turnMissed for _, _, p in sim.trace)  # navd moved on: nothing more
  logged = keep_events(event)
  assert len(logged) == 1 and logged[0].kwargs['reason'] == 'keepNoLine' and logged[0].kwargs['follow'] is True, logged


def test_a_keep_followed_under_always_drive_chimes_rerouting_only_once_navd_recomputes():
  """R7: past the fork the car went the other way and navd recomputes (a new route): only now the skip's chime and
  'Rerouting' (skipReason keepRerouted for MISSED_HOLD_S), with no skip mark for the new route (tooFastToTurn false: navd
  would read one as the new route's first step skipped) and no turnMissed (the turn signal would end the new route's turn)."""
  from openpilot.selfdrive.turnpilot.tests.synthetic import FakeNavd
  from openpilot.selfdrive.turnpilot.turnpilotd import MISSED_HOLD_S
  sim = follow_sim(line_prob=0.1)
  run_keep(sim, KEEP_J + 2.)
  assert sim.trace[-1][2].tooFastToTurn and str(sim.trace[-1][2].skipReason) == KEEP_FOLLOW
  s0 = sim.s
  sim.navd = FakeNavd([(s0 + 400., "turn", "left", junction_of("left")), (s0 + 1500., "turn", "right", junction_of("right"))],
                      route_count=1)
  start = len(sim.trace)
  run_keep(sim, s0 + 80.)
  after = [p for _, _, p in sim.trace[start:]]
  rerouted = [i for i, p in enumerate(after) if str(p.skipReason) == KEEP_REROUTED]
  assert rerouted and abs(len(rerouted) * DT_MDL - MISSED_HOLD_S) <= 2 * DT_MDL
  assert rerouted == list(range(rerouted[0], rerouted[0] + len(rerouted)))  # once
  assert not any(after[i].tooFastToTurn or after[i].turnMissed for i in rerouted)
  assert not any(p.tooFastToTurn for p in after[rerouted[0]:])  # the new route's turn ahead is never skipped by it


def test_a_keep_that_cant_be_sent_with_the_switch_off_is_the_skip_as_before():
  sim = keep_sim(line_prob=0.1)
  run_keep(sim, KEEP_J + 40.)
  said = [p for _, _, p in sim.trace if p.tooFastToTurn]
  assert said and all(str(p.skipReason) == 'keepNoLine' for p in said)
  assert not any(str(p.skipReason) in (KEEP_FOLLOW, KEEP_REROUTED) for _, _, p in sim.trace)


# 1e seg 5 (~23.4 s, 2026-11-06 after dark): a "turn, slight left" of 29 deg across an opposing approach, at 11.1 m/s; here on a
# 25 mph opposing road straight past every R_req (test_turnpilotd_left's LEFT_25), so its left is judged at its line
V_25, STRAIGHT_M = 11.18, 400.
SLIGHT_LEFT_ACROSS = [(KEEP_J, "turn", "slight left", (0., 331., 2, 'present', V_25, 1, STRAIGHT_M)),
                      (1500., "turn", "right", junction_of("right"))]


def test_a_slight_left_across_oncoming_traffic_is_driven_as_an_unprotected_left_not_a_keep():
  """1e seg 5 and 1d seg 9: slight lefts of 29 and 38 deg with an opposing approach were keeps; at night no lane line was read
  that way (keepNoLine), the keep was said and skipped, the car drove straight on, 'Fork Missed', and navd rerouted into a
  left. A keepLeft there would have crossed the oncoming lanes with no hold and no gap check. Now the follower makes it a TURN
  (route_follower.make_maneuver), so turnpilotd drives it as the autonomous unprotected left: no keep desire, never
  keep_not_sent, its pulse held on the approach and the car brought to its hold line, where the left decides (a gap the camera
  sees, here an empty road; or a loud handback)."""
  sim = keep_sim(SLIGHT_LEFT_ACROSS, cruise=11.1, model_lanes=(0, 1), line_prob=0.1)
  sim.tp.auto_left, sim.tp.left_range_m = 'auto', 210.
  sim.go_taps = False
  sim.answers_prompt = True
  with mock.patch('openpilot.selfdrive.turnpilot.turnpilotd.cloudlog.event') as event:
    while sim.s < KEEP_J - 2. and sim.t < 60.:
      sim.step(engaged=True)
  plans = [p for _, _, p in sim.trace]
  assert not keep_events(event), "said and skipped as a keep"
  assert not any(p.desire in (Desire.keepLeft, Desire.keepRight) for p in plans)
  assert not any(p.tooFastToTurn for p in plans), [str(p.skipReason) for p in plans if p.tooFastToTurn][:1]
  states = {str(p.unprotectedLeft.state) for p in plans}
  assert states - {'off'}, states  # the unprotected left's approach, and its hold line
  approach = [p for p in plans if str(p.unprotectedLeft.state) not in ('off',)]
  assert all(p.desire == Desire.none for p in plans[:plans.index(approach[0])]), "no pulse on its approach"
  # it reached its line: standing short of the junction or gone from it on its own go, never driven straight through at speed
  assert min(v for s, v, _ in sim.trace if KEEP_J - 30. < s < KEEP_J) < 0.5
