"""unprotected_left_occluded (S08): a car standing on its parking brake in the opposing inner lane, in the opposing left
turn's place past the junction, hides part of the oncoming lanes from the car's hold line (TurnPilot's own occluders()
and view_blocked() say so, before and after the full creep), with a gap stream in the outer lane beside it; the score
takes an undecided handback with the view blocked, never a go while it is, and checks the creep's reach."""
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.turnpilot import unprotected_left as ul
from openpilot.selfdrive.turnpilot.route_follower import LEFT
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, ONCOMING_Y_NEAR_M, VEL_TRUST_HITS
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, director, score
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_score import row

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


def car_frame(p, ego, heading):
  d = np.asarray(p[:2]) - np.asarray(ego[:2])
  return float(d[0] * math.cos(heading) + d[1] * math.sin(heading)), float(-d[0] * math.sin(heading) + d[1] * math.cos(heading))


@pytest.fixture(scope='module')
def plan():
  return catalog.build('unprotected_left_occluded')


def hold_pose(plan, back_m: float):
  """The car's camera back_m before the maneuver point along its approach, in its lane."""
  h = plan.heading
  lane = catalog.right_of(h) * plan.notes['ego_lane_offset_m']
  p = np.asarray(plan.junction) - back_m * catalog.unit(h)
  return p + lane


def test_a_standing_car_in_the_opposing_left_turns_place_and_a_stream_beside_it(plan):
  occ = [a for a in plan.actors if a.trigger == 'setup']
  stream = [a for a in plan.actors if a.trigger == 'oncoming']
  assert len(occ) == 1 and occ[0].kind == 'car' and occ[0].name in director.KNOWN
  assert plan.notes['stream']['headway_s'] == 10. and len(stream) >= 3 and all(a.params['timed'] for a in stream)
  o = np.asarray(occ[0].path[0][:2])
  # facing the junction, as a car waiting to turn its left
  to_j = math.atan2(plan.junction[1] - o[1], plan.junction[0] - o[0])
  assert abs(math.degrees(catalog.wrap(occ[0].heading - to_j))) < 20.
  # the stream passes it in the outer lane, a lane's width or more away, centre to centre
  for a in stream:
    path = np.asarray(a.path)[:, :2]
    assert np.min(np.hypot(*(path - o).T)) >= 3.0, a.name
  # in the zone unprotected_left reads occluders in, from the car's hold line
  ego = hold_pose(plan, HOLD_LINE_M)
  x, y = car_frame(o, ego, plan.heading)
  assert HOLD_LINE_M - ul.ONCOMING_PAST_M <= x <= HOLD_LINE_M + ul.OCCLUDER_PAST_M and ONCOMING_Y_NEAR_M <= y <= ul.OCCLUDER_Y_FAR_M


@pytest.mark.parametrize("crept_m", [0., ul.CREEP_MAX_M])
def test_turnpilot_reads_it_as_blocking_the_view_at_the_line_and_after_the_full_creep(plan, crept_m):
  occ = next(a for a in plan.actors if a.trigger == 'setup')
  back = HOLD_LINE_M - crept_m
  x, y = car_frame(occ.path[0], hold_pose(plan, back), plan.heading)
  track = SimpleNamespace(cls='car', x=x, y=y, vx=0., vy=0., age=VEL_TRUST_HITS, trackId=7)
  found = ul.occluders([track], 0., LEFT, back)
  assert found and ul.view_blocked(found, 0., LEFT, plan.notes['left_r_req_m'])


def test_the_turn_clears_it(plan):
  # the car's camera across the junction's middle, on its way into the exit, stays further than the score's 4 m from it
  occ = np.asarray(next(a for a in plan.actors if a.trigger == 'setup').path[0][:2])
  assert np.hypot(*(occ - np.asarray(plan.junction))) > score.CAR_MIN_GAP_M + 4.


def left_rows(blocked=True, go=False, creep_to=3.5):
  """At its line, creeping creep_to m, blocked; a stream car through, then an empty road; undecided or a go."""
  rows = []
  for k in range(int(40. / 0.05)):
    t = round(k * 0.05, 3)
    ahead = 18. - min(t / 4., 1.) * creep_to
    state = 'creep' if t < 4. else 'waitGap'
    if t >= 25.:
      state = 'go' if go else 'undecided'
    onc = [[abs(11. * (12. - t)), 11. * (12. - t)]]
    rows.append(row(t, v=0. if state != 'go' else 2., j_dist=ahead, j_ahead=ahead, onc=onc, car_j_dist=onc[0][0], left_state=state,
                    left_treq=6.5, left_blocked=blocked, state='holding', reason='Creeping for a view',
                    control_prompt='leftUndecidedTakeControl' if state == 'undecided' else 'creepingForView'))
  return rows


def test_undecided_with_the_view_blocked_passes_the_occluded_left_only():
  res = score.summarize('unprotected_left_occluded', left_rows(), 'timeout')
  assert not any('undecided' in f for f in res['failures']), res['failures']
  assert any('undecided' in f for f in score.summarize('unprotected_left_gap10', left_rows(), 'timeout')['failures'])
  res = score.summarize('unprotected_left_occluded', left_rows(blocked=False), 'timeout')
  assert any('undecided' in f for f in res['failures'])  # a gap it could see and didn't take


def test_a_go_with_the_view_blocked_and_a_creep_too_far_fail():
  res = score.summarize('unprotected_left_occluded', left_rows(go=True), 'arrived')
  assert any('went with the view blocked' in f for f in res['failures']), res['failures']
  res = score.summarize('unprotected_left_occluded', left_rows(creep_to=ul.CREEP_MAX_M + 2.), 'timeout')
  assert any('crept' in f for f in res['failures']), res['failures']
  assert score.summarize('unprotected_left_occluded', left_rows(), 'timeout')['left']['creep_m'] == pytest.approx(3.5, abs=0.1)


def withdrawn_go_rows(into_zone=False):
  """left_rows' blocked creep and undecided handback, but with the view clear from 8 to 11 s and a go at 10 s, the stream
  car 22 m from the junction at 11 m/s (1.3 s from the conflict zone, under tReq), withdrawn 1 s on: 2 m on, or into the
  conflict zone as the car passes through it (into_zone)."""
  rows = left_rows()
  for r in rows:
    t = r['t']
    if 8. <= t < 11.:
      r['left_blocked'] = False
    if 10. <= t:
      gone = min(t - 10., 1.) * (7. if into_zone else 2.)
      r['j_dist'] = r['j_ahead'] = round(14.5 - gone, 2)
    if 10. <= t < 11.:
      r.update(left_state='go', v=7. if into_zone else 2., control_prompt='leftGo')
  return rows


def test_a_false_go_or_a_close_pass_fails_even_when_it_ends_undecided():
  # demo skeptic (score._fails): with undecided_t set the false-go and PET checks never ran, and the occluded rule takes
  # the blocked undecided as the fault it is, so a withdrawn go into an oncoming car passed
  res = score.summarize('unprotected_left_occluded', withdrawn_go_rows(), 'timeout')
  lr = res['left']
  assert lr['undecided_t'] == pytest.approx(25.) and lr['blocked_at_undecided'] and not lr['go_blocked']
  assert any(f.startswith("1 false go(es): at t=10.0 s an oncoming car 1.27 s from the junction") for f in res['failures']), res['failures']
  res = score.summarize('unprotected_left_occluded', withdrawn_go_rows(into_zone=True), 'timeout')
  assert any(f.startswith("post-encroachment time 0.0 s with an oncoming car") for f in res['failures']), res['failures']
  assert not res['passed']
  # the blocked undecided alone still passes (test_undecided_with_the_view_blocked_passes_the_occluded_left_only)
  assert score.summarize('unprotected_left_occluded', left_rows(), 'timeout')['passed']
