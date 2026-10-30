"""The left shots' reach at the provisional range (BeamNG build findings D4-shots): S08 (unprotected_left_occluded) and S02
(unprotected_left_visible) went undecided at the line, 'Far camera range not measured', with R_req 204 and 172.9 m against
TurnPilotLeftRangeM 150. Neither junction has a set-back hold: the unprotected turn holds LINE_D (9 m) before TurnPilot's
own junction, and that ran 13 m (S08) and 6.5 m (S02) short of navd's, the sim's wheel-speed over-read (D4-speed, fixed in
the bridge), so the car stood 21.7 and 15.9 m out. At LINE_D both fit, with S08 2 m to spare: so the plans say how much
setback or junction spread each left tolerates at the provisional range, and a left's undecided says where the car stood,
navd's junction against TurnPilot's, so a recurrence reads as what it is."""
import os

import pytest

from openpilot.selfdrive.turnpilot import unprotected_left as ul
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, score
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_occluded import left_rows

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


@pytest.mark.parametrize("name,r_line,slack", [('unprotected_left_visible', 133.9, 3.7), ('unprotected_left_occluded', 141.1, 2.05)])
def test_each_left_says_how_much_setback_it_tolerates_at_the_provisional_range(name, r_line, slack):
  reach = catalog.build(name).notes['left_reach']
  assert reach['range_m'] == ul.PROVISIONAL_RANGE_M == 150.
  assert reach['r_req_at_line_m'] == pytest.approx(r_line, abs=0.2)
  assert reach['slack_m'] == pytest.approx(slack, abs=0.1)
  # R_req standing slack_m past LINE_D is the range: the live check's, with the car there or the junction read that far out
  v, lanes = reach['v_limit'], reach['lanes']
  d = ul.LINE_D + reach['slack_m']
  assert ul.r_req(v, ul.t_req(d, lanes), d) == pytest.approx(ul.PROVISIONAL_RANGE_M, abs=0.5)


def test_where_the_smokes_s08_stood_is_past_its_slack():
  # 21.7 m from the junction with TurnPilot's 13 m short of navd's: 12.7 m past LINE_D, R_req 204 m
  reach = catalog.build('unprotected_left_occluded').notes['left_reach']
  d = 21.7
  assert d - ul.LINE_D > reach['slack_m'] and ul.r_req(reach['v_limit'], ul.t_req(d + 1.9, reach['lanes']), d + 1.9) > 200.


def s08_rows():
  """left_rows' blocked creep and undecided, as the smoke's S08 stood: R_req 204.2 m, 21.78 m from the junction, navd reading
  22.3 m and TurnPilot's follower 8.8 m, the far range unmeasured."""
  rows = left_rows(blocked=False)
  for r in rows:
    if r.get('left_state') == 'undecided':
      r.update(left_rreq=204.2, left_why='farUnmeasured', j_dist=21.78, nav_dist=22.3, tp_dist=8.8)
  return rows


def test_an_undecided_left_says_where_the_car_stood():
  res = score.summarize('unprotected_left_occluded', s08_rows(), 'timeout')
  at = res['left']['undecided_at']
  assert at == {'why': 'farUnmeasured', 'r_req': 204.2, 'j_dist': 21.78, 'nav_dist': 22.3, 'tp_dist': 8.8}
  why = next(f for f in res['failures'] if f.startswith('undecided'))
  assert "R_req 204.2 m (farUnmeasured)" in why and "21.8 m from the junction" in why and "navd 22.3 m, TurnPilot 8.8 m" in why
  # rows from before the fields: said without them
  rows = left_rows(blocked=False)
  assert score.summarize('unprotected_left_occluded', rows, 'timeout')['left']['undecided_at']['tp_dist'] is None


def test_the_rows_carry_turnpilots_junction_and_the_lefts_why():
  from openpilot.cereal import messaging
  ss, cs, model = (messaging.new_message(s) for s in ('selfdriveState', 'carState', 'modelV2'))
  tp = messaging.new_message('turnpilotPlan')
  tp.turnpilotPlan.maneuverDistance = 8.8
  tp.turnpilotPlan.unprotectedLeft.why = 'farUnmeasured'
  tp.turnpilotPlan.unprotectedLeft.state = 'undecided'
  f = score.msg_fields(ss.selfdriveState, cs.carState, tp.as_reader().turnpilotPlan, None, model.modelV2)
  assert f['tp_dist'] == 8.8 and f['left_why'] == 'farUnmeasured'
  f = score.msg_fields(ss.selfdriveState, cs.carState, None, None, model.modelV2)
  assert f['tp_dist'] is None and f['left_why'] == ''


def test_the_report_says_it_too():
  res = score.summarize('unprotected_left_occluded', s08_rows(), 'timeout')
  assert "undecided 25.0: R_req 204.2 m (farUnmeasured), 21.8 m from the junction (navd 22.3 m, TurnPilot 8.8 m)" in score.report('S08', res)
