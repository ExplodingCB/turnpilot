"""turnpilotd with the stop-sign decider under TurnPilotStopSignGo=vision after dark, in the loop (test_turnpilotd_vision's
vsim: the shipped visibility file, no surveys, the map's tiers, the detector's held sweep with the side views' sectors ok, G5's
lift as plannerd applies it, the car following its command an actuator lag late), the clock at 23:00 EDT (the sun -34 deg),
19:00 EDT (dusk: -2 deg) or not set.

Kellen, 2026-11-07: "why the hell do we care about dark? its been driving at night in all our past tests... and it performed
great the streets are very lit up, its issues with out nav system not openpilot", and "no, no handbacks". Until then the vision
gate failed every stop after dark at its line (H9, "Stop sign: night at ETA +0 min: take control", 1 s after the full stop) and
the reach walk read the 'night' row, which has no table (every lane 0 m): the note above stop_rules.DARK_ROWS. Now a night stop
goes as a day one does, and a car the cameras see still holds the go. The sim's side-view sectors always read ok: whether a
real side view at a lit junction reads its sectors dark (sideview.DARK_LUMA_DN) is unmeasured (no sweep ran on 11-06)."""
import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_go as sg
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.routing import certify as ce
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, utc, vision_junction
from openpilot.selfdrive.turnpilot.stop_evidence import junction_lanes
from openpilot.selfdrive.turnpilot.tests.test_stop_evidence import track as ev_track
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import S_LINE, go_frame, run_to_line
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_vision import SHIPPED, arrived, caps_ok, vsim

NIGHT = ETA + 8 * 3600.          # 23:00 EDT
DUSK = utc(2026, 10, 28, 23, 0)  # 19:00 EDT, sunset about 18:43
CLOCKS = {'night': NIGHT, 'dusk': DUSK, 'noClock': None}
WHEN = {'day': ETA, 'night': NIGHT}


def at(sim, wall):
  """The sim's wall clock from wall on (None: not valid)."""
  sim.tp.wall_clock = (lambda: None) if wall is None else (lambda: wall + sim.t)
  return sim


def handed_back(plans):
  return [p for p in plans if p.controlPrompt == 'stopSignTakeControl']


def crossing(tid=41, role='left', d0=45., v=8., t_from=1.):
  """A car coming down the role lane toward our path at v m/s from d0 m, t_from s after it is set, then through and gone (12 m
  past), as the tracker reports it from where our car is now (the vision junction's lanes: vsim's junctionInfo)."""
  lanes = {ln.role: ln for ln in junction_lanes(vision_junction(), 'unknown', 1.9, None) if ln.index == 1}
  start = {}

  def tracks(sim):
    t0 = start.setdefault('t', sim.t_cam + t_from)
    d = d0 - v * (sim.t_cam - t0)
    if sim.t_cam < t0 or d < -12.:
      return []
    ln = lanes[role]
    s = (ln.s_min + ln.s_max) / 2. - (sim.s - S_LINE)
    trk = ev_track(tid, sim.t_cam, max(d, 0.3), ln, view='sideLeft' if role == 'left' else 'sideRight', cls=2, s=s)
    half = math.degrees(math.atan2(0.9, math.hypot(s, d)))
    trk.span_seen = tuple(sorted((trk.bearing_seen - half, trk.bearing_seen + half)))
    trk.hits = 5
    return [trk]
  return tracks, lambda: start.get('t', math.inf) + (d0 + 12.) / v  # ...and the camera time it is gone


@pytest.mark.parametrize('when', list(CLOCKS))
def test_a_night_dusk_or_clockless_stop_with_clear_cross_traffic_goes_after_the_window_never_handed_back(tmp_path, when):
  # before 2026-11-07: "Stop sign: night at ETA +0 min: take control" (or dusk, or "the clock isn't set") 1 s after the full stop
  sim = at(vsim(tmp_path), CLOCKS[when])
  run_to_line(sim)
  held_s = sim.s
  sim.run(14.)
  plans = sim.plans()
  k, a = go_frame(sim), arrived(plans)
  assert k is not None, [(str(p.stopGo.state), p.reason) for p in plans[-300::40]]
  assert (k - a) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11)
  assert all(plans[i].reason == sg.GOING_REASON for i in range(a, k) if plans[i].controlPrompt == 'stopGo')
  assert str(plans[k].stopGo.mode) == 'vision' and not plans[k].shouldStop and plans[k].speedCap <= sr.STOP_GO_V_MAX
  # the lanes judged on the day table's reach, never the 'night' row's nothing
  lanes = plans[k - 1].stopGo.lanes
  assert lanes and all(ln.rSeen >= ln.rReq > 30. for ln in lanes), [(ln.role, ln.rSeen, ln.rReq) for ln in lanes]
  assert not handed_back(plans) and not any(p.stopGo.fault for p in plans)
  assert sim.s > held_s + 15. and any(str(p.stopGo.state) in ('inLane', 'done') for p in plans) and caps_ok(plans)


@pytest.mark.parametrize('when', ['night', 'dusk'])
def test_at_night_a_crossing_car_seen_holds_the_go_until_it_has_passed_then_the_car_goes_no_handback(tmp_path, when):
  sim = at(vsim(tmp_path), CLOCKS[when])
  run_to_line(sim)
  n0, t_cam0 = len(sim.plans()), sim.t_cam
  sim.tracks, gone = crossing()
  sim.run(24.)
  plans = sim.plans()
  k, a = go_frame(sim), arrived(plans)
  assert k is not None and k > a
  assert t_cam0 + (k - n0 + 1) * DT_MDL >= gone() + sr.VISION_CLEAR_S - 0.11  # the clear window run after it is gone, never before
  # waiting on the car while it comes (its box hides the lane behind it, then it is the car from the left), and the go only
  # the clear window after the last wait, once the car has passed
  waits = [i for i in range(a, k) if plans[i].controlPrompt == 'waitingForCross' and plans[i].reason.startswith('Waiting')]
  assert waits and any(plans[i].reason == 'Waiting: a car from the left' for i in waits), {plans[i].reason for i in range(a, k)}
  assert (k - waits[-1]) * DT_MDL >= sr.VISION_CLEAR_S - 0.06
  assert sim.trace[k][2].stopGo.lanes and plans[k].reason == sg.GOING_REASON
  assert not handed_back(plans) and not any(p.stopGo.fault for p in plans) and caps_ok(plans)
  assert any(str(p.stopGo.state) in ('inLane', 'done') for p in plans)


def test_at_night_a_car_crossing_during_the_launch_withdraws_the_go_short_of_d_edge_min(tmp_path):
  # the hard rail is the same after dark: a car the side views see in the near lane once the car is moving withdraws the go
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import CYCLE
  from openpilot.selfdrive.turnpilot.tests.test_visibility import raw_box
  sim = at(vsim(tmp_path), NIGHT)
  run_to_line(sim)
  s_stop = sim.s
  for _ in range(round(20. / DT_MDL)):
    p = sim.step()
    if str(p.stopGo.state) == 'going' and sim.v > 1.0:
      break
  sim.boxes = lambda s: {v: [raw_box(65., conf=0.5, xy=(8., 17.), view='sideLeft')] for v in CYCLE}
  sim.run(6.)
  plans = sim.plans()
  assert any(p.stopGo.withdrawn for p in plans)
  assert sim.s - s_stop - 0.5 <= plans[-1].stopGo.dEdgeMin + 0.6 and sim.v < 0.05 and caps_ok(plans)


@pytest.mark.parametrize('traffic', [False, True])
def test_a_night_yield_stops_at_its_line_and_goes_after_the_window(tmp_path, traffic):
  sim = at(vsim(tmp_path, kind='yield', mph=20.), NIGHT)
  run_to_line(sim)
  if traffic:
    sim.tracks, _ = crossing(d0=30., v=6.)
  sim.run(16.)
  plans = sim.plans()
  k = go_frame(sim)
  assert k is not None and str(plans[k].stopGo.mode) == 'vision'
  assert not handed_back(plans) and caps_ok(plans)


@pytest.mark.parametrize('when', ['day', 'night'])
@pytest.mark.parametrize('mph', [20., 25.])
def test_a_right_whose_far_lane_the_line_walks_as_its_cross_movement_certifies_at_route_time_and_goes_day_or_night(tmp_path, mph, when):
  # the line walks the far lane as its cross movement under 15 deg of yaw and goes; navd's route-time walk had judged its swing
  # zone too (23.8 m of 26.7-37.6 m: 'reach'), and routed round a stop the line drives. Now the two walk the same lanes
  # (certify.vision_lanes), so navd certifies it and says nothing ahead of it
  j = vision_junction(int(mph), maneuver='right')
  v = ce.assess(j, 'right', SHIPPED, [], ETA if when == 'day' else NIGHT, ce.Runtime(mode='vision', side_views=True))
  assert (v.certification, v.reason) == ('certifiable', 'none'), v.detail
  sim = vsim(tmp_path, mph=mph, maneuver='right')
  if when == 'night':
    at(sim, NIGHT)
  run_to_line(sim)
  sim.run(14.)
  plans = sim.plans()
  k, a = go_frame(sim), arrived(plans)
  assert k is not None and (k - a) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11)
  assert not handed_back(plans) and caps_ok(plans)


@pytest.mark.parametrize('when', ['day', 'night'])
def test_a_stop_on_a_cross_road_of_two_lanes_each_way_is_short_at_the_line_too_so_navd_still_routes_round_it(tmp_path, when):
  # review of the 11-06 repairs: navd's route-time 'reach' here (the near lane 17.2 of 37.6 m on the lines the line walks) is
  # the line's own: it waits on reach, creeps for a view and hands back 15 s after the full stop, day or night. This is what
  # navd routes round, and says "Stop sign ahead needs you" for when it can't (routing/tests/test_avoid_night.py); a rule that
  # kept every vision 'reach' stop on the route left the driver this handback unwarned
  j = vision_junction(25)
  for a in j.approaches:
    if a.role in ('left', 'right'):
      a.lanes_in = a.lanes_out = 2
  v = ce.assess(j, 'straight', SHIPPED, [], WHEN[when], ce.Runtime(mode='vision', side_views=True))
  assert (v.certification, v.reason) == ('uncertifiable', 'reach') and 'left lane 1 cross' in v.detail, v.detail
  sim = vsim(tmp_path)
  for leg in sim.navd.controls[0][5]['approaches']:
    if leg['role'] in ('left', 'right'):
      leg['lanesIn'] = leg['lanesOut'] = 2
  at(sim, WHEN[when])
  run_to_line(sim)
  sim.run(20.)
  plans = sim.plans()
  a, back = arrived(plans), handed_back(plans)
  assert go_frame(sim) is None and back, [(str(p.stopGo.state), p.reason) for p in plans[-400::40]]
  k = plans.index(back[0])
  assert (k - a) * DT_MDL == pytest.approx(15., abs=0.6) and "can't see far enough from the left" in plans[k].reason, plans[k].reason
