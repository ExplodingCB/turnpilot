"""turnpilotd with the stop-sign decider under TurnPilotStopSignGo=vision in the loop (test_turnpilotd_stop's StopSim): the
shipped visibility file (PV1 tables, camera fields and mask null, the veil unmeasured), no surveys, the map's tiers (mapStrong
at an all-way, no on a two-way's through road), the clock at 15:00 EDT with the sun behind a northbound car, the detector's held
sweep with the side views' sectors ok. G5's lift as plannerd applies it, the car following its command an actuator lag late."""
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_go as sg
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table as vt
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, LAT, LON, NODE
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import FakeParams
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import CYCLE, S_LINE, StopSim, go_frame, raw_box, run_to_line
from openpilot.selfdrive.turnpilot.turnpilotd import stop_sign_mode_for

MPH = sr.MPH
SHIPPED = vt.load(vt.SHIPPED)


def vinfo(mph=25., maneuver='straight', tiers=None, sidecar=True, kind_mph=None):
  """The control's junctionInfo as Saturday's car meets it: no survey, the map's tiers (mapStrong: an all-way), each
  approach's assumedLimit (kind_mph: per role)."""
  t, per = tiers or {}, kind_mph or {}

  def leg(bearing, rel, role, way):
    v = per.get(role, mph) * MPH
    tier = t.get(role, 'mapStrong')
    return {'bearing': bearing, 'relBearing': rel, 'role': role, 'roadClass': 'residential', 'use': 'road', 'inbound': True,
            'outbound': True, 'lanesIn': 1, 'lanesOut': 1, 'valhallaControl': 'stop' if tier != 'no' else 'none', 'tier': tier,
            'assumedLimit': v, 'wayId': way, 'bikeLane': False, 'shoulderM': -1.}
  return {'status': 'ok', 'osmNodeId': NODE, 'lat': LAT, 'lon': LON, 'stopPointDistance': 13., 'stopPointSource': 'default',
          'headingIn': 0., 'maneuver': maneuver, 'turnAngle': 0., 'roundabout': False, 'divided': False, 'railDistance': -1.,
          'sidecar': sidecar, 'suspected': False, 'surveyed': False,
          'approaches': [leg(270., -90., 'left', 11), leg(90., 90., 'right', 12), leg(0., 0., 'opposite', 13)]}


def vsim(tmp_path, kind='stop', model='goes', **info_kw):
  sim = StopSim(tmp_path, mode='vision', vis=SHIPPED, entries=[], model=model)
  sim.navd.controls[0] = (S_LINE, kind, False, True, False, vinfo(**info_kw))
  return sim


def held_at(plans):
  return next(k for k, q in enumerate(plans) if str(q.mapControl.action) in ('held', 'awaitingDriver', 'waitingCross') and q.shouldStop)


def arrived(plans):
  return next(k for k, q in enumerate(plans) if str(q.stopGo.state) == 'observe')


def caps_ok(plans):
  return all(plan_accel(p.accelCap, p.speedCap, 1.) >= -2. - 1e-9 for p in plans)


@pytest.mark.parametrize('mph', [20., 25.])
def test_vision_an_empty_all_way_goes_after_the_full_stop_and_the_2_5_s_window(tmp_path, mph):
  sim = vsim(tmp_path, mph=mph)
  run_to_line(sim)
  held_s = sim.s
  sim.run(14.)
  plans = sim.plans()
  k, a = go_frame(sim), arrived(plans)
  assert k is not None, [(str(p.stopGo.state), p.reason) for p in plans[-300::40]]
  assert (k - a) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11)
  # the line and its chime from 1.5 s (the stopGo prompt, "Going"), the go, across
  said = [i for i in range(a, k) if plans[i].controlPrompt == 'stopGo']
  assert said and (k - said[0]) * DT_MDL == pytest.approx(sr.VISION_ANNOUNCE_S, abs=0.11) and all(plans[i].reason == 'Going' for i in said)
  assert all(plans[i].shouldStop for i in said)  # still held while it says it
  assert str(plans[k].stopGo.mode) == 'vision' and not plans[k].shouldStop and plans[k].speedCap <= sr.STOP_GO_V_MAX
  assert not any(p.controlPrompt == 'stopSignTakeControl' for p in plans)
  assert sim.s > held_s + 15. and any(str(p.stopGo.state) in ('inLane', 'done') for p in plans)
  assert caps_ok(plans)


def test_vision_an_empty_two_way_goes(tmp_path):
  sim = vsim(tmp_path, mph=20., tiers={'left': 'no', 'right': 'no'})
  run_to_line(sim)
  sim.run(14.)
  assert go_frame(sim) is not None and not any(p.controlPrompt == 'stopSignTakeControl' for p in sim.plans())


@pytest.mark.parametrize('mph', [30., 35., 0.])
def test_vision_a_cross_road_over_25_mph_or_unknown_goes_after_the_4_s_long_look(tmp_path, mph):
  # the owner's direction (2026-10-30 19:15-19:55): no handback; the longer look, then "Look both ways" and the go
  sim = vsim(tmp_path, mph=mph)
  run_to_line(sim)
  sim.run(16.)
  plans = sim.plans()
  k, a = go_frame(sim), arrived(plans)
  assert k is not None and (k - a) * DT_MDL >= sr.VISION_LONG_CLEAR_S - 0.06
  said = [i for i in range(a, k) if plans[i].controlPrompt == 'stopGo']
  assert said and all(plans[i].reason == sg.LOOK_REASON for i in said)
  assert not any(p.controlPrompt == 'stopSignTakeControl' for p in plans) and caps_ok(plans)


@pytest.mark.parametrize('mph, reason', [(25., sg.GOING_REASON), (35., sg.LOOK_REASON)])
def test_vision_re_engaged_at_the_line_after_the_window_says_it_1_s_before_the_go(tmp_path, mph, reason):
  # the brake at the line through the whole window (openpilot disengaged), then re-engaged: the plan's stopGo prompt with its
  # line ("Look both ways" at a long look), the car held, for 1 s before the latch lets go; never a go on the re-engaged frame
  sim = vsim(tmp_path, mph=mph)
  run_to_line(sim)
  sim.run(1.5)
  sim.brake, sim.engaged = True, False
  sim.run(6.)
  assert go_frame(sim) is None
  sim.brake, sim.engaged = False, True
  n0 = len(sim.plans())
  sim.run(6.)
  plans = sim.plans()
  k = go_frame(sim)
  assert k is not None and (k - n0) * DT_MDL == pytest.approx(1.0, abs=0.06)  # VISION_ANNOUNCE_S after the re-engage
  assert all(plans[i].controlPrompt == 'stopGo' and plans[i].reason == reason and plans[i].shouldStop for i in range(n0, k))
  assert plans[k].reason == 'Going' and not plans[k].shouldStop
  assert not any(p.controlPrompt == 'stopSignTakeControl' for p in plans) and caps_ok(plans)


@pytest.mark.parametrize('status', ['off', 'disabled', 'unset'])
def test_vision_side_views_off_or_disabled_hands_back_at_1_s_saying_so(tmp_path, status):
  sim = vsim(tmp_path)
  sim.side_status = lambda s: status
  run_to_line(sim)
  sim.run(3.)
  plans = sim.plans()
  latched = held_at(plans)
  handed = next(k for k, q in enumerate(plans) if q.controlPrompt == 'stopSignTakeControl')
  assert abs((handed - latched) * DT_MDL - 1.0) <= 2 * DT_MDL
  p = plans[-1]
  assert p.reason == "Stop sign: side cameras off: take control" and go_frame(sim) is None and p.shouldStop
  assert p.stopGo.fault == 'H9' and str(p.stopGo.mode) == 'vision'


@pytest.mark.parametrize('dt_h, words', [(8., 'night at ETA +0 min'), (None, "the clock isn't set")])
def test_vision_night_and_no_clock_hand_back_at_1_s_saying_which(tmp_path, dt_h, words):
  sim = vsim(tmp_path)
  sim.tp.wall_clock = (lambda: None) if dt_h is None else (lambda: ETA + dt_h * 3600. + sim.t)
  run_to_line(sim)
  sim.run(3.)
  p = sim.plans()[-1]
  assert p.controlPrompt == 'stopSignTakeControl' and words in p.reason and p.reason.startswith('Stop sign: ') and go_frame(sim) is None


def test_vision_without_the_sidecar_hands_back_h8(tmp_path):
  sim = vsim(tmp_path, sidecar=False)
  run_to_line(sim)
  sim.run(3.)
  p = sim.plans()[-1]
  assert p.controlPrompt == 'stopSignTakeControl' and p.reason == sg.REASONS['H8'] and go_frame(sim) is None


@pytest.mark.parametrize('traffic', [False, True])
def test_vision_a_yield_stops_at_its_line_and_goes_after_the_window(tmp_path, traffic):
  # a yield becomes a yieldStop (no rolling yield is judged this week: spec C6) and is the decider's: a full stop at its
  # line, the window, the go; with a car coming, waited for first
  sim = vsim(tmp_path, kind='yield', mph=20.)
  run_to_line(sim)
  if traffic:
    t0 = sim.t_cam
    sim.boxes = lambda s: {v: [raw_box(65., conf=0.5, xy=(8., 17.), view='sideLeft')] for v in CYCLE} if s.t_cam - t0 < 3. else {}
  sim.run(16.)
  plans = sim.plans()
  k = go_frame(sim)
  assert k is not None and str(plans[k].stopGo.mode) == 'vision'
  if traffic:
    assert any(p.controlPrompt == 'waitingForCross' and 'Waiting' in p.reason for p in plans[:k])
    last = max(i for i in range(k) if plans[i].controlPrompt == 'waitingForCross' and plans[i].reason.startswith('Waiting'))
    assert (k - last) * DT_MDL >= sr.VISION_CLEAR_S - 0.06
  assert not any(p.controlPrompt == 'stopSignTakeControl' for p in plans) and caps_ok(plans)


def test_vision_a_yield_with_its_side_views_off_says_yield(tmp_path):
  sim = vsim(tmp_path, kind='yield')
  sim.side_status = lambda s: 'off'
  run_to_line(sim)
  sim.run(3.)
  assert sim.plans()[-1].reason == "Yield: side cameras off: take control"


def test_vision_a_car_in_the_near_lane_during_the_launch_withdraws_the_go_short_of_d_edge_min(tmp_path):
  for lag in (0.2, 0.4):
    sim = vsim(tmp_path)
    sim.lag = type(sim.lag)([0.] * max(round(lag / DT_MDL), 1))
    run_to_line(sim)
    s_stop = sim.s
    for _ in range(round(20. / DT_MDL)):
      p = sim.step()
      if str(p.stopGo.state) == 'going' and sim.v > 1.0:
        break
    sim.boxes = lambda s: {v: [raw_box(65., conf=0.5, xy=(8., 17.), view='sideLeft')] for v in CYCLE}
    sim.run(6.)
    plans = sim.plans()
    assert any(p.stopGo.withdrawn for p in plans), lag
    assert sim.s - s_stop - 0.5 <= plans[-1].stopGo.dEdgeMin + 0.6 and sim.v < 0.05 and caps_ok(plans)


def test_vision_publishes_mode_vision_and_side_views_live_for_navd(tmp_path):
  sim = vsim(tmp_path)
  sim.run(2.)
  p = sim.plans()[-1]
  assert str(p.stopGo.mode) == 'vision' and p.stopGo.sideViewsLive
  sim.side_status = lambda s: 'disabled'
  sim.run(0.5)
  assert not sim.plans()[-1].stopGo.sideViewsLive
  assert stop_sign_mode_for(FakeParams(TurnPilotStopSignGo=b'vision')) == ('vision', True)


def test_vision_the_brake_during_the_go_ends_the_lift_and_the_decider_resets(tmp_path):
  sim = vsim(tmp_path, model='holds')
  run_to_line(sim)
  while go_frame(sim) is None:
    sim.step()
    assert sim.t < 60.
  sim.step()
  sim.brake, sim.engaged = True, False
  sim.run(1.)
  plans = sim.plans()
  assert str(plans[-1].goCertified.cause) == 'none' and str(plans[-1].stopGo.state) in ('done', 'off')
