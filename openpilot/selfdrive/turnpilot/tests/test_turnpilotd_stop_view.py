"""turnpilotd with the stop-sign decider under TurnPilotStopSignGo=vision in the loop (test_turnpilotd_vision's vsim: the shipped
visibility file, no surveys, the map's tiers, the detector's held sweep, G5's lift as plannerd applies it, the car following its
command an actuator lag late) where the side views can't see as far as the decider would like, by day (15:00 EDT) and at night
(23:00 EDT).

Kellen, 2026-11-07: "no, no handbacks"; stop signs a vision-based go like early Autopilot, imperfect OK, not aggressive. Until the
review of 2026-11-08 the far side-view sectors reading 'degraded' (low contrast: what ordinary footage reads at 70-85 deg, wf7
verify:LS-3-R1 point 4) zeroed the far lane, and the car waited and handed back 15 s after its full stop ("Stop sign: can't see
far enough from the left (side camera unclear): take control"), as did any lane hidden past what a creep was predicted to help.
Now a degraded sector is judged on its boxes and tracks, and a lane still short is crept for to the creep's limit, then the car
goes on what it sees: a car seen coming still holds it until it has passed (tests/test_stop_go_view.py for the decider)."""
import math

import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_go as sg
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, vision_junction
from openpilot.selfdrive.turnpilot.stop_evidence import junction_lanes
from openpilot.selfdrive.turnpilot.tests import test_turnpilotd_stop as ts
from openpilot.selfdrive.turnpilot.tests.test_stop_evidence import track as ev_track
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import S_LINE, go_frame, run_to_line
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop_night import at, crossing, handed_back
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_vision import arrived, caps_ok, vsim

NIGHT = ETA + 8 * 3600.  # 23:00 EDT
WHEN = {'day': ETA, 'night': NIGHT}


def far_sectors(quality, from_deg):
  """A side view's sectorQuality: ok from its inner edge, quality from from_deg out past the frame edge."""
  def fn(view):
    sign = 1. if view == 'sideLeft' else -1.
    return [(sign * lo, sign * (lo + 5.), quality if lo >= from_deg - 1e-9 else 'ok') for lo in np.arange(sr.INNER_BAND_HI_DEG, 90., 5.)]
  return fn


def across(plans):
  return any(str(p.stopGo.state) in ('inLane', 'done') for p in plans)


def parked_at_corner(tid=3, d=10., frac=0.05):
  """A car standing 10 m down the left lane of the vision junction at its band's near edge (a two-way's through road: no
  shield), as the tracker reports it from where our car is now: a bearing-only occluder of the lane behind it."""
  lanes = {ln.role: ln for ln in junction_lanes(vision_junction(), 'unknown', 1.9, None) if ln.index == 1}

  def tracks(sim):
    ln = lanes['left']
    s = ln.s_min + frac * (ln.s_max - ln.s_min) - (sim.s - S_LINE)
    trk = ev_track(tid, sim.t_cam, d, ln, view='sideLeft', cls=2, s=s)
    half = math.degrees(math.atan2(0.9, math.hypot(s, d)))
    trk.span_seen = tuple(sorted((trk.bearing_seen - half, trk.bearing_seen + half)))
    trk.hits = 5
    return [trk]
  return tracks


@pytest.mark.parametrize('when', list(WHEN))
def test_degraded_far_sectors_at_an_empty_stop_go_after_the_window_and_cross_never_handed_back(tmp_path, monkeypatch, when):
  monkeypatch.setattr(ts, 'sectors', far_sectors('degraded', 72.5))
  sim = at(vsim(tmp_path), WHEN[when])
  run_to_line(sim)
  held_s = sim.s
  sim.run(14.)
  plans = sim.plans()
  k, a = go_frame(sim), arrived(plans)
  assert k is not None, [(str(p.stopGo.state), p.reason) for p in plans[-300::40]]
  assert (k - a) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11) and plans[k].reason == sg.GOING_REASON
  assert not handed_back(plans) and not any(p.stopGo.fault for p in plans) and caps_ok(plans)
  assert sim.s > held_s + 15. and across(plans)


@pytest.mark.parametrize('when', list(WHEN))
def test_degraded_far_sectors_a_crossing_car_seen_holds_the_go_until_it_has_passed_then_the_car_goes(tmp_path, monkeypatch, when):
  monkeypatch.setattr(ts, 'sectors', far_sectors('degraded', 72.5))
  sim = at(vsim(tmp_path), WHEN[when])
  run_to_line(sim)
  n0, t_cam0 = len(sim.plans()), sim.t_cam
  sim.tracks, gone = crossing()
  sim.run(24.)
  plans = sim.plans()
  k, a = go_frame(sim), arrived(plans)
  assert k is not None and k > a
  assert t_cam0 + (k - n0 + 1) * DT_MDL >= gone() + sr.VISION_CLEAR_S - 0.11
  assert any(plans[i].reason == 'Waiting: a car from the left' for i in range(a, k)), {plans[i].reason for i in range(a, k)}
  assert not handed_back(plans) and not any(p.stopGo.fault for p in plans) and caps_ok(plans) and across(plans)


@pytest.mark.parametrize('when', list(WHEN))
def test_a_lane_hidden_by_a_car_parked_at_the_corner_is_crept_for_then_gone_on_what_the_cameras_see(tmp_path, when):
  # a two-way's through road with a car parked at the corner: the lane behind it hidden, which a creep was never predicted to
  # help (a bearing-only occluder), so H3 15 s after the stop. Now: creeping for a view in steps (the latch's creep path, G5's
  # creep lift), never closer to the closest plausible edge than a go needs, then "Look both ways" and across
  sim = at(vsim(tmp_path, mph=20., tiers={'left': 'no', 'right': 'no'}), WHEN[when])
  sim.tracks = parked_at_corner()
  run_to_line(sim)
  held_s = sim.s
  sim.run(40.)
  plans = sim.plans()
  a = arrived(plans)
  crept = [i for i, p in enumerate(plans) if str(p.goCertified.cause) == 'creep']
  k = go_frame(sim)
  assert crept and k is not None and crept[0] < k, [(str(p.stopGo.state), p.reason) for p in plans[a::40]]
  assert any(str(plans[i].mapControl.action) == 'creeping' and plans[i].reason == sg.VIEW_CREEP_REASON for i in range(a, k))
  pos0, pos = plans[a].stopGo.positionM, plans[k].stopGo.positionM
  assert pos - pos0 > sg.VIEW_CREEP_M  # more than one step
  assert plans[k].stopGo.dEdgeMin - pos >= sr.go_room_vision_m(sr.t_abort(0.3)) - 1e-6  # never inside the creep's floor
  said = [i for i in range(a, k) if plans[i].controlPrompt == 'stopGo']
  assert said and all(plans[i].reason == sg.LOOK_REASON for i in said)
  assert not handed_back(plans) and not any(p.stopGo.fault for p in plans) and caps_ok(plans)
  assert sim.s > held_s + 15. and across(plans)


@pytest.mark.parametrize('when', list(WHEN))
def test_on_a_limited_view_a_crossing_car_seen_holds_the_go_until_it_has_passed_then_the_car_goes(tmp_path, when):
  # the same corner; once the creep has reached its limit a car comes down the right lane at 8 m/s from 45 m: held while it
  # comes, the long look's window run again after it has gone, then across, never a handback
  sim = at(vsim(tmp_path, mph=20., tiers={'left': 'no', 'right': 'no'}), WHEN[when])
  parked = parked_at_corner()
  car, gone = crossing(role='right')
  state = {}

  def tracks(s):
    if 'on' not in state and s.tp.stop_go.view_limited:
      state['on'] = len(s.plans())
    return parked(s) + (car(s) if 'on' in state else [])
  sim.tracks = tracks
  run_to_line(sim)
  sim.run(50.)
  plans = sim.plans()
  k = go_frame(sim)
  assert 'on' in state and k is not None and k > state['on'], [(str(p.stopGo.state), p.reason) for p in plans[::80]]
  assert any(plans[i].reason == 'Waiting: a car from the right' for i in range(state['on'], k)), \
    {plans[i].reason for i in range(state['on'], k)}
  waits = [i for i in range(state['on'], k) if plans[i].reason == 'Waiting: a car from the right']
  assert (k - waits[-1]) * DT_MDL >= sr.VISION_LONG_CLEAR_S - 0.06
  assert not handed_back(plans) and not any(p.stopGo.fault for p in plans) and caps_ok(plans) and across(plans)


@pytest.mark.parametrize('when', list(WHEN))
def test_a_stuck_left_side_camera_feed_is_the_camera_fault_through_turnpilotd_said_then_handed_back(tmp_path, monkeypatch, when):
  # the review of 2026-11-08: a side run whose every sector reads frozen (sideview: the feed's luma repeated bit for bit) carried
  # into the decider through turnpilotd's runs counts as no run: "side camera frozen (left)", then the take-control (H1) at the
  # fault's grace, never a go on it and never a wait with no bound
  def fn(view):
    return far_sectors('frozen', 0.)(view) if view == 'sideLeft' else far_sectors('ok', 90.)(view)
  monkeypatch.setattr(ts, 'sectors', fn)
  sim = at(vsim(tmp_path), WHEN[when])
  run_to_line(sim)
  held_s = sim.s
  sim.run(16.)
  plans = sim.plans()
  faults = [str(p.stopGo.fault) for p in plans if str(p.stopGo.fault) not in ('', 'none')]
  assert faults and set(faults) == {'H1'}, sorted(set(faults))
  assert any(p.reason == "Waiting: side camera frozen (left)" for p in plans), {p.reason for p in plans[-400::40]}
  assert handed_back(plans) and go_frame(sim) is None and sim.s < held_s + 1. and caps_ok(plans)
