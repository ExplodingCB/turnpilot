"""turnpilotd with the stop-sign decider in the loop (stop_go.py; STOP_SIGNS.md v3 SG4a, 8.1's closed loop): test_turnpilotd's
Sim with no gas taps (go_taps False), the detector's held sweep fed to the junction watch every frame (one view a frame, the
side views' sectors ok), the visibility file, the surveys and the clock injected, a model that holds a standstill, drives
once rolling, or stops in the band, G5's lift as plannerd applies it (read from the plan's goCertified, under its gates), and
the car following its command an actuator lag late. The junction is test_certify's surveyed, validated 4-way all-way."""
from collections import deque

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.go_override import GO_ACCEL, GO_END_SPEED
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.map_controls import STOP_SIGN_REASON
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, LAT, LON, NODE, survey, visibility
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.tests.synthetic import model_v2
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Desire, FakeParams, Sim
from openpilot.selfdrive.turnpilot.tests.test_visibility import raw_box, sectors
from openpilot.selfdrive.turnpilot.turnpilotd import stop_sign_mode_for

CYCLE = ('full', 'sideLeft', 'road', 'sideRight')
S_LINE = 200.  # the stop point along the route
MPH = sr.MPH


def info(mph=25., maneuver='straight', tiers=None, certification=None):
  """The stop control's junctionInfo (log.capnp MapJunction) for test_certify's junction, our heading 0, its stop point 13 m
  before the node (and navd's route-time certification when given)."""
  v, t = mph * MPH, tiers or {}

  def leg(bearing, rel, role, way):
    return {'bearing': bearing, 'relBearing': rel, 'role': role, 'roadClass': 'residential', 'use': 'road', 'inbound': True,
            'outbound': True, 'lanesIn': 1, 'lanesOut': 1, 'valhallaControl': 'stop', 'tier': t.get(role, 'surveyed'), 'assumedLimit': v,
            'wayId': way, 'bikeLane': False, 'shoulderM': -1.}
  return {'status': 'ok', 'osmNodeId': NODE, 'lat': LAT, 'lon': LON, 'stopPointDistance': 13., 'stopPointSource': 'default',
          'headingIn': 0., 'maneuver': maneuver, 'turnAngle': 0., 'roundabout': False, 'divided': False, 'railDistance': -1.,
          'sidecar': True, 'suspected': False, 'surveyed': True,
          'approaches': [leg(270., -90., 'left', 11), leg(90., 90., 'right', 12), leg(0., 0., 'opposite', 13)],
          **({'certification': certification} if certification else {})}


class FakeTracker:
  def __init__(self, runs, tracks=()):
    self.new_runs, self.tracks, self.available = runs, list(tracks), True


class StopSim(Sim):
  """test_turnpilotd's Sim at a stop sign under the decider (module docstring). model: 'holds', 'goes' or 'stops'."""
  def __init__(self, tmp_path, mode='auto', vis=None, model='goes', lag_s=0.3, info_kw=None, entries=None, maneuvers=None, **kw):
    super().__init__(maneuvers or [(1000., "turn", "right"), (2000., "turn", "left")],
                     controls=[(S_LINE, 'stop', False, True, False, info(**(info_kw or {})))], **kw)
    self.go_taps = False
    tp = self.tp
    tp.stop_sign_mode, tp.stop_sign_known = mode, True
    self.vis = vis if vis is not None else visibility(tmp_path)
    tp.vis_source = lambda: self.vis
    tp.survey_source = lambda: (entries if entries is not None else [survey()])
    tp.wall_clock = lambda: ETA + self.t
    tp.cp_delay = lag_s
    self.behaviour = model
    self.lag = deque([0.] * max(round(lag_s / DT_MDL), 1))
    self.k, self.t_cam = 0, 100.
    self.boxes = lambda sim: {}
    self.tracks = lambda sim: ()
    self.heads = lambda sim: ()  # a forward run's traffic-light boxes (RawFrame.lights: bearing, elevation, conf)
    self.engaged = True
    self.brake = False
    self.side_status = lambda sim: 'running'
    self.accel = self.planner
    self.model = self.model_msg
    self.committed_s = None

  def model_msg(self, sim):
    goes = self.behaviour in ('goes', 'stops') and self.v >= 0.3
    if self.behaviour == 'stops' and self.committed_s is not None and self.s - self.committed_s > 1.5:
      goes = False
    return model_v2(holds=not goes)

  def planner(self, sim, plan):
    """plannerd's min() of the model's candidate (lifted by G5 under its gates) and TurnPilot's, the command a lag late."""
    goes = self.behaviour in ('goes', 'stops') and self.v >= 0.3 and not (self.behaviour == 'stops' and self.committed_s is not None and
                                                                          self.s - self.committed_s > 1.5)
    a_model, model_stop = (1.0, False) if goes else ((-0.4 if self.v > 0.05 else 0.), self.v < 0.3)
    if self.behaviour == 'stops' and not goes and self.committed_s is not None:
      a_model = -1.5 if self.v > 0.05 else 0.
    a_tp = plan_accel(plan.accelCap, plan.speedCap, self.v)
    lifted = str(plan.goCertified.cause) != 'none' and not plan.shouldStop and not (self.v < 0.3 and a_tp < 0.1) and \
      self.v < GO_END_SPEED and a_model >= -1.
    a = min(max(a_model, GO_ACCEL) if lifted else a_model, a_tp)
    if plan.shouldStop or (model_stop and not lifted):
      a = min(a, -1.) if self.v > 0. else 0.
    if self.brake:
      a = -2.
    self.lag.append(a)
    return self.lag.popleft()

  def step(self, *args, **kw):
    view = CYCLE[self.k % len(CYCLE)]
    self.k += 1
    self.t_cam += DT_MDL
    side = view.startswith('side')
    runs = []
    if not side or self.side_status(self) in ('running', 'noRoom'):
      raw = RawFrame(list(self.boxes(self).get(view, [])), 'road' if view == 'road' else 'wide', self.t_cam, view if side else 'full',
                     lights=() if side else tuple(self.heads(self)))
      runs = [(view, self.t_cam, raw, sectors(view) if side else None)]
    self.tp.side_status = self.side_status(self)
    self.tp.observe_runs(FakeTracker(runs, self.tracks(self)))
    kw.setdefault('engaged', self.engaged)
    plan = super().step(*args, **kw)
    if str(plan.stopGo.state) == 'committed' and self.committed_s is None:
      self.committed_s = self.s
    return plan

  def plans(self):
    return [p for _, _, p in self.trace]


def run_to_line(sim, limit=60.):
  while not (sim.v < 0.05 and abs(S_LINE - sim.s) < 6.):
    sim.step()
    assert sim.t < limit, "never stopped"


def go_frame(sim):
  return next((k for k, p in enumerate(sim.plans()) if str(p.goCertified.cause) == 'stopSign'), None)


# ------------------------------------------------------------------------------------------------------------- the tests

def test_C1_an_all_way_under_auto_with_its_data_goes_by_itself_no_gas_tap_and_crosses(tmp_path):
  sim = StopSim(tmp_path)
  run_to_line(sim)
  held_s = sim.s
  sim.run(14.)
  plans = sim.plans()
  k = go_frame(sim)
  assert k is not None, [(str(p.stopGo.state), p.reason) for p in plans[-300::40]]
  go = plans[k]
  assert not go.shouldStop and go.controlPrompt == 'stopGo' and str(go.stopGo.state) == 'going' and go.speedCap <= sr.STOP_GO_V_MAX
  assert not any(p.controlPrompt == 'stopSignTakeControl' for p in plans)
  assert sim.s > held_s + 15. and any(str(p.stopGo.state) in ('inLane', 'done') for p in plans)
  assert all(plan_accel(p.accelCap, p.speedCap, 0.) >= -2. - 1e-9 or not p.speedCap for p in plans)
  waiting = [p for p in plans if p.controlPrompt == 'waitingForCross']
  assert waiting and all(str(p.mapControl.action) == 'waitingCross' for p in waiting[1:])
  assert str(plans[k + 2].detectorView) == 'stopSweep' and any(str(p.detectorView) == 'stopSweepGo' for p in plans[k:])


def test_O10_shadow_or_off_keeps_the_1_s_handback_and_shadow_logs_what_auto_would_do(tmp_path):
  for mode in ('shadow', 'off'):
    sim = StopSim(tmp_path, mode=mode)
    run_to_line(sim)
    sim.run(4.)
    plans = sim.plans()
    latched = next(k for k, q in enumerate(plans) if str(q.mapControl.action) in ('held', 'awaitingDriver') and q.shouldStop)
    handed = next(k for k, q in enumerate(plans) if q.controlPrompt == 'stopSignTakeControl')
    assert abs((handed - latched) * DT_MDL - 1.0) <= 2 * DT_MDL  # STOP_SIGN_HANDBACK_S, as before
    p = sim.trace[-1][2]
    assert p.shouldStop and p.controlPrompt == 'stopSignTakeControl' and p.reason == STOP_SIGN_REASON
    assert go_frame(sim) is None and str(p.stopGo.mode) == mode
    assert p.stopGo.wouldGo == (mode == 'shadow') or mode == 'off'  # the same inputs auto would have gone on


def test_O11_auto_without_its_camera_data_hands_back_at_1_s_and_says_why(tmp_path):
  import openpilot.selfdrive.turnpilot.routing.tests.test_certify as tc
  sim = StopSim(tmp_path, vis=tc.visibility(tmp_path, cameraSetbackM=None))
  run_to_line(sim)
  sim.run(1.5)
  p = sim.trace[-1][2]
  assert p.controlPrompt == 'stopSignTakeControl' and p.reason == "Stop-sign go off: no camera measurements" and go_frame(sim) is None


def test_O16_a_build_without_the_param_acts_as_shadow_out_loud():
  assert stop_sign_mode_for(FakeParams()) == ('auto', True)
  assert stop_sign_mode_for(FakeParams(TurnPilotStopSignGo=b'trial')) == ('trial', True)
  assert stop_sign_mode_for(FakeParams(TurnPilotStopSignGo=b'yes please')) == ('shadow', False)

  class Old:
    def get(self, key):
      from openpilot.common.params import UnknownKeyName
      raise UnknownKeyName(key)
  assert stop_sign_mode_for(Old()) == ('shadow', False)


def test_O17_a_go_withdrawn_at_speed_with_the_cars_actuator_delay_ends_short_of_d_edge_min(tmp_path):
  for lag in (0.2, 0.4, 0.6):
    sim = StopSim(tmp_path, lag_s=lag, model='goes')
    run_to_line(sim)
    s_stop = sim.s
    for _ in range(round(20. / DT_MDL)):
      p = sim.step()
      if str(p.stopGo.state) == 'going' and sim.v > 1.2:
        break
    # a car in the near lane, short of our commit point: the capped abort
    sim.boxes = lambda s: {v: [raw_box(65., conf=0.5, xy=(8., 17.), view='sideLeft')] for v in CYCLE}
    sim.run(6.)
    plans = sim.plans()
    assert any(p.stopGo.withdrawn for p in plans), lag
    d_min = plans[-1].stopGo.dEdgeMin
    assert sim.s - s_stop - 0.5 <= d_min + 0.6 and sim.v < 0.05  # standing short of the closest plausible edge (its stop point
    assert all(plan_accel(q.accelCap, q.speedCap, 1.) >= -2. - 1e-9 for q in plans)  # 0.5 m short of the line, within LINE_M)


def test_O5_O15_a_model_that_never_takes_over_is_withdrawn_retried_and_handed_back_h5(tmp_path):
  sim = StopSim(tmp_path, model='holds')
  run_to_line(sim)
  sim.run(40.)
  plans = sim.plans()
  causes = [k for k, p in enumerate(plans) if str(p.goCertified.cause) == 'stopSign' and (k == 0 or str(plans[k - 1].goCertified.cause) != 'stopSign')]
  assert 2 <= len(causes) <= 3
  ids = {plans[k].goCertified.id for k in causes}
  assert len(ids) == len(causes)  # each retry a fresh go id with its own window
  last = plans[-1]
  assert last.controlPrompt == 'stopSignTakeControl' and last.reason == "Can't start: take control" and last.shouldStop
  assert not any(p.controlPrompt == 'cantStartTakeControl' for p in plans)  # the decider's watchdog, not the no-move clock's


def test_O8_a_reroute_while_held_keeps_the_deciders_state(tmp_path):
  sim = StopSim(tmp_path)
  run_to_line(sim)
  sim.run(1.6)
  sg = sim.tp.stop_go
  token, arrived = sg.token, sg.arrived_t
  assert token is not None and arrived is not None
  sim.navd.route_count += 1  # navd reroutes: the same control comes back under a new key (adopt carries its state)
  sim.run(0.5)
  assert sg.token is token and sg.arrived_t == arrived and sg.key[0] == sim.navd.route_count


def test_O7_the_brake_during_the_go_ends_the_lift_and_the_decider_resets(tmp_path):
  sim = StopSim(tmp_path, model='holds')
  run_to_line(sim)
  while go_frame(sim) is None:
    sim.step()
    assert sim.t < 60.
  sim.step()
  sim.brake, sim.engaged = True, False  # the brake disengages openpilot
  sim.run(1.)
  plans = sim.plans()
  assert str(plans[-1].goCertified.cause) == 'none' and str(plans[-1].stopGo.state) in ('done', 'off')


def shield_tracks(sim, role='left', tid=8):
  """A car waiting at the role line (its box centre LINE_D m down its lane), as the tracker reports it from where our car is
  now: its bearing and box from here, on its side view."""
  import math
  from openpilot.selfdrive.turnpilot.routing.tests.test_certify import junction
  from openpilot.selfdrive.turnpilot.stop_evidence import junction_lanes
  from openpilot.selfdrive.turnpilot.tests.test_stop_evidence import track as ev_track
  from openpilot.selfdrive.turnpilot.tests.test_stop_go import LINE_D
  ln = {x.role: x for x in junction_lanes(junction(), 'unknown', 1.9, 3.5) if x.index == 1}[role]
  s = (ln.s_min + ln.s_max) / 2. - (sim.s - S_LINE)
  trk = ev_track(tid, sim.t_cam, LINE_D, ln, view='sideLeft' if role == 'left' else 'sideRight', cls=2, s=s)
  half = math.degrees(math.atan2(0.9, math.hypot(s, LINE_D)))
  trk.span_seen = tuple(sorted((trk.bearing_seen - half, trk.bearing_seen + half)))
  trk.hits = 5
  return [trk]


def test_C1_V25_a_car_waiting_at_its_line_is_crossed_past_no_band_abort_no_h5(tmp_path):
  # closed loop: a car at the left line for the whole encounter (a later arrival waiting for us, or a shield). Moving, our
  # car restarts its trust, and the band had read it as a car not stopping: an in-band abort at 4.88 m, standing 0.05-0.16 m
  # short of dEdgeMin, then H5 'no room for a go'. It is known at the go and hasn't moved: we cross
  sim = StopSim(tmp_path, model='goes')
  run_to_line(sim)
  sim.tracks = lambda s: shield_tracks(s, 'left')
  sim.run(25.)
  plans = sim.plans()
  k = go_frame(sim)
  assert k is not None and any(ln.role == 'left' and ln.shielded for ln in plans[k - 1].stopGo.lanes)
  assert not [p.stopGo.withdrawn for p in plans if p.stopGo.withdrawn]
  assert not any(p.controlPrompt == 'stopSignTakeControl' for p in plans)
  assert any(str(p.stopGo.state) == 'done' for p in plans) and sim.s > S_LINE + 15.


def test_V9_V10_a_car_at_its_line_that_never_goes_is_a_deadlock_resolved_by_one_creep_and_our_go_no_handback(tmp_path):
  # closed loop: a car stands at the left line from before our arrival and never takes its turn (a driver waving us
  # through): one creep go (cause creep, certified for G5's lift) after DEADLOCK_S, then the stop-sign go with the order
  # waived, and across with it still standing; never a handback (SG4a had H3 after 3 x DEADLOCK_S)
  sim = StopSim(tmp_path, model='goes')
  sim.tracks = lambda s: shield_tracks(s, 'left')
  run_to_line(sim)
  sim.run(25.)
  plans = sim.plans()
  causes = [str(p.goCertified.cause) for p in plans]
  k = go_frame(sim)
  assert 'creep' in causes and k is not None and causes.index('creep') < k
  assert not any(p.controlPrompt == 'stopSignTakeControl' for p in plans)
  assert any(str(p.stopGo.state) == 'done' for p in plans) and sim.s > S_LINE + 15.
  assert all(p.accelCap >= -2. - 1e-9 for p in plans)


def test_G18_a_second_stop_sign_20_m_on_never_takes_over_a_holding_encounter(tmp_path):
  # a frontage stop 20 m past the first, engaged while the car stands at the first: after a withdrawn go (the latch let go
  # with the 'clear') and after H5, the first junction's decider still holds the car and says its handback
  sim = StopSim(tmp_path, model='holds')
  sim.navd.controls.append((S_LINE + 20., 'stop', False, True, False, info()))
  run_to_line(sim)
  s0 = sim.s
  sim.run(40.)
  plans = sim.plans()
  k = next(i for i, p in enumerate(plans) if p.stopGo.withdrawn)
  assert all(p.shouldStop or str(p.stopGo.state) in ('going', 'observe') for p in plans[k:])
  last = plans[-1]
  assert str(last.stopGo.state) == 'ladder' and last.controlPrompt == 'stopSignTakeControl' and last.shouldStop
  assert sim.s - s0 < 8.  # never carried on toward the second line


@pytest.mark.parametrize('maneuver', ['right', 'left'])
def test_after_the_clear_the_decider_reads_its_own_controls_maneuver_junction_and_route_certification(tmp_path, maneuver):
  # from the frame after the go the latch has let the control go, so neither held_control() nor an engaged control ahead
  # is it: the decider had been fed 'straight', no junction, 'unchecked' and no fix spread for every turn, so the launch's
  # re-checks, the band and every retry ran as a straight crossing (a right's blind spot, its passing-cyclist memory and its
  # GO_ROOM_RIGHT_M skipped, a left on straight's T_GAP) and the published stopGo.maneuver, which launch_verify reads, was wrong
  sim = StopSim(tmp_path, model='holds', info_kw={'maneuver': maneuver, 'certification': 'certifiable'},
                maneuvers=[(S_LINE + 13., 'turn', maneuver), (2000., 'turn', 'right' if maneuver == 'left' else 'left')])
  seen = []
  orig = sim.tp.stop_go.update

  def update(f):
    d = orig(f)
    seen.append((d.state, f.maneuver, f.junction is not None and f.junction.status == 'ok', f.route_cert, f.control is not None))
    return d
  sim.tp.stop_go.update = update
  run_to_line(sim)
  sim.run(30.)
  plans = sim.plans()
  k = go_frame(sim)
  assert k is not None and len({p.goCertified.id for p in plans if str(p.goCertified.cause) == 'stopSign'}) >= 2  # a retry
  live = [x for x in seen if x[0] != 'off']
  released = [x for x in live if not x[4]]
  assert released and all(x[1:4] == (maneuver, True, 'certifiable') for x in live), sorted(set(live))[:6]
  assert all(str(p.stopGo.maneuver) == maneuver for p in plans[k:] if str(p.stopGo.state) != 'off')


@pytest.mark.parametrize('maneuver', ['right', 'left'])
def test_a_turn_at_the_stop_sign_takes_its_first_pulse_at_the_deciders_go_never_skipped_too_late(tmp_path, maneuver):
  # its pulse waits for the go (2.4 item 3), 13 m from the turn: on the go frame the decider is 'going', which had let
  # turn_skip skip the turn tooLate (no desire ever, 'Turn missed', a reroute, the car across the junction straight)
  sim = StopSim(tmp_path, model='goes', info_kw={'maneuver': maneuver},
                maneuvers=[(S_LINE + 13., 'turn', maneuver), (2000., 'turn', 'right' if maneuver == 'left' else 'left')])
  run_to_line(sim)
  sim.run(8.)
  plans = sim.plans()
  k = go_frame(sim)
  assert k is not None and not any(p.tooFastToTurn for p in plans) and not sim.tp.skip.skipped
  want = Desire.turnRight if maneuver == 'right' else Desire.turnLeft
  assert all(p.desire == Desire.none for p in plans[:k]) and plans[k].desire == want
  # a holding model: the go withdrawn short, and its own pulse is no reason to wait out the static bound: it goes again
  sim = StopSim(tmp_path, model='holds', info_kw={'maneuver': maneuver},
                maneuvers=[(S_LINE + 13., 'turn', maneuver), (2000., 'turn', 'right' if maneuver == 'left' else 'left')])
  run_to_line(sim)
  sim.run(14.)
  plans = sim.plans()
  gos = {p.goCertified.id for p in plans if str(p.goCertified.cause) == 'stopSign'}
  assert len(gos) >= 2 and not any(p.tooFastToTurn for p in plans)
  assert not any("isn't straight" in p.reason for p in plans)


@pytest.mark.parametrize('short', [4.3, 7.])
def test_O2_a_roll_up_g5_never_lifts_hands_back_h5_never_a_silent_stand(tmp_path, short):
  # TurnPilotGoOverride off: the rollUp goes are never certified, and a holding model leaves the car standing short of the
  # stop point where something stopped it (4.3 m: latched, 7 m: never latched). It stood about 90 s saying only "Rolling up
  # to the stop sign" (or the map's "Stop sign ahead") until the generic carStoppedTakeControl; now H5 after ROLLUP_WAIT_S
  from openpilot.selfdrive.turnpilot.stop_go import ROLLUP_WAIT_S
  sim = StopSim(tmp_path, model='holds')
  sim.tp.go_override = False
  base = sim.planner

  def planner(s, plan):
    a = base(s, plan)
    room = S_LINE - short - s.s
    if s.v > 0.01 and room < 25.:
      a = min(a, -s.v * s.v / (2. * max(room, 0.2)))
    return a
  sim.accel = planner
  while not (sim.v < 0.02 and sim.s > S_LINE - 30.):
    sim.step()
    assert sim.t < 80.
  sim.accel = base
  t0 = len(sim.plans())
  sim.run(ROLLUP_WAIT_S + 10.)
  plans = sim.plans()[t0:]
  k = next((i for i, p in enumerate(plans) if p.controlPrompt == 'stopSignTakeControl'), None)
  assert k is not None and k * DT_MDL <= ROLLUP_WAIT_S + 6. and plans[k].reason == "Can't start: take control"
  assert all(p.reason == "Rolling up to the stop sign" for p in plans[round(1.5 / DT_MDL):k])
  assert all(p.shouldStop for p in plans[k + 1:]) and S_LINE - sim.s > 3.
  assert not any(p.controlPrompt == 'carStoppedTakeControl' for p in plans)


class OverridesParam(dict):
  """TurnPilotJunctionOverrides as Params holds it (a STRING key, json text); fail: put raises it (a build that predates the
  key, a full disk)."""
  def __init__(self, fail=None):
    super().__init__()
    self.fail = fail

  def get(self, key):
    return super().get(key)

  def put(self, key, value):
    if self.fail is not None:
      raise self.fail
    self[key] = value


def test_C20_a_signal_head_at_the_junction_writes_signal_seen_for_navd_and_a_failed_write_is_said(tmp_path):
  # 2.4 item 9: a class-9 head over the junction on 3 of 5 forward runs makes it uncertifiable for the encounter (H3 'signal
  # at junction') and names it in TurnPilotJunctionOverrides (signalSeen), so navd routes around it from then on; StopGo's
  # hook did nothing. Not written, it is logged and said (overrideWriteFailed), never silently lost (O16)
  import json
  from openpilot.common.params import UnknownKeyName
  from openpilot.selfdrive.turnpilot.routing.junctions import read_overrides
  for fail in (None, UnknownKeyName('TurnPilotJunctionOverrides')):
    sim = StopSim(tmp_path)
    sim.tp.params = OverridesParam(fail)
    sim.heads = lambda s: ((2., 12., 0.7),)  # over the junction, above the horizon
    run_to_line(sim)
    sim.run(20.)
    plans = sim.plans()
    assert go_frame(sim) is None
    assert any(p.controlPrompt == 'stopSignTakeControl' and p.reason == "Stop sign: signal at junction: take control" for p in plans)
    alerts = [str(p.controlAlert) for p in plans]
    if fail is None:
      got, ok, _ = read_overrides(sim.tp.params)
      assert ok and got and all(v['kind'] == 'signalSeen' for v in got.values())
      assert {k.split(':')[0] for k in got} == {str(NODE)} and {k.split(':')[1] for k in got} == {'11', '12', '13'}
      assert len(json.loads(sim.tp.params['TurnPilotJunctionOverrides'])) == 6 and 'overrideWriteFailed' not in alerts
    else:
      assert 'TurnPilotJunctionOverrides' not in sim.tp.params and 'overrideWriteFailed' in alerts


def lane_tracks(sim, role, tid, d, cls=2):
  """A car d m down the role lane (its box centre), as the tracker reports it from where our car is now (shield_tracks')."""
  import math
  from openpilot.selfdrive.turnpilot.routing.tests.test_certify import junction
  from openpilot.selfdrive.turnpilot.stop_evidence import junction_lanes
  from openpilot.selfdrive.turnpilot.tests.test_stop_evidence import track as ev_track
  ln = {x.role: x for x in junction_lanes(junction(), 'unknown', 1.9, 3.5) if x.index == 1}[role]
  s = (ln.s_min + ln.s_max) / 2. - (sim.s - S_LINE)
  trk = ev_track(tid, sim.t_cam, max(d, 0.3), ln, view='sideLeft' if role == 'left' else 'sideRight', cls=cls, s=s)
  half = math.degrees(math.atan2(0.9, math.hypot(s, d)))
  trk.span_seen = tuple(sorted((trk.bearing_seen - half, trk.bearing_seen + half)))
  trk.hits = 5
  return [trk]


def test_V3_a_car_through_its_line_at_speed_writes_its_approach_no_with_its_evidence_and_a_failed_write_is_said(tmp_path):
  # 4.4, decision D11: the left approach no for the encounter at once (H3 "cross traffic doesn't stop" after the ledger) and
  # in TurnPilotJunctionOverrides as no with its evidence, so navd avoids it until Kellen clears it (route_audit --clear).
  # Not written, it is logged and said (overrideWriteFailed), never silently lost
  import json
  from openpilot.common.params import UnknownKeyName
  from openpilot.selfdrive.turnpilot.routing.junctions import read_overrides
  for fail in (None, UnknownKeyName('TurnPilotJunctionOverrides')):
    sim = StopSim(tmp_path)
    sim.tp.params = OverridesParam(fail)
    run_to_line(sim)
    t0 = sim.t_cam + 1.

    def tracks(s, t0=t0):
      d = 45. - 8. * (s.t_cam - t0)
      return lane_tracks(s, 'left', 41, d) if s.t_cam >= t0 and d > -12. else []
    sim.tracks = tracks
    sim.run(24.)
    plans = sim.plans()
    assert go_frame(sim) is None
    assert any(p.controlPrompt == 'stopSignTakeControl' and p.reason == "Stop sign: cross traffic doesn't stop: take control" for p in plans)
    alerts = [str(p.controlAlert) for p in plans]
    if fail is None:
      got, ok, _ = read_overrides(sim.tp.params)
      assert ok and set(got) == {f"{NODE}:11:f", f"{NODE}:11:b"} and all(v['kind'] == 'no' for v in got.values())
      ev = got[f"{NODE}:11:f"]['evidence']
      assert ev['source'] == 'stopGo downgrade' and ev['trackId'] == 41 and ev['cls'] == 'car' and ev['role'] == 'left'
      assert 5. <= ev['speedMs'] <= 11. and ev['minSpeedMs'] >= 5. and ev['osmNodeId'] == NODE and ev['wallTime'] > ETA
      assert ev['written'] == [f"{NODE}:11:f", f"{NODE}:11:b"]
      assert len(json.loads(sim.tp.params['TurnPilotJunctionOverrides'])) == 2 and 'overrideWriteFailed' not in alerts
    else:
      assert 'TurnPilotJunctionOverrides' not in sim.tp.params and 'overrideWriteFailed' in alerts


def test_V19_a_car_backing_out_ahead_holds_the_go(tmp_path):
  sim = StopSim(tmp_path)
  run_to_line(sim)
  sim.run(8., objects=lambda s: ({'cls': 'car', 'x': 6.5, 'y': 0.3, 'vx': -0.8, 'inPath': True, 'camera': 'wide', 'view': 'full',
                                  'tSeen': s.t_cam},))
  assert go_frame(sim) is None and sim.trace[-1][2].shouldStop


@pytest.mark.parametrize('cls', ['car', 'pedestrian'])
def test_the_decider_reads_the_scenes_objects_as_turnpilotd_hands_them_over_never_failing_to_shadow(tmp_path, cls):
  # turnpilotd hands the decider the scene's TrackedObjects as capnp readers, their cls an ObjectClass enum; class_name read it
  # with int(), which an enum refuses, so any object near a held stop sign made the decider fail every frame (logged
  # turnpilotd.stop_go_failed) and act as shadow: the 1 s handback, the decider never running with anything in view
  sim = StopSim(tmp_path)
  run_to_line(sim)
  sim.run(4., objects=lambda s: ({'cls': cls, 'x': 30., 'y': 9., 'vx': 0., 'inPath': False, 'camera': 'wide', 'view': 'full',
                                  'tSeen': s.t_cam},))
  states = [str(p.stopGo.state) for p in sim.plans()[-60:]]
  assert all(st not in ('off',) for st in states), states
  assert str(sim.plans()[-1].controlPrompt) != 'stopSignTakeControl'


def test_the_no_move_clock_arms_on_a_straight_release_from_a_standstill(tmp_path):
  # gap B: a hold TurnPilot let go of at a standstill on a straight road (here a sim green) the model never pulls away from is
  # withdrawn and handed back like a turn's (cantStartTakeControl), not left standing silently for NEAR_CONTROL_STAND_S
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import oracle_lights
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'signal')])
  sim.go_taps = False
  sim.tp.oracle_ok = True
  sim.lights = oracle_lights('red', 200.)
  while not (sim.v < 0.05 and sim.s > 150.):
    sim.step()
    assert sim.t < 60.
  sim.run(1.)
  sim.lights = oracle_lights('green', 200.)
  sim.model_holds = True
  for _ in range(round(4. / DT_MDL)):
    sim.v = 0.
    sim.step(engaged=True)
  assert any(p.controlPrompt == 'cantStartTakeControl' for _, _, p in sim.trace)


def test_every_plan_carries_navds_runtime_inputs_steady_through_a_stop_and_a_handback(tmp_path):
  # STOP_SIGNS.md 3.2 (N11): navd re-runs its stop avoidance whenever turnpilotPlan.stopGo's mode or side views change, so
  # they are what turnpilotd acts in for the drive: the param's mode on every plan (the decider's shadow on a handback, a
  # disabled plan's), and the side views live unless off or failed for the drive; never the speed gate's or the sweep's
  # cycle, which would reroute at every stop sign
  sim = StopSim(tmp_path)
  sim.side_status = lambda s: ('running' if s.v <= 2. else 'speedGated') if abs(S_LINE - s.s) < 30. else 'idle'
  run_to_line(sim)
  sim.run(3.)
  sim.engaged = False  # openpilot off: the decider runs as shadow
  sim.run(1.)
  sim.long_control = False  # a disabled plan
  sim.run(0.5)
  plans = sim.plans()
  assert all(str(p.stopGo.mode) == 'auto' and p.stopGo.sideViewsLive and p.stopGo.mapsValid for p in plans)
  sim.side_status = lambda s: 'mapsStale'  # a calibration move: transient, navd doesn't reroute for it
  p = sim.step()
  assert p.stopGo.sideViewsLive and not p.stopGo.mapsValid
  for status in ('disabled', 'off'):
    sim.side_status = lambda s, status=status: status
    assert not sim.step().stopGo.sideViewsLive
  sim.tp.stop_sign_known = False  # a value this build doesn't know: shadow, on every plan
  sim.long_control = True
  assert str(sim.step().stopGo.mode) == 'shadow'


def test_C11_turnpilotd_hands_the_forward_runs_stop_sign_boxes_to_map_controls_unless_the_camera_reads_none(tmp_path, monkeypatch):
  # each tick's forward runs and their class-11 boxes (RawFrame.signs) reach map_controls (row C11's evidence at the hold);
  # with TurnPilotCameraStopSigns off modeld reads none, so nothing is counted and no stop is called unseen
  import openpilot.selfdrive.turnpilot.map_controls as mcm
  for read in (True, False):
    events = []
    monkeypatch.setattr(mcm.cloudlog, 'event', lambda name, events=events, **kw: events.append((name, kw)))
    sim = StopSim(tmp_path, mode='shadow')
    sim.tp.camera_signs = read
    run_to_line(sim)
    ev = [kw for name, kw in events if name == 'turnpilot.map_stop_signs']
    if read:
      assert len(ev) == 1 and ev[0]['runs'] >= 10 and ev[0]['hits'] == 0 and any(name == 'turnpilot.map_stop_unseen' for name, _ in events)
    else:
      assert not ev and not any(name == 'turnpilot.map_stop_unseen' for name, _ in events)


def test_C11_the_camera_stop_signs_param_off_reads_none_anything_else_reads_them():
  from openpilot.selfdrive.turnpilot.turnpilotd import camera_signs_read
  assert camera_signs_read(FakeParams()) and camera_signs_read(FakeParams(TurnPilotCameraStopSigns=b'shadow'))
  assert not camera_signs_read(FakeParams(TurnPilotCameraStopSigns=b'off'))


def test_the_ladders_other_turn_pulses_its_own_turn_at_the_go_never_the_routes_and_publishes_the_planned_turn_missed(tmp_path):
  # 7.1 rung 2, 3.5: going straight, a car stopped on the exit leg (no room past the junction: a static wait); at its bound
  # the decider takes a right (single-lane approach), said (otherTurn, stopGo.alternate, stopGo.maneuver right). turnpilotd
  # feeds DesireTiming the synthesized right (its own maneuver id) from the go: turnRight goes out at the go and never a turn
  # for the route; once the encounter is done the route's planned turn is published missed (turnMissed) for navd to reroute
  sim = StopSim(tmp_path)
  def stopped(s):  # on the exit leg 26 m ahead while we stand at the line (out of the scene once we move)
    return ({'cls': 'car', 'x': 26., 'y': 0., 'vx': 0., 'vy': 0., 'inPath': True, 'camera': 'wide', 'view': 'full',
             'tSeen': s.t_cam},) if s.v < 0.5 and abs(S_LINE - s.s) < 6. else ()
  run_to_line(sim)
  start = len(sim.trace)
  turned = 0.
  while sim.t < 80. and not any(p.turnMissed for p in sim.plans()[start:]):
    moving = sim.v > 0.5 and str(sim.trace[-1][2].stopGo.state) in ('committed', 'inLane')
    yaw = -0.6 if moving and turned < 80. else 0.
    turned += abs(yaw) * DT_MDL * 57.3
    sim.step(objects=stopped(sim), yaw_rate=yaw)
  plans = sim.plans()[start:]
  alt = next((k for k, p in enumerate(plans) if str(p.controlAlert) == 'otherTurn'), None)
  assert alt is not None, [(str(p.stopGo.state), p.reason) for p in plans[::40]]
  assert plans[alt].stopGo.alternate and str(plans[alt].stopGo.maneuver) == 'right'
  go = next(k for k, p in enumerate(plans) if str(p.goCertified.cause) == 'stopSign')
  assert go > alt
  desires = [p.desire for p in plans]
  assert Desire.turnLeft not in desires and Desire.turnRight in desires[go:go + 3] and Desire.turnRight not in desires[:go]
  assert any(p.turnMissed for p in plans[go:]) and not any(p.turnMissed for p in plans[:go])
  assert all(p.accelCap >= -2. - 1e-9 for p in plans)


def test_the_route_start_and_pre_stop_notices_reach_the_plan_disengaged(tmp_path):
  # STOP_SIGNS.md 3.4 (route_notice.py): navd's avoidance said at route start with openpilot not engaged, the text on the
  # plan; 300 m before the stop the route couldn't avoid, "Stop sign ahead needs you"
  sim = StopSim(tmp_path, info_kw={'certification': 'unavoidable'})
  sim.engaged = False

  def nav(s, instr):
    b = instr.as_builder()
    b.avoidStatus, b.avoidedControls, b.avoidAddedS, b.unavoidableControls = 'unavoidable', 2, 120., 1
    return b.as_reader(), True
  sim.nav_filter = nav
  sim.v = 0.
  for _ in range(round(8. / DT_MDL)):
    sim.step()
  notices = [(str(p.controlAlert), p.routeNotice) for p in sim.plans() if str(p.controlAlert) in ('routeAvoidsStops', 'stopsNeedYou')]
  assert ('routeAvoidsStops', "Route avoids 2 stop signs (+2 min)") in notices
  assert ('stopsNeedYou', "1 stop sign on this route needs you") in notices
  sim.engaged, sim.v = True, 13.
  seen = []
  while sim.s < S_LINE - 20.:
    seen.append(str(sim.step().controlAlert))
  assert 'stopAheadNeedsYou' in seen


def test_a_route_notice_behind_another_alert_is_said_once_that_one_is_done(tmp_path):
  # 3.4: "Stop sign ahead needs you" falls due 300 m out while another alert of turnpilotd's is up (O16's
  # overrideWriteFailed, 6 s from 330 m): held back, then said in full (NOTICE_S) once the slot is free; it had been used
  # up behind the other alert and never said
  from openpilot.selfdrive.turnpilot.route_notice import NOTICE_S
  sim = StopSim(tmp_path, info_kw={'certification': 'unavoidable'})
  sim.engaged, sim.v = True, 13.
  seen, other = [], False
  while sim.s < S_LINE - 20.:
    if not other and S_LINE - sim.s < 330.:
      sim.tp.override_failed_t = 6.
      other = True
    seen.append(str(sim.step().controlAlert))
  assert other and 'overrideWriteFailed' in seen
  first = seen.index('stopAheadNeedsYou')
  assert first > max(k for k, a in enumerate(seen) if a == 'overrideWriteFailed')
  assert abs(seen.count('stopAheadNeedsYou') * DT_MDL - NOTICE_S) <= DT_MDL + 1e-6


def oracle_msg(cars=(), range_m=60.):
  """turnpilotLights from the sim's oracle: no lights, the cross traffic [(id, x, y, vx, vy, cls)] in the car frame."""
  from openpilot.cereal import messaging
  msg = messaging.new_message('turnpilotLights')
  tl = msg.turnpilotLights
  tl.source, tl.crossRangeM = 'simOracle', range_m
  cs = tl.init('cross', len(cars))
  for c, (i, x, y, vx, vy, cls) in zip(cs, cars, strict=True):
    c.id, c.x, c.y, c.vx, c.vy, c.cls = i, x, y, vx, vy, cls
  return msg.as_reader().turnpilotLights


@pytest.mark.parametrize('cert', [None, 'uncertifiable'])
def test_the_sim_cross_traffic_oracle_feeds_the_decider_ground_truth_under_the_gate_and_is_refused_out_loud_off_it(tmp_path, cert):
  # STOP_SIGNS.md 8.2 (SG4c): in the sim the decider takes ground-truth cross traffic and the oracle's range as each lane's reach
  # (labelled synthetic: binding 'oracle'), only under the light oracle's gate; a car standing at the left line before our
  # arrival is an earlier arrival: we wait for it to go (3 s after our stop, pulling away through our path) and go after it.
  # Off the gate the message is refused and said (lightSourceRejected), the cameras' reach stands. navd with TurnPilotAvoidStops
  # off (the logic gate's runs) certifies from the shipped survey, which has no sim junction: 'uncertifiable', never
  # 'unavoidable' (avoid.run marks that only while avoiding), and that is no handback where the decider's own certification
  # (the sim's survey under set_sim) passes
  from openpilot.selfdrive.turnpilot.tests.test_stop_go import LINE_D
  sim = StopSim(tmp_path, info_kw={'certification': cert} if cert else None)
  sim.tp.oracle_ok = True
  sim.tp.sim_files = True  # a sim car (set_sim's gate), with this test's own files and clock in place of the sim's
  st = {'stop': None}

  def lights(s):
    sg = s.tp.stop_go
    if st['stop'] is None and s.v < 0.05 and abs(S_LINE - s.s) < 6.:
      st['stop'] = s.t
    ln = next((x for x in sg.lanes_all if (x.role, x.index) == ('left', 1)), None)
    if sg.token is None or ln is None:
      return oracle_msg()
    pos = sg.pos_ref + (s.tp.odo_m - sg.odo_ref)
    t_go = (st['stop'] + 3.) if st['stop'] is not None else float('inf')
    gone = max(s.t - t_go, 0.)
    d = LINE_D - 1.5 * gone ** 2
    v = -3. * gone
    return oracle_msg([(7, (ln.s_min + ln.s_max) / 2. - pos, d, 0., v, 'car')] if d > -25. else [])
  sim.lights = lights
  run_to_line(sim)
  sim.run(12.)
  plans = sim.plans()
  lanes = [ln for p in plans[-100:] for ln in p.stopGo.lanes]
  assert lanes and all(ln.binding == 'oracle' for ln in lanes)
  assert all(ln.rSeen == pytest.approx(60. - sr.V_CROSS_FACTOR * 25. * MPH * sr.T_LAT, abs=0.05) for ln in lanes)
  k = go_frame(sim)
  assert k is not None and st['stop'] is not None, [(str(p.stopGo.state), p.reason) for p in plans[-240::20]]
  t_go = next(t for t, (_, _, p) in zip([i * DT_MDL for i in range(len(sim.trace))], sim.trace, strict=True) if p is plans[k])
  assert t_go > st['stop'] + 3., (t_go, st['stop'])
  assert not any(str(p.controlPrompt) == 'stopSignTakeControl' for p in plans)
  sim = StopSim(tmp_path)
  sim.tp.oracle_ok = False
  sim.lights = lambda s: oracle_msg([(7, 14.5, 30., 0., 0., 'car')])
  run_to_line(sim)
  sim.run(4.)
  assert all(ln.binding != 'oracle' for p in sim.plans()[-60:] for ln in p.stopGo.lanes)
  assert any(str(p.controlAlert) == 'lightSourceRejected' for p in sim.plans())


@pytest.mark.parametrize('car', ['tesla', 'none'])
def test_the_sim_cross_traffic_oracle_is_refused_out_loud_without_a_sim_car(tmp_path, car, monkeypatch):
  # 8.2: the cross-traffic oracle is gated as the sim's files are, SIMULATION=1, the param and a sim car in CarParams
  # (visibility_table.sim_accepted). With SIMULATION=1 and the param but a real car's CarParams (or none yet), its message is
  # refused, logged once and said (lightSourceRejected, its reason where the status line has none), and no lane reads its
  # reach (binding 'oracle'): the oracle had been taken on oracle_ok alone, OracleLive's 60 m reach in every lane over the
  # shipped files, nothing said
  from types import SimpleNamespace
  from openpilot.selfdrive.turnpilot import turnpilotd, visibility_table
  from openpilot.selfdrive.turnpilot.turnpilotd import oracle_accepted
  events = []
  orig = turnpilotd.cloudlog.event
  monkeypatch.setattr(turnpilotd.cloudlog, 'event', lambda name, *a, **kw: (events.append((name, kw)), orig(name, *a, **kw)))
  env = {'SIMULATION': '1'}
  cp = SimpleNamespace(fingerprintSource='can', brand='tesla') if car == 'tesla' else None
  sim = StopSim(tmp_path)
  sim.tp.oracle_ok = oracle_accepted(env, True)
  sim.tp.set_sim(visibility_table.sim_accepted(env, True, cp))
  assert sim.tp.oracle_ok and not sim.tp.sim_files
  sim.lights = lambda s: oracle_msg([(7, 14.5, 30., 0., 0., 'car')])
  run_to_line(sim)
  sim.run(3.)
  plans = sim.plans()
  assert all(ln.binding != 'oracle' for p in plans[-40:] for ln in p.stopGo.lanes)
  assert any(ln.binding != 'oracle' for p in plans[-40:] for ln in p.stopGo.lanes)
  assert any(str(p.controlAlert) == 'lightSourceRejected' for p in plans[-40:])
  refusals = [kw for name, kw in events if name == 'turnpilot.cross_oracle_refused']
  assert len(refusals) == 1 and refusals[0]['error'] and refusals[0]['reason'] == "not a sim car"
  # under the whole gate (a sim car) it is read, and nothing is refused
  sim = StopSim(tmp_path)
  sim.tp.oracle_ok, sim.tp.sim_files = True, True
  sim.lights = lambda s: oracle_msg([(7, 14.5, 30., 0., 0., 'car')])
  run_to_line(sim)
  sim.run(3.)
  assert any(ln.binding == 'oracle' for p in sim.plans()[-40:] for ln in p.stopGo.lanes)
  assert not any(str(p.controlAlert) == 'lightSourceRejected' for p in sim.plans())


def test_the_ladders_other_turn_is_the_scenes_maneuver_and_the_turn_caps_never_the_routes(tmp_path):
  # 7.1 rung 2: the route turns left here, a car stands on the left's exit leg (no room past the junction), and at the static
  # bound the decider takes a right. From then on the turn the car makes is the right: scene_policy's junction holds and the
  # turn's speed cap are the right's, from its junction distance. turnpilotd had fed them the route's left all along, so an
  # oncoming car (confirmed, trusted, 2 s out) held the right's launch at the left's hold line ("waiting for oncoming", a stop
  # a confirmed track may harden past -2.0) for a car that is no conflict to a right
  from openpilot.selfdrive.turnpilot import stop_go as sg_mod
  from openpilot.selfdrive.turnpilot.route_follower import RIGHT
  from openpilot.selfdrive.turnpilot.turn_speed import turn_speed_cap
  sim = StopSim(tmp_path, info_kw={'maneuver': 'left'}, maneuvers=[(S_LINE + 13., 'turn', 'left'), (2000., 'turn', 'right')])
  seen = []
  orig = sim.tp.scene.update

  def update(objects, available, maneuver, distance, v_ego, rht=True, maneuver_id=None, *a, **kw):
    d = orig(objects, available, maneuver, distance, v_ego, rht, maneuver_id, *a, **kw)
    junction = sim.tp.scene.junction
    seen.append((maneuver, distance, maneuver_id, v_ego, junction.reason if junction is not None else ''))
    return d
  sim.tp.scene.update = update
  st = {'go': None, 'k': 0}

  def objects(s):
    sg = s.tp.stop_go
    out = []
    if sg.token is not None and s.v < 0.5 and abs(S_LINE - s.s) < 6.:
      pos = sg.pos_ref + (s.tp.odo_m - sg.odo_ref)
      x0 = sg._x(sg.edges.d_max, pos) + sr.LANE_W_MAX
      out.append({'trackId': 40, 'cls': 'car', 'x': x0 + sr.LANE_W_MAX, 'y': sg_mod.BOX_LEFT_M + 3., 'vx': 0., 'vy': 0., 'inPath': False,
                  'age': 200, 'camera': 'wide', 'view': 'full', 'tSeen': s.t_cam})
    if st['go'] is not None:
      st['k'] += 1
      out.append({'trackId': 41, 'cls': 'car', 'x': 30., 'y': 3.2, 'vx': -8., 'vy': 0., 'inPath': False, 'age': 40 + st['k'],
                  'camera': 'wide', 'view': 'full', 'tSeen': s.t_cam})
    return tuple(out)
  run_to_line(sim)
  start = len(sim.trace)
  turned = 0.
  while sim.t < 90. and not any(p.turnMissed for p in sim.plans()[start:]):
    if st['go'] is None and str(sim.tp.stop_go.state) == 'going':
      st['go'] = len(sim.trace)
    moving = sim.v > 0.5 and str(sim.trace[-1][2].stopGo.state) in ('committed', 'inLane')
    yaw = -0.6 if moving and turned < 80. else 0.
    turned += abs(yaw) * DT_MDL * 57.3
    sim.step(objects=objects(sim), yaw_rate=yaw)
  plans = sim.plans()
  alt = next((k for k, p in enumerate(plans) if k >= start and str(p.controlAlert) == 'otherTurn'), None)
  assert alt is not None and str(plans[alt].stopGo.maneuver) == 'right', [(str(p.stopGo.state), p.reason) for p in plans[start::40]]
  go = next(k for k, p in enumerate(plans) if str(p.goCertified.cause) == 'stopSign')
  assert go > alt and st['go'] is not None
  # the scene's maneuver from the other turn on: the right, its junction from the decider's toNode, its own id
  assert len(seen) == len(plans)
  done = next(k for k, p in enumerate(plans) if k > go and str(p.stopGo.state) in ('done', 'off'))
  assert all(x[0] is not None and x[0].side == RIGHT and abs(x[1]) < 20. and x[2] != sim.tp.follower.maneuver_id for x in seen[alt + 1:done])
  # the scene never held the right's launch for the oncoming car (its junction hold, a right's: the crosswalk), nothing
  # below -2.0, and the plan's cap no higher than the right's turn cap
  assert not any(x[4] for x in seen[alt + 1:done]), sorted({x[4] for x in seen[alt + 1:done]})
  assert all(p.accelCap >= -2. - 1e-9 for p in plans)
  assert all(p.speedCap <= turn_speed_cap(m, d, v) + 1e-6 for p, (m, d, _, v, _) in zip(plans[alt + 1:done], seen[alt + 1:done], strict=True))
  assert any(p.desire == Desire.turnRight for p in plans[go:go + 3]) and Desire.turnLeft not in [p.desire for p in plans]


def test_a_left_taken_instead_of_the_routes_right_gets_the_scenes_oncoming_hold(tmp_path):
  # the route turns right here; its exit leg and the road straight across both have a car standing on them, so at the static
  # bound the decider takes a left (a single-lane approach). The scene's far-side oncoming hold is then the left's: a car
  # coming the other way during the left's launch holds it short of the oncoming lanes (it had had none, the scene's maneuver
  # the route's right, a curb-side turn)
  from openpilot.selfdrive.turnpilot import stop_go as sg_mod
  from openpilot.selfdrive.turnpilot.route_follower import LEFT
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
  sim = StopSim(tmp_path, info_kw={'maneuver': 'right'}, maneuvers=[(S_LINE + 13., 'turn', 'right'), (2000., 'turn', 'left')])
  seen = []
  orig = sim.tp.scene.update

  def update(objects, available, maneuver, distance, v_ego, rht=True, maneuver_id=None, *a, **kw):
    d = orig(objects, available, maneuver, distance, v_ego, rht, maneuver_id, *a, **kw)
    junction = sim.tp.scene.junction
    seen.append((maneuver, distance, junction.reason if junction is not None else ''))
    return d
  sim.tp.scene.update = update
  st = {'go': None, 'k': 0}

  def objects(s):
    sg = s.tp.stop_go
    out = []
    if sg.token is not None and s.v < 0.5 and abs(S_LINE - s.s) < 6. and st['go'] is None:
      pos = sg.pos_ref + (s.tp.odo_m - sg.odo_ref)
      x0 = sg._x(sg.edges.d_min, pos)
      out.append({'trackId': 40, 'cls': 'car', 'x': x0 + sr.LANE_W_MAX, 'y': -(sg_mod.BOX_RIGHT_M + 3.), 'vx': 0., 'vy': 0.,
                  'inPath': False, 'age': 200, 'camera': 'wide', 'view': 'full', 'tSeen': s.t_cam})
      out.append({'trackId': 42, 'cls': 'car', 'x': sg._x(sg._far_edge(), pos) + 3., 'y': 0., 'vx': 0., 'vy': 0., 'inPath': True,
                  'age': 200, 'camera': 'wide', 'view': 'full', 'tSeen': s.t_cam})
    if st['go'] is not None:
      st['k'] += 1
      out.append({'trackId': 41, 'cls': 'car', 'x': 30., 'y': 3.2, 'vx': -8., 'vy': 0., 'inPath': False, 'age': 40 + st['k'],
                  'camera': 'wide', 'view': 'full', 'tSeen': s.t_cam})
    return tuple(out)
  run_to_line(sim)
  start = len(sim.trace)
  while sim.t < 60. and (st['go'] is None or len(sim.trace) - st['go'] < round(7. / DT_MDL)):
    if st['go'] is None and str(sim.tp.stop_go.state) == 'going':
      st['go'] = len(sim.trace)
    sim.step(objects=objects(sim))
  plans = sim.plans()
  alt = next((k for k, p in enumerate(plans) if k >= start and str(p.controlAlert) == 'otherTurn'), None)
  assert alt is not None and str(plans[alt].stopGo.maneuver) == 'left', [(str(p.stopGo.state), p.reason) for p in plans[start::40]]
  assert st['go'] is not None
  assert all(x[0] is not None and x[0].side == LEFT for x in seen[alt + 1:])
  assert any(x[2] == 'waiting for oncoming' for x in seen[st['go']:]), sorted({x[2] for x in seen[st['go']:]})
  # held short of the oncoming lanes (the node is 13 m past the stop point), on the speed cap: nothing below -2.0 here
  assert sim.v < 0.3 and sim.s - S_LINE < 13. - ONCOMING_ENTRY_M
  assert all(p.accelCap >= -2. - 1e-9 for p in plans)
