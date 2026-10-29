"""turnpilotd with the stop-sign decider in the loop (stop_go.py; STOP_SIGNS.md v3 SG4a, 8.1's closed loop): test_turnpilotd's
Sim with no gas taps (go_taps False), the detector's held sweep fed to the junction watch every frame (one view a frame, the
side views' sectors ok), the visibility file, the surveys and the clock injected, a model that holds a standstill, drives
once rolling, or stops in the band, G5's lift as plannerd applies it (read from the plan's goCertified, under its gates), and
the car following its command an actuator lag late. The junction is test_certify's surveyed, validated 4-way all-way."""
from collections import deque

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.go_override import GO_ACCEL, GO_END_SPEED
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.map_controls import STOP_SIGN_REASON
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, LAT, LON, NODE, survey, visibility
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.tests.synthetic import model_v2
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import FakeParams, Sim
from openpilot.selfdrive.turnpilot.tests.test_visibility import raw_box, sectors
from openpilot.selfdrive.turnpilot.turnpilotd import stop_sign_mode_for

CYCLE = ('full', 'sideLeft', 'road', 'sideRight')
S_LINE = 200.  # the stop point along the route
MPH = sr.MPH


def info(mph=25., maneuver='straight', tiers=None):
  """The stop control's junctionInfo (log.capnp MapJunction) for test_certify's junction, our heading 0, its stop point 13 m
  before the node."""
  v, t = mph * MPH, tiers or {}

  def leg(bearing, rel, role, way):
    return {'bearing': bearing, 'relBearing': rel, 'role': role, 'roadClass': 'residential', 'use': 'road', 'inbound': True,
            'outbound': True, 'lanesIn': 1, 'lanesOut': 1, 'valhallaControl': 'stop', 'tier': t.get(role, 'surveyed'), 'assumedLimit': v,
            'wayId': way, 'bikeLane': False, 'shoulderM': -1.}
  return {'status': 'ok', 'osmNodeId': NODE, 'lat': LAT, 'lon': LON, 'stopPointDistance': 13., 'stopPointSource': 'default',
          'headingIn': 0., 'maneuver': maneuver, 'turnAngle': 0., 'roundabout': False, 'divided': False, 'railDistance': -1.,
          'sidecar': True, 'suspected': False, 'surveyed': True,
          'approaches': [leg(270., -90., 'left', 11), leg(90., 90., 'right', 12), leg(0., 0., 'opposite', 13)]}


class FakeTracker:
  def __init__(self, runs, tracks=()):
    self.new_runs, self.tracks, self.available = runs, list(tracks), True


class StopSim(Sim):
  """test_turnpilotd's Sim at a stop sign under the decider (module docstring). model: 'holds', 'goes' or 'stops'."""
  def __init__(self, tmp_path, mode='auto', vis=None, model='goes', lag_s=0.3, info_kw=None, entries=None, **kw):
    super().__init__([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(S_LINE, 'stop', False, True, False, info(**(info_kw or {})))],
                     **kw)
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
      raw = RawFrame(list(self.boxes(self).get(view, [])), 'road' if view == 'road' else 'wide', self.t_cam, view if side else 'full')
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


def test_V19_a_car_backing_out_ahead_holds_the_go(tmp_path):
  sim = StopSim(tmp_path)
  run_to_line(sim)
  sim.run(8., objects=lambda s: ({'cls': 'car', 'x': 6.5, 'y': 0.3, 'vx': -0.8, 'inPath': True, 'camera': 'wide', 'view': 'full',
                                  'tSeen': s.t_cam},))
  assert go_frame(sim) is None and sim.trace[-1][2].shouldStop


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
