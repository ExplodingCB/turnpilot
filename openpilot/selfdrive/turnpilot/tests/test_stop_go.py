"""The stop-sign decider (stop_go.py; STOP_SIGNS.md v3 sections 2, 4.5, 4.6, 6, 7.1 and 8.1; SG4a). The phantom clear first,
then the property tests, the cross-cutting tests, and each in-scope catalogue row's sg test, named for its row. The junction is
test_certify's made-up 4-way all-way in Indianapolis (every approach surveyed, 25 mph, a 48 m validation stamp on the left
and right, the opposite side stamped as if step S4 had bounded it), arriving northbound at 15:00 EDT with the sun behind;
the car stands at its stop point with a 1.9 m camera setback, and the detector runs the held sweep, one view a 20 Hz frame.
Expected values come from the geometry (repo tests can't read ~/turnpilot-artifacts)."""
import dataclasses
import math
import random
from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_go as sg
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.go_override import GO_ACCEL, GO_END_SPEED, GoCertifier
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.map_controls import HOLD_ACCEL
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, junction, stamp, survey, visibility
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import ScenePolicy, Stop
from openpilot.selfdrive.turnpilot.stop_evidence import junction_lanes
from openpilot.selfdrive.turnpilot.stop_go import FAULT_GRACE_S, FRONT_ZONE_MAX_S, ROLLUP_WAIT_S, STATIC_WAIT_S, Frame, StopGo
from openpilot.selfdrive.turnpilot.tests.synthetic import model_v2
from openpilot.selfdrive.turnpilot.tests.test_stop_evidence import track as ev_track
from openpilot.selfdrive.turnpilot.tests.test_visibility import raw_box, sectors
from openpilot.selfdrive.turnpilot.visibility import LiveVisibility

CYCLE = ('full', 'sideLeft', 'road', 'sideRight')  # the held sweep, one view a frame: each 5 Hz


class World:
  """The decider at one junction: the files, the clock, the live views at the sweep's cadence (boxes(world) -> {view: [RawBox]}),
  the tracker's tracks (tracks(world)) and the scene's objects (objects(world)), and a car standing at or moving from its stop
  point (v and a set by the test, the odometer integrated)."""
  def __init__(self, tmp_path, j=None, entries=None, vis=None, maneuver='straight', mode='auto', d0=0.5, wall_t=ETA, side='running'):
    self.sg = StopGo()
    self.live = LiveVisibility()
    self.j = j if j is not None else junction(maneuver=maneuver)
    self.entries = [survey()] if entries is None else entries
    self.vis = vis if vis is not None else visibility(tmp_path)
    self.maneuver, self.mode, self.wall_t, self.side = maneuver, mode, wall_t, side
    self.state = SimpleNamespace(moved_m=0.)
    self.control = SimpleNamespace(key=(0, 1), kind='stop', d=d0, held=True, state=self.state)
    self.t_cam, self.k, self.odo, self.v, self.a = 100., 0, 0., 0., 0.
    self.boxes = lambda w: {}
    self.tracks = lambda w: ()
    self.objects = lambda w: ()
    self.model = None
    self.kw = {}
    self.trace = []

  def frame(self, **kw) -> Frame:
    view = CYCLE[self.k % len(CYCLE)]
    self.k += 1
    self.t_cam += DT_MDL
    side = view.startswith('side')
    runs = ()
    if side and self.side not in ('running', 'noRoom'):
      pass  # the speed gate (or no side pkl): the odd steps idle
    else:
      raw = RawFrame(list(self.boxes(self).get(view, [])), 'road' if view == 'road' else 'wide', self.t_cam, view if side else 'full')
      sec = sectors(view) if side else None
      self.live.observe_run(view, self.t_cam, raw=raw, sectors=sec)
      runs = ((view, self.t_cam, raw, sec),)
    self.live.set_available(True)
    fields = {'control': self.control if self.control is not None and self.control.held else None, 'junction': self.j,
                  'maneuver': self.maneuver, 'objects': tuple(self.objects(self)), 'tracks': tuple(self.tracks(self)), 'runs': runs, 'live': self.live,
                  'vis': self.vis, 'surveys': self.entries, 'wall_t': self.wall_t, 'side_status': self.side, 'v_ego': self.v, 'a_ego': self.a,
                  'odo_m': self.odo, 'model_v2': self.model, 'mode': self.mode}
    fields.update(self.kw)
    fields.update(kw)
    return Frame(**fields)

  def step(self, **kw):
    d = self.sg.update(self.frame(**kw))
    self.trace.append(d)
    if d.release is not None and self.control is not None:
      self.control.held = False  # map_controls let its latch go
    self.odo += self.v * DT_MDL
    if self.control is not None and self.control.held:
      self.state.moved_m += self.v * DT_MDL
    return d

  def run(self, seconds, **kw):
    return [self.step(**kw) for _ in range(round(seconds / DT_MDL))]


def two_way(**kw):
  """The same junction with its cross approaches not stopping (a two-way stop): mapped no, Valhalla's none."""
  j = junction(tiers={'left': 'no', 'right': 'no'}, **kw)
  for a in j.approaches:
    if a.role in ('left', 'right'):
      a.control = 'none'
  return j


# ------------------------------------------------------------------------------------------------------- the phantom clear

def test_phantom_clear_a_two_way_stop_with_nothing_detected_never_goes_and_hands_back_h3(tmp_path):
  """STOP_SIGNS.md 8.1's first test: a two-way stop, zero detections on every view, PV1 loaded (the shipped provisional side-view
  table, allowProvisionalFor allWay); the decider never sets goCertified, and the car hands back with H3 after the wait ledger
  and the ladder (met off the route: its static wait, then H3)."""
  w = World(tmp_path, j=two_way(), entries=[])
  out = w.run(STATIC_WAIT_PLUS := 20.)
  assert STATIC_WAIT_PLUS > 15. + 1.
  assert all(d.go_cause == 'none' and d.release is None and not d.would_go for d in out)
  assert all(s.accel_cap >= -2.0 - 1e-9 for d in out for s in d.stops)
  faulted = [d for d in out if d.fault == 'H3']
  assert faulted and faulted[0].prompt == 'stopSignTakeControl' and 'take control' in faulted[0].reason
  first = out.index(faulted[0])
  assert (first + 1) * DT_MDL >= 15.  # the static wait came first, its reason said every frame of it
  assert all(d.prompt == 'waitingForCross' and d.reason and d.ledger == 'static' for d in out[round(1.2 / DT_MDL):first])


def first_go(out):
  return next((i for i, d in enumerate(out) if d.go_cause == 'stopSign'), None)


def test_C1_a_surveyed_validated_all_way_goes_by_itself_after_the_full_stop_and_clear_s(tmp_path):
  w = World(tmp_path)
  out = w.run(5.)
  i = first_go(out)
  assert i is not None, [(d.state, d.reason) for d in out[::10]]
  # the full stop's dwell (STOP_DWELL_S), then every go condition CLEAR_S running: the latch let go, G5's go, the chime
  assert (i + 1) * DT_MDL >= 1.0 + sr.CLEAR_S - 1e-6
  d = out[i]
  assert d.release == ((0, 1), 'clear') and d.prompt == 'stopGo' and d.state == 'going' and d.speed_cap == sr.STOP_GO_V_MAX
  assert all(x.r_seen >= x.r_req for x in out[i - 1].lanes) and {x.role for x in out[i - 1].lanes} >= {'left', 'right', 'opposite'}
  assert all(dd.prompt == 'waitingForCross' for dd in out[round(1.1 / DT_MDL):i])


def test_a_faulted_plan_withdraws_a_go_in_hand_with_the_capped_abort_and_keeps_its_own_hold_never_the_latchs(tmp_path):
  # turnpilotd.fault_limits (update() isn't run on a faulted frame): before the go the latch holds and the decider asks
  # nothing; a go in hand is withdrawn (a fault is no time to cross) with the capped abort short of d_edge_min, never under
  # -2.0; standing again with the latch gone, the decider's own hold keeps the car (STOP_SIGNS.md 2.6, owner rule)
  w = World(tmp_path)
  out = w.run(1.5)
  assert out[-1].state in ('stopped', 'observe') and w.sg.fault_stops(0., w.odo) == []
  out = w.run(4.)
  assert first_go(out) is not None and w.sg.state == 'going'
  w.v, w.a = 1.5, 1.0
  w.run(0.3)
  v, d_min = w.v, w.sg.edges.d_min
  stops = w.sg.fault_stops(v, w.odo)
  assert w.sg.state == 'observe' and w.sg.withdrawn == 'fault' and w.sg.retries == 1
  assert len(stops) == 1 and -2.0 <= stops[0].accel_cap < 0. and stops[0].speed_cap == 0.
  assert w.sg.pos_ref + (w.odo - w.sg.odo_ref) + stops[0].d_stop <= d_min - sr.M_EDGE + 1e-6
  w.v, w.a = 0., 0.
  held = w.sg.fault_stops(0., w.odo)
  assert len(held) == 1 and held[0].should_stop and held[0].speed_cap == 0. and held[0].reason


# ------------------------------------------------------------------------------------------------- the closed loop harness



MODEL_LAUNCH = 1.0  # m/s^2: a model that has taken over drives on at this


class Drive(World):
  """The decider closing the loop: its stops (and the latch's hold or creep) through scene_policy.ScenePolicy into the plan's
  caps, the plan's accel candidate (hooks.plan_accel) and the model's (model(world) -> 'holds', 'goes' or 'stops':
  holding a standstill, driving on once rolling, or braking to a stop STOP_PAST_M past the commit point) as the planner takes
  their min(), G5's lift (go_override.GoCertifier: +GO_ACCEL while the model alone holds the car, until its window, its
  takeover or GO_END_SPEED), and the car following the command lag_s late (a pure delay), as the actuators do."""
  STOP_PAST_M = 1.5

  def __init__(self, tmp_path, lag_s=0.3, behaviour=lambda w: 'holds', **kw):
    super().__init__(tmp_path, **kw)
    self.lag_s = lag_s
    self.queue = deque([0.] * max(round(lag_s / DT_MDL), 1))
    self.scene = ScenePolicy()
    self.gc = GoCertifier()
    self.behaviour = behaviour
    self.kw['t_abort'] = sr.t_abort(lag_s)  # the delay known (S4a's actuatorDelayS)
    self.committed_at: float | None = None
    self.cmds = []

  def model_says(self, d):
    mode = self.behaviour(self)
    if mode == 'stops' and self.committed_at is not None and self.odo - self.committed_at >= self.STOP_PAST_M:
      return -1.5 if self.v > 0.05 else 0., self.v < 0.3, model_v2(holds=True)
    if mode in ('goes', 'stops') and self.v >= 0.3:
      return MODEL_LAUNCH, False, model_v2()
    return (-0.4 if self.v > 0.05 else 0.), self.v < 0.3, model_v2(holds=True)

  def step(self, **kw):
    a_model, model_stop, msg = self.model_says(None)
    self.model = msg
    self.gc.update(msg, self.v, DT_MDL)
    kw.setdefault('go_active', self.gc.cause)
    kw.setdefault('go_taken_over', self.gc.taken_over)
    kw.setdefault('go_end', self.gc.end)
    d = World.step(self, **kw)
    if d.state == 'committed' and self.committed_at is None:
      self.committed_at = self.odo
    if d.withdraw and self.gc.active:
      self.gc.withdraw(d.withdraw)
    if d.go_cause != 'none' and not self.gc.active:
      self.gc.start(d.go_cause)
    latch = []
    if self.control is not None and self.control.held:
      if d.creep is not None:
        rem = d.creep[0] - self.state.moved_m
        latch = [Stop(10., min(d.creep[1], math.sqrt(3. * max(rem - 0.1, 0.))), False, 'creep', rem)] if rem > 0.1 else \
          [Stop(HOLD_ACCEL, 0., True, 'held', 0.)]
      else:
        latch = [Stop(HOLD_ACCEL, 0., True, 'held', 0.)]
    sc = self.scene.update([], True, None, float('inf'), self.v, extra_stops=latch + d.stops)
    speed_cap = min(sc.speed_cap, d.speed_cap)
    a_tp = plan_accel(sc.accel_cap, speed_cap, self.v)
    # GoLift: the e2e candidate lifted while the go is out, unless TurnPilot's candidate holds the car (its stop, or under 0.3
    # m/s an accel under 0.1: go_override.holds), the model brakes past GO_MODEL_BRAKE, or the car is at GO_END_SPEED
    lifted = self.gc.active and not sc.should_stop and not (self.v < 0.3 and a_tp < 0.1) and self.v < GO_END_SPEED and a_model >= -1.
    a_e2e = max(a_model, GO_ACCEL) if lifted else a_model
    stop_now = sc.should_stop or (model_stop and not lifted)
    a_cmd = min(a_e2e, a_tp)
    if stop_now:
      a_cmd = min(a_cmd, -1.) if self.v > 0. else 0.
    self.cmds.append((self.t_cam, self.v, a_cmd, a_tp, sc.accel_cap, speed_cap, d.state))
    self.queue.append(a_cmd)
    self.a = self.queue.popleft()
    self.v = max(self.v + self.a * DT_MDL, 0.)
    return d


def test_closed_loop_harness_a_model_that_goes_crosses_and_one_that_holds_is_withdrawn_and_handed_back(tmp_path):
  # the harness itself: a model that drives once rolling (G5's lift rolls it) crosses to inLane; one that holds is withdrawn
  w = Drive(tmp_path, behaviour=lambda w: 'goes')
  out = w.run(14.)
  assert any(d.state in ('inLane', 'done') for d in out)
  w = Drive(tmp_path, behaviour=lambda w: 'holds')
  out = w.run(40.)
  assert [d.withdraw for d in out if d.withdraw][:1] and out[-1].fault == 'H5'


# ------------------------------------------------------------------------------------------------------ the property tests


LANE_EDGE = 3.5  # test_certify's survey: the near travel-lane edge 3.5 m from the node


def lanes_of(j, maneuver='unknown'):
  return {ln.role: ln for ln in junction_lanes(j, maneuver, 1.9, LANE_EDGE) if ln.index == 1}


def lane_box(w, role, down_m, conf=0.5, cls='car', views=('full',)):
  """{view: [RawBox]} for a box down_m m along lane role's line, seen from where the car is now."""
  ln = lanes_of(w.j)[role]
  pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref) if w.sg.token is not None else -w.control.d
  x0, y0 = ln.point(down_m, 'min')
  x1, y1 = ln.point(down_m, 'max')
  x, y = (x0 + x1) / 2. - pos, -(y0 + y1) / 2.  # tracker frame: y left
  bearing = math.degrees(math.atan2(y, x))
  side = 'sideLeft' if bearing > 0 else 'sideRight'
  b = dataclasses.replace(raw_box(bearing, conf=conf, xy=(x, y), view=side), cls=cls)
  return {v if v != 'side' else side: [b] for v in views + ('side',)}


class Scene:
  """A random scene (8.1's property tests): a junction (an all-way at 15-30 mph with a stamp of 15-60 m, or a two-way), a
  maneuver, side views running, gated or recalibrating, gaps in a view, degraded sectors, cross cars arriving (some stopping
  at their lines, some not), phantom boxes (one frame or steady), people at the crosswalk, a model that holds, goes or stops
  in the band, and an actuator lag of 0.2-0.6 s."""
  def __init__(self, rng, tmp_path, vis):
    self.rng = rng
    mph = rng.choice([15, 20, 25, 30])
    self.maneuver = rng.choice(['straight', 'left', 'right'])
    allway = rng.random() < 0.8
    j = junction(mph=mph, maneuver=self.maneuver) if allway else two_way(mph=mph, maneuver=self.maneuver)
    st = stamp(m=rng.uniform(15., 60.))
    side = rng.choice(['running', 'running', 'running', 'speedGated', 'mapsStale'])
    behaviour = rng.choice(['holds', 'goes', 'goes', 'stops'])
    self.w = Drive(tmp_path, lag_s=rng.uniform(0.2, 0.6), behaviour=lambda w: behaviour, j=j, entries=[survey(mph=mph, validations=[st])],
                   vis=vis, maneuver=self.maneuver, side=side)
    self.cars = []
    for k in range(rng.randint(0, 2)):
      self.cars.append({'id': 100 + k, 'role': rng.choice(['left', 'right']), 'd0': rng.uniform(8., 60.), 'v': rng.uniform(0., 12.),
                        'a': rng.choice([0., 1.5, 3.0, 5.0]), 't0': rng.uniform(0., 10.), 'line': rng.random() < 0.5})
    self.gap = (rng.choice(CYCLE), rng.uniform(0., 12.), rng.uniform(0.2, 3.0)) if rng.random() < 0.4 else None
    self.phantom = (rng.choice(['left', 'right']), rng.uniform(0., 12.), rng.choice([0.05, 2.0]), rng.uniform(5., 30.)) \
      if rng.random() < 0.5 else None
    self.ped = (rng.uniform(0., 10.), rng.choice([0., 1.4]), rng.uniform(1., 6.)) if rng.random() < 0.3 else None
    self.degraded = rng.uniform(0., 12.) if rng.random() < 0.3 else None
    w = self.w
    w.tracks, w.boxes, w.objects = self.tracks, self.boxes, self.objects
    self.t = 0.

  def car_d(self, c, t):
    tt = t - c['t0']
    if tt < 0.:
      return None
    stop_d = 9. if c['line'] else -1e9  # its stop line, about 9 m short of our path
    if c['a'] > 0.:
      ts = c['v'] / c['a']
      gone = c['v'] * min(tt, ts) - 0.5 * c['a'] * min(tt, ts) ** 2
    else:
      gone = c['v'] * tt
    return max(c['d0'] - gone, stop_d) if c['line'] else c['d0'] - gone

  def tracks(self, w):
    out = []
    t = w.t_cam - 100.
    pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref) if w.sg.token is not None else -w.control.d
    for c in self.cars:
      dd = self.car_d(c, t)
      if dd is None or dd < -15.:
        continue
      ln = lanes_of(w.j)[c['role']]
      s = (ln.s_min + ln.s_max) / 2. - pos
      trk = ev_track(c['id'], w.t_cam, max(dd, 0.5), ln, view='side' + c['role'].capitalize(), s=s)
      trk.span_seen = (trk.bearing_seen - 2., trk.bearing_seen + 2.)
      trk.x = np.array([s, trk.x[1], 0., -c['v'] if c['role'] == 'left' else c['v']])
      trk.hits = 5
      out.append(trk)
    return out

  def boxes(self, w):
    t = w.t_cam - 100.
    out = {}
    if self.phantom is not None:
      role, t0, dur, down = self.phantom
      if t0 <= t <= t0 + dur:
        out = lane_box(w, role, down, conf=0.36)
    if self.degraded is not None and abs(t - self.degraded) < 0.3:
      pass  # (sectors are the World's: degraded below by its own hook)
    return out

  def objects(self, w):
    if self.ped is None:
      return ()
    t0, v, dur = self.ped
    t = w.t_cam - 100.
    if not t0 <= t <= t0 + dur:
      return ()
    return (SimpleNamespace(trackId=900, cls='pedestrian', x=4.5 - (w.odo - 0.), y=-3. + v * (t - t0), vx=0., vy=v),)

  def frame_kw(self):
    w = self.w
    t = w.t_cam - 100.
    if self.gap is not None:
      view, t0, dur = self.gap
      if t0 <= t <= t0 + dur and CYCLE[w.k % len(CYCLE)] == view:
        w.k += 1  # this view's slot idles (a dropped run)
    return {}

  def run(self, seconds):
    out = []
    for _ in range(round(seconds / DT_MDL)):
      self.frame_kw()
      out.append(self.w.step())
    return out


def scenes(tmp_path, n, seed, seconds=16.):
  vis = visibility(tmp_path)
  for i in range(n):
    sc = Scene(random.Random(seed + i), tmp_path, vis)
    yield sc, sc.run(seconds)


def test_property_no_stop_the_decider_makes_asks_for_less_than_2_m_s2(tmp_path):
  """8.1: across random junctions, tracks, phantom boxes, detector gaps, model behaviours, maneuvers and lags, no stop of the
  decider's asks for less than -2.0 m/s^2, alone or composed with the latch through scene_policy, and nothing it does is a
  hard brake."""
  n = 0
  for sc, out in scenes(tmp_path, 14, 1000):
    for d in out:
      for s in d.stops:
        n += 1
        assert s.accel_cap >= -2. - 1e-9, (s, d.state)
    assert all(c[3] >= -2. - 1e-9 for c in sc.w.cmds), min(c[3] for c in sc.w.cmds)
    assert not sc.w.scene.hard
  assert n > 50


def test_property_no_go_while_any_lane_r_seen_at_the_go_position_is_under_its_r_req(tmp_path):
  """8.1: goCertified (a stopSign go) is never set while any conflicting lane's R_seen at the go position is below its R_req
  (a lane a qualified shield satisfies aside), checked independently of the decider's own numbers: the lanes walked again at
  the go frame's position with the live inputs of that frame."""
  gos = 0
  for sc, out in scenes(tmp_path, 16, 2000):
    w = sc.w
    for d in out:
      if d.go_cause != 'stopSign':
        continue
      gos += 1
      for ln in d.lanes:
        assert ln.r_seen + 1e-9 >= ln.r_req or ln.shielded, (ln, d.position_m)
      lanes = junction_lanes(w.j, sc.maneuver, 1.9, LANE_EDGE)
      assert {(x.role, x.index) for x in lanes} == {(x.role, x.index) for x in d.lanes}
  assert gos >= 3  # the property isn't vacuous: some scenes go


def test_property_closed_loop_every_withdrawn_go_ends_short_of_d_edge_min_and_every_band_abort_short_of_the_midpoint(tmp_path):
  """8.1 (O3, O17): closed loop with an actuator-lag model of 0.2-0.6 s (the delay known, as S4a's actuatorDelayS makes it): a
  go withdrawn at any moment of its launch (a box appearing in the near lane, the model never taking over) ends standing short
  of dEdgeMin, the capped abort never below -2.0; a vehicle appearing in the band makes the in-band abort only where it ends
  short of the band midpoint, and it does."""
  rng = random.Random(7)
  vis = visibility(tmp_path)
  withdrawn = band = 0
  for lag in (0.2, 0.3, 0.4, 0.5, 0.6):
    for behaviour in ('holds', 'goes', 'goes'):
      w = Drive(tmp_path, lag_s=lag, behaviour=lambda w, b=behaviour: b, vis=vis)
      go_t = None
      after = rng.uniform(0., 2.5)
      appear = None
      for _ in range(round(16. / DT_MDL)):
        d = w.step()
        t = w.t_cam
        if d.go_cause == 'stopSign' and go_t is None:
          go_t = t
        if go_t is not None and appear is None and t >= go_t + after:
          appear = t
          role = rng.choice(['left', 'right'])
          w.boxes = lambda ww, role=role: lane_box(ww, role, 18., views=('full',))
        if d.withdraw in ('bandAbort',):
          band += 1
      edges = w.sg.edges
      pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref)
      states = [x.state for x in w.trace]
      if any(x.withdraw and x.withdraw not in ('bandAbort', 'uncommitted', 'driver') for x in w.trace):
        withdrawn += 1
        if 'committed' not in states:
          assert pos <= edges.d_min + 1e-6, (lag, behaviour, pos, edges)
      if any(x.withdraw == 'bandAbort' for x in w.trace):
        assert pos <= sr.band_midpoint(edges) + 1e-6, (lag, behaviour, pos, edges)
      assert all(c[3] >= -2. - 1e-9 for c in w.cmds)
  assert withdrawn >= 5


def test_property_closed_loop_a_go_withdrawn_on_its_last_frames_before_commit_still_ends_short_of_d_edge_min(tmp_path):
  """The boundary of the same property (O17): the go withdrawn on its last frames before the decider's own commit point (found
  by a first run, then replayed with a box in the near lane 1-5 frames before it), the car launching as hard as the model
  does (MODEL_LAUNCH under the STOP_GO_V_MAX cap), for true lags 0.2-0.6 s known to T_ABORT: the capped abort ends standing
  short of dEdgeMin every time, never below -2.0."""
  vis = visibility(tmp_path)
  ends = []
  for lag in (0.2, 0.3, 0.4, 0.5, 0.6):
    def run(box_at=None, lag=lag):
      w = Drive(tmp_path, lag_s=lag, behaviour=lambda w: 'goes', vis=vis)
      for i in range(round(12. / DT_MDL)):
        if box_at is not None and i == box_at:
          w.boxes = lambda ww: lane_box(ww, 'left', 15., views=('full', 'road'))
        w.step()
      return w
    first = run()
    commit = next(i for i, x in enumerate(first.trace) if x.state == 'committed')
    for k in (1, 2, 3, 5):
      w = run(commit - k)
      pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref)
      if 'committed' in [x.state for x in w.trace]:
        continue  # the box's first run came after the commit point: the band's (the next test's)
      ends.append((lag, k, round(w.sg.edges.d_min - pos, 2)))
      assert pos <= w.sg.edges.d_min + 1e-6, (lag, k, pos, w.sg.edges)
      assert all(c[3] >= -2. - 1e-9 for c in w.cmds)
  assert len(ends) >= 12 and min(e[2] for e in ends) < 1.0, ends  # withdrawn at the boundary, some close to the edge


def test_property_every_held_state_has_a_reason_and_a_bound_or_a_dynamic_cause(tmp_path):
  """8.1 / 4.6: across random scenes every frame the decider holds the car (stopped, rolling up, observing, a fault's handback)
  says why, and its wait is dynamic (no bound: something moves), or static and ends in the ladder within STATIC_WAIT_S, or a
  fault's grace that ends in its handback (FAULT_GRACE_S, FRONT_ZONE_MAX_S), or the handback itself."""
  for _sc, out in scenes(tmp_path, 14, 3000, seconds=24.):
    static = fault = 0.
    for d in out:
      if d.state in ('stopped', 'rollUp', 'observe', 'ladder'):
        assert d.reason, d
      if d.state == 'observe':
        assert d.ledger in ('none', 'dynamic', 'static', 'fault') and d.prompt in ('waitingForCross', 'stopGo', 'stopSignTakeControl')
        static = static + DT_MDL if d.ledger == 'static' else 0.
        fault = fault + DT_MDL if d.ledger == 'fault' else 0.
        assert static <= STATIC_WAIT_S + DT_MDL + 1e-6
        assert fault <= max(FAULT_GRACE_S, FRONT_ZONE_MAX_S) + DT_MDL + 1e-6
      if d.state == 'ladder':
        assert d.prompt == 'stopSignTakeControl' and d.fault


# ----------------------------------------------------------------------------------------------- scripted cross traffic

LINE_D = 9.3  # m down its lane a cross car's box centre sits when its front is at its line (test_certify's lineM 7 + 2.3)


def at_line(t0=-10., t_stop=None, d0=30., v0=6.):
  """d(t) of a car that drives in at v0 from d0 and stops at its line (braking to it), there from t_stop on (or as it gets
  there); before t0 it isn't there."""
  def d(t):
    if t < t0:
      return None
    if t_stop is not None and t >= t_stop:
      return LINE_D
    return max(d0 - v0 * (t - t0), LINE_D)
  return d


def rolling(t0, d0=40., v0=10., a=0.):
  """d(t) of a car that doesn't stop: from d0 at v0 (slowing at a, never below 2 m/s), through our path and on."""
  def d(t):
    if t < t0:
      return None
    tt = t - t0
    if a > 0.:
      tb = max(v0 - 2., 0.) / a
      gone = v0 * min(tt, tb) - 0.5 * a * min(tt, tb) ** 2 + 2. * max(tt - tb, 0.)
    else:
      gone = v0 * tt
    return d0 - gone
  return d


def cars(*specs):
  """World.tracks for scripted cars: (trackId, role, d(t) [, cls]), each seen from where our car is now on its side view."""
  def tracks(w):
    t = w.t_cam - 100.
    pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref) if w.sg.token is not None else -w.control.d
    out = []
    for spec in specs:
      tid, role, d_fn = spec[:3]
      cls = spec[3] if len(spec) > 3 else 2
      dd = d_fn(t)
      if dd is None or dd < -12.:
        continue
      ln = lanes_of(w.j)[role]
      s = (ln.s_min + ln.s_max) / 2. - pos
      if role == 'opposite':
        # a lane parallel to ours: the car's box bottom is its front, d down the lane from where it crosses our path
        x, y = s + max(dd, 0.3), -ln.y
        trk = SimpleNamespace(track_id=tid, t_seen=w.t_cam, bearing_seen=math.degrees(math.atan2(y, x)), width_seen=1.8 * 1141. / x,
                              view='road', cls=cls, x=np.array([x, y, 0., 0.]))
        half = math.degrees(math.atan2(0.9, x))
        trk.span_seen = (trk.bearing_seen - half, trk.bearing_seen + half)
        trk.hits = 5
        out.append(trk)
        continue
      view = 'sideLeft' if role == 'left' else 'sideRight'
      trk = ev_track(tid, w.t_cam, max(dd, 0.3), ln, view=view, cls=cls, s=s)
      half = math.degrees(math.atan2(0.9, math.hypot(s, dd)))
      trk.span_seen = tuple(sorted((trk.bearing_seen - half, trk.bearing_seen + half)))
      trk.hits = 5
      out.append(trk)
    return out
  return tracks


def watch(w, seconds=2.):
  """The car standing short of its line, the stop engaged but not yet latched, for seconds: the decider watching the
  junction from its approach (as it does from STOP_SWEEP_M out), so a car already at its line is a trusted, stopped track by
  the time of our full stop."""
  w.control.held = False
  out = w.run(seconds, approach=w.control)
  w.control.held = True
  return out


def person(tid, x, y, vx=0., vy=0.):
  return SimpleNamespace(trackId=tid, cls='pedestrian', x=x, y=y, vx=vx, vy=vy)


def vehicle(tid, x, y, vx=0., vy=0., cls='car'):
  return SimpleNamespace(trackId=tid, cls=cls, x=x, y=y, vx=vx, vy=vy)


def go_index(w, seconds):
  out = w.run(seconds)
  return first_go(out), out


# ------------------------------------------------------------------------------------------------- cross-cutting (8.1)

def test_detector_gap_a_half_second_hole_restarts_clear_s_and_two_seconds_pauses_then_h1(tmp_path):
  # a 0.5 s hole in the right side view during CLEAR_S restarts the timer (DET_FRESH_S on camera time)
  w = World(tmp_path)
  w.run(2.0)  # the dwell, and a second of clear
  assert w.trace[-1].clear_s > 0.5
  idle = w.side
  base = w.k
  out = []
  for _ in range(round(0.6 / DT_MDL)):
    if CYCLE[w.k % 4] == 'sideRight':
      w.k += 1  # its run dropped
    out.append(w.step())
  assert any(d.clear_s == 0. and any(h.key == 'stale' for h in d.holds) for d in out)
  i, rest = go_index(w, 3.)
  assert i is not None and (i + 1) * DT_MDL >= sr.CLEAR_S - 1e-6
  # a 2.0 s hole while stopped: the camera paused (cameraPaused, held), and H1 FAULT_GRACE_S later
  w = World(tmp_path)
  w.run(1.5)
  out = []
  for _ in range(round(13. / DT_MDL)):
    if CYCLE[w.k % 4] == 'sideRight':
      w.k += 1
    out.append(w.step())
  paused = [k for k, d in enumerate(out) if d.alert == 'cameraPaused']
  assert paused and abs(paused[0] * DT_MDL - 2.0) < 0.3
  h1 = next(k for k, d in enumerate(out) if d.fault == 'H1')
  assert abs((h1 - paused[0]) * DT_MDL - 10.) < 0.2
  assert out[h1].prompt == 'stopSignTakeControl' and out[h1].reason == "Cross-traffic camera unavailable: take control"
  assert all(d.go_cause == 'none' for d in out) and idle == 'running' and base >= 0


def test_untrusted_tracks_a_cross_car_seen_0_8_s_is_never_stopping(tmp_path):
  # a car first seen 0.8 s ago on a stopping approach, whatever it reads (even standing far out), is tested at v_c and is
  # never stopping: the go waits for its trust
  for d_fn in (lambda t: 40., rolling(0., d0=40., v0=1.)):
    w = World(tmp_path)
    w.run(1.2)
    t_seen = w.t_cam - 100.
    w.tracks = cars((7, 'left', lambda t, f=d_fn, t0=t_seen: f(t - t0) if t >= t0 else None))
    out = w.run(0.8)
    e = w.sg.ev[7]
    assert not e.v_trusted and e.u2 == 'notStopping'
    assert any(h.key == 'u2:7' for h in out[-1].holds) and out[-1].clear_s == 0.


def test_raw_boxes_block_never_brake_become_occluders_when_steady_and_block_near_the_horizon(tmp_path):
  # a 0.36 box in the near lane band, never a track, holds the go and never brakes
  w = World(tmp_path)
  w.run(1.2)  # stopped, looking
  w.boxes = lambda ww: lane_box(ww, 'left', 20., conf=0.36, views=('full',))
  out = w.run(6.)
  assert first_go(out) is None and any(h.key.startswith('raw:left') for h in out[round(0.3 / DT_MDL)].holds)
  assert all(s.accel_cap >= -2. and s.speed_cap >= 0. for d in out for s in d.stops)
  # the same box held still 1.0 s is an occluder: no more a blocker (it caps the lane's reach instead)
  steady = [d for d in out[round(1.5 / DT_MDL):] if not any(h.key.startswith('raw:') for h in d.holds)]
  assert steady and any(h.key.startswith('reach:left') for h in steady[-1].holds)
  # a box near the horizon on a conflicting side, outside every nominal band, still blocks
  w = World(tmp_path)
  far = dataclasses.replace(raw_box(70., conf=0.4, xy=(25., 68.7), view='sideLeft'), cls='car')
  flick = {'v': 0}

  def boxes(ww):
    flick['v'] += 1
    return {'sideLeft': [dataclasses.replace(far, bearing_deg=70. + 0.6 * (flick['v'] % 3), span_deg=(69. + 0.6 * (flick['v'] % 3), 71.))]}
  w.boxes = boxes
  out = w.run(5.)
  assert first_go(out) is None and any(h.key.startswith('raw:left') for h in out[-1].holds)


def test_takeover_a_model_that_holds_is_withdrawn_short_of_the_gate_retried_twice_then_h5(tmp_path):
  w = Drive(tmp_path, behaviour=lambda w: 'holds')
  out = w.run(40.)
  gos = [k for k, d in enumerate(out) if d.go_cause == 'stopSign']
  withdrawn = [d for d in out if d.withdraw and d.withdraw not in ('fault',)]
  # the go and its retries while GO_ROOM_M remains (G5's lift carries a holding car 2.25 m each time), then H5
  assert 2 <= len(gos) <= 3 and len(withdrawn) == len(gos)
  assert all(d.withdraw in ('window', 'gate') for d in withdrawn)
  h5 = next(d for d in out if d.fault == 'H5')
  assert h5.prompt == 'stopSignTakeControl' and h5.reason == "Can't start: take control"
  pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref)
  assert pos < w.sg.edges.d_min and out[-1].state == 'ladder' and any(s.should_stop for s in out[-1].stops)
  # each retry only after CLEAR_S of the go conditions again, standing
  for a, b in zip(gos, gos[1:], strict=False):
    assert (b - a) * DT_MDL >= sr.CLEAR_S


def test_takeover_a_model_that_drives_ends_the_lift_when_the_predicate_held_0_3_s(tmp_path):
  w = Drive(tmp_path, behaviour=lambda w: 'goes')
  out = w.run(10.)
  i = first_go(out)
  assert i is not None and w.gc.end in ('takeover', 'speed') and w.gc.taken_over
  taken = next(d for d in out[i:] if d.takeover_s >= 0.)
  assert 0.3 - 1e-6 <= taken.takeover_s <= 3.
  assert any(d.state in ('committed', 'inLane', 'done') for d in out[i:]) and not any(d.withdraw for d in out)


def test_band_a_model_that_stops_in_the_band_uncommits_never_clear_box_and_goes_again_only_after_clear_s(tmp_path):
  w = Drive(tmp_path, behaviour=lambda w: 'stops')
  out = w.run(20.)
  states = [d.state for d in out]
  assert 'committed' in states
  unc = next(k for k, d in enumerate(out) if d.state == 'observe' and k > states.index('committed'))
  assert all(d.go_cause != 'clearBox' for d in out)
  pos_unc = out[unc].position_m
  assert w.sg.edges.d_min - 3. < pos_unc < w.sg.edges.d_max + 1e-6
  again = [k for k, d in enumerate(out) if d.go_cause == 'stopSign' and k > unc]
  if again:
    assert (again[0] - unc) * DT_MDL >= sr.CLEAR_S - 1e-6


def test_launch_geometry_reach_shrinking_with_our_own_motion_never_withdraws_a_box_appearing_does(tmp_path):
  # D13: the near lane's frame-edge reach falls as the car moves on, nothing new in it: the go isn't withdrawn (reach is
  # walked on the go position's geometry); a raw box appearing in a lane during the launch withdraws it
  w = Drive(tmp_path, behaviour=lambda w: 'goes')
  out = w.run(10.)
  assert first_go(out) is not None and not any(d.withdraw for d in out) and any(d.state == 'committed' for d in out)
  w = Drive(tmp_path, behaviour=lambda w: 'goes')
  out = w.run(3.2)
  i = first_go(out)
  assert i is not None
  w.boxes = lambda ww: lane_box(ww, 'right', 20., views=('full', 'road'))
  out = w.run(2.)
  assert any(d.withdraw.startswith('raw:right') for d in out)


def test_signal_head_on_3_of_5_forward_runs_makes_the_junction_uncertifiable_and_writes_the_override(tmp_path):
  w = World(tmp_path)
  seen = []
  w.sg.on_signal_seen = seen.append
  lights = [(5., 12., 0.6)]  # bearing, elevation above the horizon, conf: a head over the junction
  n = {'k': 0}

  def frame_lights(d):
    for key, _t, raw, _s in d.runs:
      if key in ('road', 'full'):
        n['k'] += 1
        raw.lights = lights if n['k'] % 5 in (1, 2, 3) else []
  out = []
  for _ in range(round(6. / DT_MDL)):
    f = w.frame()
    frame_lights(f)
    out.append(w.sg.update(f))
    w.trace.append(out[-1])
  assert first_go(out) is None and seen and w.sg.signal_seen
  assert any(h.key == 'signal' and h.kind == 'static' for h in out[-1].holds)


class Queue(Drive):
  """Drive with map_controls' approach and latch in the loop: the control engaged d0 m ahead (not held), its distance
  dead-reckoned, the map curve as a cap (map_controls.curve), the latch holding the car slower than LATCH_SPEED within
  LATCH_NEAR_M of its stop point (LATCH_PAST_M past it)."""
  def __init__(self, tmp_path, d0=8., lead=None, **kw):
    super().__init__(tmp_path, d0=d0, **kw)
    self.control.held = False
    self.lead = lead  # (world) -> m to the model's lead, or None

  def frame(self, **kw):
    from openpilot.selfdrive.turnpilot.map_controls import LATCH_NEAR_M, LATCH_PAST_M, LATCH_SPEED
    c = self.control
    if not c.held and self.v < LATCH_SPEED and -LATCH_PAST_M < c.d < LATCH_NEAR_M and not getattr(self, 'released', False):
      c.held, self.state.moved_m = True, 0.
    lead = self.lead(self) if self.lead is not None else None
    kw.setdefault('lead_m', lead)
    kw.setdefault('approach', None if c.held else c)
    return super().frame(**kw)

  def step(self, **kw):
    was = self.control.held
    d = super().step(**kw)
    if was and not self.control.held:
      self.released = True
    if not self.control.held and not getattr(self, 'released', False):
      self.control.d -= self.v * DT_MDL
    return d


# --------------------------------------------------------------------------------------------- the catalogue's sg rows

def test_C1_all_way_order_ties_shields_u2_the_go_and_the_band(tmp_path):
  # section 4.5 in one: an earlier arrival at the left line goes first, a later one at the right line waits (a shield), U2
  # holds a roller until it is through, then the go, the launch and the band to the far side
  w = Drive(tmp_path, behaviour=lambda w: 'goes')
  w.tracks = cars((1, 'left', at_line(t0=-10.)), (2, 'right', at_line(t0=4.)))
  watch(w)
  first = w.run(4.)
  assert first_go(first) is None and first[-1].rank == 1
  w.tracks = cars((1, 'left', lambda t: LINE_D - 6. * (t - 6.) if t >= 6. else LINE_D), (2, 'right', at_line(t0=4.)))
  out = w.run(16.)
  i = first_go(out)
  # the later arrival at the right line shields its lane and waits for us: we cross to the far side, no band abort
  assert i is not None and any(d.state in ('inLane', 'done') for d in out[i:])
  assert not [d.withdraw for d in out[i:] if d.withdraw] and not [d.fault for d in out if d.fault]


def test_C2_a_two_way_the_route_could_not_avoid_hands_back_h3_right_after_the_full_stop(tmp_path):
  # route-known (junctionInfo.certification uncertifiable): H3 right after the 1 s full stop, rungs 1 and 2 skipped
  w = World(tmp_path, j=two_way(), entries=[])
  out = w.run(2.)
  h3 = next(k for k, d in enumerate(w.trace) if d.fault == 'H3') if any(d.fault for d in out) else None
  assert h3 is None  # met off the route: its static wait first (the phantom clear's case)
  w = World(tmp_path, j=two_way(), entries=[])
  out = w.run(2., route_cert='uncertifiable')
  h3 = next(k for k, d in enumerate(out) if d.fault == 'H3')
  assert (h3 + 1) * DT_MDL <= 1.0 + 2 * DT_MDL + 1e-6 and out[h3].reason == "Stop sign: this stop needs you: take control"
  assert all(d.go_cause == 'none' for d in out)


@pytest.mark.parametrize('stops_not', ['left', 'right', 'opposite'])
@pytest.mark.parametrize('maneuver', ['straight', 'left', 'right'])
def test_C3_a_three_way_stop_at_a_four_leg_junction_takes_no_maneuver_its_set_touches_the_unstopped_approach(tmp_path, stops_not,
                                                                                                           maneuver):
  # 2.1's straight union: every maneuver's set touches every other approach at a 4-leg junction, so none certifies
  j = junction(maneuver=maneuver, tiers={stops_not: 'no'})
  next(a for a in j.approaches if sr.effective_role(a) == stops_not).control = 'none'
  w = World(tmp_path, j=j, maneuver=maneuver)
  out = w.run(17.)
  assert first_go(out) is None and out[-1].fault == 'H3'


def t_junction(maneuver='left', exit_control='stop'):
  """The car on a T's stem: the cross road's left and right approaches, no opposite one; the road it leaves by is the exit leg."""
  j = junction(maneuver=maneuver)
  j.approaches = [a for a in j.approaches if sr.effective_role(a) != 'opposite']
  for a in j.approaches:
    if a.role == 'exit':
      a.control = exit_control
  return j


def test_C4_a_t_junction_from_its_stem_goes_with_only_its_approaches_and_an_unknown_leg_never(tmp_path):
  w = World(tmp_path, j=t_junction('left'), maneuver='left')
  out = w.run(5.)
  assert first_go(out) is not None and {x.role for x in out[first_go(out) - 1].lanes} == {'left', 'right'}
  w = World(tmp_path, j=t_junction('left', exit_control='unknown'), maneuver='left')
  out = w.run(17.)
  assert first_go(out) is None and any(h.key == 'cert:unknownLeg' for h in out[30].holds) and out[-1].fault == 'H3'


def test_G23_the_exit_leg_unmatched_reads_unknown_and_the_junction_is_unknown_leg(tmp_path):
  j = junction(maneuver='right')
  next(a for a in j.approaches if a.role == 'exit').control = 'unknown'
  out = World(tmp_path, j=j, maneuver='right').run(5.)
  assert first_go(out) is None and any(h.key == 'cert:unknownLeg' for h in out[-1].holds)


def test_C20_a_signal_head_at_a_surveyed_junction_is_uncertifiable_for_the_encounter_then_h3_signal_at_junction(tmp_path):
  w = World(tmp_path)
  calls = []
  w.sg.on_signal_seen = calls.append
  out = []
  for _ in range(round(18. / DT_MDL)):
    f = w.frame()
    for key, _t, raw, _s in f.runs:
      if key in ('road', 'full'):
        raw.lights = [(-4., 10., 0.7)]
    out.append(w.sg.update(f))
  assert first_go(out) is None and len(calls) == 1
  h3 = next(d for d in out if d.fault == 'H3')
  assert h3.reason == "Stop sign: signal at junction: take control"


def test_G1_an_open_perpendicular_corner_decides_from_the_stop_point(tmp_path):
  out = World(tmp_path).run(5.)
  i = first_go(out)
  assert i is not None and abs(out[i].position_m - -0.5) < 1e-6
  for x in out[i - 1].lanes:
    assert x.binding == 'validated' and x.r_seen >= x.r_req


def test_G24_a_right_takes_the_straight_set_until_15_deg_so_its_right_approach_must_stop(tmp_path):
  # a right whose right approach doesn't stop (a 3-way stop): the straight union crosses that approach's far lanes
  j = junction(maneuver='right', tiers={'right': 'no'})
  next(a for a in j.approaches if a.role == 'exit').control = 'none'
  out = World(tmp_path, j=j, maneuver='right').run(5.)
  assert first_go(out) is None and any(h.key.startswith('cert:') for h in out[-1].holds)
  # the swing zone: the right approach's lane is walked from SWING_ZONE_M out for a right past 15 deg
  sw = [ln for ln in junction_lanes(junction(maneuver='right'), 'right', 1.9, LANE_EDGE, yaw_deg=20.) if ln.role == 'right']
  assert sw and sw[0].movement == 'swing' and sw[0].conflict_m == sr.SWING_ZONE_M


def test_V1_a_queue_rolls_up_on_rollup_goes_stops_at_the_line_and_counts_its_arrival_from_there(tmp_path):
  # stopped 8 m short behind a lead, the lead leaves, the model holds: rollUp goes (G5) carry it up, the latch holds it at
  # its line, the arrival is that stop; never a stop placed ahead of the standing car
  w = Queue(tmp_path, d0=8., behaviour=lambda w: 'holds', lead=lambda w: 6. if w.t_cam - 100. < 1.5 else None)
  out = w.run(16.)
  rollups = [d for d in out if d.go_cause == 'rollUp']
  assert 1 <= len(rollups) <= 4 and w.control.held or any(d.go_cause == 'stopSign' for d in out)
  arrived = next(k for k, d in enumerate(out) if d.state == 'observe')
  assert abs(out[arrived].position_m) <= 3.0 + 1e-6
  assert all(not s.should_stop for d in out[:arrived] for s in d.stops)


def test_O2_stopped_more_than_3_m_short_rolls_up_on_the_map_curve_and_stops_again(tmp_path):
  w = Drive(tmp_path, d0=4.5, behaviour=lambda w: 'holds')
  out = w.run(12.)
  assert out[0].state == 'rollUp' and out[0].creep == (4.5, 3.0)
  assert any(d.go_cause == 'rollUp' for d in out)
  arrived = next(k for k, d in enumerate(out) if d.state == 'observe')
  assert abs(out[arrived].position_m) <= 3.0 + 1e-6 and w.state.moved_m >= 1.5


def test_O2_V1_a_roll_up_that_never_moves_the_car_is_bounded_and_says_so_h5(tmp_path):
  # 4.6: every held state has a bound or a voice. With TurnPilotGoOverride off, or every go blocked (a queue's lead still
  # inside go_blocked's 15 m), the ROLLUP_GOES rollUp goes never move a holding model's car: it had stood short of its line
  # in 'rollUp' (or 'approach') saying only "Rolling up to the stop sign", no chime, no ledger, no fault, until the generic
  # 90 s stand. Now ROLLUP_WAIT_S of standing short with no lead within LEAD_GO_M, its goes spent, is H5
  for latched in (True, False):
    w = World(tmp_path, d0=6.)  # the car never moves: no G5 lift reaches it
    if not latched:
      w.control.held = False
    out = w.run(ROLLUP_WAIT_S + 8., **({} if latched else {'approach': w.control}))
    rollups = [k for k, d in enumerate(out) if d.go_cause == 'rollUp']
    assert len(rollups) == 4, latched
    h5 = next((k for k, d in enumerate(out) if d.fault == 'H5'), None)
    assert h5 is not None, (latched, out[-1].state, out[-1].reason)
    assert ROLLUP_WAIT_S - 0.1 <= h5 * DT_MDL <= ROLLUP_WAIT_S + 1.5
    assert all(d.reason for d in out[:h5]) and all(d.ledger == 'static' for d in out[round(1.2 / DT_MDL):h5])
    assert out[h5].prompt == 'stopSignTakeControl' and out[h5].reason == "Can't start: take control"
    # held: by the latch, handing back (map_controls' fault), or once it never latched by the decider's own hold
    assert all(d.prompt == 'stopSignTakeControl' and (d.latch == 'fault' and d.creep is None if latched else
                                                      any(s.should_stop for s in d.stops)) for d in out[h5:])
    assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)
  # a lead within LEAD_GO_M is a queue: waiting behind it is no fault, however long
  w = World(tmp_path, d0=6.)
  w.control.held = False
  out = w.run(ROLLUP_WAIT_S + 8., approach=w.control, lead_m=6.)
  assert not any(d.fault for d in out) and not any(d.go_cause == 'rollUp' for d in out)


def test_V2_a_cross_car_not_slowing_is_waited_for_until_it_has_passed(tmp_path):
  w = World(tmp_path)
  w.tracks = cars((5, 'left', rolling(0.5, d0=45., v0=11.)))
  out = w.run(4.)
  assert first_go(out) is None and any(h.key == 'u2:5' for d in out for h in d.holds)
  out = w.run(5.)
  assert first_go(out) is not None  # through and gone: our turn


def test_V4_a_vehicle_in_the_box_moving_is_a_dynamic_wait_standing_a_static_one_then_h3(tmp_path):
  w = World(tmp_path)
  w.objects = lambda ww: (vehicle(30, 12., 0.5, vx=0., vy=3.),)
  out = w.run(4.)
  assert first_go(out) is None and out[-1].ledger == 'dynamic' and any(h.key == 'box:30' for h in out[-1].holds)
  w = World(tmp_path)
  w.objects = lambda ww: (vehicle(30, 12., 0.5),)
  out = w.run(17.)
  assert first_go(out) is None and out[25].ledger == 'static' and out[-1].fault == 'H3'


def test_V5_vehicles_already_stopped_at_other_lines_go_first_in_the_order_first_seen_stopped(tmp_path):
  w = World(tmp_path)
  w.tracks = cars((1, 'left', at_line(t0=-10.)), (2, 'right', at_line(t0=-10., d0=12.)))
  watch(w)
  out = w.run(3.)
  assert out[-1].rank == 2 and set(w.sg.earlier) == {1, 2}
  w.tracks = cars((1, 'left', lambda t: LINE_D - 6. * (t - 5.)), (2, 'right', at_line(t0=-10., d0=12.)))
  out = w.run(4.)
  assert first_go(out) is None and out[-1].rank == 1
  w.tracks = cars((1, 'left', lambda t: None), (2, 'right', lambda t: LINE_D - 6. * (t - 9.)))
  out = w.run(8.)
  assert first_go(out) is not None


def test_V6_simultaneous_arrival_yields_to_the_right_and_goes_before_the_left(tmp_path):
  # a car stopping at the right line within TIE_S of our arrival: we yield; at the left line: we go first
  w = World(tmp_path)
  w.tracks = cars((4, 'right', at_line(t0=-3., d0=9.3 + 6. * 3.3, v0=6.)))
  out = w.run(5.)
  assert first_go(out) is None and 4 in w.sg.earlier
  w = World(tmp_path)
  w.tracks = cars((4, 'left', at_line(t0=-3., d0=9.3 + 6. * 3.3, v0=6.)))
  out = w.run(6.)
  assert 4 not in w.sg.earlier and first_go(out) is not None


def test_V14_facing_each_other_our_left_yields_to_their_straight_our_straight_goes_first(tmp_path):
  for maneuver, yields in (('left', True), ('straight', False)):
    w = World(tmp_path, maneuver=maneuver)
    w.tracks = cars((6, 'opposite', at_line(t0=-3., d0=9.3 + 6. * 3.3, v0=6.)))
    w.run(5.)
    assert (6 in w.sg.earlier) == yields, maneuver


def test_V7_V27_a_shield_that_moves_during_our_launch_stops_us_short_rank_kept_then_a_fresh_go(tmp_path):
  w = Drive(tmp_path, behaviour=lambda w: 'goes')
  w.tracks = cars((8, 'left', at_line(t0=1.5, d0=10.5, v0=2.)))  # arrives after us and waits at its line
  out = w.run(8.)
  i = first_go(out)
  assert i is not None and any(x.shielded for x in out[i - 1].lanes)
  go_t = (i + 1) * DT_MDL
  # it goes out of turn while we launch, short of our commit point: the capped abort, never below -2.0
  w2 = Drive(tmp_path, behaviour=lambda w: 'goes')
  w2.tracks = cars((8, 'left', lambda t: (LINE_D if t < go_t + 0.4 else LINE_D - 5. * (t - go_t - 0.4)) if t >= 1.5 else
                    max(10.5 - 2. * (t - 1.5), LINE_D) if t >= 1.5 else None))
  out = w2.run(16.)
  wd = [d for d in out if d.withdraw]
  assert wd and wd[0].withdraw.startswith(('outOfTurn', 'new', 'raw', 'reach:left'))  # its lane judged again, unshielded
  assert all(c[3] >= -2. - 1e-9 for c in w2.cmds) and w2.sg.edges.d_min >= max(d.position_m for d in out)


def test_V8_a_new_conflict_in_the_band_aborts_short_of_the_midpoint_and_in_a_lane_is_said_never_braked_for(tmp_path):
  for stage in ('committed', 'inLane'):
    w = Drive(tmp_path, behaviour=lambda w: 'goes', lag_s=0.2)
    out = []
    for _ in range(round(14. / DT_MDL)):
      d = w.step()
      out.append(d)
      if d.state == stage and w.tracks(w) == ():
        w.tracks = cars((9, 'right', rolling(w.t_cam - 100., d0=14., v0=8.)))
    if stage == 'committed':
      ab = [d for d in out if d.withdraw == 'bandAbort']
      said = [d for d in out if d.alert == 'crossTrafficAfterCommit']
      assert ab or said
      if ab:
        pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref)
        assert pos <= sr.band_midpoint(w.sg.edges) + 1e-6
    else:
      k = next(k for k, d in enumerate(out) if d.state == 'inLane')
      assert any(d.alert == 'crossTrafficAfterCommit' for d in out[k:])
      assert all(not d.stops for d in out[k:]) and not any(d.withdraw == 'bandAbort' for d in out[k:])


def test_V17_a_large_vehicle_gone_from_a_conflicting_lane_leaves_that_side_unseen_hide_s(tmp_path):
  w = World(tmp_path)
  w.tracks = cars((11, 'left', lambda t: 30. - 10. * t if t < 4.2 else None, 7))  # a truck through the left lane, gone at 4.2 s
  out = w.run(4.2)
  out = w.run(6.)
  hide = [k for k, d in enumerate(out) if any(h.key == 'hide:left' for h in d.holds)]
  assert hide and abs(len(hide) * DT_MDL - 2.0) < 0.15
  i = first_go(out)
  assert i is not None and i > hide[-1] + round(sr.CLEAR_S / DT_MDL) - 2


def test_V20_honking_or_a_tailgater_changes_nothing_clear_s_never_shortens(tmp_path):
  calm = first_go(World(tmp_path).run(5.))
  w = World(tmp_path)
  w.objects = lambda ww: (vehicle(40, -6., 0.), )  # a car right behind us
  assert first_go(w.run(5.)) == calm


def test_V21_a_right_turn_waits_for_the_right_blind_spot(tmp_path):
  w = World(tmp_path, maneuver='right')
  out = w.run(5., right_blindspot=True)
  assert first_go(out) is None and any(h.key == 'blindspot' for h in out[-1].holds)
  assert first_go(w.run(3.)) is not None
  out = World(tmp_path, maneuver='left').run(5., right_blindspot=True)
  assert first_go(out) is not None  # a left's go never waits for it


def test_V25_a_car_waiting_at_another_line_when_it_is_our_turn_shields_its_lane(tmp_path):
  w = World(tmp_path)
  w.tracks = cars((8, 'left', at_line(t0=1.5, d0=10.5, v0=2.)))  # arrives after us and waits
  out = w.run(8.)
  i = first_go(out)
  assert i is not None and any(x.role == 'left' and x.shielded for x in out[i - 1].lanes)
  left = next(x for x in out[i - 1].lanes if x.role == 'left')
  assert left.r_seen < left.r_req  # the shield, not reach, satisfies it (it hides the lane behind it)


@pytest.mark.parametrize('approach,shields', [({'bike_lane': True}, False), ({'shoulder_m': 2.0}, False), ({'shoulder_m': 1.5}, False),
                                              ({'shoulder_m': 0.}, True), ({'shoulder_m': -1.}, True)])
def test_V25_a_mapped_bike_lane_or_a_shoulder_wide_enough_to_pass_in_leaves_a_car_at_its_line_no_shield(tmp_path, approach, shields):
  # 2.3: the shield blocks its own lane only; a mapped bike lane or a shoulder of 1.5 m or more (Valhalla's cycle_lane and
  # shoulder, junctions.SHOULDER_MAPPED_M) lets a cyclist or a car pass it. Approach.bike_lane and shoulder_m were read and
  # used nowhere: the left lane was shielded and the car went
  j = junction()
  for a in j.approaches:
    if a.role == 'left':
      for k, v in approach.items():
        setattr(a, k, v)
  w = World(tmp_path, j=j)
  w.tracks = cars((8, 'left', at_line(t0=1.5, d0=10.5, v0=2.)))
  out = w.run(8.)
  i = first_go(out)
  lanes = out[i - 1].lanes if i else out[-1].lanes
  assert [x.shielded for x in lanes if x.role == 'left'] == [shields]
  assert (i is not None) == shields  # unshielded, the lane it hides is short of reach: no go
  e = w.sg.ev[8]
  assert e.stopped_at_line and e.shield == shields


def track_boxes(tracks_fn):
  """World.boxes for scripted tracks: each one's own raw box in its view (the detection its track is made of), from where
  the car is now."""
  def boxes(w):
    out = {}
    for trk in tracks_fn(w):
      b = dataclasses.replace(raw_box(trk.bearing_seen, conf=0.6, xy=(float(trk.x[0]), float(trk.x[1])), view=trk.view),
                              span_deg=tuple(trk.span_seen))
      out.setdefault(trk.view, []).append(b)
    return out
  return boxes


@pytest.mark.parametrize('raw', [False, True])
@pytest.mark.parametrize('role', ['left', 'right'])
def test_V25_a_shield_that_stays_stopped_through_our_crossing_is_never_a_band_abort_by_its_track_or_its_box(tmp_path, role, raw):
  # the commonest all-way scene (C1, V25): a car arrives after us and waits at its line, never moving. Once our car moves,
  # stop_evidence restarts its trust (bearings from a moving car aren't comparable), so in the band it reads notStopping and
  # untrusted, and its box's bearing swings as we pass it: neither is a vehicle that will reach us first, the shield known at
  # the go not having moved since. It had been an in-band abort 4.9-5.0 m past the stop point, then H5 ('no room for a go')
  w = Drive(tmp_path, behaviour=lambda w: 'goes', lag_s=0.3)
  w.tracks = cars((8, role, at_line(t0=1.5, d0=10.5, v0=2.)))
  if raw:
    w.boxes = track_boxes(w.tracks)
  out = w.run(25.)
  i = first_go(out)
  assert i is not None and any(x.role == role and x.shielded for x in out[i - 1].lanes)
  assert not [(k, d.withdraw) for k, d in enumerate(out) if d.withdraw]
  assert not [d.fault for d in out if d.fault]
  assert any(d.state in ('inLane', 'done') for d in out)
  # and one that moves once we are in the band is still the in-band abort or said (V8), its motion judged on the ground
  w = Drive(tmp_path, behaviour=lambda w: 'goes', lag_s=0.2)
  w.tracks = cars((8, role, at_line(t0=1.5, d0=10.5, v0=2.)))
  if raw:
    w.boxes = track_boxes(w.tracks)
  out = []
  for _ in range(round(16. / DT_MDL)):
    d = w.step()
    out.append(d)
    if d.state == 'committed' and not getattr(w, 'moved', False):
      w.moved, t0 = True, w.t_cam - 100.
      w.tracks = cars((8, role, lambda t, t0=t0: LINE_D - 6. * (t - t0) if t >= t0 else LINE_D))
      if raw:
        w.boxes = track_boxes(w.tracks)
  assert any(d.withdraw == 'bandAbort' or d.alert == 'crossTrafficAfterCommit' for d in out)


def test_V26_the_model_stopping_in_the_band_uncommits_never_clear_box(tmp_path):
  w = Drive(tmp_path, behaviour=lambda w: 'stops')
  out = w.run(14.)
  k = next(k for k, d in enumerate(out) if d.state == 'committed')
  back = next(j for j in range(k, len(out)) if out[j].state == 'observe')
  assert all(d.go_cause != 'clearBox' for d in out) and out[back].stops and out[back].stops[0].should_stop


def test_V27_a_shield_that_starts_moving_before_our_go_is_judged_again(tmp_path):
  w = World(tmp_path)
  w.tracks = cars((8, 'left', lambda t: (max(10.5 - 2. * (t - 1.5), LINE_D) if t < 3.5 else LINE_D - 0.6 * (t - 3.5)) if t >= 1.5 else None))
  out = w.run(6.)
  assert first_go(out) is None  # creeping out of its line: no shield, its lane short (and not stopping)


def test_P1_P10_people_crossing_in_front_are_a_dynamic_wait_with_no_timeout_and_a_chime_at_60_s(tmp_path):
  w = World(tmp_path)
  w.objects = lambda ww: (person(50, 6.5, 1.5 - 1.2 * ((ww.t_cam - 100.) % 6.), vy=-1.2),)  # in the crosswalk, past the front zone
  out = w.run(130.)
  assert first_go(out) is None and all(d.fault == '' for d in out)
  observing = [d for d in out if d.state == 'observe']
  assert all(d.ledger in ('dynamic', 'none') for d in observing) and sum(d.ledger == 'dynamic' for d in observing) > 0.8 * len(observing)
  chimes = [k for k, d in enumerate(out) if d.chime and (k == 0 or not out[k - 1].chime)]
  assert len(chimes) == 2 and all(out[k].reason == "Waiting for cross traffic" for k in chimes)
  w.objects = lambda ww: ()
  assert first_go(w.run(3.)) is not None


def walker(tid, d_along, y0, vy, t0=0.):
  """World.objects: a person walking across our road's line d_along m past the stop point, from y0 m left of our axis at
  vy m/s from t0 on, seen from where the car is now."""
  def objects(w):
    t = w.t_cam - 100.
    if t < t0:
      return ()
    pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref) if w.sg.token is not None else -w.control.d
    return (person(tid, w.sg.setback + d_along - pos, y0 + vy * (t - t0), vy=vy),)
  return objects


def test_P16_a_person_heading_for_the_near_crosswalk_blocks_the_go_until_they_have_crossed(tmp_path):
  # 2.4 item 7's arrival clause: on the right sidewalk 15 m out walking at 1.4 m/s toward the near crosswalk (between the
  # stop point and dEdgeMin, where the capped abort may not end short of it). With only the zones the go came at 3 s with
  # the person 10.8 m out, 5 s before they reached the crosswalk our car was still on
  w = World(tmp_path)
  w.objects = walker(50, 5.5, -15., 1.4)
  out = w.run(24.)
  i = first_go(out)
  assert i is not None
  y_go = -15. + 1.4 * (i + 1) * DT_MDL
  assert y_go > sg.HALF_LEFT_M  # across our path and on its way
  approaching = out[round(1.2 / DT_MDL):round(7. / DT_MDL)]
  assert all(any(h.key.startswith('pedArrive:50') for h in d.holds) and d.ledger == 'dynamic' for d in approaching)
  assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)
  # moving away from it, or standing still at the curb (D3, the curb rule's), doesn't block: the go at the usual time
  calm = first_go(World(tmp_path).run(5.))
  for walk in (walker(51, 5.5, -6., -1.4), walker(52, 5.5, -7., 0.)):
    w = World(tmp_path)
    w.objects = walk
    out = w.run(6.)
    assert first_go(out) == calm, [h.key for h in out[calm].holds]
  # a raw person box no track explains, on 2 of the last 3 runs of its view, is taken at PED_V_UNTRUSTED: it blocks; one
  # seen on a single run doesn't
  box = dataclasses.replace(raw_box(-50., conf=0.5, xy=(7.9, -9.), view='full'), cls='pedestrian')
  w = World(tmp_path)
  w.boxes = lambda ww: {'full': [box]}
  out = w.run(6.)
  assert first_go(out) is None and any(h.key.startswith('pedArriveRaw:full') for h in out[-1].holds)
  w = World(tmp_path)
  n = {'k': 0}

  def blip(ww):
    n['k'] += 1
    return {'full': [box]} if n['k'] == 9 else {}
  w.boxes = blip
  assert first_go(w.run(6.)) == calm


def test_P16_a_person_heading_for_the_near_crosswalk_during_the_launch_stops_us_short_of_it_or_we_carry_on_across(tmp_path):
  # 2.6: the arrival clause re-checked to commit. Seen early in the launch, the capped stop ends short of the crosswalk's
  # earliest plausible near edge (dEdgeMin less its depth) less M_EDGE, never below -2.0; seen where that stop can't end
  # short of it, the car carries on at STOP_GO_V_MAX or less rather than stop on it
  near_edge = None
  for early in (True, False):
    w = Drive(tmp_path, behaviour=lambda w: 'goes', lag_s=0.3)
    near_edge = w.sg.edges.d_min - sg.CROSSWALK_M - sr.M_EDGE if near_edge is None else near_edge
    out = []
    for _ in range(round(14. / DT_MDL)):
      d = w.step()
      out.append(d)
      pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref)
      if d.state == 'going' and w.objects(w) == () and (d.go_t >= 0.3 if early else pos >= near_edge - 0.8):
        w.objects = walker(60, 5.5, -9., 1.4, t0=w.t_cam - 100.)
    near_edge = w.sg.edges.d_min - sg.CROSSWALK_M - sr.M_EDGE
    wd = [d for d in out if d.withdraw]
    assert all(c[3] >= -2. - 1e-9 for c in w.cmds)
    if early:
      assert wd and wd[0].withdraw.startswith('pedArrive:60')
      stood = next(k for k in range(out.index(wd[0]), len(out)) if out[k].state == 'observe' and w.cmds[k][1] < 0.05)
      assert out[stood].position_m <= near_edge + 1e-6
    else:
      assert not [d.withdraw for d in out if d.withdraw.startswith('pedArrive')]
      assert any(d.state in ('committed', 'inLane') for d in out)
      assert all(c[1] <= sr.STOP_GO_V_MAX + 0.1 for c in w.cmds if c[6] == 'going')


def test_P2_a_person_in_the_far_crosswalk_holds_a_straight_crossing_not_a_right(tmp_path):
  far = sr.edge_band(13., 2, LANE_EDGE)
  w = World(tmp_path)
  x_far = 1.9 + w.sg._far_edge() + 1.5 + 0.5 if w.sg.token else None
  assert far.d_min == 7.0
  w.run(0.05)
  # beyond the box by its whole ranging band (0.7 x past the box's far side), in the far crosswalk's
  x_far = (w.sg._x(w.sg._far_edge(), -0.5) + 1.5) / 0.7
  w.objects = lambda ww: (person(51, x_far, 0.5, vy=0.8),)
  out = w.run(5.)
  assert first_go(out) is None and any(h.key == 'ped:51' and 'far crosswalk' in h.reason for h in out[-1].holds)
  w = World(tmp_path, maneuver='right')
  w.objects = lambda ww: (person(51, x_far, 0.5, vy=0.8),)
  assert first_go(w.run(5.)) is not None


def test_P6_a_person_into_the_front_zone_not_seen_leaving_holds_and_is_h7_after_30_s(tmp_path):
  w = World(tmp_path)
  w.run(1.5)
  w.objects = lambda ww: (person(60, 1.9 + 1.5, 0.3, vy=0.),)  # in front of the bumper, then lost below the hood
  w.run(0.5)
  w.objects = lambda ww: ()
  out = w.run(31.)
  assert first_go(out) is None and any(h.key == 'front' for h in out[10].holds)
  h7 = next(k for k, d in enumerate(out) if d.fault == 'H7')
  assert h7 * DT_MDL >= 29. and out[h7].reason == "Something may be in front of the car: take control"
  # seen leaving: a track ending at the zone's side moving out sideways, a person track starting outside that side within 1 s
  w = World(tmp_path)
  w.run(1.5)
  w.objects = lambda ww: (person(61, 1.9 + 1.5, 0.3 + 0.9 * (ww.t_cam - 101.5), vy=0.9),)
  w.run(0.9)
  w.objects = lambda ww: ()
  w.run(0.3)
  w.objects = lambda ww: (person(62, 1.9 + 1.5, 2.4, vy=0.9),)
  out = w.run(0.3)
  w.objects = lambda ww: ()
  out = w.run(3.)
  assert first_go(out) is not None


def test_P7_a_cyclist_passed_on_the_right_holds_a_right_turn_until_it_reappears_ahead_or_10_s(tmp_path):
  w = World(tmp_path, maneuver='right')
  w.objects = lambda ww: (SimpleNamespace(trackId=70, cls='cyclist', x=1.0, y=-1.5, vx=3., vy=0.),)
  w.run(1.5)
  w.objects = lambda ww: ()
  out = w.run(5.)
  assert first_go(out) is None and any(h.key == 'bikePassed' for h in out[-1].holds)
  w.objects = lambda ww: (SimpleNamespace(trackId=71, cls='cyclist', x=8., y=-1.5, vx=3., vy=0.),)  # ahead of us now
  w.run(0.1)
  w.objects = lambda ww: ()
  assert first_go(w.run(3.)) is not None


def test_E6_a_phantom_box_delays_the_go_only_and_never_brakes(tmp_path):
  calm = first_go(World(tmp_path).run(6.))
  w = World(tmp_path)
  w.run(2.0)
  w.boxes = lambda ww: lane_box(ww, 'left', 25., conf=0.4, views=('full', 'road'))
  out = w.run(0.2)
  w.boxes = lambda ww: {}
  out = out + w.run(4.)
  i = first_go(out)
  assert i is not None and 2.0 / DT_MDL + i > calm and all(not d.stops for d in out[:i])


def test_E7_a_stale_view_resets_clear_s_and_no_run_for_2_s_is_the_camera_paused(tmp_path):
  w = World(tmp_path, side='running')
  w.run(1.5)
  out = []
  for _ in range(round(2.5 / DT_MDL)):
    if CYCLE[w.k % 4] in ('sideLeft', 'sideRight'):
      w.k += 1
    out.append(w.step())
  assert any(h.key == 'stale' for h in out[12].holds) and out[-1].alert == 'cameraPaused' and out[-1].ledger == 'fault'


def test_E8_side_view_maps_stale_are_a_dynamic_wait_of_up_to_30_s_then_static(tmp_path):
  w = World(tmp_path, side='mapsStale')
  out = w.run(32.)
  cal = [d for d in out if any(h.key == 'calibration' for h in d.holds)]
  assert cal and cal[0].ledger == 'dynamic' and first_go(out) is None
  assert any(d.ledger == 'static' for d in out[round(31.5 / DT_MDL):]) and out[-1].maps_valid is False


def test_O1_position_uncertain_at_the_line_says_so_waits_10_s_then_h2(tmp_path):
  w = World(tmp_path)
  out = w.run(13., position_spread=5.)
  assert any(d.alert == 'junctionUncertain' for d in out)
  h2 = next(k for k, d in enumerate(out) if d.fault == 'H2')
  assert 10.5 <= (h2 + 1) * DT_MDL <= 11.5 and out[h2].reason == "Stop sign position uncertain: take control"
  w = World(tmp_path, j=junction(status='failed'))
  assert next(d for d in w.run(13.) if d.fault).fault == 'H2'


def test_O3_the_capped_abort_is_never_below_2_m_s2_and_holds_a_standing_car(tmp_path):
  from openpilot.selfdrive.turnpilot.stop_go import capped_abort
  for v in np.linspace(0., 2., 9):
    for d in (-1., 0., 0.2, 0.5, 1., 2.5, 6.):
      s = capped_abort(float(v), d, 'x')
      assert s.accel_cap >= -2. and s.speed_cap == 0. and not s.confirmed
      assert plan_accel(s.accel_cap, s.speed_cap, float(v)) >= -2.
  assert capped_abort(0.05, 0.3, 'x').should_stop


def test_O4_stalled_in_a_lane_takes_two_clear_box_goes_then_h5(tmp_path):
  w = Drive(tmp_path, behaviour=lambda w: 'goes')
  k = 0
  while w.step().state != 'inLane':
    k += 1
    assert k < 600
  w.behaviour = lambda w: 'holds'  # the model stops the car in the lane and holds it there
  w.v, w.queue = 0., deque([0.] * len(w.queue))
  out = w.run(20.)
  boxes = [d for d in out if d.go_cause == 'clearBox']
  assert k and len(boxes) == 2 and any(d.alert == 'stalledInJunction' for d in out)
  assert out[-1].fault == 'H5' and out[-1].prompt == 'stopSignTakeControl'


def test_O5_O15_a_model_that_wont_start_is_withdrawn_retried_then_h5(tmp_path):
  w = Drive(tmp_path, behaviour=lambda w: 'holds')
  out = w.run(40.)
  assert [d.withdraw for d in out if d.withdraw][0] in ('window', 'gate') and out[-1].fault == 'H5'
  assert all(d.takeover_s < 0. for d in out)


def test_a_turn_desire_before_the_first_go_holds_it_and_the_gos_own_pulse_never_holds_its_retry(tmp_path):
  # 2.4 item 3: a turn pulse before the go curls the car and throws away a side's reach: no go while one is out. The go's
  # own pulse (turnpilotd sends the turn's first pulse with the decider's go) is no such pulse: after a withdrawn go the yaw
  # and the wheel say whether the car curled, and the retry goes
  w = World(tmp_path, maneuver='right')
  out = w.run(5., desire_out=True)
  assert first_go(out) is None and any(h.key == 'yaw' for h in out[-1].holds)
  w = Drive(tmp_path, maneuver='right', behaviour=lambda w: 'holds')
  out = w.run(4.)
  i = first_go(out)
  assert i is not None
  out = w.run(12., desire_out=True)
  assert any(d.withdraw for d in out) and first_go(out) is not None
  assert not any(h.key == 'yaw' for d in out for h in d.holds)


def next_stop(d=20.):
  """Another stop sign engaged d m ahead, not held (a G18 frontage stop, the far line of a divided road)."""
  return SimpleNamespace(key=(0, 2), kind='stop', d=d, held=False, state=SimpleNamespace(moved_m=0.))


def test_G18_a_next_stop_sign_ahead_never_takes_over_an_encounter_holding_the_car_its_own_hold_or_its_handback(tmp_path):
  # once the latch has let go (a 'clear'), the decider's own hold is all that holds the car: a withdrawn go come to rest in
  # observe, or a fault's handback. Another stop control engaged within STOP_SWEEP_M (frontage stops 15-30 m apart, a
  # divided road's far line) had taken the encounter over, reset to 'approach' for it, and dropped that hold: no stop, no
  # prompt, and a model that drove carried the car from rest into the first junction with no check
  w = Drive(tmp_path, behaviour=lambda w: 'holds')
  while not any(d.withdraw for d in w.trace):
    w.step()
    assert len(w.trace) * DT_MDL < 20.
  while w.v > 0.05 or w.sg.aborting is not None:
    w.step()
  assert w.sg.state == 'observe' and not w.control.held
  token = w.sg.token
  nxt = next_stop()
  out = w.run(3., approach=nxt)
  # held in observe until its own retry, which is still this junction's (the next control's state never taken)
  assert w.sg.token is token and w.sg.key == (0, 1) and all(d.state in ('observe', 'going') for d in out)
  assert all(any(s.should_stop for s in d.stops) for d in out if d.state == 'observe')
  w = Drive(tmp_path, behaviour=lambda w: 'holds')
  out = w.run(40.)
  assert out[-1].fault == 'H5'
  out = w.run(2., approach=next_stop())
  assert all(d.prompt == 'stopSignTakeControl' and d.fault == 'H5' and any(s.should_stop for s in d.stops) for d in out)
  # a model that now drives from the observe hold, a car coming fast from the right: still the first junction's decider,
  # holding the car (it never moves), and the next control only once this encounter is done
  w = Drive(tmp_path, behaviour=lambda w: 'goes', lag_s=0.3)
  out = w.run(3.2)
  assert first_go(out) is not None
  w.boxes = lambda ww: lane_box(ww, 'right', 20., views=('full', 'road'))  # withdraws the go
  out = w.run(3.)
  assert any(d.withdraw for d in out) and w.sg.state == 'observe'
  w.boxes = lambda ww: {}
  w.model_says = lambda d: (1.0, False, model_v2())
  w.tracks = cars((31, 'right', rolling(w.t_cam - 100., d0=60., v0=12.)))
  nxt, odo0 = next_stop(), w.odo
  out = []
  for _ in range(round(4. / DT_MDL)):
    nxt.d -= w.v * DT_MDL
    out.append(w.step(approach=nxt))
  assert w.sg.token is not nxt.state and all(d.state != 'approach' for d in out)
  assert w.odo - odo0 < 0.3 and w.v < 0.1


def test_O11_O14_auto_without_its_data_acts_as_shadow_and_says_why_trial_skips_only_the_launch(tmp_path):
  import openpilot.selfdrive.turnpilot.routing.tests.test_certify as tc
  for changes, reason in (({'cameraSetbackM': None}, "Stop-sign go off: no camera measurements"),
                          ({'launchVerified': None}, "Stop-sign go off: launch not verified")):
    w = World(tmp_path, vis=tc.visibility(tmp_path, **changes))
    out = w.run(5.)
    assert first_go(out) is None and all(d.release is None and not d.stops for d in out)
    held = [d for d in out if d.state == 'observe']
    assert held and all(d.latch == 'shadow' and d.reason == reason and d.mode == 'auto' for d in held)
    assert out[-1].would_go == ('launch' in reason)  # what auto would have done, logged
  w = World(tmp_path, vis=tc.visibility(tmp_path, launchVerified=None), mode='trial')
  assert first_go(w.run(5.)) is not None


def test_O13_uncertifiable_at_the_line_with_nothing_to_wait_for_is_a_static_wait_then_h3(tmp_path):
  w = World(tmp_path, entries=[survey(validations=[])])  # surveyed, never validated
  out = w.run(17.)
  observing = [d for d in out if d.state == 'observe' and not d.fault]
  assert observing and all(d.ledger == 'static' for d in observing[1:])
  assert out[-1].fault == 'H3' and 'notValidated' in out[-1].reason


def test_O16_a_param_the_build_does_not_know_acts_as_shadow_out_loud(tmp_path):
  from openpilot.selfdrive.turnpilot.stop_go import mode_for
  assert mode_for(None) == ('auto', True) and mode_for(b'trial') == ('trial', True) and mode_for('nonsense') == ('shadow', False)
  w = World(tmp_path)
  out = w.run(4., mode_known=False)
  assert first_go(out) is None and out[-1].mode == 'shadow' and out[-1].latch == 'shadow'
  assert out[-1].reason == "Stop-sign go off: its param unknown to this build"


def test_refusals_a_lens_one_consumer_reports_differently_from_the_file_is_refused_loudly(tmp_path):
  w = World(tmp_path)
  good = 'equisolid f=452.0 k=0.3 visibility.json'
  assert first_go(w.run(5., lenses={'tracker': good, 'sideViews': good})) is not None
  w = World(tmp_path)
  out = w.run(5., lenses={'tracker': 'equidistant f=452.0 k=0.0 repo', 'sideViews': good})
  assert first_go(out) is None and out[-1].reason == "Stop-sign go off: camera lens mismatch" and out[-1].latch == 'shadow'


@pytest.mark.parametrize('stamp_m', [30., 40., 44., 48., 60.])
def test_route_and_drive_agree_on_a_junction_whose_live_reach_is_the_tables(tmp_path, stamp_m):
  from openpilot.selfdrive.turnpilot.routing import certify as ce
  s = survey(validations=[stamp(m=stamp_m)])
  vis = visibility(tmp_path)
  route = ce.certify(junction(), 'straight', vis, [s], ETA, ce.Runtime(mode='auto', side_views=True))
  out = World(tmp_path, entries=[s], vis=vis).run(5.)
  assert (route == ('certifiable', 'none')) == (first_go(out) is not None), (stamp_m, route)


def test_ledger_a_static_cause_that_flickers_still_meets_its_bound(tmp_path):
  # 4.6: a person standing in the crosswalk, detected 1.0 s of every 1.5 s (never CLEAR_S clear): the static wait's clock
  # isn't restarted by the gaps, so the ladder (H3) comes STATIC_WAIT_S of standing on, not never
  w = World(tmp_path)
  w.objects = lambda ww: (person(52, 6.5, 0.5),) if (ww.t_cam - 100.) % 1.5 < 1.0 else ()
  out = w.run(25.)
  h3 = next((k for k, d in enumerate(out) if d.fault == 'H3'), None)
  assert first_go(out) is None and h3 is not None and h3 * DT_MDL <= 1.0 + STATIC_WAIT_S * 1.6
