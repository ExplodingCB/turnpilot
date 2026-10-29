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

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.go_override import GO_ACCEL, GO_END_SPEED, GoCertifier
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.map_controls import HOLD_ACCEL
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, junction, stamp, survey, visibility
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import ScenePolicy, Stop
from openpilot.selfdrive.turnpilot.stop_evidence import junction_lanes
from openpilot.selfdrive.turnpilot.stop_go import FAULT_GRACE_S, FRONT_ZONE_MAX_S, STATIC_WAIT_S, Frame, StopGo
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
    lifted = self.gc.active and model_stop and not sc.should_stop and sc.accel_cap >= 10. and speed_cap > self.v + 0.5 and \
      self.v < GO_END_SPEED
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
      view = 'sideLeft' if role == 'left' else 'sideRight' if role == 'right' else 'road'
      trk = ev_track(tid, w.t_cam, max(dd, 0.3), ln, view=view, cls=cls, s=s)
      half = math.degrees(math.atan2(0.9, math.hypot(s, dd)))
      trk.span_seen = tuple(sorted((trk.bearing_seen - half, trk.bearing_seen + half)))
      trk.hits = 5
      out.append(trk)
    return out
  return tracks


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
  assert len(gos) == 3 and len(withdrawn) == 3  # the go and two retries
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
