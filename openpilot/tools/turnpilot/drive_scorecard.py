#!/usr/bin/env python3
"""Open-loop replay of a logged drive through the TurnPilot planner, scored per maneuver.

  drive_scorecard.py ROUTE [ROUTE ...] [--pkg-root DIR] [--cache-dir DIR] [--out PREFIX]

Each route's rlogs are read once into a per-segment cache of the planner's inputs. turnpilotd.main() then runs unmodified
over them, ticked at the car's own turnpilotPlan times: a SubMaster fed the messages logged since the last tick, the
non-conflated detector runs through the same drain, the car's params from initData, and the clock and wall time set from
the log. Each replayed plan is scored against what the driver did, per route maneuver and map control, and written as
PREFIX.md and PREFIX.json. Output names maneuvers by index and route time only: no coordinates, place or road names,
junction ids, dongle ids or log paths.

--pkg-root DIR imports openpilot.selfdrive.turnpilot from DIR/selfdrive/turnpilot instead of the repo, so a frozen copy
of the package can be scored against the working tree.

Open loop: the logged car did what the logged build and the driver did, not what the replayed planner commands. Once the
two disagree in a way that would change the car's motion (the planner stops or holds where the car drove on, or drives on
where the car stood), later frames of that maneuver no longer correspond, so each maneuver is scored only up to that point
(its divergence) and its verdict comes from what the planner did there.

By default the planner is replayed as engaged throughout (carControl enabled with lateral and longitudinal active, the
driver's pedals and steering override masked): the question asked is "would this build have driven it". --logged-engagement
replays the log's own engagement and pedals instead, which is the mode to check the instrument against the logged plans of
the build that drove (--agreement).

Lane changes: the logged car changes lanes only where its driver did, so a change the planner asks for is never seen to
happen. In the forced mode the simulated driver answers the planner's prompt to signal a lane change SIGNAL_REACT_S after it
comes up (the logged lamps are kept as well), and a request the planner then holds for LC_COMPLETE_S, the time a change
takes, is that maneuver's divergence, verdict would-drive (lane change requested): from there the car would be in another
lane than the logged one. A request dropped sooner changes nothing in the score; the row says how long it was held.
"""
import argparse
import bisect
import datetime
import glob
import json
import math
import os
import pickle
import re
import sys
import time
import traceback
import types
from collections import Counter

CACHE_VERSION = 1
DT = 0.05
# the planner's inputs (turnpilotd.main()'s SubMaster and drain) and the driver truth the score reads
INPUTS = ('carState', 'carControl', 'navInstruction', 'modelV2', 'turnpilotDetections', 'extrinsicsCalibration', 'deviceState',
          'narrowRoadCameraState', 'turnpilotLights', 'turnpilotFarDetections')
TRUTH = ('selfdriveState', 'radarState')
KEEP = INPUTS + TRUTH + ('turnpilotPlan', 'carParams')
GAP_FILL_S = 0.15     # a gap in the logged plans longer than this is ticked on a DT grid
TAKEOVER_MIN_S = 3.   # the simulated driver's shortest takeover...
REENGAGE_V = 3.       # ...and engaging again once moving over this...
REENGAGE_S = 2.       # ...this long, the plan's handback gone
SIGNAL_REACT_S = 1.   # ...and signals a lane change this long after the planner's prompt for it comes up

# scoring
TURN_WINDOW_M = 400.  # m before a route turn its row starts (its lane changes are part of it)
CONTROL_WINDOW_M = 150.
AFTER_S = 6.          # s after the junction a row runs on
ATTACH_M = (-8., 45.)  # a control this far before a turn point (m) is that turn's junction
REACHED_M = 20.       # a maneuver or control navd reported this close was reached (one dropped further out was rerouted away)
STOOD_V = 0.3         # m/s
MOVING_V = 0.5
HOLD_V = 1.0          # a planner cap under this is a stop or hold
DIVERGE_HOLD_S = 0.5  # the planner holding while the car moves over DIVERGE_MOVING_V this long: diverged
DIVERGE_MOVING_V = 2.
LEFT_HOLD_V = 0.5     # ...or held at a standstill (its speed under 0.3 m/s) while the car pulls away from one over this...
LEFT_HOLD_S = 0.25    # ...this long
PULL_AWAY_S = 3.      # s since the car last stood that its moving counts as pulling away
DIVERGE_GO_S = 1.5    # the car standing with no lead while the planner drives on this long: diverged
SLOW_MARGIN = 1.0     # m/s the planner's speed under the car's that counts as slowing
DEEP_SLOW_S = 2.      # the planner under half the car's speed (and the car over DIVERGE_MOVING_V) this long: diverged
LEAD_STOP_M = 15.     # a lead this close and slow explains the car standing
STOP_MATCH_S = 4.     # a planner stop the car itself makes within this is the same stop, made sooner
COMFORT_DECEL = 2.0   # m/s^2: the stopping curve a shouldStop is read as
FOLLOW_TURN_DEG = 30.   # the car turned at least this way at a route turn: it followed the route
FOLLOW_STRAIGHT_DEG = 45.
DEVIATION_PROMPTS = ('offRouteTakeControl', 'turnAbandonedTakeControl')  # from the car leaving the route, open loop
RED_LOOK_S = 6.       # camera red evidence: this many raw red/yellow reads in the s before divergence, over the greens
RED_MIN_READS = 3
DRIVER_LINE_M = (-8., 12.)  # driver red evidence: the driver's own stand this far before the signal's stop point...
DRIVER_STAND_S = 3.         # ...this long, no lead
CHECK_LIGHT_S = 3.    # the car stood this long, no lead, at a signalised junction the planner drove on at: the light may be red
LC_COMPLETE_S = 5.    # a lane change request held this long has made its change (lane_planner.LC_CHANGE_S; route 27's took 5.5 s)
LC_GAP_S = 0.5        # ...frames the request is withheld, its offer still up, for this long at most don't end the hold
LC_START_V = 2.3      # DesireHelper starts a requested change over this speed (NAV_LANE_CHANGE_SPEED_MIN), the blind spot clear
STOP_RAIL_M = 8.      # a stop-sign hold this close to its line where the car rolled the sign is the rail, not a wrong stop
TAKEOVER_TAIL_S = 2.  # an estimated takeover lasts to the row's end plus this
HOLD_PROMPTS = ('waitingForGap', 'creepingForView', 'waitingForCross')
SG_HOLDS = ('stopped', 'rollUp', 'observe')
MC_HOLDS = ('held', 'awaitingDriver', 'waitingCross', 'creeping')
GO_PROMPTS = ('leftGo', 'stopGo')
COORD = re.compile(r'-?\d{1,3}\.\d{4,}')
SAFE_KEYS = ('state', 'why', 'reason', 'cause', 'source', 'accepted', 'color', 'colour', 'kind', 'side', 'mode', 'status', 'view',
             'rejected', 'light', 'action', 'abort', 'phase')
VERDICTS = ('would-drive', 'would-stand', 'would-stop-wrongly', 'would-run-red', 'skipped', 'faulted')
BAD = ('would-stand', 'would-stop-wrongly', 'would-run-red', 'faulted')


def clean(s, n: int = 60) -> str:
  return COORD.sub('<c>', str(s))[:n]


# ---------------------------------------------------------------- extraction and cache

def segment_paths(logs_dir: str, route: str) -> list[str]:
  paths = glob.glob(os.path.join(logs_dir, '*', f'{route}--*', 'rlog.zst'))
  return sorted(paths, key=lambda p: int(os.path.basename(os.path.dirname(p)).rsplit('--', 1)[1]))


def extract_segment(path: str) -> dict:
  """The planner's inputs of one rlog as (logMonoTime, service, event bytes), its params and the wall clock offset."""
  from openpilot.tools.lib.logreader import LogReader
  msgs, params, wall_off = [], {}, None
  for m in LogReader(path, only_union_types=True):
    w = m.which()
    if w in KEEP:
      msgs.append((m.logMonoTime, w, m.as_builder().to_bytes()))
    elif w == 'initData':
      params = {e.key: bytes(e.value) for e in m.initData.params.entries}
    elif w == 'clocks' and wall_off is None and m.clocks.wallTimeNanos > 0:
      wall_off = (m.clocks.wallTimeNanos - m.logMonoTime) / 1e9
  msgs.sort(key=lambda x: x[0])
  return {'version': CACHE_VERSION, 'msgs': msgs, 'params': params, 'wall_off': wall_off}


def load_route(route: str, logs_dir: str, cache_dir: str) -> dict:
  paths = segment_paths(logs_dir, route)
  if not paths:
    raise SystemExit(f'no rlogs for {route} under the logs dir')
  os.makedirs(cache_dir, exist_ok=True)
  msgs, params, wall_off = [], {}, None
  for p in paths:
    seg = os.path.basename(os.path.dirname(p)).rsplit('--', 1)[1]
    cp = os.path.join(cache_dir, f'{route}--{seg}.pkl')
    data = None
    if os.path.exists(cp):
      with open(cp, 'rb') as f:
        data = pickle.load(f)
      if data.get('version') != CACHE_VERSION:
        data = None
    if data is None:
      data = extract_segment(p)
      with open(cp + '.tmp', 'wb') as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
      os.replace(cp + '.tmp', cp)
    msgs.extend(data['msgs'])
    params = params or data['params']
    wall_off = wall_off if wall_off is not None else data['wall_off']
  msgs.sort(key=lambda x: x[0])
  return {'msgs': msgs, 'params': params, 'wall_off': wall_off}


def build_frames(msgs: list) -> list[dict]:
  """One frame per logged turnpilotPlan (the car's own 20 Hz loop), gaps filled on a DT grid. A frame carries the latest
  message of each conflated service logged since the last tick, every detector run in that time, and the logged plan."""
  plans = [(t, b) for t, w, b in msgs if w == 'turnpilotPlan']
  if not plans:
    return []
  ticks, logged = [], {}
  for t, b in plans:
    if ticks and (t - ticks[-1]) / 1e9 > GAP_FILL_S:
      g = ticks[-1] + int(DT * 1e9)
      while g < t - int(DT * 0.5e9):
        ticks.append(g)
        g += int(DT * 1e9)
    ticks.append(t)
    logged[t] = b
  frames = [{'t': t, 'msgs': {}, 'dets': [], 'logged': logged.get(t)} for t in ticks]
  cp = None
  for t, w, b in msgs:
    if w == 'turnpilotPlan':
      continue
    if w == 'carParams':
      cp = cp or b
      continue
    i = bisect.bisect_left(ticks, t)  # a message logged before its tick's plan was polled for it (or the next one)
    if i >= len(frames):
      continue
    f = frames[i]
    f['msgs'][w] = b
    if w == 'turnpilotDetections':
      f['dets'].append(b)
  if frames:
    frames[0]['carParams'] = cp
  return frames


# ---------------------------------------------------------------- the package and its fakes

class ReplayDone(Exception):
  pass


CLOCK = {'mono': 0., 'wall': None}


def load_package(pkg_root: str | None):
  """turnpilotd and hooks, from pkg_root/selfdrive/turnpilot when given (before anything imports the package)."""
  if pkg_root:
    root = os.path.join(os.path.abspath(pkg_root), 'selfdrive')
    if not os.path.isfile(os.path.join(root, 'turnpilot', 'turnpilotd.py')):
      raise SystemExit('--pkg-root has no selfdrive/turnpilot/turnpilotd.py')
    if 'openpilot.selfdrive.turnpilot' in sys.modules:
      raise RuntimeError('openpilot.selfdrive.turnpilot was imported before --pkg-root was applied')
    import openpilot.selfdrive as sd
    sd.__path__.insert(0, root)
  time.monotonic = lambda: CLOCK['mono']  # every staleness and rate limit on the log's clock
  import openpilot.selfdrive.turnpilot.turnpilotd as td
  import openpilot.selfdrive.turnpilot.hooks as hooks
  return td, hooks


class FakeParams:
  """The car's params as initData logged them, typed by this repo's params library; writes stay in memory."""
  def __init__(self, raw: dict, car_params: bytes | None):
    from openpilot.common.params import Params, UnknownKeyName
    self._p, self._unknown = Params(), UnknownKeyName
    self.store = {}
    for k, v in raw.items():
      try:
        self.store[k] = self._p.cpp2python(k, v)
      except Exception:
        pass
    if car_params is not None and 'CarParams' not in self.store:
      from openpilot.cereal import messaging
      self.store['CarParams'] = messaging.log_from_bytes(car_params).carParams.as_builder().to_bytes()

  def _check(self, key):
    try:
      self._p.check_key(key)
    except Exception as e:
      raise self._unknown(key) from e

  def get(self, key, block=False, return_default=False):
    if key in self.store:
      return self.store[key]
    self._check(key)
    return self._p.get_default_value(key) if return_default else None

  def get_bool(self, key, block=False):
    v = self.get(key)
    return bool(v) if not isinstance(v, (bytes, str)) else v in (b'1', '1', 'true', b'true')

  def put(self, key, dat, block=False):
    self.store[key] = dat

  def put_bool(self, key, val, block=False):
    self.store[key] = bool(val)

  def put_nonblocking(self, key, dat):
    self.store[key] = dat

  def put_bool_nonblocking(self, key, val):
    self.store[key] = bool(val)

  def remove(self, key):
    self.store.pop(key, None)


class Harness:
  """Runs td.main() over frames: its SubMaster, drain, PubMaster, Params, Ratekeeper, TurnPilot and clock replaced."""
  def __init__(self, td, hooks, frames: list, params: dict, wall_off: float | None, logged_engagement: bool = False):
    from openpilot.cereal import messaging
    from openpilot.common.swaglog import cloudlog
    self.td, self.hooks, self.frames, self.wall_off = td, hooks, frames, wall_off
    self.real = messaging
    self.logged_engagement = logged_engagement
    self.take_control = tuple(getattr(hooks, 'TAKE_CONTROL_PROMPTS', ()))
    self.params = FakeParams(params, frames[0].get('carParams') if frames else None)
    self.i = 0
    self.cur_dets: list = []
    self.tp = None
    self.records: list[dict] = []
    self.events: list = []
    self.errors: Counter = Counter()
    self.first_tb: dict = {}
    self.truth: dict = {}
    self.nav: dict | None = None
    self.odo = 0.
    self.last_t: float | None = None
    self.agreement: Counter = Counter()
    self.agreement_n = 0
    self.sim_en = True            # the replayed engagement (forced mode): on, but for the simulated driver's takeovers
    self.sim_off_t: float | None = None
    self.sim_moving_s = 0.
    self.sim_takeovers: list = []
    self.lc_prompt: tuple = ('none', 0.)  # the planner's lane change signal prompt (side, since) the simulated driver answers
    self.sim_sig = 'none'
    self._patch(cloudlog)

  # -- patches
  def _patch(self, cloudlog):
    td, h = self.td, self

    def event(name, *a, **kw):
      h.events.append((str(name), {k: v for k, v in kw.items() if k in SAFE_KEYS and isinstance(v, (str, bool, int))}))

    def err(msg, *a, **kw):
      h.errors[clean(msg, 80)] += 1
      if sys.exc_info()[0] is not None and clean(msg, 80) not in h.first_tb:
        h.first_tb[clean(msg, 80)] = traceback.format_exc(limit=6)[-1500:]
    for name in ('event',):
      setattr(cloudlog, name, event)
    for name in ('exception', 'error', 'warning', 'critical'):
      setattr(cloudlog, name, err)
    for name in ('info', 'debug'):
      setattr(cloudlog, name, lambda *a, **k: None)

    class Proxy:
      def __getattr__(self, k):
        return getattr(h.real, k)

      def SubMaster(self, services, **kw):
        return h.make_sm(services, **kw)

      def PubMaster(self, services):
        return types.SimpleNamespace(send=h.send)

      def sub_sock(self, *a, **k):
        return 'drain'

      def drain_sock(self, sock, *a, **k):
        return h.cur_dets
    td.messaging = Proxy()
    td.Params = lambda *a, **k: h.params
    td.Ratekeeper = lambda *a, **k: types.SimpleNamespace(keep_time=lambda: None)

    class ReplayDatetime(datetime.datetime):
      @classmethod
      def now(cls, tz=None):
        return datetime.datetime.fromtimestamp(CLOCK['wall'] or 0., tz)
    td.datetime = types.SimpleNamespace(datetime=ReplayDatetime, UTC=datetime.UTC, timezone=datetime.timezone,
                                        timedelta=datetime.timedelta, date=datetime.date)
    td.system_time_valid = lambda: CLOCK['wall'] is not None
    base = td.TurnPilot

    class RecordedTurnPilot(base):
      def __init__(self, *a, **k):
        super().__init__(*a, **k)
        h.tp = self
    td.TurnPilot = RecordedTurnPilot

  def make_sm(self, services, **kw):
    real = self.real
    sub, poller = real.sub_sock, real.Poller
    real.sub_sock = lambda *a, **k: None
    real.Poller = lambda *a, **k: types.SimpleNamespace(poll=lambda *a, **k: [], registerSocket=lambda *a, **k: None)
    try:
      sm = real.SubMaster(services, **kw)
    finally:
      real.sub_sock, real.Poller = sub, poller
    sm.update = lambda timeout=100: self.next_frame(sm)
    return sm

  # -- one tick
  def next_frame(self, sm):
    if self.i >= len(self.frames):
      raise ReplayDone
    f = self.frames[self.i]
    self.i += 1
    t = f['t'] / 1e9
    CLOCK['mono'] = t
    CLOCK['wall'] = t + self.wall_off if self.wall_off is not None else None
    ev = {w: self.real.log_from_bytes(b) for w, b in f['msgs'].items()}
    self.truth_from(ev, t)
    self.drive_sim(t)
    feed = []
    for w, e in ev.items():
      if w not in sm.services:
        continue
      if not self.logged_engagement and w == 'carControl':
        b = e.as_builder()
        b.carControl.enabled = b.carControl.latActive = b.carControl.longActive = self.sim_en
        e = b.as_reader()
      elif not self.logged_engagement and self.sim_en and w == 'carState':
        b = e.as_builder()
        b.carState.gasPressed = b.carState.brakePressed = b.carState.steeringPressed = False
        side, since = self.lc_prompt
        self.sim_sig = side if side != 'none' and t - since >= SIGNAL_REACT_S else 'none'
        if self.sim_sig != 'none':  # the simulated driver's lamp for the lane change asked for; the other side's off
          b.carState.leftBlinker, b.carState.rightBlinker = self.sim_sig == 'left', self.sim_sig == 'right'
        e = b.as_reader()
      feed.append(e)
    self.cur_dets = [self.real.log_from_bytes(b) for b in f['dets']]
    self.cur_logged = f['logged']
    self.events = []
    sm.update_msgs(t, feed)

  def drive_sim(self, t: float) -> None:
    """The simulated driver of the forced mode: takes over the frame after the replayed plan hands back (a take-control
    prompt, a handback or a fault), as the driver must, and engages again once that plan has cleared, TAKEOVER_MIN_S has
    passed and the car has been moving over REENGAGE_V for REENGAGE_S (past the junction). The logged mode follows the log."""
    if self.logged_engagement:
      self.sim_en = self.truth.get('en', False)
      return
    last = self.records[-1] if self.records else None
    asks = last is not None and (last['prompt'] in self.take_control or last['handback'] != 'none' or last['fault'] != 'none')
    if self.sim_en and asks:
      self.sim_en, self.sim_off_t, self.sim_moving_s = False, t, 0.
      self.sim_takeovers.append((t, last['prompt'] if last['prompt'] in self.take_control else
                                 last['handback'] if last['handback'] != 'none' else last['fault']))
    elif not self.sim_en:
      self.sim_moving_s = self.sim_moving_s + DT if self.truth.get('v', 0.) > REENGAGE_V else 0.
      if not asks and t - self.sim_off_t >= TAKEOVER_MIN_S and self.sim_moving_s >= REENGAGE_S:
        self.sim_en = True

  def truth_from(self, ev: dict, t: float):
    tr = self.truth
    if 'carState' in ev:
      cs = ev['carState'].carState
      tr.update(v=float(cs.vEgo), a=float(cs.aEgo), gas=bool(cs.gasPressed), brake=bool(cs.brakePressed),
                steer=bool(cs.steeringPressed), lb=bool(cs.leftBlinker), rb=bool(cs.rightBlinker),
                lbs=bool(cs.leftBlindspot), rbs=bool(cs.rightBlindspot))
    if 'selfdriveState' in ev:
      tr['en'] = bool(ev['selfdriveState'].selfdriveState.enabled)
    if 'radarState' in ev:
      lo = ev['radarState'].radarState.leadOne
      tr['lead'] = float(lo.dRel) if lo.present and lo.dRel < LEAD_STOP_M and lo.vLead < 1. else None
    if 'modelV2' in ev:
      oz = ev['modelV2'].modelV2.orientationRate.z
      tr['yaw'] = -float(oz[0]) if len(oz) else 0.
    if 'turnpilotDetections' in ev:
      d = ev['turnpilotDetections'].turnpilotDetections
      if str(d.lightStatus) != 'wideCamera':
        states = [str(lt.state) for lt in d.lights.lights]
        tr['camst'] = str(d.lightStatus)
        tr['cam'] = 'red' if 'red' in states else 'yellow' if 'yellow' in states else 'green' if 'green' in states else \
          ('unknown' if states else None)
    if self.last_t is not None:
      self.odo += max(tr.get('v', 0.), 0.) * (t - self.last_t)
    self.last_t = t
    self.nav = None
    if 'navInstruction' in ev:
      n, valid = ev['navInstruction'].navInstruction, bool(ev['navInstruction'].valid)
      self.nav = {'valid': valid, 'd': float(n.maneuverDistance), 'type': str(n.maneuverType), 'mod': str(n.maneuverModifier),
                      'rc': int(n.routeCount), 'controls': [(int(c.id), str(c.kind), float(c.distance)) for c in n.controls]}

  # -- each plan
  def send(self, svc, msg):
    if svc != 'turnpilotPlan':
      return
    p = msg.as_reader().turnpilotPlan
    h, tp, tr = self.hooks, self.tp, self.truth
    mc, ul, sg = p.mapControl, p.unprotectedLeft, p.stopGo
    try:
      names = list(h.plan_event_names(p))
    except Exception:
      names = []
    try:
      names += list(h.lane_change_events(p))
    except Exception:
      pass
    fol = getattr(tp, 'follower', None) if tp is not None else None
    up = getattr(fol, 'upcoming', None) if fol is not None else None
    rec = {
      't': CLOCK['mono'], 'odo': self.odo, 'sim_en': self.sim_en, 'v': tr.get('v', 0.), 'a': tr.get('a', 0.), 'en': tr.get('en', False),
      'gas': tr.get('gas', False), 'brake': tr.get('brake', False), 'steer': tr.get('steer', False),
      'lb': tr.get('lb', False), 'rb': tr.get('rb', False),
      'lead': tr.get('lead'), 'yaw': tr.get('yaw', 0.), 'cam': tr.get('cam'), 'camst': tr.get('camst'), 'nav': self.nav,
      'state': str(p.state), 'desire': int(p.desire), 'lc': str(p.laneChange), 'offer': str(p.laneChangeOffer),
      'abort': str(p.laneChangeAbort), 'want': int(p.laneChangeWant), 'cap': float(p.speedCap), 'acap': float(p.accelCap),
      'stop': bool(p.shouldStop), 'prompt': str(p.controlPrompt), 'alert': str(p.controlAlert), 'reason': clean(p.reason),
      'md': float(p.maneuverDistance), 'mtype': str(p.maneuverType), 'mmod': str(p.maneuverModifier), 'fast': bool(p.tooFastToTurn),
      'skip': clean(p.skipReason, 30), 'fault': str(p.fault), 'handback': str(p.handback), 'sigp': str(p.turnSignal.prompt),
      'mck': str(mc.kind), 'mca': str(mc.action), 'mcd': float(mc.distance), 'mcl': str(mc.lightState), 'mcs': str(mc.lightSource),
      'uls': str(ul.state), 'ulwhy': clean(ul.why, 40), 'ulclear': float(ul.clearS), 'ulview': bool(ul.viewBlocked), 'ulchime': bool(ul.chime),
      'sgs': str(sg.state), 'sgr': clean(sg.reason, 40), 'sgchime': bool(sg.chime), 'names': names,
      'fid': getattr(fol, 'maneuver_id', None), 'fkind': getattr(up, 'kind', None), 'fside': getattr(up, 'side', None),
      'ev': [(n, kw) for n, kw in self.events if n.startswith('turnpilot')],
      'lcp': str(p.signalPrompt), 'sim_sig': self.sim_sig, 'lbs': tr.get('lbs', False), 'rbs': tr.get('rbs', False),
    }
    if rec['lcp'] != self.lc_prompt[0]:
      self.lc_prompt = (rec['lcp'], rec['t'])
    self.records.append(rec)
    self.events = []
    if self.cur_logged is not None:
      self.compare_logged(p)

  def compare_logged(self, p):
    lp = self.real.log_from_bytes(self.cur_logged).turnpilotPlan
    self.agreement_n += 1
    for k, f in (('state', lambda x: str(x.state)), ('shouldStop', lambda x: bool(x.shouldStop)),
                 ('speedCap', lambda x: round(float(x.speedCap), 0)), ('desire', lambda x: int(x.desire)),
                 ('controlPrompt', lambda x: str(x.controlPrompt)), ('controlAlert', lambda x: str(x.controlAlert)),
                 ('mapControl.action', lambda x: str(x.mapControl.action)),
                 ('mapControl.lightState', lambda x: str(x.mapControl.lightState)),
                 ('unprotectedLeft.state', lambda x: str(x.unprotectedLeft.state)),
                 ('laneChangeOffer', lambda x: str(x.laneChangeOffer)), ('tooFastToTurn', lambda x: bool(x.tooFastToTurn)),
                 ('turnSignal.prompt', lambda x: str(x.turnSignal.prompt))):
      if f(p) != f(lp):
        self.agreement[k] += 1

  def run(self):
    try:
      self.td.main()
    except ReplayDone:
      pass
    return self.records


# ---------------------------------------------------------------- maneuvers

def side_of(mod: str) -> str | None:
  return 'left' if 'left' in mod else 'right' if 'right' in mod else None


def dead_reckon(recs: list, tracks: list) -> None:
  """Each track's per-record distance (m): its last nav distance less the distance driven since."""
  for tr in tracks:
    pts = tr['pts']  # (record index, distance)
    d = {}
    for k, (i, dist) in enumerate(pts):
      end = pts[k + 1][0] if k + 1 < len(pts) else min(len(recs), i + int(30 / DT))
      for j in range(i, end):
        d[j] = dist - (recs[j]['odo'] - recs[i]['odo'])
    tr['d'] = d
    past = [j for j in sorted(d) if d[j] <= 0.]
    tr['pass_i'] = past[0] if past else None
    tr['min_d'] = min(d.values()) if d else None
    tr['obs_min'] = min(dist for _, dist in pts) if pts else None  # the closest navd itself reported


def find_maneuvers(recs: list) -> list[dict]:
  """Route maneuvers from navInstruction (type and modifier, its distance counting down; a reroute that keeps the same
  maneuver ahead is the same one) and map controls (kind, distance; matched across reroutes by kind and distance)."""
  turns, cur = [], None
  ctl_open: list[dict] = []
  ctls: list[dict] = []
  for i, r in enumerate(recs):
    n = r['nav']
    if n is None or not n['valid'] or not n['type']:
      continue
    key = (n['type'], n['mod'])
    if cur is not None and key == cur['key'] and n['d'] <= cur['pts'][-1][1] - (r['odo'] - recs[cur['pts'][-1][0]]['odo']) + 30.:
      cur['pts'].append((i, n['d']))
    else:
      cur = {'key': key, 'pts': [(i, n['d'])]}
      turns.append(cur)
    for _cid, kind, dist in n['controls']:
      if dist > CONTROL_WINDOW_M * 2 or dist < -30.:
        continue
      match = None
      for c in ctl_open:
        pi, pd = c['pts'][-1]
        if c['kind'] == kind and abs(pd - (r['odo'] - recs[pi]['odo']) - dist) < 25. and i - pi < int(20 / DT):
          match = c
          break
      if match is None:
        match = {'kind': kind, 'pts': []}
        ctl_open.append(match)
        ctls.append(match)
      if not match['pts'] or match['pts'][-1][0] != i:
        match['pts'].append((i, dist))
  dead_reckon(recs, turns)
  dead_reckon(recs, ctls)
  rows = []
  for m in turns:
    typ, mod = m['key']
    side = side_of(mod)
    if side is None or typ in ('depart', 'arrive') or m['obs_min'] is None or m['obs_min'] > REACHED_M:
      continue
    rows.append({'type': 'turn', 'side': side, 'label': f'{typ} {mod}', 'track': m, 'controls': []})
  for c in ctls:
    if c['obs_min'] is None or c['obs_min'] > REACHED_M:
      continue
    host = None
    for row in rows:
      common = [i for i in c['d'] if i in row['track']['d']]
      if common and ATTACH_M[0] <= row['track']['d'][common[-1]] - c['d'][common[-1]] <= ATTACH_M[1]:
        host = row
        break
    if host is not None:
      host['controls'].append(c)
    else:
      rows.append({'type': 'control', 'side': None, 'label': f'{c["kind"]} straight', 'track': c, 'controls': [c]})
  for row in rows:
    tr = row['track']
    reach = TURN_WINDOW_M if row['type'] == 'turn' else CONTROL_WINDOW_M
    idx = sorted(i for i, d in tr['d'].items() if d <= reach)
    start = idx[0] if idx else min(tr['d'])
    for c in row['controls']:
      ci = sorted(i for i, d in c['d'].items() if d <= CONTROL_WINDOW_M)
      if ci:
        start = min(start, ci[0])
    pass_i = tr['pass_i'] if tr['pass_i'] is not None else max(tr['d'])
    row['start'], row['pass'] = start, pass_i
    row['end'] = min(len(recs) - 1, pass_i + int(AFTER_S / DT))
    row['kind'] = row['controls'][0]['kind'] if row['controls'] else 'none'
  rows.sort(key=lambda r: r['pass'])
  for prev, row in zip(rows, rows[1:], strict=False):
    row['start'] = min(max(row['start'], prev['end']), row['pass'])  # nothing of the junction before
  return rows


# ---------------------------------------------------------------- scoring

def plan_speed(r: dict, d_row: float | None) -> float:
  """m/s the planner would let the car go at this frame: its cap, under a shouldStop the comfortable curve to its stop."""
  v = r['cap']
  if r['stop']:
    d = r['mcd'] if r['mck'] != 'none' else 0.
    v = min(v, math.sqrt(2. * COMFORT_DECEL * max(d - 1., 0.)))
  if r['prompt'] in HOLD_PROMPTS and r['v'] < HOLD_V:
    v = min(v, 0.)
  return v


def holding(r: dict) -> bool:
  """The planner keeps the car at or short of a line: a stop, a hold prompt, a stop sign's roll-up or wait, a held map stop."""
  return r['stop'] or r['prompt'] in HOLD_PROMPTS or r['cap'] < HOLD_V or r['sgs'] in SG_HOLDS or r['mca'] in MC_HOLDS


def acting(r: dict) -> bool:
  return r['state'] not in ('disabled', 'unavailable') and r['fault'] == 'none' and r['handback'] == 'none'


def score_row(recs: list, row: dict, take_control: tuple) -> dict:
  tr = row['track']
  a, z = row['start'], row['end']
  p_i = row['pass']
  turned = sum(recs[i]['yaw'] * DT for i in range(max(a, p_i - int(5 / DT)), z + 1)) * 180. / math.pi
  if row['type'] == 'turn' and 'slight' in row['label']:
    followed = True  # a slight turn's heading change is too small to tell from the trace
  elif row['type'] == 'turn':
    followed = abs(turned) >= FOLLOW_TURN_DEG and (turned > 0) == (row['side'] == 'left')
  else:
    followed = abs(turned) < FOLLOW_STRAIGHT_DEG
  if not followed:
    z = p_i  # the car left the route at the junction: nothing after it corresponds
  out = {'type': row['type'], 'side': row['side'], 'label': row['label'], 'kind': row['kind'], 't_start': recs[a]['t'],
             't_pass': recs[row['pass']]['t'], 't_end': recs[z]['t']}
  div_i, div_why = None, ''
  hold_run = go_run = deep_run = left_run = 0.
  hold_ok_until, stopped_sooner_s = -1, 0.
  stood_t = -1e9
  carried = recs[a]['prompt'] if recs[a]['prompt'] in take_control else None
  slow_s = stand_s = 0.
  fault_why, skip_why = '', ''
  first = {'desire': None, 'go': None, 'hold': None, 'sigp': None, 'offer': None}
  req, req_best = None, None  # the lane change request being held, and the row's longest (lane_request below)
  lc = Counter()
  alerts, chimes = Counter(), 0
  prev_names: set = set()
  prev_sigp = prev_ulc = prev_sgc = False
  light_status = Counter()
  light_acc, light_cam, uwhy, sreason, evs = Counter(), Counter(), Counter(), Counter(), Counter()
  stand_before_turn = []
  for i in range(a, z + 1):
    r = recs[i]
    d_row = tr['d'].get(i)
    dt = DT if i == a else max(min(r['t'] - recs[i - 1]['t'], 0.2), 0.)
    vp = plan_speed(r, d_row)
    if div_i is None and not r['sim_en']:
      hold_run = go_run = deep_run = left_run = 0.
      if not followed and r['prompt'] in DEVIATION_PROMPTS:
        div_i, div_why = i, 'the car left the route'
      elif carried is not None and r['prompt'] == carried:
        pass
      elif r['prompt'] in take_control or r['handback'] != 'none' or r['fault'] != 'none':
        fault_why = fault_why or (r['prompt'] if r['prompt'] in take_control else r['handback'] if r['handback'] != 'none'
                                  else r['fault'])
    elif div_i is None:
      # divergence first: what the planner does on the frame the car no longer follows it is open loop
      moving = r['v'] > DIVERGE_MOVING_V
      hold_run = hold_run + dt if moving and vp < HOLD_V else 0.
      stood_t = r['t'] if r['v'] < STOOD_V else stood_t
      pulling = r['v'] > LEFT_HOLD_V and r['t'] - stood_t < PULL_AWAY_S  # moving off from a standstill
      left_run = left_run + dt if pulling and vp < 0.3 else 0.
      deep_run = deep_run + dt if moving and vp < 0.5 * r['v'] else 0.
      stood_free = r['v'] < STOOD_V and r['lead'] is None
      go_run = go_run + dt if stood_free and acting(r) and not holding(r) else 0.
      if (hold_run >= DIVERGE_HOLD_S or left_run >= LEFT_HOLD_S) and i > hold_ok_until:
        stop_k = next((k for k in range(i, min(len(recs), i + int(STOP_MATCH_S / DT))) if recs[k]['v'] < STOOD_V), None)
        if stop_k is not None and left_run < LEFT_HOLD_S:
          hold_ok_until = stop_k  # the car stopped too, a little further on: the planner's stop is the car's, sooner
          stopped_sooner_s = max(stopped_sooner_s, (stop_k - i) * DT)
        else:
          div_i, div_why = i, 'planner holds where the car drove on'
      elif deep_run >= DEEP_SLOW_S:
        div_i, div_why = i, 'planner far slower than the car'
      elif go_run >= DIVERGE_GO_S:
        div_i, div_why = i, 'planner drives on where the car stood'
      if div_i is not None:
        continue
      # a lane change request held (the docstring's lane changes): its offer up, the request withheld LC_GAP_S at most
      if req is not None and (r['offer'] != req['side'] or r['t'] - req['last'] > LC_GAP_S):
        req['ended'] = r['abort'] if r['abort'] != 'none' else 'withheld' if r['offer'] == req['side'] else 'offer dropped'
        req = None
      if r['lc'] != 'none':
        if req is None:
          req = {'side': r['lc'], 't': r['t'], 'd': d_row, 'last': r['t'], 'run': 0, 'started': False, 'ended': 'row ended',
                 'of': abs(r['want'])}
          if req_best is None:
            req_best = req
        # DesireHelper starts a requested change on the request's second frame, moving, that side's blind spot clear
        req['run'] = req['run'] + 1 if r['v'] > LC_START_V and not (r['lbs'] if r['lc'] == 'left' else r['rbs']) else 0
        req['started'] = req['started'] or req['run'] >= 2
        req['last'] = r['t']
        if req['last'] - req['t'] > req_best['last'] - req_best['t']:
          req_best = req
        if req['last'] - req['t'] >= LC_COMPLETE_S:
          req['ended'] = 'held'
          div_i, div_why = i, 'lane change requested'
          continue
      # what the planner commanded
      if r['desire'] in (1, 2) and first['desire'] is None:
        first['desire'] = (r['t'], d_row)
      if (r['prompt'] in GO_PROMPTS or r['uls'] == 'go' or r['sgs'] == 'going') and first['go'] is None:
        first['go'] = (r['t'], d_row)
      if r['stop'] and first['hold'] is None:
        first['hold'] = (r['t'], d_row, r['mcd'])
      if r['stop'] and r['v'] < STOOD_V and d_row is not None and r['mck'] != 'none':
        stand_before_turn.append(d_row - r['mcd'])
      if r['sigp'] != 'none' and first['sigp'] is None:
        first['sigp'] = (r['t'], d_row)
      if r['offer'] != 'none':
        lc['offer_' + r['offer']] += 1
        if first['offer'] is None:
          first['offer'] = (r['t'], d_row)
      if r['lc'] != 'none':
        lc['request_' + r['lc']] += 1
      if r['abort'] != 'none':
        lc['abort_' + r['abort']] += 1
      if not followed and r['prompt'] in DEVIATION_PROMPTS:
        div_i, div_why = i, 'the car left the route'
        continue
      if carried is not None and r['prompt'] != carried:
        carried = None  # a take-control prompt carried in from the junction before is that junction's
      if (r['fault'] != 'none' or r['handback'] != 'none' or r['prompt'] in take_control or r['state'] == 'unavailable') and \
         not (carried is not None and r['prompt'] == carried):
        fault_why = fault_why or (r['fault'] if r['fault'] != 'none' else r['handback'] if r['handback'] != 'none' else
                                  r['prompt'] if r['prompt'] in take_control else 'unavailable')
      if r['fast'] and (d_row is None or abs(r['md'] - d_row) < 40.):
        skip_why = skip_why or (r['skip'] or 'tooFastToTurn')
      names = set(r['names'])
      key_names = {f'{n}:{r["prompt"] if "Prompt" in n else r["alert"] if "Alert" in n else ""}'.rstrip(':') for n in names}
      for n in key_names - prev_names:
        alerts[n] += 1
        if not (n.endswith(HOLD_PROMPTS) or n.endswith('mapControlsUnavailableSilent')):
          chimes += 1
      prev_names = key_names
      if r['sigp'] != 'none' and not prev_sigp:
        alerts['signalPrompt:' + r['sigp']] += 1
        chimes += 1
      prev_sigp = r['sigp'] != 'none'
      chimes += (r['ulchime'] and not prev_ulc) + (r['sgchime'] and not prev_sgc)
      prev_ulc, prev_sgc = r['ulchime'], r['sgchime']
      if r['mcs'] != 'none' and r['mcl'] != 'unknown':
        light_acc[f'{r["mcs"]}:{r["mcl"]}'] += 1
      if r['cam'] is not None:
        light_cam[r['cam']] += 1
      if r.get('camst'):
        light_status[r['camst']] += 1
      if r['ulwhy']:
        uwhy[r['ulwhy']] += 1
      if r['sgr']:
        sreason[r['sgr']] += 1
      for n, kw in r['ev']:
        evs[n + ('(' + ','.join(f'{k}={clean(v, 20)}' for k, v in sorted(kw.items())) + ')' if kw else '')] += 1
      # needless slowing against the car's trace
      if r['v'] > MOVING_V and vp < r['v'] - SLOW_MARGIN:
        slow_s += dt
      if holding(r) and r['v'] > MOVING_V:
        stand_s += dt
  # what the driver did (whole window: the trace is real)
  vs = [recs[i]['v'] for i in range(a, z + 1)]
  stood = [i for i in range(a, z + 1) if recs[i]['v'] < STOOD_V]
  en = [recs[i]['en'] for i in range(a, z + 1)]
  takeover = next((recs[i]['t'] for i in range(a + 1, z + 1) if recs[i - 1]['en'] and not recs[i]['en']), None)
  pedal = next((recs[i]['t'] for i in range(a, z + 1) if recs[i]['en'] and (recs[i]['gas'] or recs[i]['brake'])), None)
  stood_s = len(stood) * DT
  # verdict
  div_r = recs[div_i] if div_i is not None else None
  stop_rail = False
  red_evidence = ''
  if div_r is not None and div_why.startswith('planner drives on') and row['kind'] == 'signal':
    near = Counter(recs[i]['cam'] for i in range(max(a, div_i - int(RED_LOOK_S / DT)), div_i + 1))
    if near['red'] + near['yellow'] >= RED_MIN_READS and near['red'] + near['yellow'] > near['green']:
      red_evidence = 'camera'
    else:
      j = div_i  # the stand the divergence is in: the driver's own (disengaged), first in line at the signal's line
      while j > a and recs[j - 1]['v'] < STOOD_V:
        j -= 1
      own = all(not recs[i]['en'] for i in range(j, div_i + 1))
      at_line = any(DRIVER_LINE_M[0] < c['d'].get(div_i, -99.) < DRIVER_LINE_M[1] for c in row['controls'] if c['kind'] == 'signal')
      if own and at_line and (div_i - j) * DT >= DRIVER_STAND_S:
        red_evidence = 'driver stood at the line'
  check_light = False
  if div_r is not None and div_why.startswith('planner drives on') and row['kind'] == 'signal':
    j, k = div_i, div_i  # the car's stand the planner drove on in, whole
    while j > a and recs[j - 1]['v'] < STOOD_V:
      j -= 1
    while k < z and recs[k + 1]['v'] < STOOD_V:
      k += 1
    check_light = sum(DT for i in range(j, k + 1) if recs[i]['lead'] is None) >= CHECK_LIGHT_S
  if fault_why:
    verdict = 'faulted'
  elif skip_why:
    verdict = 'skipped'
  elif div_why == 'lane change requested':
    verdict = 'would-drive'
  elif red_evidence:
    verdict = 'would-run-red'
  elif div_r is not None and not div_why.startswith('planner drives on'):
    car_stopped_before = any(recs[i]['v'] < STOOD_V for i in range(a, div_i))
    if div_r['mck'] == 'stop' and div_r['mcd'] < STOP_RAIL_M and row['kind'] == 'stop':
      verdict = 'would-drive'  # the full stop at a stop sign (a rail) where the car rolled it; its go is past divergence
      stop_rail = True
    else:
      verdict = 'would-stand' if car_stopped_before and holding(div_r) else 'would-stop-wrongly' if holding(div_r) else \
        'would-drive'
  else:
    verdict = 'would-drive'
  held_after = 0.
  if div_i is not None and div_why.startswith('planner holds'):
    held_after = sum(DT for i in range(div_i, z + 1) if holding(recs[i]))
  def rel(x):
    return None if x is None else (round(x[0] - out['t_pass'], 1), None if x[1] is None else round(x[1], 1))
  out.update(
    verdict=verdict, fault=fault_why, skip=skip_why, stop_sign_rail=stop_rail, red_evidence=red_evidence, driver_followed=followed,
    at_divergence=None if div_r is None else {'control': div_r['mck'], 'action': div_r['mca'], 'stop_m': round(div_r['mcd'], 1),
                                                  'light': div_r['mcl'], 'prompt': div_r['prompt'], 'left': div_r['uls'],
                                                  'stop_go': div_r['sgs'], 'cap': round(div_r['cap'], 1), 'car_v': round(div_r['v'], 1),
                                                  'engaged': div_r['en']},
    diverged_at=None if div_i is None else round(recs[div_i]['t'] - out['t_pass'], 1), diverged_why=div_why,
    went_before_car=div_why.startswith('planner drives on') and verdict == 'would-drive',
    check_light=check_light and div_why.startswith('planner drives on') and verdict == 'would-drive',
    lane_request=None if req_best is None else {
      'side': req_best['side'], 'at': rel((req_best['t'], req_best['d'])), 'held_s': round(req_best['last'] - req_best['t'], 1),
      'of_changes': req_best['of'], 'desire_helper_would_start': req_best['started'], 'ended': req_best['ended']},
    stopped_sooner_s=round(stopped_sooner_s, 1), needless_slow_s=round(slow_s, 1), needless_stand_s=round(stand_s, 1),
    held_past_divergence_s=round(held_after, 1),
    desire=rel(first['desire']), go=rel(first['go']), hold=rel(first['hold'][:2]) if first['hold'] else None,
    hold_stop_m=round(first['hold'][2], 1) if first['hold'] else None,
    stand_before_turn_m=round(sorted(stand_before_turn)[len(stand_before_turn) // 2], 1) if stand_before_turn else None,
    signal_prompt=rel(first['sigp']), lane_change_offer=rel(first['offer']), lane_change=dict(lc),
    alerts=dict(alerts), chimes=int(chimes), light_accepted=dict(light_acc), light_camera_raw=dict(light_cam), light_status=dict(light_status),
    left_why=dict(uwhy.most_common(4)), stop_go_reason=dict(sreason.most_common(4)), events=dict(evs.most_common(8)),
    driver={'min_v': round(min(vs), 1), 'stood_s': round(stood_s, 1), 'engaged_share': round(sum(en) / len(en), 2),
                'takeover_at': None if takeover is None else round(takeover - out['t_pass'], 1),
                'pedal_at': None if pedal is None else round(pedal - out['t_pass'], 1), 'turned_deg': round(turned)},
    min_plan_cap=round(min(recs[i]['cap'] for i in range(a, (div_i or z) + 1)), 1),
  )
  return out


def drive_totals(recs: list, rows: list, logged_engagement: bool, takeovers: list) -> dict:
  moving = [i for i, r in enumerate(recs) if r['v'] > MOVING_V]
  takeover = set()
  for row in rows:
    if row['verdict'] in BAD:
      t0 = row['t_pass'] + (row['diverged_at'] if row['diverged_at'] is not None else -(row['t_pass'] - row['t_start']))
      t1 = row['t_end'] + TAKEOVER_TAIL_S
      takeover.update(i for i in moving if t0 <= recs[i]['t'] <= t1)
  est = sum(1 for i in moving if recs[i]['sim_en'] and acting(recs[i]) and i not in takeover)
  paused = sum(DT for r in recs if r['state'] == 'unavailable' or r['fault'] != 'none' or r['handback'] != 'none')
  counts = Counter(r['verdict'] for r in rows)
  alerts = Counter()
  for row in rows:
    alerts.update(row['alerts'])
  return {
    'simulated_takeovers': len(takeovers), 'takeover_causes': dict(Counter(c for _, c in takeovers)),
    'rows': len(rows), 'verdicts': {v: counts.get(v, 0) for v in VERDICTS}, 'alerts_in_rows': sum(alerts.values()),
    'chimes_in_rows': sum(r['chimes'] for r in rows), 'alert_kinds': dict(alerts.most_common(12)), 'paused_or_faulted_s': round(paused, 1),
    'needless_slow_s': round(sum(r['needless_slow_s'] for r in rows), 1),
    'needless_stand_s': round(sum(r['needless_stand_s'] for r in rows), 1),
    'held_past_divergence_s': round(sum(r['held_past_divergence_s'] for r in rows), 1),
    'moving_s': round(len(moving) * DT, 1), 'estimated_engaged_share': round(est / len(moving), 2) if moving else None,
    'logged_engaged_share': round(sum(1 for i in moving if recs[i]['en']) / len(moving), 2) if moving else None,
    'drive_s': round(recs[-1]['t'] - recs[0]['t'], 1) if recs else 0.}


def badness(row: dict) -> float:
  w = {'would-run-red': 100, 'faulted': 60, 'would-stop-wrongly': 50, 'would-stand': 40, 'skipped': 20, 'would-drive': 0}
  return w[row['verdict']] + row['needless_stand_s'] * 2 + row['needless_slow_s'] + row['held_past_divergence_s'] + row['chimes']


# ---------------------------------------------------------------- output

HEADER = """Open-loop replay: the logged car did what the logged build and its driver did, not what this planner commands. Each
maneuver is scored only up to its divergence: the first frame where the planner would have stopped or held the car while it
drove on (>{hv:.0f} m/s for {hs} s), been under half the car's speed for {ds} s, or driven on while it stood free of a lead
for {gs} s. Later frames of that maneuver no longer correspond and are not scored (held_past_divergence_s is indicative only).
A lane change the planner requests is never seen to happen (the logged car changes lanes only where its driver did): the
simulated driver signals {sr:.0f} s after a prompt to, and a request then held {lc:.0f} s (a change's length; withheld frames of
up to {lg} s with the offer up don't end it) is the row's divergence, would-drive (lane change requested). The lane change
column says how long the longest request was held and what ended it; a shorter one changes no verdict.
Planner replayed {eng}. Verdicts: would-drive (no divergence, or the planner went where the old build stood:
went_before_car; "check the light" where the car had stood {cl:.0f} s or more with no lead at a signalised junction, so an unread red is possible),
would-stand (the car stood, then drove on while the planner still held), would-stop-wrongly (the planner holds where the car
never stopped), would-run-red (the planner drives on where a camera red/yellow was read or the driver stood short of the
signal's line), skipped (too fast or too late: rerouted), faulted (fault, handback, paused or a take-control prompt before
divergence). Needless slow/stand: seconds before divergence the planner was >{sm} m/s under the car's speed / holding while it
moved. Estimated engaged share: moving time with the planner acting, less each bad row from its divergence to its end
+{tt:.0f} s (the driver taking it). Times are seconds from the drive's first plan; per-row times are relative to the junction."""


def fmt_t(s: float) -> str:
  return f'{int(s // 60):02d}:{s % 60:04.1f}'


def write_outputs(prefix: str, meta: dict, routes: list[dict]) -> None:
  allrows = [r for rt in routes for r in rt['rows']]
  worst = sorted(allrows, key=badness, reverse=True)[:10]
  doc = {'meta': meta, 'routes': routes, 'worst': [{'route': r['route'], 'index': r['index']} for r in worst]}
  with open(prefix + '.json', 'w') as f:
    json.dump(doc, f, indent=1, default=str)
  L = [f'# TurnPilot drive scorecard: {meta["label"]}', '',
       HEADER.format(sr=SIGNAL_REACT_S, lc=LC_COMPLETE_S, lg=LC_GAP_S, cl=CHECK_LIGHT_S, hv=DIVERGE_MOVING_V, hs=DIVERGE_HOLD_S,
                     ds=DEEP_SLOW_S, gs=DIVERGE_GO_S, sm=SLOW_MARGIN, tt=TAKEOVER_TAIL_S,
                     eng='engaged throughout, pedals and steering override masked' if not meta['logged_engagement']
                     else "with the log's own engagement and pedals"), '']
  for rt in routes:
    T = rt['totals']
    L += [f'## Route {rt["route"]}', '',
          f'- drive {fmt_t(T["drive_s"])}, moving {T["moving_s"]:.0f} s; rows {T["rows"]}: ' +
          ', '.join(f'{k} {v}' for k, v in T['verdicts'].items()),
          f'- estimated engaged share {T["estimated_engaged_share"]} (logged {T["logged_engaged_share"]}); paused or faulted ' +
          f'{T["paused_or_faulted_s"]} s', f'- alerts {T["alerts_in_rows"]}, chimes {T["chimes_in_rows"]}: ' +
          ', '.join(f'{k} {v}' for k, v in T['alert_kinds'].items()),
          f'- needless slowing {T["needless_slow_s"]} s, needless standing {T["needless_stand_s"]} s, held past divergence ' +
          f'{T["held_past_divergence_s"]} s (indicative)']
    if rt.get('agreement'):
      L.append('- agreement with the logged plans (frames that differ): ' +
               ', '.join(f'{k} {v / rt["agreement_n"]:.1%}' for k, v in rt['agreement'].items()) + f' of {rt["agreement_n"]}')
    if rt.get('errors'):
      L.append('- planner errors: ' + ', '.join(f'{k} x{v}' for k, v in rt['errors'].items()))
    L += ['', '| # | time | maneuver | verdict | diverged | plan | lane change | driver | alerts |', '|---|---|---|---|---|---|---|---|---|']
    for r in rt['rows']:
      plan = []
      if r['hold']:
        plan.append(f'hold {r["hold"][0]}s stop {r["hold_stop_m"]} m')
      if r['stand_before_turn_m'] is not None:
        plan.append(f'stands {r["stand_before_turn_m"]} m before turn pt')
      if r['go']:
        plan.append(f'go {r["go"][0]}s')
      if r['desire']:
        plan.append(f'desire {r["desire"][0]}s@{r["desire"][1]} m')
      plan.append(f'cap>={r["min_plan_cap"]}')
      if r['needless_slow_s'] or r['needless_stand_s']:
        plan.append(f'slow {r["needless_slow_s"]}s stand {r["needless_stand_s"]}s')
      if r['fault']:
        plan.append('fault ' + r['fault'])
      if r['skip']:
        plan.append('skip ' + r['skip'])
      lcs = ' '.join(f'{k}' for k in r['lane_change']) or '-'
      lr = r['lane_request']
      if lr is not None:
        first_of = f' (1 of {lr["of_changes"]})' if lr['of_changes'] > 1 else ''
        lcs += f'; request {lr["side"]}{first_of} at {lr["at"][1]} m held {lr["held_s"]}s, ended: {lr["ended"]}'
      why = ' (lane change requested)' if r['diverged_why'] == 'lane change requested' and r['verdict'] == 'would-drive' else \
        ' (went before car: check the light)' if r['check_light'] else ' (went before car)' if r['went_before_car'] else ''
      dr = r['driver']
      drv = f'vmin {dr["min_v"]}, stood {dr["stood_s"]}s, eng {dr["engaged_share"]:.0%}, turn {dr["turned_deg"]}°' + \
        (f', takeover {dr["takeover_at"]}s' if dr['takeover_at'] is not None else '')
      div = '-' if r['diverged_at'] is None else f'{r["diverged_at"]}s: {r["diverged_why"]}'
      L.append(f'| {r["index"]} | {fmt_t(r["t_pass_rel"])} | {r["label"]} @{r["kind"]} | {r["verdict"]}' +
               f'{why} | {div} | {"; ".join(plan)} | {lcs} | {drv} | ' +
               f'{sum(r["alerts"].values())}/{r["chimes"]} |')
    L.append('')
  L += ['## Ten worst rows', '']
  for r in worst:
    L.append(f'- {r["route"]} #{r["index"]} {fmt_t(r["t_pass_rel"])} {r["label"]} @{r["kind"]}: {r["verdict"]}' +
             f'{" (" + r["fault"] + ")" if r["fault"] else ""}{" (" + r["skip"] + ")" if r["skip"] else ""}; ' +
             f'{r["diverged_why"] or "no divergence"}; stand {r["needless_stand_s"]}s slow {r["needless_slow_s"]}s, held past ' +
             f'{r["held_past_divergence_s"]}s; left_why {list(r["left_why"])[:2]}; sg {list(r["stop_go_reason"])[:2]}; ' +
             f'lights acc {r["light_accepted"]} cam {r["light_camera_raw"]}; lc {r["lane_change"]}; alerts {r["alerts"]}')
  with open(prefix + '.md', 'w') as f:
    f.write('\n'.join(L) + '\n')


# ---------------------------------------------------------------- main

def score_route(td, hooks, route: str, args) -> dict:
  data = load_route(route, args.logs_dir, args.cache_dir)
  frames = build_frames(data['msgs'])
  h = Harness(td, hooks, frames, data['params'], data['wall_off'], logged_engagement=args.logged_engagement)
  t0 = time.perf_counter()
  recs = h.run()
  wall = time.perf_counter() - t0
  if args.save_records:
    with open(f'{args.save_records}_{route.split("--", maxsplit=1)[0]}.pkl', 'wb') as f:
      pickle.dump(recs, f, protocol=pickle.HIGHEST_PROTOCOL)
  rows_raw = find_maneuvers(recs)
  take_control = tuple(getattr(hooks, 'TAKE_CONTROL_PROMPTS', ()))
  rows = []
  for k, row in enumerate(rows_raw):
    s = score_row(recs, row, take_control)
    s['index'], s['route'] = k, route.split('--', maxsplit=1)[0]
    s['t_pass_rel'] = s['t_pass'] - recs[0]['t']
    for key in ('t_start', 't_pass', 't_end'):
      s[key] = round(s[key], 2)
    rows.append(s)
  totals = drive_totals(recs, rows, args.logged_engagement, h.sim_takeovers)
  for s in rows:
    for key in ('t_start', 't_pass', 't_end'):
      s.pop(key)
  return {'route': route.split('--', maxsplit=1)[0], 'frames': len(frames), 'replay_s': round(wall, 1), 'rows': rows, 'totals': totals,
              'agreement': dict(h.agreement), 'agreement_n': h.agreement_n, 'errors': dict(h.errors.most_common(8)),
              'first_tracebacks': h.first_tb if args.tracebacks else {}}


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
  ap.add_argument('routes', nargs='+', help='route names (counter--hash), scored one after another as one drive report')
  ap.add_argument('--pkg-root', help='import openpilot.selfdrive.turnpilot from DIR/selfdrive/turnpilot')
  ap.add_argument('--logs-dir', default=os.path.expanduser('~/turnpilot_data/logs'))
  ap.add_argument('--cache-dir', default=os.environ.get('TP_SCORECARD_CACHE', os.path.expanduser('~/.cache/tp_scorecard')))
  ap.add_argument('--out', required=True, help='output prefix: PREFIX.md and PREFIX.json')
  ap.add_argument('--label', help='what is scored, for the header (default: the package root\'s folder name, or tree)')
  ap.add_argument('--logged-engagement', action='store_true', help="replay the log's own engagement and pedals")
  ap.add_argument('--agreement', action='store_true', help='report frame agreement with the logged plans')
  ap.add_argument('--save-records', help='pickle the per-frame records to PREFIX_<route>.pkl (for debugging the score)')
  ap.add_argument('--tracebacks', action='store_true', help='keep the first traceback of each planner error in the JSON')
  args = ap.parse_args(argv)
  td, hooks = load_package(args.pkg_root)
  label = args.label or (os.path.basename(os.path.normpath(args.pkg_root)) if args.pkg_root else 'working tree')
  routes = []
  for route in args.routes:
    rt = score_route(td, hooks, route, args)
    if not args.agreement:
      rt['agreement'] = {}
    routes.append(rt)
  meta = {'label': label, 'logged_engagement': args.logged_engagement, 'package': td.__file__.rsplit('/selfdrive/', 1)[-1],
              'pkg_root_given': bool(args.pkg_root), 'cache_version': CACHE_VERSION,
              'thresholds': {'DIVERGE_HOLD_S': DIVERGE_HOLD_S, 'DIVERGE_GO_S': DIVERGE_GO_S, 'DEEP_SLOW_S': DEEP_SLOW_S,
                              'SLOW_MARGIN': SLOW_MARGIN, 'LC_COMPLETE_S': LC_COMPLETE_S, 'LC_GAP_S': LC_GAP_S, 'SIGNAL_REACT_S': SIGNAL_REACT_S,
                              'CHECK_LIGHT_S': CHECK_LIGHT_S, 'LEAD_STOP_M': LEAD_STOP_M, 'TAKEOVER_TAIL_S': TAKEOVER_TAIL_S}}
  write_outputs(args.out, meta, routes)
  for rt in routes:
    print(rt['route'], rt['frames'], 'frames', rt['replay_s'], 's', rt['totals']['verdicts'], 'est', rt['totals']['estimated_engaged_share'],
          'logged', rt['totals']['logged_engaged_share'])


if __name__ == '__main__':
  main()
