"""SAT_VALIDATION row 11 (BeamNG, 263065a, stop_allway_out_of_turn under TurnPilotStopSignGo=vision): CONTACT. We arrived first
at an all-way stop, a car from the left stood 4.4 m back of its assumed line (16.7 m down its lane), our go came at 16.90 s
with every lane clear, the car started across 0.5 s later, we committed at 18.10 s and the cars touched at about 21 s.

Why (tests/sat11_out_of_turn.json replays the decider on the run's own frames, as turnpilotd fed it the sim's cross oracle):
the car's front was outside its line region (front_from_line -4.4 m), so it was never stopped at its line, never a shield and
never in the all-way order; standing with a trusted 0.08 m/s it read U2 'far', and _known_moved judged a 'far' car known at the
go only by its go-time distance less its go-time speed carried on, never by where it is now. So the launch's re-check never
saw it move (no outOfTurn), and the band's threat test skipped it as still (no bandAbort); nothing else of TurnPilot's brakes
for a vehicle, and the model drove into it.

The fix, under vision: every vehicle standing at the go is watched for its start by its ground point (ego motion and our yaw
removed) and its ground speed toward our path, without waiting for its speed's trust (StopGo._watch_starts); a start before
the commit withdraws the go (the capped abort, the window again), after it the band's threat test sees it."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_go as sg
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table
from openpilot.selfdrive.turnpilot.junction_watch import junction_from_info
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.stop_evidence import CAR_HALF_LEN_M
from openpilot.selfdrive.turnpilot.tests.test_stop_go import first_go
from openpilot.selfdrive.turnpilot.tests.test_stop_go_vision import LINE_V, vcars, vdrive, vlanes
from openpilot.selfdrive.turnpilot.turnpilotd import ORACLE_CLASS, ORACLE_ID, SIM_VISIBILITY
from openpilot.selfdrive.turnpilot.visibility import OracleLive

FIXTURE = Path(__file__).parent / 'sat11_out_of_turn.json'
RUN11_D = LINE_V + 4.4   # m down its lane run 11's car stood (its front 4.4 m back of DEFAULT_LINE_M: front_from_line -4.4)
CAR_HALF_W = 0.93        # m, half a Model 3's width (ours, and the crossing car's half depth along our axis)
SETBACK = 1.9


# ------------------------------------------------------------------------------------------------- run 11, replayed

def replay(frames=None, until=None):
  """The decider on run 11's frames (tests/sat11_out_of_turn.json), open loop: each frame's Frame as turnpilotd built it under
  the sim's cross oracle (cross_oracle: its tracks, objects, every view's run, OracleLive), the held control mocked from the
  logged stopGo.positionM until the latch's release, G5's go from the run's go_override events. Returns [(t, decision)]."""
  data = json.loads(FIXTURE.read_text())
  info = data['junctionInfo']
  j = junction_from_info('stop', info)
  vis = visibility_table.cached(SIM_VISIBILITY, sim_ok=True)
  token = SimpleNamespace(moved_m=0., cause='', done=False)
  ctl = SimpleNamespace(key=(2, 0), kind='stop', d=30., held=False, state=token, junction_info=info, fix_spread=0.)
  dec = sg.StopGo()
  odo, t_or = 0., 0.
  released = False
  out = []
  for fr in data['frames'] if frames is None else frames:
    t = fr['t']
    if until is not None and t > until:
      break
    v = max(fr['v'], 0.)
    odo += v * DT_MDL
    t_or += DT_MDL
    tracks, objs = [], []
    for cid, x, y, vx, vy in fr['cars']:
      r = max(math.hypot(x, y), 0.5)
      b = math.degrees(math.atan2(y, x))
      half = math.degrees(math.atan2(0.9, r))
      view = 'full' if abs(b) < 40. else 'sideLeft' if b > 0. else 'sideRight'
      tracks.append(SimpleNamespace(track_id=ORACLE_ID + cid, t_seen=t_or, bearing_seen=b, width_seen=1.8 * 904. / r, view=view,
                                    cls=ORACLE_CLASS['car'], x=np.array([x, y, vx, vy]), span_seen=(b - half, b + half), hits=5))
      objs.append(SimpleNamespace(trackId=ORACLE_ID + cid, cls='car', x=x, y=y, vx=vx, vy=vy, age=10, conf=1.))
    runs = tuple((k, t_or, RawFrame([], 'road' if k == 'road' else 'wide', t_or, k if k.startswith('side') else 'full'), None)
                 for k in ('road', 'full', 'sideLeft', 'sideRight'))
    state = fr['state']
    if state == 'stopped':
      ctl.held = True
    if state in ('approach', 'stopped', 'observe', 'rollUp') and not released:
      ctl.d = -fr['pos']
    taken = t >= data['lift_end_t']
    f = sg.Frame(control=ctl if ctl.held and not released else None, approach=ctl if not ctl.held and not released else None,
                 junction=j, route_cert=str(info.get('certification', 'unchecked')), objects=tuple(objs), tracks=tuple(tracks), runs=runs,
                 live=OracleLive(fr['range']), vis=vis, wall_t=1793466000., side_status='running', v_ego=v, a_ego=fr['a'], odo_m=odo,
                 lead_m=fr['lead'], t_abort=sr.t_abort(data['cp_delay'], vis.actuator_delay_s),
                 go_active='stopSign' if data['go_t'] - 1e-3 <= t < data['lift_end_t'] else 'none', go_taken_over=taken,
                 go_end='takeover' if taken else '', mode='vision', always_drive=True)
    d = dec.update(f)
    released = released or d.release is not None
    out.append((t, fr, d))
  return out


def test_run11_replay_reproduces_the_logged_go_at_16_9_s():
  # the harness itself: up to the go the replay's states are the run's own, frame for frame (approach, stopped, observe, going)
  out = replay(until=16.95)
  assert [fr['state'] for _, fr, _ in out] == [d.state for _, _, d in out]
  assert out[-1][2].state == 'going' and abs(out[-1][0] - 16.95) < 0.03


def test_run11_the_car_from_the_left_starting_withdraws_the_go_before_the_logged_commit():
  # the run's own frames: the oracle car (id 1) first moves at 17.48 s (vy -0.10), -1.03 m/s at 17.73, -1.51 at 17.98; the run
  # committed at 18.10. The go is withdrawn for it on the run's frames before then, where the capped abort still ends short of
  # dEdgeMin, and the frames before it started (to 17.45, our own launch under way from 17.24) never withdraw it
  out = replay(until=18.10)
  wd = [(t, fr, d) for t, fr, d in out if d.withdraw]
  assert wd, [(t, d.state, [h.key for h in d.holds]) for t, _, d in out if t > 16.85]
  t, fr, d = wd[0]
  assert 17.48 <= t < 18.10 and d.withdraw.startswith('outOfTurn:1000001'), (t, d.withdraw)
  assert "from the left" in d.reason and d.prompt == 'waitingForCross'
  assert all(s.accel_cap >= -2. - 1e-9 for s in d.stops) and d.stops[0].rest
  # withdrawn while the run itself was still short of its commit point (the capped abort ends short of dEdgeMin there)
  assert fr['state'] == 'going' and d.state == 'observe'


# ------------------------------------------------------------------------- the timeline in the closed loop (Drive)

def world(tmp_path, where):
  """vdrive at 25 mph (run 11's tagged 11.1 m/s). where 'back': run 11's car 4.4 m back of its assumed line, with the sim's
  cross oracle for the live view (OracleLive: every lane seen to 60 m, as run 11's decider read it; through the cameras that car
  is an occluder there, the static wait and H3); 'line': a later arrival stopping at its line just after our full stop (the
  shield, our turn first), through the cameras' live view."""
  w = vdrive(tmp_path, 25, behaviour='goes')
  if where == 'back':
    w.kw['live'] = OracleLive(60.)
  return w


def standing_d(where):
  """d(t) down its lane of the car standing there: run 11's 16.7 m throughout, or a later arrival at its line (test_stop_go_vision's
  shield test: at its line 1.2 s after the start, our full stop at 1.0 s)."""
  if where == 'back':
    return lambda t: RUN11_D
  return lambda t: None if t < -1.8 else max(LINE_V + 18. - 6. * (t + 1.8), LINE_V)


def start(d0, a=2.0, v_max=6.):
  """d(t) of run 11's car: standing d0 m down its lane until t_s (set by the test: holder['t_s']), then off toward our path at
  a, up to v_max (run 11: about 2 m/s^2 to 7 m/s by our contact), through our path and on."""
  holder = {'t_s': None}
  d_stand = d0 if callable(d0) else (lambda t: d0)

  def d(t):
    if holder['t_s'] is None or t < holder['t_s']:
      return d_stand(t)
    d_s = d_stand(holder['t_s'])
    tt = t - holder['t_s']
    tv = v_max / a
    gone = 0.5 * a * min(tt, tv) ** 2 + v_max * max(tt - tv, 0.)
    return d_s - gone
  return d, holder


def scene_car(tid, role, d_fn, w, frac=0.5, age=20, conf=0.9, v_fn=None):
  """The scene's object for a scripted lane car (scene_policy's TrackedObject fields): its ground point where vcars puts its
  track (x m ahead of the camera, y left), its velocity toward our path, its hits."""
  t = w.t_cam - 100.
  dd = d_fn(t)
  if dd is None or dd < -12.:
    return None
  pos = w.sg.pos_ref + (w.odo - w.sg.odo_ref) if w.sg.token is not None else -w.control.d
  ln = vlanes(w.j)[role]
  s = ln.s_min + frac * (ln.s_max - ln.s_min) - pos
  side = 1. if role == 'left' else -1.
  vy = -side * (v_fn(t) if v_fn is not None else (d_fn(t - DT_MDL) - dd) / DT_MDL)
  return SimpleNamespace(trackId=tid, cls='car', x=s, y=side * dd, vx=0., vy=vy, age=age, conf=conf, inPath=False)


def lane_axis_m(w, role, frac=0.5):
  """m past the stop point the scripted car's lane crosses our axis (its ground point along our path)."""
  ln = vlanes(w.j)[role]
  return ln.s_min + frac * (ln.s_max - ln.s_min) - SETBACK


def touching(pos, car_d, axis_m):
  """Our body ([pos - 4.7, pos] m past the stop point, half-width CAR_HALF_W) and the crossing car's (its centre car_d m down
  its lane, CAR_HALF_LEN_M either side, CAR_HALF_W either side of axis_m) overlap."""
  along = pos >= axis_m - CAR_HALF_W and pos - sg.CAR_LEN_M <= axis_m + CAR_HALF_W
  across = abs(car_d) <= CAR_HALF_LEN_M + CAR_HALF_W
  return along and across


def drive_with(w, d_fn, holder, trigger, seconds, role='left', objects=True):
  """Step w; holder['t_s'] set (the car starts) on the first frame trigger(d) holds; the car's track (vcars) and, with objects,
  its scene object every frame. Returns (decisions, [(pos, car d)])."""
  w.tracks = vcars((5, role, d_fn))
  if objects:
    w.objects = lambda ww: [o for o in (scene_car(5, role, d_fn, ww),) if o is not None]
  out, geo = [], []
  for _ in range(round(seconds / DT_MDL)):
    d = w.step()
    out.append(d)
    geo.append((d.position_m, d_fn(w.t_cam - 100.)))
    if holder['t_s'] is None and trigger(d):
      holder['t_s'] = w.t_cam - 100. + DT_MDL
  return out, geo


@pytest.mark.parametrize('where', ['back', 'line'])
@pytest.mark.parametrize('after_s', [0.3, 0.5, 0.8])
def test_the_out_of_turn_start_withdraws_the_go_before_commit_then_the_window_again(tmp_path, after_s, where):
  # run 11's car: standing 16.7 m down the left lane through our full stop and the window, off at 2 m/s^2 after_s after our go
  # (run 11: 0.5 s). The go is withdrawn before the commit point (its start seen: outOfTurn), every decider stop -2.0 or
  # gentler, the car at rest short of dEdgeMin and never touching it. A later arrival at its line (the shield, through the
  # cameras) going out of turn the same way is withdrawn before commit too (its start, or its lane's reach unshielded)
  w = world(tmp_path, where)
  d_fn, holder = start(standing_d(where))
  t_go = {}

  def trigger(d):
    if d.state == 'going' and 'go' not in t_go:
      t_go['go'] = w.t_cam - 100.
    return 'go' in t_go and w.t_cam - 100. >= t_go['go'] + after_s - 1e-6
  out, geo = drive_with(w, d_fn, holder, trigger, 30.)
  i = first_go(out)
  assert i is not None
  wd = next((k for k, d in enumerate(out) if d.withdraw), None)
  assert wd is not None, [(d.state, d.withdraw) for d in out[i:i + 40]]
  assert out[wd].withdraw.startswith(('outOfTurn:5',) if where == 'back' else ('outOfTurn:5', 'reach:left1'))
  assert not any(d.state == 'committed' for d in out[:wd]) and out[wd].reason.startswith("Stopping short:")
  assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)
  assert not any(touching(p, cd, lane_axis_m(w, 'left')) for p, cd in geo)
  nxt = next((k for k in range(wd + 1, len(out)) if out[k].go_cause == 'stopSign'), None)
  assert max(p for p, _ in geo[:nxt]) <= w.sg.edges.d_min + 1e-6
  if where != 'back':
    return
  room = w.sg.edges.d_min - geo[nxt - 1 if nxt else -1][0]
  if after_s <= 0.5:
    # started early enough that the abort leaves GO_ROOM_M (run 11's 0.5 s): the window again from the withdrawal, the next
    # go only once the car has crossed our path, and we cross after it, never a handback
    assert nxt is not None and (nxt - wd) * DT_MDL >= sr.VISION_CLEAR_S - 1e-6 and room >= sr.go_room_m(w.kw['t_abort']) - 1e-6
    assert geo[nxt][1] < -CAR_HALF_LEN_M
    assert any(d.state == 'done' for d in out[nxt:]) and not any(d.fault for d in out)
  else:
    # later, the abort ends with less than GO_ROOM_M to dEdgeMin: no go from there (2.6), the loud handback (H5), held
    assert nxt is None and out[-1].fault == 'H5' and out[-1].prompt == 'stopSignTakeControl'
    assert all(w.cmds[k][1] < 0.05 for k in range(len(out) - 20, len(out)))


@pytest.mark.parametrize('where', ['back', 'line'])
def test_a_car_entering_our_path_after_the_commit_stops_us_short_of_it(tmp_path, where):
  # run 11's car off the frame we commit: the band's threat test sees it now (bandAbort, short of the band's midpoint), the
  # car comes to rest short of it, never touching it
  w = world(tmp_path, where)
  d_fn, holder = start(standing_d(where))
  out, geo = drive_with(w, d_fn, holder, lambda d: d.state == 'committed', 20.)
  k = next(k for k, d in enumerate(out) if d.state == 'committed')
  after = out[k:]
  assert any(d.withdraw == 'bandAbort' for d in after), \
    [(d.state, d.withdraw, d.alert) for d in after[:20]]
  assert not any(touching(p, cd, lane_axis_m(w, 'left')) for p, cd in geo)
  # at rest before it reached our path, short of where its lane crosses our axis
  meet = next(n for n, (_, cd) in enumerate(geo) if cd <= CAR_HALF_LEN_M + CAR_HALF_W)
  assert w.cmds[meet][1] < 0.3 and geo[meet][0] < lane_axis_m(w, 'left') - CAR_HALF_W


# --------------------------------------------------------------------------------------- what must not change

@pytest.mark.parametrize('role', ['left', 'right'])
@pytest.mark.parametrize('where', ['back', 'line'])
@pytest.mark.parametrize('jitter', [0., 0.2])
def test_a_car_that_stays_standing_lets_us_go_after_the_window_and_our_launch_never_withdraws_for_it(tmp_path, role, where, jitter):
  # all-way order: run 11's car 4.4 m back of its line (no order: never at its line), or one stopping at its line just after
  # our full stop (a later arrival: our turn first); it stays there. We go after the window, cross and are done; our own launch
  # (its bearings swing, its trust restarts) never reads as its start, with its ground point jittering by the run's camera
  # noise (0.2 m standing in run 11) or without
  w = world(tmp_path, where)
  base = vcars((6, role, standing_d(where)))
  rng = np.random.default_rng(7)

  def tracks(ww):
    out = base(ww)
    for trk in out:
      if jitter:
        trk.x = trk.x + np.array([rng.normal(0., jitter / 2.), rng.normal(0., jitter / 2.), rng.normal(0., 0.2), rng.normal(0., 0.2)])
    return out
  w.tracks = tracks
  out = w.run(20.)
  i = first_go(out)
  assert i is not None and not any(d.fault for d in out)
  assert not any(d.withdraw for d in out), [(k, d.state, d.withdraw) for k, d in enumerate(out) if d.withdraw]
  assert any(d.state == 'done' for d in out[i:])


# ------------------------------------------------------------------------- through turnpilotd, the sim's oracle

def oracle_sim(tmp_path):
  """turnpilotd under vision at test_turnpilotd_vision's all-way (25 mph, no survey, the map's tiers), the sim's cross oracle
  on its gate as run 11 had it (the light oracle's and a sim car's): the decider reads ground truth for its tracks, objects and
  reach, as the run's did."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_vision import vsim
  sim = vsim(tmp_path, mph=25.)
  sim.tp.oracle_ok = True
  sim.tp.sim_files = True
  return sim


@pytest.mark.parametrize('case', ['before', 'after'])
def test_run11_through_turnpilotd_the_oracle_car_starting_is_withdrawn_for_or_stopped_short_of(tmp_path, case):
  # run 11 end to end: the oracle's car standing 16.7 m down the left lane, off at 2 m/s^2 0.5 s after our go ('before': the
  # withdrawal on the plan, waitingForCross saying why, stopped short of dEdgeMin), or the frame we commit ('after': the band's
  # abort, the car never reaching it)
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import oracle_msg, run_to_line
  sim = oracle_sim(tmp_path)
  st = {'go': None, 'start': None}

  def lights(s):
    sg_ = s.tp.stop_go
    ln = next((x for x in sg_.lanes_all if (x.role, x.index) == ('left', 1)), None)
    if sg_.token is None or ln is None:
      return oracle_msg()
    if st['go'] is None and sg_.state == 'going':
      st['go'] = s.t
    if st['start'] is None and ((case == 'before' and st['go'] is not None and s.t >= st['go'] + 0.5) or
                                (case == 'after' and sg_.state == 'committed')):
      st['start'] = s.t
    pos = sg_.pos_ref + (s.tp.odo_m - sg_.odo_ref)
    gone = max(s.t - st['start'], 0.) if st['start'] is not None else 0.
    tv = 3.
    d = RUN11_D - (0.5 * 2. * min(gone, tv) ** 2 + 6. * max(gone - tv, 0.))
    v = 2. * min(gone, tv)
    st['d'], st['axis'] = d, (ln.s_min + ln.s_max) / 2. - SETBACK  # its lane crosses our axis there (m past the stop point)
    return oracle_msg([(1, (ln.s_min + ln.s_max) / 2. - pos, d, 0., -v, 'car')] if d > -25. else [])
  sim.lights = lights
  run_to_line(sim)
  n0, car_d = len(sim.trace), []
  for _ in range(round(20. / DT_MDL)):
    sim.step()
    car_d.append(st.get('d'))
  plans = sim.plans()[n0:]
  assert st['go'] is not None and st['start'] is not None
  assert not any(cd is not None and touching(float(p.stopGo.positionM), cd, st['axis']) for p, cd in zip(plans, car_d, strict=True))
  if case == 'before':
    wd = next(k for k, p in enumerate(plans) if p.stopGo.withdrawn)
    assert plans[wd].stopGo.withdrawn.startswith('outOfTurn:1000001') and str(plans[wd].controlPrompt) == 'waitingForCross'
    assert plans[wd].reason.startswith("Stopping short: a car from the left")
    assert not any(str(p.stopGo.state) == 'committed' for p in plans[:wd])
    assert all(plan_accel_ok(p) for p in plans)
  else:
    # the band's capped abort for it (its reason on the plan while committed)
    band = any(str(p.stopGo.state) == 'committed' and p.reason.startswith("Stopping short: a car from the left") for p in plans)
    assert band, [(str(p.stopGo.state), str(p.controlAlert), p.reason) for p in plans[::20]]
    assert all(plan_accel_ok(p) for p in plans)


def plan_accel_ok(p) -> bool:
  from openpilot.selfdrive.turnpilot.hooks import plan_accel
  return plan_accel(p.accelCap, p.speedCap, 1.) >= -2. - 1e-9
