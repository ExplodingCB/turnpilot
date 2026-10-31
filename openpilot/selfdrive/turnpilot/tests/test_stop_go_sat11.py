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
the commit withdraws the go (the capped abort, the window again), after it the band's threat test sees it; and from the commit
on, a vehicle confirmed in our path corridor ahead on RAIL_HITS detector matches is the in-path rail: the car stopped short of it,
"Car crossing" said loudly (carCrossing), allowed in a cross lane because a car is entering our own path. Only a vehicle whose
ground point was seen moving across toward our path (ego motion and the camera's drift on a standing car we pass taken out) has
its body across our corridor, never one on its vy alone (skeptic round 1: a car waiting at the opposite line, one parked at the
exit kerb)."""
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
from openpilot.selfdrive.turnpilot.scene_policy import CONFIRM_STEADY_HITS, CONFIRM_WINDOW_S, NO_ACCEL_CAP, PATH_HALF_WIDTH_M
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
  hits, sr_hits0 = {}, CONFIRM_STEADY_HITS - 1
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
      # the oracle's hits as turnpilotd's cross_oracle counts them: one more each frame it is published (ground truth)
      hits[cid] = hits.get(cid, sr_hits0) + 1
      objs.append(SimpleNamespace(trackId=ORACLE_ID + cid, cls='car', x=x, y=y, vx=vx, vy=vy, age=hits[cid], conf=1., missedRuns=0))
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


def test_run11_were_its_start_missed_the_in_path_rail_holds_the_car_short_of_it_on_the_run_s_own_frames(monkeypatch):
  # the backstop on the run's own frames: with the start watch off (the run's miss: no withdrawal, the band still blind to
  # it), the in-path rail takes the oracle's car from 19.9 s, 1.1 s before the contact (about 21.0 s), the car then at 1.9 m/s
  # with 5.3 m to its stop: under 0.4 m/s^2 to stop short of it. Open loop the logged car drives on, so only its first frames
  # are the run's: the rail is in force on every frame from there to the contact, "Car crossing" said
  monkeypatch.setattr(sg.StopGo, '_watch_starts', lambda self, f: None)
  out = replay(until=21.0)
  rail = [(t, fr, d) for t, fr, d in out if any(s.reason == sg.CROSSING_REASON for s in d.stops)]
  assert rail, [(t, d.state) for t, _, d in out if t > 18.]
  t0, fr, d = rail[0]
  assert t0 <= 20.0 and d.state == 'committed' and d.alert == 'carCrossing' and d.reason == sg.CROSSING_REASON
  s = next(s for s in d.stops if s.reason == sg.CROSSING_REASON)
  assert s.confirmed and fr['v'] ** 2 / (2. * s.d_stop) < 0.5
  assert all(any(x.reason == sg.CROSSING_REASON for x in dd.stops) for t, _, dd in out if t >= t0)
  assert not any(d.withdraw for _, _, d in out)  # the run's miss: nothing else of the decider's acted


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
  # its hits rising one a detector update (every other frame), as the tracker counts them
  return SimpleNamespace(trackId=tid, cls='car', x=s, y=side * dd, vx=0., vy=vy, age=age + w.k // 2, conf=conf, inPath=False, missedRuns=0)


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
  # started early enough that the abort leaves GO_ROOM_M (run 11's 0.5 s), or later with less (0.8 s: 2.0 m) but vision's go
  # room (go_room_vision_m, the re-run's R7: at 6ba8582 this one handed back, H5): the window again from the withdrawal, the
  # next go only once the car has crossed our path, and we cross after it, never a handback
  assert room >= (sr.go_room_m(w.kw['t_abort']) if after_s <= 0.5 else sr.go_room_vision_m(w.kw['t_abort'])) - 1e-6, room
  assert nxt is not None and (nxt - wd) * DT_MDL >= sr.VISION_CLEAR_S - 1e-6
  assert geo[nxt][1] < -CAR_HALF_LEN_M
  assert any(d.state == 'done' for d in out[nxt:]) and not any(d.fault for d in out)


@pytest.mark.parametrize('where', ['back', 'line'])
def test_a_car_entering_our_path_after_the_commit_stops_us_short_of_it_saying_car_crossing(tmp_path, where):
  # run 11's car off the frame we commit: the band's threat test sees it now (bandAbort) or the in-path rail stops the car;
  # either way the car comes to rest short of it, never touching it, and the driver hears "Car crossing" or the band's abort
  w = world(tmp_path, where)
  d_fn, holder = start(standing_d(where))
  out, geo = drive_with(w, d_fn, holder, lambda d: d.state == 'committed', 20.)
  k = next(k for k, d in enumerate(out) if d.state == 'committed')
  after = out[k:]
  assert any(d.withdraw == 'bandAbort' for d in after) or any(d.alert == 'carCrossing' for d in after), \
    [(d.state, d.withdraw, d.alert) for d in after[:20]]
  assert not any(touching(p, cd, lane_axis_m(w, 'left')) for p, cd in geo)
  # at rest before it reached our path, short of where its lane crosses our axis
  meet = next(n for n, (_, cd) in enumerate(geo) if cd <= CAR_HALF_LEN_M + CAR_HALF_W)
  assert w.cmds[meet][1] < 0.3 and geo[meet][0] < lane_axis_m(w, 'left') - CAR_HALF_W


@pytest.mark.parametrize('where', ['back', 'line'])
def test_the_band_abort_s_stand_crosses_once_the_car_has_gone_never_a_handback(tmp_path, where):
  # the re-run at 6ba8582 (R2, stop_allway_out_of_turn): the car from the left started 0.15 s after our commit (1.05 m, 1.41
  # m/s), the band abort stood us 3.84 m past the stop point, 0.54 m past dEdgeMin; it crossed ahead (6.59 m nearest), then
  # V26's un-commit looked again, said "Going" and handed back on the go (H5: "no room for a go: -0.54 m to the closest edge,
  # 2.48 needed"). A band abort stands the car past the point a go needs GO_ROOM_M from, so under vision that stand is the
  # stall's, as the rail's is: held while the car still comes, then the clearBox lifts with the lanes left to cross tested
  # first, and across after it, never a handback, never touching it, never a stop under -2.0
  w = world(tmp_path, where)
  d_fn, holder = start(standing_d(where))
  out, geo = drive_with(w, d_fn, holder, lambda d: d.state == 'committed', 40.)
  ab = next((k for k, d in enumerate(out) if d.withdraw == 'bandAbort'), None)
  assert ab is not None, [(d.state, d.withdraw, d.alert) for d in out[::10]]
  stand = next(k for k in range(ab, len(out)) if w.cmds[k][1] < sg.STAND_V)
  assert w.sg.edges.d_min - geo[stand][0] < sr.go_room_m(w.kw['t_abort'])  # no go from here could have GO_ROOM_M
  assert not any(d.fault for d in out), [(k * DT_MDL, d.fault, d.reason) for k, d in enumerate(out) if d.fault][:3]
  assert not any(touching(p, cd, lane_axis_m(w, 'left')) for p, cd in geo)
  assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)
  done = next((k for k, d in enumerate(out) if d.state == 'done'), None)
  assert done is not None and done > stand, [(round(k * DT_MDL, 2), d.state, d.reason) for k, d in enumerate(out)][stand::20]
  # it moved on only once the car had crossed our path
  moved = next(k for k in range(stand, len(out)) if geo[k][0] > geo[stand][0] + 0.2)
  assert geo[moved][1] < -CAR_HALF_LEN_M


def crossing_y(kind, t):
  """(y left of our axis, vy) of the rail tests' scene car t s after the go reaches the stage: 5 m left then, at 2.5 m/s across
  our path, on its way since earlier (t < 0: further out on its line) ('crossing'), or the same but standing across our lane
  from 2 s to 5 s ('stops': a car stopped in our path)."""
  if kind == 'stops':
    if t < 2.:
      return 5. - 2.5 * t, -2.5
    if t < 5.:
      return 0., 0.
    return -2.5 * (t - 5.), -2.5
  return 5. - 2.5 * t, -2.5


def stage_at(tmp_path, stage):
  """(frame, position_m) of the frame the go first reaches stage in vdrive's run with nothing in the scene (the harness is
  deterministic, and the rail's car changes nothing before the stage's frame but the rail itself)."""
  w = vdrive(tmp_path, 25, behaviour='goes')
  for k in range(round(25. / DT_MDL)):
    d = w.step()
    if d.state == stage:
      return k, d.position_m
  raise AssertionError(stage)


@pytest.mark.parametrize('kind', ['crossing', 'stops'])
@pytest.mark.parametrize('stage,seen_s', [('committed', 0.), ('inLane', sg.RAIL_HIST_S)])
def test_the_in_path_rail_a_confirmed_car_entering_our_corridor_stops_us_short_and_says_car_crossing_loudly(tmp_path, stage, seen_s, kind):
  # a scene car (no lane track: nothing the band's threat test reads), its near side 9 m ahead of the bumper and 5 m left the
  # frame the go reaches the stage (the band, or past the in-lane point at 5 m/s), coming across at 2.5 m/s, tracked since
  # seen_s before: the rail stops the car short of it, "Car crossing" said while it holds; past the commit point that stop may
  # be in a cross lane, because a car is entering our own path. One that stands across our lane is stood short of until it
  # has gone; then the crossing goes on (the stall's lift if the model won't), never a handback. In the band it is first seen
  # at the stage (seen any earlier, it withdraws the go): its motion across is measured first (RAIL_MOVE_S, RAIL_MOVE_HITS
  # matches: about 0.7 s), which at the band's 2.5 m/s still leaves room to stop. Past the in-lane point it is tracked from 1 s
  # before, through the band (on a collision course its bearing holds: in the camera's view all along); first seen only there,
  # 9 m out at 5-6 m/s, that 0.7 s takes the room to stop (the stop comes about 5 m out), the trade for never trusting its vy
  k_stage, pos_stage = stage_at(tmp_path / 'pre', stage)
  w = vdrive(tmp_path, 25, behaviour='goes')
  car = {'x': pos_stage + SETBACK + 9., 'k_on': k_stage - round(seen_s / DT_MDL)}

  def objs(ww):
    if ww.k <= car['k_on']:
      return []
    y, vy = crossing_y(kind, (ww.k - 1 - k_stage) * DT_MDL)
    if y < -8.:
      return []
    pos = ww.sg.pos_ref + (ww.odo - ww.sg.odo_ref) if ww.sg.token is not None else -ww.control.d
    return [SimpleNamespace(trackId=77, cls='car', x=car['x'] - pos, y=y, vx=0., vy=vy, age=detector_age(ww, car), conf=0.9,
                            inPath=abs(y) < 1.5, missedRuns=0)]
  w.objects = objs
  out, gaps = [], []
  for k in range(round(25. / DT_MDL)):
    d = w.step()
    out.append(d)
    if k == k_stage:
      assert d.state == stage and abs(d.position_m - pos_stage) < 1e-6  # the run before the stage as the empty one
    if k >= k_stage:
      y, _ = crossing_y(kind, (k - k_stage) * DT_MDL)
      gaps.append((car['x'] - SETBACK - d.position_m, y, w.v))
  after = out[k_stage:]
  said = [d for d in after if d.alert == 'carCrossing']
  assert said and said[0].reason == sg.CROSSING_REASON
  assert any(s.reason == sg.CROSSING_REASON and s.confirmed for d in after for s in d.stops)
  # it never reached the car (its near side, the ground point) while the car's body was across our corridor
  across = [(gap, v) for gap, y, v in gaps if abs(y) <= CAR_HALF_LEN_M + CAR_HALF_W]
  assert across and min(gap for gap, _ in across) > 0.3
  if kind == 'stops':
    assert any(v < 0.05 for _, v in across)  # at rest short of the car standing in our path
  # the car gone, the encounter goes on: across and done, never a handback
  assert any(d.state == 'done' for d in after) and not any(d.fault for d in after)


def rail_car_run(tmp_path, stage, seen_s, y_fn, hits_fn=None, seconds=25.):
  """vdrive with the rail tests' scene car (trackId 77, its near side 9 m ahead of the bumper the frame the go reaches stage, on
  y_fn(t s from then) -> (y, vy) or None, tracked from seen_s before), its hits rising a detector update unless hits_fn(state,
  frame) says otherwise ((age, missedRuns)). Returns (w, decisions, k_stage, [(gap, y, v)] from the stage on)."""
  k_stage, pos_stage = stage_at(tmp_path / 'pre', stage)
  w = vdrive(tmp_path, 25, behaviour='goes')
  car = {'x': pos_stage + SETBACK + 9., 'k_on': k_stage - round(seen_s / DT_MDL)}
  out, gaps = [], []

  def objs(ww):
    yv = y_fn((ww.k - 1 - k_stage) * DT_MDL) if ww.k > car['k_on'] else None
    if yv is None:
      return []
    pos = ww.sg.pos_ref + (ww.odo - ww.sg.odo_ref) if ww.sg.token is not None else -ww.control.d
    age, missed = hits_fn(car, ww, out) if hits_fn is not None else (detector_age(ww, car), 0)
    return [SimpleNamespace(trackId=77, cls='car', x=car['x'] - pos, y=yv[0], vx=0., vy=yv[1], age=age, conf=0.9, inPath=abs(yv[0]) < 1.5,
                            missedRuns=missed)]
  w.objects = objs
  for k in range(round(seconds / DT_MDL)):
    d = w.step()
    out.append(d)
    if k >= k_stage:
      yv = y_fn((k - k_stage) * DT_MDL)
      gaps.append((car['x'] - SETBACK - d.position_m, yv[0] if yv is not None else None, w.v))
  return w, out, k_stage, gaps


def railed(d) -> list:
  return [s for s in d.stops if s.reason == sg.CROSSING_REASON]


def test_the_rail_s_hard_tier_is_scene_policy_s_confirmed_a_stalled_detector_s_track_brakes_on_the_curve(tmp_path):
  # the 'stops' car taken by the rail in the band; then the detector stalls (its hits stop rising, no run missed: published,
  # never matched again). The rail's stop stays, but its hard tier goes once the track is no longer steady (scene_policy's
  # CONFIRM_RECENT_HITS in CONFIRM_WINDOW_S): the comfortable curve from there, as every other confirmed stop's
  froze = {}

  def hits(car, ww, out):
    age = detector_age(ww, car)
    if 'k' not in froze and out and railed(out[-1]):
      froze.update(k=ww.k, age=age)
    return (froze['age'], 0) if 'k' in froze else (age, 0)
  w, out, k_stage, _ = rail_car_run(tmp_path, 'committed', 0., lambda t: crossing_y('stops', t), hits_fn=hits, seconds=12.)
  k0 = next(k for k, d in enumerate(out) if railed(d))
  assert all(s.confirmed for s in railed(out[k0]))
  lost = round((CONFIRM_WINDOW_S + 0.1) / DT_MDL)
  later = [d for d in out[k0 + lost:k0 + lost + 20] if railed(d)]
  assert later and not any(s.confirmed for d in later for s in railed(d))
  assert all(s.accel_cap >= NO_ACCEL_CAP - 1e-6 for d in later for s in railed(d))  # no accel cap: the curve only


def test_the_rail_lets_go_of_a_car_parked_past_our_path_whatever_its_vy_says(tmp_path):
  # the 'stops' car stood across our lane, then drove on across and parked 5 m right of our axis (its body clear of our
  # corridor), its tracker vy there reading noise (sd 0.6) toward us and away: the rail lets go RAIL_RELEASE_S after its body
  # left our corridor (its ground point making no progress back toward our path), and the crossing goes on to done
  rng = np.random.default_rng(9)
  noise = {}

  def y_fn(t):
    if t < 5.:
      return crossing_y('stops', t)
    y = max(-2.5 * (t - 5.), -5.)
    if y > -5.:
      return y, -2.5
    k = round(t / 0.1)
    noise.setdefault(k, rng.normal(0., 0.6))
    return -5., noise[k]
  w, out, k_stage, gaps = rail_car_run(tmp_path, 'committed', 0., y_fn, seconds=25.)
  assert any(railed(d) for d in out[k_stage:])
  # the frame its body left our corridor (its near end past PATH_HALF_WIDTH_M) for good
  left = next(n for n in range(len(gaps)) if all(y is not None and y < -(PATH_HALF_WIDTH_M + sg.RAIL_HALF_LEN_M) for _, y, _ in gaps[n:]))
  last_rail = max(k for k, d in enumerate(out) if railed(d)) - k_stage
  assert last_rail <= left + round((sg.RAIL_RELEASE_S + 0.3) / DT_MDL), (left, last_rail)
  assert any(d.state == 'done' for d in out[k_stage:]) and not any(d.fault for d in out)


def test_a_car_that_crossed_toward_us_and_stopped_with_its_nose_in_our_path_holds_the_rail(tmp_path):
  # the 'crossing' car stops with its centre 2.6 m left (its nose 0.25 m short of our axis, its ground point outside the
  # narrow corridor) and stands there 6 s, then goes on across: seen moving across it stays across (the latch), so the rail
  # holds the car short of it all the while it stands, and lets the crossing go on once it has passed
  def y_fn(t):
    if t < 0.96:
      return 5. - 2.5 * t, -2.5
    if t < 6.96:
      return 2.6, 0.
    return 2.6 - 2.5 * (t - 6.96), -2.5
  w, out, k_stage, gaps = rail_car_run(tmp_path, 'committed', 0., y_fn, seconds=32.)
  standing = [k for k in range(len(gaps)) if 1.5 <= k * DT_MDL < 6.9]
  assert all(railed(out[k_stage + k]) for k in standing)
  assert min(g for g, y, _ in gaps if y is not None and abs(y) <= CAR_HALF_LEN_M + CAR_HALF_W) > 0.3
  assert any(gaps[k][2] < 0.05 for k in standing)  # at rest short of it
  assert any(d.state == 'done' for d in out[k_stage:]) and not any(d.fault for d in out)


@pytest.mark.parametrize('side', [1., -1.])
def test_a_fast_car_crossing_ahead_first_seen_past_the_in_lane_point_is_stopped_short_of(tmp_path, side):
  # past the in-lane point at 5-6 m/s, a car first seen 9 m ahead and 6 m to the side crossing at 6 m/s (skeptic round 1's
  # rail_each_side): its bearing closes (it crosses ahead of us), so its motion is judged on the noise allowance alone, never
  # the camera's drift allowance for a car we pass: latched within its first few matches and stopped short of
  w, out, k_stage, gaps = rail_car_run(tmp_path, 'inLane', 0., lambda t: (side * (6. - 6. * t), -side * 6.) if t < 3. else None, seconds=20.)
  assert any(railed(d) for d in out[k_stage:])
  across = [g for g, y, _ in gaps if y is not None and abs(y) <= CAR_HALF_LEN_M + CAR_HALF_W]
  assert across and min(across) > 0.3


@pytest.mark.parametrize('y0,u', [(4., 1.5), (4., 2.), (5., 2.)])
def test_a_car_on_a_collision_course_past_the_in_lane_point_is_judged_without_the_drift_allowance(tmp_path, y0, u):
  # past the in-lane point at about 5.5 m/s, a car tracked from 1 s before, its ground point 9 m ahead and y0 left at the stage,
  # coming across at u (1.5-2 m/s: under a third of our speed). Its bearing holds or closes as it comes (nearly a collision
  # course), so it is judged on the noise allowance alone and stopped short of. Were the camera's drift allowance for a car
  # we pass (RAIL_DRIFT_FRAC of the metres driven) applied to it, its motion would read as a standing car's and the car would
  # drive into it. Slower still, or further out with its bearing opening (1.5 m/s from 5-6 m, 2 m/s from 6 m), it is within
  # that drift and the rail doesn't take it before its ground point is in our corridor: the trade for never braking for a
  # standing car we pass
  w, out, k_stage, gaps = rail_car_run(tmp_path, 'inLane', sg.RAIL_HIST_S, lambda t: (y0 - u * t, -u) if y0 - u * t > -8. else None,
                                       seconds=20.)
  assert any(railed(d) for d in out[k_stage:])
  across = [g for g, y, _ in gaps if y is not None and abs(y) <= CAR_HALF_LEN_M + CAR_HALF_W]
  assert across and min(across) > 0.3


def test_the_rail_never_brakes_for_a_phantom_a_vehicle_in_the_corridor_for_fewer_than_rail_hits(tmp_path):
  # a confirmed-looking box in our corridor 6 m ahead for RAIL_HITS - 1 detector matches (published, held, over the frames between
  # them, as the tracker does), then gone: no stop, no alert
  w = vdrive(tmp_path, 25, behaviour='goes')
  seen = {'n': 0}

  def objs(ww):
    if ww.sg.state != 'committed' or seen['n'] >= 2 * (sg.RAIL_HITS - 1):
      return []
    seen['n'] += 1
    return [SimpleNamespace(trackId=88, cls='car', x=SETBACK + 6., y=0.2, vx=0., vy=0., age=20 + (seen['n'] - 1) // 2, conf=0.9, inPath=True,
                            missedRuns=0)]
  w.objects = objs
  out = w.run(16.)
  assert seen['n'] == 2 * (sg.RAIL_HITS - 1)
  assert not any(d.alert == 'carCrossing' for d in out)
  assert not any(s.reason == sg.CROSSING_REASON for d in out for s in d.stops)
  assert any(d.state == 'done' for d in out)


def test_the_rail_never_brakes_for_an_unconfirmed_track(tmp_path):
  # a two-hit, low-confidence track sitting in our corridor (a phantom the tracker published): never the rail
  w = vdrive(tmp_path, 25, behaviour='goes')
  w.objects = lambda ww: [SimpleNamespace(trackId=89, cls='car', x=SETBACK + 7., y=0., vx=0., vy=0., age=2, conf=0.4, inPath=True)] \
    if ww.sg.state in ('committed', 'inLane') else []
  out = w.run(16.)
  assert not any(d.alert == 'carCrossing' for d in out)
  assert not any(s.reason == sg.CROSSING_REASON for d in out for s in d.stops)


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
  assert not any(d.alert == 'carCrossing' for d in out)


# (y left at its appearance, vx, vy, m ahead of the bumper) of a car the rail must not brake for after the commit
NOT_IN_PATH = {
  'far': (10., 0., -8., 30.),        # crossing our path 30 m ahead: beyond the rail's look
  'clearing': (-1., 0., -8., 8.),    # 8 m ahead, already leaving our corridor the other way at 8 m/s: clear before we get there
  'distant': (-60., 0., 8., 6.),     # a cross car 60 m off at 8 m/s: in our path only after we are past its line of travel
  'lead': (0.3, 5., 0., 8.),         # moving on ahead of us along our path at 5 m/s: a lead, the model's
}


@pytest.mark.parametrize('case', list(NOT_IN_PATH))
def test_a_car_that_passes_far_ahead_after_the_commit_never_brakes_us(tmp_path, case):
  # past the commit point, a confirmed car that isn't entering our path: no rail stop, no "Car crossing", the crossing done
  w = vdrive(tmp_path, 25, behaviour='goes')
  y0, vx, vy, ahead = NOT_IN_PATH[case]
  car = {}

  def objs(ww):
    if 't' not in car:
      return []
    t = ww.t_cam - 100. - car['t']
    pos = ww.sg.pos_ref + (ww.odo - ww.sg.odo_ref)
    y = y0 + vy * t
    if abs(y) > 45.:
      return []
    return [SimpleNamespace(trackId=90, cls='car', x=car['x'] + vx * t - pos, y=y, vx=vx, vy=vy, age=detector_age(ww, car, 30), conf=0.95,
                            inPath=abs(y) < 1.5, missedRuns=0)]
  w.objects = objs
  out = []
  for _ in range(round(16. / DT_MDL)):
    d = w.step()
    out.append(d)
    if 't' not in car and d.state == 'committed':
      car.update(t=w.t_cam - 100., x=d.position_m + SETBACK + ahead)
  k = next(k for k, d in enumerate(out) if d.state == 'committed')
  assert not any(d.alert == 'carCrossing' for d in out[k:])
  assert not any(s.reason == sg.CROSSING_REASON for d in out[k:] for s in d.stops)
  assert any(d.state == 'done' for d in out[k:])


@pytest.mark.parametrize('mode', ['trial'])
def test_the_start_watch_and_the_rail_are_vision_s_alone(tmp_path, mode):
  # the other modes as they were (their golden traces: test_stop_go_modes_golden): a known car's start is judged as before and
  # no rail runs, so a confirmed car in the corridor after the commit is never braked for by the decider there
  from openpilot.selfdrive.turnpilot.tests.test_stop_go import Drive
  w = Drive(tmp_path, behaviour=lambda ww: 'goes', mode=mode)
  w.objects = lambda ww: [SimpleNamespace(trackId=91, cls='car', x=SETBACK + 6., y=0., vx=0., vy=0., age=30 + ww.k // 2, conf=0.95, inPath=True,
                                          missedRuns=0)] if ww.sg.state in ('committed', 'inLane') else []
  out = w.run(16.)
  assert any(d.state == 'committed' for d in out)
  assert not any(d.alert == 'carCrossing' for d in out)
  assert not any(s.reason == sg.CROSSING_REASON for d in out for s in d.stops)


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
  # abort or the in-path rail, the car never reaching it). The oracle's objects are confirmed tracks (ground truth)
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
    # the band's capped abort for it (its reason on the plan while committed), or the rail's "Car crossing"
    band = any(str(p.stopGo.state) == 'committed' and p.reason.startswith("Stopping short: a car from the left") for p in plans)
    rail = any(str(p.controlAlert) in ('carCrossing', 'hardBrake') and p.reason == sg.CROSSING_REASON for p in plans)
    assert band or rail, [(str(p.stopGo.state), str(p.controlAlert), p.reason) for p in plans[::20]]
    assert all(plan_accel_ok(p) for p in plans) or rail


def plan_accel_ok(p) -> bool:
  from openpilot.selfdrive.turnpilot.hooks import plan_accel
  return plan_accel(p.accelCap, p.speedCap, 1.) >= -2. - 1e-9


class OracleSM:
  """SubMaster's face for cross_oracle: turnpilotLights received this frame."""
  def __init__(self, msg):
    self.data, self.updated = {'turnpilotLights': msg}, {'turnpilotLights': True}

  def __getitem__(self, k):
    return self.data[k]


def test_the_oracle_s_objects_are_confirmed_tracks_for_the_rail(tmp_path):
  # turnpilotd's cross_oracle gives its road users a confirmed track's hits (ground truth), one more each frame it publishes
  # them, so the decider's own evidence (scene_policy's TrackEvidence, steadiness included) confirms them for the rail; a car
  # gone from the message and back starts afresh
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import oracle_msg
  sim = oracle_sim(tmp_path)
  dec = sg.StopGo()
  ages = []
  for k in range(round(3. / DT_MDL)):
    out = sim.tp.cross_oracle(OracleSM(oracle_msg([(4, 8. - 0.1 * k, 0.5, 0., -3., 'car')])))
    assert out is not None
    (o,) = out[3]
    ages.append(o.age)
    dec.t += DT_MDL
    dec._rail_feed(sg.Frame(objects=(o,), odo_m=0.))
    assert dec._rail_confirmed(o) and o.trackId == ORACLE_ID + 4, (k, o.age)
  assert ages == list(range(CONFIRM_STEADY_HITS, CONFIRM_STEADY_HITS + len(ages)))
  sim.tp.cross_oracle(OracleSM(oracle_msg([])))
  (o,) = sim.tp.cross_oracle(OracleSM(oracle_msg([(4, 8., 0.5, 0., -3., 'car')])))[3]
  assert o.age == CONFIRM_STEADY_HITS


# ------------------------------------------------------- the rail's phantoms: cars beside our path (skeptic round 1)

def detector_age(ww, st, base=20, ages='advancing'):
  """A scene object's hits (TrackedObject.age): base at first sight, then one more per detector update (every other frame,
  the forward camera's 10 Hz), as the tracker counts them; 'constant': base on every frame (the probe's held count)."""
  st.setdefault('k0', ww.k)
  return base if ages == 'constant' else base + (ww.k - st['k0']) // 2


def junction_stand_s(w) -> float:
  """s the car stood (under 0.05 m/s) past its commit point (committed, inLane): in the junction."""
  return sum(1 for c in w.cmds if c[1] < 0.05 and c[6] in ('committed', 'inLane')) * DT_MDL


def assert_never_railed(w, out):
  rail = [k for k, d in enumerate(out) if any(s.reason == sg.CROSSING_REASON for s in d.stops)]
  assert not rail, (rail[:3], [out[k].state for k in rail[:3]], [round(w.cmds[k][1], 2) for k in rail[:3]])
  assert not any(d.alert == 'carCrossing' for d in out)
  assert all(s.accel_cap >= -2. - 1e-9 for d in out for s in d.stops)
  assert junction_stand_s(w) == 0.
  assert any(d.state == 'done' for d in out) and not any(d.fault for d in out)


# (k, sd, y0, seed): the camera's lateral speed for a car standing at the opposite approach's line, 20 m past our stop point and
# y0 left of our axis, reading toward our path in proportion to our speed (vy = -k v_ego, X_left_g30_h10_r150: -0.36 to -1.31
# m/s at 7.6-8 m/s passing cars standing 5 m left) plus noise sd. opp_k0.1 is the probe that fired the rail in inLane at 5.5
# m/s and commanded -7.25 m/s^2, the car then standing 2.6 s in the cross road saying 'Car Crossing'
OPPOSITE_WAITING = {
  'opp_k0.1': (0.1, 0.1, 2.6, 11),
  'opp_k0.15_y3.2': (0.15, 0.2, 3.2, 11),
  'opp_k0.15_y2.6_s12': (0.15, 0.1, 2.6, 12),
  'opp_k0.3_y3.2_s13': (0.3, 0.2, 3.2, 13),
}


@pytest.mark.parametrize('ages', ['advancing', 'constant'])
@pytest.mark.parametrize('case', list(OPPOSITE_WAITING))
def test_a_car_waiting_at_the_opposite_line_as_we_pass_is_never_the_rail_s(tmp_path, case, ages):
  # after the commit we drive past a car standing at the opposite approach's line: a real car that never enters our path. Its
  # ground point never moves (y0 throughout); only its tracker vy reads toward us. The rail never stops for it, no "Car
  # crossing", nothing harder than -2.0, the car never standing in the junction, and the crossing done
  k, sd, y0, seed = OPPOSITE_WAITING[case]
  w = vdrive(tmp_path, 25, behaviour='goes')
  rng = np.random.default_rng(seed)
  st = {'k': 0, 'vy': 0., 'vx': 0.}

  def objs(ww):
    if ww.sg.state not in ('committed', 'inLane', 'done') and 'k0' not in st:
      return []
    pos = ww.sg.pos_ref + (ww.odo - ww.sg.odo_ref)
    st['k'] += 1
    if st['k'] % 2 == 1:
      st['vy'] = -k * ww.v + rng.normal(0., sd)
      st['vx'] = rng.normal(0., 0.3)
    return [SimpleNamespace(trackId=66, cls='car', x=20. + SETBACK - pos, y=y0, vx=st['vx'], vy=st['vy'], age=detector_age(ww, st, 40, ages),
                            conf=0.85, inPath=False, missedRuns=0)]
  w.objects = objs
  out = w.run(20.)
  assert any(d.state == 'committed' for d in out)
  assert_never_railed(w, out)


# (y0 right of our axis, sd of its ground point's lateral noise, sd of its vy, seed): a car parked at the exit kerb 7 m past our
# stop point, the camera's noise on it. parked_sd0.6 is the probe that fired the rail at 2.0 m/s and held the car standing
# 29.9 s at the first cross lane's edge saying 'Car Crossing', then H5
PARKED = {
  'parked_sd0.6': (-3.0, 0.3, 0.6, 3),
  'parked_sd0.6_y2.6': (-2.6, 0.15, 0.6, 4),
  'parked_sd1.0_y3.5': (-3.5, 0.3, 1.0, 5),
  'parked_sd0.6_s6': (-3.0, 0.3, 0.6, 6),
}


@pytest.mark.parametrize('ages', ['advancing', 'constant'])
@pytest.mark.parametrize('case', list(PARKED))
def test_a_car_parked_at_the_exit_kerb_with_the_camera_s_noise_is_never_the_rail_s(tmp_path, case, ages):
  # a parked car beside our exit, its track's ground point jittering about y0 and its vy noise (the logged camera's: vy -0.36
  # to -1.31 m/s on a car standing 5 m left), held between detector updates: never the rail's, the crossing done
  y0, sd_y, sd_vy, seed = PARKED[case]
  w = vdrive(tmp_path, 25, behaviour='goes')
  rng = np.random.default_rng(seed)
  st = {'k': 0, 'y': y0, 'vy': 0.}

  def objs(ww):
    if ww.sg.state not in ('committed', 'inLane', 'done') and 'k0' not in st:
      return []
    pos = ww.sg.pos_ref + (ww.odo - ww.sg.odo_ref)
    st['k'] += 1
    if st['k'] % 2 == 0:  # a detector update every other frame: held between them
      st['y'] = y0 + rng.normal(0., sd_y)
      st['vy'] = rng.normal(0., sd_vy)
    return [SimpleNamespace(trackId=55, cls='car', x=7. + SETBACK - pos, y=st['y'], vx=rng.normal(0., 0.2), vy=st['vy'],
                            age=detector_age(ww, st, 40, ages), conf=0.8, inPath=False, missedRuns=0)]
  w.objects = objs
  out = w.run(45.)
  assert any(d.state == 'committed' for d in out)
  assert_never_railed(w, out)


@pytest.mark.parametrize('y0', [2.0, 2.43, 3.0, 3.8])
@pytest.mark.parametrize('vy', [-3., -1., 1.5])
def test_no_ground_point_outside_our_path_is_in_it_on_its_vy_alone(tmp_path, y0, vy):
  # a confirmed car standing y0 left of our axis 9 m past the stop point, its ground point never moving, its tracker vy saying it
  # moves across at vy (toward our path when negative): beyond PATH_HALF_WIDTH_M nothing of its vy puts it in our corridor or
  # entering it, so never the rail's
  w = vdrive(tmp_path, 25, behaviour='goes')
  st = {}

  def objs(ww):
    if ww.sg.state not in ('committed', 'inLane', 'done') and 'k0' not in st:
      return []
    pos = ww.sg.pos_ref + (ww.odo - ww.sg.odo_ref)
    return [SimpleNamespace(trackId=57, cls='car', x=9. + SETBACK - pos, y=y0, vx=0., vy=vy, age=detector_age(ww, st, 30), conf=0.9,
                            inPath=False, missedRuns=0)]
  w.objects = objs
  out = w.run(20.)
  assert_never_railed(w, out)


# ------------------------------------------------------------ the rail on the logged camera's own objects (open loop)

CAMERA_FIXTURE = Path(__file__).parent / 'sat11_camera_rail.json'


def rail_on_log(clip, armed=lambda fr: 0.3 < fr['v'] < 8., standing_x=None):
  """The decider's in-path rail (StopGo._rail_feed every frame, _rail_pick on the frames armed(frame) says) on a logged clip's
  camera scene objects (tests/sat11_camera_rail.json), along the logged trajectory: the odometer from vEgo, heading_deg from the
  model's yaw rate. standing_x: that track's x made a standing car's (its median ground x less the odometer, the logged range
  noise taken out; its y as logged). Returns ([(t, trackId, crossing)] of each pick, the decider)."""
  frames = json.loads(CAMERA_FIXTURE.read_text())['clips'][clip]['frames']
  odos, odo, last_t = [], 0., None
  for fr in frames:
    odo += max(fr['v'], 0.) * (DT_MDL if last_t is None else min(max(fr['t'] - last_t, 0.01), 0.2))
    last_t = fr['t']
    odos.append(odo)
  gx = float(np.median([o[2] + odo for fr, odo in zip(frames, odos, strict=True) for o in fr['objs'] if o[0] == standing_x] or [0.]))
  dec = sg.StopGo()
  turned = 0.
  last_t, picks = None, []
  for fr, odo in zip(frames, odos, strict=True):
    dt = DT_MDL if last_t is None else min(max(fr['t'] - last_t, 0.01), 0.2)
    last_t = fr['t']
    v = max(fr['v'], 0.)
    turned += math.degrees(fr['yr'] * dt)
    objs = tuple(SimpleNamespace(trackId=o[0], cls=o[1], x=gx - odo if o[0] == standing_x else o[2], y=o[3], vx=o[4], vy=o[5], age=o[6], conf=o[7],
                                 missedRuns=o[8], inPath=False) for o in fr['objs'])
    dec.t += dt
    f = sg.Frame(objects=objs, v_ego=v, a_ego=fr['a'], odo_m=odo, heading_deg=turned, dt=dt)
    dec._rail_feed(f)
    if not armed(fr):
      dec.rail_count = {}
      continue
    best, _ = dec._rail_pick(f)
    if best is not None:
      picks.append((fr['t'], best[1].trackId, dec.rail_tracks[best[1].trackId].crossing))
  return picks, dec


@pytest.mark.parametrize('standing_x', [None, 6])
def test_the_rail_on_the_logged_camera_never_takes_the_standing_cars_we_pass(standing_x):
  # X_left_g30_h10_r150's camera, as if the rail were armed on every frame we move: track 6, a car standing 5.1 m left (the
  # oracle), read y 4.6 -> 2.6 and vy -0.36 to -1.31 as we passed at 7.5-8.7 m/s, its ground point drifting 1.1-1.6 m toward
  # our axis (the would-be rail fire at 730030.48 on the old vy test). Its bearing opened all the while: the drift is the
  # camera's (RAIL_DRIFT_FRAC), never crossing, never the rail's; nor is any other car of the clip. With its logged x the range
  # noise (its ground x 4 m about) reads it moving along as much as across too; with a standing car's x (standing_x) only the
  # drift allowance tells its drift from a crossing
  picks, dec = rail_on_log('x_left_track6', standing_x=standing_x)
  assert not picks
  assert 6 in dec.rail_tracks and not any(r.crossing for r in dec.rail_tracks.values())


@pytest.mark.parametrize('clip,track,commit_t,by_t', [
  ('run11_camera', 2, 728380.42, 728382.25),  # run 11: the rail's take by 19.95 s on the report's clock (contact about 21 s)
  ('x6_camera', 1, 730309.14, 730310.0),      # X_out_of_turn_r2: within 0.9 s of its commit
])
def test_the_rail_on_the_logged_camera_takes_the_out_of_turn_car_crossing_into_our_path(clip, track, commit_t, by_t):
  # the logged runs' own camera objects from their commit on (the rail armed as in committed and inLane): the car from the left
  # going out of turn is latched crossing by its ground point's measured motion (its bearing closing: never the drift
  # allowance's) and taken by the rail before by_t, the first car the rail takes
  picks, dec = rail_on_log(clip, armed=lambda fr: fr['t'] >= commit_t - 1e-3)
  assert picks, [(tid, r.crossing, r.run, r.lat) for tid, r in dec.rail_tracks.items()]
  t, tid, crossing = picks[0]
  assert tid == track and crossing and t <= by_t, picks[:3]


@pytest.mark.parametrize('clip', ['far_settling', 'far_oncoming'])
def test_far_tracks_range_noise_never_latches_one_crossing(monkeypatch, clip):
  # far out a mono track's ground point is range noise: new tracks settling from 40-76 m (one read its lateral 16.5 -> 6.9 m in
  # 1.2 s on a car standing 7.9 m left) and an oncoming car 40-55 m out reading 4-18 m/s across. The motion test runs only on a
  # window within the rail's look (and RAIL_LATCH_EXTRA_M), and only on motion more across than along: none of them is ever
  # latched crossing (the tracks the clip ends with, and any it dropped), so none passing beside us later has its body across
  # our corridor
  latched = []
  orig = sg.StopGo._rail_motion

  def spy(self, r, o, odo):
    orig(self, r, o, odo)
    if r.crossing:
      latched.append(o.trackId)
  monkeypatch.setattr(sg.StopGo, '_rail_motion', spy)
  picks, dec = rail_on_log(clip)
  assert not picks and not latched


# ------------------------------------------------------------------ the rail in a turn (open loop, consistent geometry)

LEFT_R, LEFT_LEAD = 10., 1.  # m: a left turn's arc from the stop point, after this much straight


def left_pose(sp):
  """(x, y left, heading rad) sp m along a left turn from the stop point (LEFT_LEAD straight, then LEFT_R's arc to 90 deg)."""
  if sp <= LEFT_LEAD:
    return sp, 0., 0.
  a = min((sp - LEFT_LEAD) / LEFT_R, math.pi / 2.)
  beyond = max(sp - LEFT_LEAD - LEFT_R * math.pi / 2., 0.)  # on along the cross road
  return LEFT_LEAD + LEFT_R * math.sin(a), LEFT_R * (1. - math.cos(a)) + beyond, a


def to_ego(pose, wx, wy):
  x0, y0, h = pose
  dx, dy = wx - x0, wy - y0
  return dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)


def turn_drive(phases, cars, model=True):
  """The decider's rail fed open loop along a left turn (left_pose): phases [(seconds, accel m/s^2, going)] from standing at the
  stop point (speed floored at 0), turned_deg as turnpilotd keeps it (summed while going, 0 otherwise), heading_deg never
  reset; with model, a modelV2 whose path is the turn's arc ahead (its y to the right), or none at all (the model's path
  unknown: predicted_path's straight ahead) at standstill as the model gives no path ahead then; cars {trackId: (wx, wy)}
  standing, a detector update every other frame (held between). Returns (decider, [(t, picked trackId or None, {tid: crossing})])."""
  dec = sg.StopGo()
  v = sp = turned = t = 0.
  out, k = [], 0
  held = {}
  for seconds, acc, going in phases:
    for _ in range(round(seconds / DT_MDL)):
      h0 = left_pose(sp)[2]
      v = max(v + acc * DT_MDL, 0.)
      sp += v * DT_MDL
      pose = left_pose(sp)
      turned = turned + math.degrees(pose[2] - h0) if going else 0.
      t += DT_MDL
      if k % 2 == 0:
        held = {tid: to_ego(pose, wx, wy) for tid, (wx, wy) in cars.items()}
      objs = tuple(SimpleNamespace(trackId=tid, cls='car', x=x, y=y, vx=0., vy=0., age=30 + k // 2, conf=0.9, missedRuns=0, inPath=False)
                   for tid, (x, y) in held.items())
      m = None
      if model and v > 0.3:
        ahead = [to_ego(pose, *left_pose(sp + d)[:2]) for d in np.linspace(0., 30., 31)]
        m = SimpleNamespace(position=SimpleNamespace(x=[a[0] for a in ahead], y=[-a[1] for a in ahead]))
      elif model:
        m = SimpleNamespace(position=SimpleNamespace(x=[0.] * 31, y=[0.] * 31))  # standing: nothing planned ahead
      dec.t += DT_MDL
      f = sg.Frame(objects=objs, v_ego=v, a_ego=acc, odo_m=sp, turned_deg=turned, heading_deg=math.degrees(pose[2]), model_v2=m)
      dec._rail_feed(f)
      best, _ = dec._rail_pick(f)
      out.append((t, best[1].trackId if best is not None else None, {tid: r.crossing for tid, r in dec.rail_tracks.items()}))
      k += 1
  return dec, out


OPPOSITE_WAITING_AT = (16., 2.8)  # m from our stop point: a car waiting at the opposite approach's line, in its lane


def test_a_go_withdrawn_mid_turn_never_reads_our_yaw_back_as_the_waiting_car_crossing():
  # our left turn's go withdrawn some 6 deg into the turn, the car brought to a stand, then the go again and the turn: turnpilotd
  # zeroes turned_deg at the withdrawal (summed only while going), which on the rail's ground frame read as a turn back of
  # that much, the car waiting at the opposite line 16 m out then seeming to come 1-2 m toward our axis while we stood. The
  # ground frame is heading_deg's, never reset: the waiting car is never latched crossing
  phases = [(1., 0., False), (1.6, 1.5, True), (1.2, -2.2, False), (2., 0., False), (6., 1.5, True)]
  dec, out = turn_drive(phases, {66: OPPOSITE_WAITING_AT})
  turned_at_withdrawal = math.degrees(left_pose(0.5 * 1.5 * 1.6 ** 2)[2])
  assert 3. < turned_at_withdrawal < 10.
  assert not any(c.get(66) for _, _, c in out)


def test_mid_left_turn_the_rail_s_corridor_is_our_path_never_the_straight_ahead():
  # a left turn from the stop point with the car waiting at the opposite line: mid-turn it is straight ahead of us for some
  # metres of the arc while our path (the model's) curves away from it, so it is never the rail's; a car standing in our path
  # on the arc ahead (the exit lane we turn into) is, and still once we stand short of it with the model planning nothing ahead
  # (the path kept from the last frame it gave one)
  phases = [(1., 0., False), (5., 1.2, True)]
  dec, out = turn_drive(phases, {66: OPPOSITE_WAITING_AT})
  ego = [to_ego(left_pose(sp), *OPPOSITE_WAITING_AT) for sp in np.linspace(0., 15., 61)]
  assert any(0. < x - SETBACK <= sg.RAIL_AHEAD_M and abs(y) <= PATH_HALF_WIDTH_M for x, y in ego)  # straight ahead for a while
  assert not any(p == 66 for _, p, _ in out)
  in_path = left_pose(13.)[:2]
  dec, out = turn_drive([(1., 0., False), (2.5, 1.2, True), (2.5, -1.5, True), (2., 0., True)], {66: OPPOSITE_WAITING_AT, 67: in_path})
  picks = [p for _, p, _ in out]
  assert 67 in picks and 66 not in picks
  assert picks[-1] == 67  # standing short of it, the model planning nothing: the path kept
