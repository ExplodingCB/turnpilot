"""A route turn at a signal (map_controls.py module docstring, _signal_turns; 2026-11-06): the turn stops at the signal's line
unless a steady green is read first, goes from the line on a steady green, and with no light read hands back loudly, held,
never a roll. The 11-06 night drive's right at a mapped signal (1e 8:20.4) rolled toward the line on the model's turn desire
until the driver braked, and the night reader read no colour on any governing head: a turn at a light can't rest on reading
its red. Going straight through a signal is unchanged (the model's), and the unprotected left keeps its own hold."""
from dataclasses import replace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import map_controls
from openpilot.selfdrive.turnpilot.control_tracker import TrackedControl
from openpilot.selfdrive.turnpilot.map_controls import (RIGHT_RED_CLEAR_S, RIGHT_RED_DWELL_S, RIGHT_RED_LOOKING_REASON,
                                                         RIGHT_RED_UNREAD_S, RIGHT_RED_WAIT_REASON,
                                                         CAMERA_ASSOC_PAST_M, CAMERA_GREEN_STEADY_S, CAMERA_STALE_S, CONTROL_LINE_M,
                                                         GREEN_STEADY_S, MAP_DECEL_MAX, NO_LIGHTS, OVERRUN_PAST_M, RED_GONE_S,
                                                         PROTECT_BEFORE_M, SIGNAL_UNSEEN_REASON,
                                                         SIGNAL_UNSEEN_S, TURN_HEAD_GAP_S, TURN_UNSEEN_S, UNPROTECTED, CameraLights,
                                                         ControlState, LeftCommand, Lights)
from openpilot.selfdrive.turnpilot.tests.test_camera_lights import Events
from openpilot.selfdrive.turnpilot.tests.test_map_controls import Car, held, lights

JUNCTION = 200.
LINE = 192.  # a mapped signal's line 8 m short of the junction
HEAD = 215.  # its governing head, hung on the far side


def right_at_a_signal(v=13.4, mapped=True, mid=7):
  car = Car([(LINE, 'signal', True)] if mapped else [], v=v)
  car.junction = (JUNCTION, mid)
  return car


def cam(state, s_head=HEAD, shadow=False, oracle_state=None, s_oracle=LINE):
  """The camera reading a governing head at road position s_head (add-only unless shadow), with the sim oracle's light at
  s_oracle if given."""
  def make(car):
    base = lights(oracle_state, s_oracle - car.s) if oracle_state is not None else Lights()
    return replace(base, camera=CameraLights([(state, s_head - car.s)], shadow, 'read'))
  return make


def heads(*states, s_head=HEAD):
  """Several governing heads at s_head, each reading its state."""
  return lambda car: Lights(camera=CameraLights([(s, s_head - car.s) for s in states], False, 'read'))


def held_at(car, s_line, **kw):
  out, dec = car.run_until(held, **kw)
  assert -2. < s_line - car.s < 5., car.s
  assert all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)  # never past -2.0
  return out, dec


# --- the turn stops at the line unless a steady green is read first ---

@pytest.mark.parametrize("light", [None, 'unknown'])
def test_a_turn_at_a_mapped_signal_that_is_no_curb_side_turn_in_hand_goes_with_the_model_on_no_light_read(light):
  # a turn at a light is a through signal's: no stop for want of a green, no handback for an unread light. Not a curb-side
  # turn TurnPilot is taking (update()'s right: the right-on-red procedure, below)
  car = right_at_a_signal()
  kw = {} if light is None else {'lights': cam(light)}
  for _ in range(int(40. / DT_MDL)):
    out, dec, _ = car.step(**kw)
    assert not held(out, dec) and out.prompt == 'none' and not out.stops
    if car.s > LINE + 5.:
      break
  assert car.s > LINE + 5.


def test_a_right_at_a_mapped_signal_on_a_camera_red_holds_without_a_handback_while_the_red_is_read():
  car = right_at_a_signal()
  out, dec = held_at(car, LINE, lights=cam('red'))
  assert out.stops[0].reason == "Red light (camera)"
  st = car.policy.tracker.controls[(1, 0)].state
  assert not st.turn_stop and st.camera_added and not st.accepted_stop  # a red's stop now, TurnPilot's own
  for _ in range(int(60. / DT_MDL)):
    out, dec, _ = car.step(lights=cam('red'))
    assert held(out, dec) and out.prompt == 'none'
  # the red no longer read (the light changed to a green the reader can't read, as most of route 29's night greens): held
  # through a gap in its reads, let go RED_GONE_S after the last, never stood at a light nobody reads red
  out, dec, _ = car.run(RED_GONE_S - 0.2, lights=cam('unknown'))
  assert held(out, dec) and out.prompt == 'none'
  out, dec, _ = car.run(0.4, lights=cam('unknown'))
  assert not held(out, dec) and not out.stops and out.prompt == 'none' and out.alert == 'none'
  assert st.mode == 'go' and car.policy.stop_ahead() is None


def test_a_red_read_again_inside_its_gap_keeps_the_stop():
  car = right_at_a_signal()
  held_at(car, LINE, lights=cam('red'))
  for _ in range(10):  # the red in and out of the reads, each gap under RED_GONE_S (route 29: 1.7 s at most in a 25 s stand)
    out, dec, _ = car.run(RED_GONE_S - 0.5, lights=cam('unknown'))
    assert held(out, dec)
    out, dec, _ = car.run(0.2, lights=cam('red'))
    assert held(out, dec)


def no_heads(car):
  return Lights(camera=CameraLights([], False, 'read'))


def other_heads(car):
  return Lights(camera=CameraLights([('unknown', HEAD + 150. - car.s), ('green', HEAD + 150. - car.s)], False, 'read'))


@pytest.mark.parametrize('lights', [NO_LIGHTS, no_heads, other_heads], ids=['noRead', 'noHeads', 'nextSignalsHeads'])
def test_a_red_whose_heads_are_out_of_view_is_never_let_go_as_gone(lights):
  """No head of the signal in view (a truck ahead, the heads out of the frame at the line, the reader not reading) is not its
  red gone: the stop a read red raised stands, however long, until its heads are in view RED_GONE_S on end and not read red
  (or read green)."""
  car = right_at_a_signal()
  out, dec = held_at(car, LINE, lights=cam('red'))
  st = car.policy.tracker.controls[(1, 0)].state
  out, dec, _ = car.run(RED_GONE_S - 0.5, lights=cam('unknown'))  # most of the way to gone, then hidden
  for _ in range(int(60. / DT_MDL)):
    out, dec, _ = car.step(lights=lights)
    assert held(out, dec) and st.mode == 'stop'
  for _ in range(20):  # in view and unread in bursts, out of view between them (a head boxed on some runs only): never gone
    out, dec, _ = car.run(RED_GONE_S - 0.5, lights=cam('unknown'))
    assert held(out, dec)
    out, dec, _ = car.run(0.3, lights=lights)
    assert held(out, dec)
  out, dec, _ = car.run(RED_GONE_S - 0.2, lights=cam('unknown'))  # in view again and unread, its whole run from the start
  assert held(out, dec)
  out, dec, _ = car.run(0.4, lights=cam('unknown'))
  assert not held(out, dec) and not out.stops


def test_a_red_on_the_approach_whose_heads_go_out_of_view_still_stops_the_car():
  car = right_at_a_signal()
  car.run_until(lambda out, dec: out.stops, lights=cam('red'))
  assert car.v > 5. and car.s < LINE - 20.
  out, dec = held_at(car, LINE, lights=no_heads)
  assert held(out, dec)


def test_a_red_gone_on_the_approach_lets_the_car_go_before_it_stops():
  car = right_at_a_signal()
  car.run_until(lambda out, dec: out.stops, lights=cam('red'))
  assert car.v > 5. and car.s < LINE - 20.
  out, dec, _ = car.run(RED_GONE_S + 0.2, lights=cam('unknown'))
  assert not out.stops and car.v > 2. and car.s < LINE


def test_a_steady_camera_green_before_the_stopping_point_means_no_stop():
  base, car = Car([(LINE, 'signal', True)]), right_at_a_signal()  # base: the same signal going straight
  for _ in range(int(25. / DT_MDL)):
    a, b = base.step(), car.step(lights=cam('green'))
    assert a[0].stops == b[0].stops == [] and a[2] == b[2]
  assert car.s > JUNCTION + 50. and min(r[2] for r in car.rows) >= 13.4 - 1e-6
















def test_a_camera_green_that_cant_release_the_stop_puts_off_nothing():
  # an accepted source's red made the stop (only an accepted green ends it): a camera green read after the red is lost
  # changes nothing, and the stand's unseen line comes at SIGNAL_UNSEEN_S as before, held (never a handback since 2026-11-07)
  car = right_at_a_signal()
  held_at(car, LINE, lights=lambda car: lights('red', LINE - car.s))
  out, dec, _ = car.run(SIGNAL_UNSEEN_S - 0.3, lights=cam('green', oracle_state='unknown'))
  assert held(out, dec) and out.prompt == 'none' and out.stops[0].reason != SIGNAL_UNSEEN_REASON
  out, dec, _ = car.run(0.6, lights=cam('green', oracle_state='unknown'))
  assert held(out, dec) and out.prompt == 'none' and out.stops[0].reason == SIGNAL_UNSEEN_REASON




def test_a_camera_green_never_releases_a_stop_an_accepted_red_made():
  car = right_at_a_signal()
  held_at(car, LINE, lights=lambda car: lights('red', LINE - car.s))
  st = car.policy.tracker.controls[(1, 0)].state
  assert st.accepted_stop and not st.camera_added
  out, dec, _ = car.run(5., lights=cam('green', oracle_state='unknown'))
  assert held(out, dec)
  out, dec, _ = car.run(GREEN_STEADY_S + 0.1, lights=cam('unknown', oracle_state='green'))
  assert not dec.should_stop  # only an accepted green


def test_a_camera_green_with_a_red_head_still_read_is_no_green():
  car = right_at_a_signal()
  held_at(car, LINE, lights=cam('red'))
  out, dec, _ = car.run(5., lights=heads('green', 'red'))  # one green, one red: no green
  assert held(out, dec) and not out.released_stopped and out.prompt == 'none'


def test_a_stop_is_let_go_by_more_green_heads_than_red():
  # route 29, 8:46: three green balls over the lanes and a left-turn head still red beside them. The heads disagree, so no
  # green was the signal's, and a car stopped at that light's red would have stood at its greens for good
  car = right_at_a_signal()
  held_at(car, LINE, lights=heads('red', 'red', 'red', 'red'))
  out, dec, _ = car.run(CAMERA_GREEN_STEADY_S + 0.2, lights=heads('green', 'green', 'green', 'red'))
  assert not held(out, dec) and not out.stops
  # a lone green beside more reds (a green left arrow read as a ball, the balls red) lets nothing go, however long
  car = right_at_a_signal()
  held_at(car, LINE, lights=heads('red', 'red', 'red'))
  out, dec, _ = car.run(20., lights=heads('green', 'red', 'red'))
  assert held(out, dec) and not out.released_stopped and out.prompt == 'none'


# --- going straight, and the left's own hold: unchanged ---

def test_a_straight_through_mapped_signal_with_no_read_is_unchanged():
  # no route turn at all, and a route turn at the next junction: neither slows for this signal
  for junction in (None, (JUNCTION + 300., 9)):
    car = Car([(LINE, 'signal', True)])
    car.junction = junction
    car.run(25.)
    assert car.s > JUNCTION + 50. and min(r[2] for r in car.rows) >= 13.4 - 1e-6
    assert car.policy.tracker.controls.get((1, 0)) is None or car.policy.tracker.controls[(1, 0)].state.turn_id is None


def test_a_signal_at_the_junction_the_route_goes_straight_through_before_the_turns_is_the_models():
  # 1e maneuver 12 and 1d maneuver 5 (2026-11-06): a signal 34-42 m back of the turn's junction, none of the turn's own. Both
  # were the signal of the junction the route crosses straight 29-32 m before the maneuver point (navInstruction
  # throughLanesDistance; the signal's stop point 11 m short of it), the turn a slight left at a junction of its own. Past
  # PROTECT_BEFORE_M it is no turn's signal: going straight through it is the model's, never a stop for want of a green
  for setback in (34.4, 42.1):
    car = Car([(JUNCTION - setback, 'signal', True, False)])
    car.junction = (JUNCTION, 7)
    car.run(25.)
    assert setback > PROTECT_BEFORE_M and car.s > JUNCTION + 50. and min(r[2] for r in car.rows) >= 13.4 - 1e-6
    sig = car.policy.tracker.controls.get((1, 0))
    assert sig is None or sig.state is None or sig.state.turn_id is None


def test_a_skipped_turn_goes_straight_through_its_signal_as_the_model_does():
  # update()'s junction with TurnPilot not driving the turn (turnpilotd: skipped): its signal is the model's
  car = Car([(LINE, 'signal', True)])
  for _ in range(int(25. / DT_MDL)):
    car.step(junction=lambda car: (7, JUNCTION - car.s, False))
  assert car.s > JUNCTION + 50. and min(r[2] for r in car.rows) >= 13.4 - 1e-6
  # nor an unmapped head there of unknown colour: no signal of the camera's for a turn not taken
  car = Car([])
  for _ in range(int(25. / DT_MDL)):
    car.step(junction=lambda car: (7, JUNCTION - car.s, False), lights=cam('unknown'))
  assert car.s > JUNCTION + 50. and car.policy.camera_signal is None


def test_a_skipped_turn_still_stops_on_a_camera_red_at_its_junction():
  car = Car([])
  for _ in range(int(40. / DT_MDL)):
    out, dec, _ = car.step(junction=lambda car: (7, JUNCTION - car.s, False), lights=cam('red'))
    if held(out, dec):
      break
  assert held(out, dec) and -2. < JUNCTION - CONTROL_LINE_M - car.s < 5.








def held_without_a_gap(car, s_line, junction):
  """Driven to a hold at s_line with junction (a function of the car): the stop never drops out once it is up, never past -2.0."""
  up = False
  for _ in range(int(60. / DT_MDL)):
    out, dec, _ = car.step(junction=junction)
    if held(out, dec):
      break
    up = up or out.stops != []
    assert not up or out.stops, f"the turn's stop dropped out {s_line - car.s:.1f} m short of its line"
    assert car.s < s_line + 5., "through the line with no hold"
  assert held(out, dec) and -2. < s_line - car.s < 5., car.s
  assert all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)
  return out, dec








def green_then_unread(car, near_m, far_m):
  """The camera reads the governing head green while the car is near_m to far_m short of the line, nothing at all after."""
  return lambda car: cam('green')(car) if near_m < LINE - car.s < far_m else NO_LIGHTS




def test_a_green_lost_too_close_to_stop_for_at_the_yellow_rate_is_committed(monkeypatch):
  # read green to 50 m out at 13.4 m/s: TURN_GREEN_STALE_S on, the car is ~10 m short, inside a yellow's stopping distance
  # (v*T_LAT + v^2/4 = 55.6 m): committed as such a yellow would be, never a stop at the floor in the junction's mouth
  ev = Events(monkeypatch)
  car = right_at_a_signal()
  car.s = LINE - 90.
  lit = green_then_unread(car, 50., 120.)
  for _ in range(int(12. / DT_MDL)):
    out, dec, _ = car.step(lights=lit)
    assert out.stops == [] and not dec.should_stop
  assert car.s > JUNCTION + 20. and min(r[2] for r in car.rows) >= 13.4 - 1e-6
  assert not ev.named('turnpilot.signal_turn_green_stale')
  committed = ev.named('turnpilot.signal_turn_green_committed')
  assert len(committed) == 1 and committed[0]['d'] < 55.6






def test_the_unprotected_lefts_own_hold_keeps_precedence_at_a_signal():
  # an unprotected left at a signal (opposing present): its own hold line, unchanged with the junction handed in or not
  def left_car(junction):
    car = Car([(LINE - 2., 'signal', True)])
    car.turn = (JUNCTION, 7, 'left')
    car.junction = (JUNCTION, 7) if junction else None
    return car
  a, b = left_car(False), left_car(True)
  for _ in range(int(40. / DT_MDL)):
    oa = a.step(left=LeftCommand('auto'))
    ob = b.step(left=LeftCommand('auto'))
    assert oa[0].stops == ob[0].stops and oa[0].prompt == ob[0].prompt and oa[2] == ob[2]
  assert held(*ob[:2]) and ob[0].shown.kind == UNPROTECTED
  assert b.policy.tracker.controls[(1, 0)].state.turn_id is None and b.policy.camera_signal is None
  # its go lets its hold go; the signal's line just behind the car is never a new stop there
  s_go = b.s
  for _ in range(int(6. / DT_MDL)):
    out, dec, _ = b.step(left=LeftCommand('auto', go=True))
    sig = b.policy.tracker.controls.get((1, 0))
    assert sig is None or sig.state.turn_id is None
  assert b.s > s_go + 5. and not dec.should_stop




def gas_until(car, s_lift, lights=NO_LIGHTS, seconds=40.):
  """openpilot engaged, the driver's gas holding the car's speed (the plan overridden) until s_lift, then off: [(m short of
  the line, v, cmd, alert)] until the car is through the junction or stands."""
  rows = []
  for _ in range(int(seconds / DT_MDL)):
    gas = car.s < s_lift
    out, dec, cmd = car.step(gas=gas, accel=0. if gas else None, engaged=True, lights=lights)
    rows.append((LINE - car.s, car.v, cmd, out.alert))
    if car.s > JUNCTION + 10. or (car.v < 0.05 and not gas):
      break
  return rows




def brake_by_hand(car, s_stop):
  """The driver's brake (openpilot disengaged) to a stand at about s_stop."""
  for _ in range(int(20. / DT_MDL)):
    a = -car.v ** 2 / (2. * max(s_stop - car.s, 0.3)) if car.v > 0.05 else -1.
    car.step(accel=a, engaged=False)
    if car.v < 0.01:
      return
  raise AssertionError("never stood")


def lifted_engaged(car, s_lift, coast_s=0.3, **kw):
  """openpilot engaged, the driver's gas holding the car's speed until s_lift, then lifted there with the plan applied for
  coast_s."""
  while car.s < s_lift:
    car.step(gas=True, accel=0., engaged=True, **kw)
  return car.run(coast_s, engaged=True, **kw)












def test_a_red_read_over_the_line_on_the_drivers_go_is_never_a_stop_in_the_junction(monkeypatch):
  # over the line faster than COMMIT_SPEED the go is the car's: a red read there (the light changing, the far head read late)
  # never brakes the car into the junction
  ev = Events(monkeypatch)
  car = right_at_a_signal(v=11.)
  rows = gas_until(car, LINE - 15., lights=lambda car: cam('red')(car) if car.s > LINE + 0.5 else NO_LIGHTS)
  after = [r for r in rows if r[0] < 15.]
  assert car.s > JUNCTION + 10. and min(r[2] for r in after) > -0.5
  assert not ev.named('turnpilot.map_stop_overrun') and not ev.named('turnpilot.signal_turn_go_revoked')
  assert car.policy.camera_signal is None






def test_a_reds_stop_driven_through_under_the_gas_keeps_its_overrun(monkeypatch):
  # a red read made the stop: the gas never makes that the driver's go. Lifted where the stop no longer fits short of the
  # crossing lanes (15 m out at 11 m/s), the car goes through and says so, never braked at into the junction
  ev = Events(monkeypatch)
  car = right_at_a_signal(v=11.)
  rows = gas_until(car, LINE - 15., lights=cam('red'))
  assert 'cannotStopComfortably' in {r[3] for r in rows} and min(r[2] for r in rows) >= 0.
  assert [e['why'] for e in ev.named('turnpilot.map_signal_decided') if e['decision'] == 'go'] == ['driven']
  assert not ev.named('turnpilot.signal_turn_driven')






def test_a_camera_red_at_a_straight_mapped_signal_is_released_by_a_steady_camera_green():
  car = Car([(LINE, 'signal', True)])
  held_at(car, LINE, lights=cam('red'))
  car.run(10., lights=cam('red'))
  out, dec, _ = car.run(CAMERA_GREEN_STEADY_S - 0.1, lights=cam('green'))
  assert held(out, dec)
  out, dec, _ = car.run(0.2, lights=cam('green'))
  assert not dec.should_stop and out.prompt == 'none'
  car.run(10., lights=cam('green'))
  assert car.s > JUNCTION + 20.


# --- a colourless head at the route turn's unmapped junction: two road runs (review 2026-11-06) ---

def road_runs(runs, s_head=HEAD):
  """turnpilotd's camera input (camera_lights) for the reader's road runs [(car time, state or None: a run that read no
  head)]: each run's read made once (its own CameraLights.run) with the head at road position s_head as the car stood then,
  carried unchanged until the next run, gone CAMERA_STALE_S after it."""
  reads = {}

  def make(car):
    due = [r for r in runs if r[0] <= car.t + 1e-9]
    if not due or car.t - due[-1][0] > CAMERA_STALE_S + 1e-9:
      return NO_LIGHTS
    t, state = due[-1]
    if t not in reads:
      reads[t] = Lights(camera=CameraLights([] if state is None else [(state, s_head - car.s)], False, 'read'))
    return reads[t]
  return make


def unmapped_right(runs, v=13.4):
  """A right at a junction OSM has no signal for, 120 m ahead at t=0, the reader's road runs as road_runs's."""
  car = Car([], v=v)
  car.s = JUNCTION - 120.
  car.junction = (JUNCTION, 7)
  return car, road_runs(runs)


def test_one_road_run_of_a_colourless_head_makes_no_stop(monkeypatch):
  # one run's unknown box (a streetlight, a sign, a head misjudged), carried CAMERA_STALE_S as turnpilotd carries it: no
  # light at the junction, no stop and no handback. Before: one frame made a stop and a loud handback
  ev = Events(monkeypatch)
  car, lit = unmapped_right([(0.5, 'unknown')])
  car.run(20., lights=lit)
  assert car.policy.camera_signal is None and not ev.named('turnpilot.camera_signal_added')
  assert car.s > JUNCTION + 50. and min(r[2] for r in car.rows) >= 13.4 - 1e-6 and not any(r[5].stops for r in car.rows)


def test_one_road_run_of_a_camera_red_still_stops_the_car():
  # a red or a yellow needs no count (light_state's vote spans VOTE_N runs already): one run's red is the camera's signal
  car, lit = unmapped_right([(0.5, 'red')])
  car.run(1., lights=lit)
  assert car.policy.camera_signal is not None




def test_two_road_runs_further_apart_than_the_gap_make_no_stop():
  # the first run's read gone, then TURN_HEAD_GAP_S and more with no head there: the count starts again, one run, no stop
  car, lit = unmapped_right([(0.5, 'unknown'), (0.5 + CAMERA_STALE_S + TURN_HEAD_GAP_S + 0.3, 'unknown')])
  car.run(20., lights=lit)
  assert car.policy.camera_signal is None and car.s > JUNCTION + 50. and not any(r[5].stops for r in car.rows)




SET_BACK = JUNCTION - 25.  # 1e maneuver 14's shape: a mapped line 25 m back of the right's junction...
FAR_HEAD = JUNCTION + 10.  # ...its far-side head 35 m past that line



def every_run(t0, seconds, step=0.15, state='unknown'):
  """The reader's road runs from car time t0 for seconds, step apart, each reading state (road_runs's)."""
  return [(round(t0 + i * step, 3), state) for i in range(int(seconds / step))]


def set_back_right(v, s0):
  car = Car([(SET_BACK, 'signal', True, False)], v=v)
  car.junction = (JUNCTION, 7)
  car.s = s0
  return car


def far_head_from(s_from, state='unknown', s_head=FAR_HEAD):
  """No light until the car is at s_from, then the far-side head read state (unknown: the night) in road runs from there."""
  runs = {}

  def make(car):
    if car.s < s_from:
      return NO_LIGHTS
    if 'f' not in runs:
      runs['f'] = road_runs(every_run(car.t + 0.05, 8., state=state), s_head=s_head)
    return runs['f'](car)
  return make








def test_a_junction_read_again_only_once_its_mapped_signal_is_done_makes_no_camera_line(monkeypatch):
  # navd's gap (no junction) from far out until the car is past the mapped line at 6 m/s: the signal went by as a straight
  # one's (the model's), done. The junction read again 22 m short of it, the far-side head unknown in every road run: a
  # mapped signal done but still in the junction's window is the junction's, never a second line 15 m past it
  ev = Events(monkeypatch)
  car = set_back_right(6., SET_BACK - 60.)
  car.cruise = 6.
  lit = road_runs(every_run(0., 30.), s_head=FAR_HEAD)
  junction = lambda car: (7, JUNCTION - car.s) if car.s > SET_BACK + OVERRUN_PAST_M + 0.5 else None  # noqa: E731
  for _ in range(int(30. / DT_MDL)):
    car.step(junction=junction, lights=lit)
    if car.s > JUNCTION + 10. or car.v < 0.05:
      break
  assert car.s > JUNCTION + 10. and car.policy.camera_signal is None and not ev.named('turnpilot.camera_signal_added')






def drive_through(car, lit, gas_to=None, seconds=30.):
  """openpilot engaged, the driver's gas holding the car's speed until gas_to (None: no gas), then the plan: the alerts said,
  until the car is through the junction or stands."""
  alerts = []
  for _ in range(int(seconds / DT_MDL)):
    gas = gas_to is not None and car.s < gas_to
    out, dec, cmd = car.step(gas=gas, accel=0. if gas else None, engaged=True, lights=lit)
    alerts.append(out.alert)
    if car.s > JUNCTION + 10. or (not gas and car.v < 0.05):
      break
  return alerts


def no_second_line(ev, car, alerts, after_s, refused=None):
  """Through the junction past its set-back line: no camera line, no braking; refused: the red refused there said (why),
  once (turnpilot.camera_red_refused), or None for none read."""
  assert car.s > JUNCTION + 10., car.s
  assert car.policy.camera_signal is None and not ev.named('turnpilot.camera_signal_added')
  assert not {'cannotStopComfortably', 'stopOverrun'} & set(alerts) and not ev.named('turnpilot.map_stop_overrun')
  assert min(r[3] for r in car.rows if r[1] > after_s) > -0.5
  if refused is not None:
    said = ev.named('turnpilot.camera_red_refused')
    assert [k['why'] for k in said] == [refused] and said[0]['error'], said






@pytest.mark.parametrize("red_from", [OVERRUN_PAST_M + 1., 0.3], ids=['once_done', 'just_over_the_line'])
def test_a_green_go_at_a_set_back_line_then_a_red_on_the_junctions_head_makes_no_stop(monkeypatch, red_from):
  # a steady camera green read on the way in lets the turn go at 6 m/s; the far-side head then reads red, from just over the
  # line (the mapped signal still live, crossed on its go) or once it is done. Before: just over the line the red re-armed
  # the crossed line's own stop (-2.0 for a few frames, then stopOverrun); once done, the camera's own line 15 m on held the
  # car short of the junction after its own green
  ev = Events(monkeypatch)
  car = set_back_right(6., SET_BACK - 30.)
  car.cruise = 6.
  green, red = cam('green', s_head=FAR_HEAD), far_head_from(SET_BACK + red_from, state='red')
  alerts = drive_through(car, lambda car: green(car) if car.s < SET_BACK else red(car))
  no_second_line(ev, car, alerts, SET_BACK - 5., refused='passed')
  assert car.policy.signal_went(JUNCTION - car.s) or car.policy.tracker.controls.get((1, 0)) is None




def test_a_red_on_a_set_back_lines_far_head_out_of_its_window_is_its_red(monkeypatch):
  # the same line and head, the red read all the way in: the mapped line's red stop ("Red light"), held at the line with no
  # handback while the red is read, never the turn's want of a green, which hands back in TURN_UNSEEN_S. A red its live line
  # stops for is never said refused (turnpilot.camera_red_refused)
  ev = Events(monkeypatch)
  line = JUNCTION - 26.
  car = Car([(line, 'signal', True)], v=13.4)
  car.junction = (JUNCTION, 7)
  out, dec = held_at(car, line, lights=cam('red'))
  st = car.policy.tracker.controls[(1, 0)].state
  assert st.light == 'red' and not st.turn_stop and dec.reason.startswith('Red light')
  out, dec, _ = car.run(TURN_UNSEEN_S + 1., lights=cam('red'))
  assert held(out, dec) and out.prompt == 'none' and car.policy.camera_signal is None
  assert not ev.named('turnpilot.camera_red_refused')


def test_a_red_on_a_lines_own_head_is_never_outvoted_by_another_head_of_the_junction():
  # the set-back line's own head (inside its window) red, the far-side head (the junction's own line's) green: two phases,
  # a red arrow over the right lane and a green ball. The line's own red stands (the light's stop, "Red light"), never the
  # junction's disagreeing read (unknown: the turn's want of a green alone)
  line = JUNCTION - 26.
  car = Car([(line, 'signal', True)], v=13.4)
  car.junction = (JUNCTION, 7)
  near_head, far_head = line + 4., HEAD
  assert far_head - line > CAMERA_ASSOC_PAST_M and near_head < JUNCTION - CONTROL_LINE_M - map_controls.CAMERA_ASSOC_BEFORE_M
  lit = lambda car: Lights(camera=CameraLights([('red', near_head - car.s), ('green', far_head - car.s)], False, 'read'))  # noqa: E731
  out, dec = held_at(car, line, lights=lit)
  st = car.policy.tracker.controls[(1, 0)].state
  assert st.light == 'red' and not st.turn_stop and dec.reason.startswith('Red light')
  out, dec, _ = car.run(CAMERA_GREEN_STEADY_S + 1., lights=lit)
  assert held(out, dec)  # the junction's green never releases a red its own head still reads


def test_a_camera_red_on_the_frame_the_car_crosses_a_line_on_its_go_adds_nothing():
  # _camera_adds runs before the crossing is recorded (crossed_v): the crossing frame itself counts, so a red read just as the
  # car goes over the line on its go never re-arms that line's stop
  pol = map_controls.MapControlPolicy()
  over, short = TrackedControl((1, 0), 'signal', -0.2), TrackedControl((1, 0), 'signal', 0.2)
  assert pol._camera_adds(over, ControlState(mode='go'), 'red', 6.) == 'unknown'  # crossing now at 6 m/s: gone through
  assert pol._camera_adds(over, ControlState(mode='go'), 'red', 1.5) == 'red'  # a creep over in a queue: held there (past_red)
  assert pol._camera_adds(short, ControlState(mode='go'), 'red', 6.) == 'red'  # short of it: a red ahead stops the car
  went = ControlState(mode='go', crossed_v=6.)
  assert pol._camera_adds(TrackedControl((1, 0), 'signal', -1.5), went, 'red', 0.5) == 'unknown'  # slowed after going


def test_the_next_turns_junction_past_a_mapped_one_still_makes_its_camera_line_on_a_red(monkeypatch):
  # the gate is the maneuver's: past the mapped junction (maneuver 7, its green go), the route's next turn (maneuver 8) at a
  # junction OSM has no signal for, 150 m on, reads red there: the camera's own line, the car held at it
  ev = Events(monkeypatch)
  car = set_back_right(8., SET_BACK - 40.)
  nxt = JUNCTION + 150.
  junction = lambda car: (7, JUNCTION - car.s) if car.s < JUNCTION + 5. else (8, nxt - car.s)  # noqa: E731
  green, red = cam('green', s_head=FAR_HEAD), cam('red', s_head=nxt + 10.)
  lit = lambda car: green(car) if car.s < SET_BACK + 5. else red(car) if car.s > JUNCTION + 40. else NO_LIGHTS  # noqa: E731
  out, dec = car.run_until(held, junction=junction, lights=lit)
  assert -2. < nxt - CONTROL_LINE_M - car.s < 5. and car.policy.camera_signal.key[1] == 8
  assert [k['key'][1] for k in ev.named('turnpilot.camera_signal_added')] == [8]


# --- the junction's mapped signal on most frames' evidence, never one frame's (review 2026-11-06) ---

PREV = JUNCTION - 34.  # 1e maneuver 12's and 1d maneuver 5's shape: the previous junction's signal 34 m back of the turn's


def jittered(frames=(40,), by=6.):
  """update()'s junction for maneuver 7, read by m nearer on the given frames (route_follower's junction distance jitters
  1.4-6.8 m against the tracker's lines over one approach, 2026-11-06)."""
  n = {'k': 0}

  def junction(car):
    n['k'] += 1
    return (7, JUNCTION - car.s - (by if n['k'] in frames else 0.))
  return junction


def head_then(s_until, first, then):
  """The camera reading head (state, road position) first until the car is at s_until, then head then."""
  def make(car):
    state, s_head = first if car.s < s_until else then
    return Lights(camera=CameraLights([(state, s_head - car.s)], False, 'read'))
  return make


def test_the_previous_junctions_signal_read_once_in_the_window_never_keeps_the_turns_red_from_stopping_the_car(monkeypatch):
  # the previous junction's signal 34 m back of the right's junction, the right's junction one OSM has no signal for. The
  # junction read 6 m nearer for one frame 54 m out: the signal in its window that frame alone. Its own head green, the car
  # through it at 8 m/s; the junction's own head (12 m past it) then red, its line about 21 m ahead: the camera's own line,
  # the car stopped there gently. Before: that one frame made the signal the turn's and the junction a mapped signal's for
  # the maneuver; through it on its green, the red stopped nothing, said nowhere, and the car drove the right through it
  ev = Events(monkeypatch)
  car = Car([(PREV, 'signal', True)], v=8.)
  car.cruise = 8.
  car.s = PREV - 70.
  lit = head_then(PREV + 3., ('green', PREV + 20.), ('red', JUNCTION + 12.))
  out, dec = car.run_until(lambda out, dec: held(out, dec) and car.s > PREV + 5., junction=jittered(), lights=lit)
  assert -2. < JUNCTION - CONTROL_LINE_M - car.s < 5. and car.policy.camera_signal.key == (map_controls.CAMERA_ROUTE, 7)
  assert dec.reason.startswith('Red light') and all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)
  assert [k['camera'] for k in ev.named('turnpilot.camera_signal_added')] == ['red']
  assert [1, 0] not in [k['key'] for k in ev.named('turnpilot.signal_turn')]  # never the turn's
  assert not ev.named('turnpilot.camera_red_refused')
  rec = car.policy._junction_lines[7][(1, 0)]
  assert rec.inside == 1 and rec.passed and not rec.governs


def test_the_previous_junctions_signal_read_once_in_the_window_is_never_the_turns_stop(monkeypatch):
  # the same frame of jitter, nothing read at all (the night): the signal 34 m back is the junction the route goes straight
  # through, the model's, and the car carries on through it. Before: that one frame marked it the turn's, a stop for want of
  # a green 34 m short of the turn's junction, the car held there and handed back (signalUnseenTakeControl)
  ev = Events(monkeypatch)
  car = Car([(PREV, 'signal', True)], v=8.)
  car.cruise = 8.
  car.s = PREV - 70.
  junction = jittered()
  for _ in range(int(15. / DT_MDL)):
    out, dec, _ = car.step(junction=junction)
    assert not held(out, dec) and out.prompt == 'none'
    if car.s > PREV + 10.:
      break
  assert car.s > PREV + 10. and min(r[2] for r in car.rows) >= 8. - 1e-6
  assert not ev.named('turnpilot.signal_turn') and car.policy.tracker.controls[(1, 0)].state.turn_id is None


@pytest.mark.parametrize("setback,s_head", [(25., FAR_HEAD), (29., JUNCTION + 14.)], ids=['set_back_25m', 'within_29m'])
def test_a_red_refused_past_a_line_that_governs_the_junction_is_said(monkeypatch, setback, s_head):
  # a mapped line 25 m back of the right's junction (1e maneuver 14's shape), or 29 m back (in the window too: a wide
  # junction's line, or a mid-block crossing's or a near junction's signal, can't be told apart by where it is), green, the
  # car through it at 6 m/s; then a red on the junction's own head with the car short of the junction's line. The line
  # governs the junction, so the car clears it (no second line, no braking), and the red refused is said, once, as an
  # error (turnpilot.camera_red_refused). Before: driven through, said nowhere
  ev = Events(monkeypatch)
  line = JUNCTION - setback
  car = Car([(line, 'signal', True, False)], v=6.)  # MapControl.junction false, as the 11-06 set-back lines read
  car.cruise = 6.
  car.junction = (JUNCTION, 7)
  car.s = line - 30.
  alerts = drive_through(car, head_then(line + 1., ('green', line + 5.), ('red', s_head)))
  no_second_line(ev, car, alerts, line - 5.)
  refused = ev.named('turnpilot.camera_red_refused')
  assert len(refused) == 1 and refused[0]['error'] and refused[0]['why'] == 'passed', refused
  assert refused[0]['key'] == [map_controls.CAMERA_ROUTE, 7] and refused[0]['d'] > 0.
  assert [k for k, *_ in refused[0]['lines']] == [[1, 0]]


def test_a_red_on_a_skipped_turns_set_back_lines_far_head_out_of_its_window_stops_the_car_at_that_line(monkeypatch):
  # S1 (review 2026-11-07): a turn not driven (going straight on the model) at a mapped line 26 m back, the junction's far-side
  # head 41 m past it, out of the line's own window, red from far out: the line governs the junction unmarked, and its far
  # head's red is its red, a stop at its own line. Before: the car went through at 13.4 m/s, said only in the log
  ev = Events(monkeypatch)
  line = JUNCTION - 26.
  assert HEAD - line > CAMERA_ASSOC_PAST_M
  car = Car([(line, 'signal', True)], v=13.4)
  car.s = line - 150.
  for _ in range(int(30. / DT_MDL)):
    car.step(lights=cam('red'), junction=lambda car: (7, JUNCTION - car.s, False))
  assert line - 3. < car.s <= line + 0.5 and car.v < 0.1 and car.policy.camera_signal is None
  assert not ev.named('turnpilot.camera_red_refused')


def test_a_set_back_line_out_of_the_window_on_a_third_of_its_frames_still_governs_the_junction(monkeypatch):
  # a turn not driven (no mark: the window's evidence alone), a mapped line 28 m back of its junction, the junction read 4 m
  # further on every third frame (out of the window then: 1d maneuver 3's line flapped out 15 times). Gone through on its
  # green at 6 m/s, a red then on the far-side head: the line governs the junction on most of its frames, no camera line
  # 18 m past it, no braking into the junction's approach
  ev = Events(monkeypatch)
  line = JUNCTION - 28.
  car = Car([(line, 'signal', True, False)], v=6.)
  car.cruise = 6.
  car.s = line - 60.
  n = {'k': 0}

  def junction(car):
    n['k'] += 1
    return (7, JUNCTION - car.s + (4. if n['k'] % 3 == 0 else 0.), False)
  lit = head_then(line + OVERRUN_PAST_M + 1., ('green', line + 5.), ('red', FAR_HEAD))
  alerts = []
  for _ in range(int(30. / DT_MDL)):
    out, dec, _ = car.step(engaged=True, lights=lit, junction=junction)
    alerts.append(out.alert)
    if car.s > JUNCTION + 10. or car.v < 0.05:
      break
  no_second_line(ev, car, alerts, line - 5.)
  rec = car.policy._junction_lines[7][(1, 0)]
  assert 0 < rec.n - rec.inside < rec.inside and rec.governs and not rec.marked


def test_a_junctions_signal_lost_unpassed_governs_nothing(monkeypatch):
  # the right's junction's mapped signal (8 m short of it) listed, then no longer (a reroute's controls, navd's lookup) with
  # the car 100 m short of it and nothing read yet; then the junction's head reads red: the camera's own line, the car held
  # at it. Before: once seen in the window the junction was a mapped signal's for the maneuver, lost or not, and the red
  # stopped nothing
  ev = Events(monkeypatch)
  car = right_at_a_signal(v=11.)
  car.s = LINE - 200.
  lit = lambda car: cam('red')(car) if car.s > LINE - 80. else NO_LIGHTS  # noqa: E731
  while car.s < LINE - 100.:
    car.step()
  car.controls = []
  out, dec = car.run_until(held, lights=lit)
  assert -2. < JUNCTION - CONTROL_LINE_M - car.s < 5. and car.policy.camera_signal.key == (map_controls.CAMERA_ROUTE, 7)
  assert car.policy.tracker.controls.get((1, 0)) is None and all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)
  assert [k['camera'] for k in ev.named('turnpilot.camera_signal_added')] == ['red']


# --- the queue, a late junction, stop_ahead ---



def test_a_turn_signal_too_close_to_stop_for_is_said_and_never_braked_at_the_floor(monkeypatch):
  ev = Events(monkeypatch)
  car = Car([(LINE, 'signal', True)], v=13.4)
  car.s = LINE - 25.  # a reroute's right at a light 25 m ahead at 30 mph: -3.6 m/s^2 to stop at the line
  car.junction = (JUNCTION, 7)
  out, dec, _ = car.run(2.)
  assert car.policy.tracker.controls[(1, 0)].state.turn_id is None and all(r[3] >= -1. for r in car.rows)
  late = ev.named('turnpilot.signal_turn_late')
  assert len(late) == 1 and late[0]['maneuver'] == 7






def test_signal_turn_module_docstring_names_the_rules():
  doc = map_controls.__doc__
  for word in ('A route turn at a signal', 'TURN_UNSEEN_S', 'CAMERA_GREEN_STEADY_S', 'turn_stop', 'driving_turn', 'TURN_GREEN_STALE_S',
               'the window picks the first mark only', 'TURN_HEAD_RUNS', 'signal_turn_driven', 'put off while a camera green',
               'throughLanesDistance', 'driver_go', 'Only provisional', 'signal_turn_go_revoked', '_junction_lines', '_turn_camera',
               'whatever the head reads, red too', 'a red ahead of a line not yet passed stops the', 'crossed_v',
               'JunctionLine.governs', 'most of the maneuver\'s frames', 'One lost unpassed', 'camera_red_refused'):
    assert word in ' '.join(doc.split()), word


# --- through turnpilotd ---

def turn_sim(maneuvers, state=None, s_head=310., signal=290.):
  """turnpilotd driving maneuvers with a mapped signal at signal (None: none), the camera (add-only) reading a governing head
  at s_head every other frame in state(sim) (a state or a function of the sim; None: no head at all)."""
  from openpilot.cereal import messaging
  from openpilot.selfdrive.turnpilot.tests.test_camera_lights import detections
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
  sim = Sim(maneuvers, controls=[(signal, 'signal')] if signal is not None else ())
  sim.go_taps = False
  sim.tp.camera_mode = 'addOnly'
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    s = state(sim) if callable(state) else state
    if s is not None and int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', detections(s, s_head - sim.s, shadow=False))
    return none
  sim.lights = lights
  return sim


def right_turn():
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import junction_of
  return [(300., "turn", "right", junction_of("right")), (1500., "turn", "left", junction_of("left"))]








def test_turnpilotd_a_signal_the_route_goes_straight_through_is_unchanged():
  # the same signal with the route's turn 1 km on: nothing read, nothing slows the car (the model's own red handling)
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import junction_of
  sim = turn_sim([(1300., "turn", "right", junction_of("right")), (2500., "turn", "left", junction_of("left"))])
  sim.run(30.)
  assert sim.s > 330. and not any(p.shouldStop for _, _, p in sim.trace)
  assert min(v for s, v, _ in sim.trace if 250. < s < 300.) >= 12.


def test_turnpilotd_a_right_skipped_on_the_approach_goes_straight_through_its_signal():
  # review 2026-11-06: a right at a mapped signal skipped 150 m out (laneBlocked), nothing read (night). The route goes
  # straight here now and the signal is the model's, as any straight one's (driving_turn: update()'s junction's third field,
  # turnpilotd's `driving and not alt_replaced`). With turnpilotd's two-field junction every TURN maneuver counted as driven:
  # the car stopped at the line ("Turn at a light: stopping"), handed back, and stood there held 90 s on a straight approach.
  # This fails until turnpilotd passes the third field (the lefts package's turnpilotd.py, the `junction = ...` line before
  # map_update): the two land together
  sim = turn_sim(right_turn())
  seen = []
  update = sim.tp.map_controls.update

  def spy(*a, **kw):
    seen.append(kw.get('junction'))
    return update(*a, **kw)
  sim.tp.map_controls.update = spy
  skipped = False
  while sim.s < 340. and sim.t < 60.:
    rf = sim.tp.follower
    if not skipped and rf.upcoming is not None and 300. - sim.s < 150.:
      sim.tp.skip.skip(rf.upcoming, rf.distance, rf.maneuver_id, sim.v, 'laneBlocked')
      skipped = True
    sim.step(engaged=True)
  turns = [j for j in seen if j is not None]
  assert turns and all(len(j) >= 3 for j in turns), \
    "turnpilotd passes a two-field junction: a skipped turn's signal stops the car (lefts package: pass driving and not alt_replaced)"
  plans = [p for _, _, p in sim.trace]
  assert skipped and sim.s >= 340., sim.s
  assert not any(p.shouldStop for p in plans)
  assert all(str(p.controlPrompt) == 'none' for p in plans)


def test_turnpilotd_the_drivers_gas_through_a_rights_stop_lifted_near_the_line_is_their_go():
  # 1e 5:37 (2026-11-06): the driver pressed through a right's stop for want of a green (the press disengaging), openpilot was
  # engaged again with the gas still down, and the gas came off 22 m short of the line at 9.6 m/s (-2.1 m/s^2 to stop there).
  # Before: the plan then braked at the -2.0 floor into the line and said stopOverrun. Through turnpilotd: no braking, no
  # alert, no prompt, the car on through the junction
  from openpilot.selfdrive.turnpilot.hooks import plan_accel
  sim = turn_sim(right_turn())
  gas = {'on': False}

  def accel(sim, plan):
    if gas['on']:
      return 0.  # the driver's foot holding the speed: the plan overridden
    a = min(plan_accel(plan.accelCap, plan.speedCap, sim.v), (sim.cruise - sim.v) / 2., 1.5)
    return min(a, -1.) if plan.shouldStop else a
  sim.accel = accel
  lifted = None
  while sim.s < 340. and sim.t < 60.:
    gas['on'] = 200. < sim.s < 270.
    sim.step(gas=gas['on'], engaged=not 200. < sim.s < 265.)
    if lifted is None and sim.s >= 270.:
      lifted = len(sim.trace)
  after = [p for _, _, p in sim.trace[lifted:]]
  assert sim.s >= 340. and min(v for _, v, _ in sim.trace[lifted:]) > 12.
  assert not any(p.shouldStop or p.accelCap < -0.5 for p in after)
  assert not any(str(p.controlAlert) == 'stopOverrun' for _, _, p in sim.trace)
  assert all(str(p.controlPrompt) == 'none' for _, _, p in sim.trace)






def test_a_red_read_too_late_for_its_line_is_not_said_where_the_lefts_own_hold_stops_the_car_just_past_it():
  """Route 29's late red at a left: the car was braking for the left's hold in the junction's mouth when the red was first
  read, too late for the signal's own line short of it. It stops at the mouth all the same, so cannotStopComfortably ("can't
  stop") over a stop the car is making is noise; with no hold of the left's past the line it is still said."""
  line = JUNCTION - 22.  # a line set back of the junction: the left's hold is in the mouth, 16 m past it

  def run(with_left):
    car = Car([(line, 'signal', True)], v=11., cruise=11.)
    if with_left:
      car.turn = (JUNCTION, 7, 'left')
    car.junction = (JUNCTION, 7)
    kw = {'left': LeftCommand('auto')} if with_left else {}
    alerts, late_v = set(), None
    for _ in range(int(40. / DT_MDL)):
      late = line - car.s < 2.
      if late and late_v is None:
        late_v = car.v
      out, dec, _ = car.step(lights=cam('red', s_head=JUNCTION + 15.) if late else NO_LIGHTS, **kw)
      alerts.add(out.alert)
      if (held(out, dec) and car.v < 0.1) or car.s > JUNCTION + 20.:
        break
    return car, out, dec, alerts, late_v
  car, out, dec, alerts, late_v = run(True)
  assert not map_controls.red_stop_fits(late_v, 2.)  # too late for the signal's own line
  assert held(out, dec) and line + 10. < car.s < JUNCTION - 4.  # stood in the mouth, at the left's hold
  assert not {'cannotStopComfortably', 'stopOverrun'} & alerts
  car, out, dec, alerts, late_v = run(False)
  assert car.s > JUNCTION and 'cannotStopComfortably' in alerts


# --- a curb-side turn at a signal: through on a green, the right-on-red procedure on a red or an unread light ---

def right_in_hand(clear=True, v=13.4):
  car = right_at_a_signal(v=v)
  car.right = clear
  return car


@pytest.mark.parametrize("light", [NO_LIGHTS, cam('unknown')], ids=['noRead', 'unknown'])
def test_a_right_at_a_signal_nobody_reads_stops_at_the_line_looks_and_goes(light, monkeypatch):
  """Routes 29 and 26: three rights at signals the reader boxed no head of, the lamps red in the video, the driver standing
  12-25 s at each; the planner drove each at the corner's speed, and on 11-06 ran a right on red so. A right flows through
  only on a green. On an unread light: a stop at the line, a stand of RIGHT_RED_DWELL_S, then its go once the way has been
  clear RIGHT_RED_CLEAR_S, never a handback."""
  ev = Events(monkeypatch)
  car = right_in_hand()
  out, dec = held_at(car, LINE, lights=light)
  st = car.policy.tracker.controls[(1, 0)].state
  assert out.prompt == 'none' and out.stops[0].reason == RIGHT_RED_LOOKING_REASON and car.policy.stop_ahead() == 0.
  assert not car.policy.right_on_red(JUNCTION - car.s)
  t_held = car.t
  out, dec = car.run_until(lambda out, dec: not held(out, dec), max_s=10., lights=light)
  assert RIGHT_RED_DWELL_S - 2 * DT_MDL <= car.t - t_held <= RIGHT_RED_DWELL_S + 0.2
  assert st.done == 'released' and st.cause == 'rightOnRed' and car.policy.right_on_red(JUNCTION - car.s)
  assert [e for e in ev.named('turnpilot.map_stop_released') if e['cause'] == 'rightOnRed']
  for _ in range(int(8. / DT_MDL)):  # on through the junction: never stopped again, nothing said
    out, dec, _ = car.step(lights=light)
    assert not out.stops and out.prompt == 'none' and out.alert == 'none'
  assert car.s > JUNCTION and car.policy.stop_ahead() is None
  assert not ev.named('turnpilot.signal_unseen')


def test_a_right_never_goes_on_a_red_it_is_reading_and_goes_after_its_look_once_none_is_read(monkeypatch):
  """The owner's standing line: "it should always stop on red". The look is the forward cameras' tracks (it misses some
  crossing cars) and no "no turn on red" sign is read, so a right at a red that stays read stands for as long as it is read,
  its way clear the whole time, its line saying it waits for the green; nothing is said or handed back. Once no red or yellow
  has been read for RIGHT_RED_UNREAD_S (the reader's dropouts are shorter) it goes on its look."""
  ev = Events(monkeypatch)
  car = right_in_hand()
  out, dec = held_at(car, LINE, lights=cam('red'))
  st = car.policy.tracker.controls[(1, 0)].state
  for i in range(int(40. / DT_MDL)):
    # the reader dropping the red for 1.3 s at a time, as on the logs, is still the red
    light = cam('unknown') if 200 <= i % 400 < 200 + int(1.3 / DT_MDL) else cam('red')
    out, dec, _ = car.step(lights=light)
    assert held(out, dec) and out.prompt == 'none' and out.alert == 'none' and out.stops[0].reason == RIGHT_RED_WAIT_REASON
    assert not car.policy.right_on_red(JUNCTION - car.s)
  assert not st.done and not ev.named('turnpilot.signal_unseen')
  t_unread = car.t
  out, dec = car.run_until(lambda out, dec: not held(out, dec), max_s=10., lights=cam('unknown'))
  assert RIGHT_RED_UNREAD_S - 2 * DT_MDL <= car.t - t_unread <= RIGHT_RED_UNREAD_S + 0.2
  assert st.done == 'released' and st.cause == 'rightOnRed' and car.policy.right_on_red(JUNCTION - car.s)


def test_a_right_at_a_read_red_goes_on_its_green():
  car = right_in_hand()
  out, dec = held_at(car, LINE, lights=cam('red'))
  for _ in range(int(10. / DT_MDL)):
    out, dec, _ = car.step(lights=cam('red'))
    assert held(out, dec)
  out, dec = car.run_until(lambda out, dec: not held(out, dec), max_s=5., lights=cam('green'))
  assert not held(out, dec) and car.t < 60.
  for _ in range(int(8. / DT_MDL)):
    out, dec, _ = car.step(lights=cam('green'))
    assert not out.stops
  assert car.s > JUNCTION


def test_a_right_on_a_steady_green_flows_through_unslowed():
  base, car = Car([(LINE, 'signal', True)]), right_in_hand()  # base: the same signal going straight
  for _ in range(int(25. / DT_MDL)):
    a, b = base.step(), car.step(lights=cam('green'))
    assert a[0].stops == b[0].stops == [] and a[2] == b[2]
  assert car.s > JUNCTION


def test_a_right_held_by_what_is_seen_waits_for_it_and_goes_once_it_is_clear():
  """A crossing vehicle or a person seen holds the right's go for as long as it is seen (turnpilotd's look: update()'s
  right): the one wait there is, said on its line, never a take-control, never the unseen-light chime; and heads in view
  that read no red never let its stop go as a red gone (RED_GONE_S), the light unread as it is."""
  car = right_in_hand(clear=False)
  out, dec = held_at(car, LINE, lights=cam('unknown'))
  for _ in range(int(40. / DT_MDL)):
    out, dec, _ = car.step(lights=cam('unknown'))
    assert held(out, dec) and out.prompt == 'none' and out.alert == 'none'
    assert out.stops[0].reason == "Right on red: waiting for cross traffic"
  car.right = True
  out, dec, _ = car.run(RIGHT_RED_CLEAR_S - 0.2, lights=cam('unknown'))  # clear on end first: one clear frame is no go
  assert held(out, dec)
  car.right = False
  car.run(0.2, lights=cam('unknown'))
  car.right = True
  out, dec, _ = car.run(RIGHT_RED_CLEAR_S - 0.2, lights=cam('unknown'))
  assert held(out, dec)
  out, dec, _ = car.run(0.4, lights=cam('unknown'))
  assert not held(out, dec) and not out.stops


def test_a_right_whose_signal_is_first_known_too_close_to_stop_for_goes_with_the_model():
  car = right_in_hand(v=13.4)
  car.controls = []
  car.run_until(lambda out, dec: LINE - car.s < 15.)
  car.controls = [(LINE, 'signal', True)]  # listed late (a tile loaded late): 15 m out at 13.4 m/s is past any stop at -2.0
  alerts = set()
  for _ in range(int(6. / DT_MDL)):
    out, dec, _ = car.step()
    alerts.add(out.alert)
    assert not out.stops
  assert car.s > JUNCTION and alerts == {'none'}


def obj(cls, x, y, vx=0., vy=0., age=20, conf=0.8):
  from types import SimpleNamespace
  return SimpleNamespace(cls=cls, x=x, y=y, vx=vx, vy=vy, age=age, conf=conf, trackId=1)


def test_the_right_on_red_look_holds_for_what_the_scene_sees_and_nothing_else():
  from openpilot.selfdrive.turnpilot.map_controls import (RIGHT_RED_BLIND_REASON, RIGHT_RED_CROSS_REASON, RIGHT_RED_PERSON_REASON,
                                                           right_red_blocked)
  from openpilot.selfdrive.turnpilot.route_follower import RIGHT

  def look(objects, available=True, person=False):
    return right_red_blocked(objects, available, person, 10., RIGHT)
  assert look([]) == ''
  assert look([], available=False) == RIGHT_RED_BLIND_REASON  # no live scene is no look
  assert look([], person=True) == RIGHT_RED_PERSON_REASON
  assert look([obj('pedestrian', 12., -5.)]) == RIGHT_RED_PERSON_REASON  # in the turn's crosswalk
  assert look([obj('car', 14., 30., vy=-9.)]) == RIGHT_RED_CROSS_REASON  # closing from the left on the cross road
  assert look([obj('car', 14., 60., vy=-13.)]) == RIGHT_RED_CROSS_REASON
  assert look([obj('car', 14., -8., vy=-9.)]) == RIGHT_RED_CROSS_REASON  # crossing in the junction, just past the car's nose
  assert look([obj('cyclist', 12., 6., vy=-4.)]) == RIGHT_RED_CROSS_REASON
  assert look([obj('car', 14., 30., vy=0., age=3)]) == RIGHT_RED_CROSS_REASON  # on the cross road, too new to say where it goes
  assert look([obj('car', 14., 30., vy=0., age=40)]) == ''  # ...seen standing there: parked, or waiting to turn
  assert look([obj('car', 1., 3.5, vy=0., age=3)]) == ''  # a car beside this one at the same light
  assert look([obj('car', 14., -40., vy=-9., age=40)]) == ''  # driving away down the road the car turns into
  assert look([obj('car', 8., 0., vx=0., age=40)]) == ''  # a lead: the planner's own, and the go's in-path check


def test_turnpilotd_a_right_at_a_signal_nobody_reads_green_stops_looks_and_goes_certified_over_the_models_hold():
  """Through turnpilotd as the build ships (TurnPilotAlwaysDrive on): the right stops at its signal's line on an unread head
  or no head at all, stands for its look and is let go with no prompt and no alert; the model still holding the car there,
  TurnPilot's go is certified for plannerd's lift (turnRelease) on that look, never the no-move handback."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import stand
  for state in (None, 'unknown'):
    sim = turn_sim(right_turn(), state=state)
    sim.tp.always_drive = True
    while sim.t < 60. and not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05):
      sim.step(engaged=True)
    assert 286. < sim.s < 291. and str(sim.trace[-1][2].mapControl.kind) == 'signal', (state, sim.s)
    assert all(int(p.desire) == 0 for _, _, p in sim.trace)  # no turn desire on the way to the line
    plans = stand(sim, 6.)
    causes = [str(p.goCertified.cause) for p in plans]
    go = next(i for i, c in enumerate(causes) if c != 'none')
    assert causes[go] == 'turnRelease' and go * DT_MDL < RIGHT_RED_DWELL_S + 0.5, (state, go)
    assert not plans[go].shouldStop and all(p.shouldStop for p in plans[:go - 1])
    every = [p for _, _, p in sim.trace]
    assert all(str(p.controlPrompt) == 'none' and str(p.controlAlert) == 'none' for p in every), state
    assert sim.tp.map_controls.right_on_red(300. - sim.s)


def test_turnpilotd_a_right_at_a_red_that_stays_read_stands_with_nothing_certified():
  """Through turnpilotd: the right stops at its line on a read red and stands for as long as it is read, its way clear: no
  turn desire, no go certified (no lift over the model's hold at the red), no prompt and no alert."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import stand
  sim = turn_sim(right_turn(), state='red')
  sim.tp.always_drive = True
  while sim.t < 60. and not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05):
    sim.step(engaged=True)
  assert 286. < sim.s < 291. and str(sim.trace[-1][2].mapControl.kind) == 'signal', sim.s
  plans = stand(sim, 20.)
  assert all(p.shouldStop and str(p.goCertified.cause) == 'none' and int(p.desire) == 0 for p in plans)
  assert all(str(p.controlPrompt) == 'none' and str(p.controlAlert) == 'none' for _, _, p in sim.trace)
  assert not sim.tp.map_controls.right_on_red(300. - sim.s)


def test_turnpilotd_a_rights_certified_go_is_taken_back_when_a_car_is_seen_crossing():
  """The look goes on through the lift: the go from the line is certified on the look of each frame (right_on_red), so a
  car first seen coming along the cross road while the car is lifted on from its line ends the lift at once."""
  def run(cross):
    sim = turn_sim(right_turn(), state='unknown')
    sim.tp.always_drive = True
    while sim.t < 60. and not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05):
      sim.step(engaged=True)
    sim.model_holds, causes = True, []
    for _ in range(int(4. / DT_MDL)):
      sim.v = 0.
      seen = cross and 'turnRelease' in causes
      objects = ({'cls': 'car', 'x': 304. - sim.s, 'y': 25., 'vx': 0., 'vy': -8., 'age': 40, 'trackId': 7, 'conf': 0.8},) if seen else ()
      causes.append(str(sim.step(objects, engaged=True).goCertified.cause))
    return causes
  free, crossed = run(False), run(True)
  k = free.index('turnRelease')
  assert free[k:k + 10] == ['turnRelease'] * 10  # the lift stands while the road stays clear
  assert crossed[k] == 'turnRelease' and all(c == 'none' for c in crossed[k + 3:k + 20]), crossed[k:k + 20]


def test_turnpilotd_a_right_on_a_steady_camera_green_is_never_slowed_for_its_signal():
  sim = turn_sim(right_turn(), state='green')
  sim.tp.always_drive = True
  while sim.s < 320. and sim.t < 60.:
    sim.step(engaged=True)
  assert sim.s >= 320. and not any(p.shouldStop or str(p.mapControl.action) in ('braking', 'held') for _, _, p in sim.trace)

