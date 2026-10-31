"""MapControlPolicy: stop signs, yields and traffic signals from the map (map_controls.py), on a point-mass car."""
import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S, SPEED_CAP_ACCEL_MIN, plan_accel
from openpilot.selfdrive.turnpilot.map_controls import (COMMIT_SPEED, DRIVEN_THROUGH_M, STOP_SIGN_HANDBACK_S, GREEN_STEADY_S, LIGHT_STALE_S,
                                                         MAP_DECEL_MAX, NO_LIGHTS,
                                                         OFF_ROUTE_REASON, OFF_ROUTE_SPEED, T_LAT, UNPROTECTED, YIELD_SPEED, CameraLights,
                                                         Lights, MapControlPolicy, StopCommand, engage_distance, light_input, unprotected_turn)
from openpilot.selfdrive.turnpilot.route_follower import make_maneuver
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, NO_ACCEL_CAP, ScenePolicy, decide
from openpilot.selfdrive.turnpilot.tests.synthetic import nav_instruction


class Car:
  """A car on a straight road with map controls at s positions [(s, kind[, line_mapped, junction, cross_suspect])],
  navd's fixes once a second (offset by gps(t) m), following the policy's limits through ScenePolicy's composition."""
  def __init__(self, controls, v=13.4, cruise=None, route_count=1, status='ok', yellow_decel=2.0):
    self.controls = list(controls)
    self.policy = MapControlPolicy(yellow_decel)
    self.scene = ScenePolicy()
    self.s, self.v, self.t = 0., v, 0.
    self.cruise = v if cruise is None else cruise
    self.route_count, self.status = route_count, status
    self.frame = 0
    self.gps = lambda t: 0.
    self.nav_valid = True
    self.lead_stop: float | None = None  # a lead stopped here: the car stops 0.5 m short of it
    self.turn: tuple | None = None  # (s of the junction, maneuver id, 'left'): a far-side turn there, unprotected-shaped
    self.off_route = None  # off the route (nav_valid False): navd's lookup ahead, [(s, kind)], or None for none (no destination)
    self.off_status = 'ok'
    self.rows = []

  def nav(self):
    controls = [(i, c[1], c[0] - self.s + self.gps(self.t)) + tuple(c[2:]) for i, c in enumerate(self.controls)
                if -15. < c[0] - self.s < 400.][:8]
    off = [] if self.nav_valid or self.off_route is None else \
      [(i, c[1], c[0] - self.s) for i, c in enumerate(self.off_route) if -15. < c[0] - self.s < 400.]
    off_status = 'none' if self.nav_valid or self.off_route is None else self.off_status
    return nav_instruction(500., route_count=self.route_count, controls=controls if self.nav_valid else [],
                           controls_status=self.status, valid=self.nav_valid, off_route=off, off_route_status=off_status)

  def step(self, gas=False, lights=NO_LIGHTS, accel=None, **kw):
    """lights: Lights, or a function of the car giving them; kw: the stop-sign decider's for MapControlPolicy.update (release,
    creep, stop), each a value or a function of the car. openpilot is engaged (the plan applied) unless the driver's gas or
    accel (the plan not applied) says otherwise."""
    lights = lights(self) if callable(lights) else lights
    kw = {k: (v(self) if callable(v) else v) for k, v in kw.items()}
    kw.setdefault('engaged', not gas and accel is None)
    due = self.frame % 20 == 0
    self.frame += 1
    turn = None if self.turn is None else (self.turn[1], self.turn[0] - self.s, self.turn[2])
    out = self.policy.update(self.nav() if due else None, due, self.nav_valid, self.v, gas, lights, DT_MDL, turn=turn, **kw)
    dec = decide(out.stops, self.v)
    dec.accel_cap = self.scene.ramp(dec.accel_cap, DT_MDL)
    cmd = min(plan_accel(dec.accel_cap, dec.speed_cap, self.v), (self.cruise - self.v) / 2., 1.5)
    if dec.should_stop:
      cmd = min(cmd, -1.)
    if gas:
      cmd = 1.0 if accel is None else accel  # the driver's foot: openpilot's long plan is overridden
    elif accel is not None:
      cmd = accel
    if self.lead_stop is not None and self.s > self.lead_stop - 8. - self.v ** 2 / 3. and self.v > 0.:
      cmd = min(cmd, -self.v ** 2 / (2. * max(self.lead_stop - self.s - 0.5, 0.1)) - 0.3)
    self.v = max(self.v + cmd * DT_MDL, 0.)
    self.s += self.v * DT_MDL
    self.t += DT_MDL
    self.rows.append((self.t, self.s, self.v, cmd, dec, out))
    return out, dec, cmd

  def run(self, seconds, **kw):
    res = None
    for _ in range(int(round(seconds / DT_MDL))):
      res = self.step(**kw)
    return res

  def run_until(self, cond, max_s=60., **kw):
    for _ in range(int(max_s / DT_MDL)):
      out, dec, cmd = self.step(**kw)
      if cond(out, dec):
        return out, dec
    raise AssertionError("condition never met")


def test_the_junctions_mapped_stop_line_setback():
  """BUILD_FINDINGS D4: route time's reach margin for an unprotected left is the junction's mapped stop line's setback where
  the map has one (MapControlPolicy.line_setback: how far back of the hold line, HOLD_LINE_M + MAP_MARGIN_M before the
  junction, it lies), else unprotected_left.ROUTE_REACH_MARGIN_M. Only a line mapped (not the junction less 10 m), of the
  junction's own controls (PROTECT_BEFORE_M before to PROTECT_PAST_M past), never the cross road's yield."""
  from openpilot.selfdrive.turnpilot.map_controls import MAP_MARGIN_M
  hold = HOLD_LINE_M + MAP_MARGIN_M
  car = Car([(280., 'signal', True)], v=10.)
  car.run(1.)
  j = 300. - car.s
  assert math.isclose(car.policy.line_setback(j), 20. - hold, abs_tol=0.5)
  car = Car([(295., 'signal', True)], v=10.)  # a line nearer than the hold line: none back of it
  car.run(1.)
  assert car.policy.line_setback(300. - car.s) == 0.
  for controls in ([(280., 'signal', False)], [(200., 'signal', True)], [], [(280., 'yield', True, True, True, {'status': 'ok', 'crossGoverned': True})]):
    car = Car(controls, v=10.)
    car.run(1.)
    assert car.policy.line_setback(300. - car.s) is None, controls


def held(out, dec):
  return dec.should_stop and out.shown is not None and out.shown.action in ('held', 'awaitingDriver')


def drive_off(car, seconds=3.):
  """The driver takes over from a hold (a gas press disengages) and drives away: the plan isn't applied meanwhile."""
  return car.run(seconds, gas=True, accel=1.5)


# --- stop sign lifecycle ---

def test_stop_sign_lifecycle():
  car = Car([(250., 'stop')])
  engaged_at = None
  caps = []
  for _ in range(int(40. / DT_MDL)):
    out, dec, cmd = car.step()
    if engaged_at is None and out.stops:
      engaged_at = 250. - car.s
      assert engaged_at <= engage_distance(13.4) + 1.
      assert engaged_at > engage_distance(13.4) - 2.
    if out.stops and not dec.should_stop and car.v > 0.3:
      caps.append(out.stops[0].speed_cap)
    assert cmd >= MAP_DECEL_MAX - 1e-6  # never harder than -2.0
    if held(out, dec):
      break
  else:
    raise AssertionError("never held")
  assert all(b <= a + 1e-9 for a, b in zip(caps, caps[1:], strict=False))  # the cap never rises on the approach
  assert car.v < 0.3 and -2. < 250. - car.s < 5.
  assert out.reason == '' and 'stop sign' in dec.reason.lower()
  # the full stop, with no prompt yet, then the handback: held, take control
  out, dec, _ = car.run(STOP_SIGN_HANDBACK_S - 0.2)
  assert held(out, dec) and out.prompt == 'none'
  out, dec, _ = car.run(0.3)
  assert held(out, dec) and out.prompt == 'stopSignTakeControl'
  # the driver takes over and goes: released, and nothing left
  drive_off(car)
  out, dec, _ = car.run(3.)
  assert not out.stops and not dec.should_stop and car.s > 260.


def test_no_motion_while_handed_back():
  car = Car([(150., 'stop')])
  car.run_until(held)
  s_held = car.s
  car.run(30.)
  assert car.s - s_held < 0.05 and car.rows[-1][-1].prompt == 'stopSignTakeControl'


@pytest.mark.parametrize("v0", [11.2, 15.6])  # 25 and 35 mph
def test_stop_error_and_creep_with_gps_noise(v0):
  for sign in (-1., 1.):
    car = Car([(300., 'stop')], v=v0)
    car.gps = lambda t, sign=sign: sign * 3. * math.cos(t)  # +-3 m GPS error, swinging
    car.run_until(held)
    s_held = car.s
    assert -2. < 300. - car.s < 5.
    car.run(10.)
    assert car.s - s_held < 0.3


# --- the latch ---

def test_latch_survives_jumps_gaps_new_routes_and_invalid_nav():
  car = Car([(120., 'stop')])
  car.run_until(held)
  s_held = car.s
  car.gps = lambda t: 3.
  car.run(1.5)
  car.gps = lambda t: -3.
  car.run(1.5)
  car.nav_valid = False
  car.run(5.)
  car.nav_valid, car.route_count, car.controls = True, 2, []  # a reroute that doesn't list it
  out, dec, _ = car.run(1.5)
  assert held(out, dec) and car.s - s_held < 0.05
  # it still releases only by its rule: handed back long since, the driver drives off
  drive_off(car)
  out, dec, _ = car.step()
  assert not dec.should_stop


def test_driven_off_in_the_full_stop_under_the_gas_is_the_drivers():
  car = Car([(120., 'stop')])
  car.run_until(held)
  car.run(0.5)
  out, dec, _ = car.run(3., gas=True, accel=1.)  # the driver drives off before the handback
  assert not dec.should_stop
  assert not any(o.alert == 'stopOverrun' for *_, o in car.rows[-60:])
  out, dec, _ = car.run(3.)
  assert not dec.should_stop and car.v > 2.


# --- gas before the stop ---

def test_gas_before_the_stop_then_lift_past_the_line_is_an_overrun():
  car = Car([(150., 'stop')])
  car.run_until(lambda out, dec: out.stops)
  car.run_until(lambda out, dec: car.s > 151., gas=True, accel=0.5)
  out, dec, _ = car.step()
  assert not out.stops and out.alert == 'stopOverrun'
  out, dec, _ = car.run(1.)
  assert out.alert == 'stopOverrun' and not dec.should_stop


def test_gas_before_the_stop_then_lift_short_resumes_at_the_floor():
  car = Car([(150., 'stop')], v=10.)
  car.run_until(lambda out, dec: out.stops)
  car.run_until(lambda out, dec: car.s > 128., gas=True, accel=1.)  # 22 m short at about 14 m/s: needs about -4.4
  out, dec, cmd = car.step()
  assert out.alert == 'cannotStopComfortably' and out.stops[0].accel_cap == MAP_DECEL_MAX
  assert out.shown.action == 'infeasible'
  for _ in range(40):
    out, dec, cmd = car.step()
    assert cmd >= MAP_DECEL_MAX - 1e-6


def test_infeasible_at_first_sight_brakes_at_the_floor_and_alerts():
  car = Car([(20., 'stop')], v=15.)
  out, dec, cmd = car.step()
  assert out.stops[0].accel_cap == MAP_DECEL_MAX == SPEED_CAP_ACCEL_MIN
  assert out.alert == 'cannotStopComfortably' and out.shown.action == 'infeasible'
  cmds = [car.step()[2] for _ in range(30)]
  assert min(cmds) >= MAP_DECEL_MAX - 1e-6
  # it runs the line at the floor: an overrun, said out loud, never harder braking
  out, dec = car.run_until(lambda out, dec: out.alert == 'stopOverrun')
  assert not dec.should_stop


# --- queue ---

def test_queue_behind_a_lead_creeps_to_the_line():
  car = Car([(200., 'stop')])
  car.lead_stop = 180.  # stopped behind a car 20 m short of the line: no latch there
  car.run_until(lambda out, dec: car.v < 0.01 and car.s > 150.)
  assert 200. - car.s > 5.
  out, dec, _ = car.run(3.)
  assert not dec.should_stop and out.shown.action != 'held'
  car.lead_stop = None  # the lead leaves: the curve re-seeds and the car creeps up
  car.run_until(held, max_s=30.)
  assert -2. < 200. - car.s < 5.


# --- signals with the sim oracle ---

def lights(state, distance):
  return Lights([(state, distance)], 'simOracle')


def oracle(state, offset=0.):
  """The sim oracle reporting the first control's light, as the car moves."""
  return lambda car: lights(state, car.controls[0][0] - car.s + offset)


def test_green_has_no_map_cap():
  car = Car([(200., 'signal')])
  for _ in range(int(25. / DT_MDL)):
    out, dec, cmd = car.step(lights=oracle('green'))
    assert dec.speed_cap >= car.v and not dec.should_stop
  assert car.s > 210. and out.reason in ('', 'Green (sim oracle)')


def test_red_holds_and_steady_green_releases():
  car = Car([(150., 'signal')])
  car.run_until(held, lights=oracle('red'))
  out, dec, _ = car.run(20., lights=oracle('red'))
  assert held(out, dec) and out.prompt == 'none' and dec.reason == 'Red light (sim oracle)'
  out, dec, _ = car.run(GREEN_STEADY_S - 0.1, lights=oracle('green'))  # 0.4 s of green: not yet
  assert held(out, dec)
  out, dec, _ = car.run(0.15, lights=oracle('green'))
  assert not dec.should_stop and any(r[-1].released_stopped for r in car.rows[-3:])
  out, dec, _ = car.run(5., lights=oracle('green'))
  assert car.s > 155. and not out.stops


def test_gas_on_red_never_releases_it():
  # no gas go (Kellen, 2026-10-28): a press at a red hold is the driver's, never a release; the hold stays
  car = Car([(150., 'signal')])
  car.run_until(held, lights=oracle('red'))
  out, dec, _ = car.step(gas=True, accel=0., lights=oracle('red'))
  out, dec, _ = car.step(accel=0., lights=oracle('red'))
  assert held(out, dec) and out.alert == 'none' and not out.released_stopped


def test_stale_or_far_light_is_unknown():
  # a red 25 m+ off the control isn't its light, and two lights disagreeing about it are unknown: nothing to stop for
  car = Car([(150., 'signal')])
  for _ in range(int(5. / DT_MDL)):
    out, dec, _ = car.step(lights=oracle('red', 40.))  # 40 m past it
    assert not out.stops
  out, dec, _ = car.step(lights=Lights([('green', car.controls[0][0] - car.s), ('red', car.controls[0][0] - car.s + 3.)], 'simOracle'))
  assert out.shown.light_state == 'unknown' and not out.stops and out.prompt == 'none'


@pytest.mark.parametrize("mph", [25, 35, 45])
@pytest.mark.parametrize("side", [-1., 1.])
def test_yellow_boundary(mph, side):
  v = mph * 0.44704
  d_stop = v * T_LAT + v ** 2 / (2. * 2.0)
  car = Car([(400., 'signal')], v=v)
  car.run_until(lambda out, dec: 400. - car.s < d_stop + side * 1. + 5., lights=oracle('green'))
  car.run_until(lambda out, dec: 400. - car.s < d_stop + side * 1., lights=oracle('green'))
  out, dec, _ = car.step(lights=oracle('yellow'))
  if side > 0:  # 1 m beyond the stopping distance: stop, never past the yellow floor
    assert out.shown.action in ('braking', 'approaching') and out.alert == 'none'
    for _ in range(int(20. / DT_MDL)):
      out, dec, cmd = car.step(lights=oracle('red'))
      assert cmd >= -2.0 - 1e-6
      if held(out, dec):
        break
    assert held(out, dec) and 400. - car.s > -2.
  else:  # 1 m inside it: committed, never reversed by the red
    assert out.shown.action == 'proceedingOnYellow' and out.alert == 'proceedingOnYellow'
    for _ in range(int(10. / DT_MDL)):
      out, dec, cmd = car.step(lights=oracle('red'))
      assert not dec.should_stop and dec.speed_cap >= car.v
    assert car.s > 405.


def test_flicker_green_unknown_green():
  # green, a spell with no light, green: unknown is nothing to stop for, so never a cap
  car = Car([(300., 'signal')])
  for lit, secs in ((oracle('green'), 3.), (NO_LIGHTS, 0.4), (oracle('green'), 0.4), (NO_LIGHTS, 2.)):
    for _ in range(int(round(secs / DT_MDL))):
      out, dec, _ = car.step(lights=lit)
      assert not out.stops and dec.speed_cap >= car.v


def test_a_red_whose_light_is_lost_stays_stopped_until_a_steady_green():
  car = Car([(150., 'signal')])
  car.run_until(held, lights=oracle('red'))
  out, dec, _ = car.run(10., lights=NO_LIGHTS)
  assert held(out, dec) and car.policy.stop_ahead() == 0.
  out, dec, _ = car.run(GREEN_STEADY_S + 0.1, lights=oracle('green'))
  assert not dec.should_stop


def test_a_held_signal_whose_light_goes_unseen_hands_back_loudly_and_stays_held():
  """BeamNG S14 (diag s14_red_right_frames, BUILD_FINDINGS N3): held at a red, the car 1.3 m past the mapped line, the sim's
  light oracle stopped listing the light, and the hold waited 108.6 s on "Stopped at signal: waiting for green" with no
  prompt. A held signal with no accepted light read for SIGNAL_UNSEEN_S hands back loudly (signalUnseenTakeControl,
  "Signal not seen: take control"), the car held: never released by the timeout. A red read meanwhile, however long, is the
  light seen; a light read again takes the handback down, and a steady green still releases."""
  from openpilot.selfdrive.turnpilot.map_controls import SIGNAL_UNSEEN_REASON, SIGNAL_UNSEEN_S
  car = Car([(150., 'signal')])
  car.run_until(held, lights=oracle('red'))
  for _ in range(int(60. / DT_MDL)):  # a long red, read all along: no handback
    out, dec, _ = car.step(lights=oracle('red'))
    assert held(out, dec) and out.prompt == 'none'
  for _ in range(int((SIGNAL_UNSEEN_S - 0.2) / DT_MDL)):  # the light gone unknown: still waiting
    out, dec, _ = car.step(lights=NO_LIGHTS)
    assert held(out, dec) and out.prompt == 'none', car.t
  out, dec, _ = car.run(0.4, lights=NO_LIGHTS)
  assert held(out, dec) and out.prompt == 'signalUnseenTakeControl' and dec.reason == SIGNAL_UNSEEN_REASON == \
    "Signal not seen: take control"
  for _ in range(int(60. / DT_MDL)):  # handed back, held: the timeout never lets go
    out, dec, _ = car.step(lights=NO_LIGHTS)
    assert held(out, dec) and out.prompt == 'signalUnseenTakeControl' and not out.released_stopped
  out, dec, _ = car.run(0.2, lights=oracle('red'))  # read again: TurnPilot's hold to release once more
  assert held(out, dec) and out.prompt == 'none'
  out, dec, _ = car.run(GREEN_STEADY_S + 0.1, lights=oracle('green'))
  assert not dec.should_stop
  # handed back, the driver takes over and drives off: theirs (driverWent), never a stop again in the junction
  car = Car([(150., 'signal')])
  car.run_until(held, lights=oracle('red'))
  out, dec, _ = car.run(SIGNAL_UNSEEN_S + 0.2, lights=NO_LIGHTS)
  assert out.prompt == 'signalUnseenTakeControl'
  drive_off(car)
  out, dec, _ = car.run(3., lights=NO_LIGHTS)
  assert not dec.should_stop and not out.stops and car.s > 160.


def test_light_input_acceptance():
  class Msg:
    source = 'simOracle'
    lights = [type('L', (), {'state': 'green', 'distance': 50., 'instance': 'trafficLight 1'})()]
  ok = light_input(Msg(), age_s=0.1, simulation=True, oracle_param=True)
  assert ok.lights == [('green', 50.)] and not ok.rejected
  for kw in ({'simulation': False, 'oracle_param': True}, {'simulation': True, 'oracle_param': False}):
    r = light_input(Msg(), age_s=0.1, **kw)
    assert r.lights is None and r.rejected
  stale = light_input(Msg(), age_s=LIGHT_STALE_S + 0.1, simulation=True, oracle_param=True)
  assert stale.lights is None and not stale.rejected
  assert light_input(None, age_s=0., simulation=True, oracle_param=True).lights is None


# --- yields ---

def rolling_ok(car):
  """update()'s yields: every yield's rolling test passing now (STOP_SIGNS.md 2.7), its commit point the default."""
  from openpilot.selfdrive.turnpilot.map_controls import YIELD_COMMIT_M
  return {c.key: (True, YIELD_COMMIT_M) for c in car.policy.tracker.controls.values() if c.kind == 'yield'}


def test_C5_a_yield_whose_rolling_test_passes_through_its_commit_point_caps_the_speed_and_never_stops():
  # 2.7: while the rolling test passes the map-stop curve's cap lifts to YIELD_SPEED at the line; passing it at the rolling
  # commit point the yield is taken rolling and passed, never shouldStop
  car = Car([(150., 'yield')])
  for _ in range(int(20. / DT_MDL)):
    out, dec, cmd = car.step(yields=rolling_ok)
    assert not dec.should_stop and dec.accel_cap == NO_ACCEL_CAP or dec.accel_cap >= MAP_DECEL_MAX
    if 0. < 150. - car.s < 1.:
      assert car.v < YIELD_SPEED + 0.6
    if out.stops:
      assert out.prompt == 'yieldAhead'
  assert car.s > 160.


def test_a_stop_sign_brings_the_car_to_rest_and_a_yield_or_the_off_route_cap_never_does():
  """A stop in force says where it brings the car to rest (decision.rest_d), braking toward it as well as held at it; a
  yield's cap or the off-route cap is a speed, never a rest (turnpilotd keys a veto on a stop, never on a cap)."""
  car = Car([(150., 'stop')])
  out, dec = car.run_until(lambda out, dec: out.stops and dec.holding and car.v > 3.)
  assert not dec.should_stop and dec.rest_d is not None and abs(dec.rest_d - (150. - car.s)) < 2.
  out, dec = car.run_until(held)
  assert dec.rest_d == 0.
  # a yield taken rolling (its rolling test passing: STOP_SIGNS.md 2.7) is a cap, never a rest
  car = Car([(150., 'yield')])
  rests = [car.step(yields=rolling_ok)[1].rest_d for _ in range(int(20. / DT_MDL))]
  assert all(r is None for r in rests)
  # one whose rolling test fails (this week every yield's: SG4c's C5) is a stop: a rest at its line as it brakes toward it,
  # before it becomes yieldStop at its commit point as after, and held there (merging tp/blinker, whose yield was only a cap)
  car = Car([(150., 'yield')])
  out, dec = car.run_until(lambda out, dec: out.stops and dec.holding and car.v > 3. and 150. - car.s > 40.)
  assert car.policy.tracker.controls and dec.rest_d is not None and abs(dec.rest_d - (150. - car.s)) < 2.
  out, dec = car.run_until(held)
  assert dec.rest_d == 0.
  car = Car([(400., 'signal')])
  car.run(2.)
  car.nav_valid, car.off_route, car.off_status = False, [], 'failed'
  out, dec, _ = car.run(8.)
  assert OFF_ROUTE_REASON in [s.reason for s in out.stops] and dec.holding and dec.rest_d is None


def test_C5_a_yield_whose_rolling_test_fails_is_a_stop_yieldstop_at_its_line_on_the_map_curve_latched_under_the_decider():
  # 2.7, row C5 (mc): this week no yield's rolling test passes (no survey: 64-84 m of cross lane needed): the approach is the
  # map-stop curve to a full stop at the yield line (-1.5, floor -2.0), it becomes a stop (yieldStop) at its rolling commit
  # point, latches, and from then on is a stop under the decider: as shadow its 1 s handback ("Yield: take control"), under a
  # StopCommand wait held with no handback, released by the decider's 'clear'. Before SG4c the car rolled through at 5 m/s
  from openpilot.selfdrive.turnpilot.map_controls import YIELD_STOP, is_stop
  car = Car([(150., 'yield')])
  out, dec = car.run_until(held)
  assert abs(150. - car.s) < 2.5 and min(r[3] for r in car.rows) >= MAP_DECEL_MAX - 1e-6
  c = car.policy.held_control()
  assert c is not None and c.kind == 'yield' and c.state.mode == YIELD_STOP and is_stop(c)
  assert car.policy.stop_ahead() == 0.
  out, dec, _ = car.run(STOP_SIGN_HANDBACK_S + 0.2)
  assert held(out, dec) and out.prompt == 'stopSignTakeControl' and 'Yield' in dec.reason
  car = Car([(150., 'yield')])
  car.run_until(held)
  out, dec, _ = car.run(5., stop=StopCommand('wait', 'Waiting: cross traffic', 'waitingCross'))
  assert dec.should_stop and out.prompt == 'none' and out.shown.action == 'waitingCross'
  key = car.policy.held_control().key
  out, dec, _ = car.step(stop=StopCommand('wait', 'Going', 'going'), release=(key, 'clear'))
  assert car.policy.held_control() is None and not dec.should_stop


def test_C5_a_yield_whose_rolling_test_fails_late_becomes_a_stop_at_its_last_stopping_point_never_below_2_m_s2():
  # on the lifted cap while the test passes, then it fails just before the last point a stop at the line is still -2.0 from
  # the car's speed (yield_last_stop_m): a stop from there, at the floor at most, latched at the line, never an overrun into
  # the junction (2.7's commit point 7.6 m from the edge would end that stop up to 2.5 m past the line, beyond the latch)
  from openpilot.selfdrive.turnpilot.map_controls import YIELD_COMMIT_M, YIELD_STOP, yield_last_stop_m
  def flip(car):  # passing until 0.3 m before its last stopping point, failing from there
    c = next((c for c in car.policy.tracker.controls.values() if c.kind == 'yield'), None)
    return {} if c is None else {c.key: (c.d > yield_last_stop_m(car.v) + 0.3, YIELD_COMMIT_M)}
  car = Car([(150., 'yield')])
  out, dec = car.run_until(held, yields=flip)
  assert car.policy.held_control().state.mode == YIELD_STOP and abs(150. - car.s) < 2.
  assert min(r[3] for r in car.rows) >= MAP_DECEL_MAX - 1e-6
  assert not any(r[5].alert == 'stopOverrun' for r in car.rows)


def test_C5_a_cross_suspect_yield_is_kept_unless_the_sidecar_confirms_it_governs_the_cross_approach():
  # 2.7: a yield on a road that outranks every crossing one (crossSuspect) was dropped; now only where the OSM sidecar has the
  # junction's give_way or stop on a crossing approach and none on ours (junctionInfo.crossGoverned); otherwise ours: a stop
  info = {'status': 'ok', 'crossGoverned': True}
  car = Car([(150., 'yield', False, True, True, info)])
  car.run(15.)
  assert car.s > 160. and min(r[2] for r in car.rows) > 13.
  car = Car([(150., 'yield', False, True, True, {'status': 'ok', 'crossGoverned': False})])
  out, dec = car.run_until(held)
  assert abs(150. - car.s) < 2.5


def test_cross_suspect_stop_still_stops():
  car = Car([(150., 'stop', False, True, True)])
  out, dec = car.run_until(held)
  assert 'unverified' in dec.reason


# --- status ---

def test_controls_status_not_ok_is_loud():
  car = Car([], status='unsupported')
  out, dec, _ = car.step()
  assert out.alert == 'mapControlsUnavailable' and out.reason == 'Map stops/signals: unsupported'
  out, dec, _ = car.run(10.)
  assert out.reason == 'Map stops/signals: unsupported'
  ok = Car([], status='ok')
  out, dec, _ = ok.step()
  assert out.alert == 'none' and out.reason == ''


def test_partial_controls_still_stop_and_are_said():
  """navd's partial (STOP_SIGNS.md N7: the OSM stop-sign sidecar missing or stale, so the stops Valhalla doesn't flag are
  unknown; routing/controls.py): Valhalla's own stops are still stopped for, and the status is said all the way."""
  car = Car([(150., 'stop')], status='partial')
  out, dec, _ = car.step()
  assert out.alert == 'mapControlsUnavailable' and out.reason == 'Map stops/signals: partial'
  out, dec = car.run_until(held)
  assert 140. < car.s < 150.


def test_light_rejected_alert():
  car = Car([(300., 'signal')])
  out, dec, _ = car.step(lights=Lights(None, 'simOracle', rejected=True))
  assert out.alert == 'lightSourceRejected'


def test_crossing_reason():
  car = Car([(200., 'signal', False, False, False)])
  out, dec = car.run_until(held, lights=oracle('red'))
  assert 'crossing' in dec.reason


# --- unprotected turns: a far-side turn across oncoming traffic, with no stop sign or signal at its junction ---

def test_unprotected_turn_from_map_data():
  def m(mtype, modifier, opposing='present', geometry=True, branches=2, after=270.):
    return make_maneuver(mtype, modifier, (), geometry, 0., after, branches, opposing)
  assert unprotected_turn(m('turn', 'left'), True)                       # a left across the road ahead's traffic
  assert unprotected_turn(m('turn', 'sharp left', after=225.), True)
  assert unprotected_turn(m('turn', 'left', geometry=False), True)       # no junction data: unknown, never clear
  assert unprotected_turn(m('turn', 'left', opposing='unknown'), True)   # a navd from before the field
  assert not unprotected_turn(m('turn', 'left', opposing='none'), True)  # a T's stem: no oncoming approach
  assert not unprotected_turn(m('turn', 'right', after=90.), True)       # curb side
  assert unprotected_turn(m('turn', 'right', after=90.), False)          # driving on the left, a right is the far side
  assert not unprotected_turn(m('turn', 'left'), False)
  assert not unprotected_turn(m('turn', 'slight left', after=330.), True)  # a keep
  assert not unprotected_turn(m('turn', 'left', branches=0), True)        # a bend: the model's, not a turn
  assert not unprotected_turn(None, True)


def left_car(controls=(), v=13.4, at=250.):
  car = Car(controls, v=v)
  car.turn = (at, 7, 'left')
  return car


def test_unprotected_left_stops_before_the_hold_line_and_hands_back():
  car = left_car()
  out, dec = car.run_until(held)
  assert car.v < 0.3 and 0. <= 250. - HOLD_LINE_M - car.s < 3.  # before the line, never past it
  assert all(cmd >= MAP_DECEL_MAX - 1e-6 for *_, cmd, _, _ in car.rows)
  assert out.shown.kind == UNPROTECTED and out.prompt == 'unprotectedTurnTakeControl'  # handed back at once
  assert dec.reason == "Unprotected left: take control" and out.shown.action == 'awaitingDriver'
  s_held = car.s
  out, dec, _ = car.run(20.)
  assert held(out, dec) and car.s - s_held < 0.05
  n = len(car.rows)
  drive_off(car, 2.)
  out, dec, _ = car.run(5.)
  assert not dec.should_stop and out.prompt == 'none' and out.shown.kind == 'none'
  assert car.s > 250.
  # never a second stop; the crossing the driver made with no go is said until the car is through (DRIVEN_THROUGH_M past
  # the line, owner decision (b)), then nothing
  line = 250. - HOLD_LINE_M - 1.
  after = car.rows[n:]
  assert not any(r[4].should_stop for r in after if r[1] > line + 3.)
  assert all(r[5].prompt == 'unprotectedTurnTakeControl' for r in after if line + 3. < r[1] < line + DRIVEN_THROUGH_M - 1.)
  assert all(r[5].prompt == 'none' for r in after if r[1] > line + DRIVEN_THROUGH_M + 1.)


def test_unprotected_left_approach_reason_and_engage():
  car = left_car()
  out, dec, _ = car.run_until(lambda out, dec: out.stops) + (None,)
  assert 250. - HOLD_LINE_M - car.s <= engage_distance(13.4) + 2.
  assert out.stops[0].reason == "Unprotected left ahead" and out.shown.kind == UNPROTECTED


def test_a_signal_at_the_junction_leaves_the_left_unprotected():
  # signals don't hold, and a green ball can't be told from an arrow (SIGNAL_PROTECTS_LEFT): the left stops at its hold line
  car = left_car([(250. - 11., 'signal')])
  out, dec = car.run_until(held, lights=lambda car: lights('green', 239. - car.s))
  assert out.shown.kind == UNPROTECTED and -2. < 250. - HOLD_LINE_M - 1. - car.s < 5.


@pytest.mark.parametrize("kind", ['stop'])
def test_stop_sign_at_the_junction_protects_the_left(kind):
  # its own latch holds the car: one stop, one handback, no second stop at the hold line
  car = left_car([(250. - 11., kind)])
  out, dec = car.run_until(held)
  assert out.shown.kind == kind and -2. < 239. - car.s < 5.
  car.run(STOP_SIGN_HANDBACK_S + 0.5)
  drive_off(car, 2.)
  car.run(8.)
  assert car.s > 252.
  assert not any(o.shown is not None and o.shown.kind == UNPROTECTED for *_, o in car.rows)
  assert not any(d.should_stop for *_, d, o in car.rows[-140:])


def test_a_yield_at_the_junction_protects_the_left_and_a_stop_far_upstream_leaves_it_unprotected():
  # a yield at the left's junction becomes a stop there (yieldStop, STOP_SIGNS.md 2.7, SG4c), so the stop-sign decider, which
  # watches its cross traffic, has the turn, not the unprotected left, which watches only the oncoming (it used to roll the
  # yield at 5 m/s and hold at the left's line); a stop far upstream still leaves it unprotected
  from openpilot.selfdrive.turnpilot.map_controls import YIELD_STOP
  car = left_car([(250. - 11., 'yield')])
  car.run_until(held)
  assert car.rows[-1][-1].shown.kind == 'yield' and car.policy.held_control().state.mode == YIELD_STOP
  assert car.policy.turn is None and abs(239. - car.s) < 2.5
  car = left_car([(150., 'stop')])
  car.run_until(held)
  car.run(STOP_SIGN_HANDBACK_S + 0.5)
  drive_off(car, 2.)
  out, dec = car.run_until(held)
  assert out.shown.kind == UNPROTECTED and 0. <= 242. - car.s < 3.


def test_a_yield_that_wont_stop_the_car_never_protects_the_left():
  # a crossSuspect yield the sidecar says is the cross road's (crossGoverned) is dropped (_yield: done 'dropped'), so it stops
  # nothing and the left across our oncoming traffic is the unprotected left's: held at its hold line. protected() had
  # counted any yield there, dropped or not (and _turn_control reads it before _yield has run), so the left was never held
  # and passed at 13.4 m/s. A yield taken rolling (yieldRolling) is no protection either: nobody would watch the oncoming
  from types import SimpleNamespace
  from openpilot.selfdrive.turnpilot.map_controls import YIELD_ROLLING, YIELD_STOP, ControlState, protected
  car = left_car([(250. - 11., 'yield', False, True, True, {'status': 'ok', 'crossGoverned': True})])
  out, dec = car.run_until(held, max_s=40.)
  assert out.shown.kind == UNPROTECTED and 0. <= 250. - HOLD_LINE_M - car.s < 3.
  assert all(cmd >= MAP_DECEL_MAX - 1e-6 for *_, cmd, _, _ in car.rows)
  # not crossGoverned it is ours: a stop at its line protects the left, as before
  car = left_car([(250. - 11., 'yield', False, True, True, {'status': 'ok'})])
  car.run_until(held)
  assert car.policy.held_control().state.mode == YIELD_STOP and car.policy.turn is None

  def y(mode='stop', done='', cross=False):
    return SimpleNamespace(kind='yield', d=10., cross_suspect=cross, junction_info={'crossGoverned': cross},
                           state=ControlState(mode=mode, done=done))
  assert protected([y()], 12.) and protected([y(YIELD_STOP)], 12.) and protected([y(YIELD_STOP, 'released')], 12.)
  assert not protected([y(YIELD_ROLLING)], 12.) and not protected([y(done='dropped')], 12.) and not protected([y(cross=True)], 12.)
  assert not protected([y(done='passed')], 12.)
  # ours, its rolling test passing (none does this week: dormant): undecided it protects without latching, so once it is
  # taken rolling at its commit point the left's own stop is made from there, or said infeasible, never rolled through
  car = left_car([(250. - 11., 'yield')])
  rows = []
  for _ in range(int(30. / DT_MDL)):
    rows.append(car.step(yields=rolling_ok))
    if car.v < 0.05:
      break
  assert car.policy.turn is not None and car.s < 250. and car.v < 0.05
  assert any(o.alert == 'cannotStopComfortably' for o, _, _ in rows) or 0. <= 250. - HOLD_LINE_M - car.s < 3.
  assert all(cmd >= MAP_DECEL_MAX - 1e-6 for _, _, cmd in rows)


def test_unprotected_latch_is_position_free_and_says_so_through_a_fault():
  car = left_car()
  car.run_until(held)
  s_held = car.s
  car.turn = None  # the route went away (a reroute, a gap): the latch holds on
  out, dec, _ = car.run(3.)
  assert held(out, dec) and car.s - s_held < 0.05
  car.turn = (400., 8, 'left')  # a new route's maneuver
  out, dec, _ = car.run(1.)
  assert held(out, dec) and out.shown.kind == UNPROTECTED
  assert car.policy.held_stops() and car.policy.held_stops()[0].should_stop
  drive_off(car, 2.)
  out, dec, _ = car.step()
  assert not dec.should_stop


def test_a_left_not_held_goes_with_its_maneuver():
  car = left_car()
  car.run_until(lambda out, dec: out.stops)
  car.turn = None  # skipped too fast, or the route changed: no stop for a turn the car isn't taking
  out, dec, _ = car.step()
  assert not out.stops and out.shown.kind == 'none'


def test_unprotected_turn_texts():
  from openpilot.selfdrive.turnpilot.control_text import hold_text, is_map_reason
  assert hold_text(UNPROTECTED, 'awaitingDriver') == ('Stopped at unprotected turn', 'take control')
  assert hold_text(UNPROTECTED, 'held') == ('Stopped at unprotected turn', '')
  assert is_map_reason("Unprotected left ahead") and is_map_reason("Unprotected left: take control")
  assert is_map_reason("Stop sign: take control to proceed")


@pytest.mark.parametrize('action, reason, want', [
  ('waitingCross', 'Waiting: a car from the left not stopping', ('Waiting for cross traffic', 'a car from the left not stopping')),
  ('waitingCross', 'Waiting for cross traffic', ('Waiting for cross traffic', '')),
  ('waitingCross', 'Waiting: signal at junction', ('Signal at junction', '')),
  ('waitingCross', "Waiting: can't see far enough from the right (sunAhead)", ('Sun glare', "can't see far enough from the right")),
  ('waitingCross', "Waiting: can't see far enough from the left (occluder)", ('View blocked', "can't see far enough from the left")),
  ('creeping', 'Creeping for view', ('Creeping for view', '')),
  ('creeping', 'Rolling up to the stop sign', ('Rolling up to stop sign', '')),
  ('creeping', 'Waiting: taking our turn: creeping forward', ('Taking our turn', '')),
  ('going', 'Going', ('Going', '')),
  ('committed', '', ('Committed', '')),
  ('committed', 'Stalled in junction', ('Stalled in junction', '')),
  ('awaitingDriver', 'Stop sign: signal at junction: take control', ('Signal at junction', 'take control')),
  ('awaitingDriver', "Stop sign: can't see far enough from the right (lowSun): take control", ('Sun glare', 'take control')),
  ('awaitingDriver', "Stop sign: can't see far enough from the right (occluder): take control", ('View blocked', 'take control')),
  ('awaitingDriver', 'Stop sign: this stop needs you: take control', ('Stopped at stop sign', 'take control')),
])
def test_hold_text_says_every_stop_sign_ledger_reason(action, reason, want):
  # STOP_SIGNS.md v3 SG4c, 4.6: every held state of the stop-sign decider says what it is on the device and the phone: its
  # wait, its creeps, its go, the band, a stall in the junction, and the ladder's ends by their reason
  from openpilot.selfdrive.turnpilot.control_text import hold_text, is_map_reason
  assert hold_text('stop', action, reason=reason) == want
  assert reason == '' or is_map_reason(reason)


def test_hold_text_for_a_yield_that_became_a_stop():
  from openpilot.selfdrive.turnpilot.control_text import hold_text
  assert hold_text('yield', 'held') == ('Stopped at yield', '')
  assert hold_text('yield', 'awaitingDriver', reason='Yield: take control to proceed') == ('Stopped at yield', 'take control')
  assert hold_text('yield', 'waitingCross', reason='Waiting for cross traffic') == ('Waiting for cross traffic', '')


# --- found in review: the car never left stuck, never let go of a light by a nudge, never re-held where it isn't ---

@pytest.mark.parametrize("v_creep,past", [(1.5, 3.), (1.9, 6.), (1.0, 12.)])
def test_creeping_over_the_line_under_the_drivers_foot_is_never_stopped_in_the_junction(v_creep, past):
  # the driver's foot takes the car over the stop sign's line slower than COMMIT_SPEED and lifts past it: the car isn't
  # held there, so it is an overrun, said out loud, and the stop is dropped. Never a speed cap of 0 in the junction with no
  # hold, no prompt and no way out
  car = Car([(200., 'stop')], v=v_creep, cruise=v_creep)
  car.s = 180.
  car.run_until(lambda out, dec: out.stops)
  car.run_until(lambda out, dec: car.s > 200. + past, gas=True, accel=0.)
  alerts = [car.step()[0].alert]
  for _ in range(int(10. / DT_MDL)):
    out, dec, _ = car.step()
    alerts.append(out.alert)
  assert 'stopOverrun' in alerts
  assert not out.stops and dec.speed_cap > 5. and car.v > v_creep - 0.05


def test_a_gas_nudge_in_a_queue_short_of_a_red_doesnt_let_go_of_it():
  # queued behind a lead 22 m short of a red, the driver creeps up a little under the gas: the car still stops at the line
  # once the lead leaves, and is held there
  car = Car([(200., 'signal')])
  red = oracle('red')
  car.run_until(lambda out, dec: out.stops, lights=red)
  car.lead_stop = 178.
  out, dec = car.run_until(lambda out, dec: car.v < 0.01 and car.s > 150., lights=red)
  assert 200. - car.s > 20.
  out, dec, _ = car.run(1., lights=red)
  assert out.prompt == 'none'
  car.run(0.5, gas=True, accel=0.4, lights=red)
  car.run(2., lights=red)
  car.lead_stop = None
  out, dec = car.run_until(held, max_s=40., lights=red)
  assert -2. < 200. - car.s < 5.


def test_a_reroute_that_adopts_the_held_stop_keeps_its_latch_and_the_next_stop_holds_too():
  # held at a stop sign, a reroute lists the same sign under a new key: the driver driving off releases it, and the next
  # stop sign latches the car with its own handback, never a cap of 0 with no hold
  car = Car([(120., 'stop'), (300., 'stop')])
  car.run_until(held)
  car.route_count = 2
  car.run(3.)
  drive_off(car, 2.)
  out, dec, _ = car.step()
  assert not dec.should_stop and car.policy.held is None
  out, dec = car.run_until(held, max_s=60.)
  assert -2. < 300. - car.s < 5.
  out, dec, _ = car.run(STOP_SIGN_HANDBACK_S + 0.5)
  assert out.prompt == 'stopSignTakeControl'


def test_a_held_car_that_moved_off_without_the_plan_is_let_go_never_re_held_down_the_road():
  # held at a red (a hold with no handback), the plan stops being applied (shadow mode turned on) and openpilot drives on
  # with no gas: the latch lets go once the car is moving off, out loud, so it isn't replayed when the plan applies again
  car = Car([(120., 'signal')])
  car.run_until(held, lights=oracle('red'))
  car.run(1., lights=oracle('red'))
  alerts = []
  for _ in range(int(20. / DT_MDL)):
    out, dec, _ = car.step(accel=1.0 if car.v < 14. else 0.)  # the plan not applied: the car drives on regardless
    alerts.append(out.alert)
  assert car.s > 300.
  assert 'stopOverrun' in alerts and car.policy.held is None
  out, dec, _ = car.step()
  assert not dec.should_stop and not out.stops


def test_unprotected_left_whose_junction_is_read_nearer_than_the_car_can_stop_short_of_stops_at_the_floor():
  # the route's junction estimate moves 20 m nearer (route_follower.py: the nearest of navd's readings) once the car is
  # braking for its hold line: the line is now behind the car. Never dropped as an overrun and crossed blind: the car
  # brakes at the floor, no harder, stops as soon as it can, said out loud, and is held there and handed back
  for jump in (20., 12.):
    car = left_car()
    car.run_until(lambda out, dec, car=car: out.stops and 250. - car.s < 28.)
    car.turn = (car.turn[0] - jump,) + car.turn[1:]
    alerts = []
    out, dec = car.run_until(lambda out, dec, alerts=alerts: alerts.append(out.alert) or held(out, dec), max_s=20.)
    assert car.v < 0.3 and 250. - car.s > HOLD_LINE_M, jump  # short of the true hold line
    assert all(cmd >= MAP_DECEL_MAX - 1e-6 for *_, cmd, _, _ in car.rows), jump
    assert 'cannotStopComfortably' in alerts, jump
    assert out.shown.kind == UNPROTECTED and out.prompt == 'unprotectedTurnTakeControl'
    s_held = car.s
    out, dec, _ = car.run(5.)
    assert held(out, dec) and car.s - s_held < 0.05
  # a reading a little nearer close to the line, where the latch reaches past it: no floor, no take control, held
  car = left_car()
  car.run_until(lambda out, dec, car=car: 250. - HOLD_LINE_M - car.s < 3.)
  car.turn = (car.turn[0] - 1.5,) + car.turn[1:]
  alerts = []
  out, dec = car.run_until(lambda out, dec, alerts=alerts: alerts.append(out.alert) or held(out, dec), max_s=20.)
  assert 'cannotStopComfortably' not in alerts and car.v < 0.3


def test_off_route_holds_for_the_signal_on_its_own_road_not_the_stale_routes():
  """BeamNG route_demo, 2026-10-28: rerouting, the car drove through the signal on its road at speed while the tracker
  dead-reckoned the old route's, 106 m off. navd's lookup ahead of the car (offRouteControls) holds it at its own."""
  car = Car([(250., 'signal')])
  car.run(2.)
  car.nav_valid, car.off_route = False, [(car.s + 90., 'signal')]
  line = car.off_route[0][0]
  red = lambda car: lights('red', line - car.s)  # noqa: E731
  out, dec = car.run_until(held, max_s=30., lights=red)
  assert line - 3. < car.s < line + 0.5
  assert out.prompt == 'none' and out.alert == 'none'
  assert all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)
  car.run(3., lights=red)
  assert car.v == 0.  # held for the green, like any red
  car.run(GREEN_STEADY_S + 0.1, lights=lambda car: lights('green', line - car.s))
  car.run(4.)
  assert car.s > line + 2.


def test_off_route_an_engaged_signal_the_lookup_takes_over_is_held_at_the_lookups_line():
  """Engaged on the route's signal, then off the route: the lookup's signal of the same kind, nearer or further by up to
  ADOPT_M, takes it over, and the car is held at the lookup's line (measured on the road the car is on), not at the
  stale route's dead-reckoned one, braking no harder than the floor."""
  for stale_off in (15., -15., 19.):
    car = Car([(150., 'signal')])
    while not any(c.engaged for c in car.policy.tracker.controls.values()):
      car.step(lights=oracle('red'))
    line = 150. - stale_off
    car.nav_valid, car.off_status, car.off_route = False, 'ok', [(line, 'signal')]
    out, dec = car.run_until(held, max_s=40., lights=lambda car, line=line: lights('red', line - car.s))
    assert line - 3. < car.s < line + 0.5, (stale_off, car.s - line)
    assert all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows), stale_off


def test_off_route_with_the_stops_ahead_unknown_caps_the_speed_and_says_so():
  car = Car([(400., 'signal')])
  car.run(2.)
  car.nav_valid, car.off_route, car.off_status = False, [], 'failed'
  out, dec, _ = car.run(1.)
  assert out.alert == 'none'  # the grace: navd's one invalid message between routes
  out, dec, _ = car.run(2.)
  assert out.alert == 'offRouteStopsUnknown' and out.reason == OFF_ROUTE_REASON  # from the grace on, at whatever speed
  out, dec, _ = car.run(10.)
  assert out.alert == 'offRouteStopsUnknown' and OFF_ROUTE_REASON in [s.reason for s in out.stops]
  assert car.v <= OFF_ROUTE_SPEED + 0.3 and not dec.should_stop
  assert all(r[3] >= -1.0 - 1e-6 for r in car.rows)  # never braked harder than a gentle ramp
  car.off_status = 'ok'  # the lookup works again: nothing ahead
  out, dec, _ = car.run(2.)
  assert out.alert == 'none' and not out.stops
  car.off_status = 'failed'
  car.run(3.)
  car.nav_valid = True  # rerouted
  out, dec, _ = car.run(1.5)
  assert out.alert == 'none' and not any(s.reason == OFF_ROUTE_REASON for s in out.stops)


def test_no_destination_is_not_off_route():
  car = Car([])
  car.nav_valid, car.off_route = False, None
  out, dec, _ = car.run(10.)
  assert out.alert == 'none' and not out.stops and car.v > 13.


# --- G1 (2026-10-28): a signal adds nothing unless a light source reads red or yellow for it ---

@pytest.mark.parametrize("light", [None, 'green', 'unknown'])
def test_unknown_or_green_signal_adds_no_slowing_at_all(light):
  # no light source (the car this week), a green, or an unknown light: no stop, no cap, no prompt, nothing to stop at
  car = Car([(200., 'signal'), (215., 'signal')])
  lit = NO_LIGHTS if light is None else oracle(light)
  for _ in range(int(25. / DT_MDL)):
    out, dec, cmd = car.step(lights=lit)
    assert out.stops == [] and out.prompt == 'none' and not dec.should_stop, car.s
    assert car.policy.stop_ahead() is None
  assert car.s > 300. and min(r[2] for r in car.rows) >= 13.4 - 1e-6


def test_a_red_seen_inside_the_engage_distance_stops_on_a_seeded_curve():
  # passing through, the light reads red 60 m out at 13.4 m/s (inside engage_distance, ~88 m): a stop on the curve from
  # there, never harder than -2.0, held at the line, and the signal is now one the car is stopping at
  car = Car([(260., 'signal')])
  car.run_until(lambda out, dec: car.s > 200.)
  assert car.policy.stop_ahead() is None
  out, dec, _ = car.step(lights=oracle('red'))
  assert out.stops and car.policy.stop_ahead() is not None
  out, dec = car.run_until(held, lights=oracle('red'))
  assert all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)
  assert -2. < 260. - car.s < 5.


def test_a_red_too_close_to_stop_for_gently_brakes_at_minus_2_and_says_so():
  car = Car([(250., 'signal')])
  car.run_until(lambda out, dec: car.s > 220.)
  out, dec, _ = car.step(lights=oracle('red'))
  assert out.alert == 'cannotStopComfortably' and out.stops
  car.run(4., lights=oracle('red'))
  assert all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)


def test_yellow_decel_param_never_takes_a_light_stop_past_minus_2():
  # TurnPilotYellowDecel 2.5 would have floored a yellow stop at -2.5: the standing rule is -2.0 for every map or light stop
  # a green, then the yellow 97 m out at 20 m/s: stoppable at 2.5 after T_LAT, not at 2.0
  car = Car([(400., 'signal')], v=20., yellow_decel=2.5)
  car.run_until(lambda out, dec: car.s > 400. - 97., lights=oracle('green'))
  car.run(6., lights=oracle('yellow'))
  assert min(r[3] for r in car.rows) >= MAP_DECEL_MAX - 1e-6


# --- G1: no gas go anywhere; a hold TurnPilot can't release is a loud handback (Kellen, 2026-10-28) ---

def test_a_stop_sign_hands_back_after_a_second_stopped_and_no_gas_releases_it():
  car = Car([(150., 'stop')])
  car.run_until(held)
  out, dec, _ = car.run(0.5)
  assert held(out, dec) and out.prompt == 'none'
  out, dec, _ = car.run(0.6)
  assert held(out, dec) and out.prompt == 'stopSignTakeControl' and dec.reason == "Stop sign: take control to proceed"
  assert out.shown.action == 'awaitingDriver'
  # a gas press is the driver's (a disengage), never TurnPilot's go: the hold stays, and nothing is released or re-pulsed
  out, dec, _ = car.step(gas=True, accel=0.)
  out, dec, _ = car.step(accel=0.)
  assert held(out, dec) and not out.released_stopped and out.prompt == 'stopSignTakeControl'


def test_the_driver_driving_off_a_handed_back_stop_is_a_clean_release():
  for gas in (True, False):  # under the gas, or disengaged and away on the brake's release (no plan applied)
    car = Car([(150., 'stop')])
    car.run_until(held)
    car.run(1.5)
    car.run(3., gas=gas, accel=1.5)
    out, dec, _ = car.step()
    assert not dec.should_stop and car.policy.held is None, gas
    assert all(r[-1].alert != 'stopOverrun' for r in car.rows), gas


def test_the_driver_creeping_past_a_handed_back_line_slowly_releases_it():
  # skeptic G1 (residual): _held let a handed-back latch go only past DRIVE_OFF_M faster than COMMIT_SPEED, so a driver who
  # took over and peeked past the line at about 1.45 m/s, 3.6 m on, then re-engaged was stopped there, 4.5 m past the line in
  # the crossing lanes. Handed back, the car gone past the latch's reach (LATCH_PAST_M) at any speed is the driver's
  # (driverWent); under the driver's gas, driven off (drivenOff). A peek inside the latch's reach stays held where it stands
  for kind in ('stop', 'left'):
    car = left_car() if kind == 'left' else Car([(150., 'stop')])
    line = 250. - HOLD_LINE_M - 1. if kind == 'left' else 150.
    car.run_until(held)
    car.run(1.5)
    assert car.rows[-1][-1].prompt in ('stopSignTakeControl', 'unprotectedTurnTakeControl')
    while car.s < line + 3.6:  # the driver takes over (the gas disengages) and creeps to about 1.45 m/s
      car.step(gas=True, accel=0.5 if car.v < 1.45 else 0.)
    assert car.v < COMMIT_SPEED
    out, dec, _ = car.step()  # re-engaged, no gas
    assert not dec.should_stop and car.policy.held is None, kind
    s0 = car.s
    car.run(3.)
    assert car.v > 1. and car.s - s0 > 3., kind  # never stopped in the junction
    assert all(r[-1].alert != 'stopOverrun' for r in car.rows), kind
  # a peek inside the latch's reach: still held, the handback still said
  car = Car([(150., 'stop')])
  car.run_until(held)
  car.run(1.5)
  while car.s < 150. + 1.:
    car.step(gas=True, accel=0.5 if car.v < 1. else 0.)
  out, dec, _ = car.run(2.)
  assert held(out, dec) and out.prompt == 'stopSignTakeControl' and car.v < 0.3


def test_driven_slowly_past_a_held_stop_under_the_gas_is_driven_off():
  # the same under the driver's gas with no handback (the stop-sign decider waiting; DisengageOnAccelerator off: the gas
  # overrides): past the latch's reach at a crawl, lifting must not stop the car in the junction
  car = Car([(150., 'stop')])
  car.run_until(held, stop=StopCommand('wait'))
  while car.s < 150. + 3.:
    out, _, _ = car.step(gas=True, accel=0.5 if car.v < 1.2 else 0., stop=StopCommand('wait'))
    assert out.prompt == 'none' or car.policy.held is None
  out, dec, _ = car.step(stop=StopCommand('wait'))
  assert not dec.should_stop and car.policy.held is None


def test_a_hold_turnpilot_moved_on_and_then_handed_back_is_never_let_go_with_no_driver_input():
  # skeptic residual-check-1 (058df13): _held read what was asked of the latch (the left's creep, the decider's roll-up) from
  # the current frame's command, which on an undecided left frame (creep_m 0) or a decider fault (latch not 'wait') is 0, and
  # took a prompt being up as the driver's: so a left that crept 4 m for a view and then went undecided, or a stop the
  # decider rolled up 4.9 m and then faulted or went shadow, was let go as driverWent on the first handback frame, prompt
  # 'none', and the model drove the car on (to 12.8 and 5 m/s). The largest creep asked this hold is the latch's own, and
  # only the driver's own movement is the driver's (owner decision (i): the car standing, none): held, the handback said
  car = left_car()
  held_left(car, left_cmd(reason="Waiting for a gap"))
  line = car.s
  run_left(car, 12., left_cmd(creep_m=4., reason="Creeping for a view"))
  assert 3.7 < car.s - line < 4.3 and car.v < 0.3
  rows = [car.policy_step(left_cmd(take_control=True, reason="View of oncoming traffic blocked: take control"))
          for _ in range(int(10. / DT_MDL))]
  assert all(held(out, dec) and out.prompt == 'leftUndecidedTakeControl' for out, dec, _ in rows)
  assert car.policy.held is not None and car.s - line < 4.4 and car.v < 0.3
  # the driver then takes over under the gas and goes: theirs (driverWent), no overrun
  run_left(car, 3., left_cmd(take_control=True), gas=True)
  assert car.policy.turn.state.done == 'released' and car.policy._alert == 'none'
  # the decider's roll-up from a queue latch, then its fault, or a shadow handback (no command)
  for stop in (StopCommand('fault', 'Camera not seeing cross traffic: take control'), None):
    car = Car([(150., 'stop')], v=5.)
    car.lead_stop = 146.5
    car.run_until(held, stop=StopCommand('wait'))
    target = car.policy.held_control().d
    assert target > 3.
    car.lead_stop = None
    car.run(8., stop=StopCommand('wait', '', 'creeping'), creep=(target, 3.0))
    assert abs(car.s - 150.) < 0.5 and car.v < 0.3
    rows = [car.step(stop=stop) for _ in range(int(10. / DT_MDL))]
    assert all(held(out, dec) for out, dec, _ in rows[int(STOP_SIGN_HANDBACK_S / DT_MDL) + 1:]), stop
    assert all(out.prompt == 'stopSignTakeControl' for out, _, _ in rows[int(STOP_SIGN_HANDBACK_S / DT_MDL) + 1:]), stop
    assert car.policy.held is not None and abs(car.s - 150.) < 0.5 and car.v < 0.3, stop


def test_a_gas_nudge_short_of_the_line_never_lets_the_latch_go():
  """Skeptic residual-check-1 / SG-residual-main (90c67c0, still at be3c0ed): the latch was let go as the driver's on 2.2 m
  moved under the gas, with nothing subtracted for what TurnPilot asked and no check that the car was past the stop point. A
  queue latch 5 m short that the decider rolls up (StopCommand wait, creep (5, 3)), nudged 2.2 m at 0.5 m/s and lifted
  2.58 m short of the line, let go and drove on 38.7 m past at 5 m/s, prompt 'none'. Owner decision (a): let go as the
  driver going only past the stop point by the driver's own gas, never by metres TurnPilot asked (a roll-up, a creep)."""
  for nudge_m in (1.5, 2.2, 3.0, 4.0):
    car = Car([(150., 'stop')], v=5.)
    car.lead_stop = 146.5
    car.run_until(held, stop=StopCommand('wait'))
    target = car.policy.held_control().d
    assert target > 3.
    car.lead_stop = None
    wait = StopCommand('wait', '', 'creeping')
    car.run(0.5, stop=wait, creep=(target, 3.0))  # the roll-up under way
    s0 = car.s
    while car.s < s0 + nudge_m:
      car.step(gas=True, accel=0.4 if car.v < 0.5 else 0., stop=wait, creep=(target, 3.0))
    assert car.s < 150., nudge_m
    assert car.policy.held is not None, f"let go {150. - car.s:.2f} m short of the line by a {nudge_m} m gas nudge"
    rows = [car.step(stop=wait, creep=(target, 3.0)) for _ in range(int(10. / DT_MDL))]
    assert car.policy.held is not None and car.s <= 150.3 and car.v < 0.05, nudge_m  # rolled up to the line, held there
    assert any(dec.should_stop for _, dec, _ in rows[-20:]) and getattr(car.policy, 'driven', None) is None, nudge_m
  # the left's creep for a view asked 4 m; the driver's gas moves the car 2.2 m inside that ask, past the hold line: still
  # TurnPilot's, held (skeptic: 2.63 m past the line, let go)
  car = left_car()
  held_left(car, left_cmd(reason="Waiting for a gap"))
  line = car.s
  creep = left_cmd(creep_m=4., reason="Creeping for a view")
  run_left(car, 1., creep)
  s0 = car.s
  while car.s < s0 + 2.2:
    car.step(gas=True, accel=0.4 if car.v < 0.6 else 0., left=creep)
  assert car.s - line > 2. and car.policy.held is not None, f"let go {car.s - line:.2f} m past the line inside a 4 m creep"
  out, dec, _ = run_left(car, 5., creep)
  assert held(out, dec) and car.policy.driven is None and car.s - line < 4.3


def test_a_line_crossed_under_the_drivers_gas_is_said_until_the_car_is_through():
  """Owner decision (b), 2026-10-30 (skeptic SG-residual-main): the driver's gas taking the car past a stop line, or a left's
  hold line, with no go of TurnPilot's let the latch go and left the crossing silent: re-engaged, the car drove through the
  junction with no check and no prompt. Now its take-control (stopSignTakeControl, unprotectedTurnTakeControl) and its
  reason stay up, engaged or not, until the car is DRIVEN_THROUGH_M past the stop point, a re-engage inside the junction
  hearing it again; nothing holds or brakes the car for it. The same for a stop sign overrun on the approach."""
  for kind in ('stop', 'left'):
    if kind == 'left':
      car = left_car()
      held_left(car, left_cmd(reason="Waiting for a gap"))
      line, cmd, prompt = 250. - HOLD_LINE_M - 1., {'left': left_cmd(reason="Waiting for a gap")}, 'unprotectedTurnTakeControl'
      reason = "Unprotected left not checked: take control"
    else:
      car = Car([(150., 'stop')])
      car.run_until(held, stop=StopCommand('wait'))
      line, cmd, prompt = 150., {'stop': StopCommand('wait')}, 'stopSignTakeControl'
      reason = "Stop sign passed unchecked: take control"
    while car.s < line + 3.:  # the driver crawls past the line under the gas (openpilot disengaged), and lifts
      car.step(gas=True, accel=0.4 if car.v < 1. else 0., **cmd)
    assert car.policy.held is None, kind
    rows = []
    while car.s < line + DRIVEN_THROUGH_M + 5.:  # re-engaged rolling: the model drives on through the junction
      rows.append((car.s,) + car.step(accel=0.5, **cmd))
    inside = [(out, dec) for s, out, dec, _ in rows if s < line + DRIVEN_THROUGH_M - 0.5]
    assert inside and all(out.prompt == prompt and out.driven == reason for out, _ in inside), kind
    assert not any(dec.should_stop or dec.speed_cap < car.v - 1. for _, dec in inside), kind  # said, never held
    assert all(out.prompt == 'none' and not out.driven for s, out, _, _ in rows if s > line + DRIVEN_THROUGH_M + 0.5), kind
  # a gas nudge, then the car rolling on past the latch's reach (no gas: a coast, a creep mode): the driver's gas moved it,
  # so it is theirs, never a hold in the crossing lanes on a re-engage; said the same
  car = Car([(150., 'stop')])
  car.run_until(held, stop=StopCommand('wait'))
  s0 = car.s
  while car.s < s0 + 0.5:
    car.step(gas=True, accel=0.4, stop=StopCommand('wait'))
  while car.s < 150. + 3.5:
    car.step(accel=0. if car.v > 0.3 else 0.2, stop=StopCommand('wait'))  # disengaged: the plan isn't applied
  out, dec, _ = car.step(stop=StopCommand('wait'))
  assert car.policy.held is None and not dec.should_stop and out.prompt == 'stopSignTakeControl'
  # overrun on the approach: the driver's gas over the line, lifting past the latch's reach
  car = Car([(200., 'stop')], v=1.5, cruise=1.5)
  car.s = 180.
  car.run_until(lambda out, dec: out.stops)
  car.run_until(lambda out, dec: car.s > 203., gas=True, accel=0.)
  out, dec, _ = car.step()
  assert out.prompt == 'stopSignTakeControl' and out.driven == "Stop sign passed unchecked: take control" and not dec.should_stop
  car.run_until(lambda out, dec: car.s > 200. + DRIVEN_THROUGH_M + 1., accel=0.5)
  assert car.step()[0].prompt == 'none'


def test_a_gas_nudge_past_a_red_and_a_re_engage_still_stops_for_the_red():
  # skeptic residual-check-1 (058df13, d): a red held at its line, the driver nudges 2.5 m at 0.8 m/s under the gas and
  # re-engages under the red: let go as drivenOff, the car ran the red at 9.8 m/s. A signal's stop is never the driver's to
  # release by a crawl: only a steady green, or driving off past DRIVE_OFF_M faster than COMMIT_SPEED
  def red(car):
    return lights('red', 150. - car.s)
  car = Car([(150., 'signal')])
  car.run_until(held, lights=red)
  car.run(2., lights=red)
  s0 = car.s
  while car.s < s0 + 2.5:
    car.step(gas=True, accel=0.4 if car.v < 0.8 else 0., lights=red)
  rows = [car.step(lights=red) for _ in range(int(6. / DT_MDL))]
  assert car.policy.held is not None and rows[-1][1].should_stop and car.v < 0.3 and car.s < s0 + 3.5
  # the green lets it go, as before
  car.run_until(lambda out, dec: not dec.should_stop, max_s=3., lights=lambda car: lights('green', 150. - car.s))
  assert car.policy.held is None


def test_an_unprotected_left_hands_back_at_its_hold_line_and_no_gas_goes():
  car = left_car()
  out, dec = car.run_until(held)
  assert out.prompt == 'unprotectedTurnTakeControl' and dec.reason == "Unprotected left: take control"
  car.step(gas=True, accel=0.)
  out, dec, _ = car.run(2.)
  assert held(out, dec) and out.prompt == 'unprotectedTurnTakeControl' and not out.released_stopped


def test_a_yield_is_passed_only_by_passing_it():
  car = Car([(150., 'yield')])
  car.run_until(lambda out, dec: out.stops, yields=rolling_ok)
  out, dec, _ = car.step(gas=True, yields=rolling_ok)
  assert out.stops and out.prompt == 'yieldAhead'


# --- the autonomous unprotected left (unprotected_left.py, DESIGN phase 4 G4): the hold line's latch takes its commands ---

def left_cmd(mode='auto', go=False, creep_m=0., take_control=False, reason='', **kw):
  from openpilot.selfdrive.turnpilot.map_controls import LeftCommand
  return LeftCommand(mode, go, creep_m, take_control, reason, **kw)


def run_left(car, seconds, cmd, **kw):
  res = None
  for _ in range(int(round(seconds / DT_MDL))):
    res = car.policy_step(cmd, **kw)
  return res


def policy_step(self, left, gas=False):
  """Car.step with the left's command handed to the policy; gas: the driver's foot (1.5 m/s^2), the plan not applied."""
  due = self.frame % 20 == 0
  self.frame += 1
  turn = None if self.turn is None else (self.turn[1], self.turn[0] - self.s, self.turn[2])
  out = self.policy.update(self.nav() if due else None, due, self.nav_valid, self.v, gas, NO_LIGHTS, DT_MDL, turn=turn, left=left,
                           engaged=not gas)
  dec = decide(out.stops, self.v)
  dec.accel_cap = self.scene.ramp(dec.accel_cap, DT_MDL)
  cmd = min(plan_accel(dec.accel_cap, dec.speed_cap, self.v), (self.cruise - self.v) / 2., 1.5)
  if dec.should_stop:
    cmd = min(cmd, -1.)
  if gas:
    cmd = 1.5
  self.v = max(self.v + cmd * DT_MDL, 0.)
  self.s += self.v * DT_MDL
  self.t += DT_MDL
  self.rows.append((self.t, self.s, self.v, cmd, dec, out))
  return out, dec, cmd


Car.policy_step = policy_step


def held_left(car, cmd):
  for _ in range(int(60. / DT_MDL)):
    out, dec, _ = car.policy_step(cmd)
    if held(out, dec):
      return out, dec
  raise AssertionError("never held")


def test_auto_left_waits_at_its_line_without_handing_back():
  # no intervention (Kellen, 2026-10-28): held at the line for the left's go, however long, never a take-control
  car = left_car()
  out, dec = held_left(car, left_cmd(reason="Waiting for a gap"))
  assert out.prompt == 'none' and out.shown.action == 'held' and dec.reason == "Waiting for a gap"
  s_held = car.s
  out, dec, _ = run_left(car, 120., left_cmd(reason="Waiting for a gap"))
  assert held(out, dec) and out.prompt == 'none' and car.s - s_held < 0.05


def test_auto_left_go_releases_the_latch_with_the_car_stopped():
  car = left_car()
  held_left(car, left_cmd())
  out, dec, _ = car.policy_step(left_cmd(go=True))
  assert out.released_stopped and not dec.should_stop and out.shown is None or out.shown.kind != UNPROTECTED or \
    out.shown.action == 'released'
  # released for good: nothing of the latch comes back however the car stands after
  out, dec, _ = run_left(car, 5., left_cmd())
  assert not dec.should_stop and not any(s.should_stop for s in out.stops)


def test_auto_left_creeps_slowly_to_its_target_and_holds_there():
  car = left_car()
  held_left(car, left_cmd())
  s0 = car.s
  from openpilot.selfdrive.turnpilot.map_controls import CREEP_ACCEL, CREEP_SPEED
  out, dec, _ = car.policy_step(left_cmd(creep_m=2., reason="Creeping for a view"))
  assert not dec.should_stop and dec.speed_cap <= CREEP_SPEED + 1e-6 and dec.accel_cap <= CREEP_ACCEL + 1e-6
  assert dec.reason == "Creeping for a view" and out.prompt == 'none'
  out, dec, _ = run_left(car, 10., left_cmd(creep_m=2., reason="Creeping for a view"))
  assert held(out, dec) and 1.7 < car.s - s0 < 2.2 and max(r[2] for r in car.rows[-200:]) <= CREEP_SPEED + 0.05
  # frozen where it is (a car came into the band): held there, never on
  s1 = car.s
  out, dec, _ = run_left(car, 5., left_cmd(creep_m=car.s - s0))
  assert held(out, dec) and car.s - s1 < 0.05


def test_auto_left_undecided_hands_back_held():
  car = left_car()
  held_left(car, left_cmd())
  out, dec, _ = car.policy_step(left_cmd(take_control=True, reason="View of oncoming traffic blocked: take control"))
  assert held(out, dec) and out.prompt == 'leftUndecidedTakeControl' and out.shown.action == 'awaitingDriver'
  assert dec.reason == "View of oncoming traffic blocked: take control"
  # the driver takes over and drives off: theirs, no overrun
  run_left(car, 3., left_cmd(take_control=True), gas=True)
  assert car.policy.turn.state.done == 'released' and car.policy._alert == 'none'


def test_announce_and_off_hand_back_at_the_line_as_before():
  for cmd, reason in ((left_cmd('announce', reason="Unprotected left: would go"), "Unprotected left: would go: take control"),
                      (left_cmd('off'), "Unprotected left: take control"), (None, "Unprotected left: take control")):
    car = left_car()
    out, dec = held_left(car, cmd)
    assert out.prompt == 'unprotectedTurnTakeControl' and dec.reason == reason and out.shown.action == 'awaitingDriver'
    out, dec, _ = car.policy_step(left_cmd('announce', go=True) if cmd is not None else None)
    assert held(out, dec)  # announce never goes


def test_the_turn_light_is_the_signal_at_the_lefts_junction_from_any_source():
  # the autonomous left's go needs its junction's light (unprotected_left.py; skeptic G4 blocker 2): 'none' without a signal
  # there; otherwise every source's reads around the junction, the camera's in shadow too (a veto never brakes), red over
  # yellow over green, unknown with no read. A signal seen there stays the junction's after it drops out behind the car
  car = left_car()
  car.run(1.)
  assert car.policy.turn_light(7, 250. - car.s) == 'none'
  car = left_car([(250. - 11., 'signal')])
  car.run(1., lights=lambda car: lights('green', 239. - car.s))
  assert car.policy.turn_light(7, 250. - car.s) == 'green'
  car.run(0.1, lights=lambda car: lights('red', 239. - car.s))
  assert car.policy.turn_light(7, 250. - car.s) == 'red'
  car.run(0.1)
  assert car.policy.turn_light(7, 250. - car.s) == 'unknown'
  shadow = Lights(None, 'none', camera=CameraLights([('red', 250. + 20. - car.s)], shadow=True))
  car.run(0.1, lights=shadow)
  assert car.policy.turn_light(7, 250. - car.s) == 'red'
  both = Lights([('green', 239. - car.s)], 'simOracle', camera=CameraLights([('yellow', 270. - car.s)], shadow=True))
  car.run(0.1, lights=both)
  assert car.policy.turn_light(7, 250. - car.s) == 'yellow'
  # a light past the junction's window (the next junction on) isn't its
  car.run(0.1, lights=Lights(None, 'none', camera=CameraLights([('red', 250. + 60. - car.s)], shadow=True)))
  assert car.policy.turn_light(7, 250. - car.s) == 'unknown'
  # another maneuver's junction isn't this one's
  assert car.policy.turn_light(8, 400.) == 'none'
  # navd's controls failing (no junction data is never clear): whether there is a signal isn't known, so its light isn't
  car = left_car()
  car.status = 'failed'
  car.run(1.)
  assert car.policy.turn_light(7, 250. - car.s) == 'unknown'


def test_a_red_or_yellow_at_a_junction_with_no_mapped_signal_is_read_for_the_left_and_any_release():
  # skeptic G4/G5 (residual): turn_light and release_light returned 'none' wherever no signal was mapped at the junction,
  # before reading any light, so a red the camera (or the sim's oracle) read at an unmapped signal let the left go and
  # plannerd lift the car through it. A red or yellow from any source from PROTECT_BEFORE_M before the junction to
  # CAMERA_ASSOC_PAST_M past it counts, mapped or not; a green there is no signal TurnPilot knows of ('none', as before)
  car = left_car()
  car.run(1.)
  d = 250. - car.s
  for state in ('red', 'yellow'):
    car.run(0.1, lights=Lights(None, 'none', camera=CameraLights([(state, 250. - 10. - car.s)], shadow=True)))
    d = 250. - car.s
    assert car.policy.turn_light(7, d) == state and car.policy.release_light(d) == state
    car.run(0.1, lights=lambda car, state=state: lights(state, 250. + 15. - car.s))
    d = 250. - car.s
    assert car.policy.turn_light(7, d) == state and car.policy.release_light(d) == state
  car.run(0.1, lights=Lights(None, 'none', camera=CameraLights([('green', 250. - 10. - car.s)], shadow=True)))
  d = 250. - car.s
  assert car.policy.turn_light(7, d) == 'none' and car.policy.release_light(d) == 'none'
  # outside the junction's window (the next junction's light): not this junction's
  car.run(0.1, lights=Lights(None, 'none', camera=CameraLights([('red', 250. + 60. - car.s)], shadow=True)))
  d = 250. - car.s
  assert car.policy.turn_light(7, d) == 'none' and car.policy.release_light(d) == 'none'


def rail(rail_m, stop_point_m=10.):
  """A stop's junctionInfo near a mapped rail crossing: railDistance from its node, stopPointDistance from its stop point."""
  return {'railDistance': rail_m, 'stopPointDistance': stop_point_m}


def second_stop_after_a_handback_at_the_first(controls, drive_s=2.):
  """Held and handed back at the first stop, the driver takes over and goes for drive_s, then the plan applies again: (where
  it applied, where the car was held next or None, what was said after)."""
  car = Car(controls)
  car.run_until(held)
  car.run(STOP_SIGN_HANDBACK_S + 0.3)
  assert car.rows[-1][-1].prompt == 'stopSignTakeControl'
  drive_off(car, drive_s)
  s_engage, said = car.s, set()
  for _ in range(int(30. / DT_MDL)):
    out, dec, _ = car.step()
    said |= {x for x in (out.alert, out.prompt) if x and x != 'none'}
    if held(out, dec):
      return s_engage, car.s, said
  return s_engage, None, said


def test_a_released_stop_never_releases_a_stop_across_rail_tracks():
  # residual-check-2: _release let go of every non-yield control up to RELEASE_SPAN_M past a released one, so after a handback
  # at a junction's stop (Indianapolis 17464804:b) the driver's go released the rail orphan's stop 29.6 m on (the tracks 8 m
  # past it), and the reverse (a junction's stop 20 m past a rail orphan's), both silently: re-engaged before them the car
  # crossed at 13.4 m/s with nothing said. A control near a mapped rail crossing (junctionInfo.railDistance >= 0) is never
  # released by another's release, and releasing one releases nothing past it: each stops the car itself
  s_engage, stopped, said = second_stop_after_a_handback_at_the_first(
    [(126.5, 'stop', False, True, False, rail(25., 12.5)), (156.1, 'stop', False, False, False, rail(7.9, 37.9))])
  assert s_engage < 150. and stopped is not None and stopped < 164. - 4.6, (s_engage, stopped, said)
  s_engage, stopped, said = second_stop_after_a_handback_at_the_first(
    [(120., 'stop', False, False, False, rail(6., 30.)), (140., 'stop', False, True, False, {'railDistance': -1.})])
  assert stopped is not None and abs(140. - stopped) < 5., (s_engage, stopped, said)
  # with no rail anywhere, a second line RELEASE_SPAN_M on is the same junction's (a divided road): released with the first
  s_engage, stopped, said = second_stop_after_a_handback_at_the_first([(126.5, 'stop'), (150., 'stop')])
  assert stopped is None


def test_a_rail_stop_before_a_left_never_protects_it():
  # residual-check-2: protected() counted a rail orphan's stop up to PROTECT_BEFORE_M before a junction as its own, so a route
  # left there with no stop sign of its own was taken as protected and never held for its gap. A stop whose rail crossing lies
  # between it and its node (railDistance under stopPointDistance) is the tracks', never the junction's
  orphan = (225., 'stop', False, False, False, rail(18., 25.))
  car = left_car([orphan])
  car.run_until(held)
  car.run(STOP_SIGN_HANDBACK_S + 0.5)
  drive_off(car, 2.)
  out, dec = car.run_until(held, max_s=20.)
  assert out.shown.kind == UNPROTECTED and 250. - HOLD_LINE_M - car.s < 3.
  # the junction's own stop sign, near tracks past it, still protects its left (one stop, the stop sign's)
  car = left_car([(240., 'stop', False, True, False, rail(20., 10.))])
  car.run_until(held)
  assert car.policy.held_control().kind == 'stop' and car.policy.turn is None


# --- the stop-sign decider's latch (STOP_SIGNS.md v3 SG4a: stop_go.py through turnpilotd) ---

def stand_at(car, line):
  car.run_until(lambda out, dec: held(out, dec))
  return car.policy.held_control()


def test_held_control_is_the_latched_stop_and_survives_a_reroute_by_its_state():
  car = Car([(150., 'stop'), (400., 'signal')])
  assert car.policy.held_control() is None
  c = stand_at(car, 150.)
  assert c is not None and c.kind == 'stop' and c.held and c.key == car.policy.held
  # a reroute: the new route's same control takes the latch over under its own key, the state object carried (adopt)
  car.route_count += 1
  car.frame = 0
  car.run(0.1, stop=StopCommand('wait'))
  c2 = car.policy.held_control()
  assert c2 is not None and c2.key != c.key and c2.state is c.state


def test_under_the_decider_a_held_stop_sign_waits_without_the_1_s_handback_and_its_faults_hand_back():
  car = Car([(150., 'stop')])
  stand_at(car, 150.)
  out, dec, _ = car.run(STOP_SIGN_HANDBACK_S + 3., stop=StopCommand('wait', "Waiting: a car from the left", 'waitingCross'))
  assert dec.should_stop and out.prompt == 'none' and out.shown.action == 'waitingCross' and dec.reason == "Waiting: a car from the left"
  out, dec, _ = car.run(0.1, stop=StopCommand('fault', "Cross-traffic camera unavailable: take control"))
  assert held(out, dec) and out.prompt == 'stopSignTakeControl' and dec.reason == "Cross-traffic camera unavailable: take control"
  assert out.shown.action == 'awaitingDriver'
  # as shadow (H4, H6, H8): the 1 s handback exactly as before, its reason saying why
  car = Car([(150., 'stop')])
  stand_at(car, 150.)
  out, dec, _ = car.run(STOP_SIGN_HANDBACK_S - 0.2, stop=StopCommand('shadow', "Stop-sign go off: no camera measurements"))
  assert held(out, dec) and out.prompt == 'none'
  out, dec, _ = car.run(0.3, stop=StopCommand('shadow', "Stop-sign go off: no camera measurements"))
  assert out.prompt == 'stopSignTakeControl' and dec.reason == "Stop-sign go off: no camera measurements"


def test_the_deciders_clear_releases_the_latch_through_the_lefts_mechanism_and_never_the_stop_past_it():
  # G11: a 'clear' releases only its own control, never a stop RELEASE_SPAN_M past it (a divided road's second line has its own
  # decision); the release lets the car go standing (released_stopped)
  car = Car([(150., 'stop'), (165., 'stop')])
  c = stand_at(car, 150.)
  car.run(1.5, stop=StopCommand('wait'))
  out, dec, _ = car.step(stop=StopCommand('wait'), release=(c.key, 'clear'))
  assert out.released_stopped and not dec.should_stop and car.policy.held_control() is None
  second = next(o for o in car.policy.tracker.controls.values() if o.key != c.key)
  assert not (second.state is not None and second.state.done)
  # a release for another key, or none, lets nothing go
  car = Car([(150., 'stop')])
  c = stand_at(car, 150.)
  out, dec, _ = car.run(1., stop=StopCommand('wait'), release=((99, 99), 'clear'))
  assert held(out, dec)


def test_a_held_stop_signs_stop_point_moves_on_by_the_deciders_creep_on_the_map_curve_never_should_stop():
  # O2, V1: latched 4 m short, rolled up on the map curve (up to the creep's cap) with shouldStop false, held again there
  car = Car([(150., 'stop')], v=5.)
  car.lead_stop = 146.5
  stand_at(car, 150.)
  c = car.policy.held_control()
  assert c.d > 3.
  car.lead_stop = None
  target = c.d
  rows = []
  for _ in range(int(8. / DT_MDL)):
    out, dec, _ = car.step(stop=StopCommand('wait', '', 'creeping'), creep=(target, 3.0))
    rows.append((car.v, out, dec))
  assert all(not dec.should_stop for v, out, dec in rows[:5]) and max(v for v, _, _ in rows) <= 3.0 + 1e-6
  assert any(s.speed_cap > 0. for s in rows[2][1].stops) and rows[2][1].shown.action == 'creeping'
  assert abs(c.state.moved_m - target) <= 0.3 and rows[-1][2].should_stop
  # a creep keeps its slow cap (CREEP_SPEED): the left's own
  from openpilot.selfdrive.turnpilot.map_controls import CREEP_SPEED
  car = Car([(150., 'stop')], v=5.)
  car.lead_stop = 146.5
  stand_at(car, 150.)
  car.lead_stop = None
  vs = [car.step(stop=StopCommand('wait'), creep=(3.0, CREEP_SPEED))[0] and car.v for _ in range(int(6. / DT_MDL))]
  assert max(vs) <= CREEP_SPEED + 1e-6


# --- the stop-sign decider's map rows (STOP_SIGNS.md v3 SG4b) ---

def released_through(car, release_key, seconds=25., **kw):
  """The decider's 'clear' for release_key once, then the car on, the model driving (cruise), the decider waiting at whatever
  holds it next."""
  car.step(stop=StopCommand('wait'), release=(release_key, 'clear'), **kw)
  return car.run_until(lambda out, dec: held(out, dec) and car.policy.held_control().key != release_key, max_s=seconds,
                       stop=StopCommand('wait'))


@pytest.mark.parametrize('gap', [15., 20., 30.])
def test_G11_G18_a_clear_releases_its_own_line_only_and_the_next_line_stops_and_holds_the_car_on_its_own(gap):
  # G11 (a divided road's two lines) and G18 (a frontage pair, 15-30 m apart): each node's own control, decision and latch;
  # the decider's 'clear' at the first never span-releases the second (RELEASE_SPAN_M 30 m), which stops the car at its own
  # line on the map curve and holds it there, its own key held, under the decider's wait
  car = Car([(150., 'stop'), (150. + gap, 'stop')], v=8.)
  first = stand_at(car, 150.)
  car.run(1.5, stop=StopCommand('wait'))
  car.cruise = 4.
  out, dec = released_through(car, first.key)
  second = car.policy.held_control()
  assert second is not None and second.key != first.key and abs(150. + gap - car.s) < 5. and dec.should_stop
  assert all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)
  # held there under the decider's wait, no 1 s handback
  out, dec, _ = car.run(3., stop=StopCommand('wait'))
  assert held(out, dec) and out.prompt == 'none'


def test_G13_a_stop_line_far_back_is_where_the_car_stops_and_decides_no_second_pull_up():
  # the OSM stop node's or the survey's line is the control's stop point (navd's d): a line set back 20 m from the junction
  # node stops the car there, latched, and it stays there while the decider waits (no creep it didn't ask for)
  car = Car([(150., 'stop', True)])
  c = stand_at(car, 150.)
  s_held = car.s
  assert c.line_mapped and abs(150. - s_held) < 3.
  out, dec, _ = car.run(10., stop=StopCommand('wait', "Waiting: a car from the left", 'waitingCross'))
  assert dec.should_stop and out.shown.action == 'waitingCross' and car.s - s_held < 0.05 and out.shown.line_mapped


@pytest.mark.parametrize('past', [1.5, 3.0])
def test_G14_the_latch_holds_the_car_only_within_its_reach_past_the_line_and_beyond_it_is_an_overrun(past):
  # the pessimistic end: the decider measures commit, creep and the band from the closest plausible edge, from where the
  # latch holds it, and the latch holds only up to LATCH_PAST_M past the stop point: a car (its model, not TurnPilot) braking
  # late from 2 m/s to stand 1.5 m past the line is held there; 3 m past, it is an overrun, said, never a hold in the junction
  from openpilot.selfdrive.turnpilot.map_controls import LATCH_PAST_M
  car = Car([(30., 'stop')], v=2., cruise=2.)

  def accel(c):
    return 0. if c.s < 30. else -2. ** 2 / (2. * past) if c.v > 0. else 0.
  for _ in range(int(20. / DT_MDL)):
    car.step(accel=accel(car))
  held_rows = [r for r in car.rows if r[4].should_stop and r[5].shown is not None and r[5].shown.action == 'held']
  if past <= LATCH_PAST_M:
    assert held_rows and abs(car.s - 30. - past) < 0.3 and car.policy.held_control() is not None
  else:
    assert not held_rows and any(r[5].alert == 'stopOverrun' for r in car.rows) and car.policy.held_control() is None


@pytest.mark.parametrize('grade', [0., 0.2])
def test_G20_a_stop_just_past_a_crest_on_a_downhill_engages_from_the_map_and_lands_in_the_latch_window(grade):
  # the map distance is known before the crest (the stop engages at engage_distance, whatever the camera sees over it), and
  # the -1.5 curve is an accel target: the car's longitudinal control holds it against the hill (openpilot's long control
  # closes on aEgo), here with grade m/s^2 of the hill left over in its lag; it stops within the latch's window and is held,
  # nothing commanded below MAP_DECEL_MAX and no overrun
  car = Car([(200., 'stop')], v=11.)
  orig = car.step

  def step(*a, **kw):
    out, dec, cmd = orig(*a, **kw)
    if not dec.should_stop and car.v > 0.:
      car.v += grade * DT_MDL
    return out, dec, cmd
  car.step = step
  engaged = None
  for _ in range(int(40. / DT_MDL)):
    out, dec, _ = car.step()
    if engaged is None and out.stops:
      engaged = 200. - car.s
    if held(out, dec):
      break
  assert engaged is not None and engaged > 40.  # well before a crest 20 m short of the line
  assert held(out, dec) and -2. < 200. - car.s < 5. and all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)
  assert not any(r[5].alert == 'stopOverrun' for r in car.rows)


def test_C11_a_mapped_stop_the_camera_never_saw_still_stops_gently_and_is_logged_with_the_class_11_evidence(monkeypatch):
  # the gentle error: a stop mapped where there is none (or the camera misses it) still stops the car on the -1.5 curve and
  # holds it (certification at the line proceeds as mapped); at the hold the class-11 shadow evidence of its approach is logged
  # (forward runs, stop-sign boxes, the best conf), and with none seen in them, said as unseen for the replay gate. Never a
  # brake, never a release: nothing here acts on a sign
  import openpilot.selfdrive.turnpilot.map_controls as mcm
  events = []
  monkeypatch.setattr(mcm.cloudlog, 'event', lambda name, **kw: events.append((name, kw)))
  for seen in (False, True):
    events.clear()
    car = Car([(150., 'stop')])
    def signs(c, seen=seen):
      return (1, [(-12., 2., 0.7)] if seen and 150. - c.s < 40. else [])
    out, dec = car.run_until(lambda out, dec: held(out, dec), signs=signs)
    assert all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows) and dec.should_stop
    ev = [kw for name, kw in events if name == 'turnpilot.map_stop_signs']
    assert len(ev) == 1 and ev[0]['runs'] > 50 and (ev[0]['hits'] > 0) == seen and ev[0]['seen'] == seen
    unseen = [kw for name, kw in events if name == 'turnpilot.map_stop_unseen']
    assert bool(unseen) == (not seen)
    # the same stop with the camera's signs not read (off, or no detector): nothing claimed either way
    events.clear()
    car = Car([(150., 'stop')])
    car.run_until(lambda out, dec: held(out, dec), signs=None)
    assert not [1 for name, _ in events if name in ('turnpilot.map_stop_unseen',)]


def test_the_drivers_crossing_past_the_lefts_own_hold_is_said_until_the_car_is_through():
  """Skeptic SIMFIX round 2 (owner decision (b)): after its go let the latch go (cause gap), the left's own hold
  (unprotected_left: hold_here) let go as the driver's says so (LeftCommand.crossed, once): the left's crossing is the
  latch's (Driven), its take-control and reason up, engaged or not, until the car is DRIVEN_THROUGH_M past the line, then
  nothing; never a hold or a cap for it."""
  car = left_car()
  held_left(car, left_cmd(reason="Waiting for a gap"))
  line = 250. - HOLD_LINE_M - 1.
  car.policy_step(left_cmd(go=True))
  assert car.policy.held is None and car.policy.turn.state.cause == 'gap' and car.policy.driven is None
  run_left(car, 1., left_cmd(reason="Turning"))  # gone
  out, dec, _ = run_left(car, 2., left_cmd(take_control=True, reason="Oncoming, holding"))  # its own hold (unprotected_left's)
  assert out.prompt == 'none' and car.policy.driven is None
  out, dec, _ = car.policy_step(left_cmd(take_control=True, crossed='drivenOff'))  # the driver's: said once
  dr = car.policy.driven
  assert dr is not None and dr.kind == UNPROTECTED and dr.maneuver_id == 7 and dr.cause == 'drivenOff'
  assert out.prompt == 'unprotectedTurnTakeControl' and out.driven == "Unprotected left not checked: take control"
  rows = []
  while car.s < line + DRIVEN_THROUGH_M + 5.:
    rows.append((car.s,) + car.policy_step(left_cmd(take_control=True)))
  inside = [(out, dec) for s, out, dec, _ in rows if s < line + DRIVEN_THROUGH_M - 0.5]
  assert inside and all(out.prompt == 'unprotectedTurnTakeControl' and out.driven for out, _ in inside)
  assert not any(dec.should_stop or dec.speed_cap < car.v - 1. for _, dec in inside)
  assert all(out.prompt == 'none' and not out.driven for s, out, _, _ in rows if s > line + DRIVEN_THROUGH_M + 0.5)


@pytest.mark.parametrize("nudge,theirs", [(1.5, True), (1.0, True), (0.4, False)])
def test_a_nudge_on_from_the_lefts_full_creep_is_never_held_in_the_oncoming_lanes(nudge, theirs):
  """The invariant, never a TurnPilot hold standing the car in the oncoming lanes on a re-engage (found fixing SIMFIX round
  2, at e93750b): the latch's reach, LATCH_PAST_M past both its stop point and the creep it asked, lies 3 m from the
  junction after the left's full creep (CREEP_MAX_M on, to ONCOMING_ENTRY_M + CREEP_SHORT_M out), 1 m inside the lanes: a
  1-1.5 m gas nudge on from there and a re-engage held the car standing 3.5-4 m from the centre. At an unprotected turn the
  car moved on past the latch's ask is the driver's too once a hold could no longer stand it short of the lanes (from its
  speed as the latch's own braking takes it, reengage_stop_m; the junction HOLD_LINE_M + MAP_MARGIN_M past the stop point as
  latched), said until through. Short of that, held there (c)."""
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
  junction = 250.
  car = left_car(at=junction)
  held_left(car, left_cmd(reason="Waiting for a gap"))
  creep = left_cmd(creep_m=4., reason="Creeping for a view")
  run_left(car, 12., creep)
  assert abs(junction - car.s - 5.) < 0.35 and car.v < 0.05
  s0 = car.s
  while car.s < s0 + nudge:
    car.step(gas=True, accel=0.4 if car.v < 0.6 else 0., left=creep)
  rows = [car.step(left=creep) for _ in range(int(5. / DT_MDL))]  # re-engaged rolling, the left still asking its creep
  in_lanes = [round(junction - r[1], 2) for r in car.rows[-len(rows):] if r[2] < 0.05 and junction - r[1] < ONCOMING_ENTRY_M]
  assert not in_lanes, f"held standing {in_lanes[0]} m from the junction, in the oncoming lanes"
  if theirs:
    assert car.policy.held is None and car.policy.driven is not None and not any(dec.should_stop for _, dec, _ in rows)
    assert all(out.prompt == 'unprotectedTurnTakeControl' for out, _, _ in rows)
  else:
    assert car.policy.held is not None and car.policy.driven is None and car.v < 0.05 and junction - car.s > ONCOMING_ENTRY_M


def idle_creep(car, line, past, **kw):
  """openpilot disengaged (the plan not applied) and the brake let off: the car idles on at 1.2 m/s with no gas to past m
  beyond line, then is engaged again (the plan applied, the model driving on). (the rows of the re-engage, the causes of
  the crossings said)"""
  causes = set()
  while car.s < line + past:
    car.step(accel=0.5 if car.v < 1.2 else 0., **kw)
    causes |= {car.policy.driven.cause} if car.policy.driven is not None else set()
  n = len(car.rows)
  for _ in range(int(10. / DT_MDL)):
    car.step(**kw)
    causes |= {car.policy.driven.cause} if car.policy.driven is not None else set()
  return car.rows[n:], causes


@pytest.mark.parametrize("past", [3., 5., 8.])
def test_an_idle_creep_past_a_held_line_with_no_gas_is_the_drivers_never_held_in_the_crossing_lanes(past):
  """Owner decision (i), 2026-10-30, Kellen's delegate (srm test_srm_idle_creep_reengage, left open at e93750b): held and
  handed back, or waiting, the driver disengages on the brake and lets the car idle on with no gas past the line, then
  re-engages: only gas metres counted as the driver's, so the latch held the car again where it stood, 3-8 m past a stop
  sign's line or a left's hold line, in the crossing or the oncoming lanes. Any movement past the stop point and what
  TurnPilot asked of the latch is the driver's for (a) and (b): let go past the latch's reach (or, at an unprotected turn,
  where a hold could no longer stand the car short of the oncoming lanes), driverWent handed back, else movedOff, never
  held in the junction on the re-engage, its crossing said until the car is DRIVEN_THROUGH_M past the line."""
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
  cases = [('stop shadow', lambda: Car([(150., 'stop')]), 150., {}, 'stopSignTakeControl', 'driverWent'),
           ('stop waiting', lambda: Car([(150., 'stop')]), 150., {'stop': StopCommand('wait')}, 'stopSignTakeControl', 'movedOff')]
  line = 250. - HOLD_LINE_M - 1.
  for name, cmd, cause in (('left off', left_cmd('off'), 'driverWent'),
                           ('left undecided', left_cmd(take_control=True, reason="View of oncoming traffic blocked: take control"),
                            'driverWent'),
                           ('left waiting', left_cmd(reason="Waiting for a gap"), 'movedOff')):
    cases.append((name, left_car, line, {'left': cmd}, 'unprotectedTurnTakeControl', cause))
  for name, make, at, kw, prompt, cause in cases:
    car = make()
    car.run_until(held, **kw)
    car.run(1.5, **kw)
    rows, causes = idle_creep(car, at, past, **kw)
    assert car.policy.held is None, f"{name}: still latched {car.s - at:.2f} m past its line on the re-engage"
    assert causes == {cause}, (name, causes)
    assert not any(r[4].should_stop for r in rows), name
    if 'left' in name:
      assert not any(r[2] < 0.05 and 250. - r[1] < ONCOMING_ENTRY_M for r in rows), name
    dr = [r[5] for r in car.rows if r[5].driven]
    assert dr and car.policy.driven is None and car.s > at + DRIVEN_THROUGH_M + 1., name  # said, and done once through
    inside = [r[5] for r in rows if r[1] < at + DRIVEN_THROUGH_M - 0.5]
    assert inside and all(out.prompt == prompt and out.driven for out in inside), (name, sorted({out.prompt for out in inside}))
    assert all(r[5].prompt == 'none' for r in rows if r[1] > at + DRIVEN_THROUGH_M + 0.5), name


def test_an_idle_creep_short_of_the_latchs_reach_stays_held_for_the_full_check():
  # owner decision (c): rolled on with no gas less than LATCH_PAST_M past the line, the latch is TurnPilot's still: the
  # re-engage holds the car where it stands, handed back as before, never driven on unchecked
  car = Car([(150., 'stop')])
  car.run_until(held)
  car.run(1.5)
  while car.s < 150. + 1.5:
    car.step(accel=0.5 if car.v < 1.2 else 0.)
  out, dec, _ = car.run(5.)
  assert held(out, dec) and out.prompt == 'stopSignTakeControl' and car.v < 0.05 and car.s < 150. + 2.5
  assert car.policy.driven is None


def test_a_holds_braking_after_a_re_engage_is_modelled_on_what_it_asks_never_on_its_accel_cap():
  """Skeptic round 1 (tp/left-hold): the lanes test (a hold that could no longer stand the car short of the oncoming lanes)
  took a hold re-engaged rolling to brake at REENGAGE_DECEL, -HOLD_ACCEL (0.5 m/s^2): the held car's accel cap, not what a
  hold brakes at. From 1.92 and 2.12 m/s it put the car 3.7 and 4.5 m on, where the left's own hold and the latch stood it
  in 1.66 and 1.55 m, so it let go of holds that would have stood the car short of the lanes. reengage_stop_m follows the
  hold's own caps through plan_accel, the car first rolling on REENGAGE_LAG_S: never short of where the hold stands the car
  (the sims' braking, shouldStop's -1 m/s^2 floor and all), and never far past it."""
  from openpilot.selfdrive.turnpilot.map_controls import HOLD_ACCEL, REENGAGE_LAG_S, latch_hold_caps, reengage_stop_m
  from openpilot.selfdrive.turnpilot.scene_policy import Stop, stop
  from openpilot.selfdrive.turnpilot.unprotected_left import hold_caps

  def stands(v, held):
    """m the hold held(v) (a Stop) stands a car rolling at v in, the plan applied as the sims apply it."""
    d = 0.
    while v > 0.:
      dec = decide([held(v)], v)
      a = plan_accel(dec.accel_cap, dec.speed_cap, v)
      if dec.should_stop:
        a = min(a, -1.)
      v = max(v + a * DT_MDL, 0.)
      d += v * DT_MDL
    return d
  holds = ((hold_caps, lambda v: stop(v, 0., '', confirmed=False)),  # unprotected_left's own hold
           (latch_hold_caps, lambda v: Stop(HOLD_ACCEL, 0., True, '', 0., rest=True)))  # the latch's
  for v in (0.2, 0.5, 0.8, 1.2, 1.5, 1.92, 2.12, 2.4, 3., 4., 6.):
    for caps, held_at in holds:
      model, actual = reengage_stop_m(v, caps), stands(v, held_at)
      assert actual + v * REENGAGE_LAG_S <= model <= actual + v * REENGAGE_LAG_S + 0.35, (v, caps.__name__, round(model, 2),
                                                                                         round(actual, 2))
  # the skeptic's two re-engages, well short of 0.5 m/s^2's 3.7 and 4.5 m
  assert reengage_stop_m(1.92, hold_caps) < 2.4 and reengage_stop_m(2.12, latch_hold_caps) < 2.6
  assert reengage_stop_m(0., hold_caps) == 0.


def creep_then_gas(gas_accel, gas_to=1.9, creep_m=2., seconds=8.):
  """left_car held at its line, then the left asking a creep_m creep for a view; 0.5 m into it the driver's gas (openpilot
  disengaged) at gas_accel takes the car on to gas_to m past where it latched, and they lift and re-engage rolling, the left
  still asking its creep, for seconds. (car, s where it latched, rows from the gas on: (engaged, still latched, the row))"""
  car = left_car()
  cmd = left_cmd(creep_m=0.)
  car.run_until(held, left=cmd)
  car.run(2., left=cmd)
  s_held = car.s
  cmd = left_cmd(creep_m=creep_m, reason="Creeping for a view")
  while car.s - s_held < 0.5:
    car.step(left=cmd)
  rows = []
  while car.s - s_held < gas_to and car.policy.held is not None:
    car.step(left=cmd, gas=True, accel=gas_accel)
    rows.append((False, car.policy.held is not None, car.rows[-1]))
  for _ in range(int(seconds / DT_MDL)):
    car.step(left=cmd)
    rows.append((True, car.policy.held is not None, car.rows[-1]))
  return car, s_held, rows


def test_the_latch_reengaged_inside_its_creep_is_never_let_go_on_its_own_metres():
  """Skeptic round 1 (tp/left-hold): the driver's gas (disengaged) inside the left's 2 m creep, then a re-engage rolling at
  2.1 m/s 1.96 m past the latch: drv = min(past, driver_m) took the latch's own stop past the creep (engaged) for the
  driver's metres, since the driver had banked metres inside it, and the lanes test at 0.5 m/s^2 let go at the first engaged
  frame, 3.09 m short of the oncoming lanes; the car drove off at cruise into the junction, the full check (c) skipped.
  e93750b held it 1.75 m short of them. Held, standing short of the lanes, waiting; never let go engaged."""
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
  car, s_held, rows = creep_then_gas(1.5)
  lift = [r for engaged, _, r in rows if not engaged][-1]
  assert 1.9 <= lift[1] - s_held < 2. and 2. < lift[2] < 2.2  # inside the creep, rolling at about 2.1 m/s
  assert all(latched for _, latched, _ in rows), "re-engaged inside its creep: let go on the latch's own metres"
  assert car.v < 0.05 and 250. - ONCOMING_ENTRY_M - car.s > 1. and car.policy.driven is None
  assert all(r[5].prompt == 'none' for _, _, r in rows[-20:]) and rows[-1][2][4].should_stop


def test_the_latch_never_counts_its_own_stop_past_the_creep_as_the_drivers_whatever_the_order():
  """Skeptic round 1 (tp/left-hold), the order the earlier tests missed: the driver's metres inside the creep, the latch's
  own stop (engaged) carrying the car on past it, then a nudge of the driver's under the reach. drv = min(past, driver_m)
  added up the two and let go 1.1 m into the nudge. Owner decision (a): the driver's metres are those they move the car
  past the furthest TurnPilot asked of the latch, never inside it nor TurnPilot's own stop: held for the full check (c),
  let go only once their own metres beyond it pass the reach (LATCH_PAST_M) or the oncoming lanes come too near."""
  from openpilot.selfdrive.turnpilot.map_controls import LATCH_PAST_M
  car, s_held, rows = creep_then_gas(0.6, gas_to=1.95, seconds=5.)
  carried = car.s - s_held - 2.
  assert all(latched for _, latched, _ in rows) and car.v < 0.05 and 0.6 < carried < 1.2, carried  # stood past the creep
  stood = car.s
  cmd = left_cmd(creep_m=2., reason="Creeping for a view")
  while car.s < stood + 1.4:
    car.step(left=cmd, gas=True, accel=0.4 if car.v < 0.5 else 0.)
  nudged = car.s - stood
  car.run(4., left=cmd)  # re-engaged: the latch stops the nudge, its own metres
  assert car.policy.held is not None and car.policy.driven is None and car.v < 0.05
  assert carried + nudged > LATCH_PAST_M  # what min(past, driver_m) let go on
  stood, let_go = car.s, None
  while car.s < stood + 3. and let_go is None:
    car.step(left=cmd, gas=True, accel=0.4 if car.v < 0.5 else 0.)
    let_go = nudged + car.s - stood if car.policy.held is None else None  # the driver's own metres past the creep
  assert let_go is not None and LATCH_PAST_M < let_go < LATCH_PAST_M + 0.1, let_go
  assert car.policy.driven is not None and car.policy.driven.cause == 'drivenOff'


def test_the_latch_reengaged_too_fast_inside_its_creep_is_let_go_at_the_re_engage_never_on_its_own_braking():
  """The invariant, never a TurnPilot hold that stands the car in the oncoming lanes on a re-engage, without a release on
  TurnPilot's metres: the driver's gas inside the creep to about 3.5 m/s, from where the latch taken up by a re-engage stands
  the car inside the lanes. The lanes test was reached only past the creep, so the latch was let go engaged, on its own
  braking. Let go at the re-engage itself, judged from its frame's speed before the latch brakes the car (skeptic round 2:
  never for good on a driver's frame), its decision no stop, its crossing said (Driven)."""
  car, s_held, rows = creep_then_gas(4.5)
  first = next(k for k, (_, latched, _) in enumerate(rows) if not latched)
  reengage = next(k for k, (engaged, _, _) in enumerate(rows) if engaged)
  _, _, row = rows[first]
  # the re-engage, lifted about where the creep ends, far short of the reach: only the lanes test lets it go
  assert first == reengage and rows[first - 1][2][1] - s_held < 2.2, (first, reengage)
  assert not any(r[4].should_stop or r[4].speed_cap < r[2] - 0.5 for _, _, r in rows[first:])  # never braked by the latch
  assert row[5].driven and row[5].prompt == 'unprotectedTurnTakeControl'
  inside = [r for e, _, r in rows[first:] if e and r[1] < 250. - HOLD_LINE_M - 1. + DRIVEN_THROUGH_M - 1.]
  assert inside and all(r[5].prompt == 'unprotectedTurnTakeControl' and not r[4].should_stop for r in inside)


def test_metres_the_driver_moved_past_the_latch_that_a_later_creep_takes_in_are_turnpilots_again():
  """The driver's metres past the latch count only while past what TurnPilot asks (min(past, driver_past_m)): a 1 m gas
  nudge from where it latched (held again, the left waiting), then the left's 2 m creep for a view takes the car on over
  them, and a 1.6 m nudge on from the creep is under the reach from where TurnPilot put it: held (c), never let go on the
  2.3 m the driver moved past its stop point in all."""
  car = left_car()
  wait = left_cmd(reason="Waiting for a gap")
  car.run_until(held, left=wait)
  car.run(2., left=wait)
  s_held = car.s
  while car.s < s_held + 1.:
    car.step(left=wait, gas=True, accel=0.4 if car.v < 0.5 else 0.)
  car.run(3., left=wait)
  assert car.policy.held is not None and car.v < 0.05 and car.policy.turn.state.driver_past_m > 0.6
  creep = left_cmd(creep_m=2., reason="Creeping for a view")
  car.run(8., left=creep)
  assert car.policy.held is not None and car.v < 0.05 and abs(car.s - s_held - 2.) < 0.2
  stood = car.s
  while car.s < stood + 1.6:
    car.step(left=creep, gas=True, accel=0.4 if car.v < 0.5 else 0.)
  out, dec, _ = car.run(4., left=creep)
  assert car.policy.held is not None and car.policy.driven is None and held(out, dec) and out.prompt == 'none'


@pytest.mark.parametrize("v_gas", [0.8, 1.2, 1.6])
@pytest.mark.parametrize("nudge", [0.2, 0.35])
def test_a_nudge_from_the_lefts_full_creep_braked_short_of_the_lanes_is_held_on_the_re_engage(nudge, v_gas):
  """Skeptic round 2 (tp/left-hold): the latch's lanes test (8cf3b40) was judged on the driver's frame from the speed then,
  and its let-go was for good. After the left's full creep (5 m from the centre, 1 m short of the oncoming entry) a 0.35 m
  gas nudge at 1.2 m/s let the latch go; the driver braked the car to a stop 0.6 m short of the entry, far under the
  latch's reach, and re-engaged standing: no hold, the car free to pull away into the junction (e93750b held it). Owner
  decision (c), the invariant: judged at the re-engage, from its frame's speed and where the car is; standing short of the
  lanes, held there for the full check, never let go, nothing said for a crossing that never was."""
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
  junction = 250.
  car = left_car(at=junction)
  held_left(car, left_cmd(reason="Waiting for a gap"))
  creep = left_cmd(creep_m=4., reason="Creeping for a view")
  run_left(car, 12., creep)
  assert abs(junction - car.s - 5.) < 0.35 and car.v < 0.05
  s0 = car.s
  while car.s < s0 + nudge:
    car.step(gas=True, accel=2.0 if car.v < v_gas else 0., left=creep)
  while car.v > 0.:
    car.step(accel=-4.0, left=creep)  # the driver's brake, disengaged
  at = junction - car.s
  assert at > ONCOMING_ENTRY_M + 0.5
  rows = [car.step(left=creep) for _ in range(int(5. / DT_MDL))]  # re-engaged standing
  assert car.policy.held is not None and car.policy.driven is None, f"stood {at:.2f} m out, re-engaged: let go"
  assert all(dec.should_stop and out.prompt == 'none' for out, dec, _ in rows) and junction - car.s == at


def test_a_nudge_from_the_lefts_full_creep_into_the_lanes_is_said_as_it_enters_them_short_of_the_reach():
  """The latch's lanes on the driver's own frame: after the left's full creep (5 m from the centre, 1 m short of the
  oncoming entry, the reach 1 m inside them), the driver's gas (disengaged) crawls the car on into the oncoming lanes. A hold
  there could only stand the car in them and it never comes back short of them: theirs as it enters them (drivenOff, the
  crossing said), before the reach and before any re-engage, never 'waiting for a gap' inside them."""
  from openpilot.selfdrive.turnpilot.map_controls import LATCH_PAST_M
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
  junction = 250.
  car = left_car(at=junction)
  held_left(car, left_cmd(reason="Waiting for a gap"))
  creep = left_cmd(creep_m=4., reason="Creeping for a view")
  run_left(car, 12., creep)
  assert abs(junction - car.s - 5.) < 0.35 and car.v < 0.05
  stood, let_go = car.s, None
  while junction - car.s > ONCOMING_ENTRY_M - 0.5:
    out, _, _ = car.step(gas=True, accel=0.4 if car.v < 0.6 else 0., left=creep)
    if let_go is None and car.policy.held is None:
      let_go = junction - car.s
      assert out.prompt == 'unprotectedTurnTakeControl' and car.policy.driven.cause == 'drivenOff'
  assert let_go is not None and ONCOMING_ENTRY_M - 0.1 < let_go < ONCOMING_ENTRY_M, let_go  # a frame or so inside them
  assert junction - let_go - stood < LATCH_PAST_M - 0.5  # well short of the reach


@pytest.mark.parametrize("gas_accel", [0.6, 1.0, 1.5, 2.5])
def test_a_re_engage_the_latch_takes_up_is_never_let_go_on_a_later_frame(gas_accel):
  """The latch's lanes test is the re-engage's alone (skeptic rounds 1, 2): the driver's gas inside the left's 2 m creep
  over a sweep of distances, then a re-engage rolling, the car keeping its speed REENGAGE_LAG_S (the plan's frame, the
  actuator's delay) before the latch brakes it. Let go at the re-engage itself or held: a latch taken up there stands the car
  short of the oncoming lanes, and the frames after it never let it go (judged again on them, the lag taken again with the
  car still at its speed, it let go on the latch's own metres)."""
  from openpilot.selfdrive.turnpilot.map_controls import REENGAGE_LAG_S
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
  for k in range(15):
    gas_to = 0.6 + 0.1 * k
    car = left_car()
    cmd = left_cmd(creep_m=0.)
    car.run_until(held, left=cmd)
    car.run(2., left=cmd)
    s_held = car.s
    cmd = left_cmd(creep_m=2., reason="Creeping for a view")
    while car.s - s_held < 0.5:
      car.step(left=cmd)
    while car.s - s_held < gas_to and car.policy.held is not None:
      car.step(left=cmd, gas=True, accel=gas_accel)
    assert car.policy.held is not None, gas_to
    rows = [car.step(left=cmd, accel=0., engaged=True) for _ in range(round(REENGAGE_LAG_S / DT_MDL))]
    rows += [car.step(left=cmd) for _ in range(int(6. / DT_MDL))]
    released = [i for i, r in enumerate(car.rows[-len(rows):]) if r[5].driven]
    assert not released or released[0] == 0, (gas_to, released[:1])
    if not released:
      assert car.policy.held is not None and car.v < 0.05 and 250. - car.s > ONCOMING_ENTRY_M, (gas_to, 250. - car.s)


def test_the_map_stops_status_chimes_once_a_status_and_again_only_worse_under_always_drive():
  """ALWAYS_DRIVE.md MC-03: under TurnPilotAlwaysDrive 'Map stops and signals unavailable' chimes once a drive for each status
  navd reports (mapControlsUnavailable, ABORT_HOLD_S), and again only when the status gets worse; otherwise it is the same line
  with no sound (mapControlsUnavailableSilent), so its later onsets, after an alert covered it, are silent. Switch off: the
  chiming alert all along, as today."""
  def alerts_for(car, seconds):
    return [car.step()[0].alert for _ in range(round(seconds / DT_MDL))]
  hold = round(ABORT_HOLD_S / DT_MDL)
  car = Car([], status='partial')
  car.policy.always_drive = True
  seen = alerts_for(car, 20.)
  assert seen[:hold] == ['mapControlsUnavailable'] * hold and set(seen[hold + 1:]) == {'mapControlsUnavailableSilent'}
  car.status = 'failed'  # worse: chimes
  seen = alerts_for(car, 4.)
  first = seen.index('mapControlsUnavailable')
  assert first <= 21 and seen[first:first + hold] == ['mapControlsUnavailable'] * hold and seen[-1] == 'mapControlsUnavailableSilent'
  car.status = 'partial'  # better, and said already: silent
  assert 'mapControlsUnavailable' not in alerts_for(car, 4.)
  car.status = 'ok'
  assert set(alerts_for(car, 2.)[21:]) == {'none'}
  car.status = 'partial'  # back, said already, no worse than the last: silent
  assert 'mapControlsUnavailable' not in alerts_for(car, 4.)
  car.status = 'failed'  # worse than the last again: chimes
  assert 'mapControlsUnavailable' in alerts_for(car, 4.)
  off = Car([], status='partial')
  assert set(alerts_for(off, 20.)) == {'mapControlsUnavailable'}
