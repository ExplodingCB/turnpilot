"""turnpilotd after dark (2026-11-06): the road camera's exposure (narrowRoadCameraState) through darkness.DarkFilter says
when it is dark. Since 2026-11-07 (no handbacks) a night left is judged on the band the camera covers and goes when it is clear,
with no night cap on its range; the night is logged once per left (turnpilot.left_night). Closed loop
on test_turnpilotd_left's synthetic route, an empty oncoming band, the car following the plan. The light count runs only with
the car moving (a lit stand never makes the night light; a stand holds it, so a stop-and-go day turns light), every
TurnPilot starts dark off the sim, and the exposure is logged
from the drive's first reading on (its state, every change, a summary a minute, each left judged after dark)."""
import os
from unittest import mock

import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.darkness import ENTER_S, EXPOSURE_LOG_S, LEAVE_MOVING_V, LEAVE_S, DarkFilter, dark_filter_for
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M
from openpilot.selfdrive.turnpilot.tests.synthetic import camera_state, junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import STRAIGHT_M, TEST_RANGE_M, V_25, road_runs

Desire = log.Desire
NIGHT_EV, DAY_EV = 7.0, 0.1  # % exposureValPercent: 1e's night median, a daylight route's (0f, 10: 0.01-0.13)
LIT_EV = 0.3  # % under LEAVE_EV: a lit forecourt canopy or a brightly lit junction stand at night (review 2026-11-06)


def events(event, name):
  return [c.kwargs for c in event.call_args_list if c.args[0] == name]


class NightSim(Sim):
  """test_turnpilotd's Sim with camerad's narrowRoadCameraState on every frame: ev(sim) its exposure, None for no message."""
  def __init__(self, maneuvers, ev=None, **kw):
    super().__init__(maneuvers, **kw)
    self.ev = ev
    plan = self.tp.plan

    def with_camera(sm, *args, **kwargs):  # after Sim.step's own reset of sm.updated, as camerad's message would come
      value = self.ev(self) if self.ev is not None else None
      if value is not None:
        sm.put('narrowRoadCameraState', camera_state(value))
      return plan(sm, *args, **kwargs)
    self.tp.plan = with_camera


def left_at(junction_s, ev=None, dark_filter=None):
  """A 25 mph one-lane left at junction_s (in TEST_RANGE_M by day), TurnPilotAutoLeft auto, an empty band, the driver answering
  the signal prompt; dark_filter: the TurnPilot's (None: the harness's light one)."""
  sim = NightSim([(junction_s, "turn", "left", (0., 270., 2, 'present', V_25, 1, STRAIGHT_M)),
                  (junction_s + 1500., "turn", "right", junction_of("right"))], ev=ev)
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
  sim.answers_prompt = True
  if dark_filter is not None:
    sim.tp.dark_filter = dark_filter
  return sim


def drive(sim, junction_s, until):
  """Engaged, turning at an 8 m radius through the junction once the turn is under way, until until(sim, plan) or 150 s."""
  plans = []
  while not plans or not until(sim, plans[-1]):
    turning = sim.tp.desires.started and abs(junction_s - sim.s) < 12. and sim.v > 0.3
    plans.append(sim.step(engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
    assert sim.t < 150., (sim.s, sim.t)
  return plans


def held_for(seconds):
  held = [0.]

  def until(sim, plan):
    held[0] = held[0] + DT_MDL if plan.shouldStop and sim.v < 0.05 else 0.
    return held[0] >= seconds
  return until


def through(junction_s):
  return lambda sim, plan: sim.s > junction_s + 30.


def assert_night_go(sim, plans, line):
  """Kellen, 2026-11-07 (no handbacks): the night left is judged on the band the camera covers, its day range (no night cap),
  and goes from its line on the empty band: never undecided, never a take-control prompt, never skipped or rerouted."""
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  s_go = sim.trace[len(sim.trace) - len(plans) + go][0]  # the trace holds any frames stepped before drive()
  assert str(plans[go].unprotectedLeft.state) == 'go' and s_go > line - 30., s_go
  assert all(p.unprotectedLeft.rangeM == TEST_RANGE_M for p in plans if str(p.unprotectedLeft.state) != 'off')
  assert not any(str(p.unprotectedLeft.state) == 'undecided' or str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert not any(p.tooFastToTurn or str(p.unprotectedLeft.state) == 'skipped' for p in plans), "never skipped or rerouted"
  assert sim.s > line + 30.


def test_at_night_exposure_a_left_is_driven_to_its_line_and_goes_on_the_clear_band():
  # 1e's exposure all drive (p10 5.0, p50 7.1 %): dark ENTER_S after the first frame, logged once with the reading and the
  # sensor; the left rolls up to its line and goes on the band the camera covers, empty (2026-11-07: no night handback)
  sim = left_at(300., ev=lambda sim: NIGHT_EV)
  line = 300. - HOLD_LINE_M - 1.
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 300., through(300.))
  assert sim.tp.dark
  # the drive's first reading, the state it starts in (the harness's light), then the change
  assert events(event, 'turnpilot.dark') == [{'initial': True, 'dark': False, 'ev': NIGHT_EV, 'sensor': 'os04c10'},
                                             {'dark': True, 'ev': NIGHT_EV, 'sensor': 'os04c10'}]
  # the left judged after dark says what made it dark, once, on its day range (no night cap)
  night, = events(event, 'turnpilot.left_night')
  assert night['dark'] and night['ev'] == NIGHT_EV and night['sensor'] == 'os04c10' and night['range_m'] == TEST_RANGE_M
  assert night['maneuver_id'] is not None and night['r_req'] > 0.
  assert_night_go(sim, plans, line)


def test_the_car_starts_dark_and_with_no_exposure_ever_read_the_left_still_goes():
  # main()'s filter (dark_filter_for, no SIMULATION) is dark from the start, and with no camera message at all it stays so:
  # the left still goes on the clear band from its line (2026-11-07: no handbacks), and the night is logged
  sim = left_at(300., dark_filter=dark_filter_for({}))
  assert sim.tp.dark
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 300., through(300.))
  assert_night_go(sim, plans, 300. - HOLD_LINE_M - 1.)
  # nothing read: no exposure line at all, and the night's log says so (ev None, no sensor)
  assert not events(event, 'turnpilot.dark') and not events(event, 'turnpilot.exposure')
  night, = events(event, 'turnpilot.left_night')
  assert night['dark'] and night['ev'] is None and night['sensor'] == ''


def test_after_30_s_of_daylight_exposure_the_left_goes_as_before():
  # started dark (the car's), daylight exposure for LEAVE_S on the long approach: light again, said once, and the left judged
  # on its day range goes from its line on the empty band, no handback; its rangeM the day's
  sim = left_at(600., ev=lambda sim: DAY_EV, dark_filter=dark_filter_for({}))
  dark = []

  def until(sim, plan):
    dark.append(sim.tp.dark)
    return sim.s > 630.
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 600., until)
  assert events(event, 'turnpilot.dark') == [{'initial': True, 'dark': True, 'ev': DAY_EV, 'sensor': 'os04c10'},
                                             {'dark': False, 'ev': DAY_EV, 'sensor': 'os04c10'}]
  assert not events(event, 'turnpilot.left_night')
  changed = dark.index(False)
  assert abs(changed * DT_MDL - LEAVE_S) <= 2 * DT_MDL and sim.trace[changed][0] < 600. - 100., "light well before the line"
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert str(plans[go].unprotectedLeft.state) == 'go' and plans[go].unprotectedLeft.rangeM == TEST_RANGE_M
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or p.tooFastToTurn for p in plans) and sim.s > 630.


def test_dusk_falling_on_the_approach_still_goes_from_the_line():
  # light at the start (the harness's), night exposure from 10 s on: dark ENTER_S later, still on the approach; at the line the
  # left goes as at night, on the clear band (2026-11-07: no handbacks)
  sim = left_at(300., ev=lambda sim: DAY_EV if sim.t < 10. else NIGHT_EV)
  assert not sim.tp.dark
  plans = drive(sim, 300., through(300.))
  assert sim.tp.dark
  assert_night_go(sim, plans, 300. - HOLD_LINE_M - 1.)


def test_light_the_same_left_goes_as_before():
  sim = left_at(300., ev=lambda sim: DAY_EV)
  plans = drive(sim, 300., lambda sim, plan: sim.s > 330.)
  assert any(p.desire == Desire.turnLeft for p in plans) and not sim.tp.dark
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def test_the_filter_runs_while_turnpilot_is_off():
  # the road's state, ready for the next left: plan_inner reads the exposure on disabled plans too (TurnPilotEnabled off)
  sim = left_at(300., ev=lambda sim: NIGHT_EV)
  for _ in range(round((ENTER_S + 0.5) / DT_MDL)):
    sim.sm.updated = dict.fromkeys(sim.sm.data, False)
    sim.sm.put('narrowRoadCameraState', camera_state(NIGHT_EV))
    turnpilotd.TurnPilot.plan(sim.tp, sim.sm, None, False, False, long_control=True)
  assert sim.tp.dark


def test_a_failing_exposure_read_keeps_the_state_and_the_plan():
  sim = left_at(300., ev=lambda sim: NIGHT_EV, dark_filter=DarkFilter(dark=True))
  with mock.patch.object(DarkFilter, 'update', side_effect=RuntimeError("synthetic")), \
       mock.patch.object(turnpilotd, 'log_error') as log_error:
    plans = [sim.step(engaged=True) for _ in range(5)]
  assert sim.tp.dark and all(str(p.state) != 'disabled' for p in plans)
  assert "turnpilotd.watch_dark_failed" in [c.args[0] for c in log_error.call_args_list]


def test_a_lit_stand_at_night_never_makes_the_night_light():
  # review 2026-11-06: night exposure, then the car standing 60 s under a lit forecourt canopy (0.3 %, under LEAVE_EV): the
  # light count runs only moving (LEAVE_MOVING_V), so it is still night, and the left 200 m on (the stand's light the whole way
  # there, under LEAVE_S of it moving) is judged after dark (logged as a night left) and goes on the clear band. Counted
  # standing, the stand made it light at 30 s
  sim = left_at(200., ev=lambda sim: NIGHT_EV if sim.t < 10. else LIT_EV)
  sim.cruise = sim.v = 0.
  stand = []
  while sim.t < 70.:
    sim.v = 0.
    sim.step()
    stand.append((sim.tp.dark, sim.tp.dark_filter.leave_t))
  assert stand[-1][0] and all(dark for dark, _ in stand[int(ENTER_S / DT_MDL) + 2:]), "dark from the night readings on"
  assert all(leave_t == 0. for _, leave_t in stand), "the light count held at 0 standing"
  sim.cruise = 13.
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 200., through(200.))
  assert sim.tp.dark and events(event, 'turnpilot.left_night'), "still night at the left"
  assert_night_go(sim, plans, 200. - HOLD_LINE_M - 1.)
  moving = [s for s, v, _ in sim.trace if v > LEAVE_MOVING_V]
  assert moving and len(moving) * DT_MDL < LEAVE_S, "under LEAVE_S of it moving on the way: still the stand's verdict"


def test_a_stop_and_go_day_drive_turns_light_and_its_left_goes_on_the_day_range():
  # second review, 2026-11-06: started dark (the car's), a day drive stopping every 25 s (a stop sign or a light every 200-250
  # m), standing 10 s each time. The light count started over at every stand, so it never reached LEAVE_S over 5 m/s and every
  # left of the drive was judged as a night left. Held through a stand, two legs make it light, and the left 1 km on goes from its
  # line on the day range, no handback
  sim = left_at(1000., ev=lambda sim: DAY_EV, dark_filter=dark_filter_for({}))
  legs = []
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    for _ in range(2):
      t0, moving_s = sim.t, 0.
      while sim.t - t0 < 25.:
        sim.step(engaged=True)
        moving_s += DT_MDL if sim.v > LEAVE_MOVING_V else 0.
      legs.append(moving_s)
      sim.cruise = sim.v = 0.
      for _ in range(round(10. / DT_MDL)):
        sim.v = 0.
        sim.step(engaged=True)
      sim.cruise = 13.
    assert all(m < LEAVE_S for m in legs) and sum(legs) >= LEAVE_S, legs  # no one leg is enough, the two are
    assert not sim.tp.dark and sim.s < 1000. - 300., "light from the second leg, well short of the left"
    plans = drive(sim, 1000., lambda sim, plan: sim.s > 1030.)
  assert [e['dark'] for e in events(event, 'turnpilot.dark')] == [True, False]
  assert not events(event, 'turnpilot.left_night')
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert str(plans[go].unprotectedLeft.state) == 'go' and plans[go].unprotectedLeft.rangeM == TEST_RANGE_M
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or p.tooFastToTurn for p in plans) and sim.s > 1030.


def test_a_drive_dark_from_the_start_logs_its_state_at_once_and_its_exposure_every_minute():
  # review 2026-11-06: a drive dark all the way logged nothing (turnpilot.dark only on a change), nor did a day drive stuck dark
  # in the dead band: now the first reading says the state it starts in, and every EXPOSURE_LOG_S a summary of the readings
  # (min, median, max, how many, the seconds of them moving over LEAVE_MOVING_V) with the state and the sensor
  readings = (5.0, 7.1, 9.7, 6.2)
  sim = NightSim([(5000., "turn", "right", junction_of("right"))], ev=lambda sim: readings[len(sim.trace) % 4])
  sim.tp.dark_filter = dark_filter_for({})
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    for _ in range(round((2. * EXPOSURE_LOG_S + 5.) / DT_MDL)):
      sim.step(engaged=True)
  assert events(event, 'turnpilot.dark') == [{'initial': True, 'dark': True, 'ev': 5.0, 'sensor': 'os04c10'}]
  summaries = events(event, 'turnpilot.exposure')
  n = round(EXPOSURE_LOG_S / DT_MDL)
  assert len(summaries) == 2
  for x in summaries:
    assert x == {'dark': True, 'n': n, 'sensor': 'os04c10', 'ev_min': 5.0, 'ev_p50': 7.1, 'ev_max': 9.7,
                 'moving_s': pytest.approx(EXPOSURE_LOG_S, abs=0.1)}
  # a drive the dead band holds dark (0.6-0.9 %: an overcast day, say) is in the logs too, its readings with it
  sim = NightSim([(5000., "turn", "right", junction_of("right"))], ev=lambda sim: 0.6 if len(sim.trace) % 2 else 0.9)
  sim.tp.dark_filter = dark_filter_for({})
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    for _ in range(round((EXPOSURE_LOG_S + 1.) / DT_MDL)):
      sim.step(engaged=True)
  x, = events(event, 'turnpilot.exposure')
  assert x['dark'] and (x['ev_min'], x['ev_max']) == (0.6, 0.9) and not events(event, 'turnpilot.dark')[1:]


def test_every_turnpilot_starts_dark_off_the_sim():
  # review 2026-11-06: TurnPilot() started light and only main() made it dark, so any other entry point (a replay harness, a
  # refactored main) would judge a night left on the day range. Now it is TurnPilot's own: dark unless SIMULATION=1
  with mock.patch.dict(os.environ):
    os.environ.pop('SIMULATION', None)
    assert turnpilotd.TurnPilot().dark
    os.environ['SIMULATION'] = '0'
    assert turnpilotd.TurnPilot().dark
    os.environ['SIMULATION'] = '1'
    assert not turnpilotd.TurnPilot().dark
  assert not Sim([(300., "turn", "right")]).tp.dark, "test_turnpilotd's Sim is the daylight harness"


def test_main_starts_the_car_dark():
  # main() builds its TurnPilot, whose filter is from os.environ (dark_filter_for): dark on the car (no SIMULATION), and with no
  # exposure read (LoopSM's narrowRoadCameraState never updated) it stays dark
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_main import run_main
  built, real = [], turnpilotd.TurnPilot

  def make():
    built.append(real())
    return built[-1]
  with mock.patch.dict(os.environ), mock.patch.object(turnpilotd, 'TurnPilot', side_effect=make), \
       mock.patch.object(turnpilotd, 'dark_filter_for', wraps=turnpilotd.dark_filter_for) as made:
    os.environ.pop('SIMULATION', None)
    run_main(frames=3)
    made.assert_called_once_with(os.environ)
  assert len(built) == 1 and built[0].dark
