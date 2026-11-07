"""turnpilotd at dusk (review LN-3, 2026-11-07): the sun under 0 deg is dark whatever the road camera's exposure reads
(darkness.py), so a left at dusk is judged as at night (logged as a night left). Since 2026-11-07 (no handbacks) a night
left goes on the band the camera covers, with no night cap on its range. The exposure alone kept a drive light by day light through the dead band (0.5-1.0 %)
until 1.0 % held 5 s, and nothing logged places that against sunset; the stop-sign go the same minute already held its vision
go to night (junction_watch.lighting_at: dusk). The sun is stop_rules.sun_position at the car's last fix (LastGPSPosition
through main()'s Params) on decider_clock's wall clock, worked out every SUN_EVERY_S; with no fix or no clock the exposure
decides alone, as before. Each change of what makes it dark is logged once (turnpilot.dark_why). Closed loop on
test_turnpilotd_night's left (test_turnpilotd_left's synthetic route, an empty oncoming band, the car following the plan). A
synthetic place only: the equator at the prime meridian, the evening of the 2026 March equinox (no real one)."""
from unittest import mock

import pytest

from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.darkness import ENTER_S, SUN_EVERY_S, DarkFilter, dark_filter_for, sun_elevation
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M
from openpilot.selfdrive.turnpilot.tests.test_darkness import sun_time
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import TEST_RANGE_M
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_night import DAY_EV, NIGHT_EV, assert_night_go, drive, events, left_at, through

Desire = log.Desire
DEAD_BAND_EV = 0.75  # % between LEAVE_EV and ENTER_EV: neither count runs, the state kept (darkness.py)
FIX = (0., 0.)       # synthetic: no real place


def at_sun(sim, el_deg, rate=1.):
  """The car's fix at FIX and the wall clock where the sun there falls through el_deg, running rate x the sim's time."""
  t0 = sun_time(el_deg)
  sim.tp.sun_fix = lambda: FIX
  sim.tp.wall_clock = lambda: t0 + rate * sim.t
  return sim


def went_on(plans, range_m):
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert str(plans[go].unprotectedLeft.state) == 'go' and plans[go].unprotectedLeft.rangeM == range_m
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or p.tooFastToTurn for p in plans)


@pytest.mark.parametrize('ev', [DAY_EV, DEAD_BAND_EV])
def test_a_daylight_exposure_drive_with_the_sun_at_minus_3_deg_is_dark_and_the_left_goes_at_its_line(ev):
  # LN-3: the filter light by day (the harness's), the exposure bright or in the dead band, the sun 3 deg down: dark from the
  # first frame (the sun worked out there); the left is judged as a night left and goes from its line on the clear band
  # (2026-11-07: no handbacks, no night cap)
  sim = at_sun(left_at(300., ev=lambda sim: ev), -3.)
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 300., through(300.))
  assert sim.tp.dark and not sim.tp.dark_filter.dark, "dark by the sun alone: the exposure's own verdict stays light"
  assert_night_go(sim, plans, 300. - HOLD_LINE_M - 1.)
  # said once, with the sun's elevation and never a position; the exposure's own log says light throughout
  why, = events(event, 'turnpilot.dark_why')
  assert why['dark'] and why['why'] == 'sun' and -3.2 < why['sun_el'] < -2.8 and set(why) == {'dark', 'why', 'sun_el', 'ev'}
  assert [e['dark'] for e in events(event, 'turnpilot.dark')] == [False]
  night, = events(event, 'turnpilot.left_night')
  assert night['dark'] and night['range_m'] == TEST_RANGE_M


def test_the_sun_10_deg_up_and_a_bright_exposure_the_left_goes_on_the_day_range():
  sim = at_sun(left_at(300., ev=lambda sim: DAY_EV), 10.)
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 300., lambda sim, plan: sim.s > 330.)
  assert not sim.tp.dark and 9.5 < sim.tp.dark_filter.sun_el <= 10.
  went_on(plans, TEST_RANGE_M)
  why, = events(event, 'turnpilot.dark_why')
  assert not why['dark'] and why['why'] == 'light' and why['sun_el'] == pytest.approx(10., abs=0.2)
  # the car's own start (dark, fail closed) is let go only by the exposure, the sun up or not
  sim = at_sun(left_at(600., ev=lambda sim: DAY_EV, dark_filter=dark_filter_for({})), 10.)
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 600., lambda sim, plan: sim.s > 630.)
  went_on(plans, TEST_RANGE_M)
  assert [(e['why'], e['sun_el'] is not None) for e in events(event, 'turnpilot.dark_why')] == [('exposure', True), ('light', True)]


@pytest.mark.parametrize('known', ['no fix', 'no clock'])
def test_no_sun_known_is_the_exposure_alone(known):
  # no fix saved or no valid clock: the exposure decides as before (light by day here: the left goes on the day range)
  sim = at_sun(left_at(300., ev=lambda sim: DAY_EV), -3.)
  if known == 'no fix':
    sim.tp.sun_fix = lambda: None
  else:
    sim.tp.wall_clock = lambda: None
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 300., lambda sim, plan: sim.s > 330.)
  assert sim.tp.dark_filter.sun_el is None and not sim.tp.dark
  went_on(plans, TEST_RANGE_M)
  assert events(event, 'turnpilot.dark_why') == [{'dark': False, 'why': 'light', 'sun_el': None, 'ev': DAY_EV}]
  # ...and at night the exposure alone is dark, as before
  sim = left_at(300., ev=lambda sim: NIGHT_EV)
  sim.tp.sun_fix = lambda: None
  plans = drive(sim, 300., through(300.))
  assert sim.tp.dark
  assert_night_go(sim, plans, 300. - HOLD_LINE_M - 1.)


def test_sunset_on_the_approach_is_said_once_and_the_left_still_goes():
  # the sun 0.5 deg up at the start, its clock run 20x: under the horizon about 6 s in, on the approach; logged light (the sun
  # known), then dark by the sun, once each, never per frame; the left 600 m on goes from its line (2026-11-07: no handbacks)
  sim = at_sun(left_at(600., ev=lambda sim: DAY_EV), 0.5, rate=20.)
  dark = []

  def until(sim, plan):
    dark.append(sim.tp.dark)
    return sim.s > 630.
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans = drive(sim, 600., until)
  assert_night_go(sim, plans, 600. - HOLD_LINE_M - 1.)
  whys = events(event, 'turnpilot.dark_why')
  assert [(e['dark'], e['why']) for e in whys] == [(False, 'light'), (True, 'sun')]
  assert whys[0]['sun_el'] > 0. > whys[1]['sun_el'] > -0.5
  went = dark.index(True)
  assert went * DT_MDL < 30. and sim.trace[went][0] < 600. - 200., "dark well before the line"


def test_the_sun_is_read_every_10_s_from_last_gps_position():
  # the real wiring: sun_fix is LastGPSPosition through main()'s Params (ubloxd's saved gpsLocationExternal), read every
  # SUN_EVERY_S, never per frame; the sun 3 deg down there makes the day's light dark
  class FakeParams(dict):
    reads = 0

    def get(self, key):
      FakeParams.reads += key == 'LastGPSPosition'
      return super().get(key)
  saved = messaging.new_message('gpsLocationExternal', valid=True)
  saved.gpsLocationExternal.latitude, saved.gpsLocationExternal.longitude, saved.gpsLocationExternal.hasFix = *FIX, True
  sim = left_at(1000., ev=lambda sim: DAY_EV)
  t0 = sun_time(-3.)
  sim.tp.wall_clock = lambda: t0 + sim.t
  for _ in range(round(25. / DT_MDL)):
    sim.step(engaged=True)
  assert not sim.tp.dark and sim.tp.dark_filter.sun_el is None, "no Params (a harness): no sun, the exposure's light"
  sim.tp.params = FakeParams(LastGPSPosition=saved.to_bytes())
  for _ in range(round(30. / DT_MDL)):
    sim.step(engaged=True)
  assert sim.tp.dark and sim.tp.dark_filter.why == 'sun'
  assert sim.tp.dark_filter.sun_el == pytest.approx(sun_elevation(FIX, t0 + sim.t), abs=0.05)
  assert 3 <= FakeParams.reads <= 30. / SUN_EVERY_S + 1, FakeParams.reads


def test_a_failing_sun_is_logged_and_the_exposure_runs_on():
  # a fix or clock read that raises is no sun (rate-limited log_error), and the exposure's filter goes on: night readings
  # still make it dark ENTER_S later
  sim = left_at(300., ev=lambda sim: NIGHT_EV)

  def broken():
    raise RuntimeError("synthetic")
  sim.tp.sun_fix = broken
  with mock.patch.object(turnpilotd, 'log_error') as log_error:
    for _ in range(round((ENTER_S + 0.5) / DT_MDL)):
      sim.step(engaged=True)
  assert sim.tp.dark and sim.tp.dark_filter.sun_el is None and sim.tp.dark_filter.why == 'exposure'
  assert "turnpilotd.watch_sun_failed" in [c.args[0] for c in log_error.call_args_list]


def test_the_sim_is_never_dark_by_the_wall_clock_sun():
  # BeamNG renders its own daylight: the sim's filter (dark_filter_for, SIMULATION=1) has the sun off, so the PC's evening
  # never hands its day scenario's left back
  sim = at_sun(left_at(300., ev=lambda sim: DAY_EV, dark_filter=dark_filter_for({'SIMULATION': '1'})), -3.)
  plans = drive(sim, 300., lambda sim, plan: sim.s > 330.)
  assert not sim.tp.dark and sim.tp.dark_filter.sun_el is None
  went_on(plans, TEST_RANGE_M)


def test_the_harness_filter_counts_the_sun():
  # test_turnpilotd's Sim's light filter (DarkFilter(dark=False)) counts the sun like the car's: only the exposure starts light
  assert DarkFilter(dark=False).sun_on


def test_nothing_read_and_no_sun_says_nothing_then_the_sun_is_said_once():
  # the car's dark start with no camera message and no sun is no reason at all: no turnpilot.dark_why; the sun once known
  # (the fix saved, the clock valid) says it once, dark by the sun 3 deg down, the exposure never read
  sim = left_at(3000., dark_filter=dark_filter_for({}))
  sim.tp.sun_fix = lambda: None
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    for _ in range(round(15. / DT_MDL)):
      sim.step(engaged=True)
    assert not events(event, 'turnpilot.dark_why') and sim.tp.dark
    at_sun(sim, -3.)
    for _ in range(round((SUN_EVERY_S + 1.) / DT_MDL)):
      sim.step(engaged=True)
  why, = events(event, 'turnpilot.dark_why')
  assert why['dark'] and why['why'] == 'exposure+sun' and why['ev'] is None and why['sun_el'] < 0.
