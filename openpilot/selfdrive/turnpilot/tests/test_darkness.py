"""darkness.DarkFilter (2026-11-06): the road camera's exposure, with hysteresis, for the unprotected left's night range cap.
Thresholds from this comma four's exposureValPercent on the downloaded routes: day 0.01-0.13 %, a garage 2.2-3.3 %, dusk and
night 3-10 %; camerad's auto-exposure starts each drive at 0.015 % and climbs for about 2 s. The sun under 0 deg is dark too,
whatever the exposure reads (review LN-3, 2026-11-07)."""
import datetime
import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.darkness import (ENTER_EV, ENTER_S, EXPOSURE_LOG_S, LEAVE_EV, LEAVE_MOVING_V, LEAVE_S, SUN_DARK_EL,
                                                     SUN_EVERY_S, DarkFilter, car_fix, dark_filter_for, sun_elevation)


def feed(f, ev, seconds, moving=True):
  out = []
  for _ in range(round(seconds / DT_MDL)):
    out.append(f.update(ev, DT_MDL, moving))
  return out


def test_the_thresholds_are_the_measured_ones():
  assert (ENTER_EV, ENTER_S, LEAVE_EV, LEAVE_S) == (1.0, 5., 0.5, 30.)
  assert (LEAVE_MOVING_V, EXPOSURE_LOG_S) == (5., 60.)
  assert DarkFilter().dark, "fail closed: dark until daylight readings say otherwise"


def test_the_light_count_runs_only_while_the_car_moves():
  # review 2026-11-06: 30 s standing under a lit forecourt canopy or at a bright junction (at or under LEAVE_EV) made a night
  # drive light, its next left judged on the day range. Standing, the light count is held where it is; it runs only moving
  f = DarkFilter()
  assert all(feed(f, 0.3, 120., moving=False)), "two minutes of a lit stand never make it light"
  assert f.leave_t == 0.
  # a stand in the middle of a daylight count holds it, never starts it over (second review, 2026-11-06: zeroed, a day drive
  # stopping every 25 s never turned light): 29 s moving, a 10 s stand, then 1.2 s more moving is light
  f = DarkFilter()
  assert all(feed(f, 0.1, LEAVE_S - 1.)) and all(feed(f, 0.1, 10., moving=False))
  assert abs(f.leave_t - (LEAVE_S - 1.)) < 1e-6, "the stand held the count"
  assert not feed(f, 0.1, 1.2)[-1], "30 s of daylight driving is light, a stand or not between"
  # by day, once light, standing never makes it dark again (0.01-0.13 % reads no dark count) ...
  assert not any(feed(f, 0.1, 120., moving=False))
  # ... and the dark count runs standing (a garage, a tunnel queue): fail closed whatever the car does
  f = DarkFilter(dark=False)
  assert feed(f, 2.77, ENTER_S + 0.1, moving=False)[-1]


def test_a_stop_and_go_day_turns_light_and_a_lit_stand_at_night_never_does():
  # second review, 2026-11-06: a residential day drive, a stop sign or a light every 100-250 m at 25 mph, is legs of 10-25 s
  # moving and stands between. With the count zeroed at each stand, 35 minutes of 25 s legs and 10 s stands never turned
  # light; held, it is light 5 s into the second leg (40 s on, 30 s of it driven), never during a stand
  f, t, light_at = DarkFilter(), 0., None
  for _ in range(60):
    for moving, seconds in ((True, 25.), (False, 10.)):
      for dark in feed(f, 0.1, seconds, moving):
        t += DT_MDL
        if not dark and light_at is None:
          light_at, light_moving = t, moving
  assert light_at is not None and abs(light_at - 40.) < 0.1 and light_moving, light_at
  # night legs (1e: p50 7.1 %) with long lit stands between (0.3 %, a forecourt): the night's moving readings zero the
  # count, the stands never add to it, so it is never light
  f = DarkFilter()
  for ev, seconds, moving in ((7.0, 10., True), (0.3, 120., False), (6.0, 60., True), (0.3, 120., False), (0.75, 5., True),
                              (0.3, 120., False)):
    assert all(feed(f, ev, seconds, moving)), (ev, seconds, moving)
  # a lit stretch driven under 30 s, then a lit stand, then the dark again: the stand adds nothing and the dark zeroes it
  f = DarkFilter()
  assert all(feed(f, 0.3, 20.)) and all(feed(f, 0.3, 120., moving=False)) and abs(f.leave_t - 20.) < 1e-6
  assert f.update(7., DT_MDL) and f.leave_t == 0.


def test_it_goes_dark_after_5_s_at_or_above_1_pct_never_after_4():
  f = DarkFilter(dark=False)
  assert not any(feed(f, ENTER_EV, 4.)), "4 s is not enough"
  f = DarkFilter(dark=False)
  states = feed(f, ENTER_EV, ENTER_S + 0.1)
  assert not any(states[:round(ENTER_S / DT_MDL) - 1]) and states[-1]
  assert abs(states.index(True) * DT_MDL - (ENTER_S - DT_MDL)) < 1e-6  # the frame its 5 s complete
  # night readings (1e: p10 5.0, p50 7.1, p90 9.7 %) and a garage's (2.8 %)
  for ev in (5.0, 7.1, 9.7, 2.77):
    assert feed(DarkFilter(dark=False), ev, ENTER_S + 0.1)[-1], ev


def test_it_is_light_again_only_after_30_s_at_or_under_half_a_pct():
  f = DarkFilter()
  assert all(feed(f, LEAVE_EV, LEAVE_S - 1.)), "29 s of daylight is not enough"
  f = DarkFilter()
  states = feed(f, 0.1, LEAVE_S + 0.1)
  assert all(states[:round(LEAVE_S / DT_MDL) - 1]) and not states[-1]
  # daylight (routes 0f, 10: 0.01-0.13 %) stays light
  assert not any(feed(f, 0.13, 120.))


def test_one_reading_the_other_way_starts_a_count_over():
  f = DarkFilter()
  feed(f, 0.1, LEAVE_S - 1.)
  assert f.update(2., DT_MDL)  # one dark frame: the 29 s of daylight start over
  assert all(feed(f, 0.1, LEAVE_S - 1.)) and not feed(f, 0.1, 1.2)[-1]
  f = DarkFilter(dark=False)
  feed(f, 7., ENTER_S - 1.)
  assert not f.update(0.1, DT_MDL)
  assert not any(feed(f, 7., ENTER_S - 1.)) and feed(f, 7., 1.2)[-1]


def test_the_dead_band_counts_neither_way_and_keeps_the_state():
  # 0.5-1.0 %: 1e's longest spell under 1 % after the start, 7.9 s, was there. Neither dark nor light: both counts start over
  for start in (True, False):
    f = DarkFilter(dark=start)
    assert all(s == start for s in feed(f, 0.75, 120.)), start
    assert f.enter_t == 0. and f.leave_t == 0.
  f = DarkFilter()
  feed(f, 0.1, LEAVE_S - 1.)
  feed(f, 0.75, 1.)
  assert all(feed(f, 0.1, LEAVE_S - 1.)), "the dead band broke the daylight count"


@pytest.mark.parametrize('missing', [None, math.nan, math.inf])
def test_a_missing_reading_keeps_the_state_and_both_counts(missing):
  for start in (True, False):
    f = DarkFilter(dark=start)
    assert all(s == start for s in feed(f, missing, 60.))
  f = DarkFilter()
  feed(f, 0.1, LEAVE_S - 1.)
  feed(f, missing, 5.)
  assert not feed(f, 0.1, 1.2)[-1], "the daylight count went on past the gap"


def test_the_auto_exposure_ramp_at_the_start_of_a_night_drive_never_reads_light():
  # camerad starts at 0.015 % and climbs to the scene's exposure in about 1.7-2.2 s (1c, 1d, 1e alike): started dark (the car's
  # start), it stays dark through it
  f = dark_filter_for({})
  ramp = [0.015 + (7. - 0.015) * min(k * DT_MDL / 2.2, 1.) for k in range(round(10. / DT_MDL))]
  assert all(f.update(ev, DT_MDL) for ev in ramp)


def test_the_car_starts_dark_and_the_sim_light():
  assert dark_filter_for({}).dark and dark_filter_for({'SIMULATION': '0'}).dark
  assert not dark_filter_for({'SIMULATION': '1'}).dark  # BeamNG's bridge publishes no exposure: its day keeps its go
  assert not any(feed(dark_filter_for({'SIMULATION': '1'}), 0., 60.))


# the sun (review LN-3, 2026-11-07): dark is the exposure's verdict or the sun under SUN_DARK_EL. A synthetic place only (the
# equator at the prime meridian, no real one), the evening of the 2026 March equinox
def sun_time(el_deg: float) -> float:
  """Unix time the sun at (0, 0) falls through el_deg on the evening of 2026-03-20 (10 s steps)."""
  from openpilot.selfdrive.turnpilot.stop_rules import sun_position
  t = datetime.datetime(2026, 3, 20, 15, 0, tzinfo=datetime.UTC).timestamp()
  while sun_position(0., 0., t)[0] > el_deg:
    t += 10.
  return t


def test_the_sun_thresholds():
  assert (SUN_DARK_EL, SUN_EVERY_S) == (0., 10.)
  f = DarkFilter()
  assert f.sun_on and f.sun_el is None and dark_filter_for({}).sun_on, "the car's sun counts"


def test_a_daylight_exposure_with_the_sun_at_minus_3_deg_is_dark():
  # LN-3: a drive light by day (30 s of 0.1 % moving), the sun 3 deg under the horizon: dark, whatever the exposure reads,
  # bright or in the dead band (0.5-1.0 %, where the exposure alone kept the day's light until 1.0 % held 5 s)
  el = sun_elevation((0., 0.), sun_time(-3.))
  assert -3.1 < el <= -3.
  for ev in (0.1, 0.75):
    f = dark_filter_for({})
    assert not feed(f, 0.1, LEAVE_S + 0.1)[-1], "light by the exposure"
    f.set_sun(el)
    assert f.is_dark and f.sun_dark and f.why == 'sun'
    assert all(feed(f, ev, 600.)), ev
    assert not f.dark, "the exposure's own verdict stays light: the sun alone makes it dark"
  # and the exposure going dark under it is both
  assert feed(f, 7., ENTER_S + 0.1)[-1] and f.why == 'exposure+sun'
  # just under the horizon is dark, just over is the exposure's
  f = DarkFilter(dark=False)
  f.set_sun(-0.01)
  assert f.is_dark
  f.set_sun(0.)
  assert not f.is_dark and f.why == 'light'


def test_the_sun_10_deg_up_and_a_bright_exposure_is_light():
  el = sun_elevation((0., 0.), sun_time(10.))
  assert 9.9 < el <= 10.
  f = dark_filter_for({})  # the car's start: dark (fail closed)
  f.set_sun(el)
  assert f.is_dark and f.why == 'exposure', "the sun up never lets the car's dark start go: only the exposure does"
  states = feed(f, 0.1, LEAVE_S + 0.1)
  assert all(states[:round(LEAVE_S / DT_MDL) - 1]) and not states[-1] and f.why == 'light'
  # ...and the sun up never makes a night light: night exposure in daylight's sun is dark (a garage, a tunnel)
  assert feed(f, 2.77, ENTER_S + 0.1)[-1] and f.why == 'exposure'


@pytest.mark.parametrize('sun', [None, math.nan, math.inf])
def test_no_sun_known_is_the_exposure_alone(sun):
  # no fix saved, no valid clock, no Params (a harness, a replay): the exposure's verdict as before, fail closed
  f = dark_filter_for({})
  f.set_sun(sun)
  assert f.sun_el is None and not f.sun_dark and f.is_dark and f.why == 'exposure'
  assert not feed(f, 0.1, LEAVE_S + 0.1)[-1] and f.why == 'light'
  assert feed(f, 7., ENTER_S + 0.1)[-1]
  # a sun known, then lost: back to the exposure alone
  f = DarkFilter(dark=False)
  f.set_sun(-5.)
  assert f.is_dark
  f.set_sun(sun)
  assert not f.is_dark
  assert sun_elevation(None, sun_time(-3.)) is None and sun_elevation((0., 0.), None) is None


def test_the_sun_is_off_in_the_sim():
  # BeamNG renders its own daylight: the sim's filter starts light and the wall clock's sun never darkens it
  f = dark_filter_for({'SIMULATION': '1'})
  f.set_sun(-10.)
  assert not f.sun_on and f.sun_el is None and not f.is_dark
  assert not any(feed(f, 0., 60.))


def test_the_sun_agrees_with_the_stop_sign_go_on_dusk():
  # the stop-sign go holds its vision go to night from the sun's 0 deg down (junction_watch.lighting_at: dusk, then night;
  # stop_go maps every row but day and sunAhead to night): the left's dark is the same line, hour by hour through an evening
  from openpilot.selfdrive.turnpilot import stop_rules as sr
  from openpilot.selfdrive.turnpilot.junction_watch import lighting_at
  j = sr.Junction(kind='stop', status='ok', lat=0., lon=0.)
  t0 = sun_time(15.)
  for k in range(0, 4 * 3600, 60):
    f = DarkFilter(dark=False)
    f.set_sun(sun_elevation((0., 0.), t0 + k))
    row, _ = lighting_at(j, t0 + k, None)
    assert f.is_dark == (row not in ('day', 'sunAhead')), (k, row, f.sun_el)


def test_the_car_fix_is_last_gps_position():
  # ubloxd's or qcomgpsd's saved fix (a gpsLocationExternal event), as navd seeds from it; nothing saved, unreadable, or no
  # Params at all is no fix (the exposure alone)
  from openpilot.cereal import messaging

  class FakeParams(dict):
    def get(self, key):
      return super().get(key)
  saved = messaging.new_message('gpsLocationExternal', valid=True)
  saved.gpsLocationExternal.latitude, saved.gpsLocationExternal.longitude, saved.gpsLocationExternal.hasFix = 0., 0., True
  assert car_fix(FakeParams(LastGPSPosition=saved.to_bytes())) == (0., 0.)
  assert car_fix(FakeParams()) is None and car_fix(FakeParams(LastGPSPosition=b'not capnp')) is None and car_fix(None) is None
