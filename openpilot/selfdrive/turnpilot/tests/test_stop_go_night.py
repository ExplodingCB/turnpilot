"""The stop-sign decider under TurnPilotStopSignGo=vision after dark (test_stop_go_vision's World frames: the shipped visibility
file, no surveys, the map's tiers, the side views running), and the light rules it reads (stop_rules.light_window, walk_row).

Kellen, 2026-11-07: "why the hell do we care about dark? its been driving at night in all our past tests... and it performed
great the streets are very lit up, its issues with out nav system not openpilot". Until then vision's gate failed every stop
after dark (H9, "Stop sign: night at ETA +0 min: take control"), and its walk read the 'night' row, which has no table, so every
lane's reach was 0. Now dark walks the day table and the cameras judge the rest: a side view whose sectors read dark holds the
go on reach as any blind bearing does, a car coming holds it until it has passed (the note above stop_rules.DARK_ROWS)."""
import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_go as sg
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.routing import certify as ce
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, LAT, LON, utc, vision_junction
from openpilot.selfdrive.turnpilot.tests import test_stop_go as tsg
from openpilot.selfdrive.turnpilot.tests.test_stop_go import arrived_at, first_go
from openpilot.selfdrive.turnpilot.tests.test_stop_go_vision import SHIPPED, gate_fault, vcars, vworld
from openpilot.selfdrive.turnpilot.tests.test_visibility import sectors

NIGHT = ETA + 8 * 3600.          # 23:00 EDT, the sun -34 deg
DUSK = utc(2026, 10, 28, 23, 0)  # 19:00 EDT, sunset about 18:43: the sun -2 deg
DARK = {'night': NIGHT, 'dusk': DUSK, 'noClock': None}


def reaches(out, i):
  return {(x.role, x.index): round(x.r_seen, 2) for x in out[i - 1].lanes}


# ------------------------------------------------------------------------------------------------------- the light rules

def test_the_sun_says_night_and_dusk_there():
  assert sr.lighting(LAT, LON, NIGHT, 0.) == 'night' and sr.lighting(LAT, LON, DUSK, 0.) == 'dusk'


def test_vision_walks_the_day_table_for_a_dark_row_or_no_clock_every_other_mode_its_own_row():
  for row in ('night', 'dusk', 'unknown'):
    assert sr.walk_row(row, True) == sr.DARK_WALK_ROW == 'day' and sr.walk_row(row, False) == row
  for row in ('day', 'sunAhead'):
    assert sr.walk_row(row, True) == sr.walk_row(row, False) == row


@pytest.mark.parametrize('eta', [NIGHT, DUSK, utc(2026, 10, 28, 22, 30)])  # 18:30 EDT: the window's late end is dusk
def test_light_window_dark_ok_passes_night_and_dusk_on_the_day_row_and_without_it_fails_them(eta):
  rows, why = sr.light_window(LAT, LON, eta, 0., 'unmeasured', dark_ok=True)
  assert (rows, why) == ({'day'}, '')
  _, why = sr.light_window(LAT, LON, eta, 0., 'unmeasured')
  assert why.startswith(('night', 'dusk')), why  # the other modes, as they were (their stamps are per lighting row)


def test_light_window_dark_ok_still_fails_glare():
  low = utc(2026, 10, 28, 22, 0)  # 18:00 EDT: the sun low in the west-south-west
  el, az = sr.sun_position(LAT, LON, low)
  assert 0. < el < sr.SUN_EL_MAX
  _, why = sr.light_window(LAT, LON, low, 0., 'unmeasured', [az], dark_ok=True)
  assert why.startswith('low sun'), why
  _, why = sr.light_window(LAT, LON, utc(2026, 10, 28, 17, 0), 180., 'unmeasured', dark_ok=True)  # 13:00 EDT, heading south
  assert why.startswith('sun ahead'), why


# ------------------------------------------------------------------------------------------------------ the decider, after dark

@pytest.mark.parametrize('when', list(DARK))
def test_vision_after_dark_or_without_a_clock_goes_as_by_day_on_the_same_reach(tmp_path, when):
  # before 2026-11-07: the gate's handback at once (shadow: "Stop sign: night at ETA +0 min: take control", or dusk, or "the
  # clock isn't set"), and with that gate passed every lane read 0 m on the 'night' row
  day = vworld(tmp_path)
  out_day = day.run(6.)
  w = vworld(tmp_path, wall_t=DARK[when])
  out = w.run(6.)
  a, i = arrived_at(out), first_go(out)
  assert gate_fault(out) is None and not any(d.fault for d in out)
  assert i is not None and (i - a) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11) and out[i].reason == sg.GOING_REASON
  assert reaches(out, i) == reaches(out_day, first_go(out_day)) and all(x.r_seen >= x.r_req for x in out[i - 1].lanes)
  assert w.sg.gate is None


@pytest.mark.parametrize('when', ['night', 'dusk'])
def test_vision_at_night_a_car_coming_holds_the_go_until_it_has_passed(tmp_path, when):
  # a car down the left lane at 8 m/s from 45 m, 1 s after our arrival: held while it comes, the go VISION_CLEAR_S after it
  # has gone, never a handback
  w = vworld(tmp_path, wall_t=DARK[when])
  w.tracks = vcars((41, 'left', lambda t: None if t < 2. else 45. - 8. * (t - 2.)))
  out = w.run(14.)
  a, i = arrived_at(out), first_go(out)
  gone_t = 2. + 57. / 8.  # s of camera time from the start: 12 m past our path, out of the tracks
  assert i is not None and (i + 1) * DT_MDL >= gone_t + sr.VISION_CLEAR_S - 0.11
  held = [d for d in out[a:i] if any(h.key.startswith(('tracked:', 'reach:')) for h in d.holds)]
  assert held and any(h.key == 'tracked:41' for d in held for h in d.holds)
  assert not any(d.fault for d in out) and gate_fault(out) is None


def every_sector(view, quality):
  """A side view's sectorQuality with every sector quality (the inner edge to past the frame edge)."""
  sign = 1. if view == 'sideLeft' else -1.
  return [(sign * lo, sign * (lo + 5.), quality) for lo in np.arange(sr.INNER_BAND_HI_DEG, 90., 5.)]


@pytest.mark.parametrize('quality', ['dark', 'blocked'])
@pytest.mark.parametrize('when', ['night', 'dusk', 'noClock'])
def test_vision_at_night_side_views_reading_dark_go_on_the_walk_and_never_hand_back(tmp_path, monkeypatch, quality, when):
  """Review 2026-11-07 (LS-3): a side view's band at night reads under the 45 DN the sectors' dark and blocked tests read (the
  wide camera's own exposure median 31-33 raw DN at night on 1d and 1e, 17-22 full range), so every side bearing was zeroed:
  "Waiting: can't see far enough from the left (sector)" from 1 s, H3 at 16 s, never a go, at every night stop, unwarned
  (navd now certifies them). Now a dark sector, or a blocked one after dark, is judged by the walk's other rules: the car
  goes as by day, VISION_CLEAR_S after its stop, and a car coming still holds it (the next test)."""
  monkeypatch.setattr(tsg, 'sectors', lambda view: every_sector(view, quality))
  w = vworld(tmp_path, wall_t=DARK[when])
  out = w.run(10.)
  a, i = arrived_at(out), first_go(out)
  assert i is not None and (i - a) * DT_MDL == pytest.approx(sr.VISION_CLEAR_S, abs=0.11), [(d.state, d.reason) for d in out[a::20]]
  assert not any(d.fault for d in out) and gate_fault(out) is None


def test_vision_at_night_with_the_side_views_dark_a_car_coming_still_holds_the_go(tmp_path, monkeypatch):
  monkeypatch.setattr(tsg, 'sectors', lambda view: every_sector(view, 'dark'))
  w = vworld(tmp_path, wall_t=NIGHT)
  w.tracks = vcars((41, 'left', lambda t: None if t < 2. else 45. - 8. * (t - 2.)))
  out = w.run(14.)
  i = first_go(out)
  assert i is not None and (i + 1) * DT_MDL >= 2. + 57. / 8. + sr.VISION_CLEAR_S - 0.11
  assert not any(d.fault for d in out)


def test_vision_by_day_a_blocked_side_view_and_at_night_a_washed_one_still_hold_the_go_until_it_reads_again(tmp_path, monkeypatch):
  # by day a blocked sector (half its pixels static and dark: a wall, a dark van close by) is a bearing not seen, and so is a
  # washed one (headlights in the lens) at night: the lane behind it short of reach, the go waits, said in plain words; once
  # the sectors read ok the window runs and the car goes. The sim's sectors only: no real side view has been read at night
  for when, quality in ((None, 'blocked'), (NIGHT, 'washed')):
    clear_at, world = [104.], []  # camera time (World starts at 100)

    def lit_or_not(view, quality=quality, clear_at=clear_at, world=world):
      return sectors(view, bad=70., quality=quality) if world[0].t_cam < clear_at[0] else sectors(view)
    monkeypatch.setattr(tsg, 'sectors', lit_or_not)
    w = vworld(tmp_path) if when is None else vworld(tmp_path, wall_t=when)
    world.append(w)
    out = w.run(10.)
    a, i = arrived_at(out), first_go(out)
    assert i is not None and (i + 1) * DT_MDL >= (clear_at[0] - 100.) + sr.VISION_CLEAR_S - 0.11, quality
    waits = [d for d in out[a:i] if d.holds and all(h.key.startswith('reach:') for h in d.holds)]
    assert waits and any(sg.REACH_WORDS['sector'] in d.reason for d in waits), {d.reason for d in out[a:i]}
    assert not any('(sector)' in d.reason for d in out)  # the driver's words, never the walk's
    assert not any(d.fault for d in out) and gate_fault(out) is None


def test_vision_the_gate_at_night_is_glare_alone_and_geometry_still_hands_back(tmp_path):
  # what vision's gate still refuses after dark is what it refuses by day (a divided cross road here): the 1 s handback
  d = gate_fault(vworld(tmp_path, j=vision_junction(divided=True), wall_t=NIGHT).run(4.))
  assert d is not None and d.reason == "Stop sign: a divided cross road: take control" and d.fault == 'H9'
  v = ce.assess(vision_junction(), 'straight', SHIPPED, [], NIGHT, ce.Runtime(mode='vision', side_views=True, window=(0., 0.)))
  assert (v.certification, v.reason, v.rows) == ('certifiable', 'none', {'day'})
