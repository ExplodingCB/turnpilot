"""visibility.py under TurnPilotStopSignGo=vision where the walk's glare zeroes (low sun along an approach, the sun ahead) meet the
side views' own sectors, and a side run's sectors that say its feed is stuck (the review of 2026-11-08).

Until then the glare zero ended a lane's walk where the sun's zone starts (low sun: the side view's inner edge; the sun ahead:
67 deg), so no sector inside the zone was ever read: a side camera washed out there (the sun in its lens) read 'lowSun' or
'sunAhead', and once the creep for a view had reached its limit the stop-sign decider went on that lane by its boxes alone,
where the same sectors on a plain day held the go ('washed': never gone on). Now the lane is walked again on the sectors alone,
and where the camera goes blind short of the lane's vision floor that is its why; blind only beyond the floor, the glare's why
stays, as a plain day's walk would have gone on it."""
from dataclasses import replace

import pytest

from openpilot.cereal import custom
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.tests.test_visibility import lane_of, live, report, vision_report, vision_setup
from openpilot.selfdrive.turnpilot.visibility import CAMERA_BLIND, sector_rows, stuck_run

LOW = {'low_sun_sides': ('left',)}
AHEAD = {'lighting': 'sunAhead'}


@pytest.mark.parametrize('quality', CAMERA_BLIND)
@pytest.mark.parametrize('glare', ['lowSun', 'sunAhead'])
def test_vision_a_blind_sector_inside_the_suns_zone_short_of_the_floor_is_the_lanes_why_its_reach_the_glares(glare, quality):
  # before: why 'lowSun' (7.5 m) or 'sunAhead' (13.2 m) whatever the sectors read past the zone's start
  kw = LOW if glare == 'lowSun' else AHEAD
  plain = lane_of(vision_report(live(), **kw), 'left')
  assert plain.why == glare
  for bad in (70., 75., 84.):  # the 67.5-72.5, 72.5-77.5 and 82.5-87.5 deg sectors: all short of the 37.6 m floor (25 mph)
    x = lane_of(vision_report(live(bad=bad, quality=quality), **kw), 'left')
    assert x.why == quality and x.reach_m == pytest.approx(plain.reach_m), (bad, x.why, x.reach_m)


def test_vision_a_dim_or_low_texture_sector_inside_the_suns_zone_leaves_the_glares_why():
  # dark, blocked and degraded are judged by the walk's other rules under vision (VISION_ACCEPT): nothing the camera can't see
  for quality in ('degraded', 'dark', 'blocked'):
    assert lane_of(vision_report(live(bad=70., quality=quality), **LOW), 'left').why == 'lowSun'
    assert lane_of(vision_report(live(bad=75., quality=quality), **AHEAD), 'left').why == 'sunAhead'


def test_vision_a_blind_sector_only_beyond_the_lanes_floor_leaves_the_glares_why_as_a_plain_day_goes_on_it():
  # a 15 mph cross road (vision floor 17.6 m): a washed sector at 80 deg ends the plain-day walk 23.2 m down the near lane, past
  # its floor, so a plain day goes on it; under low sun it stays the glare's (the creep, then the go on what is seen). A washed
  # sector at 75 deg ends it at 17.2 m, short of the floor: the camera blind where it matters, under the sun as by day
  vis, lns, edges = vision_setup()
  slow = [replace(x, limit=15 * sr.MPH) for x in lns]
  floor = sr.reach_needed_vision(15 * sr.MPH, None)

  def left(bad, **kw):
    return lane_of(live(bad=bad, quality='washed').lanes(slow, 0., vis, rot_deg=sr.angle_bound(False, 1.), edges=edges, vision=True, **kw),
                   'left')
  day = left(80.)
  assert day.why == 'washed' and day.reach_m > floor + 1.
  assert left(80., **LOW).why == 'lowSun'
  day = left(75.)
  assert day.why == 'washed' and day.reach_m < floor
  assert left(75., **LOW).why == 'washed'


def test_the_other_modes_walk_glare_as_they_did():
  assert lane_of(report(live(bad=70., quality='washed'), **LOW), 'left').why == 'lowSun'
  assert lane_of(report(live(bad=75., quality='washed'), lighting='sunAhead'), 'left').why == 'sunAhead'


def test_a_side_run_whose_every_sector_reads_frozen_is_its_feed_stuck_tuples_or_the_capnp_list():
  rows = [(52.5 + 5. * k, 57.5 + 5. * k, 'frozen') for k in range(8)]
  assert stuck_run(rows) and stuck_run([(-a, -b, q) for a, b, q in rows])
  assert not stuck_run(rows[:-1] + [(87.5, 92.5, 'washed')]) and not stuck_run(rows[:-1] + [(87.5, 92.5, 'ok')])
  assert not stuck_run([]) and not stuck_run(None)  # a run with no sectors read says nothing about its feed
  msg = custom.TurnpilotDetections.new_message()
  secs = msg.init('sectorQuality', 3)
  for k, s in enumerate(secs):
    s.inner, s.outer, s.quality = -(52.5 + 5. * k), -(57.5 + 5. * k), 'frozen'
  assert stuck_run(secs) and sector_rows(secs)[0] == (52.5, 57.5, 'frozen')
  secs[2].quality = 'ok'
  assert not stuck_run(secs)
