"""After dark: the road camera's exposure, filtered with hysteresis, for the unprotected left's night range cap
(unprotected_left.NIGHT_LEFT_RANGE_M; turnpilotd: TurnPilot.dark).

Kellen, 2026-11-06: the left goes where the camera covers the oncoming road, else it holds at its line and hands back loudly.
At night it doesn't cover the road out to the left's R_req: on the 11-06 night drive (route 1e) the full pass's raw vehicle
boxes reached p99 50 m and max 78 m, oncoming tracks first published at p90 28-53 m, so a band read clear to 134-200 m (a 25-35
mph road's R_req) was unverified. turnpilotd had no lighting input at all (no GPS, no clock, no sun), so this reads the one
that is there from the first frame: narrowRoadCameraState.exposureValPercent, which camerad publishes with every road frame.

What it read on this comma four (os04c10, gain 1.0 throughout; seen across every downloaded route, 11-06):
  daylight, sun +16 deg (routes 0f, 10, qlogs):              0.01-0.13 %
  a garage, standing, no GPS (routes 12-1c):                  2.2-3.3 % (1c: p10 2.77, p50 2.77)
  dusk to night, sun -10 to -12 deg (1d):                     p10 3.0, p50 5.6 %
  night, sun -15 to -17 deg (1e):                             p10 5.0, p50 7.1, p90 9.7 %
So dark is ENTER_EV (1.0 %, 8x the brightest daylight seen, a third of the darkest dusk) held ENTER_S (5 s), and light again
only at LEAVE_EV (0.5 %) or under held LEAVE_S (30 s): a bright stretch under street lights or a lit forecourt never makes a
night drive light (1e's longest spell under 1.0 % after the start was 7.9 s, in the dead band; none under 0.5 %), and a garage
or a tunnel counts as dark, rightly. Between the two (the dead band) neither count runs: both start over, the state kept.
camerad's auto-exposure starts every drive at 0.015 % and climbs for about 1.7-2.2 s (1c, 1d, 1e alike), so starting light
would read the first two seconds of a night drive as daylight; turnpilotd starts the car dark (fail closed: dark_filter_for),
the sim light (SIMULATION=1: BeamNG's bridge publishes no exposure, so its day scenarios keep their go path).
A missing reading (None, or not a number: no camera message this frame) keeps the state and both counts: it is no evidence
either way, and with no road frames the left has its own fault for that (stale).
The light count runs only while the car moves (moving: turnpilotd's carState.vEgo over LEAVE_MOVING_V, 5 m/s); standing or
crawling holds it where it is (review, 2026-11-06). A car standing 30 s under a lit forecourt canopy or at a brightly lit
junction reads at or under LEAVE_EV, and a night drive read light there would judge its next left on the day range (210 m),
the band nobody has verified at night. Held, not started over (second review, 2026-11-06): zeroed on every stand or crawl, the
count needed 30 s of unbroken driving over 5 m/s, and a day drive stopping every 10-25 s of driving (a stop sign or a light
every 100-250 m at 25 mph) never turned light, every left of it handed back 'night' (35 minutes of 25 s legs and 10 s stands:
never light; held, light 40 s in, 5 s into the second leg). Holding keeps the lit stand dark all the same: standing time
never counts, and a night drive's moving readings (1.0 % and over, or the dead band) zero the count before any stand.
Replayed with the logged exposure and vEgo (1e, 1d, 1c, 0f, 10), holding and zeroing give the same states and no flips: the
night drives had no moving frame at or under 0.5 %. The daylight readings behind LEAVE_EV are the weaker evidence: routes 0f
and 10 are qlogs of 12 and 167 frames taken with the car parked (vEgo 0 throughout), and no moving daylight exposure has been
logged yet, so a lit stand is exactly where 0.5 % is least proven. The parked daylight read 0.01-0.13 %; no moving daylight has been logged,
so the 30 s daylight count over 5 m/s that makes a day drive light is unexercised on the car (watch_dark's per-minute summary
is there to check it: replayed, the parked routes 0f and 10 now never turn light, which is what this rule means for a car
that never drives off). The dark count runs standing or moving (a garage, a tunnel).
camerad's auto-exposure starts each drive at 0.015 % (above), so the drive's first logged reading (turnpilot.dark initial)
and its first minute's minimum are that ramp's start, never the scene's.
The numbers are this camera's; another sensor's exposure scale may differ (turnpilotd logs the sensor with each change, the
first reading of the drive, and a summary of the readings every EXPOSURE_LOG_S: watch_dark).
A false dark only costs a handback at the left's line; a false light is what the hysteresis is for.

The sun too (review LN-3, 2026-11-07): dark is the exposure's verdict or the sun under SUN_DARK_EL (0 deg), whatever the
exposure reads (DarkFilter.is_dark). Exposure alone left dusk uncalibrated: a drive light by day stays light through the dead
band until 1.0 % holds 5 s, and nothing logged places that against sunset (no route has a moving daylight frame, and none at
all between sun +16.5 deg, 0f and 10 parked, and -9.6 deg, 1d; modelled from the anchors the flip fell anywhere from before
sunset to sun -4 deg, the anchors 60x apart). In that window a left across a 25-35 mph road went on the day range (210 m, a
band certified clear to 134-183 m) while turnpilotd's stop-sign go the same minute held its vision go to night
(junction_watch.lighting_at: dusk from 0 to -6 deg; stop_go.py maps every row but day and sunAhead to night). Replayed through
turnpilotd (a left at 300 m, the filter light from the day, 0.75 % at sun -3 deg): the left went on the day range. 0 deg is
the geometric horizon (stop_rules.sun_position has no refraction), about 5 minutes before the sun's apparent set.
The sun's elevation is stop_rules.sun_position at the car's last fix (LastGPSPosition: ubloxd's or qcomgpsd's, saved once a
minute with a fix and kept offroad while TurnPilot is enabled, so at ignition it is where the car is parked) on turnpilotd's
wall clock while valid (decider_clock), worked out every SUN_EVERY_S (turnpilotd.watch_sun). A fix minutes old or km off moves
it by hundredths of a degree (at sunset on the 11-06 drives the sun fell 0.18 deg a minute, about 0.01 deg per km east or west). No fix
saved, no valid clock, or no Params (main() gives them: a test harness or a replay has none) is no sun: the exposure's verdict
alone, as before, the car starting dark (fail closed). The sun only ever adds dark: a wrong fix or clock never makes a night
light. Its cost: from the sun's 0 deg to the exposure's own dark (minutes to the half hour of civil twilight, not measured on
the car), every left across a 25 mph or faster road hands back 'night' at its line in light the camera may read as day, and
likewise before sunrise. turnpilotd logs each change of what makes it dark (turnpilot.dark_why: the exposure, the sun, both, or light;
the sun's elevation; once per change, never per frame). In the sim the sun is off (sun_on: BeamNG renders its own daylight),
as the exposure starts light there.
"""
import math
from dataclasses import dataclass

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.web.store import last_gps_position

ENTER_EV = 1.0     # % exposureValPercent at or above which the road reads dark...
ENTER_S = 5.       # ...held this long, it is dark
LEAVE_EV = 0.5     # % at or below which it reads light...
LEAVE_S = 30.      # ...held this long, it is light again...
LEAVE_MOVING_V = 5.  # m/s: ...counted only while the car moves faster than this (standing: held, not counted; module docstring)
EXPOSURE_LOG_S = 60.  # s: turnpilotd logs a summary of the readings this often (turnpilot.exposure: watch_dark)
SUN_DARK_EL = 0.   # deg: the sun under this is dark whatever the exposure reads (dusk and night, as the stop-sign go's)
SUN_EVERY_S = 10.  # s: turnpilotd works the sun out this often (the fix and the clock; watch_sun): 0.03 deg of it at sunset


@dataclass
class DarkFilter:
  enter_ev: float = ENTER_EV
  enter_s: float = ENTER_S
  leave_ev: float = LEAVE_EV
  leave_s: float = LEAVE_S
  dark: bool = True    # the exposure's verdict; fail closed: dark until LEAVE_S of daylight readings say otherwise
  enter_t: float = 0.  # s the readings have been at or above enter_ev without a break
  leave_t: float = 0.  # s they have been at or below leave_ev without a break, counted while the car moves
  sun_on: bool = True  # the sun counts (dark_filter_for: on the car; off in the sim, which renders its own daylight)
  sun_el: float | None = None  # deg, the sun's elevation at the car (set_sun); None: not known, the exposure's verdict alone

  def update(self, ev: float | None, dt: float, moving: bool = True) -> bool:
    """This frame's exposure reading (% of the camera's range; None for none): whether it is dark now, by the exposure or
    the sun (is_dark; module docstring).
    moving: the car is moving (turnpilotd: vEgo over LEAVE_MOVING_V); while it isn't, the light count is held where it is
    (neither counted nor started over: module docstring)."""
    if ev is None or not math.isfinite(ev):
      return self.is_dark
    if ev >= self.enter_ev:
      self.enter_t, self.leave_t = self.enter_t + dt, 0.
    elif ev <= self.leave_ev:
      self.enter_t, self.leave_t = 0., self.leave_t + dt if moving else self.leave_t
    else:
      self.enter_t = self.leave_t = 0.
    if not self.dark and self.enter_t >= self.enter_s - 1e-6:
      self.dark, self.enter_t = True, 0.
    elif self.dark and self.leave_t >= self.leave_s - 1e-6:
      self.dark, self.leave_t = False, 0.
    return self.is_dark

  def set_sun(self, el: float | None) -> None:
    """The sun's elevation at the car now (deg; None: not known, or not a number); ignored with the sun off (the sim)."""
    self.sun_el = el if self.sun_on and el is not None and math.isfinite(el) else None

  @property
  def sun_dark(self) -> bool:
    """The sun is known and under SUN_DARK_EL (an unknown sun is never dark by itself: the exposure decides)."""
    return self.sun_el is not None and self.sun_el < SUN_DARK_EL

  @property
  def is_dark(self) -> bool:
    """Dark by the exposure's verdict or the sun (module docstring): what the left's night cap reads."""
    return self.dark or self.sun_dark

  @property
  def why(self) -> str:
    """What makes it dark now: 'exposure', 'sun', 'exposure+sun', or 'light' (turnpilotd's turnpilot.dark_why)."""
    return {(True, True): 'exposure+sun', (True, False): 'exposure', (False, True): 'sun'}.get((self.dark, self.sun_dark), 'light')


def car_fix(params) -> tuple[float, float] | None:
  """(latitude, longitude) of the car's last GPS fix (LastGPSPosition, as navd seeds from it), or None: no Params (a test
  harness, a replay), none saved, or unreadable. Only ever turned into the sun's elevation, never logged."""
  return last_gps_position(params) if params is not None else None


def sun_elevation(fix: tuple[float, float] | None, wall_t: float | None) -> float | None:
  """deg, the sun's elevation at fix at Unix time wall_t (stop_rules.sun_position, as the stop-sign go's lighting); None
  without either."""
  if fix is None or wall_t is None:
    return None
  el = sr.sun_position(float(fix[0]), float(fix[1]), float(wall_t))[0]
  return el if math.isfinite(el) else None


def dark_filter_for(environ) -> DarkFilter:
  """turnpilotd's filter at start (every TurnPilot's, from its __init__, so no entry point fails open): dark on the car (fail
  closed: camerad's exposure starts low every drive and climbs, module docstring), the sun counting; light in the sim
  (SIMULATION=1), whose camera publishes no exposure, and the sun off there (BeamNG renders its own daylight)."""
  sim = environ.get('SIMULATION') == '1'
  return DarkFilter(dark=not sim, sun_on=not sim)
