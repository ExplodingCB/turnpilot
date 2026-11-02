"""navd itself (navigation/navd.py RouteEngine) stepped by hand at its 1 Hz, on a synthetic route: east JUNCTION_M to a left
at a T, north NORTH_M to a right at another, east EAST_M to the end. The car's true place is s m along that polyline; its
GPS fix, speed and heading are what each tick says, so a test can lose, jitter or degrade the GPS at any point and read
what navd publishes. Positions are synthetic, around (0, 0)."""
import math
import time
from unittest.mock import MagicMock

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.navigation.navd import RouteEngine
from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE

JUNCTION_M, NORTH_M, EAST_M = 300., 400., 200.
ARC_M = 8.  # the car's turn starts this far before a junction's node and ends this far past it


def point(east: float, north: float) -> tuple:
  return east / METERS_PER_DEGREE, north / METERS_PER_DEGREE


END = point(JUNCTION_M + EAST_M, NORTH_M)


def place(s: float) -> tuple:
  """(longitude, latitude) s m along the route's polyline."""
  if s <= JUNCTION_M:
    return point(s, 0.)
  if s <= JUNCTION_M + NORTH_M:
    return point(JUNCTION_M, s - JUNCTION_M)
  return point(s - NORTH_M, NORTH_M)


def heading(s: float) -> float:
  """deg the car heads s m along the route, turning over ARC_M either side of each junction."""
  def turn(at, before, after):
    return before + (after - before) * min(max((s - at + ARC_M) / (2 * ARC_M), 0.), 1.)
  return turn(JUNCTION_M + NORTH_M, turn(JUNCTION_M, 90., 0.), 90.) if s > JUNCTION_M + ARC_M else turn(JUNCTION_M, 90., 0.)


def yaw_rate(s: float, v: float) -> float:
  """rad/s, left positive (carState.yawRate's sense), of a car at v m/s driving heading()."""
  return -math.radians(heading(s + 0.05) - heading(s - 0.05)) / 0.1 * v


def route_steps() -> list[dict]:
  """navd's steps (Mapbox's format, as the router gives them) for the route, with Valhalla's junction data."""
  def coords(*points):
    return [list(point(e, n)) for e, n in points]
  j, n, e = JUNCTION_M, NORTH_M, EAST_M
  return [
    {'geometry': {'coordinates': coords((0, 0), (j / 2, 0), (j, 0))}, 'distance': j, 'duration': j / 10.,
     'bannerInstructions': [{'distanceAlongGeometry': 200., 'primary': {'text': 'Turn left', 'type': 'turn', 'modifier': 'left'}}]},
    {'geometry': {'coordinates': coords((j, 0), (j, n / 2), (j, n))}, 'distance': n, 'duration': n / 10.,
     'maneuver': {'type': 'turn', 'modifier': 'left', 'bearing_before': 90., 'bearing_after': 0.},
     'intersections': [{'location': list(point(j, 0)), 'bearings': [0, 180, 270], 'in': 2, 'out': 0}],
     'bannerInstructions': [{'distanceAlongGeometry': 200., 'primary': {'text': 'Turn right', 'type': 'turn', 'modifier': 'right'}}]},
    {'geometry': {'coordinates': coords((j, n), (j + e, n))}, 'distance': e, 'duration': e / 10.,
     'maneuver': {'type': 'turn', 'modifier': 'right', 'bearing_before': 0., 'bearing_after': 90.},
     'intersections': [{'location': list(point(j, n)), 'bearings': [90, 180, 270], 'in': 1, 'out': 0}],
     'bannerInstructions': [{'distanceAlongGeometry': 100., 'primary': {'text': 'Arrive', 'type': 'arrive', 'modifier': 'straight'}}]},
    {'geometry': {'coordinates': coords((j + e, n), (j + e, n))}, 'distance': 0., 'duration': 0.,
     'maneuver': {'type': 'arrive', 'modifier': ''}, 'bannerInstructions': []},
  ]


class StepsRouter:
  """navd's router (turnpilot/routing/sources.py) answering every route with the same steps and no map controls."""
  def __init__(self):
    self.calls: list = []
    self.status = MagicMock()

  def route(self, start, destination, bearing, language, position=None, cancelled=None):
    self.calls.append((start, bearing))
    return route_steps()

  def controls(self, steps):
    return [], 'ok'

  def opposing(self, steps):
    return {}

  def controls_ahead(self, position, bearing):
    return [], 'ok', []

  def clear(self):
    pass


class NavdSM:
  """navd's SubMaster, filled by NavdLoop.tick: gpsLocationExternal (gpsLocation stays without a fix), carState, deviceMotion
  and turnpilotPlan as the tick says, each received just now."""
  def __init__(self):
    self.frame = -1
    self.data, self.valid, self.alive, self.updated, self.recv_time, self.logMonoTime = {}, {}, {}, {}, {}, {}
    msg = messaging.new_message('gpsLocation')
    self.put('gpsLocation', msg.as_reader().gpsLocation, valid=False)

  def __getitem__(self, s):
    return self.data[s]

  def update(self, timeout=0):
    self.frame += 1

  def put(self, s, reader, valid=True):
    self.data[s], self.valid[s], self.alive[s], self.updated[s] = reader, valid, True, True
    self.recv_time[s], self.logMonoTime[s] = time.monotonic(), self.frame + 1


class NavdLoop:
  """A RouteEngine routing to the route's end, ticked once per navd update. motion: carState is published (onroad); yaw:
  deviceMotion is too."""
  def __init__(self, motion: bool = True, yaw: bool = True):
    self.values = {'NavDestination': {'latitude': END[1], 'longitude': END[0]}, 'LanguageSetting': 'main_en'}
    params = MagicMock()
    params.get.side_effect = self.values.get
    params.remove.side_effect = lambda key: self.values.pop(key, None)
    self.router, self.sm, self.pm, self.t = StepsRouter(), NavdSM(), MagicMock(), 0.
    self.motion, self.yaw = motion, yaw
    self.engine = RouteEngine(self.sm, self.pm, params=params, router=self.router, background=False, monotonic=lambda: self.t)

  def tick(self, s: float, v: float, fix='true', car_heading: float | None = None, dt: float = 1., turn_missed: bool = False):
    """One navd update for the car s m along the route at v m/s, dt s after the last. fix: 'true' (its true place, 3 m
    accuracy), None (no fix: hasFix false), or (east m, north m, accuracy m), a fix that far off the true place reporting
    that accuracy. car_heading: deg, deviceMotion's heading instead of the route's (a car that left it). turn_missed:
    turnpilotd's turnMissed. (instruction reader, valid) of what navd published."""
    self.t += dt
    self.sm.updated = dict.fromkeys(self.sm.data, False)
    msg = messaging.new_message('gpsLocationExternal')
    gps = msg.gpsLocationExternal
    lon, lat = place(s)
    gps.bearingDeg = heading(s) if car_heading is None else car_heading
    if fix is None:
      gps.hasFix = False
    else:
      east, north, accuracy = (0., 0., 3.) if fix == 'true' else fix
      gps.hasFix, gps.horizontalAccuracy = True, accuracy
      gps.speed = float(v)
      gps.bearingAccuracyDeg = 2. if fix == 'true' and v > 0. else 0.  # a good heading of motion only moving on a true fix
      gps.longitude, gps.latitude = float(lon + east / METERS_PER_DEGREE), float(lat + north / METERS_PER_DEGREE)
    self.sm.put('gpsLocationExternal', msg.as_reader().gpsLocationExternal)
    if self.motion:
      cs = messaging.new_message('carState')
      cs.carState.vEgo = float(v)
      self.sm.put('carState', cs.as_reader().carState)
    if self.yaw:
      dm = messaging.new_message('deviceMotion')
      dm.deviceMotion.orientationNED.z = math.radians(heading(s) if car_heading is None else car_heading)
      dm.deviceMotion.orientationNED.valid = True
      self.sm.put('deviceMotion', dm.as_reader().deviceMotion)
    if turn_missed:
      tp = messaging.new_message('turnpilotPlan')
      tp.turnpilotPlan.turnMissed = True
      self.sm.put('turnpilotPlan', tp.as_reader().turnpilotPlan)
    self.engine.update()
    sent = next(c.args[1] for c in reversed(self.pm.send.call_args_list) if c.args[0] == 'navInstruction')
    return sent.as_reader().navInstruction, sent.valid
