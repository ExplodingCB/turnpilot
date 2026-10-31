"""Planar geometry for routes, and route-relative map pose and progress tracking."""
import numpy as np

METERS_PER_DEGREE = 2 * np.pi * 6371008.8 / 360  # mean Earth radius, as Mapbox measures distances

# Training renders the map on the route rather than at the measured position:
# the ego is projected onto the route, the frame is centered there and headed along the route's tangent.
PROJECTION_WINDOW = (-30.0, 100.0)  # meters around the previous progress to search; a loop cannot capture the ego
HEADING_HALF_WINDOW = 50.0  # meters; the heading is the chord of the route this far behind and ahead
# TurnPilot (off-route turn, F3): navInstruction.routeTurns, the route's heading change per ROUTE_TURNS_STEP_M of it from
# ROUTE_TURNS_BEHIND_M behind the car's matched place to ROUTE_TURNS_AHEAD_M ahead. navd's fix is 1 Hz and the car runs on from
# it (15 m a second at 15 m/s, and the fix's own latency): the span covers turnpilotd's 30 m window until the next message
ROUTE_TURNS_STEP_M = 5.0
ROUTE_TURNS_BEHIND_M = 60.0
ROUTE_TURNS_AHEAD_M = 60.0


def local_meters(points, origin) -> np.ndarray:
  """Equirectangular east/north meters of (longitude, latitude) points around an origin."""
  origin = np.asarray(origin, dtype=np.float64)
  scale = np.array([METERS_PER_DEGREE * np.cos(np.radians(origin[1])), METERS_PER_DEGREE])
  return (np.asarray(points, dtype=np.float64) - origin) * scale


def nearest_segment(starts: np.ndarray, ends: np.ndarray, point) -> tuple[int, float, float]:
  """(index, fraction along it, distance) of the closest spot on the segments, in the inputs' units."""
  ab = ends - starts
  t = np.clip(((point - starts) * ab).sum(axis=1) / np.maximum((ab * ab).sum(axis=1), 1e-12), 0, 1)
  d = np.linalg.norm(starts + t[:, None] * ab - point, axis=1)
  i = int(d.argmin())
  return i, float(t[i]), float(d[i])


def cumulative_distance(route: np.ndarray) -> np.ndarray:
  """Meters along [(longitude, latitude), ...]."""
  (lon1, lat1), (lon2, lat2) = route[:-1].T, route[1:].T
  steps = np.hypot((lon2 - lon1) * np.cos(np.radians((lat1 + lat2) / 2)), lat2 - lat1) * METERS_PER_DEGREE
  return np.r_[0.0, np.cumsum(steps)]


def along_path(path: np.ndarray, point) -> tuple[float, float]:
  """(meters along a (longitude, latitude) path to a point, and the point's distance from the path):
  the path up to the start of the nearest segment, then straight to the point."""
  local, p = local_meters(path, path[0]), local_meters(point, path[0])
  segment, _, lateral = nearest_segment(local[:-1], local[1:], p)
  return float(cumulative_distance(path)[segment] + np.linalg.norm(local_meters(point, path[segment]))), lateral


def route_turns(geometry, step_idx: int, along_step: float) -> tuple[list[float], float]:
  """(routeTurns, routeTurnsStart) for a car along_step m along step step_idx of a route's per-step (longitude, latitude)
  geometries: the polyline's heading change (deg, + right: bearings run clockwise) over each ROUTE_TURNS_STEP_M bin, the bins
  edged at the car's place, from ROUTE_TURNS_BEHIND_M behind to ROUTE_TURNS_AHEAD_M ahead, clipped at the route's ends, and the
  first bin's start from the car's place (m, <= 0). ([], 0.) for a route with no length."""
  path = np.concatenate([np.asarray(g, dtype=np.float64).reshape((-1, 2)) for g in geometry])
  cum = cumulative_distance(path)
  keep = np.flatnonzero(np.diff(cum) > 0.1)  # a step's last point repeats as the next one's first
  if not len(keep):
    return [], 0.
  (lon1, lat1), (lon2, lat2) = path[keep].T, path[keep + 1].T
  bearings = np.degrees(np.arctan2((lon2 - lon1) * np.cos(np.radians((lat1 + lat2) / 2)), lat2 - lat1))
  starts = cum[keep]
  car = sum(float(cumulative_distance(np.asarray(g, dtype=np.float64).reshape((-1, 2)))[-1]) for g in geometry[:step_idx]) + along_step
  car = min(max(car, 0.), float(cum[-1]))
  behind = int(np.floor(min(ROUTE_TURNS_BEHIND_M, car) / ROUTE_TURNS_STEP_M + 1e-9))
  ahead = int(np.floor(min(ROUTE_TURNS_AHEAD_M, float(cum[-1]) - car) / ROUTE_TURNS_STEP_M + 1e-9))
  edges = car + np.arange(-behind, ahead + 1) * ROUTE_TURNS_STEP_M
  at = bearings[np.clip(np.searchsorted(starts, edges, side='right') - 1, 0, len(starts) - 1)]
  turns = (np.diff(at) + 180.) % 360. - 180.
  return [float(t) for t in turns], -behind * ROUTE_TURNS_STEP_M


class RouteTracker:
  """Progress of the ego along a route, in meters, and the map pose at that progress."""

  def __init__(self, route: np.ndarray):
    self.route = np.asarray(route, dtype=np.float64).reshape((-1, 2))
    self.path = local_meters(self.route, self.route[0])
    self.distance = cumulative_distance(self.route)
    self.progress = -np.inf  # the first fix may land anywhere on the route

  def project(self, longitude: float, latitude: float) -> float:
    """Advance to the nearest spot on the route within the window ahead of the previous progress."""
    near = np.flatnonzero((self.distance[1:] >= self.progress + PROJECTION_WINDOW[0]) & (self.distance[:-1] <= self.progress + PROJECTION_WINDOW[1]))
    if not len(near):  # only before the first fix, when the progress is still -inf
      near = np.arange(len(self.path) - 1)
    point = local_meters([longitude, latitude], self.route[0])
    segment, along, _ = nearest_segment(self.path[near], self.path[near + 1], point)
    segment = near[segment]
    self.progress = max(self.progress, self.distance[segment] + along * (self.distance[segment + 1] - self.distance[segment]))
    return self.progress

  def pose(self, progress: float) -> tuple[float, float, float]:
    """(longitude, latitude, bearing) on the route at a progress, headed along the +-50m chord."""
    ends = np.clip([progress - HEADING_HALF_WINDOW, progress, progress + HEADING_HALF_WINDOW], 0, self.distance[-1])
    (lon0, lon, lon1), (lat0, lat, lat1) = (np.interp(ends, self.distance, self.route[:, axis]) for axis in (0, 1))
    bearing = np.degrees(np.arctan2((lon1 - lon0) * np.cos(np.radians(lat0)), lat1 - lat0)) % 360
    return float(lon), float(lat), float(bearing)

  def snap(self, longitude: float, latitude: float) -> tuple[float, float, float]:
    return self.pose(self.project(longitude, latitude))
