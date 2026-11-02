"""The route on the nav view's map: navRoute's polyline rasterized north up, framed on the route ahead and the car, and rebuilt only
when the route or the frame changes. Numpy only (navigation/renderer.py's antialiased strokes), no raylib: the view uploads the
image to a texture once per rebuild and draws the car, which moves every frame, on top with a few draw calls.

Why not mapsd's renderer: it rasterizes Protomaps roads at 1 m per pixel around the car for the model, 256 px across, from tiles
fetched along the route. A whole-route overview needs a scale tens of times coarser and tiles the device never downloaded, and a
second road raster in the UI process is CPU the comma doesn't have. The camera view's nav panel already shows mapsd's frame.

Why north up: a heading-up map redraws every time the car turns. North up, only the car's arrow turns.
"""
import numpy as np

from openpilot.selfdrive.navigation.renderer import draw_segments
from openpilot.selfdrive.navigation.route import RouteTracker, cumulative_distance, local_meters

MIN_SPAN_M = 300.  # never zoomed in past this many meters across: a short route isn't a few blown-up pixels
PAD_PX = 20.  # kept free inside the map's edge for the car's arrow and the destination's marker
EDGE_PX = 8.  # the car this near the edge reframes, so it never leaves the map
ZOOM_IN_FILL = 0.5  # zoom in once the route ahead and the car fit in half the frame: a few steps a drive, each one rebuild
ROUTE_WIDTH_PX = 5.
SIMPLIFY_PX = 0.5  # Douglas-Peucker tolerance: under a pixel, so the line is unchanged, and segments stay a few hundred
CORNER_RADIUS_PX = 24.  # the camera view's rounded corners
STALE_S = 2.  # this long without a fix (the view off screen, a GPS gap), progress may be anywhere: search the whole route again
ROUTE_RGB = (72, 235, 132)  # the camera view's nav panel's route green


def simplify(points: np.ndarray, tolerance: float) -> np.ndarray:
  """Indices of the points a Douglas-Peucker simplification keeps, ends included. Distances are to the chord as a segment, not a
  line, so a route that goes out and back along one street keeps its far end."""
  n = len(points)
  if n < 3:
    return np.arange(n)
  keep = np.zeros(n, dtype=bool)
  keep[[0, -1]] = True
  stack = [(0, n - 1)]
  while stack:
    i, j = stack.pop()
    if j <= i + 1:
      continue
    a, ab, inner = points[i], points[j] - points[i], points[i + 1:j]
    t = np.clip(((inner - a) @ ab) / max(float(ab @ ab), 1e-12), 0., 1.)
    d = np.hypot(*(inner - a - t[:, None] * ab).T)
    k = int(d.argmax())
    if d[k] > tolerance:
      keep[i + 1 + k] = True
      stack += [(i, i + 1 + k), (i + 1 + k, j)]
  return np.flatnonzero(keep)


def rotated(points: np.ndarray, bearing: float) -> np.ndarray:
  """Screen points (x right, y down) turned clockwise by a compass bearing in degrees, for the car's arrow on a north-up map:
  (0, -1), up, is north, and at 90 points east."""
  c, s = np.cos(np.radians(bearing)), np.sin(np.radians(bearing))
  return np.asarray(points, dtype=np.float64) @ np.array([[c, s], [-s, c]])


def rounded_mask(width: int, height: int, radius: float) -> np.ndarray:
  """(height, width) coverage of a rounded rectangle, antialiased at the corners."""
  px, py = np.arange(width) + 0.5, (np.arange(height) + 0.5)[:, None]
  dx = np.maximum(np.maximum(radius - px, px - (width - radius)), 0.)
  dy = np.maximum(np.maximum(radius - py, py - (height - radius)), 0.)
  return np.clip(radius + 0.5 - np.hypot(dx, dy), 0., 1.)


class Frame:
  """North-up pixels of a width x height image: east right, north up, scale px per meter around center (meters east and north of
  origin, a (longitude, latitude))."""

  def __init__(self, origin, center, scale: float, width: int, height: int):
    self.origin, self.center, self.scale, self.width, self.height = np.asarray(origin), np.asarray(center), scale, width, height

  def to_px(self, meters: np.ndarray) -> np.ndarray:
    m = np.asarray(meters, dtype=np.float64).reshape((-1, 2)) - self.center
    return np.column_stack((self.width / 2 + m[:, 0] * self.scale, self.height / 2 - m[:, 1] * self.scale))

  def inside(self, px, margin: float) -> bool:
    x, y = np.asarray(px, dtype=np.float64).reshape(2)
    return margin <= x <= self.width - margin and margin <= y <= self.height - margin


def fit(meters: np.ndarray, origin, width: int, height: int) -> Frame:
  """The frame showing every point (meters east and north of origin) inside PAD_PX, at least MIN_SPAN_M across."""
  lo, hi = meters.min(axis=0), meters.max(axis=0)
  room = np.array([width, height], dtype=np.float64) - 2 * PAD_PX
  scale = float(min(room / np.maximum(hi - lo, 1e-6)))
  scale = min(scale, float(min(room)) / MIN_SPAN_M)
  return Frame(origin, (lo + hi) / 2, scale, width, height)


class RouteMap:
  """The route image for one map size, and where things are on it. set_route on every navRoute, update on every GPS fix: both are
  cheap unless the image needs rebuilding, which bumps version so the view re-uploads it."""

  def __init__(self, width: int, height: int):
    self.width, self.height = width, height
    self.mask = rounded_mask(width, height, CORNER_RADIUS_PX)
    self.route = np.empty((0, 2))
    self.meters = np.empty((0, 2))
    self.distance = np.empty(0)
    self.tracker: RouteTracker | None = None
    self.frame: Frame | None = None
    self.image: np.ndarray | None = None  # (height, width, 4) straight-alpha RGBA, transparent off the route
    self.version = 0
    self.last_fix = -np.inf

  @property
  def has_route(self) -> bool:
    return self.tracker is not None

  def set_route(self, route: np.ndarray) -> None:
    """navRoute's (longitude, latitude) points. navd resends the same route every 10 s: only a different one resets."""
    route = np.asarray(route, dtype=np.float64).reshape((-1, 2))
    if np.array_equal(route, self.route):
      return
    self.route = route
    self.tracker, self.frame, self.image = None, None, None
    self.version += 1
    if len(route) >= 2:
      self.meters = local_meters(route, route.mean(axis=0))
      self.distance = cumulative_distance(route)
      self.tracker = RouteTracker(route)

  @property
  def progress(self) -> float:
    """Meters along the route the car was last matched to, -inf before the first fix."""
    return self.tracker.progress if self.tracker is not None else -np.inf

  def point_at(self, along: float) -> np.ndarray:
    """Meters east and north of the route's origin at a distance along it, clipped to its ends."""
    along = float(np.clip(along, 0., self.distance[-1]))
    return np.array([np.interp(along, self.distance, self.meters[:, axis]) for axis in (0, 1)])

  def car_meters(self, car) -> np.ndarray:
    return local_meters(np.asarray(car, dtype=np.float64), self.route.mean(axis=0))

  def update(self, car, now: float) -> bool:
    """car: (longitude, latitude) of a new fix, None without one. Moves the progress along the route, reframes when the car nears
    the edge or the route ahead has shrunk to half the frame, and rebuilds the image if the frame changed. True if it did."""
    if self.tracker is None:
      return False
    car_m = None
    if car is not None:
      if now - self.last_fix > STALE_S:
        self.tracker.progress = -np.inf  # projection only searches near the last progress, which may be far behind now
      self.last_fix = now
      self.tracker.project(*car)
      car_m = self.car_meters(car)
    if not self._needs_frame(car_m):
      return False
    ahead = self.meters[self.distance >= self.progress] if np.isfinite(self.progress) else self.meters
    points = np.vstack([p for p in (ahead, self.meters[-1:], car_m) if p is not None])
    self.frame = fit(points, self.route.mean(axis=0), self.width, self.height)
    self._rebuild()
    return True

  def _needs_frame(self, car_m) -> bool:
    if self.frame is None:
      return True
    if car_m is not None and not self.frame.inside(self.frame.to_px(car_m), EDGE_PX):
      return True
    max_scale = (min(self.width, self.height) - 2 * PAD_PX) / MIN_SPAN_M
    if self.frame.scale >= max_scale * 0.99 or not np.isfinite(self.progress):
      return False
    # the route ahead (and the car) would fit in ZOOM_IN_FILL of the frame: zoom in on it
    ahead = self.meters[self.distance >= self.progress]
    points = np.vstack([p for p in (ahead, self.meters[-1:], car_m) if p is not None])
    span = (points.max(axis=0) - points.min(axis=0)) * self.frame.scale
    room = np.array([self.width, self.height]) - 2 * PAD_PX
    return bool(np.all(span <= room * ZOOM_IN_FILL))

  def _rebuild(self) -> None:
    assert self.frame is not None
    px = self.frame.to_px(self.meters)
    px = px[simplify(px, SIMPLIFY_PX)]
    coverage = np.zeros((self.height, self.width), dtype=np.uint8)
    draw_segments(coverage, px[:-1], px[1:], ROUTE_WIDTH_PX / 2, 255)
    image = np.empty((self.height, self.width, 4), dtype=np.uint8)
    image[..., :3] = ROUTE_RGB
    image[..., 3] = np.rint(coverage * self.mask).astype(np.uint8)
    self.image = image
    self.version += 1
