#!/usr/bin/env python3
import time

import numpy as np

import openpilot.cereal.messaging as messaging
from msgq.visionipc import VisionIpcServer
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.pmtiles import PMTiles
from openpilot.selfdrive.navigation.renderer import SIZE, MapRenderer
from openpilot.selfdrive.navigation.route import RouteTracker
from openpilot.selfdrive.navigation.tiles import RemoteTiles

RETRY_SECONDS = 10
OFFROAD_PERIOD = 1.0  # TurnPilot: s between updates offroad, where mapsd only fetches the route's tiles (main)


class MapsD:
  def __init__(self):
    self.renderer = None
    self.tracker = None
    self.route = np.empty((0, 2))
    self.prefetched = False
    self.retry_at = 0.0

  def update(self, sm, render: bool = True) -> tuple[np.ndarray, int]:
    """The map frame for the latest messages: black with a zero timestamp without a usable map, never stale. TurnPilot:
    render False (offroad) only fetches the route's tiles, so the first frame onroad is a cache hit."""
    if sm.updated['navRoute']:
      route = np.array([(c.longitude, c.latitude) for c in sm['navRoute'].coordinates], dtype=np.float64).reshape((-1, 2))
      if not np.array_equal(route, self.route):
        self.route = route
        self.tracker = RouteTracker(route) if len(route) >= 2 else None  # progress along the route starts over
        self.prefetched = False
    now = time.monotonic()
    source = latest_gps_source(sm)
    try:
      if self.tracker is not None and now >= self.retry_at:
        if self.renderer is None:
          self.renderer = MapRenderer(PMTiles(RemoteTiles()))
        if not self.prefetched:
          # TODO: fetch the tiles more intelligently on demand
          # Every tile the route can show is fetched up front, so each render below is a cache hit.
          self.renderer.prefetch(self.route)
          self.prefetched = True
        if render and source is not None:
          gps = sm[source]
          # As in training, the map is centered on the route and headed along it, not at the GPS fix.
          image, valid = self.renderer.render(*self.tracker.snap(gps.longitude, gps.latitude), self.route)
          if valid:
            return image, sm.logMonoTime[source]
    except Exception:  # a failed fetch, or a corrupt cached tile
      cloudlog.exception('mapsd.render_failed')
      self.close()  # a changed archive needs a fresh reader
      self.retry_at = now + RETRY_SECONDS
    return np.zeros((SIZE, SIZE), dtype=np.uint8), 0

  def close(self):
    if self.renderer is not None:
      self.renderer.archive.close()
    self.renderer = None
    self.prefetched = False


def started(sm) -> bool:
  """TurnPilot: mapsd runs offroad too while TurnPilot is enabled (process_config.navigation), to fetch the tiles of the
  route navd plans while the car is parked. Before deviceState is heard it is onroad, as mapsd always was."""
  return not sm.seen['deviceState'] or sm['deviceState'].started


def main():
  rk = Ratekeeper(20.0)
  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal', 'navRoute', 'deviceState'])
  server = VisionIpcServer('mapsd')
  server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_MAP, 4, SIZE, SIZE, SIZE * SIZE, SIZE, SIZE * SIZE)
  server.start_listener()
  mapsd = MapsD()
  frame = 0
  try:
    while True:
      sm.update(0)
      onroad = started(sm)
      image, timestamp = mapsd.update(sm, render=onroad)
      server.send(VisionStreamType.VISION_STREAM_MAP, image.tobytes(), frame, timestamp, time.monotonic_ns())
      frame += 1
      if onroad:
        rk.keep_time()
      else:
        time.sleep(OFFROAD_PERIOD)  # nothing renders or reads frames offroad; the Ratekeeper resyncs onroad
  finally:
    mapsd.close()


if __name__ == '__main__':
  main()
