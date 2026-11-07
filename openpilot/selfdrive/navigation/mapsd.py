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
from openpilot.selfdrive.navigation.tiles import METADATA_STEP, RemoteTiles, TileFetchError

RETRY_SECONDS = 10
OFFROAD_PERIOD = 1.0  # TurnPilot: s between updates offroad, where mapsd only fetches the route's tiles (main)
REFUSED_STATUSES = (401, 403, 404)  # TurnPilot: nav/tiles answers that won't change in seconds (tiles.TileFetchError)
REFUSED_RETRY_S = 300  # TurnPilot: s between tries after a refusal; a 401 is also a device JWT from a skewed clock (common/api.py)


def refusal(e: Exception) -> int | None:
  """TurnPilot: the HTTP status when comma's nav/tiles call refused this device outright (401, 403 or 404), else None.
  A range read's error is not one: a 403 or 412 there is the signed URL expiring or the archive changing, and the next
  reader fetches a fresh URL."""
  if isinstance(e, TileFetchError) and e.step == METADATA_STEP and e.status in REFUSED_STATUSES:
    return e.status
  return None


class MapsD:
  def __init__(self):
    self.renderer = None
    self.tracker = None
    self.route = np.empty((0, 2))
    self.prefetched = False
    self.retry_at = 0.0
    self.refused: int | None = None  # TurnPilot: nav/tiles' last refusal status, logged when it changes (update)
    self.relog = False  # TurnPilot: a drive started since that refusal was logged: logged again, in this drive's log
    self.rendering = False  # TurnPilot: the last update's render, whose rise is a drive starting

  def update(self, sm, render: bool = True) -> tuple[np.ndarray, int]:
    """The map frame for the latest messages: black with a zero timestamp without a usable map, never stale. TurnPilot:
    render False (offroad) only fetches the route's tiles, so the first frame onroad is a cache hit.

    TurnPilot, 2026-11-06: when comma's nav/tiles call refuses the device (401, 403 or 404: refusal), mapsd logs
    mapsd.tiles_refused with the status when it changes, not a traceback, and tries again every REFUSED_RETRY_S, and at
    once when a drive starts (render rising), which logs it again in that drive's log. Both drives tried every 10 s while
    they had a route, each a traceback that said nothing about why (81 in 1e's qlogs, 57 in 1d's), while the map stayed
    black. The hold is 5 minutes, not the rest of the drive as first written (review, the same day): a 401 is also comma's
    answer to a device JWT signed by a clock that is off (common/api.py get_token signs nbf and exp from it), which a time
    sync fixes, and a Tesla's drive lasts as long as the car is awake. A reroute is not a new drive: 1e had 18 routes, rerouting in
    loops, and none starts the tries over. Any other failure retries after RETRY_SECONDS, as before. The next answer that
    isn't a refusal ends it, and a fetch that works again (mid-drive or at a drive's start) logs mapsd.tiles_refusal_over."""
    if render and not self.rendering and self.refused is not None:  # a drive starts: a refusal is tried at once, and logged again
      self.retry_at = 0.
      self.relog = True
    self.rendering = render
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
          if self.refused is not None:  # nav/tiles answered and the archive's header read: the refusal is over
            cloudlog.event('mapsd.tiles_refusal_over', step=METADATA_STEP, status=self.refused)
            self.refused, self.relog = None, False
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
    except Exception as e:  # a failed fetch, or a corrupt cached tile
      if (status := refusal(e)) is not None:
        if status != self.refused or self.relog:
          cloudlog.event('mapsd.tiles_refused', error=True, step=METADATA_STEP, status=status, retry_s=REFUSED_RETRY_S)
        self.refused, self.relog = status, False
        self.retry_at = now + REFUSED_RETRY_S
      else:
        cloudlog.exception('mapsd.render_failed')
        self.refused, self.relog = None, False
        self.retry_at = now + RETRY_SECONDS
      self.close()  # a changed archive needs a fresh reader
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
