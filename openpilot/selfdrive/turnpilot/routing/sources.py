"""Where navd's route comes from: Valhalla on a corridor of tiles from TurnPilot's map server, or on local tiles.

There is no Mapbox and no fallback. Before routing, the car and the destination are checked against the cached
coverage; then the corridor for the drive is downloaded, verified and unpacked, and every route and reroute for that
destination runs locally on it. Any failure is a MapError, reported in TurnPilotMapStatus, and navd has no route.

TurnPilotValhallaTiles is the developer and sim override (BeamNG sets it): when set, its tiles are used as they are,
with no geofence and no download.

The route's controls (routing/controls.py) come from the source that routed it, with the OSM stop-sign sidecar from the
same place (routing/osm_sidecar.py): a corridor's pack directory (its osm_controls/ and the manifest's osm_timestamp), or
the local tiles' own directory (osm_controls/ and meta.json); a valhalla_service URL has none, which is said. A sidecar is
always expected: a missing or stale one makes the controls' status partial, out loud. Their junctions are read against
the surveys (maps/surveyed_junctions.json) and TurnPilotJunctionOverrides, read for each lookup (routing/junctions.py).
"""
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.maps import corridor
from openpilot.selfdrive.turnpilot.maps.client import MapClient, MapError, Throttle, storage_error
from openpilot.selfdrive.turnpilot.maps.coverage import CoverageCache, check_trip, no_coverage, refresh
from openpilot.selfdrive.turnpilot.maps.geofence import Region
from openpilot.selfdrive.turnpilot.maps.paths import maps_dir
from openpilot.selfdrive.turnpilot.maps import surveys
from openpilot.selfdrive.turnpilot.maps.status import NavStatus
from openpilot.selfdrive.turnpilot.routing import junctions
from openpilot.selfdrive.turnpilot.routing.osm_sidecar import NO_DIRECTORY
from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource

PACKS_DIR = 'corridors'
CONFIG_KEYS = ('TurnPilotMapHost', 'TurnPilotMapKey', 'TurnPilotMapCert')
RETRY_SECONDS = 30.  # between downloads after one that could work on a retry (a network blip)
REFUSED_RETRY_SECONDS = 300.  # after a refusal (422, bad key, bad TLS, bad pack, comma storage), unless the destination or settings change
CANCELLED = 'cancelled'


def local_override(params) -> str:
  return (params.get('TurnPilotValhallaTiles') or '').strip()


def check_route_source(params):
  """TurnPilotRouteSource is left over from Mapbox routing. 'mapbox' fails loudly, so nobody thinks they're on it."""
  mode = (params.get('TurnPilotRouteSource') or 'auto').strip().lower()
  if mode == 'mapbox':
    raise MapError('mapbox_removed', "Mapbox routing was removed from TurnPilot. Set TurnPilotRouteSource back to auto.",
                   'Mapbox routing was removed')


@dataclass
class Corridor:
  destination: tuple
  pack: corridor.Pack
  source: ValhallaSource
  region: Region


@dataclass
class Failure:
  destination: tuple
  config: tuple
  error: MapError
  at: float


def cancelled_error() -> MapError:
  return MapError(CANCELLED, "The destination changed during the map download.", 'Map download cancelled')


class MapRouter:
  """navd's route source. route() returns the steps or raises a MapError; the status param says what happened."""

  def __init__(self, params, directory: Path | None = None, client_factory=MapClient.from_params, valhalla=ValhallaSource):
    self.params = params
    self.directory = directory
    self.client_factory = client_factory
    self.valhalla = valhalla
    self.status = NavStatus(params)
    self.coverage = CoverageCache(directory)
    self.local: tuple[str, ValhallaSource] | None = None
    self.corridor: Corridor | None = None
    self.failure: Failure | None = None
    # (steps, source, tiles_dir, sidecar_dir) of the last route: controls() asks the same source
    self.served: tuple | None = None

  @property
  def packs_dir(self) -> Path:
    return (self.directory or maps_dir()) / PACKS_DIR

  def config(self) -> tuple:
    return tuple(self.params.get(k) for k in CONFIG_KEYS)

  def junction_context(self) -> junctions.Context:
    """The surveys and the overrides param the junctions are read against, now (a param the build doesn't know is
    unreadable: controls.py fails every junction and says partial)."""
    overrides, ok, why = junctions.read_overrides(self.params)
    return junctions.Context(surveys.cached(surveys.SHIPPED).entries, overrides, ok, why)

  def controls(self, steps) -> tuple[list, str]:
    """(controls, status) for steps, the last route this router gave (routing/controls.py), from the source that routed it:
    local tiles are checked for their controls (the controls_built marker, the sim's sidecar), a corridor from real OSM
    carries them; the OSM stop-sign sidecar from the same place (module docstring). ([], 'failed') for steps it didn't
    route."""
    served = self.served
    if served is None or served[0] is not steps:
      cloudlog.error("turnpilot: controls asked for a route this router didn't give")
      return [], 'failed'
    return served[1].controls(steps, tiles_dir=served[2], sidecar_dir=served[3], context=self.junction_context())

  def opposing(self, steps) -> dict:
    """{step index: (m/s, lanes)} of the roads straight across the route's maneuvers (routing/controls.py: opposing_roads),
    from the source that routed it; {} (unknown) for steps it didn't route."""
    served = self.served
    if served is None or served[0] is not steps:
      cloudlog.error("turnpilot: opposing roads asked for a route this router didn't give")
      return {}
    return served[1].opposing(steps)

  def controls_ahead(self, position, bearing) -> tuple[list, str, list]:
    """(controls, status, road) on the road ahead of the car off the route (routing/controls.py: controls_ahead), from the source
    that routed the last route, or the local tiles (loaded here if no route has yet: TurnPilotValhallaTiles). position:
    navd's (longitude, latitude); bearing: deg. ([], 'failed') with neither (a corridor still downloading): nothing is
    known about the map here, said so."""
    tiles = local_override(self.params)
    if self.served is None and tiles and (self.local is None or self.local[0] != tiles):
      try:
        self.local = (tiles, self.valhalla(tiles))
      except Exception:
        cloudlog.exception(f"turnpilot: TurnPilotValhallaTiles ({tiles}) doesn't load for the controls ahead")
    source = self.served[1:] if self.served is not None else self.local_served(self.local[1]) if self.local is not None else None
    if source is None:
      cloudlog.warning("turnpilot: controls ahead asked for with no map source yet")
      return [], 'failed', []
    return source[0].controls_ahead((position[1], position[0]), bearing, tiles_dir=source[1], sidecar_dir=source[2],
                                    context=self.junction_context())

  @staticmethod
  def local_served(source) -> tuple:
    """(source, tiles_dir, sidecar_dir) for the local tiles: their own directory holds the sidecar; a valhalla_service
    URL has none (NO_DIRECTORY, which controls.py says out loud)."""
    tiles_dir = getattr(source, 'tiles_dir', None)
    return source, tiles_dir, tiles_dir if tiles_dir else NO_DIRECTORY

  def own_edge(self, position, bearing):
    """(way id, forward, end node) of the edge the car is on, from the source that routed the last route (routing/controls.py:
    own_edge); None when nothing is routed or it can't say. position: navd's (longitude, latitude)."""
    served = self.served
    if served is None or not hasattr(served[1], 'own_edge'):
      return None
    try:
      return served[1].own_edge((position[1], position[0]), bearing)
    except Exception:
      cloudlog.exception("turnpilot: the car's own edge failed")
      return None

  def reroute(self, start, destination, bearing, language, exclude) -> list[dict] | None:
    """The steps from start to destination around exclude (valhalla_source: exclude_locations), for the stop-sign
    avoidance loop (navd; STOP_SIGNS.md 3.3), or None for a failed iteration. Status-neutral: it never writes
    TurnPilotMapStatus and never downloads. It routes on the local tiles (TurnPilotValhallaTiles), or on this destination's
    corridor while the car is on it; a route that leaves the corridor, no route, or any routing error is None and leaves
    the served route as it was. A route it gives becomes the served one, so controls() and opposing() answer for it."""
    try:
      tiles = local_override(self.params)
      if tiles:
        if self.local is None or self.local[0] != tiles:
          cloudlog.warning("turnpilot reroute: the local tiles aren't loaded")
          return None
        source, extra = self.local[1], self.local_served(self.local[1])[1:]
      else:
        current = self.corridor
        if current is None or current.destination != destination or not current.pack.path.is_dir() or not current.pack.covers(*start):
          cloudlog.warning("turnpilot reroute: no corridor for this destination under the car")
          return None
        source, extra = current.source, (None, current.pack.path)
      steps = source.route(start, destination, bearing, language, exclude=exclude)
      if not steps:
        cloudlog.warning("turnpilot reroute: no route around the exclusions")
        return None
      if not tiles and not self.corridor.pack.holds(steps):
        cloudlog.warning("turnpilot reroute: the route around the exclusions leaves the corridor")
        return None
    except Exception as e:
      cloudlog.warning(f"turnpilot reroute failed: {type(e).__name__}: {e}")
      return None
    self.served = (steps, source, *extra)
    return steps

  def clear(self):
    """No destination."""
    self.failure = None
    self.status.set('idle')

  def route(self, start, destination, bearing, language, position=None, cancelled=None) -> list[dict]:
    """The steps from start to destination. position() gives the car's (start, bearing) now: after a download, which
    can take minutes, the route starts there. cancelled() true stops a download with a MapError 'cancelled', which
    leaves the status and the retry wait alone: the destination changed, and that is the new status."""
    try:
      check_route_source(self.params)
      tiles = local_override(self.params)
      if tiles:
        return self.route_local(tiles, start, destination, bearing, language)
      return self.route_server(start, destination, bearing, language, position, cancelled)
    except MapError as e:
      if e.code == CANCELLED or (cancelled is not None and cancelled()):
        # navd has moved on to another destination and doesn't wait for this job, which can finish after the next one
        # started: whatever happened to it is not the status, nor a failure the new destination waits out
        cloudlog.warning(f"turnpilot: the route to {destination} was cancelled ({e.code})")
        raise
      cloudlog.error(f"turnpilot route refused: {e.code}: {e.message}")
      self.status.error(e, destination=list(destination))
      raise

  def route_local(self, tiles, start, destination, bearing, language):
    if self.local is None or self.local[0] != tiles:
      try:
        self.local = (tiles, self.valhalla(tiles))
      except Exception as e:
        raise MapError('local_tiles', f"TurnPilotValhallaTiles ({tiles}) doesn't load: {e}", 'Local tiles do not load') from None
    steps = self.routed(self.local[1], start, destination, bearing, language, 'the local tiles')
    if not steps:
      raise MapError('no_route', f"Valhalla found no route on the local tiles ({tiles}).", 'No route found')
    self.served = (steps, *self.local_served(self.local[1]))
    self.status.set('ready', 'Routing on local tiles (developer override, no geofence).', 'Local tiles', source='local',
                    destination=list(destination))
    return steps

  def coverage_or_fetch(self):
    """The cached coverage. With none cached yet, one fetch now, since navd may be the first to run online."""
    coverage = self.coverage.get()
    if coverage is None:
      try:
        return refresh(self.params, self.directory)
      except MapError as e:
        raise no_coverage(f"Fetching it now failed: {e.message}") from None
    return coverage

  def route_server(self, start, destination, bearing, language, position=None, cancelled=None):
    current = self.corridor
    if current is not None and current.destination == destination and current.pack.path.is_dir():
      # a reroute on the corridor, which is wider than the route, while the car is on its local roads
      if current.pack.covers(*start):
        steps = self.routed(current.source, start, destination, bearing, language, 'the map for this drive')
        if not steps:
          raise MapError('no_route', "No route on the map for this drive.", 'No route found')
        if current.pack.holds(steps):
          self.ready(current.region, destination)
          self.served = (steps, current.source, None, current.pack.path)
          return steps
        cloudlog.warning("turnpilot: the reroute leaves the corridor, downloading a new corridor")
      else:
        cloudlog.warning(f"turnpilot: the car at {start} left the corridor, downloading a new corridor")
    elif (self.status.last or {}).get('destination') != list(destination):  # a retry keeps showing its error meanwhile
      self.status.set('checking', 'Checking the mapped area', destination=list(destination))

    coverage = self.coverage_or_fetch()
    region = check_trip(coverage, start, destination)
    downloaded = self.download(start, destination, region, coverage.names(), cancelled)
    if position is not None:  # the download took a while: route from where the car is now
      start, bearing = position()
    try:
      steps = self.routed(downloaded.source, start, destination, bearing, language, 'the downloaded map')
      if not steps:
        raise MapError('no_route', "No route on the downloaded map for this drive.", 'No route found')
      if not downloaded.pack.holds(steps):
        raise MapError('route_off_map', "The route for this drive leaves the map downloaded for it, so TurnPilot won't follow it.",
                       'Route leaves the downloaded map')
    except MapError as e:
      # the corridor is kept, and the same destination waits out the retry time before it is downloaded again
      if not (cancelled is not None and cancelled()):
        self.failure = Failure(destination, self.config(), e, time.monotonic())
      raise
    if cancelled is not None and cancelled():
      raise cancelled_error()
    self.ready(region, destination)
    self.served = (steps, downloaded.source, None, downloaded.pack.path)
    return steps

  def ready(self, region: Region, destination):
    self.status.set('ready', f"Ready: map for this drive in the {region.name}", 'Ready', source='server', region=region.name,
                    destination=list(destination))

  def routed(self, source, start, destination, bearing, language, what):
    try:
      return source.route(start, destination, bearing, language)
    except Exception as e:
      raise MapError('routing_failed', f"Routing on {what} failed: {e}", 'Routing failed') from None

  def download(self, start, destination, region: Region, names: str, cancelled=None) -> Corridor:
    """The corridor for the drive, which becomes self.corridor unless the job was cancelled meanwhile."""
    config = self.config()
    failure = self.failure
    if failure is not None and failure.destination == destination and failure.config == config:
      wait = RETRY_SECONDS if failure.error.retry else REFUSED_RETRY_SECONDS
      if time.monotonic() - failure.at < wait:
        raise failure.error
    def was_cancelled() -> bool:
      return cancelled is not None and bool(cancelled())

    try:
      downloaded = self.fetch(start, destination, region, names, cancelled)
      if was_cancelled():
        raise cancelled_error()
      self.corridor = downloaded
      self.failure = None
      return downloaded
    except MapError as e:
      if e.code == CANCELLED or was_cancelled():
        raise
      self.failure = Failure(destination, config, e, time.monotonic())
      raise
    except OSError as e:  # the comma's own storage (making the download's directory, say): not a network blip
      if was_cancelled():
        raise cancelled_error() from None
      error = storage_error(e)
      self.failure = Failure(destination, config, error, time.monotonic())
      raise error from None
    except Exception as e:  # anything else is still a failure the driver sees, and waits out the retry time
      if was_cancelled():
        raise cancelled_error() from None
      cloudlog.exception("turnpilot: corridor download failed")
      error = MapError('download_failed', f"The map download failed: {type(e).__name__}: {e}", 'Map download failed', retry=True)
      self.failure = Failure(destination, config, error, time.monotonic())
      raise error from None

  def fetch(self, start, destination, region: Region, names: str, cancelled=None) -> Corridor:
    client = self.client_factory(self.params, self.directory)
    packs_dir = self.packs_dir
    packs_dir.mkdir(parents=True, exist_ok=True)
    download = packs_dir / f".download-{uuid.uuid4().hex[:8]}.tar"
    base = {'destination': list(destination), 'region': region.name, 'source': 'server'}

    def progress(done, total):
      self.status.set('downloading', 'Downloading map for this drive…', 'Downloading map', progress=round(done / total, 3) if total else None,
                      bytes=done, total=total, **base)

    report = Throttle(progress)

    def received(done, total):
      if cancelled is not None and cancelled():
        raise cancelled_error()
      report(done, total)

    received(0, None)  # shows the download at once, and checks for a cancel before connecting
    try:
      size = client.corridor(start, destination, download, received, names)
      if cancelled is not None and cancelled():
        raise cancelled_error()
      report(size, size, final=True)
      pack = corridor.unpack(download, packs_dir)
    finally:
      download.unlink(missing_ok=True)
    if cancelled is not None and cancelled():
      raise cancelled_error()  # the pack stays until a later prune
    if pack.manifest['region'] != region.id:
      cloudlog.warning(f"turnpilot: corridor is for region {pack.manifest['region']}, the geofence says {region.id}")
    try:
      source = self.valhalla(str(pack.path))
    except Exception as e:
      raise MapError('routing_failed', f"The downloaded map doesn't load in Valhalla: {e}", 'Downloaded map does not load') from None
    corridor.prune(packs_dir, in_use=pack.path)
    cloudlog.info(f"turnpilot: corridor {pack.path.name} for {destination}, {size / 1e6:.1f} MB, {pack.manifest.get('route_km')} km")
    return Corridor(destination, pack, source, region)
