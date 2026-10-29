"""Valhalla as a route source: pyvalhalla in this process when it imports, else a local valhalla_service."""
import importlib
import math
import os
import sys
import threading
from pathlib import Path

import requests

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.routing.controls import (LocateCache, Opposing, actor_call, controls_ahead, http_call, opposing_roads,
                                                            route_controls)

# tools/turnpilot/install_valhalla_device.sh puts pyvalhalla here, since the system venv is read-only
DEVICE_PYVALHALLA = '/data/turnpilot/pyvalhalla'
DEFAULT_URL = 'http://127.0.0.1:8002'  # valhalla_service's default port
HEADING_TOLERANCE = 90  # degrees either side of the bearing, the same range navd gives Mapbox


def locate(tiles) -> dict | None:
  """What a TurnPilotValhallaTiles value points at, or None if there is nothing usable there:
  {'url': base} for a running valhalla_service, {'config': path} for a Valhalla config file, or
  {'extract': path, 'dir': path} for a tile extract (.tar) and/or tile directory, given directly or as the
  directory tools/turnpilot/build_valhalla_tiles.sh wrote (tiles.tar, tiles/)."""
  if not tiles:
    return None
  if tiles.startswith(('http://', 'https://')):
    return {'url': tiles.rstrip('/')}
  path = Path(tiles).expanduser()
  if path.is_file():
    if path.suffix == '.json':
      return {'config': str(path)}
    return {'extract': str(path), 'dir': ''} if path.suffix == '.tar' else None
  if not path.is_dir():
    return None
  extract, tile_dir = path / 'tiles.tar', path / 'tiles'
  if extract.is_file():
    return {'extract': str(extract), 'dir': str(tile_dir) if tile_dir.is_dir() else ''}
  if tile_dir.is_dir():
    return {'extract': '', 'dir': str(tile_dir)}
  if (path / 'valhalla.json').is_file():
    return {'config': str(path / 'valhalla.json')}
  if any(child.is_dir() and child.name.isdigit() for child in path.iterdir()):  # a tile hierarchy: 0/, 1/, 2/
    return {'extract': '', 'dir': str(path)}
  return None


def tiles_usable(tiles) -> bool:
  return locate(tiles) is not None


def import_valhalla():
  """The valhalla module, or None. Falls back to the device install, since the system venv is read-only."""
  try:
    return importlib.import_module('valhalla')
  except ImportError:
    pass
  path = os.environ.get('TURNPILOT_PYVALHALLA', DEVICE_PYVALHALLA)
  if not os.path.isdir(path):
    return None
  if path not in sys.path:
    sys.path.append(path)
  importlib.invalidate_caches()
  try:
    return importlib.import_module('valhalla')
  except ImportError:
    cloudlog.exception(f"pyvalhalla at {path} does not import")
    return None


def tiles_dir_of(tiles) -> str | None:
  """The directory a TurnPilotValhallaTiles value's tiles live in (for routing/controls.py's marker and sidecar), or
  None for a service URL."""
  if not tiles or tiles.startswith(('http://', 'https://')):
    return None
  path = Path(tiles).expanduser()
  return str(path if path.is_dir() else path.parent)


class ValhallaSource:
  def __init__(self, tiles):
    where = locate(tiles)
    if where is None:
      raise FileNotFoundError(f"no Valhalla config or tiles at {tiles!r}")
    self.tiles_dir = tiles_dir_of(tiles)
    self.url = where.get('url', '')
    self.actor = None
    # navd's route job and its off-route lookup run in threads of their own: one call into the actor at a time
    self.lock = threading.Lock()
    self.cache = LocateCache()  # Q1 and Q2 for the lookups ahead, by edge id: this source's tiles only
    self.opposing_of: tuple | None = None  # (steps, their opposing roads) from the last controls() that read them
    if self.url:
      return
    valhalla = import_valhalla()
    if valhalla is None:
      self.url = os.environ.get('TURNPILOT_VALHALLA_URL', DEFAULT_URL)
      cloudlog.warning(f"pyvalhalla is not installed: routing through valhalla_service at {self.url}")
    elif 'config' in where:
      self.actor = valhalla.Actor(where['config'])
    else:
      self.actor = valhalla.Actor(valhalla.get_config(tile_extract=where['extract'], tile_dir=where['dir']))

  def request(self, start, destination, bearing, language) -> dict:
    """Valhalla's /route request for what navd gives Mapbox, with the bearing as the start's heading."""
    origin = {'lon': start[0], 'lat': start[1]}
    if bearing is not None and math.isfinite(bearing):  # a GPS fix without a heading can report NaN
      origin.update(heading=round(bearing) % 360, heading_tolerance=HEADING_TOLERANCE)
    request = {
      'locations': [origin, {'lon': destination[0], 'lat': destination[1]}],
      'costing': 'auto',
      'format': 'osrm',
      'shape_format': 'geojson',  # navd reads step geometry as geojson; the default polyline6 breaks it
      'banner_instructions': True,
      'voice_instructions': True,
      'units': 'miles',
    }
    if language is not None:
      request['language'] = language
    return request

  def route(self, start, destination, bearing, language):
    request = self.request(start, destination, bearing, language)
    if self.actor is not None:
      with self.lock:
        r = self.actor.route(request)
    else:
      resp = requests.post(self.url + '/route', json=request, timeout=10)
      resp.raise_for_status()
      r = resp.json()
    return r['routes'][0]['legs'][0]['steps'] if len(r.get('routes', [])) else []

  def controls(self, steps, tiles_dir=None, *, sidecar_dir, context=None):
    """(controls, status) along the route steps (routing/controls.py). tiles_dir: the tiles' directory when their
    controls must be checked for (local tiles: the controls_built marker and the sim's sidecar), None when the tiles come
    from real OSM. sidecar_dir: where the OSM stop-sign sidecar and the tiles' stamp are (routing/osm_sidecar.py), always
    given (sources.py), so no route goes without it unsaid. context: the surveys and overrides the junctions are read
    against (routing/junctions.py). The opposing roads are read in the same batched locate, for opposing()."""
    out = Opposing()
    result = route_controls(self._call(), steps, tiles_dir, sidecar_dir, opposing=out, context=context)
    self.opposing_of = (steps, out.roads) if out.roads is not None else None
    return result

  def opposing(self, steps):
    """{step index: (m/s, lanes)} of the roads straight across the route's maneuvers (routing/controls.py: opposing_roads):
    the last controls()' for these very steps, else read alone (it didn't run its batched locate, or other steps)."""
    done = self.opposing_of
    if done is not None and done[0] is steps:
      return dict(done[1])
    return opposing_roads(self._call(), steps)

  def controls_ahead(self, position, heading, tiles_dir=None, *, sidecar_dir, context=None):
    """(controls, status, road) ahead of the car at position (lat, lon) heading that way (routing/controls.py:
    controls_ahead), each with its stop point. tiles_dir, sidecar_dir and context: as for controls(); Q1 and Q2 kept in
    this source's cache between lookups."""
    return controls_ahead(self._call(), position, heading, tiles_dir, sidecar_dir, context=context, cache=self.cache)

  def _call(self):
    if self.actor is None:
      return http_call(self.url, requests.post)
    call = actor_call(self.actor)
    def locked(endpoint, req):
      with self.lock:
        return call(endpoint, req)
    return locked
