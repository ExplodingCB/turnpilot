"""The params the phone app reads and writes: the pairing token, the destination, favorites and recents."""
import math
import secrets
import threading
import uuid

from openpilot.cereal import log

RECENTS_MAX = 10
SAME_PLACE = 2e-5  # degrees, about 2 m


def valid_latlon(latitude, longitude) -> bool:
  nums = (latitude, longitude)
  if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in nums):
    return False
  return -90 <= latitude <= 90 and -180 <= longitude <= 180


def coords_text(latitude, longitude) -> str:
  return f"{latitude:.5f}, {longitude:.5f}"


def same_place(a, b) -> bool:
  return abs(a["latitude"] - b["latitude"]) < SAME_PLACE and abs(a["longitude"] - b["longitude"]) < SAME_PLACE


def ensure_token(params) -> str:
  token = params.get("TurnPilotWebToken")
  if not token:
    token = secrets.token_urlsafe(16)
    params.put("TurnPilotWebToken", token, block=True)
  return token


def last_gps_position(params):
  """(latitude, longitude) of the last GPS fix ubloxd or qcomgpsd saved (both as a gpsLocationExternal event), or None."""
  raw = params.get("LastGPSPosition")
  if not raw:
    return None
  try:
    with log.Event.from_bytes(raw) as evt:
      gps = evt.gpsLocationExternal
      return (gps.latitude, gps.longitude) if valid_latlon(gps.latitude, gps.longitude) else None
  except Exception:
    return None


class Store:
  """All access to Params goes through one lock, since every request runs in its own thread."""
  def __init__(self, params):
    self.params = params
    self.lock = threading.RLock()

  def token(self) -> str:
    with self.lock:
      return self.params.get("TurnPilotWebToken") or ""

  def last_gps(self):
    with self.lock:
      return last_gps_position(self.params)

  def flag(self, key) -> bool:
    with self.lock:
      return self.params.get_bool(key)

  def _load(self) -> dict:
    """Both lists, keeping only entries with numeric coordinates: the param can be edited by hand or left by an older version."""
    data = self.params.get("TurnPilotFavorites")
    data = data if isinstance(data, dict) else {}
    loaded = {}
    for key in ("favorites", "recents"):
      items = data.get(key)
      items = items if isinstance(items, list) else []
      usable = [p for p in items if isinstance(p, dict) and valid_latlon(p.get("latitude"), p.get("longitude"))]
      loaded[key] = [{**p, "name": str(p.get("name") or "") or coords_text(p["latitude"], p["longitude"])} for p in usable]
    return loaded

  def _save(self, data):
    self.params.put("TurnPilotFavorites", data, block=True)

  def places(self) -> dict:
    with self.lock:
      return self._load()

  def navigate(self, latitude, longitude, name=""):
    """Set the destination. The name goes into recents, since NavDestination itself only holds coordinates."""
    if not valid_latlon(latitude, longitude):
      raise ValueError("invalid coordinates")
    place = {"name": str(name or "").strip()[:120] or coords_text(latitude, longitude),
             "latitude": float(latitude), "longitude": float(longitude)}
    with self.lock:
      self.params.put("NavDestination", {"latitude": place["latitude"], "longitude": place["longitude"]}, block=True)
      data = self._load()
      data["recents"] = ([place] + [p for p in data["recents"] if not same_place(p, place)])[:RECENTS_MAX]
      self._save(data)
    return place

  def stop(self):
    with self.lock:
      self.params.remove("NavDestination")

  def destination(self):
    """The current destination with a name, or None."""
    with self.lock:
      dest = self.params.get("NavDestination")
      if not isinstance(dest, dict) or not valid_latlon(dest.get("latitude"), dest.get("longitude")):
        return None
      data = self._load()
    named = next((p for p in data["favorites"] + data["recents"] if same_place(p, dest)), None)
    return {"name": named["name"] if named else coords_text(dest["latitude"], dest["longitude"]),
            "latitude": dest["latitude"], "longitude": dest["longitude"]}

  def add_favorite(self, latitude, longitude, name=""):
    if not valid_latlon(latitude, longitude):
      raise ValueError("invalid coordinates")
    place = {"name": str(name or "").strip()[:120] or coords_text(latitude, longitude),
             "latitude": float(latitude), "longitude": float(longitude)}
    with self.lock:
      data = self._load()
      old = next((p for p in data["favorites"] if same_place(p, place)), None)
      place["id"] = old.get("id") if old and old.get("id") else uuid.uuid4().hex[:8]
      data["favorites"] = [p for p in data["favorites"] if p is not old] + [place]
      self._save(data)
    return place

  def remove_favorite(self, fav_id) -> bool:
    with self.lock:
      data = self._load()
      kept = [p for p in data["favorites"] if p.get("id") != fav_id]
      if len(kept) == len(data["favorites"]):
        return False
      data["favorites"] = kept
      self._save(data)
    return True
