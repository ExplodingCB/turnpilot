"""The surveyed junctions (surveyed_junctions.json beside this file): what Kellen has seen at an all-way stop, which is the
only evidence that counts an approach as stopping this week (STOP_SIGNS.md 4.1, 4.3, decision D10).

The file is a JSON list of entries, written by hand at step S2 from `route_audit --survey-template` (which fills what the
map knows and leaves null what must be measured). The shipped file is an empty list. One entry:

  {"osmNodeId": OSM junction node, "lat", "lon",
   "date": "YYYY-MM-DD" of the survey, "source": "inPerson" | "streetImagery",
   "imageryDate": "YYYY-MM" or "YYYY-MM-DD" (streetImagery only: the imagery's own date),
   "confirmedBy": ["inPerson", "streetImagery"] (the independent looks; an implausible all-way needs both, 4.2),
   "approaches": [{"wayId", "bearingDeg" (the approach's heading toward the junction, deg from north),
                   "stops", "plaque" (a stop sign and the ALL WAY plaque seen), "postedMph" (the sign, null if none seen),
                   "laneEdgeM" (node to the cross road's near travel-lane edge, m), "lineM" (node to the painted line, 0 none),
                   "nearestParkingM": {"left", "right"} (m along each curb to the first legal space), "sightM" (m),
                   "stopPointM" (optional: node to where a car on this approach stops)}, ...],
   "validations": [{"lighting", "date", "passes", "approachBearingDeg" (the heading toward the junction of the approach it
                    was measured from: always written, a stamp without it validates no approach), "cars": {"left", "right",
                    "opposite"}, "minFirstTrackM", "maxDropoutS", "allStopped", "verdict"}, ...] (written by
                    junction_validate.py, never by hand; it writes no "opposite" side until step S4 bounds its ground
                    range, junction_validate.STAMP_SIDES)}

An example, for a made-up junction (its node id can't match anything, and none of its numbers may be copied):

  {"osmNodeId": 0, "lat": 39.8000, "lon": -86.1500, "date": "2026-10-29", "source": "streetImagery",
   "imageryDate": "2026-06", "confirmedBy": [],
   "approaches": [
    {"wayId": 0, "bearingDeg": 90, "stops": true, "plaque": true, "postedMph": 20, "laneEdgeM": 4.0, "lineM": 7.0,
     "nearestParkingM": {"left": 36.0, "right": 40.0}, "sightM": 60, "stopPointM": 9.8},
    {"wayId": 0, "bearingDeg": 180, "stops": true, "plaque": true, "postedMph": 20, "laneEdgeM": 4.0, "lineM": 7.5,
     "nearestParkingM": {"left": 0.0, "right": 38.0}, "sightM": 55}, ...],
   "validations": []}

load() checks each entry against the schema. An entry with a field missing or null (postedMph and stopPointM may be null)
is left out, and a stopPointM closer than STOP_CLEAR_M + POS_ERR_M + SURVEY_ERR_M outside its approach's laneEdgeM is
dropped (the approach stops at the default point); each is logged as an error and listed in errors, never silently
ignored. counts(): an entry dated over SURVEY_MAX_AGE_DAYS ago, or from imagery over that old (a month-precision imagery
date is taken as the month's first day), doesn't count. match(): by OSM node id; a junction whose id isn't known (no
sidecar record) matches only an entry whose position is on its node (within SAME_NODE_M, the precision OSM and Valhalla
share for a node's position), never by a neighbourhood radius: a 20 m radius could attach an all-way's survey to a
staggered or frontage neighbour 10-30 m away.
"""
import json
import math
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

from openpilot.common.swaglog import cloudlog

SHIPPED = Path(__file__).with_name('surveyed_junctions.json')
SURVEY_MAX_AGE_DAYS = 365
POS_ERR_M = 2.0      # m: the car's along-track position error (STOP_SIGNS.md 1.2)
SURVEY_ERR_M = 0.5   # m: a surveyed distance's error
STOP_CLEAR_M = 3.3   # m a stop point keeps outside the closest plausible edge after POS_ERR_M (the G5 nudge's reach, 2.25 m)
SAME_NODE_M = 1.5    # m: a position this close to a junction node is on it
SOURCES = ('inPerson', 'streetImagery')
EARTH_R = 6371000.


@dataclass
class Surveys:
  entries: list[dict] = field(default_factory=list)
  errors: list[str] = field(default_factory=list)


def parse_date(text) -> date | None:
  """A 'YYYY-MM-DD' or 'YYYY-MM' (its first day) date, or None."""
  if not isinstance(text, str):
    return None
  parts = text.split('-')
  try:
    if len(parts) == 3:
      return date(int(parts[0]), int(parts[1]), int(parts[2]))
    if len(parts) == 2:
      return date(int(parts[0]), int(parts[1]), 1)
  except ValueError:
    return None
  return None


def _number(x) -> bool:
  return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _approach_problem(a) -> str:
  if not isinstance(a, dict):
    return "an approach isn't an object"
  for k in ('wayId',):
    if not isinstance(a.get(k), int) or isinstance(a.get(k), bool) or a[k] < 0:
      return f"approach {k} must be a whole number"
  for k in ('bearingDeg', 'laneEdgeM', 'lineM', 'sightM'):
    if not _number(a.get(k)):
      return f"approach {k} must be a number"
  for k in ('stops', 'plaque'):
    if not isinstance(a.get(k), bool):
      return f"approach {k} must be true or false"
  if a.get('postedMph') is not None and not _number(a['postedMph']):
    return "approach postedMph must be a number or null"
  parking = a.get('nearestParkingM')
  if not isinstance(parking, dict) or not all(_number(parking.get(k)) for k in ('left', 'right')):
    return "approach nearestParkingM must be {left, right} numbers"
  if a.get('stopPointM') is not None and not _number(a['stopPointM']):
    return "approach stopPointM must be a number or absent"
  return ''


def problem(entry) -> str:
  """Why entry isn't a survey, or ''."""
  if not isinstance(entry, dict):
    return "not an object"
  if not isinstance(entry.get('osmNodeId'), int) or isinstance(entry.get('osmNodeId'), bool) or entry['osmNodeId'] < 0:
    return "osmNodeId must be a whole number"
  if not (_number(entry.get('lat')) and _number(entry.get('lon'))):
    return "lat and lon must be numbers"
  if parse_date(entry.get('date')) is None or len(entry['date'].split('-')) != 3:
    return "date must be YYYY-MM-DD"
  if entry.get('source') not in SOURCES:
    return f"source must be one of {', '.join(SOURCES)}"
  if entry['source'] == 'streetImagery' and parse_date(entry.get('imageryDate')) is None:
    return "imageryDate (YYYY-MM or YYYY-MM-DD) is required for streetImagery"
  if not isinstance(entry.get('confirmedBy'), list) or not all(x in SOURCES for x in entry['confirmedBy']):
    return f"confirmedBy must be a list of {', '.join(SOURCES)}"
  if not isinstance(entry.get('approaches'), list) or not entry['approaches']:
    return "approaches must be a non-empty list"
  for a in entry['approaches']:
    if why := _approach_problem(a):
      return why
  if not isinstance(entry.get('validations'), list) or not all(isinstance(v, dict) for v in entry['validations']):
    return "validations must be a list of objects"
  return ''


def load(path: Path = SHIPPED) -> Surveys:
  """The entries of the file at path that are surveys (module docstring), and what was wrong with the rest."""
  out = Surveys()
  try:
    raw = json.loads(Path(path).read_text())
  except (OSError, ValueError) as e:
    out.errors.append(f"{path} doesn't read: {type(e).__name__}: {e}")
    cloudlog.error(f"turnpilot surveys: {out.errors[-1]}")
    return out
  if not isinstance(raw, list):
    out.errors.append(f"{path} isn't a JSON list")
    cloudlog.error(f"turnpilot surveys: {out.errors[-1]}")
    return out
  for i, entry in enumerate(raw):
    if why := problem(entry):
      out.errors.append(f"entry {i} (osmNodeId {entry.get('osmNodeId') if isinstance(entry, dict) else '?'}) left out: {why}")
      cloudlog.error(f"turnpilot surveys: {out.errors[-1]}")
      continue
    entry = dict(entry, approaches=[dict(a) for a in entry['approaches']])
    for k, a in enumerate(entry['approaches']):
      a.setdefault('stopPointM', None)
      need = a['laneEdgeM'] + STOP_CLEAR_M + POS_ERR_M + SURVEY_ERR_M
      if a['stopPointM'] is not None and a['stopPointM'] < need - 1e-6:
        out.errors.append(f"entry {i} (osmNodeId {entry['osmNodeId']}) approach {k}: stopPointM {a['stopPointM']} m is inside " +
                          f"laneEdgeM {a['laneEdgeM']} + {STOP_CLEAR_M + POS_ERR_M + SURVEY_ERR_M:.1f} m ({need:.1f} m): dropped, " +
                          "the default stop point applies")
        cloudlog.error(f"turnpilot surveys: {out.errors[-1]}")
        a['stopPointM'] = None
    out.entries.append(entry)
  return out


_cache: dict[Path, tuple[tuple, Surveys]] = {}


def cached(path: Path = SHIPPED) -> Surveys:
  """load(path), read again only when the file changes (navd asks every second, off the route): its errors are logged
  once per change of the file."""
  path = Path(path)
  try:
    st = path.stat()
    stamp = (st.st_mtime_ns, st.st_size)
  except OSError:
    stamp = None
  hit = _cache.get(path)
  if hit is not None and hit[0] == stamp and stamp is not None:
    return hit[1]
  out = load(path)
  _cache[path] = (stamp, out)
  return out


def counts(entry: dict, today: date) -> tuple[bool, str]:
  """(whether the entry counts today, why not)."""
  surveyed = parse_date(entry.get('date'))
  if surveyed is None or (today - surveyed).days > SURVEY_MAX_AGE_DAYS:
    return False, f"surveyed {entry.get('date')}, over a year ago"
  if entry.get('source') == 'streetImagery':
    imagery = parse_date(entry.get('imageryDate'))
    if imagery is None or (today - imagery).days > SURVEY_MAX_AGE_DAYS:
      return False, f"imagery from {entry.get('imageryDate')}, over a year old"
  return True, ''


def _haversine(a, b) -> float:
  x = math.radians(b[1] - a[1]) * math.cos(math.radians((a[0] + b[0]) / 2))
  return EARTH_R * math.hypot(x, math.radians(b[0] - a[0]))


def match(entries: list[dict], osm_node_id: int, pos: tuple[float, float]) -> dict | None:
  """The entry for the junction with OSM node id osm_node_id (0: not known) at pos (lat, lon), the newest if several, or
  None (module docstring)."""
  if osm_node_id:
    hits = [e for e in entries if e['osmNodeId'] == osm_node_id]
  else:
    hits = [e for e in entries if _haversine((e['lat'], e['lon']), pos) <= SAME_NODE_M]
  return max(hits, key=lambda e: e['date']) if hits else None
