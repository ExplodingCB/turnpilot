#!/usr/bin/env python3
"""The visibility file, maps/visibility.json (STOP_SIGNS.md 1.5, 1.6): what the camera sees, read by navd's certifier
(routing/certify.py) and turnpilotd's visibility runtime alike, so the route-time and drive-time verdicts can't drift.

  python -m openpilot.selfdrive.turnpilot.visibility_table --check FILE

checks a file against the schema (step S1's fit and step S4a's launch_verify.py run it on what they write): exit 1, naming
the field, when one is missing or bad; it also says which measurements are still empty.

Every key below must be present; a missing one is a loud VisibilityError naming it. The camera's measurements may be null
until step S1 fills them, and then certify() returns noTable (camera_ready()):
  cameraSetbackM, lateralOffsetM, heightM   m, from step S1
  lens          {model: equidistant | equisolid, f (px), k, edgeDeg: {left, right}}: the frame edge per side in calibrated bearing
  mask          {left, right}: [[bearingDeg, elevLoDeg, elevHiDeg], ...] per 1 deg of calibrated bearing, the elevation band a
                car-sized footprint may occupy unobstructed; a bearing not listed is masked
  darkThreshold median luma, DN
  veil          unmeasured | none | withCover | present (null: not ready)
  tables        [{source: provisional | measured | sim, bound: conservative | optimistic (default conservative),
                  view: road | full | side, lighting: day | sunAhead | overcast | night, bins: [[bearingLoDeg, metres], ...]
                  (ascending, calibrated bearing; a bin bounds a point's range), cars, date}]
  allowProvisionalFor  allWay | null: where a provisional table may be read (route_audit reads them anyway, as candidates)
and these may be absent or null:
  actuatorDelayS  s, step S4a's measured longitudinal delay (stop_rules.t_abort takes the larger of it and CarParams')
  launchVerified  {date, goes, takenOver, maxTakeoverS, verdict}, written by launch_verify.py (S4a), never by hand

The shipped file is PV1 (1.6) as provisional tables, the camera fields empty and launchVerified absent, so certify() says
noTable for every stop and yield and navd routes around all of them until steps S1 and S4a.

A sim table is read only in the sim: SIMULATION=1, the sim's CLEAR_ON_MANAGER_START param (TurnPilotSimLightOracle, which
the sim launch sets and a manager start clears) and a sim car in CarParams (a fingerprint the sim forced, or the mock car),
the way turnpilotd.oracle_accepted gates the light oracle. Anywhere else it is refused, logged as an error and listed in
refused for the caller to say. There is no TurnPilotSim param.
"""
import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot import stop_rules as sr

SHIPPED = Path(__file__).parent / 'maps' / 'visibility.json'
REQUIRED = ('cameraSetbackM', 'lateralOffsetM', 'heightM', 'lens', 'mask', 'darkThreshold', 'veil', 'tables', 'allowProvisionalFor')
CAMERA = ('cameraSetbackM', 'lateralOffsetM', 'heightM', 'lens', 'mask', 'darkThreshold', 'veil')
OPTIONAL = ('actuatorDelayS', 'launchVerified')
LENS_MODELS = ('equidistant', 'equisolid')
VEILS = ('unmeasured', 'none', 'withCover', 'present')
SOURCES = ('provisional', 'measured', 'sim')
BOUNDS = ('conservative', 'optimistic')
VIEWS = ('road', 'full', 'side')
LIGHTINGS = ('day', 'sunAhead', 'overcast', 'night')
LAUNCH_FIELDS = ('date', 'goes', 'takenOver', 'maxTakeoverS', 'verdict')
PROVISIONAL_USES = ('allWay',)
DEFAULT_EDGE_DEG = 85.2   # the frame edge at lens k = 0, the bracket's pessimistic end until S1 fits the lens (1.1)
SIM_PARAM = 'TurnPilotSimLightOracle'
TABLE_RANK = {'measured': 0, 'sim': 1, 'provisional': 2}


class VisibilityError(ValueError):
  """The file is missing a field or has a bad one: the message names it."""


@dataclass
class Table:
  source: str
  bound: str
  view: str
  lighting: str
  bins: list
  cars: int
  date: str


@dataclass
class Visibility:
  raw: dict
  tables: list[Table] = field(default_factory=list)
  refused: list[str] = field(default_factory=list)  # tables not read here (sim tables off the sim), said by the caller

  def camera_ready(self) -> tuple[bool, list[str]]:
    """(the camera fields, lens, mask, darkThreshold and veil are all filled, the empty ones)."""
    missing = [k for k in CAMERA if self.raw.get(k) is None]
    return not missing, missing

  @property
  def setback(self) -> float | None:
    return self.raw.get('cameraSetbackM')

  @property
  def height(self) -> float | None:
    return self.raw.get('heightM')

  @property
  def veil(self) -> str | None:
    return self.raw.get('veil')

  @property
  def mask(self) -> dict | None:
    return self.raw.get('mask')

  @property
  def actuator_delay_s(self) -> float | None:
    return self.raw.get('actuatorDelayS')

  @property
  def launch_verified(self) -> dict | None:
    return self.raw.get('launchVerified')

  @property
  def allow_provisional_for(self) -> str | None:
    return self.raw.get('allowProvisionalFor')

  @property
  def edge_deg(self) -> dict:
    """{side: deg} the frame edge in calibrated bearing: the lens fit's, else the bracket's pessimistic DEFAULT_EDGE_DEG."""
    lens = self.raw.get('lens')
    if lens:
      return {'left': float(lens['edgeDeg']['left']), 'right': float(lens['edgeDeg']['right'])}
    return {'left': DEFAULT_EDGE_DEG, 'right': DEFAULT_EDGE_DEG}

  def views(self, lighting: str, bound: str = 'conservative', use: str = 'allWay', candidates: bool = False) -> dict:
    """{view: bins} for a lighting row: per view the measured table, else a sim one (read only where accepted), else a
    provisional one where allowProvisionalFor covers use (or for route_audit's candidates); an optimistic bound falls
    back to the conservative table of a view that has no optimistic one. sunAhead and overcast read the day rows (nothing
    on the car detects overcast; the sun-ahead zeroing is stop_rules'); night has no table."""
    row = 'day' if lighting in ('sunAhead', 'overcast') else lighting
    out = {}
    for view in VIEWS:
      fits = [t for t in self.tables if t.view == view and t.lighting == row and
              (t.source != 'provisional' or candidates or (use in PROVISIONAL_USES and self.allow_provisional_for == use))]
      for b in ((bound, 'conservative') if bound != 'conservative' else ('conservative',)):
        pick = sorted((t for t in fits if t.bound == b), key=lambda t: TABLE_RANK[t.source])
        if pick:
          out[view] = [list(x) for x in pick[0].bins]
          break
    return out

  def sight(self, views: dict, zero=None, fresh=None, occluders=()) -> sr.Sight:
    """A stop_rules.Sight over views, with this file's frame edge and footprint mask (none until S1 measures it)."""
    mask, height = self.mask, self.height
    mask_fn = (lambda side, bearing, rng: sr.footprint_ok(mask, side, bearing, rng, height)) if mask and height is not None else None
    return sr.Sight(self.edge_deg, views, zero=zero, mask=mask_fn, fresh=fresh, occluders=list(occluders))


def _number(x) -> bool:
  return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _bad(name: str, why: str):
  raise VisibilityError(f"visibility.json {name}: {why}")


def _check_lens(lens):
  if not isinstance(lens, dict):
    _bad('lens', "must be {model, f, k, edgeDeg} or null")
  if lens.get('model') not in LENS_MODELS:
    _bad('lens.model', f"must be one of {', '.join(LENS_MODELS)}")
  for k in ('f', 'k'):
    if not _number(lens.get(k)):
      _bad(f'lens.{k}', "must be a number")
  edge = lens.get('edgeDeg')
  if not isinstance(edge, dict) or not all(_number(edge.get(s)) and 0. < edge[s] < 180. for s in ('left', 'right')):
    _bad('lens.edgeDeg', "must be {left, right} in deg")


def _check_mask(mask):
  if not isinstance(mask, dict):
    _bad('mask', "must be {left, right} or null")
  for side in ('left', 'right'):
    rows = mask.get(side)
    if not isinstance(rows, list):
      _bad(f'mask.{side}', "must be a list of [bearingDeg, elevLoDeg, elevHiDeg]")
    for r in rows:
      if not (isinstance(r, list) and len(r) == 3 and all(_number(x) for x in r) and r[1] < r[2]):
        _bad(f'mask.{side}', f"row {r!r} isn't [bearingDeg, elevLoDeg, elevHiDeg] with lo under hi")


def _check_table(i: int, t) -> Table:
  name = f'tables[{i}]'
  if not isinstance(t, dict):
    _bad(name, "must be an object")
  for k, allowed in (('source', SOURCES), ('view', VIEWS), ('lighting', LIGHTINGS)):
    if t.get(k) not in allowed:
      _bad(f'{name}.{k}', f"must be one of {', '.join(allowed)}")
  bound = t.get('bound', 'conservative')
  if bound not in BOUNDS:
    _bad(f'{name}.bound', f"must be one of {', '.join(BOUNDS)}")
  bins = t.get('bins')
  if not isinstance(bins, list) or not bins or not all(isinstance(b, list) and len(b) == 2 and all(_number(x) for x in b) and b[1] >= 0.
                                                        for b in bins):
    _bad(f'{name}.bins', "must be a non-empty list of [bearingLoDeg, metres >= 0]")
  if any(b[0] <= a[0] for a, b in zip(bins, bins[1:], strict=False)):
    _bad(f'{name}.bins', "bearings must ascend")
  if not isinstance(t.get('cars'), int) or isinstance(t.get('cars'), bool) or t['cars'] < 0:
    _bad(f'{name}.cars', "must be a whole number")
  if not isinstance(t.get('date'), str):
    _bad(f'{name}.date', "must be a date")
  return Table(t['source'], bound, t['view'], t['lighting'], [[float(a), float(b)] for a, b in bins], t['cars'], t['date'])


def validate(raw) -> list[Table]:
  """The file's tables, or a VisibilityError naming the first missing or bad field."""
  if not isinstance(raw, dict):
    _bad('(top level)', "must be an object")
  for k in REQUIRED:
    if k not in raw:
      _bad(k, "is missing (every field must be present; the camera's measurements may be null until step S1)")
  for k in ('cameraSetbackM', 'lateralOffsetM', 'heightM', 'darkThreshold'):
    if raw[k] is not None and not _number(raw[k]):
      _bad(k, "must be a number or null")
  if raw['lens'] is not None:
    _check_lens(raw['lens'])
  if raw['mask'] is not None:
    _check_mask(raw['mask'])
  if raw['veil'] is not None and raw['veil'] not in VEILS:
    _bad('veil', f"must be one of {', '.join(VEILS)} or null")
  if raw['allowProvisionalFor'] is not None and raw['allowProvisionalFor'] not in PROVISIONAL_USES:
    _bad('allowProvisionalFor', f"must be {' or '.join(PROVISIONAL_USES)} or null")
  delay = raw.get('actuatorDelayS')
  if delay is not None and (not _number(delay) or delay < 0.):
    _bad('actuatorDelayS', "must be a number of s, 0 or more, or absent")
  lv = raw.get('launchVerified')
  if lv is not None and (not isinstance(lv, dict) or any(k not in lv for k in LAUNCH_FIELDS)):
    _bad('launchVerified', f"must be {{{', '.join(LAUNCH_FIELDS)}}} (written by launch_verify.py) or absent")
  if not isinstance(raw['tables'], list):
    _bad('tables', "must be a list")
  return [_check_table(i, t) for i, t in enumerate(raw['tables'])]


def load(path: Path = SHIPPED, sim_ok: bool = False) -> Visibility:
  """The visibility file at path, or a VisibilityError (logged) naming what is wrong. sim_ok: sim_accepted() said so."""
  try:
    raw = json.loads(Path(path).read_text())
  except (OSError, ValueError) as e:
    err = VisibilityError(f"visibility.json at {path} doesn't read: {type(e).__name__}: {e}")
    cloudlog.error(f"turnpilot visibility: {err}")
    raise err from None
  try:
    tables = validate(raw)
  except VisibilityError as e:
    cloudlog.error(f"turnpilot visibility: {path}: {e}")
    raise
  out = Visibility(raw)
  for i, t in enumerate(tables):
    if t.source == 'sim' and not sim_ok:
      out.refused.append(f"tables[{i}] is a sim table, refused off the sim (SIMULATION=1, {SIM_PARAM} and a sim car)")
      cloudlog.error(f"turnpilot visibility: {out.refused[-1]}")
      continue
    out.tables.append(t)
  return out


_cache: dict = {}


def cached(path: Path = SHIPPED, sim_ok: bool = False) -> Visibility | None:
  """load(path), read again only when the file changes; None (logged once per change) when it is missing or bad, which
  certify() reads as noTable."""
  path = Path(path)
  try:
    st = path.stat()
    stamp = (st.st_mtime_ns, st.st_size, sim_ok)
  except OSError:
    stamp = None
  hit = _cache.get(path)
  if hit is not None and hit[0] == stamp and stamp is not None:
    return hit[1]
  try:
    out = load(path, sim_ok) if stamp is not None else None
  except VisibilityError:
    out = None
  if stamp is None:
    cloudlog.error(f"turnpilot visibility: no visibility file at {path}: every stop and yield is noTable")
  _cache[path] = (stamp, out)
  return out


def sim_car(car_params) -> bool:
  """CarParams (or anything with its fields) of a car the sim made: a fingerprint forced (fingerprintSource fixed, as the
  sim's FINGERPRINT does) or the mock car."""
  if car_params is None:
    return False
  return str(getattr(car_params, 'fingerprintSource', '')) == 'fixed' or str(getattr(car_params, 'brand', '')) == 'mock'


def sim_accepted(environ, sim_param, car_params) -> bool:
  """A sim table is read only with SIMULATION=1, the sim's CLEAR_ON_MANAGER_START param set and a sim car (module docstring)."""
  return environ.get('SIMULATION') == '1' and bool(sim_param) and sim_car(car_params)


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--check', metavar='FILE', required=True, help='a visibility.json to check against the schema')
  args = ap.parse_args(argv)
  try:
    v = load(Path(args.check), sim_ok=True)
  except VisibilityError as e:
    print(f"refused: {e}", file=sys.stderr)
    return 1
  ready, missing = v.camera_ready()
  print(f"{args.check}: schema ok, {len(v.tables)} tables ({', '.join(sorted({t.source for t in v.tables})) or 'none'})")
  print("camera measurements ready" if ready else f"camera measurements not ready (certify says noTable): {', '.join(missing)} empty")
  print(f"launchVerified: {'present' if v.launch_verified else 'absent (certify says noLaunch outside trial)'}")
  return 0


if __name__ == '__main__':
  sys.exit(main())
