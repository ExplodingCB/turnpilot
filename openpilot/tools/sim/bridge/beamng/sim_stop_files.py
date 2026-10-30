#!/usr/bin/env python3
"""The sim's own visibility file and survey of its all-way stops (STOP_SIGNS.md v3 section 8.2, SG4c), for the stop-sign logic
gate: every value synthetic, labelled so, and read by turnpilotd only under the sim gate (SIMULATION=1, the
TurnPilotSimLightOracle param and a sim car: TurnPilot.set_sim), never on the car.

- visibility(): the shipped visibility.json with camera fields for BeamNG's wide camera (a 1344x760 pinhole, f 425.25 px,
  57.7 deg to the edge: cameras.py), an open mask, the veil measured none (BeamNG has no windshield), a 0.3 s actuator delay
  and a passing launchVerified. Reach in the sim is the cross-traffic oracle's range (visibility.OracleLive), not these tables.
- surveys(tiles): every junction of the sim tiles' OSM sidecar (build_sim_tiles.sh from the level's export) whose approaches
  all stop and number three or more, surveyed: each approach's heading arriving, stopping, its posted limit from the sidecar's
  maxspeed, the default lane edge and line, open parking and sight; one passing day validation stamp per approach, its
  first-track range the oracle's (ORACLE_RANGE_M) on every lane.

python -m openpilot.tools.sim.bridge.beamng.sim_stop_files [--tiles DIR] writes both into selfdrive/turnpilot/maps/sim/."""
import argparse
import datetime
import glob
import json
import os
from pathlib import Path

from openpilot.selfdrive.turnpilot import visibility_table as vt

TILES = Path.home() / 'turnpilot_data' / 'valhalla' / 'west_coast_usa_signals'
OUT = Path(__file__).resolve().parents[4] / 'selfdrive' / 'turnpilot' / 'maps' / 'sim'
ORACLE_RANGE_M = 60.     # m: the cross-traffic oracle's range (the bridge's), each validated lane's first-track range
MPH = 0.44704
LANES = 4                # lanes a stamp covers per side (more than any sim approach has)
SYNTHETIC = "synthetic: the sim's (sim_stop_files.py), a logic check's, never range evidence"


def visibility() -> dict:
  raw = json.loads(vt.SHIPPED.read_text())
  raw.update(cameraSetbackM=1.9, lateralOffsetM=0.0, heightM=1.3, darkThreshold=45, veil='none', actuatorDelayS=0.3,
             lens={'model': 'equidistant', 'f': 425.25, 'k': 0.0, 'edgeDeg': {'left': 57.7, 'right': 57.7}},
             mask={'left': [[b, -30., 30.] for b in range(89)], 'right': [[b, -30., 30.] for b in range(89)]},
             launchVerified={'date': datetime.date.today().isoformat(), 'goes': 10, 'takenOver': 10, 'maxTakeoverS': 1.0, 'verdict': 'pass'},
             synthetic=SYNTHETIC)
  return raw


def stamp(heading: float, today: datetime.date) -> dict:
  sides = ('left', 'right', 'opposite')
  return {'lighting': 'day', 'date': today.isoformat(), 'passes': 3, 'approachBearingDeg': heading, 'cars': dict.fromkeys(sides, 10),
          'minFirstTrackM': {s: [ORACLE_RANGE_M] * LANES for s in sides}, 'maxDropoutS': {s: [0.2] * LANES for s in sides},
          'allStopped': True, 'verdict': 'pass', 'synthetic': True}


def surveys(tiles: Path = TILES, today: datetime.date | None = None) -> list[dict]:
  today = today or datetime.date.today()
  out = []
  for path in sorted(glob.glob(os.path.join(tiles, 'osm_controls', '*.json'))):
    for node, rec in sorted(json.loads(Path(path).read_text()).get('junctions', {}).items(), key=lambda kv: int(kv[0])):
      legs = rec.get('approaches') or []
      if len(legs) < 3 or not all(a.get('stops') and a.get('kind') == 'stop' for a in legs) or not rec.get('pos'):
        continue
      approaches = []
      for a in legs:
        heading = round((float(a.get('bearing_deg', 0.)) + 180.) % 360., 1)  # the leg leaves the node: arriving is reversed
        mph = round(float(a['maxspeed']) / MPH) if a.get('maxspeed') else 25
        approaches.append({'wayId': int(a['way_id']), 'bearingDeg': heading, 'stops': True, 'plaque': True, 'postedMph': mph,
                           'laneEdgeM': 3.5, 'lineM': 7., 'nearestParkingM': {'left': 60., 'right': 60.}, 'sightM': 100.,
                           'stopPointM': None})
      out.append({'osmNodeId': int(node), 'lat': float(rec['pos'][0]), 'lon': float(rec['pos'][1]), 'date': today.isoformat(),
                  'source': 'inPerson', 'confirmedBy': ['inPerson'], 'synthetic': True, 'note': SYNTHETIC, 'approaches': approaches,
                  'validations': [stamp(a['bearingDeg'], today) for a in approaches]})
  return out


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--tiles', default=str(TILES), help='the sim tiles (their osm_controls sidecar)')
  ap.add_argument('--out', default=str(OUT))
  args = ap.parse_args(argv)
  out = Path(args.out)
  out.mkdir(parents=True, exist_ok=True)
  entries = surveys(Path(args.tiles))
  (out / 'surveyed_junctions_sim.json').write_text(json.dumps(entries, indent=1) + '\n')
  (out / 'visibility_sim.json').write_text(json.dumps(visibility(), indent=1) + '\n')
  print(f"{len(entries)} sim all-way junctions surveyed (synthetic) into {out}")
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
