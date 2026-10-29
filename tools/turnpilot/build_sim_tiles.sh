#!/usr/bin/env bash
# Valhalla tiles for the sim's stock map, with lanes and stop controls: the west_coast_usa road-graph export from the PC,
# plus turn lanes (openpilot/tools/sim/bridge/beamng/maplanes.py), plus the level's stop signs and traffic signals
# (mapsignals.py, from the committed wcusa_map_node_signals.json), built into tiles with build_valhalla_tiles.sh.
#
#   tools/turnpilot/build_sim_tiles.sh [export.osm [out_dir]]
#
# The export defaults to ~/pc_relay/incoming/osm_export/west_coast_usa.osm and out_dir to
# $TURNPILOT_DATA/valhalla/west_coast_usa_signals (TURNPILOT_DATA defaults to ~/turnpilot_data). The tiles built earlier stay
# where they are: the export alone in west_coast_usa, with lanes in west_coast_usa_lanes (never overwritten here). out_dir
# also gets controls.json (the stop lines, which routing/controls.py projects onto a route), the controls_built marker
# (without it navd reports the tiles' controls as unsupported), and the OSM stop-sign sidecar the real regions carry
# (osm_controls/, from mapserver/osm_controls.py on the same OSM the tiles came from), stamped with the export's checksum,
# which meta.json holds as the tiles' osm_timestamp (without both, navd says the stop-sign data is missing). Point TurnPilotValhallaTiles at out_dir (launch_openpilot.sh:
# TURNPILOT_TILES), and set TurnPilotRouteSource to valhalla so that a destination outside the map is an error instead of a
# route on Indiana's real roads. $VALHALLA_PYTHON is the Python with pyvalhalla and openpilot's dependencies, the repo venv
# by default. Needs no game: the signal and stop-prop dumps are committed with the bridge.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
DATA="${TURNPILOT_DATA:-$HOME/turnpilot_data}"
EXPORT="${1:-$HOME/pc_relay/incoming/osm_export/west_coast_usa.osm}"
OUT="${2:-$DATA/valhalla/west_coast_usa_signals}"
PY="${VALHALLA_PYTHON:-$ROOT/.venv/bin/python}"
LANES_OSM="$DATA/osm/$(basename "$OUT")_lanes.osm"
SIGNALS_OSM="$DATA/osm/$(basename "$OUT").osm"
BRIDGE="$ROOT/openpilot/tools/sim/bridge/beamng"
SIGNALS_JSON="${SIGNALS_JSON:-$BRIDGE/wcusa_map_node_signals.json}"
STOP_PROPS="${STOP_PROPS:-$BRIDGE/wcusa_stop_props.json}"

[ -f "$EXPORT" ] || { echo "no export at $EXPORT" >&2; exit 2; }
[ -x "$PY" ] || { echo "no Python at $PY: set VALHALLA_PYTHON" >&2; exit 2; }
case "$(basename "$OUT")" in
  west_coast_usa|west_coast_usa_lanes) echo "refusing to overwrite $OUT: pick another out_dir" >&2; exit 2 ;;
esac

mkdir -p "$DATA/osm"
PYTHONPATH="$ROOT" "$PY" -m openpilot.tools.sim.bridge.beamng.maplanes "$EXPORT" "$LANES_OSM"
PYTHONPATH="$ROOT" "$PY" -m openpilot.tools.sim.bridge.beamng.mapsignals "$LANES_OSM" "$SIGNALS_JSON" "$SIGNALS_OSM" \
  --stop-props "$STOP_PROPS" --sidecar "$DATA/osm/$(basename "$OUT")_controls.json"
VALHALLA_PYTHON="$PY" "$HERE/build_valhalla_tiles.sh" "$SIGNALS_OSM" "$OUT"
cp "$DATA/osm/$(basename "$OUT")_controls.json" "$OUT/controls.json"
STAMP="sim-$("$PY" -c 'import hashlib, sys; print(hashlib.sha256(open(sys.argv[1], "rb").read()).hexdigest()[:16])' "$SIGNALS_OSM")"
printf '{"id": "%s", "osm_timestamp": "%s"}\n' "$(basename "$OUT")" "$STAMP" > "$OUT/meta.json"
PYTHONPATH="$ROOT" "$PY" "$ROOT/tools/turnpilot/mapserver/osm_controls.py" "$SIGNALS_OSM" "$OUT" --timestamp "$STAMP" --tiles "$OUT/tiles"
touch "$OUT/controls_built"
echo "sim tiles with stop controls in $OUT"
