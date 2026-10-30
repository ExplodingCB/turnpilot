#!/usr/bin/env bash
# Build driving-only Valhalla tiles from an OSM extract (.osm XML or .osm.pbf).
#
#   tools/turnpilot/build_valhalla_tiles.sh <input.osm|input.osm.pbf> <out_dir>
#
# Writes <out_dir>/{tiles/, tiles.tar, valhalla.json, admin.sqlite, build.log}, and controls_built for an OSM extract
# (.pbf), whose stop signs and signals navd reads. Point the
# TurnPilotValhallaTiles param at <out_dir>. The car only needs tiles.tar.
#
# Needs pyvalhalla (pinned below). It uses $VALHALLA_PYTHON, else the repo venv, else python3 if one of
# them has it, and otherwise creates $TURNPILOT_DATA/vh-venv (default ~/turnpilot_data/vh-venv) with uv.
#
# Valhalla only knows the driving side from a country polygon in its admin database, and defaults to
# left-hand traffic without one. Extracts and the BeamNG export don't have one, so valhalla_tiles_helper.py
# adds a polygon for ADMIN_ISO (default US) around the data.
set -euo pipefail

PYVALHALLA_VERSION="${PYVALHALLA_VERSION:-3.9.1}"
ADMIN_ISO="${ADMIN_ISO:-US}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="${TURNPILOT_DATA:-$HOME/turnpilot_data}"

if [ $# -ne 2 ] || [ ! -f "$1" ]; then
  echo "usage: $0 <input.osm|input.osm.pbf> <out_dir>" >&2
  exit 2
fi
IN="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
mkdir -p "$2"
OUT="$(cd "$2" && pwd)"

has_valhalla() { "$1" -c 'import valhalla' >/dev/null 2>&1; }

pick_python() {
  local py
  for py in "${VALHALLA_PYTHON:-}" "$HERE/../../.venv/bin/python" python3; do
    if [ -n "$py" ] && command -v "$py" >/dev/null && has_valhalla "$py"; then
      echo "$py"
      return
    fi
  done
  local venv="$DATA/vh-venv"
  [ -x "$venv/bin/python" ] || uv venv --python 3.12 "$venv" >&2
  uv pip install --python "$venv/bin/python" "pyvalhalla==$PYVALHALLA_VERSION" >&2
  echo "$venv/bin/python"
}

PY="$(pick_python)"
BIN="$("$PY" -m valhalla print_bin_path)"
helper() { "$PY" -I "$HERE/valhalla_tiles_helper.py" "$@"; }
START=$SECONDS

# osm_controls/ too: an OSM stop-sign sidecar from before is for other tiles (build_region.py and build_sim_tiles.sh rebuild it)
rm -rf "$OUT/tiles" "$OUT/tiles.tar" "$OUT/admin.sqlite" "$OUT/admin.osm" "$OUT/controls_built" "$OUT/controls.json" "$OUT/osm_controls"
mkdir -p "$OUT/tiles"

build() {
  echo "input $IN"
  echo "python $PY (pyvalhalla $("$PY" -c 'import valhalla; print(valhalla.__version__)'))"
  helper config "$OUT"

  echo "== admins"
  "$BIN/valhalla_build_admins" -c "$OUT/valhalla.json" "$IN" || true
  if ! { [ -f "$OUT/admin.sqlite" ] && helper has_country "$OUT/admin.sqlite"; }; then
    rm -f "$OUT/admin.sqlite"
    helper country "$IN" "$OUT/admin.osm" "$ADMIN_ISO"
    "$BIN/valhalla_build_admins" -c "$OUT/valhalla.json" "$IN" "$OUT/admin.osm"
  fi

  echo "== tiles"
  "$BIN/valhalla_build_tiles" -c "$OUT/valhalla.json" "$IN"
  echo "== extract"
  "$PY" -m valhalla.valhalla_build_extract -c "$OUT/valhalla.json" --overwrite -v
  rm -f "$OUT/admin.osm"

  echo "== done in $((SECONDS - START)) s"
  du -sh "$OUT/tiles" "$OUT/tiles.tar" "$OUT/admin.sqlite"
}

# Valhalla logs one harmless error per country in its planet access table when the extract lacks that country.
build 2>&1 | { grep -v "admin_access.admin_id" || true; } | tee "$OUT/build.log"

# An OSM extract (.pbf) carries its stop signs and signals, which Valhalla keeps: navd may read them off these tiles
# (selfdrive/turnpilot/routing/controls.py). An .osm file may be the sim's export, which has none: build_sim_tiles.sh
# adds them and marks its tiles itself.
case "$IN" in
  *.pbf) touch "$OUT/controls_built" ;;
esac
