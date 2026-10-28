#!/usr/bin/env bash
# Copy the map server and its regions from this machine to the VM, and restart it.
#
#   deploy.sh [region ...]      default: every region built under $REGIONS
#
# The VM's address and SSH key come from TURNPILOT_MAPS_HOST (default turnpilot@136.113.85.41) and
# TURNPILOT_MAPS_SSH_KEY (default ~/.ssh/turnpilot_maps). Run setup_server.sh on the VM once first;
# "deploy.sh --setup" does that. Each region is uploaded to a staging directory, then swap_region.sh copies it beside
# the live one and swaps it in with renames, restarts the server at once (it holds each region's index in memory) and
# checks that it serves the region, rolling back if not. So the server never serves a half-copied region, nor tiles
# that don't match the index it loaded.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST="${TURNPILOT_MAPS_HOST:-turnpilot@136.113.85.41}"
KEY="${TURNPILOT_MAPS_SSH_KEY:-$HOME/.ssh/turnpilot_maps}"
REGIONS="${REGIONS:-$HOME/turnpilot_data/mapserver/regions}"
SSH=(ssh -i "$KEY" -o BatchMode=yes "$HOST")

if [ "${1:-}" = "--setup" ]; then
  shift
  rsync -e "ssh -i $KEY" "$HERE/setup_server.sh" "$HOST:/tmp/setup_server.sh"
  "${SSH[@]}" "bash /tmp/setup_server.sh ${HOST#*@}"
fi

# server.py requires --tmp-dir, which only the unit setup_server.sh writes has. With an older unit the new server would
# not start, and swap_region.sh would take that for a bad region and roll it back: refuse here instead, before anything.
if ! "${SSH[@]}" "grep -q -- '--tmp-dir' /etc/systemd/system/turnpilot-maps.service"; then
  echo "The server's systemd unit has no --tmp-dir (corridors would be built in a tmpfs /tmp that counts against MemoryMax)." >&2
  echo "Run: $0 --setup ${*:-}" >&2
  exit 1
fi

rsync -e "ssh -i $KEY" "$HERE/server.py" "$HERE/geo.py" "$HERE/tiles.py" "$HERE/swap_region.sh" "$HOST:/opt/turnpilot-maps/"

if [ $# -eq 0 ]; then
  set -- $(cd "$REGIONS" && ls)
fi
for region in "$@"; do
  src="$REGIONS/$region"
  for f in fence.geojson meta.json index.json tiles.tar; do
    [ -e "$src/$f" ] || { echo "$src/$f is missing; build the region first" >&2; exit 1; }
  done
  # the OSM stop-sign sidecar: without it the cars miss the stops Valhalla doesn't flag, and say so on every route
  if ! [ -d "$src/osm_controls" ] || ! grep -q '"osm_controls/' "$src/index.json"; then
    echo "$src has no OSM stop-sign sidecar in its index; run: build_region.py $region --sidecar-only" >&2
    exit 1
  fi
  echo "== $region"
  rsync -a --delete -e "ssh -i $KEY" --include='/fence.geojson' --include='/meta.json' --include='/index.json' \
    --include='/tiles.tar' --include='/tiles/***' --include='/osm_controls/***' --exclude='*' \
    "$src/" "$HOST:/srv/turnpilot-maps/staging/$region/"
  "${SSH[@]}" "bash /opt/turnpilot-maps/swap_region.sh /srv/turnpilot-maps/regions /srv/turnpilot-maps/staging $region"
done

# the new server code, when no region was swapped (each swap already restarted it)
if [ $# -eq 0 ]; then
  "${SSH[@]}" "sudo systemctl restart turnpilot-maps && sleep 3 && systemctl is-active turnpilot-maps"
fi
"${SSH[@]}" "curl -sfk --max-time 10 https://127.0.0.1/v1/health"
echo
