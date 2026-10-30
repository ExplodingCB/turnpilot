# TurnPilot map server

The comma routes on Valhalla tiles that it downloads for each drive. This server holds the tiles for the areas TurnPilot can drive in, and hands the comma the tiles along its route. It also publishes those areas as a geofence, which the comma uses to refuse destinations outside them.

The server runs on a Google Cloud e2-micro VM (`turnpilot-maps`, project `gdp-credits`, us-central1-a). That machine size is in Google's free tier.

## Areas

| Region | Fence | Data |
| --- | --- | --- |
| `indianapolis` | 300 km around Indianapolis, clipped to the states below | Indiana, Illinois, Ohio, Kentucky, Michigan |
| `seattle` | 250 km around Seattle, clipped to the states below | Washington, Oregon |

The fence is the circle clipped to the states' Geofabrik boundaries, so the Seattle fence stops at the Canadian border. The map data is cut 25 km wider than the fence, so a route between two points just inside the fence still has the roads around it. To add a region, add an entry to `REGIONS` in `build_region.py`.

## API

Every call except `/v1/health` needs `Authorization: Bearer <key>`. The key is in `/etc/turnpilot-maps/key` on the VM. The server uses a self-signed certificate for its IP address (`/etc/turnpilot-maps/cert.pem`), and the comma checks against that exact certificate.

`GET /v1/health`
: Returns `{"ok": true, "regions": [...]}`.

`GET /v1/coverage`
: Returns the fences as a GeoJSON FeatureCollection, with a `version` that changes whenever a region is rebuilt.

`POST /v1/corridor` with `{"start": [lon, lat], "destination": [lon, lat], "osm_controls": true}`
: Returns a tar of `manifest.json` and every tile within 15 km of the route, gzip-compressed, with each tile's SHA-256 in the manifest. With `"osm_controls": true`, which the car sends, it also holds the OSM stop-sign sidecar's shards for those tiles (`osm_controls/<level-2 tile>.json`), listed the same way, and the manifest carries the region's `osm_timestamp`, which the car checks the shards against. A car from before the sidecar doesn't ask, and gets the tiles alone. If either end is outside the fences, or the two ends are in different regions, or Valhalla finds no route, it returns a 422 with `error` set to `outside_coverage` (with `which`), `different_regions`, or `no_route`.

## Build and deploy

Build on a machine with enough memory; the VM has 1 GB, which is too little. The build needs osmium-tool (`brew install osmium-tool`), and shapely in a separate venv:

```bash
uv venv --python 3.12 ~/turnpilot_data/mapbuild-venv
```

```bash
uv pip install --python ~/turnpilot_data/mapbuild-venv/bin/python shapely==2.0.6
```

To build a region, run:

```bash
VALHALLA_PYTHON=.venv/bin/python ~/turnpilot_data/mapbuild-venv/bin/python -I tools/turnpilot/mapserver/build_region.py seattle
```

It downloads the states from Geofabrik if they aren't in `~/turnpilot_data/osm`, and writes the region to `~/turnpilot_data/mapserver/regions/<region>`.

Right after the tiles, the build writes the OSM stop-sign sidecar (`osm_controls.py`) from the same PBF: every stop sign, yield and level crossing, by the approach it governs, one shard per level-2 tile in `osm_controls/`, stamped with the region's `osm_timestamp` and listed in `index.json` beside the tiles. Valhalla misses about a quarter of the mapped stop approaches, and the car unions the sidecar's with Valhalla's (`openpilot/selfdrive/turnpilot/routing/controls.py`). To rebuild only the sidecar of a built region, from its PBF and with its timestamp, which takes about a minute for Indianapolis, run:

```bash
.venv/bin/python -I tools/turnpilot/mapserver/build_region.py indianapolis --sidecar-only
```

`deploy.sh` refuses a region whose index lists no sidecar shards.

To set up a new VM (once), then copy the server and the built regions and restart it, run:

```bash
tools/turnpilot/mapserver/deploy.sh --setup
```

After that, `deploy.sh` alone redeploys every built region, and `deploy.sh seattle` redeploys one. Each region is uploaded to a staging directory, then `swap_region.sh` on the VM swaps it in with renames, restarts the server and checks `/v1/health`. If the server doesn't come back serving the region, the old copy goes back and the deploy stops with an error. Geofabrik's extracts update daily. Rebuild and redeploy when the maps need refreshing.

## Cost

The e2-micro and 30 GB of standard disk are in Google's free tier. The other 30 GB of disk, the public IPv4 address, and outbound traffic past the free 1 GB a month are billed. The account's $10 a month of Google Developer Program credit covers all of that, and a budget emails the owner if real charges pass $0.50 in a month. A drive's download is usually 20-50 MB: Purdue to Peru, Indiana (91 km) was 24 MB, and Seattle to Tacoma (54 km) was 47 MB.
