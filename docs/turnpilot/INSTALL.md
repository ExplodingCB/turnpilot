# Install TurnPilot on a comma four

This guide takes a comma four with a chestnut eGPU from stock openpilot to a TurnPilot install that is ready for the first shadow-mode drive. It is written for Chase's car, a 2017 Model 3 with HW3. The drives are in [SHADOW_CHECKLIST.md](SHADOW_CHECKLIST.md).

Run every command on the comma from `/data/openpilot` over SSH, unless the step says it runs on your computer.

## What you need

- A comma four with the chestnut attached and the car harness installed.
- Home Wi-Fi for the install. The download is more than 800 MB, mostly the 775 MB big driving model.
- A Model 3 whose software is new enough for this openpilot. opendbc treats a Tesla without the 3-bit steering firmware as dashcam only; its comment dates that firmware to 2026.8.6 on HW3. If the comma stays in dashcam mode once it has seen the car, the car's software is the reason, and preflight reports it.
- An iPhone with Personal Hotspot.
- A GitHub account with an SSH key. The comma fetches the public keys from GitHub.
- A computer with `ssh` and `scp`, and the Indiana tile file `tiles.tar` (390 MB). `tools/turnpilot/build_valhalla_tiles.sh` builds it; it is in `~/turnpilot_data/valhalla/indiana/` on Kellen's Mac.

## What the branch name does

The installer URL is `installer.comma.ai/kaileh57/turnpilot`. The installer clones `github.com/kaileh57/openpilot.git`, which GitHub redirects to `kaileh57/turnpilot`, and checks out the branch `turnpilot`. To check this from any computer:

```
git ls-remote https://github.com/kaileh57/openpilot.git turnpilot
```

The command prints one line, `<commit>  refs/heads/turnpilot`.

openpilot decides what a branch is from its name. `RELEASE_BRANCHES` in `openpilot/common/version.py` lists `release-mici`, `nightly-chestnut` and a few others. `turnpilot` is not in the list, and that has three effects this install depends on:

1. The manager sets `IsReleaseBranch` to 0, so **Settings > Developer** shows the **alpha longitudinal** and **navigation (alpha)** toggles. Both are hidden on release branches. Tesla longitudinal (Autospeed) is alpha and needs the first one.
2. The branch has no `prebuilt` file, so the comma runs the build on every boot, and the first boot compiles openpilot from source, including the panda firmware. Later boots finish in seconds, until an update changes files. The panda build without `RELEASE=1` defines `ALLOW_DEBUG`, and the Tesla safety code only accepts longitudinal control in that build (`tesla.h`). The first boot reflashes the panda with the debug firmware.
3. The comma shows "WARNING: This branch is not tested" at startup. That is expected.

## Install

Do this at home on Wi-Fi, with the comma powered for the whole first boot. The chestnut occupies the comma four's USB-C port, so power comes from the harness: keep the car awake, or use a bench 12 V supply.

1. Join your home Wi-Fi.
2. If openpilot is already installed, open **Settings > Software**, tap **uninstall openpilot**, and confirm. The device resets its data partition and restarts into setup, so upload any routes you want to keep first.
3. On the setup screen, slide to install custom software, accept the warning, pick your Wi-Fi network, and tap **choose software**.
4. Enter `installer.comma.ai/kaileh57/turnpilot`. Setup also accepts the short form `kaileh57/turnpilot`.
5. Wait for the install. The screen shows a percentage while the comma clones the branch and its submodules and pulls the big driving model from comma's Hugging Face LFS server.
6. The comma restarts into the first boot. A progress bar shows the compile. This takes a while; nobody has timed it for this branch. Do not cut power. If the compile fails, the screen shows the compiler output.

When the comma shows the road view, the install is done.

## Turn on SSH

1. Open **Settings > Developer** and turn on **enable SSH**.
2. Tap **SSH keys** and enter your GitHub username.
3. Find the comma's address under **Settings > Network**, next to the Wi-Fi name.
4. On your computer, on the same network, run:

```
ssh comma@<address>
cd /data/openpilot
```

## Let the comma see the car

The comma only offers alpha longitudinal after it has identified the car once. Turn the car on (drive-ready, brake pedal pressed) for about a minute, then turn it off. Then run:

```
python3 -m openpilot.selfdrive.turnpilot.preflight
```

The `car` line shows `tesla TESLA_MODEL_3` once the comma has identified the car. Other lines fail at this point, and the next steps fix them.

## Set the toggles

On the comma screen, in **Settings > Developer**:

- Turn on **alpha longitudinal** and confirm. The comma cycles its driving processes to apply it. Without it, openpilot does not control speed on the Tesla, and TurnPilot's stops and speed caps do nothing.
- Turn on **navigation (alpha)**. This is the `NavigationEnabled` param. It starts navd, which routes, and mapsd, which draws the map.
- Turn on **Disengage on accelerator pedal**. This is the `DisengageOnAccelerator` param. Any gas press must disengage, because TurnPilot never takes a gas press as its go. A device that had this toggle off for an earlier TurnPilot keeps it off until you turn it on, and while it is off TurnPilot shows "TurnPilot Unavailable: turn on Disengage on accelerator pedal" and does nothing. Preflight's `gas disengages` line fails while it is off.

Over SSH, turn on TurnPilot in shadow mode. The stock params tool can't write boolean params, so use preflight:

```
python3 -m openpilot.selfdrive.turnpilot.preflight --set TurnPilotEnabled=1 --set TurnPilotShadowMode=1
```

| Param | Value | What it does |
| --- | --- | --- |
| `TurnPilotEnabled` | 1 | Starts `turnpilot_web` (the phone app) within a few seconds, makes `turnpilotd` produce a plan, and makes modeld load the detector the next time the car turns on. |
| `TurnPilotShadowMode` | 1 | TurnPilot computes the plan and publishes it marked as shadow. modeld and the planner ignore it, so the car drives as stock openpilot. Set it to 0 only from the second rung of the car ladder. |
| `TurnPilotMapHost` | `https://<ip>` | TurnPilot's map server. Set it in the phone app's Map server section. |
| `TurnPilotMapKey` | the API key | Sent as a Bearer token. Set it in the phone app, which never shows it again (only its last 4 characters). |
| `TurnPilotMapCert` | PEM | The server's self-signed certificate. Every request is verified against it. |
| `TurnPilotValhallaTiles` | unset | Developer and sim override (BeamNG sets it): route on these local tiles, with no geofence and no download. Leave it unset in the car. |
| `TurnPilotRouteSource` | `auto` | Left over from Mapbox routing. `mapbox` is now an error; anything else is ignored. |
| `TurnPilotWebToken` | generated | The phone's password. `turnpilot_web` creates it on its first run. |

## Install Valhalla for offline routing

The system Python on the comma is read-only, so pyvalhalla goes into `/data/turnpilot/pyvalhalla`, which survives updates. The comma needs internet once for the 35 MB wheel, from home Wi-Fi or the hotspot:

```
bash tools/turnpilot/install_valhalla_device.sh
```

The script checks the wheel's sha256, unpacks it, and imports it as a test. It needs 300 MB free. For an install with no internet on the comma, copy the wheel over with `scp` and run `PYVALHALLA_WHEEL=/path/to/wheel bash tools/turnpilot/install_valhalla_device.sh`.

## Set up the map server

TurnPilot routes only inside its mapped areas, around Indianapolis and Seattle. There is no Mapbox and no fallback.

1. Open the phone app, then **Map server**. Enter the address, the API key and the server's certificate (PEM), and tap **Save and check**. The comma fetches the mapped area at once, so the comma must be online. It then refreshes it every 6 hours while online and keeps it in `/data/turnpilot/maps/coverage.json`.
2. The **Mapped area** section draws the areas, the car and the destination. A destination outside them is refused with "Outside TurnPilot's mapped area". Until the comma has fetched the mapped area once, every destination is refused.
3. When a destination is set, navd downloads that drive's map (a corridor of tiles around the route, 20-50 MB) from the server, checks every file's SHA-256, and unpacks it under `/data/turnpilot/maps/corridors/`. The phone shows "Downloading map for this drive…" and then "Ready". Reroutes run on the comma with no internet. The last 3 corridors are kept.
4. Any failure means no route, with the reason on the phone and on the comma's onroad screen ("No route"): server unreachable, wrong key, certificate mismatch, a corridor that fails its check, or the server refusing the drive.

The developer override is `TurnPilotValhallaTiles`: point it at a tile directory or `tiles.tar` (as the sim does) and navd routes on those tiles with no geofence and no download. Preflight then routes about 2 km across downtown Indianapolis on them.

## Restart the car

Turn the car off and on. modeld reads `TurnPilotEnabled` when it starts, and it loads the detector then. Run preflight again. The `web token`, `pairing URL` and `web server` lines pass once `turnpilot_web` is running, which starts within seconds of `TurnPilotEnabled` being set. It creates the token on its first run.

## Pair the phone

1. On the comma, open **Settings > Network** and join the iPhone's hotspot. Set the connection to metered, so the updater does not pull downloads over the phone's data. Join your computer to the hotspot too, and open a new SSH session to the comma's address on the hotspot (**Settings > Network** shows it). The old session ends when the comma leaves home Wi-Fi.
2. Run preflight with a QR code:

   ```
   python3 -m openpilot.selfdrive.turnpilot.preflight --qr
   ```

3. Scan the QR code with the phone camera and open the link in Safari. The link ends in `?t=` and the token, which is the password for the comma. Anyone on the hotspot who has it can set the destination.
4. Optional: tap Share, then Add to Home Screen, while the address still ends in `?t=...`.
5. Optional: add the Apple Maps share-sheet shortcut from [IOS_SHORTCUT.md](IOS_SHORTCUT.md).

The page shows the route, the next maneuver, and TurnPilot's state. To start navigating, search for a place and tap Navigate. Place search needs internet through the hotspot.

## Run preflight before every drive

```
python3 -m openpilot.selfdrive.turnpilot.preflight
```

Each line starts with PASS, FAIL, WARN or INFO. A FAIL line is followed by a `fix:` line, and the command exits with status 1 if any check fails. A WARN means TurnPilot is live, not in shadow mode. The checks are:

| Check | What passes |
| --- | --- |
| `non-release branch` | The branch is not in `RELEASE_BRANCHES` |
| `car` | The comma has identified the car, and it is not in dashcam mode |
| `alpha longitudinal` | The toggle is on, and the last drive started with it |
| `navigation`, `TurnPilot` | `NavigationEnabled` and `TurnPilotEnabled` are on |
| `map server` | The address, key and certificate are set and the mapped area has been fetched |
| `pyvalhalla`, `valhalla tiles`, `routing test` | pyvalhalla imports; with `TurnPilotValhallaTiles` set, those tiles are usable and a real route comes back |
| `big driving model` | The 775 MB pickle and both warps are real files, not LFS pointers. If they are pointers, modeld quietly runs the small model |
| `detector pkl` | `big_detector_1344x760_tinygrad.pkl` exists. Without it the planner holds every left turn |
| `chestnut` | The chestnut is present with its firmware and a USB 3 link |
| `web token`, `pairing URL`, `web server` | The token exists, the comma has a network address, and `turnpilot_web` answers on port 8088 |

`shadow mode` and `route source` are INFO lines. `route source` warns when `TurnPilotValhallaTiles` is set, since the car should use the map server.

## Update

The comma pulls from the `turnpilot` branch of `github.com/kaileh57/openpilot`, so an update is whatever has been pushed there.

1. Join home Wi-Fi.
2. Open **Settings > Software** and tap **check for update**. If it offers **download update**, tap that. When the download finishes, **install now** appears (the car must be off). The comma restarts and recompiles, which takes several minutes.
3. Run preflight.

The updater applies a downloaded update at the next boot, and the comma boots every time the car wakes. On a test day, stop surprise updates with:

```
python3 -m openpilot.selfdrive.turnpilot.preflight --set DisableUpdates=1
```

Set it to 0 when you want to update. The files in `/data/turnpilot` (pyvalhalla and the maps) stay through updates, but not through an uninstall.

## When something fails

| What you see | What it means |
| --- | --- |
| The alpha longitudinal toggle is missing | The comma hasn't identified the car yet, or the branch is a release branch. See the `car` and `non-release branch` lines |
| `big driving model` fails | The LFS download failed during the install. Run `git lfs pull` in `/data/openpilot`, or reinstall on better Wi-Fi |
| `chestnut` fails | Check the cable and the 12 V supply, then power the comma off and on |
| `pyvalhalla` fails after the script ran | The script's import test failed, which means the system glibc or libstdc++ is older than the wheel needs. The script prints which |
| The phone page says Not paired | The link is missing `?t=` and the token. Scan the QR code again |
| `detector pkl` fails | The branch doesn't carry the compiled detector yet. Update, or ask Kellen |

## Decisions for Kellen

Two safety choices are open. Both are set to the cautious option in this build, and neither changes unless you decide.

**The 20 mph minimum for nav lane changes.** DesireHelper refuses lane changes below 20 mph (`LANE_CHANGE_SPEED_MIN` in `openpilot/selfdrive/controls/lib/desire_helper.py`), and TurnPilot's lane planner adds 0.5 m/s on top (`LC_SPEED_MARGIN`) so a request does not wait below the limit. Nav lane changes inherit both. On city streets below that speed, the car does not change lanes for a turn, so a turn from the wrong lane is missed, or the driver changes lanes. Lowering the limit for nav requests only is a safety change in DesireHelper, and the model's lane-change behavior at low speed has not been looked at. The suggestion: keep 20 mph through the shadow and empty-lot rungs, count how many needed lane changes happen below it on the empty-streets rung, and decide with those notes.

**The stalk signal prompt.** The Tesla port cannot send turn signals: the panda allows only three Tesla messages, and the controller has a `TODO: HUD control`. TurnPilot treats every car except Hyundai as one that can't signal (`OPENPILOT_SIGNALS_BRANDS` in `lane_planner.py`). On those cars it publishes a "signal left" or "signal right" prompt and holds the lane change request until the driver taps the stalk. Then the change goes ahead without a steering nudge, and the request drops after 8 seconds if the blindspot check or the driver doesn't clear it. This keeps signals legal, but a lane change needs a hand on the stalk. Nothing in this build displays the prompt: the phone page and the comma screen don't read `signalPrompt` yet. Until an overlay ships, Chase gets no cue, and a nav lane change only happens if he signals on his own. The suggestion: keep the prompt, and treat lane changes as driver-initiated on the first rungs. The other options are to change lanes without signaling, which is unsafe for the cars around the Tesla and illegal in most places, or to turn off nav lane changes on Tesla.
