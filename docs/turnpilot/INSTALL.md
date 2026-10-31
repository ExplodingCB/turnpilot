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

## Install from the Mac (private opendbc)

The installer path in "Install" below can't install this build. The build's `opendbc_repo` gitlink is a commit in the private repo `github.com/kaileh57/turnpilot-opendbc`, and the installer clones over unauthenticated https with its submodules, so the clone fails at opendbc. No credential goes on the comma or into the repo, so the fix is to build the tree on Kellen's Mac and copy it over SSH. Until opendbc is public, use this section in place of steps 2 to 5 of "Install". The comma fetches nothing from GitHub.

`openpilot/tools/turnpilot/deploy_to_comma.py` does the whole job and prints each step. It runs with the Mac's `python3` and needs `git-lfs` on the Mac.

1. It reads the comma's state over SSH and changes nothing. It refuses if the comma is onroad (`IsOffroad` is 0, `IsOnroad` is 1, or neither is set), if SSH fails, or if `/data` has less free space than the tree, plus a copy of the LFS files that git-lfs on the comma makes, plus 3 GB for the first build (about 5.3 GB in all).
2. It builds the tree in a temporary directory from the Mac's own clones: the commit on branch `turnpilot`, tracking `origin/turnpilot` (origin is `https://github.com/kaileh57/turnpilot.git`, which nothing fetches while `DisableUpdates` is 1), depth 1. Each submodule is at its gitlink, from `.git/modules/<name>`. A submodule commit missing there comes from the public URL in `.gitmodules`; the opendbc commit is on the Mac. Every git-lfs file (the driving models, the fonts, the AGNOS updater) is its real bytes, checked against its sha256. No build output from the Mac goes, because the tree comes from git objects, not from a working copy.
3. It copies the tree to `/data/openpilot.tp-incoming-<stamp>`, then checks it on the comma: every file's sha256 against the Mac's manifest, the file count, the symlinks, `HEAD`, the branch and its upstream, and each submodule's commit. It runs the same git commands as `is_dirty()` in `openpilot/common/version.py`, which also makes git-lfs hash the model files once, so the first boot doesn't. The tree's own git config sets git-lfs's clean filter (`filter.lfs.clean`, `filter.lfs.required`), so this check and the device's `is_dirty()` don't depend on how AGNOS configures git for user `comma`. It sets no smudge filter, so checkouts keep whatever AGNOS gives them.
4. It checks again that the comma is offroad, renames `/data/openpilot` to `/data/openpilot.prev-<stamp>`, and moves the new tree to `/data/openpilot`. The comma does both moves even if the SSH connection drops between them.
5. It sets `DisableUpdates` to 1, so the updater doesn't try GitHub, and asks before it reboots the comma. Before the reboot, the comma checks again that it is offroad, that `DisableUpdates` is 1, and that `/data/openpilot/.overlay_init` doesn't exist. Sunnypilot's updater keeps running until the reboot, and it creates that file when it sets up its overlay over the new tree. With the file there, the next boot would install any update the updater stages from GitHub over the deployed build.

If a step fails, the script stops and says which state the comma is in: as it was, with both trees intact (the old one still in `/data/openpilot`, the new one beside it), or unknown. The state is unknown when the SSH connection ends during the swap or the rollback. Then don't reboot the comma or start the car until `--dry-run` has shown what `/data/openpilot` holds. The message gives the commands that put a missing tree back and that set `DisableUpdates`. A leftover `openpilot.tp-incoming-*` directory is listed on the next run with the command that removes it.

### Run it

Do the first install on home Wi-Fi, with the Mac and the comma on the same network, and with the comma powered through the first boot. Turn on SSH first ("Turn on SSH" below), and use the comma's address from **Settings > Network**.

1. On the Mac, see what the default deploys, the tip of the local `turnpilot` branch, with its gitlinks:

   ```
   cd ~/Projects/turnpilot
   git log -1 --oneline turnpilot
   git ls-tree turnpilot | grep commit
   ```

2. Check everything without changing the comma:

   ```
   python3 openpilot/tools/turnpilot/deploy_to_comma.py comma@<address> --dry-run
   ```

   To deploy a different commit, add `--commit <sha>`. The script runs from any checkout that contains it, and reads commits from the repo that checkout belongs to.

3. Deploy:

   ```
   python3 openpilot/tools/turnpilot/deploy_to_comma.py comma@<address>
   ```

4. Answer `y` to reboot. If you answer `n`, don't start the car until the comma has rebooted. Until then sunnypilot keeps running from the renamed tree, and going onroad would start its processes out of TurnPilot's unbuilt tree. Reboot later, with the car off, with the same checks:

   ```
   python3 openpilot/tools/turnpilot/deploy_to_comma.py comma@<address> --reboot
   ```

   If the reboot is refused because `/data/openpilot/.overlay_init` exists, remove that file (`ssh comma@<address> rm /data/openpilot/.overlay_init`), then run `--reboot`. Without the file, the launcher installs nothing.

The copy is about 1.2 GB (1.18 GB and 4,229 files for commit `c8dff2e`; the 776 MB big driving model is most of it). It doesn't compress. At 5 MB/s it takes about 4 minutes, and at 1 MB/s about 20. Every deploy sends the whole tree again, so a later deploy over the phone hotspot takes as long as the first.

### What the first boot does

The comma boots `/data/openpilot` the way sunnypilot's tree booted (`/data/continue.sh` starts `launch_openpilot.sh`). The branch has no `prebuilt` file, so the first boot compiles openpilot from source, with a progress bar, and then pandad flashes the panda with this build's debug firmware ("What the branch name does" explains why). Nobody has timed the compile on this branch. Keep the comma powered until the road view shows. The tree has no `.overlay_init`, so the launcher doesn't install an update that sunnypilot left in `/data/safe_staging`. If sunnypilot's updater creates one after the swap, the script refuses to reboot. Then continue at "Turn on SSH" below. SSH is already on, because the settings carry over.

### What stays

The script creates and renames only `/data/openpilot`, `/data/openpilot.prev-*` and `/data/openpilot.tp-incoming-*`, and writes one param, `DisableUpdates`. Everything else in `/data` stays as sunnypilot left it:

- `/data/media`: sunnypilot's routes, in `/data/media/0/realdata`.
- `/data/params`: the settings, the calibration and the SSH keys. This differs from an uninstall, which wipes them. Sunnypilot's values carry over, so check the toggles in "Set the toggles": Chase's sunnypilot had `DisengageOnAccelerator` at 0, and TurnPilot needs 1.
- The sunnypilot tree, with its models, as `/data/openpilot.prev-<stamp>`. Any model bundle sunnypilot downloaded elsewhere in `/data` also stays.

TurnPilot's models are inside its tree: `openpilot/selfdrive/modeld/models` (the driving models) and `openpilot/selfdrive/turnpilot/models` (the detectors).

### Roll back to sunnypilot

```
python3 openpilot/tools/turnpilot/deploy_to_comma.py comma@<address> --rollback
```

The rollback refuses while the comma is onroad. It moves the newest `/data/openpilot.prev-*` back to `/data/openpilot` and keeps the tree it replaces as a new `/data/openpilot.prev-<stamp>`, so a second `--rollback` swaps back. After two deploys the newest kept tree is the earlier TurnPilot, not sunnypilot. Every run lists the kept trees with their branch and commit; name sunnypilot's with `--prev openpilot.prev-<stamp>`. Add `--dry-run` to see which tree would come back.

On its next boot, sunnypilot's pandad flashes sunnypilot's own panda firmware over TurnPilot's debug build. `DisableUpdates` stays 1 and `DisengageOnAccelerator` keeps TurnPilot's value; set both back in sunnypilot's settings if Chase wants them.

Each kept tree takes about 1.2 GB, more once it has been built. The script deletes nothing. To free the space, remove a kept tree by hand once you are sure you don't need it: `ssh comma@<address> rm -rf /data/openpilot.prev-<stamp>`.

### Not checked on a real comma

The script's tests run the whole flow against a stand-in comma on the Mac (`openpilot/tools/turnpilot/tests/test_deploy_to_comma.py`). A real comma four can differ in these places:

- that AGNOS has git-lfs installed. Its git config for user `comma` doesn't matter, because the tree's own config sets the clean filter. Without git-lfs, the script warns and `is_dirty()` reads True, though every file is still checked.
- that sunnypilot's updater behaves like this fork's `openpilot/system/updated/updated.py` between the swap and the reboot: it stages in `/data/safe_staging` and creates `.overlay_init` in `/data/openpilot` when it sets up its overlay. The reboot check looks for that file.
- that sunnypilot keeps `IsOffroad` in `/data/params/d`.
- that `/data/continue.sh` on this comma starts `/data/openpilot`. The script warns if it doesn't name it.
- how long the first compile takes, and whether 3 GB of headroom covers it.
- that sunnypilot reflashes its panda firmware after a rollback.

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

## Let the car signal its own turns

On a Tesla, TurnPilot can work the car's own turn signal through `DAS_bodyControls` (0x3E9). It is off by default, and it stays off on a car until a lamp check passes on that car and you confirm on the phone that the outside lamps flashed. Until then, and whenever the car's signal fails, every turn below 20 mph prompts "Signal Left" or "Signal Right" and the driver signals. The panda allows the message only in a debug build, the build this branch already flashes.

1. On the phone page's **Car turn signal** section, set the mode to **check**, then turn the car off and on. In Park, with hands off the stalk, the check runs by itself after 3 s: it blinks each side, says each step, and takes 75 to 110 s. If Park gives no lamp, set **check** again, restart, stop in Drive with the brake held, and tap **Start check in Drive**.
2. When it passes, the card asks whether the front, side and rear lamps flashed outside on each side. Answer it, then set the mode to **on**.
3. At the next ignition the car checks the record against its VIN and firmware. A record from a check in Park also blinks each side once in Park before the first drive. A Tesla update that changes the firmware voids the record: run the check again.

A Tesla's VIN reads as 17 zeros (opendbc has no Tesla VIN query), so on a Tesla the record can't match by VIN and the car signal stays off. To match it without a VIN, set `TurnPilotCarSignalsVinless` to 1 **before** the check (`python3 -m openpilot.selfdrive.turnpilot.preflight --set TurnPilotCarSignalsVinless=1`), then run the check in Park. The record then fits only this comma (its registered dongle ID), the platform the car's firmware fingerprints exactly, and the exact firmware list. A record from a check in Drive never matches this way, and neither does a record from a check run before the setting was on. A Tesla update, another comma, or the comma moved to a car whose firmware differs voids the record, and the car says why ("Car Signals Not Verified"). The dongle ID names the comma, not the car: moved to another car of the same model and firmware, the record still fits, and only the Park re-check at each ignition tests that car's lamps. A car whose VIN does read matches by VIN only.

| Param | Value | What it does |
| --- | --- | --- |
| `TurnPilotCarSignals` | `off` | `off`: the driver signals, prompted, and no 0x3E9 is ever sent. `check`: the lamp check runs on the next ignition, and the comma writes `off` back as soon as it reads it. `on`: the car signals TurnPilot's turns and lane changes, if this car has a verified record. Any other value is `off`, logged as an error. Read once per ignition. |
| `TurnPilotCarSignalsVerified` | written by the comma | The verified record: VIN, firmware, the request reason that lit the lamp, the measured latencies, and when you confirmed the outside lamps. Deleted if the car ever lights the wrong side. |
| `TurnPilotCarSignalsCheck` | written by the comma | The last check's result, pass or fail, with every measurement and the step that failed. |
| `TurnPilotCarSignalsCheckNow` | set by the phone | One-shot "Start check in Drive". Cleared at every ignition. |
| `TurnPilotCarSignalsExterior` | set by the phone | Your answer to the outside-lamp question, for the pending check only. |
| `TurnPilotCarSignalsVinless` | 0 | 1: the owner's opt-in. With the car's VIN unreadable, as on every Tesla, a check records this comma's dongle ID and the car's firmware fingerprint, and a record from a check in Park matches on those and the exact firmware list instead of the VIN. 0: a record matches by VIN only. Read once per ignition. |

In the BeamNG sim the car signals by default: `launch_openpilot.sh` and `run_bridge.py` write `on` and a sim record that the comma accepts only with `SIMULATION=1`. `run_bridge.py --no-car-signals` (with `TURNPILOT_CAR_SIGNALS=0` for `launch_openpilot.sh`) writes `off`.

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

**The stalk signal prompt.** Until the car's own turn signal is verified and on (see "Let the car signal its own turns"), TurnPilot can't signal on any car. It publishes a "signal left" or "signal right" prompt and holds the lane change request until the driver taps the stalk. Then the change goes ahead without a steering nudge, and the request drops after 8 seconds if the blindspot check or the driver doesn't clear it. This keeps signals legal, but a lane change needs a hand on the stalk. The comma screen shows the prompt: "Signal Left to Change Lanes" for a lane change, and "Signal Left" or "Signal Right" for a turn below 20 mph. The suggestion: keep the prompt, and treat lane changes as driver-initiated on the first rungs. The other options are to change lanes without signaling, which is unsafe for the cars around the Tesla and illegal in most places, or to turn off nav lane changes on Tesla.
