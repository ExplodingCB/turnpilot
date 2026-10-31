# Install TurnPilot on a comma four

This guide takes a comma four with a chestnut eGPU from sunnypilot or stock openpilot to TurnPilot, ready to drive. It is written for Chase's car, a 2017 Model 3 with HW3. "Install like stock" is the whole install as Chase does it, with the comma's screen, his phone and the map key Kellen texts him. Everything after it is for Kellen or a developer with SSH. The drives are in [SHADOW_CHECKLIST.md](SHADOW_CHECKLIST.md).

## Install like stock

TurnPilot installs the way sunnypilot does: the comma's own setup screens, one URL, then TurnPilot's own setup screens through to "ready to drive". There is no SSH, laptop or document in it.

The numbers in brackets name screenshots of the real comma screens. `python -m openpilot.selfdrive.ui.tests.turnpilot_setup_screenshots <dir>` draws them; Kellen's copy is in `turnpilot-artifacts/phase4/night/onboarding_shots`, with the phone page's three shots beside them (`phone_1` to `phone_3`). The stock screens before the terms page aren't drawn there, so their words below come from the stock setup code (`openpilot/system/ui/mici_setup.py` and `mici_updater.py`).

### What you need

- The comma four with the chestnut attached, mounted in the car on its harness. The chestnut occupies the comma's USB-C port, so the car powers the comma. Keep the car awake from the first step to "ready to drive", for example with the climate kept on. The downloads are about 1 GB for the comma's OS update and more than 800 MB for TurnPilot, and nobody has timed the first compile on a comma four.
- Home Wi-Fi that reaches the parked car.
- A phone that can join that Wi-Fi, and later share its own hotspot with the comma (iPhone: Personal Hotspot).
- The map key, which Kellen texts as a message of its own.
- A Model 3 whose software is new enough for this openpilot. opendbc treats a Tesla without the 3-bit steering firmware as dashcam only, and its comment dates that firmware to 2026.8.6 on HW3. Chase's car ran sunnypilot `staging-chestnut`, which has the same requirement.

### Remove sunnypilot

1. On the comma, open sunnypilot's settings, go to **Software**, and uninstall. These are sunnypilot's screens, which this guide hasn't checked. Stock openpilot has **uninstall openpilot** at the end of **Settings > Software**: tap it, then slide to uninstall.
2. Stock openpilot's uninstall restarts the comma, erases `/data` ("resetting device") and opens the stock setup. sunnypilot is built on it and is expected to do the same. The erase takes sunnypilot, its settings, the camera calibration and the recorded drives, so upload any drive you want to keep first.

### Install from the stock setup

1. Tap **start**.
2. Slide **slide to install custom software**, the lower of the two sliders.
3. Swipe through "caution: installing 3rd party software" and tap **next**.
4. On "connect to internet", tap the Wi-Fi card, pick the home network, and enter its password. The pill reads "waiting for internet..." until the comma is online, then turns into **choose software**. Tap it.
5. Type `kaileh57/turnpilot` and confirm. Setup turns the short form into `installer.comma.ai/kaileh57/turnpilot`, and typing the full URL works too.
6. The screen shows "downloading..." while it fetches comma's installer, then "installing..." with a percentage while the installer clones TurnPilot and pulls the 776 MB driving model, then "finishing setup". The comma restarts.

If the URL is mistyped, setup shows "download failed" with the reason, such as "No custom software found at this URL". Swipe down and type it again.

### First boot

1. The comma asks to update its operating system: "update required", "the download size is approximately 1 GB". TurnPilot needs AGNOS 20.1, and sunnypilot `staging-chestnut` at the commit Chase's car ran (820eaf287d) uses 19.9, so expect this screen. Tap **next**, check the Wi-Fi, and tap **download & install**. The comma downloads, installs and restarts. A comma already on 20.1 skips this screen.
2. The comma compiles TurnPilot behind a spinning comma logo with a progress bar, and then flashes the panda with this build's firmware. Don't let the car sleep. If the compile fails, the screen shows the compiler's output: send Kellen a photo of it.

### Stock terms and training

1. "terms and conditions" [01]: swipe through and slide **accept terms**.
2. The stock training: "what is openpilot?" with its four rules, then **next**; "driver monitoring check", where the comma's camera has to see your face; and "cabin camera data", where you slide **allow data uploading** or **no, don't upload**.

### TurnPilot setup

TurnPilot setup opens by itself when the training ends. It uses the stock cards and buttons: each step is a row of cards that you swipe left and right, and the green pill at the end moves on. A step that is still waiting shows a dimmed pill. A step that fails says what to do and adds **retry**. Every step but the last ends with **set up later** (the X): tap it and slide to confirm, and setup closes until the next boot.

1. **what is turnpilot?** [02-04]. "TurnPilot drives your navigation route: the turns, the lane changes and the stops." Tap **next**. This also fills in the map server's address and certificate (`maps/defaults.py`), so only the key is left to enter.
2. **pair your phone** [05-10]. With the phone on the same Wi-Fi as the comma, scan the QR code with the phone camera and open the link. It opens the TurnPilot page in the phone's browser, at an address like `http://192.168.1.68:8088/?t=...` [07], which can also be typed by hand. The part after `?t=` is the comma's password, so keep it to yourself. The pill reads "waiting for phone..." [08] and turns into a green **next** once the page is open [09], with "phone connected" on the status card [10]. Optional: in Safari, tap Share, then Add to Home Screen.
3. **add the map key** [11-18]. "Open the TurnPilot page on your phone and paste the map key Kellen sent you." [12]. Copy Kellen's text, paste it in the page's **Map key** box ("Paste the map key here" [phone_1]) and tap **Save key**. The page answers "Key works. TurnPilot can drive around Indianapolis and Seattle. Now tap next on the comma." [phone_3], and the comma's card reads "map key works" with the mapped area [17]. Tap **next** [18]. If the comma reads "key not accepted" [14] and the page "That key didn't work" [phone_2], copy the whole text again and paste it in place of the first try. "map server offline" [16] means the comma can't reach the server: check the comma's Wi-Fi, then tap **retry** [15].
4. **map engine** [19-25]. The comma installs its map engine (pyvalhalla, a 35 MB download) by itself whenever it is online, so this is often done before the step opens. The card shows "installing" with a percentage [20], then "map engine installed" [24]; tap **next**. "waiting for internet" [21] goes on by itself once the comma is online. "install failed" [22] gives the reason, such as "storage full" (it needs 300 MB free): tap **retry** [23].
5. **car turn signals**, optional [26-42]. Tap **skip** [29], the default. With car signals off, the comma chimes "Signal Left" or "Signal Right" at a turn below 20 mph, and you tap the stalk that way. The check lets the car work its own turn signal, and it can be run on any later day from **Settings > turnpilot > run setup again**. It needs the car parked in P (the **check signals** button [30] is dimmed otherwise) and someone outside to watch the lamps:
   1. Tap **check signals**. Park in P with room on both sides [32].
   2. Turn the car off and on, with hands off the stalk [33]. The car blinks each side.
   3. Step out and watch each side's lamps [34], then answer on the phone whether they flashed [35, 37]. The page's **Car turn signal** section opens the question by itself.
   4. If they flashed, try the wipers, the headlights, the high beams and both stalk signals [38]. Tap **they all work** [39], then turn the car off and on once more [41]. The car signals its own turns from then on. **something's wrong** [40], or any failed check, turns car signals off again [42].
6. **ready to drive** [43-49]. "1. Drive straight above 15 mph for a few minutes. The camera calibrates." "2. Engage with the cruise stalk." "3. Enter a destination on the phone." "TurnPilot starts live: it drives the route." [47]. With car signals off it adds the stalk line [48]. Tap **done** [49].

Setup opens again at every boot, at the first step that isn't finished, until **done** is tapped. While anything is missing, the home screen counts one more alert [61], "TurnPilot is not ready to drive", which says what is missing and where to fix it [60]. It goes away once nothing is missing [62]. A fresh install also shows the stock "Finish setup. Pair your device with comma connect" alert, which TurnPilot doesn't need.

**Settings > turnpilot** [50-57] has **pair phone**, **watch only** (off: live), **always drive**, **stop-sign go** and **far pedestrians** (the last three on), the map engine's state, and **run setup again**.

### First drive

Setup stays up while the car is parked with the comma on, and closes once the car moves.

1. In the car the comma needs internet for each drive's map, and the phone has to reach the comma. Turn on the phone's hotspot, then on the comma open **Settings > Network** and join it. The comma remembers it for later drives.
2. The page's address changes on the hotspot. Open **Settings > turnpilot > pair phone** [51] and scan the QR code again.
3. When the car turns on, the comma shows "WARNING: This branch is not tested". That is expected on this branch.
4. Drive straight above 15 mph for a few minutes, so the camera calibrates. The uninstall erased sunnypilot's calibration.
5. Engage with the cruise stalk, then search for a destination on the phone page and tap Navigate. Place search needs the hotspot's internet.

### Go back to sunnypilot

1. On the comma, open **Settings > Software**, tap **uninstall openpilot**, and slide to uninstall. The comma erases `/data`, including TurnPilot's settings, pairing and map key, and opens the stock setup.
2. Follow "Install from the stock setup" with sunnypilot's URL in step 5: `sunnypilot/staging-chestnut`, the branch Chase's car ran (at 820eaf287d on 2026-10-31), or `sunnypilot/release-mici`, sunnypilot's release for the comma four. Neither has been tried from this guide, and sunnypilot's own instructions may give another URL.
3. sunnypilot `staging-chestnut` uses AGNOS 19.9, so its first boot may show the OS update screen again.

To come back to TurnPilot later, start again at "Remove sunnypilot". The map key has to be entered again.

### What was checked

From a Mac on 2026-10-31, at the merge of the guided setup and the defaults (b8a4fbe on `tp/sat-setup`): the 253 git-lfs files are the same set of objects as at 263065a, every one of which is on the `.lfsconfig` server (comma's Hugging Face LFS repo); `github.com/kaileh57/openpilot.git` redirects to this repo; and the installer's own commands from `openpilot/selfdrive/ui/installer/installer.cc` (`git clone --depth=1 --recurse-submodules`, `checkout`, `reset --hard`, `submodule update --init`) with no GitHub login leave every submodule at its gitlink from its public URL and no LFS pointer anywhere. The big driving model's sha256 matches its pointer. The TurnPilot screens are the real UI code drawn on the Mac.

Not checked, because only a comma or the car can show it:

- that AGNOS's git runs git-lfs during the installer's clone. Stock openpilot's own LFS files depend on it too.
- that sunnypilot's uninstall erases `/data` as stock's does. The defaults land only on params that are unset, so a comma that kept sunnypilot's params keeps its old values.
- the OS update from 19.9 to 20.1, the free space on `/data`, and how long the first compile takes.
- the map engine's self-install on the comma, "phone connected" over a real hotspot, setup staying up with the car on and closing when it moves, and the turn signal check across real ignition cycles.

## Install from the Mac (fallback)

The comma's own installer ("Install like stock" above) is the install path. Every submodule in `.gitmodules` is public, `github.com/kaileh57/turnpilot-opendbc` included, so the installer's unauthenticated clone gets them all, and every git-lfs file the device reads is on the `.lfsconfig` server (comma's Hugging Face LFS repo). Use this section only when the installer can't be used, in place of "Remove sunnypilot" and "Install from the stock setup". It needs SSH on the comma ("Turn on SSH" below), which takes the comma's screen and a GitHub username. It builds the tree on Kellen's Mac and copies it over SSH, and the comma fetches nothing from GitHub.

With `--map-key` the script also writes the map server's API key from the Mac's `~/turnpilot_data/mapserver/client/key` into the comma's `TurnPilotMapKey`. `--map-key-only` writes only the key, onroad or off, and changes nothing else. The key travels over SSH's standard input, never on a command line, and the script never prints it. Without either flag it never opens the key file.

`openpilot/tools/turnpilot/deploy_to_comma.py` does the whole job and prints each step. It runs with the Mac's `python3` and needs `git-lfs` on the Mac.

1. It reads the comma's state over SSH and changes nothing. It refuses if the comma is onroad (`IsOffroad` is 0, `IsOnroad` is 1, or neither is set), if SSH fails, or if `/data` has less free space than the tree, plus a copy of the LFS files that git-lfs on the comma makes, plus 3 GB for the first build (about 5.3 GB in all).
2. It builds the tree in a temporary directory from the Mac's own clones: the commit on branch `turnpilot`, tracking `origin/turnpilot` (origin is `https://github.com/kaileh57/turnpilot.git`, which nothing fetches while `DisableUpdates` is 1), depth 1. Each submodule is at its gitlink, from `.git/modules/<name>`. A submodule commit missing there comes from the public URL in `.gitmodules`; the opendbc commit is on the Mac. Every git-lfs file (the driving models, the fonts, the AGNOS updater) is its real bytes, checked against its sha256. No build output from the Mac goes, because the tree comes from git objects, not from a working copy.
3. It copies the tree to `/data/openpilot.tp-incoming-<stamp>`, then checks it on the comma: every file's sha256 against the Mac's manifest, the file count, the symlinks, `HEAD`, the branch and its upstream, and each submodule's commit. It runs the same git commands as `is_dirty()` in `openpilot/common/version.py`, which also makes git-lfs hash the model files once, so the first boot doesn't. The tree's own git config sets git-lfs's clean filter (`filter.lfs.clean`, `filter.lfs.required`), so this check and the device's `is_dirty()` don't depend on how AGNOS configures git for user `comma`. It sets no smudge filter, so checkouts keep whatever AGNOS gives them.
4. It checks again that the comma is offroad, renames `/data/openpilot` to `/data/openpilot.prev-<stamp>`, and moves the new tree to `/data/openpilot`. The comma does both moves even if the SSH connection drops between them.
5. It sets `DisableUpdates` to 1, so the updater doesn't try GitHub, and asks before it reboots the comma. This differs from the installer path, where updates stay on (the default, 0) so a pushed fix can reach the car; whether the script should leave them on too is Kellen's call, and until then set `DisableUpdates=0` after a Mac deploy to get updates ("Update" below). Before the reboot, the comma checks again that it is offroad, that `DisableUpdates` is 1, and that `/data/openpilot/.overlay_init` doesn't exist. Sunnypilot's updater keeps running until the reboot, and it creates that file when it sets up its overlay over the new tree. With the file there, the next boot would install any update the updater stages from GitHub over the deployed build.

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

The comma boots `/data/openpilot` the way sunnypilot's tree booted (`/data/continue.sh` starts `launch_openpilot.sh`). The branch has no `prebuilt` file, so the first boot compiles openpilot from source, with a progress bar, and then pandad flashes the panda with this build's debug firmware ("What the branch name does" explains why). Nobody has timed the compile on this branch. Keep the comma powered until the road view shows. The tree has no `.overlay_init`, so the launcher doesn't install an update that sunnypilot left in `/data/safe_staging`. If sunnypilot's updater creates one after the swap, the script refuses to reboot. The params carry over from sunnypilot, so the stock terms and training show only if their versions are new to this comma; TurnPilot setup opens either way, as in "Install like stock", and with `--map-key` its key step only waits for the map server's answer. SSH stays on, because the settings carry over.

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

## Work over SSH

The rest of this guide is for Kellen or a developer. Run every command on the comma from `/data/openpilot` over SSH, unless the step says it runs on your computer. Besides what "Install like stock" needs, you need:

- A GitHub account with an SSH key. The comma fetches the public keys from GitHub.
- A computer with `ssh` and `scp`. For the sim's local routing, the Indiana tile file `tiles.tar` (390 MB): `tools/turnpilot/build_valhalla_tiles.sh` builds it, and it is in `~/turnpilot_data/valhalla/indiana/` on Kellen's Mac.

TurnPilot setup never opens while `TurnPilotValhallaTiles` is set, as in the sim, since local tiles need no map server and no phone.

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

In this fork the toggles below are on by default, along with `TurnPilotEnabled` and the rest of Saturday's params, so a fresh install needs none of this section: the manager writes each param's default at boot when it is unset, and uninstalling openpilot clears every param. A device that kept its params from an earlier install keeps the values it had; turn them on there.

On the comma screen, in **Settings > Developer**:

- Turn on **alpha longitudinal** and confirm. The comma cycles its driving processes to apply it. Without it, openpilot does not control speed on the Tesla, and TurnPilot's stops and speed caps do nothing.
- Turn on **navigation (alpha)**. This is the `NavigationEnabled` param. It starts navd, which routes, and mapsd, which draws the map.
- Turn on **Disengage on accelerator pedal**. This is the `DisengageOnAccelerator` param. Any gas press must disengage, because TurnPilot never takes a gas press as its go. A device that had this toggle off for an earlier TurnPilot keeps it off until you turn it on, and while it is off TurnPilot shows "TurnPilot Unavailable: turn on Disengage on accelerator pedal" and does nothing. Preflight's `gas disengages` line fails while it is off.

TurnPilot is on and live by default. For a drive in shadow mode, set it over SSH. The stock params tool can't write boolean params, so use preflight:

```
python3 -m openpilot.selfdrive.turnpilot.preflight --set TurnPilotShadowMode=1
```

| Param | Value | What it does |
| --- | --- | --- |
| `TurnPilotEnabled` | 1 | Starts `turnpilot_web` (the phone app) within a few seconds, makes `turnpilotd` produce a plan, and makes modeld load the detector the next time the car turns on. |
| `TurnPilotShadowMode` | 0 | 1: TurnPilot computes the plan and publishes it marked as shadow. modeld and the planner ignore it, so the car drives as stock openpilot. 0, the default: live. |
| `TurnPilotMapHost` | `https://<ip>` | TurnPilot's map server. Set it in the phone app's Map server section. |
| `TurnPilotMapKey` | the API key | Sent as a Bearer token. Set it in the phone app, which never shows it again (only its last 4 characters). |
| `TurnPilotMapCert` | PEM | The server's self-signed certificate. Every request is verified against it. |
| `TurnPilotValhallaTiles` | unset | Developer and sim override (BeamNG sets it): route on these local tiles, with no geofence and no download. Leave it unset in the car. |
| `TurnPilotRouteSource` | `auto` | Left over from Mapbox routing. `mapbox` is now an error; anything else is ignored. |
| `TurnPilotWebToken` | generated | The phone's password. `turnpilot_web` creates it on its first run. |

## Let the car signal its own turns

On a Tesla, TurnPilot can work the car's own turn signal through `DAS_bodyControls` (0x3E9). It is off by default, and it stays off on a car until a lamp check passes on that car and you confirm on the phone that the outside lamps flashed. Until then, and whenever the car's signal fails, every turn below 20 mph prompts "Signal Left" or "Signal Right" and the driver signals. The panda allows the message only in a debug build, the build this branch already flashes.

1. On the phone page's **Car turn signal** section, set the mode to **check**, then turn the car off and on. In Park, with hands off the stalk, the check runs by itself after 3 s: it blinks each side, says each step, and takes 75 to 110 s. If Park gives no lamp, set **check** again, restart, stop in Drive with the brake held, and tap **Start check in Drive**. The page shows that button only with `&dev=1` added to its address; TurnPilot setup runs the check in Park only.
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

## Let the car go by itself at stop signs

By default (`vision`) the car goes by itself from what the cameras see live. (`auto` acts only on a camera fit and a junction survey, neither of which exists for this car yet, so under `auto` every stop sign and yield hands back after a 1 s full stop.) Under `vision` the car stops fully at the line, looks for 2.5 s with every view fresh and nothing in a conflicting lane, chimes "Going", and goes. Where a cross road is over 25 mph, or its speed isn't known, it looks for 4 s and says "Look both ways" first, so check both ways yourself. If you brake or press the gas at the line and then re-engage, it still says "Going" (or "Look both ways") for 1 s before it goes, never going the moment you re-engage. It hands back after 1 s, saying why, at night, at dusk, in low sun, with the sun ahead, with the side cameras off, without the stop-sign map data, and at a divided road, a roundabout or a rail crossing.

To have every stop sign and yield hand back instead (Saturday's fallback when preflight's `stop-sign go` line warns):

```
python3 -m openpilot.selfdrive.turnpilot.preflight --set TurnPilotStopSignGo=off
```

| Param | Value | What it does |
| --- | --- | --- |
| `TurnPilotStopSignGo` | `vision` | `off` and `shadow`: every stop sign and yield hands back after 1 s. `auto` and `trial`: the go only at a surveyed, validated junction with the camera fit, so on this car they act as `shadow`. `vision`: the go from the live cameras (above). Any other value acts as `shadow`, logged. Read every second. |
| `TurnPilotAvoidStops` | 1 | navd routes around the stops and yields TurnPilot can't take. Under `vision` it keeps every one vision can take on the route, and avoids the rest (the ones it would hand back). Off, every stop stays on the route and the ones vision can't take hand back at the line. |

## Install Valhalla for offline routing

Driving needs pyvalhalla even with the map server: navd routes each drive's corridor with it on the comma. The system Python on the comma is read-only, so pyvalhalla goes into `/data/turnpilot/pyvalhalla`, which survives updates.

`turnpilot_web` installs it by itself (`valhalla_install.py`): once a boot while it is missing and the comma is online, again every 30 s while offline, and again when you tap `retry` in setup or in **Settings > turnpilot > map engine**. It runs at the lowest CPU priority and never holds up boot or a drive. It uses the same wheel and sha256 as the script below, and preflight's `pyvalhalla` line shows how it is going.

To install it by hand instead, the comma needs internet once for the 35 MB wheel, from home Wi-Fi or the hotspot:

```
bash tools/turnpilot/install_valhalla_device.sh
```

The script checks the wheel's sha256, unpacks it, and imports it as a test. It needs 300 MB free. For an install with no internet on the comma, copy the wheel over with `scp` and run `PYVALHALLA_WHEEL=/path/to/wheel bash tools/turnpilot/install_valhalla_device.sh`.

## Set up the map server

TurnPilot routes only inside its mapped areas, around Indianapolis and Seattle. There is no Mapbox and no fallback.

1. The comma already has the map server's address and certificate. Open the phone app and paste the API key in **Map key**, then tap **Save key**. To use another server, open **Map server**, enter the address, the API key and the server's certificate (PEM) together, and tap **Save and check**. The comma fetches the mapped area at once, so the comma must be online. It then refreshes it every 6 hours while online and keeps it in `/data/turnpilot/maps/coverage.json`.
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

## Let TurnPilot drive on where it is unsure

`TurnPilotAlwaysDrive` is the always-drive switch. On, the default, TurnPilot drives on where it is only unsure, and hands back, loudly, only where it made a mistake. Off, it hands back, skips or waits wherever it is unsure. It does nothing in shadow mode. Its default comes with the no-move limit Saturday uses, 20 s. To turn it off, parked:

```
python3 -m openpilot.selfdrive.turnpilot.preflight --set TurnPilotAlwaysDrive=0 --set TurnPilotNoMoveS=2
```

| Param | Value | What it does |
| --- | --- | --- |
| `TurnPilotAlwaysDrive` | 1 | 1: always drive (above). turnpilotd reads it every second, and navd reads it each time it routes. |
| `TurnPilotNoMoveS` | 20 | Seconds a turn TurnPilot let go of at a standstill waits for the car to pull away before "Can't start the turn: take control", 2 to 20. An unprotected left's 1 s is fixed. Read every second. |

The switch doesn't set or change `TurnPilotStopSignGo`. The car goes by itself at stop signs and yields only with `TurnPilotStopSignGo=vision`, which you set on its own (see "Let the car go by itself at stop signs"). With the switch on, every stop-sign mode, `vision` included, stops, waits, goes and hands back exactly as it does with the switch off, and navd keeps and avoids the same stops.

Whatever the switch says, a lane change the turn needs that hasn't happened by 60 to 120 m before the turn skips the turn ("Can't Reach Turn Lane", "Rerouting"), and navd reroutes. The turn isn't made from the wrong lane. openpilot changes no lanes under 20 mph, so in slow traffic every turn that needs a lane change is skipped.

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
| `always drive` | An INFO line saying whether `TurnPilotAlwaysDrive` is on. It warns while TurnPilot is live with it off, and when the build's params don't know it |
| `map server` | The address, key and certificate are set and the mapped area has been fetched |
| `pyvalhalla`, `valhalla tiles`, `routing test` | pyvalhalla imports; with `TurnPilotValhallaTiles` set, those tiles are usable and a real route comes back |
| `big driving model` | The 775 MB pickle and both warps are real files, not LFS pointers. If they are pointers, modeld quietly runs the small model |
| `detector pkl` | `big_detector_1344x760_tinygrad.pkl` exists. Without it the planner holds every left turn |
| `stop-sign go` | An INFO line saying what `TurnPilotStopSignGo` does on this build. It warns under `vision` when the side-view pkl is missing or a pointer (every stop hands back) or `TurnPilotAvoidStops` is off, and under `auto` or `trial` without the camera fit (they act as `shadow`) |
| `chestnut` | The chestnut is present with its firmware and a USB 3 link |
| `web token`, `pairing URL`, `web server` | The token exists, the comma has a network address, and `turnpilot_web` answers on port 8088 |

`shadow mode` and `route source` are INFO lines. `shadow mode` warns when it is on, since TurnPilot then drives nothing. `route source` warns when `TurnPilotValhallaTiles` is set, since the car should use the map server.

## Update

The comma pulls from the `turnpilot` branch of `github.com/kaileh57/openpilot`, so an update is whatever has been pushed there.

1. Join home Wi-Fi.
2. Open **Settings > Software** and tap **check for update**. If it offers **download update**, tap that. When the download finishes, **install now** appears (the car must be off). The comma restarts and recompiles, which takes several minutes.
3. Run preflight.

Updates are on, as in stock openpilot (`DisableUpdates` 0, the default since SATURDAY_PARAMS.md's 11:50 change), so a fix pushed to the `turnpilot` branch reaches the comma through **Settings > Software** with no SSH. The updater applies a downloaded update at the next boot, and the comma boots every time the car wakes. To keep a build fixed for a test day, turn updates off over SSH and back on afterwards:

```
python3 -m openpilot.selfdrive.turnpilot.preflight --set DisableUpdates=1
python3 -m openpilot.selfdrive.turnpilot.preflight --set DisableUpdates=0
```

A comma installed with the Mac script ("Install from the Mac") has `DisableUpdates` 1, which that script sets, so **check for update** finds nothing there until it is set to 0. The files in `/data/turnpilot` (pyvalhalla and the maps) stay through updates, but not through an uninstall.

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
