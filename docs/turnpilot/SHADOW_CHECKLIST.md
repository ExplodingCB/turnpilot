# Shadow checklist for the first drives

For Chase and whoever rides along. Print it. Install first: [INSTALL.md](INSTALL.md).

Two people on every drive. The driver watches the road and never looks at the phone. The second person watches the phone and writes the notes.

## The car ladder

Climb one rung at a time. Move up when every problem in your notes from the current rung is understood, and Kellen has read them. Drop back a rung after any drive where you had to take over for safety.

| Rung | Where | TurnPilot | Who drives |
| --- | --- | --- | --- |
| 1. Shadow | Ordinary drives, any route | On, shadow mode on | Chase steers by hand. Do not engage openpilot |
| 2. Empty lot | An empty parking lot | On, shadow mode off | openpilot engaged, low speed. The lot tests turns, holds and stops; nav lane changes are off below 20 mph |
| 3. Empty streets | Streets with almost no traffic, such as early on a Sunday | On, shadow mode off | openpilot engaged, second person in the car |
| 4. Indiana routes | Planned routes in real traffic | On, shadow mode off | openpilot engaged, second person in the car |

To change rung, run this on the comma (`cd /data/openpilot` first). It takes effect within a second:

```
python3 -m openpilot.selfdrive.turnpilot.preflight --set TurnPilotShadowMode=0
```

Use `=1` to go back to shadow. Preflight prints a WARN line while shadow mode is on, as a reminder that nothing TurnPilot plans acts. With shadow mode off its line reads `off: TurnPilot is live`.

On rungs 2 to 4, keep your hands on the wheel and a foot over the brake. Hands-off comes after the notes come back clean.

## Before every drive

1. The comma is on the phone's hotspot. The phone is on the same hotspot.
2. Over SSH from a laptop on the hotspot, run `cd /data/openpilot` and then `python3 -m openpilot.selfdrive.turnpilot.preflight`. No FAIL lines. The shadow mode WARN line appears only on rung 1.
3. Open the TurnPilot page on the phone and set the destination. The page shows the next maneuver and its distance.
4. The TurnPilot row on the page shows `(shadow)` on rung 1. If it says Not running, stop and run preflight again. To restart the comma, stop in Park, disengage, then open **Restart the comma** on the page and press **Restart now**. It takes about 2 minutes. The page refuses while openpilot is engaged, out of Park, or during a driver monitoring lockout, and says which.
5. Write down the time the car turned on. Each time it turns on, the comma starts a new route, and that time finds it later.

## What to watch in shadow mode

The comma screen shows the stock navigation map and maneuver banner. TurnPilot's own overlays are not on the comma screen in this build. In shadow mode the evidence is the TurnPilot row on the phone page, which shows what TurnPilot would do, and the log, which the comma saves with the route (`turnpilotPlan`, `turnpilotDetections`, `turnpilotScene`).

The second person watches the page. The TurnPilot row shows one of these:

| The page says | TurnPilot would be |
| --- | --- |
| Following route | Driving the route with no maneuver close |
| Approaching maneuver | Getting ready for a turn or lane change within about 150 m |
| Changing lanes | Asking for a lane change now |
| Turning | Sending the turn now |
| Holding | Slowing or stopping. The Reason line says why |
| No route, Arrived, Off | Not navigating |

The Reason line reads `pedestrian in path`, `pedestrian in crosswalk`, `waiting for oncoming`, `car in target lane`, `lane change timed out`, or a line ending in `take control` when TurnPilot hands the car back. With no detector running it reads `detector not running: take control`, and preflight's `detector pkl` line says whether the detector is installed.

An unprotected left is a left across oncoming traffic with no stop sign of its own, a left on a green ball included. TurnPilot drives each one to its line. It never routes around one or skips it because the camera can't see far enough. The page shows `Stopped at unprotected turn` while TurnPilot waits at the line for a clear gap in oncoming traffic, then Turning. When it can't judge the oncoming road, it hands back at the line with the car held:

- `Can't see oncoming far enough: take control`: the oncoming road is faster than about 35 mph, or has no speed limit on the map.
- `Dark, can't see oncoming far enough: take control`: after dark the camera's reach for oncoming cars is capped at 50 m, so at night a left across any road faster than about 8 mph hands back at its line. Each drive starts out dark until the camera has seen 30 seconds of daylight while moving faster than about 11 mph, so a left early in a day drive can hand back the same way.
- `Oncoming road bends out of view: take control`: the map doesn't show the oncoming road straight far enough.

On rungs 2 to 4, TurnPilot works the car's own turn signal for its turns and lane changes on every drive, with no lamp check to run (`TurnPilotCarSignalsForceOn`, on by default). Note a turn or lane change it signalled late, the wrong way, or not at all. If a signal it asks for doesn't light, the comma says so and asks the driver to signal. On this car the comma reaches the lamps' controller only through a vehicle-CAN tap: xnor's Vehicle CAN Addon Cable at the CAN debug port in the passenger-side A-pillar, behind the right lower A-pillar trim (`car_preflight` calls it X179). The cable is listed for left-hand-drive Model 3/Y built after 2021, so this 2017 car may have no such port (INSTALL.md, "Let the car signal its own turns", cites the sources). On 2026-11-06, without it, no lamp lit and the driver was prompted. Note whether the cable is fitted.

Compare the page with what the driver does, and note every difference:

1. The next maneuver names the street the driver takes. Note a wrong street or a wrong distance.
2. Turning appears at the corner the driver turns at. Note early, late, or at the wrong corner.
3. Changing lanes appears before a turn the driver makes from the wrong lane, toward the right side. Note a missing one, a wrong direction, or one into a lane that wasn't clear.
4. Holding has a real reason behind it. Note a hold with nothing there.
5. The driver brakes or waits for a pedestrian, cyclist or car and the page does not say Holding. Note the miss.

## Traffic lights

The comma reads traffic lights from the road camera (`TurnPilotCameraLights`, `addOnly` by default). A comma still set to the old `shadow` default moves to `addOnly` once, at its next start. Leave it at `addOnly`. On rungs 2 to 4 what the camera reads can stop the car. A green it reads only ends a stop TurnPilot made, and never lifts the driving model's own hold:

- A red, or a yellow the car can still stop for gently, stops the car at a signal on the map, and at the junction of a route turn where the map has no signal.
- Going straight through a signal, the car doesn't slow for a green. The driving model brakes for reds on its own as well.
- A route turn at a signal stops at its line unless the camera reads a steady green first. A steady green read there lets the car go. With no colour read for 5 seconds at the line, the comma says `Turn at a light: take control` and holds the car. On the two night drives of 2026-11-06 the camera read no green in 667 light reads, and no colour at all on any light governing the car's lane. So after dark expect every turn at a light on the map to stop at its line and say `Turn at a light: take control`.
- An unprotected left at a signal goes only on a green the camera reads.

The camera can still miss a red, at night most of all. A red it misses is the driving model's alone, so the driver watches every red and brakes for any the car isn't stopping for. On rung 1, shadow mode, nothing TurnPilot reads acts: the lights are only logged.

At each signal, the second person notes the light's colour as the car reaches it and whether the car stopped. That is what the logs are checked against. A "Light Source Rejected" alert reading "Camera light reader failing: lights unknown" means the reader failed three reads in a row. It shows once per failing spell, again only after the reader has read again, and no more than once a minute. Until a read works again, a camera red stops nothing and only the driving model brakes for reds. The reader keeps retrying, no more than a minute apart, and comes back by itself at its first good read. Note the time.

When checking the logs, a `turnpilot.camera_red_refused` error means the camera read a red at a turn's junction that no stop line took, so the car drove on. That happens when the car had already gone through the junction's mapped line on a go, or when the line wasn't that junction's. Check each one against the second person's note for that light.

## When to take over

On rungs 2 to 4, steer or brake at once when:

- The car starts a turn at the wrong place, or goes toward the wrong lane or street.
- It does not slow for a pedestrian, cyclist or car in or entering its path.
- It moves toward the curb, an oncoming lane or another car.
- It stops somewhere that blocks traffic or sits unsafe, and does not move on.
- The comma shows a red alert you do not understand, or the page shows Not running while you navigate.
- You are unsure about anything.

Taking over is a note, not a failure. Write it down and carry on by hand. If you took over for safety, end the session and call Kellen before the next drive.

## How to note a problem

1. On the comma, swipe left on the road view. It shows "Bookmark Saved", and the moment is stored in the route's log.
2. The second person writes one row, as below. Use the phone's clock to the minute.
3. The route ID is the one comma connect shows for the drive, in the form `<dongle ID>|<date>--<time>`. The comma starts a new route each time the car turns on. Without internet, the route is on the comma under `/data/media/0/realdata`, in a folder named by the same date and time.
4. TurnPilot uploads the drive to Kellen's server by itself whenever the comma is online, the phone's hotspot included: every segment's small log first, then the road videos, then the full logs, newest first. Video of segments older than 48 hours stays on the comma. Comma connect gets comma's own uploads as before. Send Kellen the notes with the route IDs.

| # | Time | Route ID | Where (street, cross street) | Rung | The page said (state, reason) | What the driver did |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |
| 4 | | | | | | |
| 5 | | | | | | |
| 6 | | | | | | |

## After every drive

1. Home on Wi-Fi, run preflight once more, and note any FAIL.
2. Check that the routes uploaded in comma connect. TurnPilot's own uploads to Kellen's server need nothing from you.
3. Send Kellen the notes, with the rung and the route IDs.
4. If you will not update before the next drive, set `DisableUpdates=1` as described in [INSTALL.md](INSTALL.md#update). After an update there is nothing to flash by hand: the comma reflashes the panda by itself at the next start when the update changed its firmware.
