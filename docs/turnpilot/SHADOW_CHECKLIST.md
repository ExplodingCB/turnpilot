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

Use `=1` to go back to shadow. Preflight prints a WARN line while shadow mode is off, as a reminder that TurnPilot is live.

On rungs 2 to 4, keep your hands on the wheel and a foot over the brake. Hands-off comes after the notes come back clean.

## Before every drive

1. The comma is on the phone's hotspot. The phone is on the same hotspot.
2. Over SSH from a laptop on the hotspot, run `cd /data/openpilot` and then `python3 -m openpilot.selfdrive.turnpilot.preflight`. No FAIL lines. The WARN line appears only on rungs 2 to 4.
3. Open the TurnPilot page on the phone and set the destination. The page shows the next maneuver and its distance.
4. The TurnPilot row on the page shows `(shadow)` on rung 1. If it says Not running, stop and run preflight again.
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

The Reason line reads `pedestrian in path`, `pedestrian in crosswalk`, `waiting for oncoming`, `take control` under `Stopped at unprotected turn` (every left across a road ahead with no stop sign of its own, and a left on a green ball), `car in target lane`, `lane change timed out`, or `detector unavailable`. With no detector running, every left turn reads Holding with `detector unavailable`. That is expected until the detector is installed, and preflight's `detector pkl` line says whether it is.

Compare the page with what the driver does, and note every difference:

1. The next maneuver names the street the driver takes. Note a wrong street or a wrong distance.
2. Turning appears at the corner the driver turns at. Note early, late, or at the wrong corner.
3. Changing lanes appears before a turn the driver makes from the wrong lane, toward the right side. Note a missing one, a wrong direction, or one into a lane that wasn't clear.
4. Holding has a real reason behind it. Note a hold with nothing there.
5. The driver brakes or waits for a pedestrian, cyclist or car and the page does not say Holding. Note the miss.

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
4. Back at home on Wi-Fi, let the routes upload, and send Kellen the notes with the route IDs.

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
2. Check that the routes uploaded in comma connect.
3. Send Kellen the notes, with the rung and the route IDs.
4. If you will not update before the next drive, set `DisableUpdates=1` as described in [INSTALL.md](INSTALL.md#update).
