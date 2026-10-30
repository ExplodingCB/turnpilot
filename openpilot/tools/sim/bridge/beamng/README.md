# BeamNG.tech bridge

openpilot on the Mac drives a car in stock BeamNG.tech on the Windows PC, as a comma four in a Tesla Model 3. The bridge
runs on the Mac and talks to the game over BeamNGpy 1.36 at `192.168.137.1:25252`. It loads `west_coast_usa` with one
ETK 800 (the stock sedan closest to a Model 3: 4.67 m long and 1.39 m tall, against 4.69 m and 1.44 m) and publishes:

- `can` and `pandaStates` for a Model 3 on 3-bit DAS firmware, including `DI_autonomyHealth` (0x054) on bus 0, so card
  fingerprints `TESLA_MODEL_3` with `dashcamOnly` false and alpha longitudinal
- road and wide frames on VisionIPC at 1344x760 with `narrowRoadCameraState` and `wideRoadCameraState` (`sensor` os04c10)
- `accelerometer` and `gyroscope` at 100 Hz, `gpsLocationExternal` at 10 Hz (with a fix, in `frame.py`'s projection),
  and upstream's fake driver monitoring and peripheral state

openpilot's own `DAS_steeringControl` and `DAS_control` come back from `sendcan`, after openpilot's stock Tesla limits,
and drive the game car. The steering angle goes through the Model 3's 12:1 ratio and the ETK 800's 38 degree road-wheel
lock. The acceleration goes through `control.py`, a feedforward plus a PI controller that holds the car's speed on the
integral of the requested acceleration. It doesn't use the game's measured acceleration, which arrives in 100 ms batches
and bounces with the suspension.

The bridge never closes the game. On exit it removes its sensors, brakes the car and disconnects, and the scenario stays
loaded so the next start takes seconds instead of a minute. It does leave the game's global state as it set it (see
[Known limits](#known-limits)) unless you pass `--restore-on-exit`.

## Requirements

- BeamNGpy 1.36 in the repo venv. It isn't in `pyproject.toml`:

  ```sh
  uv pip install --python /Users/kaileh57/Projects/turnpilot/.venv/bin/python beamngpy==1.36
  ```

  `cameras.py` reads camera frames through BeamNGpy's private `connection._pack_data` and `connection.skt`, so keep to
  1.36 or check those two before upgrading.
- An openpilot checkout with the `("pc", "os04c10")` camera config in `common/transformations/camera.py` (turnpilot
  commit 353f4d3), since the bridge publishes os04c10 frames at 1344x760.
- `ffmpeg` on `PATH`, for `--capture` only.

## Run it

Use the repo venv, from the repo root. `OPENPILOT_PREFIX` keeps msgq, params and logs apart from other runs on the Mac.

1. Start openpilot as a comma four in a Model 3:

   ```sh
   OPENPILOT_PREFIX=tpsim openpilot/tools/sim/bridge/beamng/launch_openpilot.sh
   ```

   It is upstream's sim launch with `FINGERPRINT=TESLA_MODEL_3`, alpha longitudinal on, and loggerd and logmessaged kept, so
   every run leaves an rlog in `~/.commatpsim/media/0/realdata` with cloudlog's records in it as `logMessage` (TurnPilot's
   reasons, such as `turnpilot.go_not_certified`). Add more processes to block with `BLOCK=...`; blocking loggerd blocks
   logmessaged with it.

2. In a second terminal, start the bridge:

   ```sh
   OPENPILOT_PREFIX=tpsim python -m openpilot.tools.sim.bridge.beamng.run_bridge --engage --rolling-start 20 --set-speed 30
   ```

   `--engage` pulls the cruise stalk as soon as openpilot is engageable. With `--rolling-start`, the bridge first plays
   a driver who presses the gas up to that speed (km/h): the driving model holds a car that stands on an empty road,
   because its standstill acceleration stays under the 0.1 m/s^2 of `drive_helpers.should_stop`.

modeld doesn't reconnect to a new VisionIPC server. Restart openpilot (or at least modeld) whenever you restart the
bridge, or modeld keeps waiting on the old one.

Other options: `--spawn x,y,z --rot qx,qy,qz,qw` (BeamNG world meters; the default is 140 m before the junction
`group_80_81_82_83`, eastbound in the right-hand lane), `--reload`, `--signals` (leave the traffic lights running; by
default every light is forced green and held), `--capture chase.mp4`, `--duration`, `--restore-on-exit`, and
`--camera-update` and `--camera-poll` for the camera timing.

Without a TTY, the bridge reads one command per line on stdin: `cruise_up` (engage, or raise the set speed by 5 km/h),
`cruise_down`, `cruise_cancel`, `blinker_left`, `blinker_right`, `throttle_0.5`, `brake_1`, `steer_-0.2`, `reset` and
`quit`. At the end of stdin (`< /dev/null`, `nohup`, a closed pipe) it keeps running until `quit`, Ctrl-C, SIGTERM or
`--duration`, as with `--no-keyboard`. With a TTY, the keys are upstream's (`tools/sim/README.md`).

## Gate 1

`gate1.py` checks whether one desire pulse makes the driving model take a 90-degree junction. It spawns the car on the
approach to a junction from the road-graph export, engages, and publishes `turnpilotPlan` at 20 Hz with the desire set
for `--pulse-frames` messages once the car is `--leads` meters from the junction. It logs the heading change, the path
and whether the car left on the exit road, to `--out`. Block turnpilotd, the other `turnpilotPlan` publisher:

```sh
BLOCK=turnpilotd OPENPILOT_PREFIX=tpsim openpilot/tools/sim/bridge/beamng/launch_openpilot.sh
OPENPILOT_PREFIX=tpsim python -m openpilot.tools.sim.bridge.beamng.gate1 --junction group_80_81_82_83 --turn left --leads 10,20,30,40
```

`--list` prints the signalized junctions with legal 90-degree turns. The road graph is read from
`~/pc_relay/incoming/osm_export/west_coast_usa_graph.json`.

Gate 1 passed on 2026-10-27 with the big model (`big_driving_tinygrad.pkl`, LFS oid `f0667cf6`, Flip Flop) on the
chestnut, the detector running beside it, 20 camera pairs a second, 25 km/h and a 4-message `turnLeft`/`turnRight` pulse:

| Junction | Turn | Lead | Taken |
| --- | --- | --- | --- |
| `group_80_81_82_83` | left | 10 / 20 / 30 / 40 m | 0/2, 1/2, 2/2, 2/2 |
| `group_80_81_82_83` | right | 10 / 20 / 30 / 40 m | 0/2, 0/2, 2/2, 2/2 |
| `group_46_47_48_49` | left | 40 m | 5/5, heading change 89.0 to 89.8 degrees |
| `group_46_47_48_49` | right | 40 m | 5/5, heading change -89.5 to -91.4 degrees |

At 10 m the car goes straight; at 20 m it turns late and clips the corner. With `--capture` (about 12 pairs a second)
the same right turn at `group_46_47_48_49` was taken once in three runs: run gates at full camera rate.

## Gate 3

`gate3.py` stands the Italy mannequin in the ego's lane `--ped-ahead` m ahead on a junction's approach, engages at
`--speed` and logs at 10 Hz what the driving model (`action.shouldStop`, `desiredAcceleration`, its lead), the detector
(person boxes and confidence by range), the scene (`turnpilotScene` pedestrians, `inPath`) and turnpilotd's plan did, and
the final gap from the camera and from the front bumper. It removes the mannequin after every run. For the model alone,
block turnpilotd and keep `TurnPilotEnabled` set, so modeld still runs the detector and nothing publishes `turnpilotPlan`:

```sh
BLOCK=turnpilotd OPENPILOT_PREFIX=tpsim openpilot/tools/sim/bridge/beamng/launch_openpilot.sh
OPENPILOT_PREFIX=tpsim python -m openpilot.tools.sim.bridge.beamng.gate3 --runs 2 --speed 35 --out gate3.json
```

The road graph's lane centre can be a parking lane away from where the model drives, so once the car drives, gate3
moves the mannequin sideways onto the car's own line at 90 m, beyond the detector's range. Results on 2026-10-27, at
`group_80_81_82_83`, the mannequin 115 m ahead, 35 km/h, two runs each:

- The big model alone never set `shouldStop`. Once it barely braked and drifted 1 m right over the last 30 m, so the
  left front corner clipped the mannequin at 25 km/h; once it braked from 28 m and touched it at about 2.5 km/h. It
  does see a lead there, at the wrong range.
- YOLO26n sees the mannequin as a person from 47 to 60 m on the road camera, confidence 0.5 to 0.85, and loses it
  closer than 5.3 m from the camera.
- With turnpilotd and no route, the scene tracks it in the path from 41 to 55 m and the car stops 3.5 to 3.6 m from the
  camera (2.0 m from the bumper), short of the 5 m target, and holds.

## Routed drives

`route_drive.py` drives a route from `routes.py` with the whole stack: navd routing on Valhalla tiles built from the
road-graph export, turnpilotd, the detector in modeld and turnpilot_web. The bridge plays the driver. It sets each
leg's destination through turnpilot_web's `/api/destination`, as the phone app does, pulls away and engages, and
signals when turnpilotd prompts for a nav lane change (`driver.py`; the Tesla port can't signal). The model stops on
its own at stop signs, and now and then where nothing is in the road, and waits; after 4 s standing without a
TurnPilot hold the driver taps the gas, an override that keeps openpilot engaged, and each tap is logged. The driver
engages again after a disengagement and logs the events behind it, and brakes at the destination. One bridge session runs
`--runs` drives, and `route_stats.py` turns the session's rlogs into per-run stats. Start openpilot with these
params set and nothing extra blocked: `NavigationEnabled`, `TurnPilotEnabled`, `TurnPilotRouteSource=valhalla`
and `TurnPilotValhallaTiles=~/turnpilot_data/valhalla/west_coast_usa`.

```sh
python -m openpilot.tools.sim.bridge.beamng.route_drive --route loop --runs 3 --out loop.json --no-keyboard < /dev/null
python -m openpilot.tools.sim.bridge.beamng.route_stats loop.json   # writes loop_stats.json
```

`--traffic N` spawns N BeamNG traffic cars and removes them at the end, even if the bridge dies. `--capture f.mp4
--capture-fps 5` records the chase view live, at a camera rate the turns fail at; render it after the drive instead
([Chase view](#chase-view)). A loop needs more than one leg, because on this map one leg from a start back near itself
never has more than a few turns.

A full drive stops telling you anything at its first missed turn, since the reroute goes elsewhere. `--screen` runs
one short drive per turn and keep instead (`screen.py`): each starts 130 m before its maneuver in the lane for it,
ends 70 m past it, and stops early once navd says the car is off the route. Use it to find the junctions a route
can't use before you drive the route. A collision while engaged latches `Offroad_ExcessiveActuation` in selfdrived,
and nothing engages again until selfdrived restarts. route_drive stops the session there, and `--start-at N`
resumes the plan on a new stack.

`route_stats.py` reads every route navd published (`navRoute`). A route whose end moved is a new leg, and any other
new route is a reroute. It places the route's turns and keeps from the first `navInstruction` after the route
(`allManeuvers`). A maneuver counts as attempted once the car reaches it along the route, and as taken once the car
is 30 m past it and still on the route. It also counts disengagements with their `onroadEvents`, TurnPilot holds
with their reasons, desire pulses and lane changes.

Results on 2026-10-27, big model on the chestnut, 20 camera pairs a second, set speed 40 km/h:

- The model forgets a desire pulse when it leaves its desire history, 132 frames (6.6 s at 20 Hz) after it (first
  read here as about 5 s): `desireState` falls back then, and the planned turn goes with it. Gate 1's 40 m lead at 25 km/h fits inside that window. A
  car slowed for a junction does not: at g62 the model planned the left, dropped it 10 m out and went straight,
  and a late retry pulse brought the plan back at once. So turnpilotd now pulses on the approach every 3.5 s and
  once more 10 m out, holds the car at 25 km/h until the first pulse, and slows it to 4.3 m/s by 15 m out.
  It sends no pulse once the car has turned 15 degrees: a pulse during a right turn turned the car on past the
  exit road and into the curb.
- Junctions differ. Screening the first 16 maneuvers of `loop` took 9, and after the fixes 3 of the 7 that had
  failed. Some junctions stop the car whatever the desire does. On the highway east of the town the model stops
  under an overpass at (507, 716). At (-30, 423) and (-601, 617) it stops at or just after the turn. At the fork
  (246, 628) it takes the other branch at 11 m/s. The g62 left is marginal: it was taken in 2 of 2 screening
  runs and in 0 of 2 full runs that started 150 m before it.
- A leg that ends on a divided road breaks the next leg. Valhalla snapped the moving car onto the other
  carriageway, and the route started the wrong way (town14, the DR652/DR666 pair after (-311, 337)).
- `town15b` (15 maneuvers in 4 legs, every leg ending on a two-way street): 3 runs without traffic, legs 1/4 in
  each, maneuvers 4/5, 6/7, 6/7 taken, no disengagement. Every run ended stuck in leg 2, at (-706, 131) or just
  after the left at (-539, 157), with the driver's gas taps unable to move the car. That is a takeover in a car,
  although no openpilot disengagement was logged. With 3 BeamNG traffic cars: 4/5, 3/3 and 1/2, all stuck in leg 1
  or 2 (one just after the left at (-601, 221), where the car had turned on into the curb).
- `short2` (3 turns, 550 m) on the final code: arrived with no disengagement, no gas tap and no hold in 3 of 4
  runs. The fourth took all 3 turns and then stopped 50 m short of the destination, on the divided road.
- The chase capture cuts the camera rate to 11-13 pairs a second at `--capture-fps` 5, 2 and 1 alike (bridge alone
  at 1: 12.8 to 14.0), so the cost is the third camera's existence, not its frame rate. The first turn of `short2`
  failed at that rate, and the demo run was recorded without the chase capture.
- Per-run stats are in `~/turnpilot-artifacts/phase2/h2/*_stats.json` and `stats_table.txt`.

Later on 2026-10-27 (H3), same setup, 19.2-20.0 model runs a second in every run:

- The big model keeps the rising edges of its last 132 frames in a state tensor, `state_desire_q` (132, 1, 8), a
  shift register max-pooled in groups of 4 (the small model: 100). In H1/H2's rlogs, wherever the car had not
  turned, `desireState` fell under 0.5 125-134 model frames after a lone pulse.
- A pulse still in that queue at the end of a turn turns the car on. town15b's leg-2 "left" at (-539, 157) is a
  52-60 degree turn by the route geometry: the car reached the exit heading at about 46 degrees, kept turning while
  the 10 m entry pulse was queued, and stopped against the curb at 122-126 degrees in H2's runs 2 and 3.
  turnpilotd now sends desire 255 (`hooks.CLEAR_DESIRE`) once a turn is done (navd past the junction and 45 degrees
  turned, or the nominal angle less 10), and modeld zeroes the turn columns of `state_desire_q`. At that left the
  clear went out at 42-50 degrees and the car peaked at 50-55 degrees, 8 runs of 8. At the 90-degree turns it went
  out at 46-83 degrees and the car ended at 85-97, except at short2's last right (below).
- `town15b2` (town15b's first leg and its second up to just past that left, 7 turns, 0.9 km): arrived with no
  disengagement, gas tap or hold in 5 of 7 runs with the detector up (3 recorded with the UI); 1 more arrived after
  one gas tap and 1 ended stuck, both at the leg-2 right at (-706, 115), and an eighth run, with the detector
  already off (below), also stuck there (a ninth held at a left for it). There the car sometimes starts turning only 2-3 m
  before the maneuver point (4-5 m when it gets round), ends up pointing at the storefront across the street, and
  the model sees it as a lead at 6.5-8.6 m (prob 0.76-0.91) and stops: 5 of 11 passes today in all routes (3 stuck,
  2 going on after a tap), 2 of 5 in H2's. The late starts had the car 0.4 m further from the right road edge 10 m
  out (3.2-3.6 m against 2.7-3.3, the model's own estimate) and 4.2-4.6 m/s there against 3.0-4.6. Screened alone
  from 73 m out it passed 12 of 12, at a 3.5 or 4.3 m/s corner speed alike, so the slower corner was not kept.
- `town15b` itself stops at the fork DR272_7 (-486, 159), 55 m past that left: the route goes on at 108 degrees
  between roads at 82 and 130, with no maneuver. The model took 130 with no desire (3 runs of 3) and 82 with a
  keepLeft held from 20 m (2 of 2, reverted), so no keep desire fixes it. `fork272` starts just before it.
  Screening the rest of town15b: the model stops and won't go on at (-167 to -180, -2 to 8) on leg 3, misses the
  sharp right at (-539, 157) from the east on leg 4, and takes leg 2's last two turns and leg 4's last.
- `short2`: 3 of 4 clean; the fourth turned the last right on to 127 degrees into the curb after a late start
  (51 degrees turned at the maneuver point), with the clear at 81 degrees. Gate 1 at group_46_47_48_49, 40 m:
  left 3 of 4, right 2 of 2.
- The detector in modeld stops for good after a burst of slow frames: it skips a frame when the model time plus
  the median of its last 5 run times passes 45 ms, and it only adds a time when it runs. After 3 runs of 37-92 ms
  (model runs of 58-122 ms, while other work loaded the Mac) it never ran again, and TurnPilot then holds before
  every left for "detector unavailable" (one town15b2 run held 374 s). Seen in the last 3 sessions today.
- `control.py` now drops its wound-up speed reference when the car accelerates 1.5 m/s^2 past the request: out of
  a tight turn the ETK 800 surged at 4.5-4.8 m/s^2 on a 1.0 request, and openpilot latched excessiveActuation.
- route_drive's JSON has `poses` per run: wall-clock and run time, position and BeamNG's rotation quaternion at
  20 Hz (`pose_fields`), from the IMU pose, which trails the game by 150-210 ms. Per-run stats are in
  `~/turnpilot-artifacts/phase3/h3/*_stats.json` and `stats_table.txt`.

`route_stats.py` also says whether the detector was still running at the end of each run (`detector_up`: a
`turnpilotDetections` in the last 5 s). Count a run only with the detector up and the model at 18 Hz or more.

## Map stops and signals

The driving model doesn't stop for red lights (0 of 31 in the phase 3 eval) and rarely for stop signs, so TurnPilot stops
for the route's mapped controls (`selfdrive/turnpilot/map_controls.py`). In the sim the controls come from tiles built
with the level's own stop signs and signals:

```bash
tools/turnpilot/build_sim_tiles.sh   # -> ~/turnpilot_data/valhalla/west_coast_usa_signals (seconds, no game needed)
```

It runs `maplanes.py`, then `mapsignals.py` on the committed `wcusa_map_node_signals.json` and `wcusa_stop_props.json`
(re-dump them with `probe_signals.py` after a game or level update), builds the tiles, and adds `controls.json` (each
control's real stop line) and the `controls_built` marker. Tiles without the marker are reported unsupported: on the old
`west_coast_usa` and `west_coast_usa_lanes` tiles navd sends no controls, and TurnPilot says so on screen.
`launch_openpilot.sh` uses the signals tiles by default once they are built, and turns `DisengageOnAccelerator` on, as
on the car: there is no gas go (2026-10-28), any press disengages. A stop sign stops and hands back ("Stop sign: take
control to proceed"), and so does an unprotected turn at its hold line.

A signal adds nothing unless a light source reads red or yellow for it (no light source on the car this week). The sim's light oracle gives
turnpilotd the game's light states instead, SIM ONLY:

```bash
TURNPILOT_LIGHT_ORACLE=1 OPENPILOT_PREFIX=tpsim openpilot/tools/sim/bridge/beamng/launch_openpilot.sh
python -m openpilot.tools.sim.bridge.beamng.run_bridge --signals --light-oracle
```

`scenarios/run.py` and `route_drive.py` take the same two flags and start the oracle (`run_bridge.light_oracle`) for
the drive, stopping it at the end. `--signals` without `--light-oracle` leaves the lights running with nothing reading
them: every signal is held for the driver's go, and the sim driver taps only on a ground-truth green, so it never does.

turnpilotd takes the oracle only with `SIMULATION=1` and `TurnPilotSimLightOracle` (which the launch sets, and which
clears on every manager start); otherwise it rejects it out loud and holds. The scenario runner and `route_drive.py` tap
the gas where a driver gives the go (`driver.GasTaps`, each tap logged with its reason and how long the car had stood):
once the car has stood 2 s where TurnPilot asks for a go, at a stop sign TurnPilot holds past its dwell, at an
unprotected left's hold line, at an unchecked signal only when the ground-truth light is green (`--signal-tap off`
never), and standing engaged with nothing held. Never while TurnPilot hands back, is unavailable or faulted, or its
plan is stale: a prompt left up then is no go. `light_oracle.py` and `probe_signals.py` haven't been run against the
game yet: check the published states against the HUD first.

## Unprotected left gate

The autonomous unprotected left (`selfdrive/turnpilot/unprotected_left.py`, DESIGN phase 4 G4) is gated on its own cell
(CRITIQUE REQUIRED 5): a 9 m hold, no approach pulse, a stand of 10 s or more, a cold go pulse, 12 runs or more, no
misses and no stalls. The four gap-stream scenarios are that cell: the visible approach to (-650.1, 481.7), mapped 25 mph
across (R_req 133 m), a queue of oncoming cars released through the junction at 4, 6, 8 and 10 s headways from the car's
stand at its line, then an empty road. No gas tap anywhere; the score (`scenarios/score.py`) fails a post-encroachment time
under 2 s, a false go, a stall or an unmoving go, a missed turn, and undecided after a gap of tReq was offered, and counts
missed gaps.

What the gate can't prove: `TurnPilotLeftRangeM` is unset by default (unmeasured: 0), which skips every autonomous left at
route time and reroutes, so the gate sets it to `unprotected_left.PROVISIONAL_RANGE_M` (150 m, pixel arithmetic, not a
measurement; on the car it is set only from a measured recall range). And the junction's straight arm across is about
106 m, under its R_req of 133 m: a car beyond the bend is out of sight, so a go here is certified past the sightline. The
gate checks the left's logic (waiting through the streams, going after them, never a false go, PET, no stall), never its
reach; nothing in the stack checks a junction's sightline, so the demo junction is chosen by eye for a straight, flat
approach of R_req or more.

The junction is signalized and mapped, so the left goes only on a read green (`unprotected_left.SIGNAL_NEEDS_GREEN`: it
doesn't check the cross traffic). Run it with the light oracle (`TURNPILOT_LIGHT_ORACLE=1` on the launch, `--light-oracle`
on the bridge; without `--signals` the game's lights are held green, so it reads green), or the left waits at its line and
hands back on the light it can't read ("Can't read the light: take control").

```sh
python -c "from openpilot.common.params import Params; Params().put('TurnPilotLeftRangeM', 150.0)"  # sim only, see above
for h in 4 6 8 10; do
  python -m openpilot.tools.sim.bridge.beamng.scenarios.run unprotected_left_gap$h --runs 3 -- --light-oracle <bridge args>
done
```

| param | values | default |
| --- | --- | --- |
| `TurnPilotAutoLeft` | `off` (hand back at the line, G1), `announce` (hold, say whether it would go, hand back), `auto` | `auto` |
| `TurnPilotLeftRangeM` | m, 0 to 400: the detector's proven range for oncoming traffic, from a measurement; a left whose R_req is past it is skipped at route time | `0.0` (unmeasured: every autonomous left skipped and rerouted) |
| `TurnPilotLeftFarRangeM` | m, 0 to 400: the far crop's proven range (below); while the far crop reads it extends the reach, so a 45-55 mph left (R_req 231-296 m) needs it | `0.0` (unmeasured: no left past `TurnPilotLeftRangeM`) |
| `TurnPilotFarCrop` | the far crop on (`1`) or off (`0`) in modeld | `1` (runs when its pkl is in `selfdrive/turnpilot/models`) |

The far crop (DESIGN phase 4 G5): modeld runs a 640x384 window of the road frame at native resolution (twice the full pass's
pixel density) on the frames between the full pass's runs, aimed up the oncoming approach while a left is in hand
(`turnpilotPlan.farCropBearing`), published as `turnpilotFarDetections`; the full pass's messages say what it is doing
(`farStatus`). An R_req past `TurnPilotLeftRangeM` is judged on its runs too: one covering the band past that range every
0.3 s, and any car it sees in the band blocks at any range. It going off for the drive is said once (`farViewOff`) and a left
needing it hands back at its line (`farOff`). `unprotectedLeft.farNeeded`, `farRangeM` and `farBlockers` log it. In the sim
it sees BeamNG's frames; its range there is no measure of the car's.

The model may hold the car at its go (G0: 14 of 24 tapless goes stood). G5's bounded go (next section) lifts that hold;
with `TurnPilotGoOverride` off, or a go the car doesn't follow, the go is withdrawn and handed back (`cantStartTakeControl`),
which the score fails as a stall.

## Bounded go (G5)

TurnPilot's go from a standstill (a turn its hold let go of, or the autonomous left's go) is certified to plannerd
(`turnpilotPlan.goCertified`, cause `turnRelease` or `unprotectedLeft`, `selfdrive/turnpilot/go_override.py`). For it the
longitudinal planner lifts the model's standstill hold: the e2e candidate goes to +0.5 m/s^2 at most and its shouldStop is
dropped, for 3 s at most, under 3 m/s, and only while nothing else holds the car (no lead within 15 m, no TurnPilot hold,
the mpc and cruise candidates not holding, no pedal). Lead, cruise and TurnPilot's caps stay in the min(). The lift ends
once the model takes over (its shouldStop false and its planned speed 0.5 m/s or more from 1 s to 2 s ahead, held 0.3 s),
and at once on a brake or gas press, a road user in the path within 15 m (a track or a raw box), a lead, a hold, or the
go withdrawn. Alone it moves the car 2.25 m at most, at 1.5 m/s at most.

A `turnRelease` go (a turn TurnPilot's own hold let go of: its walker cleared, its red went green) certifies only that
TurnPilot's blocker left the path, never why the model still holds the car, so it rests on its junction's own
certification, as the left's go rests on its gap: the junction's signal read green by an accepted source (here the light
oracle), checked at the go and on every frame of it (`turnpilot.go_not_certified` why `light`, `turnpilot.go_end` reason
`light`). The camera's shadow reader never certifies one (its red or yellow keeps one off), and with no signal mapped at
the junction there is none (`RELEASE_UNSIGNALIZED` in `turnpilotd.py`). On the car, where no light source is accepted,
a walker clearing at a signal is the model's go or the no-move handback, as before G5. Nor is a `turnRelease` ever
certified for a turn across the oncoming road (its go is the left's, on its gap) or for a far-side turn released in the
junction (why `unprotected`, `junction`): nothing the lift checks sees the lanes it crosses, and the stall's handback
(`stalledTakeControl`) covers that stand.

A go the model never takes over is withdrawn at its 3 s and handed back (`cantStartTakeControl`, the car stopped at 1.5
m/s^2 and held): with a model that holds, the car ends about 3.0 m past its line. A go the car doesn't move on within 2.0 s
(1.0 s for the left) is withdrawn as before. With an actuator lag over about 0.4 s the lift doesn't reach 0.3 m/s inside
the left's 1.0 s, so the left's go is withdrawn and handed back; measure the lag (`carState.aEgo` behind
`carControl.actuators.accel`) before relying on it. That is the total from turnpilotd's go to the wheels: the plan's hop to
plannerd and controlsd and the CAN frame add about 0.1-0.15 s on the car to the actuator's own lag.

The left's go is lifted only from its line (4.5 m or more short of the oncoming lanes: the nudge, its stop and 1.5 m), never
from a creep, and it is followed into the oncoming lanes only by a launch: a car under 2.0 m/s with 1.5 m left to stop short
of them is withdrawn and handed back (`turnpilot.left_not_launched`). A model that only keeps the speed the lift gave it
satisfies the takeover predicate, and would otherwise coast across at under 1 m/s.

| param | values | default |
| --- | --- | --- |
| `TurnPilotGoOverride` | BOOL: TurnPilot's go certified for the planner's lift | `1` (on); a build that doesn't know the key is on |

The planner logs `longitudinalPlan.turnpilotGoLift` (lifted this frame) and `turnpilotGoEnd` (why the last lift ended:
`window`, `takeover`, `speed`, `driver`, `lead`, `holder`, `modelBrake`, `withdrawn`, `replaced`; it keeps the last go's
reason until the next go starts), and cloudlog events
`turnpilot.go_lift` and `turnpilot.go_lift_end`. turnpilotd logs `turnpilot.go_certified`, `turnpilot.go_not_certified`
(with why), `turnpilot.go_end` and `turnpilot.go_not_moving`.

## Post-go profile sweep

A turn TurnPilot lets go of at a standstill is driven under a post-go profile from the go (`turn_speed.py`, DESIGN phase 4
section 3). Two params pick it by name, read once a second, so the sweep's columns need no code edit:

| param | values | default |
| --- | --- | --- |
| `TurnPilotPostGoProfile` | `none` (V1 tapless, no cap), `flat3` (3.0 m/s to 45 deg), `graduated` (3.0 m/s to 15 deg, 4.3 at 45), `flat43` (4.3 m/s to 45 deg), `left` | `graduated` |
| `TurnPilotLeftGoProfile` | the same names, for a go into an unprotected turn (a left across the oncoming road) | `left` (1.5 m/s^2, 4.3 m/s to 45 deg) |

The capped profiles other than `left` cap the accel at 1.0 m/s^2 until 45 deg, and all of them blend into the turn's own
corner cap between 45 and 60 deg. An unset or unknown value is the default, logged. Set one for a run with the stack's
prefix, after `scons` in the main checkout has built the keys into the params library (an older build logs the key as
unknown and uses the default):

```sh
OPENPILOT_PREFIX=tpsim python -c "from openpilot.common.params import Params; Params().put('TurnPilotPostGoProfile', 'flat3')"
```

Whatever the profile, a go the car hasn't pulled away on (0.3 m/s) within 2.0 s, 1.0 s into an unprotected turn
(`turnpilotd.NO_MOVE_S`, `LEFT_NO_MOVE_S`), is withdrawn and handed back (`cantStartTakeControl`). The car is then held
standing until openpilot disengages. That applies only to a go from the hold line (`turnpilotd.GO_LINE_M`, 7 m or more
before the junction). A go in the junction, such as a pedestrian hold released partway round, is never withdrawn: the car
is never held there, and standing gets the stall handback (`stalledTakeControl`) with no hold. For the V1 tapless column, which measures pull-aways later than 2 s, set
`TurnPilotNoMoveS` (FLOAT, 2.0 to 20.0 s, default 2.0) the same way; it sets the standstill turn's limit only, and the
unprotected turn's 1.0 s stays. Put it back to 2.0 after the sweep. turnpilotd logs `turnpilot.post_go` (the profile chosen) at each go and `turnpilot.go_not_moving` at a withdrawal.

## Chase view

A live chase camera (`--capture`) costs the driving cameras 6 to 9 pairs a second at any frame rate, and the model's
turns fail at the 11 to 14 left, so the demo's chase view is rendered after the drive. `chase_render.py` reads a run's
pose log (route_drive's `poses`, or the scenario runner's `poses` and `actor_poses`), teleports the car to each pose
on a 20 Hz grid on the game's clock, takes one frame from capture.py's chase camera for each, and writes an mp4 with
the sidecar `demo_video.py` aligns on. Run it with the bridge stopped, since it moves the same car:

```sh
python -m openpilot.tools.sim.bridge.beamng.chase_render demo.json --run 1 -o demo_chase.mp4
python -m openpilot.tools.sim.bridge.beamng.route_stats demo.json   # demo_stats.json
python -m openpilot.tools.turnpilot.demo_video --chase demo_chase.mp4 --ui demo_ui.mp4 --result demo_stats.json --run 1 -o demo.mp4
```

`record_route.sh` and `record_scenario.sh` do it all on a fresh stack (`restart_stack.sh`): the drive with the UI
recorded by `record_ui.py`, the stats, the render and the composed video.

The render also replays the lights and the indicator the run logged (`chase_track.Replay`): a forced light's states from
its `light_events`, exactly when the director set them; any other light the light oracle read, from the rows' `lights`,
0.3 s earlier than the row that first shows each change (the oracle's latency); and the ego's indicator from carState's
blinkers in the rows (`lamp`), else the stalk the runner held. A light the log says nothing of stays as the render's
world has it, every light green and held. The sidecar lists the replayed lights (`lights_replayed`,
`indicator_replayed`) and says whether the run's own lights were running (`signals_running`, `lights_note`): with
`--signals`, a lamp in view that wasn't replayed isn't the run's, so caption it. Not yet checked in the game: that a
strict state set between frames shows in the next chase frame, and `set_lights` on the teleported ego.

Measured on 2026-10-27:

- The log is the IMU's pose at the camera mount. It holds each IMU batch for one to three 20 Hz rows and trails the
  game by 150 to 210 ms, so `chase_track.py` keeps the first row of each new value, moves it 0.16 s back, and fits
  a Gaussian-weighted line (sigma 80 ms) through the rows within 0.25 s of each frame.
- The teleport position is (0.33, -0.332, 1.066) m from the IMU in the vehicle frame (x left, -y forward, z up), not
  `cameras.MOUNT`. Teleported with that offset, the IMU read back within 1 cm of the logged pose, and the car settled
  by less than 1.2 cm.
- After a teleport, the first one or two new chase frames still show the old pose; in 12 trials the third always
  showed the new one. The render takes the fourth, with the game running. Paused, the camera renders only on a
  physics step, and the frame after one step was sometimes the old pose and sometimes missing.
- BeamNG's own replay (`core_replay`) records in this headless game, and the bridge kept its 20 pairs a second while it
  did, with the car standing (not measured while driving). But a camera created while the replay is paused never
  rendered, and the replay's own timeline came out shorter than the wall time it recorded, so it would need its own
  alignment. The pose log works for any drive that logged poses, so the replay wasn't taken further.
- The render shows the car's pose only: the wheels don't steer and the body doesn't roll, and BeamNG traffic that
  isn't a scenario actor isn't there.

## Scenario hooks

`scenario.py` works with any BeamNGpy client: in the bridge process use `world.bng` under `world.bng_lock`, from another
script open a second client. It can teleport the ego, read its pose, spawn BeamNG traffic, spawn and move the Italy
mannequin (the only human-shaped stock asset) as a pedestrian prop, remove the traffic again, and force or release
traffic signals. `restore_game` undoes the bridge's global changes, and running the module does that from the shell after
a run that crashed or didn't restore:

```sh
python -m openpilot.tools.sim.bridge.beamng.scenario  # signals running, HUD and particles on, 08:52 light
```

`BeamNGBridge(scenario=...)` runs a callable in the bridge loop at 100 Hz; gate1 is the example.

## Check it without openpilot

```sh
OPENPILOT_PREFIX=tpcheck python -m openpilot.tools.sim.bridge.beamng.check_bridge
```

It starts the bridge and plays the parts of openpilot the bridge talks to: a VisionIPC client, a camera-turn test that
times a frame from the game to VisionIPC, opendbc's Tesla CarState on `can`, the GPS against the game's own position,
and openpilot's Tesla commands on `sendcan`. `check_vipc.py` is the VisionIPC client on its own. Unit tests are in
`tests/`.

## Cameras

Both cameras sit at one mount near the top of the windshield, 1.34 m above the road, level and pointing straight
ahead: the road camera at a 36.82 degree vertical field of view and the wide one at 83.57 degrees, a pinhole stand-in
for the fisheye with the focal length modeld's warp assumes.

The game renders a camera on its frame once the camera's update time has passed. With a 0.02 s update time both cameras
render on every frame. A poll returns the newest render and the game answers it on its main thread, once a frame, so
polling costs frame time: polling each camera every 10 ms on its own connection dropped the game to 9 fps. The bridge
polls both cameras in one pipelined burst on their own connection every 30 ms, which puts both answers on the same game
frame. The publisher sends a pair as soon as both frames are new by hash and its 20 Hz slot is due, converts RGB to NV12
with upstream's `rgb_to_nv12`, and stamps both frames with one time. After 150 ms without a new pair it republishes the
last pair (a standing car can render identical frames), but only while polls succeed: when the cameras go away, frames
stop within a second, the poller reconnects, and after 5 s of failed polls the bridge stops.

The game's frame rate with both cameras sets the ceiling. Rendering both 1344x760 cameras without reading them back
already holds the PC to about 23 fps, and at 896x506 or 672x380 it reaches only about 27 fps, so the cost is per render,
not per pixel or in the transfer. On 2026-10-27 the game ran at 48 to 50 fps idle and 16 to 25 fps with both cameras (a
day earlier: 62 to 77 idle, 23 to 24 with cameras). Measured on 2026-10-27, game on the PC, bridge on the Mac over the
USB4 link:

| What | Value |
| --- | --- |
| With openpilot and modeld (small model on METAL), driving at 25 km/h, 140 s | 17.5 pairs a second at `narrowRoadCameraState` and `modelV2`, median interval 54 ms, p95 104 ms, max 119 ms, 10 s windows 15.1 to 18.8; game at 19.3 fps on average |
| Same, standing still, 90 s | 17.9 a second, 10 s windows 13.7 to 20.0 |
| Bridge alone, standing still, VisionIPC client, game at 20.3 fps | 19.7 a second, none repeated, no frame id gaps, p90 interval 52 ms; the earlier fixed 20 Hz publisher gave 18.0 and 18.8 back to back |
| Road and wide from the same game frame | 17 of 17 camera-turn trials over three runs |
| Frame age, game frame to VisionIPC | 102 to 131 ms, from the camera-turn test |
| Frame age, first sight on the Mac to a VisionIPC client | median 62 to 73 ms |
| RGB to NV12, upstream `rgb_to_nv12` | 3.9 ms a frame on the bench, 4.2 to 4.4 ms in the running bridge |
| Ad-hoc polling, for comparison | 8 to 10.5 pairs a second, 100 to 570 ms old |
| With `--capture` | about 12 pairs a second |

The comma four's cameras run at exactly 20 Hz and modeld steps its temporal context by 50 ms per frame, so at 17.5 Hz the
model sees about 12% less time pass than really does, with occasional 100 ms gaps.

## Known limits

- The camera rate is below 20 Hz whenever the game runs below 20 fps with both cameras, which it did for most of
  2026-10-27 (see [Cameras](#cameras)). Pointing the game's own view at the sky added about 1 fps; it isn't done.
- The game's render pipeline adds about 120 ms between a game frame and its VisionIPC publish, and published timestamps
  are 40 to 50 ms later than the true render time; the comma four's cameras add far less.
- The pose behind the GPS and IMU lags the game by 150 to 210 ms (heading against the game's own, in a 16 to 38 deg/s turn):
  BeamNG ships its 100 Hz IMU readings in batches about every 100 ms, so the 100 Hz IMU messages hold each batch's
  values. The gyro comes from differencing the IMU's world-space axes; the readings' `angVel` doesn't use the documented
  sensor axes (the yaw rate shows up in its third, "right", component).
- The game's IMU has no noise or bias.
- The ETK 800's steering lock (38 degrees) comes from yaw rate at low speed on one street; paramsd learns the rest.
- `--capture` renders a third camera and costs about 8 camera pairs a second; use `chase_render.py` after the drive.
- modeld doesn't reconnect to a new VisionIPC server: restart it whenever you restart the bridge.
- A teleport resets the car to P; the bridge shifts it back to D, but openpilot raises `wrongGear` in between and
  disengages, as a real Model 3 would.
- The bridge changes global game state on the shared PC and leaves it that way: every traffic signal forced green and
  held, the HUD hidden, particles off, and the time of day held at 13:12. Pass `--restore-on-exit`, or run
  `python -m openpilot.tools.sim.bridge.beamng.scenario` afterwards.
- The World base class allocates two unused 1928x1208 image buffers, and SimulatorBridge an unused Ratekeeper.
