# BeamNG scenarios

Scored runs of TurnPilot on the BeamNG bridge, and side-by-side demo videos of them.

## What a run does

Each run follows the same steps:

1. Teleport the car to the scenario's start.
2. Place the scenario's mannequin or AI cars.
3. Send the destination to turnpilot_web's `/api/destination` with the `TurnPilotWebToken` bearer, as the phone does.
4. Engage through the bridge with a rolling start.
5. Watch `turnpilotPlan`, `selfdriveState`, `carState`, `navInstruction`, `navRoute` and `modelV2` until the car arrives, disengages or times out. `ped_in_path` ends instead when the car has stood still for 4 s.

While it runs, the runner plays the driver: it holds the stalk whenever `turnpilotPlan.signalPrompt` asks for a signal, which is the PLAN's answer for cars where openpilot can't signal.

Everything a run spawns is removed after it. The parent process sweeps the game again by name (`tp_sc_*`) before the bridge starts and after it exits, so a killed bridge leaves nothing behind.

## Run it

Start openpilot first. The runner sets `NavigationEnabled` and `TurnPilotEnabled`, which navd, turnpilotd and turnpilot_web run on.

```sh
OPENPILOT_PREFIX=tpsim openpilot/tools/sim/bridge/beamng/launch_openpilot.sh
OPENPILOT_PREFIX=tpsim python -m openpilot.tools.sim.bridge.beamng.scenarios.run ped_crosswalk_right --runs 3 --record ~/turnpilot-artifacts/demo
```

Bridge options (`--host`, `--signals`, `--reload` and so on) pass through. Every run of an invocation shares one bridge session, because modeld doesn't reconnect to a new VisionIPC server.

- Without `--record`, results go to `~/turnpilot-artifacts/scenarios/<scenario>-<time>/result.json`.
- With `--record DIR`, they go to `DIR/<scenario>-<time>/`, along with `ui.mp4` (the upstream UI recorded by `openpilot/tools/turnpilot/record_ui.py` while the runs drive) and `chase_run<N>.mp4` per run, rendered by `../chase_render.py` from the run's pose log once the bridge has exited. Each video gets a `<video>.start.json` sidecar holding the wall-clock time of its first frame. Block the stack's own UI (`BLOCK=ui`) so only the recorder draws. `--no-chase` skips the render. There is no live chase camera: a third camera costs the driving cameras 6 to 9 pairs a second, at which the model's turns fail.
- `--cleanup` only sweeps leftover scenario actors out of the game.

For the demo's shots:

- `--end-after-handback N`: N s after TurnPilot hands back (a take-control prompt, `turnpilotPlan.handback`, or a soft disable), the sim driver brakes as a driver taking over does, and the run ends `taken over` instead of standing to its timeout. `driver_takeover` says when, and after what.
- `--gas-press-at [S]`: the sim driver presses the gas once (0.3 for 0.5 s), S s after engagement (`gas_press`). It implies the `gasDisengage` expectation.
- `--drop-detections-at [S]`: S s after engagement the runner sets `TurnPilotSimDropDetectionsS` to 20 s, and the detector skips its runs that long (`detection_drop`). modeld honours it only with `SIMULATION=1`; on the car it is never read. The runner clears it at each run's start and end; a build that doesn't know the param ends the run `error` (rebuild the stack). It implies `handback:detectorStopped`.
- `--expect E` (repeatable): what the run must show, in place of the scenario's own `expect` (below).

Without S, the two take the scenario's `fault_at_s`.

## Scenarios

The turn scenarios use junctions from piston's list of signalized junctions with crosswalks. Of the six he listed, (-445.3, 147.0) and (-32.5, 403.8) have no 90-degree turn in the road graph.

| Scenario | Setup | Pass |
| --- | --- | --- |
| `ped_crosswalk_right` | Right turn at (-768.6, 555.2). The mannequin leaves the inside corner, crosses the road being turned into, then walks 10 m away along the far sidewalk. It sets off on the car's go at the junction (`timing.GoWatch`): the hold within 35 m of the junction let go with the car standing, the car moving again after standing there, or, if it never stood (a green), the turn's desire out there with nothing holding the car. A stand or a release further out, at a signal before the junction's own, is no go. It reaches the crosswalk zone 0.9 s later and stays in it about 15 s, while the car takes 2 s or more from the stop line to its hold line. | Holds to a stop for "pedestrian in crosswalk", moves on, takes the turn, arrives, no disengagement, never within 2.5 m |
| `unprotected_left` | Left turn at (-650.1, 481.7). Two AI cars stand 35 and 60 m out on the arm straight across. They go together on an AI script a set time after the car stands at a hold within 35 m of the junction (`release_after_stop_s`, from `timing.oncoming_release_delay`), so the first reaches the junction 3.5 s after the driver's go (2 s after the stop), as the car pulls away; on a go that comes sooner they go at once. On the signal tiles the junction's signal stops the car at its line, 11 to 27 m out. | Stops for the junction's own control and gets the driver's go there: `signalGo` at a mapped signal, `unprotectedTurnGo` where there is none (the sim driver taps the gas once the car has stood 2 s, logged in `gas_taps`). With an oncoming car at the junction while the car is at it, holds for "waiting for oncoming" until it is through, goes after, takes the turn, arrives, no disengagement |
| `unprotected_left_visible` | The same left turn at (-650.1, 481.7), approached from the south-east up a gentle grade. `unprotected_left` comes down a ~21% hill onto a shelf, and the arm across drops away past it, so its oncoming cars stay out of view until they are about 13 m from the car. Here the arm across rises about 3 m over its first 85 m. | As `unprotected_left` |
| `ped_in_path` | The mannequin stands in the lane 40 m before the junction at (-768.9, 661.4), on gate1's eastbound straight. | Stops with the camera 3 to 8 m from it (scene_policy stops 5 m short, camera to pedestrian), no disengagement |
| `occupied_lane` | A data entry (below). When the car engages, an AI car is placed in the target lane just ahead of it and paces it at the set speed until 6 s into the lane change window, then drops back to 40%. | The planner reports "car in target lane", and the lane change starts with the pacer more than 15 m away |
| `lane_change` | A data entry: a multi-lane approach to a turn that needs a lane change. | Changes lanes, takes the turn, arrives, no disengagement |
| `route_demo` | From 20 m past the bridge's default spawn to road-graph node DR249_3, about 850 m away. | Arrives, takes every maneuver, no disengagement |
| `straight_cruise` | `ped_in_path`'s straight with nothing on it, to 140 m past the junction straight on. The stage for `--drop-detections-at` (S01) and `--gas-press-at` (S04): at its `fault_at_s`, 3 s after engagement, the disengagement lands 10 m or more short of the junction's stop line. | Its expectation: the detector's handback, or the gas press disengaging |

Each scenario is an `Entry` in `catalog.py`. The builders there turn an entry into world positions from the road-graph export, without touching the game.

### Junction data

`lane_change` and `occupied_lane` read a JSON list, by default `lane_change_junctions.json` next to this file. Pick another file with `--data` and an entry with `--entry NAME`:

```json
{"name": "autojunction_1311_left", "junction": [-885.9, -407.3], "turn": "left",
 "spawn": [-627.09, -201.98, 93.2], "heading_deg": -177.1, "dest": [-841.58, -447.58, 96.79], "speed_kph": 45}
```

Positions are BeamNG world meters and `heading_deg` is counterclockwise from east. `junction` and `turn` are needed for the pacer and for the checks against the mapped controls. The set speed is 45 km/h because DesireHelper changes lanes only above 20 mph. The two entries shipped here come from the road graph:

- `autojunction_1311_left`, the default: a divided road with two lanes our way, spawn 350 m out in the right lane. The only mapped control on the approach is the junction's own stop sign, so nothing stops the car in the lane change window (250 to 75 m before the turn at 45 km/h).
- `wpTown_5_left`: a four-lane road, spawn 460 m out in the right lane. A signal stands 160 m before the turn, inside the window, so the car stops there. It is kept for the lane planner's work on a change before a mapped stop. `DR290_1_left` was dropped: its approach has a signal in the window too, and the nearest spawn clear of the mapped controls is 600 m out, behind four signals.

### Mapped controls

The sim tiles (`west_coast_usa_signals`) map every signal and stop sign of the level, from the committed `../wcusa_map_node_signals.json` (`../mapsignals.py`). The catalog reads the same file:

- Every spawn is more than `spawn_clear_m` before each control on its approach (the rolling start, a second to engage, then a map stop's engage distance: 65 m at 30 km/h, 118 m at 45 km/h), or at least 15 m past it. Otherwise the rolling start runs over the line before openpilot is engaged to stop for it ("Stop Line Passed"). A turn scenario's spawn moves to the nearest clear spot (`notes.approach_m`); a data entry that isn't clear is refused.
- A plan's notes say which controls are on its approach (`controls_from_spawn`, m from the spawn), the junction's own control and its stop line (`junction_control`, `stop_line_m`), and for the data scenarios the controls that stop the car in the lane change window (`controls_in_window`).

## Results

`result.json` holds every run. Each run records:

- the outcome: `arrived`, `disengaged`, `taken over` (the driver's brake from `--end-after-handback`), `stopped`, `timeout`, `no destination`, `no route`, `no engagement` or `error`
- its expectations (`expect`: the scenario's `Entry.expect`, or the runner's) and whether each was met (`expected`). An expectation is what the run's shot exists to show: `handback:<kind>` (a `turnpilotPlan.handback` such as `detectorStopped`, or a take-control prompt such as `cantStartTakeControl`), `gasDisengage`, `skip` (too fast to turn, or the left skipped at route time), `turnMissed`, `hardBrake`. One not seen fails the run. One seen explains only the failures it causes: the disengagements at or after the handback, the press's own disengagement within 0.3 s, the route not finished, the left's stall. A disengagement before the handback, a turn desire after the skip, or a handback after the gas press still fails.
- maneuvers taken out of attempted. A maneuver is attempted once the car is within 30 m of it while engaged. It is taken if navd then moves on to the next step without a reroute, the car stays on the route for 20 m of the next step (`left_route_after`), and for a turn, the car's heading turns at least 45 degrees its way (`turn_deg`; a recorded drive's rows have no heading). Going straight on past a turn moves navd on to the next step all the same, then leaves the car off the route, so it counts as not taken.
- gas taps: each press of the gas while engaged, with the go TurnPilot asked for (`prompt`), the hold's reason, how long the car had stood (`stopped_s`) and how far it was from the junction (`j_dist`). `driver_taps` is the sim driver's own log of the taps it gave.
- the go the junction's turn takes (`junction_go`), from its own mapped control (`junction_control`, from the plan's notes; a rescore reads them from `result.json`): `signalGo` at a signal, `stopSignGo` at a stop sign, `unprotectedTurnGo` where there is none. Only a stop and a tap within 35 m of the junction count for it.
- whether an oncoming car was presented (`oncoming_presented`): one within 30 m of the junction while the car was within 35 m of it and still before it. The left scenarios require the "waiting for oncoming" hold when one was, and fail as untested when none was. Rows from before this was recorded require the hold as before.
- disengagements, each with the driver override or the last alert as its cause
- metres driven in an oncoming lane (`wrong_side_m`, each stretch in `wrong_side`): the car's centreline more than 1 m left of the divider between the road's two directions, outside every junction box (the hold line plus the widest road at the junction, where a turn crosses; for a car heading into the junction, its farthest mapped stop line plus 2 m, where a car pulls away from the line toward its turn). A one-way road has no oncoming lane, and the wrong way down one is all oncoming. 3 m or more fails any scenario. The divider comes from the road graph's lane strings, kept with the results as `road` so a rescore checks it too; a run with positions and no road model fails, and a recorded drive's rows have no positions, so it isn't checked there.
- holds, each with its reason, its duration, and whether the car stopped and then moved on
- the closest the camera mount came to the mannequin and to the other cars (`min_gap_ped_m`, `min_gap_car_m`)
- lane changes, and the pacer's offset when each started
- the rlog segments written during the run
- wall-clock start, engage and end times
- pass or fail with reasons
- the 20 Hz rows it was scored from
- a 20 Hz pose log from the teleport to the end: the car's IMU pose (`poses`, the fields route_drive logs) and each actor's (`actor_poses`), which `chase_render.py` re-renders

Rescore saved results, or score a recorded drive's rlogs (a real car drive works too):

```sh
python -m openpilot.tools.sim.bridge.beamng.scenarios.score result.json
python -m openpilot.tools.sim.bridge.beamng.scenarios.score --scenario route_demo ~/.commatpsim/media/0/realdata/<route>--*/rlog.zst
```

## Demo videos

```sh
python -m openpilot.tools.turnpilot.demo_video --chase DIR/chase_run1.mp4 --ui DIR/ui.mp4 --result DIR/result.json --run 1 -o crosswalk.mp4
```

`../record_scenario.sh` does the whole thing on a fresh stack: restart, record, render, compose.

The tool puts the chase view and the UI side by side, each 540 px high, over a caption strip giving the scenario, pass or fail, maneuvers and disengagements. It cuts to the run's span, plus 2 s at the end.

- The two recordings line up on their sidecars.
- The UI recorder writes frames at a nominal 60 fps, but the Mac draws about 57, so the tool retimes the UI by the frame count and real span in its sidecar.
- `--chase-start` and `--ui-start` override the sidecars, for recordings that have none.

To test it without the bridge, play any video into the UI with `openpilot/tools/turnpilot/fake_ui_feed.py --video X --start-out X.start.json`, and record that with `record_ui.py`.

## Not yet run against the game

Everything here was built and tested offline, with unit tests and a dry run of the runner against a fake bridge, director, SubMaster and clock (`tests/`). These parts touch the live game and are unverified:

- The pacer's AI script. The oncoming cars' scripts were checked in the game on 2026-10-27: a car stands on the parking brake until it is released, then drives its script across the junction. A car held with `brake=1` instead backs away at full power, because BeamNG's arcade automatic reads the brake pedal at a standstill as reverse.
- Where the crosswalk is. It's placed from road widths (2.5 m past the edge of the road the car comes from), not from the level's crossing decals, and `result.json` notes this.
- The mission-group sweep for mannequins (`getCount`/`getObject` on `scenetree.MissionGroup`) and the vehicle sweep by name.
- Whether turnpilot_web and navd are up, with a route source for the sim map, in the sim launch.
