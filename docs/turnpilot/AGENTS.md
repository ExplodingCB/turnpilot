# Working on TurnPilot

Read this before you change anything. The plan is [PLAN.md](PLAN.md). The readiness findings are in [readiness-2026-10-27.txt](readiness-2026-10-27.txt) and the research notes are in [research-2026-10-26.md](research-2026-10-26.md).

## Goal

The goal is a destination-to-destination drive with zero disengagements in a real car: eyes up, hands off. The sim is a test harness. Any code on the car path has to work in a real car, on any openpilot-supported car and in any region.

## Code rules

- Keep the code minimal, modular and fast, and match openpilot's style:
  - two-space indent
  - short modules
  - no frameworks openpilot doesn't already use
  - comments only where the why isn't obvious
- New code goes under `openpilot/selfdrive/turnpilot/`, or under `openpilot/tools/sim/bridge/beamng/` for the sim. Edit upstream files only where your task says to, and keep each hunk small.
- Sim-only code stays under `openpilot/tools/sim/`. The car path has no sim flags.
- The interfaces are fixed:
  - the cereal structs `TurnpilotPlan`, `TurnpilotDetections` and `TurnpilotScene` in `openpilot/cereal/custom.capnp`
  - the services `turnpilotPlan`, `turnpilotDetections` and `turnpilotScene`
  - the params `TurnPilotEnabled`, `TurnPilotShadowMode`, `TurnPilotRouteSource`, `TurnPilotValhallaTiles`, `TurnPilotWebToken`, `TurnPilotFavorites`, `TurnPilotMapHost`, `TurnPilotMapKey`, `TurnPilotMapCert`, `TurnPilotMapCoverageStatus`, `TurnPilotMapStatus`, and the car-signal params `TurnPilotCarSignals`, `TurnPilotCarSignalsVerified`, `TurnPilotCarSignalsCheck`, `TurnPilotCarSignalsCheckNow` and `TurnPilotCarSignalsExterior` (INSTALL.md, "Let the car signal its own turns")

  If you need a change, make it in your branch and say so in your report.
- Don't get stuck in analysis paralysis. Build the simple version that works, test it, and move on.

## Environment

- Repo root: `/Users/kaileh57/Projects/turnpilot`. The integration branch is `turnpilot`. Python is `/Users/kaileh57/Projects/turnpilot/.venv/bin/python` (3.12, every openpilot dependency installed).
- In a worktree:
  - Run Python with `PYTHONPATH=$PWD`, so `import openpilot` resolves to your worktree.
  - The submodules (`opendbc_repo`, `msgq_repo`, `tinygrad_repo`, `panda`, `rednose_repo`, `teleoprtc_repo`) aren't checked out there. Symlink them from the repo root if you need them, for example `ln -s /Users/kaileh57/Projects/turnpilot/opendbc_repo opendbc_repo`, but never commit the symlinks.
  - Add files by explicit path. Never use `git add -A` or `git add .` in a worktree.
  - `git status` fails on the symlinked submodules; use `git status --short --ignore-submodules=all`.
  - Tests that touch Params or msgq need the built libraries. Symlink `openpilot/common/libparams_c.dylib` from the repo root (it's gitignored), and run pytest as `python -m pytest` from the worktree root.
- The main checkout's `msgq_repo` carries an uncommitted macOS patch (poll tick 10 ms to 1 ms, `openpilot/tools/turnpilot/msgq-macos-poll.patch`). Without it, the sim stack on the Mac lags and won't engage reliably. Never run `git submodule update` in the main checkout. If the patch is lost, reapply it and rebuild with scons.
- Tests: follow the style of upstream's existing tests next to the code you touch. Add the missing test tools to the repo venv with `uv pip install --python /Users/kaileh57/Projects/turnpilot/.venv/bin/python pytest`.
- Commits: work on your own branch (`tp/<task>`), commit as you go, and end every commit message with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Don't push, merge, or touch the `turnpilot` branch. The integrator merges.

## Shared hardware: use it only if your task says so

There's one of each piece of hardware, and two users at once break things:

- Chestnut eGPU: a tinygrad USB device on the Mac, opened by one process at a time.
- BeamNG.tech on the Windows PC, at `192.168.137.1:25252` with BeamNGpy 1.36. The PC agent, piston, keeps it running. Connect with `BeamNGpy('192.168.137.1', 25252)` and `open(launch=False)`. Always disconnect cleanly, and never close or quit the game.
  - Drive it through the bridge in `openpilot/tools/sim/bridge/beamng/`; its README has the launch commands, gate 1 and the scenario hooks.
  - If the game stops answering for more than 2 minutes, write `~/pc_relay/incoming/piston_restart.md` (what you saw and when) and say so in your report. piston checks for that file and relaunches the game on the same address with the `user_stock` folder; its crash notes are in `~/pc_relay/incoming/piston_log.md`.
  - The PC has little free RAM. Keep to the road and wide cameras plus at most one 1280x720 chase camera.

## Files from the PC (`~/pc_relay/incoming/`)

- `osm_export/`:
  - `west_coast_usa.osm`, an OSM export of the stock map's road graph
  - `frame.py`, with `to_latlon(x, y, z)` and `from_latlon(lat, lon, h)` on an ENU plane at 39.7684, -86.1581, where BeamNG x is east and y is north
  - `west_coast_usa_graph.json`, the raw road graph
- `piston_assets.md` and `wcusa_signals.json`:
  - the only human-shaped stock asset, `/levels/italy/art/shapes/buildings/human_scale_190.dae`, spawned with `StaticObject` at yaw = heading + pi/2
  - 37 signalized junctions, each controllable with `set_instance_strict_state`
  - signalized junctions that have crosswalks
- `camera_bench/`: BeamNGpy camera timings measured on the PC.

## Prior attempt: mine it for parts, don't build on it

`/Users/kaileh57/Projects/commahack` (in particular `sim-device/tools/simulated_tesla.py`, `sim-device/adapters/beamng/`, `sim-device/tools/navd/` and `example/comma_hack_7`) and the `turnpilot-internal` clone at `/private/tmp/claude-501/-Users-kaileh57-Projects-turnpilot/753fc3b6-93e6-456d-ba00-1b240425a64b/scratchpad/src/turnpilot-internal` have useful pieces in poorly built systems. Copy small, verified pieces. Never copy the structure: no postboxes, latches, flags-on-flags or handoff files.
