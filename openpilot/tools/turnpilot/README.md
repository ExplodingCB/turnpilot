# TurnPilot bench tools

These tools run openpilot on the Mac, which stands in for the comma four, with the big model on chestnut. Run every command from the repo root with the venv active:

```sh
cd /Users/kaileh57/Projects/turnpilot && source .venv/bin/activate
```

## Before the first build

On macOS, msgq polls in 10 ms ticks. A 100 Hz reader like card then merges two CAN messages now and then, carState drops to about 87 Hz, and selfdrived raises `selfdrivedLagging`, which blocks engagement. The patch cuts the macOS tick to 1 ms. It touches only the `__APPLE__` branch, so the comma four build doesn't change.

```sh
git -C msgq_repo apply ../openpilot/tools/turnpilot/msgq-macos-poll.patch
scons -j$(sysctl -n hw.ncpu)
```

The patch stays uncommitted in the `msgq_repo` submodule, so `git status` shows `msgq_repo (modified content)`. Committing it would pin a msgq commit that doesn't exist on commaai/msgq.

## Chestnut

Only one process can hold chestnut. Before you use it, stop modeld and the manager, and close any WebUSB tab. This command must print 0:

```sh
ioreg -p IOService -l -w0 -r -n "custom 8516b768-CLEAN" | grep -c UserClient
```

- `python -m openpilot.tools.turnpilot.chestnut_smoke [--hz 20]` loads the big model the way modeld does, then times one packed upload of both 1344x760 frames and `model.run`.
- `python -m openpilot.tools.turnpilot.chestnut_recovery_check [--modeld]` forces a big model failure by raising from its runner, then checks that the mid-drive recovery (`selfdrive/modeld/chestnut_recovery.py`) reloads the big model in a successor process, bit-exact, while the small model keeps publishing. With `--modeld` it runs the real `modeld --demo` against fake cameras and checks that modelV2 goes big, small, then big again with no gap.
- `python -m openpilot.tools.turnpilot.gate2_modeld [--frames 600]` runs the real `modeld --demo` against fake os04c10 road and wide cameras. It reports `modelV2.modelExecutionTime`, frame age and frame drops, then checks that chestnut was released. With `--detector` it sets `TurnPilotEnabled`, so modeld also runs the TurnPilot detector, and reports its time and rate.

## Sim stack

`launch_sim.sh` is upstream's `tools/sim/launch_openpilot.sh` with two changes:

- `TURNPILOT_LOG=1` keeps loggerd, encoderd and logmessaged running, so a run leaves an rlog in `~/.comma${OPENPILOT_PREFIX}/media/0/realdata`.
- `FINGERPRINT` can be overridden.

On PC, encoderd writes lossless video at about 1.8 GB a minute per camera on noise frames. Real frames compress better, but it's still gigabytes a minute. For gate runs, add `BLOCK=encoderd` to keep only the logs (about 1.4 MB of rlog a minute).

```sh
OPENPILOT_PREFIX=tpbench TURNPILOT_LOG=1 BLOCK=encoderd openpilot/tools/turnpilot/launch_sim.sh
OPENPILOT_PREFIX=tpbench python -m openpilot.tools.turnpilot.null_bridge   # second terminal, no simulator
```

`null_bridge` drives the stack onroad with no simulator: a parked Honda with ignition on (the upstream `SimulatedCar`), IMU, GPS, fake DM, and gray 1344x760 os04c10 frames. Use it to check that every process comes up, before the BeamNG bridge exists.

`OPENPILOT_PREFIX` keeps msgq, params and logs apart from other runs on this Mac. To clean up afterwards, run `rm -rf ~/.commatpbench`.
