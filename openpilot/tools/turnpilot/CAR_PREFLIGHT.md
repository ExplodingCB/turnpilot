# Car preflight

`car_preflight.py` checks the car that TurnPilot is about to drive, Chase's 2017 Model 3 (HW3). It prints GO, NO-GO, WARN, UNKNOWN or INFO for each car-dependent item. It reads a running comma four, or a route pulled off one. It complements `openpilot/selfdrive/turnpilot/preflight.py`, which checks the install (branch, toggles, map server, model files, the chestnut's USB link) and is unchanged.

The tool only reads. It subscribes to messages for the window, reads params and log files, and never writes a param, publishes on a socket or opens the panda. It reads no location service, and from a route's `initData.params` it keeps only the keys it checks, so it prints no GPS position.

## Run it on the device

Park, put the car in P, switch it on and keep your hands off the stalk. Then, over SSH:

```sh
cd /data/openpilot && python3 -m openpilot.tools.turnpilot.car_preflight --live --seconds 60
```

The CAN items count only frames after the panda enters tesla safety, which on Chase's route came 11.6 s after the log started. Start the tool once the comma shows the car is ready. A shorter window works, but the absence checks (no 0x489 or 0x054, no UI_warning) need at least 5 seconds of CAN carrying DI_speed after the tesla time.

card publishes `carParams` once every 50 seconds. A window that misses it takes the `CarParams` param (this drive's), else `CarParamsPersistent`, and the car line says "the last drive" when it falls back that far.

## Run it on a pulled route

Copy the rlogs off the device, for example the first segment of the latest drive:

```sh
ssh comma@<address> 'ls -t /data/media/0/realdata | head -5'
scp -r comma@<address>:/data/media/0/realdata/<route>--0 ~/chase_routes/
python3 -m openpilot.tools.turnpilot.car_preflight --route ~/chase_routes/<route>--0/rlog.zst
```

`--route` takes files or directories (it reads every file whose name contains `rlog` or `qlog`, in segment order). Run it from the repo root with the venv active. For rlogs someone else sent, run it under `python3 -I` from a directory other than the one holding them; `-I` drops the current directory and `PYTHONPATH`, so use a small runner that puts this checkout first on `sys.path` and calls `car_preflight.main`.

A qlog keeps no `modelV2` or detections and about 3 `can` messages a segment, so the CAN, big-model and detector lines print UNKNOWN on one. comma connect holds qlogs unless an rlog upload was requested.

A route can't show `TurnPilotMapKey` or `TurnPilotMapCert`, which loggerd logs without a value (DONT_LOG). Those lines are UNKNOWN on every route, so a route exits 2 at best. Check them with `--live`.

## Read the output

Each line is `STATUS  name  detail`, with a `fix:` line under a NO-GO or WARN. The header gives the window's length and the panda's tesla time.

| Status | Meaning |
| --- | --- |
| GO | The item is as Saturday needs it. |
| NO-GO | Don't drive until the fix is done. |
| WARN | Drive only once someone has read it and decided. |
| UNKNOWN | The window or log doesn't hold the fact. Never read it as GO. |
| INFO | Context. The four lines named panda build, Tesla software, outside lamps and mount are facts no message holds; each says where to read it by hand. |

The exit status is 0 when no line is NO-GO or UNKNOWN, 1 on any NO-GO, and 2 on any UNKNOWN without a NO-GO.

Before the panda enters tesla safety it is in elm327 mode, the harness relay connects the Autopilot computer straight to party CAN, and openpilot's first 0x2B9 come back refused. Every CAN line counts only frames after the tesla time, and so does `carState.canValid`. Refusals before it print as one INFO line.

A param that is unset on the device, or absent from a route's `initData.params`, takes this build's default, and the line says `(default)`. A key this build's params library doesn't know is NO-GO: libparams was built before the key.

## Sunnypilot logs

On a log whose `initData.gitRemote` names sunnypilot, TurnPilot's own lines (car signals, detector, TurnPilot faults, the params) print INFO "not applicable". Sunnypilot puts its own messages on the union slots of `turnpilotPlan`, `turnpilotDetections` and `turnpilotFarDetections`, so the tool never reads those, nor `carState.carSignal`. Sunnypilot numbers some events differently, so event names are listed and not judged. A `fixed` fingerprint source is its vehicle selector, and the EPS line says whether this fork would match the car by its firmware.

On Chase's sunnypilot route of 2026-10-30 (two segments, the car in P) the tool printed 11 GO, 1 NO-GO and 14 INFO and exited 1. The NO-GO is the big model: `modelV2.big` was false on all 1,029 frames. Every car line was GO: the EPS string `(35),E014.21.2` is listed, the 3-bit messages are present, longitudinal is on, the panda entered tesla safety at 11.57 s and returned all 1,247 of openpilot's 0x2B9 sent after that.

## Where it lives

The spec put the tool at `openpilot/selfdrive/turnpilot/car_preflight.py`. It sits here instead because `--route` uses `openpilot.tools.lib.logreader.LogReader`, and `test_preflight.py`'s `test_car_path_imports` fails any import of `openpilot.tools` under `selfdrive/turnpilot`. `--live` imports none of the tools code.

## Tests

```sh
python -m pytest openpilot/tools/turnpilot/tests/test_car_preflight.py
```

The tests build synthetic logs with cereal's messaging and run each case live (the device's Params faked, with no way to write) and from an rlog written to disk. One test runs `--live` on real msgq sockets.
