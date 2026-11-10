# Confident test build: what was loosened

This branch is a supervised test build. It drives on its best judgement where a public build would wait, slow or hand
back. This file lists every rule loosened for it, with the earlier value, so a public build can restore them.

The hard rails are unchanged: a read red or a stoppable yellow stops the car, a person in the path holds it, an oncoming or
crossing vehicle the cameras see holds a left or a stop-sign go, a left from the wrong lane is skipped and rerouted, real
faults alert, and driver monitoring is stock.

## Unprotected lefts (`unprotected_left.py`, `turnpilotd.py`)

| Rule | Public value | This build |
| --- | --- | --- |
| `WINDOW_FROM_APPROACH` | the clear window restarts at the hold line | counts on the approach too. The car still brakes to its line (there is no rolling left), and on logged approaches the window rarely filled before the line |
| `LEFT_SIGNAL_WAIT_S` | 10 s, and never a go with no turn signal lit | 3 s, then the left goes on its gap unsignalled |
| `LEFT_LAUNCH_STAND_S` | 2.0 s (`NO_MOVE_S`) before the no-move fault | 4.0 s |
| Oncoming band boxes | every vehicle box in the band blocks | a box going away up the road, or standing wholly outside the opposing lanes, does not block |
| A committed left and a red | a red read refuses the go | a left already across the oncoming lanes finishes |

## Lights (`map_controls.py`, `detector/light_state.py`)

| Rule | Public value | This build |
| --- | --- | --- |
| `RED_DECEL` | 2.0 m/s² | 3.0 m/s² for a read red |
| `RED_LAG_S`, `RED_PAST_M` | none; an overrun 2 m past the line drops the stop | 0.5 s and 4 m: a red is stopped for if the stop ends within 4 m past its line, otherwise the car goes through and says so |
| `RED_GONE_S` | a camera red holds until a steady green | released after 2.5 s unbroken of its head in view and not red |
| Camera green | every head of the signal agrees | more green heads than red |
| `VOTE_N` | 4 of 5 reads | 2 of 5, a tie is unknown |
| Lamp position rule | red counted in the upper half, green in the lower | hue only; position places a yellow only |
| Right at a signal | goes with the model when no colour is read | flows only on a read green; otherwise stops at the line, stands `RIGHT_RED_DWELL_S` (1.5 s), looks for `RIGHT_RED_CLEAR_S` (1.0 s) and goes. Never while a red has been read in the last `RIGHT_RED_UNREAD_S` (2.5 s) |
| Off-route speed cap | 11.2 m/s with an alert when a destination is set and no route is valid | no cap, one silent status line (with `TurnPilotAlwaysDrive` on) |
| A stop driven past under the gas | dropped when the gas lifts | dropped 2 m past the line above 2 m/s |

## Stop signs (`stop_go.py`, `stop_rules.py`)

| Rule | Public value | This build |
| --- | --- | --- |
| `VISION_CLEAR_S` | 2.5 s | 1.5 s |
| `VISION_LONG_CLEAR_S` | 4.0 s | 2.5 s |
| `VISION_ANNOUNCE_S` | 1.0 s | 0.5 s |
| `VISION_CREEP_AFTER_S` | 2.0 s | 1.0 s |
| Creep for a view | on every wait for reach, in 0.5 m steps | only for an occluder, or where the furthest reachable stand can show enough |
| Vision gate handbacks | skewed junction, unknown leg, no sidecar, side views off, divided road, roundabout, stale map | none of these; only a rail crossing, a missing visibility file or a lens mismatch hand back |
| Washed or frozen side view, yaw over 8° | holds with no bound | goes on what is seen after the long look |
| A junction the map has nothing for | handback | judged as a plain cross road (`ASSUMED_LINE_M` 10 m) |

## Lane changes and prompts (`lane_planner.py`, `hooks.py`)

| Rule | Public value | This build |
| --- | --- | --- |
| `LC_END_S` / `LC_END_MIN_M` / `LC_END_MAX_M` | 6 s / 60 m / 120 m | 4.5 s / 35 m / 100 m |
| `LC_ABORT_COOLDOWN_S` | 6 s | 3 s |
| `PAINT_CARRY_S` | none: a frame with no paint read is unread | 0.25 s: judged on the last read |
| Solid white line | never crossed | crossed into a turn lane the map's lane list names, when the model sees the lane and it is free. Never yellow, never a line with no colour read |
| Signal prompt chime | loops for the whole prompt | once, then every `PROMPT_REPEAT_S` (8 s) |
| Lane change ask | after a 3 s countdown | with the offer |

## Routing and handbacks (`navd.py`, `routing/avoid.py`, `handback.py`)

| Rule | Public value | This build |
| --- | --- | --- |
| Stop signs under vision stop-go | routed around when the line would hand back | never routed around |
| Detector handback | latched to a full disengage | clears if the detector returns within 3 s |
| Detector re-measure | waits 16 frames (0.8 s) after any pass over budget | skips one run (`FORCE_SKIP_FRAMES` 4) when only slightly over |

## Not proven on the car

Everything above is tested in unit simulations and open-loop replays of logged drives
(`openpilot/tools/turnpilot/drive_scorecard.py`). None of it has been driven, and no left's verdict changed on a replayed
drive: the replay is open loop and reuses the logged build's detections. The right-on-red look and the stop-sign go judge
cross traffic on the forward cameras only. On the last logged night drive the car boxed no signal head at four of five red
stands, so a red at night may go unread: a left then rolls to the junction mouth and a straight goes through.
