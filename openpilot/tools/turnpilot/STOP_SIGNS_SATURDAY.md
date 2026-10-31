# Stop signs on Saturday 2026-10-31

The first real drive, in Chase's 2017 Model 3, makes no stop-sign go of its own, in `trial` or `auto`. The certifier's rules block both modes at several points, and none of those rules changes here. Each item below is enough on its own:

- No stop-sign sidecar on the map server (step S0). The region `index.json` lists no `osm_controls/` shard, so the car can't name the junction's OSM node (`routing/controls.py`, `junctions.py`). Every stop acts as shadow with H8 (`stop_go.py`), and `junction_validate.py` skips every evidence event with node 0, so without S0 even shadow evidence counts for nothing.
- The camera fields in `maps/visibility.json` (lens, mask, `darkThreshold`, the taped offsets) are null, and `certify()` says `noTable` until step S1 fills them.
- Only one kind of junction can certify: a surveyed 3-way all-way stop, arrived at on the stem of the T. `junction_validate.py` never stamps the opposite approach's lanes (`STAMP_SIDES`), so no 4-way certifies. The cross road must be posted 15 mph, or 20 mph if the stamp confirms PV1's optimistic reach. Validation needs at least 10 fully stopping cross cars a side. One rolling stop fails the row, and so does one car first seen too late.
- The checklist's rule for Halloween keeps engaged driving off residential streets, a go needs openpilot engaged, and every candidate near Indianapolis is residential.
- `auto` also needs 10 `trial` goes passing `launch_verify.py` (step S4a).

Saturday can produce the evidence a later go needs: S0 (Friday, at a desk), the S1 camera fields, an in-person survey look, shadow evidence or a validation stamp at one candidate, and the measured longitudinal delay (`actuatorDelayS`). `~/turnpilot-artifacts/phase4/chase_car/STOPS_STEPS.md` has the steps, written for the checklist.

## The tools

All three run on the Mac, read only the files named here, and write only copies. None of them changes a decider rule, `stop_go.py`, `stop_rules.py`, `visibility.py` or routing.

### allway_candidates.py (T2)

Lists the 3-way all-way stops near a point that a car can arrive at on the T's stem with the sun off its nose over a time window. It reads the junctions from the region's sidecar shards and the legs from the region's PBF with osmium. A sidecar record lists only arriving approaches, so a one-way leg that only leaves is in none; 181199929's north leg is one. Such a leg is allowed when it runs within 15 deg of straight on, because nothing arrives on it and `stop_rules.leg_skew` allows it, and the tool flags it. Any other extra leg means the junction isn't a T. The light test is `stop_rules.light_window` with the veil unmeasured, the test `certify()` makes. The tool refuses a region with no shards, a shard stamped otherwise than `meta.json`, and a window that touches dusk.

On the scratch sidecar (607 shards, built from the region's PBF), the 10:00-15:30 EDT window and 30 km around downtown, it lists 181199929, 180698706 and 179811104, the three the plan found. The audit trip it prints ends 40 m out on the cross leg, so the route has to pass the node; a 150 m exit let the route skip 179811104's junction.

### camera_fit.py (T1)

Fits the wide lens, its frame edge, the footprint mask and `darkThreshold` (step S1) and writes them, with the taped setback, offset and height, into a copy of `visibility.json`. The module docstring has the method and every refusal. Two choices differ from the plan's wording, and each one is the stricter reading:

- Assignment. The plan assigned each dwell to the mark whose pixel box over the lens bracket contains it. On a comma four, the boxes of the 70, 80, 85 and 88 deg marks overlap by tens of pixels across k 0-0.6, so that rule would refuse nearly every real run. The tool takes every assignment that keeps the standing order and that the boxes allow, fits each one, and requires exactly one to pass every check. When none passes, or more than one does, it refuses.
- Exposure. The plan said "exposure-normalised". In daylight the wide camera integrates 4 to 8 lines, so one line is a 15-25% step in exposure. Rescaling luma by exposure would assume a linear tone curve, which the ISP's isn't. Pooling the frames would lift a static dark pixel's temporal std past the 4.0 DN threshold, and a pillar would read as clear. The tool groups frames into exposure bins 10% wide, drops a group under 20 s, and marks a pixel blocked when any group finds it static and dark. On 200 s of a public comma four drive, 83 s survived the grouping, so step 16 asks for 10 minutes of driving after calibration.

A third choice the plan didn't cover: at 88 deg a person is half out of the frame, and the box's bottom centre sits inside the feet. On synthetic data, 6 px of that moved the fitted edge 0.4 deg outward, which would overstate reach at the edge. A dwell whose boxes touch the frame's edge counts as seen, has to lie within 2.5 deg of the fitted edge, and is left out of the least squares.

The tests use no YOLO. The detection path (YOLO26n on the full letterbox and both side views, under `TinyJit`) and the mask path were run on a public comma four segment: 120 frames took 33 s, about 0.08 s a frame for all three passes once the jit had captured them, so an S1 recording takes about 20 s per minute. No real S1 recording exists yet, so the full run has not been seen on a real one.

### route_audit.py's reroute (F1)

A reroute that Valhalla can't route ("No path could be found") is a failed iteration, the way navd's `router.reroute` makes it, so `avoid.run` keeps the best route and marks the stop unavoidable instead of the audit crashing. navd is unchanged.

## Decisions taken, the safer option each time

1. The Halloween rule against residential streets stands, so nothing goes at a stop sign on Saturday.
2. The veil stays `unmeasured`: sun ahead zeroes reach from 67 deg on.
3. A rolling stop fails the validation row, and `--exclude` is only for a car whose clip shows it entering its lane inside the distance.
4. 4-way junctions stay out until step S4 bounds the opposite approach's ground range.
5. T1 refuses rather than assume a lens or keep a bearing it has no data for, and `darkThreshold` is never under 45 DN. It is rounded up, never down.
6. `actuatorDelayS` is measured (`launch_verify.py` on step 18's and 19's logs) before any `trial` go.
7. If the Tesla's cruise won't set below 18 mph, a 15 mph showcase breaks the rule "set speed no higher than posted". Step 18 decides that.
8. D9 stays off: no chained goes when the model won't take over.
9. T2 keeps a junction with a one-way exit leg straight on, flagged, instead of dropping it. It writes nothing the car reads, and the junction builder and `certify()` judge the junction.
