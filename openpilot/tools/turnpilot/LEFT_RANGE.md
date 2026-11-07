# Left range and left sites

`TurnPilotLeftRangeM` defaults to 210 m and `TurnPilotLeftFarRangeM` to 0 (unmeasured). The 210 m is pixel arithmetic for the native full pass (a car 1.8 m wide is about 10 px at 200 m), not a measurement: it covers lefts across roads of 35 mph or less (`maps/defaults.py`). turnpilot_web's start writes it over an unmeasured 0 once per comma, and over the earlier 150 m default, and leaves any other value alone.

Since 2026-11-06 turnpilotd drives every left across an oncoming approach to its hold line and judges it there, never skipping or rerouting one for its range (Kellen: "drive to the line; go where the camera covers the oncoming road, else hold and hand back loudly"). Where R_req fits the range the left waits for a clear gap and goes. Past it, the car is held at the line and the driver is told to take control ("Can't see oncoming far enough: take control"). Two tools on this branch, both read only, help pick the range and the junctions where a left goes by itself:

- `left_range.py` measures the two ranges on rlogs this fork recorded on the car, and prints the `preflight --set` line only when the evidence supports a range.
- `left_sites.py` lists the junctions near a point where a left would go from its line at a given range, ranked by margin, with the reason every other candidate fails (most of them hand back at the line).

Neither tool writes a param. Kellen runs the `--set` line on the comma, parked, if he decides to.

## What the range means

The left goes when no road camera box (a vehicle at confidence 0.35 or more in the oncoming band) with a nearest plausible range (`near_m`) inside R_req has appeared for `CLEAR_WINDOW_S` (2.5 s). It judges a gap only when R_req, from where the car stands at its line, fits under `TurnPilotLeftRangeM`, or past it under `TurnPilotLeftFarRangeM` while the far crop reads; otherwise it hands back there. So the range is safe only if every oncoming car inside it is boxed, with no gap long enough for the window to fill. After dark the reach is capped at 50 m (`NIGHT_LEFT_RANGE_M`, what the detector was seen to reach at night), whatever the ranges say, so at night every left across a road faster than about 8 mph hands back at its line.

`left_range.py` counts a car as confirmed from the farthest `near_m` after which it never went `CLEAR_WINDOW_S / 3` (0.83 s) without a box at the left. CRITIQUE REQUIRED 2 sets that ratio: the window must be at least 3 times the longest dropout. A stretch with no road run (the full pass on the wide camera, an uncalibrated spell, the detector stopped) counts as no box, so unknown is never a clear road.

The drive is not the left, so a gap on the drive counts as the longer of two stretches it would become at the left:

- The same missed runs. On the drive the full pass is on the road camera every run. Held at the line (`roadAndWide`), only 2 runs in 3 (`WIDE_EVERY`) are, so the same misses take 1.5 times as long.
- The same stretch of road. On the drive an oncoming car closes at its own speed plus Chase's car's. At the line it closes at its own speed only, so it stays in each stretch longer: (its speed + ours) / its speed.

The drive time allowed is therefore 0.56 s at most, about 4 missed runs in a row. It drops to 0.50 s with Chase's car at 20 mph and oncoming traffic at 30 mph, and to 0.42 s with both at 25 mph. The slower Chase's car goes, the less the second rule costs. The far crop's chain uses the same rule, although the far crop's runs aren't thinned at the left. That is the safe side.

A box counts for a car only where the car's own motion puts it. The chain's range must fit the seed's speed and ours from the seed, and from the car's last box. It is never re-centred on a hit, so the box of something closing at another rate (a parked car, a lead) drifts out. Any other band box within twice those gates of where the car would be ends the chain as `ambiguous`, and so does the one box there when it falls outside them. Something else could be taken for the car there, so the tool gives no credit past that point. The cost is that close traffic shortens the chain. For the road camera's 150 m (ground), oncoming cars need to be about 2 s apart. For the far crop's 300-350 m, they need to be about 4-5 s apart. Cars parked by the oncoming lanes, and a lead in our lane near an oncoming car's range, end chains too.

Two limits remain. First, a box of another object first seen inside a car's gate on the run the car's own box goes can still carry the chain until their motions part by the gate's width (2 m plus 15% of range), a second or more. For example, a stopped car that comes into view as the car passing it leaves. Second, a chain runs only back from its seed (10-80 m), so gaps between the seed and the car aren't checked.

The recommendation is the shortest confirmed range among at least 59 oncoming cars, rounded down to 10 m. With 59 cars the shortest is a one-sided 95% lower confidence bound on the 5th percentile (Wilks), which matches DESIGN section 6's 95% recall rule. It also means every sampled car was confirmed by that range. `--allow-misses` takes Wilks' k-th shortest instead, which lets the k-1 shortest fall short; it is off by default because it loosens the rule, and it lists the cars it lets through.

The units are `near_m`, the same units the gap rule compares with R_req: the box bottom 3 px lower, so a pitch error of that size can't stretch the range. A car first boxed at a ground range of 150 m reads about 113 m, one at 300 m about 182 m (camera 1.22 m up). The report prints each car's ground range too, for context only.

At the 9 m hold line R_req is DESIGN's figure. Route time's R_req adds the D4 margin (`ROUTE_REACH_MARGIN_M`, 15 m past the 9 m hold line at a junction with no mapped stop line), the reach a car standing that far back would need. Until 2026-11-06 a left whose route-time R_req was past the range was skipped and rerouted; now that figure only goes in the logs (navd's `turnpilot lefts:` line names the steps that may hand back), and the line's own check decides:

| Opposing limit | At the line, one lane | At the line, lanes unmapped (two) | Route time, one lane | Route time, two lanes |
| --- | --- | --- | --- | --- |
| 20 mph | 110 m | 115 m | 165 m | 171 m |
| 25 mph | 134 m | 141 m | 200 m | 207 m |
| 30 mph | 158 m | 167 m | 234 m | 243 m |
| 35 mph | 183 m | 193 m | 269 m | 279 m |
| 40 mph | 207 m | 219 m | 303 m | 315 m |

So a 25 mph left goes from its line once oncoming cars are confirmed out to a `near_m` of about 134-141 m, and the 210 m default covers 35 mph only if the road camera's full pass really boxes cars that far out, which `left_range.py` is there to show. The far range only counts once `TurnPilotLeftRangeM` is above 0, because turnpilotd extends the reach by the far crop only past a road range.

## Measure the range on Saturday

The measurement comes from checklist step 17, the shadow drive. To give it enough samples:

- Keep TurnPilot enabled in shadow mode and `TurnPilotFarCrop` on, so both passes run and are logged.
- Finish calibration first (step 16). Runs before calibration can't be ranged.
- Drive straight two-way roads with steady oncoming traffic, where oncoming cars come singly, a few seconds apart, and no cars are parked along the oncoming side. On curves the full pass moves to the wide camera and the chain breaks. Cars close together end their chains early (`ambiguous`), and the shortest car sets the range.
- Drive no faster than traffic needs. The faster Chase's car goes past an oncoming car, the shorter the gap in its boxes that is allowed.
- Plan for at least 60 oncoming cars that come within 80 m in the oncoming lanes. That can take more than one drive. Each drive can go in one run. The tool starts a new drive at each new route's initData, and wherever log time goes back more than 10 s (a reboot) or jumps ahead more than 5 s. It measures each drive on its own time base. The report says how many cars it found.

After the drive, parked, copy every segment of the route off the comma and run the tool on the Mac from this worktree:

```sh
ssh comma@<address> 'ls -t /data/media/0/realdata | head -20'
scp -r 'comma@<address>:/data/media/0/realdata/<route>--*' ~/chase_routes/
cd ~/Projects/turnpilot/.claude/worktrees/tp-chase-range
PYTHONPATH=$PWD:$HOME/Projects/turnpilot/.claude/worktrees/tp-blinker/opendbc_repo \
  ~/Projects/turnpilot/.venv/bin/python -B -m openpilot.tools.turnpilot.left_range ~/chase_routes/
```

The tool reads every rlog under the directories it is given, in segment order. Several routes can sit in one directory, or each in its own. Each continuous stretch of log is measured as a drive of its own, and its cars' lines say `drive N`. A track id that turnpilotd's tracker hands out again (after a restart, or on another route) is never taken for the earlier car. Any drive that can't be used (a sunnypilot log, no carState, no camera) refuses the whole run. It prints the run counts, the number of oncoming cars sampled, how their chains ended, the shortest five with why each one's confirmation ended, and a recommendation or the reason there is none. It reads no location service, so it prints no position. It refuses a sunnypilot log, since sunnypilot's own messages sit on `turnpilotDetections`' slot.

Exit status 0 means a road range is recommended, 2 means none (too few cars, nothing calibrated), and 1 means the log can't be used.

Each sample line gives its reason. `unboxed` means runs went on and none boxed the car, which is either the detector's reach or a car that pulled in from a side street. `noRuns` and `uncalibrated` are stretches with no usable road run. `ambiguous` means another box sat where the car could be, or its one box there didn't fit its motion, so the tool gave no credit. Many `ambiguous` lines mean the traffic was too close together. Look at the short ones before deciding: a car from a driveway caps the range as hard as a real miss.

If a range is recommended and Kellen decides to use it, the report ends with the command to run on the comma, parked in P, for example:

```sh
python3 -m openpilot.selfdrive.turnpilot.preflight --set TurnPilotLeftRangeM=110 --set TurnPilotLeftFarRangeM=180
```

`car_preflight` requires 210, this build's default: GO at 210, a WARN at any other range above 0 (a measured one is the owner's call), and NO-GO at 0, where no left would ever go.

## Find a site

Give `left_sites.py` the measured ranges (or the 210 m default) and a point to search around. Use the map server's region, which is what the car's corridors are cut from:

```sh
cd ~/Projects/turnpilot/.claude/worktrees/tp-chase-range
PYTHONPATH=$PWD:$HOME/Projects/turnpilot/.claude/worktrees/tp-blinker/opendbc_repo \
  ~/Projects/turnpilot/.venv/bin/python -B -m openpilot.tools.turnpilot.left_sites \
  --tiles ~/turnpilot_data/mapserver/regions/indianapolis --range 110 --far-range 210 --radius 1500 <lat,lon>
```

It finds junctions in Valhalla's graph, routes each left from 150 m back on its approach, and judges the route with the fork's own functions (`upcoming_maneuvers`, `unprotected_turn`, `opposing_roads`, `route_controls`, `reach_at_line`). A site passes only if all of these hold:

- The turn is a left across one road straight on, and its bearings agree with its modifier.
- The controls are known, and the junction has no signal, stop or yield of ours.
- R_req, from the opposing road's mapped limit and lanes (45 mph and two lanes when unmapped), fits under the range. A left that fails here is driven to its line and hands back there.
- The opposing road is mapped straight past R_req. A left that fails here hands back at its line too ("Oncoming road bends out of view: take control").

Every failing candidate prints each check it fails. On the Indianapolis region the controls read as partial, because the region has no OSM stop-sign sidecar and Valhalla's flags miss about a quarter of OSM's stop signs. Partial passes, with a note to check the junction for a stop sign by eye.

`~/turnpilot_data/valhalla/indiana/tiles.tar` has no `controls_built` marker and no `meta.json`, so `route_controls` reads its controls as unsupported and every site fails. The same applies to `route_audit` with those tiles, which checklist step 4 names: it prints "controls status unsupported" and lists no stop or yield at all, so that step's GO can't be read from those tiles. Give `route_audit` the region directory as well.

The tool can't see the sightline to R_req (a crest, a hedge, parked cars), signals and signs the map lacks, or whether the car can stop at the hold line from the approach speed. Drive past every listed site by hand before using it. DESIGN section 6 then asks for at least 10 decisions with `TurnPilotAutoLeft` set to `announce`, each compared with what the driver saw, before the first autonomous go.

## Tests

```sh
cd ~/Projects/turnpilot/.claude/worktrees/tp-chase-range
PYTHONPATH=$PWD:$HOME/Projects/turnpilot/.claude/worktrees/tp-blinker/opendbc_repo \
  ~/Projects/turnpilot/.venv/bin/python -m pytest openpilot/tools/turnpilot/tests/test_left_range.py openpilot/tools/turnpilot/tests/test_left_sites.py
```

`test_left_range.py` builds synthetic shadow-drive logs. Its oncoming cars are boxed from known ranges through the fork's own lens, so each box ranges back to where the car was. `test_left_sites.py` judges synthetic route steps and fakes Valhalla's locate answers. The worktree needs `openpilot/common/libparams_c.dylib`, copied from a built worktree, for the imports to load.
