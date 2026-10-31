# Left range and left sites

`TurnPilotLeftRangeM` and `TurnPilotLeftFarRangeM` default to 0, so turnpilotd skips and reroutes every left across an oncoming approach. Two tools on this branch, both read only, decide whether a demo left is possible:

- `left_range.py` measures the two ranges on rlogs this fork recorded on the car, and prints the `preflight --set` line only when the evidence supports a range.
- `left_sites.py` lists the junctions near a point where a left passes route time at a given range, ranked by margin, with the reason every other candidate fails.

Neither tool writes a param. Kellen runs the `--set` line on the comma, parked, if he decides to.

## What the range means

The left goes when no road camera box (a vehicle at confidence 0.35 or more in the oncoming band) with a nearest plausible range (`near_m`) inside R_req has appeared for `CLEAR_WINDOW_S` (2.5 s). Route time attempts a left only when R_req fits under `TurnPilotLeftRangeM`, or past it under `TurnPilotLeftFarRangeM` while the far crop reads. So the range is safe only if every oncoming car inside it is boxed, with no gap long enough for the window to fill.

`left_range.py` counts a car as confirmed from the farthest `near_m` after which it never went `CLEAR_WINDOW_S / 3` (0.83 s) without a box at the left. CRITIQUE REQUIRED 2 sets that ratio: the window must be at least 3 times the longest dropout. A stretch with no road run (the full pass on the wide camera, an uncalibrated spell, the detector stopped) counts as no box, so unknown is never a clear road.

The drive is not the left, so a gap on the drive counts as the longer of two stretches it would become at the left:

- The same missed runs. On the drive the full pass is on the road camera every run. Held at the line (`roadAndWide`), only 2 runs in 3 (`WIDE_EVERY`) are, so the same misses take 1.5 times as long.
- The same stretch of road. On the drive an oncoming car closes at its own speed plus Chase's car's. At the line it closes at its own speed only, so it stays in each stretch longer: (its speed + ours) / its speed.

The drive time allowed is therefore 0.56 s at most, about 4 missed runs in a row. It drops to 0.50 s with Chase's car at 20 mph and oncoming traffic at 30 mph, and to 0.42 s with both at 25 mph. The slower Chase's car goes, the less the second rule costs. The far crop's chain uses the same rule, although the far crop's runs aren't thinned at the left. That is the safe side.

A box counts for a car only where the car's own motion puts it. The chain's range must fit the seed's speed and ours from the seed, and from the car's last box. It is never re-centred on a hit, so the box of something closing at another rate (a parked car, a lead) drifts out. Any other band box within twice those gates of where the car would be ends the chain as `ambiguous`, and so does the one box there when it falls outside them. Something else could be taken for the car there, so the tool gives no credit past that point. The cost is that close traffic shortens the chain. For the road camera's 150 m (ground), oncoming cars need to be about 2 s apart. For the far crop's 300-350 m, they need to be about 4-5 s apart. Cars parked by the oncoming lanes, and a lead in our lane near an oncoming car's range, end chains too.

Two limits remain. First, a box of another object first seen inside a car's gate on the run the car's own box goes can still carry the chain until their motions part by the gate's width (2 m plus 15% of range), a second or more. For example, a stopped car that comes into view as the car passing it leaves. Second, a chain runs only back from its seed (10-80 m), so gaps between the seed and the car aren't checked.

The recommendation is the shortest confirmed range among at least 59 oncoming cars, rounded down to 10 m. With 59 cars the shortest is a one-sided 95% lower confidence bound on the 5th percentile (Wilks), which matches DESIGN section 6's 95% recall rule. It also means every sampled car was confirmed by that range. `--allow-misses` takes Wilks' k-th shortest instead, which lets the k-1 shortest fall short; it is off by default because it loosens the rule, and it lists the cars it lets through.

The units are `near_m`, the same units the gap rule compares with R_req: the box bottom 3 px lower, so a pitch error of that size can't stretch the range. A car first boxed at a ground range of 150 m reads about 113 m, one at 300 m about 182 m (camera 1.22 m up). The report prints each car's ground range too, for context only.

Route time's R_req includes the D4 margin (`ROUTE_REACH_MARGIN_M`, 15 m past the 9 m hold line at a junction with no mapped stop line). With that margin the reach is much longer than DESIGN's 134-141 m at 25 mph:

| Opposing limit | R_req, one lane mapped | R_req, lanes unmapped (two) |
| --- | --- | --- |
| 20 mph | 165 m | 171 m |
| 25 mph | 200 m | 207 m |
| 30 mph | 234 m | 243 m |
| 35 mph | 269 m | 279 m |

A 25 mph left therefore needs oncoming cars confirmed out to a `near_m` of 200 m, which is a ground range of about 350 m. The road camera's full pass alone is unlikely to reach that, so a demo left rests on the far crop. The far range only counts once `TurnPilotLeftRangeM` is above 0, because turnpilotd extends the reach by the far crop only past a measured road range.

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

After that, `car_preflight` prints NO-GO on `TurnPilotLeftRangeM`, because Saturday morning's required value is 0. That line is expected once the range is set deliberately.

## Find a site

Give `left_sites.py` the measured ranges and a point to search around. Use the map server's region, which is what the car's corridors are cut from:

```sh
cd ~/Projects/turnpilot/.claude/worktrees/tp-chase-range
PYTHONPATH=$PWD:$HOME/Projects/turnpilot/.claude/worktrees/tp-blinker/opendbc_repo \
  ~/Projects/turnpilot/.venv/bin/python -B -m openpilot.tools.turnpilot.left_sites \
  --tiles ~/turnpilot_data/mapserver/regions/indianapolis --range 110 --far-range 210 --radius 1500 <lat,lon>
```

It finds junctions in Valhalla's graph, routes each left from 150 m back on its approach, and judges the route with the fork's own functions (`upcoming_maneuvers`, `unprotected_turn`, `opposing_roads`, `route_controls`, `reach_at_line`). A site passes only if all of these hold:

- The turn is a left across one road straight on, and its bearings agree with its modifier.
- The controls are known, and the junction has no signal, stop or yield of ours.
- R_req, from the opposing road's mapped limit and lanes (45 mph and two lanes when unmapped), fits under the range.
- The opposing road is mapped straight past R_req.

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
