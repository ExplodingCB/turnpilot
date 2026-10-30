# Left range and left sites

`TurnPilotLeftRangeM` and `TurnPilotLeftFarRangeM` default to 0, so turnpilotd skips and reroutes every left across an oncoming approach. Two tools on this branch, both read only, decide whether a demo left is possible:

- `left_range.py` measures the two ranges on rlogs this fork recorded on the car, and prints the `preflight --set` line only when the evidence supports a range.
- `left_sites.py` lists the junctions near a point where a left passes route time at a given range, ranked by margin, with the reason every other candidate fails.

Neither tool writes a param. Kellen runs the `--set` line on the comma, parked, if he decides to.

## What the range means

The left goes when no road camera box (a vehicle at confidence 0.35 or more in the oncoming band) with a nearest plausible range (`near_m`) inside R_req has appeared for `CLEAR_WINDOW_S` (2.5 s). Route time attempts a left only when R_req fits under `TurnPilotLeftRangeM`, or past it under `TurnPilotLeftFarRangeM` while the far crop reads. So the range is safe only if every oncoming car inside it is boxed, with no gap long enough for the window to fill.

`left_range.py` counts a car as confirmed from the farthest `near_m` after which it never went `CLEAR_WINDOW_S / 3` (0.83 s) without a box. CRITIQUE REQUIRED 2 sets that ratio: the window must be at least 3 times the longest dropout. A stretch with no road run (the full pass on the wide camera, an uncalibrated spell, the detector stopped) counts as no box, so unknown is never a clear road.

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
- Drive straight two-way roads with steady oncoming traffic. On curves the full pass moves to the wide camera and the chain breaks.
- Plan for at least 60 oncoming cars that come within 80 m in the oncoming lanes. How long that takes depends on the road; the report says how many it found.

After the drive, parked, copy every segment of the route off the comma and run the tool on the Mac from this worktree:

```sh
ssh comma@<address> 'ls -t /data/media/0/realdata | head -20'
scp -r 'comma@<address>:/data/media/0/realdata/<route>--*' ~/chase_routes/
cd ~/Projects/turnpilot/.claude/worktrees/tp-chase-range
PYTHONPATH=$PWD:$HOME/Projects/turnpilot/.claude/worktrees/tp-blinker/opendbc_repo \
  ~/Projects/turnpilot/.venv/bin/python -B -m openpilot.tools.turnpilot.left_range ~/chase_routes/
```

The tool reads every rlog under the directories it is given, in segment order. It prints the drive's run counts, the number of oncoming cars sampled, the shortest five with why each one's confirmation ended, and a recommendation or the reason there is none. It reads no location service, so it prints no position. It refuses a sunnypilot log, since sunnypilot's own messages sit on `turnpilotDetections`' slot.

Exit status 0 means a road range is recommended, 2 means none (too few cars, nothing calibrated), and 1 means the log can't be used.

Each sample line gives its reason. `unboxed` means runs went on and none boxed the car, which is either the detector's reach or a car that pulled in from a side street. `noRuns` and `uncalibrated` are stretches with no usable road run. `ambiguous` means two boxes sat where the car could be, so the tool gave no credit. Look at the short ones before deciding: a car from a driveway caps the range as hard as a real miss.

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
