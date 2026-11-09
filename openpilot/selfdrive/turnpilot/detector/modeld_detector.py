"""The detector inside modeld: after modelV2 is sent, on frames modeld already uploaded, it runs YOLO26 on one camera and
publishes turnpilotDetections. The road camera sees far ahead on straights; the wide one sees the crosswalk and cross
traffic in turns. turnpilotPlan.detectorView (read through fresh_plan, shadow too) overrides that pick: the unprotected left
(unprotected_left.py) asks for the road camera from its go to its commit, whatever its turn desire (CRITIQUE REQUIRED 3).
It runs in modeld because the chestnut's USB interface is claimed by one process.

Each run on the road camera also reads the lane paint (scene/lane_paint.py) on that frame, from the host copy modeld made
of it, with the frame's modelV2 and calibration read back off the bus and the detections masking what they hide, and
publishes it in the same message, with lanePaintStatus saying why when it isn't read. A run on the wide camera reads none
on its own frame (no boxes for the road frame: unmasked, a white car over a yellow line reads as broken white paint), but
while the full pass looks through the wide camera (a turn, a curve, their WIDE_HOLD_S) the far crop's run on the frame
between reads it on that road frame, behind the far crop's boxes, where the step has room (read_odd_paint), and the next
wide run's message carries it (lanePaintStatus read): the wrong-side guard's paint on a turn's approach and exit (PF-03).
On comma's public comma four routes (lane_paint.py's, 37 turns), the guard's line was read on none of the moving approach,
turn and exit steps before and on 27%, 13% and 37% of them so (21%, 15% and 21% of the 24 in right-hand traffic, where the
guard runs), with no wrong-side fire added in any; without lane_paint's chromaticity test it added two, in a turn and an
exit. The far crop's boxes mask only its window, so the replay also read the same frames behind a full road pass's boxes
on the frame before (which the car doesn't have in a turn): the far side's ego line read broken white (the far-side
keep's evidence) on 304 of 4591 right-hand-traffic wide steps against 302 so, never where that read yellow.
A read took 7.5-8 ms on the comma four (0.79 ms on an M-series Mac), so until 11-01 the paint went off every drive and the
wrong-side guard and the lane changes had none; lane_paint.sample_lines, vectorized to the same output (2026-11-01), took
3.3-3.5 ms on the car (11-06). Its time is the time budget's (estimate()): on a road run that has room for the detector
and the lights but not the paint too, the paint is skipped (lanePaintStatus noRoom), the lights before the paint, and with
the car under PAINT_SKIP_V near a mapped signal with the light reader reading (lights_near) it is skipped outright: the
lane changes and the wrong-side guard read it only faster, the light reader standing (11-06: a 38-40 ms model, 10.4 ms
pass, 3.4 ms paint and 3.1-3.9 ms of lights at a junction's heads had no room together, so the road runs went forced only,
0.8-1.0 s apart, the 1 s camera light read aging out between them); with no reader reading there, nothing would take the
time it frees, so the paint is read as anywhere else. A paint skipped for room PAINT_FORCE_S running is read anyway, its estimate
starting over from that read, so a slow burst can't keep it skipped. Nothing turns it off: a median over PAINT_BUDGET_S
is warned once a minute (the 11-06 medians peaked at 4.29-4.59 ms against 5.0, a hair from the old off-for-the-drive
rule, against the no-features-off rule), and a failed read is retried after a backoff (lanePaintStatus retrying), logged
each time.

The traffic lights are read the same way (DESIGN phase 4 G2, detector/light_state.py): on road camera runs, from the host
copy of the frame, on the detector's class-9 boxes (decode_lights, never in the detections), with the frame's calibration
and modelV2 off the bus, into the message's lights (source camera, shadow unless TurnPilotCameraLights is addOnly), each
head with this frame's read and its reason (the gate that stopped it: 11-06's 667 night reads gave 0 greens and no colour
on 85% of plausible heads, and the log couldn't say why). Their time is the budget's too, a median over LIGHT_BUDGET_S
warned once a minute (11-06: 3.58-3.74 ms against 4.0), never off for the drive; a failed read is retried after a backoff
(lightStatus retrying), logged each time; the detector keeps running. A reader that fails LIGHT_DISABLED_FAILS reads in a
row says lightStatus disabled on its road runs (logged once) until a read succeeds, still retried: turnpilotd keeps that
read past its age-out and map_controls says it once (camera_lights_off), so a reader failing every retry all night is the
driver's to know, not only the log's (11-06 review: retrying alone reached no one). The boxes that never become heads (the
size and horizon gates, past MAX_READ_HEADS) are counted by the reader and logged with the heads' reasons at most once in
LIGHT_GATES_LOG_S (turnpilot.light_gates), for the night tuning. TurnPilotCameraLights off reads none.

Which camera (choose_camera): the wide one in a turn (desiredCurvature at or above WIDE_CURVATURE while moving) and
WIDE_HOLD_S after it, at a standstill and a crawl but for one road run in WIDE_SLOW_ROAD_EVERY, else the road one; a turn
desire before the car turns, and a signal mapped near (signal_near) at a crawl or in the turn with the light reader reading
(lights_near: TurnPilotCameraLights on, out of its retry backoff), alternate them 1:1, so the light reader has road frames
from a signalized turn's pulse to its line and through a stand at a red. 11-06: below
MIN_LAT_CONTROL_SPEED modeld freezes desiredCurvature, at 0.011-0.014 at the stands, and with a turn desire also holding
the wide camera, stands within 40 m of a signal had 8-17% road runs (1e 3+10 to 3+36: none in 259), a signalized right
none from its pulse at 39 m.

The far crop (DESIGN phase 4 G5, FarCrop): a FAR_CROP_SIZE window of the road camera's frame at native resolution
(graph.Crop: twice the full pass's pixel density, a car 1.8 m wide is 9 px at 230 m), run on the model frames between the
full pass's runs and published apart (turnpilotFarDetections), so it never takes a frame the full pass is due on, forced or
not: the full pass always wins, but far first (below). It is aimed by turnpilotPlan.farCropBearing (read through
fresh_plan, shadow too: the unprotected left aims it up the oncoming approach, else straight ahead) at the calibrated
horizon (FAR_HORIZON_ROW down the crop), and one compiled graph serves every aim: it takes two byte windows of modeld's
device copy of the frame (crop_view_offsets). Its own budget: a run only where its estimate, what its step costs (the run,
its decode, its message and its send, its lights apart: FarCrop.estimate), leaves the next full pass its room (the time
budget below). Nothing of its own turns it off for the drive since 2026-11-07 (no features off): a failed run is retried
after a backoff (FAR_DISABLED_FAILS in a row say farStatus disabled, which turnpilotd raises, until one succeeds), a median
run over FAR_BUDGET_S and FAR_OVERRUNS of its last TIME_WINDOW steps past DEBT_CAP_S are warned, and FAR_FORCE_S wanting a
frame with no room drops its measured costs so it measures again; nothing of the full pass's (its runs, its backoff) changes.
On the car it went off for the drive in both 11-06 drives' first minutes (1d 79.5 s and 1e 190.6 s into their logs, after 28 and 153 runs),
all of them between two wide runs (after a road run's 5.8 ms of debt its 2.7 never fit): the room check took its 10.5 ms
run alone, its step cost 2.4 ms more, so runs it fit left 6.5-7.1 ms of debt (median; p90 7.4-8.5, a 13-15 ms run 10-11),
and three of five past DEBT_CAP_S was its off rule. Its class-9 boxes go to a light reader of its own, always in shadow, its
time and failures ruled as the full pass's lights are (never off for the drive). TurnPilotFarCrop off, or no far pkl,
runs none (farStatus off).

Far first (plan_inputs' far_first: an unprotected left in hand, turnpilotPlan.detectorView road or roadAndWide, from
VIEW_FROM_M to its commit, whose R_req is past the full pass's range: the band past it the far slot's to read), standing or
slow only (under FAR_FIRST_V: at its line, its roll-up and creep, the go's first second). Each frame goes to the pass due
first that fits on its own step cost (left_pick): the far crop's next run FAR_DUE_S after its last, the full pass's
ROAD_DUE_S after its last road run (a wide run's WIDE_DUE_S after its last, or the road's sooner: the next road run waits on
it), 4 runs a second each, under unprotected_left's ROAD_FRESH_S and FAR_FRESH_S (0.5 s since 2026-11-07, L2: a quarter of
PET_S), past which a stream's gap starts the left's clear window over. So the far crop takes a full pass's frame when it is
due sooner, the full pass then running on the next frame it fits, and between the full runs it takes a frame where its run
leaves the next full pass its room. Both judged on what their steps cost (the full pass's measured past its run, paint and
lights: full_extra_s). The paint is skipped
outright (noRoom; read_odd_paint too), the car standing or creeping freeing the step's time: nothing reads the paint there.
The slot is the far crop's, the native's only without one (left_far): a native strip's 15.4 ms beside the car's model needs
a paid-back step and another after it. Moving it is auto's schedule (the 11-06 review, 2026-11-07: far first there halved
the road runs to 5 a second, the person hard brake's 5 hits in 1 s and the light vote with no margin, for far runs the
left's latch throws away): the full pass every RUN_EVERY frames, the far slot between them where it leaves the next its room,
which beside a road run's paint on the car's budget is never, so on the approach it reads nothing, as before far first.
On 11-06's timings replayed (tests/test_far_first.py), held at the line: the far crop about 4.1 reads a second where it read
never, the road camera's runs 4.15 (6.7 before), the wide camera's 2.1 (3.3), every step's debt within DEBT_CAP_S (the old
schedule's road runs, their step's extra uncounted, passed it on 20% of steps). 10 runs a second is all the frames hold
beside the model (each run's step costs more than one frame's slack, so the frame after it pays it back), and road and far
4 a second each and the wide camera 2 ask all 10: their gaps sit on the 0.3 s edge (10 minutes at 11-06's model: 3-4 of the
road camera's 2,490 gaps past it, 8-9 of the far crop's 2,480; at a model 1 ms slower 13-17 and 38). Judged by the left's
window (test_far_first.left_waits) with every timing spread as on 11-06, an empty road certifies at once on most holds (the
wait's p90 2.5 s at 11-06's model, 4.3 s at one 1 ms slower, at most 7.2 s); at a model 2 ms slower the far crop reads 3.1 a
second and a third of its gaps pass 0.3 s, so the left waits (a median of 15-19 s, some over a minute), never shorter, and
no far gap passes STALE_UNDECIDED_S (FAR_FORCE_S), so none goes blind past ~98 m (that replica, without unprotected_left's
far_behind; the real left there waits p50 2.5 s, test_far_first.real_left_waits). unprotected_left's ROAD_FRESH_S and
FAR_FRESH_S are 0.5 s since 2026-11-07 (L2: a quarter of PET_S, what the gap rule can spare between runs). A left's road camera runs 4 a second at its line
(LEFT_ROAD_RUNS_HZ, left_range.py's). Its boxes hold the left as the full pass's do, a car going the car's own way too:
unprotected_left's band widens by BAND_TOL_DEG with range, so the car's own lane is in it from 18 m out and the lane beside
from 121 m, and a far box blocks at any range where the left needs the far slot. So a car going straight on from ahead of
the car holds it until it is past the far crop's reach (the turnpilotd Sim, a 35 mph left, that car leaving at the limit
from 10 m past the junction: gone 15 s after the hold with far boxes to 210 m, 7.8 s with none), and traffic our way in the
lane beside past 121 m holds it as long as it is boxed there. Telling it from oncoming traffic is unprotected_left's band
rule's (the 11-06 review, 2026-11-07), unchanged here: the far crop's boxes stay a car the detector sees.
At a signalized left (review 2026-11-07, R1) the light reads cost 2-4.6 ms with 1-4 heads (11-06's rlogs; this tree's reader
~1.4x on a Mac): with the far crop's shadow reads on top of its step and the full pass judged with its step's extra, neither
stream fit and an empty road's window waited p50 8-19 s, or never filled. Far first now reads no far lights (nothing acts
on them at a left), a road run due (road_due) is judged as auto judges it, its frame never the far crop's, and one a frame
past due (road_overdue, review 2026-11-07 CS-1) runs whatever its debt: at 3 road heads at the 1.55x per head measured on
this tree's reader, or 3 at 1.4x with the model 0.7 ms slower, a due road run's own step left 8-10 ms of debt and was
refused until the forced run, and the real left's empty-road wait went p50 6-10 s, p90 17-21 s. Replayed with the 11-06
spread through the real UnprotectedLeft (test_far_first.real_left_waits, both seeds), an empty road's wait is p50 2.5 s, p90
2.5 s from 1 to 4 road heads at 1.0-1.55x and the model up to 0.7 ms slower (p90 2.9-3.8 s at 2 heads 1.55x and 1.5 ms,
4.1-5.2 s at 2 ms), the road camera 3.4-4.3 runs a second (its longest gap 8 frames), the far crop 2.7-4.3; up to 19% of
steps pass DEBT_CAP_S, the latest 17.9 ms, under half a frame (the car dropped no frame at 47.9 ms on 1e).

The native schedule (TurnPilotNativeDetect, NativeStrips; full by default, Kellen's ship call; off and nothing here differs
from the far crop alone): Kellen's whole camera frame at native resolution, in the far crop's slot. strip runs graph.NATIVE_SIZES' 1344x384
full-width strip of the road frame with the horizon at FAR_HORIZON_ROW (graph.crop_at, so every bearing at once: the plan's
aim moves nothing); full runs two 1344x512 strips, the frame's top (rows 0-511) and bottom (248-759), alternating, so the
whole frame natively every 200 ms and the horizon band, in both, every 100 ms. Its runs are the far crop's message
(turnpilotFarDetections, its crop the strip's rect, boxes in road frame pixels), so tracker B, the left's far reach, the far
lights and the odd paint read take them as they take the far crop's. Where the strips meet, a near object taller than the
264 shared rows is cut in both: the top strip's bottom edge is no frame edge, so a box there is one tracker B's crop-edge
rule drops (scene/tracker.py tile_boxes), and the bottom strip holds its feet with only its head cut, which that rule keeps;
every ground object's bottom is whole in one strip. It is loaded with the detector, for the mode the param says then (a
change mid-drive to another mode takes effect at the next load; to off, at once), and has its own budget: a run only where
its estimate leaves the next full pass its room (else the far crop takes that frame if it fits: never nothing); a failure,
a median run over NATIVE_BUDGET_S, or FAR_OVERRUNS of its last TIME_WINDOW runs pushing the debt past DEBT_CAP_S send it
back to the far crop for the drive, logged and said by every full pass message (nativeStatus fallback), the far crop then on its own budget
as without it. Measured on the chestnut beside the big model on the bench (models/README.md): the step 34.86 ms median with
strip, 35.71 ms with full, against 33.39 ms with the far crop; on the car the big model alone is 13 ms slower (the time
budget below), so a strip (10.7-12 ms) has no room on the frame after a road run, and the far crop takes it where it fits.
On the car it never ran (11-06, both drives: 0 native runs beside a 38.5 ms model: a strip's 14.9-15.4 ms warm run left
6.4 ms or more of debt from a paid-back step before its step's own 2.4, and the frame between the full runs had to leave
the next full pass its 1.7-5.8 ms too, so no frame could hold it; its median over NATIVE_BUDGET_S would have sent it back
to the far crop in five runs anyway), the far crop went off for the drive in its first minutes, and every message still
said nativeStatus full and farStatus noRoom, so turnpilotd took the far view for alive and never said farViewOff. So a
schedule loaded NATIVE_NO_RUN_S without a run says nativeStatus unavailable (logged once), and with the far crop off for
the drive (or none, or failing) and no native run for NATIVE_NO_RUN_S the slot says farStatus disabled, which turnpilotd
raises. That clock counts only the slot's free, calibrated time, where no room beside the
driving model is the one thing that can keep the native from running (native_idle_s, native_hold): a stop sweep holds the
slot for its side views, TurnPilotFarCrop off turns it off, a native that finds no calibration has no horizon to run on
(a fresh install, a recalibration: the 11-06 review's probe logged 'no room' there), and a gap in the steps ran nothing
past its first STEP_GAP_S (native_gap: a host stepping below 10 Hz still runs the clock, STEP_GAP_S a step), so their time
is off the clock; and a dead slot says disabled through a sweep, never sweep (which turnpilotd takes for a live far view),
so farViewOff is said once a drive, not again after every stop-sign hold.

The stop sweep (STOP_SIGNS.md section 1.3, SideViews): turnpilotPlan.detectorView stopSweep (held at a stop or yield) or
stopSweepGo (crossing, from the commit point), read through fresh_plan, shadow too. Its even steps (the full pass's, every
RUN_EVERY) run the forward views: the full wide pass and the road camera alternating when held, the full wide pass every
time when crossing, whatever the turn desire or the wide hold after a turn. Its odd steps (the far crop's otherwise)
alternate the two side views (sideview.py: level pinholes at yaw 75 deg left and right of the calibrated axis, 768x512,
gathered from the wide frame through maps that are graph inputs), but only with the car at or under SIDE_VIEW_V_MAX: above
it the odd steps idle, so the approach costs what the detector costs today. The side views' boxes go back to the wide
camera's fisheye pixels (camera wide, view sideLeft or sideRight), each with its sectors' image quality (sectorQuality) from
the host copy of the wide frame. Their maps follow the bus's calibration: recomputed off the step (a thread) when it moves
more than sideview.RECALIB_DEG, uploaded on an odd step, the views invalid until then (sideViewStatus mapsStale). Each
view's run is governed by the time budget on its own estimate (noRoom); a failure, a median run over SIDE_BUDGET_S, or
SIDE_OVERRUNS of the last TIME_WINDOW runs pushing the debt past DEBT_CAP_S turn them off for the drive (sideViewStatus disabled,
which turnpilotd says once, sideCamerasOff), and both cycles fall back to the forward views. While a sweep runs, the lane
paint isn't read (lanePaintStatus sweep), the light reader reads colour only with a signal mapped within SWEEP_SIGNAL_M
(lightStatus sweep otherwise), every forward run publishes its class-9 boxes (lightBoxes), and the far crop is suspended
(farStatus sweep). Every message says what the side views are doing (sideViewStatus).

Stop signs (COCO 11), on every forward run under TurnPilotCameraStopSigns shadow (the default; off reads none): decoded
apart into the message's signs, each with the luma below it for plaque labelling (decoder.fill_signs), never into its
detections, so nothing stops, caps or releases on one.

The time budget (STEP_BUDGET_S, DEBT_CAP_S): the detector runs after modelV2 is sent, so a step that takes longer than
modeld's 50 ms cycle drops nothing; it only starts the next frame late, and a lighter frame after it catches up. On the car
(comma four and chestnut, 2026-11-02, build 95a27d7) the big model takes 38.9 ms median (p99 40.2, max 41.1), not the
bench's 25-27 ms, and a full pass 10.4 ms (p99 10.7) with the lane paint 3.5 ms more on the road camera: model and detector
never fit a per-frame cap of 45 ms, and the detector ran only forced, 1.27 Hz. Those forced steps (about 53 ms) dropped no
model frame: a road run started the next frame 7.8 ms late (median; max 9.8), a wide one 3.5 ms, and the model-only frame
after it was back on time. So each step's driving model time (modeld's measured model_execution_time, so a slower host backs
off) and detector work past STEP_BUDGET_S is a debt the next step starts with (paid back by the frames under it), and a run
goes only where the debt it leaves stays within DEBT_CAP_S. A run on the frames between the full runs (the far crop, the
native schedule, the side views, the odd paint read) must also leave the next full pass its room, so the full pass always
wins (a run that, for all its estimate, left it none costs the full pass a frame, paid back by the next), but far first,
where the far crop's frames are as due as the full pass's (left_pick); one that pushed the debt past DEBT_CAP_S itself
counts against its own overrun budget. With the car's numbers the full pass runs every RUN_EVERY frame with the paint,
its debt (5.8 ms on the estimate, 7.4-8.5 measured, its step's own 1.3-2.2 ms past the run uncounted) paid back on the
frame after; the far crop's step (13.7 ms on 11-06) has no room after a road run and, judged on that cost, little between
the wide runs of a turn, so it reads where it matters, far first at a left's line; a 45 ms driving model gets only forced
runs, which stay the backstop.

Nothing here runs unless TurnPilotEnabled is set and the compiled pkl for this device and camera size exists
(compile_detector.py builds it; the pkls are not in git).

SIM ONLY, the demo's stale-detection fault (DetectorHost.sim_drop): with SIMULATION=1, TurnPilotSimDropDetectionsS set to
S > 0 (the scenario runner's --drop-detections-at; CLEAR_ON_MANAGER_START) skips every detector step for S s from the
check that first reads it, so turnpilotd's deadline hands back as it would for a detector gone silent. A value still set
after its drop is no new drop; only a new value is. Without SIMULATION=1 the param is never read, so nothing on the car
can trigger it, and with it unset nothing changes.

Once it's on it stays on for the drive, and nothing here turns it off quietly: without fresh detections turnpilotd hands
control back to the driver (a soft disable), so every gap is loud. A slow spell only skips frames until a forced run,
inside turnpilotd's deadline, measures the time again, a failed run is retried with a backoff and logged, a mid-drive
TurnPilotEnabled starts it, and a fallback to the small driving model switches to that model's detector, where one
exists.
"""
import math
import os
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from openpilot.cereal import custom, log, messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.turnpilot.detector.decoder import (LIGHT_READ_CONF, decode, decode_far, decode_far_lights, decode_lights,
                                                           decode_side, decode_signs, fill_detections, fill_far, fill_signs)
from openpilot.selfdrive.turnpilot.detector.graph import (FAR_CROP_SIZE, NATIVE_MODES, NATIVE_SIZES, SIDE_INPUT_SIZE, Crop, crop_at,
                                                         crop_views, detector_pkl_path, far_detector_pkl_path, letterbox,
                                                         native_detector_pkl_path, side_detector_pkl_path, side_input_specs)
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT, LightHead, LightReader, camera_lights_mode
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LanePaintClassifier, LinePaint, fill_lane_paint
from openpilot.selfdrive.turnpilot.detector.sideview import (RECALIB_DEG, SIDE_VIEWS, SectorQuality, SideGeometry, SideMap,
                                                             calib_moved_deg, compute_geometry)
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view, camera_height, road_lens, wide_lens
from openpilot.selfdrive.turnpilot.scene.tracker import predicted_path
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

# chestnut, 2026-10-27, yolo26n at 1344x760 on the bench: big model 25-27 ms, detector 7.9 ms in modeld (5.9 ms alone). On
# the car (2026-11-02, module docstring's time budget) the big model is 38.9 ms and the full pass 10.4 ms: every 2nd frame,
# its debt paid back on the frame between
RUN_EVERY = 2              # at most every 2nd model frame (10 Hz)
CYCLE_S = 1. / 20          # modeld's cycle
# a step more than this after the one before follows a gap (the host stepped nothing a while): it carries no debt (step), and
# the native's no-run clock counts this much of it, the rest nobody's (native_gap)
STEP_GAP_S = 2 * CYCLE_S
# the rest of modeld's step (the frame's wait and upload, its messages): the car's model-only frame after a road run paid back
# all of its 7.8 ms late start (2026-11-02), so the rest beside a 38.9 ms model is at most 3 ms
REST_S = 0.003
STEP_BUDGET_S = CYCLE_S - REST_S  # driving model + detector past this in a step are debt the next step starts with
# a run only where the debt it leaves stays within this. On the car a road run's step (model 38.9 + full pass 10.4 + paint
# 3.5 ms) carried 7.8 ms into the next (median; max 9.8) and modeld dropped no frame; the estimate projects 5.8 ms (8.0 at
# the model's max). A host 6 ms slower (a 45 ms model) projects 12: only forced runs
DEBT_CAP_S = 0.008
WIDE_CURVATURE = 0.01      # 1/m (a 100 m radius): at or above this while moving (MOVING_V), look through the wide camera
WIDE_HOLD_S = 2.0          # and keep looking through it this long after the turn ends
# modeld's MIN_LAT_CONTROL_SPEED: at or below it modeld freezes desiredCurvature at its last value (modeld.py
# get_action_from_model), so the curvature says nothing about a turn there. 11-06: frozen at 0.0107-0.0161 at the stands,
# over WIDE_CURVATURE, it held the wide camera through whole waits at red lights (1e 3+7 for 31 s, 4+9.5 for 20 s)
MOVING_V = 0.3             # m/s
# at a standstill and a crawl the full pass looks through the wide camera (SAT_VALIDATION rows 6 and X3: standing, the road
# camera's pass lost a walker 2.8 m ahead, below its view, and a release lifted the car past her), but for one run in
# WIDE_SLOW_ROAD_EVERY on the road camera, whose runs alone read the lights and the lane paint (turnpilotd ages a camera light
# read out at CAMERA_STALE_S, 1 s: the camera red's stop, the unsignalized release's live reader); with a signal mapped near
# (signal_near) one in 2, the wide camera keeping every other run for walkers at the corner. One model, one frame size:
# the same run's cost, inside room(). The far crop's odd steps on the road frame are unchanged
WIDE_SLOW_V = 3.0          # m/s
WIDE_SLOW_ROAD_EVERY = 3
# a turn desire before the car turns, a turn near a mapped signal and a crawl near one alternate the road and wide cameras
# 1:1 (choose_camera): the light reader from a signalized turn's pulse to its line, the wide camera every other run for the
# crosswalk
ALT_EVERY = 2
TURN_DESIRES = (log.Desire.turnLeft, log.Desire.turnRight)
# turnpilotPlan.detectorView (DESIGN phase 4 G4, CRITIQUE REQUIRED 3): road is the road camera only, overriding a turn's wide
# hold (an unprotected left from its go to its commit); roadAndWide the road camera with the wide one run in WIDE_EVERY (held
# at its line: its exit crosswalk); auto, choose_camera's own pick
WIDE_EVERY = 3
TIME_WINDOW = 5            # the detector time estimate is the median of this many runs, so one slow run can't stop it
# The estimate only changes when it runs, so after this many model frames skipped in a row, or this long by the clock
# (a loaded host runs the driving model below 20 Hz), it runs anyway and the estimate starts over from that run: a burst
# of slow runs under load can't keep it off once the load has gone. The wait never grows: turnpilotd hands back
# DETECTOR_DEADLINE_S (1.5 s) after the last detections, so a forced run must land well inside (DETECTOR_DEADLINE_S -
# STALE_S) of the one before; each may leave a debt past DEBT_CAP_S, one in 16 frames while nothing fits, which the frames
# after it pay back. A detector that can't run in time even so (a forced run itself too slow, failures) is a loud handback
# in turnpilotd, never quietly off.
FORCE_RUN_FRAMES = 16      # 0.8 s at 20 Hz
FORCE_RUN_S = 0.8
RETRY_S = 1.0              # a failed run is retried after this, doubled for each failure in a row...
RETRY_MAX_S = 60.          # ...to this
PARAM_CHECK_S = 1.0        # TurnPilotEnabled is read this often
# the lane paint read's median over TIME_WINDOW reads above this is warned (BUDGET_WARN_S), never off for the drive (the
# no-features-off rule): its time is in the estimate, so the time budget skips it where it doesn't fit (noRoom). On the car
# (2026-11-02) a read at it beside the 38.9 ms model and the 10.4 ms full pass projects a 7.3 ms debt, inside DEBT_CAP_S;
# the read there was 3.5 ms (median; p90 3.8); 11-06 medians peaked at 4.29-4.59 ms
PAINT_BUDGET_S = 0.005
LIGHT_BUDGET_S = 0.004     # the same for the light read (11-06: 3.1-3.9 ms a read at a junction's heads, medians to 3.74)
BUDGET_WARN_S = 60.        # a reader's median over its budget is warned at most this often
# a light reader with this many failed reads in a row says lightStatus disabled on its road runs (logged once) until a read
# succeeds, still retried on backoff_s: that is 3 s of nothing but failures (1 + 2 s of backoff), where one bad frame is one
# failure and its retry. turnpilotd keeps a disabled read past CAMERA_STALE_S and map_controls says it once
# (camera_lights_off): the 11-06 review found retrying alone reached no one, a reader failing every retry all night silent
LIGHT_DISABLED_FAILS = 3
# the light readers' funnel (LightReader.take_counts: boxes stopped at the size and horizon gates or past MAX_READ_HEADS,
# never heads, and the heads by reason and colour) is logged at most this often, for a window that saw a class-9 box
LIGHT_GATES_LOG_S = 60.
# a paint skipped for room (noRoom) this long running is read anyway on the next road run, its estimate starting over from
# that read (the full pass's FORCE_RUN_S, for its own estimate): a burst of slow reads can't keep it skipped once it's gone
PAINT_FORCE_S = 2.0
# under this with a signal mapped near, a road run skips the paint outright (noRoom), leaving the step's time to the lights:
# under every paint reader's floor (turnpilotd's wrong-side guard from WRONG_SIDE_MIN_SPEED 2.0 m/s, the lane planner's
# changes from LC_SPEED_MIN 2.74 m/s), so nothing that reads the paint goes without it
PAINT_SKIP_V = 2.0         # m/s
# the far crop (FarCrop). Chestnut, 2026-10-28, yolo26n on the 640x384 crop alone: 6.0 ms (the full pass 5.9 ms alone, 7.9 ms
# beside the big model); on the car (11-06, both drives) 10.5 ms median (p99 13.6) beside the 38.5 ms model, the full pass's
# 10.2. A median run over FAR_BUDGET_S, or FAR_OVERRUNS of its last TIME_WINDOW runs pushing the debt past DEBT_CAP_S, is
# warned at most once in BUDGET_WARN_S, never off for the drive (11-06: three of five past the cap turned it off 79.5 s
# into 1d's log and 190.6 s into 1e's, for the rest of both drives)
FAR_BUDGET_S = 0.012
FAR_OVERRUNS = 3
# what a far run's step costs past its run and its lights (decode, the message, the send), the estimate's until its own step
# costs are measured (FarCrop.costs). 11-06: a far run's step cost 2.4-2.5 ms (median) past them, the full pass's 1.3 (wide)
# to 2.2 ms (road, with its paint and lights); the room check took the run alone, so runs it fit left 6.5-7.1 ms of debt
# (median; p90 7.4-8.5, a 13-15 ms run 10-11)
FAR_STEP_OVERHEAD_S = 0.0025
# a far slot that wanted a frame this long and had no room on any measures again: its step costs dropped, the estimate the
# load's warm-up run plus FAR_STEP_OVERHEAD_S (the paint's PAINT_FORCE_S, the full pass's FORCE_RUN_S): a slow burst's median
# can't keep it from ever running again, its estimate never updated. The full pass's FORCE_RUN_S, well inside
# unprotected_left's STALE_UNDECIDED_S (2 s), past which a far-needed left's far slot counts as stale and the left is judged
# on the full pass's band alone: at 2 s (until the 11-06 review, 2026-11-07), with a model 2 ms slower than 11-06's and every
# timing spread as there, a left's line saw far gaps of 2.3-2.4 s (its median step cost, a slow run or two in it, fitting no
# frame), the left judged blind past ~98 m for the rest of each
FAR_FORCE_S = 0.8
# a failed far run is retried after backoff_s, logged each time, never off for the drive; this many in a row say farStatus
# disabled until a run succeeds (the lights' LIGHT_DISABLED_FAILS), which turnpilotd raises once (farViewOff)
FAR_DISABLED_FAILS = 3
FAR_STATUS_S = 0.5         # farStatus read: it ran within this
FAR_VIEW_CACHE = 64        # device views kept, one per crop position (graph.crop_at aligns them to a coarse grid)
FAR_CROP_DEFAULT = True    # TurnPilotFarCrop unset or unknown to this build
# the native schedule (NativeStrips). Chestnut, 2026-10-31, beside the big model: the 1344x384 strip 10.73 ms median, a 1344x512
# strip 12.02 ms (alone 8.29 and 9.06 ms); a median run over NATIVE_BUDGET_S sends it back to the far crop for the drive
NATIVE_BUDGET_S = 0.015
NATIVE_DEFAULT = 'full'    # TurnPilotNativeDetect unset, unknown to this build, or not one of NATIVE_MODES (35.7 ms step)
# a native schedule with no run this long after its load says nativeStatus unavailable; with the far crop off for the drive
# (or none), its slot with no native run this long says farStatus disabled, stop sweeps through too (module docstring, 11-06).
# Only the slot's free, calibrated time counts (native_idle_s): never a stop sweep's, TurnPilotFarCrop off, no calibration
# (native_hold) or a gap in the steps past its first STEP_GAP_S (native_gap)
NATIVE_NO_RUN_S = 30.
# far first (plan_inputs' far_first: an unprotected left in hand whose R_req is past the full pass's range, from VIEW_FROM_M
# to its commit; module docstring). Each frame goes to whichever of the full pass and the far crop is due first that fits
# on its own step cost, so the far crop takes a full pass's frame when its own is due sooner: each stream's next run due
# this long after its last, under unprotected_left's ROAD_FRESH_S and FAR_FRESH_S (0.5 s, past which a stream's gap starts
# the clear window over; 0.3 s, one frame past these, until 2026-11-07 L2), the wide camera's (roadAndWide: the exit
# crosswalk) twice that
ROAD_DUE_S = 0.25
FAR_DUE_S = 0.25
WIDE_DUE_S = 0.5
FAR_MIN_S = 1.5 * CYCLE_S  # a far run at most every other frame: the frame after one pays its debt back
# far first only standing or slow (the car at its line, its roll-up and creep, the go's first second): moving, the full pass
# keeps auto's cadence, its road runs 10 a second (the far crop between them where it leaves the next its room, which on the
# car's budget is never beside a road run's paint). The 11-06 review (2026-11-07): far first on the approach and after the go
# halved the road runs to 5 a second, the person hard brake's CONFIRM_RECENT_HITS 5 in CONFIRM_WINDOW_S 1 s (half of 10 Hz)
# with no margin (its tier held on 57.5% of frames at a 0.9 recall, against 100%) and the light vote's 4 of 5 twice as slow,
# for far runs the left throws away: the latch is taken under LATCH_SPEED and restarts the window and far_stale there. Under
# this the comfortable stop (scene_policy.STOP_DECEL, 1.5 m/s^2) stops the car within 1.4 m, so the hard tier's cadence isn't
# what holds it, and nothing reads the paint (PAINT_SKIP_V, its readers' floor)
FAR_FIRST_V = PAINT_SKIP_V
# so held at a left's line on the car's budget (11-06's timings replayed: detector/tests/test_far_first) the full pass's road
# runs come about 4.15 a second, the far crop's 4.1 and the wide camera's 2.1, where before they were 6.7 and 3.3 and the far
# crop's none: this rate, rounded down, is what tools/turnpilot/left_range.py stretches a drive's missed runs to at a left
# (LEFT_SCHEDULE)
LEFT_ROAD_RUNS_HZ = 4.0
# the step cost estimate of a full pass run before one is measured (ModeldDetector.costs): 11-06's 1.3-2.2 ms past its run,
# its paint and its lights (FAR_STEP_OVERHEAD_S)
FULL_STEP_OVERHEAD_S = 0.002
# the stop sweep (SideViews). Chestnut, 2026-10-28, yolo26n on the 768x512 side view alone: 7.0 ms (the full pass 5.9 ms alone,
# 7.9 ms beside the big model)
SWEEPS = ('stopSweep', 'stopSweepGo')
SIDE_VIEW_V_MAX = 2.0      # m/s (STOP_GO_V_MAX): side views only at or under this, inside a sweep
SIDE_BUDGET_S = 0.015      # a median side-view run over this (the odd step's room beside the driving model): off for the drive
SIDE_OVERRUNS = 3          # of the last TIME_WINDOW side runs pushing the debt past DEBT_CAP_S: off for the drive
SIDE_STATUS_S = 0.5        # sideViewStatus noRoom: the gate open this long with no side run
# a signal mapped within this, short of its line (plan_inputs' signal_near): during a sweep the light reader reads colour only
# then; out of one, with the reader reading (lights_near), it alternates the cameras at a crawl and in a turn (choose_camera)
# and skips the paint standing
SWEEP_SIGNAL_M = 60.
STOP_SIGNS_DEFAULT = 'shadow'  # TurnPilotCameraStopSigns: off, or shadow (logged, never acted on)
SIM_DROP_PARAM = 'TurnPilotSimDropDetectionsS'  # SIM ONLY (module docstring): s of no detector, read only with SIMULATION=1
LanePaintStatus = custom.TurnpilotDetections.LanePaintStatus
LightStatus = custom.TurnpilotDetections.LightStatus
FarStatus = custom.TurnpilotDetections.FarStatus
SideViewStatus = custom.TurnpilotDetections.SideViewStatus
NativeStatus = custom.TurnpilotDetections.NativeStatus


def frame_view(model_state, index: int):
  """A uint8 device view of packed frame `index` (0 main camera, 1 extra) in the input modeld uploads every step."""
  from tinygrad import Tensor, dtypes
  from tinygrad.uop.ops import UOp
  size = model_state.frame_copy_size
  offset = model_state.packed_input.nbytes - 2 * size + index * size
  view = model_state.input_device.view(size, offset).ensure_allocated()
  return Tensor(UOp.from_buffer(view, dtypes.uint8))


def crop_view_offsets(packed_nbytes: int, frame_size: int, index: int, crop: Crop) -> tuple[tuple[int, int], tuple[int, int]]:
  """(offset, size) in modeld's packed input of the far crop's two views (graph.crop_views) of packed frame `index`."""
  base = packed_nbytes - 2 * frame_size + index * frame_size
  (lo, ln), (co, cn) = crop_views(crop)
  return (base + lo, ln), (base + co, cn)


def crop_frame_views(model_state, index: int):
  """Crop -> (luma, chroma) uint8 device views of packed frame `index`, what the far crop's graph takes."""
  from tinygrad import Tensor, dtypes
  from tinygrad.uop.ops import UOp

  def views(crop: Crop):
    out = []
    for offset, size in crop_view_offsets(model_state.packed_input.nbytes, model_state.frame_copy_size, index, crop):
      out.append(Tensor(UOp.from_buffer(model_state.input_device.view(size, offset).ensure_allocated(), dtypes.uint8)))
    return tuple(out)
  return views


def far_crop_on(params) -> bool:
  """TurnPilotFarCrop: FAR_CROP_DEFAULT (on) if unset or unknown to this build's params (said)."""
  try:
    value = params.get("TurnPilotFarCrop")
  except UnknownKeyName:
    cloudlog.error(f"turnpilot: TurnPilotFarCrop unknown to this build's params, far crop {'on' if FAR_CROP_DEFAULT else 'off'}; run scons")
    return FAR_CROP_DEFAULT
  if value is None:
    return FAR_CROP_DEFAULT
  if isinstance(value, (bytes, str)):
    return (value.decode() if isinstance(value, bytes) else value).strip() == '1'
  return bool(value)


def native_detect_mode(params) -> str:
  """TurnPilotNativeDetect: off, strip or full (NATIVE_DEFAULT); unset, unknown to this build's params (said) or anything
  else is full. A load that can't serve it (no pkl, no strip for the camera, a failed run, an overrun) is the far crop."""
  try:
    value = params.get("TurnPilotNativeDetect")
  except UnknownKeyName:
    cloudlog.error(f"turnpilot: TurnPilotNativeDetect unknown to this build's params, native schedule {NATIVE_DEFAULT}; run scons")
    return NATIVE_DEFAULT
  value = (value.decode() if isinstance(value, bytes) else value or '').strip()
  return value if value in NATIVE_MODES else NATIVE_DEFAULT


def native_strips(cam_w: int, cam_h: int, mode: str) -> list[Crop] | None:
  """The full mode's strips (the frame's top and bottom, graph.NATIVE_SIZES), None for strip (aimed, FarCrop.aim) or off."""
  size = NATIVE_SIZES.get((cam_w, cam_h), {}).get(mode)
  if mode != 'full' or size is None:
    return None
  w, h = size
  return [Crop(cam_w, cam_h, 0, 0, w, h), Crop(cam_w, cam_h, 0, (cam_h - h) // 2 * 2, w, h)]


def camera_stop_signs_mode(params) -> str:
  """TurnPilotCameraStopSigns: off, or shadow (the default, and anything else: nothing acts on a stop sign seen this week)."""
  try:
    value = params.get("TurnPilotCameraStopSigns")
  except UnknownKeyName:
    cloudlog.error("turnpilot: TurnPilotCameraStopSigns unknown to this build's params, stop signs in shadow; run scons")
    return STOP_SIGNS_DEFAULT
  value = value.decode() if isinstance(value, bytes) else value
  return 'off' if value == 'off' else STOP_SIGNS_DEFAULT


def sim_drop_seconds(params) -> float:
  """TurnPilotSimDropDetectionsS as seconds, 0 when unset or unreadable; UnknownKeyName from a build without it. Called
  only with SIMULATION=1 (DetectorHost)."""
  value = params.get(SIM_DROP_PARAM)
  if isinstance(value, bytes):
    value = value.decode()
  try:
    return max(float(value), 0.) if value not in (None, '') else 0.
  except (TypeError, ValueError):
    return 0.


def backoff_s(failures: int) -> float:
  """The wait before a retry after `failures` failures in a row: RETRY_S, doubled for each, to RETRY_MAX_S (the full pass's,
  and its readers': a failure is retried, never off for the drive)."""
  return min(RETRY_S * 2 ** (failures - 1), RETRY_MAX_S)


def warn_over_budget(owner, attr: str, now: float, message: str) -> None:
  """A reader's median over its budget, warned at most once in BUDGET_WARN_S (owner.attr: when it was last, None never)."""
  last = getattr(owner, attr)
  if last is None or now - last >= BUDGET_WARN_S:
    setattr(owner, attr, now)
    cloudlog.warning(message)


def failing_status(failures: int) -> int:
  """lightStatus of a light reader with failures failed reads in a row, waiting out its retry: retrying, and from
  LIGHT_DISABLED_FAILS in a row disabled (the driver's to know), until a read succeeds."""
  return LightStatus.disabled if failures >= LIGHT_DISABLED_FAILS else LightStatus.retrying


def light_failed(owner, what: str) -> int:
  """A light read of owner's (owner.light_failures, light_retry_at) raised, in its except: retried after backoff_s, logged
  each time, never off for the drive; at LIGHT_DISABLED_FAILS in a row its status turns disabled, logged once. Returns the
  status to publish (failing_status)."""
  owner.light_failures += 1
  delay = backoff_s(owner.light_failures)
  owner.light_retry_at = time.monotonic() + delay
  cloudlog.exception(f"turnpilot {what} failed ({owner.light_failures} in a row), retrying in {delay:.0f} s")
  if owner.light_failures == LIGHT_DISABLED_FAILS:
    cloudlog.error(f"turnpilot {what}: {LIGHT_DISABLED_FAILS} failed reads in a row, lightStatus disabled until a read succeeds (still retried)")
  return failing_status(owner.light_failures)


def light_read_ok(owner, what: str) -> None:
  """A light read of owner's succeeded: its failures in a row start over, and a reader that had said disabled says so."""
  if owner.light_failures >= LIGHT_DISABLED_FAILS:
    cloudlog.warning(f"turnpilot {what} read again after {owner.light_failures} failed reads in a row")
  owner.light_failures = 0


def log_light_gates(owner, reader, now: float, source: str, signal_near: bool) -> None:
  """The light reader's funnel since its last log (LightReader.take_counts), at most once in LIGHT_GATES_LOG_S and only for
  a window that saw a class-9 box: the frames read, the boxes that never became heads (small, belowHorizon: its size and
  horizon gates; overCap: past MAX_READ_HEADS) and the heads by this frame's reason and colour. Light.reason carries the
  heads' alone, so without it the night tuning couldn't see how many boxes die at the gates (11-06 review: 667 night reads,
  0 greens). Replayed offline, the 11-06 drives' 673 class-9 boxes on road runs met the gates as 4 small and 6 below the
  horizon, 663 heads (631 of them unknown): that night the colour died past the gates, and this says whether a drive's
  does too. owner.light_gates_t: when its window started (None: no read yet); reader: a CameraLightReader."""
  inner = getattr(reader, 'reader', None)
  if not isinstance(inner, LightReader):
    return  # the device isn't known yet, or a stand-in
  start = owner.light_gates_t
  if start is None:
    owner.light_gates_t = now
    return
  if now - start < LIGHT_GATES_LOG_S or not inner.boxes_seen():
    return
  frames, gates, heads, colours = inner.take_counts()
  owner.light_gates_t = now
  cloudlog.event("turnpilot.light_gates", source=source, signal_near=signal_near, window_s=round(now - start, 1), frames=frames,
                 boxes=sum(gates.values()) + sum(heads.values()), gates=dict(gates), heads=dict(heads), colours=dict(colours))


LEFT_VIEWS = ('road', 'roadAndWide')  # turnpilotPlan.detectorView: only the unprotected left sets them (junction_watch)


def plan_inputs(plan) -> tuple[str, float, bool, bool]:
  """What modeld takes from fresh_plan's turnpilotPlan (shadow too): the detector's view, the far crop's bearing, whether
  a signal is mapped within SWEEP_SIGNAL_M, short of its line (signal_near: a sweep's light reader reads then, and out of a
  sweep choose_camera gives the light reader its road runs), and far_first: the unprotected left's view (from VIEW_FROM_M to
  its commit) with its R_req past the full pass's range (unprotectedLeft.rReq over rangeM), the band past it the far slot's
  (applied standing or slow only: ModeldDetector.detect, FAR_FIRST_V). Taken from R_req and the range alone, never from
  farNeeded, which goes false while the far slot is stale: a stale far view would then never get the frames to read again."""
  if plan is None:
    return 'auto', 0., False, False
  mc = plan.mapControl
  view, ul = str(plan.detectorView), plan.unprotectedLeft
  far_first = view in LEFT_VIEWS and float(ul.rReq) > float(ul.rangeM)
  return view, float(plan.farCropBearing), str(mc.kind) == 'signal' and 0. <= mc.distance <= SWEEP_SIGNAL_M, far_first


def frame_bus():
  return messaging.SubMaster(['modelV2', 'extrinsicsCalibration', 'deviceState', 'narrowRoadCameraState'])


class LanePaintReader:
  """Lane paint on the road camera's frame: modelV2 and calibration for that very frame come back off the bus (modeld has
  just sent the modelV2), and the lens from the device type."""
  def __init__(self, cam_w: int, cam_h: int, sm=None):
    self.sm = sm if sm is not None else frame_bus()
    self.stride, self.y_height, _, _ = get_nv12_info(cam_w, cam_h)
    self.classifier: LanePaintClassifier | None = None

  def read(self, frame: np.ndarray, meta, boxes) -> tuple[list[LinePaint], int]:
    """(paint, lanePaintStatus). frame: the road camera's padded NV12, flat; boxes: this frame's detections (x0 y0 x1 y1 ...),
    or None."""
    sm = self.sm
    sm.update(0)
    if self.classifier is None:
      if not (sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']):
        return [UNKNOWN] * 4, LanePaintStatus.noModel
      cam = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]
      self.classifier = LanePaintClassifier(road_lens(cam.narrow_road))
    model, cal = sm['modelV2'], sm['extrinsicsCalibration']
    if not sm.seen['modelV2'] or model.frameId != meta.frame_id % 2**32:
      return [UNKNOWN] * 4, LanePaintStatus.noModel  # not this frame's lines
    lines = [(np.array(ln.x), np.array(ln.y), np.array(ln.z)) for ln in model.laneLines]
    calibrated = sm.seen['extrinsicsCalibration'] and cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated
    return self.classifier.update(meta.timestamp_eof * 1e-9, frame, self.stride, self.y_height, list(cal.rpyCalib), calibrated,
                                  lines, list(model.laneLineProbs), boxes), LanePaintStatus.read


class CameraLightReader:
  """The lights on the road camera's frame (light_state.py), with the lane paint's bus: calibration for the horizon, and
  modelV2 for the lane's bearing when it is this very frame's."""
  def __init__(self, sm):
    self.sm = sm
    self.reader: LightReader | None = None

  def read(self, frame: np.ndarray, meta, boxes: np.ndarray) -> tuple[list[LightHead], int]:
    sm = self.sm
    sm.update(0)
    if self.reader is None:
      if not (sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']):
        return [], LightStatus.noCalibration
      cam = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]
      self.reader = LightReader(road_lens(cam.narrow_road))
      self.stride, self.y_height, _, _ = get_nv12_info(cam.narrow_road.width, cam.narrow_road.height)
    cal = sm['extrinsicsCalibration']
    if not (sm.seen['extrinsicsCalibration'] and cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated):
      return [], LightStatus.noCalibration
    model = sm['modelV2']
    path = predicted_path(model) if sm.seen['modelV2'] and model.frameId == meta.frame_id % 2**32 else None
    heads = self.reader.read(meta.timestamp_eof * 1e-9, frame, self.stride, self.y_height, list(cal.rpyCalib), camera_height(cal),
                             boxes, path)
    return heads, LightStatus.read


class FarCrop:
  """The far crop pass (module docstring): its compiled graph run(luma=, chroma=), views (Crop -> the two device views),
  its aim from the bus's calibration, its own time estimate and budget, and its own light reader. The native schedule's
  strips are one too (NativeStrips): its window's size, its budget, and fixed strips taken in turn instead of the aim."""
  size = FAR_CROP_SIZE
  budget_s = FAR_BUDGET_S
  mode = 'off'                     # the TurnPilotNativeDetect mode it serves (off: the far crop)
  strips: list[Crop] | None = None  # fixed windows taken in turn, one a run, instead of the aim (None: aimed)

  def __init__(self, run, cam_w: int, cam_h: int, views, sm=None):
    self.run, self.cam_w, self.cam_h, self.views_fn, self.sm = run, cam_w, cam_h, views, sm
    self.next = 0                   # the strip the next run takes (strips' index)
    self.lens: Lens | None = None
    self.times: deque[float] = deque(maxlen=TIME_WINDOW)  # its runs (executionTime)
    self.costs: deque[float] = deque(maxlen=TIME_WINDOW)  # its runs' step costs, their lights apart (estimate)
    self.overruns: deque[bool] = deque(maxlen=TIME_WINDOW)
    self.light_times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.warmup_s = 0.              # the load's warm-up runs' time, the estimate before its first run
    self.disabled = False           # off for the drive: the native schedule's fallback (NativeStrips), never the far crop's own
    self.failures = 0               # failed runs in a row (failed)...
    self.retry_at = 0.              # ...and the time.monotonic() their retry waits for (backoff_s)
    self.wanted_t: float | None = None  # the first frame since its last run it wanted and had no room for (no_room)
    self.budget_warn_t: float | None = None  # its budgets last warned (over_budget)
    self.force_warn_t: float | None = None   # its no-room measure-again last said (no_room)
    self.load_t = time.monotonic()  # its load (NATIVE_NO_RUN_S)
    self.last_run_t: float | None = None  # time.monotonic() of its last run
    self.held_s = 0.                # the ended holds' time since its last run (or load), off its no-run clock (native_idle_s)
    self.why = FarStatus.noRoom     # what kept it from running last
    self.no_run_said = False        # nativeStatus unavailable for no run in NATIVE_NO_RUN_S, logged
    self.lights: CameraLightReader | None = None  # its own (None: off)
    self.light_failures = 0         # its lights' failed reads in a row...
    self.light_retry_at = 0.        # ...and the time.monotonic() their retry waits for (backoff_s)
    self.light_warn_t: float | None = None  # its lights' median over LIGHT_BUDGET_S last warned
    self.light_gates_t: float | None = None  # its lights' gate window's start (log_light_gates)
    self._views: dict = {}

  def calibration(self) -> list | None:
    """rpyCalib when calibrated (and the road lens, once the device is known), else None."""
    sm = self.sm
    sm.update(0)
    if self.lens is None:
      if not (sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']):
        return None
      self.lens = road_lens(DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))].narrow_road)
    cal = sm['extrinsicsCalibration']
    if not (sm.seen['extrinsicsCalibration'] and cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated):
      return None
    return list(cal.rpyCalib)

  @staticmethod
  def view_ray(bearing_deg: float, rpy) -> np.ndarray:
    """The road camera's view-frame ray to the horizon bearing_deg left of the calibrated forward axis."""
    b = np.radians(bearing_deg)
    return calib_from_view(list(rpy)).T @ np.array([np.cos(b), -np.sin(b), 0.])  # calib frame: x forward, y right, z down

  def aim(self, bearing_deg: float, rpy) -> Crop:
    """The crop centred on the horizon bearing_deg left of the calibrated forward axis (graph.crop_at), at its size; with
    fixed strips, the next one."""
    if self.strips:
      return self.strips[self.next % len(self.strips)]
    assert self.lens is not None
    u, v = self.lens.project(self.view_ray(bearing_deg, rpy))
    return crop_at(self.cam_w, self.cam_h, float(u), float(v), self.size)

  def views(self, crop: Crop):
    key = (crop.x0, crop.y0)
    if key not in self._views:
      if len(self._views) >= FAR_VIEW_CACHE:
        self._views.clear()
      self._views[key] = self.views_fn(crop)
    return self._views[key]

  def estimate(self, lights_on: bool) -> float:
    """What a run's step costs, the room check's: the median of its measured step costs (costs: the run, its decode, its
    message and its send, its lights apart), else the load's warm-up run plus FAR_STEP_OVERHEAD_S; and its lights' median
    while they read. Until 2026-11-07 the run alone: 11-06's far runs left 2.4 ms more debt than it said, and three of five
    past DEBT_CAP_S turned the far crop off for the drive (FAR_STEP_OVERHEAD_S)."""
    est = float(np.median(self.costs)) if self.costs else self.warmup_s + FAR_STEP_OVERHEAD_S
    if lights_on and self.lights is not None and self.light_times:
      est += float(np.median(self.light_times))
    return est

  def failing(self) -> bool:
    """FAR_DISABLED_FAILS failed runs in a row, waiting out a retry or retried (said disabled until one succeeds)."""
    return self.failures >= FAR_DISABLED_FAILS

  def retrying(self, now: float) -> bool:
    """A failed run's retry is not yet due (backoff_s)."""
    return self.failures > 0 and now < self.retry_at

  def status(self, now: float) -> int:
    if self.disabled or self.failing():
      return FarStatus.disabled
    if self.last_run_t is not None and now - self.last_run_t <= FAR_STATUS_S:
      return FarStatus.read
    return self.why

  def disable(self, why: str) -> None:
    """Off for the drive: the native schedule's fallback to the far crop (NativeStrips), and a test's stand-in for a slot
    with no far crop; nothing of the far crop's own turns it off since 2026-11-07 (failed, over_budget)."""
    if not self.disabled:
      cloudlog.error(f"turnpilot far crop {why}, off for this drive")
    self.disabled = True

  def failed(self) -> None:
    """A run raised (in far_run's except, which logged it): retried after backoff_s, never off for the drive (the full
    pass's and its readers' rule); from FAR_DISABLED_FAILS in a row it says disabled (logged once) until a run succeeds."""
    self.failures += 1
    delay = backoff_s(self.failures)
    self.retry_at = time.monotonic() + delay
    cloudlog.error(f"turnpilot far crop failed ({self.failures} in a row), retrying in {delay:.0f} s")
    if self.failures == FAR_DISABLED_FAILS:
      cloudlog.error(f"turnpilot far crop: {FAR_DISABLED_FAILS} failed runs in a row, farStatus disabled until a run succeeds (still retried)")

  def no_room(self, now: float) -> None:
    """It wanted this frame and had none (noRoom): FAR_FORCE_S of that and its step costs are dropped, so the estimate falls
    back to the warm-up's and a frame with room for that measures it again (said at most once in BUDGET_WARN_S)."""
    if self.wanted_t is None:
      self.wanted_t = now
    if now - self.wanted_t >= FAR_FORCE_S and self.costs:
      ms = 1e3 * float(np.median(self.costs))
      self.costs.clear()
      self.wanted_t = now
      warn_over_budget(self, 'force_warn_t', now, f"turnpilot far crop: no room for {FAR_FORCE_S:.0f} s at {ms:.1f} ms; measuring again")

  def ran(self, now: float, dt: float, cost: float, overran: bool) -> None:
    """A run of dt s at now (its executionTime), its step costing cost s past its lights' read (the estimate's); overran:
    the step's debt after it (ModeldDetector.debt_after) past DEBT_CAP_S."""
    if self.failures >= FAR_DISABLED_FAILS:
      cloudlog.warning(f"turnpilot far crop ran again after {self.failures} failed runs in a row")
    self.last_run_t, self.held_s, self.failures, self.wanted_t = now, 0., 0, None
    self.next += 1
    self.times.append(dt)
    self.costs.append(cost)
    self.overruns.append(overran)
    self.over_budget(now)

  def over_budget(self, now: float) -> None:
    """The far crop's budgets, warned at most once in BUDGET_WARN_S, never off for the drive (no features off): a median run
    over budget_s, or FAR_OVERRUNS of its last TIME_WINDOW steps past DEBT_CAP_S. Its runs go only where its own estimate
    (its measured step costs) leaves the debt within the cap, so a slow spell only spaces them out."""
    if len(self.times) == TIME_WINDOW and float(np.median(self.times)) > self.budget_s:
      ms = 1e3 * float(np.median(self.times))
      warn_over_budget(self, 'budget_warn_t', now, f"turnpilot far crop over budget ({ms:.1f} ms against {self.budget_s * 1e3:.0f}): run where it fits")
    elif sum(self.overruns) >= FAR_OVERRUNS:
      n, cap_ms = sum(self.overruns), DEBT_CAP_S * 1e3
      said = f"turnpilot far crop pushed the debt past {cap_ms:.0f} ms {n} times in {len(self.overruns)} runs: run where it fits"
      warn_over_budget(self, 'budget_warn_t', now, said)


class NativeStrips(FarCrop):
  """The native schedule (module docstring): a FarCrop of the mode's strip size and NATIVE_BUDGET_S, aimed (strip) or its
  fixed strips in turn (full); off for the drive means back to the far crop, said: a failure, a median run over its budget
  or FAR_OVERRUNS of its last TIME_WINDOW steps past DEBT_CAP_S. The far crop goes on in the slot, so the far view is never
  off for it."""
  budget_s = NATIVE_BUDGET_S

  def __init__(self, run, cam_w: int, cam_h: int, views, sm=None, mode: str = 'strip'):
    super().__init__(run, cam_w, cam_h, views, sm)
    self.mode, self.size, self.strips = mode, NATIVE_SIZES[(cam_w, cam_h)][mode], native_strips(cam_w, cam_h, mode)

  def disable(self, why: str) -> None:
    if not self.disabled:
      cloudlog.error(f"turnpilot native schedule ({self.mode}) {why}: back to the far crop for this drive")
    self.disabled = True

  def failed(self) -> None:
    self.disable("failed")

  def over_budget(self, now: float) -> None:
    if len(self.times) == TIME_WINDOW and float(np.median(self.times)) > self.budget_s:
      self.disable(f"over budget ({np.median(self.times) * 1e3:.1f} ms)")
    elif sum(self.overruns) >= FAR_OVERRUNS:
      self.disable(f"pushed the debt past {DEBT_CAP_S * 1e3:.0f} ms {sum(self.overruns)} times in {len(self.overruns)} runs")


def upload_maps(name: str, side_map: SideMap, device: str | None = None) -> tuple:
  """A side view's maps on device (the wide frame's: SideViews.frame_device; None, tinygrad's default), as its graph takes them
  (luma_idx, chroma_idx, weights). On another device than the frame's the compiled graph refuses them (BUILD_FINDINGS N4: a
  JitError on the sim host, the frame on its AMD eGPU, the maps on METAL)."""
  from tinygrad import Tensor
  return tuple(Tensor(a, device=device).realize() for a in (side_map.luma_idx, side_map.chroma_idx, side_map.weights))


SIDE_MAPS_CORES = [0, 1, 2, 3]  # the little cores, where openpilot's own background daemons run


def side_maps_thread() -> None:
  """The side maps' worker leaves modeld's realtime slot. A thread inherits its creator's SCHED_FIFO 54 and core 7, where
  dmonitoringmodeld runs at FIFO 5: there compute_geometry's 0.1-0.3 s (more on the comma's CPU) would hold off driver
  monitoring, and modeld's own step, every time the calibration moves. Both calls act on this thread alone."""
  from openpilot.common.realtime import drop_realtime, set_core_affinity
  drop_realtime()
  set_core_affinity(SIDE_MAPS_CORES)


def side_maps_executor() -> ThreadPoolExecutor:
  return ThreadPoolExecutor(max_workers=1, thread_name_prefix='side_maps', initializer=side_maps_thread)


def blank_maps(device: str | None = None) -> tuple:
  """A view's maps of zeros on device, for the side graph's warm-up at load."""
  n = SIDE_INPUT_SIZE[0] * SIDE_INPUT_SIZE[1]
  return upload_maps('blank', SideMap(np.zeros(n, np.int32), np.zeros(n, np.int32), np.zeros((3, n), np.uint8)), device)


def check_side_device(art: dict, device: str) -> None:
  """The side pkl's inputs (input_specs, graph.side_input_specs: compile_detector writes the device it compiled for) are on
  device, the wide frame's. Raises ValueError otherwise: its jit would refuse every run."""
  from tinygrad import Device
  want = Device.canonicalize(device)
  got = {Device.canonicalize(v[2]) for v in art['input_specs'].values() if len(v) > 2 and v[2]}
  if got and got != {want}:
    raise ValueError(f"side views compiled for {sorted(got)}, the wide frame is on {want}")


class SideViews:
  """The stop sweep's side views (module docstring): their compiled graph run(input_frame=, luma_idx=, chroma_idx=,
  weights=), the wide frame's device view, their maps for the bus's calibration (geometry and its device copies),
  recomputed off the step by the executor, each view's own time estimate, and their budget for the drive."""
  def __init__(self, run, cam_w: int, cam_h: int, frame, sm=None, executor=None, upload=None, frame_device: str | None = None):
    self.run, self.cam_w, self.cam_h, self.frame, self.sm = run, cam_w, cam_h, frame, sm
    self.executor = executor
    self.frame_device = frame_device  # the wide frame's: every map the graph takes goes there (upload_maps)
    self.upload = upload or (lambda name, side_map: upload_maps(name, side_map, self.frame_device))
    self.lens: Lens | None = None
    self.times: dict[str, deque] = {name: deque(maxlen=TIME_WINDOW) for name in SIDE_VIEWS}  # each view's estimate
    self.recent: deque[float] = deque(maxlen=TIME_WINDOW)    # every side run's time, for SIDE_BUDGET_S
    self.overruns: deque[bool] = deque(maxlen=TIME_WINDOW)
    self.quality_times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.warmup_s = 0.              # the load's warm-up runs' time, the estimate before a view's first run
    self.disabled = False           # failed or over budget: off for the drive
    self.geometry: SideGeometry | None = None  # the maps on the device are for this calibration
    self.device: dict[str, tuple] = {}
    self.pending = None             # (future, rpy, wide euler) of a recompute under way
    self.staged: dict[str, tuple] = {}  # its views' maps uploaded so far, one an odd step
    self.why = SideViewStatus.noCalibration  # why the maps aren't ready
    self.next = 0                   # the side the next odd step runs (SIDE_VIEWS index)
    self.last_run_t: float | None = None  # time.monotonic() of the last side run
    self.started_t: float | None = None   # when the speed gate last opened
    self.quality: SectorQuality | None = SectorQuality()  # None: failed, off for the drive (no sectors: none ok)

  def calibration(self) -> tuple[list, list] | None:
    """(rpyCalib, wideFromDeviceEuler) when calibrated (and the wide lens, once the device is known), else None."""
    sm = self.sm
    sm.update(0)
    if self.lens is None:
      if not (sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']):
        return None
      device_type = str(sm['deviceState'].deviceType)
      self.lens = wide_lens(DEVICE_CAMERAS[(device_type, str(sm['narrowRoadCameraState'].sensor))].wide_road, device_type)
      cloudlog.info(f"turnpilot side views: wide lens {self.lens.spec}")
    cal = sm['extrinsicsCalibration']
    if not (sm.seen['extrinsicsCalibration'] and cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated):
      return None
    if len(cal.rpyCalib) != 3 or len(cal.wideFromDeviceEuler) != 3:
      return None
    return list(cal.rpyCalib), list(cal.wideFromDeviceEuler)

  def prepare(self, upload: bool = True) -> bool:
    """Whether the maps on the device are for the calibration (within RECALIB_DEG). If not, the views are invalid at once,
    their recompute runs off the step, and once it finishes they are uploaded one view a step (upload: this step may; it
    then runs no view; both views' maps at once took 27 ms on the chestnut, more than an odd step's room). why says what
    they wait for."""
    cal = self.calibration()
    if cal is None or self.lens is None:
      self.why = SideViewStatus.noCalibration
      return False
    rpy, euler = cal
    geom = self.geometry
    if geom is not None:
      moved = calib_moved_deg(geom.rpy, geom.wide_euler, rpy, euler)
      if moved <= RECALIB_DEG and geom.lens == self.lens:
        return True
      cloudlog.warning(f"turnpilot side views: calibration moved {moved:.2f} deg, invalid until their maps are recomputed")
      self.geometry, self.device = None, {}
    self.why = SideViewStatus.mapsStale
    if self.pending is not None and calib_moved_deg(self.pending[1], self.pending[2], rpy, euler) > RECALIB_DEG:
      self.pending, self.staged = None, {}  # for a calibration since left behind: never used
    if self.pending is None:
      if self.executor is None:
        self.executor = side_maps_executor()
      self.pending = (self.executor.submit(compute_geometry, self.lens, rpy, euler, self.cam_w, self.cam_h), rpy, euler)
    future = self.pending[0]
    if upload and future.done():
      geom = future.result()  # a failed recompute raises: the side views' failure, off for the drive
      name = next(n for n in SIDE_VIEWS if n not in self.staged)
      t0 = time.perf_counter()
      self.staged[name] = self.upload(name, geom.maps[name])
      rpy_deg = np.round(np.degrees(geom.rpy), 2).tolist()
      cloudlog.warning(f"turnpilot side views: {name}'s maps for rpy {rpy_deg} deg on the device in {1e3 * (time.perf_counter() - t0):.1f} ms")
      if len(self.staged) == len(SIDE_VIEWS):
        self.pending, self.device, self.staged, self.geometry = None, self.staged, {}, geom
    return False

  def estimate(self, name: str) -> float:
    est = float(np.median(self.times[name])) if self.times[name] else self.warmup_s
    if self.quality is not None and self.quality_times:
      est += float(np.median(self.quality_times))
    return est

  def gate(self, open_: bool, now: float) -> None:
    """The speed gate this step: when it opens, noRoom's clock starts."""
    if not open_:
      self.started_t = None
    elif self.started_t is None:
      self.started_t = now

  def status(self, now: float, sweep: bool, gated: bool) -> int:
    if self.disabled:
      return SideViewStatus.disabled
    if not sweep:
      return SideViewStatus.idle
    if not gated:
      return SideViewStatus.speedGated
    if self.geometry is None:
      return self.why
    last = max(t for t in (self.last_run_t, self.started_t, -np.inf) if t is not None)
    return SideViewStatus.running if now - last <= SIDE_STATUS_S else SideViewStatus.noRoom

  def disable(self, why: str) -> None:
    if not self.disabled:
      cloudlog.error(f"turnpilot side views {why}, off for this drive: stop sweeps run the full wide pass and the road camera")
    self.disabled = True

  def ran(self, name: str, now: float, dt: float, overran: bool) -> None:
    """A run of view name, dt s at now; overran: with its sectors, the step's debt after it past DEBT_CAP_S."""
    self.last_run_t = now
    self.times[name].append(dt)
    self.recent.append(dt)
    self.overruns.append(overran)
    if len(self.recent) == TIME_WINDOW and float(np.median(self.recent)) > SIDE_BUDGET_S:
      self.disable(f"over budget ({np.median(self.recent) * 1e3:.1f} ms)")
    elif sum(self.overruns) >= SIDE_OVERRUNS:
      self.disable(f"pushed the debt past {DEBT_CAP_S * 1e3:.0f} ms {sum(self.overruns)} times in {len(self.overruns)} runs")


def fill_lights(tl, heads: list[LightHead], shadow: bool) -> None:
  """A TurnpilotLights builder from the camera's heads: each head's vote, and this frame's read and its reason (the gate
  that stopped it), so a night drive's rlog says which gate kills the colour (11-06: 667 reads, 0 greens, unexplained).
  Since 2026-11-07 the read's detail (light_state's HeadRead.detail: the thirds' shares, the lit run, its core, the hues)
  rides on the instance after the head's id, so every road run's rlog carries it, not only the SAMPLE_S samples (instance
  is Text and only ever logged; turnpilotDetections' lights are no accepted source)."""
  tl.source, tl.shadow = custom.TurnpilotLights.Source.camera, shadow
  out = tl.init('lights', len(heads))
  for o, h in zip(out, heads, strict=True):
    o.state, o.distance, o.instance = h.state, float(h.distance), f"head{h.id} {h.detail}".rstrip()
    o.conf, o.box, o.bearing, o.votes, o.governs = float(h.conf), [float(b) for b in h.box], float(h.bearing), int(h.votes), h.governs
    o.read, o.reason = h.read, str(h.reason)


class ModeldDetector:
  failures = 0                    # failed runs in a row
  retry_at = 0.0                  # time.monotonic() a failed detector waits for
  last_run_t: float | None = None  # time.monotonic() of the last run (None: none yet)
  light_mode = LIGHT_MODE_DEFAULT  # TurnPilotCameraLights, set by DetectorHost
  lights: CameraLightReader | None = None  # None: no road camera
  light_failures = 0              # failed light reads in a row...
  light_retry_at = 0.             # ...and the time.monotonic() their retry waits for (backoff_s)
  light_warn_t: float | None = None  # the lights' median over LIGHT_BUDGET_S last warned (warn_over_budget)
  light_gates_t: float | None = None  # the lights' gate window's start (log_light_gates)
  paint_failures = 0             # failed paint reads in a row...
  paint_retry_at = 0.             # ...and the time.monotonic() their retry waits for
  paint_warn_t: float | None = None  # the paint's median over PAINT_BUDGET_S last warned
  paint_skipped_t: float | None = None  # time.monotonic() of the first skip for room since the last paint read (PAINT_FORCE_S)
  model_time = 0.                 # this step's driving model time (s), for the logs
  view_runs = 0                   # runs under the roadAndWide view, for its one wide run in WIDE_EVERY
  slow_runs = 0                   # runs at or under WIDE_SLOW_V in auto, for its one road run in WIDE_SLOW_ROAD_EVERY
  alt_runs = 0                    # runs in auto's 1:1 road and wide alternation (ALT_EVERY), the wide first
  far: FarCrop | None = None      # the far crop (None: no road camera, no far pkl, or it failed to load)
  far_on = FAR_CROP_DEFAULT       # TurnPilotFarCrop, set by DetectorHost
  native: NativeStrips | None = None  # the native schedule loaded with it (None: off at load, no pkl, or it failed to load)
  native_mode = NATIVE_DEFAULT    # TurnPilotNativeDetect, set by DetectorHost
  side: SideViews | None = None   # the side views (None: no wide camera, no side pkl, or it failed to load)
  signs_mode = STOP_SIGNS_DEFAULT  # TurnPilotCameraStopSigns, set by DetectorHost
  sweep: str | None = None        # the stop sweep running (SWEEPS), or None
  # why the native's slot isn't its own to miss now (native_hold: sweep, farOff, noCalibration), and since when (None: free)
  native_holds: frozenset = frozenset()
  hold_t0: float | None = None
  sweep_runs = 0                 # forward runs in this sweep, for the held cycle's full wide and road alternation
  signal_near = False             # a signal mapped within SWEEP_SIGNAL_M (plan_inputs), this step
  lights_near = False             # ...and the light reader reading (light_reading): the 1:1 alternation's and the paint skip's
  steps = 0                       # step() calls, for odd_paint's age
  last_cam: str | None = None     # the camera of the last full run
  odd_paint: tuple | None = None  # (paint, lanePaintStatus, its time, steps) read on the far crop's frame (read_odd_paint)
  debt = 0.                       # s this step started late: the last one's time past STEP_BUDGET_S (the time budget)
  last_step_t: float | None = None  # time.monotonic() at the last step()'s start
  step_t0 = 0.                    # time.perf_counter() at this step()'s start
  next_cam: str | None = None     # the camera a full run would take on this frame (choose_camera's peek), for odd_room
  far_first = False               # plan_inputs' far_first, this step: the far crop's frames as due as the full pass's (left_pick)
  v_ego: float | None = None      # carState.vEgo, this step
  # the full pass's last run on each camera (time.monotonic()), the far-first pick's deadlines (ROAD_DUE_S, WIDE_DUE_S); made
  # on the first step (a test's detector skips __init__)
  run_t: dict | None = None
  # the full pass's step costs past its run's own time, paint and lights (decode, its message, the send): the far-first pick
  # judges both passes on what their steps cost (full_cost), the time budget elsewhere on the runs' times as before
  full_extra: deque | None = None

  def __init__(self, model_state, cam_w: int, cam_h: int, cameras: dict[str, int]):
    """cameras: 'road' and/or 'wide' -> index of its frame in modeld's packed input."""
    from tinygrad.engine.realize import lower_and_compile
    from openpilot.selfdrive.modeld.helpers import load_oob
    art = load_oob(detector_pkl_path(cam_w, cam_h, model_state.chestnut), model_state.chestnut)
    if art['input_specs']['input_frame'][0] != (model_state.frame_copy_size,):
      raise ValueError(f"detector compiled for frames of {art['input_specs']['input_frame'][0]}, modeld has {model_state.frame_copy_size}")
    self.run = art['run']
    self.run.captured._linear = lower_and_compile(self.run.captured._linear)
    self.lb = letterbox(cam_w, cam_h, tuple(art['metadata']['input_size']))
    self.model_state = model_state
    self.cameras = cameras
    self.frames = {cam: frame_view(model_state, i) for cam, i in cameras.items()}
    self.times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.paint_times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.light_times: deque[float] = deque(maxlen=TIME_WINDOW)
    for _ in range(3):  # warm up, and a first time estimate
      t0 = time.perf_counter()
      self.run(input_frame=self.frames[next(iter(self.frames))]).numpy()
      self.times.append(time.perf_counter() - t0)
    self.since_run, self.last_run_t = RUN_EVERY, time.monotonic()
    self.wide_until = -1.0
    bus = frame_bus()
    self.lane_paint = LanePaintReader(cam_w, cam_h, bus) if 'road' in cameras else None
    self.lights = CameraLightReader(bus) if 'road' in cameras else None
    self.bus = bus
    self.far = load_far(model_state, cam_w, cam_h, cameras['road'], bus) if 'road' in cameras else None
    self.side = load_side(model_state, cam_w, cam_h, cameras['wide'], bus) if 'wide' in cameras else None


  def choose_camera(self, desired_curvature: float, desire: int, now: float, view: str = 'auto', v_ego: float | None = None,
                    peek: bool = False) -> str:
    """The camera for this run (module docstring), by the plan's view (WIDE_EVERY, the sweeps) and in auto by this table,
    v_ego None unknown:
      turning (desiredCurvature at or above WIDE_CURVATURE, moving over MOVING_V or speed unknown), and WIDE_HOLD_S after:
        the wide camera; with lights_near, the road and wide cameras 1:1 (ALT_EVERY)
      a turn desire before the car turns: 1:1
      at or under WIDE_SLOW_V: one road run in WIDE_SLOW_ROAD_EVERY; with lights_near, 1:1
      else: the road camera
    A turn desire alone no longer holds the wide camera, nor a curvature at a standstill, which modeld freezes there (11-06:
    the stands and every signalized turn's approach were wide only, so no light was read where a red must stop the car).
    The wide camera keeps every other run near a signal and in a turn's approach (its crosswalk), 2 of 3 at a crawl.
    lights_near is signal_near with the light reader reading (light_reading): with TurnPilotCameraLights off or the reader
    in its retry backoff a signal's 1:1 would take the wide camera's runs for no read (11-06 review). A device with one
    camera runs it. peek: the same pick, its cycles left as they were (the time budget's estimate, before the run).
    self.lights_near and self.alt_runs are read only in the rows that use them."""
    cycles = {}  # the cycles' next values, set on self unless peek
    wide_until = self.wide_until
    turning = abs(desired_curvature) >= WIDE_CURVATURE and (v_ego is None or v_ego > MOVING_V)
    if turning:
      wide_until = cycles['wide_until'] = now + WIDE_HOLD_S
    slow = alternate = False
    want = 'road'
    if view == 'stopSweep':  # held: the full wide pass and the road camera alternating, the wide first
      want = 'wide' if self.sweep_runs % 2 == 0 else 'road'
      cycles['sweep_runs'] = self.sweep_runs + 1
    elif view == 'stopSweepGo':  # crossing: the full wide pass every time
      want = 'wide'
    elif view == 'road':
      want = 'road'
    elif view == 'roadAndWide':
      cycles['view_runs'] = (self.view_runs + 1) % WIDE_EVERY
      want = 'wide' if cycles['view_runs'] == 0 else 'road'
    elif now < wide_until:
      want, alternate = 'wide', self.lights_near
    elif desire in TURN_DESIRES:
      alternate = True
    elif v_ego is not None and v_ego <= WIDE_SLOW_V:
      slow = True
      if self.lights_near:
        alternate = True
      else:
        want = 'road' if self.slow_runs == WIDE_SLOW_ROAD_EVERY - 1 else 'wide'
        cycles['slow_runs'] = (self.slow_runs + 1) % WIDE_SLOW_ROAD_EVERY
    if alternate:
      want = 'road' if self.alt_runs == ALT_EVERY - 1 else 'wide'
      cycles['alt_runs'] = (self.alt_runs + 1) % ALT_EVERY
    else:
      cycles['alt_runs'] = 0
    if not slow:
      cycles['slow_runs'] = 0
    if not peek:
      for name, value in cycles.items():
        setattr(self, name, value)
    return want if want in self.frames else next(iter(self.frames))

  def estimate(self, cam: str | None = None, paint: bool = True) -> float:
    """The full pass's time estimate on camera cam (None: unknown, as the road camera): the detector's, and on the road
    camera the lane paint read's (unless paint is False: this run skips it) and the lights' while they would read (a sweep
    reads no paint, and lights only near a mapped signal; a reader waiting out its retry reads nothing). A wide run reads
    neither: on the car (2026-11-02) its step carried 3.5 ms into the next, a road run's 7.8. Far first, the step's cost
    past them too (full_extra_s), as the far crop's estimate is its step's (FarCrop.estimate)."""
    estimate = float(np.median(self.times)) if self.times else 0.
    if self.far_first and not self.road_due(time.monotonic()):
      # judged on its step's cost against the far crop's (left_pick), but a road run due (ROAD_DUE_S) is judged as auto
      # judges it. Review 2026-11-07 (R1): at a signalized
      # left's line the light reads (2-4.6 ms with 1-4 heads on 11-06's rlogs, this tree's reader ~1.4x on a Mac) left no
      # step a road run fit on with the extra: road runs fell to the 0.8 s forced ones, 1.2-1.6 a second, and an empty
      # road's window never filled (3 heads: 200 of 200 holds over 60 s)
      estimate += self.full_extra_s()
    if cam == 'wide':
      return estimate
    now = time.monotonic()
    if paint and self.sweep is None and self.lane_paint is not None and self.paint_times and now >= self.paint_retry_at:
      estimate += float(np.median(self.paint_times))
    if self.light_reading(now) and self.light_times:
      estimate += float(np.median(self.light_times))
    return estimate

  def road_due(self, now: float) -> bool:
    """The full pass's road run is due (full_deadline's: ROAD_DUE_S after its last road run, to the frame: half a CYCLE_S
    either way, the run's time read at its step's end and each frame's model time its own), far first."""
    return (self.run_t or {}).get('road', -math.inf) + ROAD_DUE_S <= now + CYCLE_S / 2

  def full_extra_s(self) -> float:
    """A full pass step's measured cost past its run, paint and lights (full_extra's median; FULL_STEP_OVERHEAD_S before one)."""
    return float(np.median(self.full_extra)) if self.full_extra else FULL_STEP_OVERHEAD_S

  def full_deadline(self, now: float) -> float:
    """When the full pass's next run is due, far first (left_pick): ROAD_DUE_S after its last road run on the road camera,
    and for a wide run (roadAndWide's one in WIDE_EVERY) WIDE_DUE_S after its last, or sooner where the road stream's falls
    sooner, the next road run waiting on it. -inf before the camera's first run."""
    run_t = self.run_t or {}
    road = run_t.get('road', -math.inf) + ROAD_DUE_S
    if self.next_cam != 'wide':
      return road
    return min(run_t.get('wide', -math.inf) + WIDE_DUE_S, road)

  def debt_after(self, model_execution_time: float, more_s: float = 0.) -> float:
    """The debt this step leaves the next if it spends more_s s more (the time budget): the debt it started with, plus the
    driving model's time and the detector's work so far and more_s, past STEP_BUDGET_S. Never more than a cycle: a step a
    whole cycle late drops a frame, and the next starts on a fresh one."""
    used = model_execution_time + (time.perf_counter() - self.step_t0) + more_s
    return min(CYCLE_S, max(0., self.debt + used - STEP_BUDGET_S))

  def odd_room(self, model_execution_time: float, more_s: float = 0.) -> bool:
    """Whether a run of more_s s on a frame between the full runs leaves the next full pass its room: the debt it leaves,
    plus the full pass's own past STEP_BUDGET_S at this model time on the camera it would take on this frame (next_cam),
    within DEBT_CAP_S. A full pass that can't fit even with no debt runs only forced, and has no room to keep."""
    full = max(0., model_execution_time + self.estimate(self.next_cam) - STEP_BUDGET_S)
    return self.debt_after(model_execution_time, more_s) + (full if full <= DEBT_CAP_S else 0.) <= DEBT_CAP_S

  def fits(self, model_execution_time: float, cam: str | None = None, paint: bool = True) -> bool:
    """Whether a run on camera cam (paint: with the lane paint read) leaves a debt within DEBT_CAP_S, on its estimate."""
    return self.debt_after(model_execution_time, self.estimate(cam, paint)) <= DEBT_CAP_S

  def road_overdue(self, now: float) -> bool:
    """Far first, the full pass's road run a frame past due (road_due's ROAD_DUE_S, and a CYCLE_S more: 0.3 s since its
    last), so the road camera's band, the one the left falls back on (unprotected_left far_behind), keeps its 3.3 runs a
    second or more whatever the step costs, its gaps well inside unprotected_left's ROAD_FRESH_S."""
    return (self.run_t or {}).get('road', -math.inf) + ROAD_DUE_S + CYCLE_S <= now + CYCLE_S / 2

  def room(self, model_execution_time: float, cam: str | None = None, paint: bool = True) -> bool:
    """Whether this frame has room for a run on camera cam (paint: with the lane paint read): the debt it would leave
    (debt_after, on the estimate) stays within DEBT_CAP_S, or the detector has gone FORCE_RUN_FRAMES or FORCE_RUN_S without
    and runs to measure again. Far first, a road run a frame past due runs whatever its debt (road_overdue): review
    2026-11-07 (CS-1), at a signalized left's line with 3 road heads (this tree's reader 1.55x per head) or the model 0.7 ms
    slower, a road run's own step left 8-10 ms of debt from a paid-back step and was refused, left_pick gave its frame to
    no far run, and the road camera fell to the forced runs: the left's window kept starting over (p50 6-10 s, p90 17-21 s
    on an empty road). Run, its step's lateness stays under a third of a frame (test_far_first.within_cap; the car dropped no
    frame at lateness up to 47.9 ms on 1e)."""
    estimate = self.estimate(cam, paint)
    if self.debt_after(model_execution_time, estimate) <= DEBT_CAP_S:
      return True
    if self.far_first and cam != 'wide' and self.road_overdue(time.monotonic()):
      return True
    waited_s = time.monotonic() - self.last_run_t if self.last_run_t is not None else 0.
    if self.since_run < FORCE_RUN_FRAMES and waited_s < FORCE_RUN_S:
      return False
    model_ms, debt_ms = 1e3 * model_execution_time, 1e3 * self.debt
    cloudlog.info(f"turnpilot detector: no room for {waited_s:.2f} s at {1e3 * estimate:.0f} ms, model {model_ms:.0f}, debt {debt_ms:.0f}; measuring again")
    self.times.clear()
    return True

  def paint_skip(self, model_execution_time: float, cam: str | None, v_ego: float | None) -> int | None:
    """Why a full run on camera cam skips the lane paint (LanePaintStatus noRoom), or None: it reads it as before (or reads
    none anyway: the wide camera, a sweep, no reader, its retry). The lights before the paint: with the car under
    PAINT_SKIP_V near a mapped signal and the light reader reading (lights_near) it is skipped outright (the lane changes
    and the wrong-side guard read it only faster, the light reader standing), and where the run has room without the paint
    and not with it, skipped for room (the estimate counts the lights only while they read); a paint skipped for room
    PAINT_FORCE_S running is read anyway, its estimate starting over from that read (said). With TurnPilotCameraLights off
    or the reader in its retry backoff the outright skip would free the step's time for nothing (11-06 review). Far first
    (an unprotected left's view, its R_req past the full pass's range) it is skipped outright under PAINT_SKIP_V too, the
    step's time the far crop's: under every paint reader's floor nothing goes without it, and at a left's line (11-06's
    numbers) a road run with the paint costs 16.5 ms of step, without it 12.6, the far crop's 13.7 fitting beside the
    latter's only."""
    if cam == 'wide' or self.sweep is not None or self.lane_paint is None or time.monotonic() < self.paint_retry_at:
      return None
    if v_ego is not None and v_ego < PAINT_SKIP_V and (self.lights_near or self.far_first):
      return LanePaintStatus.noRoom
    if self.fits(model_execution_time, cam) or not self.fits(model_execution_time, cam, paint=False):
      return None  # room for it all, or for none of it: room() decides, its forced runs with the paint as before
    now = time.monotonic()
    if self.paint_skipped_t is None:
      self.paint_skipped_t = now
    skipped_s = now - self.paint_skipped_t
    if skipped_s < PAINT_FORCE_S:
      return LanePaintStatus.noRoom
    paint_ms = 1e3 * float(np.median(self.paint_times))
    cloudlog.info(f"turnpilot lane paint: no room for {skipped_s:.1f} s at {paint_ms:.1f} ms, model {1e3 * model_execution_time:.0f}; measuring again")
    self.paint_times.clear()
    return None

  def step(self, pm, model_state, model_execution_time: float, desired_curvature: float, desire: int, metas: list,
           view: str = 'auto', far_bearing: float = 0., v_ego: float | None = None, signal_near: bool = False,
           far_first: bool = False) -> None:
    """Called once per model frame after modelV2 is sent, with the driving model this detector was loaded for.
    metas: FrameMeta of packed frames 0 and 1; view: turnpilotPlan.detectorView (choose_camera, the sweeps); far_bearing:
    turnpilotPlan.farCropBearing (FarCrop); v_ego: carState.vEgo for the side views' speed gate (None: unknown, closed);
    signal_near: a signal mapped within SWEEP_SIGNAL_M (plan_inputs): the light reader during a sweep, the camera choice
    and the paint's skip standing; far_first: plan_inputs' (left_pick). Whatever it runs, the step's time past
    STEP_BUDGET_S is the next step's debt (the time budget)."""
    now = time.monotonic()
    if self.last_step_t is None or now - self.last_step_t > STEP_GAP_S:
      self.debt = 0.  # the first step, or after a gap (the host stepped nothing, modeld idled): nothing carried
      if self.last_step_t is not None:
        self.native_gap(self.last_step_t, now)  # and the native, stepped by no one past STEP_GAP_S, missed nothing more
    self.last_step_t, self.step_t0, self.model_time = now, time.perf_counter(), model_execution_time
    try:
      self.detect(pm, model_state, model_execution_time, desired_curvature, desire, metas, view, far_bearing, v_ego, signal_near,
                  far_first)
    finally:
      self.debt = self.debt_after(model_execution_time)

  def detect(self, pm, model_state, model_execution_time: float, desired_curvature: float, desire: int, metas: list,
             view: str, far_bearing: float, v_ego: float | None, signal_near: bool, far_first: bool = False) -> None:
    """step()'s runs on this frame, in its time budget."""
    if model_state is not self.model_state:
      # DetectorHost picks the detector by model, so this is a bug: raise for the host to log and retry, never a quiet off
      raise ValueError("turnpilot detector stepped with a driving model it wasn't loaded for")
    if self.run_t is None:
      self.run_t, self.full_extra = {}, deque(maxlen=TIME_WINDOW)
    now = time.monotonic()
    self.steps += 1
    sweep = view if view in SWEEPS else None
    if sweep != self.sweep:
      self.sweep, self.sweep_runs = sweep, 0
      if self.side is not None:
        self.side.next = 0
    self.native_hold('sweep', sweep is not None, now)  # its odd steps are the side views'
    self.native_hold('farOff', not self.far_on, now)    # TurnPilotFarCrop off turns the slot off
    # far first standing or slow only (FAR_FIRST_V; v_ego unknown is moving): moving, the full pass's cadence as auto's
    self.far_first = far_first and sweep is None and v_ego is not None and v_ego < FAR_FIRST_V
    self.v_ego = v_ego
    self.native_hold('farFirst', self.far_first and self.far is not None and self.left_far() is self.far, now)  # the far crop's slot
    gated = sweep is not None and v_ego is not None and v_ego <= SIDE_VIEW_V_MAX
    if self.side is not None:
      self.side.gate(gated, now)
    # before the peek: this step's, which its pick and the paint's skip take
    self.signal_near = signal_near
    self.lights_near = signal_near and self.light_reading(now)
    self.next_cam = self.choose_camera(desired_curvature, desire, now, view, v_ego, peek=True)
    self.since_run += 1
    full_due = self.since_run >= RUN_EVERY
    if self.far_first and self.left_pick(pm, model_state, model_execution_time, metas, far_bearing, full_due):
      return  # the far crop's frame
    if not full_due:  # a frame between the full runs: the sweep's side views, else the far crop (far first: left_pick's)
      if sweep is not None:
        self.side_step(pm, model_state, model_execution_time, metas, gated)
      elif not self.far_first:
        self.far_step(pm, model_state, model_execution_time, metas, far_bearing)
      return
    if sweep is not None and self.side is not None and not self.side.disabled:
      self.side_ready(upload=False)  # the maps' recompute starts with the sweep, above the gate too; uploaded on an odd step
    lights_read = self.lights_read()
    if time.monotonic() < self.retry_at:
      return
    paint_skip = self.paint_skip(model_execution_time, self.next_cam, v_ego)
    if not self.room(model_execution_time, self.next_cam, paint_skip is None):
      return
    try:
      cam = self.choose_camera(desired_curvature, desire, now, view, v_ego)
      t0 = time.perf_counter()
      raw = self.run(input_frame=self.frames[cam]).numpy()
      dt = time.perf_counter() - t0
      self.since_run, self.last_run_t, self.last_cam = 0, time.monotonic(), cam
      meta = metas[self.cameras[cam]]
      dets = decode(raw, self.lb)
      msg = messaging.new_message('turnpilotDetections', valid=bool(np.isfinite(raw).all()))
      td = msg.turnpilotDetections
      fill_detections(td, dets, meta.frame_id, meta.timestamp_eof, cam, dt)
      self.times.append(dt)
      odd, self.odd_paint = self.odd_paint, None
      paint_s = light_s = 0.  # this step's own reads, off its measured extra (full_extra)
      if cam == 'road' and sweep is None and paint_skip is not None:
        td.lanePaintStatus = paint_skip  # the lights' room, or far first the far crop's (paint_skip)
      elif cam == 'road' and sweep is None:
        self.read_lane_paint(td, model_state, metas, dets)
        paint_s = td.lanePaintTime
      elif cam == 'road':
        td.lanePaintStatus = LanePaintStatus.sweep if self.lane_paint is not None else LanePaintStatus.disabled
      elif sweep is None and odd is not None and odd[3] == self.steps - 1 and self.lane_paint is not None:
        fill_lane_paint(td.init('lanePaint', 4), odd[0])  # read on the far crop's frame, the step before (read_odd_paint)
        td.lanePaintStatus, td.lanePaintTime = odd[1], odd[2]
      else:
        td.lanePaintStatus = LanePaintStatus.wideCamera if 'road' in self.cameras else LanePaintStatus.disabled
      if cam == 'road' and lights_read:
        self.read_lights(td, model_state, metas, raw)
        light_s = td.lightTime
      elif cam == 'road':
        td.lightStatus = LightStatus.sweep if self.light_status() == LightStatus.read else self.light_status()
      else:
        td.lightStatus = self.light_status() if self.light_status() != LightStatus.read else LightStatus.wideCamera
      boxes = decode_lights(raw, self.lb)
      for o, (x0, y0, x1, y1, conf, cls) in zip(td.init('lightBoxes', len(boxes)), boxes, strict=True):
        o.cls, o.conf, o.x0, o.y0, o.x1, o.y1 = int(cls), float(conf), float(x0), float(y0), float(x1), float(y1)
      signs = decode_signs(raw, self.lb) if self.signs_mode != 'off' else ()
      if len(signs):
        fill_signs(td, signs, model_state.frames[self.cameras[cam]], self.lb.cam_w, self.lb.cam_h)
      td.farStatus = self.far_status()
      td.sideViewStatus = self.side_status(gated)
      td.nativeStatus = self.native_status()
      pm.send('turnpilotDetections', msg)
      self.failures = 0
      self.run_t[cam] = time.monotonic()
      self.full_extra.append(max(0., time.perf_counter() - t0 - dt - paint_s - light_s))
    except Exception:
      self.failures += 1
      delay = min(RETRY_S * 2 ** (self.failures - 1), RETRY_MAX_S)
      self.retry_at = time.monotonic() + delay
      cloudlog.exception(f"turnpilot detector failed ({self.failures} in a row), retrying in {delay:.0f} s")

  def lights_read(self) -> bool:
    """Whether a road run reads the lights' colour: always, but during a sweep only with a signal mapped near."""
    return self.sweep is None or self.signal_near

  def light_reading(self, now: float) -> bool:
    """Whether a road run now would read the lights: the reader on (a road camera, TurnPilotCameraLights not off), out of
    its retry backoff (the run its retry is due on reads), and lights_read. What the estimate counts the lights' time for,
    and with signal_near what gives them the road runs (lights_near)."""
    return self.lights is not None and self.light_mode != 'off' and now >= self.light_retry_at and self.lights_read()

  def load_native(self, mode: str) -> None:
    """The native schedule for mode (TurnPilotNativeDetect at the load; DetectorHost), on the road camera; None if off."""
    self.native = load_native(self.model_state, self.lb.cam_w, self.lb.cam_h, self.cameras['road'], mode, self.bus) \
      if mode != 'off' and 'road' in self.cameras else None

  def active_native(self) -> NativeStrips | None:
    """The native schedule when it takes the far crop's slot: loaded for the param's mode and not back to the far crop."""
    native = self.native
    if native is None or native.disabled or self.native_mode != native.mode:
      return None
    return native

  def native_status(self) -> int:
    """nativeStatus for the full pass's messages (the schema's NativeStatus)."""
    if self.native_mode == 'off':
      return NativeStatus.off
    native = self.native
    if native is None or native.mode != self.native_mode:
      return NativeStatus.unavailable
    if native.disabled:
      return NativeStatus.fallback
    if native.last_run_t is None and self.native_idle_s(native, time.monotonic()) > NATIVE_NO_RUN_S:
      # 11-06: loaded and on, it never had room beside the car's 38.5 ms model, and every message said full. The clock is
      # the slot's free, calibrated time (native_idle_s), so no room is the cause when it runs out, never calibration
      if not native.no_run_said:
        native.no_run_said = True
        model_ms = 1e3 * self.model_time
        cloudlog.error(f"turnpilot native schedule: no room in {NATIVE_NO_RUN_S:.0f} s of its free, calibrated slot beside a {model_ms:.0f} ms model")
      return NativeStatus.unavailable
    return NativeStatus.full if native.mode == 'full' else NativeStatus.strip

  def far_status(self) -> int:
    """farStatus for the full pass's messages: off with TurnPilotFarCrop off or no far crop, sweep while a stop sweep
    suspends it, else the far crop's own; with the native schedule in its slot, the slot's (native_far_status)."""
    if self.active_native() is not None:
      return self.native_far_status()
    if self.far is None or (not self.far_on and not self.far.disabled):
      return FarStatus.off
    if self.sweep is not None and not (self.far.disabled or self.far.failing()):
      return FarStatus.sweep  # a failing far crop's disabled outlasts a sweep, as a dead slot's (native_far_status)
    return self.far.status(time.monotonic())

  def native_far_status(self) -> int:
    """farStatus while the native schedule holds the far crop's slot: off with TurnPilotFarCrop off; disabled when the far
    crop is off for the drive (or none) and the native schedule hasn't run for NATIVE_NO_RUN_S of the slot's free,
    calibrated time (native_idle_s; since its load, if never: the slot is dead, 11-06's noRoom for the rest of both drives),
    in a stop sweep too; else sweep in a stop sweep, read when it or the far crop (on a frame it had no room for) ran within
    FAR_STATUS_S, else why it didn't. The dead slot is checked before the sweep, as the far crop's own disabled outlasts one
    (far_status): turnpilotd.watch_far raises farViewOff on every change into disabled and takes sweep for a live far view
    (FAR_ON), so a dead slot said sweep through each stop-sign hold would chime Far View Off after every one (11-06 review)."""
    native = self.active_native()
    assert native is not None
    if not self.far_on:
      return FarStatus.off
    now = time.monotonic()
    far = self.far
    far_off = far is None or far.disabled or far.failing()
    if far_off and self.native_idle_s(native, now) > NATIVE_NO_RUN_S:
      return FarStatus.disabled  # NATIVE_NO_RUN_S > FAR_STATUS_S: nor did it read lately
    if self.sweep is not None:
      return FarStatus.sweep
    status = native.status(now)
    if status != FarStatus.read and far is not None and not far.disabled and far.status(now) == FarStatus.read:
      return FarStatus.read
    return status

  def native_idle_s(self, native: NativeStrips, now: float) -> float:
    """The native schedule's time with its slot free, calibrated and no run: since its last run (its load, if none), less
    its holds' since then (the ended ones, held_s, and the running one's from hold_t0: native_hold) and the gaps in the
    steps past STEP_GAP_S each (native_gap). So a hold pauses the NATIVE_NO_RUN_S clock: a long wait at a stop sign, a
    drive with TurnPilotFarCrop off, a fresh install's or a recalibration's minutes without calibration never count toward
    nativeStatus unavailable (never run) nor the slot's farStatus disabled, and a slot already dead stays disabled through
    one (the clock never runs back)."""
    last = native.last_run_t if native.last_run_t is not None else native.load_t
    held = native.held_s
    if self.hold_t0 is not None:
      held += max(0., now - max(self.hold_t0, last))
    return now - last - held

  def native_hold(self, reason: str, on: bool, now: float) -> None:
    """The native's slot held for reason (on) or no longer (off), at now: while any reason holds (native_holds), its time
    since hold_t0 is off the native's no-run clock, and when the last one ends it goes on held_s (native_idle_s). One
    spell for every reason at once, so time held for two (a sweep without calibration) counts once. The reasons: sweep (its
    odd steps are the side views'), farOff (TurnPilotFarCrop off: no slot), noCalibration (far_run found no calibrated
    horizon: a fresh install, a recalibration), farFirst (an unprotected left's far-first frames are the far crop's:
    left_far). Until the first step that holds, the clock runs from the load."""
    holds = self.native_holds | {reason} if on else self.native_holds - {reason}
    if holds and not self.native_holds:
      self.hold_t0 = now
    elif self.native_holds and not holds:
      native, t0 = self.native, self.hold_t0
      if native is not None and t0 is not None:
        last = native.last_run_t if native.last_run_t is not None else native.load_t
        native.held_s += max(0., now - max(t0, last))
      self.hold_t0 = None
    self.native_holds = holds

  def native_gap(self, last_step_t: float, now: float) -> None:
    """A gap in the steps from last_step_t to now (over STEP_GAP_S: TurnPilot off a while, a host retry, a host stepping
    below 10 Hz), its time past STEP_GAP_S off the native's no-run clock as a hold's is: the native ran nowhere then, so it
    missed nothing more. Only the excess, so each step counts at most STEP_GAP_S, the clock continuous in the step's
    length: the 11-06 review found the whole interval taken off, so a host stepping steadily below 10 Hz (a loaded one:
    FORCE_RUN_S) took every interval for a gap, its clock never ran and a dead slot said nativeStatus full and farStatus
    noRoom for good (0.125 s steps for 120 s: held_s 119.875 s), 11-06's silent far view again, though its odd steps
    offered the native the slot. Now such a host's clock runs at STEP_GAP_S a step (8 Hz: a dead slot said
    after 300 steps, 37.5 s), and a 60 s gap counts 0.1 s. The car isn't such a host: 11-06's rlogs had 3 of 30535 modelV2
    intervals over 0.1 s (0.65 s in all, the longest 0.4 s), p99 61 ms. Inside a hold, the hold has it."""
    native = self.native
    if native is not None and not self.native_holds:
      last = native.last_run_t if native.last_run_t is not None else native.load_t
      native.held_s += max(0., now - max(last_step_t, last) - STEP_GAP_S)

  def side_status(self, gated: bool) -> int:
    """sideViewStatus for every message: off with no side views, else theirs (SideViews.status)."""
    if self.side is None:
      return SideViewStatus.off
    return self.side.status(time.monotonic(), self.sweep is not None, gated)

  def side_ready(self, upload: bool) -> bool:
    """SideViews.prepare, its failures the side views' (off for the drive, loudly), never the full pass's."""
    side = self.side
    assert side is not None
    try:
      return side.prepare(upload)
    except Exception:
      cloudlog.exception("turnpilot side views: their maps failed")
      side.disable("failed (maps)")
      return False

  def side_step(self, pm, model_state, model_execution_time: float, metas: list, gated: bool) -> None:
    """A sweep's odd step: the next side view, while the speed gate is open, its maps are ready and it has room (module
    docstring). Its failures turn the side views off for the drive and touch nothing of the full pass's."""
    side = self.side
    if side is None or side.disabled or not gated or not self.side_ready(upload=True):
      return
    name = SIDE_VIEWS[side.next]
    if not self.odd_room(model_execution_time, side.estimate(name)):
      return  # stale, said once it outlasts SIDE_STATUS_S (noRoom)
    try:
      geom = side.geometry
      assert geom is not None
      luma, chroma, weights = side.device[name]
      t0 = time.perf_counter()
      raw = side.run(input_frame=side.frame, luma_idx=luma, chroma_idx=chroma, weights=weights).numpy()
      dt = time.perf_counter() - t0
      side.next = (side.next + 1) % len(SIDE_VIEWS)
      i = self.cameras['wide']
      meta = metas[i]
      msg = messaging.new_message('turnpilotDetections', valid=bool(np.isfinite(raw).all()))
      td = msg.turnpilotDetections
      fill_detections(td, decode_side(raw, geom, name), meta.frame_id, meta.timestamp_eof, 'wide', dt)
      td.view, td.lens = name, geom.lens.spec
      self.read_sectors(td, model_state.frames[i], meta, name)
      td.lanePaintStatus = LanePaintStatus.sweep if self.lane_paint is not None else LanePaintStatus.disabled
      td.lightStatus = self.light_status() if self.light_status() != LightStatus.read else LightStatus.wideCamera
      td.farStatus, td.sideViewStatus = self.far_status(), SideViewStatus.running
      td.nativeStatus = self.native_status()
      pm.send('turnpilotDetections', msg)
      side.ran(name, time.monotonic(), dt, self.debt_after(model_execution_time) > DEBT_CAP_S)
    except Exception:
      cloudlog.exception("turnpilot side views failed")
      side.disable("failed")

  def read_sectors(self, td, frame: np.ndarray, meta, name: str) -> float:
    """td.sectorQuality and sectorTime for side view name from the wide frame's host copy. A failure turns only the sector
    quality off, for the drive (no sectors: none is ok). Returns the time it took."""
    side = self.side
    assert side is not None and side.geometry is not None
    if side.quality is None:
      return 0.
    t0 = time.perf_counter()
    try:
      sectors = side.quality.update(side.geometry, name, frame, meta.timestamp_eof * 1e-9)
    except Exception:
      cloudlog.exception("turnpilot side views: sector quality failed, off for this drive (no sector reads ok)")
      side.quality = None
      return time.perf_counter() - t0
    dt = time.perf_counter() - t0
    for o, (inner, outer, quality) in zip(td.init('sectorQuality', len(sectors)), sectors, strict=True):
      o.inner, o.outer, o.quality = float(inner), float(outer), quality
    td.sectorTime = dt
    side.quality_times.append(dt)
    return dt

  def far_step(self, pm, model_state, model_execution_time: float, metas: list, bearing: float) -> None:
    """The far crop on this frame between the full runs, where it has room (module docstring), or the native schedule in its
    place; a frame the native schedule has no room for is the far crop's if it fits. A native failure sends it back to the
    far crop for the drive, a far crop failure is retried after a backoff (FarCrop.failed); neither touches the full pass's."""
    native = self.active_native()
    if native is not None and self.far_on:
      if self.far_run(native, pm, model_state, model_execution_time, metas, bearing) != FarStatus.noRoom:
        return
    far = self.far
    if far is None or far.disabled or not self.far_on:
      return
    self.far_run(far, pm, model_state, model_execution_time, metas, bearing)

  def left_far(self) -> FarCrop | None:
    """The far slot's pass far first: the far crop (on, not off for the drive), else the native schedule in the slot, else
    None. The far crop before the native: aimed up the oncoming approach it covers the band past the full pass's range as
    the native strips do, at their pixel density, for 13.7 ms of step on the car against a strip's 15.4 ms run alone, which
    leaves more than DEBT_CAP_S beside a 38.5 ms model from a paid-back step, so the full pass would lose two frames to it."""
    far = self.far
    if far is not None and not far.disabled and self.far_on:
      return far
    return self.active_native() if self.far_on else None

  def left_pick(self, pm, model_state, model_execution_time: float, metas: list, bearing: float, full_due: bool) -> bool:
    """A far-first frame (module docstring): the far slot's run (left_far) where it is due no later than the full pass's
    next (FAR_DUE_S after its last against full_deadline), at most every FAR_MIN_S, on its own step cost alone (debt_after
    within DEBT_CAP_S: no room kept for the full pass, which takes the next frame it fits on); on a frame between the full
    runs also where it isn't, if it leaves the next full pass its room (odd_room). Whether it ran: the frame is then its.
    Otherwise the frame is the full pass's (if due) as it always was, its room on its step cost too (estimate)."""
    far = self.left_far()
    now = time.monotonic()
    if far is None or (far.last_run_t is not None and now - far.last_run_t < FAR_MIN_S):
      return False
    due = far.last_run_t + FAR_DUE_S if far.last_run_t is not None else -math.inf
    # a road run due keeps its frame (R1, R2, review 2026-11-07): the road camera's band is the one the left is judged on
    # once the far slot can't keep up (unprotected_left far_behind), and on a model 2.5-3 ms slower a far run taking it left
    # the road's gaps past ROAD_FRESH_S on a quarter of runs
    first = due <= self.full_deadline(now) and not (self.next_cam != 'wide' and self.road_due(now))
    if not first and (full_due or self.road_due(now)):
      return False  # a road run due is the road camera's frame (R1: a far run between a wide run and it put it 7 frames out)
    return self.far_run(far, pm, model_state, model_execution_time, metas, bearing, reserve=not first) == FarStatus.read

  def far_run(self, far: FarCrop, pm, model_state, model_execution_time: float, metas: list, bearing: float,
              reserve: bool = True) -> int:
    """One run of far (the far crop, or the native schedule) on this frame where it has room: FarStatus read if it ran, else
    why not. reserve: its room leaves the next full pass its own (odd_room), else its own step alone (left_pick). Its
    estimate and its measured cost are its step's, the run to the send, its lights apart (FarCrop.estimate). A failure is
    the pass's (FarCrop.failed: the far crop retried, the native back to the far crop for the drive)."""
    now = time.monotonic()
    if far.retrying(now):
      return far.why  # a failed run's backoff: said disabled from FAR_DISABLED_FAILS in a row (FarCrop.status)
    try:
      rpy = far.calibration()
      if far is self.native:
        self.native_hold('noCalibration', rpy is None, now)  # no horizon to run on: never no room
      if rpy is None:
        far.why = FarStatus.noCalibration
        return far.why
      lights_on = far.lights is not None and self.light_mode != 'off' and not self.far_first  # far first: none (read_far_lights)
      estimate = far.estimate(lights_on)
      if not (self.odd_room(model_execution_time, estimate) if reserve else self.debt_after(model_execution_time, estimate) <= DEBT_CAP_S):
        far.why = FarStatus.noRoom
        far.no_room(now)
        return far.why
      crop = far.aim(bearing, rpy)
      luma, chroma = far.views(crop)
      t0 = time.perf_counter()
      raw = far.run(luma=luma, chroma=chroma).numpy()
      dt = time.perf_counter() - t0
      i = self.cameras['road']
      meta = metas[i]
      msg = messaging.new_message('turnpilotFarDetections', valid=bool(np.isfinite(raw).all()))
      boxes = decode_far(raw, crop)
      fill_far(msg.turnpilotFarDetections, boxes, crop, bearing, meta.frame_id, meta.timestamp_eof, dt)
      light_s = self.read_far_lights(msg.turnpilotFarDetections.run, model_state, metas, raw, crop, far)
      pm.send('turnpilotFarDetections', msg)
      far.ran(time.monotonic(), dt, time.perf_counter() - t0 - light_s, self.debt_after(model_execution_time) > DEBT_CAP_S)
    except Exception:
      cloudlog.exception("turnpilot native schedule failed" if far.mode != 'off' else "turnpilot far crop failed")
      far.failed()
      return FarStatus.disabled
    self.read_odd_paint(model_state, metas, boxes, model_execution_time)
    return FarStatus.read

  def read_odd_paint(self, model_state, metas: list, boxes: np.ndarray, model_execution_time: float) -> None:
    """While the full pass looks through the wide camera (a turn, a curve, their WIDE_HOLD_S), the lane paint on the far
    crop's road frame, behind the far crop's boxes, kept in odd_paint for the next full run's message, where the step has
    room for it (odd_room, with the step's time so far; with no paint time yet, as when the drive starts in a turn, a read
    is taken to cost PAINT_BUDGET_S, the most one may). Its failures and budget are the paint's own (read_lane_paint's).
    Far first under PAINT_SKIP_V, none (paint_skip's rule)."""
    if self.last_cam != 'wide' or self.lane_paint is None or 'road' not in self.cameras or time.monotonic() < self.paint_retry_at:
      return
    if self.far_first and self.v_ego is not None and self.v_ego < PAINT_SKIP_V:
      return  # far first under every paint reader's floor: the step's time is the far crop's (paint_skip)
    estimate = float(np.median(self.paint_times)) if self.paint_times else PAINT_BUDGET_S
    if not self.odd_room(model_execution_time, estimate):
      return  # no room: this wide run's message says wideCamera, as before
    t0 = time.perf_counter()
    try:
      i = self.cameras['road']
      paint, status = self.lane_paint.read(model_state.frames[i], metas[i], boxes)
    except Exception:
      self.paint_failed()
      return
    dt = time.perf_counter() - t0
    self.odd_paint = (paint, status, dt, self.steps)
    self.paint_over_budget(dt)

  def read_far_lights(self, td, model_state, metas: list, raw: np.ndarray, crop: Crop, far: FarCrop | None = None) -> float:
    """The far crop's class-9 boxes through its own reader, always shadow: td.lights, lightStatus, lightTime. Its time and
    failures are ruled as the full pass's lights are (read_lights): a failure is retried after a backoff (lightStatus
    retrying, disabled from LIGHT_DISABLED_FAILS in a row until a read succeeds), a median over LIGHT_BUDGET_S warned,
    never off for the drive, its gates logged (log_light_gates). Returns the time it took. far: the pass that ran (the
    native schedule's lights are its own), default the far crop."""
    far = self.far if far is None else far
    assert far is not None
    if far.lights is None or self.light_mode == 'off' or self.far_first:
      # far first (an unprotected left in hand) none: its reads are shadow, and at a left nothing acts on them (turnpilotd
      # logs them, far_light_shadow), the paint skipped there on the same reasoning. Review 2026-11-07 (R1): at a signalized
      # left with far-side heads in its window (1-3 by geometry by day, at twice the full pass's pixel density) their reads
      # (0.7 ms bare, 1.5-3.9 ms with 1-3 heads on 11-06's rlogs) put the far crop's step past DEBT_CAP_S: it ran 0-2 times a
      # second, the left waited on its gaps (p50 8-19 s at 1-2 road heads) or went on the full pass's band alone
      td.lightStatus = LightStatus.off
      return 0.
    if time.monotonic() < far.light_retry_at:
      td.lightStatus = failing_status(far.light_failures)
      return 0.
    t0 = time.perf_counter()
    try:
      i = self.cameras['road']
      heads, status = far.lights.read(model_state.frames[i], metas[i], decode_far_lights(raw, crop))
    except Exception:
      td.lightStatus = light_failed(far, "far crop lights")
      return time.perf_counter() - t0
    light_read_ok(far, "far crop lights")
    dt = time.perf_counter() - t0
    fill_lights(td.lights, heads, True)
    td.lightStatus, td.lightTime = status, dt
    far.light_times.append(dt)
    if len(far.light_times) == TIME_WINDOW and float(np.median(far.light_times)) > LIGHT_BUDGET_S:
      ms = 1e3 * float(np.median(far.light_times))
      warn_over_budget(far, 'light_warn_t', time.monotonic(),
                       f"turnpilot far crop lights over budget ({ms:.1f} ms against {LIGHT_BUDGET_S * 1e3:.1f}): read where they fit")
    log_light_gates(far, far.lights, time.monotonic(), 'far' if far.mode == 'off' else 'native', self.signal_near)
    return dt

  def read_lane_paint(self, td, model_state, metas: list, boxes: np.ndarray) -> None:
    """Fill td.lanePaint, lanePaintStatus and lanePaintTime from the road camera's frame, behind this run's detections on
    it. A failure is retried after a backoff (retrying, paint_failed), a slow read skipped where it doesn't fit
    (paint_skip): never off for the drive. Anything but read is unknown paint to turnpilotd, which then plans no lane
    change."""
    if self.lane_paint is None:
      td.lanePaintStatus = LanePaintStatus.disabled
      return
    if time.monotonic() < self.paint_retry_at:
      td.lanePaintStatus = LanePaintStatus.retrying
      return
    t0 = time.perf_counter()
    try:
      i = self.cameras['road']
      paint, status = self.lane_paint.read(model_state.frames[i], metas[i], boxes)
    except Exception:
      self.paint_failed()
      td.lanePaintStatus = LanePaintStatus.retrying
      return
    dt = time.perf_counter() - t0
    fill_lane_paint(td.init('lanePaint', 4), paint)
    td.lanePaintStatus, td.lanePaintTime = status, dt
    self.paint_over_budget(dt)

  def paint_failed(self) -> None:
    """A paint read raised: retried after backoff_s, logged each time, never off for the drive (the full pass's rule)."""
    self.paint_failures += 1
    delay = backoff_s(self.paint_failures)
    self.paint_retry_at = time.monotonic() + delay
    cloudlog.exception(f"turnpilot lane paint failed ({self.paint_failures} in a row), retrying in {delay:.0f} s")

  def paint_over_budget(self, dt: float) -> None:
    """Counts a paint read's time (the estimate's: paint_skip skips a read where it doesn't fit); a median over
    PAINT_BUDGET_S is warned at most once in BUDGET_WARN_S, never off for the drive."""
    now = time.monotonic()
    self.paint_failures, self.paint_skipped_t = 0, None
    self.paint_times.append(dt)
    if len(self.paint_times) == TIME_WINDOW and float(np.median(self.paint_times)) > PAINT_BUDGET_S:
      ms = 1e3 * float(np.median(self.paint_times))
      warn_over_budget(self, 'paint_warn_t', now, f"turnpilot lane paint over budget ({ms:.1f} ms against {PAINT_BUDGET_S * 1e3:.1f}): read where it fits")

  def light_status(self) -> int:
    """What a run without a read says: off with the param off or no road camera, else read."""
    return LightStatus.off if self.lights is None or self.light_mode == 'off' else LightStatus.read

  def read_lights(self, td, model_state, metas: list, raw: np.ndarray) -> None:
    """Fill td.lights, lightStatus and lightTime from the road camera's frame and this run's class-9 boxes. Never off for
    the drive (11-06: medians to 3.74 ms against the 4.0 ms that turned them off, at the junctions where a red must stop the
    car): its time is the estimate's, a median over LIGHT_BUDGET_S warned at most once in BUDGET_WARN_S, and a failure is
    retried after a backoff (backoff_s), logged each time, the runs between saying retrying; from LIGHT_DISABLED_FAILS
    failures in a row they say disabled (logged once), which turnpilotd keeps and map_controls says once, until a read
    succeeds (light_failed). A read's gates go to log_light_gates."""
    status = self.light_status()
    if status != LightStatus.read:
      td.lightStatus = status
      return
    if time.monotonic() < self.light_retry_at:
      td.lightStatus = failing_status(self.light_failures)
      return
    t0 = time.perf_counter()
    try:
      i = self.cameras['road']
      heads, status = self.lights.read(model_state.frames[i], metas[i], decode_lights(raw, self.lb, LIGHT_READ_CONF))
    except Exception:
      td.lightStatus = light_failed(self, "lights")
      return
    light_read_ok(self, "lights")
    dt = time.perf_counter() - t0
    fill_lights(td.lights, heads, self.light_mode != 'addOnly')
    td.lightStatus, td.lightTime = status, dt
    self.light_times.append(dt)
    if len(self.light_times) == TIME_WINDOW and float(np.median(self.light_times)) > LIGHT_BUDGET_S:
      ms = 1e3 * float(np.median(self.light_times))
      warn_over_budget(self, 'light_warn_t', time.monotonic(),
                       f"turnpilot lights over budget ({ms:.1f} ms against {LIGHT_BUDGET_S * 1e3:.1f}): read where they fit")
    log_light_gates(self, self.lights, time.monotonic(), 'road', self.signal_near)


def load_far(model_state, cam_w: int, cam_h: int, index: int, bus) -> FarCrop | None:
  """The far crop for the road camera, packed frame `index`, or None: no far pkl for this device (logged), or it failed to
  load (logged); the full pass runs either way."""
  try:
    from tinygrad.engine.realize import lower_and_compile
    from openpilot.selfdrive.modeld.helpers import load_oob
    path = far_detector_pkl_path(cam_w, cam_h, model_state.chestnut)
    if not path.is_file():
      cloudlog.warning(f"turnpilot far crop: no {path.name}, far crop off")
      return None
    art = load_oob(path, model_state.chestnut)
    centre = crop_at(cam_w, cam_h, cam_w / 2, cam_h / 2)
    (_, ln), (_, cn) = crop_views(centre)
    specs = art['input_specs']
    if art['metadata'].get('kind') != 'far' or specs['luma'][0] != (ln,) or specs['chroma'][0] != (cn,):
      raise ValueError(f"far crop compiled for {art['metadata']}, {specs}; this camera's views are {ln} and {cn} bytes")
    run = art['run']
    run.captured._linear = lower_and_compile(run.captured._linear)
    far = FarCrop(run, cam_w, cam_h, crop_frame_views(model_state, index), bus)
    times = []
    for _ in range(3):  # warm up, and a first time estimate
      t0 = time.perf_counter()
      run(**dict(zip(('luma', 'chroma'), far.views(centre), strict=True))).numpy()
      times.append(time.perf_counter() - t0)
    far.warmup_s = float(np.median(times))
    far.lights = CameraLightReader(bus)
    cloudlog.warning(f"turnpilot far crop loaded ({1e3 * far.warmup_s:.1f} ms warm)")
    return far
  except Exception:
    cloudlog.exception("turnpilot far crop failed to load, far crop off")
    return None


def load_native(model_state, cam_w: int, cam_h: int, index: int, mode: str, bus) -> NativeStrips | None:
  """The native schedule (mode strip or full) for the road camera, packed frame `index`, or None: no strip for this camera
  size or no native pkl for this device (logged), or it failed to load (logged); the far crop runs in its slot either way."""
  try:
    from tinygrad.engine.realize import lower_and_compile
    from openpilot.selfdrive.modeld.helpers import load_oob
    size = NATIVE_SIZES.get((cam_w, cam_h), {}).get(mode)
    if size is None:
      cloudlog.warning(f"turnpilot native schedule: no {mode} strip for {cam_w}x{cam_h} frames, the far crop")
      return None
    path = native_detector_pkl_path(cam_w, cam_h, size, model_state.chestnut)
    if not path.is_file():
      cloudlog.warning(f"turnpilot native schedule: no {path.name}, the far crop")
      return None
    art = load_oob(path, model_state.chestnut)
    strips = native_strips(cam_w, cam_h, mode) or [crop_at(cam_w, cam_h, cam_w / 2, cam_h / 2, size)]
    (_, ln), (_, cn) = crop_views(strips[0])
    specs = art['input_specs']
    if (art['metadata'].get('kind') != 'native' or tuple(art['metadata'].get('input_size', ())) != size or specs['luma'][0] != (ln,) or
        specs['chroma'][0] != (cn,)):
      raise ValueError(f"native strip compiled for {art['metadata']}, {specs}; this camera's {size} views are {ln} and {cn} bytes")
    run = art['run']
    run.captured._linear = lower_and_compile(run.captured._linear)
    native = NativeStrips(run, cam_w, cam_h, crop_frame_views(model_state, index), bus, mode)
    times = []
    for k in range(4):  # warm up (each strip), and a first time estimate
      t0 = time.perf_counter()
      run(**dict(zip(('luma', 'chroma'), native.views(strips[k % len(strips)]), strict=True))).numpy()
      times.append(time.perf_counter() - t0)
    native.warmup_s = float(np.median(times))
    native.lights = CameraLightReader(bus)
    cloudlog.warning(f"turnpilot native schedule {mode} loaded ({size[0]}x{size[1]}, {1e3 * native.warmup_s:.1f} ms warm)")
    return native
  except Exception:
    cloudlog.exception("turnpilot native schedule failed to load, the far crop")
    return None


def check_side_specs(art: dict, frame_size: int) -> None:
  """A side-view pkl's input_specs and metadata against what modeld gives it (graph.side_input_specs): the wide
  frame's view and a SIDE_INPUT_SIZE view's maps, by shape and dtype. Raises ValueError on a mismatch."""
  meta, specs = art['metadata'], art['input_specs']
  want = side_input_specs(*meta.get('camera_size', (0, 0)), '')
  got = {k: tuple(v[:2]) for k, v in specs.items()}
  if (meta.get('kind') != 'side' or tuple(meta.get('input_size', ())) != SIDE_INPUT_SIZE or
      got != {k: (tuple(v[0]), v[1]) for k, v in want.items()} or specs['input_frame'][0] != (frame_size,)):
    raise ValueError(f"side views compiled for {meta}, {specs}; modeld has frames of {frame_size} bytes and {SIDE_INPUT_SIZE} views")


def load_side(model_state, cam_w: int, cam_h: int, index: int, bus) -> SideViews | None:
  """The side views on the wide camera, packed frame `index`, or None: no side pkl for this device (logged), or it failed
  to load (logged); the forward views run either way, and a sweep then runs only them (sideViewStatus off)."""
  try:
    from tinygrad.engine.realize import lower_and_compile
    from openpilot.selfdrive.modeld.helpers import load_oob
    path = side_detector_pkl_path(cam_w, cam_h, model_state.chestnut)
    if not path.is_file():
      cloudlog.warning(f"turnpilot side views: no {path.name}, stop sweeps run the full wide pass and the road camera only")
      return None
    art = load_oob(path, model_state.chestnut)
    check_side_specs(art, model_state.frame_copy_size)
    frame = frame_view(model_state, index)
    check_side_device(art, frame.device)  # the maps and the warm-up go on the frame's device, never tinygrad's default
    run = art['run']
    run.captured._linear = lower_and_compile(run.captured._linear)
    side = SideViews(run, cam_w, cam_h, frame, bus, frame_device=frame.device)
    blank = blank_maps(side.frame_device)
    times = []
    for _ in range(3):  # warm up, and a first time estimate
      t0 = time.perf_counter()
      run(input_frame=side.frame, luma_idx=blank[0], chroma_idx=blank[1], weights=blank[2]).numpy()
      times.append(time.perf_counter() - t0)
    side.warmup_s = float(np.median(times))
    cloudlog.warning(f"turnpilot side views loaded ({1e3 * side.warmup_s:.1f} ms warm)")
    return side
  except Exception:
    cloudlog.exception("turnpilot side views failed to load, stop sweeps run the full wide pass and the road camera only")
    return None


def load(model_state, cam_w: int, cam_h: int, main_wide_camera: bool, use_extra_client: bool) -> ModeldDetector | None:
  """The detector for one driving model's device, or None. DetectorHost only calls this with TurnPilotEnabled set."""
  try:
    path = detector_pkl_path(cam_w, cam_h, model_state.chestnut)
    if not path.is_file():
      cloudlog.warning(f"turnpilot detector: no {path.name}, so none for the {'big' if model_state.chestnut else 'small'} driving model")
      return None
    cameras = {'wide': 0} if main_wide_camera else ({'road': 0, 'wide': 1} if use_extra_client else {'road': 0})
    return ModeldDetector(model_state, cam_w, cam_h, cameras)
  except Exception:
    cloudlog.exception("turnpilot detector failed to load, detector off")
    return None


class DetectorHost:
  """What modeld holds: a ModeldDetector for each driving model it can run, loaded once TurnPilotEnabled is set and stepped
  while it stays set. With it unset or no pkl, nothing runs and modeld runs as without TurnPilot.

  Loading takes seconds (the pkl, its compile, warm-up runs), so after startup it waits for openpilot to be disengaged,
  when a late modelV2 steers nothing. The small model's detector loads with the big one's so a fallback switches over at
  once. That needs a pkl compiled on the small model's device, and models/ has only the chestnut's, so today a fallback
  leaves the detector off (logged), which turnpilotd sees as an unavailable scene."""

  def __init__(self, params, cam_w: int, cam_h: int, main_wide_camera: bool, use_extra_client: bool, models: list, load_fn=None,
               environ=None):
    """models: the driving models modeld may run (the active one first); None and repeats are skipped. environ: the
    process environment (os.environ), read once here for SIMULATION."""
    self.params = params
    self.sim = (os.environ if environ is None else environ).get('SIMULATION') == '1'  # the sim drop's gate (module docstring)
    self.drop_until = 0.  # SIM ONLY: no detector step before this (time.monotonic())
    self.drop_seen = 0.   # SIM ONLY: the drop param's value at the last check
    self.drop_unknown = False  # SIM ONLY: the build's params don't know it (said once)
    self.args = (cam_w, cam_h, main_wide_camera, use_extra_client)
    self.models = [m for i, m in enumerate(models) if m is not None and all(m is not n for n in models[:i])]
    self.load_fn = load_fn or load
    self.detectors: list[tuple] | None = None  # (driving model, its detector or None), once loaded
    self.enabled = False
    self.light_mode = LIGHT_MODE_DEFAULT  # TurnPilotCameraLights, read with TurnPilotEnabled
    self.far_on = FAR_CROP_DEFAULT        # TurnPilotFarCrop, read with TurnPilotEnabled
    self.native_mode = NATIVE_DEFAULT     # TurnPilotNativeDetect, read with TurnPilotEnabled
    self.native_said: str | None = None   # a mode the loaded detectors lack, said once
    self.signs_mode = STOP_SIGNS_DEFAULT  # TurnPilotCameraStopSigns, read with TurnPilotEnabled
    self.next_check = 0.0
    self.missing = None  # the driving model last reported as having no detector
    self.failures = 0    # host failures in a row

  def check_enabled(self, now: float) -> bool:
    if now >= self.next_check:
      self.next_check = now + PARAM_CHECK_S
      self.enabled = self.params.get_bool("TurnPilotEnabled")
      self.light_mode = camera_lights_mode(self.params)
      self.far_on = far_crop_on(self.params)
      self.native_mode = native_detect_mode(self.params)
      self.signs_mode = camera_stop_signs_mode(self.params)
      if self.sim:
        self.sim_drop(now)
    return self.enabled

  def sim_drop(self, now: float) -> None:
    """SIM ONLY (module docstring): a new TurnPilotSimDropDetectionsS value S > 0 starts S s of no detector from now."""
    try:
      value = sim_drop_seconds(self.params)
    except UnknownKeyName:
      if not self.drop_unknown:
        cloudlog.error(f"turnpilot: {SIM_DROP_PARAM} unknown to this build's params, no sim detection drop; run scons")
      self.drop_unknown, value = True, 0.
    if value > 0. and value != self.drop_seen:
      self.drop_until = now + value
      cloudlog.warning(f"turnpilot detector: SIMULATION, {SIM_DROP_PARAM} {value:.1f}: no detector runs for {value:.1f} s")
    self.drop_seen = value

  def dropping(self, now: float) -> bool:
    return self.sim and now < self.drop_until

  def load(self) -> None:
    self.detectors = [(m, self.load_fn(m, *self.args)) for m in self.models]
    if self.native_mode != 'off':
      for _, det in self.detectors:
        if det is not None:
          det.load_native(self.native_mode)

  def step(self, pm, model_state, model_execution_time: float, desired_curvature: float, desire: int, metas: list,
           engaged: bool, view: str = 'auto', far_bearing: float = 0., v_ego: float | None = None, signal_near: bool = False,
           far_first: bool = False) -> None:
    """Called once per model frame after modelV2 is sent, with the model that made it. view, far_bearing, signal_near,
    far_first: plan_inputs of turnpilotPlan; v_ego: carState.vEgo."""
    try:
      if not self.check_enabled(time.monotonic()) or self.dropping(time.monotonic()):
        return
      if self.detectors is None:
        if engaged:
          return
        cloudlog.warning("turnpilot detector: TurnPilot turned on, loading the detector")
        self.load()
      det = next((d for m, d in self.detectors or () if m is model_state), None)
      if det is None:
        if self.missing is not model_state:
          self.missing = model_state
          cloudlog.warning("turnpilot detector: none for the driving model modeld is running, detector off")
        return
      det.light_mode, det.far_on, det.signs_mode = self.light_mode, self.far_on, self.signs_mode
      det.native_mode = self.native_mode
      native = getattr(det, 'native', None)
      if self.native_mode != 'off' and (native is None or native.mode != self.native_mode) and self.native_said != self.native_mode:
        self.native_said = self.native_mode
        cloudlog.warning(f"turnpilot native schedule: TurnPilotNativeDetect {self.native_mode} isn't loaded, the far crop until the next load")
      det.step(pm, model_state, model_execution_time, desired_curvature, desire, metas, view, far_bearing, v_ego, signal_near,
               far_first)
      self.failures = 0
    except Exception:
      # never take modeld down: off until the next TurnPilotEnabled check, after a wait doubling from RETRY_S. turnpilotd
      # hands back to the driver if that outlasts its deadline
      self.failures += 1
      delay = min(RETRY_S * 2 ** (self.failures - 1), RETRY_MAX_S)
      cloudlog.exception(f"turnpilot detector host failed ({self.failures} in a row), retrying in {delay:.0f} s")
      self.enabled, self.next_check = False, time.monotonic() + delay
