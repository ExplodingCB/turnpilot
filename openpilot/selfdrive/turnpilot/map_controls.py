"""Map stop control: stop signs, yields and traffic signals on the route (navInstruction.controls), turned into stops.

TurnPilot stops for them from the map. For stop signs the evidence is sim only: in BeamNG (2026-10-27) the driving model
came to a full stop at 2 of 8. It stopped at 0 of 31 reds there too, a count that says nothing about real roads: BeamNG
renders a red lamp salmon (hue 18-20 deg, saturation 0.31-0.33, against 349-354 deg and 0.67-0.83 for real comma four red
heads), and the fork's own light reader read no colour on any of 184 BeamNG light boxes (lights r2, 2026-10-30). On real
roads, the 11-06 night drive: every signal stand was the big model's own hold, and given a turn desire it rolled a right
toward a mapped signal's line at 6.8 mph until the driver braked (1e 8:20.4, the light unread: below), the reader reading no
green in 667 reads and no colour on any governing head. Each control becomes a scene_policy.Stop, composed
with the pedestrian, oncoming and turn limits by ScenePolicy (its reason, holding and the accel cap's jerk ramp). It runs
every frame, whatever the route's maneuver is: a skipped turn or a straight-through junction keeps its stop.

The rules (Kellen, 2026-10-27; /private/tmp/claude-501/stop_design.md):
- The approach: a control engages within engage_distance (braking at MAP_STOP_DECEL from v, a second's lag, a margin).
  From then on a speed cap on a MAP_STOP_DECEL curve to the stop point, which only ever comes down (a cap that rises and
  falls with a GPS fix hunts), re-seeded only at standstill short of the latch (stopped in a queue: the lead leaves, the
  car creeps up to the line). An accel cap where the curve is late, never past MAP_DECEL_MAX (-2.0 m/s^2, the speed
  cap's own floor): a map stop never takes stop()'s hard-stop path. A stop that needs more than that, checked when the
  control is first seen, when the driver's gas comes off, and after a jump in its distance, brakes at -2.0 and says so
  (cannotStopComfortably, repeating while it lasts). Never harder, never silent.
- The latch (HELD): slower than LATCH_SPEED within LATCH_NEAR_M of the stop point (LATCH_PAST_M past it), the car is held:
  shouldStop, speed cap 0. The latch is the policy's and position-free: GPS fixes, nav gaps, reroutes, a skipped turn,
  a handback and a fault leave it be (turnpilotd keeps this policy through a fault). Only its release rule lets go, and
  only TurnPilot turned off clears it. A reroute's same control takes the latch over under its own key. A held car gone
  more than DRIVE_OFF_M at more than COMMIT_SPEED is let go: the driver's, once the hold has handed back (driverWent), or
  under the driver's gas (drivenOff); otherwise, with no go at all (the plan not applied: shadow mode turned on), an
  overrun. At a stop or an unprotected turn, the car past its stop point and past the furthest TurnPilot asked of the
  latch (a creep, a roll-up) by the latch's reach (LATCH_PAST_M) of the driver's own metres, at any speed, is the driver's
  too (driverWent handed back, else drivenOff under their gas, movedOff with none: an idle creep with the brake let off;
  owner decision (i), 2026-10-30: any movement TurnPilot didn't ask for). The driver's metres are those they move the car
  past both, with openpilot disengaged or under the gas (update()'s engaged): inside the ask they are TurnPilot's, whoever
  drove them, and with openpilot engaged the plan is TurnPilot's, so the latch's own stop carrying a car re-engaged rolling
  is never theirs, in whatever order the two come. A peek-creep past the line is never held again in the crossing lanes on
  a re-engage. At an unprotected turn so is a car the driver has moved this hold, past both or not, once it is in the
  oncoming lanes (ONCOMING_ENTRY_M before the junction as latched), or at the re-engage once a hold could no longer stand
  it short of them (from the re-engage's own speed as the latch's own braking takes it, reengage_stop_m): after the left's
  full creep the reach lies inside them, and the driver's gas inside a creep can leave the car too fast to stand short.
  The reach and the lanes are decided on the driver's own frame, a car too fast for a hold at the re-engage itself, before
  the latch brakes it: never let go engaged on TurnPilot's metres (skeptic round 1: a re-engage inside the creep let go a
  frame on, and openpilot drove the car into the junction), and never for good on a driver's frame's speed (skeptic round
  2: braked to a stand short of the lanes after, the car was free to pull away into the junction). Short of either nothing
  is let go (owner decision (a), 2026-10-30): a gas nudge inside a roll-up or a creep is TurnPilot's metres, a handback
  the car doesn't move from lets nothing go (the car is held, the handback said), and a re-engage short of the line goes
  back to the full check (c). The autonomous left's own hold after a withdrawn go (unprotected_left: hold_here, the latch already let go by the go) has the
  same rule there, its crossing said here (update()'s left, LeftCommand.crossed: Driven). A stop's or an unprotected turn's line crossed with no go
  (let go so, or overrun) is never silent (owner decision (b)): its take-control (DRIVEN_PROMPT) stays up, engaged or not,
  until the car is DRIVEN_THROUGH_M past the stop point (Driven), and turnpilotd misses an unprotected turn so driven
  straight past its junction (turnMissed). A signal's stop is the light's: a crawl under the gas releases nothing. The latch is never
  replayed down the road. A car stopped further out (a queue) isn't latched: the MPC holds it behind the lead.
- No gas go anywhere (Kellen, 2026-10-28; DESIGN phase 4, G1). Any gas press disengages (DisengageOnAccelerator, on by
  default): it is the driver taking over, never TurnPilot's go, and releases nothing. A hold TurnPilot has no rule to
  release ends in a loud handback, the car held while it lasts: a stop sign after STOP_SIGN_HANDBACK_S stopped
  (stopSignTakeControl, "Stop sign: take control to proceed"), an unprotected turn at once at its hold line
  (unprotectedTurnTakeControl) unless the autonomous left holds it. Releasing a control releases those RELEASE_SPAN_M
  past it too (the far side of a divided road), but never past the stop-sign decider's 'clear': each line has its own; nor
  across rail tracks (near_rail: a released stop near a mapped rail crossing releases nothing, and one near it is never
  released by another's), so a rail orphan's stop and the junction's either side of the tracks each stop the car.
- A held stop sign under the stop-sign decider (stop_go.py, STOP_SIGNS.md v3 SG4a, through turnpilotd; update()'s stop, a
  StopCommand): wait holds it with no handback while the decider waits (its reason and action shown), fault hands it back
  now (stopSignTakeControl, the decider's reason: its faults only), shadow (TurnPilotStopSignGo shadow or off, auto without
  its data, H4, H6, H8) is the 1 s handback above exactly as before, its reason saying why. Without a command it is shadow.
- One release path (held_control(), update()'s release, a (key, cause)): the autonomous left's go (cause gap) and the
  decider's (cause clear) let the latch go the same way, in the frame they are given. The latch's stop point moves on by
  update()'s creep, (m past where it latched, its speed cap): the left's creep for a view (CREEP_SPEED) and the decider's
  roll-up to its stop point (the map curve, up to its cap), shouldStop false, held again within CREEP_AT_M.
- A signal the route goes straight through adds nothing (Kellen, 2026-10-28: through greens without slowing; DESIGN phase 4,
  G1): no stop, no cap, no prompt, unless an accepted light source, or the camera's adding (below), reads red or yellow for
  it. One exception: the autonomous left standing at its line never goes on a red or yellow any source reads at its
  junction, mapped signal or not, nor while a stop one raised there is in force (turn_light); a route turn's signal is a
  through signal's since 2026-11-07 (below). Going straight the driving model's own braking is the car's red-light
  handling (every signal stand of the 11-06 night drive was the model's own hold); the map signal is only where a light is
  looked for. On the car the
  camera's reader is the light source that adds stops (add-only, TurnPilotCameraLights addOnly, the default since
  2026-11-06); in the sim the light oracle stands in for it, so a sim red still stops. A red is a stop on the MAP_STOP_DECEL
  curve, seeded where it is first read (the curve from engage_distance if it is further out), never harder than
  MAP_DECEL_MAX: one that needs more brakes at that floor, cannotStopComfortably. A yellow is decided once: stop when the
  stopping distance at YELLOW_DECEL after T_LAT fits (YELLOW_DECEL, TurnPilotYellowDecel, no more than -MAP_DECEL_MAX);
  otherwise committed, with proceedingOnYellow, and a later red never reverses it. A stop is released only by a steady
  green: an accepted source's (GREEN_STEADY_S), or the camera's (CAMERA_GREEN_STEADY_S, the reader acting) for a stop of
  TurnPilot's own (_camera_releases, below). A light gone unknown keeps it, and held with no light read for SIGNAL_UNSEEN_S
  it is said (SIGNAL_UNSEEN_REASON, "Light not seen: waiting for green"; turnpilot.signal_unseen once), still held for a
  green, never released by the timeout and never handed back (until 2026-11-07: signalUnseenTakeControl, "Signal not seen:
  take control"). Held, not let go to the model: a release at a standstill runs turnpilotd's no-move clock into its own
  handback while the model holds at the red, and a turn desire out has run a red before (SIGNAL_UNSEEN_S's comment); a
  light read again is TurnPilot's to release once more. The driver's gas never releases one.
- The camera's light reader (DESIGN phase 4 G2, detector/light_state.py, its heads in turnpilotDetections.lights) is a
  second source that may only ever add a stop. Its governing heads are associated with the signals (a head from
  CAMERA_ASSOC_BEFORE_M short of a signal's line to CAMERA_ASSOC_PAST_M past it is that signal's, the last line before it
  winning) and what it would do is logged on every change (turnpilot.camera_light_shadow; turnpilot.camera_light_unmapped
  for a red or yellow no signal owns). In shadow (TurnPilotCameraLights shadow) nothing it reads reaches a stop, a cap, a
  prompt or a release: the one thing it gates is the autonomous left's go (turn_light, its red too: keeping the car
  standing where it is never brakes it), which at a signal needs a green read from some source
  (unprotected_left.SIGNAL_NEEDS_GREEN). That go only lets TurnPilot's own latch go: plannerd's lift, which overrides the
  model's hold, needs a green from an accepted source (release_light; turnpilotd.certify_go), so a camera green never
  removes the model's red-light stop. Its reader failing (lightStatus disabled: three failed reads in a row, still retried
  and read again at its first good read, never off for the drive since 2026-11-06) is a status line for the whole failing
  spell (lightSourceRejected, "Lights Unknown"), only where no other alert is said, so it never mutes a take-control,
  overrun or yellow alert: its onset chimed (CAMERA_OFF_REASON "Camera light reader failing: lights unknown") once a spell,
  again once the reader has read again and no sooner than CAMERA_OFF_AGAIN_S after the last, run down only where it could be
  shown (no map prompt, engaged); the rest of the spell silent (CAMERA_OFF_STILL_REASON). Above the lane planner's
  can't-confirm line and a keep's status line (hooks.plan_events), and its words over the lane planner's (turnpilotd: S2,
  review 2026-11-07, said once for 2 s it was lost). Add-only (the default) needs both keys, modeld's message not shadow
  and turnpilotd's own
  TurnPilotCameraLights addOnly (modeld's follows the same param): then a camera red raises a signal's light to red, and a
  camera yellow to yellow only while a yellow stop at it is still feasible (a yellow too late to stop for would commit the
  signal to go, and a commit outlasts a later red: a removed stop). A camera green or unknown never enters the light, so a
  camera red can hold a stop the oracle's green would have released, never the reverse. A stop of TurnPilot's own
  (camera_added: a camera red or yellow raised it, or a route turn's want of a green made it) with no accepted source's red
  or yellow in it (accepted_stop) is released by the camera's steady green once the camera's red has gone, as by an
  accepted green (2026-11-06: until then, on the car, where no other source is accepted, every camera stop ended in the stand
  handback); an accepted source's red only by an accepted green. That release lets TurnPilot's latch go and nothing more: the
  model's own red-light hold stays in the planner's min(), and plannerd's lift needs an accepted green (release_light). With
  no green read it stays held, said (SIGNAL_UNSEEN_S): never a silent roll, never a handback since 2026-11-07.
- Since 2026-11-07 (Kellen: test builds act on what they see, a safety driver on every drive) a route turn's signal is a
  through signal's: only a red, or a yellow still stoppable, read stops it, and with no colour read the turn goes with the
  model. No stop for want of a green is made (turn_stop is never set: _signal_mode, _camera_signal), so the paragraph below
  is the build before for it: its marking of the turn's signal (turn_id), its green's TURN_GREEN_STALE_S and its red's stop
  stand; its want-of-green stop, its TURN_UNSEEN_S handback, the driver's go through it and turnpilotd's skips for a signal
  too close to stop for (signal_turn_late, signalStop) are gone.
- A route turn at a signal (2026-11-06 night drive, 1e 8:20.4: the model, given the turnRight desire, rolled toward a mapped
  signal's line at 6.8 mph until the driver braked; at night the reader read no green in 667 reads, and no colour on any
  governing head, so a turn at a light can't rest on reading its red): the signals of the route's TURN maneuver's junction
  (update()'s junction;
  one mapped from PROTECT_BEFORE_M before it to PROTECT_PAST_M past it, or the camera's signal there, below), for a turn
  TurnPilot drives (driving_turn: not skipped) that isn't the unprotected left's (its own hold needs a green there already,
  turn_light: it keeps precedence at its junction), are the turn's (_signal_turns, ControlState.turn_id). Going, such a
  signal is a stop for want of a green (turn_stop): the curve from engage_distance, the latch at its line (the mapped line,
  else the junction less CONTROL_LINE_M), every rule of a red's stop, unless a steady green (accepted GREEN_STEADY_S, the
  camera's CAMERA_GREEN_STEADY_S) is read before the car is held; a green being read defers it until it is steady or gone.
  Held, a steady green releases its latch (the camera's: TurnPilot's latch alone, as above); with no light read for
  TURN_UNSEEN_S it hands back (signalUnseenTakeControl, TURN_LIGHT_REASON, "Turn at a light: take control"), held, never
  a roll, put off while a camera green that may still release it is being read (it releases at its steady time, or, broken,
  hands back that frame: never a take-control just before the car pulls away on its own); a red read there makes it the
  red's stop (its stand SIGNAL_UNSEEN_S's once the red is lost). The driver's gas through it releases nothing, but lifted
  with openpilot engaged faster than COMMIT_SPEED where a stop at its line would need more than MAP_DECEL_MAX, or past the
  line, it is the driver's go (driver_go, turnpilot.signal_turn_driven): no stop, no overrun, no light read to run, as one
  driven through disengaged. Only provisional (_driver_go): short of the line a brake, the car down to COMMIT_SPEED or a
  red or yellow read takes it back, the curve re-seeded where the car is and checked (turnpilot.signal_turn_go_revoked;
  review 2026-11-06: dropped for good at the lift, a driver who lifted to brake by hand and engaged at the line was
  pulled away on nothing read); it is the car's once OVERRUN_PAST_M past the line faster than COMMIT_SPEED, and slower there
  the car is held where it stops. Lifted short of that, the curve carries on, and a creep lifted past the line is held
  where it stops. A red's or a yellow's stop keeps its overrun (stopOverrun). Let go by a steady
  green it goes (turn_go) until a red or yellow is read, as any signal, or TURN_GREEN_STALE_S passes with no green read,
  moving or standing (a head lost from view, a queue at the back of a green: the light may have changed unread): then it is
  a stop for want of a green again wherever a yellow there would still be stopped for (beyond v*T_LAT + v^2/(2*yellow) and
  LATCH_NEAR_M; standing, anywhere short of its line), and committed, as such a yellow, where it wouldn't
  (turnpilot.signal_turn_green_committed). Marked only short of the latch's reach past its line and where a stop at its
  line is MAP_DECEL_MAX or gentler (said, turnpilot.signal_turn_late, and left to turn_skip, which skips a turn too fast for
  it); the window picks the first mark only, on a frame the signal is in it and has been on most of the maneuver's frames
  (JunctionLine.most: never on one frame's read), and a signal once the maneuver's stays the maneuver's however its
  junction's distance is re-read (it jitters metres against the line's). Unmarked, its stop for want of a green dropped,
  only when the route's turn moves on, is skipped or is the unprotected left's, unless held (position-free). stop_ahead()
  has its stop (never while the driver's go stands), so turn_skip takes the turn from it, never too fast. A signal mapped
  further back than PROTECT_BEFORE_M is no turn's: on both 11-06 drives each was the signal of the junction the route goes straight
  through before the turn's (1e maneuver 12, 1d maneuver 5: 34-42 m back, navInstruction.throughLanesDistance putting
  that junction 29-32 m before the maneuver point and the signal's stop point 11 m short of it, the turn a slight left at
  a junction of its own), the model's as any straight one's.
- A route turn at a junction with no mapped signal (2026-11-06, the real car: a right on red rolled through at a junction
  OSM has no signal for, the camera reading the red in shadow, the model driving the turnRight desire through it): with
  the camera acting (add-only), a red, or a yellow still feasible to stop for, read on a governing head associated with
  the route's TURN maneuver's junction (update()'s junction; the line CONTROL_LINE_M short of it, the association window
  as a mapped signal's, its near edge from PROTECT_BEFORE_M before the junction for a head no other line takes:
  CAMERA_JUNCTION_BEFORE_M, rails-3, review 2026-11-07, 1e 7:06's head read 9-12 m short of the line and a junction read
  1.5 m longer made no signal at all) and owned by no mapped signal makes a signal control of its own there (camera_signal, key
  (CAMERA_ROUTE, maneuver id)): every rule above, the curve, the latch, the green release and the stand handback. For a
  turn TurnPilot drives, so does any governing head associated with it, whatever its colour (unknown too, as every head of
  the 11-06 night read): a light is there, and the turn stops for want of its green (above), once the head is seen there in
  TURN_HEAD_RUNS road runs of the reader (one run's box, a streetlight or a sign, is no light) and where a stop at the line
  is still MAP_DECEL_MAX or gentler (said once if not, turnpilot.camera_signal_late). Not where a mapped signal is at the
  junction (its own stop is the stop), whatever the head reads, red too: one that governs it for the maneuver, still listed
  or passed, done or not (_junction_lines, JunctionLine.governs: the turn's, or in the junction's window on most of the
  maneuver's frames; review 2026-11-06: tested on this frame's live signals alone, a junction re-read further on, a mapped
  stop released by a green or driven through, or a handback's takeover made a second stop for want of a green 10-22 m past
  the mapped line, the car stopped again in the junction's approach; and a red read on the far-side head after a go through
  a line set back 25 m, a green, the driver's go or a takeover, made the camera's line 15 m past it and braked the car at
  -2.0 into the junction's approach, cannotStopComfortably. Remembered from any one frame's window instead, the previous
  junction's signal 34 m back, read once 6 m nearer through route_follower's 1.4-6.8 m jitter, made the junction a mapped
  signal's, and after a go through it a red on the junction's own head stopped nothing, said nowhere). One lost unpassed (a
  reroute's controls no longer list it) governs nothing. A red the gate refuses with the car short of the junction's own
  line and no line taking it is logged once a maneuver (turnpilot.camera_red_refused, error): a signal 30 m or less back of
  the junction, a mid-block crossing's or a near junction's too, can't be told from a wide junction's set-back line by
  where it is (MapControl.junction is false on both: the 11-06 drives' set-back signals, 21-25 m back of their maneuver
  points, read false), and a red on the junction's own head past it is driven through, said there. The junction's own line
  counts toward the mapped line instead (_turn_camera: a line set back 26 m has its far-side heads out of its own window): their
  green releases it, and their red, its own heads reading none, is its red (a red ahead of a line not yet passed stops the
  car at that line, takes back the driver's go), and so it is for a governing line the turn isn't marked on (a turn not
  driven: skipped, going straight on the model, or the unprotected left's; S1, review 2026-11-07: a line 26 m back, its
  junction's far head 41 m past it red from 166 m out, and the car went through at 13.4 m/s, said only in the log). Gone
  through on a go (crossed faster than COMMIT_SPEED with nothing to stop for: crossed_v), a line takes no camera red
  (_camera_adds): the car clears the junction. Nor where the unprotected turn's own hold holds the car at it (the
  autonomous left's go already needs no red at its junction, turn_light). Once made it stays, its line re-read from the
  junction each frame (dead-reckoned without one, frozen while held) until passed, released or overrun, or dropped
  un-engaged when the route's maneuver moves on.
- A mapped stop the camera never saw (row C11; STOP_SIGNS.md v3 SG4b) is stopped for anyway, the gentle error, and the
  decider certifies it as mapped; at its hold its approach's class-11 shadow evidence is logged (update()'s signs: the
  forward runs counted from SIGN_LOOK_M, their stop-sign boxes, the best conf; turnpilot.map_stop_signs), and SIGN_UNSEEN_RUNS
  or more with none is said as unseen (turnpilot.map_stop_unseen) for the replay gate. Nothing here acts on a sign.
- A yield caps the speed to YIELD_SPEED at its line and goes once passed, but only while its rolling test passes
  (_yield, STOP_SIGNS.md 2.7, row C5); one whose test fails becomes a full stop at its line (yieldStop), the stop-sign
  decider's like any stop sign, and its handback says "Yield" (YIELD_STOP_REASON; YD-01). One whose road outranks every
  crossing one (crossSuspect: likely mapped for the cross road) is dropped and logged. Such a stop sign still stops, and
  says the approach is unverified.
- The driver's gas before a stop sign is an override; when it comes off past the line moving faster than COMMIT_SPEED,
  or past where the latch reaches (LATCH_PAST_M) at any speed, the stop is dropped (stopOverrun), else the curve resumes
  from there (re-seeded, checked for feasibility). Past the line beyond the latch's reach with no release, at any speed,
  is an overrun: dropped, logged, alerted, never a stop in the junction with nothing to release it. Driven off a held stop
  under the driver's foot is the same.
- An unprotected turn (Kellen, 2026-10-27: the same rule as stop signs and lights) is a routed far-side turn (a left on the
  right) across an oncoming approach (navInstruction.maneuverOpposing present, or unknown: no junction data is never
  clear) with no stop sign of its own at the junction (one within PROTECT_BEFORE_M before it to PROTECT_PAST_M past it, not
  a rail orphan's with the tracks between it and the junction (rail_between): a stop sign already stops; the maneuver stays
  protected once seen so, after its control drops out behind the car). A signal there doesn't protect it
  (SIGNAL_PROTECTS_LEFT): a left on a green ball is unprotected. turnpilotd hands it in each frame (update()'s turn) and
  it becomes a control of its own, kind unprotectedTurn, at the junction's hold line less
  MAP_MARGIN_M, or where a signal is mapped there in the junction's mouth (SIGNAL_LEFT_HOLD_M, 2026-11-08: a left on a green
  ball rolls up to the junction and judges the oncoming lanes there, the signal's own control stopping the car at its line
  for a red or yellow read): the curve, the latch and the stop sign's rules. Held, it does what the unprotected left asks (update()'s
  left, a LeftCommand from unprotected_left.py through turnpilotd; TurnPilotAutoLeft): auto (the default; Kellen,
  2026-10-28, "No it has to ship taking the turn", no intervention) holds it, never handing back but for the left's fault
  (leftUndecidedTakeControl), creeps its stop point on to where the left asks (CREEP_SPEED, CREEP_ACCEL; still the latch,
  never past it), and releases it on the left's go (cause gap); announce and off hand it back at once
  (unprotectedTurnTakeControl), announce saying whether it would have gone.
  A failing or missing controls status leaves the turn unprotected. One the
  car isn't held at goes with its maneuver (skipped too fast, rerouted away, passed), never with a fault (it carries on,
  dead-reckoned); a held one is position-free like any latch. Its junction is the nearest of navd's readings
  (route_follower.py), so a new one can put the line behind where the car can stop at MAP_DECEL_MAX: it is never dropped
  as an overrun then (crossed blind), but moved up to where the car stops braking at that floor, no harder, said out
  loud (cannotStopComfortably), and handed back there like any other. scene_policy.py's oncoming and pedestrian holds
  only add holds.
- Off the route with a destination, the controls are navd's lookup ahead of the car (control_tracker.py), under the same
  rules. With nothing known ahead past the tracker's grace (off_route_unknown: the lookup pending, failed, unsupported,
  partial, or stale), the car never drives on silently: a speed cap comes down from its speed at OFF_ROUTE_DECEL to
  OFF_ROUTE_SPEED, and offRouteStopsUnknown says so ("Off route, map stops unknown"), for as long as it lasts. Never a stop:
  nothing is known to stop for, and the driver is asked to take control or wait for the reroute. Under TurnPilotAlwaysDrive
  (ALWAYS_DRIVE.md R17) the model drives on as stock openpilot would: the cap comes down only to OFF_ROUTE_DRIVE_SPEED (25
  mph), or the last valid navInstruction.speedLimit before the route was lost where lower, offRouteStopsUnknown is said for
  OFF_ROUTE_ALERT_S at its onset and the map status line (mapControlsUnavailable) after, saying the car won't stop for signs
  or signals there, and nothing says take control.
- navd's controlsStatus other than ok (navd always sets it; none is a navd from before controls) raises
  mapControlsUnavailable and keeps "Map stops/signals: <status>" in the reason: no controls on tiles without control
  data is never shown as clear.
"""
import itertools
import math
from dataclasses import dataclass, field

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.control_tracker import JUMP_M, ControlTracker, TrackedControl
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S, SPEED_CAP_ACCEL_MIN, TAKE_CONTROL_PROMPTS, plan_accel
from openpilot.selfdrive.turnpilot.route_follower import LEFT, TURN
from openpilot.selfdrive.turnpilot.scene_policy import (HARD_STOP_MIN_M, HOLD_LINE_M, NO_ACCEL_CAP, ONCOMING_ENTRY_M, STOP_LAG_S,
                                                        STOP_SPEED, Stop)
from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP

MAP_STOP_DECEL = 1.5          # m/s^2, the braking curve to a map stop point
MAP_DECEL_MAX = SPEED_CAP_ACCEL_MIN  # -2.0 m/s^2: a map stop never brakes harder
YELLOW_DECEL = 2.0            # m/s^2 for a stop decided on a yellow (TurnPilotYellowDecel)
YELLOW_DECEL_MAX = -MAP_DECEL_MAX  # 2.0: a light stop never brakes harder than any map stop (Kellen, 2026-10-28)
CONTROL_LINE_M = 10.          # without a mapped line, the stop line is the junction less this (routing/controls.py)
LATCH_SPEED = 0.3             # m/s: slower than this...
LATCH_NEAR_M = 5.             # ...this close to the stop point...
LATCH_PAST_M = 2.             # ...or this far past it, the latch holds the car
HOLD_ACCEL = -0.5             # m/s^2, the held car's accel cap
# s a hold taken up by a re-engage leaves a rolling car at its speed before it brakes (reengage_stop_m): the plan's frame and
# the car's longitudinal actuator delay (opendbc's default 0.15 s, the Model 3's)
REENGAGE_LAG_S = 0.2
REENGAGE_MAX_S = 60.          # s of a hold's braking reengage_stop_m follows at most
STOP_SIGN_HANDBACK_S = 1.0    # s stopped at a stop sign before it hands back (DESIGN phase 4: a 1 s full stop)
GREEN_STEADY_S = 0.5          # s of green before it counts (the sim oracle; a camera would need 1.0)
# s of the camera's green (its reader acting: add-only) before it lets TurnPilot's own stop at a signal go (a route turn's want
# of a green, or a stop a camera red or yellow added): over the reader's own vote (light_state.VOTE_N of VOTE_M road runs, about
# 1 s at the 1-in-3 road run of a stand), so one flickering read never lets the car go. Never plannerd's lift: that needs an
# accepted green (release_light)
CAMERA_GREEN_STEADY_S = 1.0
# a held signal with no light read for it this long (a red read, then lost: on the car the camera's own stop, its read gone
# for CAMERA_STALE_S and more) is said, once as it starts (turnpilot.signal_unseen) and on its line (SIGNAL_UNSEEN_REASON),
# and held for a green, never let go by the timeout. BeamNG S14 (BUILD_FINDINGS N3) stood 108.6 s on "waiting for green" once
# the oracle stopped listing the light it stood 1.3 m past: longer than a flicker or a read lost behind a truck, it is said.
# Until 2026-11-07 it handed back here (signalUnseenTakeControl, "Signal not seen: take control"); Kellen: "no, no handbacks".
# Held rather than released to the model (the module docstring's choice): a release at a standstill starts turnpilotd's
# no-move clock (gap B: NO_MOVE_S, NO_MOVE_MAX_S under TurnPilotAlwaysDrive), which ends in cantStartTakeControl while the
# model holds at the red it may still be (a cycle runs 30-90 s), so it would trade this handback for that one; and the one
# red the model was seen to run was with a turn desire out (1e 8:20.4), where a release would leave nothing holding the car.
# The reader gets road frames at a stand since 7f67b8684, so a green ends the hold where it is read (not yet seen on the car)
SIGNAL_UNSEEN_S = 10.
SIGNAL_UNSEEN_REASON = "Light not seen: waiting for green"
# A route turn at a signal (module docstring; 2026-11-06): until 2026-11-07 held at its line for want of a green and, no light
# read TURN_UNSEEN_S, handed back (TURN_LIGHT_REASON). No stop for want of a green is made since (Kellen: an unread light is no
# reason to wait; the turn goes with the model), so neither is said. Kept for the shadow checklist's words
# (tools/turnpilot/tests/test_docs_match_code.py reads them) until that page drops them
TURN_UNSEEN_S = 5.
TURN_LIGHT_REASON = "Turn at a light: take control"
TURN_STOPPING_REASON = "Turn at a light: stopping"
TURN_WAITING_REASON = "Turn at a light: waiting for green"
# a steady green let a route turn's signal go: it goes while a green is read for it (a red or yellow read stops it as any
# signal's), but this long with no green read, moving or standing, the light may have changed unread and it needs a green
# again: stopped for as a yellow would be, where a stop at the yellow rate is still feasible (beyond v*T_LAT + v^2/(2*yellow),
# and LATCH_NEAR_M), committed where it isn't (_signal_reads). Under the 3 s minimum yellow, over CAMERA_STALE_S (a read's own
# life) and the reader's 1-in-3 road runs, so a head lost for a second behind a truck never brakes the car. Review 2026-11-06:
# counted only standing, a green read once 100-120 m out then never again let the car turn through the line 7.5 s after it
# at 13.4 m/s, with no stop and nothing said
TURN_GREEN_STALE_S = 3.
T_LAT = 0.8                   # s from a yellow to braking
COMMIT_SPEED = 2.0            # m/s: past the line faster than this, the car went
YIELD_SPEED = 5.0             # m/s at a yield's line, while its rolling test passes (STOP_SIGNS.md 2.7)
YIELD_STOP = 'yieldStop'      # ControlState.mode of a yield that became a stop (its rolling test failed): a stop from then on
YIELD_ROLLING = 'yieldRolling'  # ...of one whose rolling test held at its rolling commit point: taken rolling at YIELD_SPEED
# m before a yield's stop point its rolling commit point lies at least (the yields input may put it further out): see
# yield_commit_m, the point a stop at its line is still MAP_DECEL_MAX from the car's speed
YIELD_COMMIT_M = 4.3
YIELD_COMMIT_MARGIN_M = 1.0
YIELD_COMMIT_MAX_M = 15.      # m: never decided further out (the rolling test is re-read every frame until then; on the yield's
                              # curve the car is about 6.5 m/s here, a stop at the line from it -1.5 m/s^2)
YIELD_STOP_REASON = "Yield: take control to proceed"
RELEASE_SPAN_M = 30.          # releasing a control releases those this far past it
OVERRUN_PAST_M = 2.           # m past the line, moving faster than COMMIT_SPEED with no release: an overrun
# a red (or a stop decided on a yellow) the car crosses no faster than COMMIT_SPEED is never dropped as an overrun (SAT_VALIDATION
# row 14, red_right_g62: BeamNG's light brake below 3 m/s crept the car 2 m over a red's line, the red was dropped and the turn
# went on red; Kellen: "it should always stop on red"): the car stops where it is on the map curve's MAP_STOP_DECEL and is held
# there, waiting for a steady green as at the line, saying so. Faster than COMMIT_SPEED past the line it is in the junction:
# the overrun as before, never a stop there
RED_PAST_REASON = "Waiting at the light"
DRIVE_OFF_M = 2.              # m a held car moved under the driver's gas: driven off
LIGHT_ASSOC_M = 25.           # a light belongs to the signal control whose stop point is this close to its own
LIGHT_STALE_S = 0.6           # s: an older light message is stale (unknown)
CAMERA_STALE_S = 1.0          # s: a camera light read older than this is none (it reads on road camera runs only, ~5 Hz)
CAMERA_ASSOC_BEFORE_M = 10.   # a camera head from this far short of a signal's line...
CAMERA_ASSOC_PAST_M = 40.     # ...to this far past it is that signal's (far-side heads hang ~18-35 m past the line)
# A head no line's window takes, short of the camera's own line at a route turn's junction (CAMERA_ROUTE: the junction less
# CONTROL_LINE_M, nothing mapped to say where the stop is) from PROTECT_BEFORE_M before the junction, the window that makes a
# mapped signal the junction's, is that line's too (camera_owner). 1e 7:03-7:07 (2026-11-06 replay): route_follower's junction
# read long (the car's steering turned in about 9 m past the line), the governing head read 9-12 m short of the line, its margin
# to the 10 m near edge -1.9 to +0.7 m over eight road runs; the junction read 1.5 m or 3 m longer, inside the 1.4-6.8 m jitter
# of one approach, and no camera signal was made at all: a right driven on a light nobody read (rails-3, review 2026-11-07).
# Only heads no other line owns: a mapped line's own heads stay its own. CAMERA_JUNCTION_BEFORE_M, below PROTECT_BEFORE_M
# A colourless head (unknown, as every governing head of the 11-06 night read) makes the camera's signal at a route turn's
# unmapped junction (_camera_signal) only once it is associated with the junction's line in TURN_HEAD_RUNS road runs of the
# reader (CameraLights.run), none more than TURN_HEAD_GAP_S of no head there after the last: one run's box (a streetlight, a
# sign, the next junction's head at a misjudged distance) makes no stop and no handback (review 2026-11-06: one frame of an
# unknown head was a stop and a loud handback at a junction with no light). Counted in runs, not seconds held: turnpilotd
# carries a run's read CAMERA_STALE_S, but the next run replaces it, and at night the head blinks in and out of the runs. 1e's
# two camera signals (2026-11-06 replay, the road runs 0.10-0.15 s apart): the head in 8 of 21 runs over 2.9 s, 20-22 m short
# of the junction, inside the association window (its near edge) only in its last runs, then no road run at all from 40 m
# out; and in 5 of 11 runs over 1.2 s, blank runs between, then none. Neither was associated 1.5 s together (a blank run
# replaces the carried read) nor 1.5 s from its first associated run, so a rule in seconds would have made neither stop; two
# runs made both, 0.1 s later than one (31.0 and 25.0 m from the line at 6.5 m/s, -0.7 and -0.85 m/s^2). A red or yellow needs
# no count: light_state's vote already spans VOTE_N of VOTE_M runs
TURN_HEAD_RUNS = 2
TURN_HEAD_GAP_S = CAMERA_STALE_S
# The camera's light reader failing (lightStatus disabled: since 2026-11-06 three failed reads in a row, still retried 1-60 s
# apart and read again at its first good read, never off for the drive; modeld_detector LIGHT_DISABLED_FAILS): a status line
# for the whole failing spell, from its disabled to its next good read, where nothing else is (S2, review 2026-11-07: said on
# a 2 s hold once a spell, it was lost under a lane line or a prompt, or run down before the driver engaged, and for the rest
# of the drive no camera red could stop the car with nothing saying so). Its onset chimes (CAMERA_OFF_REASON, its own hold
# ABORT_HOLD_S, run down only where it could be shown): once a failing spell, the next only after the reader read again, and
# no sooner than CAMERA_OFF_AGAIN_S after the last (a reader failing three reads and reading one, as one that throws on a head
# in view and reads a run without one would, chimes once a minute, not every 4 s; a spell still on then chimes then). The rest
# of the spell the same line, silent (CAMERA_OFF_STILL_REASON)
CAMERA_OFF_REASON = "Camera light reader failing: lights unknown"
CAMERA_OFF_STILL_REASON = "Camera light reader still failing: watch every light"
CAMERA_OFF_REASONS = (CAMERA_OFF_REASON, CAMERA_OFF_STILL_REASON)
CAMERA_OFF_AGAIN_S = 60.
SIGNAL_GO, SIGNAL_STOP, SIGNAL_COMMITTED = 'go', 'stop', 'committed'  # a signal's ControlState.mode
OFF_ROUTE_SPEED = 5.          # m/s: off the route with the stops ahead unknown, the speed is capped to this...
OFF_ROUTE_DECEL = 1.0         # m/s^2: ...coming down from the car's speed this gently
OFF_ROUTE_REASON = "Off route, map stops unknown — take control or wait for reroute"
# TurnPilotAlwaysDrive (ALWAYS_DRIVE.md R17): the model drives on as stock openpilot does off a route, the cap coming down only
# to OFF_ROUTE_DRIVE_SPEED (25 mph), or to the last valid navInstruction.speedLimit before the route was lost where that is
# lower; said once at onset (offRouteStopsUnknown, OFF_ROUTE_ALERT_S), then the map status line (mapControlsUnavailable)
OFF_ROUTE_DRIVE_SPEED = 11.2  # m/s
OFF_ROUTE_ALERT_S = ABORT_HOLD_S
OFF_ROUTE_DRIVE_REASON = "Off route, map stops unknown — won't stop for signs or signals"

UNPROTECTED = 'unprotectedTurn'  # MapControlStatus.Kind's name for it
UNPROTECTED_ROUTE = -1        # its key is (UNPROTECTED_ROUTE, maneuver id): a maneuver's, not a route's control
CAMERA_ROUTE = -2             # a camera signal's key is (CAMERA_ROUTE, maneuver id): a junction's the map has no signal for
KEEP_TURN = 'keep'            # update()'s turn when the route isn't known this frame (a fault): the turn's stop carries on
MAP_MARGIN_M = 1.             # m short of the hold line an unprotected turn stops, as navd's controls stop short of their line
PROTECT_BEFORE_M = 30.        # a stop sign or signal this far before the junction (its line, mapped or the junction less 11)...
PROTECT_PAST_M = 2.           # ...to this far past it is the junction's own: the turn isn't unprotected
CAMERA_JUNCTION_BEFORE_M = PROTECT_BEFORE_M - CONTROL_LINE_M  # 20 m short of the camera's own line (above, camera_owner)
# A route turn's junction's mapped signals, as read against it (MapControlPolicy._junction_reads), are kept for this many
# maneuvers back: one hunted back to is its own again (maneuver ids only rise along a route)
JUNCTION_LINES_KEPT = 4
# A signal at the junction doesn't protect a left (DESIGN phase 4, decision 4): signals don't hold now, and nothing here can
# tell a green arrow from a green ball, so a left at a signal is unprotected. True: a signal protects it, as a stop sign does
SIGNAL_PROTECTS_LEFT = False
# ...but it isn't held at the stop line either (Kellen, 2026-11-08, drive 29: every left at a green ball held at a 9 m line
# and the driver drove them all): at a junction where a signal is mapped, the left's hold is where a driver waits on a green
# ball, in the junction's mouth, this far before the maneuver point: CREEP_SHORT_M short of where the left's full creep
# stands the car (unprotected_left: ONCOMING_ENTRY_M + CREEP_SHORT_M) and 1.5 m past its position fault (HELD_MIN_M), so
# the latch's own stopping spread never reads as one. The approach is the model's (no stop at the line on a green: the
# mapped signal's own control stops the car at its line for a red or yellow read, as at any signal), the car rolls up to the
# mouth and judges the oncoming lanes from there (the gap rule, unprotected_left.py), and goes when they are clear. A red
# read too late for the signal's line is still a stop short of the oncoming lanes at the mouth (2026-11-08 cause 6: overrun
# at the line). Where no signal is mapped (none, or one only the camera boxes) the hold stays at the line
# (left_hold_m): there the left's hold is the only stop the junction has (_camera_signal's own_turn gate)
SIGNAL_LEFT_HOLD_M = ONCOMING_ENTRY_M + 2.

STOP_SIGN_REASON = "Stop sign: take control to proceed"
# A stop sign's line or an unprotected turn's hold line crossed with no go of TurnPilot's (its latch let go as the driver's,
# or overrun: owner decision (b), 2026-10-30) is never a silent crossing: its take-control prompt (DRIVEN_PROMPT) and
# reason stay up until the car is this far past the stop point, through the junction (its line to its centre is
# CONTROL_LINE_M without a mapped line, an unprotected turn's hold line LINE_D short of it, and the far side beyond), engaged
# or not, so a re-engage inside the junction hears it again. It assumes a junction under about 20 m across past its centre:
# a mapped line set well back from a wide junction (BUILD_FINDINGS D4's setback) can end it before the far side
DRIVEN_THROUGH_M = 30.
# navd's bad controls statuses, how much they leave unknown (MC-03: a worse one is said again with its chime): a lookup pending,
# a walk or snap route or some tiles without the sidecar, none at all
STATUS_RANK = {'pending': 1, 'walkOrSnap': 2, 'partial': 2, 'unsupported': 3, 'failed': 3}
CROSSED_CAUSES = ('driverWent', 'drivenOff', 'movedOff')  # a latch let go by the car's motion, never by a go
DRIVEN_PROMPT = {'stop': 'stopSignTakeControl', UNPROTECTED: 'unprotectedTurnTakeControl'}
# The autonomous unprotected left (unprotected_left.py, DESIGN phase 4 G4) creeps its held stop point on for a view
CREEP_SPEED = 0.7             # m/s
CREEP_ACCEL = 0.5             # m/s^2
CREEP_AT_M = 0.1              # crept to within this of its target: held there again
# A mapped stop the camera never saw (row C11; STOP_SIGNS.md v3 SG4b): stopped for anyway, logged at its hold with the
# class-11 shadow evidence of its approach (TurnPilotCameraStopSigns shadow: nothing acts on a sign this week)
SIGN_LOOK_M = 40.             # m before its stop point its forward runs are counted
SIGN_SEEN_HITS = 2            # stop-sign boxes in them that say the camera saw a sign
SIGN_UNSEEN_RUNS = 10         # forward runs with none, and the stop is logged as unseen (the replay gate's)


@dataclass
class Driven:
  """A stop line or an unprotected turn's hold line crossed with no go (DRIVEN_THROUGH_M)."""
  kind: str                  # 'stop' (a stop sign, or a yield become one) or UNPROTECTED
  key: tuple[int, int]
  left_m: float              # m still to go to DRIVEN_THROUGH_M past the stop point
  prompt: str
  reason: str
  cause: str                 # driverWent, drivenOff, movedOff, overrun

  @property
  def maneuver_id(self) -> int | None:
    """An unprotected turn's maneuver (its key's second half), None for a stop sign."""
    return self.key[1] if self.kind == UNPROTECTED else None


@dataclass
class StopCommand:
  """What the stop-sign decider (stop_go.py, through turnpilotd) asks of a held stop sign's latch (module docstring). latch:
  shadow (the 1 s stopSignTakeControl handback, as before), wait (held, no handback), fault (handed back now); reason: what to
  say (shadow's default is STOP_SIGN_REASON); action: the latch's MapControlStatus.Action while it waits (waitingCross,
  creeping)."""
  latch: str = 'shadow'
  reason: str = ''
  action: str = ''


@dataclass
class LeftCommand:
  """What the unprotected left (unprotected_left.py, through turnpilotd) asks of its hold line's latch. mode:
  TurnPilotAutoLeft's (off: handed back at once, as G1; announce: handed back, the reason its verdict; auto: held for its
  go, no handback); go: release it now (cause gap); creep_m: its stop point this far past where it latched;
  take_control: a fault (undecided), handed back held (leftUndecidedTakeControl); reason: what to say; crossed: the frame
  the driver took the car past the left's own hold after its go (CROSSED_CAUSES), its crossing said (Driven)."""
  mode: str = 'off'
  go: bool = False
  creep_m: float = 0.
  take_control: bool = False
  reason: str = ''
  crossed: str = ''


def latch_hold_caps(v: float) -> tuple[float, float]:
  """The held latch's (accel cap, speed cap) at v m/s: its stop's (_held's, held_stops)."""
  return HOLD_ACCEL, 0.


def reengage_stop_m(v: float, caps) -> float:
  """m a hold taken up by a re-engage carries a car rolling at v m/s before it stands (the lanes test of _held's latch and
  unprotected_left's own hold): REENGAGE_LAG_S on at v, then braking as the hold itself asks, caps(v) its (accel cap, speed
  cap) at each speed through plan_accel (its candidate in the planner), down to STOP_SPEED, and from there at -HOLD_ACCEL;
  inf if it never brakes. Never HOLD_ACCEL from v: that is the held car's accel cap, not what a hold brakes at, and put a car
  re-engaged at 1.9-2.1 m/s 3.7-4.5 m on where the holds stood it in 1.55-1.66 m (skeptic round 1). In the car a hold that
  says shouldStop still rolling (the latch's) goes to longcontrol's stopping state, which ramps its accel down from the
  last output at 1 m/s^2/s instead of following plan_accel: from 2.1 m/s that stands the car about 2.9 m on, past this."""
  v = max(v, 0.)
  d = v * REENGAGE_LAG_S
  for _ in range(round(REENGAGE_MAX_S / DT_MDL)):
    if v <= STOP_SPEED:
      return d + v * v / (2. * -HOLD_ACCEL)
    a = plan_accel(*caps(v), v)
    if a >= 0.:
      break
    v = max(v + a * DT_MDL, 0.)
    d += v * DT_MDL
  return math.inf


def yield_last_stop_m(v_ego: float) -> float:
  """m before a yield's line the last point is, at speed v_ego, from which a stop at the line is MAP_DECEL_MAX or gentler
  through the speed cap's lag (STOP_LAG_S), plus YIELD_COMMIT_MARGIN_M: where a yield whose rolling test fails becomes a stop."""
  v = max(v_ego, 0.)
  return v * STOP_LAG_S + v * v / (2. * -MAP_DECEL_MAX) + YIELD_COMMIT_MARGIN_M


def yield_commit_m(v_ego: float, commit_m: float = YIELD_COMMIT_M) -> float:
  """m before a yield's stop point its rolling commit point lies at speed v_ego (STOP_SIGNS.md 2.7): where the rolling test
  passing takes it rolling, the last stopping point (yield_last_stop_m) or commit_m if further, never more than
  YIELD_COMMIT_MAX_M out (the test re-read every frame until then). 2.7's 7.6 m from the closest edge at 5 m/s is nearer: a
  test failing there would end the stop up to 2.5 m past the line, beyond the latch's LATCH_PAST_M (an overrun into the
  junction with nothing to release it), so a failing test makes the yield a stop at its last stopping point instead."""
  return min(max(commit_m, yield_last_stop_m(v_ego)), YIELD_COMMIT_MAX_M)


def is_stop(c) -> bool:
  """A control that is a stop: a stop sign, or a yield that became one (yieldStop: STOP_SIGNS.md 2.7), which the stop-sign
  decider then takes as any stop sign (held_control, its go a stopSign go)."""
  if c is None:
    return False
  return c.kind == 'stop' or (c.kind == 'yield' and getattr(getattr(c, 'state', None), 'mode', '') == YIELD_STOP)


def engage_distance(v_ego: float) -> float:
  """m from the stop point a control engages: braking at MAP_STOP_DECEL, a second's lag, 15 m to spare."""
  return v_ego ** 2 / (2. * MAP_STOP_DECEL) + v_ego * STOP_LAG_S + 15.


def curve(v_ego: float, d: float) -> float:
  return math.sqrt(2. * MAP_STOP_DECEL * max(d - v_ego * STOP_LAG_S, 0.))


def needed(v_ego: float, d: float) -> float:
  return -v_ego ** 2 / (2. * max(d, HARD_STOP_MIN_M))


LEFT_LINE_M = HOLD_LINE_M + MAP_MARGIN_M  # m before the maneuver point an unprotected turn's hold is with no signal mapped


def turn_stop_feasible(v_ego: float, distance: float, hold_m: float = LEFT_LINE_M) -> bool:
  """An unprotected turn distance m ahead can be stopped for at its hold (hold_m before it: MapControlPolicy.left_hold_m) at
  MAP_DECEL_MAX or gentler."""
  return needed(max(v_ego, 0.), distance - hold_m) >= MAP_DECEL_MAX - 1e-9


def signal_stopping(st) -> bool:
  """A signal's state is stopping the car (a red, a yellow decided a stop), not yet released by a steady green, never one it
  goes through nor the driver's go through it (driver_go). One rule for stop_ahead (turnpilotd's other_hold for the left)
  and turn_light (the left's light), so the two never disagree (review 2026-11-07)."""
  return st is not None and not st.done and st.mode == SIGNAL_STOP and not st.driver_go


def unprotected_turn(maneuver, right_hand_traffic: bool = True) -> bool:
  """A routed turn across the oncoming traffic of the road straight ahead: a junction turn to the far side (a left on the
  right) whose junction has an oncoming approach, or might have (opposing unknown: no junction data is never clear). A
  keep, a bend and a T's stem (opposing none) aren't. The junction's own stop sign or signal is MapControlPolicy's to
  check (protected())."""
  return maneuver is not None and maneuver.kind == TURN and (maneuver.side == LEFT) == right_hand_traffic and \
    maneuver.opposing != 'none'


def driving_turn(junction) -> bool:
  """update()'s junction names a route turn TurnPilot is taking: its third field (turnpilotd's: not skipped, not dropped at a
  handback, not replaced by another turn), True with none (a two-field junction: the turn is taken, as turnpilotd's at
  5dcb363 means it). A skipped turn's junction goes straight through: its signal is the model's, as any straight one's."""
  return isinstance(junction, tuple) and (len(junction) < 3 or bool(junction[2]))


def cross_road_yield(c) -> bool:
  """A crossSuspect yield the sidecar confirms governs a crossing approach (crossGoverned): the cross road's, not ours, so it
  stops nothing (_yield drops it: STOP_SIGNS.md 2.7)."""
  return c.kind == 'yield' and bool(getattr(c, 'cross_suspect', False)) and \
    bool((getattr(c, 'junction_info', None) or {}).get('crossGoverned'))


def near_rail(c) -> bool:
  """A control near a mapped rail level crossing (junctionInfo.railDistance >= 0: routing/controls.py, the OSM sidecar's): a
  rail orphan's stop before its tracks, or a junction's within 30 m of them. Never released by another's release (_release)."""
  rail = (getattr(c, 'junction_info', None) or {}).get('railDistance')
  return rail is not None and float(rail) >= 0.


def rail_between(c) -> bool:
  """Its mapped rail crossing lies between its stop point and its node (railDistance under stopPointDistance): the stop is the
  tracks', a rail orphan's, never its junction's (protecting())."""
  info = getattr(c, 'junction_info', None) or {}
  return near_rail(c) and float(info.get('railDistance', -1.)) < float(info.get('stopPointDistance', 0.))


def line_setback_at(distance: float, controls) -> float | None:
  """MapControlPolicy.line_setback's rule over controls (each with d, its stop point's m ahead, line_mapped, kind, cross_suspect
  and junction_info as a dict): m the junction distance m ahead's mapped stop line lies back of the unprotected turn's hold
  line, 0 at or short of it, None without one. Its own controls only (PROTECT_BEFORE_M before it to PROTECT_PAST_M past it),
  never the cross road's yield nor a rail orphan's stop. One rule for turnpilotd's tracked controls (line_setback) and navd's
  route controls (routing/lefts.py), so the R_req both log through unprotected_left.route_range agrees: turnpilotd's
  left_not_straight line and navd's 'turnpilot lefts:' line. Logs only: since 2026-11-06 no left is skipped, routed around or
  refused for its reach (Kellen: "never route around or skip a left for range"); each is driven to its hold line and judged
  there, from where the car stands (unprotected_left)."""
  lines = [distance - c.d for c in controls
           if c.line_mapped and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M and not cross_road_yield(c) and
           not rail_between(c)]
  return max(max(lines) - HOLD_LINE_M - MAP_MARGIN_M, 0.) if lines else None


def protecting(c) -> str:
  """How control c stops the car at its junction (protected()): 'stop' a stop sign (a signal too, with SIGNAL_PROTECTS_LEFT) or
  a yield that became one (yieldStop), whose cross traffic the stop-sign decider watches, not only the oncoming the unprotected
  left would; 'pending' a yield of ours not yet decided (it becomes a stop unless certified rolling at its commit point:
  STOP_SIGNS.md 2.7); '' one that won't stop the car: taken rolling (yieldRolling, nobody watching the oncoming), passed or
  dropped, or the cross road's (cross_road_yield: read here before _yield has dropped it, a new control has no state yet)."""
  if rail_between(c):
    return ''  # a rail orphan's: it stops the car short of the tracks, not at the junction past them (residual-check-2)
  if c.kind == 'stop' or (c.kind == 'signal' and SIGNAL_PROTECTS_LEFT):
    return 'stop'
  if c.kind != 'yield' or cross_road_yield(c):
    return ''
  st = getattr(c, 'state', None)
  if st is None:
    return 'pending'
  if st.mode == YIELD_STOP:
    return 'stop'
  return '' if st.mode == YIELD_ROLLING or st.done else 'pending'


def protection(controls, distance: float) -> str:
  """The strongest protecting() of the controls of the junction distance m ahead (PROTECT_BEFORE_M before it to PROTECT_PAST_M
  past it): 'stop', 'pending' or ''."""
  near = {protecting(c) for c in controls if distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M}
  return 'stop' if 'stop' in near else 'pending' if 'pending' in near else ''


def protected(controls, distance: float) -> bool:
  """A control of the junction distance m ahead stops the car there already, or will unless its yield is certified rolling
  (protection())."""
  return protection(controls, distance) != ''


_READS = itertools.count(1)


@dataclass
class CameraLights:
  heads: list                  # [(voted state, m to the head)] of the governing heads (light_state.py)
  shadow: bool = True          # logged, never acted on; False only with both keys (camera_light_input)
  status: str = 'read'         # turnpilotDetections.lightStatus
  # the road run this read is: each read made its own number, one turnpilotd carries between runs keeps it (TURN_HEAD_RUNS)
  run: int = field(default_factory=lambda: next(_READS), compare=False)


@dataclass
class Lights:
  lights: list | None = None   # [(state, distance)] from an accepted, fresh source; None without one
  source: str = 'none'         # TurnpilotLights.Source of the last message
  rejected: bool = False       # a source that isn't accepted here sent lights (lightSourceRejected)
  camera: CameraLights | None = None  # the camera's reader, fresh (camera_light_input); never in lights


NO_LIGHTS = Lights()


def camera_light_input(msg, status: str, age_s: float, mode: str) -> CameraLights | None:
  """The camera's lights from the last road camera run's turnpilotDetections.lights and lightStatus, age_s old; mode:
  turnpilotd's TurnPilotCameraLights. None when off, stale, or not read (no calibration yet, a retry's backoff, a sweep). A
  reader failing (disabled: three failed reads in a row, still retried, read again at its first good read) comes through,
  with no heads, to be said (CAMERA_OFF_REASON)."""
  if mode == 'off' or msg is None:
    return None
  if status == 'disabled':
    return CameraLights([], True, status)
  if status != 'read' or age_s > CAMERA_STALE_S or str(msg.source) != 'camera':
    return None
  heads = [(str(light.state), float(light.distance)) for light in msg.lights if light.governs]
  return CameraLights(heads, bool(msg.shadow) or mode != 'addOnly', status)


def camera_owner(dist: float, signals):
  """The signal control a camera head dist m ahead belongs to: the last line before it within the association window. A
  head no line's window takes, up to CAMERA_JUNCTION_BEFORE_M short of the camera's own line at a route turn's junction (key
  CAMERA_ROUTE: the junction less CONTROL_LINE_M), is that line's: from PROTECT_BEFORE_M before the junction (rails-3, 1e
  7:06: its governing head read 9-12 m short of the line). Never a head another line's window takes."""
  owners = [o for o in signals if o.d - CAMERA_ASSOC_BEFORE_M <= dist <= o.d + CAMERA_ASSOC_PAST_M]
  if not owners:
    owners = [o for o in signals if o.key[0] == CAMERA_ROUTE and o.d - CAMERA_JUNCTION_BEFORE_M <= dist < o.d]
  return max(owners, key=lambda o: o.d, default=None)


def camera_state(c, signals, camera: CameraLights | None) -> str:
  """The camera's state for signal c: its heads' states, if they agree."""
  if camera is None:
    return 'unknown'
  states = {st for st, dist in camera.heads if st != 'unknown' and camera_owner(dist, signals) is c}
  return states.pop() if len(states) == 1 else 'unknown'


def light_input(msg, age_s: float, simulation: bool, oracle_param: bool) -> Lights:
  """Lights from a turnpilotLights message age_s old. The sim oracle only with SIMULATION=1 and TurnPilotSimLightOracle
  set: anything else that sends lights is rejected, out loud, and its lights are unknown. Stale is unknown."""
  if msg is None or str(msg.source) == 'none':
    return Lights()
  source = str(msg.source)
  if not (source == 'simOracle' and simulation and oracle_param):
    return Lights(None, source, rejected=True)
  if age_s > LIGHT_STALE_S:
    return Lights(None, source)
  return Lights([(str(light.state), float(light.distance)) for light in msg.lights], source)


@dataclass
class Shown:
  id: int
  kind: str
  distance: float
  action: str
  light_state: str = 'unknown'
  light_source: str = 'none'
  controls_status: str = 'none'
  line_mapped: bool = False


@dataclass
class MapDecision:
  stops: list = field(default_factory=list)
  prompt: str = 'none'
  alert: str = 'none'
  shown: Shown | None = None
  reason: str = ''               # the controls' status to show when nothing holds
  released_stopped: bool = False  # a hold let go this frame with the car stopped: turnpilotd re-pulses the turn
  driven: str = ''               # a line crossed with no go: its take-control reason while it is the prompt (Driven), else ''
  # the light reader's failing line is the alert and reason its words (S2): 'onset' its chimed onset with no map prompt over
  # it (turnpilotd puts its words over a hold's then), 'still' after it (over the lane planner's alone), '' none
  reader_line: str = ''


@dataclass
class ControlState:
  cap: float = NO_SPEED_CAP
  infeasible: bool = False
  override: bool = False        # the driver's gas on the approach
  mode: str = 'stop'            # signals: SIGNAL_GO (the default: nothing to stop for), SIGNAL_STOP (a red, or a yellow
                                # decided so), SIGNAL_COMMITTED (a yellow too late to stop for)
  yellow: bool = False          # a stop decided on a yellow
  green_t: float = 0.
  light: str = 'unknown'
  unseen_t: float = 0.          # held at a signal: s since an accepted light was last read for it (SIGNAL_UNSEEN_S)
  unseen_said: bool = False     # ...said last frame (SIGNAL_UNSEEN_REASON, logged once as it starts; never a handback)
  past_red: bool = False        # a signal's stop crept past its line slower than COMMIT_SPEED: stopped where it is (RED_PAST_REASON)
  crossed_v: float = -1.        # a signal's line crossed with nothing to stop for: the speed it was, else -1 (signal_went)
  dwell_t: float = 0.
  moved_m: float = 0.
  asked_m: float = 0.           # held: the furthest the latch's stop point was asked on this hold (a creep, a roll-up)
  gas_moved_m: float = 0.       # held: m moved under the driver's gas this hold
  driver_m: float = 0.          # held: m moved with openpilot disengaged or under the gas this hold (the driver's)
  driver_past_m: float = 0.     # held: of those, m moved past both its stop point and the furthest asked (_held)
  drivers_frame: bool = False   # held: the last frame was the driver's (the next one the plan applies on is a re-engage)
  done: str = ''              # 'released', 'passed', 'overrun', 'dropped'
  cause: str = ''               # released: why (gap, clear, green, or the car's motion: CROSSED_CAUSES)
  action: str = 'none'
  sign_runs: int = 0            # a stop sign's approach, from SIGN_LOOK_M: forward runs counted (the camera's signs read)...
  sign_hits: int = 0            # ...the stop-sign boxes in them...
  sign_conf: float = 0.         # ...and the best conf (row C11's evidence)
  yield_cap: float = NO_SPEED_CAP  # a yield's lifted cap, the curve down to YIELD_SPEED at its line (its rolling test passing)
  # signals at a route turn (module docstring, _signal_turns), and who a signal's stop is
  turn_id: int | None = None    # the route TURN maneuver whose junction it is, None for none (going straight through it)
  turn_stop: bool = False       # stopping for want of a green at a route turn, no red or yellow read for it this stop
  turn_go: bool = False         # a steady green let it go (TURN_GREEN_STALE_S unread, still stoppable, needs another)
  green_age_t: float = 0.       # s since a green was last read for it, moving or standing (TURN_GREEN_STALE_S)
  green_committed: bool = False  # its turn's green went stale too close to stop for at the yellow rate: committed (logged once)
  accepted: str = 'unknown'     # this frame's light from the accepted sources alone (_light)
  camera: str = 'unknown'       # this frame's camera state for it, the reader acting (add-only), else unknown
  camera_green_t: float = 0.    # s of the camera's green for it (CAMERA_GREEN_STEADY_S)
  camera_added: bool = False    # its stop is TurnPilot's own: a camera red or yellow, or a route turn's want of a green
  accepted_stop: bool = False   # an accepted source read red or yellow for it this stop: only an accepted green ends it
  # a route turn's stop for want of a green the driver's gas went through, lifted engaged too late to stop for (_driver_go):
  # no stop while it stands, provisional until the car is OVERRUN_PAST_M past the line; a brake, the car down to COMMIT_SPEED
  # or a red or yellow read short of the line takes it back
  driver_go: bool = False


@dataclass
class JunctionLine:
  """A mapped signal as read against a route turn's junction, frame by frame over the maneuver (_junction_reads): the
  junction's on most of those frames, never on any one of them (review 2026-11-06: route_follower's junction distance
  jitters 1.4-6.8 m against the tracker's line over one approach on both drives, and the previous junction's signal sat
  34-42 m back; one frame's read 6 m nearer put it in the window, made it the turn's and the junction a mapped signal's for
  the maneuver, and once the car went through it a red on the junction's own head stopped nothing)."""
  n: int = 0             # frames read with the junction
  inside: int = 0        # ...of them in its window (PROTECT_BEFORE_M before it to PROTECT_PAST_M past it)
  marked: bool = False   # the turn's signal (ControlState.turn_id), on any frame
  passed: bool = False   # its line behind the car, or done (passed, released, overrun), on any frame
  seen: bool = False     # in this frame's controls

  @property
  def most(self) -> bool:
    """In the junction's window on most of its frames: the line itself, never one frame's read of the junction."""
    return 2 * self.inside > self.n

  @property
  def governs(self) -> bool:
    """The junction's signal: the turn's (marked, on most frames' evidence, _signal_turns), or in its window on most frames
    (a turn not driven has no mark)."""
    return self.marked or self.most


def description(c: TrackedControl, turn_name: str = 'left') -> str:
  if c.kind == UNPROTECTED:
    return f"unprotected {turn_name}"
  if c.kind == 'stop':
    return "map stop sign (approach unverified)" if c.cross_suspect else "stop sign"
  if c.kind == 'yield':
    return "yield"
  return "signal" if c.junction else "signal (crossing)"


class MapControlPolicy:
  def __init__(self, yellow_decel: float = YELLOW_DECEL):
    self.tracker = ControlTracker()
    self.yellow_decel = yellow_decel
    self.always_drive = False  # TurnPilotAlwaysDrive, handed over every frame (turnpilotd.hand_always_drive; ALWAYS_DRIVE.md section 1)
    self.held: tuple[int, int] | None = None  # the latched control's key
    self.prev_gas = False
    self._alert, self._alert_t = 'none', 0.
    self.turn: TrackedControl | None = None  # the unprotected turn's control (module docstring)
    self.camera_signal: TrackedControl | None = None  # a signal the camera read at the route turn's unmapped junction
    # the last maneuver the unprotected turn's control was made for: its junction's signal is the left's own (turn_light), never
    # a route turn's stop for want of a green (_signal_turns), its hold done or not (after the left's go the junction's line
    # lies just behind the car: a new stop there would hold it past its line)
    self._unprotected_mid: int | None = None
    self._turn_late_log: tuple = ()  # (key, maneuver id) of the route turn's signal last logged too close to mark (_signal_turns)
    # a colourless head at the route turn's unmapped junction (_camera_signal, TURN_HEAD_RUNS): [maneuver id, the road runs it
    # was associated in, the last of them (CameraLights.run), s since it was last associated], None with none
    self._turn_head: list | None = None
    # the route turn's junction's mapped signals as read against it, frame by frame ({maneuver id: {signal key:
    # JunctionLine}}, the last JUNCTION_LINES_KEPT maneuvers; _junction_reads): one that governs it, live or passed, keeps the
    # camera's own line out there (_camera_signal), so no head there makes a second line past the mapped one
    self._junction_lines: dict = {}
    self._red_refused_mid: int | None = None  # the maneuver a camera red was last logged refused at (_red_refused)
    self._refusing: tuple | None = None  # this frame's gate on the camera's own line, its (maneuver id, line, governing, mapped)
    # (maneuver id, m to its junction) as last read, dead-reckoned on a frame without one: the junction's own line for the
    # turn's green (_turn_camera)
    self._junction_d: tuple | None = None
    self._turn_floor: float | None = None  # m, dead-reckoned: its stop point read behind where the car can stop at the floor
    self._turn_raw = 0.  # m to its stop point as last read, before that floor
    self._turn_hold_m = LEFT_LINE_M  # m before its junction the unprotected turn's hold is (left_hold_m)
    self._signal_hold_id: int | None = None  # the maneuver a mapped signal at its junction moved its hold to the mouth for
    self._signal_line: tuple | None = None  # (maneuver id, m before its junction) of the mapped signal's own line there
    self.turn_name = 'left'
    self._protected_id: int | None = None  # the maneuver a stop sign, signal or yieldStop of its junction protected: it stays so
    self._off_cap: float | None = None  # m/s: the off-route cap while the stops ahead are unknown (module docstring)
    self._off_t = 0.  # s into the off-route stretch with the stops ahead unknown (the switch's onset alert, OFF_ROUTE_ALERT_S)
    self._route_limit: float | None = None  # m/s: the last valid navInstruction's speedLimit, None for none (R17)
    self._camera_would: dict = {}  # signal key -> what the camera would have it do, as last logged
    self._camera_unmapped: tuple = ()  # the unmapped camera states last logged
    self._camera_failing = False  # the reader's failing spell: from its disabled to its next good read (its line up all of it)
    self._camera_off_said = False  # this failing spell's onset chimed (CAMERA_OFF_REASON); a good read clears it
    # s of the spell's chimed onset still to say: it runs down only on frames it could be the line shown (no map prompt over
    # it, openpilot engaged: turnpilotControlAlert is a WARNING, never shown disengaged)
    self._camera_off_t = 0.
    self._camera_off_ago = math.inf  # s since it was last said (CAMERA_OFF_AGAIN_S)
    # turnpilotd's word on whether last frame's plan showed that onset (count_shown: hooks.shows_alert, engaged), None where
    # nothing says (this policy alone): then no map prompt over it and openpilot engaged (SF-2, review 2026-11-07)
    self.camera_off_shown: bool | None = None
    self._lights = NO_LIGHTS  # the last update's lights (turn_light)
    self._signal_turn_id: int | None = None  # the unprotected turn a signal at its junction was seen for (turn_light)
    # [maneuver id, s of steady green] of a red or yellow read at the unprotected left's own junction with no signal mapped
    # there: the left's light until a steady green (_left_red_latch, HR-1)
    self._left_red: list | None = None
    self._left_red_read = False         # ...a red or yellow read for it on the last update (turn_light_read)
    self._latch_run: int | None = None  # ...the camera read it last looked at (CameraLights.run)...
    self._latch_run_m = 0.              # ...and m driven since it was made: a carried read's heads dead-reckoned
    self.driven: Driven | None = None  # a line crossed with no go, its take-control up until through (DRIVEN_THROUGH_M)
    self._status_said: set = set()  # under the switch, navd's bad controls statuses said with mapControlsUnavailable's chime (MC-03)
    self._status_last = ''          # ...the last bad status reported
    self._status_chime_t = 0.       # ...s of the chiming alert left

  def all_controls(self) -> list:
    """The route's tracked controls and the camera's signal at an unmapped junction (camera_signal), if there is one."""
    cs = self.camera_signal
    return list(self.tracker.controls.values()) + ([cs] if cs is not None else [])

  def stop_ahead(self) -> float | None:
    """m to the nearest stop sign or signal the car is going to stop at, or None: a stop sign not released, a signal only
    while it is stopping the car (a red, a yellow decided a stop, a route turn's want of a green), never one it goes through,
    the driver's go through a turn's stop included (driver_go: turn_skip would cap the turn from a stop not being made). A
    turn just past one is taken from a stop, never too fast (turn_skip.py), and the lane change window plans around it
    (lane_planner.py)."""
    def stopping(c):
      st = c.state
      if c.kind == 'signal':
        return signal_stopping(st)
      if st is not None and st.done:
        return False
      return c.kind == 'stop' or (c.kind == 'yield' and (st is None or st.mode != YIELD_ROLLING))  # a yield stops unless taken rolling
    ds = [0. if c.held else c.d for c in self.all_controls() if stopping(c) and (c.held or c.d > -LATCH_PAST_M)]
    return min(ds) if ds else None

  def line_setback(self, distance: float) -> float | None:
    """m the mapped stop line of the junction distance m ahead lies back of the unprotected turn's hold line (HOLD_LINE_M +
    MAP_MARGIN_M before the junction), 0 at or short of it; None where the junction has no mapped line (only the junction less
    10 m). Its own controls only (PROTECT_BEFORE_M before it to PROTECT_PAST_M past it), never the cross road's yield or a
    rail orphan's stop. turnpilotd's route-time check (route_range) takes an unprotected left's reach this much past LINE_D
    (BUILD_FINDINGS D4) for its log line alone: no left is skipped or rerouted on it since 2026-11-06. The rule is
    line_setback_at's, which navd's log line of the route's lefts shares (routing/lefts.py)."""
    return line_setback_at(distance, self.tracker.controls.values())

  def left_hold_m(self, maneuver_id: int, distance: float, controls=None) -> float:
    """m before the junction distance m ahead the unprotected turn maneuver_id holds (SIGNAL_LEFT_HOLD_M): in the junction's
    mouth where a signal is mapped there (PROTECT_BEFORE_M before it to PROTECT_PAST_M past it; once seen, the maneuver's for
    good: a mapped signal drops out of the controls 15 m behind the car), else at its line (LEFT_LINE_M). A signal only the
    camera reads there (camera_signal, the left's own red latch) leaves it at the line: there its hold is the junction's only
    stop. controls: this frame's (the tracker's by default)."""
    if self._signal_hold_id == maneuver_id:
      return SIGNAL_LEFT_HOLD_M
    cs = self.tracker.controls.values() if controls is None else controls
    if any(c.kind == 'signal' and c.key[0] != CAMERA_ROUTE and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M
           for c in cs):
      if self._signal_hold_id != maneuver_id:
        cloudlog.event("turnpilot.left_signal_hold", maneuver_id=maneuver_id, distance=round(distance, 1),
                       hold_m=SIGNAL_LEFT_HOLD_M)
      self._signal_hold_id = maneuver_id
      return SIGNAL_LEFT_HOLD_M
    return LEFT_LINE_M

  def signal_line_m(self, maneuver_id: int, distance: float) -> float | None:
    """m before the junction distance m ahead of the unprotected turn maneuver_id that the stop line of the signal mapped
    there lies (its control's own d; PROTECT_BEFORE_M before the junction to PROTECT_PAST_M past it, never the camera's own
    line), kept for the maneuver once seen (the control drops out 15 m behind the car); None with none mapped. The left judges
    the car past that line by it (unprotected_left: the review of 2026-11-08, until then by LINE_D, 9 m, whatever the line)."""
    cs = [c for c in self.tracker.controls.values() if c.kind == 'signal' and c.key[0] != CAMERA_ROUTE and
          distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M]
    if cs:
      self._signal_line = (maneuver_id, distance - min(cs, key=lambda c: abs(distance - c.d)).d)
    return self._signal_line[1] if self._signal_line is not None and self._signal_line[0] == maneuver_id else None

  def turn_engaged(self, maneuver_id: int) -> bool:
    """The unprotected turn of this maneuver is engaged (stopping for it, held at it, or released from it)."""
    tc = self.turn
    return tc is not None and tc.key == (UNPROTECTED_ROUTE, int(maneuver_id)) and tc.engaged

  def junction_lights(self, distance: float) -> tuple[set, set]:
    """(the accepted sources' states, the camera's) read around the junction distance m ahead (PROTECT_BEFORE_M before it to
    CAMERA_ASSOC_PAST_M past it), mapped signal or not. The last update's lights."""
    lo, hi = distance - PROTECT_BEFORE_M, distance + CAMERA_ASSOC_PAST_M
    lights = self._lights
    accepted = {st for st, d in (lights.lights or []) if lo <= d <= hi}
    camera = {st for st, d in lights.camera.heads if lo <= d <= hi} if lights.camera is not None else set()
    return accepted, camera

  def signal_known(self, maneuver_id: int, distance: float) -> bool:
    """A signal is mapped at the junction distance m ahead (PROTECT_BEFORE_M before it to PROTECT_PAST_M past it), or was seen
    there for the unprotected turn maneuver_id (turn_light's latch: it drops out of the controls 15 m behind the car)."""
    return self._signal_turn_id == maneuver_id or \
      any(c.kind == 'signal' and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M for c in self.all_controls())

  def turn_light(self, maneuver_id: int, distance: float) -> str:
    """The light of the signal at the junction of the unprotected turn maneuver_id, distance m ahead, for the autonomous
    left's go (unprotected_left.py: DESIGN G4 go condition 5). A red or yellow from any source around the junction
    (junction_lights: the camera's in shadow too, which only ever keeps the car standing where it is) is that, whether a
    signal is mapped there or not (skeptic G4: OSM missing a signal let the left go into a red its own reader saw). Else
    'none' with no signal there (one from PROTECT_BEFORE_M before it to PROTECT_PAST_M past it, once seen the junction's for
    good: it drops out 15 m behind the car), 'unknown' with navd's controls failing or unsupported (whether there is one isn't
    known); at a signal, green from any source, the camera's included, or 'none' with no colour read (Kellen, 2026-11-07: an
    unread light is no reason to wait; a red or yellow read still holds the left, a green goes). A camera green here lets
    the left release TurnPilot's own latch only: the model's own red-light hold stays, since plannerd's lift for the left's go
    needs an accepted green (release_light, turnpilotd.certify_go). The last update's lights.
    A signal stop of TurnPilot's own in force there (signal_stopping: a red or yellow read raised it, the camera's add-only or
    an accepted source's, and no steady green has released it) is 'red' with nothing read now: a light gone unread
    keeps the stop (module docstring), so the left is held by it (stop_ahead, turnpilotd's other_hold) and says so. Review
    2026-11-07: turn_light gave 'none' once the camera's red was lost, so the left, held by that stop, said "Waiting for a
    gap" at an empty band for as long as it stood (a 150 s Sim run, addOnly, no oracle), logged nothing, and was never going
    on the gap its line promised; now its line is the light's wait (unprotected_left.LIGHT_WAIT_REASON), its light wait said
    once (turnpilot.left_light_wait), gone once a steady green releases the stop. The same predicate and window as
    stop_ahead's, so the left's light and its other hold never disagree. With no signal mapped there, a red or yellow read
    there from an accepted source or the camera acting is 'red' the same once no longer read, until a steady green there
    (_left_red_latch, review 2026-11-07 HR-1), and marks a signal there for the lift (signal_known)."""
    if any(c.kind == 'signal' and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M for c in self.all_controls()):
      self._signal_turn_id = maneuver_id
    accepted, camera = self.junction_lights(distance)
    states = accepted | camera
    stop = next((st for st in ('red', 'yellow') if st in states), None)
    if stop is not None:
      return stop
    in_force = next((c.state for c in self.all_controls() if c.kind == 'signal' and signal_stopping(c.state) and
                     distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M), None)
    if in_force is not None:
      # 'red' for a yellow's stop too: a yellow decided a stop is red by the time its read is lost (st.yellow is never cleared
      # by the red after it), and the left waits for either the same way (LIGHT_WAIT_REASON)
      return 'red'
    if self._left_red is not None and self._left_red[0] == maneuver_id:
      return 'red'  # a red or yellow read at its junction with no signal mapped there, no steady green since (_left_red_latch)
    if self._signal_turn_id != maneuver_id:
      # navd's controls failing or unsupported: a signal there isn't known not to be (no junction data is never clear)
      return 'unknown' if self.tracker.route and self.tracker.status not in ('ok', 'none') else 'none'
    return 'green' if 'green' in states else 'none'

  def turn_light_read(self, maneuver_id: int, distance: float) -> bool:
    """Whether turn_light's red or yellow for the unprotected turn maneuver_id, distance m ahead, is read now: a red or yellow
    from any source around the junction (turn_light's first test), or one its latch read on the last update
    (_left_red_latch). False with turn_light 'red' is a stop of TurnPilot's in force there, or the latch, with nothing read
    now (review 2026-11-07, AL-2: the left says it as a through signal says its light unseen). The last update's lights."""
    accepted, camera = self.junction_lights(distance)
    if (accepted | camera) & {'red', 'yellow'}:
      return True
    return self._left_red_read and self._left_red is not None and self._left_red[0] == maneuver_id

  def _left_red_latch(self, turn, controls, lights: Lights, v: float, dt: float) -> None:
    """The unprotected left's own junction's red, kept once no longer read (turn_light), where no signal is mapped there
    (one mapped keeps its own stop in force: signal_stopping). Review 2026-11-07 (HR-1): there the camera's signal is never
    made (_camera_signal's own_turn gate: the left's hold is the junction's), so a red the camera read there (add-only, the
    car's only source) was forgotten as soon as it stopped being read: lost behind a truck, read 'unknown' (the 11-06 night's
    usual read), or the reader gone disabled, and the left went 2.6-3.5 s later on the oncoming band empty at its own red,
    its go certified for plannerd's lift over the model's own red hold. Now a red or yellow read there from an accepted
    source or the camera acting (never its shadow: that adds no stop anywhere) latches the junction's light red for the
    maneuver, and marks a signal there (_signal_turn_id: signal_known, so plannerd's lift then needs an accepted green,
    turnpilotd.release_blocked); it is let go only by CAMERA_GREEN_STEADY_S of green read there and no red or yellow (an
    accepted source's or the camera's), as a camera stop is (_camera_releases). Only heads the camera's own line at the
    junction (the junction less CONTROL_LINE_M) would take (CAMERA_ASSOC_BEFORE_M short of it to CAMERA_ASSOC_PAST_M past
    it): a light 20 m or more short of the junction (the previous signal, a mid-block crossing) is never its red, a read
    turnpilotd carries between road runs (up to CAMERA_STALE_S) dead-reckoned by the metres driven since its run. turn:
    update()'s (maneuver id, m to its junction, side) of the unprotected turn the car is taking, or None (the latch kept for
    its maneuver)."""
    self._left_red_read = False
    cam = lights.camera
    if cam is not None and cam.run != self._latch_run:
      self._latch_run, self._latch_run_m = cam.run, 0.
    else:
      self._latch_run_m += v * dt
    if not isinstance(turn, tuple):
      return
    mid, dist = int(turn[0]), float(turn[1])
    if self._left_red is not None and self._left_red[0] != mid:
      self._left_red = None
    mapped = [c for c in controls if c.kind == 'signal' and c.key[0] != CAMERA_ROUTE and
              dist - PROTECT_BEFORE_M <= c.d <= dist + PROTECT_PAST_M]
    if mapped and (self._signal_hold_id != mid or any(c.d > 0. for c in mapped)):
      self._left_red = None  # a mapped signal there: its own stop in force is the left's light
      return
    # ...but past that signal's line, a left held in the junction's mouth (left_hold_m: SIGNAL_LEFT_HOLD_M, 2026-11-08) stands
    # where the signal's stop is never made (behind the car) nor in force: a red read there is the left's until a steady
    # green as at a junction with no signal mapped, never forgotten the moment it goes unread (a Sim run: read at 45 s, lost
    # at 55 s, the left gone on the band at 58.5 s through the red no longer read)
    line = dist - CONTROL_LINE_M
    lo, hi = line - CAMERA_ASSOC_BEFORE_M, line + CAMERA_ASSOC_PAST_M
    states = {st for st, d in (lights.lights or []) if lo <= d <= hi}
    if cam is not None and not cam.shadow:
      states |= {st for st, d in cam.heads if lo <= d - self._latch_run_m <= hi}
    if states & {'red', 'yellow'}:
      if self._left_red is None:
        cloudlog.event("turnpilot.left_red_latched", maneuver_id=mid, states=sorted(states), distance=round(dist, 1))
      self._left_red = [mid, 0.]
      self._signal_turn_id = mid
      self._left_red_read = True
    elif self._left_red is not None:
      self._left_red[1] = self._left_red[1] + dt if 'green' in states else 0.
      if self._left_red[1] >= CAMERA_GREEN_STEADY_S - 1e-9:
        cloudlog.event("turnpilot.left_red_released", maneuver_id=mid, green_s=round(self._left_red[1], 2), distance=round(dist, 1))
        self._left_red = None

  def release_light(self, distance: float) -> str:
    """The light of the route's junction distance m ahead for a go plannerd lifts (turnpilotd: turnRelease, skeptic G5; the
    autonomous left's unprotectedLeft, skeptic G5 residual). A red or a yellow from any source around it (junction_lights), the
    camera's in shadow too (here it only ever keeps a go off), mapped signal or not; else 'none' with no signal there
    (PROTECT_BEFORE_M before it to PROTECT_PAST_M past it) and navd's controls ok, 'unknown' with them anything else (whether
    there is one isn't known); at a signal a green only from an accepted source, never the camera's (add-only: the camera
    may only add a stop, and a phantom green would lift the car into a red), else 'unknown'. Latches nothing (turn_light's
    latch is the unprotected left's). The last update's lights."""
    accepted, camera = self.junction_lights(distance)
    stop = next((st for st in ('red', 'yellow') if st in accepted | camera), None)
    if stop is not None:
      return stop
    if not any(c.kind == 'signal' and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M
               for c in self.all_controls()):
      return 'none' if self.tracker.status == 'ok' else 'unknown'
    return 'green' if 'green' in accepted else 'unknown'

  def signal_went(self, distance: float) -> bool:
    """The car went through the signal of the junction distance m ahead (PROTECT_BEFORE_M before it to PROTECT_PAST_M past
    it): committed on its yellow (SIGNAL_COMMITTED: a later red never reverses a commit), or over its line faster than
    COMMIT_SPEED with nothing to stop for (crossed_v; a red crept over it, or a go from standing on it, is no go). turnpilotd:
    a later red never takes back the turn's desire."""
    return any(c.kind == 'signal' and c.state is not None and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M and
               (c.state.mode == SIGNAL_COMMITTED or c.state.crossed_v > COMMIT_SPEED) for c in self.all_controls())

  def held_control(self) -> TrackedControl | None:
    """The control the latch holds the car at, or None: the stop-sign decider's (turnpilotd), keyed on its state object, which
    a reroute's adopt carries to the new route's same control."""
    if self.held is None:
      return None
    if self.turn is not None and self.turn.key == self.held:
      return self.turn
    if self.camera_signal is not None and self.camera_signal.key == self.held:
      return self.camera_signal
    return self.tracker.controls.get(self.held)

  def held_stops(self) -> list:
    """The latch alone, for a faulted plan whose map update failed: a held car stays held."""
    return [Stop(HOLD_ACCEL, 0., True, "Stopped at a map stop", 0., rest=True)] if self.held is not None else []

  def _status_alert(self, dt: float) -> str:
    """navd's controls status not ok, said (MC-03): mapControlsUnavailable, with its chime; under TurnPilotAlwaysDrive the
    chiming alert ABORT_HOLD_S once a drive for each status, and again only when the status gets worse than the last one
    reported (STATUS_RANK), else its silent twin, so its later onsets, after an alert covered it, make no sound."""
    if not self.always_drive:
      return 'mapControlsUnavailable'
    st = self.tracker.status
    if st != self._status_last:
      if st not in self._status_said or STATUS_RANK.get(st, 3) > STATUS_RANK.get(self._status_last, 0):
        self._status_said.add(st)
        self._status_chime_t = ABORT_HOLD_S
      self._status_last = st
    chime = self._status_chime_t > 1e-6
    self._status_chime_t = max(self._status_chime_t - dt, 0.)
    return 'mapControlsUnavailable' if chime else 'mapControlsUnavailableSilent'

  def _transient(self, alert: str) -> None:
    self._alert, self._alert_t = alert, ABORT_HOLD_S

  def _light(self, c: TrackedControl, controls, lights: Lights) -> str:
    """The light state for signal control c: the lights within LIGHT_ASSOC_M of it that have no nearer signal control,
    if they agree."""
    if lights.lights is None:
      return 'unknown'
    signals = [o for o in controls if o.kind == 'signal' and not (o.state is not None and o.state.done)]
    states = {s for s, dist in lights.lights
              if abs(dist - c.d) < LIGHT_ASSOC_M and min(signals, key=lambda o: abs(dist - o.d), default=c) is c}
    return states.pop() if len(states) == 1 else 'unknown'

  def _turn_control(self, turn, controls, v: float, dt: float) -> TrackedControl | None:
    """The unprotected turn's control this frame. turn: (maneuver id, m to its junction, 'left' or 'right') for an
    unprotected_turn() the car is taking (not skipped), None, or KEEP_TURN: not known this frame (turnpilotd faulted
    before it knew the route), the one there carries on, dead-reckoned."""
    tc = self.turn
    if tc is not None and tc.held:
      if isinstance(turn, tuple):
        self._turn_raw = turn[1] - self._turn_hold_m  # kept current: a release must not read the car's moves as a jump
      return tc  # position-free: only its release lets go
    if isinstance(turn, str):
      if tc is not None:
        tc.d -= v * dt
      return tc
    guard = protection(controls, turn[1]) if turn is not None else ''
    if guard == 'stop':
      # once protected, protected for good: a signal mapped well before a wide junction drops out of the controls 15 m
      # behind the car (control_tracker.DROP_PAST_M), still short of the hold line: never a second stop for the same go.
      # A yield latches only once it is a stop (yieldStop): one taken rolling leaves the left unprotected, its hold made
      # from there or said infeasible (cannotStopComfortably), never rolled through with nobody watching the oncoming
      self._protected_id = int(turn[0])
    want = turn is not None and int(turn[0]) != self._protected_id and guard != 'pending'
    key = (UNPROTECTED_ROUTE, int(turn[0])) if turn is not None else None
    if tc is not None and (not want or tc.key != key):
      if tc.engaged and not (tc.state is not None and tc.state.done):
        cloudlog.event("turnpilot.unprotected_turn_dropped", key=list(tc.key), d=round(tc.d, 1), v=round(v, 2),
                       why='protected' if turn is not None and key == tc.key else 'maneuver')
      tc = None
    if tc is None:
      self._turn_floor = None
    if want:
      self._turn_hold_m = self.left_hold_m(int(turn[0]), turn[1], controls)
      d = turn[1] - self._turn_hold_m
      raw, was_raw = d, self._turn_raw - v * dt
      self._turn_raw = raw
      if tc is None:
        tc = TrackedControl(key, UNPROTECTED, d)
      else:
        was = tc.d - v * dt
        if self._turn_floor is not None:
          self._turn_floor -= v * dt
        stop_d = v ** 2 / (2. * -MAP_DECEL_MAX)
        if tc.engaged and tc.state is not None and not tc.state.override and not tc.state.done and raw < was_raw - 2. and \
           raw < stop_d - LATCH_PAST_M:
          # its junction read nearer (route_follower.py: the nearest of navd's readings), its line now where the car,
          # braking at the floor, would stop past the latch's reach: never dropped as an overrun and crossed blind, the
          # car stops as soon as it can at the floor, no harder, and is held there and handed back. Said out loud.
          # Nearer than that the latch holds it as for any stop
          self._turn_floor = max(self._turn_floor or 0., stop_d)
          self._transient('cannotStopComfortably')
          cloudlog.event("turnpilot.unprotected_turn_line_behind", error=True, key=list(tc.key), d=round(d, 1),
                         v=round(v, 2), stop_d=round(stop_d, 1))
        if self._turn_floor is not None:
          d = max(d, self._turn_floor)
        tc.jumped = abs(d - was) > JUMP_M
        tc.d = d
      self.turn_name = turn[2]
    if tc is not None:
      self._unprotected_mid = tc.key[1]
    self.turn = tc
    return tc

  def _camera_signal(self, junction, controls, camera: CameraLights | None, v: float, dt: float) -> TrackedControl | None:
    """The camera's signal at the route turn's junction this frame (module docstring): junction is update()'s (maneuver id,
    m to its junction[, TurnPilot driving the turn]) or None; controls the route's mapped ones and the unprotected turn's.
    Made only with the camera acting (add-only), at a junction with no mapped signal and no unprotected-turn hold of its own,
    from a red (or a yellow a stop is still feasible for) its heads read there, or, for a turn TurnPilot drives, from any
    governing head read there, whatever its colour (a signal's head is there: the turn stops for want of a green,
    _signal_turns), once seen there in TURN_HEAD_RUNS road runs (_head_runs: one run's box is no light) and where a stop at
    its line is still MAP_DECEL_MAX or gentler (said once if not, turnpilot.camera_signal_late); kept until done, dropped
    un-engaged when the maneuver moves on. None at all, a red's and a yellow's neither, where a mapped signal governs the
    junction (_junction_lines, JunctionLine.governs: the turn's, or in its window on most frames of the maneuver, never on
    one frame's read) and is still listed or was passed: a live one takes the junction's red (_turn_camera), and one passed
    was the junction's governing line, gone through (2026-11-06: a red read after a go through a line set back 25 m made a
    line 15 m past it, the car braked at -2.0 into the junction's approach). One lost unpassed (a reroute's controls no
    longer list it) governs nothing. A red refused there with the car short of the junction's own line and no live line
    of the junction's taking it is never silent (turnpilot.camera_red_refused, error, once a maneuver): a signal 30 m or
    less back of the junction, crossed on a go, can't be told from a wide junction's set-back line."""
    if isinstance(junction, tuple):
      self._junction_d = (int(junction[0]), float(junction[1]))
    elif self._junction_d is not None:
      self._junction_d = (self._junction_d[0], self._junction_d[1] - v * dt)
    cs = self.camera_signal
    if cs is not None and cs.state is not None and cs.state.done and not cs.held:
      cs = None  # passed, released or overrun last frame: a new red at the next junction makes a new one
    if cs is not None and not cs.held:
      here = isinstance(junction, tuple) and int(junction[0]) == cs.key[1]
      if here:
        d = float(junction[1]) - CONTROL_LINE_M
        cs.jumped = abs(d - (cs.d - v * dt)) > JUMP_M
        cs.d = d
      else:
        cs.d -= v * dt  # dead-reckoned: a fault's frame, or the route's maneuver moved on
        cs.jumped = False
        if not cs.engaged:
          cloudlog.event("turnpilot.camera_signal_dropped", key=list(cs.key), d=round(cs.d, 1), why='maneuver')
          cs = None
    counted, mid = False, int(junction[0]) if isinstance(junction, tuple) else None
    self._refusing = None
    if cs is None and isinstance(junction, tuple) and camera is not None and not camera.shadow:
      dist = float(junction[1])
      line = dist - CONTROL_LINE_M
      mapped = [c for c in controls if c.kind == 'signal' and not (c.state is not None and c.state.done)]
      own_turn = self.turn is not None and self.turn.key == (UNPROTECTED_ROUTE, mid) and \
        not (self.turn.state is not None and self.turn.state.done)
      # never at a junction a mapped signal governs for this maneuver (_junction_lines: the turn's, or in its window on most
      # frames; listed this frame or passed), a red's and a yellow's line too (2026-11-06): its live mapped line takes the red
      # of any head of the junction's (_turn_camera, _camera_adds), and one the car went through on a go is behind it, the
      # junction its own to clear. Tested on this frame's live signals (near, as before), a red on the far-side head after a
      # go through a line set back 25 m made the camera's line at the junction less 10 m, 15 m past the governing one, and
      # braked the car at -2.0 into the junction's approach (cannotStopComfortably); remembered from any one frame's window
      # (review 2026-11-06), the previous junction's signal 34 m back, read once 6 m nearer, kept a red on the junction's own
      # head from stopping anything. Another maneuver's junction (the route's next turn) is its own
      lines = self._junction_lines.get(mid, {})
      governing = [(k, r) for k, r in lines.items() if r.governs and (r.seen or r.passed)]
      if line > 0. and not own_turn and governing:
        self._refusing = (mid, line, governing, mapped)  # said or not once this frame's marks are made (_red_refused)
      elif line > 0. and not own_turn:
        new = TrackedControl((CAMERA_ROUTE, mid), 'signal', line)
        # only the heads no mapped signal owns: a mapped line set back further than PROTECT_BEFORE_M keeps its own heads
        # (its add-only stop is at its own line, never moved up to the junction's)
        free = [(h, hd) for h, hd in camera.heads if camera_owner(hd, mapped) is None]
        cam = camera_state(new, [new], CameraLights(free, camera.shadow, camera.status))
        # any head of the junction's, unknown too (the 11-06 night: every governing head read unknown), for a turn TurnPilot
        # drives that isn't the unprotected left's: there is a light, and the turn waits for its green (_signal_turns). Once
        # seen in TURN_HEAD_RUNS road runs: one run's box is no light. Never at a junction a mapped signal is the turn's (the
        # gate above): its far-side heads are that line's light (_turn_camera), never a second line in its approach
        seen = driving_turn(junction) and mid != self._unprotected_mid and \
          any(camera_owner(hd, [new]) is new for _, hd in free)
        runs, counted = self._head_runs(mid, camera, seen, dt), True
        # a colourless head makes no signal (Kellen, 2026-11-07: no stop for want of a green, the turn goes with the model):
        # only a red, or a yellow still stoppable, read there
        if cam == 'red' or (cam == 'yellow' and line >= v * T_LAT + v ** 2 / (2. * min(self.yellow_decel, YELLOW_DECEL_MAX))):
          cs = new
          self._turn_head = None
          cloudlog.event("turnpilot.camera_signal_added", key=list(cs.key), camera=cam, d=round(line, 1), v=round(v, 2),
                         turn=False, runs=runs, heads=[(h, round(hd, 1)) for h, hd in camera.heads])
    if cs is None and not counted:
      self._head_runs(mid, camera, False, dt)  # no read acting, or no junction to see a head at: the gap runs on
    self.camera_signal = cs
    return cs

  def _junction_reads(self, junction, controls) -> None:
    """Each mapped signal read against the route turn's junction this frame (update()'s junction; JunctionLine): its
    frames, those in the junction's window, the turn's mark, its line passed, listed now. Read before anything else each
    frame, so a stretch with no camera read never misses one; a frame with no junction reads nothing."""
    if not isinstance(junction, tuple):
      return
    mid, dist = int(junction[0]), float(junction[1])
    lines = self._junction_lines.get(mid)
    if lines is None:
      lines = self._junction_lines[mid] = {}
      while len(self._junction_lines) > JUNCTION_LINES_KEPT:
        del self._junction_lines[next(iter(self._junction_lines))]  # the oldest maneuver's
    for r in lines.values():
      r.seen = False
    for c in controls:
      if c.kind != 'signal':
        continue
      r = lines.setdefault(c.key, JunctionLine())
      st = c.state
      r.n += 1
      r.inside += dist - PROTECT_BEFORE_M <= c.d <= dist + PROTECT_PAST_M
      r.marked = r.marked or (st is not None and st.turn_id == mid)
      r.passed = r.passed or c.d < 0. or (st is not None and bool(st.done))
      r.seen = True

  def _red_refused(self, mid: int, line: float, governing, mapped, controls, camera: CameraLights, v: float) -> None:
    """A red the camera reads at the route turn's junction (on its heads no live mapped signal owns, mapped as
    _camera_signal read them, associated with the junction's own line line m ahead) that the gate keeps from a line of its
    own while no line takes it: the junction's mapped signal (governing) passed, a go through it (the car clears the
    junction). A live one takes it, marked the turn's or not (_turn_camera: since S1, review 2026-11-07, a turn not driven's
    governing line stops for its junction's far-side red too). Read once this frame's marks are made (controls, after
    _signal_turns: a line marked the turn's this frame takes it). Never silent: logged once a maneuver
    (turnpilot.camera_red_refused, error). A signal 30 m or less back of the
    junction is the junction's by its window, a mid-block crossing's or a near junction's one too: past it, a red on the
    junction's own head (the light changed behind the car, or a second light) can't be told apart, and is driven through,
    said here."""
    if self._red_refused_mid == mid:
      return
    # a live line of the junction's the car hasn't gone through on a go takes any red of the junction's (_turn_camera): its
    # stop. The turn's (marked), or one that governs it unmarked (a turn not taken: S1, review 2026-11-07)
    lines = self._junction_lines.get(mid, {})

    def takes(c):
      rec = lines.get(c.key)
      return c.state.turn_id == mid or (c.state.turn_id is None and rec is not None and rec.governs and rec.seen and not rec.passed)
    if any(c.kind == 'signal' and c.state is not None and not c.state.done and takes(c) and
           not (c.state.mode == SIGNAL_COMMITTED or c.state.crossed_v > COMMIT_SPEED) for c in controls):
      return
    new = TrackedControl((CAMERA_ROUTE, mid), 'signal', line)
    free = [(h, hd) for h, hd in camera.heads if camera_owner(hd, mapped) is None]
    if camera_state(new, [new], CameraLights(free, camera.shadow, camera.status)) != 'red':
      return
    self._red_refused_mid = mid
    cloudlog.event("turnpilot.camera_red_refused", error=True, key=list(new.key), d=round(line, 1), v=round(v, 2),
                   why='passed' if any(r.passed for _, r in governing) else 'unmarked',
                   lines=[[list(k), r.inside, r.n, r.marked, r.passed] for k, r in governing],
                   heads=[(h, round(hd, 1)) for h, hd in camera.heads])

  def _head_runs(self, mid: int | None, camera: CameraLights | None, seen: bool, dt: float) -> int:
    """The road runs a colourless head was associated with maneuver mid's junction in (TURN_HEAD_RUNS), this frame's
    included: seen, the read's own run counted once (turnpilotd carries one read between runs). Counted afresh for another
    maneuver, and after TURN_HEAD_GAP_S with no head there (a frame with no read, or no junction, is none). Once a frame."""
    rec = self._turn_head
    if rec is not None and mid is not None and rec[0] != mid:
      rec = None
    if seen:
      if rec is None:
        rec = [mid, 0, None, 0.]
      if camera.run != rec[2]:
        rec[1], rec[2] = rec[1] + 1, camera.run
      rec[3] = 0.
    elif rec is not None:
      rec[3] += dt
      if rec[3] > TURN_HEAD_GAP_S + 1e-9:
        rec = None
    self._turn_head = rec
    return rec[1] if rec is not None else 0

  def _signal_turns(self, junction, controls, v: float) -> None:
    """Mark the signals of the route turn's junction (module docstring: a route turn at a signal): junction is update()'s;
    controls this frame's, the camera's signal among them. A live signal from PROTECT_BEFORE_M before the junction to
    PROTECT_PAST_M past it is the turn's (ControlState.turn_id), for a turn TurnPilot drives (driving_turn) that isn't the
    unprotected left's (_unprotected_mid: its own hold needs a green there already, turn_light). Marked only short of the
    latch's reach past its line (never a new stop in the junction) and where a stop at its line is MAP_DECEL_MAX or gentler:
    one too close (a reroute's right 40 m ahead at speed) is said and left to turn_skip, which skips a turn too fast for it.
    The window picks the first mark only, on a frame the signal is in it and has been on most of the maneuver's frames
    (_junction_lines, JunctionLine.most: review 2026-11-06, one frame's read of the junction 6 m nearer made the previous
    junction's signal 34 m back the turn's): once a signal is the maneuver's it stays the maneuver's while that maneuver is
    driven, wherever its junction is read. Unmarked only when the route's turn moves on (a reroute), is skipped or is the
    unprotected left's, unless held: the latch is position-free. Never on the window alone (review 2026-11-06: route_follower's
    junction distance jitters 1.4-6.8 m against the tracker's line over one approach on both drives, 1d maneuver 3 14.5-21.4
    m; a line 28.5-29.8 m back of its junction flapped out of the window 15 times, each frame out dropping the stop and
    re-seeding the curve, until re-marking was refused as late at 33 m and 11.9 m/s and the car turned through the line at
    13.3 m/s, said only in the log; one re-read 6 m further on dropped a line 26 m back for good). A frame with no junction
    (navd's gap, a fault) leaves the marks as they are: a stop that rises and falls with navd hunts."""
    if not isinstance(junction, tuple):
      return
    mid, dist = int(junction[0]), float(junction[1])
    taking = driving_turn(junction) and mid != self._unprotected_mid
    for c in controls:
      if c.kind != 'signal':
        continue
      st = c.state = c.state or ControlState(mode=SIGNAL_GO)
      if st.done or c.held:
        continue
      # the window this frame, and on most of the maneuver's frames (the camera's own signal, the junction less
      # CONTROL_LINE_M, is in it by its making)
      rec = self._junction_lines.get(mid, {}).get(c.key)
      ours = taking and (st.turn_id == mid or (dist - PROTECT_BEFORE_M <= c.d <= dist + PROTECT_PAST_M and
                                               (rec is None or rec.most)))
      if ours and st.turn_id != mid:
        stopping = st.mode == SIGNAL_STOP
        if not stopping and (c.d <= -LATCH_PAST_M or st.mode == SIGNAL_COMMITTED or st.crossed_v > COMMIT_SPEED):
          continue  # past its line, or gone through it on a yellow: never a stop in the junction
        if not stopping and needed(v, c.d) < MAP_DECEL_MAX - 1e-9:
          # too close to mark: logged, never a refusal (no want-of-green stop to refuse since 2026-11-07: turnpilotd's
          # signal_turn_late skip and handback, which read the refusal, are gone, never a turn skipped or handed back for an
          # unread light). A red read there is still its own stop, as a through signal's
          if self._turn_late_log != (c.key, mid):
            self._turn_late_log = (c.key, mid)
            cloudlog.event("turnpilot.signal_turn_late", key=list(c.key), maneuver=mid, d=round(c.d, 1),
                           v=round(v, 2), needed=round(needed(v, c.d), 2))
          continue
        if st.turn_go and st.green_age_t >= TURN_GREEN_STALE_S - 1e-9:
          # a green read while it was a straight signal, gone stale since (kept only as committed, _signal_reads): never the
          # go of a turn a reroute makes here now, where its stop is still feasible (review 2026-11-06)
          st.turn_go = False
        st.turn_id = mid
        cloudlog.event("turnpilot.signal_turn", key=list(c.key), maneuver=mid, d=round(c.d, 1), junction=round(dist, 1),
                       v=round(v, 2), light=st.light, mode=st.mode)
      elif not ours and st.turn_id is not None:
        why = 'skipped' if not driving_turn(junction) else 'unprotected' if mid == self._unprotected_mid else 'maneuver'
        cloudlog.event("turnpilot.signal_turn_dropped", key=list(c.key), maneuver=st.turn_id, d=round(c.d, 1), v=round(v, 2),
                       why=why)
        st.turn_id = None
        if st.turn_stop:
          self._signal_go(c)  # its stop was only the turn's want of a green: going straight, the signal is the model's

  def _release(self, c: TrackedControl, controls, cause: str, v: float, out: MapDecision) -> None:
    st = c.state
    st.done, st.cause = 'released', cause
    if self.held == c.key:
      self.held, c.held = None, False
      out.released_stopped = out.released_stopped or v < LATCH_SPEED
    for o in controls:
      if c.kind == UNPROTECTED:
        break  # past a turn's hold line the route is on another road: nothing there is the same junction's
      if cause == 'clear':
        break  # the stop-sign decider's go certifies its own line only: a second line (a divided road) has its own (G11)
      if near_rail(c):
        break  # tracks past it: whatever is past them stops the car itself (residual-check-2)
      if o is not c and c.d <= o.d <= c.d + RELEASE_SPAN_M and o.kind not in ('yield', UNPROTECTED) and not near_rail(o):
        o.state = o.state or ControlState()
        o.state.done = 'released'
    cloudlog.event("turnpilot.map_stop_released", kind=c.kind, key=list(c.key), cause=cause, d=round(c.d, 1), v=round(v, 2))

  def _overrun(self, c: TrackedControl, v: float) -> None:
    c.state.done = 'overrun'
    if self.held == c.key:
      self.held, c.held = None, False
    self._transient('stopOverrun')
    cloudlog.event("turnpilot.map_stop_overrun", error=True, kind=c.kind, key=list(c.key), d=round(c.d, 1), v=round(v, 2))
    self._crossed(c, -c.d, 'overrun', v)

  def _crossed(self, c: TrackedControl, past_m: float, cause: str, v: float) -> None:
    """A stop sign's or an unprotected turn's line crossed with no go (its latch let go as the driver's, or overrun):
    never silent (DRIVEN_THROUGH_M, owner decision (b)). A signal's stop is the light's, never this."""
    kind = 'stop' if is_stop(c) else UNPROTECTED if c.kind == UNPROTECTED else None
    if kind is None:
      return
    if kind == 'stop':
      reason = "Yield passed unchecked: take control" if c.kind == 'yield' else "Stop sign passed unchecked: take control"
    else:
      reason = f"Unprotected {self.turn_name} not checked: take control"
    self.driven = Driven(kind, c.key, DRIVEN_THROUGH_M - max(past_m, 0.), DRIVEN_PROMPT[kind], reason, cause)
    cloudlog.event("turnpilot.map_stop_crossed", error=True, kind=c.kind, key=list(c.key), cause=cause, past_m=round(past_m, 1),
                   v=round(v, 2))

  def update(self, nav, updated: bool, valid: bool, v_ego: float, gas_pressed: bool, lights: Lights = NO_LIGHTS,
             dt: float = DT_MDL, turn=None, left: LeftCommand | None = None, release: tuple | None = None,
             creep: tuple | None = None, stop: StopCommand | None = None, signs: tuple | None = None,
             yields: dict | None = None, engaged: bool | None = None, junction: tuple | None = None) -> MapDecision:
    """nav: the latest navInstruction, updated: it arrived this frame, valid: its valid flag; carState's vEgo and
    gasPressed; lights: light_input()'s; turn: (maneuver id, m to its junction, 'left' or 'right') of the route's
    unprotected_turn() the car is taking, or None; left: the unprotected left's command for its latch (None: off); release:
    (control key, cause) to let the latch go this frame (the stop-sign decider's 'clear'); creep: (m past where it latched,
    m/s cap) for a held stop sign's stop point (the decider's roll-up); stop: the decider's StopCommand for a held stop sign
    (None: shadow, as before); signs: (this tick's forward runs, their stop-sign boxes [(bearing, elevation, conf)]) for row
    C11's evidence, None with the camera's signs not read; yields: {a yield's key: (its rolling test passes now, m before its
    stop point to its rolling commit point)} (STOP_SIGNS.md 2.7), a yield not in it failing (no yield certifies this week);
    engaged: openpilot engaged (carControl.enabled), the plan applied: a held car's metres then are TurnPilot's unless under
    the gas (None, not known: every metre is the driver's); junction: (maneuver id, m to its junction[, TurnPilot driving
    the turn]) of the route's TURN maneuver ahead, any side, skipped or not, or None: its signal, mapped or the camera's
    (camera_signal), stops a turn TurnPilot drives for want of a green (_signal_turns; driving_turn: no third field is a turn
    driven), and a red the camera reads there stops the car at a junction the map has no signal for, driven or not."""
    self._engaged = engaged
    self._yields = yields or {}
    self.left = left if left is not None else LeftCommand()
    self.stop_cmd = stop if stop is not None else StopCommand()
    self._creep = creep
    self._lights = lights
    # one release path (module docstring): the autonomous left's go is its turn's release, cause gap
    self._release_req = release
    if self.left.mode == 'auto' and self.left.go and self.turn is not None:
      self._release_req = (self.turn.key, 'gap')
    v = max(v_ego, 0.)
    if updated and valid and nav is not None:  # on the route: off it navd's messages are invalid and say no limit
      limit = float(getattr(nav, 'speedLimit', 0.))
      self._route_limit = limit if limit > 0. else None
    controls = self.tracker.update(nav, updated, valid, v, dt)
    if self.held is not None and all(c.key != self.held for c in controls) and \
       not any(o is not None and o.key == self.held for o in (self.turn, self.camera_signal)):
      # a reroute's same control took over the held one under its own key (control_tracker.py): the latch follows it
      moved = next((c for c in controls if c.held), None)
      cloudlog.event("turnpilot.map_stop_latch_moved", key=list(self.held), to=list(moved.key) if moved else None)
      self.held = moved.key if moved is not None else None
    tc = self._turn_control(turn, controls, v, dt)
    if tc is not None:
      controls = sorted(controls + [tc], key=lambda c: c.d)
    self._left_red_latch(turn, controls, lights, v, dt)
    self._junction_reads(junction, controls)
    cs = self._camera_signal(junction, controls, lights.camera, v, dt)
    if cs is not None:
      controls = sorted(controls + [cs], key=lambda c: c.d)
    self._signal_turns(junction, controls, v)
    if self._refusing is not None:
      self._red_refused(*self._refusing, controls, lights.camera, v)
    gas_lift = self.prev_gas and not gas_pressed
    self.prev_gas = gas_pressed
    out = MapDecision()
    prompts = []
    infeasible = False
    if any(c.engaged for c in self.tracker.lost):
      self._transient('mapControlsUnavailable')
    camera = lights.camera
    self._camera_off_ago += dt
    if camera is not None and camera.status == 'read' and self._camera_failing:
      # the reader read again (disabled clears on its next good read, detector-2 2026-11-06): the spell is over, its line
      # down, and its next failing spell is said again. Only a good read: a retry still in its backoff, a sweep or a wide run
      # reads nothing (camera None), and the spell stays on through it
      self._camera_failing, self._camera_off_t = False, 0.
      if self._camera_off_said:
        cloudlog.event("turnpilot.camera_lights_back", said_s=round(self._camera_off_ago, 1))
      self._camera_off_said = False
    if camera is not None and camera.status == 'disabled':
      self._camera_failing = True
    if self._camera_failing and not self._camera_off_said and self._camera_off_ago >= CAMERA_OFF_AGAIN_S - 1e-9:
      self._camera_off_said, self._camera_off_ago = True, 0.
      self._camera_off_t = ABORT_HOLD_S  # its chimed onset, its own hold, never _transient: it must not overwrite a live one-shot
      cloudlog.event("turnpilot.camera_lights_off", error=True)
    live_signals = [c for c in controls if c.kind == 'signal' and not (c.state is not None and c.state.done)]
    self._signs = signs
    if signs is not None:
      # each forward run counted for the nearest stop sign ahead within SIGN_LOOK_M not yet held (row C11)
      ahead = [c for c in controls if c.kind == 'stop' and not c.held and not (c.state is not None and c.state.done) and
               -LATCH_PAST_M < c.d <= SIGN_LOOK_M]
      if ahead:
        st = min(ahead, key=lambda c: c.d)
        st.state = st.state or ControlState()
        runs, boxes = signs
        st.state.sign_runs += int(runs)
        st.state.sign_hits += len(boxes)
        st.state.sign_conf = max([st.state.sign_conf] + [float(b[2]) for b in boxes])
    for c in controls:
      st = c.state = c.state or ControlState(mode=SIGNAL_GO if c.kind == 'signal' else 'stop')
      if st.done:
        continue
      if c.kind == 'signal':
        st.light = st.accepted = self._light(c, controls, lights)
        acting = camera is not None and not camera.shadow
        st.camera = self._turn_camera(c, live_signals, camera) if acting else 'unknown'
        if acting:
          # its own heads' red or yellow, else the turn's junction's (_turn_camera: a far-side head out of a set-back line's
          # own window is still that line's red, 2026-11-06), never a group's disagreeing read in place of its own red
          own = camera_state(c, live_signals, camera)
          st.light = self._camera_adds(c, st, own if own in ('red', 'yellow') else st.camera, v)
        st.green_t = st.green_t + dt if st.light == 'green' else 0.
        st.camera_green_t = st.camera_green_t + dt if st.camera == 'green' and st.light not in ('red', 'yellow') else 0.
        self._signal_reads(c, st, v, dt)
      if c.held:
        self._held(c, controls, v, gas_pressed, dt, out, prompts)
      elif c.kind == 'yield':
        infeasible = self._yield(c, controls, v, gas_pressed, gas_lift, out, prompts) or infeasible
      else:
        infeasible = self._approach(c, controls, v, gas_pressed, gas_lift, out, prompts) or infeasible

    unknown = self.tracker.off_route_unknown
    if unknown:
      floor, off_reason = OFF_ROUTE_SPEED, OFF_ROUTE_REASON
      if self.always_drive:
        floor, off_reason = min(OFF_ROUTE_DRIVE_SPEED, self._route_limit or OFF_ROUTE_DRIVE_SPEED), OFF_ROUTE_DRIVE_REASON
      if self._off_cap is None:
        self._off_cap, self._off_t = max(v, floor), 0.
        cloudlog.event("turnpilot.off_route_stops_unknown", error=True, status=self.tracker.off_status, v=round(v, 2),
                       floor=round(floor, 2))
      self._off_cap = max(self._off_cap - OFF_ROUTE_DECEL * dt, floor)
      self._off_t += dt
      out.stops.append(Stop(NO_ACCEL_CAP, self._off_cap, False, off_reason, 0.))
    elif self._off_cap is not None:
      self._off_cap = None
      cloudlog.event("turnpilot.off_route_stops_known", route=self.tracker.route, status=self.tracker.off_status)

    # the driver took the car past the autonomous left's own hold after its go let the latch go (unprotected_left: hold_here):
    # the left's crossing, as one of the latch's (owner decision (b)), m past its line as read now
    tc = self.turn
    if self.left.crossed and tc is not None and not tc.held and (self.driven is None or self.driven.key != tc.key):
      self._crossed(tc, -tc.d, self.left.crossed, v)
    # a line crossed with no go: its take-control up until the car is through (DRIVEN_THROUGH_M), engaged or not, ahead of
    # anything but another take-control (a held control's own handback)
    dr = self.driven
    if dr is not None:
      dr.left_m -= v * dt
      if dr.left_m <= 0.:
        cloudlog.event("turnpilot.map_stop_crossed_through", kind=dr.kind, key=list(dr.key), cause=dr.cause)
        self.driven = None
      else:
        if not (prompts and prompts[0] in TAKE_CONTROL_PROMPTS):
          prompts.insert(0, dr.prompt)
          out.driven = dr.reason
    # what the driver sees and hears
    out.prompt = prompts[0] if prompts else 'none'
    # 'none' is a navd that predates controls: nothing to say about a route it never looked at
    status_bad = self.tracker.route and self.tracker.status not in ('ok', 'none')
    if status_bad:
      out.reason = f"Map stops/signals: {self.tracker.status}"
    if unknown:
      out.reason = OFF_ROUTE_DRIVE_REASON if self.always_drive else OFF_ROUTE_REASON
    self._alert_t = max(self._alert_t - dt, 0.)
    if infeasible:
      out.alert = 'cannotStopComfortably'
    elif self._alert_t > 0.:
      out.alert = self._alert
    elif unknown:
      onset = not self.always_drive or self._off_t <= OFF_ROUTE_ALERT_S + 1e-6
      out.alert = 'offRouteStopsUnknown' if onset else 'mapControlsUnavailable'  # the switch's status line after its onset
    elif status_bad:
      out.alert = self._status_alert(dt)
    elif lights.rejected:
      out.alert = 'lightSourceRejected'
    elif self._camera_failing:
      # the light reader's status line, up for the whole failing spell (S2, review 2026-11-07: said once for 2.0 s, lost under
      # a lane line, a prompt or before the driver engaged, and no camera red could stop the car for the rest of the drive with
      # nothing saying so): said only where nothing else of map_controls' is, held back (not lost) behind a take-control,
      # overrun or yellow alert. Its chimed onset (CAMERA_OFF_REASON, ABORT_HOLD_S) runs down only on frames it could be the
      # line shown: no map prompt over it, openpilot engaged (unknown counts as engaged); after it the same line, silent
      # (CAMERA_OFF_STILL_REASON: events.py gives it no sound)
      out.alert = 'lightSourceRejected'
      onset = self._camera_off_t > 0.
      shown = self.camera_off_shown if self.camera_off_shown is not None else out.prompt == 'none' and self._engaged is not False
      if onset and shown:
        self._camera_off_t = max(self._camera_off_t - dt, 0.)
      if not out.reason:
        out.reason = CAMERA_OFF_REASON if onset else CAMERA_OFF_STILL_REASON
        # its words over others' only with no map prompt up, whose line says the plan's reason (turnpilotd)
        out.reader_line = '' if out.prompt != 'none' else 'onset' if onset else 'still'
    live = [c for c in controls if c.state is not None and not c.state.done]
    pick = next((c for c in live if c.held), None) or next((c for c in live if c.engaged), None) or \
      next((c for c in live if c.d > -OVERRUN_PAST_M), None)
    if pick is not None:
      st = pick.state
      out.shown = Shown(pick.key[1] & 0xFFFF, pick.kind, pick.d, st.action, st.light, lights.source, self.tracker.status, pick.line_mapped)
      if not out.reason and pick.kind == 'signal' and st.mode == SIGNAL_GO and st.light == 'green':
        out.reason = "Green (sim oracle)"
    elif self.tracker.route:
      out.shown = Shown(0, 'none', 0., 'none', 'unknown', lights.source, self.tracker.status)
    self._camera_log(live_signals, camera, v)
    return out

  def _turn_camera(self, c, signals, camera: CameraLights) -> str:
    """The camera's state for signal c (ControlState.camera: its green, camera_green_t; and its red or yellow where its own
    heads read neither, _camera_adds): its own heads' (camera_state), or at a route turn's junction the heads of all the
    junction's lines (a mapped line and the junction's own both on the route: the last line before a head owns it, so the
    near line, where the car stops, would see none of them), if they agree. The junction's own line (CONTROL_LINE_M short of
    it, _junction_d) is one of them whether the camera's signal is made there or not: a mapped line set back 26 m has its
    far-side heads 41 m and more past it, out of its own window (CAMERA_ASSOC_PAST_M), and with no second line of the
    camera's past a mapped one that governs the junction (_junction_lines) they would release nothing, and a red on them
    would stop nothing (2026-11-06). A live line that governs the route turn's junction unmarked (a turn TurnPilot isn't
    taking: skipped, going straight on the model, or the unprotected left's; _junction_lines, JunctionLine.governs) groups the
    same way: its far-side heads are its red too, a stop from far out at its own line (S1, review 2026-11-07: a line 26 m back,
    the far head 41 m past it red from 166 m out, and the car went through at 13.4 m/s with nothing said but an error log)."""
    st = c.state
    mid = st.turn_id if st is not None else None
    jd = self._junction_d
    if st is not None and mid is None and jd is not None:
      rec = self._junction_lines.get(jd[0], {}).get(c.key)
      mid = jd[0] if rec is not None and rec.governs and rec.seen and not rec.passed else None
    if mid is None:
      return camera_state(c, signals, camera)
    owners = list(signals)
    group = {id(o) for o in owners if o is c or (o.state is not None and o.state.turn_id == mid)}
    if jd is not None and jd[0] == mid:
      own = TrackedControl((CAMERA_ROUTE, jd[0]), 'signal', jd[1] - CONTROL_LINE_M)
      owners.append(own)
      group.add(id(own))
    states = {s for s, dist in camera.heads if s != 'unknown' and id(camera_owner(dist, owners)) in group}
    return states.pop() if len(states) == 1 else 'unknown'

  def _signal_reads(self, c, st, v: float, dt: float) -> None:
    """Who a signal's stop is, and a route turn's green, from this frame's reads (ControlState): an accepted red or yellow in
    a stop makes it the light's (accepted_stop: only an accepted green ends it, never the camera's); any red or yellow read
    ends a turn's want of a green (turn_stop: the stop is the light's, its stand SIGNAL_UNSEEN_S's); a steady green, accepted
    or the camera's, lets the turn go (turn_go) while a green is read for it. TURN_GREEN_STALE_S with none read, moving or
    standing (green_age_t), the light may have changed unread: the turn needs a green again where a stop for it is still a
    yellow's stop (c.d beyond v*T_LAT + v^2/(2*yellow), and LATCH_NEAR_M; standing, anywhere short of the line), and is
    committed where it isn't, as a yellow there would be (logged once, turnpilot.signal_turn_green_committed)."""
    if st.mode == SIGNAL_STOP and st.accepted in ('red', 'yellow'):
      st.accepted_stop = True
    if st.light in ('red', 'yellow'):
      st.turn_stop = False
    if st.green_t >= GREEN_STEADY_S - 1e-9 or st.camera_green_t >= CAMERA_GREEN_STEADY_S - 1e-9:
      st.turn_go = True
    st.green_age_t = 0. if 'green' in (st.light, st.camera) else st.green_age_t + dt
    if not st.turn_go or st.green_age_t < TURN_GREEN_STALE_S - 1e-9:
      return
    # standing, the car stops where it stands (a queue at the back of a green, a stand for people after a release short of
    # the line): any stale green short of the line needs another. Moving, only where a yellow would still be stopped for
    stop_m = 0. if v < STOP_SPEED else max(v * T_LAT + v ** 2 / (2. * min(self.yellow_decel, YELLOW_DECEL_MAX)), LATCH_NEAR_M)
    if c.d > stop_m:
      st.turn_go = False
      if st.turn_id is not None:
        cloudlog.event("turnpilot.signal_turn_green_stale", key=list(c.key), maneuver=st.turn_id, d=round(c.d, 1), v=round(v, 2),
                       age_s=round(st.green_age_t, 1))
    elif st.turn_id is not None and not st.green_committed:
      st.green_committed = True
      cloudlog.event("turnpilot.signal_turn_green_committed", key=list(c.key), maneuver=st.turn_id, d=round(c.d, 1), v=round(v, 2),
                     age_s=round(st.green_age_t, 1))

  def _camera_releases(self, st) -> bool:
    """A steady camera green (the reader acting) lets this signal's stop go: a stop of TurnPilot's own (camera_added: a camera
    red or yellow, or a route turn's want of a green) with no accepted source's red or yellow in it (accepted_stop), the
    camera's red gone (camera_green_t counts only while no red or yellow is read for it). TurnPilot's latch alone: the
    model's own hold stays in the planner's min(), and plannerd's lift needs an accepted green (release_light)."""
    return st.camera_added and not st.accepted_stop and st.camera_green_t >= CAMERA_GREEN_STEADY_S - 1e-9

  def _camera_adds(self, c, st, cam: str, v: float) -> str:
    """Add-only: signal c's light raised by the camera's (module docstring). Never lowered, and never on a line the car went
    through on a go (crossed faster than COMMIT_SPEED with nothing to stop for, this frame's crossing too: crossed_v,
    signal_went): a red read then (the light changing behind the car, a far-side head read late) is that passed line's, and
    the car clears the junction. Raised there, it re-armed the crossed line's own stop: -2.0 for the frames to
    OVERRUN_PAST_M, then stopOverrun said for a line crossed on its green (2026-11-06). A creep over the line in a queue
    (COMMIT_SPEED or slower) is no go: its red still holds the car where it stands (past_red)."""
    crossed_v = st.crossed_v if st.crossed_v >= 0. else v if c.d < 0. else -1.
    if st.mode == SIGNAL_GO and crossed_v > COMMIT_SPEED:
      return st.light
    if cam == 'red':
      return 'red'
    if cam == 'yellow' and st.light != 'red' and st.mode == SIGNAL_GO and \
       c.d >= v * T_LAT + v ** 2 / (2. * min(self.yellow_decel, YELLOW_DECEL_MAX)):
      return 'yellow'
    return st.light

  def _camera_log(self, signals, camera: CameraLights | None, v: float) -> None:
    """What the camera would have each signal do, logged on a change; it reads nothing back into the policy."""
    keys = set()
    for c in signals:
      keys.add(c.key)
      cam = camera_state(c, signals, camera)
      if cam not in ('red', 'yellow') and camera is not None:
        cam = self._turn_camera(c, signals, camera)  # the turn's junction's heads too, as update() reads them
      st = c.state
      if cam == 'red':
        need = needed(v, c.d)
        would = 'held' if c.held else 'stop' if need >= -MAP_STOP_DECEL else 'stopFirm' if need >= MAP_DECEL_MAX else \
          'cannotStopComfortably'
      elif cam == 'yellow':
        would = 'stop' if c.d >= v * T_LAT + v ** 2 / (2. * min(self.yellow_decel, YELLOW_DECEL_MAX)) else 'proceedOnYellow'
      else:
        would = 'none'
      if self._camera_would.get(c.key, ('unknown', 'none')) != (cam, would):
        self._camera_would[c.key] = (cam, would)
        cloudlog.event("turnpilot.camera_light_shadow", key=list(c.key), d=round(c.d, 1), v=round(v, 2), camera=cam, would=would,
                       acting=camera is not None and not camera.shadow, policy_light=st.light if st else 'unknown',
                       policy_mode=st.mode if st else '', needed=round(needed(v, c.d), 2),
                       heads=[(h, round(dist, 1)) for h, dist in camera.heads] if camera is not None else [])
    for k in [k for k in self._camera_would if k not in keys]:
      del self._camera_would[k]
    unmapped = tuple(sorted({h for h, dist in camera.heads if h in ('red', 'yellow') and camera_owner(dist, signals) is None})) \
      if camera is not None else ()
    if unmapped != self._camera_unmapped:
      self._camera_unmapped = unmapped
      if unmapped:
        cloudlog.event("turnpilot.camera_light_unmapped", states=list(unmapped), v=round(v, 2),
                       heads=[(h, round(dist, 1)) for h, dist in camera.heads])

  def _held(self, c, controls, v, gas_pressed, dt, out, prompts) -> None:
    st = c.state
    st.dwell_t += dt
    st.moved_m += v * dt
    if gas_pressed:
      st.gas_moved_m += v * dt
    # the driver's own frame: openpilot disengaged (the plan not applied) or under the gas. Engaged, the plan is TurnPilot's:
    # the latch's own stop carrying the car on is never the driver's, and nothing is let go on its metres
    drivers_frame = gas_pressed or not getattr(self, '_engaged', None)
    # the re-engage: the first frame the plan applies after the driver's (the lanes test below is judged on it)
    reengage = st.drivers_frame and not drivers_frame
    st.drivers_frame = drivers_frame
    if drivers_frame:
      st.driver_m += v * dt
    what = description(c, self.turn_name)
    cause, prompt = None, None
    left = getattr(self, 'left', LeftCommand())
    req = getattr(self, '_release_req', None)
    stop_cmd = getattr(self, 'stop_cmd', StopCommand())
    if c.kind == UNPROTECTED and left.mode == 'auto':
      # the autonomous left's (unprotected_left.py): held for its go, never handed back but for its fault
      if left.take_control:
        prompt, reason = 'leftUndecidedTakeControl', left.reason or f"Unprotected {self.turn_name}: take control"
      elif req is not None and req[0] == c.key:
        cause = req[1]
        reason = left.reason
      else:
        reason = left.reason or f"Unprotected {self.turn_name}: waiting for a gap"
    elif c.kind == UNPROTECTED:
      # off (G1): handed back at once, held while the driver takes over; announce: the same, saying whether it would go
      prompt = 'unprotectedTurnTakeControl'
      reason = f"{left.reason}: take control" if left.mode == 'announce' and left.reason else \
        f"Unprotected {self.turn_name}: take control"
    elif is_stop(c) and req is not None and req[0] == c.key and stop_cmd.latch == 'wait':
      cause, reason = req[1], stop_cmd.reason or "Going"  # the stop-sign decider's go (cause clear)
    elif is_stop(c) and stop_cmd.latch == 'wait':
      reason = stop_cmd.reason or f"Stopped at {what}"  # the stop-sign decider waits: held, no handback
    elif is_stop(c) and stop_cmd.latch == 'fault':
      prompt, reason = 'stopSignTakeControl', stop_cmd.reason or self._stop_reason(c)  # its fault: handed back now
    elif is_stop(c):
      # as shadow: a full stop, then handed back (module docstring), the decider's reason saying why
      prompt = 'stopSignTakeControl' if st.dwell_t >= STOP_SIGN_HANDBACK_S - 1e-9 else None
      reason = (stop_cmd.reason or self._stop_reason(c)) if prompt is not None else f"Stopped at {what}"
    else:
      if st.green_t >= GREEN_STEADY_S - 1e-9:
        cause = 'green'
      elif self._camera_releases(st):
        cause = 'cameraGreen'  # TurnPilot's own stop, the camera's steady green: its latch only (module docstring)
      reason = RED_PAST_REASON if st.past_red else \
        f"Red light ({self._light_source()}){'' if c.junction else ' (crossing)'}" if st.light == 'red' else \
        TURN_WAITING_REASON if st.turn_stop else f"Stopped at {what}: waiting for green"
      # no light read for it this long (SIGNAL_UNSEEN_S): said once, its line saying so, and held for a green (an accepted
      # source's, or the camera's for a stop of TurnPilot's own); read red again, the light's as before. Never a handback
      # (until 2026-11-07: signalUnseenTakeControl, "Signal not seen: take control") and never let go by the timeout (held
      # rather than released to the model: SIGNAL_UNSEEN_S's comment)
      st.unseen_t = st.unseen_t + dt if st.light not in ('red', 'yellow', 'green') else 0.
      # a camera green being read that may still let this stop go (_camera_releases' own, short of its steady time) defers
      # the line not yet said: it releases at CAMERA_GREEN_STEADY_S, or, broken first, is said that frame (review 2026-11-06:
      # counted as unseen, a green from 4.5 s into a turn's hold said "take control" 0.5 s before its own release let the car
      # pull away). Deferred, never reset: a flickering green never puts it off by more than one green's run, and a green
      # that can't release (an accepted red's stop) defers nothing
      run = st.camera_green_t if st.camera_added and not st.accepted_stop else 0.
      unseen = cause is None and st.unseen_t - run >= SIGNAL_UNSEEN_S - 1e-9
      if unseen:
        if not st.unseen_said:
          cloudlog.event("turnpilot.signal_unseen", error=True, key=list(c.key), d=round(c.d, 1), unseen_s=round(st.unseen_t, 1),
                         turn=st.turn_id, camera=st.camera, accepted=st.accepted_stop, action='held')
          # its onset heard, once (SF-4, review 2026-11-07: the status line alone said it, the car held at a green the reader
          # never read, until the driver's gas): signalUnseen's chime, ABORT_HOLD_S, never a handback
          self._transient('signalUnseen')
        reason = SIGNAL_UNSEEN_REASON
      st.unseen_said = unseen
    # moved where it was asked (a creep, a roll-up) isn't driven off: only past that. The furthest asked this hold is the
    # latch's own: an undecided left's command asks no creep and a decider's fault or shadow no roll-up, yet the car stands
    # where it was moved (skeptic residual-check-1: read live, a 4 m creep then undecided was let go as driverWent)
    asked = left.creep_m if c.kind == UNPROTECTED and left.mode == 'auto' else \
      (getattr(self, '_creep', None) or (0., 0.))[0] if is_stop(c) and stop_cmd.latch == 'wait' else 0.
    st.asked_m = max(st.asked_m, asked)
    moved = st.moved_m - st.asked_m
    # m past both the stop point (c.d, frozen while held, from where it latched) and the furthest TurnPilot asked of the
    # latch this hold: a stop or an unprotected turn is let go by the car's motion only past both (owner decision (a),
    # 2026-10-30; skeptic residual-check-1: a 2.2 m gas nudge inside a queue latch's roll-up let go 2.6 m short of the line,
    # and re-engaged the car ran the stop sign silently). Short of them the latch holds: a re-engage there goes back to the
    # full check (the decider's observe, its clear window, its certified go; a shadow stop's handback), never straight on
    past = st.moved_m - max(c.d, st.asked_m)
    # ...of the driver's own metres (disengaged or under the gas) past both, as they moved it: those inside the ask are
    # TurnPilot's, whoever drove them, and the latch's own stop carrying a car re-engaged rolling is its own (skeptic round 1:
    # min(past, driver_m) took the gas inside a creep, then the latch's stop past it, engaged, for the driver's past it).
    # Capped by past, so a stop point TurnPilot asks on later takes the metres it covers back
    if drivers_frame:
      st.driver_past_m += max(min(v * dt, past), 0.)
    drv = min(past, st.driver_past_m)
    stop_like = is_stop(c) or c.kind == UNPROTECTED
    # past both by the latch's reach at any speed, at a stop or an unprotected turn: a peek-creep at a walking pace is the
    # driver going (skeptic G1: held after a 1.45 m/s creep 3.6 m on, a re-engage stopped the car in the crossing lanes; a
    # gas nudge then a roll on past is theirs too, never a hold in the lanes), and so is an idle creep with no gas, the brake
    # let off (owner decision (i), 2026-10-30: any movement past the stop point TurnPilot didn't ask for is the driver's; srm:
    # held again 3-8 m past the line on the re-engage). Never a prompt being up alone (the car must have moved), never
    # TurnPilot's own metres (asked_m), and never at a signal: its stop is the light's, which a crawl doesn't change
    # (residual-check-1, d)
    # at an unprotected turn, short of the reach too, the driver having moved the car this hold, once it is in the oncoming
    # lanes (ONCOMING_ENTRY_M before its junction, left_hold_m past the stop point as latched) or a hold
    # taken up by a re-engage could no longer stand it short of them (it carries a rolling car on as reengage_stop_m, the
    # latch's own braking): the left's full creep stands the car CREEP_SHORT_M short of them, where the reach (LATCH_PAST_M
    # past it) is 1 m inside them (the invariant: never a TurnPilot hold there). Inside the ask too: the driver's gas inside
    # a creep can leave the car too fast for any hold to stand it short (skeptic round 1)
    room = c.d + self._turn_hold_m - ONCOMING_ENTRY_M
    # the reach and the lanes reached only on the driver's own frame, a car too fast for a hold only at the re-engage, judged
    # from its frame's speed and where the car is before the latch brakes it: never let go engaged on the latch's own
    # metres, which drove a car re-engaged inside the creep on into the junction, the full check (c) skipped (skeptic round
    # 1), and never on the speed of a driver's frame, for good: a car braked to a stand short of the lanes after it is held
    # there on the re-engage, never free to pull away into the junction (skeptic round 2: e93750b held it)
    lanes = c.kind == UNPROTECTED and st.driver_m > 0. and \
      ((drivers_frame and st.moved_m > room) or (reengage and st.moved_m + reengage_stop_m(v, latch_hold_caps) > room))
    drivers = stop_like and ((drivers_frame and drv > LATCH_PAST_M) or lanes)
    if cause is None and drivers:
      # handed back: theirs, said already (driverWent); else lifting must not stop the car in the junction (drivenOff), nor
      # a re-engage after a roll with no gas (movedOff): said either way (Driven)
      cause = 'driverWent' if prompt is not None else 'drivenOff' if st.gas_moved_m > 0. else 'movedOff'
    elif cause is None and moved > DRIVE_OFF_M and v > COMMIT_SPEED and (past > 0. or not stop_like):
      # handed back, the driver took over and went: theirs, said already. Driven off under the driver's foot: lifting must
      # not stop the car in the junction. Or gone with no go at all, the plan not applied (shadow mode turned on): the
      # latch must not stop the car wherever the plan applies again
      if prompt is not None:
        cause = 'driverWent'
      else:
        if not gas_pressed:
          self._transient('stopOverrun')
        cause = 'drivenOff' if gas_pressed else 'movedOff'
    if cause in ('green', 'cameraGreen'):  # a red's stop, or one decided on a yellow, is undone by the next steady green
      self._signal_go(c)
      st.action = 'released'
      self.held, c.held = None, False
      out.released_stopped = out.released_stopped or v < LATCH_SPEED
      cloudlog.event("turnpilot.map_stop_released", kind=c.kind, key=list(c.key), cause=cause, v=round(v, 2))
      return
    if cause is not None:
      st.action = 'released'
      self._release(c, controls, cause, v, out)
      if cause in CROSSED_CAUSES:
        self._crossed(c, st.moved_m - c.d, cause, v)  # no go of TurnPilot's: the crossing is never silent (owner decision (b))
      return
    if prompt is not None:
      prompts.insert(0, prompt)  # the held control's prompt first
    st.action = 'awaitingDriver' if prompt is not None else (stop_cmd.action or 'held') if is_stop(c) and \
      stop_cmd.latch == 'wait' else 'held'
    # the latch's stop point moved on (module docstring): the left's creep for a view, the stop-sign decider's roll-up
    creep_m, creep_v, accel = 0., CREEP_SPEED, CREEP_ACCEL
    if c.kind == UNPROTECTED and left.mode == 'auto' and prompt is None:
      creep_m = left.creep_m
    elif is_stop(c) and stop_cmd.latch == 'wait' and getattr(self, '_creep', None) is not None:
      creep_m, creep_v = self._creep
      accel = CREEP_ACCEL if creep_v <= CREEP_SPEED else NO_ACCEL_CAP  # a roll-up: the map curve alone, no accel cap
    creep = creep_m - st.moved_m
    if creep > CREEP_AT_M:
      # on slowly to where it is asked: still the latch's (position-free), never past its target; a roll-up on the map curve
      ramp = curve(v, creep) if creep_v > CREEP_SPEED else math.sqrt(2. * MAP_STOP_DECEL * max(creep - CREEP_AT_M, 0.))
      out.stops.append(Stop(accel, min(creep_v, ramp), False, reason, creep, rest=True))
      return
    out.stops.append(Stop(HOLD_ACCEL, 0., True, reason, 0., rest=True))

  def _light_source(self) -> str:
    """Who read a signal's light, for its reason: the sim oracle (the one accepted source) when it sends lights, else the
    camera (add-only), which is the only other way a light is ever red or yellow here."""
    return 'sim oracle' if self._lights.lights is not None else 'camera'

  def _stop_reason(self, c) -> str:
    return YIELD_STOP_REASON if c.kind == 'yield' else STOP_SIGN_REASON

  def _yield(self, c, controls, v, gas_pressed, gas_lift, out, prompts) -> bool:
    """A yield (STOP_SIGNS.md 2.7, row C5): a two-way stop taken rolling only while certified at the rolling speed. The approach
    is the map-stop curve toward a full stop at its line; while its rolling test passes (update()'s yields) the cap lifts to
    the curve down to YIELD_SPEED there. At its rolling commit point (YIELD_COMMIT_M, or the yields input's) a test that still
    passes takes it rolling (yieldRolling, passed once by); one that fails makes it a stop (yieldStop) from there: _approach's
    curve, its feasibility checked again (-2.0 at most, said), the latch, and the decider's from then on. Failing before then
    with the car slow enough to latch, it is a stop at once (the same curve either way). A crossSuspect yield (our road
    outranks every crossing one) is dropped only where the sidecar confirms it governs the cross approach (crossGoverned);
    otherwise it is ours. True while it is infeasible."""
    st = c.state
    info = getattr(c, 'junction_info', None) or {}
    if c.cross_suspect and info.get('crossGoverned'):
      st.done = 'dropped'
      cloudlog.event("turnpilot.map_yield_dropped", kind=c.kind, key=list(c.key), reason="crossGoverned", d=round(c.d, 1))
      return False
    if st.mode == YIELD_STOP:
      return self._approach(c, controls, v, gas_pressed, gas_lift, out, prompts)
    if not c.engaged:
      if c.d > engage_distance(v):
        return False
      c.engaged = True
      st.cap, st.yield_cap = curve(v, c.d), NO_SPEED_CAP
      if c.cross_suspect:
        cloudlog.event("turnpilot.map_yield_kept", key=list(c.key), reason="crossSuspect, not crossGoverned", d=round(c.d, 1))
    if c.d < 0.:
      st.done, st.action = 'passed', 'released'
      return False
    rolling, commit_m = getattr(self, '_yields', {}).get(c.key, (False, YIELD_COMMIT_M))
    last, commit_m = yield_last_stop_m(v), yield_commit_m(v, commit_m)
    if st.mode != YIELD_ROLLING:
      if rolling and c.d <= commit_m:
        st.mode = YIELD_ROLLING
        cloudlog.event("turnpilot.map_yield_rolling", key=list(c.key), d=round(c.d, 1), v=round(v, 2))
      elif not rolling and (c.d <= max(last, commit_m) or (v < LATCH_SPEED and c.d < LATCH_NEAR_M)):
        st.mode = YIELD_STOP
        st.cap = min(st.cap, curve(v, c.d)) if v >= LATCH_SPEED else curve(v, c.d)
        self._check(c, v, 'yieldStop')
        cloudlog.event("turnpilot.map_yield_stop", key=list(c.key), d=round(c.d, 1), v=round(v, 2), infeasible=st.infeasible)
        return self._approach(c, controls, v, gas_pressed, gas_lift, out, prompts)
    cv = curve(v, c.d)
    st.cap = cv if v < LATCH_SPEED else min(st.cap, cv)  # the stop curve kept, for a test that fails later
    st.yield_cap = min(st.yield_cap, math.sqrt(YIELD_SPEED ** 2 + 2. * MAP_STOP_DECEL * max(c.d - v * STOP_LAG_S, 0.)))
    taken_rolling = st.mode == YIELD_ROLLING or rolling
    if taken_rolling:
      cap, accel, reason = st.yield_cap, NO_ACCEL_CAP, "Yield ahead"
    else:
      need = needed(v, c.d)
      cap, accel, reason = st.cap, max(need, MAP_DECEL_MAX) if need < -MAP_STOP_DECEL else NO_ACCEL_CAP, "Yield ahead: stopping"
    st.action = 'braking' if cap < v + 0.5 or accel < NO_ACCEL_CAP else 'approaching'
    # on the stop curve to its line it brings the car to rest there (Stop.rest: a turn signal veto then is decided as standing
    # at a stop, car_signals.py), as it does once yieldStop; taken rolling it is a cap, never a rest
    out.stops.append(Stop(accel, cap, False, reason, c.d, rest=not taken_rolling))
    prompts.append('yieldAhead')
    return st.infeasible

  def _approach(self, c, controls, v, gas_pressed, gas_lift, out, prompts) -> bool:
    """A stop sign or signal not held: the curve, the decisions, the latch. True while it is infeasible."""
    st = c.state
    if c.kind == 'signal':
      self._signal_mode(c, v)
      if st.mode != SIGNAL_STOP:  # nothing to stop for: no stop, no cap, no prompt (module docstring)
        st.action = 'proceedingOnYellow' if st.mode == SIGNAL_COMMITTED else 'none'
        if c.d < 0. and st.crossed_v < 0.:
          st.crossed_v = v
        if c.d < -OVERRUN_PAST_M:
          st.done = 'passed'
        return False
    if not c.engaged:
      if c.d > engage_distance(v) or c.d < -OVERRUN_PAST_M:
        return False
      c.engaged = True
      st.cap = curve(v, c.d)
      self._check(c, v, 'engaged')
    elif c.jumped:
      st.cap = curve(v, c.d)
      self._check(c, v, 'jumped')

    if c.kind == 'signal':
      lit = {'red': f"Red light ({self._light_source()})", 'yellow': f"Yellow light ({self._light_source()})"}.get(
        st.light, TURN_STOPPING_REASON if st.turn_stop else "Stopping for the signal")
      reason = lit + ('' if c.junction else ' (crossing)')
      if st.turn_stop:
        # a route turn's stop for want of a green, no red or yellow read for it, under the driver's gas: lifted with openpilot
        # engaged (the plan applied from here) faster than COMMIT_SPEED where a stop at its line would need more than
        # MAP_DECEL_MAX, or past the line, the driver went: the driver's go (driver_go, _driver_go), no stop and no overrun, as
        # driven through disengaged below, since no light was read to run. Lifted short of that, the curve carries on from
        # where the car is (a stop at -2.0 or gentler); a creep at COMMIT_SPEED or slower lifted past the line is held where it
        # stops, as a red crept over (1d 4:51: the driver crept a right 4.6 m over its line under the gas and stood; re-engaged
        # there, the turn would go on nothing read). Review 2026-11-06: at night every signalized turn stops, so drivers press
        # through on a green they see; lifting near the line then braked at the -2.0 floor into the line and said stopOverrun
        # (1e 5:37: the gas lifted 22 m short at 9.6 m/s, -2.1 m/s^2 to stop, openpilot engaged). Lifted disengaged it is no go:
        # the plan isn't applied, the car the driver's, a drive through dropped below and a stand held at the line. A red's or
        # a yellow's stop keeps its overrun
        if gas_pressed:
          st.override = True
        elif st.override and gas_lift:
          st.override = False
          if getattr(self, '_engaged', None) and v > COMMIT_SPEED and needed(v, c.d) < MAP_DECEL_MAX - 1e-9 and not st.driver_go:
            st.driver_go = True
            cloudlog.event("turnpilot.signal_turn_driven", key=list(c.key), maneuver=st.turn_id, d=round(c.d, 1), v=round(v, 2),
                           why='gas', needed=round(needed(v, c.d), 2))
      if st.driver_go and self._driver_go(c, st, v, gas_pressed):
        return False
    else:
      if c.kind == UNPROTECTED:
        reason = f"Unprotected {self.turn_name} ahead"
      elif c.kind == 'yield':
        reason = "Yield ahead: stopping"
      else:
        reason = "Map stop sign, approach unverified" if c.cross_suspect else "Stop sign ahead"
      if gas_pressed:
        st.override = True
      elif st.override and gas_lift:
        st.override = False
        if c.d < -LATCH_PAST_M or (c.d < 0. and v > COMMIT_SPEED):
          # past the line under the driver's foot, beyond where the latch can hold it: the driver went. Stopping the car
          # in the junction with nothing to release it would be worse
          st.action = 'overrun'
          self._overrun(c, v)
          return False
        st.cap = curve(v, c.d)
        self._check(c, v, 'gasLifted')

    if c.kind == 'signal' and c.d < 0. and not gas_pressed and v <= COMMIT_SPEED:
      # a red crept over its line (RED_PAST_REASON): never dropped, the car stopped where it is and held for a steady green
      if not st.past_red:
        cloudlog.event("turnpilot.map_red_past_line", error=True, key=list(c.key), d=round(c.d, 1), v=round(v, 2), light=st.light)
      st.past_red = True
      if v < LATCH_SPEED and self.held is None:
        self.held, c.held = c.key, True
        st.dwell_t, st.moved_m, st.infeasible, st.unseen_t = 0., 0., False, 0.
        st.asked_m, st.gas_moved_m, st.driver_m, st.driver_past_m = 0., 0., 0., 0.
        st.drivers_frame = False
        cloudlog.event("turnpilot.map_stop_held", kind=c.kind, key=list(c.key), d=round(c.d, 1), light=st.light, pastLine=True)
        self._held(c, controls, v, gas_pressed, 0., out, prompts)
        return False
      st.action, st.cap = 'braking', 0.
      out.stops.append(Stop(-MAP_STOP_DECEL, 0., False, RED_PAST_REASON, c.d, rest=True))
      return False
    if c.d < -OVERRUN_PAST_M and not gas_pressed and c.kind == 'signal' and st.turn_stop and not getattr(self, '_engaged', None):
      # the driver took the car through a route turn's stop for want of a green, openpilot disengaged, no red or yellow read
      # for it (1e 5:35: driven through by hand at 10 m/s): theirs, dropped with no overrun alert, as no light was read to
      # run. Engaged, under the gas, the same at the lift (above); with the plan applied and no gas it is an overrun, said as
      # any other
      st.done, st.action = 'passed', 'none'
      cloudlog.event("turnpilot.signal_turn_driven", key=list(c.key), maneuver=st.turn_id, d=round(c.d, 1), v=round(v, 2),
                     why='disengaged')
      return False
    if c.d < -OVERRUN_PAST_M and not gas_pressed:
      # past the line and not held, however slow: never a stop in the junction with no hold and no go to release it
      st.action = 'overrun'
      self._overrun(c, v)
      return False

    cv = curve(v, c.d)
    st.cap = cv if v < LATCH_SPEED else min(st.cap, cv)  # re-seeded only at standstill (a queue)
    need = needed(v, c.d)
    floor = MAP_DECEL_MAX  # a yellow's stop too (YELLOW_DECEL_MAX)
    accel = max(need, floor) if need < -MAP_STOP_DECEL else NO_ACCEL_CAP
    if st.infeasible and need >= floor:
      st.infeasible = False
    if v < LATCH_SPEED and -LATCH_PAST_M < c.d < LATCH_NEAR_M and not gas_pressed and self.held is None:
      self.held, c.held = c.key, True
      st.dwell_t, st.moved_m, st.infeasible, st.unseen_t = 0., 0., False, 0.
      st.asked_m, st.gas_moved_m, st.driver_m, st.driver_past_m = 0., 0., 0., 0.
      st.drivers_frame = False
      cloudlog.event("turnpilot.map_stop_held", kind=c.kind, key=list(c.key), d=round(c.d, 1), light=st.light)
      if c.kind == 'stop' and getattr(self, '_signs', None) is not None:
        self._sign_evidence(c)
      self._held(c, controls, v, gas_pressed, 0., out, prompts)
      return False
    st.action = 'infeasible' if st.infeasible else 'braking' if (st.cap < v + 0.5 or accel < NO_ACCEL_CAP) else 'approaching'
    out.stops.append(Stop(accel, st.cap, False, reason, c.d, rest=True))
    return st.infeasible

  def _driver_go(self, c, st, v: float, gas_pressed: bool) -> bool:
    """The driver's go through a route turn's stop for want of a green (driver_go, _approach): True while it stands, no stop
    this frame. Provisional: never the stop dropped at the lift (review 2026-11-06: dropped there for good, a driver who
    lifted 25 m short at 11 m/s to brake by hand and engaged openpilot at the line was pulled away 46.5 m on nothing read, the
    light's rail gone, a red read later too. Every stop by hand starts with the gas lifted, and a stop at -2.0 is v^2/4 m
    out, 45 m at 30 mph). Short of the line it is taken back by a brake (openpilot disengaged off the gas), the car down to
    COMMIT_SPEED (stopping, not going: the latch then holds it at its line) or a red or yellow read for it (the light's stop
    now, _signal_reads): the curve re-seeded where the car is and checked, the gas lift's own check the go stood in for (a
    re-engage onto a stop too close for -2.0 is said, never braked at the floor in silence; disengaged, turnpilotd's control
    alerts are engaged only), logged (turnpilot.signal_turn_go_revoked). Past the line it is the car's once OVERRUN_PAST_M on
    faster than COMMIT_SPEED (passed, its crossed_v: signal_went, a later red never takes the turn back), never a stop in
    the junction, a red read there too; at COMMIT_SPEED or slower there off the gas it is no go, and the car is held where it
    stops (past_red), as a creep over the line is."""
    engaged = getattr(self, '_engaged', None)
    why = ''
    if c.d >= 0.:
      why = 'light' if not st.turn_stop else 'slow' if v <= COMMIT_SPEED else 'brake' if not engaged and not gas_pressed else ''
    elif v <= COMMIT_SPEED and not gas_pressed:
      why = 'stoodPast'
    if why:
      st.driver_go = False
      if c.d >= 0.:
        st.cap = curve(v, c.d)
        self._check(c, v, 'goRevoked')
      cloudlog.event("turnpilot.signal_turn_go_revoked", key=list(c.key), maneuver=st.turn_id, d=round(c.d, 1), v=round(v, 2),
                     why=why, light=st.light, needed=round(needed(v, c.d), 2))
      return False
    if c.d < -OVERRUN_PAST_M:
      st.done, st.crossed_v = 'passed', v
    st.action = 'none'
    return True

  def _sign_evidence(self, c) -> None:
    """Row C11 at a stop sign's hold: its approach's class-11 shadow evidence logged (the forward runs counted from
    SIGN_LOOK_M, the stop-sign boxes in them, the best conf), and with SIGN_UNSEEN_RUNS or more and no box, said as a mapped
    stop the camera didn't see (a sign missing, turned or hidden, or the map's own error) for the replay gate. The stop
    stands either way (the gentle error) and the decider certifies it as mapped: nothing here acts on a sign."""
    st = c.state
    seen = st.sign_hits >= SIGN_SEEN_HITS
    cloudlog.event("turnpilot.map_stop_signs", key=list(c.key), runs=st.sign_runs, hits=st.sign_hits, conf=round(st.sign_conf, 2),
                   seen=seen, lineMapped=bool(c.line_mapped), crossSuspect=bool(c.cross_suspect))
    if not seen and st.sign_runs >= SIGN_UNSEEN_RUNS:
      cloudlog.event("turnpilot.map_stop_unseen", key=list(c.key), runs=st.sign_runs, hits=st.sign_hits, conf=round(st.sign_conf, 2),
                     junction=bool(c.junction), osmNodeId=int((getattr(c, 'junction_info', None) or {}).get('osmNodeId', 0) or 0))

  def _signal_go(self, c) -> None:
    """A signal back to nothing to stop for (a steady green): its curve gone, re-seeded if it stops the car again."""
    st = c.state
    st.mode, st.yellow, st.infeasible, st.cap, st.past_red = SIGNAL_GO, False, False, NO_SPEED_CAP, False
    st.turn_stop, st.camera_added, st.accepted_stop, st.override, st.driver_go = False, False, False, False, False
    c.engaged = False

  def _signal_mode(self, c, v: float) -> None:
    """A signal's light into its mode (module docstring): a steady green undoes a stop (the camera's only TurnPilot's own,
    _camera_releases), a red going is a stop, a yellow going is decided once, and a route turn's signal going is a stop for
    want of a green (turn_stop) unless a steady green let it go (turn_go) or a green is being read now (decided once it is
    steady or gone). A stop is engaged where it is first read (or at engage_distance), its curve seeded there."""
    st = c.state
    if st.mode == SIGNAL_STOP and (st.green_t >= GREEN_STEADY_S - 1e-9 or self._camera_releases(st)):
      cause = 'green' if st.green_t >= GREEN_STEADY_S - 1e-9 else 'cameraGreen'
      self._signal_go(c)
      cloudlog.event("turnpilot.map_stop_released", kind=c.kind, key=list(c.key), cause=cause, d=round(c.d, 1), v=round(v, 2))
    elif st.mode == SIGNAL_GO and st.light == 'red':
      st.mode, st.yellow = SIGNAL_STOP, False
      st.accepted_stop = st.accepted == 'red'
      st.camera_added = not st.accepted_stop  # raised by the camera (add-only): a steady camera green ends it
      cloudlog.event("turnpilot.map_signal_decided", decision='stop', light=st.light, d=round(c.d, 1), v=round(v, 2),
                     camera=st.camera_added, turn=st.turn_id)
    elif st.mode == SIGNAL_GO and st.light == 'yellow':
      # decided once: a later red never reverses a commit, and only a steady green undoes a stop
      if c.d >= v * T_LAT + v ** 2 / (2. * min(self.yellow_decel, YELLOW_DECEL_MAX)):
        st.mode, st.yellow = SIGNAL_STOP, True
        st.accepted_stop = st.accepted == 'yellow'
        st.camera_added = not st.accepted_stop
        cloudlog.event("turnpilot.map_signal_decided", decision='stop', light=st.light, d=round(c.d, 1), v=round(v, 2),
                       camera=st.camera_added, turn=st.turn_id)
      else:
        st.mode = SIGNAL_COMMITTED
        self._transient('proceedingOnYellow')
        cloudlog.event("turnpilot.map_signal_decided", decision='go', light=st.light, d=round(c.d, 1), v=round(v, 2))
    # A route turn's signal is a through signal's (Kellen, 2026-11-07: test builds act on what they can see, a safety driver
    # supervising): only a red, or a yellow still stoppable, read stops it; with no colour read the turn goes with the model.
    # No stop for want of a green, so no TURN_UNSEEN_S handback

  def _check(self, c, v, when: str) -> None:
    """Can the car stop at -MAP_DECEL_MAX from here? If not it brakes at the floor, alerted (cannotStopComfortably)."""
    floor = MAP_DECEL_MAX
    c.state.infeasible = needed(v, c.d) < floor - 1e-9
    if c.state.infeasible:
      cloudlog.event("turnpilot.map_stop_infeasible", error=True, kind=c.kind, key=list(c.key), when=when, d=round(c.d, 1),
                     v=round(v, 2), needed=round(needed(v, c.d), 2))
