using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct TurnpilotPlan @0x81c2f05a394cf4af {
  # Published by turnpilotd at 20 Hz. Everything the rest of openpilot takes from TurnPilot.
  enum State {
    disabled @0;     # TurnPilotEnabled off
    noRoute @1;      # enabled, no active route
    following @2;    # on route, nothing pending
    approaching @3;  # maneuver ahead, desire not yet sent
    turning @4;      # turn desire sent, maneuver in progress
    laneChange @5;   # nav lane change requested or in progress
    holding @6;      # scene logic is holding the car (pedestrian, oncoming, cross traffic)
    arrived @7;
    unavailable @8;  # TurnPilot isn't acting: the detector hasn't started yet, or it handed back (see handback)
  }
  enum Handback {
    none @0;
    detectorStopped @1;  # the detector produced nothing fresh within its deadline
    modelFellBack @2;    # modeld fell back from the big driving model to the small one
    noDetector @3;       # the detector never produced a result after TurnPilot was turned on
  }
  enum LaneChange {
    none @0;
    left @1;
    right @2;
  }
  enum Fault {
    none @0;
    error @1;           # turnpilotd failed this frame (logged); TurnPilot does nothing until it recovers
    noLongitudinal @2;  # openpilot doesn't control speed on this car, so TurnPilot does nothing
    disengageOnAccelerator @3;  # DisengageOnAccelerator is off: a gas press must disengage (no gas go), so TurnPilot refuses
  }
  enum LaneMismatch {
    none @0;
    turnOnlyGoingStraight @1;  # the car is in a lane that only turns, at an intersection the route goes straight through
    wrongLaneForTurn @2;       # the route turns at the junction ahead and the car's lane doesn't turn that way
    laneUnknown @3;            # the car's lane in the map's lane list can't be confirmed, so the lane planner changes none
  }
  state @0 :State;
  desire @1 :UInt8;               # index into log.Desire; modeld uses it while DesireHelper is idle
  laneChange @2 :LaneChange;      # request for DesireHelper (acts like blinker plus nudge)
  signalPrompt @3 :LaneChange;    # ask the driver to signal (cars where openpilot can't)
  accelCap @4 :Float32;           # m/s^2, upper bound on planner accel; 10 means none
  speedCap @5 :Float32;           # m/s, turn speed cap; 100 means none
  shouldStop @6 :Bool;
  reason @7 :Text;                # one short line for the UI, e.g. "pedestrian in crosswalk"
  maneuverDistance @8 :Float32;   # m to the next maneuver
  maneuverType @9 :Text;          # OSRM/Mapbox maneuver type, e.g. "turn"
  maneuverModifier @10 :Text;     # e.g. "left", "slight right"
  routeValid @11 :Bool;
  shadow @12 :Bool;               # computed but not applied (TurnPilotShadowMode)
  fault @13 :Fault;               # why a disabled plan is disabled while TurnPilot is on; selfdrived shows it
  tooFastToTurn @14 :Bool;        # the next turn is skipped, too fast to make it: no desire, no cap; navd reroutes past it
  laneMismatch @15 :LaneMismatch; # shown and logged; the lane planner should have moved the car earlier
  maneuverAngle @16 :Float32;     # deg, unsigned: from the junction's bearings when maneuverGeometryValid, else nominal
  maneuverGeometryValid @17 :Bool;
  maneuverBend @18 :Bool;         # the road bends and no other road meets it there: the model's, not a turn
  handback @19 :Handback;         # why TurnPilot handed control back; selfdrived soft-disables on it (turnpilot/handback.py)
  laneChangePaintRule @20 :LaneChange;  # laneChange, or signalPrompt where the driver signals, as it would be with
                                        # TurnPilotLanePaintActive on; equal to it when it is on
  # A planned lane change (lane_planner.py): warned of, confirmed by the driver's blinker where openpilot can't signal,
  # and checked every frame until DesireHelper starts it; selfdrived raises turnpilotLaneChangeLeft/Right and
  # turnpilotLaneChangeAborted from these.
  enum LaneChangeAbort {
    none @0;
    lineNotBroken @1;  # the ego line that way no longer reads broken white (solid, yellow, unknown, not read)
    noLane @2;         # the model sees no lane that way, or only beyond a yellow line
    laneParked @3;     # something stopped in that lane ahead: a parking strip
    blindspot @4;      # the car's blind spot signal on that side
    carInLane @5;      # a car in the target lane, by TurnPilot's cameras
    noSignal @6;       # no blinker from the driver in time, or it went off before the change started
    routeChanged @7;   # a new maneuver, or no route
    notNeeded @8;      # already in the lane the maneuver needs
    tooLate @9;        # too close to the maneuver to start
    tooSlow @10;       # below DesireHelper's lane change speed
    timedOut @11;      # DesireHelper waited too long to start (blind spot)
    turnedOff @12;     # TurnPilotLanePaintActive turned off
    badInputs @13;     # carState or modelV2 failing their checks
    stale @14;         # turnpilotPlan stopped arriving (raised by selfdrived, never published)
    handedBack @15;    # TurnPilot handed back (handback): the offer is cancelled with it
    fault @16;         # TurnPilot faulted (fault): the plan goes disabled, and the offer with it
  }
  laneChangeOffer @21 :LaneChange;      # the change warned of, from the warning until it starts or aborts
  laneChangeCountdown @22 :Float32;     # s of the warning left; 0 once the change may go
  laneChangeAbort @23 :LaneChangeAbort; # why the last change was aborted, held about 2 s; none otherwise

  # Map stop control (selfdrive/turnpilot/map_controls.py): stop signs, yields and traffic signals on the route, from
  # navInstruction.controls. The enums mirror log.capnp's NavInstruction.ControlKind / ControlsStatus and
  # TurnpilotLights' State / Source, which this file can't import.
  struct MapControlStatus {
    enum Kind {
      none @0;
      stop @1;
      yield @2;
      signal @3;
      unprotectedTurn @4;  # a far-side turn across oncoming traffic, no stop sign or signal at its junction: stops at
                           # the hold line for the driver's go (map_controls.py)
    }
    enum Action {
      none @0;
      approaching @1;        # engaged: on the braking curve to the stop line
      braking @2;            # the curve bites (or the infeasible floor)
      held @3;               # stopped at the line: the latch holds the car (shouldStop)
      awaitingDriver @4;     # held, and only the driver's gas tap releases it
      released @5;           # released by its rule (a steady green, the driver's go)
      proceedingOnYellow @6; # a yellow too late to stop for: committed, no cap
      overrun @7;            # passed the line moving, never released: dropped (logged, alerted)
      infeasible @8;         # can't stop at -2.0 m/s^2: braking at the floor, alert
      # STOP_SIGNS.md v3 SG4a, the stop-sign decider (stop_go.py) at a held stop sign
      waitingCross @9;       # held: the decider waits for its go conditions (its reason on the plan)
      creeping @10;          # held, its stop point moved on: a roll-up to the stop point (and SG4b's creeps)
      going @11;             # released on the decider's go, under STOP_GO_V_MAX, short of the commit point
      committed @12;         # past the commit point (the band, then in a lane): never braked for a cross car in a lane
    }
    enum LightState {
      unknown @0;
      red @1;
      yellow @2;
      green @3;
    }
    enum LightSource {
      none @0;
      simOracle @1;
      camera @2;
    }
    enum ControlsStatus {
      none @0;
      ok @1;
      walkOrSnap @2;
      partial @3;
      failed @4;
      unsupported @5;
      pending @6;
    }
    id @0 :UInt16;                   # the control's index along its route; (routeCount, id) is its identity
    kind @1 :Kind;
    distance @2 :Float32;            # m to the stop point (filtered, dead-reckoned)
    action @3 :Action;
    lightState @4 :LightState;
    lightSource @5 :LightSource;
    controlsStatus @6 :ControlsStatus;
    lineMapped @7 :Bool;             # the stop line came from mapped data (the sim sidecar), not the junction less 10 m
  }
  enum ControlPrompt {
    none @0;
    stopSignGo @1;   # held at a stop sign past its dwell: check cross traffic, press the accelerator to go
    signalGo @2;     # a signal ahead with the light not checked: press the accelerator to proceed
    yieldAhead @3;   # visual only
    unprotectedTurnGo @4;  # held at an unprotected turn's hold line: check oncoming traffic, press the accelerator to go
    carStoppedGo @5;       # standing engaged with nothing of TurnPilot's holding it, no lead and no map control near: the
                           # model stopped on its own; the driver's gas tap goes, TurnPilot never does (turnpilotd.py)
    # Handbacks (DESIGN phase 4, G1: no gas go anywhere; the four above are no longer sent). The car stays held (by
    # TurnPilot's latch, or the model) while the driver is told loudly to take control; any gas press disengages
    # (DisengageOnAccelerator), and the driver driving off releases the hold
    stopSignTakeControl @6;         # stopped STOP_SIGN_HANDBACK_S at a stop sign: no cross-traffic check, take control
    unprotectedTurnTakeControl @7;  # held at an unprotected turn's hold line: no gap judgement yet, take control
    carStoppedTakeControl @8;       # standing engaged with nothing of TurnPilot's holding it (turnpilotd.py: stopped_unheld)
    stalledTakeControl @9;          # standing in the turn after its go (turnpilotd.py: stalled_in_turn)
    cantStartTakeControl @10;       # not moving NO_MOVE_S after TurnPilot's go from a standstill: the go is withdrawn
                                    # (held standing) and handed back (turnpilotd.py: no_move)
    # The autonomous unprotected left (unprotected_left.py, DESIGN phase 4 G4): no intervention while it can judge the
    # junction, so waiting and creeping are visual only; undecided is a fault's handback
    waitingForGap @11;              # held at the hold line until the oncoming band is certified clear (visual only)
    creepingForView @12;            # creeping past the line for a view an occluder blocks (visual only)
    leftGo @13;                     # going: a short chime
    leftUndecidedTakeControl @14;   # can't judge the oncoming traffic (stale camera, position unknown, beyond the detector's
                                    # range, yawed, view blocked after the full creep): held, take control
    # STOP_SIGNS.md v3 SG4a: the stop-sign decider's wait and go. stopSignTakeControl @6 is now its faults only (H1-H8 and the
    # ladder's end, the reason line saying which); under TurnPilotStopSignGo shadow or off it is the 1 s handback, as before
    waitingForCross @15;            # held at a stop sign for the go conditions, the reason saying for what (visual only;
                                    # stopGo.chime: the wait's info chime at WAIT_INFO_S and every WAIT_INFO_S after)
    stopGo @16;                     # going from the stop sign: a short chime
  }
  enum ControlAlert {
    none @0;
    cannotStopComfortably @1;   # the stop needs more than -2.0 m/s^2: braking at -2.0, take control
    stopOverrun @2;             # passed a stop line moving, never released
    proceedingOnYellow @3;      # a yellow too late to stop for
    goingOnRed @4;              # the driver released a red-light hold with the gas
    mapControlsUnavailable @5;  # navInstruction.controlsStatus isn't ok: stops and signals may be missing
    lightSourceRejected @6;     # a light-state source that isn't accepted here (the sim oracle outside the sim)
    junctionUncertain @7;       # navd's readings of a committed unprotected turn's junction disagree by more than 15 m:
                                # its hold line is at the nearest of them, so the car may stop early (route_follower.py)
    offRouteStopsUnknown @8;    # off the route, the map stops ahead of the car couldn't be looked up (navd's
                                # offRouteControlsStatus not ok): the speed is capped, take control or wait for the reroute
    hardBrake @9;               # braking harder than -2.0 m/s^2 for a confirmed in-path pedestrian or oncoming vehicle
                                # (scene_policy.py), while it lasts; logged with its evidence (turnpilot.hard_brake)
    oncomingHolding @10;        # an unprotected left's go withdrawn before its commit for a car in the oncoming band:
                                # stopping short of the oncoming lanes on the comfortable curve, waiting again
    farViewOff @11;             # the far crop went off for the drive (failed or over budget): no unprotected left needing it
    sideCamerasOff @12;         # the side views went off for the drive (failed or overran, turnpilotDetections.sideViewStatus
                                # disabled): stop sweeps run the full wide pass and the road camera only (STOP_SIGNS.md 1.3)
    # STOP_SIGNS.md v3 SG4a, the stop-sign decider
    crossTrafficAfterCommit @13; # a cross vehicle met past the in-lane point: said, never braked for (stopping in a lane is worse)
    cameraPaused @14;           # a view the decider needs hasn't run for DET_FAULT_S: held, waiting FAULT_GRACE_S for it (H1 after)
    stalledInJunction @15;      # standing past the in-lane point: a clearBox go (up to CLEAR_BOX_GOES), then H5
    overrideWriteFailed @16;    # a TurnPilotJunctionOverrides entry (a signal head seen at a stop-sign junction) couldn't be
                                # written: navd may route through that junction again; logged with the entry (STOP_SIGNS.md O16)
    # STOP_SIGNS.md v3 SG4c
    otherTurn @17;              # the stop-sign ladder's other turn (7.1 rung 2, O12): the route's maneuver can't be taken here,
                                # another that certifies is ("Turning right instead", stopGo.maneuver), and navd reroutes after it
  }
  mapControl @24 :MapControlStatus;
  controlPrompt @25 :ControlPrompt;
  controlAlert @26 :ControlAlert;   # held about 2 s (ABORT_HOLD_S), like laneChangeAbort; permanent ones while they last

  # Why a lane change the route wants isn't offered now (lane_planner.py), every frame, so a missing offer is explained
  enum LaneChangeWait {
    none @0;           # no change wanted (or no maneuver), or one is offered or under way
    paintRuleOff @1;   # TurnPilotLanePaintActive off: no changes planned
    inputsInvalid @2;  # carState or modelV2 failing their checks
    laneUnknown @3;    # the car's lane in the map's lane list can't be confirmed
    beforeWindow @4;   # the maneuver is further than the window's start
    pastWindow @5;     # closer than the window's end
    tooSlow @6;        # below DesireHelper's lane change speed
    cooldown @7;       # just after a lane change or an abort
    lineNotBroken @8;  # the ego line that way isn't read broken white
    noLane @9;         # no travel lane that way
    laneParked @10;    # something stopped in that lane ahead
    blindspot @11;
    carInLane @12;
    confirming @13;    # the same change wanted for under LC_CONFIRM_FRAMES yet
    afterStop @14;     # a map stop in the window, no room at speed before it: the change goes after its go
    unreachable @15;   # a map stop in the window, no room before it nor after its go: the lane can't be reached
  }
  laneChangeWait @27 :LaneChangeWait;
  laneChangeWant @28 :Int8;           # lane changes the route wants before the maneuver, positive left, at any speed
  turnMissed @29 :Bool;               # a turn the car pulled away into from a standstill went past its junction turned under
                                      # 15 deg: missed, held about 3 s; selfdrived chimes (turnpilotTooFastToTurn, "Turn
                                      # Missed") and navd reroutes at once (turnpilotd.py)

  # The camera modeld's detector runs on (detector/modeld_detector.py: choose_camera), read through fresh_plan, so a shadow
  # plan asks too
  enum DetectorView {
    auto @0;         # modeld's choice: the wide camera in a turn or on a curve, else the road camera
    road @1;         # the road camera only: an unprotected left from its go to its commit (CRITIQUE REQUIRED 3)
    roadAndWide @2;  # the road camera, the wide camera one run in WIDE_EVERY: held at an unprotected left's line (its exit
                     # crosswalk)
    # The stop sweeps (STOP_SIGNS.md section 1.3): the detector's even steps run the forward views, its odd steps the side
    # views (only at or under SIDE_VIEW_V_MAX); lane paint and the far crop are off, lights read only near a mapped signal
    stopSweep @3;    # held at a stop or yield: full wide, side left, road, side right
    stopSweepGo @4;  # crossing, from the commit point: full wide, side left, full wide, side right
  }
  detectorView @30 :DetectorView;
  # The autonomous unprotected left's decision (unprotected_left.py), for the logs and the sim's scorer
  struct UnprotectedLeftStatus {
    enum State {
      off @0;        # TurnPilotAutoLeft off, or no unprotected left in hand
      approach @1;
      waitGap @2;
      creep @3;
      go @4;
      committed @5;
      undecided @6;  # a fault: handed back (leftUndecidedTakeControl)
      announce @7;   # TurnPilotAutoLeft announce: held and handed back, saying whether it would go
      skipped @8;    # its R_req past the detector's proven range: skipped at route time (tooFastToTurn), navd reroutes
    }
    state @0 :State;
    tReq @1 :Float32;          # s the oncoming lanes must stay clear
    rReq @2 :Float32;          # m up them that must be seen clear
    vLimit @3 :Float32;        # m/s, the opposing road's limit it took (mapped, or the 45 mph default)
    rangeM @4 :Float32;        # the detector's proven range rReq is checked against (TurnPilotLeftRangeM)
    clearS @5 :Float32;        # s the band has been clear on road camera runs
    blockers @6 :UInt8;        # raw boxes in the band on the last road run
    viewBlocked @7 :Bool;      # an occluder hides the oncoming lanes
    crosswalkClear @8 :Bool;   # the exit crosswalk and the junction box
    creepM @9 :Float32;        # m past its line the hold may creep to
    why @10 :Text;             # undecided: its fault; announce: go or wait
    farRangeM @11 :Float32;    # the far crop's proven range (TurnPilotLeftFarRangeM), 0 when unmeasured or the crop is off
    farNeeded @12 :Bool;       # rReq is past rangeM: the band out to rReq is judged on the far crop's runs too
    farBlockers @13 :UInt8;    # raw boxes in the band on the last far crop run
  }
  unprotectedLeft @31 :UnprotectedLeftStatus;
  # TurnPilot's certified go (selfdrive/turnpilot/go_override.py; DESIGN phase 4 decision 1, G5): asserted on every frame of
  # the go, from the go until it ends (GO_WINDOW_S at most) or is withdrawn. plannerd lifts the e2e model's hold on the car
  # for it, bounded: +0.5 m/s^2 at most, 3 s at most, under 3 m/s, only while nothing else holds the car. One go at a time,
  # each a new id; clearing it (cause none) ends the lift that frame
  struct GoCertified {
    enum Cause {
      none @0;
      turnRelease @1;      # a turn TurnPilot's hold let go of at a standstill (its walker cleared, its light went green)
      unprotectedLeft @2;  # the autonomous unprotected left's gap certified (unprotected_left.py)
      stopSign @3;         # a stop sign's cross traffic certified clear from the line, also a yield that became a stop
      rollUp @4;           # rolling up to the stop line after a queue
      creep @5;            # one creep step
      clearBox @6;         # stalled past the in-lane point after the commit
    }
    cause @0 :Cause;
    id @1 :UInt32;     # a new one for every go (never repeated in a drive): plannerd gives each go its own window
    t @2 :Float32;     # s since the go
  }
  goCertified @32 :GoCertified;
  # The far crop's aim (DESIGN phase 4 G5, detector/modeld_detector.py): deg left of the calibrated forward axis, read by
  # modeld through fresh_plan (shadow too). 0 is straight ahead; an unprotected left aims it up the oncoming approach
  farCropBearing @33 :Float32;

  # STOP_SIGNS.md v3 SG4a: the stop-sign decider (selfdrive/turnpilot/stop_go.py) at the stop control in hand, every frame; what
  # step S4a's launch_verify.py reads per go (goT, takeoverS, retries, withdrawn), and the runtime inputs navd's certifier reads
  # (sideViewsLive, mapsValid, mode)
  struct StopGoStatus {
    enum State {
      off @0;        # no stop control in hand
      approach @1;   # an engaged stop sign within STOP_SWEEP_M, not yet held (a queue's roll-up goes from here)
      stopped @2;    # held at its line: the full stop's dwell (STOP_DWELL_S)
      rollUp @3;     # held short of its stop point: its stop point moved on to it (map_controls' creep path)
      observe @4;    # the go conditions (section 2.4) every frame; CLEAR_S of them and our turn is the go
      going @5;      # released: the launch under STOP_GO_V_MAX, short of the commit point, every object re-checked
      committed @6;  # the band: past the commit point, short of the in-lane point
      inLane @7;     # past the in-lane point: carried on, never braked for a cross car
      done @8;       # its rear bumper past the farthest conflicting lane (or 60 deg turned)
      ladder @9;     # a handback (H1-H8, the ladder's end): held, stopSignTakeControl with the reason
    }
    enum Ledger {
      none @0;
      dynamic @1;    # traffic taking turns, people moving, a queue: no bound, an info chime at WAIT_INFO_S
      static @2;     # nothing moving that would end it: STATIC_WAIT_S, then the ladder
      fault @3;      # a fault's grace or its handback
    }
    enum Mode {
      off @0;
      shadow @1;
      trial @2;
      auto @3;
    }
    struct Lane {
      role @0 :Text;       # left, right, opposite (where its traffic comes from)
      index @1 :UInt8;     # 1 nearest
      approach @2 :UInt8;  # its approach's index in the control's junctionInfo.approaches
      rSeen @3 :Float32;   # m seen down it from our conflict point, less v_c T_LAT (at the go position once going)
      rReq @4 :Float32;    # m it needs (all-way or two-way)
      binding @5 :Text;    # what ended its reach walk (stop_rules.reach_walk_why), or why it isn't walked
      shielded @6 :Bool;   # a vehicle stopped at its line satisfies it (section 2.3)
    }
    state @0 :State;
    reason @1 :Text;          # what it waits for, or why it goes, or its fault
    ledger @2 :Ledger;
    wouldGo @3 :Bool;         # every go condition held CLEAR_S and our turn: what auto would do (logged in shadow)
    maneuver @4 :Text;        # straight, left, right or unknown (off the route)
    lanes @5 :List(Lane);
    arrivalRank @6 :UInt8;    # earlier arrivals still to go before us
    dEdgeMin @7 :Float32;     # m from the stop point to the closest plausible edge of the first conflicting lane
    dEdgeMax @8 :Float32;     # ...and to its farthest plausible edge
    committed @9 :Bool;
    inLane @10 :Bool;
    retries @11 :UInt8;       # goes withdrawn this encounter
    creptM @12 :Float32;      # m the car moved on from where it latched (a roll-up)
    sideViewsLive @13 :Bool;  # the side views run (turnpilotDetections.sideViewStatus running or noRoom, fresh)
    mapsValid @14 :Bool;      # the side views' maps match the calibration (sideViewStatus not mapsStale)
    mode @15 :Mode;           # TurnPilotStopSignGo as acted on: auto without its data acts as shadow (H4, H8)
    clearS @16 :Float32;      # s the go conditions have held
    goT @17 :Float32;         # s since the decider's go, 0 with none
    takeoverS @18 :Float32;   # s from the go to the model's takeover (go_override.model_takes_over held TAKEOVER_S), -1 not yet
    withdrawn @19 :Text;      # why the last go was withdrawn ('' none)
    positionM @20 :Float32;   # m the car's bumper is past the stop point (negative: short of it)
    chime @21 :Bool;          # the dynamic wait's info chime is due (with waitingForCross)
    fault @22 :Text;          # the fault in hand (H1-H8), '' none
    # STOP_SIGNS.md v3 SG4c: the ladder's other turn (7.1 rung 2, O12): maneuver is a turn the decider synthesized from the
    # junction's approaches in place of the route's, not the route's own; the route's planned turn is published missed once
    # the encounter is done (turnpilotPlan.turnMissed), and navd reroutes
    alternate @23 :Bool;
  }
  stopGo @34 :StopGoStatus;
}

struct TurnpilotDetections @0xaedffd8f31e7b55d {
  # Raw YOLO26 detections on the full camera frame, published by modeld when the detector runs.
  enum Camera {
    road @0;
    wide @1;
  }
  struct Detection {
    cls @0 :UInt8;      # COCO class id (0 person, 1 bicycle, 2 car, 3 motorcycle, 5 bus, 7 truck); traffic lights (9) go in lights
    conf @1 :Float32;
    x0 @2 :Float32;     # pixels in the full sensor frame
    y0 @3 :Float32;
    x1 @4 :Float32;
    y1 @5 :Float32;
  }
  struct LanePaint {
    # What one of modelV2's lane lines is painted as, read off the road camera (selfdrive/turnpilot/scene/lane_paint.py)
    enum Kind {
      unknown @0;  # not readable: out of view, hidden, low line probability, uncalibrated, too few samples
      none @1;     # readable road, no paint
      broken @2;   # dashes or Botts' dots
      solid @3;
    }
    enum Color {
      unknown @0;
      white @1;
      yellow @2;
    }
    kind @0 :Kind;
    color @1 :Color;
    confidence @2 :Float32;  # 0-1
    coverage @3 :Float32;    # share of the sampled road with paint, over the last 1.5 s
  }
  frameId @0 :UInt32;
  timestampEof @1 :UInt64;
  camera @2 :Camera;
  executionTime @3 :Float32;   # s
  detections @4 :List(Detection);
  lanePaint @5 :List(LanePaint);  # on the road camera's frame of this step, aligned with modelV2.laneLines; empty when not read
  enum LanePaintStatus {
    unset @0;        # a log from before the field
    read @1;
    wideCamera @2;   # this run was on the wide camera; the paint is only read on the road camera
    noModel @3;      # no modelV2 for this very frame yet, or no device type to pick the lens
    disabled @4;     # the paint failed earlier and is off for the drive, or modeld has no road camera
    sweep @5;        # a stop sweep runs (turnpilotPlan.detectorView): no paint read
  }
  lanePaintStatus @6 :LanePaintStatus;
  lanePaintTime @7 :Float32;      # s spent reading the paint in this step, 0 when not read; executionTime is the detector's alone
  # Traffic lights read on the road camera's frame of this step (detector/light_state.py), source camera; shadow while
  # TurnPilotCameraLights is shadow: turnpilotd logs what it would do and never acts on it
  lights @8 :TurnpilotLights;
  enum LightStatus {
    unset @0;        # a log from before the field
    read @1;
    wideCamera @2;   # this run was on the wide camera; lights are only read on the road camera
    noCalibration @3;  # no calibration yet, so no horizon to gate the boxes by: nothing read
    disabled @4;     # the reader failed or ran over its budget earlier and is off for the drive
    off @5;          # TurnPilotCameraLights off, or modeld has no road camera
    sweep @6;        # a stop sweep runs with no signal mapped within SWEEP_SIGNAL_M (60 m): no colour read (lightBoxes are)
  }
  lightStatus @9 :LightStatus;
  lightTime @10 :Float32;         # s spent reading the lights in this step, 0 when not read
  # DESIGN phase 4 G5, the far crop (detector/graph.py Crop): a native-resolution window of the road camera's frame, run on
  # the model frames between the full pass's runs and published apart (turnpilotFarDetections), its run a TurnpilotDetections
  # of the road camera with view far: boxes in the road camera's full-frame pixels, so they range with its lens
  enum View {
    full @0;         # the whole camera frame, letterboxed (turnpilotDetections); with camera road it is the sweep's road slot
    far @1;          # the far crop (turnpilotFarDetections.run)
    # STOP_SIGNS.md section 1.3, the stop sweep's side views (detector/sideview.py): a level pinhole at yaw 75 deg left or
    # right of the calibrated forward axis, f 904 px, 768x512, gathered from the wide frame; camera wide, boxes mapped back
    # to the wide camera's fisheye pixels
    sideLeft @2;
    sideRight @3;
  }
  view @11 :View;
  enum FarStatus {
    unset @0;          # a log from before the field
    read @1;           # the far crop ran within FAR_STATUS_S
    off @2;            # TurnPilotFarCrop off, no far pkl for this device, or no road camera
    disabled @3;       # it failed, or ran over its budget or the step's: off for the drive (logged, said once)
    noCalibration @4;  # no calibrated horizon to aim it by yet
    noRoom @5;         # on, but no frame between the full runs had room for it within FAR_STATUS_S
    sweep @6;          # on, suspended while a stop sweep runs (its frames are the side views')
  }
  farStatus @12 :FarStatus;  # the full pass's messages say what the far crop is doing
  # STOP_SIGNS.md SG3a. Stop signs (COCO 11), decoded apart like the lights and never into detections: in shadow
  # (TurnPilotCameraStopSigns), read by no stop, cap or release; the luma below each, for labelling all-way plaques
  struct Sign {
    box @0 :Detection;             # full-frame pixels of this run's camera
    plaque @1 :Data;               # luma (0-255, the camera's own range), row-major, plaqueW x plaqueH, subsampled
    plaqueW @2 :UInt16;
    plaqueH @3 :UInt16;
    plaqueRect @4 :List(Float32);  # x0, y0, x1, y1 of the plaque crop in full-frame pixels
  }
  signs @13 :List(Sign);
  # Image quality per 5 deg of bearing on a side view's horizon band (STOP_SIGNS.md section 1.3; detector/sideview.py
  # SectorQuality), on that view's messages: anything but ok has zero reach
  struct Sector {
    enum Quality {
      unknown @0;
      ok @1;
      degraded @2;   # contrast ratio under 0.5 of the full wide frame's 10-35 deg band
      washed @3;     # 5th-percentile lift over 50 DN with contrast under 0.35, or 30% of pixels saturated
      dark @4;       # median luma under the dark threshold
      frozen @5;     # no frame-to-frame change over the noise floor for 1.0 s
      blocked @6;    # half its pixels static and dark
    }
    inner @0 :Float32;   # deg left of the calibrated forward axis of its edge nearer the axis (a right one's negative)
    outer @1 :Float32;   # ...and of its edge farther out
    quality @2 :Quality;
  }
  sectorQuality @14 :List(Sector);
  # What the side views are doing, on every message: the stop-sign decider counts a side view's freshness only while the
  # gate runs them (running, noRoom)
  enum SideViewStatus {
    unset @0;          # a log from before the field
    off @1;            # no side-view pkl for this device, or no wide camera
    idle @2;           # no stop sweep asked for
    running @3;        # a sweep at or under SIDE_VIEW_V_MAX: the odd steps alternate the side views
    speedGated @4;     # a sweep above SIDE_VIEW_V_MAX: the odd steps idle, no side view
    noCalibration @5;  # no calibration to map the views by yet
    mapsStale @6;      # the calibration moved more than 0.3 deg from the maps': invalid until they are recomputed
    noRoom @7;         # running, but no side view had room within SIDE_STATUS_S
    disabled @8;       # failed or overran: off for the drive (sideCamerasOff, said once)
  }
  sideViewStatus @15 :SideViewStatus;
  lightBoxes @16 :List(Detection);  # every forward run's traffic light boxes (COCO 9), read or not: a signal head at a junction
  lens @17 :Text;                   # the wide lens a side view was mapped with (ranging.Lens.spec); empty on forward runs
  sectorTime @18 :Float32;          # s spent on sectorQuality in this step
}

struct TurnpilotScene @0xf35cc4560bbf6ec2 {
  # Tracked, ranged objects, published by turnpilotd's scene module.
  enum ObjectClass {
    pedestrian @0;
    cyclist @1;
    car @2;
    truck @3;
    bus @4;
    motorcycle @5;
  }
  struct TrackedObject {
    trackId @0 :UInt32;
    cls @1 :ObjectClass;
    conf @2 :Float32;
    x @3 :Float32;      # m forward, calibrated (road) frame
    y @4 :Float32;      # m left
    vx @5 :Float32;     # m/s
    vy @6 :Float32;
    age @7 :UInt32;     # frames tracked
    inPath @8 :Bool;    # inside the planned driving corridor
    camera @9 :TurnpilotDetections.Camera;
    box @10 :List(Float32);  # x0, y0, x1, y1 normalized 0-1 in that camera's frame, for the UI
    view @11 :TurnpilotDetections.View;  # the pass of its last match: far for the far crop (its camera is road)
    farHits @12 :UInt32;     # matches in the far crop; age counts the full pass's alone (a hard stop's confirmation reads age)
    # STOP_SIGNS.md 1.3: the camera time of its last match (TurnpilotDetections.timestampEof, s), the stop-sign evidence's
    # clock (never turnpilotd's). view above is the pass of that match: sideLeft or sideRight for a side view's
    tSeen @13 :Float64;
  }
  available @0 :Bool;          # false when detections are stale; never read as "nothing there"
  objects @1 :List(TrackedObject);
}

struct TurnpilotLights @0xda96579883444c35 {
  # Traffic light states ahead, for TurnPilot's map signal holds (selfdrive/turnpilot/map_controls.py). The BeamNG
  # bridge's oracle publishes it as its own message (simOracle), and turnpilotd takes that only with SIMULATION=1 and
  # TurnPilotSimLightOracle set: never on the car. modeld's camera reader fills one inside turnpilotDetections (camera).
  enum Source {
    none @0;
    simOracle @1;
    camera @2;
  }
  enum State {
    unknown @0;
    red @1;
    yellow @2;
    green @3;
  }
  struct Light {
    state @0 :State;
    distance @1 :Float32;  # m ahead to its stop line; source camera: to the head itself, from its elevation (the line is unknown)
    instance @2 :Text;     # the sim's signal instance, for logs
    # source camera only (detector/light_state.py)
    conf @3 :Float32;      # the detector's class-9 confidence
    box @4 :List(Float32); # x0, y0, x1, y1 normalized 0-1 in the road camera's frame
    bearing @5 :Float32;   # deg left of the calibrated forward axis
    votes @6 :UInt8;       # of the last VOTE_M reads of this head (light_state.py), how many read this state
    governs @7 :Bool;      # its bearing lies over the car's lane (within GOVERN_BEARING_DEG)
  }
  source @0 :Source;
  lights @1 :List(Light);
  shadow @2 :Bool;   # source camera: logged by turnpilotd, never acted on (TurnPilotCameraLights shadow)
}

struct TurnpilotFarDetections @0x80ae746ee2596b11 {
  # The far crop's run (DESIGN phase 4 G5; detector/modeld_detector.py FarCrop), published by modeld on the model frames
  # between the full pass's runs. Its own service, so the full pass's stream (turnpilotDetections: the scene's freshness,
  # turnpilotd's detector deadline, the lane paint) is untouched by it.
  run @0 :TurnpilotDetections;  # camera road, view far; detections in the road camera's full-frame pixels; lights its class-9
                                # heads (a reader of its own, shadow unless TurnPilotCameraLights is addOnly)
  crop @1 :List(Float32);       # x0, y0, x1, y1 of the crop, road camera full-frame pixels
  bearing @2 :Float32;          # the aim it was given (turnpilotPlan.farCropBearing), deg left of the calibrated forward axis
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
