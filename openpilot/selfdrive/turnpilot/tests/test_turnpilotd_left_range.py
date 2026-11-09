"""The autonomous unprotected left's reach and lane, closed loop on test_turnpilotd_left's synthetic route (review 2026-11-07).

LN-2: the band is certified only as far as what runs on the car can box. The full pass is the road frame letterboxed into
640x384, which boxes no car past about 98 m (unprotected_left.FULL_PASS_RANGE_M); TurnPilotLeftRangeM's 210 m (Kellen's,
maps/defaults.CAR_LEFT_RANGE_M) is pixel arithmetic for the native schedule, and is the reach only while that schedule fills
the far crop's slot (nativeStatus full or strip), its runs judged over the band past the full pass's range. On the car on
11-06 neither the native schedule nor the far crop read, and a 25-35 mph left went on the full pass alone at 210 m.

Since 2026-11-07 (Kellen: no handbacks) a left past the full pass's reach with nothing in the far slot goes from its line on the
band the full pass covers; a car boxed in that band still holds it.

LN-5: an autonomous left the lane planner's LC-03 places in a lane it doesn't leave from (the change it needed blocked to the
window's end) is never skipped and rerouted (Kellen, 2026-11-06: never route around a left) and, since 2026-11-07, never handed
back: it is driven to its line and taken from the lane it is in, said once (turnpilot.left_wrong_lane).

The shared Sim lifts the car's full-pass cap (its detector boxes every car at any range); the tests of the cap set it back."""

import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.maps.defaults import CAR_LEFT_RANGE_M, FAR_CROP_LEFT_RANGE_M
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import BROKEN_WHITE_LEFT, THREE_LANES_LEFT_TURN, Sim
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import (JUNCTION, LINE, STRAIGHT_M, TEST_RANGE_M, V_25, drive,
                                                                      far_runs,
                                                                     road_runs)
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_night import NIGHT_EV, NightSim
from openpilot.selfdrive.turnpilot.turnpilotd import TurnPilot
from openpilot.selfdrive.turnpilot.unprotected_left import FULL_PASS_RANGE_M, reach_at_line

Desire = log.Desire
V_35 = 15.65
V_15 = 6.7
NATIVE_COVER = (-30.5, 30.5)  # a native strip's bearings at the horizon: the 1344 px road frame's width at f 1141.5 px


def left_on(v_limit, full_pass=FULL_PASS_RANGE_M, sim_cls=Sim, **kw):
  """A one-lane left at JUNCTION across a v_limit road, TurnPilotAutoLeft auto, TurnPilotLeftRangeM the car's (210 m), the
  full pass capped at full_pass (the car's FULL_PASS_RANGE_M), an empty band, the driver answering the signal prompt."""
  sim = sim_cls([(JUNCTION, "turn", "left", (0., 270., 2, 'present', v_limit, 1, STRAIGHT_M)),
                 (1500., "turn", "right", junction_of("right"))], **kw)
  sim.tp.auto_left, sim.tp.left_range_m, sim.tp.full_pass_range_m = 'auto', CAR_LEFT_RANGE_M, full_pass
  sim.go_taps, sim.answers_prompt, sim.raw = False, True, road_runs()
  return sim


def native_reading(sim, status='full', far='read'):
  """The full pass's messages say nativeStatus status and farStatus far (the native schedule in the far crop's slot)."""
  sim.far_status = lambda s: far
  put = sim.sm.put

  def put_native(service, msg, valid=True):
    if service == 'turnpilotDetections':
      b = msg.as_builder()
      b.nativeStatus = status
      msg = b.as_reader()
    put(service, msg, valid)
  sim.sm.put = put_native


def through(sim, limit_s=120.):
  plans = []
  while sim.s < JUNCTION + 30. and sim.t < limit_s:
    plans += drive(sim, DT_MDL)
  return plans


# ---------------------------------------------------------------------------------------------------- LN-2: the reach

def test_the_full_pass_range_is_what_the_letterboxed_pass_can_box():
  # 1.8 m at f 1141.5 px is 978/R px in the 640/1344 letterbox; 10 px, the detector's smallest boxes, at 97.8 m
  assert abs(1.8 * 1141.5 * 640. / 1344. / FULL_PASS_RANGE_M - 10.) < 0.1
  tp = TurnPilot()
  assert tp.full_pass_range_m == FULL_PASS_RANGE_M  # the car's: never lifted outside the tests' simulated detector
  tp.left_range_m = CAR_LEFT_RANGE_M
  assert tp.left_ranges() == (FULL_PASS_RANGE_M, 0.)  # nothing in the far slot: the full pass's alone
  tp.left_far_range_m = 250.
  assert tp.left_ranges() == (FULL_PASS_RANGE_M, 250.)  # a measured far crop's
  tp.left_far_range_m, tp.native_status, tp.far_status_age = 0., 'full', 0.
  assert tp.left_ranges() == (FULL_PASS_RANGE_M, CAR_LEFT_RANGE_M)  # the native schedule's: TurnPilotLeftRangeM
  for status in ('unavailable', 'fallback', 'off'):  # never run, gone back to the far crop, off: the far crop's (unmeasured)
    tp.native_status = status
    assert tp.left_ranges() == (FULL_PASS_RANGE_M, 0.), status
  tp.native_status, tp.far_status_age = 'full', 2.  # the full pass's message stale: not known to fill the slot
  assert tp.left_ranges() == (FULL_PASS_RANGE_M, 0.)
  tp.left_range_m = 60.  # a range measured under the cap is the range
  assert tp.left_ranges()[0] == 60.


def test_a_live_far_crop_extends_the_band_to_its_provisional_range():
  # 2026-11-07 (bleeding edge): TurnPilotLeftFarRangeM unmeasured (0) and the far crop on, the far slot's range is
  # FAR_CROP_LEFT_RANGE_M, the native strips' pixel arithmetic at the same density (maps/defaults): 1.8 m at f 1141.5 px is
  # ~10 px at 205.5 m, 9.8 at 210. Until then a live far crop extended nothing, and the band was the full pass's ~98 m
  assert abs(1.8 * 1141.5 / FAR_CROP_LEFT_RANGE_M - 9.8) < 0.05 and FAR_CROP_LEFT_RANGE_M == CAR_LEFT_RANGE_M
  tp = TurnPilot()
  tp.left_range_m = CAR_LEFT_RANGE_M
  for status in ('read', 'noRoom', 'noCalibration', 'sweep'):  # on: reading, or waiting for room (far_live, route time's)
    tp.far_status, tp.far_status_age = status, 0.
    assert tp.left_ranges() == (FULL_PASS_RANGE_M, FAR_CROP_LEFT_RANGE_M), status
  for status in ('disabled', 'off', 'unset'):  # off, failing, or a log from before the field: nothing past the full pass
    tp.far_status = status
    assert tp.left_ranges() == (FULL_PASS_RANGE_M, 0.), status
  tp.far_status, tp.far_status_age = 'read', 2.  # its status stale: not known to read
  assert tp.left_ranges() == (FULL_PASS_RANGE_M, 0.)
  tp.far_status_age, tp.left_far_range_m = 0., 150.  # a measured range wins over the provisional one
  assert tp.left_ranges() == (FULL_PASS_RANGE_M, 150.)
  tp.far_status = 'disabled'  # ...and is kept as before, the far slot's liveness the left's own input (far_live)
  assert tp.left_ranges() == (FULL_PASS_RANGE_M, 150.)


def assert_went(sim, plans):
  """Gone from its line, never undecided or a take-control prompt, never skipped or rerouted (2026-11-07: no handbacks)."""
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert str(plans[go].unprotectedLeft.state) == 'go'
  assert not any(str(p.unprotectedLeft.state) == 'undecided' or str(p.controlPrompt) == 'leftUndecidedTakeControl' for p in plans)
  assert not any(p.tooFastToTurn for p in plans), "never skipped or rerouted"
  assert sim.s > JUNCTION + 30.
  return go


@pytest.mark.parametrize("v_limit", [V_25, V_35])
def test_by_day_a_left_past_what_the_full_pass_boxes_goes_on_the_band_it_covers(v_limit):
  """The car's ranges as they were on 11-06 (TurnPilotLeftRangeM 210, the far crop unmeasured, the native schedule not
  running): a 25 or 35 mph left (R_req 134 and 183 m at its line), past the ~98 m the full pass boxes. Kellen, 2026-11-07 (no
  handbacks): driven to its line, never skipped, and gone from it on the band the full pass covers, its range capped there."""
  sim = left_on(v_limit)
  assert not sim.tp.dark and reach_at_line(v_limit, 1) > FULL_PASS_RANGE_M
  plans = through(sim)
  go = assert_went(sim, plans)
  assert plans[go].unprotectedLeft.rangeM == FULL_PASS_RANGE_M < plans[go].unprotectedLeft.rReq


def test_by_day_a_car_the_full_pass_boxes_still_holds_the_left_past_its_reach():
  # the same 25 mph left, an oncoming car at the limit 80 m up the road when the car nears its line, inside the full pass's
  # reach: the car waits for it to pass and only then goes
  s0 = [None]

  def cars(sim):
    if s0[0] is None and JUNCTION - sim.s < 40.:
      s0[0] = sim.t
    dt = sim.t - s0[0] if s0[0] is not None else 0.
    return [(JUNCTION + 80. - V_25 * dt, 3.6)]
  sim = left_on(V_25)
  sim.raw = road_runs(cars)
  plans = through(sim)
  go = assert_went(sim, plans)
  assert go * DT_MDL > s0[0] + (80. + HOLD_LINE_M) / V_25 and any(p.unprotectedLeft.blockers for p in plans[:go])


def test_by_day_a_left_within_what_the_full_pass_boxes_goes_on_it_alone():
  # a 15 mph road's R_req (85 m at its line) is inside the full pass's reach: the cap leaves its own range to it
  sim = left_on(V_15)
  assert reach_at_line(V_15, 1) < FULL_PASS_RANGE_M
  plans = through(sim)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert str(plans[go].unprotectedLeft.state) == 'go' and not plans[go].unprotectedLeft.farNeeded
  assert sim.s > JUNCTION + 30.


@pytest.mark.parametrize("v_limit", [V_25, V_35])
def test_with_the_native_schedule_reading_a_left_goes_on_210_m_judged_on_its_runs(v_limit):
  """Kellen's 210 m while the native schedule fills the far crop's slot: the band past the full pass's ~98 m is judged on its
  runs (far_needed), and the left goes from its line on an empty band."""
  sim = left_on(v_limit)
  native_reading(sim)
  sim.raw_far = far_runs(cover=NATIVE_COVER)
  plans = through(sim)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  ul = plans[go].unprotectedLeft
  assert str(ul.state) == 'go' and ul.farNeeded and ul.rangeM == FULL_PASS_RANGE_M and ul.farRangeM == CAR_LEFT_RANGE_M
  assert sim.trace[go - 1][1] < 0.3 and LINE - 1. < sim.trace[go][0] <= LINE + 0.1
  assert sim.s > JUNCTION + 30. and not any(p.tooFastToTurn for p in plans)


def test_with_the_native_schedule_reading_a_car_only_its_runs_box_holds_the_left():
  # an oncoming car at the limit 180 m up the road when the car nears its line: past the full pass's reach (its runs box nothing
  # here), boxed by the native runs alone: the car waits for it to pass and only then goes
  s0 = [None]

  def cars(sim):
    if s0[0] is None and JUNCTION - sim.s < 40.:
      s0[0] = sim.t
    dt = sim.t - s0[0] if s0[0] is not None else 0.
    return [(JUNCTION + 180. - V_25 * dt, 3.6)]
  sim = left_on(V_25)
  native_reading(sim)
  sim.raw_far = far_runs(cars, cover=NATIVE_COVER)
  plans = through(sim)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert go * DT_MDL > s0[0] + (180. + HOLD_LINE_M) / V_25 and any(p.unprotectedLeft.farBlockers for p in plans[:go])


def test_the_native_schedule_said_on_with_no_runs_at_the_line_goes_on_the_full_pass_band():
  # nativeStatus full and farStatus noRoom (11-06: every message said so, the schedule never ran): no run covers the band past
  # the full pass's range. Kellen, 2026-11-07 (no handbacks): the left goes from its line on the band the full pass covers
  sim = left_on(V_25)
  native_reading(sim, far='noRoom')
  assert_went(sim, through(sim))
  # never run in its first NATIVE_NO_RUN_S (nativeStatus unavailable): the far crop's slot, its provisional range
  # (FAR_CROP_LEFT_RANGE_M, TurnPilotLeftFarRangeM unmeasured): gone on its runs over an empty band
  sim = left_on(V_25)
  native_reading(sim, status='unavailable')
  sim.raw_far = far_runs(cover=NATIVE_COVER)
  plans = through(sim)
  go = assert_went(sim, plans)
  assert plans[go].unprotectedLeft.farRangeM == FAR_CROP_LEFT_RANGE_M and plans[go].unprotectedLeft.farNeeded


# ---------------------------------------------------------------------------------------------------- the far crop at a left

FAR_CROP_COVER = (-15.6, 15.6)  # the far crop's 640 px at the horizon, at f 1141.5 px (aimed straight here: far_runs')


def car_cadence(monkeypatch, frames=1200):
  """The detector's frames at a left's line on the car's budget, far first (detector/tests/test_far_first.car_drive: the
  11-06 timings replayed through modeld_detector on a simulated clock): per frame 'road', 'wide', 'far' or None."""
  from openpilot.selfdrive.turnpilot.detector.tests.test_far_first import car_drive
  _, _, what, _ = car_drive(monkeypatch, frames, 'roadAndWide', 0., far_first=True)
  return what


def cadenced(what, cars, kind, cover=FAR_CROP_COVER):
  """A raw (kind 'road') or raw_far ('far') of the Sim's on the cadence what (cycled): road_runs' and far_runs' boxes, on
  the frames the schedule ran that pass."""
  import dataclasses
  from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import raw_box
  frame = [0]

  def raw(sim):
    k, frame[0] = frame[0] % len(what), frame[0] + 1
    if what[k] != kind:
      return None
    if kind == 'road':
      return RawFrame([raw_box(s - sim.s, y) for s, y in cars(sim) if -5. < s - sim.s < FULL_PASS_RANGE_M], 'road', sim.t)
    boxes = [dataclasses.replace(raw_box(s - sim.s, y), view='far') for s, y in cars(sim) if -5. < s - sim.s < 800.]
    return RawFrame(boxes, 'road', sim.t, 'far', cover)
  return raw


def oncoming_at_line(distance_m, v):
  """An oncoming car distance_m up the road from the junction when the car is first held at its line, closing at v from
  then (standing out there before, where nothing judges it yet): cars(sim), and its clock (s0[0], the hold's time)."""
  s0 = [None]

  def cars(sim):
    if s0[0] is None and sim.s > LINE - 2. and sim.v < 0.3:
      s0[0] = sim.t
    dt = sim.t - s0[0] if s0[0] is not None else 0.
    return [(JUNCTION + distance_m - v * dt, 3.6)]
  return cars, s0


def test_a_car_only_the_far_crop_sees_at_150_m_holds_the_left_until_it_passes(monkeypatch):
  """The car's far reading at a left (2026-11-07): a 35 mph left (R_req 183 m at its line), the far crop reading on the
  far-first schedule's cadence on the car's budget (car_cadence: road and far runs about 4 a second each), its range the
  provisional one, an oncoming car at the limit 150 m up the road when the car is held at its line: past the ~98 m the
  full pass boxes, so only the far crop's runs see it. Held while it comes, gone once it has passed, never handed back."""
  what = car_cadence(monkeypatch)
  cars, s0 = oncoming_at_line(150., V_35)
  sim = left_on(V_35)
  sim.far_status = lambda s: 'read'
  sim.raw, sim.raw_far = cadenced(what, cars, 'road'), cadenced(what, cars, 'far')
  plans = through(sim, 150.)
  go = assert_went(sim, plans)
  passed_t = s0[0] + (150. + HOLD_LINE_M) / V_35  # the car past the junction's conflict point
  assert go * DT_MDL > passed_t and any(p.unprotectedLeft.farBlockers for p in plans[:go])
  held = [p for i, p in enumerate(plans[:go]) if s0[0] + 1. < i * DT_MDL < passed_t]
  assert held and all(p.unprotectedLeft.farNeeded and p.unprotectedLeft.farRangeM == FAR_CROP_LEFT_RANGE_M for p in held)
  assert all(str(p.unprotectedLeft.state) != 'go' for p in held)
  # what the far reading catches: the same left with 11-06's far slot (no far run; its range unmeasured then, so nothing
  # waited for one: here the far crop off) goes with that car still coming inside its R_req, before it is in the ~98 m the
  # full pass boxes
  cars, s0 = oncoming_at_line(150., V_35)
  sim = left_on(V_35)
  sim.far_status = lambda s: 'off'
  sim.raw = cadenced(what, cars, 'road')
  plans = through(sim, 150.)
  # its first go (on the car's cadence since review 2026-11-07, R1, the road runs come on their due frame, so the go can
  # come late enough that the band's re-check in its launch, that car now inside the full pass's ~98 m, withdraws it:
  # either way it was gone blind to it)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert go * DT_MDL < s0[0] + (150. + JUNCTION - LINE - FULL_PASS_RANGE_M) / V_35
  assert plans[go].unprotectedLeft.rReq > 150. + JUNCTION - LINE - V_35 * (go * DT_MDL - s0[0])  # inside its R_req
  assert not any(p.tooFastToTurn for p in plans)


def test_a_far_crop_that_stops_reading_at_the_line_never_hands_back(monkeypatch):
  # the far crop reads on the car's cadence as the car comes to its line, then stops (a stall, a failure's backoff: farStatus
  # noRoom, then disabled): judged on the band the full pass covers after STALE_UNDECIDED_S without a far run over the band
  # (unprotected_left: far_needed off), or at once with the far slot off; gone on an empty band, never undecided
  from openpilot.selfdrive.turnpilot.unprotected_left import STALE_UNDECIDED_S
  what = car_cadence(monkeypatch)
  for after in ('noRoom', 'disabled'):
    held = [None]

    def status(s, after=after, held=held):
      if held[0] is None and s.s > LINE - 2. and s.v < 0.3:
        held[0] = s.t
      return 'read' if held[0] is None or s.t < held[0] + 0.5 else after
    far = cadenced(what, lambda s: (), 'far')

    def raw_far(s, far=far, held=held):
      return far(s) if held[0] is None or s.t < held[0] + 0.5 else None
    sim = left_on(V_25)
    sim.far_status = status
    sim.raw, sim.raw_far = cadenced(what, lambda s: (), 'road'), raw_far
    plans = through(sim)
    go = assert_went(sim, plans)
    assert go * DT_MDL > held[0] + 0.5 + (STALE_UNDECIDED_S if after == 'noRoom' else 0.), after
    assert not plans[go].unprotectedLeft.farNeeded, after


# ---------------------------------------------------------------------------------------------------- LN-5: the lane

def lane_left(sim_cls=Sim, paint=BROKEN_WHITE_LEFT, blind_left=True, **kw):
  """A 25 mph one-lane left from the middle of three lanes (THREE_LANES_LEFT_TURN: the turn leaves from the left one), the
  model's lanes one each side: the plan wants one change left. blind_left: the left blind spot occupied the whole way."""
  sim = left_on(V_25, full_pass=TEST_RANGE_M, sim_cls=sim_cls, nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(1, 1),
                paint=paint, **kw)
  if blind_left:
    sim.blindspot = lambda s: (True, False)
  return sim


def assert_wrong_lane_skipped(sim, plans, reasons):
  """Kellen, 2026-11-08 (drive 29: lefts driven across a lane from the wrong one after failed changes): a left whose turn lane
  the car can't reach is skipped out loud and rerouted (turn_skip: tooFastToTurn, its reason), never taken across a lane
  from the wrong one, never handed back; the blind spot's car still held the change."""
  assert reasons & {'laneBlocked', 'laneUnreachable'}, reasons
  assert any(p.tooFastToTurn for p in plans) and any(str(p.skipReason) in ('laneBlocked', 'laneUnreachable') for p in plans)
  assert not any(p.desire == log.Desire.turnLeft for p in plans)
  assert not any(str(p.unprotectedLeft.state) in ('go', 'committed') for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert any(p.laneChangeWant == 1 for p in plans) and all(p.laneChange == 0 for p in plans)
  assert sim.tp.left_lane_id is None  # never driven from its lane (turnpilot.left_wrong_lane)


def through_reasons(sim):
  plans, reasons = [], set()
  while sim.s < JUNCTION + 30. and sim.t < 120.:
    plans += drive(sim, DT_MDL, yaw=False)
    reasons.add(sim.tp.skip.reason)
  return plans, reasons


@pytest.mark.parametrize("night", [False, True])
def test_a_left_whose_lane_change_never_went_is_skipped_and_rerouted_never_taken_from_the_wrong_lane(night):
  """Drive 29 (2026-11-08, cause 3): lefts LC-03 left in a lane they don't leave from (the change held to the window's end:
  the blind spot here; lineNotBroken at night, noSignal, tooLate on the car) were driven from the lane the car was in, across
  a lane at the junction (LN-5, 2026-11-07). The owner's ruling: skipped out loud and rerouted, never taken across a lane."""
  sim = lane_left(NightSim, ev=lambda s: NIGHT_EV) if night else lane_left()
  plans, reasons = through_reasons(sim)
  assert sim.tp.dark == night
  assert_wrong_lane_skipped(sim, plans, reasons)


def test_a_night_left_whose_paint_is_never_read_is_skipped_and_rerouted():
  # 1e's night (no change offered on 16,798 plan frames), drive 29's lineNotBroken: the left line never read, so the change
  # waits lineNotBroken to the window's end, the car in a lane the left doesn't leave from: skipped and rerouted (2026-11-08)
  sim = lane_left(NightSim, paint=(UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN), blind_left=False, ev=lambda s: NIGHT_EV)
  plans, reasons = through_reasons(sim)
  assert sim.tp.dark and any(str(p.laneChangeWait) == 'lineNotBroken' for p in plans)
  assert_wrong_lane_skipped(sim, plans, reasons)


# LS-1 and LEFTS-2 (a car beside in the lanes a wrong-lane left crosses holds it: unprotected_left.crossed_lane_cars) stay for
# a left LC-03 flags only once the car stands held at its line, past where a skip could take it straight on (turnpilotd); the
# band's width is tested here on its own


def test_the_crossed_band_is_as_wide_as_the_lanes_the_left_crosses():
  # LEFTS-2: a lane of 3.6 m per change LC-03 left unmade, from ONCOMING_Y_NEAR_M; one lane (no lane list: LC-03 wants one
  # change at a time) as LS-1 shipped it, so oncoming cars passing beside the car never hold it
  from openpilot.selfdrive.turnpilot.unprotected_left import CROSSED_LANE_Y_FAR_M, in_crossed_lane
  for y in (3.4, 5.0):
    assert in_crossed_lane(y) and in_crossed_lane(y, 2)
  for y in (5.6, 7.2, 8.9):
    assert not in_crossed_lane(y) and in_crossed_lane(y, 2), y
  assert not in_crossed_lane(CROSSED_LANE_Y_FAR_M + 3.7, 2) and not in_crossed_lane(1.0, 2)


def test_a_far_side_left_is_skipped_for_its_lane_with_the_mode_off_too():
  # TurnPilotAutoLeft off: LC-03 skips such a left out loud for its lane, as it does every left since 2026-11-08 (drive 29)
  sim = lane_left()
  sim.tp.auto_left = 'off'
  plans = drive(sim, 40., yaw=False)
  assert any(p.tooFastToTurn and str(p.skipReason) == 'laneBlocked' for p in plans)
  assert not any(p.desire == log.Desire.turnLeft for p in plans)
