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
from unittest import mock

import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.maps.defaults import CAR_LEFT_RANGE_M
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import BROKEN_WHITE_LEFT, THREE_LANES_LEFT_TURN, Sim
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import (JUNCTION, LINE, STRAIGHT_M, TEST_RANGE_M, V_25, drive, far_runs,
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
  # never run in its first NATIVE_NO_RUN_S (nativeStatus unavailable): the far crop's slot, its range unmeasured: the same
  sim = left_on(V_25)
  native_reading(sim, status='unavailable')
  sim.raw_far = far_runs(cover=NATIVE_COVER)
  plans = through(sim)
  go = assert_went(sim, plans)
  assert plans[go].unprotectedLeft.farRangeM == 0.


# ---------------------------------------------------------------------------------------------------- LN-5: the lane

def lane_left(sim_cls=Sim, paint=BROKEN_WHITE_LEFT, blind_left=True, **kw):
  """A 25 mph one-lane left from the middle of three lanes (THREE_LANES_LEFT_TURN: the turn leaves from the left one), the
  model's lanes one each side: the plan wants one change left. blind_left: the left blind spot occupied the whole way."""
  sim = left_on(V_25, full_pass=TEST_RANGE_M, sim_cls=sim_cls, nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(1, 1),
                paint=paint, **kw)
  if blind_left:
    sim.blindspot = lambda s: (True, False)
  return sim


def assert_wrong_lane_go(sim, plans):
  """Kellen, 2026-11-07 (no handbacks): taken from the lane it is in, never skipped and rerouted, never handed back; the blind
  spot's car still held the change."""
  assert_went(sim, plans)
  assert any(p.laneChangeWant == 1 for p in plans) and all(p.laneChange == 0 for p in plans)


def through_reasons(sim):
  plans, reasons = [], set()
  while sim.s < JUNCTION + 30. and sim.t < 120.:
    plans += drive(sim, DT_MDL, yaw=False)
    reasons.add(sim.tp.skip.reason)
  return plans, reasons


@pytest.mark.parametrize("night", [False, True])
def test_a_left_whose_lane_change_never_went_is_taken_from_its_lane_never_skipped(night):
  """Review 2026-11-07 (LN-5): the change into the left's lane held by the blind spot to the window's end. LC-03 skipped the
  left 'Can't Reach Turn Lane' about 60 m out and navd rerouted (the probe: laneBlocked at 59.5 m, by night and by day), the
  reroute bringing the next left. Now it is driven to its line and taken from the lane it is in, said once."""
  sim = lane_left(NightSim, ev=lambda s: NIGHT_EV) if night else lane_left()
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    plans, reasons = through_reasons(sim)
  assert sim.tp.dark == night
  assert_wrong_lane_go(sim, plans)
  assert 'laneBlocked' not in reasons
  said = [c.kwargs for c in event.call_args_list if c.args[0] == 'turnpilot.left_wrong_lane']
  assert len(said) == 1 and said[0]['why'] == 'laneBlocked' and said[0]['want'] == 1 and said[0]['distance'] > 50.


def test_a_night_left_whose_paint_is_never_read_is_taken_from_its_lane_not_skipped():
  # 1e's night (no change offered on 16,798 plan frames): the left line never read, so the change waits lineNotBroken to the
  # window's end; LC-03 skipped it at 59.5 m. Now taken from the lane it is in (2026-11-07: no handbacks)
  sim = lane_left(NightSim, paint=(UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN), blind_left=False, ev=lambda s: NIGHT_EV)
  plans, _ = through_reasons(sim)
  assert sim.tp.dark and any(str(p.laneChangeWait) == 'lineNotBroken' for p in plans)
  assert_wrong_lane_go(sim, plans)


def test_a_far_side_left_is_never_skipped_for_its_lane_even_with_the_mode_off():
  # TurnPilotAutoLeft off: LC-03 skipped such a left out loud for its lane. 2026-11-07 (no handbacks, never route around a
  # left): a far-side unprotected left is never skipped or rerouted for its lane, in hand or not
  sim = lane_left()
  sim.tp.auto_left = 'off'
  plans = drive(sim, 40., yaw=False)
  assert not any(p.tooFastToTurn or p.skipReason == 'laneBlocked' for p in plans)
  # ...nor one the left held for its lane that it no longer has in hand (here the mode turned off on the approach)
  sim = lane_left()
  plans = []
  while sim.tp.left_lane_id is None:
    plans += drive(sim, DT_MDL, yaw=False)
    assert sim.t < 60.
  sim.tp.auto_left = 'off'
  plans += drive(sim, 2., yaw=False)
  assert not any(p.tooFastToTurn or p.skipReason == 'laneBlocked' for p in plans)
