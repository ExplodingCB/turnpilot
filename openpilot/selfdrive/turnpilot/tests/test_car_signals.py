"""TurnPilot's turn signal intent, status, prompt, veto and the verified record (car_signals.py, DESIGN phase4/blinker 11.3)."""
import types

import pytest

from opendbc.car import structs
from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom, messaging
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from openpilot.selfdrive.turnpilot import car_signals as cs_mod
from openpilot.selfdrive.turnpilot.car_signals import (ARM_LAG_S, CAR_SIGNAL_WAIT_S, LEAD_LONG_M, LEAD_SHORT_M, TRAVEL_BACKSTOP_M,
                                                        TurnSignalIntent, check_record, command_side, driver_signals_for, fw_list,
                                                        lead_m, port_signals, promote, record_matches, resolver, vin_valid)
from openpilot.selfdrive.turnpilot.route_follower import PASSED_DIST_M, classify, Maneuver

LaneChange = custom.TurnpilotPlan.LaneChange
Status = custom.TurnpilotPlan.TurnSignal.Status
DT = 0.05
VIN = "5YJ3E1EA8KF000001"
OTHER_VIN = "5YJ3E1EAXKF000002"


class FakeParams(dict):
  def get(self, key):
    return super().get(key)

  def put(self, key, value, block=False):
    self[key] = value

  def remove(self, key):
    self.pop(key, None)


def port(status='ready', left=False, right=False, block='none', fault='none', hazard=False, left_lamp=False, right_lamp=False):
  return types.SimpleNamespace(status=status, left=left, right=right, block=block, fault=fault, hazard=hazard, leftLamp=left_lamp,
                               rightLamp=right_lamp, checkWhy='')


def desires(mid=0, started=False, done=False, turned=0.):
  return types.SimpleNamespace(maneuver_id=mid, started=started, done=done, turned_deg=turned)


def maneuver(mtype="turn", modifier="left"):
  kind, side = classify(mtype, modifier)
  return Maneuver(kind, side, mtype, modifier, 90.)


class Drive:
  """A car approaching one maneuver at v, the intent updated each frame; the port answers ready, or as set."""
  def __init__(self, m=None, v=11., distance=400., status=Status.ready):
    self.intent = TurnSignalIntent()
    self.m = m if m is not None else maneuver()
    self.v, self.distance, self.status = v, distance, status
    self.port = port()
    self.d = desires()
    self.kw = {}
    self.out = None

  def step(self, n=1, **kw):
    for _ in range(n):
      args = {'status': self.status, 'port': self.port, 'gates_ok': True, 'engaged': True, 'lane_side': LaneChange.none,
              'lane_pending': False, 'maneuver': self.m, 'maneuver_id': 0, 'distance': self.distance, 'desires': self.d,
              'missed': False, 'v_ego': self.v, 'dt': DT}
      args.update(self.kw)
      args.update(kw)
      self.out = self.intent.update(**args)
      self.distance -= self.v * DT
    return self.out

  def until(self, distance, **kw):
    while self.distance > distance:
      self.step(**kw)
    return self.out


def test_lead_m():
  assert lead_m(0., 10.) == LEAD_LONG_M  # no mapped limit: 300 ft
  assert lead_m(40 * 0.44704, 15.) == LEAD_SHORT_M  # a known 40 mph limit under 50 mph: 200 ft
  assert lead_m(40 * 0.44704, 52 * 0.44704) == LEAD_LONG_M  # the car itself at 52 mph
  assert lead_m(55 * 0.44704, 20.) == LEAD_LONG_M


def test_arms_at_lead_plus_lag():
  v = 24.6
  drive = Drive(v=v, distance=300.)
  arm_m = LEAD_LONG_M + v * ARM_LAG_S
  assert 125. < arm_m < 127.
  drive.until(arm_m + 2.)
  assert drive.out.side == LaneChange.none
  drive.until(arm_m - 2.)
  assert drive.out.side == LaneChange.left and abs(drive.out.arm_m - arm_m) < 2.


def test_arms_at_once_inside():
  drive = Drive(distance=50.)
  assert drive.step().side == LaneChange.left
  assert drive.out.arm_m == pytest.approx(50.)


@pytest.mark.parametrize("mtype,modifier,signals", [
  ("turn", "left", LaneChange.left), ("turn", "right", LaneChange.right), ("turn", "slight left", LaneChange.left),
  ("turn", "sharp right", LaneChange.right), ("fork", "slight right", LaneChange.right), ("off ramp", "slight left", LaneChange.left),
  ("continue", "slight left", LaneChange.none), ("continue", "straight", LaneChange.none), ("turn", "uturn", LaneChange.none),
  ("arrive", "left", LaneChange.none), ("roundabout", "right", LaneChange.none), ("new name", "slight right", LaneChange.none),
])
def test_kinds(mtype, modifier, signals):
  drive = Drive(m=maneuver(mtype, modifier), distance=50.)
  assert drive.step().side == signals


def test_held_through_a_stop_and_a_long_wait():
  drive = Drive(distance=100.)
  drive.until(5.)
  drive.v = 0.
  drive.step(int(120. / DT), standing=True)
  assert drive.out.side == LaneChange.left


@pytest.mark.parametrize("end", ["done", "passed", "new_id", "skipped", "missed", "disengaged", "gates"])
def test_ends(end):
  drive = Drive(distance=60.)
  assert drive.step().side == LaneChange.left
  if end == "done":
    drive.d = desires(started=True, done=True, turned=90.)
  elif end == "passed":
    drive.distance = -PASSED_DIST_M - 1.  # past the junction as the route follower counts a pass
  elif end == "new_id":
    drive.distance = 500.  # the new maneuver, far off: not armed in its turn
  kw = {"new_id": {'maneuver_id': 1}, "skipped": {'maneuver': None}, "missed": {'missed': True},
        "disengaged": {'engaged': False}, "gates": {'gates_ok': False}}.get(end, {})
  assert drive.step(**kw).side == LaneChange.none


def test_passed_with_the_yaw_holds_until_done():
  drive = Drive(distance=20.)
  drive.step()
  drive.d = desires(started=True, turned=20.)
  drive.distance = -3.
  assert drive.step().side == LaneChange.left


def test_a_near_reading_past_the_junction_never_ends_it():
  """rf.distance is the nearest of navd's readings (route_follower.JunctionSpan), up to UNCERTAIN_M short of the truth:
  without a desire out, the signal ends only PASSED_DIST_M past it, as every other TurnPilot rule counts a pass."""
  drive = Drive(distance=20.)
  drive.step()
  drive.distance = -2.
  assert drive.step().side == LaneChange.left
  drive.distance = -PASSED_DIST_M + 1.
  assert drive.step().side == LaneChange.left


def test_a_started_turn_ends_only_done_missed_or_passed_on():
  """Once its desire is out, the turn's signal ends when the turn is done (cleared at its angle, or the follower's pass),
  missed, or the maneuver moves on, never on a reading past the junction while the car pulls into it."""
  drive = Drive(distance=20.)
  drive.step()
  drive.d = desires(started=True, turned=3.)
  drive.distance = -14.
  drive.v = 2.
  assert all(s == LaneChange.left for s in (drive.step().side for _ in range(40)))
  assert drive.step(missed=True).side == LaneChange.none


def test_unprotected_left_keeps_its_signal_through_the_go():
  """At the go from the left's hold line the lamp stays lit while the car pulls across the oncoming road, with the
  junction's nearest reading at -1 m (the car at its line, LINE_D short of the maneuver point) and the turn under 15 deg."""
  drive = Drive(m=maneuver("turn", "left"), distance=140., v=10.)
  drive.kw = {'left_lamp': True}
  drive.port = port(status='lit', left=True, left_lamp=True)
  drive.until(6.)
  assert drive.out.side == LaneChange.left
  drive.v, drive.distance = 0., -1.
  drive.step(100, at_left_line=True, standing=True)
  assert drive.out.side == LaneChange.left
  drive.d = desires(started=True)
  sides = []
  for i in range(20):
    drive.v = min(0.5 + 0.2 * i, 4.)
    drive.d.turned_deg = 0.6 * i  # about 12 deg over the first second from a standstill
    sides.append(drive.step(go_now=i == 0).side)
  assert all(s == LaneChange.left for s in sides), sides


def test_stop_sign_right_keeps_its_signal_through_the_go():
  drive = Drive(m=maneuver("turn", "right"), distance=120., v=8.)
  drive.kw = {'right_lamp': True}
  drive.port = port(status='lit', right=True, right_lamp=True)
  drive.d = desires(started=True)
  drive.until(2.)
  drive.v, drive.distance = 0., -0.5  # stopped at the line; navd's nearest reading puts the junction 0.5 m behind
  drive.step(60, standing=True)
  assert drive.out.side == LaneChange.right
  drive.v = 0.3
  assert drive.step(go_now=True).side == LaneChange.right, "right signal dropped the frame the car pulled away"


def test_travel_backstop_says_so():
  drive = Drive(distance=80.)
  drive.step()
  drive.v = 0.  # a follower that never counts down: the car travels past it
  drive.intent.arm_travel = TRAVEL_BACKSTOP_M + 1.
  out = drive.step()
  assert out.side == LaneChange.none and out.notice == 'ranPast'


def test_lit_m_restarts_when_the_port_leaves_lit():
  drive = Drive(distance=100.)
  drive.port = port(status='lit', left=True)
  drive.step(20)
  assert drive.out.lit_m == pytest.approx(19 * 11. * DT, rel=0.1)
  drive.port = port(status='requesting', left=True)
  assert drive.step().lit_m == 0.
  drive.port = port(status='lit', left=True)
  assert drive.step().lit_m == pytest.approx(11. * DT)
  drive.port = port(status='lit', right=True)  # the port lit for another side: not ours
  assert drive.step().lit_m == 0.


def test_lane_change_outranks_the_turn():
  drive = Drive(distance=60.)
  assert drive.step(lane_side=LaneChange.right).side == LaneChange.right
  assert drive.step().side == LaneChange.left


@pytest.mark.parametrize("status", [Status.off, Status.checking, Status.unverified, Status.verifying, Status.waiting,
                                    Status.unavailable, Status.failed])
def test_prompt_whenever_not_ready(status):
  drive = Drive(v=12., distance=150., status=status)
  drive.until(100.)
  assert drive.out.prompt == LaneChange.none  # held above 20 mph
  drive.v = LANE_CHANGE_SPEED_MIN - 0.5
  assert drive.step().prompt == LaneChange.left
  assert drive.step(lane_pending=True).prompt == LaneChange.none  # a lane change wanted or offered holds it
  assert drive.step(left_lamp=True).prompt == LaneChange.none  # cleared once the lamp is lit...
  assert drive.step(left_lamp=True, right_lamp=True).prompt == LaneChange.left  # ...but hazards aren't the turn's signal
  assert drive.step(maneuver=None).prompt == LaneChange.none  # the turn ended


def test_no_prompt_when_ready():
  drive = Drive(v=5., distance=50.)
  assert drive.step().prompt == LaneChange.none and drive.out.side == LaneChange.left


@pytest.mark.parametrize("p,veto", [
  (port(block='otherSide'), 'driverSignal'), (port(block='hazard'), 'hazards'), (port(hazard=True), 'hazards'),
  (port(block='inherited', right_lamp=True), 'driverSignal'),
])
def test_veto_before_the_turn(p, veto):
  drive = Drive(distance=60.)
  drive.step()
  drive.port = p
  kw = {'right_lamp': True} if p.block == 'inherited' else {}
  assert drive.step(**kw).skip == veto
  # standing at a stop line holding it too
  drive2 = Drive(distance=10.)
  drive2.d = desires(started=True)
  drive2.step(standing=True)
  drive2.port = p
  assert drive2.step(standing=True, **kw).skip == veto


def test_lamp_lost_vetoes():
  drive = Drive(distance=60.)
  drive.step()
  drive.status, drive.port = Status.failed, port(status='failed', fault='lampLost')
  assert drive.step().skip == 'signalLost'


def test_veto_at_the_left_line_hands_back():
  drive = Drive(distance=10.)
  drive.step()
  drive.port = port(block='otherSide')
  out = drive.step(at_left_line=True, standing=True)
  assert out.skip == 'none' and out.left_veto == 'driverSignal'


def test_a_lamp_on_when_the_turn_arms_vetoes():
  drive = Drive(distance=200.)
  drive.port = port(block='otherSide', right_lamp=True)
  drive.until(150.)
  assert drive.out.skip == 'none'  # not armed yet
  drive.until(100.)
  assert drive.out.skip == 'driverSignal'


def test_after_the_turn_starts_no_skip_but_a_notice():
  drive = Drive(distance=30.)
  drive.step()
  drive.d = desires(started=True)
  drive.port = port(block='otherSide')
  out = drive.step()
  assert out.skip == 'none' and out.notice == 'changedMidTurn'


def test_nothing_vetoes_unless_ready():
  drive = Drive(distance=60., status=Status.off)
  drive.port = port(status='off', block='otherSide')
  assert drive.step().skip == 'none'


def test_lane_change_lamp_is_never_a_veto():
  drive = Drive(distance=60.)
  drive.port = port(block='otherSide')
  assert drive.step(lane_side=LaneChange.right).skip == 'none'


def test_handed_over_after_the_mask_is_down():
  drive = Drive(distance=200.)
  drive.port = port(status='lit', left=True)
  drive.until(100.)
  drive.port = port(status='releasing', left=True)
  assert drive.step(engaged=False).handed_over == LaneChange.none  # release still under way
  drive.port = port(status='ready')
  assert drive.step(engaged=False).handed_over == LaneChange.left
  drive.step(int(3.5 / DT), engaged=False)
  assert drive.out.handed_over == LaneChange.none


def test_not_handed_over_when_the_driver_signals_or_the_turn_ends():
  drive = Drive(distance=200.)
  drive.port = port(status='lit', left=True)
  drive.until(100.)
  drive.port = port(status='ready')
  assert drive.step(engaged=False, left_lamp=True).handed_over == LaneChange.none
  drive = Drive(distance=20.)
  drive.port = port(status='lit', left=True)
  drive.step()
  drive.d = desires(started=True, done=True, turned=90.)
  drive.port = port(status='ready')
  drive.step()
  assert drive.step().handed_over == LaneChange.none


def test_signal_lit_for_the_left():
  drive = Drive(distance=20.)
  assert not drive.step().signal_lit
  assert drive.step(left_lamp=True).signal_lit
  assert drive.step(int(0.3 / DT)).signal_lit  # a blink's dark phase
  assert not drive.step(int(0.2 / DT)).signal_lit  # a dark run past LAMP_GAP_S
  assert not drive.step(left_lamp=True, right_lamp=True).signal_lit  # hazards


def test_status_for():
  intent = TurnSignalIntent()
  assert intent.status_for(0, 'off', port('off'), DT) == Status.off
  assert intent.status_for(0, 'on', port('off'), DT, "Run the turn signal check") == Status.unverified
  assert intent.why == "Run the turn signal check"
  check = int(TeslaFlags.CAR_SIGNALS | TeslaFlags.CAR_SIGNALS_CHECK)
  assert intent.status_for(check, 'off', port('checking'), DT) == Status.checking
  on = int(TeslaFlags.CAR_SIGNALS)
  for p in ('ready', 'requesting', 'lit', 'releasing'):
    assert intent.status_for(on, 'on', port(p), DT) == Status.ready
  assert intent.status_for(on, 'on', port('verifying'), DT) == Status.verifying
  assert intent.status_for(on, 'on', port('failed', fault='lampTimeout'), DT) == Status.failed
  statuses = [intent.status_for(on, 'on', port('waiting'), DT) for _ in range(int(CAR_SIGNAL_WAIT_S / DT) + 2)]
  assert statuses[0] == Status.waiting and statuses[-1] == Status.unavailable


def test_driver_signals_follows_status_not_brand():
  assert driver_signals_for(Status.off) and driver_signals_for(Status.failed) and driver_signals_for(Status.verifying)
  assert not driver_signals_for(Status.ready)


def test_port_signals_and_command_side():
  CP = structs.CarParams()
  CP.brand = 'tesla'
  assert not port_signals(CP)
  CP.flags = int(TeslaFlags.CAR_SIGNALS)
  assert port_signals(CP)
  CP.flags = int(TeslaFlags.CAR_SIGNALS | TeslaFlags.CAR_SIGNALS_CHECK)
  assert not port_signals(CP)
  CP.brand, CP.flags = 'hyundai', int(TeslaFlags.CAR_SIGNALS)
  assert not port_signals(CP)
  msg = messaging.new_message('turnpilotPlan')
  msg.turnpilotPlan.turnSignal.side = 'left'
  plan = msg.as_reader().turnpilotPlan
  assert command_side(plan) == LaneChange.none  # not ready
  msg.turnpilotPlan.turnSignal.status = 'ready'
  assert command_side(msg.as_reader().turnpilotPlan) == LaneChange.left
  assert command_side(None) == LaneChange.none


# ------------------------------------------------------------------------------------------- the record and the resolver

def fws():
  fw = structs.CarParams.CarFw(ecu='eps', address=0x730, subAddress=0, fwVersion=b'TeM3_E014p10_0.0.0 (16),E4H014.04.1')
  return [fw]


def record(**kw):
  r = {'onReason': 9, 'gear': 'P', 'onSeparates': False, 'vin': VIN, 'fw': fw_list(fws()), 'exteriorConfirmed': 1.}
  r.update(kw)
  return r


def test_vin_check_digit():
  assert vin_valid(VIN)
  assert not vin_valid("0" * 17)
  assert vin_valid(OTHER_VIN)
  assert not vin_valid(VIN[:8] + "5" + VIN[9:])  # wrong check digit
  assert not vin_valid("5YJ3E1EA7KF00000")


def test_record_matches():
  assert record_matches(record(), VIN, fws()) == (True, "")
  assert record_matches(None, VIN, fws())[1] == "Run the turn signal check"
  assert record_matches(record(), OTHER_VIN, fws())[1] == "Recorded for another car: run the check"
  other_fw = fws()
  other_fw[0].fwVersion = b'TeM3_E014p10_0.0.0 (16),E4H014.05.0'
  assert record_matches(record(), VIN, other_fw)[1] == "Car software changed: run the check again"
  assert not record_matches(record(), "0" * 17, fws())[0]
  assert not record_matches(record(sim=True), VIN, fws())[0]
  assert record_matches(record(sim=True, vin="0" * 17), "0" * 17, fws(), simulation=True)[0]
  assert record_matches(record(exteriorConfirmed=None), VIN, fws())[1] == "Confirm the outside lamps on the phone"


def test_resolver_modes():
  params = FakeParams(TurnPilotCarSignals='off')
  assert resolver(params)(VIN, fws()).mode == 'off'
  params = FakeParams(TurnPilotCarSignals='check')
  assert resolver(params)(VIN, fws()).mode == 'check'
  assert params['TurnPilotCarSignals'] == 'off'  # written back at once: the check lasts one ignition
  params = FakeParams(TurnPilotCarSignals='on', TurnPilotCarSignalsVerified=record())
  config = resolver(params)(VIN, fws())
  assert (config.mode, config.on_reason, config.recheck, config.on_separates) == ('on', 9, True, False)
  params = FakeParams(TurnPilotCarSignals='on', TurnPilotCarSignalsVerified=record(gear='D', onSeparates=True))
  config = resolver(params)(VIN, fws())
  assert not config.recheck and config.on_separates
  params = FakeParams(TurnPilotCarSignals='on')
  assert resolver(params)(VIN, fws()).mode == 'off'  # no record: off, said as unverified
  params = FakeParams(TurnPilotCarSignals='bogus')
  assert resolver(params)(VIN, fws()).mode == 'off'


def test_resolver_unknown_key():
  class Old(FakeParams):
    def get(self, key):
      raise KeyError(key)
  assert resolver(Old())(VIN, fws()).mode == 'off'


def test_promotion():
  result = {'pass': True, 'step': 'done', 'record': {'onReason': 8, 'gear': 'P', 'onSeparates': False}}
  check = check_record(result, VIN, fws())
  assert check['exterior'] == 'pending' and len(check['checkId']) == 8
  params = FakeParams(TurnPilotCarSignalsCheck=check, TurnPilotCarSignalsExterior='yes:stale')
  promote(params)
  assert 'TurnPilotCarSignalsVerified' not in params and params['TurnPilotCarSignalsCheck']['exterior'] == 'pending'
  params['TurnPilotCarSignalsExterior'] = f"yes:{check['checkId']}"
  params['TurnPilotCarSignals'] = 'on'
  config = resolver(params)(VIN, fws())
  assert config.mode == 'on' and config.on_reason == 8 and config.recheck
  assert params['TurnPilotCarSignalsVerified']['checkId'] == check['checkId']
  assert params['TurnPilotCarSignalsCheck']['exterior'] == 'confirmed'


def test_refusal():
  result = {'pass': True, 'step': 'done', 'record': {'onReason': 8, 'gear': 'P', 'onSeparates': False}}
  check = check_record(result, VIN, fws())
  params = FakeParams(TurnPilotCarSignalsCheck=check, TurnPilotCarSignalsExterior=f"no:{check['checkId']}", TurnPilotCarSignals='on')
  assert resolver(params)(VIN, fws()).mode == 'off'
  assert params['TurnPilotCarSignalsCheck']['exterior'] == 'refused'
  assert 'TurnPilotCarSignalsVerified' not in params


def test_failed_check_has_no_id():
  check = check_record({'pass': False, 'step': 'release', 'why': 'x'}, VIN, fws())
  assert 'checkId' not in check and 'exterior' not in check


def test_module_constants():
  assert cs_mod.LEAD_LONG_M == pytest.approx(300 * 0.3048) and cs_mod.LEAD_SHORT_M == pytest.approx(200 * 0.3048)
