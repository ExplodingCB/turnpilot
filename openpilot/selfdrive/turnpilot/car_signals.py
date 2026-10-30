"""TurnPilot's turn signal: which side to signal and when, and who signals it, the car or the driver.

DESIGN phase4/blinker (2026-10-28). On a Tesla verified by the lamp check (opendbc tesla/car_signal_check.py) the car
works its own turn signal: TurnSignalIntent's side reaches controlsd through turnpilotPlan.turnSignal, controlsd puts it
in CarControl's blinkers, and the Tesla port sends DAS_bodyControls and masks its own lamp out of the driver's blinker
(opendbc car_signal.py). Everywhere else, and whenever the car's signal isn't ready, the driver signals, and every armed
turn prompts "Signal Left/Right" under 20 mph with no lane change pending (STOP_SIGNS D15's holds: above DesireHelper's
LANE_CHANGE_SPEED_MIN a one-sided driver's blinker is a lane change on a nudge).

The legal distance (IC 9-21-8-25): a turn or lane change is signalled continuously for the last 200 ft, 300 ft where the
limit is 50 mph or more. The car doesn't know its road's limit unless navd says (navInstruction.speedLimit), so 300 ft
is the default. A turn arms at that lead plus ARM_LAG_S of travel, the longest the lamp can take to light, and its
signal is held through stops, the unprotected left's wait and the go from a line until the turn is done (cleared at its
angle, or passed as the route follower counts it), skipped, missed, rerouted, handed back or disengaged; one whose desire
never went out also ends PASSED_DIST_M past its junction (passed straight). Only a handback or a disengagement, with the
maneuver still ahead, tells the driver to signal it ("Signal Off", handedOver), and never while openpilot steers at 20 mph
or more; a turn the maneuver ended (done, skipped, passed, missed) never does.

With the car signalling, the driver overrules it before the turn starts, or while a TurnPilot stop brings the car to
rest short of the junction, braking toward it or standing at it, or with the car at rest short of it whatever stopped it
(a lead in the queue: turnpilotd's at_rest), or while the stop-sign decider's go can still be withdrawn (going, short of
its commit point: the decider withdraws it on the veto a frame later), under TURN_STARTED_DEG into the turn (their lamp on
the other side, hazards, or our lamp put out): the turn is skipped out loud and navd reroutes (and stays skipped at its
junction across a reroute: turnpilotd's carry_veto), except at the unprotected left's hold line and in
its go while the car can still stop short of the oncoming lanes, where the left hands back (a skip there would let go of
the car in a left-turn position; the go is withdrawn) and the request for the left ends there: never signalled again,
prompted or handed over for that maneuver, nor for the same left there under a new id (navd's reroute: turnpilotd's
carry_veto, end()). Once the turn has started (moving into it with its desire out and no stop in
force, or TurnPilot's go from a standstill out into it and not spent) it finishes, said ("Turn Signal Changed"), and a
stop that then has the car never skips it: one veto, one decision per maneuver. A wrongSide fault with the other side's
lamp lit (ours on the wrong side, or the driver's) is a veto too: the car never turns with the other side lit.
"""
import datetime
import json
import secrets
from dataclasses import dataclass

from opendbc.car.car_signal import CarSignalsConfig
from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from openpilot.selfdrive.turnpilot.route_follower import LEFT, PASSED_DIST_M, TURN, KEEP

LaneChange = custom.TurnpilotPlan.LaneChange
TurnSignal = custom.TurnpilotPlan.TurnSignal
Status = TurnSignal.Status

LEAD_SHORT_M = 60.96        # 200 ft
LEAD_LONG_M = 91.44         # 300 ft, where the limit is 50 mph or more, and whenever the limit isn't known
LIMIT_LONG_MS = 22.35       # 50 mph
ARM_LAG_S = 1.41            # the next stock frame (0.51 s), ON latency (0.5 s), UI_warning (0.1 s), margin (0.3 s)
CAR_SIGNAL_WAIT_S = 15.     # mode on, no stock body frame this long: the car can't signal
TRAVEL_BACKSTOP_M = 200.    # an armed turn's signal travels no further than this past its arming
HANDED_OVER_S = 3.
NOTICE_S = 3.
LAMP_GAP_S = 0.35           # opendbc car_signal.LAMP_GAP_S: the longest dark run a held lamp shows
TURN_STARTED_DEG = 15.      # desire_timing.TURN_STARTED_DEG
READY_PORT = ('ready', 'requesting', 'lit', 'releasing')
MODES = ('off', 'check', 'on')
SIDE_NAME = {LaneChange.left: 'left', LaneChange.right: 'right'}


def lead_m(limit_ms: float, v_ego: float) -> float:
  """m a signal must lead the turn or change by: 200 ft only where the approach road's mapped limit is known and under
  50 mph and the car is under it, else 300 ft."""
  if 0. < limit_ms < LIMIT_LONG_MS and v_ego < LIMIT_LONG_MS:
    return LEAD_SHORT_M
  return LEAD_LONG_M


def maneuver_side(maneuver) -> int:
  """The LaneChange side of a route maneuver (route_follower: LEFT 1, RIGHT -1)."""
  if maneuver is None or maneuver.side == 0:
    return LaneChange.none
  return LaneChange.left if maneuver.side == LEFT else LaneChange.right


def other_side(side: int) -> int:
  return LaneChange.right if side == LaneChange.left else LaneChange.left


def port_signals(CP) -> bool:
  """Whether this car's port works its own turn signal this drive (mode on with a verified record, not the check)."""
  flags = int(CP.flags)
  return CP.brand == 'tesla' and bool(flags & TeslaFlags.CAR_SIGNALS) and not flags & TeslaFlags.CAR_SIGNALS_CHECK


def car_signals_now(plan) -> bool:
  """Whether the car signals TurnPilot's maneuvers itself this frame (turnpilotPlan.turnSignal.status == ready)."""
  return plan is not None and plan.turnSignal.status == Status.ready


def command_side(plan) -> int:
  """The side controlsd commands: the plan's turn signal side while the car's signal is ready, else none."""
  return plan.turnSignal.side if car_signals_now(plan) else LaneChange.none


def car_blinkers(CP, enabled: bool, model_v2, sm) -> tuple[bool, bool]:
  """controlsd's CarControl blinkers. On a car whose port works its own turn signal for TurnPilot (port_signals: a
  verified Tesla), only TurnPilot's turn signal while it is ready and openpilot is engaged, and none from a stale,
  invalid, shadow or disabled plan: DesireHelper's own lane changes are the driver's, signalled by the driver (a car
  blinker on the side the driver signalled would hide their lamp and loop). Every other car: DesireHelper's lane change,
  as stock openpilot."""
  from openpilot.cereal import log
  from openpilot.selfdrive.turnpilot.hooks import active_plan
  if port_signals(CP):
    side = command_side(active_plan(sm)) if enabled else LaneChange.none
    return side == LaneChange.left, side == LaneChange.right
  if model_v2.meta.laneChangeState != log.LaneChangeState.off:
    return (model_v2.meta.laneChangeDirection == log.LaneChangeDirection.left,
            model_v2.meta.laneChangeDirection == log.LaneChangeDirection.right)
  return False, False


def driver_signals_for(status) -> bool:
  """The driver signals (prompted) unless the car's own signal is ready: every brand, every mode."""
  return status != Status.ready


# ------------------------------------------------------------------------------------------- the verified record (card)

def vin_valid(vin: str) -> bool:
  """A 17-character VIN with a valid North American check digit (position 9), never all zeros."""
  if not isinstance(vin, str) or len(vin) != 17 or set(vin) <= {'0'}:
    return False
  values = {**{str(d): d for d in range(10)}, **dict(zip("ABCDEFGH", range(1, 9), strict=True)),
            **dict(zip("JKLMN", range(1, 6), strict=True)), 'P': 7, 'R': 9, **dict(zip("STUVWXYZ", range(2, 10), strict=True))}
  weights = (8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2)
  try:
    total = sum(values[c] * w for c, w in zip(vin.upper(), weights, strict=True))
  except KeyError:
    return False
  check = total % 11
  return vin[8].upper() == ('X' if check == 10 else str(check))


def fw_list(car_fw) -> list:
  return sorted([[str(f.ecu), int(f.address), int(f.subAddress), bytes(f.fwVersion).hex()] for f in car_fw])


def record_matches(record, vin: str, car_fw, simulation: bool = False) -> tuple[bool, str]:
  """Whether a TurnPilotCarSignalsVerified record is this car's, and why not."""
  if not record:
    return False, "Run the turn signal check"
  if record.get('sim') and not simulation:
    return False, "A simulator record: run the check on the car"
  if not record.get('sim') and not vin_valid(vin):
    return False, "Car's VIN unreadable: can't match the record"
  if record.get('vin') != vin:
    return False, "Recorded for another car: run the check"
  if record.get('fw') != fw_list(car_fw):
    return False, "Car software changed: run the check again"
  if not record.get('exteriorConfirmed'):
    return False, "Confirm the outside lamps on the phone"
  return True, ""


def wall_time() -> str:
  """When a check ran or was confirmed, for the record (a date, not a clock to time anything by)."""
  return datetime.datetime.now(datetime.UTC).isoformat(timespec='seconds')


def read_mode(params) -> str:
  try:
    mode = params.get("TurnPilotCarSignals")
  except Exception:  # UnknownKeyName: a params library that predates the key
    cloudlog.error("turnpilot: TurnPilotCarSignals unknown to this build's params, car signals off; run scons")
    return 'off'
  if mode is None:
    return 'off'
  if mode not in MODES:
    cloudlog.error(f"turnpilot: TurnPilotCarSignals {mode!r} isn't off, check or on: car signals off")
    return 'off'
  return mode


def promote(params) -> None:
  """A passed check whose outside lamps Kellen confirmed on the phone becomes the verified record; a refusal is noted."""
  try:
    check = params.get("TurnPilotCarSignalsCheck")
    answer = params.get("TurnPilotCarSignalsExterior")
  except Exception:
    return
  if not check or not check.get('pass') or check.get('exterior') != 'pending' or not answer:
    return
  verdict, _, check_id = str(answer).partition(':')
  if check_id != check.get('checkId'):
    return  # an answer to another check: nothing
  if verdict == 'yes':
    record = {**check['record'], 'vin': check.get('vin'), 'fw': check.get('fw'), 'checkId': check_id,
              'exteriorConfirmed': wall_time(), 'when': check.get('when'), 'opendbc': check.get('opendbc', '')}
    if check.get('sim'):
      record['sim'] = True
    params.put("TurnPilotCarSignalsVerified", record)
    check['exterior'] = 'confirmed'
    cloudlog.warning(f"turnpilot: car signals verified for {check.get('vin')}, check {check_id}")
  elif verdict == 'no':
    check['exterior'] = 'refused'
    check['why'] = "outside lamps not seen"
    cloudlog.error(f"turnpilot: car signal check {check_id} refused: outside lamps not seen")
  else:
    return
  params.put("TurnPilotCarSignalsCheck", check)
  params.remove("TurnPilotCarSignalsExterior")


def resolver(params, simulation: bool = False):
  """card's callback for get_car: (vin, car_fw) -> CarSignalsConfig, read once per drive."""
  def resolve(vin: str, car_fw) -> CarSignalsConfig:
    promote(params)
    mode = read_mode(params)
    if mode == 'off':
      return CarSignalsConfig()
    if mode == 'check':
      params.put("TurnPilotCarSignals", "off")  # the check never outlasts this ignition
      cloudlog.warning("turnpilot: car signal check on this ignition; TurnPilotCarSignals back to off")
      return CarSignalsConfig(mode='check')
    try:
      record = params.get("TurnPilotCarSignalsVerified")
    except Exception:
      record = None
    ok, why = record_matches(record, vin, car_fw, simulation)
    if not ok:
      cloudlog.error(f"turnpilot: car signals on but not verified for this car: {why}")
      return CarSignalsConfig()
    return CarSignalsConfig(mode='on', on_reason=int(record.get('onReason', 8)), recheck=record.get('gear') == 'P',
                            on_separates=bool(record.get('onSeparates')))
  return resolve


def check_record(result: dict, vin: str, car_fw, sim: bool = False) -> dict:
  """What card writes to TurnPilotCarSignalsCheck from a finished check (opendbc LampCheck.result)."""
  out = {**result, 'vin': vin, 'fw': fw_list(car_fw), 'when': wall_time()}
  if result.get('pass'):
    out['checkId'] = secrets.token_hex(4)
    out['exterior'] = 'pending'
  if sim:
    out['sim'] = True
  return out


# ------------------------------------------------------------------------------------------- the intent (turnpilotd)

@dataclass
class TurnSignalOut:
  status: int = Status.off
  side: int = LaneChange.none
  lead_m: float = LEAD_LONG_M
  lit_m: float = 0.
  handed_over: int = LaneChange.none
  prompt: int = LaneChange.none
  skip: str = 'none'          # a veto or side uncertainty: turnpilotd skips the turn ahead with this reason
  left_veto: str = ''         # at the unprotected left's hold line: the veto is its handback, not a skip
  notice: str = 'none'
  arm_m: float = 0.
  why: str = ''
  signal_lit: bool = False    # the armed turn's side lit with the other dark (the unprotected left's go)

  def fill(self, ts) -> None:
    ts.status, ts.side, ts.leadM, ts.litM = self.status, self.side, float(self.lead_m), float(self.lit_m)
    ts.handedOver, ts.prompt, ts.skip, ts.notice = self.handed_over, self.prompt, self.skip, self.notice
    ts.armM, ts.why = float(self.arm_m), self.why


class TurnSignalIntent:
  """One per turnpilotd, updated every frame after the lane planner."""

  def __init__(self):
    self.waiting_s = 0.
    self.status = Status.off
    self.prev_status = Status.off
    self.armed_id = None
    self.arm_side = LaneChange.none
    self.arm_m = 0.
    self.arm_travel = 0.
    self.ended_id = None
    self.side = LaneChange.none
    self.lit_m = 0.
    self.logged_id = None
    self.handover_side = LaneChange.none
    self.handed_over = LaneChange.none
    self.handed_t = 0.
    self.notice = 'none'
    self.notice_t = 0.
    self.changed_id = None
    self.dark_s = {LaneChange.left: 99., LaneChange.right: 99.}
    self.why = ''

  # ------------------------------------------------------------------ status (DESIGN 7.3)

  def status_for(self, flags: int, mode: str, port, dt: float, unverified_why: str = '') -> int:
    if flags & TeslaFlags.CAR_SIGNALS_CHECK:
      self.why = ''
      return Status.checking
    if flags & TeslaFlags.CAR_SIGNALS:
      p = str(port.status)
      if p == 'failed':
        self.why = f"{port.fault}"
        return Status.failed
      if p == 'verifying':
        self.why = str(port.checkWhy)
        return Status.verifying
      if p in READY_PORT:
        self.waiting_s, self.why = 0., ''
        return Status.ready
      self.waiting_s += dt
      if self.waiting_s >= CAR_SIGNAL_WAIT_S:
        self.why = "No body-control message"
        return Status.unavailable
      self.why = ''
      return Status.waiting
    self.why = unverified_why if mode == 'on' else ''
    return Status.unverified if mode == 'on' else Status.off

  # ------------------------------------------------------------------ the frame

  def update(self, *, status, port, gates_ok: bool, engaged: bool, lane_side: int, lane_pending: bool, maneuver, maneuver_id,
             distance: float, desires, missed: bool, v_ego: float, dt: float, limit_ms: float = 0.,
             left_lamp: bool = False, right_lamp: bool = False, at_left_line: bool = False, standing: bool = False,
             stopping: bool = False, go_now: bool = False, skipped: bool = False, passed: bool = False) -> TurnSignalOut:
    """status: this frame's TurnSignal.Status; port: carState.carSignal; gates_ok: TurnPilot acting, not shadow, no fault;
    engaged: carControl.enabled; lane_side: the lane planner's side from its warning to the change's end; lane_pending: a
    change is wanted or offered for this maneuver; maneuver: the turn TurnPilot is taking (desire_on), None once skipped,
    dropped or handed back; desires: DesireTiming; missed: turnMissed; left_lamp/right_lamp: the lamps as UI_warning shows
    them (ours and the driver's); at_left_line: the unprotected left stands at its hold line; standing: the car stands
    held at a line; stopping: a TurnPilot stop in force brings the car to rest at or short of the junction (shouldStop
    at its end: a hold, a red, a stop sign), braking toward it or standing at it, or the car stands at rest short of it,
    whatever stopped it, with no go of TurnPilot's out into the turn, or the stop-sign decider's go can still be withdrawn
    (turnpilotd's short_of_turn); go_now: TurnPilot's go from a line this
    frame; skipped: TurnSkip has skipped this maneuver (too fast, lane unreachable, side uncertain, a veto); passed: the
    route follower counts it passed and navd hasn't moved on."""
    out = TurnSignalOut(status=status)
    self.prev_status, self.status = self.status, status
    lead = out.lead_m = lead_m(limit_ms, v_ego)
    lamps = {LaneChange.left: left_lamp, LaneChange.right: right_lamp}
    for s in (LaneChange.left, LaneChange.right):
      self.dark_s[s] = 0. if lamps[s] else self.dark_s[s] + dt

    # the turn: arm, hold, end (DESIGN 6.2)
    m_side = maneuver_side(maneuver)
    signals_turn = maneuver is not None and maneuver.kind in (TURN, KEEP) and m_side != LaneChange.none
    d_this = desires.maneuver_id == maneuver_id
    ended = ''
    if self.armed_id is not None:
      # the maneuver's own ends first (done, skipped, passed, missed): never handed over (DESIGN 7.7), whatever else
      # happened on the same frame; on the follower's pass frame DesireTiming is already done for a turn the car took
      if maneuver_id != self.armed_id:
        ended = 'maneuver changed'
      elif d_this and desires.done:
        ended = 'done'
      elif skipped:
        ended = 'skipped'
      elif passed:
        ended = 'passed'
      elif distance < -PASSED_DIST_M and not standing and not (d_this and (desires.started or desires.turned_deg >= TURN_STARTED_DEG)):
        # no desire out and past the junction as the route follower counts a pass: distance is navd's nearest reading,
        # up to UNCERTAIN_M short of the truth (never while held at a line: a reading past it isn't driving past). A turn
        # whose desire is out ends only done, missed or passed on, never as the car pulls into it from a line
        ended = 'passed without turning'
      elif missed:
        ended = 'missed'
      elif not gates_ok or not engaged:
        ended = 'disengaged'  # openpilot disengaged, or TurnPilot handed back: the maneuver still ahead
      elif maneuver is None:
        ended = 'dropped'
      elif self.arm_travel > TRAVEL_BACKSTOP_M:
        ended = 'travel backstop'
        cloudlog.event("turnpilot.turn_signal_backstop", error=True, arm_m=round(self.arm_m, 1), travel=round(self.arm_travel, 1))
        self._notice('ranPast')
      if ended:
        if ended == 'disengaged' and self.side == self.arm_side and self._lit_ours(port, self.arm_side) and status == Status.ready:
          self.handover_side = self.arm_side
        if ended not in ('done',) and self.lit_m > 0.:
          cloudlog.event("turnpilot.turn_signal_ended", error=ended in ('dropped', 'passed without turning'),
                         why=ended, lit_m=round(self.lit_m, 1), arm_m=round(self.arm_m, 1))
        if ended in ('done', 'passed without turning', 'missed', 'travel backstop'):
          self.ended_id = self.armed_id  # never armed again for this maneuver
        self.armed_id, self.arm_side = None, LaneChange.none
    if self.armed_id is None and signals_turn and gates_ok and engaged and maneuver_id != self.ended_id:
      if distance <= lead + v_ego * ARM_LAG_S or (d_this and desires.started):
        self.armed_id, self.arm_side, self.arm_travel = maneuver_id, m_side, 0.
        self.arm_m = max(distance, 0.)
    if self.armed_id is not None:
      self.arm_travel += v_ego * dt
      out.arm_m = self.arm_m

    # the driver overrules the car's signal (DESIGN 6.6): before the turn starts, or while a stop brings the car to rest
    # short of the junction (braking toward it or standing at it) or it stands at rest there whatever stopped it, under
    # TURN_STARTED_DEG into the turn (its desire out from the approach or not), a skip out loud; at the unprotected left's
    # line or in its go while still withdrawable (at_left_line), its handback, which ends the request
    # for the left from this frame (never claimed again once the driver's lamp is out, never handed over); once started
    # (moving into it with its desire out and no stop in force, or TURN_STARTED_DEG into it), finished and said. Decided
    # once per maneuver: a turn said to be finishing is never skipped after (a stop that then has the car, the lamp on)
    started = d_this and desires.started and (not (standing or stopping) or desires.turned_deg >= TURN_STARTED_DEG)
    turn_side = self.arm_side if lane_side == LaneChange.none else LaneChange.none
    if turn_side != LaneChange.none and self.armed_id is not None:
      veto = self._veto(port, status, turn_side, lamps)
      if veto:
        if at_left_line:
          out.left_veto = 'driverSignal'
          cloudlog.event("turnpilot.turn_signal_ended", why='vetoed at the left line', veto=veto, lit_m=round(self.lit_m, 1),
                         arm_m=round(self.arm_m, 1))
          self.ended_id = self.armed_id  # never armed again for this maneuver
          self.armed_id, self.arm_side = None, LaneChange.none
        elif started or self.changed_id == self.armed_id:
          if self.changed_id != self.armed_id:
            self.changed_id = self.armed_id
            self._notice('changedMidTurn')
            cloudlog.event("turnpilot.turn_signal_changed", why=veto, lit_m=round(self.lit_m, 1))
        else:
          out.skip = veto

    # the side, whoever signals it: a lane change outranks the turn (usually the same side, into the turn lane)
    side = LaneChange.none
    if gates_ok and engaged:
      side = lane_side if lane_side != LaneChange.none else self.arm_side
    if side != self.side:
      self.lit_m = 0.
    self.side = out.side = side

    # metres continuously lit (DESIGN 6.2), restarting whenever the port leaves lit
    if status == Status.ready and side != LaneChange.none and str(port.status) == 'lit' and self._lit_ours(port, side):
      self.lit_m += v_ego * dt
    else:
      self.lit_m = 0.
    out.lit_m = self.lit_m

    # at the turn: the signal's lead logged, an error when short of the legal distance
    if self.armed_id is not None and self.logged_id != self.armed_id and status == Status.ready and \
       (go_now or (d_this and desires.turned_deg >= TURN_STARTED_DEG)):
      self.logged_id = self.armed_id
      short = self.lit_m < lead
      cloudlog.event("turnpilot.turn_signal", error=short, lit_m=round(self.lit_m, 1), lead_m=round(lead, 1),
                     arm_m=round(self.arm_m, 1))

    # the driver signals instead (DESIGN 6.5): every armed turn while the car isn't signalling, under D15's holds; never
    # one this frame's veto skips (our lamp put out fails the port on the veto's own frame)
    if status != Status.ready and self.armed_id is not None and gates_ok and out.skip == 'none':
      own_lit = lamps[self.arm_side] and not lamps[other_side(self.arm_side)]
      if v_ego < LANE_CHANGE_SPEED_MIN and not lane_pending and not own_lit:
        out.prompt = self.arm_side

    # the unprotected left's go: the turn's side lit (ours or the driver's) with no long dark run, the other side dark
    if self.arm_side != LaneChange.none:
      out.signal_lit = self.dark_s[self.arm_side] <= LAMP_GAP_S and self.dark_s[other_side(self.arm_side)] > LAMP_GAP_S

    # "Signal Off": a lit car signal ended by a disengagement or handback with its maneuver ahead, once the release is
    # done and the mask is down, unless the driver's lamp on that side is already lit. Never while openpilot steers at
    # 20 mph or more (a TurnPilot handback): a driver's lamp rising there puts DesireHelper in preLaneChange, a lane
    # change on a nudge; the handback's own alert says take control
    if self.handover_side != LaneChange.none and str(port.status) in ('ready', 'failed') and not (port.left or port.right):
      if engaged and v_ego >= LANE_CHANGE_SPEED_MIN:
        cloudlog.event("turnpilot.turn_signal_handover_withheld", v_ego=round(v_ego, 2), side=int(self.handover_side))
      elif not lamps[self.handover_side]:
        self.handed_over, self.handed_t = self.handover_side, HANDED_OVER_S
      self.handover_side = LaneChange.none
    if self.handed_t > 0.:
      self.handed_t -= dt
      out.handed_over = self.handed_over
    if self.notice_t > 0.:
      self.notice_t -= dt
      out.notice = self.notice
    out.why = self.why
    return out

  def end(self, maneuver_id) -> None:
    """The request for maneuver_id is over from this frame: disarmed if armed for it, never armed again for it, and never
    handed over (no "Signal Off"). turnpilotd's carry_veto: a veto at the unprotected left's line carried to the same left
    under a new maneuver id (navd's reroute), whose request the veto ended."""
    self.ended_id = maneuver_id
    if self.armed_id == maneuver_id:
      self.armed_id, self.arm_side = None, LaneChange.none

  def _notice(self, notice: str) -> None:
    self.notice, self.notice_t = notice, NOTICE_S

  @staticmethod
  def _lit_ours(port, side: int) -> bool:
    return bool(port.left) if side == LaneChange.left else bool(port.right)

  def _veto(self, port, status, side: int, lamps) -> str:
    fault = str(port.fault)
    if status == Status.ready:
      block = str(port.block)
      if block == 'hazard' or port.hazard:
        return 'hazards'
      if block == 'otherSide':
        return 'driverSignal'
      if block == 'inherited' and lamps[other_side(side)]:
        return 'driverSignal'
    if fault == 'lampLost' and self.prev_status == Status.ready and status == Status.failed:
      return 'signalLost'  # our lamp put out: how a driver's touch of the stalk looks (the skip latches on this frame)
    if fault == 'wrongSide' and status == Status.failed and lamps[other_side(side)]:
      # the other side lit in the request's first second: ours on the wrong side, which may stay lit, or the driver's
      # press the other way (DESIGN 5.2 rule 3). Either way the car never turns with the other side lit (7.6)
      return 'hazards' if port.hazard else 'driverSignal'
    return ''


def check_result_json(result: dict | None) -> str:
  return json.dumps(result) if result is not None else ''
