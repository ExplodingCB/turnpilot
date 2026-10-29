"""Property tests for TurnPilot's commanded turn signal (DESIGN phase4/blinker 11.4), seeded random runs of the Tesla port
behind the real panda safety code (opendbc tests' PandaGate over libsafety) and BodyModel's car side, with DesireHelper
fed the carState blinkers the port publishes.

1. A car-commanded blinker never starts a lane change: DesireHelper enters preLaneChange only toward a side whose lamp a
   driver input caused, never from a lamp a request caused, never while the hazard button is on; with no driver input it
   never leaves off.
2. A request never outlives its want: within one stock period and a card cycle (and the release's own frames) of the
   intent ending, the port has no request out, and no request lasts past REQUEST_CAP_S.
3. No TX alters a non-turn field: every 0x3E9 the port sends differs from the stock frame it copies only in the request,
   reason, counter and checksum, and the panda accepts every release frame.
4. Never both sides, never the side opposite the maneuver, never a turn started after a veto (TurnSignalIntent).
5. The driver's lamp is never erased by a CANCEL the design could have withheld: none reaches the body while the
   driver's stalk is latched on the other side or the hazard button is on (after the lamps could show it), none after the
   released lamp read 0 twice and relit, at most one on a handover. A driver's same-side press absorbed during our request
   is the one accepted case: counted and reported, not hidden.
"""
import random
import types

import pytest

from opendbc.car import structs
from opendbc.car.car_signal import LEFT, NONE, REQUEST_CAP_S, RIGHT
from opendbc.car.tesla.body_model import SEMANTICS, BodyModel
from opendbc.car.tesla.car_signal import REQ_CANCEL, TeslaCarSignal
from opendbc.car.tesla.teslacan import body_controls_request
from opendbc.car.tesla.tests.panda_gate import PortHarness
from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom, log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.turnpilot.car_signals import TurnSignalIntent
from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
LaneChange = custom.TurnpilotPlan.LaneChange
Status = custom.TurnpilotPlan.TurnSignal.Status
FREE_BITS = set(range(8, 11)) | set(range(33, 38)) | set(range(52, 64))
GRACE_S = 0.3  # the lamps show a driver input on the next UI_warning frames: the port can't know it sooner
SEEDS = range(24)


def new_port():
  CP = structs.CarParams()
  CP.flags = int(TeslaFlags.CAR_SIGNALS)
  return TeslaCarSignal(CP)


class World:
  """Card, the panda and the car around the port, a DesireHelper on the published blinkers, and a driver."""

  def __init__(self, seed: int, semantics: str | None = None, driver: bool = True):
    self.rng = rng = random.Random(seed)
    self.semantics = semantics or rng.choice(SEMANTICS)
    body = BodyModel(semantics=self.semantics, on_latency=rng.uniform(0.02, 0.9), rng=rng, glitch_rate=rng.choice((0., 0.01)),
                     early_rate=rng.choice((38 / 2717, 0.05)), on_bits=rng.choice(('driver', 'any')))
    self.h = PortHarness(new_port(), body=body, rng=rng)
    self.h.engaged = True
    self.dh = DesireHelper()
    self.driver = driver
    self.v = rng.uniform(0., 35.)
    self.violations: list[str] = []
    self.absorbed_cancels = 0
    self.stalk_since = {LEFT: None, RIGHT: None}
    self.hazard_since: float | None = None
    self.pre_entries: list[tuple[float, int, set]] = []
    self.cancels_seen = 0
    self.frame = 0

  @property
  def body(self):
    return self.h.body

  def _driver_inputs(self):
    rng, t = self.rng, self.h.t
    if not self.driver:
      return
    r = rng.random()
    if r < 0.002:
      side = rng.choice((LEFT, RIGHT))
      on = not self.body.stalk[side]
      self.body.driver_stalk(side, on)
    elif r < 0.0025:
      self.body.hazards(not self.body.hazard_button)
    elif r < 0.003:
      self.body.driver_touch(rng.choice((LEFT, RIGHT)))
    for side in (LEFT, RIGHT):
      if self.body.stalk[side]:
        self.stalk_since[side] = self.stalk_since[side] if self.stalk_since[side] is not None else t
      else:
        self.stalk_since[side] = None
    if self.body.hazard_button:
      self.hazard_since = self.hazard_since if self.hazard_since is not None else t
    else:
      self.hazard_since = None

  def step(self):
    rng, h = self.rng, self.h
    self._driver_inputs()
    cancels_before = self.body.cancels_received
    stalk_before = dict(self.body.stalk)
    side_before = h.port.side
    h.step()
    self.frame += 1
    t = h.t
    # property 5: every CANCEL that reached the body, against what the driver had on
    if self.body.cancels_received > cancels_before:
      self.cancels_seen += 1
      for s in (LEFT, RIGHT):
        since = self.stalk_since[s]
        if stalk_before[s] and since is not None and t - since > GRACE_S:
          if s == side_before:
            self.absorbed_cancels += 1  # the driver's same-side press, absorbed under our request (rule 6)
          else:
            self.violations.append(f"{t:.2f} CANCEL erased the driver's {s} stalk ({self.semantics})")
      if self.hazard_since is not None and t - self.hazard_since > GRACE_S:
        self.violations.append(f"{t:.2f} CANCEL with the hazard button on ({self.semantics})")
    # DesireHelper at 20 Hz on the blinkers the port publishes (carState's)
    if self.frame % 5 == 0:
      _, left, right = h.blinkers[-1]
      torque = rng.choice((-2., 0., 0., 2.))
      cs = types.SimpleNamespace(vEgo=self.v, leftBlinker=left, rightBlinker=right, steeringPressed=torque != 0., steeringTorque=torque,
                                 leftBlindspot=False, rightBlindspot=False)
      prev = self.dh.lane_change_state
      self.dh.update(cs, h.engaged, 0., LaneChangeDirection.none)
      if prev == LaneChangeState.off and self.dh.lane_change_state == LaneChangeState.preLaneChange:
        side = LEFT if self.dh.lane_change_direction == LaneChangeDirection.left else RIGHT
        caused = self.body.caused_by(side)
        self.pre_entries.append((t, side, caused))
        if 'driver' not in caused or self.body.hazard_button:
          hazard = self.body.hazard_button
          self.violations.append(f"{t:.2f} preLaneChange {side} from a lamp caused by {caused} ({self.semantics}, {hazard=})")

  def run(self, seconds: float):
    for _ in range(int(round(seconds / 0.01))):
      self.step()

  def restart_card(self):
    """card dies and comes back: a fresh port, the body's lamps carried over, the panda's ownership timing out."""
    self.h.card_alive = False
    self.run(self.rng.uniform(0.1, 1.5))
    old = self.h
    self.h = PortHarness(new_port(), body=old.body, gate=old.gate, rng=self.rng, t0=old.t)
    self.h.engaged, self.h.want = old.engaged, NONE
    self.h.body.next_stock_t, self.h.body.next_ui_t = old.body.next_stock_t, old.body.next_ui_t


@pytest.mark.parametrize("seed", SEEDS)
def test_car_blinker_never_starts_a_lane_change(seed):
  w = World(seed)
  rng = w.rng
  w.run(rng.uniform(0.5, 2.))
  for _ in range(14):
    w.h.want = rng.choice((NONE, LEFT, RIGHT, LEFT, RIGHT))
    w.h.engaged = rng.random() < 0.85  # a gas takeover, a brake
    w.v = rng.uniform(0., 35.)
    w.h.cycle_jitter = rng.uniform(0., 0.12)
    w.run(rng.uniform(0.3, 8.))
    if rng.random() < 0.1:
      w.restart_card()
  w.h.want = NONE
  w.run(8.)
  assert not w.violations, w.violations[:5]


@pytest.mark.parametrize("seed", range(8))
def test_no_driver_input_no_lane_change(seed):
  w = World(seed, driver=False)
  rng = w.rng
  w.v = 25.
  w.run(1.)
  for _ in range(12):
    w.h.want = rng.choice((NONE, LEFT, RIGHT))
    w.h.engaged = rng.random() < 0.9
    w.run(rng.uniform(0.5, 6.))
    if rng.random() < 0.1:
      w.restart_card()
  assert not w.pre_entries and w.dh.lane_change_state == LaneChangeState.off, (w.semantics, w.pre_entries[:3])


@pytest.mark.parametrize("seed", range(12))
def test_requests_end_with_their_want(seed):
  w = World(seed, semantics='level', driver=False)
  rng, h = w.rng, w.h
  w.run(1.)
  for _ in range(10):
    h.want = rng.choice((LEFT, RIGHT))
    w.run(rng.uniform(1., 6.))
    h.want = NONE
    # one stock period and a card cycle to send the release, the release's frames (CANCEL, NONE, one retry) after it
    w.run(0.52 + 0.01 + 1.6)
    assert h.port.phase not in ('requesting', 'lit'), (seed, h.port.phase)
    sent = [(t, body_controls_request(d)[0]) for t, d, ok in h.sent if ok]
    assert not sent or sent[-1][1] not in (1, 2) or h.t - sent[-1][0] > 0.5
  # the cap: a want that never ends is released and faulted by REQUEST_CAP_S
  h.want = LEFT
  w.run(REQUEST_CAP_S + 2.)
  assert h.port.fault == 'capped' and h.port.phase != 'lit'


@pytest.mark.parametrize("seed", range(12))
def test_no_tx_alters_a_non_turn_field(seed):
  w = World(seed)
  rng, h = w.rng, w.h
  stocks = []
  orig_tick = h.body.tick

  def tick(t):
    stock, ui = orig_tick(t)
    for dat in stock:
      stocks.append((t, dat))
    # the DAS changes its high beams, wipers and heater bits now and then: the copies must follow
    if rng.random() < 0.01:
      h.body.stock[0] = rng.randrange(256)
      h.body.stock[2] = rng.randrange(256)
      h.body.stock[3] = rng.randrange(256)
    return stock, ui
  h.body.tick = tick
  for _ in range(12):
    h.want = rng.choice((NONE, LEFT, RIGHT))
    h.engaged = rng.random() < 0.9
    w.run(rng.uniform(0.3, 5.))
  for t_sent, dat, ok in h.sent:
    stock = next(d for t, d in reversed(stocks) if t <= t_sent + 1e-9 and d[6] >> 4 == ((dat[6] >> 4) - 1) % 16)
    diff = int.from_bytes(stock, 'little') ^ int.from_bytes(dat, 'little')
    assert {b for b in range(64) if diff >> b & 1} <= FREE_BITS, (stock.hex(), dat.hex())
    if body_controls_request(dat)[0] in (0, REQ_CANCEL):
      assert ok, dat.hex()


@pytest.mark.parametrize("seed", SEEDS)
def test_cancel_never_erases_the_drivers_lamp(seed):
  w = World(seed, semantics=random.Random(seed).choice(('level', 'level', 'latched', 'noneIgnored', 'tap')))
  rng, h = w.rng, w.h
  w.run(1.)
  for _ in range(16):
    h.want = rng.choice((NONE, LEFT, RIGHT, LEFT, RIGHT))
    h.engaged = rng.random() < 0.85
    w.run(rng.uniform(0.3, 6.))
  h.want = NONE
  w.run(6.)
  assert not w.violations, w.violations[:5]
  # reported, not hidden: CANCELs that met a driver's same-side press absorbed under our request
  absorbed = w.absorbed_cancels
  print(f"seed {seed} ({w.semantics}): {w.cancels_seen} CANCELs reached the body, {absorbed} met an absorbed same-side press")


def test_handover_sends_at_most_one_cancel():
  for seed in range(10):
    w = World(seed, semantics='latched', driver=False)
    h = w.h
    w.run(1.)
    h.want = LEFT
    w.run(3.)
    before = w.body.cancels_received
    h.engaged, h.want = False, NONE
    w.run(6.)
    assert w.body.cancels_received - before <= 1


# ------------------------------------------------------------------------------------------- 4: the intent

def maneuver(modifier):
  kind, side = classify("turn", modifier)
  return Maneuver(kind, side, "turn", modifier, 90.)


@pytest.mark.parametrize("seed", range(30))
def test_intent_never_both_sides_never_opposite_never_after_a_veto(seed):
  rng = random.Random(seed)
  intent = TurnSignalIntent()
  modifier = rng.choice(("left", "right"))
  m = maneuver(modifier)
  turn_side = LaneChange.left if modifier == "left" else LaneChange.right
  distance, v = rng.uniform(150., 400.), rng.uniform(5., 20.)
  d = types.SimpleNamespace(maneuver_id=0, started=False, done=False, turned_deg=0.)
  vetoed_at = None
  started_at = None
  block_from = rng.uniform(0., 25.)
  block = rng.choice(('otherSide', 'hazard', 'none'))
  t = 0.
  skipped = False
  changed_expected = False
  while distance > -5. and t < 60.:
    t += 0.05
    if not d.started and distance < 40.:
      d.started = True
      started_at = t
    if d.started and distance < 10.:
      d.turned_deg += 3.  # the car turning through the junction
    port = types.SimpleNamespace(status='lit', left=turn_side == LaneChange.left, right=turn_side == LaneChange.right,
                                 block=block if t >= block_from else 'none', fault='none', hazard=block == 'hazard' and t >= block_from,
                                 leftLamp=False, rightLamp=False)
    out = intent.update(status=Status.ready, port=port, gates_ok=True, engaged=True, lane_side=LaneChange.none, lane_pending=False,
                        maneuver=None if skipped else m, maneuver_id=0, distance=distance, desires=d, missed=False, v_ego=v, dt=0.05)
    assert out.side in (LaneChange.none, turn_side)  # never the opposite side, and one side only (an enum)
    if port.block != 'none' and started_at is not None and out.side == turn_side and vetoed_at is None:
      changed_expected = True  # the driver overruled the car after the turn started: finished, said
    if out.skip != 'none' and vetoed_at is None:
      vetoed_at = t
      skipped = True  # turnpilotd skips it: the maneuver is gone from here on
    if skipped:
      assert out.side == LaneChange.none or t == vetoed_at
    distance -= v * 0.05
  if vetoed_at is not None and started_at is not None:
    assert vetoed_at < started_at  # a turn that started after the veto never happens: the veto comes first
  if block != 'none' and started_at is not None and block_from > started_at:
    assert vetoed_at is None
  if changed_expected:
    assert intent.notice == 'changedMidTurn'
