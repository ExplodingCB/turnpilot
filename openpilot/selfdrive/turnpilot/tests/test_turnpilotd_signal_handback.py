"""A route turn's signal through TurnPilot's handback (review 2026-11-06), closed loop on test_turnpilotd's Sim: the stop for
want of a green a route turn makes at its junction's signal (map_controls: a route turn at a signal; driving_turn, update()'s
junction's third field) stays through a handback, as the stops in force do. With the handback in that field (round 1's
`driving and not alt_replaced`), the detector stopping 25 m short of the line dropped it (turnpilot.signal_turn_dropped
why=skipped) and the car, nothing read, drove on through the signal: a T-left with its desire held out at 4.3 m/s, a right (its
one pulse not out until 8 m) at 5.6 m/s. At night the camera reads no colour (1e: none on any of 667 governing heads), so that
stop is the only thing holding a turn at a red (1e 8:20.4, the right rolled toward a mapped signal's line until the driver
braked). And a take-up finds the stop in force: never skipped signalStop for a stop the handback let lapse. Only the turn in
hand at the handback: one that comes up while TurnPilot isn't acting (paused on a model fallback for the drive) is the
model's at its signal, as any straight one's, since no desire will ever go for it."""
from unittest import mock


from openpilot.cereal import log
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.handback import Handback
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import RIGHT_AT_A_CORNER, T_LEFT, Sim

Desire = log.Desire
JUNCTION = 300.
SIGNAL_S = 290.  # the mapped signal 10 m short of the junction, its line there (map_controls.CONTROL_LINE_M)


def events(event, name):
  return [c.kwargs for c in event.call_args_list if c.args[0] == name]


def through_a_handback(maneuvers, handback_from, back_at=None, cruise=10., limit=45.):
  """maneuvers with a mapped signal at SIGNAL_S, no light read by any source (night); openpilot engaged throughout, the
  detector stopped (TurnPilot hands back, openpilot steering on) from handback_from until back_at (None: for good). Runs
  until the car is 10 m past the junction or limit s. Returns (sim, the plans from the handback, the cloudlog events, the
  turn desire out at the handback)."""
  sim = Sim(maneuvers, cruise=cruise, controls=[(SIGNAL_S, 'signal')])
  sim.go_taps = False
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    while sim.s < handback_from:
      sim.step(engaged=True)
      assert sim.t < 120.
    desire_out = sim.tp.desires.started
    start = len(sim.trace)
    while sim.s < JUNCTION + 10. and sim.t < limit:
      sim.step(available=back_at is not None and sim.s >= back_at, engaged=True)
  return sim, [p for _, _, p in sim.trace[start:]], event, desire_out


def stood_at_the_line(sim):
  return sim.v < 0.05 and SIGNAL_S - 1.5 < sim.s < SIGNAL_S + 0.5


def test_a_skipped_turn_still_goes_straight_through_its_signal_through_a_handback():
  # the field's other half unchanged: a turn TurnPilot won't take (skipped) has no stop for want of a green, handback or not:
  # its signal is the model's, as any straight one's (a camera red still stops it)
  sim = Sim(RIGHT_AT_A_CORNER, cruise=10., controls=[(SIGNAL_S, 'signal')])
  sim.go_taps = False
  while sim.s < 150.:
    sim.step(engaged=True)
  rf = sim.tp.follower
  sim.tp.skip.skip(rf.upcoming, rf.distance, rf.maneuver_id, sim.v, 'laneBlocked')
  with mock.patch.object(turnpilotd.cloudlog, 'event'):
    while sim.s < JUNCTION + 10. and sim.t < 60.:
      sim.step(available=sim.s < 265., engaged=True)
  assert sim.s >= JUNCTION + 10. and not any(p.shouldStop for _, _, p in sim.trace)



def test_a_turn_that_comes_up_while_turnpilot_is_paused_is_the_models_at_its_signal():
  # the big model gone 200 m before the route's first turn (a model fallback: TurnPilot paused for the drive, openpilot engaged
  # again on the small model, map_update still run): the turn after it, at a mapped signal, comes up with TurnPilot never
  # acting for it, so no desire goes and the car drives straight on through its signal on the model, nothing read, never
  # stood there for want of a green. With every turn of the route the turn's (round 2's first cut, `not skipped` alone) the
  # car stood at that signal, a take-control said, for a turn TurnPilot would never pulse; and the turn in hand at the
  # fallback keeps its signal's stop (the tests above)
  route = [(300., "turn", "right", junction_of("right")), (800., "turn", "left", T_LEFT), (2000., "turn", "right", junction_of("right"))]
  sim = Sim(route, cruise=10., controls=[(790., 'signal')])
  sim.go_taps, sim.big = False, True
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    while sim.s < 100.:
      sim.step(engaged=True)
    sim.big = False  # modeld fell back
    while sim.s < 820. and sim.t < 150.:
      sim.step(engaged=True)
  assert sim.tp.fell_back and sim.trace[-1][2].handback == Handback.modelFellBack
  assert sim.s >= 820., (round(sim.s, 1), round(sim.t, 1))
  assert not any(p.shouldStop for s, _, p in sim.trace if s > 700.)
  assert not events(event, 'turnpilot.signal_turn'), "never the turn's signal"
  assert not any(p.desire == Desire.turnLeft for _, _, p in sim.trace)
