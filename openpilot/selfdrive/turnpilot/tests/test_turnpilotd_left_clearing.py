"""The review of 2026-11-08 of a red read with an unprotected left past its line ('clearing the junction', Kellen 2026-11-08):
it cleared on a red only read late as the car rolled over the line (a light it never stood on a green or an unread stand
for), judged past the line by LINE_D (9 m) whatever the mapped signal's own line, and its go never looked at the cross
road, which has the green on our red. Now (unprotected_left: CLEAR_STAND_S, signal_line_m, cross_closing) a red first read
while rolling holds the car where its hold is, for the green; past the line is the mapped signal's own line; and the clearing
go waits for the cross lanes (no vehicle closing within the time the car needs to be out of the junction), holding in plain
words where it has no view of the cross road, never handed back. The probes: review2/probe_late_red_cam.py, probe_line7.py,
probe_flick.py, probe_cross.py."""
import pytest

from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
from openpilot.selfdrive.turnpilot.tests import test_turnpilotd_left as T
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import (JUNCTION, LEFT_25, TEST_RANGE_M, Desire, camera_signal_sim,
                                                                      road_runs, with_wide)
from openpilot.selfdrive.turnpilot.unprotected_left import CROSS_REASON, LIGHT_CHANGED_REASON, LIGHT_IN_JUNCTION_REASON

RED_UNTIL = 60.


def STREAM(sim):
  """camera_signal_sim's oncoming car every 3 s at 25 mph until 50 s."""
  return ((JUNCTION + 60. + 11.18 * (3. * (int(sim.t / 3.) + 1) - sim.t), 3.6),) if sim.t < 50. else ()


def signal_at(line_back, read, stream=False):
  """camera_signal_sim with the mapped signal's line line_back m before the junction, the camera reading read(t), the band
  empty (stream: STREAM's oncoming cars), a wide run every third frame without a road run (the cross road's view)."""
  if line_back == 8.5:
    sim = camera_signal_sim(read)
  else:
    sim = Sim(LEFT_25, controls=[(JUNCTION - line_back, 'signal')])
    sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
    sim.answers_prompt, sim.tp.oracle_ok, sim.tp.camera_mode = True, False, LIGHT_MODE_DEFAULT
    none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

    def lights(sim):
      state = read(sim.t)
      if state is not None and int(round(sim.t / DT_MDL)) % 2 == 0:
        sim.sm.put('turnpilotDetections', T.light_detections(state, JUNCTION + 15. - sim.s, shadow=False))
      return none
    sim.lights = lights
  sim.raw = with_wide(road_runs(STREAM if stream else (lambda sim: ())))
  return sim


def late_red(line_back, trigger_d, flick=None, flick_state=None, seconds=70.):
  """A red first read with the car trigger_d m from the junction on its roll-up (the light unread before), red until
  RED_UNTIL then green; flick: (t0, t1) of flick_state read in the red. The model stops for the red it sees. Returns
  (sim, plans, the red's first read time, the car's speed then)."""
  st = {'red': False, 't': None, 'v': None}
  holder = {}

  def read(t):
    sim = holder['sim']
    if not st['red'] and JUNCTION - sim.s <= trigger_d and t < RED_UNTIL:
      st['red'], st['t'], st['v'] = True, t, sim.v
    if t >= RED_UNTIL:
      return 'green'
    if st['red'] and flick is not None and flick[0] <= t < flick[1]:
      return flick_state
    return 'red' if st['red'] else None
  sim = signal_at(line_back, read)
  holder['sim'] = sim
  plans = []
  for _ in range(int(round(seconds / DT_MDL))):
    sim.model_holds = st['red'] and sim.t < RED_UNTIL and sim.s < JUNCTION
    turning = sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    plans.append(sim.step((), engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
  return sim, plans, st['t'], st['v']


def into_oncoming_t(sim):
  return next((i * DT_MDL for i, (s, _, _) in enumerate(sim.trace) if s > JUNCTION - ONCOMING_ENTRY_M), None)


@pytest.mark.parametrize('line_back, trigger_d', [(8.5, 8.0), (8.5, 7.0), (12., 8.0), (12., 11.0)])
def test_a_red_first_read_while_rolling_over_the_line_holds_for_the_green(line_back, trigger_d):
  """probe_late_red_cam, probe_line7: the red first read with the car rolling past the signal's line (1-4 m past it, at
  1.9-3.5 m/s). Until the review the cases past LINE_D (7-8 m from the junction) cleared the junction at 32 s, into the
  oncoming lanes at 34 s on a red that held to 60 s. Now the car holds where its hold is (the junction's mouth), short of
  the oncoming lanes, for the green, its line the light's wait, never a handback."""
  sim, plans, red_t, v_red = late_red(line_back, trigger_d)
  assert red_t is not None and v_red > 0.5  # read while rolling
  assert sim.trace[int(round(red_t / DT_MDL))][0] > JUNCTION - line_back  # past the mapped signal's own line
  red = plans[:int(round(RED_UNTIL / DT_MDL))]
  assert not any(str(p.controlPrompt) == 'leftGo' or p.desire == Desire.turnLeft for p in red)
  assert into_oncoming_t(sim) > RED_UNTIL
  assert not any(p.reason in (LIGHT_IN_JUNCTION_REASON, LIGHT_CHANGED_REASON) for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or str(p.unprotectedLeft.state) == 'undecided' for p in plans)
  assert any(p.desire == Desire.turnLeft for p in plans[int(round(RED_UNTIL / DT_MDL)):]) and sim.s > JUNCTION + 5.


@pytest.mark.parametrize('flick, flick_state', [((40., 40.2), 'green'), ((40., 41.), 'green'), ((40., 43.), 'unknown'),
                                                ((40., 43.), None)])
def test_a_flicker_in_a_red_first_read_rolling_arms_no_clearing(flick, flick_state):
  """probe_flick: the red first read rolling (held for the green), then the reader flickers: a green under a second (short
  of CAMERA_GREEN_STEADY_S), or 3 s unread or unknown. The next red read is the same red: the unread spell's stand never arms
  the clearing after a red this hold, only a steady green does. Held for the green throughout."""
  sim, plans, red_t, _ = late_red(8.5, 9.5, flick, flick_state)
  assert red_t is not None
  assert not any(str(p.controlPrompt) == 'leftGo' for p in plans[:int(round(RED_UNTIL / DT_MDL))])
  assert into_oncoming_t(sim) > RED_UNTIL
  assert not any(p.reason in (LIGHT_IN_JUNCTION_REASON, LIGHT_CHANGED_REASON) for p in plans)


def cross_cars(start, end, v=15., period=7.):
  """Cross-road cars from the right in the far half of the junction (3 m past the maneuver point) at v m/s, one every
  period s from start, the last starting before end: tracks, as the wide camera and the side views see them."""
  def objects(sim):
    out = []
    k = 0
    while start + k * period < end:
      y = -60. + v * (sim.t - (start + k * period))
      if sim.t >= start + k * period and -60. <= y <= 60.:
        out.append({'cls': 'car', 'x': JUNCTION + 3. - sim.s, 'y': y, 'vx': 0., 'vy': v, 'age': 40, 'trackId': 200 + k, 'conf': 0.8,
                    'camera': 'wide'})
      k += 1
    return tuple(out)
  return objects


def stood_then_red(objects=lambda sim: (), wide=True, red_at=40., seconds=40.):
  """camera_signal_sim: the light unread while the car waits in the junction's mouth from ~25 s (stood past CLEAR_STAND_S),
  red from red_at to RED_UNTIL, then green; the oncoming band clear from 50 s. Runs to red_at + seconds."""
  sim = camera_signal_sim(lambda t: None if t < red_at else 'red' if t < RED_UNTIL else 'green')
  if not wide:
    sim.raw = road_runs(lambda sim: ((JUNCTION + 60. + 11.18 * (3. * (int(sim.t / 3.) + 1) - sim.t), 3.6),) if sim.t < 50. else ())
  T.run_objects(sim, red_at)
  assert sim.s > JUNCTION - 8.5 + 1. and JUNCTION - sim.s > ONCOMING_ENTRY_M  # stood in the mouth, past the signal's line
  plans = []
  for _ in range(int(round(seconds / DT_MDL))):
    sim.model_holds = red_at <= sim.t < RED_UNTIL and sim.s < JUNCTION
    turning = sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    plans.append(sim.step(objects(sim), engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
  return sim, plans


def test_the_clearing_go_waits_for_the_cross_traffic_closing():
  """probe_cross: stood in the mouth, the red read, cross cars (which have the green) coming from the right every 7 s at
  15 m/s from 41 s to 55 s. Until the review the go came at 32 s on the oncoming band alone, a cross car through our path
  while the car was in the junction. Now the go waits while any is closing within the time the car needs to be out of the
  junction, saying so, and comes once the last has passed, never with one in the box; no handback."""
  objects = cross_cars(41., 55.)
  sim, plans = stood_then_red(objects)
  t0 = 40.
  go = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo')
  last_passed = 55. - 7. + 120. / 15.  # the last car (started 48 s) leaves the 60 m window at 56 s
  assert t0 + go * DT_MDL > last_passed - 2.
  assert any(p.reason == CROSS_REASON['traffic'] for p in plans[:go])
  assert t0 + go * DT_MDL < RED_UNTIL  # still cleared on the red, once the cross road was clear
  assert sim.s > JUNCTION + 5. and any(p.desire == Desire.turnLeft for p in plans[go:])
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or str(p.unprotectedLeft.state) == 'undecided' for p in plans)


def test_the_clearing_holds_in_the_mouth_with_no_view_of_the_cross_road():
  """No wide camera run (only the road camera's): the cross road unseen, the clearing holds the car in the mouth through the
  red, saying so in plain words, never a handback; it goes on the green."""
  sim, plans = stood_then_red(wide=False, seconds=30.)
  red = plans[:int(round((RED_UNTIL - 40.) / DT_MDL))]
  assert not any(str(p.controlPrompt) == 'leftGo' for p in red)
  assert any(p.reason == CROSS_REASON['view'] for p in red)
  assert into_oncoming_t(sim) > RED_UNTIL
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or str(p.unprotectedLeft.state) == 'undecided' for p in plans)
  assert sim.s > JUNCTION + 5.


def test_past_the_line_is_the_mapped_signals_own_line_never_line_d():
  """The review of 2026-11-08: past the line was judged from LINE_D (9 m before the junction) wherever the mapped signal's
  line was. A signal whose line lies 5 m before the junction, the left's hold in its mouth 6 m before it: the car stands short
  of that line, so a red read there (after a long unread stand) is a red at the line, held for the green, never a clearing
  of a junction it isn't in (LINE_D had it 3 m past)."""
  sim = signal_at(5., lambda t: None if t < 40. else 'red' if t < RED_UNTIL else 'green', stream=True)
  T.run_objects(sim, 40.)
  assert sim.v < 0.05 and sim.s < JUNCTION - 5.  # standing short of the signal's own line
  plans = []
  for _ in range(int(round(30. / DT_MDL))):
    sim.model_holds = 40. <= sim.t < RED_UNTIL and sim.s < JUNCTION
    turning = sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    plans.append(sim.step((), engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
  red = plans[:int(round((RED_UNTIL - 40.) / DT_MDL))]
  assert not any(str(p.controlPrompt) == 'leftGo' for p in red)
  assert not any(p.reason in (LIGHT_IN_JUNCTION_REASON, LIGHT_CHANGED_REASON) for p in plans)
  assert into_oncoming_t(sim) > RED_UNTIL and sim.s > JUNCTION + 5.
