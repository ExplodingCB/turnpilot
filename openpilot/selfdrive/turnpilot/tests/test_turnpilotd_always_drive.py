"""TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1, step S1): turnpilotd hands the switch every frame to the lane planner, the
unprotected left, the map controls, the turn skip and the stop-sign decider's Frame, which act on it from steps S2 to S5; it
does nothing in shadow mode, and a component made afresh mid-frame (a reset on a fault or a handback) has it at once."""
from unittest import mock

import pytest

from openpilot.selfdrive.turnpilot.stop_go import Frame
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim, stop_at
from openpilot.selfdrive.turnpilot.turnpilotd import TurnPilot


def parts(tp: TurnPilot) -> dict:
  return {'lanes': tp.lanes, 'paint_rule_lanes': tp.paint_rule_lanes, 'left': tp.left, 'map_controls': tp.map_controls,
          'skip': tp.skip}


def test_the_switch_is_off_by_default_everywhere():
  tp = TurnPilot()
  assert tp.always_drive is False and tp.always_drive_on is False
  assert all(part.always_drive is False for part in parts(tp).values())
  assert Frame().always_drive is False


@pytest.mark.parametrize("shadow", [False, True])
@pytest.mark.parametrize("on", [False, True])
def test_every_frame_hands_the_switch_on_and_never_in_shadow(on, shadow):
  sim = Sim([(400., 'turn', 'right')], shadow=shadow)
  sim.tp.always_drive = on
  sim.run(0.5)
  want = on and not shadow
  assert sim.tp.always_drive_on is want
  assert {k: part.always_drive for k, part in parts(sim.tp).items()} == dict.fromkeys(parts(sim.tp), want)
  sim.tp.always_drive = not on  # main()'s refresh flips it: the next frame has it everywhere
  sim.step(engaged=True)
  assert all(part.always_drive is ((not on) and not shadow) for part in parts(sim.tp).values())


def test_components_made_afresh_have_the_switch_at_once():
  sim = Sim([(400., 'turn', 'right')])
  sim.tp.always_drive = True
  sim.run(0.5)
  sim.tp.reset()  # a fault's or a turn-off's new components, before the next frame's hand-off
  assert all(part.always_drive is True for part in parts(sim.tp).values())


@pytest.mark.parametrize("on", [False, True])
def test_the_stop_sign_deciders_frame_carries_the_switch(on):
  sim = Sim([(1000., 'turn', 'right')], controls=[(200., 'stop')])
  sim.tp.always_drive = on
  frames = []
  real = sim.tp.stop_go.update

  def update(f, *a, **kw):
    frames.append(f)
    return real(f, *a, **kw)
  with mock.patch.object(sim.tp.stop_go, 'update', side_effect=update):
    stop_at(sim, 200.)
  assert frames and all(f.always_drive is on for f in frames)
