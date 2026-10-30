"""The sim's light oracle (light_oracle.py): which lights are ahead, and what they say; it waits for the level's signals
(core_trafficSignals isn't there until the level has loaded) and exits non-zero if they never come; a light the car has
just passed stays listed (build findings H2)."""
import sys
from types import SimpleNamespace

import pytest

from openpilot.tools.sim.bridge.beamng import light_oracle
from openpilot.tools.sim.bridge.beamng.light_oracle import (MAX_LIGHTS, PAST_M, light_state, lights_ahead, make_msg, stop_lines,
                                                            wait_for_dirs)


def test_states():
  assert [light_state(s) for s in ('greenTrafficLight', 'redTrafficLight', 'yellowTrafficLight', 'amber', None, 'off')] == \
    ['green', 'red', 'yellow', 'yellow', 'unknown', 'unknown']


def test_ahead_facing_nearest_first():
  lines = {'a': (0., 50.), 'b': (0., 120.), 'behind': (0., -20.), 'far': (0., 260.), 'side': (40., 60.), 'away': (0., 80.)}
  dirs = dict.fromkeys(lines, (0., 1.)) | {'away': (0., -1.)}
  assert lights_ahead((0., 0.), (0., 1.), lines, dirs) == [('a', 50.), ('b', 120.)]
  assert lights_ahead((0., 0.), (0., 1.), lines, {}) == []  # no direction from the game: not known to face the car
  many = {f'l{i}': (0., 10. * i + 5.) for i in range(10)}
  assert len(lights_ahead((0., 0.), (0., 2.), many, dict.fromkeys(many, (0., 1.)))) == MAX_LIGHTS


def test_message_and_the_committed_stop_lines():
  m = make_msg([('trafficLight 1', 42., 'red')]).turnpilotLights
  assert str(m.source) == 'simOracle' and (str(m.lights[0].state), m.lights[0].distance) == ('red', 42.)
  assert len(stop_lines()) > 100


def test_a_light_stays_listed_past_its_line():
  # BeamNG s14_red_right_frames: the car stood 1.3 m past trafficLight 62's line and the oracle dropped it, so the held red
  # went unknown; a camera there still sees the far-side head. Listed to PAST_M past its point (negative: behind), nearest first
  lines = {'past': (0., -1.3), 'gone': (0., -(PAST_M + 1.)), 'edge': (0., -PAST_M), 'a': (0., 50.), 'near': (0., 5.)}
  dirs = dict.fromkeys(lines, (0., 1.))
  assert lights_ahead((0., 0.), (0., 1.), lines, dirs) == [('past', -1.3), ('near', 5.), ('edge', -PAST_M), ('a', 50.)]
  assert PAST_M == 15.


def test_nearest_first_whichever_side_of_the_line():
  lines = {'p': (0., -14.), 'n': (0., 5.), 'm': (0., 30.), 'f': (0., 60.), 'g': (0., 90.)}
  assert [n for n, _ in lights_ahead((0., 0.), (0., 1.), lines, dict.fromkeys(lines, (0., 1.)))] == ['n', 'p', 'm', 'f']


class NotLoaded(Exception):
  """beamngpy's BNGValueError('core_trafficSignals not available'), before the level has loaded."""


class Game:
  def __init__(self, ready_after: int, dirs=True):
    self.calls, self.ready_after, self.dirs = 0, ready_after, dirs

  def get_state(self, name):
    self.calls += 1
    if self.calls <= self.ready_after:
      raise NotLoaded('core_trafficSignals not available')
    return {'state': 'green', 'dir': [0., 1., 0.]} if self.dirs else {'state': 'green'}


class Clock:
  def __init__(self):
    self.t = 0.

  def __call__(self):
    return self.t

  def sleep(self, s):
    self.t += s


def test_the_directions_are_read_again_until_the_level_has_loaded():
  # BeamNG ctl_ped_in_path (2026-10-30): the oracle read the directions once, before the bridge loaded the level (49 s),
  # and died; every light would then have been unknown, every signal held for the driver's go
  game, clock, said = Game(ready_after=5), Clock(), []
  dirs = wait_for_dirs(game.get_state, ['trafficLight 1', 'trafficLight 2'], timeout=120., retry=2., clock=clock, sleep=clock.sleep,
                       log=said.append)
  assert dirs == {'trafficLight 1': (0., 1.), 'trafficLight 2': (0., 1.)} and clock.t == 10. and said


def test_no_directions_in_time_gives_up():
  for game in (Game(ready_after=10 ** 6), Game(ready_after=0, dirs=False)):  # never loaded; loaded, no light with a direction
    clock = Clock()
    assert wait_for_dirs(game.get_state, ['trafficLight 1'], timeout=60., retry=2., clock=clock, sleep=clock.sleep, log=lambda m: None) \
      is None
    assert 60. <= clock.t <= 62.


def test_the_retry_outlasts_a_level_load():
  assert light_oracle.STARTUP_TIMEOUT_S >= 120.


def test_the_oracle_exits_non_zero_when_the_signals_never_come(monkeypatch):
  game, clock = Game(ready_after=10 ** 6), Clock()
  bng = SimpleNamespace(traffic_signals=SimpleNamespace(get_instance_state=game.get_state), disconnect=lambda: None)
  monkeypatch.setattr('beamngpy.BeamNGpy', lambda host, port: SimpleNamespace(open=lambda launch: bng))
  monkeypatch.setattr(light_oracle.messaging, 'PubMaster', lambda services: pytest.fail("published with no signals"))
  monkeypatch.setattr(light_oracle.time, 'monotonic', clock)
  monkeypatch.setattr(light_oracle.time, 'sleep', clock.sleep)
  monkeypatch.setattr(sys, 'argv', ['light_oracle.py'])
  with pytest.raises(SystemExit) as e:
    light_oracle.main()
  assert e.value.code == light_oracle.EXIT_NOT_READY != 0 and clock.t >= light_oracle.STARTUP_TIMEOUT_S
