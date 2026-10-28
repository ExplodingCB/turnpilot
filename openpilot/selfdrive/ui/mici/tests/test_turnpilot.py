from openpilot.cereal import log, messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.lib.turnpilot import active_plan, signal_prompt, status_text


def plan(**fields):
  msg = messaging.new_message('turnpilotPlan', valid=True)
  for k, v in fields.items():
    setattr(msg.turnpilotPlan, k, v)
  return msg.as_reader().turnpilotPlan


def car_state(**fields):
  msg = messaging.new_message('carState', valid=True)
  for k, v in fields.items():
    setattr(msg.carState, k, v)
  return msg.as_reader().carState


class FakeSM:
  def __init__(self, p, alive=True, valid=True):
    self.alive = {'turnpilotPlan': alive}
    self.valid = {'turnpilotPlan': valid}
    self._plan = p

  def __getitem__(self, service):
    return self._plan


class TestTurnPilotOnroad(OpenpilotTestCase):
  def setup_method(self):
    # imported per test: ui_state reads Params at import, which needs the test's prefix
    from openpilot.selfdrive.ui.mici.onroad import turnpilot
    from openpilot.selfdrive.ui.mici.layouts.settings.turnpilot import shown_url
    self.tp = turnpilot
    self.shown_url = shown_url

  def test_nothing_while_following(self):
    for state in ('noRoute', 'following', 'approaching'):
      assert status_text(plan(state=state, reason=''), True) == ('', '')
    assert status_text(None, True) == ('', '')

  def test_unavailable_shows_engaged_or_not(self):
    for engaged in (True, False):
      assert status_text(plan(state='unavailable', reason='detector stopped: take control'), engaged) == \
        ('TurnPilot unavailable', 'detector stopped: take control')

  def test_turning(self):
    assert status_text(plan(state='turning', desire=log.Desire.turnLeft), True) == ('Turning left', '')
    assert status_text(plan(state='turning', desire=log.Desire.keepRight), True) == ('Keeping right', '')
    # between desire pulses the maneuver gives the side
    assert status_text(plan(state='turning', maneuverModifier='sharp right'), True) == ('Turning right', '')
    assert status_text(plan(state='turning', maneuverModifier='uturn'), True) == ('Turning', '')

  def test_lane_change(self):
    assert status_text(plan(state='laneChange', laneChange='left'), True) == ('Lane change left', '')
    assert status_text(plan(state='laneChange', signalPrompt='right'), True) == ('Lane change right', '')
    assert status_text(plan(state='laneChange'), True) == ('Lane change', '')

  def test_holding_and_arrived(self):
    assert status_text(plan(state='holding', reason='pedestrian in path'), True) == ('Holding', 'pedestrian in path')
    assert status_text(plan(state='arrived', reason='car in target lane'), True) == ('Arrived', '')

  def test_shadow(self):
    assert status_text(plan(state='turning', desire=log.Desire.turnRight, shadow=True), True) == ('Turning right', 'shadow')
    assert status_text(plan(state='holding', reason='waiting for oncoming', shadow=True), True) == ('Holding', 'shadow: waiting for oncoming')
    assert status_text(plan(state='following', shadow=True), True) == ('', '')

  def test_signal_prompt(self):
    p = plan(state='laneChange', signalPrompt='left')
    assert signal_prompt(p, car_state()) == 'left'
    assert signal_prompt(p, car_state(rightBlinker=True)) == 'left'
    assert signal_prompt(p, car_state(leftBlinker=True)) == ''  # signaled: now just "Lane change left"
    assert signal_prompt(plan(state='laneChange', signalPrompt='right'), car_state(leftBlinker=True)) == 'right'
    assert signal_prompt(plan(state='laneChange'), car_state()) == ''
    assert signal_prompt(plan(state='holding', signalPrompt='left'), car_state()) == ''  # the hold is what to show
    assert signal_prompt(None, car_state()) == ''

  def test_prompt_alert(self):
    assert self.tp.prompt_alert(plan(state='laneChange'), car_state()) is None
    alert = self.tp.prompt_alert(plan(state='laneChange', signalPrompt='right'), car_state())
    assert alert.text1 == 'Signal Right to Change Lanes'
    # the warning's countdown, before the change may go
    counting = self.tp.prompt_alert(plan(state='laneChange', signalPrompt='right', laneChangeOffer='right', laneChangeCountdown=2.3),
                                    car_state())
    assert counting.text1 == 'Lane Change Right in 3'
    assert alert.status == log.SelfdriveState.AlertStatus.userPrompt
    assert alert.alert_type.startswith('preLaneChangeRight/')  # upstream's turn signal icon on that side
    shadow = self.tp.prompt_alert(plan(state='laneChange', signalPrompt='left', shadow=True), car_state())
    assert shadow.text2 == 'shadow mode'

  def test_active_plan(self):
    p = plan(state='turning')
    assert active_plan(FakeSM(p), True) is p
    assert active_plan(FakeSM(p), False) is None
    assert active_plan(FakeSM(p, alive=False), True) is None
    assert active_plan(FakeSM(p, valid=False), True) is None
    assert active_plan(FakeSM(plan(state='disabled')), True) is None

  def test_status_needs_engaged_unless_shadow(self):
    # the driver is steering: nothing is holding or turning the car
    assert status_text(plan(state='holding', reason='pedestrian in path'), False) == ('', '')
    assert status_text(plan(state='turning', desire=log.Desire.turnLeft), False) == ('', '')
    # shadow mode is for comparing with what the driver does, so it shows while disengaged
    assert status_text(plan(state='turning', desire=log.Desire.turnLeft, shadow=True), False) == ('Turning left', 'shadow')
    # the prompt is a navigation hint, shown either way
    assert signal_prompt(plan(state='laneChange', signalPrompt='left'), car_state()) == 'left'

  def test_pair_url_without_network(self):
    assert self.shown_url('http://127.0.0.1:8088/?t=abc') == 'no network'
    assert self.shown_url('http://172.20.10.2:8088/?t=abc') == 'http://172.20.10.2:8088/?t=abc'


class TestMapStatus:
  def test_route_refusal_and_download_on_the_status_line(self):
    from openpilot.selfdrive.ui.lib.turnpilot import map_status_text
    here = {'latitude': 39.8, 'longitude': -86.1}
    at = {'destination': [-86.1, 39.8]}
    refused = {'state': 'error', 'code': 'outside_coverage', 'short': 'Destination outside mapped area',
               'message': "Outside TurnPilot's mapped area. TurnPilot can only navigate around Indianapolis and Seattle.", **at}
    assert map_status_text(refused, here) == ('No route', 'Destination outside mapped area')
    assert map_status_text(refused, None) == ('', '')  # nothing once the destination is gone
    assert map_status_text({'state': 'error', 'message': 'Routing failed', **at}, here) == ('No route', 'Routing failed')
    assert map_status_text({'state': 'downloading', 'progress': 0.42, **at}, here) == ('Downloading map', 'for this drive, 42%')
    assert map_status_text({'state': 'downloading', 'progress': None, **at}, here) == ('Downloading map', 'for this drive')
    for state in ('idle', 'checking', 'ready'):
      assert map_status_text({'state': state, **at}, here) == ('', '')
    assert map_status_text(None, here) == ('', '')

  def test_a_status_for_another_destination_is_not_shown(self):
    """navd's status from the previous destination (an error, or a download it is still finishing) is not this one's."""
    from openpilot.selfdrive.ui.lib.turnpilot import map_status_text
    old = {'state': 'error', 'short': 'Car outside mapped area', 'message': 'The car is outside', 'destination': [-122.3, 47.6]}
    assert map_status_text(old, {'latitude': 39.8, 'longitude': -86.1}) == ('Checking', 'the mapped area')
    assert map_status_text({'state': 'error', 'short': 'x'}, {'latitude': 39.8, 'longitude': -86.1}) == ('Checking', 'the mapped area')

  def test_mapped_area_text(self, tmp_path):
    from openpilot.selfdrive.turnpilot.maps.coverage import CoverageCache, coverage_path, save
    from openpilot.selfdrive.turnpilot.maps.tests.fakes import CHICAGO, INDY, coverage_doc
    from openpilot.selfdrive.ui.lib.turnpilot import mapped_area_text
    cache = CoverageCache(tmp_path)
    assert mapped_area_text(cache, (INDY[1], INDY[0])) == 'no mapped area downloaded'
    save(coverage_doc(), tmp_path)
    assert mapped_area_text(cache, (INDY[1], INDY[0])) == 'Indianapolis area'
    assert mapped_area_text(cache, (CHICAGO[1], CHICAGO[0])) == "Outside TurnPilot's mapped area"
    assert mapped_area_text(cache, None) == 'car position unknown'  # two lines at most in the button; the phone names the areas
    assert mapped_area_text(cache, None, '/sim/tiles') == 'local tiles (developer), no geofence'
    coverage_path(tmp_path).write_text('garbage')
    assert mapped_area_text(cache, (INDY[1], INDY[0])) == 'saved mapped area is unreadable'


class TestMapControlStatus:
  def test_hold_at_a_stop_sign_or_signal(self):
    held = plan(state='holding', shouldStop=True, reason='Stopped at stop sign', mapControl={'kind': 'stop', 'action': 'held'})
    assert status_text(held, True) == ('Stopped at stop sign', '')
    waiting = plan(state='holding', shouldStop=True, reason='Stop sign: take control to proceed',
                   mapControl={'kind': 'stop', 'action': 'awaitingDriver'})
    assert status_text(waiting, True) == ('Stopped at stop sign', 'take control')
    left = plan(state='holding', shouldStop=True, reason='Unprotected left: take control',
                mapControl={'kind': 'unprotectedTurn', 'action': 'awaitingDriver'})
    assert status_text(left, True) == ('Stopped at unprotected turn', 'take control')
    red = plan(state='holding', shouldStop=True, reason='Red light (sim oracle)',
               mapControl={'kind': 'signal', 'action': 'held', 'lightState': 'red', 'lightSource': 'simOracle'})
    assert status_text(red, True) == ('Red light', 'sim oracle')
    # another hold in force with it (a pedestrian) is what the detail says
    ped = plan(state='holding', shouldStop=True, reason='pedestrian in path', mapControl={'kind': 'stop', 'action': 'awaitingDriver'})
    assert status_text(ped, True) == ('Stopped at stop sign', 'pedestrian in path')
    assert status_text(plan(state='holding', reason='Stop sign ahead', mapControl={'kind': 'stop', 'action': 'braking'}), True) == \
      ('Holding', 'Stop sign ahead')
    assert status_text(plan(state='holding', mapControl={'kind': 'stop', 'action': 'infeasible'}), True) == \
      ('Cannot stop gently', 'take control')

  def test_car_stopped_with_nothing_holding_it_is_said_on_the_device(self):
    # the model standing the car with nothing of TurnPilot's holding it: said on the device's status line well before
    # the handback (turnpilotd's STOPPED_NOTE), never a blank line for a minute
    from openpilot.selfdrive.turnpilot.control_text import STOPPED_NOTE
    p = plan(state='following', reason=STOPPED_NOTE)
    assert status_text(p, True) == ('Car stopped', 'nothing ahead holding it')

  def test_map_controls_unavailable_is_never_clear(self):
    p = plan(state='following', reason='Map stops/signals: unsupported', mapControl={'controlsStatus': 'unsupported'})
    assert status_text(p, True) == ('Map stops unavailable', 'unsupported')
    assert status_text(plan(state='following', mapControl={'controlsStatus': 'ok'}), True) == ('', '')
    assert status_text(plan(state='following', mapControl={'controlsStatus': 'none'}), True) == ('', '')
