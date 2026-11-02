"""selfdrived's side of a TurnPilot handback: its events through openpilot's own state machine."""
from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import ET, EVENTS, Events
from openpilot.selfdrive.selfdrived.state import SOFT_DISABLE_TIME, StateMachine
from openpilot.selfdrive.turnpilot.handback import Handback, HandbackEvents

State = log.SelfdriveState.OpenpilotState
EventName = log.OnroadEvent.EventName
ENABLED = (State.enabled, State.softDisabling)


def plan(handback=Handback.none):
  msg = messaging.new_message('turnpilotPlan')
  msg.turnpilotPlan.handback = handback
  return msg.as_reader().turnpilotPlan


class Selfdrived:
  """The part of selfdrived a handback goes through: HandbackEvents into Events, then the StateMachine."""
  def __init__(self, engaged=True):
    self.handback, self.sm = HandbackEvents(), StateMachine()
    self.sm.state = State.enabled if engaged else State.disabled
    self.events = Events()

  def step(self, p, engage=False):
    self.events.clear()
    self.handback.apply(self.events, p, self.sm.state in ENABLED)
    if engage:
      self.events.add(EventName.buttonEnable)
    self.sm.update(self.events)
    return self.sm.state

  def run(self, p, seconds, **kw):
    return [self.step(p, **kw) for _ in range(int(seconds / DT_CTRL))]


def test_detector_stopped_soft_disables_and_blocks_engaging():
  sd = Selfdrived()
  assert set(sd.run(plan(), 5.)) == {State.enabled}
  states = sd.run(plan(Handback.detectorStopped), SOFT_DISABLE_TIME + 1.)
  # openpilot's standard soft disable: the take-control alert and chime for SOFT_DISABLE_TIME, then disengaged
  assert states[0] == State.softDisabling and states[-1] == State.disabled
  assert abs(states.index(State.disabled) * DT_CTRL - SOFT_DISABLE_TIME) < 2 * DT_CTRL
  alert = EVENTS[EventName.turnpilotDetectorStopped][ET.SOFT_DISABLE]
  assert "TurnPilot unavailable: detector stopped" in alert(None, None, None, False, int(SOFT_DISABLE_TIME / DT_CTRL), None).alert_text_2
  # while it stays stopped, engaging is refused
  assert set(sd.run(plan(Handback.detectorStopped), 1., engage=True)) == {State.disabled}
  # back: engages
  assert sd.run(plan(), 0.1, engage=True)[-1] == State.enabled


def test_detector_back_during_the_countdown_still_disengages():
  sd = Selfdrived()
  sd.run(plan(Handback.detectorStopped), 0.5)
  states = sd.run(plan(), SOFT_DISABLE_TIME)  # it blinks back: the handback is still finished, not aborted halfway
  assert states[-1] == State.disabled and State.enabled not in states


def test_model_fallback_soft_disables_then_engages_on_the_small_model():
  sd = Selfdrived()
  states = sd.run(plan(Handback.modelFellBack), SOFT_DISABLE_TIME + 1.)
  assert states[0] == State.softDisabling and states[-1] == State.disabled
  # the fallback lasts the drive; openpilot can be engaged again on the small model, without TurnPilot, and stays engaged
  assert sd.run(plan(Handback.modelFellBack), 0.1, engage=True)[-1] == State.enabled
  assert set(sd.run(plan(Handback.modelFellBack), 10.)) == {State.enabled}


def test_never_a_silent_disengage():
  # every way into a handback while engaged goes through a soft disable with its own event
  for handback, event in ((Handback.detectorStopped, EventName.turnpilotDetectorStopped),
                          (Handback.noDetector, EventName.turnpilotNoDetector),
                          (Handback.modelFellBack, EventName.turnpilotModelFellBack)):
    sd = Selfdrived()
    sd.step(plan())
    sd.step(plan(handback))
    assert sd.sm.state == State.softDisabling and sd.events.names == [event]
    assert ET.SOFT_DISABLE in EVENTS[event] and ET.PERMANENT in EVENTS[event]


def test_disengaged_handback_alerts_once_and_shadow_does_nothing():
  sd = Selfdrived(engaged=False)
  events = []
  for _ in range(100):
    sd.step(plan(Handback.modelFellBack))
    events.append(list(sd.events.names))
  assert events[0] == [EventName.turnpilotModelFellBack] and not any(events[1:])  # its permanent alert, once
  sd = Selfdrived()
  assert set(sd.run(None, 2.)) == {State.enabled}  # turnpilotd in shadow mode or down: active_plan() is None


def test_the_driver_reads_turnpilots_alert_on_a_fallback():
  # bigModelFailed soft-disables on the same fallback; on the same frame its alert would win the priority tie
  def shown(frames):
    am, hb = AlertManager(), HandbackEvents()
    for frame, (big_failed, handback) in enumerate(frames):
      events = Events()
      if big_failed:
        events.add(EventName.bigModelFailed)
      hb.apply(events, plan(handback), True)
      args = [car.CarParams.new_message(), car.CarState.new_message(), None, False, int(SOFT_DISABLE_TIME / DT_CTRL), None]
      am.add_many(frame, events.create_alerts([ET.SOFT_DISABLE], args))
      am.process_alerts(frame, set())
    return am.current_alert.alert_text_2
  assert shown([(True, Handback.modelFellBack)]) == "Big model stopped: TurnPilot paused"
  assert shown([(True, Handback.none), (True, Handback.modelFellBack)]) == "Big model stopped: TurnPilot paused"
  assert shown([(True, Handback.none)]) == "Big Model Failed"  # TurnPilot off or in shadow mode: upstream's, unchanged


def test_detector_that_never_started_is_loud_and_blocks_engaging():
  # disengaged when the deadline passes: its alert, and engaging is refused (the refuse chime) while it lasts
  sd = Selfdrived(engaged=False)
  sd.step(plan(Handback.noDetector))
  assert sd.events.names == [EventName.turnpilotNoDetector]
  assert set(sd.run(plan(Handback.noDetector), 1., engage=True)) == {State.disabled}
  assert sd.run(plan(), 0.1, engage=True)[-1] == State.enabled
  # engaged: the soft disable, with TurnPilot's handback chime
  sd = Selfdrived()
  states = sd.run(plan(Handback.noDetector), SOFT_DISABLE_TIME + 1.)
  assert states[0] == State.softDisabling and states[-1] == State.disabled
  alert = EVENTS[EventName.turnpilotNoDetector][ET.SOFT_DISABLE](None, None, None, False, int(SOFT_DISABLE_TIME / DT_CTRL), None)
  assert "detector not running" in alert.alert_text_2 and alert.audible_alert == log.SelfdriveState.AudibleAlert.tpHandback
