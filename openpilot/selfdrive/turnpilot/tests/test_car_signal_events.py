"""selfdrived's turnpilotCarSignal lines (hooks.CarSignalEvents) and the tooFastToTurn alert's titles for the turn signal's
skips (DESIGN phase4/blinker 7.7)."""
import types

from opendbc.car.tesla.values import TeslaFlags
from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, turnpilot_car_signal_alert, turnpilot_too_fast_to_turn_alert
from openpilot.selfdrive.turnpilot import hooks
from openpilot.selfdrive.turnpilot.hooks import CarSignalEvents, too_fast_title

EventName = log.OnroadEvent.EventName
DT = 0.01
ON = int(TeslaFlags.CAR_SIGNALS)
RECHECKED = ON | int(TeslaFlags.CAR_SIGNALS_RECHECK)


def plan(status='off', prompt='none', handed='none', notice='none', skip='none', why='', modifier='left'):
  msg = messaging.new_message('turnpilotPlan')
  ts = msg.turnpilotPlan.turnSignal
  ts.status, ts.prompt, ts.handedOver, ts.notice, ts.skip, ts.why = status, prompt, handed, notice, skip, why
  msg.turnpilotPlan.maneuverModifier = modifier
  return msg.as_reader().turnpilotPlan


def port(status='ready', fault='none', check='idle', why='', still=False, rejected=0):
  return types.SimpleNamespace(status=status, fault=fault, check=check, checkWhy=why, stillOn=still, rejected=rejected)


def run(ev, n, p, cs, flags=RECHECKED):
  lines = []
  for _ in range(n):
    names = ev.update(p, cs, flags, DT)
    lines.append(ev.line if names else None)
  return lines


def test_prompt_chimes_once_and_holds():
  ev = CarSignalEvents()
  lines = run(ev, 50, plan(prompt='left'), port())
  # the chime stays on the line for as long as it shows: soundd plays a one-shot sound once however long it is held
  assert all(line == ("Signal Left", "For the turn ahead", 'tpLaneChangeLeft') for line in lines)
  assert run(ev, 1, plan(), port()) == [None]
  # back within the quiet time: no new chime; a new prompt after it: chimed again
  assert all(line[2] == 'none' for line in run(ev, 50, plan(prompt='left'), port()))
  run(ev, int(11. / DT), plan(), port())
  assert run(ev, 1, plan(prompt='left'), port())[0][2] == 'tpLaneChangeLeft'
  assert run(CarSignalEvents(), 1, plan(prompt='right'), port())[0][2] == 'tpLaneChangeRight'


def test_fault_said_once():
  ev = CarSignalEvents()
  lines = run(ev, int(5. / DT), plan('failed'), port('failed', 'lampLost'), ON)
  said = ("Car Signal Failed", "Signal went out: stalk touched?. You'll be asked to signal", 'prompt')
  assert all(line == said for line in lines[:int(3.9 / DT)])  # its chime on the line for the whole hold
  assert lines[int(4.5 / DT)] is None
  ev = CarSignalEvents()
  assert "panda refused it" in run(ev, 1, plan('failed'), port('failed', 'lampTimeout', rejected=3), ON)[0][1]


def test_still_on_until_off():
  ev = CarSignalEvents()
  lines = run(ev, 20, plan('ready'), port(still=True))
  assert all(line == ("Turn Signal Still On", "Cancel it if you're not turning", 'prompt') for line in lines)
  assert run(ev, 1, plan('ready'), port()) == [None]


def test_two_chimes_at_once_each_aired():
  """A fault and a prompt in the same frame: the fault's line and chime first, for CHIME_S (longer than either sound, so
  the prompt's chime never cuts it off), then the prompt's line with its own chime, never lost behind the fault's."""
  ev = CarSignalEvents()
  lines = run(ev, int(3. / DT), plan('failed', prompt='left'), port('failed', 'lampLost'), ON)
  n = int(round(hooks.CHIME_S / DT))
  assert all(line[0] == "Car Signal Failed" and line[2] == 'prompt' for line in lines[:n])
  assert all(line == ("Signal Left", "For the turn ahead", 'tpLaneChangeLeft') for line in lines[n:])
  # the prompt over before the fault's hold: the fault's line back, its chime already heard: silent
  lines = run(ev, 20, plan('failed'), port('failed', 'lampLost'), ON)
  assert all(line[0] == "Car Signal Failed" and line[2] == 'none' for line in lines)


def test_prompt_put_aside_before_its_chime_aired_chimes_on_return():
  ev = CarSignalEvents()
  run(ev, 1, plan(prompt='left'), port())
  # still on comes: its chime first, then the prompt's own (cut off after one frame), then still on's line, silent
  lines = run(ev, int(3. / DT), plan(prompt='left'), port(still=True))
  n = int(round(hooks.CHIME_S / DT))
  assert all(line == ("Turn Signal Still On", "Cancel it if you're not turning", 'prompt') for line in lines[:n])
  assert all(line == ("Signal Left", "For the turn ahead", 'tpLaneChangeLeft') for line in lines[n:2 * n - 1])
  assert all(line == ("Turn Signal Still On", "Cancel it if you're not turning", 'none') for line in lines[2 * n:])
  # still on over: the prompt back, its chime heard already
  assert run(ev, 1, plan(prompt='left'), port())[0] == ("Signal Left", "For the turn ahead", 'none')


SOUNDD_EVERY = 5  # soundd's Ratekeeper(20) over selfdrived's 100 Hz, through a conflated SubMaster: one frame in five


def alert_sounds(frames, flags=RECHECKED):
  """selfdriveState.alertSound per selfdrived frame for (plan, carSignal) inputs, through CarSignalEvents, the
  turnpilotCarSignal alert and selfdrived's own Events and AlertManager."""
  from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
  from openpilot.selfdrive.selfdrived.events import Events
  import openpilot.selfdrive.selfdrived.events as events_mod
  ev, old = CarSignalEvents(), events_mod.CAR_SIGNAL_EVENTS
  events_mod.CAR_SIGNAL_EVENTS = ev
  try:
    am = AlertManager()
    sounds = []
    for frame, (p, cs) in enumerate(frames):
      events = Events()
      for name in ev.update(p, cs, flags, DT):
        events.add(getattr(EventName, name))
      am.add_many(frame, events.create_alerts([ET.PERMANENT], [car.CarParams.new_message(), car.CarState.new_message(), {}, False, 0, None]))
      am.process_alerts(frame, set())
      sounds.append(am.current_alert.audible_alert)
    return sounds
  finally:
    events_mod.CAR_SIGNAL_EVENTS = old


def soundd_hears(sounds, sound) -> bool:
  """Whether soundd, reading every fifth selfdriveState, hears sound whatever its phase against selfdrived's frames."""
  return all(sound in sounds[phase::SOUNDD_EVERY] for phase in range(SOUNDD_EVERY))


def test_every_chime_reaches_soundd():
  AudibleAlert = log.SelfdriveState.AudibleAlert
  fault = [(plan('ready'), port())] * 37 + [(plan('failed'), port('failed', 'lampLost'))] * 300
  assert soundd_hears(alert_sounds(fault, ON), AudibleAlert.prompt), "'Car Signal Failed'"
  prompt = [(plan('failed'), port('failed', 'lampLost'))] * 700 + [(plan('failed', prompt='left'), port('failed', 'lampLost'))] * 300
  assert soundd_hears(alert_sounds(prompt, ON)[700:], AudibleAlert.tpLaneChangeLeft), "'Signal Left'"
  both = [(plan('failed', prompt='right'), port('failed', 'lampLost'))] * 300
  sounds = alert_sounds(both, ON)
  assert soundd_hears(sounds, AudibleAlert.prompt) and soundd_hears(sounds, AudibleAlert.tpLaneChangeRight), "fault and prompt"
  still = [(plan('ready'), port(still=True))] * 300
  assert soundd_hears(alert_sounds(still), AudibleAlert.prompt), "'Turn Signal Still On'"
  for status, flags in (('unverified', 0), ('unavailable', ON), ('ready', ON)):
    assert soundd_hears(alert_sounds([(plan(status), port())] * 300, flags), AudibleAlert.prompt), status
  handed = [(plan('ready', handed='left'), port())] * 300
  assert soundd_hears(alert_sounds(handed), AudibleAlert.prompt), "'Signal Off'"
  for notice in ('changedMidTurn', 'ranPast'):
    assert soundd_hears(alert_sounds([(plan('ready', notice=notice), port())] * 300), AudibleAlert.prompt), notice
  check = ON | int(TeslaFlags.CAR_SIGNALS_CHECK)
  for stage in ('reason', 'passed', 'failed', 'aborted'):
    seq = [(plan('checking'), port('checking', check=stage, why='x'))] * 300
    assert soundd_hears(alert_sounds(seq, check), AudibleAlert.prompt), stage


def test_once_per_drive():
  for status, flags, title in (('unverified', 0, "Car Signals Not Verified"), ('unavailable', ON, "Car Can't Signal"),
                               ('ready', ON, "Car Signals Not Re-checked")):
    ev = CarSignalEvents()
    lines = run(ev, int(7. / DT), plan(status, why='Car software changed: run the check again'), port(), flags)
    assert lines[0][0] == title and lines[0][2] == 'prompt' and lines[-1] is None, status
    assert run(ev, int(7. / DT), plan(status), port(), flags)[0] is None
  ev = CarSignalEvents()
  assert run(ev, 1, plan('unverified', why='Car software changed: run the check again'), port(), 0)[0][1] == \
    'Car software changed: run the check again'
  assert run(CarSignalEvents(), 1, plan('ready'), port(), RECHECKED) == [None]  # a P-verified car was re-checked


def test_notices_and_handed_over():
  ev = CarSignalEvents()
  assert run(ev, 1, plan('ready', handed='left'), port())[0] == ("Signal Off", "Signal left yourself", 'prompt')
  assert run(CarSignalEvents(), 1, plan('ready', notice='changedMidTurn'), port())[0][:2] == ("Turn Signal Changed", "Finishing the turn")
  assert run(CarSignalEvents(), 1, plan('ready', notice='ranPast'), port())[0][:2] == ("Turn Signal Ended", "Signal ran past the turn")


def test_check_steps():
  check = ON | int(TeslaFlags.CAR_SIGNALS_CHECK)
  ev = CarSignalEvents()
  assert run(ev, 1, plan('checking'), port('checking', check='reason', why='Hands off the signal, stay stopped'), check)[0] == \
    ("Turn Signal Check", "Hands off the signal, stay stopped", 'prompt')
  assert run(ev, 1, plan('checking'), port('checking', check='pattern', why='Checking turn signals'), check)[0][0] == "Checking Turn Signals"
  line = run(ev, 1, plan('checking'), port('checking', check='passed', why='Confirm the outside lamps on the phone'), check)[0]
  assert line == ("Turn Signal Check Passed", "Confirm the outside lamps on the phone", 'prompt')
  ev = CarSignalEvents()
  line = run(ev, 1, plan('verifying'), port('verifying', check='left'), RECHECKED)[0]
  assert line == ("Turn Signal Re-check", "Stay in Park", 'none')


def test_too_fast_titles():
  assert too_fast_title(plan(skip='driverSignal', modifier='left')) == "Turn Cancelled: You Signalled Right"
  assert too_fast_title(plan(skip='driverSignal', modifier='slight right')) == "Turn Cancelled: You Signalled Left"
  assert too_fast_title(plan(skip='hazards')) == "Turn Cancelled: Hazards On"
  assert too_fast_title(plan(skip='signalLost')) == "Turn Cancelled: Signal Went Out"
  assert too_fast_title(plan(skip='sideUncertain')) == "Turn Side Uncertain"
  assert too_fast_title(plan()) is None


class FakeSM(dict):
  def __init__(self, p):
    super().__init__(turnpilotPlan=p)
    import time
    self.recv_time, self.valid = {'turnpilotPlan': time.monotonic()}, {'turnpilotPlan': True}


def test_alerts():
  assert ET.PERMANENT in EVENTS[EventName.turnpilotCarSignal]
  hooks.CAR_SIGNAL_EVENTS.line = ("Signal Left", "For the turn ahead", 'tpLaneChangeLeft')
  a = turnpilot_car_signal_alert(car.CarParams.new_message(), car.CarState.new_message(), FakeSM(plan()), False, 0, None)
  assert (a.alert_text_1, a.audible_alert) == ("Signal Left", log.SelfdriveState.AudibleAlert.tpLaneChangeLeft)
  hooks.CAR_SIGNAL_EVENTS.line = None
  p = messaging.new_message('turnpilotPlan')
  p.turnpilotPlan.tooFastToTurn, p.turnpilotPlan.turnSignal.skip = True, 'hazards'
  a = turnpilot_too_fast_to_turn_alert(car.CarParams.new_message(), car.CarState.new_message(), FakeSM(p.as_reader().turnpilotPlan),
                                       False, 0, None)
  assert (a.alert_text_1, a.alert_text_2) == ("Turn Cancelled: Hazards On", "Rerouting")
