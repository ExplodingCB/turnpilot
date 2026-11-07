import threading

from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import log, messaging
from openpilot.cereal.messaging import SubMaster, PubMaster
from openpilot.selfdrive.ui.soundd import SELFDRIVE_STATE_TIMEOUT, Soundd, check_selfdrive_timeout_alert

AudibleAlert = log.SelfdriveState.AudibleAlert
AlertSize = log.SelfdriveState.AlertSize


class FakeSM(dict):
  """selfdriveState, carState and turnpilotPlan, all alive, valid and updated."""

  def __init__(self, prompt='right', state='laneChange', blinker=False, alert_size=AlertSize.none, sound=AudibleAlert.none):
    ss = messaging.new_message('selfdriveState', valid=True)
    ss.selfdriveState.alertSize = alert_size
    ss.selfdriveState.alertSound = sound
    cs = messaging.new_message('carState', valid=True)
    cs.carState.rightBlinker = blinker
    tp = messaging.new_message('turnpilotPlan', valid=True)
    tp.turnpilotPlan.state = state
    tp.turnpilotPlan.signalPrompt = prompt
    super().__init__(selfdriveState=ss.as_reader().selfdriveState, carState=cs.as_reader().carState,
                     turnpilotPlan=tp.as_reader().turnpilotPlan)
    self.alive = self.valid = self.updated = dict.fromkeys(self, True)


class TestSoundd(OpenpilotTestCase):
  def test_check_selfdrive_timeout_alert(self, mocker):
    sm = SubMaster(['selfdriveState'])
    pm = PubMaster(['selfdriveState'])

    cs = messaging.new_message('selfdriveState')
    cs.selfdriveState.enabled = True
    threading.Timer(0.01, pm.send, args=("selfdriveState", cs)).start()
    sm.update(100)
    assert sm.updated['selfdriveState']

    sm.recv_time['selfdriveState'] = 0
    clock = mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=SELFDRIVE_STATE_TIMEOUT)
    assert not check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 0.1
    assert check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 10
    assert not check_selfdrive_timeout_alert(sm)

  def test_turnpilot_lane_change_chimes_come_from_selfdrived(self):
    # a TurnPilot prompt alone plays nothing: its chime is selfdrived's turnpilotLaneChange* alert, so one path, no double
    s = Soundd()
    s.get_audible_alert(FakeSM())
    assert s.current_alert == AudibleAlert.none
    s.get_audible_alert(FakeSM(alert_size=AlertSize.mid, sound=AudibleAlert.prompt))
    assert s.current_alert == AudibleAlert.prompt


def test_turnpilot_chimes_load_and_differ():
  from openpilot.selfdrive.ui.soundd import Soundd, sound_list
  s = Soundd()
  tp = [name for name, value in AudibleAlert.schema.enumerants.items() if name.startswith('tp')]
  assert len(tp) == 7 and all(AudibleAlert.schema.enumerants[name] in sound_list for name in tp)
  left, right = s.loaded_sounds[AudibleAlert.tpLaneChangeLeft], s.loaded_sounds[AudibleAlert.tpLaneChangeRight]
  assert len(left) == len(right) and not (left == right).all()
  # the driver's prompt to signal (2026-11-07): each side's own chime, on repeat, very clear left from right
  sl, sr = s.loaded_sounds[AudibleAlert.tpSignalLeft], s.loaded_sounds[AudibleAlert.tpSignalRight]
  assert len(sl) == len(sr) > len(left) and not (sl == sr).all()
  assert sound_list[AudibleAlert.tpSignalLeft][1] is None and sound_list[AudibleAlert.tpSignalRight][1] is None


def test_turnpilot_handback_ramps_like_warning_soft():
  # the handback replaces warningSoft in TurnPilot's soft disable: in a quiet cabin it climbs to full volume as that does
  from unittest import mock
  from openpilot.selfdrive.ui.soundd import MAX_VOLUME, Soundd
  volumes = {}
  with mock.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.) as clock:
    for alert in (AudibleAlert.warningSoft, AudibleAlert.tpHandback):
      s = Soundd()
      s.current_volume = 0.07  # a quiet cabin's
      clock.return_value = 100.
      s.update_alert(alert)
      clock.return_value = 102.5
      s.ramp_volume()
      volumes[alert] = s.current_volume
  assert volumes[AudibleAlert.tpHandback] == volumes[AudibleAlert.warningSoft] > 0.5 * MAX_VOLUME
