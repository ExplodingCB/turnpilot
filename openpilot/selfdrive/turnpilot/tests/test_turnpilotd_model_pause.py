"""A big model fallback pauses TurnPilot with its route kept, and the driver's next engagement after the big model is back
resumes it, said (turnpilotd.py: watch_model). modeld brings the big model back mid-drive (modeld/chestnut_recovery.py)."""
from unittest import mock

import pytest

from openpilot.cereal import custom, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.handback import Handback
from openpilot.selfdrive.turnpilot.tests.synthetic import model_v2, nav_instruction
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_main import FakeParams, LoopSM, Stop, car_params_bytes
from openpilot.selfdrive.turnpilot.turnpilotd import (BIG_BACK_REASON, DETECTOR_START_DEADLINE_S, PAUSED_REASON, RESUMED_ALERT_S,
                                                       WAITING_REASON)

State = custom.TurnpilotPlan.State
ROUTE = [(900., "turn", "left"), (2500., "turn", "right")]


def steps(sim, seconds, **kwargs):
  return [sim.step(**kwargs) for _ in range(round(seconds / DT_MDL))]


def paused(engaged: bool, available: bool = True):
  """The big model drives 3 s, then modeld falls back: TurnPilot paused 5 s, openpilot engaged (plain openpilot on the small
  model, the soft disable's end aside) or not."""
  sim = Sim(ROUTE)
  sim.big = True
  steps(sim, 3., engaged=engaged)
  sim.big = False
  plans = steps(sim, 5., engaged=engaged, available=available)
  assert all(p.handback == Handback.modelFellBack and p.state == State.unavailable and p.reason == PAUSED_REASON for p in plans)
  return sim


def resumed_alerts(plans) -> list[int]:
  return [i for i, p in enumerate(plans) if str(p.controlAlert) == 'turnpilotResumed']


def test_the_big_model_back_resumes_turnpilot_at_the_next_engagement_and_says_so():
  sim = paused(engaged=False)
  sim.big = True  # modeld's successor publishes, openpilot disengaged
  plans = steps(sim, 2., engaged=False)
  # back, but not acting: the driver hasn't engaged since, and every surface says what it waits for
  assert all(p.handback == Handback.modelFellBack and p.state == State.unavailable and p.reason == BIG_BACK_REASON for p in plans)
  assert not resumed_alerts(plans)
  plans = steps(sim, 6., engaged=True)
  assert all(p.handback == Handback.none and p.state != State.unavailable for p in plans)
  said = resumed_alerts(plans)
  assert said == list(range(round(RESUMED_ALERT_S / DT_MDL)))  # from the engagement, once
  assert plans[-1].routeValid  # the route it was paused on, kept all along


def test_an_engagement_on_the_small_model_is_never_taken_over():
  # the driver engaged openpilot on the small model (plain openpilot) after the handback; the big model comes back while it
  # drives: TurnPilot stays paused for as long as that engagement lasts, and resumes only at the next one
  sim = paused(engaged=True)
  sim.big = True
  plans = steps(sim, 30., engaged=True)
  assert all(p.handback == Handback.modelFellBack and p.reason == BIG_BACK_REASON for p in plans)
  assert all(p.desire == 0 and str(p.laneChange) == 'none' and not resumed_alerts([p]) for p in plans)
  sim.step(engaged=False)  # the driver disengages...
  plan = sim.step(engaged=True)  # ...and engages again: TurnPilot's now
  assert plan.handback == Handback.none and str(plan.controlAlert) == 'turnpilotResumed'


def test_engaging_when_the_big_model_comes_back_isnt_engaging_after_it():
  # the engagement and the big model's first frame together: the driver couldn't have known, so it isn't the resume
  sim = paused(engaged=False)
  sim.big = True
  plan = sim.step(engaged=True)
  assert plan.handback == Handback.modelFellBack and plan.reason == BIG_BACK_REASON


def test_the_big_model_gone_again_before_the_engagement_stays_paused():
  sim = paused(engaged=False)
  sim.big = True
  steps(sim, 1., engaged=False)
  sim.big = False  # the successor stopped: the parent's small model publishes again
  plans = steps(sim, 1., engaged=False) + steps(sim, 2., engaged=True)
  assert all(p.handback == Handback.modelFellBack and p.reason == PAUSED_REASON for p in plans)
  # back once more: the arming starts over
  sim.big = True
  assert sim.step(engaged=True).handback == Handback.modelFellBack
  sim.step(engaged=False)
  assert sim.step(engaged=True).handback == Handback.none


def test_a_second_fallback_after_the_resume_pauses_again():
  sim = paused(engaged=False)
  sim.big = True
  sim.step(engaged=False)
  assert sim.step(engaged=True).handback == Handback.none
  steps(sim, 5., engaged=True)
  sim.big = False
  plan = sim.step(engaged=True)
  assert plan.handback == Handback.modelFellBack and plan.reason == PAUSED_REASON


def test_the_resume_waits_for_the_detector_never_hands_back_a_stopped_one():
  # the small model has no detector, so the pause ran with no detections: at the resume the detector's start deadline runs
  # afresh (waiting for it), never a stopped detector's handback at the engagement
  sim = paused(engaged=False, available=False)
  steps(sim, 30., engaged=False, available=False)
  sim.big = True
  sim.step(engaged=False, available=False)
  plans = steps(sim, 3., engaged=True, available=False)
  assert all(p.handback == Handback.none and p.state == State.unavailable and p.reason == WAITING_REASON for p in plans)
  assert resumed_alerts(plans) == list(range(round(RESUMED_ALERT_S / DT_MDL)))
  plan = sim.step(engaged=True, available=True)  # the successor's detector runs
  assert plan.handback == Handback.none and plan.state != State.unavailable
  # one that never runs is the detector that never started, out loud, after its start deadline
  sim = paused(engaged=False, available=False)
  sim.big = True
  sim.step(engaged=False, available=False)
  plans = steps(sim, DETECTOR_START_DEADLINE_S + 1., engaged=True, available=False)
  assert plans[-1].handback == Handback.noDetector


def test_a_paused_turnpilot_turned_off_and_on_stays_paused_until_the_big_model_is_back():
  sim = paused(engaged=False)
  fresh = turnpilotd.TurnPilot()  # TurnPilot off and on: a new turnpilotd, modeld's ChestnutActive False
  fresh.big_model_failed(enabled=True)
  sim.tp = fresh
  assert sim.step(engaged=False).handback == Handback.modelFellBack
  sim.big = True
  sim.step(engaged=False)
  assert sim.step(engaged=True).handback == Handback.none


class FlippingSM(LoopSM):
  """LoopSM whose modelV2 is the big model's for the first big_frames, then the small model's."""
  def __init__(self, big_frames):
    super().__init__(0, lambda frame: True)
    self.big_frames = big_frames
    self.data['navInstruction'] = nav_instruction(500.)
    self.updated['navInstruction'] = True

  def __getitem__(self, s):
    if s == 'modelV2':
      return model_v2(big=self.frame < self.big_frames)
    return super().__getitem__(s)


class RecordingParams(FakeParams):
  def __init__(self, **kw):
    super().__init__(**kw)
    self.removed = []

  def remove(self, key):
    self.removed.append(key)
    self.pop(key, None)


@pytest.mark.parametrize("shadow", [False, True])
def test_main_keeps_the_route_through_a_fallback(shadow):
  # main() used to remove NavDestination on a fallback, ending navd's route: the pause keeps it
  sm, sent = FlippingSM(big_frames=20), []
  pm = mock.MagicMock()
  pm.send.side_effect = lambda s, msg: sent.append((s, msg.as_reader()))
  rk = mock.MagicMock()
  rk.keep_time.side_effect = [None] * 59 + [Stop()]
  params = RecordingParams(TurnPilotEnabled=True, TurnPilotShadowMode=shadow, CarParams=car_params_bytes(), DisengageOnAccelerator=True,
                           NavDestination={"place": "somewhere"})
  with mock.patch.object(messaging, 'SubMaster', return_value=sm), mock.patch.object(messaging, 'PubMaster', return_value=pm), \
       mock.patch.object(messaging, 'sub_sock', return_value=mock.MagicMock()), \
       mock.patch.object(messaging, 'drain_sock', side_effect=lambda sock, wait_for_one=False: []), \
       mock.patch.object(turnpilotd, 'Params', return_value=params), mock.patch.object(turnpilotd, 'Ratekeeper', return_value=rk), \
       mock.patch.object(turnpilotd, 'SceneTracker', return_value=mock.MagicMock()), \
       mock.patch.object(turnpilotd, 'DEVICE_CAMERAS', mock.MagicMock()), mock.patch.object(turnpilotd, 'log_error'), pytest.raises(Stop):
    turnpilotd.main()
  plans = [msg.turnpilotPlan for s, msg in sent if s == 'turnpilotPlan']
  assert plans[-1].handback == Handback.modelFellBack
  assert params.removed == [] and params['NavDestination'] == {"place": "somewhere"}


def test_the_comma_the_nav_view_and_the_phone_say_paused_then_back():
  from openpilot.selfdrive.turnpilot.web.status import format_status
  from openpilot.selfdrive.ui.lib.turnpilot import status_text
  from openpilot.selfdrive.ui.lib.turnpilot_nav import state_lines
  sim = paused(engaged=True)
  for big, reason in ((False, PAUSED_REASON), (True, BIG_BACK_REASON)):
    sim.big = big
    plan = sim.step(engaged=True)
    assert status_text(plan, True) == ('TurnPilot unavailable', reason)  # the camera view's status line
    assert state_lines(plan, True)[1] == reason  # the nav view
    snap = {'at': 0., 'onroad': True, 'route': None, 'gps': None,
            'plan': {'state': str(plan.state), 'reason': plan.reason, 'shadow': plan.shadow, 'map': None}}
    assert format_status(snap, None, True, True, now=0.)['turnpilot']['reason'] == reason  # the phone
