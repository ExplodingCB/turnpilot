"""Gate 1 publishes the turnpilotPlan in place of turnpilotd: plannerd and selfdrived take a plan only while it is valid
(hooks.fresh_plan), so one sent invalid would have its caps and stops silently ignored."""
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.turnpilot.hooks import fresh_plan
from openpilot.tools.sim.bridge.beamng.gate1 import Gate1


class FakePM:
  def __init__(self):
    self.sent = []

  def send(self, service, msg):
    self.sent.append((service, msg))


def test_published_plan_is_valid():
  gate = Gate1(SimpleNamespace(direction='left'), [40.], approach=120., speed_kph=25., pulse_frames=4, out='/dev/null')
  gate.pm = FakePM()
  gate.publish(log.Desire.turnLeft, 40.)
  (service, msg), = gate.pm.sent
  assert service == 'turnpilotPlan' and msg.valid
  sm = type('SM', (), {'recv_time': {'turnpilotPlan': 0.}, 'valid': {'turnpilotPlan': msg.valid},
                       '__getitem__': lambda self, s: msg.as_reader().turnpilotPlan})()
  assert fresh_plan(sm, now=0.) is not None
