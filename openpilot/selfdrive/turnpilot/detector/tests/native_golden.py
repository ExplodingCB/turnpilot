"""The off path's golden (test_native_detect.py): a fixed drive of the fake detector (test_far_crop's far crop, test_modeld_detector's
lane paint and lights) through straight road, a moved aim, a curve on the wide camera (the odd paint read) and a right turn,
every message it sends (turnpilotDetections and turnpilotFarDetections) as canonical JSON with the measured times zeroed. The
hashes in test_native_detect.py are this on 263065a, before the native schedule; nativeStatus, a field since, is checked
there and left out here only when off (its default), and the lights' reason and read (11-06) always, so the off path's
messages are today's to the field.

  PYTHONPATH=<tree>:<opendbc_repo> python -m openpilot.selfdrive.turnpilot.detector.tests.native_golden
"""
import hashlib
import json

from openpilot.cereal import log

TIME_FIELDS = ('executionTime', 'lanePaintTime', 'lightTime', 'sectorTime', 'logMonoTime')


NEW_FIELDS = ('reason', 'read')  # TurnpilotLights.Light, 2026-11-06: each head's read and reason this frame, not 263065a's


def canonical(obj):
  if isinstance(obj, dict):
    return {k: (0 if k in TIME_FIELDS else canonical(v)) for k, v in obj.items()
            if not (k == 'nativeStatus' and v == 'off') and k not in NEW_FIELDS}
  if isinstance(obj, list):
    return [canonical(v) for v in obj]
  if isinstance(obj, bytes):
    return obj.hex()
  if isinstance(obj, float):
    return round(obj, 4)
  return obj


class RecordingPM:
  def __init__(self):
    self.sent: list[tuple[str, str]] = []
    self.full, self.far = [], []

  def send(self, name, msg):
    r = msg.as_reader()
    self.sent.append((name, json.dumps(canonical(r.to_dict()), sort_keys=True)))
    (self.full if name == 'turnpilotDetections' else self.far).append(getattr(r, name))


DRIVE = ([(0.0, log.Desire.none, 0.)] * 12 + [(0.0, log.Desire.none, 8.)] * 6 + [(0.02, log.Desire.none, 0.)] * 8 +
         [(0.0, log.Desire.turnRight, -5.)] * 6 + [(0.0, log.Desire.none, 0.)] * 8)


def drive(det, pm, model_time=0.010):
  from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import Meta
  for k, (curvature, desire, bearing) in enumerate(DRIVE):
    det.step(pm, det.model_state, model_time, curvature, desire, [Meta(2 * k), Meta(2 * k + 1)], 'auto', bearing)
  return pm


def golden_detector():
  """The far crop detector of the golden drive (today's code paths only), TurnPilotNativeDetect off as DetectorHost sets it
  from the param (unset is full now, NATIVE_DEFAULT)."""
  from openpilot.selfdrive.turnpilot.detector.tests.test_far_crop import far_detector
  from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import FakeLights, with_lights
  det = with_lights(far_detector())
  det.far.lights = FakeLights()
  det.native_mode = 'off'
  return det


def digest(pm) -> tuple[int, int, str]:
  """(full messages, far messages, sha256 of every message in order)."""
  h = hashlib.sha256()
  for name, text in pm.sent:
    h.update(name.encode() + b'\0' + text.encode() + b'\n')
  return len(pm.full), len(pm.far), h.hexdigest()


if __name__ == '__main__':
  print(digest(drive(golden_detector(), RecordingPM())))
