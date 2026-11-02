import math
import time


def latest_gps_source(sm, max_accuracy: float | None = None) -> str | None:
  """The service with the freshest usable GPS fix, or None. TurnPilot: with max_accuracy, only a fix whose reported
  horizontalAccuracy is at most that (0, not reported, counts), so a poor fix on one service doesn't hide a good one on the
  other (navigation/hold.py)."""
  now = time.monotonic()
  sources = [s for s in ('gpsLocation', 'gpsLocationExternal')
             if sm.valid[s] and sm[s].hasFix and 0 <= now - sm.recv_time[s] < 2 and (max_accuracy is None or accuracy_of(sm[s]) <= max_accuracy)]
  return max(sources, key=lambda s: sm.logMonoTime[s]) if sources else None


def accuracy_of(gps) -> float:
  """TurnPilot: a fix's horizontalAccuracy in m, 0 when not reported, inf when not finite."""
  a = float(getattr(gps, 'horizontalAccuracy', 0.) or 0.)
  return a if math.isfinite(a) else float('inf')


def gps_status(sm) -> tuple[str, bool]:
  """TurnPilot: the nav panel's GPS line (text, ok). navd's own word while its navInstruction is alive (positionSource,
  navigation/hold.py): the route held through a GPS loss, or the position lost; else whether there is a fix at all."""
  if sm.alive['navInstruction']:
    source = str(sm['navInstruction'].positionSource)
    if source == 'deadReckoned':
      return 'GPS lost, holding route', False
    if source == 'lost':
      return 'GPS lost: take control', False
  ok = latest_gps_source(sm) is not None
  return f'GPS fix: {"yes" if ok else "no"}', ok
