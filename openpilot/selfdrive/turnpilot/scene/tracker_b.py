"""Tracker B: the far crop's people as a comfortable-tier veto (TurnPilotFarPeds; PERCEPTION_PLAN item 5, peds P3 variant g:C).

Tracker A (turnpilotd main()'s SceneTracker) takes the far crop's boxes only as tags (update_far): a person only the far crop
scores is never tracked, so a person beside the road at 25-40 m is first published at about 20 m, from the full pass. Tracker B
is a second SceneTracker fed the same full-pass runs and, through update_tiles, the far crop's people and two-wheelers, which
start and extend its tracks. Each pedestrian B publishes that no A pedestrian holds (box IoU MATCH_IOU or more in the same camera,
or ground positions MATCH_M or less apart in any camera: the replays ran the road camera only, and the any-camera distance only
drops more B people in turns, where the full pass looks through the wide camera) and that isn't inside a two-wheeler's box
(RIDER_OVERLAP of the smaller box: one of B's published two-wheelers, or a bicycle or motorcycle box of the latest run of any
view, tracked or not) goes to the scene policy as a b_only object: trackId offset by scene_policy.B_ID, age 0, conf capped under
CROSSWALK_CONF. The policy never confirms one, never makes it a walker or a curb-cap track (scene_policy's module docstring), so
all it can do is a "pedestrian in path" stop on the STOP_DECEL 1.5 m/s^2 curve: a veto, never a gate, and never the hard tier.
turnpilotScene, scene.available, path_blocker, go_blocked, the left's raw runs and the goes' views stay tracker A's; with the far
crop off for the drive or aimed at the left's band, B adds fewer people or none, and the car has A's perception, nothing worse.

Replayed as it ships (this module, update_tiles and ScenePolicy.update's extra_objects, no patches: peds/trackerb's replay_b.py,
TurnPilotFarPeds off then on, over replay_g.py's ticks; tick for tick and track for track the peds r2 study's g:C+ghn run on all
56 segments). 56 public comma four segments (27.8 km, 0.93 h): people tracked 87 to 110, all 58 confirmed people confirmed at the
same frame, near-path people first published at a median 28.15 m instead of 20.0 m (n 36), false person units 1.08 to 1.44 per km,
hard-tier ticks 4 and 4, tracker A's rows and hard tier identical on and off on every tick; B-only stop asks below -0.3 m/s^2 19
ticks, every one a labelled person, none harder than -1.5, no curb cap. comma's demo route (14 segments, 12.8 km): people 9 to 12,
no false track in the path. KITTI tracking (21 sequences at comma four scale): people first labelled 25 m eq or more first
published at a median 34.9 m eq instead of 29.5 (n 114). The S10 step-out Monte Carlo (n 400 a case): contact at 29 km/h
32.5/36.0% to 8.0/11.0% (held / held-joint), at 40 km/h 73.8/75.7% to 44.3/44.3%.

TrackerB.step times B's update and merge each frame; turnpilot.tracker_b logs their p50 and p95 every LOG_PERIOD_S with the count
of B-only objects that asked for a stop, and turnpilot.tracker_b_stop each B-only object's first stop ask, with where its box was,
for the morning drive's check against the road video."""
import math
import time

import numpy as np

from openpilot.cereal import messaging
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.scene_policy import B_ID, CROSSWALK_CONF, RIDER_OVERLAP, TWO_WHEELERS, box_overlap

MATCH_IOU = 0.3       # an A pedestrian holds a B one: box IoU this much in the same camera,
MATCH_M = 2.0         # or ground positions this close, in any camera
B_CONF_MAX = CROSSWALK_CONF - 0.01  # a B-only object's conf is capped under the crosswalk's
LOG_PERIOD_S = 60.


class BObject:
  """A B-only pedestrian as the scene policy reads it: a TrackedObject's fields (tracker B's scene's), trackId offset by B_ID, age 0,
  conf capped at B_CONF_MAX, b_only True."""
  __slots__ = ('trackId', 'cls', 'conf', 'x', 'y', 'vx', 'vy', 'age', 'inPath', 'camera', 'view', 'farHits', 'tSeen', 'missedRuns', 'box',
               'b_only')

  def __init__(self, o):
    self.trackId = B_ID + int(o.trackId)
    self.cls, self.camera, self.view = str(o.cls), str(o.camera), str(o.view)
    self.conf = min(float(o.conf), B_CONF_MAX)
    self.x, self.y, self.vx, self.vy = float(o.x), float(o.y), float(o.vx), float(o.vy)
    self.age = 0
    self.inPath = bool(o.inPath)
    self.farHits, self.tSeen, self.missedRuns = int(o.farHits), float(o.tSeen), int(o.missedRuns)
    self.box = [float(v) for v in o.box]
    self.b_only = True


def box_iou(a, b) -> float:
  x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
  inter = max(x1 - x0, 0.) * max(y1 - y0, 0.)
  union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
  return inter / union if union > 0. else 0.


def held(b, a) -> bool:
  """A's pedestrian a holds B's b (module docstring)."""
  same_box = str(a.camera) == str(b.camera) and len(a.box) == 4 and len(b.box) == 4 and box_iou(a.box, b.box) >= MATCH_IOU
  return same_box or math.hypot(a.x - b.x, a.y - b.y) <= MATCH_M


def bike_boxes(tracker_a, b_objects) -> list[tuple[str, list]]:
  """(camera, box) of every two-wheeler a B person may be the rider of: B's published ones, and every bicycle or motorcycle box of
  the latest run of each view (tracker A's raw_by_view: the full pass on either camera, the side views, the far crop), tracked or
  not. Boxes normalized in their camera's frame."""
  out = [(str(o.camera), list(o.box)) for o in b_objects if str(o.cls) in TWO_WHEELERS and len(o.box) == 4]
  for frame in tracker_a.raw_by_view.values():
    if frame is not None:
      out += [(rb.camera, list(rb.box)) for rb in frame.boxes if rb.cls in TWO_WHEELERS and len(rb.box) == 4]
  return out


def b_only_objects(a_objects, b_objects, bikes) -> list[BObject]:
  """B's published pedestrians no A pedestrian holds and no two-wheeler's box takes as its rider, as BObjects."""
  a_peds = [a for a in a_objects if str(a.cls) == 'pedestrian']
  out = []
  for o in b_objects:
    if str(o.cls) != 'pedestrian' or any(held(o, a) for a in a_peds):
      continue
    camera = str(o.camera)
    if len(o.box) == 4 and any(c == camera and box_overlap(o.box, b) >= RIDER_OVERLAP for c, b in bikes):
      continue
    out.append(BObject(o))
  return out


class TrackerB:
  """Tracker B beside tracker A (module docstring): step() each frame, after_plan() with the plan's pedestrian stops."""

  def __init__(self, tracker):
    self.tracker = tracker      # a SceneTracker of its own, never tracker A
    self.objects: list[BObject] = []  # this frame's B-only objects
    self.times: list[float] = []      # s, step() this period
    self.period_t: float | None = None
    self.stop_ids: set[int] = set()   # B-only objects that asked for a stop this period
    self.logged_ids: set[int] = set()  # ...and ever (turnpilot.tracker_b_stop, once each)

  def step(self, runs, far, calibration, car_state, model_v2, tracker_a, scene, now: float | None = None,
           yaw_rate: float | None = None) -> list[BObject]:
    """The frame's runs (tracker A's, as A took them) and far run (detector_run's) into tracker B, in camera-time order around a
    new far run, then its B-only objects against tracker A's published scene (none while that is unavailable). now, yaw_rate: as
    SceneTracker.update's (turnpilotd passes neither, as for tracker A)."""
    t0 = time.monotonic()
    trk = self.tracker
    trk.coast_s = tracker_a.coast_s
    run = far.run if far is not None else None
    new_far = run is not None and run.timestampEof > 0 and (run.frameId, run.timestampEof) != trk.tile_key
    runs = list(runs)
    if new_far:
      t_far = run.timestampEof
      trk.update_runs([d for d in runs if d.timestampEof <= t_far], calibration, car_state, model_v2, now, yaw_rate)
      trk.update_tiles(far, calibration, car_state, model_v2, now, yaw_rate)
      later = [d for d in runs if d.timestampEof > t_far]
      if later:
        trk.update_runs(later, calibration, car_state, model_v2, now, yaw_rate)
    else:
      trk.update_runs(runs, calibration, car_state, model_v2, now, yaw_rate)
    self.objects = []
    if scene.available:
      msg = messaging.new_message('turnpilotScene', valid=True)
      trk.fill_scene(msg.turnpilotScene)
      b_objects = list(msg.as_reader().turnpilotScene.objects)
      self.objects = b_only_objects(scene.objects, b_objects, bike_boxes(tracker_a, b_objects))
    self.times.append(time.monotonic() - t0)
    return self.objects

  def after_plan(self, stops, v_ego: float, now: float | None = None) -> None:
    """stops: the plan's pedestrian stops this frame (ScenePolicy.carried), each B-only one counted for the period's log and its
    first logged with the object's box (turnpilot.tracker_b_stop); the period's log when it is due."""
    now = time.monotonic() if now is None else now
    for s in stops:
      tid = (s.evidence or {}).get('track_id')
      if tid is None or tid < B_ID:
        continue
      self.stop_ids.add(tid)
      if tid not in self.logged_ids:
        self.logged_ids.add(tid)
        o = next((o for o in self.objects if o.trackId == tid), None)
        where = {} if o is None else {'x': round(o.x, 2), 'y': round(o.y, 2), 'vx': round(o.vx, 2), 'vy': round(o.vy, 2),
                                      'conf': round(o.conf, 3), 'camera': o.camera, 'view': o.view, 't_seen': round(o.tSeen, 3),
                                      'box': [round(v, 4) for v in o.box]}
        cloudlog.event("turnpilot.tracker_b_stop", track_id=int(tid), reason=s.reason, ask=round(plan_accel(s.accel_cap, s.speed_cap, v_ego), 2),
                       d_stop=round(s.d_stop, 2), v_ego=round(v_ego, 2), **where)
    if self.period_t is None:
      self.period_t = now
    if now - self.period_t >= LOG_PERIOD_S:
      ms = np.array(self.times) * 1e3 if self.times else np.zeros(1)
      cloudlog.event("turnpilot.tracker_b", p50_ms=round(float(np.percentile(ms, 50)), 3), p95_ms=round(float(np.percentile(ms, 95)), 3),
                     max_ms=round(float(ms.max()), 3), frames=len(self.times), b_stop_objects=len(self.stop_ids),
                     b_objects=len(self.objects), tracks=len(self.tracker.tracks))
      self.times, self.stop_ids, self.period_t = [], set(), now
