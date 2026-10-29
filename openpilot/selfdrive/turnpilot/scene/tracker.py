"""Tracks YOLO26 detections over time and ranges them: turnpilotDetections in, TurnpilotScene objects out.

A minimal ByteTrack-style tracker (the idea from github.com/ifzhang/ByteTrack, MIT; this is our own code): high-confidence
detections match tracks by box IoU first, the rest get a second, stricter IoU pass, and only high-confidence leftovers
start tracks. Two TurnPilot additions: detections alternate between the road and wide cameras, whose boxes can't be
compared, so a third pass matches what's left by ground position; and a constant-velocity Kalman filter on ground
position, with the car's own motion taken out in the predict step, so velocities are over the ground, not relative.
Every run's boxes also come out as they are (raw, raw_new: RawFrame), tracked or not, with their bearing and ground range
uncapped, for the unprotected left's gap rule: a distant car that never scores NEW_TRACK_CONF never becomes a track, so
track absence certifies nothing (CRITIQUE REQUIRED 2).

The far crop's runs (DESIGN phase 4 G5: turnpilotFarDetections, update_far) come out the same way (raw_far, raw_far_new),
view far: their boxes are in the road camera's full-frame pixels, so they range and bear with the road lens, and the run
says which bearings it covered at the horizon (cover_deg). A far box matching a track by its box tags it (view far,
farHits) and does nothing else: it never starts a track, never counts as a hit (TrackedObject.age, which a hard stop's
confirmation reads: scene_policy), never moves the filter, never keeps a track alive, never makes the scene fresh. So the
full pass alone feeds every stop, as before the far crop.

The stop sweep's runs (STOP_SIGNS.md 1.3, SG3b) say their view too: a side view's boxes (sideLeft, sideRight) are in the wide
camera's fisheye pixels, so they range, bear and match as the wide camera's. The latest RawFrame of each view is kept in
raw_by_view (view_key: road, full, sideLeft, sideRight, far), a track says the view and camera time of its last match
(TrackedObject.view, tSeen), and turnpilotd hands every run it drains to update_runs, which takes them in camera-time
order. A track coasts MAX_COAST_S after its last match, SWEEP_COAST_S while a stop sweep runs (coast_s, set by turnpilotd):
two missed passes of a 5 Hz view.
"""
import time
from dataclasses import dataclass, field

import numpy as np

from openpilot.cereal import custom
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DeviceCameraConfig
from openpilot.selfdrive.turnpilot.detector.decoder import BICYCLE, BUS, CAR, MAX_DETECTIONS, MOTORCYCLE, PERSON, TRUCK
from openpilot.selfdrive.turnpilot.detector.graph import FAR_HORIZON_ROW
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view, camera_height, ground_point, road_lens, wide_lens

ObjectClass = custom.TurnpilotScene.ObjectClass
Camera = custom.TurnpilotDetections.Camera
View = custom.TurnpilotDetections.View
OBJECT_CLASS = {PERSON: ObjectClass.pedestrian, BICYCLE: ObjectClass.cyclist, CAR: ObjectClass.car,
                MOTORCYCLE: ObjectClass.motorcycle, BUS: ObjectClass.bus, TRUCK: ObjectClass.truck}
OBJECT_CLASS_NAME = {PERSON: 'pedestrian', BICYCLE: 'cyclist', CAR: 'car', MOTORCYCLE: 'motorcycle', BUS: 'bus', TRUCK: 'truck'}
# classes YOLO confuses with each other match as one
CLASS_GROUP = {PERSON: 0, BICYCLE: 1, MOTORCYCLE: 1, CAR: 2, BUS: 2, TRUCK: 2}

HIGH_CONF = 0.5          # ByteTrack's first pass; below it a detection only extends a track
NEW_TRACK_CONF = 0.45    # a detection starts a track at or above this
IOU_FIRST = 0.3
IOU_SECOND = 0.5
GROUND_GATE_M = 2.0      # third pass: max distance between predicted and measured ground position,
GROUND_GATE_FRAC = 0.15  # plus this fraction of the range
MIN_HITS = 2             # a track is published after this many matched detections
MAX_COAST_S = 0.5        # and for this long after its last match;
SWEEP_COAST_S = 0.65     # (while a stop sweep runs: two missed passes of a 5 Hz side view, 0.6 s between its hits)
MAX_AGE_S = 1.5          # it's dropped after this
MAX_TRACKS = 3 * MAX_DETECTIONS // 2  # a full message's worth and then some coasting; past it, the soonest to expire go
# a pedestrian in the path this close can drop out of view (feet below the frame, or YOLO loses a person filling the
# frame: the BeamNG mannequin at 5.3 m from the camera). The car is stopping for them, so keep them, carried along with
# the car's own motion, for this long instead of releasing the stop.
CLOSE_HOLD_M = 8.0
CLOSE_HOLD_S = 3.0
CLOSE_HOLD_MIN_HITS = 4  # matched detections before a track earns that hold; a 2-3 hit blip gets the normal MAX_COAST_S
STALE_S = 0.5            # no new detections for this long makes the scene unavailable
CORRIDOR_HALF_WIDTH = 1.5  # m either side of the predicted path
PATH_X = 50.             # m: objects up to this far ahead can be in the path
MIN_MODEL_PATH_X = 10.   # m: a model path shorter than this (stopped, crawling) reads as straight ahead
PERSON_HEIGHT = 1.7      # m: a pedestrian's range is the nearer of the ground and height estimates (hidden feet read far)
# A person box that can't be a standing person is dropped, not tracked. Thresholds come from the public demo route (comma four)
# and the BeamNG rlogs.
MIN_PERSON_ASPECT = 1.0  # box h/w: the overpass pillar and bush read 0.45-0.52, real pedestrians 1.8-3.8, the cyclist (a person
                         # box) 1.39-1.64
MAX_PERSON_GROUND_HEIGHT = 12.  # m, ground range x box height / focal length: pedestrians with visible feet read 1.5-2.8 m, the
                                # cyclist, whose box ends a few px under the horizon, 7.6-9.8 m, the pillar 14-33 m. No lower bound:
                                # there's no example of a too-short person box to set one from
# every run's boxes, tracked or not, for the unprotected left's gap rule (unprotected_left.py; CRITIQUE REQUIRED 2): a box's
# nearest plausible range is its ground range with the box bottom this many pixels lower (about 1.5 px at the YOLO input)
NEAR_PX = 3.
FAR_COVER_MARGIN_PX = 16  # a far run covers its bearings only with the horizon this far inside its rows (the band's far cars are
                          # within ~10 px under it)
ACCEL_STD = 3.0          # m/s^2, the filter's process noise
VEL_STD0 = 5.0           # m/s, a new track's velocity uncertainty


def iou(a: np.ndarray, b: np.ndarray) -> np.ndarray:
  """IoU between every box in a (N, 4) and b (M, 4), x0 y0 x1 y1."""
  if len(a) == 0 or len(b) == 0:
    return np.zeros((len(a), len(b)))
  x0 = np.maximum(a[:, None, 0], b[None, :, 0])
  y0 = np.maximum(a[:, None, 1], b[None, :, 1])
  x1 = np.minimum(a[:, None, 2], b[None, :, 2])
  y1 = np.minimum(a[:, None, 3], b[None, :, 3])
  inter = np.clip(x1 - x0, 0, None) * np.clip(y1 - y0, 0, None)
  area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
  area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
  union = area_a[:, None] + area_b[None, :] - inter
  return np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)


def greedy_match(score: np.ndarray, min_score: float) -> list[tuple[int, int]]:
  """Pairs (row, col), best score first, each row and column used once, scores >= min_score only."""
  pairs: list[tuple[int, int]] = []
  if score.size == 0:
    return pairs
  s = score.astype(np.float64)
  while True:
    i, j = np.unravel_index(np.argmax(s), s.shape)
    if not s[i, j] >= min_score:
      return pairs
    pairs.append((int(i), int(j)))
    s[i, :] = -np.inf
    s[:, j] = -np.inf


@dataclass(frozen=True)
class RawBox:
  camera: str              # 'road' or 'wide'
  cls: str                 # TurnpilotScene.ObjectClass name
  conf: float
  bearing_deg: float       # the box's bottom centre, left of the calibrated forward axis
  span_deg: tuple          # (right edge, left edge) bearings at its bottom
  xy: tuple | None         # ground point (x forward, y left), not capped at ranging.MAX_RANGE; None unranged
  near_m: float | None     # nearest plausible range (the box bottom NEAR_PX lower), None unranged
  box: tuple = ()          # x0, y0, x1, y1 normalized in its camera's frame
  view: str = 'full'       # 'full', 'far' for the far crop's (camera 'road'), 'sideLeft' or 'sideRight' (camera 'wide')


@dataclass
class RawFrame:
  boxes: list              # [RawBox]: every detection of a road user in one detector run
  camera: str
  t: float                 # the run's camera time, s
  view: str = 'full'       # 'far': a far crop run (update_far); 'sideLeft', 'sideRight': a stop sweep's side view
  cover_deg: tuple | None = None  # a far run's (right, left) bearings at the horizon; None if the horizon isn't inside it


def view_key(camera: str, view: str) -> str:
  """raw_by_view's key for a run: the full pass on the road camera is 'road', on the wide camera 'full'; a far crop or a
  side view is its view."""
  if view == 'full':
    return 'road' if camera == 'road' else 'full'
  return view


def ground_xy(lens: Lens, rot: np.ndarray, height: float, u: float, v: float) -> tuple[float, float] | None:
  """As ranging.ground_point, with no range cap."""
  ray = rot @ lens.ray(u, v)
  if ray[2] <= 1e-6 or ray[0] <= 0.:
    return None
  return float(ray[0] * height / ray[2]), float(-ray[1] * height / ray[2])


def bearing_deg(lens: Lens, rot: np.ndarray, u: float, v: float) -> float:
  ray = rot @ lens.ray(u, v)
  return float(np.degrees(np.arctan2(-ray[1], ray[0])))


@dataclass
class Measurement:
  box: np.ndarray         # x0 y0 x1 y1, pixels in its camera's frame
  conf: float
  cls: int
  xy: np.ndarray | None = None   # ground position (x forward, y left), None when it can't be ranged
  R: np.ndarray | None = None    # its covariance
  plausible: bool = True         # False for a person box that can't be a standing person; update() drops those
  bearing: float | None = None   # deg left of the calibrated axis of its bottom centre (calibrated runs only)
  span: tuple | None = None      # (right edge, left edge) bearings at its bottom


@dataclass
class Track:
  track_id: int
  camera: str
  box: np.ndarray        # predicted to the latest detection time
  box_seen: np.ndarray   # as last measured
  t_seen: float          # time of the last match, s (camera clock)
  conf: float
  class_votes: dict = field(default_factory=dict)
  box_vel: np.ndarray = field(default_factory=lambda: np.zeros(4))  # px/s
  hits: int = 1
  t: float = 0.0               # time the state is predicted to
  x: np.ndarray | None = None  # x, y, vx, vy: ground position and velocity over the ground, car frame
  P: np.ndarray | None = None
  in_path: bool = False
  far_hits: int = 0            # far crop boxes matching it (update_far): never in hits
  view: str = 'full'           # the pass of its last match: full, far, sideLeft, sideRight
  bearing_seen: float | None = None  # deg left of the calibrated axis of its last match's bottom centre (stop_evidence.py)
  width_seen: float = 0.             # px, its last match's box width in its camera's frame
  span_seen: tuple | None = None     # (right edge, left edge) bearings of its last match's bottom (visibility.py's occluders)

  @property
  def cls(self) -> int:
    return max(self.class_votes, key=lambda c: self.class_votes[c])


def range_noise(xy: np.ndarray) -> np.ndarray:
  """Covariance of a mono ground-plane position: along the ray it grows fast with range, across it slowly."""
  rng = max(float(np.hypot(*xy)), 1e-3)
  along, across = 0.3 + 0.08 * rng, 0.2 + 0.02 * rng
  d = xy / rng
  rot = np.array([[d[0], -d[1]], [d[1], d[0]]])
  return rot @ np.diag([along ** 2, across ** 2]) @ rot.T


def default_yaw_rate(car_state, model_v2) -> float:
  """rad/s, left positive. The driving model's own yaw rate when there is a modelV2 (its z is down, so right positive):
  many ports, Tesla among them, never set carState.yawRate, and a missing yaw rate makes everything beside the road
  look like it's crossing in a turn. On the demo route, -orientationRate.z tracks the Ford's carState.yawRate (r 0.97)."""
  if model_v2 is not None and len(model_v2.orientationRate.z):
    return -float(model_v2.orientationRate.z[0])
  return float(car_state.yawRate) if car_state is not None else 0.0


def predicted_path(model_v2) -> tuple[np.ndarray, np.ndarray]:
  """(x, y left) of the path ahead to PATH_X: modelV2.position, carried on along its last heading past its end, or
  straight ahead when there's no model path or it's too short to have a heading."""
  straight = np.array([0.0, PATH_X]), np.zeros(2)
  if model_v2 is None or len(model_v2.position.x) < 2:
    return straight
  x, y = np.array(model_v2.position.x), -np.array(model_v2.position.y)  # modelV2's y is to the right
  rising = x > np.maximum.accumulate(np.r_[-np.inf, x[:-1]])  # np.interp needs increasing x
  x, y = x[rising], y[rising]
  if len(x) < 2 or x[-1] < MIN_MODEL_PATH_X:
    return straight
  if x[-1] < PATH_X:
    slope = (y[-1] - y[-2]) / (x[-1] - x[-2])
    x, y = np.r_[x, PATH_X], np.r_[y, y[-1] + slope * (PATH_X - x[-1])]
  return x, y


class SceneTracker:
  def __init__(self, device_camera: DeviceCameraConfig, device_type: str = ''):
    """device_camera: DEVICE_CAMERAS[(deviceType, sensor)]. The wide camera's lens follows device_type (the sim's is a pinhole)."""
    self.lenses: dict[str, Lens] = {'road': road_lens(device_camera.narrow_road), 'wide': wide_lens(device_camera.wide_road, device_type)}
    self.lens_spec = self.lenses['wide'].spec  # the wide lens ranging and RawBox bearings use (ranging.py: one lens)
    cloudlog.info(f"turnpilot tracker: wide lens {self.lens_spec}")
    self.sizes = {'road': device_camera.narrow_road.size, 'wide': device_camera.wide_road.size}
    self.tracks: list[Track] = []
    self.next_id = 1
    self.last_key: tuple | None = None
    self.last_t: float | None = None     # camera time of the latest detections, s
    self.last_recv: float | None = None  # the caller's clock when they arrived
    self.available = False
    self.raw: RawFrame | None = None     # the latest run's boxes (raw_frame)
    self.raw_new = False                 # ...which arrived in the last update()
    self.raw_far: RawFrame | None = None  # the latest far crop run's boxes (update_far)
    self.raw_far_new = False              # ...which arrived in the last update_far()
    self.far_key: tuple | None = None
    self.raw_by_view: dict[str, RawFrame] = {}  # the latest run of each view (view_key)
    self.new_views: set[str] = set()            # ...the views a run of which arrived in the last update_runs()
    self.new_runs: list[tuple] = []             # ...and each run, in order: (view key, camera time, RawFrame, a side view's
                                                # sectors [(inner, outer, quality)]), for the stop-sign evidence
    self.coast_s = MAX_COAST_S                  # SWEEP_COAST_S while a stop sweep runs (turnpilotd)

  def measure(self, det, camera: str, calibration) -> Measurement:
    box = np.array([det.x0, det.y0, det.x1, det.y1], dtype=np.float64)
    m = Measurement(box, float(det.conf), int(det.cls))
    if m.cls == PERSON and box[3] - box[1] < MIN_PERSON_ASPECT * (box[2] - box[0]):
      m.plausible = False
      return m
    if calibration is None or len(calibration.rpyCalib) != 3:
      return m
    wide_euler = list(calibration.wideFromDeviceEuler) if camera == 'wide' and len(calibration.wideFromDeviceEuler) == 3 else None
    lens = self.lenses[camera]
    rot = calib_from_view(list(calibration.rpyCalib), wide_euler)
    m.bearing = bearing_deg(lens, rot, (box[0] + box[2]) / 2, box[3])
    m.span = (bearing_deg(lens, rot, box[2], box[3]), bearing_deg(lens, rot, box[0], box[3]))
    xy = ground_point(lens, rot, camera_height(calibration), (box[0] + box[2]) / 2, box[3])
    if xy is None:
      return m
    m.xy = np.array(xy)
    if m.cls == PERSON and box[3] > box[1]:
      rng, rng_h = float(np.hypot(*m.xy)), PERSON_HEIGHT * lens.f / (box[3] - box[1])
      if rng * (box[3] - box[1]) / lens.f > MAX_PERSON_GROUND_HEIGHT:
        m.plausible = False
        return m
      if rng_h < rng:
        m.xy = m.xy * rng_h / rng
    m.R = range_noise(m.xy)
    return m

  @staticmethod
  def predict(trk: Track, t: float, v_ego: float, yaw_rate: float) -> None:
    dt = t - trk.t
    if dt <= 0:
      return
    trk.box = trk.box + trk.box_vel * dt
    trk.t = t
    if trk.x is None or trk.P is None:
      return
    F = np.eye(4)
    F[0, 2] = F[1, 3] = dt
    # the car moves v_ego*dt forward and turns yaw_rate*dt to the left: move the world the other way
    a = -yaw_rate * dt
    R2 = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    G = np.zeros((4, 4))
    G[:2, :2] = G[2:, 2:] = R2
    x = F @ trk.x
    x[:2] -= (v_ego * dt, 0.0)
    q = ACCEL_STD ** 2
    Q = q * np.array([[dt**4 / 4, 0, dt**3 / 2, 0], [0, dt**4 / 4, 0, dt**3 / 2],
                      [dt**3 / 2, 0, dt**2, 0], [0, dt**3 / 2, 0, dt**2]])
    trk.x = G @ x
    trk.P = G @ (F @ trk.P @ F.T + Q) @ G.T

  @staticmethod
  def init_state(trk: Track, m: Measurement) -> None:
    assert m.xy is not None and m.R is not None
    trk.x = np.array([m.xy[0], m.xy[1], 0.0, 0.0])
    trk.P = np.zeros((4, 4))
    trk.P[:2, :2] = m.R
    trk.P[2, 2] = trk.P[3, 3] = VEL_STD0 ** 2

  def correct(self, trk: Track, m: Measurement, camera: str, t: float, view: str = 'full') -> None:
    dt = t - trk.t_seen
    trk.box_vel = 0.5 * trk.box_vel + 0.5 * (m.box - trk.box_seen) / dt if trk.camera == camera and dt > 1e-3 else np.zeros(4)
    trk.box, trk.box_seen, trk.camera, trk.t_seen, trk.conf = m.box, m.box, camera, t, m.conf
    trk.class_votes[m.cls] = trk.class_votes.get(m.cls, 0.0) + m.conf
    trk.hits += 1
    trk.view = view
    trk.bearing_seen, trk.width_seen, trk.span_seen = m.bearing, float(m.box[2] - m.box[0]), m.span
    if m.xy is None:
      return
    if trk.x is None or trk.P is None:
      self.init_state(trk, m)
      return
    H = np.eye(2, 4)
    S = H @ trk.P @ H.T + m.R
    K = trk.P @ H.T @ np.linalg.inv(S)
    trk.x = trk.x + K @ (m.xy - H @ trk.x)
    trk.P = (np.eye(4) - K @ H) @ trk.P

  def new_track(self, m: Measurement, camera: str, t: float, view: str = 'full') -> None:
    trk = Track(self.next_id, camera, m.box, m.box, t, m.conf, {m.cls: m.conf}, t=t, view=view, bearing_seen=m.bearing,
                width_seen=float(m.box[2] - m.box[0]), span_seen=m.span)
    self.init_state(trk, m)
    self.next_id += 1
    self.tracks.append(trk)

  def associate(self, meas: list[Measurement], camera: str) -> tuple[list[tuple[int, int]], list[int]]:
    """(track index, measurement index) pairs, and the unmatched measurements."""
    pairs: list[tuple[int, int]] = []
    free_t, free_m = set(range(len(self.tracks))), set(range(len(meas)))
    if not self.tracks or not meas:
      return pairs, sorted(free_m)
    # score every track against every measurement at once: IoU for boxes in the same camera and class group, the
    # distance between ground positions within the gate
    same_group = np.array([CLASS_GROUP[trk.cls] for trk in self.tracks])[:, None] == np.array([CLASS_GROUP[m.cls] for m in meas])[None]
    same_camera = np.array([trk.camera == camera for trk in self.tracks])[:, None]
    iou_score = np.where(same_group & same_camera, iou(np.array([trk.box for trk in self.tracks]), np.array([m.box for m in meas])), -1.)
    trk_xy = np.array([trk.x[:2] if trk.x is not None else (np.nan, np.nan) for trk in self.tracks])
    m_xy = np.array([m.xy if m.xy is not None else (np.nan, np.nan) for m in meas])
    diff = trk_xy[:, None] - m_xy[None]
    dist = np.hypot(diff[..., 0], diff[..., 1])  # as the pair-by-pair hypot did, to the bit: a near tie goes the same way
    gate = GROUND_GATE_M + GROUND_GATE_FRAC * np.hypot(m_xy[:, 0], m_xy[:, 1])[None]
    with np.errstate(invalid='ignore'):
      ground_score = np.where(same_group & (dist <= gate), -dist, -np.inf)

    def run(ms, score, min_score):
      tracks = sorted(free_t)
      if not tracks or not ms:
        return
      for a, b in greedy_match(score[np.ix_(tracks, ms)], min_score):
        pairs.append((tracks[a], ms[b]))
        free_t.discard(tracks[a])
        free_m.discard(ms[b])

    run(sorted(j for j in free_m if meas[j].conf >= HIGH_CONF), iou_score, IOU_FIRST)
    run(sorted(j for j in free_m if meas[j].conf < HIGH_CONF), iou_score, IOU_SECOND)
    run(sorted(free_m), ground_score, -1e9)
    return pairs, sorted(free_m)

  def update(self, detections, calibration, car_state, model_v2=None, now: float | None = None,
             yaw_rate: float | None = None) -> list[Track]:
    """detections: the latest TurnpilotDetections, new or not (None before the first). calibration: extrinsicsCalibration,
    upstream's name for liveCalibration. car_state: CarState, for vEgo. model_v2: the path for inPath, else straight ahead.
    now: the caller's clock in s, for staleness (default time.monotonic()). yaw_rate: rad/s, left positive (default
    default_yaw_rate()). Returns the tracks to publish."""
    now = time.monotonic() if now is None else now
    view = str(detections.view) if detections is not None else 'full'
    key = (str(detections.camera), view, detections.frameId, detections.timestampEof) if detections is not None else None
    self.raw_new = False
    if key is not None and key != self.last_key and detections.timestampEof > 0:
      self.last_key, self.last_recv = key, now
      t = detections.timestampEof * 1e-9
      camera = 'wide' if str(detections.camera) == 'wide' else 'road'
      vkey = view_key(camera, view)
      try:
        self.raw, self.raw_new = self.raw_frame(detections, camera, calibration, t, view), True
        self.raw_by_view[vkey] = self.raw
      except Exception:
        cloudlog.exception("turnpilot tracker: raw boxes failed")  # the left then has no fresh road run: never clear
        self.raw = None
        self.raw_by_view.pop(vkey, None)
      self.new_views.add(vkey)
      sectors = [(float(q.inner), float(q.outer), str(q.quality)) for q in detections.sectorQuality] if view.startswith('side') else None
      self.new_runs.append((vkey, t, self.raw, sectors))
      v_ego = float(car_state.vEgo) if car_state is not None else 0.0
      if yaw_rate is None:
        yaw_rate = default_yaw_rate(car_state, model_v2)
      for trk in self.tracks:
        self.predict(trk, t, v_ego, yaw_rate)
      meas = [m for m in (self.measure(d, camera, calibration) for d in detections.detections if int(d.cls) in OBJECT_CLASS) if m.plausible]
      pairs, unmatched = self.associate(meas, camera)
      for i, j in pairs:
        self.correct(self.tracks[i], meas[j], camera, t, view)
      for j in unmatched:
        if meas[j].conf >= NEW_TRACK_CONF and meas[j].xy is not None:
          self.new_track(meas[j], camera, t, view)
      self.tracks = [trk for trk in self.tracks if t - trk.t_seen <= max(MAX_AGE_S, self.hold_s(trk))]
      if len(self.tracks) > MAX_TRACKS:  # a burst of false positives: the close-held pedestrian expires last, so it stays
        self.tracks = sorted(self.tracks, key=lambda trk: (trk.t_seen + self.hold_s(trk), trk.hits), reverse=True)[:MAX_TRACKS]
      self.last_t = t
      self.update_in_path(model_v2)
    self.available = self.last_recv is not None and now - self.last_recv <= STALE_S
    return self.published()

  def update_runs(self, runs, calibration, car_state, model_v2=None, now: float | None = None,
                  yaw_rate: float | None = None) -> list[Track]:
    """Every detector run turnpilotd drained this tick (TurnpilotDetections readers, valid ones only), taken in camera-time
    order (module docstring): raw is the newest, raw_new whether any came, new_views the views that did. A run older than the
    newest already taken is dropped, logged: the filter only moves forward. No runs: staleness still ticks (update(None))."""
    self.new_views, self.new_runs = set(), []
    newest = self.last_t
    taken = []
    for d in sorted(runs, key=lambda d: d.timestampEof):
      t = d.timestampEof * 1e-9
      if newest is not None and t < newest:
        cloudlog.event("turnpilot.tracker_run_out_of_order", view=str(d.view), camera=str(d.camera), t=t, newest=newest)
        continue
      newest = t
      taken.append(d)
    if not taken:
      return self.update(None, calibration, car_state, model_v2, now, yaw_rate)
    tracks, any_new = [], False
    for d in taken:
      tracks = self.update(d, calibration, car_state, model_v2, now, yaw_rate)
      any_new = any_new or self.raw_new
    self.raw_new = any_new
    return tracks

  def update_far(self, far, calibration, now: float | None = None) -> None:
    """far: the latest TurnpilotFarDetections (None before the first). A new run comes out as raw_far (raw_far_new), and
    its boxes tag the tracks they match by box (module docstring): nothing else here changes."""
    now = time.monotonic() if now is None else now
    run = far.run if far is not None else None
    key = (run.frameId, run.timestampEof) if run is not None else None
    self.raw_far_new = False
    if key is None or key == self.far_key or run.timestampEof <= 0:
      return
    self.far_key = key
    t = run.timestampEof * 1e-9
    try:
      self.raw_far, self.raw_far_new = self.raw_frame(run, 'road', calibration, t, 'far', tuple(far.crop)), True
      self.raw_by_view['far'] = self.raw_far
    except Exception:
      cloudlog.exception("turnpilot tracker: far boxes failed")  # the left then has no fresh far run: never clear
      self.raw_far = None
      self.raw_by_view.pop('far', None)
      return
    self.tag_far(run, t)

  def tag_far(self, run, t: float) -> None:
    """Tag the road camera's tracks the far run's boxes match by box, predicted to its time (view far, farHits)."""
    tracks = [trk for trk in self.tracks if trk.camera == 'road']
    dets = [d for d in run.detections if int(d.cls) in OBJECT_CLASS]
    if not tracks or not dets:
      return
    boxes = np.array([[d.x0, d.y0, d.x1, d.y1] for d in dets], dtype=np.float64)
    predicted = np.array([trk.box_seen + trk.box_vel * (t - trk.t_seen) for trk in tracks])
    same_group = np.array([CLASS_GROUP[trk.cls] for trk in tracks])[:, None] == np.array([CLASS_GROUP[int(d.cls)] for d in dets])[None]
    for i, _ in greedy_match(np.where(same_group, iou(predicted, boxes), -1.), IOU_FIRST):
      tracks[i].far_hits += 1
      tracks[i].view = 'far'

  def far_cover(self, crop, rot: np.ndarray) -> tuple | None:
    """(right, left) bearings of the far crop's edges at the horizon, None if the horizon isn't FAR_COVER_MARGIN_PX inside it."""
    x0, y0, x1, y1 = crop
    lens = self.lenses['road']
    u, v = lens.project(rot.T @ np.array([1., 0., 0.]))  # the horizon straight ahead: x forward in the calibrated frame
    if not (y0 + FAR_COVER_MARGIN_PX <= v <= y1 - FAR_COVER_MARGIN_PX):
      return None
    row = y0 + FAR_HORIZON_ROW if y0 + FAR_COVER_MARGIN_PX <= y0 + FAR_HORIZON_ROW <= y1 - FAR_COVER_MARGIN_PX else v
    return bearing_deg(lens, rot, x1, row), bearing_deg(lens, rot, x0, row)

  def raw_frame(self, detections, camera: str, calibration, t: float, view: str = 'full', crop=None) -> RawFrame:
    """Every road user's box in this run, ranged where its bottom is below the horizon (RawBox), tracked or not. A far run
    (view far) also says what it covers (far_cover of crop)."""
    boxes = []
    calibrated = calibration is not None and len(calibration.rpyCalib) == 3
    lens, (w, h) = self.lenses[camera], self.sizes[camera]
    rot = None
    if calibrated:
      wide_euler = list(calibration.wideFromDeviceEuler) if camera == 'wide' and len(calibration.wideFromDeviceEuler) == 3 else None
      rot = calib_from_view(list(calibration.rpyCalib), wide_euler)
    height = camera_height(calibration)
    for d in detections.detections:
      cls = int(d.cls)
      if cls not in OBJECT_CLASS or rot is None:
        continue
      u = (d.x0 + d.x1) / 2
      xy = ground_xy(lens, rot, height, u, d.y1)
      near = ground_xy(lens, rot, height, u, d.y1 + NEAR_PX) if xy is not None else None
      boxes.append(RawBox(camera, OBJECT_CLASS_NAME[cls], float(d.conf), bearing_deg(lens, rot, u, d.y1),
                          (bearing_deg(lens, rot, d.x1, d.y1), bearing_deg(lens, rot, d.x0, d.y1)), xy,
                          float(np.hypot(*near)) if near is not None else None,
                          (float(d.x0 / w), float(d.y0 / h), float(d.x1 / w), float(d.y1 / h)), view))
    cover = self.far_cover(crop, rot) if view == 'far' and crop is not None and len(crop) == 4 and rot is not None else None
    return RawFrame(boxes, camera, t, view, cover)

  def update_in_path(self, model_v2) -> None:
    path_x, path_y = predicted_path(model_v2)
    for trk in self.tracks:
      trk.in_path = bool(trk.x is not None and 0 < trk.x[0] <= PATH_X and
                         abs(trk.x[1] - np.interp(trk.x[0], path_x, path_y)) <= CORRIDOR_HALF_WIDTH)

  def hold_s(self, trk: Track) -> float:
    """How long a track is kept after its last match: longer for a pedestrian close in the path who was matched enough
    times; coast_s otherwise (MAX_COAST_S, SWEEP_COAST_S while a stop sweep runs)."""
    close = trk.cls == PERSON and trk.in_path and trk.x is not None and trk.x[0] < CLOSE_HOLD_M and trk.hits >= CLOSE_HOLD_MIN_HITS
    return CLOSE_HOLD_S if close else self.coast_s

  def published(self) -> list[Track]:
    if self.last_t is None:
      return []
    return [trk for trk in self.tracks if trk.hits >= MIN_HITS and trk.x is not None and self.last_t - trk.t_seen <= self.hold_s(trk)]

  def fill_scene(self, scene, tracks: list[Track] | None = None) -> None:
    """Fill a TurnpilotScene builder. Unavailable means stale detections, never "nothing there"."""
    tracks = (self.published() if tracks is None else tracks) if self.available else []
    scene.available = self.available
    out = scene.init('objects', len(tracks))
    for o, trk in zip(out, tracks, strict=True):
      assert trk.x is not None
      w, h = self.sizes[trk.camera]
      o.trackId = trk.track_id
      o.cls = OBJECT_CLASS[trk.cls]
      o.conf = trk.conf
      o.x, o.y, o.vx, o.vy = (float(v) for v in trk.x)
      o.age = trk.hits
      o.inPath = trk.in_path
      o.camera = Camera.wide if trk.camera == 'wide' else Camera.road
      o.view, o.farHits = getattr(View, trk.view, View.full), trk.far_hits
      o.tSeen = float(trk.t_seen)
      o.box = [float(np.clip(v / s, 0, 1)) for v, s in zip(trk.box_seen, (w, h, w, h), strict=True)]
