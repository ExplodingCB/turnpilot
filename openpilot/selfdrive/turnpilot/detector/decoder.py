"""YOLO26's (300, 6) output to full-frame detections, and the turnpilotDetections message."""
import numpy as np

from openpilot.cereal import custom
from openpilot.selfdrive.turnpilot.detector.graph import Letterbox

# COCO ids kept: road users only
PERSON, BICYCLE, CAR, MOTORCYCLE, BUS, TRUCK = 0, 1, 2, 3, 5, 7
KEEP_CLASSES = (PERSON, BICYCLE, CAR, MOTORCYCLE, BUS, TRUCK)
# COCO 9, traffic light: decoded apart (decode_lights) for the camera light reader (light_state.py) when
# TurnPilotCameraLights isn't off, into the message's lights, never into its detections: the tracker takes road users only,
# and a junction full of heads never pushes a pedestrian out of the MAX_DETECTIONS
TRAFFIC_LIGHT = 9
MIN_CONF = 0.35
MAX_DETECTIONS = 64  # the message never carries more than this, highest confidence first
MAX_LIGHTS = 16      # traffic light boxes, apart, highest confidence first
# our own hood (and on the wide camera, the dashboard) reads as a car: a box at least this wide (fraction of the frame)
# that reaches the bottom edge. A real vehicle's visible bottom ends at the hood line, so it only drops one closer than that.
HOOD_MIN_WIDTH, HOOD_MIN_BOTTOM = 0.5, 0.97

CAMERAS = {'road': custom.TurnpilotDetections.Camera.road, 'wide': custom.TurnpilotDetections.Camera.wide}


def is_hood(boxes: np.ndarray, w: int, h: int) -> np.ndarray:
  return ((boxes[:, 2] - boxes[:, 0]) >= HOOD_MIN_WIDTH * w) & (boxes[:, 3] >= HOOD_MIN_BOTTOM * h)


def decode(raw: np.ndarray, lb: Letterbox, min_conf: float = MIN_CONF) -> np.ndarray:
  """raw: (300, 6) or (1, 300, 6), x0 y0 x1 y1 in model input pixels, conf, class.
  Returns (N, 6) float32 rows of x0 y0 x1 y1 in full-frame pixels (clipped to the frame), conf, class; highest conf first.
  Drops other classes, low confidence and our own hood."""
  return _decode(raw, lb, min_conf, KEEP_CLASSES, MAX_DETECTIONS)


def decode_lights(raw: np.ndarray, lb: Letterbox, min_conf: float = MIN_CONF) -> np.ndarray:
  """decode()'s rows for traffic lights alone, at most MAX_LIGHTS."""
  return _decode(raw, lb, min_conf, (TRAFFIC_LIGHT,), MAX_LIGHTS)


def _decode(raw: np.ndarray, lb: Letterbox, min_conf: float, classes: tuple, limit: int) -> np.ndarray:
  out = np.asarray(raw, dtype=np.float32).reshape(-1, 6)
  out = out[np.isfinite(out).all(axis=1)]
  cls = np.rint(out[:, 5])
  keep = (out[:, 4] >= min_conf) & np.isin(cls, classes)
  out = out[keep]
  out = out[np.argsort(-out[:, 4], kind='stable')][:limit]
  x0, y0 = lb.to_frame(out[:, 0], out[:, 1])
  x1, y1 = lb.to_frame(out[:, 2], out[:, 3])
  boxes = np.stack([np.clip(x0, 0, lb.cam_w), np.clip(y0, 0, lb.cam_h), np.clip(x1, 0, lb.cam_w), np.clip(y1, 0, lb.cam_h)], axis=1)
  dets = np.concatenate([boxes, out[:, 4:5], np.rint(out[:, 5:6])], axis=1).astype(np.float32)
  return dets[(dets[:, 2] > dets[:, 0]) & (dets[:, 3] > dets[:, 1]) & ~is_hood(dets, lb.cam_w, lb.cam_h)]


def fill_detections(td, dets: np.ndarray, frame_id: int, timestamp_eof: int, camera: str, execution_time: float) -> None:
  """Fill a TurnpilotDetections builder from decode()'s rows."""
  td.frameId = int(frame_id) % 2**32
  td.timestampEof = int(timestamp_eof)
  td.camera = CAMERAS[camera]
  td.executionTime = float(execution_time)
  out = td.init('detections', len(dets))
  for d, (x0, y0, x1, y1, conf, cls) in zip(out, dets, strict=True):
    d.cls, d.conf = int(cls), float(conf)
    d.x0, d.y0, d.x1, d.y1 = float(x0), float(y0), float(x1), float(y1)
