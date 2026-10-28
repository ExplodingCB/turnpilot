import numpy as np

from openpilot.cereal import messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot.detector.decoder import CAR, MAX_DETECTIONS, MAX_LIGHTS, MIN_CONF, PERSON, TRAFFIC_LIGHT, TRUCK, \
  decode, decode_lights, fill_detections
from openpilot.selfdrive.turnpilot.detector.graph import letterbox

LB = letterbox(1344, 760)  # into 640x384: scale 0.476, 11 px of padding top and bottom


def raw_output(rows):
  """A (1, 300, 6) end-to-end output: the given rows, then zero-confidence filler, as YOLO26 pads it."""
  out = np.zeros((1, 300, 6), dtype=np.float32)
  out[0, :len(rows)] = np.asarray(rows, dtype=np.float32).reshape(-1, 6)
  return out


class TestDecoder(OpenpilotTestCase):
  def test_maps_boxes_to_frame_pixels(self):
    x0, y0 = LB.to_input(100.0, 200.0)
    x1, y1 = LB.to_input(300.0, 500.0)
    dets = decode(raw_output([[x0, y0, x1, y1, 0.9, PERSON]]), LB)
    assert dets.shape == (1, 6)
    np.testing.assert_allclose(dets[0, :4], [100, 200, 300, 500], atol=1e-3)
    assert dets[0, 4] == np.float32(0.9) and dets[0, 5] == PERSON

  def test_filters_class_and_confidence(self):
    rows = [[10, 50, 60, 120, 0.9, PERSON],
            [10, 50, 60, 120, 0.9, 9],                     # traffic light: not a road user
            [10, 50, 60, 120, MIN_CONF - 0.01, CAR],
            [10, 50, 60, 120, MIN_CONF, TRUCK],
            [10, 50, 60, 120, np.nan, CAR]]
    dets = decode(raw_output(rows), LB)
    assert sorted(dets[:, 5].tolist()) == [PERSON, TRUCK]

  def test_sorted_by_confidence_and_clipped(self):
    rows = [[-20, 0, 60, 120, 0.5, CAR], [600, 300, 700, 400, 0.8, CAR]]
    dets = decode(raw_output(rows), LB)
    assert dets[0, 4] > dets[1, 4]
    assert dets[:, 0].min() >= 0 and dets[:, 2].max() <= LB.cam_w
    assert dets[:, 1].min() >= 0 and dets[:, 3].max() <= LB.cam_h

  def test_drops_own_hood(self):
    hood = [*LB.to_input(10, 440), *LB.to_input(1330, 755), 0.9, CAR]
    hood_and_dash = [*LB.to_input(0, 212), *LB.to_input(1338, 758), 0.4, CAR]  # as seen on a comma four's wide camera
    truck = [*LB.to_input(100, 100), *LB.to_input(1300, 600), 0.9, TRUCK]   # wide and near, but ends at the hood line
    dets = decode(raw_output([hood, hood_and_dash, truck]), LB)
    assert dets[:, 5].tolist() == [TRUCK]

  def test_only_padding_rows(self):
    assert decode(raw_output([]), LB).shape == (0, 6)

  def test_fill_message(self):
    dets = decode(raw_output([[*LB.to_input(100, 200), *LB.to_input(300, 500), 0.75, PERSON]]), LB)
    msg = messaging.new_message('turnpilotDetections')
    fill_detections(msg.turnpilotDetections, dets, 1234, 5_000_000_000, 'wide', 0.012)
    td = msg.as_reader().turnpilotDetections
    assert td.frameId == 1234 and td.timestampEof == 5_000_000_000 and str(td.camera) == 'wide'
    assert len(td.detections) == 1
    d = td.detections[0]
    assert d.cls == PERSON and abs(d.conf - 0.75) < 1e-6
    np.testing.assert_allclose([d.x0, d.y0, d.x1, d.y1], [100, 200, 300, 500], atol=1e-3)

  def test_lights_decoded_apart(self):
    rows = [[10, 50, 60, 120, 0.9, PERSON], [100, 20, 110, 50, 0.8, TRAFFIC_LIGHT], [200, 20, 210, 50, MIN_CONF - 0.01, TRAFFIC_LIGHT]]
    lights = decode_lights(raw_output(rows), LB)
    assert lights[:, 5].tolist() == [TRAFFIC_LIGHT]
    np.testing.assert_allclose(lights[0, :4], [*LB.to_frame(100, 20), *LB.to_frame(110, 50)], atol=1e-3)
    assert decode(raw_output(rows), LB)[:, 5].tolist() == [PERSON]

  def test_lights_never_displace_road_users(self):
    # a dense signalized scene: the lights' boxes, however confident, never push a road user out of the message
    users = [[10 + i, 50, 60 + i, 120, 0.36 + 0.001 * i, PERSON] for i in range(MAX_DETECTIONS)]
    lights = [[300 + 5 * i, 20, 305 + 5 * i, 40, 0.99, TRAFFIC_LIGHT] for i in range(MAX_LIGHTS + 4)]
    raw = raw_output(lights + users)
    assert len(decode(raw, LB)) == MAX_DETECTIONS
    assert len(decode_lights(raw, LB)) == MAX_LIGHTS
