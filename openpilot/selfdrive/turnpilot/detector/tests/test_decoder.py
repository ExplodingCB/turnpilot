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


class TestFarDecoder(OpenpilotTestCase):
  """The far crop's boxes (graph.Crop): crop input pixels to road frame pixels by the crop's offset, clipped to the crop."""
  def test_maps_by_the_crops_offset(self):
    from openpilot.selfdrive.turnpilot.detector.decoder import decode_far, decode_far_lights
    from openpilot.selfdrive.turnpilot.detector.graph import Crop
    crop = Crop(1344, 760, 352, 120)
    rows = [[100, 200, 112, 208, 0.6, CAR], [-5, 10, 30, 40, 0.5, TRUCK], [300, 20, 310, 50, 0.7, TRAFFIC_LIGHT],
            [600, 300, 660, 400, 0.4, PERSON], [10, 10, 20, 20, MIN_CONF - 0.01, CAR]]
    dets = decode_far(raw_output(rows), crop)
    assert dets[:, 5].tolist() == [CAR, TRUCK, PERSON]
    np.testing.assert_allclose(dets[0, :4], [452, 320, 464, 328], atol=1e-4)
    np.testing.assert_allclose(dets[1, :4], [352, 130, 382, 160], atol=1e-4)  # clipped to the crop, not the frame
    np.testing.assert_allclose(dets[2, :4], [952, 420, 992, 504], atol=1e-4)
    lights = decode_far_lights(raw_output(rows), crop)
    np.testing.assert_allclose(lights[:, :4], [[652, 140, 662, 170]], atol=1e-4)

  def test_far_message(self):
    # turnpilotFarDetections: its run a TurnpilotDetections of the road camera, view far, its crop and aim beside it
    from openpilot.selfdrive.turnpilot.detector.decoder import decode_far, fill_far
    from openpilot.selfdrive.turnpilot.detector.graph import Crop
    crop = Crop(1344, 760, 352, 120)
    dets = decode_far(raw_output([[100, 200, 112, 208, 0.6, CAR]]), crop)
    msg = messaging.new_message('turnpilotFarDetections')
    fill_far(msg.turnpilotFarDetections, dets, crop, 3.5, 77, 9_000_000_000, 0.006)
    fd = msg.as_reader().turnpilotFarDetections
    run = fd.run
    assert str(run.camera) == 'road' and str(run.view) == 'far' and run.frameId == 77 and run.timestampEof == 9_000_000_000
    assert len(run.detections) == 1 and abs(run.detections[0].x0 - 452) < 1e-3 and abs(run.executionTime - 0.006) < 1e-6
    assert list(fd.crop) == [352., 120., 992., 504.] and abs(fd.bearing - 3.5) < 1e-6
    # the full pass's messages say view full and what the far pass is doing
    full = messaging.new_message('turnpilotDetections').turnpilotDetections
    assert str(full.view) == 'full' and str(full.farStatus) == 'unset'


class TestSideDecoder(OpenpilotTestCase):
  def test_stop_signs_never_reach_the_detections(self):
    # COCO 11, decoded apart (decode_signs) like the class-9 lights: the tracker, and so every stop, cap and release, never
    # sees one
    from openpilot.selfdrive.turnpilot.detector.decoder import STOP_SIGN, decode_signs
    rows = [[10, 50, 60, 120, 0.9, STOP_SIGN], [100, 50, 160, 120, 0.8, PERSON], [200, 50, 260, 120, 0.7, TRAFFIC_LIGHT]]
    assert decode(raw_output(rows), LB)[:, 5].tolist() == [PERSON]
    signs = decode_signs(raw_output(rows), LB)
    assert signs[:, 5].tolist() == [STOP_SIGN]
    np.testing.assert_allclose(signs[0, :4], [*LB.to_frame(10, 50), *LB.to_frame(60, 120)], atol=1e-3)

  def test_side_boxes_map_back_to_fisheye_pixels_and_the_dash_is_dropped(self):
    from openpilot.selfdrive.turnpilot.detector.decoder import decode_side
    from openpilot.selfdrive.turnpilot.detector.tests.test_sideview import geometry
    geom = geometry()
    rows = [[300, 160, 380, 215, 0.7, CAR],
            [20, 380, 760, 512, 0.9, CAR],    # our dash: wider than 0.9 of the view and at its bottom
            [20, 300, 700, 400, 0.9, TRUCK],  # wide, but clear of the bottom: a real truck close by
            [300, 160, 380, 215, 0.9, 11],    # a stop sign: never a detection
            [300, 160, 380, 215, 0.2, CAR]]   # too unsure
    for name in ('sideLeft', 'sideRight'):
      dets = decode_side(raw_output(rows), geom, name)
      assert dets[:, 5].tolist() == [TRUCK, CAR], name
      np.testing.assert_allclose(dets[1, :4], geom.box_to_fisheye(name, (300, 160, 380, 215)), atol=1e-3)
      assert np.all(dets[:, :4] >= 0) and np.all(dets[:, [0, 2]] <= geom.cam_w) and np.all(dets[:, [1, 3]] <= geom.cam_h)
