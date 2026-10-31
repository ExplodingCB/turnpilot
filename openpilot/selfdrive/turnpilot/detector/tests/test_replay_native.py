"""replay_native's count: a schedule's objects on a frame are its passes' boxes, one per IoU-0.5 cluster of a class group."""
import numpy as np

from openpilot.selfdrive.turnpilot.detector.decoder import BUS, CAR, PERSON, TRAFFIC_LIGHT, TRUCK
from openpilot.selfdrive.turnpilot.detector.replay_native import count, union


def rows(*boxes):
  return np.array([[*b, 0.8, cls] for *b, cls in boxes], np.float32).reshape(-1, 6)


def test_union_is_one_object_per_cluster_of_a_group():
  a = rows((100, 100, 200, 200, CAR), (102, 101, 201, 199, TRUCK), (100, 100, 200, 200, PERSON), (500, 100, 520, 120, TRAFFIC_LIGHT))
  objs = union(a)
  assert [g for _, g in objs] == ['car', 'person']  # a truck on the car is one vehicle; a light is neither
  merged = union(rows((0, 0, 10, 10, CAR)), rows((0, 0, 10, 10, BUS), (50, 0, 60, 10, CAR)))
  assert [(b.tolist(), g) for b, g in merged] == [([0, 0, 10, 10], 'car'), ([50, 0, 60, 10], 'car')]


def test_count_says_what_the_slot_added():
  full_pass = rows((100, 300, 200, 400, CAR))
  found, beyond = count({'off': (rows((100, 300, 200, 400, CAR)),), 'full': (rows((10, 350, 14, 360, PERSON)), rows((900, 380, 910, 386, CAR)))},
                        full_pass)
  assert found['off'] == {'car': 1} and beyond['off'] == {}
  assert found['full'] == {'car': 2, 'person': 1} and beyond['full'] == {'car': 1, 'person': 1}
