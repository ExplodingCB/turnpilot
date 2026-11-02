"""maplanes.py and demo_junctions.py on small made-up maps, and on the real west_coast_usa export when it and pyvalhalla are here."""
import importlib.util
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from openpilot.selfdrive.turnpilot.lane_planner import ModelLanes, lane_changes_needed
from openpilot.selfdrive.turnpilot.route_follower import maneuver_from_instruction
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint
from openpilot.selfdrive.turnpilot.unprotected_left import ANNOUNCE
from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource
from openpilot.tools.sim.bridge.beamng import demo_junctions, frame, maplanes

ROOT = Path(__file__).parents[6]
BUILD_TILES = ROOT / "tools/turnpilot/build_valhalla_tiles.sh"
HAVE_VALHALLA = importlib.util.find_spec("valhalla") is not None
EXPORT_READY = Path(maplanes.EXPORT).is_file() and Path(demo_junctions.GRAPH_JSON).is_file()

# x east, y north, in meters: a crossing at J with a 2-lane one-way road from the south
NODES = {1: (0, -900), 2: (0, -500), 3: (0, -250), 4: (0, 0), 5: (0, 300), 6: (-300, 0), 7: (300, 0), 8: (700, 0), 9: (1000, 0),
         10: (1300, 0), 11: (1000, 300)}
PAVED = {"surface": "paved", "highway": "primary"}


def osm_file(path, ways):
  lines = ["<?xml version='1.0' encoding='UTF-8'?>", "<osm version='0.6'>"]
  for i, (x, y) in NODES.items():
    lat, lon, _ = frame.to_latlon(x, y)
    lines.append(f"<node id='{i}' lat='{lat:.8f}' lon='{lon:.8f}'><tag k='ele' v='10'/><tag k='beamng:node' v='n{i}'/></node>")
  for i, (nodes, tags) in enumerate(ways, 1):
    lines.append(f"<way id='{i}'>" + "".join(f"<nd ref='{n}'/>" for n in nodes) + "".join(f"<tag k='{k}' v='{v}'/>" for k, v in tags.items()) + "</way>")
  path.write_text("\n".join(lines + ["</osm>"]))
  return str(path)


def crossing(path, extra=()):
  ways = [
    ([1, 2], {**PAVED, "oneway": "yes", "lanes": "2"}),  # the road in from the south, 900 m in two ways
    ([2, 3, 4], {**PAVED, "oneway": "yes", "lanes": "2"}),
    ([4, 5], {**PAVED, "oneway": "yes", "lanes": "2"}),  # straight on
    ([6, 4], {**PAVED, "lanes": "2", "lanes:forward": "1", "lanes:backward": "1"}),  # west
    ([4, 7], {**PAVED, "lanes": "4", "lanes:forward": "2", "lanes:backward": "2"}),  # east, 2 lanes each way
    ([8, 9], {**PAVED, "oneway": "yes", "lanes": "2"}),  # a second junction: straight on, or left onto a private road
    ([9, 10], {**PAVED, "oneway": "yes", "lanes": "2"}),
    ([9, 11], {**PAVED, "oneway": "yes", "lanes": "1", "access": "private"}),
    *extra,
  ]
  return osm_file(path, ways)


def tags_of(osm, way_id):
  return next(w.tags for w in osm.ways if w.id == way_id)


class TestLaneString:
  def test_exits_decide_the_lanes(self):
    cases = [
      (2, ("left", "through", "right"), "left|through;right"),
      (3, ("left", "through", "right"), "left|through|through;right"),
      (2, ("left", "through"), "left|through"),
      (3, ("left", "through"), "left|through|through"),
      (2, ("through", "right"), "through|through;right"),
      (3, ("through", "right"), "through|through|through;right"),
      (2, ("left", "right"), "left|right"),
      (3, ("left", "right"), "left|left;right|right"),
      (4, ("left", "right"), "left|left|right|right"),
      (2, ("left",), None),  # nothing to choose from
      (2, ("through",), None),
      (1, ("left", "through", "right"), None),  # one lane
    ]
    for n, kinds, expected in cases:
      assert maplanes.lane_string(n, kinds) == expected, (n, kinds)

  def test_turn_kinds(self):
    kinds = {0: "through", 29: "through", -29: "through", 31: "left", 90: "left", 149: "left", -31: "right", -90: "right", 151: None, 180: None}
    for degrees, kind in kinds.items():
      assert maplanes.turn_kind(math.radians(degrees)) == kind, degrees


class TestMakeTags:
  def test_crossing(self, tmp_path):
    osm = maplanes.OsmMap(crossing(tmp_path / "crossing.osm"))
    counts = maplanes.add_turn_lanes(osm)
    assert tags_of(osm, 2)["turn:lanes"] == "left|through;right"  # heading north into J: west is left, east is right
    assert "turn:lanes" not in tags_of(osm, 1)  # no junction at its end
    assert not any(k.startswith("turn:lanes") for k in tags_of(osm, 4))  # one lane each way
    assert not any(k.startswith("turn:lanes") for k in tags_of(osm, 3))  # and a dead end
    assert counts == {"left|through;right": 1, "through|through;right": 1}
    # the two-way road east from J: heading west into J, north is on the right, west goes through
    assert tags_of(osm, 5)["turn:lanes:backward"] == "through|through;right"
    assert "turn:lanes:forward" not in tags_of(osm, 5)  # it ends at a dead end

  def test_private_exits_are_not_exits(self, tmp_path):
    osm = maplanes.OsmMap(crossing(tmp_path / "crossing.osm"))
    maplanes.add_turn_lanes(osm)
    assert not any(k.startswith("turn:lanes") for k in tags_of(osm, 6))  # through, or left onto a private road: no choice

  def test_unpaved_and_private_ways_get_nothing(self, tmp_path):
    for override in ({"access": "private"}, {"surface": "unpaved"}, {"highway": "track"}):
      ways = [([2, 4], {**PAVED, "oneway": "yes", "lanes": "2", **override}), ([4, 5], {**PAVED, "oneway": "yes", "lanes": "2"}),
              ([6, 4], {**PAVED, "lanes": "2", "lanes:forward": "1", "lanes:backward": "1"})]
      osm = maplanes.OsmMap(osm_file(tmp_path / "ways.osm", ways))
      assert maplanes.add_turn_lanes(osm) == {}, override

  def test_main_writes_the_export_with_the_new_tags(self, tmp_path):
    source, out = crossing(tmp_path / "crossing.osm"), tmp_path / "lanes" / "out.osm"
    with patch.object(sys, "argv", ["maplanes", source, str(out)]):
      maplanes.main()
    written = maplanes.OsmMap(str(out))
    assert tags_of(written, 2) == {**PAVED, "oneway": "yes", "lanes": "2", "turn:lanes": "left|through;right"}  # the others as they were
    assert len(written.xy) == len(NODES)


class TestDemoJunctions:
  RADIUS = {f"n{i}": 4.0 for i in NODES}

  def approach(self, tmp_path, kind):
    osm = maplanes.OsmMap(crossing(tmp_path / "crossing.osm"))
    found = [a for a in demo_junctions.approaches(osm, kind) if a[0].id == 2]
    assert len(found) == 1
    return osm, found[0]

  def test_the_road_up_to_the_junction(self, tmp_path):
    _, (way, forward, angle, onto, chain, length) = self.approach(tmp_path, "left")
    assert (forward, round(angle)) == (True, 90)
    assert [w.id for w, _ in chain] == [1, 2] and round(length) == 900
    assert (onto[0].id, onto[1]) == (4, False)  # west, against the way's node order

  def test_start_pose_in_the_curb_lane_for_a_left(self, tmp_path):
    osm, approach = self.approach(tmp_path, "left")
    demo = demo_junctions.make_demo(osm, self.RADIUS, "left", approach, 400.0, 150.0)
    x, y, z = demo.start
    assert (round(x, 2), round(y, 2), z) == (2.0, -400.0, pytest.approx(10.3))  # 8 m wide in 2 lanes: 2 m right of the centreline
    assert (demo.heading, demo.bearing, demo.start_lane, demo.lanes) == (pytest.approx(math.pi / 2), pytest.approx(0.0), 1, 2)
    assert [round(v, 1) for v in demo.destination[:2]] == [-150.0, 0.0] and demo.node == "n4"
    assert demo.onto == "primary" and demo.oneway

  def test_start_pose_beside_the_oncoming_traffic_for_a_right(self, tmp_path):
    osm, approach = self.approach(tmp_path, "right")
    demo = demo_junctions.make_demo(osm, self.RADIUS, "right", approach, 300.0, 80.0)
    assert [round(v, 2) for v in demo.start[:2]] == [-2.0, -300.0] and demo.start_lane == 0
    assert [round(v, 1) for v in demo.destination[:2]] == [80.0, 0.0]

  def test_lanes_split_the_road_evenly(self):
    # a two-way road 7.2 m wide with two lanes each way, heading east: the lane beside the centreline is 0.9 m to its right
    assert demo_junctions.lane_position((0, 0, 0), 0.0, 3.6, 4, 2, 0)[:2] == pytest.approx((0.0, -0.9))
    assert demo_junctions.lane_position((0, 0, 0), 0.0, 3.6, 4, 2, 1)[:2] == pytest.approx((0.0, -2.7))
    assert demo_junctions.lane_position((0, 0, 0), math.pi / 2, 3.6, 4, 2, 1)[:2] == pytest.approx((2.7, 0.0))


def build_tiles(osm, out):
  subprocess.run([str(BUILD_TILES), str(osm), str(out)], env={**os.environ, "VALHALLA_PYTHON": sys.executable}, check=True, capture_output=True)
  return str(out)


@pytest.fixture(scope="module")
def tiles(tmp_path_factory):
  directory = tmp_path_factory.mktemp("lanes")
  lanes = directory / "west_coast_usa_lanes.osm"
  with patch.object(sys, "argv", ["maplanes", maplanes.EXPORT, str(lanes)]):
    maplanes.main()
  return {"plain": build_tiles(maplanes.EXPORT, directory / "plain"), "lanes": build_tiles(lanes, directory / "lanes")}


@pytest.fixture(scope="module")
def demos(tiles):
  osm = maplanes.OsmMap(maplanes.EXPORT)
  radius = {k: v["r"] for k, v in json.loads(Path(demo_junctions.GRAPH_JSON).read_text())["nodes"].items()}
  source = ValhallaSource(tiles["lanes"])
  return {kind: [d for d in demo_junctions.find(osm, radius, source, kind) if not d.verified] for kind in ("left", "right")}


@pytest.mark.skipif(not (HAVE_VALHALLA and EXPORT_READY), reason="needs pyvalhalla and the west_coast_usa export")
class TestWestCoastUSA:
  def test_demo_junctions(self, demos):
    assert len(demos["left"]) >= 3 and len(demos["right"]) >= 2
    for demo in demos["left"] + demos["right"]:
      assert demo.lanes >= 2 and demo.run_m >= demo_junctions.MIN_RUN_M and demo.start_m >= 300

  def test_only_the_lanes_tiles_have_lanes(self, tiles, demos):
    demo = demos["left"][0]
    start, destination = demo.latlon(demo.start)[::-1], demo.latlon(demo.destination)[::-1]
    lanes = {name: ValhallaSource(directory).route(start, destination, demo.bearing, "en") for name, directory in tiles.items()}
    assert [s["distance"] for s in lanes["plain"]] == [s["distance"] for s in lanes["lanes"]]  # the same route, and:
    assert demo_junctions.approach_lanes(lanes["plain"], 0, demo.start_m) == []
    assert len(demo_junctions.approach_lanes(lanes["lanes"], 0, demo.start_m)) == demo.lanes

  def test_navd_fills_the_lanes_before_a_left_and_the_planner_changes_lane(self, tiles, demos):
    from openpilot.selfdrive.navigation.navd import RouteEngine  # needs the params library, which the geometry tests don't
    demo = demos["left"][0]
    # the lane fill is under test, not the left's autonomy: in auto (the default) with the range unmeasured, navd routes round
    # every left the car would skip (routing/lefts.py), this one too (+24 s, its opposing road not known straight out to its
    # R_req), and the route kept has no lane tags at its first left. In announce no left is refused for its range
    values = {"TurnPilotValhallaTiles": tiles["lanes"], "TurnPilotRouteSource": "valhalla", "LanguageSetting": "main_en",
              "TurnPilotAutoLeft": ANNOUNCE}
    params, pm = MagicMock(), MagicMock()
    params.get.side_effect = values.get
    engine = RouteEngine(MagicMock(), pm, params=params, background=False)
    engine.last_position, engine.last_bearing = demo.latlon(demo.start)[::-1], demo.bearing
    engine.calculate_route(demo.latlon(demo.destination)[::-1])
    assert engine.step_idx == 0 and engine.lefts_avoided == 0
    engine.send_instruction()
    instruction = pm.send.call_args.args[1].navInstruction
    assert (instruction.maneuverType, instruction.maneuverModifier) == ("turn", "left")
    lanes = instruction.to_dict()["lanes"]
    assert [lane["active"] for lane in lanes] == [True] + [False] * (demo.lanes - 1)
    assert lanes[0]["activeDirection"] == "left"
    # in the curb lane the model sees one lane to the left and none to the right, the line between them broken white
    # (tp/lanepaint: no change without that paint read): change left once
    paint = (UNKNOWN, LinePaint('broken', 'white', 0.9, 0.3), LinePaint('solid', 'white', 0.9, 0.95), UNKNOWN)
    assert lane_changes_needed(maneuver_from_instruction(instruction), ModelLanes(left=1, right=0), paint=paint) == 1
    assert lane_changes_needed(maneuver_from_instruction(instruction), ModelLanes(left=1, right=0)) == 0  # no paint read
    assert lane_changes_needed(maneuver_from_instruction(instruction), ModelLanes(left=0, right=1), paint=paint) == 0  # in it
