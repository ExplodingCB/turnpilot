import json
import math
import unittest
from pathlib import Path

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.turnpilot.route_follower import KEEP, NONE, RIGHT, TURN, maneuver_from_instruction
from openpilot.selfdrive.turnpilot.routing.guidance import (MAX_MANEUVERS, approach_lanes, banner_lanes, fill_guidance,
                                                            intersection_lanes, maneuver_junction, through_lanes, to_direction,
                                                            upcoming_maneuvers)

FIXTURES = Path(__file__).parent / 'fixtures'


def steps_of(name):
  """The steps of a response recorded from Valhalla (format=osrm, shape_format=geojson, banner_instructions=true)."""
  return json.loads((FIXTURES / name).read_text())['routes'][0]['legs'][0]['steps']


def lane(directions, active, active_direction='none'):
  return {'directions': directions, 'active': active, 'activeDirection': active_direction}


class TestGuidance(unittest.TestCase):
  def test_directions(self):
    expected = {'left': 'left', 'sharp left': 'left', 'slight left': 'slightLeft', 'straight': 'straight', 'slight right': 'slightRight',
                'sharp right': 'right', 'right': 'right', 'uturn': 'none', 'none': 'none', '': 'none', None: 'none'}
    for indication, direction in expected.items():
      self.assertEqual(to_direction(indication), direction, indication)

  def test_valhalla_left_turn_lanes(self):
    steps = steps_of('valhalla_left_then_right.json')
    # Valhalla only adds the sub banner for the last 400 m of the step, but the first intersection of the next step has the lanes all along.
    left_lane = [lane(['left'], True, 'left'), lane(['straight'], False), lane(['straight', 'right'], False)]
    self.assertEqual(banner_lanes(steps[0], 590), [])
    self.assertEqual(banner_lanes(steps[0], 300), left_lane)
    self.assertEqual(intersection_lanes(steps[1]), left_lane)
    for distance in (590, 300, 20, -4):
      self.assertEqual(approach_lanes(steps, 0, distance), left_lane)
    # A step without lane data on its approach has none, and nor has the last one.
    self.assertEqual(approach_lanes(steps, 1, 200), [])
    self.assertEqual(approach_lanes(steps, 3, 0), [])

  def test_valhalla_right_turn_lanes(self):
    steps = steps_of('valhalla_right_turn.json')
    self.assertEqual(approach_lanes(steps, 0, 500), [lane(['left'], False), lane(['straight'], False), lane(['straight', 'right'], True, 'right')])

  def test_mapbox_sub_banner(self):
    # Mapbox's banner marks the recommended lane active, and gives a valid lane only its active_direction.
    components = [{'type': 'lane', 'directions': ['left', 'sharp left'], 'active': False},
                  {'type': 'lane', 'directions': ['straight', 'slight left'], 'active': False, 'active_direction': 'slight left'},
                  {'type': 'lane', 'directions': ['straight'], 'active': True, 'active_direction': 'straight'},
                  {'type': 'icon', 'text': 'not a lane'}]
    steps = [{'distance': 900, 'bannerInstructions': [{'distanceAlongGeometry': 900, 'primary': {'text': 'Exit'}},
                                                      {'distanceAlongGeometry': 400, 'primary': {'text': 'Exit'}, 'sub': {'components': components}}]},
             {'distance': 100, 'bannerInstructions': []}]
    self.assertEqual(approach_lanes(steps, 0, 700), [])
    self.assertEqual(approach_lanes(steps, 0, 350), [lane(['left'], False), lane(['straight', 'slightLeft'], True, 'slightLeft'),
                                                     lane(['straight'], True, 'straight')])

  def test_mapbox_style_intersection_lanes(self):
    steps = [{'distance': 100, 'bannerInstructions': []},
             {'distance': 100, 'bannerInstructions': [], 'intersections': [
               {'lanes': [{'valid': False, 'indications': ['left']}, {'valid': True, 'indications': ['straight', 'right'], 'valid_indication': 'right'}]},
               {'lanes': [{'valid': True, 'indications': ['straight']}]}]}]
    self.assertEqual(approach_lanes(steps, 0, 50), [lane(['left'], False), lane(['straight', 'right'], True, 'right')])  # the first intersection only

  def test_upcoming_maneuvers(self):
    steps = steps_of('valhalla_left_then_right.json')
    maneuvers = upcoming_maneuvers(steps, 0, 500)
    self.assertEqual([(m['type'], m['modifier']) for m in maneuvers], [('turn', 'left'), ('new name', 'right'), ('arrive', '')])
    # The first is where the current step ends, and each one after it is a step further.
    self.assertEqual([m['distance'] for m in maneuvers], [500, 500 + 445.0, 500 + 445.0 + 257.0])
    self.assertEqual([m['type'] for m in upcoming_maneuvers(steps, 2, 30)], ['arrive'])
    self.assertEqual(upcoming_maneuvers(steps, 3, 0), [])

  def test_maneuvers_are_capped(self):
    steps = [{'distance': 100, 'maneuver': {'type': 'turn', 'modifier': 'left'}} for _ in range(40)]
    maneuvers = upcoming_maneuvers(steps, 5, 30)
    self.assertEqual(len(maneuvers), MAX_MANEUVERS)
    self.assertEqual([m['distance'] for m in maneuvers], [30 + 100 * i for i in range(MAX_MANEUVERS)])  # the nearest ones
    self.assertEqual(len(upcoming_maneuvers(steps, 35, 30)), 4)
    instruction = messaging.new_message('navInstruction').navInstruction
    fill_guidance(instruction, steps, 0, 30)
    self.assertEqual(len(instruction.allManeuvers), MAX_MANEUVERS)

  def test_fill_navinstruction(self):
    steps = steps_of('valhalla_left_then_right.json')
    instruction = messaging.new_message('navInstruction').navInstruction
    fill_guidance(instruction, steps, 0, 300)
    lanes = instruction.to_dict()['lanes']
    self.assertEqual(lanes[0], {'directions': ['left'], 'active': True, 'activeDirection': 'left'})
    self.assertEqual(lanes[2], {'directions': ['straight', 'right'], 'active': False, 'activeDirection': 'none'})
    maneuvers = instruction.to_dict()['allManeuvers']
    self.assertEqual([(m['type'], m.get('modifier', '')) for m in maneuvers], [('turn', 'left'), ('new name', 'right'), ('arrive', '')])
    self.assertAlmostEqual(maneuvers[1]['distance'], 745, places=1)
    fill_guidance(instruction, steps, 3, 0)  # on the last step there is nothing ahead
    self.assertEqual(len(instruction.lanes), 0)
    self.assertEqual(len(instruction.allManeuvers), 0)


  def test_junction_or_bend(self):
    # recorded from Valhalla: a left at a four-way junction, then the road bending right where no other road meets it
    steps = steps_of('valhalla_left_then_right.json')
    self.assertEqual(maneuver_junction(steps, 0), {'before': 90., 'after': 0., 'branches': 2, 'opposing': 'present', 'opposing_rel': 0.})
    bend = maneuver_junction(steps, 1)
    self.assertTrue(math.isnan(bend.pop('opposing_rel')))  # no road across: not known, never 0 (dead ahead)
    self.assertEqual(bend, {'before': 0., 'after': 90., 'branches': 0, 'opposing': 'none'})
    self.assertIsNone(maneuver_junction(steps, 2))  # the arrival has no exit
    self.assertIsNone(maneuver_junction(steps, 3))  # nothing after the last step
    instruction = messaging.new_message('navInstruction').navInstruction
    fill_guidance(instruction, steps, 0, 300)
    self.assertTrue(instruction.maneuverGeometryValid)
    self.assertEqual((instruction.maneuverBearingBefore, instruction.maneuverBearingAfter, instruction.maneuverBranches), (90., 0., 2))
    instruction.maneuverType, instruction.maneuverModifier = 'turn', 'left'
    turn = maneuver_from_instruction(instruction)
    self.assertEqual((turn.kind, turn.angle_deg, turn.bend), (TURN, 90., False))
    fill_guidance(instruction, steps, 1, 300)
    instruction.maneuverType, instruction.maneuverModifier = 'new name', 'right'
    bend = maneuver_from_instruction(instruction)
    self.assertEqual((bend.kind, bend.angle_deg, bend.bend), (NONE, 90., True))  # the model's, not a turn
    fill_guidance(instruction, steps, 2, 0)
    self.assertFalse(instruction.maneuverGeometryValid)

  def test_maneuvers_ahead_carry_their_junctions(self):
    # the lookahead past a bend runs the same junction test as the current maneuver (turnpilot/route_follower.py)
    steps = steps_of('valhalla_left_then_right.json')
    maneuvers = upcoming_maneuvers(steps, 0, 500)
    self.assertEqual([(m['geometryValid'], m['bearingBefore'], m['bearingAfter'], m['branches']) for m in maneuvers],
                     [(True, 90., 0., 2), (True, 0., 90., 0), (False, 0., 0., 0)])
    instruction = messaging.new_message('navInstruction').navInstruction
    fill_guidance(instruction, steps, 0, 300)
    ahead = instruction.allManeuvers[1]
    self.assertEqual((ahead.geometryValid, ahead.bearingBefore, ahead.bearingAfter, ahead.branches), (True, 0., 90., 0))

  def test_one_way_road_in_counts_as_a_branch(self):
    # a T where the other road is one-way toward the junction (entry false): still a road the model could take
    step = {'maneuver': {'type': 'end of road', 'modifier': 'left', 'bearing_before': 114, 'bearing_after': 24},
            'intersections': [{'bearings': [24, 203, 294], 'entry': [True, False, False], 'in': 2, 'out': 0}]}
    self.assertEqual(maneuver_junction([{}, step], 0)['branches'], 1)
    bend = {'maneuver': {'type': 'turn', 'modifier': 'right', 'bearing_before': 20, 'bearing_after': 71},
            'intersections': [{'bearings': [71, 200], 'entry': [True, False], 'in': 1, 'out': 0}]}
    self.assertEqual(maneuver_junction([{}, bend], 0)['branches'], 0)
    self.assertIsNone(maneuver_junction([{}, {'maneuver': {'type': 'turn'}, 'intersections': []}], 0))

  def test_opposing_approach(self):
    # a road leaving the junction straight ahead (within OPPOSING_DEG of the way the car comes in) carries the oncoming
    # traffic a far-side turn crosses; the one-way-away road isn't told apart from a two-way one, so it counts too
    def junction(bearings, entry, i_in, out, before, after):
      step = {'maneuver': {'type': 'turn', 'modifier': 'left', 'bearing_before': before, 'bearing_after': after},
              'intersections': [{'bearings': bearings, 'entry': entry, 'in': i_in, 'out': out}]}
      return maneuver_junction([{}, step], 0)
    # recorded from Valhalla: the left at a four-way junction has a road straight ahead
    self.assertEqual(maneuver_junction(steps_of('valhalla_left_then_right.json'), 0)['opposing'], 'present')
    # a left off the through road of a T, the road going on ahead
    self.assertEqual(junction([0, 90, 270], [True, True, False], 2, 0, 90, 0)['opposing'], 'present')
    # a left from a T's stem: nothing ahead (cross traffic only)
    self.assertEqual(junction([24, 203, 294], [True, False, False], 2, 0, 114, 24)['opposing'], 'none')
    # the road ahead kinks 30 degrees: still ahead; a road 60 degrees off isn't
    self.assertEqual(junction([0, 120, 270], [True, True, False], 2, 0, 90, 0)['opposing'], 'present')
    self.assertEqual(junction([0, 150, 270], [True, True, False], 2, 0, 90, 0)['opposing'], 'none')
    # one-way toward the junction (entry false): oncoming traffic all the same
    self.assertEqual(junction([0, 90, 270], [True, False, False], 2, 0, 90, 0)['opposing'], 'present')
    # into navInstruction, and onto the Maneuver; no junction data is unknown, never none
    instruction = messaging.new_message('navInstruction').navInstruction
    fill_guidance(instruction, steps_of('valhalla_left_then_right.json'), 0, 300)
    self.assertEqual(instruction.maneuverOpposing, 'present')
    self.assertEqual(instruction.allManeuvers[0].opposing, 'present')
    self.assertEqual(instruction.allManeuvers[2].opposing, 'unknown')  # the arrival: no junction
    instruction.maneuverType, instruction.maneuverModifier = 'turn', 'left'
    self.assertEqual(maneuver_from_instruction(instruction).opposing, 'present')
    fill_guidance(instruction, steps_of('valhalla_left_then_right.json'), 2, 0)
    self.assertEqual(instruction.maneuverOpposing, 'unknown')
    blank = messaging.new_message('navInstruction').navInstruction
    blank.maneuverType, blank.maneuverModifier = 'turn', 'left'
    self.assertEqual(maneuver_from_instruction(blank).opposing, 'unknown')

  def test_the_opposing_roads_bearing_off_straight_on(self):
    """The rights review (2026-11-06): opposing present is any other road within OPPOSING_DEG of straight on, so a main road
    bending 25-30 deg left past a lone side road 40 deg to the right read present, and route_follower made that slight left a
    TURN (an autonomous unprotected left held at a line on a through road). A cone on the opposing road's leg off straight on
    was to tell them apart, which navd never sent: maneuver_junction's opposing_rel (+ right), the road nearest straight on
    (the one opposing_leg probes for the left's speed and lanes), NaN with none (never 0: dead ahead), into
    maneuverOpposingRelBearing and each allManeuvers entry's opposingRelBearing. By its bearing alone that road off the far side
    was more often the approach's own road going on, whose oncoming traffic the left crosses (review of 11-06, the module
    docstring), so route_follower's cone is gone (2026-11-07): the slight left across any present approach is a TURN, the road
    off the far side (40, and 30 past the old cone's 25) included, with nothing within OPPOSING_DEG a keep, as navd's message,
    its look-ahead and route time (routing/lefts.py) read it alike."""
    from openpilot.selfdrive.turnpilot.routing.guidance import OPPOSING_DEG, maneuver_entry, opposing_leg
    from openpilot.selfdrive.turnpilot.routing.lefts import route_maneuver
    from openpilot.selfdrive.turnpilot.route_follower import LEFT, maneuver_from_ahead

    def steps(others, after=332., modifier='slight left', i_in=None):
      """The car coming in heading north (in: the leg south, 180), leaving by `after`, with other roads at `others`."""
      bearings = [180., after] + list(others)
      first = {'location': [0., 0.], 'bearings': bearings, 'entry': [False] + [True] * (len(bearings) - 1),
               'in': 0 if i_in is None else i_in, 'out': 1}
      return [{'distance': 300., 'geometry': {'coordinates': [[0., -0.0027], [0., 0.]]}, 'bannerInstructions': []},
              {'distance': 200., 'geometry': {'coordinates': [[0., 0.], [-0.0008, 0.0016]]}, 'bannerInstructions': [],
               'maneuver': {'type': 'turn', 'modifier': modifier, 'bearing_before': 0., 'bearing_after': after},
               'intersections': [first]},
              {'distance': 0., 'geometry': {'coordinates': [[-0.0008, 0.0016], [-0.0008, 0.0016]]}, 'bannerInstructions': [],
               'maneuver': {'type': 'arrive', 'modifier': ''}}]

    def rel(others, **kw):
      return maneuver_junction(steps(others, **kw), 0)['opposing_rel']
    # signed off straight on, + right (route_follower's sense, JunctionApproach.relBearing's): ahead, kinked right, kinked left
    self.assertEqual(rel([0.]), 0.)
    self.assertAlmostEqual(rel([40.]), 40.)
    self.assertAlmostEqual(rel([340.]), -20.)
    # straight on from bearing_before when the leg the car comes in by isn't listed
    self.assertAlmostEqual(rel([20.], i_in=-1), 20.)
    # the road nearest straight on of two, whichever side
    self.assertAlmostEqual(rel([10., 40.]), 10.)
    self.assertAlmostEqual(rel([345., 20.]), -15.)
    # none within OPPOSING_DEG (a road 60 deg off, a T's stem): opposing none, and the bearing not known
    self.assertTrue(math.isnan(rel([60.])) and maneuver_junction(steps([60.]), 0)['opposing'] == 'none')
    self.assertTrue(math.isnan(rel([])))
    # present exactly when the bearing is known, and it is the road opposing_leg probes for the left's R_req
    for b in range(0, 360, 5):
      j = maneuver_junction(steps([float(b)]), 0)
      self.assertEqual(j['opposing'] == 'present', math.isfinite(j['opposing_rel']), b)
      self.assertEqual(math.isfinite(j['opposing_rel']), abs((b + 180.) % 360. - 180.) <= OPPOSING_DEG, b)
      leg = opposing_leg(steps([float(b), (b + 180.) % 360.]), 0)
      j2 = maneuver_junction(steps([float(b), (b + 180.) % 360.]), 0)
      self.assertEqual(leg is None, math.isnan(j2['opposing_rel']), b)
      if leg is not None:
        self.assertAlmostEqual((j2['opposing_rel'] - leg[2] + 180.) % 360. - 180., 0., msg=b)

    # into navInstruction and allManeuvers, and the slight left as turnpilotd and route time read it
    instruction = messaging.new_message('navInstruction').navInstruction
    def same(a, b):
      return a == b or (math.isnan(a) and math.isnan(b))
    # a road off the other side (40, and 30 past the old cone's 25: the approach's own road going on, whose oncoming lanes the
    # left crosses), the road ahead (5, -10), one on the turn's side (-40): each a TURN; and one 60 deg off to the left (outside
    # OPPOSING_DEG: opposing none, so a keep with nothing oncoming to cross)
    for others, kind in (([40.], TURN), ([30.], TURN), ([5.], TURN), ([350.], TURN), ([320.], TURN), ([300.], KEEP)):
      route = steps(others)
      fill_guidance(instruction, route, 0, 100.)
      self.assertTrue(same(instruction.allManeuvers[0].opposingRelBearing, instruction.maneuverOpposingRelBearing), others)
      instruction.maneuverType, instruction.maneuverModifier = 'turn', 'slight left'
      m = maneuver_from_instruction(instruction)
      if math.isfinite(m.opposing_rel_deg or math.nan):
        self.assertAlmostEqual(m.opposing_rel_deg, (others[0] + 180.) % 360. - 180., places=4)
      self.assertEqual(m.side, LEFT, others)
      ahead, at_route_time = maneuver_from_ahead(instruction.allManeuvers[0]), route_maneuver(route, 0)  # routing/lefts.py's
      self.assertTrue(same(ahead.opposing_rel_deg if ahead.opposing_rel_deg is not None else math.nan,
                           instruction.maneuverOpposingRelBearing), others)
      self.assertTrue(same(at_route_time.opposing_rel_deg if at_route_time.opposing_rel_deg is not None else math.nan,
                           instruction.maneuverOpposingRelBearing), others)
      self.assertEqual((m.kind, ahead.kind, at_route_time.kind), (kind, kind, kind), others)
      self.assertTrue(same(maneuver_entry(route, 0, 100.)['opposingRelBearing'], maneuver_junction(route, 0)['opposing_rel']), others)
    fill_guidance(instruction, steps([300.]), 0, 100.)
    self.assertEqual(str(instruction.maneuverOpposing), 'none')
    self.assertTrue(math.isnan(instruction.maneuverOpposingRelBearing))
    # no junction data, and the arrival: NaN (not known), never 0
    fill_guidance(instruction, steps_of('valhalla_left_then_right.json'), 2, 0)
    self.assertTrue(math.isnan(instruction.maneuverOpposingRelBearing))
    fill_guidance(instruction, steps_of('valhalla_left_then_right.json'), 0, 300)
    self.assertEqual(instruction.maneuverOpposingRelBearing, 0.)
    self.assertEqual([a.opposingRelBearing for a in instruction.allManeuvers][0], 0.)
    self.assertTrue(all(math.isnan(a.opposingRelBearing) for a in list(instruction.allManeuvers)[1:]))  # a bend, the arrival
    self.assertTrue(math.isnan(maneuver_entry(steps_of('valhalla_left_then_right.json'), 2, 0.)['opposingRelBearing']))
    # a message nothing filled (a navd before the field, an invalid one) reads NaN: the schema's default
    self.assertTrue(math.isnan(messaging.new_message('navInstruction').navInstruction.maneuverOpposingRelBearing))

  def test_real_angle_and_side(self):
    instruction = messaging.new_message('navInstruction').navInstruction
    instruction.maneuverGeometryValid, instruction.maneuverBranches = True, 1
    for mtype, modifier, before, after, kind, angle in (('turn', 'left', 94., 42., TURN, 52.), ('turn', 'right', 350., 80., TURN, 90.),
                                                        ('turn', 'slight right', 92., 110., KEEP, 18.),
                                                        ('turn', 'sharp right', 10., 160., TURN, 150.)):
      instruction.maneuverType, instruction.maneuverModifier = mtype, modifier
      instruction.maneuverBearingBefore, instruction.maneuverBearingAfter = before, after
      m = maneuver_from_instruction(instruction)
      self.assertEqual(m.kind, kind)
      self.assertAlmostEqual(m.angle_deg, angle)
    self.assertEqual(m.bearing_side, RIGHT)

  def test_through_lanes(self):
    # a straight-through intersection with a left-turn-only lane, 200 m into a 500 m step
    step = {'distance': 500., 'geometry': {'coordinates': [[0., 0.], [0.0045, 0.]]},
            'intersections': [{'location': [0., 0.]},
                              {'location': [0.0018, 0.], 'lanes': [{'indications': ['left'], 'valid': False},
                                                                   {'indications': ['straight'], 'valid': True}]},
                              {'location': [0.003, 0.], 'lanes': [{'indications': ['straight'], 'valid': True}]}]}
    lanes, distance = through_lanes([step], 0, 450.)  # 50 m into the step
    self.assertEqual(lanes, [lane(['left'], False), lane(['straight'], True)])
    self.assertAlmostEqual(distance, 150., delta=1.)
    self.assertEqual(through_lanes([step], 0, 250.), ([], 0.))  # past it; the next has a single lane


if __name__ == '__main__':
  unittest.main()


class TestOpposingRoad(unittest.TestCase):
  """The unprotected left's R_req is from the opposing road's mapped speed limit (unprotected_left.py): one verbose locate per
  route, a point OPPOSING_PROBE_M out along each opposing leg, the edge coming into the junction (routing/controls.py)."""

  def test_the_opposing_leg_of_a_junction(self):
    from openpilot.selfdrive.turnpilot.routing.guidance import opposing_leg
    steps = steps_of('valhalla_left_then_right.json')
    lat, lon, bearing = opposing_leg(steps, 0)
    # the car comes in heading 90 (in: bearing 270 out of the junction); straight across leaves at 90
    self.assertAlmostEqual(bearing, 90.)
    self.assertAlmostEqual(lat, 39.77, places=5)
    self.assertGreater(lon, -86.163)
    self.assertIsNone(opposing_leg(steps, 1))  # a bend: nothing across
    self.assertIsNone(opposing_leg(steps, 3))

  def test_locate_reads_the_inbound_edges_speed_limit_and_lanes(self):
    from openpilot.selfdrive.turnpilot.routing.controls import opposing_roads
    steps = steps_of('valhalla_left_then_right.json')
    calls = []

    def call(endpoint, req):
      calls.append((endpoint, req))
      loc = req['locations'][0]
      inbound = {'heading': loc['heading'], 'distance': 0.5, 'edge': {'lane_count': 1, 'forward': True}, 'edge_info': {'speed_limit': 40}}
      outbound = {'heading': (loc['heading'] + 180) % 360, 'distance': 0.5, 'edge': {'lane_count': 2}, 'edge_info': {'speed_limit': 90}}
      return [{'edges': [outbound, inbound]}]
    roads = opposing_roads(call, steps)
    self.assertEqual(list(roads), [0])  # the left's junction; the bend has no opposing road
    speed, lanes, straight = roads[0]
    self.assertEqual(straight, 0.)  # its straightness probes found nothing here: not known
    self.assertAlmostEqual(speed, 40 / 3.6, places=3)
    self.assertEqual(lanes, 1)
    self.assertEqual(len(calls), 1)  # one batched call
    self.assertEqual(calls[0][1]['locations'][0]['heading'], 270)  # coming into the junction from across
    # untagged (0): unknown; unlimited (255): unknown too (the left takes 45 mph and skips it past the range)
    for limit in (0, 255):
      roads = opposing_roads(lambda e, r, limit=limit: [{'edges': [{'heading': 270, 'distance': 0., 'edge': {'lane_count': 1},
                                                                    'edge_info': {'speed_limit': limit}}]}], steps)
      self.assertEqual(roads[0], (0., 1, 0.))
    # nothing found, or the call failing: no entry (unknown), never a speed
    self.assertEqual(opposing_roads(lambda e, r: [{'edges': []}], steps), {})

    def boom(e, r):
      raise RuntimeError("valhalla down")
    self.assertEqual(opposing_roads(boom, steps), {})

  def test_the_opposing_roads_straight_span_from_our_approach_is_probed_in_the_same_call(self):
    # skeptic G4 (residual): the unprotected left's band is straight on from our approach, and nothing checked the opposing
    # road's shape: on a 400 m radius curving away from the turn an oncoming car 80-135 m up was outside the band. Its straight
    # span: locate every OPPOSING_STRAIGHT_STEP_M on that line, the road coming in found no more than OPPOSING_STRAIGHT_AWAY_M
    # to the side away from the turn and OPPOSING_STRAIGHT_TOWARD_M toward it, out to the last of those in a row
    import math
    from openpilot.selfdrive.turnpilot.routing.controls import (EARTH_M_PER_DEG, OPPOSING_STRAIGHT_MAX_M, OPPOSING_STRAIGHT_RADIUS_M,
                                                                OPPOSING_STRAIGHT_STEP_M, opposing_roads)
    from openpilot.selfdrive.turnpilot.routing.guidance import approach_line
    steps = steps_of('valhalla_left_then_right.json')
    lat0, lon0, heading, side = approach_line(steps, 0)
    self.assertEqual((round(heading), side), (90, 1))  # heading east, a left: the turn's side is our left

    def road(radius=None, toward=1., inbound=True, found=True):
      """A locate over a road that leaves the line on a circle of radius (None straight), toward the turn (1) or away (-1)."""
      def call(endpoint, req):
        out = []
        for loc in req['locations']:
          east = (loc['lon'] - lon0) * EARTH_M_PER_DEG * math.cos(math.radians(lat0))
          north = (loc['lat'] - lat0) * EARTH_M_PER_DEG
          rad = math.radians(heading)
          m = east * math.sin(rad) + north * math.cos(rad)
          y = 0. if radius is None else toward * side * m * m / (2. * radius)  # left of our heading
          lat = loc['lat'] + y * math.sin(rad) / EARTH_M_PER_DEG  # left of heading: (east, north) = (-cos, sin)
          lon = loc['lon'] - y * math.cos(rad) / (EARTH_M_PER_DEG * math.cos(math.radians(lat0)))
          head = loc['heading'] if inbound else (loc['heading'] + 180) % 360
          edge = {'heading': head, 'distance': abs(y), 'correlated_lat': lat, 'correlated_lon': lon, 'edge': {'lane_count': 1},
                  'edge_info': {'speed_limit': 40}}
          out.append({'edges': [edge] if found and abs(y) <= OPPOSING_STRAIGHT_RADIUS_M else []})
        return out
      return call
    self.assertEqual(opposing_roads(road(), steps)[0][2], OPPOSING_STRAIGHT_MAX_M)
    # the probes reach far enough that a straight road never skips a 55 mph left (owner: support 45-55 mph oncoming), lanes
    # unknown taken as two
    from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, reach_at_line
    self.assertLessEqual(reach_at_line(24.6, None) - LINE_D, OPPOSING_STRAIGHT_MAX_M)
    # 400 m away from the turn: 0.8 m off at 25 m, 3.1 m at 50 m (past OPPOSING_STRAIGHT_AWAY_M): straight to 25 m
    self.assertEqual(opposing_roads(road(400., toward=-1.), steps)[0][2], OPPOSING_STRAIGHT_STEP_M)
    # toward the turn the band reaches further: 7 m off at 75 m, 12.5 m at 100 m
    self.assertEqual(opposing_roads(road(400., toward=1.), steps)[0][2], 3 * OPPOSING_STRAIGHT_STEP_M)
    # 4 km away from the turn: 1.95 m off at 125 m, 2.8 m at 150 m
    self.assertEqual(opposing_roads(road(4000., toward=-1.), steps)[0][2], 5 * OPPOSING_STRAIGHT_STEP_M)
    # nothing coming in toward the junction there (a one-way away, or another road's edge): not straight
    self.assertEqual(opposing_roads(road(inbound=False), steps), {})
    self.assertEqual(opposing_roads(lambda e, r: road()(e, r)[:1] + [{'edges': []}] * (len(r['locations']) - 1), steps)[0][2], 0.)

  def test_into_navinstruction(self):
    steps = steps_of('valhalla_left_then_right.json')
    instruction = messaging.new_message('navInstruction').navInstruction
    fill_guidance(instruction, steps, 0, 300, opposing={0: (11.11, 1)})
    self.assertAlmostEqual(instruction.maneuverOpposingSpeed, 11.11, places=3)
    self.assertEqual(instruction.maneuverOpposingLanes, 1)
    self.assertAlmostEqual(instruction.allManeuvers[0].opposingSpeed, 11.11, places=3)
    self.assertEqual(instruction.allManeuvers[1].opposingSpeed, 0.)
    fill_guidance(instruction, steps, 1, 300, opposing={0: (11.11, 1)})
    self.assertEqual((instruction.maneuverOpposingSpeed, instruction.maneuverOpposingLanes), (0., 0))
    # the straight span rides along (0 unknown, and from a road with none)
    self.assertEqual(instruction.maneuverOpposingStraightM, 0.)
    fill_guidance(instruction, steps, 0, 300, opposing={0: (11.11, 1, 175.)})
    self.assertEqual((instruction.maneuverOpposingStraightM, instruction.allManeuvers[0].opposingStraightM), (175., 175.))
    self.assertEqual(instruction.allManeuvers[1].opposingStraightM, 0.)


class TestJunctionSchema(unittest.TestCase):
  """navInstruction's junction for a stop or yield control (MapControl.junctionInfo, STOP_SIGNS.md 3.1, SG1b) and the
  avoidance fields SG2 fills: every field and enum value round-trips, and an unset junction reads none, never ok."""

  def test_every_field_and_enum_round_trips(self):
    msg = messaging.new_message('navInstruction')
    ni = msg.navInstruction
    ni.avoidedControls = 3
    ni.avoidStatus = 'keptBest'
    approach = {'bearing': 270., 'relBearing': -90., 'role': 'exit', 'roadClass': 'secondary', 'use': 'road', 'inbound': True,
                'outbound': True, 'lanesIn': 2, 'lanesOut': 1, 'valhallaControl': 'unknown', 'osmStop': 'all', 'tier': 'mapStrong',
                'taggedLimit': 13.4, 'assumedLimit': 20.1, 'speedSource': 'classDefault', 'wayId': 2 ** 40, 'bikeLane': True,
                'shoulderM': -1.}
    junction = {'status': 'failed', 'osmNodeId': 180786870, 'lat': 39.77, 'lon': -86.16, 'stopPointDistance': 16.7,
                'stopPointSource': 'survey', 'headingIn': 359.5, 'maneuver': 'uTurn', 'turnAngle': -170., 'roundabout': True,
                'divided': True, 'railDistance': 12.5, 'sidecar': True, 'suspected': True, 'surveyed': True, 'validated': True,
                'certification': 'unavoidable', 'certReason': 'notValidated', 'approaches': [approach]}
    ni.controls = [{'id': 1, 'kind': 'stop', 'distance': 40., 'junction': True, 'junctionInfo': junction}]
    back = messaging.log_from_bytes(msg.to_bytes()).navInstruction
    self.assertEqual((back.avoidedControls, str(back.avoidStatus)), (3, 'keptBest'))
    got = back.controls[0].junctionInfo.to_dict()
    for k, v in junction.items():
      if k != 'approaches':
        self.assertAlmostEqual(got[k], v, places=4) if isinstance(v, float) else self.assertEqual(got[k], v, k)
    for k, v in approach.items():
      self.assertAlmostEqual(got['approaches'][0][k], v, places=4) if isinstance(v, float) else self.assertEqual(got['approaches'][0][k], v, k)
    schema = messaging.log.NavInstruction
    self.assertEqual(list(schema.ApproachControl.schema.enumerants), ['none', 'stop', 'yield', 'signal', 'unknown'])
    self.assertEqual(list(schema.OsmStop.schema.enumerants), ['none', 'stop', 'all', 'minor', 'unknown'])
    self.assertEqual(list(schema.AllWayTier.schema.enumerants), ['unknown', 'no', 'mapWeak', 'mapStrong', 'surveyed'])
    self.assertEqual(list(schema.SpeedSource.schema.enumerants), ['survey', 'tagged', 'classDefault'])
    self.assertEqual(list(schema.ApproachRole.schema.enumerants), ['left', 'right', 'opposite', 'exit', 'other'])
    self.assertEqual(list(schema.LineSource.schema.enumerants), ['default', 'mappedLine', 'survey'])
    self.assertEqual(list(schema.Certification.schema.enumerants), ['unchecked', 'certifiable', 'uncertifiable', 'unavoidable'])
    self.assertEqual(list(schema.CertReason.schema.enumerants), ['none', 'noTable', 'modeOff', 'noLaunch', 'geometry', 'skew', 'unknownLeg',
                                                                  'notSurveyed', 'override', 'light', 'notValidated', 'reach'])
    self.assertEqual(list(schema.AvoidStatus.schema.enumerants), ['none', 'clean', 'unavoidable', 'failed', 'keptBest'])

  def test_an_unset_junction_reads_none(self):
    ni = messaging.new_message('navInstruction').navInstruction
    ni.controls = [{'id': 0, 'kind': 'signal', 'distance': 10.}]
    c = ni.controls[0]
    self.assertFalse(c._has('junctionInfo'))
    self.assertEqual((str(c.junctionInfo.status), str(c.junctionInfo.maneuver), str(c.junctionInfo.certification)),
                     ('none', 'unknown', 'unchecked'))
    self.assertEqual(str(ni.avoidStatus), 'none')


class TestFillControls(unittest.TestCase):
  def test_the_junction_rides_with_its_control(self):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    from openpilot.selfdrive.turnpilot.routing.guidance import fill_controls
    from openpilot.selfdrive.turnpilot.routing.junctions import Junction
    ni = messaging.new_message('navInstruction').navInstruction
    fill_controls(ni, [Control(0, 'stop', 50., 0, 50., junction_info=Junction(osm_node_id=7, suspected=True, status='failed')),
                       Control(1, 'signal', 80., 0, 80.)], 'partial', 0.)
    self.assertEqual([(c.junctionInfo.osmNodeId, c.junctionInfo.suspected, str(c.junctionInfo.status)) for c in ni.controls],
                     [(7, True, 'failed'), (0, False, 'none')])


def divided_left(openings, modifier='left', left_entry=False, right_entry=True):
  """Two steps east along the equator (111 m each): the first ends at a turn of modifier (north for a left), and passes
  straight through an intersection each of openings m before it, whose roads north (the left) and south (the right) the car
  may enter or not (OSRM's entry). As Valhalla gives a left onto a divided road: its near carriageway, one-way toward the car
  from the left, crossed first."""
  m_per_deg = 111195.
  crossings = [{'location': [round((111.195 - d) / m_per_deg, 9), 0.], 'bearings': [0, 90, 180, 270],
                'entry': [left_entry, True, right_entry, False], 'in': 3, 'out': 1} for d in openings]
  first = {'location': [0., 0.], 'bearings': [90], 'entry': [True], 'out': 0}
  left = 'left' in modifier
  return [{'geometry': {'coordinates': [[0., 0.], [0.0005, 0.], [0.001, 0.]]}, 'distance': 111.195, 'intersections': [first] + crossings,
           'maneuver': {'type': 'depart', 'bearing_after': 90}},
          {'geometry': {'coordinates': [[0.001, 0.], [0.001, 0.001]]}, 'distance': 111.195,
           'maneuver': {'type': 'turn', 'modifier': modifier, 'bearing_before': 90, 'bearing_after': 0 if left else 180},
           'intersections': [{'location': [0.001, 0.], 'bearings': [0, 180, 270], 'entry': [True, True, False], 'in': 2,
                              'out': 0 if left else 1}]}]


class TestWrongWay(unittest.TestCase):
  """guidance.wrong_way_before: an opening on the turn's side the car may not enter, before the turn's junction, which the
  turn's desire waits past (desire_timing.wrong_way_wait). The first drive's "left onto a one-way at a red light"."""

  def test_a_one_way_toward_the_car_on_the_turns_side_before_it(self):
    from openpilot.selfdrive.turnpilot.routing.guidance import wrong_way_before
    self.assertAlmostEqual(wrong_way_before(divided_left([16.]), 0), 16., delta=0.5)
    self.assertAlmostEqual(wrong_way_before(divided_left([40., 16.]), 0), 16., delta=0.5)  # the one nearest the junction
    # a road the car may enter there is no wrong way (a turn there is a wrong turn, not a one-way, and is left as it was)
    self.assertEqual(wrong_way_before(divided_left([16.], left_entry=True), 0), 0.)
    # on the other side of the turn: none for this one, but it is for a right
    self.assertEqual(wrong_way_before(divided_left([16.], left_entry=True, right_entry=False), 0), 0.)
    self.assertAlmostEqual(wrong_way_before(divided_left([16.], 'right', left_entry=True, right_entry=False), 0), 16., delta=0.5)
    # past the window, no maneuver after the step, a maneuver with no side: none
    self.assertEqual(wrong_way_before(divided_left([70.]), 0), 0.)
    self.assertEqual(wrong_way_before(divided_left([16.]), 1), 0.)
    self.assertEqual(wrong_way_before(divided_left([16.], 'straight'), 0), 0.)

  def test_navd_sends_it_with_the_maneuver_and_turnpilotd_reads_it(self):
    steps = divided_left([16.])
    ni = messaging.new_message('navInstruction').navInstruction
    fill_guidance(ni, steps, 0, 60.)
    self.assertAlmostEqual(ni.maneuverWrongWayM, 16., delta=0.5)
    self.assertAlmostEqual(ni.allManeuvers[0].wrongWayM, 16., delta=0.5)
    self.assertAlmostEqual(maneuver_from_instruction(ni).wrong_way_m, 16., delta=0.5)
