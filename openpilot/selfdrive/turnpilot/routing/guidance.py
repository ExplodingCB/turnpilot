"""The lanes and the upcoming maneuvers for navInstruction, read from Mapbox-shaped route steps.

Mapbox and Valhalla (format=osrm) both put the lanes of a maneuver in two places: the sub banner of the step
before it, and the first intersection of the step that starts with it. Valhalla only adds the sub banner for
the last 400 m of the step, while the intersection has the lanes all along, so the banner comes first (it is
what Mapbox shows) and the intersection fills in the rest. A lane is active if it can be used for the maneuver,
and activeDirection is the direction it takes from there.

The junction of the maneuver is the first intersection of the step that starts with it, as Valhalla gives it
(format=osrm): the bearings of every road there, which one the route comes in by ('in') and leaves by ('out'), and
the maneuver's bearing_before and bearing_after. TurnPilot takes a maneuver as a junction turn only where some other
road meets it (turnpilot/route_follower.py), and its angle from the two bearings.
A far-side turn (a left on the right) crosses the oncoming traffic of the road straight ahead, if there is one: opposing
is present when a road other than the route's in and out leaves the junction within OPPOSING_DEG of straight on, none when
none does. A road one-way away from the junction can't be told from a two-way one (OSRM's entry is true for both), so it
counts: a hold too many, never one too few (turnpilot/map_controls.py). No junction data is unknown, never none. The opposing
road's mapped speed limit and lanes (maneuverOpposingSpeed, maneuverOpposingLanes: the unprotected left's R_req) come from a
locate OPPOSING_PROBE_M out along it (opposing_leg; routing/controls.py: opposing_roads), 0 when unknown.
Within a step the route goes straight through intersections without a maneuver; the next one with a lane list is sent
as throughLanes, so a car in a turn-only lane there is flagged.
"""
import numpy as np

from openpilot.selfdrive.navigation.route import along_path

MAX_MANEUVERS = 10  # allManeuvers is for the planner's lookahead; a long route would otherwise send every step each second
CONTROLS_BEHIND_M, CONTROLS_AHEAD_M = 15., 400.  # navInstruction.controls: the stop signs, yields and signals this near
MAX_CONTROLS = 8
OPPOSING_DEG = 45.  # a road leaving the junction this close to straight on is the oncoming approach
OPPOSING_PROBE_M = 15.  # the opposing road's speed and lanes are read this far out along it (routing/controls.py)


def to_direction(indication) -> str:
  """An OSRM lane indication as a NavInstruction.Direction. Sharp turns count as turns, and u-turns as none."""
  indication = indication or ''
  for direction in ('left', 'right', 'straight'):
    if direction in indication:
      return 'slight' + direction.capitalize() if 'slight' in indication and direction != 'straight' else direction
  return 'none'


def make_lane(indications, usable, active_direction) -> dict:
  return {'directions': list(dict.fromkeys(map(to_direction, indications or []))), 'active': bool(usable),
          'activeDirection': to_direction(active_direction)}


def banner_lanes(step, distance_to_maneuver) -> list[dict]:
  """The lanes in the sub banner navd would show for this step now. A lane is active if it can be used for the maneuver."""
  banners = step.get('bannerInstructions') or []
  if not banners:
    return []
  banner = next((b for b in reversed(banners) if distance_to_maneuver < b['distanceAlongGeometry']), banners[0])
  components = (banner.get('sub') or {}).get('components') or []
  return [make_lane(c.get('directions'), c.get('active') or c.get('active_direction'), c.get('active_direction'))
          for c in components if c.get('type') == 'lane']


def intersection_lanes(step) -> list[dict]:
  """The lanes of the intersection that starts the step, which are the approach lanes of its maneuver."""
  intersections = step.get('intersections') or []
  lanes = (intersections[0].get('lanes') if intersections else None) or []
  return [make_lane(lane.get('indications'), lane.get('valid') or lane.get('active'), lane.get('valid_indication'))
          for lane in lanes]


def approach_lanes(steps, step_idx, distance_to_maneuver) -> list[dict]:
  """The lanes on the approach to the maneuver that ends the current step, empty if the route has none there."""
  lanes = banner_lanes(steps[step_idx], distance_to_maneuver)
  if not lanes and step_idx + 1 < len(steps):
    lanes = intersection_lanes(steps[step_idx + 1])
  return lanes


def maneuver_junction(steps, step_idx) -> dict | None:
  """{'before', 'after' (deg), 'branches'} for the junction of the maneuver that ends step step_idx, or None when the
  route has no intersection data for it (or there is no next step). branches counts every other road at the node,
  one-way roads coming in included: the model doesn't know a road is one-way, and could follow it."""
  if step_idx + 1 >= len(steps):
    return None
  step = steps[step_idx + 1]
  maneuver = step.get('maneuver') or {}
  intersections = step.get('intersections') or []
  if not intersections or maneuver.get('bearing_before') is None or maneuver.get('bearing_after') is None:
    return None
  first = intersections[0]
  bearings, out = first.get('bearings'), first.get('out')
  if not bearings or out is None:
    return None
  i_in = first.get('in')
  others = [i for i in range(len(bearings)) if i not in (i_in, out)]
  # straight on is the way the car comes in: the reverse of its road's bearing out of the junction, or bearing_before
  ahead = (float(bearings[i_in]) + 180.) % 360. if i_in is not None and 0 <= i_in < len(bearings) else float(maneuver['bearing_before'])
  opposing = any(abs((float(bearings[i]) - ahead + 180.) % 360. - 180.) <= OPPOSING_DEG for i in others)
  return {'before': float(maneuver['bearing_before']), 'after': float(maneuver['bearing_after']), 'branches': len(others),
          'opposing': 'present' if opposing else 'none'}


def opposing_leg(steps, step_idx) -> tuple[float, float, float] | None:
  """(lat, lon, bearing) of a point OPPOSING_PROBE_M out along the road straight across the junction of the maneuver that ends
  step step_idx (maneuver_junction's opposing road, the one nearest straight on), bearing its way out of the junction; None
  without one, or without junction data or its location."""
  if step_idx + 1 >= len(steps):
    return None
  step = steps[step_idx + 1]
  maneuver = step.get('maneuver') or {}
  intersections = step.get('intersections') or []
  if not intersections or maneuver.get('bearing_before') is None or not intersections[0].get('location'):
    return None
  first = intersections[0]
  bearings, out, i_in = first.get('bearings'), first.get('out'), first.get('in')
  if not bearings or out is None:
    return None
  ahead = (float(bearings[i_in]) + 180.) % 360. if i_in is not None and 0 <= i_in < len(bearings) else float(maneuver['bearing_before'])
  off = sorted((abs((float(bearings[i]) - ahead + 180.) % 360. - 180.), float(bearings[i]))
               for i in range(len(bearings)) if i not in (i_in, out))
  if not off or off[0][0] > OPPOSING_DEG:
    return None
  b = off[0][1]
  lon, lat = (float(v) for v in first['location'])
  rad = np.radians(b)
  return (lat + OPPOSING_PROBE_M * np.cos(rad) / 111320., lon + OPPOSING_PROBE_M * np.sin(rad) / (111320. * np.cos(np.radians(lat))), b)


def through_lanes(steps, step_idx, distance_to_maneuver) -> tuple[list[dict], float]:
  """(lanes, m to it) for the next intersection ahead on the current step with a lane list of two or more, which the
  route goes straight through; ([], 0) if there is none before the maneuver."""
  step = steps[step_idx]
  coordinates = (step.get('geometry') or {}).get('coordinates') or []
  if len(coordinates) < 2:
    return [], 0.
  path = np.array(coordinates, dtype=np.float64).reshape((-1, 2))
  ego_along = step['distance'] - distance_to_maneuver
  for intersection in (step.get('intersections') or [])[1:]:  # the first is the maneuver that starts the step
    lanes = intersection.get('lanes') or []
    if len(lanes) < 2 or not intersection.get('location'):
      continue
    along = along_path(path, intersection['location'])[0]
    if along > ego_along:
      return [make_lane(lane.get('indications'), lane.get('valid') or lane.get('active'), lane.get('valid_indication'))
              for lane in lanes], along - ego_along
  return [], 0.


def upcoming_maneuvers(steps, step_idx, distance_to_maneuver, opposing=None) -> list[dict]:
  """The next MAX_MANEUVERS maneuvers ahead, with their distance from the ego: the one that ends the current step first.
  Each carries its junction (maneuver_junction), so TurnPilot can look past a bend to the turn after it, and its opposing
  road's speed and lanes (opposing: {step index: (m/s, lanes)}, routing/controls.py: opposing_roads; 0 unknown)."""
  opposing = opposing or {}
  maneuvers = []
  distance = distance_to_maneuver
  for i, ahead in enumerate(steps[step_idx + 1:step_idx + 1 + MAX_MANEUVERS]):
    maneuver = ahead.get('maneuver') or {}
    junction = maneuver_junction(steps, step_idx + i)
    maneuvers.append({'distance': distance, 'type': maneuver.get('type') or '', 'modifier': maneuver.get('modifier') or '',
                      'geometryValid': junction is not None, 'bearingBefore': junction['before'] if junction else 0.,
                      'bearingAfter': junction['after'] if junction else 0., 'branches': min(junction['branches'], 255) if junction else 0,
                      'opposing': junction['opposing'] if junction else 'unknown',
                      'opposingSpeed': float(opposing.get(step_idx + i, (0., 0))[0]),
                      'opposingLanes': min(int(opposing.get(step_idx + i, (0., 0))[1]), 255)})
    distance += ahead['distance']
  return maneuvers


def fill_guidance(instruction, steps, step_idx, distance_to_maneuver, opposing=None):
  """Fill NavInstruction.lanes and allManeuvers for the ego being distance_to_maneuver short of the end of the step.
  opposing: {step index: (m/s, lanes)} of the route's opposing roads (routing/controls.py: opposing_roads)."""
  opposing = opposing or {}
  instruction.lanes = approach_lanes(steps, step_idx, distance_to_maneuver)
  instruction.allManeuvers = upcoming_maneuvers(steps, step_idx, distance_to_maneuver, opposing)
  speed, lanes = opposing.get(step_idx, (0., 0))
  instruction.maneuverOpposingSpeed, instruction.maneuverOpposingLanes = float(speed), min(int(lanes), 255)
  junction = maneuver_junction(steps, step_idx)
  instruction.maneuverGeometryValid = junction is not None
  if junction is not None:
    instruction.maneuverBearingBefore = junction['before']
    instruction.maneuverBearingAfter = junction['after']
    instruction.maneuverBranches = min(junction['branches'], 255)
  instruction.maneuverOpposing = junction['opposing'] if junction is not None else 'unknown'
  instruction.throughLanes, instruction.throughLanesDistance = through_lanes(steps, step_idx, distance_to_maneuver)


def control_msg(c, distance: float, ident: int) -> dict:
  """A NavInstruction.MapControl for a route's or a lookup's control (routing/controls.py), with its junction when it has
  one (a stop or yield: MapJunction, routing/junctions.py)."""
  out = {'id': ident, 'kind': c.kind, 'distance': float(distance), 'lineMapped': c.line_mapped, 'junction': c.junction,
         'crossSuspect': c.cross_suspect}
  info = getattr(c, 'junction_info', None)
  if info is not None:
    out['junctionInfo'] = info.to_msg()
  return out


def fill_controls(instruction, controls, status: str, car_along: float):
  """navInstruction.controls and controlsStatus: the route's controls (routing/controls.py) from CONTROLS_BEHIND_M behind
  the car to CONTROLS_AHEAD_M ahead, nearest first along the route, at most MAX_CONTROLS, their distances in
  maneuverDistance's frame (car_along: the steps before the car's, then along its step), each with its junction."""
  instruction.controlsStatus = status
  near = sorted(((c.route_m - car_along, c) for c in controls if -CONTROLS_BEHIND_M < c.route_m - car_along < CONTROLS_AHEAD_M),
                key=lambda dc: dc[0])[:MAX_CONTROLS]
  instruction.controls = [control_msg(c, d, c.id) for d, c in near]
