"""Synthetic cereal messages for TurnPilot's tests."""
import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.modeld.constants import ModelConstants

LANE_WIDTH = 3.6


def set_lanes(builder, field, lanes):
  builder.init(field, len(lanes))
  for lane, (directions, active) in zip(getattr(builder, field), lanes, strict=True):
    lane.directions = list(directions)
    lane.active = active


def nav_instruction(distance, mtype="turn", modifier="left", remaining=None, lanes=(), valid=True, junction=None,
                    through=None, route_count=0, ahead=(), controls=(), controls_status=None, off_route=(), off_route_status=None):
  """A navInstruction reader. lanes: ((directions, active), ...) left to right. junction: (bearing_before,
  bearing_after, branches[, opposing[, opposing speed m/s[, opposing lanes]]]) from Valhalla, or None for a route without
  intersection data; opposing is NavInstruction.Opposing's name, unknown when left out; its speed and lanes 0 (unknown). through: (lanes, distance).
  route_count: navd's routeCount. ahead: the maneuvers after this one for allManeuvers, [(distance, type, modifier[,
  junction]), ...]; allManeuvers starts with this one, as navd's does. controls: map stop controls, [(id, kind, distance[,
  line_mapped, junction, cross_suspect]), ...]; controls_status: navd's controlsStatus ('ok' when there are controls).
  off_route, off_route_status: offRouteControls as controls, and offRouteControlsStatus ('ok' when there are controls)."""
  msg = messaging.new_message('navInstruction', valid=valid)
  ni = msg.navInstruction
  ni.routeCount = route_count
  maneuvers = [(distance, mtype, modifier, junction)] + [tuple(a) + (None,) * (4 - len(a)) for a in ahead]
  ni.init('allManeuvers', len(maneuvers))
  for m, (m_distance, m_type, m_modifier, m_junction) in zip(ni.allManeuvers, maneuvers, strict=True):
    m.distance, m.type, m.modifier = float(m_distance), m_type, m_modifier
    if m_junction is not None:
      m.geometryValid = True
      m.bearingBefore, m.bearingAfter, m.branches = m_junction[:3]
      m.opposing = m_junction[3] if len(m_junction) > 3 else 'unknown'
      m.opposingSpeed = float(m_junction[4]) if len(m_junction) > 4 else 0.
      m.opposingLanes = int(m_junction[5]) if len(m_junction) > 5 else 0
  ni.maneuverDistance = float(distance)
  ni.maneuverType = mtype
  ni.maneuverModifier = modifier
  ni.distanceRemaining = float(distance + 500. if remaining is None else remaining)
  if lanes:
    set_lanes(ni, 'lanes', lanes)
  if junction is not None:
    ni.maneuverGeometryValid = True
    ni.maneuverBearingBefore, ni.maneuverBearingAfter, ni.maneuverBranches = junction[:3]
    ni.maneuverOpposing = junction[3] if len(junction) > 3 else 'unknown'
    ni.maneuverOpposingSpeed = float(junction[4]) if len(junction) > 4 else 0.
    ni.maneuverOpposingLanes = int(junction[5]) if len(junction) > 5 else 0
  if through is not None:
    set_lanes(ni, 'throughLanes', through[0])
    ni.throughLanesDistance = float(through[1])
  ni.init('controls', len(controls))
  for c, fields in zip(ni.controls, controls, strict=True):
    c.id, c.kind, c.distance = fields[0], fields[1], float(fields[2])
    c.lineMapped, c.junction, c.crossSuspect = (tuple(fields[3:]) + (False, True, False)[len(fields) - 3:])[:3]
  if controls_status is not None or controls:
    ni.controlsStatus = controls_status or 'ok'
  ni.init('offRouteControls', len(off_route))
  for c, fields in zip(ni.offRouteControls, off_route, strict=True):
    c.id, c.kind, c.distance = fields[0], fields[1], float(fields[2])
    c.lineMapped, c.junction, c.crossSuspect = (tuple(fields[3:]) + (False, True, False)[len(fields) - 3:])[:3]
  if off_route_status is not None or off_route:
    ni.offRouteControlsStatus = off_route_status or 'ok'
  return msg.as_reader().navInstruction


def junction_of(modifier, branches=2, angle=None, opposing='present'):
  """(bearing_before, bearing_after, branches, opposing) for a turn heading north: a right turns to 90, a left to 270 (or
  by angle). opposing: a road straight ahead (a four-way junction's), 'none' for a T's stem."""
  side = -1 if 'left' in modifier else 1
  return 0., (side * (angle if angle is not None else 90.)) % 360., branches, opposing


class FakeNavd:
  """navd's 1 Hz instruction stream for a car driving along a route of maneuvers [(s, type, modifier[, junction]), ...]:
  the distance to the end of the current step, which runs to -10 m before navd moves to the next step. junction:
  (bearing_before, bearing_after, branches), as nav_instruction takes it; without it the route has no junction data.
  through: {step: (lanes, s)}, an intersection the route goes straight through at s with those lanes.
  route_count: navd's routeCount for this route. controls: [(s, kind[, line_mapped, junction, cross_suspect]), ...] stop
  points along the route, sent with ids in route order, -15 m to 400 m ahead and at most 8, nearest first, as navd does;
  controls_status: navd's controlsStatus."""
  def __init__(self, maneuvers, lanes=None, period=20, through=None, route_count=0, controls=(), controls_status=None):
    self.maneuvers = sorted(maneuvers, key=lambda m: m[0])
    self.controls = sorted(controls, key=lambda c: c[0])
    self.controls_status = controls_status
    self.lanes = lanes or {}
    self.through = through or {}
    self.period = period
    self.route_count = route_count
    self.step = 0
    self.frame = 0

  def update(self, s):
    """(instruction or None when not due this frame) for a car at s m along the route."""
    while self.step + 1 < len(self.maneuvers) and self.maneuvers[self.step][0] - s < -10.:
      self.step += 1
    due = self.frame % self.period == 0
    self.frame += 1
    if not due:
      return None
    pos, mtype, modifier = self.maneuvers[self.step][:3]
    junction = self.maneuvers[self.step][3] if len(self.maneuvers[self.step]) > 3 else None
    through = self.through.get(self.step)
    if through is not None:
      through = (through[0], through[1] - s) if through[1] > s else None
    ahead = [(m[0] - s,) + tuple(m[1:]) for m in self.maneuvers[self.step + 1:]]
    controls = [(i, c[1], c[0] - s) + tuple(c[2:]) for i, c in enumerate(self.controls) if -15. < c[0] - s < 400.][:8]
    return nav_instruction(pos - s, mtype, modifier, remaining=self.maneuvers[-1][0] - s, lanes=self.lanes.get(self.step, ()),
                           junction=junction, through=through, route_count=self.route_count, ahead=ahead, controls=controls,
                           controls_status=self.controls_status)


def road(lanes_left, lanes_right, line_prob=0.9, edge_std=0.3, lane_width=LANE_WIDTH, shoulder=0.5):
  """(line_ys, line_probs, edge_ys, edge_stds) as modelV2 sees a straight road: the car centered in its lane with
  lanes_left and lanes_right more lanes beside it; y positive right, as modelV2 publishes it."""
  half = lane_width / 2
  line_ys = [-half - lane_width, -half, half, half + lane_width]
  line_probs = [line_prob if lanes_left else 0.05, line_prob, line_prob, line_prob if lanes_right else 0.05]
  edge_ys = [-half - lanes_left * lane_width - shoulder, half + lanes_right * lane_width + shoulder]
  return line_ys, line_probs, edge_ys, [edge_std, edge_std]


DRIVING_PLAN_V = 5.  # m/s: the speed plan of a model driving on (go_override.model_takes_over: it means to keep moving)


def model_v2(lanes_left=0, lanes_right=0, lane_change_state='off', big=False, lead_prob=None, lead_x=30., holds=False, accel=0.,
             should_stop=None, **kwargs):
  """A modelV2 reader with flat lane lines and road edges from road(); big: from the big (chestnut) driving model;
  lead_prob: leadsV3[0].prob, lead_x m ahead (no leads without it). holds: the model holds the car (action.shouldStop, a
  speed plan of 0, G0's stand); otherwise it drives on (shouldStop false, DRIVING_PLAN_V planned). accel: its
  action.desiredAcceleration; should_stop: its action.shouldStop, if not holds's."""
  line_ys, line_probs, edge_ys, edge_stds = road(lanes_left, lanes_right, **kwargs)
  msg = messaging.new_message('modelV2')
  m = msg.modelV2
  m.action.shouldStop = holds if should_stop is None else should_stop
  m.action.desiredAcceleration = float(accel)
  m.velocity.x = [0. if holds else DRIVING_PLAN_V] * ModelConstants.IDX_N
  x = [float(v) for v in ModelConstants.X_IDXS]
  m.init('laneLines', 4)
  for line, y in zip(m.laneLines, line_ys, strict=True):
    line.x = x
    line.y = [float(y)] * len(x)
  m.laneLineProbs = [float(p) for p in line_probs]
  m.init('roadEdges', 2)
  for edge, y in zip(m.roadEdges, edge_ys, strict=True):
    edge.x = x
    edge.y = [float(y)] * len(x)
  m.roadEdgeStds = [float(s) for s in edge_stds]
  m.meta.laneChangeState = lane_change_state
  m.big = big
  if lead_prob is not None:
    m.init('leadsV3', 3)
    for lead in m.leadsV3:
      lead.prob, lead.x = float(lead_prob), [float(lead_x)] * 6
  return msg.as_reader().modelV2


def car_state(v_ego=10., steering_angle_deg=0., left_blinker=False, right_blinker=False, gas_pressed=False, yaw_rate=0.,
              brake_pressed=False):
  msg = messaging.new_message('carState')
  cs = msg.carState
  cs.yawRate = float(yaw_rate)
  cs.vEgo = float(v_ego)
  cs.steeringAngleDeg = float(steering_angle_deg)
  cs.leftBlinker = left_blinker
  cs.rightBlinker = right_blinker
  cs.gasPressed = gas_pressed
  cs.brakePressed = brake_pressed
  return msg.as_reader().carState


def car_control(lat_active=True, enabled=False):
  msg = messaging.new_message('carControl')
  msg.carControl.latActive = lat_active
  msg.carControl.enabled = enabled
  return msg.as_reader().carControl


def scene(objects=(), available=True):
  """A turnpilotScene reader. objects: dicts of TrackedObject fields (cls, x, y, vx, vy, inPath)."""
  msg = messaging.new_message('turnpilotScene')
  sc = msg.turnpilotScene
  sc.available = available
  sc.init('objects', len(objects))
  for i, (obj, fields) in enumerate(zip(sc.objects, objects, strict=True)):
    obj.trackId = i
    obj.conf = 0.9
    obj.age = 10
    for k, v in fields.items():
      setattr(obj, k, v)
  return msg.as_reader().turnpilotScene
