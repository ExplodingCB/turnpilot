"""Turn speed: a speed cap that slows the car before the route's turns.

At a junction nothing in openpilot knows which way the route goes, so the car would reach a turn at road
speed. The target speed comes from a lateral accel limit on the corner's radius, estimated from the turn angle
(the junction's real one from Valhalla's bearings, else the modifier's nominal one; route_follower.py), but never
under 25 km/h, with a braking curve back to where the turn desire goes out. That is the speed the big model took
gate 1's turns from (a pulse 40 m out at 6.9 m/s, after which the model slows itself to about 4.5 m/s for the turn).
Braking on to a geometric 4-5 m/s before the pulse put the car at the junction 6 s after it, by which time the
model had dropped the desire and drove straight on (four junctions of four, 2026-10-27). Past the pulse a second
curve brings a junction turn down to the 4.3 m/s gate 1's model chose by itself 15 m out: at junctions where the
model planned the turn but held 7 m/s, it gave the turn up or stopped at the corner. The planner turns the cap into
an accel with a 1 s time constant, so the car trails the curve; the curve leads the car by 2 s.
A keep (a slight turn at a junction) is capped only from town speeds: above KEEP_CAP_SPEED_MAX it is a road bearing
off, and the cap would brake the car at 2 m/s^2 from highway speed for a bend it takes as it is. Ramps and forks are
never capped. Only junction maneuvers are capped at all: a road that bends is the model's (route_follower.py), and a
turn the car is too fast for is skipped, cap and all (turn_skip.py).

The post-go profile (DESIGN phase 4 section 3, G3): a turn a TurnPilot hold lets go of at a standstill (turnpilotd's go)
is driven under its own caps from the go, keyed on the heading turned toward the maneuver (DesireTiming.turned_deg at the
go, then the car's yaw; PostGo). In BeamNG the turns the car pulled away into were missed by going straight at 4-12 m/s,
the model's curvature collapsing as speed rose, or taken swinging past the divider after the launch overshot; capping the
accel removes the overshoot, and the speed cap keeps the car slow while it is still pointed across the junction. The
graduated profile, the default: accel cap 1.0 m/s^2 from the go until 45 deg; speed cap 3.0 m/s until 15 deg, ramping
linearly to 4.3 at 45 deg; from 45 deg the speed cap blends linearly into the maneuver's own corner cap (turn_speed_cap,
where the car is) by 60 deg, and the profile ends there, or sooner where the turn is done (turn_done_deg: a 50 deg junction
is done at 40, as DesireTiming clears its desire, and the car is straight on the exit road). It ends too POST_GO_MAX_M from
the go however little the car has turned: a turn missed straight on (turnpilotd's turnMissed) is not held to 3 m/s down the road. The caps only lower
speed (a positive accel cap, a speed cap braking at no more than -2.0 m/s^2, hooks.plan_accel), so they never brake
hard; they can't start a turn the model holds, which turnpilotd's no-move handback covers.
Profiles by name, so the BeamNG sweep needs no code edit: param TurnPilotPostGoProfile picks the standstill turn's
(none: V1 tapless, no cap at all; flat3: 3.0 m/s to 45 deg; graduated; flat43: 4.3 m/s to 45 deg; the capped ones with
the 1.0 m/s^2 accel cap, all with the same 45-60 deg blend), and TurnPilotLeftGoProfile the unprotected left's (left,
the default: 1.5 m/s^2 and 4.3 m/s flat to 45 deg, DESIGN section 2, because the left starts straight and the slow first
stage would hold it in the oncoming lanes about 1.2 s longer; any of the others for the sweep's comparison cell). The
left's profile gets the same 45-60 deg blend: section 2 says "then the normal corner cap", which for a left of 45 deg or
more is V_CORNER where the car is, the profile's own 4.3.
"""
import math
from dataclasses import dataclass

from openpilot.selfdrive.turnpilot.desire_timing import CLEAR_MARGIN_DEG, CLEAR_MIN_FRACTION, TURN_DESIRE_LEAD_M
from openpilot.selfdrive.turnpilot.route_follower import KEEP, LEFT, TURN

NO_SPEED_CAP = 100.
NO_ACCEL_CAP = 10.
A_LAT_MAX = 2.0         # m/s^2 through the corner
A_DECEL = 1.5           # m/s^2 braking curve into the corner
V_TURN_MIN = 6.9        # m/s (25 km/h), gate 1's speed at the pulse
V_TURN_MAX = 15.0
KEEP_CAP_SPEED_MAX = 15.0  # m/s: a keep is capped only below this
LAG_S = 2.0             # s the curve leads the car
V_CORNER = 4.3          # m/s a junction turn slows to by CORNER_M before the junction (gate 1's model, by itself)
CORNER_M = 15.
# a junction turn this sharp or more gets the V_CORNER curve. With nominal angles that was every left and right (90);
# real ones from the bearings come in under 90 at about half of right-angle junctions (89, 70 in Indiana), and a
# Valhalla "left" or "right" starts at about 45 degrees
CORNER_ANGLE_MIN_DEG = 45.
CORNER_TANGENT_M = 8.0  # m from the corner's start to the junction center, turning to the curb side
FAR_SIDE_TANGENT_M = 12.0  # turning across the junction (left where traffic keeps right) is a wider arc


def corner_tangent(curb_side: bool) -> float:
  return CORNER_TANGENT_M if curb_side else FAR_SIDE_TANGENT_M


def turn_target_speed(angle_deg: float, curb_side: bool) -> float:
  """m/s to take a turn of this unsigned angle."""
  half = math.radians(min(max(angle_deg, 1.), 179.)) / 2
  radius = corner_tangent(curb_side) / math.tan(half)
  return min(max(math.sqrt(A_LAT_MAX * radius), V_TURN_MIN), V_TURN_MAX)


def turn_speed_cap(maneuver, distance: float, v_ego: float, right_hand_traffic: bool = True) -> float:
  """Speed cap for now, from the next maneuver (RouteFollower.upcoming) and the distance to it."""
  town_keep = maneuver is not None and maneuver.kind == KEEP and not maneuver.ramp and v_ego < KEEP_CAP_SPEED_MAX
  if maneuver is None or not (maneuver.kind == TURN or town_keep):
    return NO_SPEED_CAP  # ramps and forks: braking on the main road is wrong, and the model follows the ramp's curve
  curb_side = (maneuver.side == LEFT) != right_hand_traffic
  v_turn = turn_target_speed(maneuver.angle_deg, curb_side)
  lag = max(v_ego, 0.) * LAG_S
  cap = math.sqrt(v_turn ** 2 + 2. * A_DECEL * max(distance - TURN_DESIRE_LEAD_M - lag, 0.))
  if maneuver.kind == TURN and maneuver.angle_deg >= CORNER_ANGLE_MIN_DEG:
    cap = min(cap, math.sqrt(V_CORNER ** 2 + 2. * A_DECEL * max(distance - CORNER_M - lag / 2., 0.)))
  return min(cap, NO_SPEED_CAP)


POST_GO_RAMP_START_DEG = 15.  # the speed cap is the profile's v_start up to here...
POST_GO_CAP_END_DEG = 45.     # ...ramping to its v_end here, where the accel cap ends...
POST_GO_BLEND_END_DEG = 60.   # ...then blended into the corner cap by here, where the profile ends
POST_GO_MAX_M = 60.           # m from the go: a 20 m hold, a 90 deg arc of 12 m (19 m), and room; past it the profile ends
POST_GO_DEFAULT = 'graduated'  # TurnPilotPostGoProfile unset or unknown
LEFT_GO_PROFILE = 'left'       # TurnPilotLeftGoProfile unset or unknown


@dataclass(frozen=True)
class PostGoProfile:
  accel_cap: float = NO_ACCEL_CAP  # m/s^2 from the go until POST_GO_CAP_END_DEG
  v_start: float = NO_SPEED_CAP    # m/s up to POST_GO_RAMP_START_DEG
  v_end: float = NO_SPEED_CAP      # m/s at POST_GO_CAP_END_DEG

  @property
  def capped(self) -> bool:
    return self.v_start < NO_SPEED_CAP or self.accel_cap < NO_ACCEL_CAP


POST_GO_PROFILES = {
  'none': PostGoProfile(),                    # V1 tapless: the model alone
  'flat3': PostGoProfile(1.0, 3.0, 3.0),      # V3's cap, without the gas tap
  'graduated': PostGoProfile(1.0, 3.0, 4.3),
  'flat43': PostGoProfile(1.0, 4.3, 4.3),
  'left': PostGoProfile(1.5, 4.3, 4.3),       # the unprotected left (DESIGN section 2)
}


def turn_done_deg(angle_deg: float) -> float:
  """deg turned at which a turn of this angle is done: DesireTiming's clear before the junction."""
  return max(angle_deg - CLEAR_MARGIN_DEG, CLEAR_MIN_FRACTION * angle_deg)


def post_go_caps(profile: PostGoProfile, turned_deg: float, corner_cap: float) -> tuple[float, float]:
  """(accel cap, speed cap) of profile turned_deg into the turn from its go; corner_cap: the maneuver's own speed cap where
  the car is, which the speed cap blends into from POST_GO_CAP_END_DEG to POST_GO_BLEND_END_DEG."""
  if not profile.capped or turned_deg >= POST_GO_BLEND_END_DEG:
    return NO_ACCEL_CAP, NO_SPEED_CAP
  if turned_deg < POST_GO_CAP_END_DEG:
    f = min(max((turned_deg - POST_GO_RAMP_START_DEG) / (POST_GO_CAP_END_DEG - POST_GO_RAMP_START_DEG), 0.), 1.)
    return profile.accel_cap, profile.v_start + (profile.v_end - profile.v_start) * f
  f = (turned_deg - POST_GO_CAP_END_DEG) / (POST_GO_BLEND_END_DEG - POST_GO_CAP_END_DEG)
  return NO_ACCEL_CAP, profile.v_end + (min(corner_cap, NO_SPEED_CAP) - profile.v_end) * f


class PostGo:
  """A turn let go of at a standstill, under profile from its go: maneuver, the m to its junction and the deg turned toward
  it (DesireTiming.turned_deg) at the go. update() each frame with the car's speed and yaw rate; over once the profile is
  done (module docstring)."""
  def __init__(self, profile: PostGoProfile, maneuver, distance: float, turned_deg: float, right_hand_traffic: bool = True):
    self.profile, self.maneuver, self.right_hand_traffic = profile, maneuver, right_hand_traffic
    self.distance_at_go, self.turned_deg, self.travelled_m = distance, turned_deg, 0.

  @property
  def over(self) -> bool:
    return not self.profile.capped or self.turned_deg >= min(POST_GO_BLEND_END_DEG, turn_done_deg(self.maneuver.angle_deg)) or \
      self.travelled_m > POST_GO_MAX_M

  def update(self, v_ego: float, yaw_rate: float, dt: float) -> None:
    self.travelled_m += max(v_ego, 0.) * dt
    self.turned_deg += math.degrees(yaw_rate * dt) * self.maneuver.side

  def caps(self, v_ego: float) -> tuple[float, float]:
    if self.over:
      return NO_ACCEL_CAP, NO_SPEED_CAP
    corner = turn_speed_cap(self.maneuver, self.distance_at_go - self.travelled_m, v_ego, self.right_hand_traffic)
    return post_go_caps(self.profile, self.turned_deg, corner)
