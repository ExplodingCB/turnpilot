"""Turn speed: a speed cap that slows the car before the route's turns.

At a junction nothing in openpilot knows which way the route goes, so the car would reach a turn at road
speed. The target speed comes from a lateral accel limit on the corner's radius, estimated from the turn angle
(the junction's real one from Valhalla's bearings, else the modifier's nominal one; route_follower.py), but never
under 25 km/h, with a braking curve back to TURN_CAP_LEAD_M (40 m), where gate 1's turn desire went out and a far-side
turn's still does at that speed. That is the speed the big model took gate 1's turns from (a pulse 40 m out at 6.9 m/s,
after which the model slows itself to about 4.5 m/s for the turn). The curve is its own constant, not the desire's lead:
a curb-side turn's one pulse goes at 8 m and a far-side one's comes in with speed (desire_timing.turn_lead, 2026-11-06),
and the cap still has the car at 6.9 m/s by 40 m.
Braking on to a geometric 4-5 m/s before the pulse put the car at the junction 6 s after it, by which time the
model had dropped the desire and drove straight on (four junctions of four, 2026-10-27). Past the pulse a second
curve, the corner curve, brings a junction turn of CORNER_ANGLE_MIN_DEG or more down to its corner speed: at junctions where
the model planned the turn but held 7 m/s, it gave the turn up or stopped at the corner. The planner turns the cap into
an accel with a 1 s time constant, so the car trails the curve; the curve leads the car by 2 s.
The corner speed comes from the corner (2026-11-07; Kellen: "not all turns should be taken at 7mph"). Until then it was one
number a side: 4.3 m/s by 15 m for a far-side turn (gate 1's model, by itself) and 3.0 by 20 m for a curb-side one (BeamNG:
every right taken, R6 right_green and the g62 rights, came to its junction at 3.5 m/s or less from 15 m out; every one lost,
S3, C5 and F3 right_green, at 4.6-4.7 with the 4.3 cap binding, began turning 4-6 m out and was given up, stood in the corner
or driven straight), whatever the junction's angle. The evidence, every turn of the three drives on disk (0000003, 1d, 1e:
30 turns of 30 deg or more on a radius under 50 m, 25 by hand, the 5 engaged ones all before the 8 m pulse):
- the radius: the human's 60-100 deg rights went round a median 7.6 m (4.9-12.8, 11 turns), the lefts 9.9 m (7.6-19.7, 6),
  so a right angle's corner is the tangent's 8 m to the curb side and 12 m across (corner_tangent, which turn_skip shares;
  over the human's 9.9 m on purpose: the far side's ceiling, below); sharper turns, 100-176 deg, went round 5.6-13.7 m
  (median 8.4, 8 turns), about the right angle's, not the 3.3 m tangent / tan(angle / 2) gives a 135 deg right, so the angle
  is taken as 90 at most (corner_radius);
- the comfort: the human's 12 rolling turns (not from a stand) went through their cores at a median 1.82 m/s^2 of lateral
  accel (p10 1.59, p90 2.24, the gentlest 1.55): A_LAT_CORNER. The cap is a ceiling, and where the model knows the turn it
  slows by itself: on 1e's 52 deg right at 6:41, its desire 36 m out, the model planned 4.5-5.0 m/s for the corner and
  through it, held at 3.0, asked +0.6 to +1.1 m/s^2; on 7:14's 90 deg right it chose 2.1-2.5 under the 3.0 cap;
- the curb side's one pulse: a right is told at 8 m (desire_timing.CURB_TURN_PULSE_M), so its speed there is TurnPilot's, and
  the model starts round up to MODEL_START_S (1.5 s) after it. The right angle's speed is the evidence's: the 3.0 the BeamNG
  rights were taken at and the car's engaged rights were driven at (1e 8:28's 91 deg right at 2.8-3.0 on an 8-10 m radius;
  6:41's 52 deg one at 3.0, the driver's hand light on the wheel), every one of them told early (the 40 m pulse, a repeat, a
  re-engage): no right on disk was pulsed at 8 m, at any speed. one_pulse_speed carries it to the other angles, and is a
  bound relative to the right angle, not an absolute one (2026-11-07 review). Its sum, v^2 * tan(angle / 2) =
  A_LAT_TURN_MAX * (CURB_TURN_PULSE_M - v * MODEL_START_S), puts the right angle at 2.97, the 3.0 again, as the edge where it
  just makes its corner from the latest start at the most lateral accel the human used at a junction (A_LAT_TURN_MAX, 2.5),
  and every angle at that same edge: 3.3 at 70, 3.5 at 60, 3.6 at 52, 3.8 at 45. So wherever the pulse really lands, however
  soon the model starts, at whatever lateral accel up to 2.5, a shallower right at its speed comes out of its corner no wider
  than the right angle does at 2.97 (a pulse a metre late puts its exit sin(angle) of a metre wider, the right angle's a
  whole metre; test_turn_speed_corner over landings 3-9 m out, with the car's full lock and the right lane's corner, 1.8 m x
  tan(angle / 2) short of navd's centre-line node, or without). As a bound of its own the sum is no guarantee at any angle:
  desire_timing's replay of 1d/1e has the pulse land at the furthest reading's 8 m (that reading 1.5-2 m long) or the
  latest's 6 m (CURB_LATEST_PULSE_M), about 6-6.5 m from the corner, where it gives the right angle 2.4-2.6, and the right
  angle at 2.97 from the latest start comes out 1.5-2 m wide at 2.5 m/s^2 (2.3-2.8 m on the car's full lock, about a 4.3 m
  radius; even from 8 m that start asks 3.5 m, under it), which is the pulse's placement, not this speed. On 1e 7:14 the
  model, its curvature 0.078 1/m 6 m out at 2.2 m/s, was late for the corner and the driver turned it on 7 m at 3.5-3.7 with
  the gas: no car evidence takes a right faster from a pulse that late, so the tight right stays where it was;
- where: a curb-side turn is at its corner speed by CORNER_M (15 m), as a far-side one has been, not by 20 m. The 3.0 cap by
  20 m (set for the 40 m pulse) was on from 22-23 m out on 1e's rights and had the car at 3.5 m/s or under from 20-21 m,
  6.7-8 s to the junction at about 7 mph, where the human was at 5.9 m/s 20 m out and 4.7 at 8 m (the median of the human's 4
  rolling rights). Now 4.5 m/s at 20 m, 3.4 at 15 m (BeamNG's rights taken came at 3.5 or less from there) and 3.0 at the
  pulse 8 m out. Not later: with the curve at its corner speed only by the pulse, the car, trailing the cap by the planner's
  1 s, is still braking there at 3.4 m/s, over its corner speed (one_pulse_speed) as the model is told;
- the far side's ceiling: a left is told 6 s out (desire_timing.turn_lead), so the model knows it and slows by itself, and its
  cap is a ceiling over the model's own speed, not the evidence's median. The evidence gives 4.1-4.5 m/s at a right angle (the
  human's 60-100 deg lefts went round a median 9.9 m, where A_LAT_CORNER gives 4.2, and through at a median 4.1, 3.7-6.2;
  gate 1's model, no corner cap, slowed itself to 4.3-4.5), and a cap under the model's own 4.5 (4.2 from the human's
  radius, the old 4.3) would hold every right-angle left under the model's own plan (Kellen, 2026-11-07: "worried about it
  overslowing before turns"). So the far side keeps the 12 m tangent (turn_skip's, not a fit to the human): 4.65, over the
  model's 4.5, and on the human's own 9.9 m radius 2.18 m/s^2, under the human's p90 (2.24) and A_LAT_TURN_MAX. Over the old
  4.3 it raises every rolling left with no oncoming (a T's stem, a one-way), every sharp left, and the unprotected left's
  post-go profile, which now rises from its 4.3 to 4.65 between 45 and 60 deg turned, mid-junction (below).
So a junction turn's corner speed is sqrt(A_LAT_CORNER * corner_radius) and a curb-side one's no more than one_pulse_speed
(corner_speed): to the curb side 2.97 m/s at 90 deg (3.0 before), 3.5 at 60, 3.8 at 45; across 4.65 at 90 (4.3 before), 6.1
at 60, 7.2 at 45. navd sends nothing of the exit road (no class, lanes or width); the one junction turn off a bigger road,
1d 10:15's left off a 35 mph road with two lanes each way (maneuverOpposingLanes 2), went round 11.1 m like the town lefts,
so no lane count widens a corner here.
A keep (a slight turn at a junction) is capped only from town speeds: above KEEP_CAP_SPEED_MAX it is a road bearing
off, and the cap would brake the car at 2 m/s^2 from highway speed for a bend it takes as it is. But one the route's
bearings put at a junction (geometry: a branch meets it there) at KEEP_CORNER_MIN_DEG or more is a corner at any speed
(BUILD_FINDINGS D5: S19's reroute slight left was taken at 19 m/s, the bend after it asking 3.8 m/s^2 of lateral
accel). Ramps and forks are never capped. Only junction maneuvers are capped at all: a road that bends is the model's (route_follower.py), and a
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
more is its corner speed where the car is (corner_speed: V_CORNER, 4.65 m/s, at 90 deg; 4.3, the profile's own, before
2026-11-07), so the blend carries it from the profile's 4.3 up to that, on purpose (the far side's ceiling, above): past
45 deg turned, clearing the oncoming lanes, the car is no longer held under the model's own speed, and one the model has
slowed under the ceiling stays as slow.
"""
import math
from dataclasses import dataclass

from openpilot.selfdrive.turnpilot.desire_timing import CLEAR_MARGIN_DEG, CLEAR_MIN_FRACTION, CURB_TURN_PULSE_M
from openpilot.selfdrive.turnpilot.hooks import SPEED_CAP_TAU
from openpilot.selfdrive.turnpilot.route_follower import KEEP, LEFT, TURN

NO_SPEED_CAP = 100.
NO_ACCEL_CAP = 10.
A_LAT_MAX = 2.0         # m/s^2 through the corner at the 40 m target (turn_target_speed), and turn_skip's A_LAT_SAFE
A_DECEL = 1.5           # m/s^2 braking curve into the corner
V_TURN_MIN = 6.9        # m/s (25 km/h), gate 1's speed at the pulse
V_TURN_MAX = 15.0
KEEP_CAP_SPEED_MAX = 15.0  # m/s: a keep is capped only below this...
KEEP_CORNER_MIN_DEG = 20.  # ...or at any speed at a junction (its bearings, a branch) this sharp or more
LAG_S = 2.0             # s the curve leads the car
TURN_CAP_LEAD_M = 40.   # m: the braking curve reaches the turn's target speed here (gate 1's pulse; module docstring)
# The corner (module docstring, 2026-11-07: every turn of the drives on disk, 0000003/1d/1e): a junction turn's corner speed
# is sqrt(A_LAT_CORNER * corner_radius), and a curb-side one's never over one_pulse_speed, the speed at which its one 8 m
# pulse leaves it no less room than the right angle's at V_CORNER_CURB (a bound relative to the right angle, not absolute)
A_LAT_CORNER = 1.8      # m/s^2: the median of the human's 12 rolling junction turns through their cores (p10 1.59, p90 2.24)
A_LAT_TURN_MAX = 2.5    # m/s^2: the hardest junction corner the human drove (1d 10:15, 2.52 through its core): the most the
                        # model, trained on people, is taken to turn at after its latest start
MODEL_START_S = 1.5     # s: the latest the model started a right after a fresh edge on 1e (desire_timing: 0.3-1.5 s, edges
                        # into a model the 40 m pulse had told; the same figure turn_skip's CURB_FIRST_PULSE_M deadline is set from)
CORNER_RADIUS_MAX_DEG = 90.  # sharper than this the human's radius was the right angle's: 8 sharp turns (101-176 deg) at 5.6-13.7 m
CORNER_M = 15.          # m: a junction turn is at its corner speed by here (gate 1's model, by itself, 4.3-4.5 m/s from 15 m
                        # out; BeamNG's rights taken came at 3.5 m/s or less from 15 m out), 7 m before a right's one pulse
# a junction turn this sharp or more gets the corner curve. With nominal angles that was every left and right (90);
# real ones from the bearings come in under 90 at about half of right-angle junctions (89, 70 in Indiana), and a
# Valhalla "left" or "right" starts at about 45 degrees
CORNER_ANGLE_MIN_DEG = 45.
SKIP_MARGIN_V = 0.1  # m/s a shallow curb-side turn's cap keeps under turn_skip's safe speed by its pulse (turn_speed_cap, R1)
CORNER_TANGENT_M = 8.0  # m from the corner's start to the junction center, turning to the curb side (the human's 60-100
                        # deg rights went round a median 7.6 m, 11 turns)
FAR_SIDE_TANGENT_M = 12.0  # turning across the junction (left where traffic keeps right) is a wider arc. turn_skip's, not a
                           # fit to the human (whose 60-100 deg lefts went round a median 9.9 m, 6 turns): over it on purpose,
                           # so the far side's corner speed is a ceiling over the model's own 4.3-4.5 (module docstring)


def corner_tangent(curb_side: bool) -> float:
  return CORNER_TANGENT_M if curb_side else FAR_SIDE_TANGENT_M


def corner_radius(angle_deg: float, curb_side: bool) -> float:
  """m: the radius a junction corner of this unsigned angle implies, its tangent over tan(angle / 2), the angle no more than
  CORNER_RADIUS_MAX_DEG (a sharper corner is driven on about the right angle's radius, module docstring)."""
  return corner_tangent(curb_side) / math.tan(math.radians(min(max(angle_deg, 1.), CORNER_RADIUS_MAX_DEG)) / 2)


def one_pulse_speed(angle_deg: float) -> float:
  """m/s a curb-side turn of this angle comes to its one pulse (desire_timing.CURB_TURN_PULSE_M) at: the speed v at which
  v^2 * tan(angle / 2) = A_LAT_TURN_MAX * (CURB_TURN_PULSE_M - v * MODEL_START_S), the angle as corner_radius has it, the
  edge where the model, starting MODEL_START_S after a pulse at 8 m, just makes the corner at A_LAT_TURN_MAX. A bound
  relative to the right angle, not an absolute one (module docstring): the right angle's 2.97 is the evidence's 3.0, and at
  this speed a shallower corner comes out no wider than the right angle at it wherever the pulse truly lands, however soon
  the model starts and at whatever lateral accel up to A_LAT_TURN_MAX. Not the fastest the car can come and still make the
  corner: from the replay's real landing, 6-6.5 m out, the same sum gives the right angle 2.4-2.6."""
  k = math.tan(math.radians(min(max(angle_deg, 1.), CORNER_RADIUS_MAX_DEG)) / 2)
  b = A_LAT_TURN_MAX * MODEL_START_S
  return (-b + math.sqrt(b * b + 4. * k * A_LAT_TURN_MAX * CURB_TURN_PULSE_M)) / (2. * k)


def corner_speed(angle_deg: float, curb_side: bool) -> float:
  """m/s a junction turn of this unsigned angle goes through its corner at (module docstring)."""
  v = math.sqrt(A_LAT_CORNER * corner_radius(angle_deg, curb_side))
  return min(v, one_pulse_speed(angle_deg)) if curb_side else v


V_CORNER = corner_speed(90., curb_side=False)      # m/s, a right-angle far-side turn's: 4.65, a ceiling over the model's own
                                                   # 4.3-4.5 (gate 1), not the evidence's 4.1-4.2 (module docstring)
V_CORNER_CURB = corner_speed(90., curb_side=True)  # m/s, a right-angle curb-side turn's: 2.97, the 3.0 the rights were taken
                                                   # at, all told early (the cap before 2026-11-07; module docstring)
CORNER_CURB_M = CORNER_M  # m: a curb-side turn is at its corner speed by here too (20 before 2026-11-07, module docstring)


def turn_target_speed(angle_deg: float, curb_side: bool) -> float:
  """m/s the braking curve reaches TURN_CAP_LEAD_M out, for a turn of this unsigned angle."""
  return min(max(math.sqrt(A_LAT_MAX * corner_radius(angle_deg, curb_side)), V_TURN_MIN), V_TURN_MAX)


def turn_speed_cap(maneuver, distance: float, v_ego: float, right_hand_traffic: bool = True) -> float:
  """Speed cap for now, from the next maneuver (RouteFollower.upcoming) and the distance to it."""
  town_keep = maneuver is not None and maneuver.kind == KEEP and not maneuver.ramp and \
    (v_ego < KEEP_CAP_SPEED_MAX or (maneuver.geometry and maneuver.angle_deg >= KEEP_CORNER_MIN_DEG))
  if maneuver is None or not (maneuver.kind == TURN or town_keep):
    return NO_SPEED_CAP  # ramps and forks: braking on the main road is wrong, and the model follows the ramp's curve
  curb_side = (maneuver.side == LEFT) != right_hand_traffic
  v_turn = turn_target_speed(maneuver.angle_deg, curb_side)
  lag = max(v_ego, 0.) * LAG_S
  cap = math.sqrt(v_turn ** 2 + 2. * A_DECEL * max(distance - TURN_CAP_LEAD_M - lag, 0.))
  if maneuver.kind == TURN and maneuver.angle_deg >= CORNER_ANGLE_MIN_DEG:
    v_corner = corner_speed(maneuver.angle_deg, curb_side)
    cap = min(cap, math.sqrt(v_corner ** 2 + 2. * A_DECEL * max(distance - CORNER_M - lag / 2., 0.)))
  elif maneuver.kind == TURN and curb_side:
    # a curb-side turn under CORNER_ANGLE_MIN_DEG has no corner curve, so its cap stayed at V_TURN_MIN (6.9 m/s) to the
    # junction, and from 40 deg turn_skip's can_make_turn (its safe speed on the same corner tangent: 6.63 at 40 deg, 6.29 at
    # 44) refused it at the one pulse 8 m out: skipped tooFast on a turn the car was slowing for (review 2026-11-07, R1, the
    # corner speed; Valhalla calls such a junction a slight right, a keep, on every route sampled, so its reach is a 'right'
    # tagged under 45 deg). Now the cap reaches that safe speed by the pulse, never more than it needs (the corner curve's
    # one_pulse_speed would hold a 40 deg right at 3.9 m/s, a corner of about 22 m)
    # SKIP_MARGIN_V under that safe speed, reached SPEED_CAP_TAU before the pulse: inside v * T_LAG of it can_make_turn
    # leaves no room over its safe speed, and a car following a cap down never quite reaches it (plan_accel's approach is
    # exponential: 5 mm/s over at the pulse, refused). Under about 32 deg turn_target_speed is that safe speed itself, so
    # those were on the boundary too
    v_safe = max(skip_safe_speed(maneuver.angle_deg, curb_side) - SKIP_MARGIN_V, 0.)
    reach = CURB_TURN_PULSE_M + max(v_ego, 0.) * SPEED_CAP_TAU + lag / 2.
    cap = min(cap, math.sqrt(v_safe ** 2 + 2. * A_DECEL * max(distance - reach, 0.)))
  return min(cap, NO_SPEED_CAP)


def skip_safe_speed(angle_deg: float, curb_side: bool) -> float:
  """m/s turn_skip's can_make_turn takes a junction turn of this unsigned angle at (its safe_turn_speed: A_LAT_MAX on the
  corner's tangent over tan(angle / 2)), for the cap of a shallow curb-side turn (turn_speed_cap)."""
  half = math.radians(min(max(angle_deg, 1.), 179.)) / 2
  return min(math.sqrt(A_LAT_MAX * corner_tangent(curb_side) / math.tan(half)), V_TURN_MAX)


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
