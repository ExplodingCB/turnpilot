"""The corner speed from the corner (turn_speed.py module docstring, 2026-11-07; Kellen: "not all turns should be taken at
7mph"): a junction turn's corner speed is sqrt(A_LAT_CORNER * corner_radius) for its angle and side, and a curb-side one's
no more than one_pulse_speed, the speed its one 8 m pulse leaves the model room to turn from. Before, it was 3.0 m/s by 20 m
to the curb side and 4.3 by 15 m across, whatever the angle. The evidence rows below are the turns of the drives on disk
(0000003, 1d, 1e: route, clock m:ss, angle turned, radius and speed through the turn's core), no positions."""
import math
import statistics

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import CURB_TURN_PULSE_M, turn_lead
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
from openpilot.selfdrive.turnpilot.turn_skip import can_make_turn
from openpilot.selfdrive.turnpilot.turn_speed import (A_LAT_CORNER, A_LAT_MAX, A_LAT_TURN_MAX, CORNER_CURB_M, CORNER_M, MODEL_START_S,
                                                      POST_GO_PROFILES, V_CORNER, V_CORNER_CURB, PostGo, corner_radius,
                                                      corner_speed, one_pulse_speed, turn_speed_cap)

# the human's rolling turns (by hand, not from a stand nor stopping in it): (route, clock, side, deg, core radius m, core speed m/s)
HUMAN_ROLLING = [
  ('0000003', '3:33', 'curb', 79, 9.5, 4.0), ('0000003', '8:11', 'far', 121, 13.7, 5.4), ('0000003', '8:43', 'curb', 78, 10.1, 4.5),
  ('0000003', '10:18', 'curb', 174, 8.1, 3.8), ('1d', '2:49', 'far', 176, 8.8, 3.8), ('1d', '4:27', 'far', 91, 19.7, 6.2),
  ('1d', '11:26', 'curb', 82, 12.8, 4.8), ('1d', '11:33', 'far', 89, 7.6, 3.7), ('1e', '2:20', 'far', 88, 8.7, 3.7),
  ('1e', '11:53', 'curb', 82, 9.2, 4.6), ('1e', '13:11', 'far', 85, 11.0, 4.9), ('1e', '13:56', 'curb', 123, 7.4, 3.5),
]
# the core lateral accels those turns were driven at (v^2 / R per frame through the core, its median: not the row's v^2 / R)
HUMAN_ROLLING_ALAT = [1.75, 2.11, 2.17, 1.66, 1.59, 1.86, 1.81, 1.83, 1.55, 2.40, 2.25, 1.65]
HUMAN_HARDEST_ALAT = 2.52  # 1d 10:15, a left off a 35 mph road, 100 deg on 11.1 m at 5.3 m/s: the hardest junction corner by hand
OLD_CURB, OLD_FAR = 3.0, 4.3  # m/s, the one corner speed a side before 2026-11-07


def turn(modifier, angle):
  return Maneuver(*classify("turn", modifier), "turn", modifier, angle, geometry=True)


def approach(m, until_m, v=13., d=300., right_hand_traffic=True):
  """The car under the turn's cap (the planner's 1 s lag, hooks.plan_accel) from d m out at v: ({m out: v}, min accel), the
  first frame at or inside each of until_m."""
  out, min_a = {}, 0.
  marks = sorted(until_m, reverse=True)
  while d > marks[-1]:
    a = min(plan_accel(10., turn_speed_cap(m, d, v, right_hand_traffic), v), 0.)
    min_a = min(min_a, a)
    v = max(v + a * DT_MDL, 0.)
    d -= v * DT_MDL
    for mark in marks:
      if mark not in out and d <= mark:
        out[mark] = v
  return out, min_a


def test_the_comfort_and_the_hardest_corner_are_the_humans():
  """A_LAT_CORNER is the median of the human's rolling turns' core lateral accel (1.82 m/s^2: half went harder), and
  A_LAT_TURN_MAX the hardest junction corner the human drove (2.52): the numbers come from the turns, not a guess."""
  assert len(HUMAN_ROLLING) == len(HUMAN_ROLLING_ALAT) == 12
  assert math.isclose(A_LAT_CORNER, round(statistics.median(HUMAN_ROLLING_ALAT), 1))
  assert math.isclose(A_LAT_TURN_MAX, round(HUMAN_HARDEST_ALAT, 1)) and max(HUMAN_ROLLING_ALAT) < A_LAT_TURN_MAX
  # turn_skip's own lateral limit (A_LAT_SAFE, A_LAT_MAX) stays over the cap's: a car that follows the cap is never too fast
  assert A_LAT_CORNER < A_LAT_MAX


def test_the_corner_speed_comes_from_the_corner():
  """The table of the module docstring. To the curb side: 2.97 m/s at 90 deg and sharper (the old 3.0, which the one pulse's
  sum lands on), 3.3 at 70, 3.5 at 60, 3.6 at 52, 3.8 at 45; across: 4.65 at 90 (4.3 before), 5.55 at 70, 6.1 at 60, 7.2 at 45. The
  old one-number-a-side caps fail every shallower row (3.0 and 4.3 at every angle)."""
  table = {(45., True): 3.77, (52., True): 3.62, (60., True): 3.47, (70., True): 3.30, (90., True): 2.97, (135., True): 2.97,
           (45., False): 7.22, (52., False): 6.65, (60., False): 6.12, (70., False): 5.55, (90., False): 4.65, (135., False): 4.65}
  for (angle, curb), v in table.items():
    assert abs(corner_speed(angle, curb) - v) < 0.01, (angle, curb, corner_speed(angle, curb))
    m = turn("right" if curb else "left", angle)
    assert abs(turn_speed_cap(m, 0., v) - v) < 0.01, (angle, curb)  # the cap at the junction is the corner speed
  assert abs(V_CORNER_CURB - OLD_CURB) < 0.05  # the tight right stays where the evidence has it
  assert V_CORNER > OLD_FAR
  for curb in (True, False):
    speeds = [corner_speed(a, curb) for a in range(45, 181, 5)]
    assert all(x >= y - 1e-9 for x, y in zip(speeds, speeds[1:], strict=False)), curb  # never faster for a sharper corner
    assert corner_speed(52., curb) > (OLD_CURB if curb else OLD_FAR) + 0.5  # 1e 6:41's 52 deg right was held at 3.0
  # left-hand traffic: the curb side is the left
  assert math.isclose(turn_speed_cap(turn("left", 60.), 0., 3., right_hand_traffic=False), corner_speed(60., True))
  assert math.isclose(turn_speed_cap(turn("right", 60.), 0., 3., right_hand_traffic=False), corner_speed(60., False))


def test_a_sharp_corner_is_driven_on_the_right_angles_radius():
  """The human's sharp turns (100-176 deg) went round 5.6-13.7 m, about a right angle's, not the 3.3 m a 135 deg corner's
  tangent over tan(angle / 2) gives: corner_radius takes the angle as 90 at most, so a sharp turn's corner speed is the right
  angle's (as V_CORNER and V_CORNER_CURB were for every turn of 45 deg or more before)."""
  for curb in (True, False):
    assert math.isclose(corner_radius(135., curb), corner_radius(90., curb))
    assert math.isclose(corner_radius(179., curb), corner_radius(90., curb))
    assert corner_radius(60., curb) > corner_radius(90., curb)
  # each of the human's sharp rolling turns went round nearer the right angle's radius than the tangent's own at its angle
  sharp = [(side == 'curb', deg, radius) for _, _, side, deg, radius, _ in HUMAN_ROLLING if deg > 100]
  assert len(sharp) == 4
  for curb, deg, radius in sharp:
    by_tangent = (8. if curb else 12.) / math.tan(math.radians(deg) / 2)
    assert abs(radius - corner_radius(deg, curb)) < abs(radius - by_tangent), (curb, deg, radius)


@pytest.mark.parametrize("angle", [45., 52., 60., 70., 90., 120., 160.])
def test_a_curb_side_turns_speed_is_its_one_pulse_edge(angle):
  """one_pulse_speed: a curb-side turn's corner speed is the edge where the model, told at CURB_TURN_PULSE_M and starting
  round MODEL_START_S later, just makes the corner at A_LAT_TURN_MAX, and it is that edge that holds a curb-side turn of 45 deg
  or more (the human's comfort on the corner's own radius, sqrt(A_LAT_CORNER * R), would have it 0.8-2.1 m/s faster). The
  edge is relative to the right angle, not a bound of its own (the two tests below)."""
  v = corner_speed(angle, True)
  start_m = CURB_TURN_PULSE_M - v * MODEL_START_S
  k = math.tan(math.radians(min(angle, 90.)) / 2)
  assert start_m > 0.
  assert math.isclose(v ** 2 * k / start_m, A_LAT_TURN_MAX), angle  # just makes it, from the latest start at the nominal 8 m
  assert math.isclose(v, one_pulse_speed(angle))
  assert math.sqrt(A_LAT_CORNER * corner_radius(angle, True)) > v + 0.8, angle
  # a far-side turn is told 6 s out (desire_timing.turn_lead), so no such edge: its comfort holds it
  assert math.isclose(corner_speed(angle, False), math.sqrt(A_LAT_CORNER * corner_radius(angle, False)))


FULL_LOCK_RADIUS_M = 4.3  # m, about a Model 3's path at full lock (an 11.7 m turning circle, curb to curb, on 2.875 m of wheelbase)
RIGHT_LANE_M = 1.8        # m, a right lane's centre off the road's centre line: the path's corner this x tan(angle / 2) short of navd's node


def exit_wide(angle, v, pulse_m, start_s, a_lat, r_min=0., lane_m=0.):
  """m a curb-side turn comes out of its corner wide of its exit lane: told pulse_m before navd's node at v, the model
  starting round start_s later on the tightest arc it takes (v^2 / a_lat, no tighter than r_min), the path's corner lane_m x
  tan(angle / 2) short of the node. An arc started short of the room it needs settles on an exit line parallel to the lane's,
  its missing tangent x sin(angle) outside it."""
  a = math.radians(min(angle, 90.))
  k = math.tan(a / 2)
  start = pulse_m - lane_m * k - v * start_s
  return max(max(v * v / a_lat, r_min) * k - start, 0.) * math.sin(a)


def edge_speed(angle, pulse_m):
  """m/s one_pulse_speed's sum gives for a pulse truly pulse_m out (one_pulse_speed has it at CURB_TURN_PULSE_M)."""
  k = math.tan(math.radians(min(angle, 90.)) / 2)
  b = A_LAT_TURN_MAX * MODEL_START_S
  return (-b + math.sqrt(b * b + 4. * k * A_LAT_TURN_MAX * pulse_m)) / (2. * k)


# where the pulse truly lands (3-9 m out: desire_timing's replay of 1d/1e has it at the furthest reading's 8 m, that reading
# 1.5-2 m long, or the latest's 6 m, that reading 2.5 m short to 1.5 m long), how soon the model starts (desire_timing: 0.3-1.5
# s), how hard it turns (the human's gentlest rolling corner to the hardest junction one), on the car's full lock or not, the
# right lane's corner or the centre line's
LANDINGS = [(p / 4., s / 10., a / 10., r_min, lane_m) for p in range(12, 37) for s in range(3, 16) for a in range(12, 26)
            for r_min in (0., FULL_LOCK_RADIUS_M) for lane_m in (0., RIGHT_LANE_M)]


def test_a_shallower_right_comes_out_no_wider_than_the_right_angle():
  """2026-11-07 review: one_pulse_speed isn't the fastest the car can come and still make the corner (below), so the shallower
  rights' 3.3-3.8 m/s rest on this instead: wherever the pulse truly lands, however soon the model starts and at whatever
  lateral accel up to A_LAT_TURN_MAX, a right of 45-90 deg at its corner speed comes out of its corner no wider than the
  right angle at V_CORNER_CURB, the 3.0 the evidence's rights were taken at, does under the same pulse. That is all the
  shallower speeds claim, and they are the most that claims: 0.02 m/s more and some landing has them wider, and a model
  turning harder than A_LAT_TURN_MAX could have them wider too."""
  angles = (45., 50., 52., 55., 60., 65., 70., 75., 80., 85., 90.)

  def worst(angle, v, landings):
    return max(exit_wide(angle, v, *x) - exit_wide(90., V_CORNER_CURB, *x) for x in landings)

  for angle in angles:
    assert worst(angle, corner_speed(angle, True), LANDINGS) < 1e-9, angle
  for angle in (45., 60., 70.):
    assert worst(angle, corner_speed(angle, True) + 0.02, LANDINGS) > 0.03, angle  # the most: no slack left
    assert worst(angle, OLD_CURB, LANDINGS) < 1e-9  # the old 3.0 everywhere claimed it too, more slowly
    assert worst(angle, math.sqrt(A_LAT_CORNER * corner_radius(angle, True)), LANDINGS) > 3.  # the human's comfort: far wider
  harder = [(p / 4., s / 10., 3.0, 0., 0.) for p in range(12, 37) for s in range(3, 16)]
  assert worst(45., corner_speed(45., True), harder) > 0.05  # tied to A_LAT_TURN_MAX: at 3.0 m/s^2 the shallow right is wider


def test_the_one_pulse_edge_is_no_bound_of_its_own():
  """2026-11-07 review: the edge is at the nominal 8 m pulse, which no logged right was pulsed at. Where desire_timing's replay
  of 1d/1e has the pulse land, 6-6.5 m from the corner (the furthest reading's 8 m, it 1.5-2 m long), the same sum gives the
  right angle 2.4-2.6 m/s and every shallower row the old 3.0 or under at 6 m (within 0.25 of it at 6.5), and the right
  angle at V_CORNER_CURB from the latest start comes out 1.5-2 m wide (2.3-2.8 on the car's full lock); even from 8 m that
  start asks a radius under full lock. So
  the right angle's speed is the evidence's (the module docstring), not the sum's, and the sum only carries it to the other
  angles (above)."""
  assert 2.4 < edge_speed(90., 6.) < edge_speed(90., 6.5) < 2.6
  assert all(edge_speed(a, 6.5) < OLD_CURB + 0.25 for a in (45., 52., 60., 70.))
  assert all(edge_speed(a, 6.) < OLD_CURB + 0.01 for a in (45., 52., 60., 70.))
  for pulse_m, wide, wide_locked in ((6., 2.0, 2.76), (6.5, 1.5, 2.26)):
    assert abs(exit_wide(90., V_CORNER_CURB, pulse_m, MODEL_START_S, A_LAT_TURN_MAX) - wide) < 0.02, pulse_m
    assert abs(exit_wide(90., V_CORNER_CURB, pulse_m, MODEL_START_S, A_LAT_TURN_MAX, FULL_LOCK_RADIUS_M) - wide_locked) < 0.02
  assert exit_wide(90., V_CORNER_CURB, CURB_TURN_PULSE_M, MODEL_START_S, A_LAT_TURN_MAX) < 1e-6  # the edge itself...
  assert CURB_TURN_PULSE_M - V_CORNER_CURB * MODEL_START_S < FULL_LOCK_RADIUS_M  # ...asks a radius the car can't turn on


MODEL_LEFT_TOP = 4.5       # m/s: gate 1's model, no corner cap, slowed itself to 4.3-4.5 for its right-angle lefts (BeamNG)
HUMAN_LEFT_RADIUS_M = 9.9  # m: the median of the human's 6 hand-driven 60-100 deg lefts (7.6-19.7)
HUMAN_LEFT_SPEED = 4.1     # m/s: their median core speed (3.7-6.2)


def test_the_far_sides_corner_is_a_ceiling_over_the_models_own_speed():
  """2026-11-07 review: V_CORNER, 4.65 m/s, isn't the evidence's 4.1-4.2 (the human's lefts on their own radius at
  A_LAT_CORNER, and their median speed). It is a ceiling set over the model's own speed on purpose: a left is told 6 s out
  and the model slows itself to 4.3-4.5, and a cap under that, the human fit's 4.2 or the old 4.3, holds every right-angle
  left under the model's own plan ("worried about it overslowing before turns", 2026-11-07). On the human's own radius it
  asks no more lateral accel than the human's p90 and under the hardest corner the human drove."""
  human_fit = math.sqrt(A_LAT_CORNER * HUMAN_LEFT_RADIUS_M)
  assert abs(human_fit - 4.22) < 0.01 and human_fit < MODEL_LEFT_TOP and HUMAN_LEFT_SPEED < MODEL_LEFT_TOP
  assert V_CORNER > MODEL_LEFT_TOP and OLD_FAR < MODEL_LEFT_TOP  # the old 4.3 held the model under its own 4.5
  p90 = statistics.quantiles(HUMAN_ROLLING_ALAT, n=10, method='inclusive')[-1]
  assert abs(p90 - 2.24) < 0.01
  assert V_CORNER ** 2 / HUMAN_LEFT_RADIUS_M < p90 < A_LAT_TURN_MAX
  # and turn_skip, on the same 12 m tangent at its own A_LAT_MAX, still has the car that follows the cap able to make it
  assert can_make_turn(V_CORNER, CORNER_M, 90., False) and can_make_turn(V_CORNER, 0., 90., False)


@pytest.mark.parametrize("angle", [45., 60., 90., 135.])
def test_a_curb_side_turn_is_at_its_corner_speed_by_15_m_not_20(angle):
  """1e's rights under the 3.0 cap by 20 m (set for the 40 m pulse): the cap at 3.0 from 22-23 m out, the car at 3.5 m/s or
  under from 20-21 m, 6.7-8 s to the junction, where the human was at 5.9 m/s 20 m out and 4.7 at 8 m. With the one pulse at
  8 m the car is at its corner speed by CORNER_M, as a far-side turn is, still at 3.5 m/s or less from 15 m out at a right
  angle (BeamNG's rights taken), at it by the pulse, and over the old crawl 20 m out, on the same gentle braking from town
  speed."""
  assert CORNER_CURB_M == CORNER_M == 15.
  m = turn("right", angle)
  v_c = corner_speed(angle, True)
  for v0 in (8., 13., 18.):
    at, min_a = approach(m, (20., 15., CURB_TURN_PULSE_M, 0.), v=v0)
    assert at[20.] > 4.2, (angle, v0, at)       # the 3.0 by 20 m had it at 3.4 there
    assert at[CURB_TURN_PULSE_M] < v_c + 0.1, (angle, v0, at)
    assert at[0.] < v_c + 0.05 and min_a > -1.6, (angle, v0, at, min_a)
    if angle >= 90.:
      assert at[15.] <= 3.5, (angle, v0, at)
  if angle < 90.:
    assert at[CURB_TURN_PULSE_M] > OLD_CURB + 0.3, at  # a wider right goes faster than the old 3.0 through its corner


def test_a_far_side_turn_comes_to_its_corner_speed_by_corner_m():
  for angle in (60., 90.):
    at, min_a = approach(turn("left", angle), (CORNER_M, 0.))
    v_c = corner_speed(angle, False)
    assert v_c - 0.05 < at[0.] < v_c + 0.05 and at[CORNER_M] < v_c + 0.5 and min_a > -1.6, (angle, at, min_a)
  assert approach(turn("left", 60.), (0.,))[0][0.] > OLD_FAR + 1.5  # the old 4.3 at every angle


@pytest.mark.parametrize("curb", [True, False])
@pytest.mark.parametrize("angle", [20., 25., 30., 35., 40., 44., 45., 60., 70., 90., 135.])
@pytest.mark.parametrize("v0", [8., 13., 18., 25.])
def test_a_car_that_follows_the_cap_is_never_too_fast_for_turn_skip(curb, angle, v0):
  """turn_skip's rule: a car that follows the turn speed cap never skips (can_make_turn at its 2 m/s^2 and 1 s lag, on its own
  corner radius and A_LAT_SAFE), so the faster corners keep under its curve on every frame until the turn's first pulse, that
  frame included. Review 2026-11-07 (R1, the corner speed): checked only from 45 deg and never on the pulse frame, where a
  40-44 deg curb-side right on the V_TURN_MIN floor (6.9 m/s) was refused (skipped tooFast 6.1-7.1 m out), and under about
  32 deg the cap equalled the safe speed with no margin."""
  m = turn("right" if curb else "left", angle)
  d, v = 300., v0
  while d > 0.:
    pulse_at = CURB_TURN_PULSE_M if curb else turn_lead(m, v)
    assert can_make_turn(v, d, angle, curb), (d, v)
    if d <= pulse_at:
      break  # the pulse frame checked: from here the turn is under way
    v = max(v + min(plan_accel(10., turn_speed_cap(m, d, v), v), 0.) * DT_MDL, 0.)
    d -= v * DT_MDL


def test_a_curb_side_corner_is_under_every_rolling_human_right():
  """The human's rolling rights went through their corners at 3.5-4.8 m/s: the curb side's corner speed at their angles is
  under every one of them (the one pulse holds it, not the human's comfort)."""
  rights = [(deg, v) for _, _, side, deg, _, v in HUMAN_ROLLING if side == 'curb']
  assert len(rights) == 6 and all(corner_speed(deg, True) < v for deg, v in rights)


def test_a_left_after_its_go_blends_into_its_own_corner_speed():
  """The unprotected left's post-go profile (4.3 m/s to 45 deg) blends from 45 to 60 deg into the turn's corner cap where the
  car is: now its corner speed (V_CORNER at 90 deg, 4.65), where before it was 4.3 flat on through 60 deg. The rise is on
  purpose (the far side's ceiling, the test above): the car isn't held under the model's own speed as it clears the oncoming
  lanes."""
  left = turn("left", 90.)
  pg = PostGo(POST_GO_PROFILES['left'], left, 10., 52.5)
  _, cap = pg.caps(4.)
  assert math.isclose(cap, 4.3 + (turn_speed_cap(left, 10., 4.) - 4.3) / 2., abs_tol=1e-3) and cap > 4.3 + 0.1
