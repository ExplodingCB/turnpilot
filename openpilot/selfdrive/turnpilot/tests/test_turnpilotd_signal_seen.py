"""A junction mapped as a stop sign that is really a signal (route 29, 2026-11-08, 126-164 s): the camera saw its heads 29 m
out (turnpilot.stop_go_signal_head), navd had no OSM node for it so the override write failed, the vision gate failed on
skew (a stop sign's rule) and the car was handed back, with "Stop sign ahead needs you" said 300 m out. Now the junction is
remembered for the drive by its node or its position (stop_go.SignalJunctions), the stop sign's rules are no handback there,
the notice isn't said for it. The review of 2026-11-08: its control is never made a signal (it was, from the frame the
camera read any colour on a head 10 m short of its line to 40 m past it, so the next junction's green drove the car through
the stop sign): the full stop is always made there, a red or yellow read on this junction's heads holds the go, and a head is
this junction's only from its stop point to a few metres past its far side (stop_go._head_here)."""
import copy
import math

import pytest

from openpilot.selfdrive.turnpilot.detector.light_state import ASSUMED_HEAD_M
from openpilot.selfdrive.turnpilot.map_controls import CameraLights
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import LAT, LON
from openpilot.selfdrive.turnpilot.stop_go import RED_HOLD_REASON, SIGNAL_SAME_M, SignalJunctions
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import S_LINE, run_to_line
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_vision import SHIPPED, vinfo
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import StopSim

HEADS = ((2., 12., 0.7),)  # a head over the junction, above the horizon, on every forward run
M_PER_DEG = 111_320.


def test_signal_junctions_key_by_node_or_position():
  seen = SignalJunctions()
  assert not seen.add({'osmNodeId': 0, 'lat': 0., 'lon': 0.})  # nothing to key it by
  assert seen.add({'osmNodeId': 0, 'lat': LAT, 'lon': LON})
  assert not seen.add({'osmNodeId': 0, 'lat': LAT, 'lon': LON})  # known
  near = (SIGNAL_SAME_M - 5.) / M_PER_DEG
  far = (SIGNAL_SAME_M + 5.) / M_PER_DEG
  assert seen.matches({'osmNodeId': 0, 'lat': LAT + near, 'lon': LON})  # a reroute's lookup of the same junction
  assert not seen.matches({'osmNodeId': 0, 'lat': LAT + far, 'lon': LON})  # the next junction
  assert not seen.matches(None)
  seen.add({'osmNodeId': 77, 'lat': 0., 'lon': 0.})
  assert seen.matches({'osmNodeId': 77, 'lat': LAT + 1., 'lon': LON})


def skewed(**kw):
  """Route 29's junction as navd had it: no OSM node, the opposite approach 21 deg off straight on (the gate's skew)."""
  info = copy.deepcopy(vinfo(**kw))
  info['osmNodeId'] = 0
  info['approaches'][2]['bearing'], info['approaches'][2]['relBearing'] = 339., -21.
  return info


def ssim(tmp_path, heads=(), info=None):
  sim = StopSim(tmp_path, mode='vision', vis=SHIPPED, entries=[], model='goes')
  sim.navd.controls[0] = (S_LINE, 'stop', False, True, False, info or skewed())
  sim.heads = heads if callable(heads) else (lambda s: heads)
  return sim


def took_control(plans):
  return [p.reason for p in plans if p.controlPrompt == 'stopSignTakeControl']


def test_the_skewed_stop_sign_hands_back_at_1_s_as_before(tmp_path):
  """No head seen: a stop sign whose opposite leg is 21 deg off is H9 under vision, as route 29's was."""
  sim = ssim(tmp_path)
  run_to_line(sim)
  sim.run(3.)
  assert took_control(sim.plans()) and '21 deg off straight on' in took_control(sim.plans())[-1]


def test_heads_seen_on_the_approach_end_the_handback_and_the_junction_is_remembered_with_no_node(tmp_path):
  """Route 29: the heads seen on the approach, no OSM node (the override not written, said once). The stop sign's rules are
  no handback at a signal: the car stops at the line (no colour was read: the full stop is the safe side) and goes on what it
  sees, no lift (the model's own go and red-light stop). The junction is remembered by its position for the drive."""
  sim = ssim(tmp_path, heads=HEADS)
  sim.tp.params = None
  run_to_line(sim)
  sim.run(14.)
  plans = sim.plans()
  assert not took_control(plans), took_control(plans)[:1]
  assert sim.tp.signal_junctions.matches({'osmNodeId': 0, 'lat': LAT, 'lon': LON})
  assert any(str(p.stopGo.state) in ('going', 'committed', 'inLane', 'done') for p in plans)
  assert sum(p.controlAlert == 'overrideWriteFailed' for p in plans[:1]) <= 1


def test_a_junction_remembered_this_drive_is_no_handback_from_the_start(tmp_path):
  """A later pass (or a reroute's new control) at the junction, no head in view this time: the encounter starts with the
  signal known (Frame.signal_junction), so the gate never hands it back."""
  sim = ssim(tmp_path)
  sim.tp.signal_junctions.add({'osmNodeId': 0, 'lat': LAT, 'lon': LON})
  run_to_line(sim)
  sim.run(14.)
  assert not took_control(sim.plans())


def camera(sim, state):
  """The camera's reader acting (addOnly), its governing head 25 m past the stop point read state on every road run (the
  reader's own message is test_camera_lights'; StopSim's detections carry the side views' status, so the read is handed in
  where turnpilotd takes it, camera_lights)."""
  sim.tp.camera_mode = 'addOnly'

  def lights(sm, data):
    sim.tp.camera_read = CameraLights([(state, S_LINE + 25. - sim.s)], False, 'read')
    return sim.tp.camera_read
  sim.tp.camera_lights = lights


@pytest.mark.parametrize('state', ['green', 'red'])
def test_a_colour_read_at_a_remembered_junction_keeps_the_full_stop_and_a_red_holds_the_go(tmp_path, state):
  """Remembered (heads seen this drive), the camera reads a colour on its head. Until the review of 2026-11-08 the control
  became a signal: a green went through with no stop at the line. Now it stays a stop sign: the car makes its full stop at
  the line whatever is read; on a green it goes after on what the cameras see (no handback for the stop sign's rules), on a red
  it stands at its line, the go held for the light, never a handback."""
  sim = ssim(tmp_path)
  sim.tp.signal_junctions.add({'osmNodeId': 0, 'lat': LAT, 'lon': LON})
  camera(sim, state)
  run_to_line(sim)
  stopped_at = sim.s
  sim.run(14.)
  plans = sim.plans()
  assert not any(str(p.mapControl.kind) == 'signal' for p in plans)
  assert abs(S_LINE - stopped_at) < 6.  # the full stop at the stop sign's line
  assert not took_control(plans)
  if state == 'green':
    assert any(str(p.stopGo.state) in ('going', 'committed', 'inLane', 'done') for p in plans)
  else:
    assert sim.s < S_LINE + 1. and sim.v < 0.1 and plans[-1].shouldStop
    assert not any(str(p.stopGo.state) in ('going', 'committed', 'inLane', 'done') for p in plans)
    assert any(RED_HOLD_REASON in p.reason for p in plans[-100:]), {p.reason for p in plans[-100:]}


@pytest.mark.parametrize('head_d', [35., 55., 69., 75.])
def test_no_head_ever_makes_a_stop_control_a_signal(tmp_path, head_d):
  """probe_stop_signal (the review of 2026-11-08): a green head 5-45 m past the stop line, the junction remembered as a
  signal: until then its control became a signal on any colour read there (signal_controls) and the car drove the stop on the
  next junction's green. Now nothing makes it one: it stays a stop control and the car stops at its line."""
  sim = ssim(tmp_path)
  sim.tp.signal_junctions.add({'osmNodeId': 0, 'lat': LAT, 'lon': LON})
  sim.tp.camera_mode = 'addOnly'

  def lights(sm, data):
    sim.tp.camera_read = CameraLights([('green', S_LINE + head_d - 30. - sim.s)], False, 'read')
    return sim.tp.camera_read
  sim.tp.camera_lights = lights
  run_to_line(sim)
  assert not any(str(p.mapControl.kind) == 'signal' for p in sim.plans())
  assert sim.v < 0.05 and abs(S_LINE - sim.s) < 6.


def head_at(past_m, bearing=1.):
  """A signal head past_m m past the stop point, ASSUMED_HEAD_M over the road, as a forward run sees it from the car (its
  elevation from the camera, its setback behind the bumper and its height over the road as stop_go takes them)."""
  def heads(sim):
    x = S_LINE + past_m - sim.s + (SHIPPED.setback if SHIPPED.setback is not None else 1.9)  # stop_go's without one
    if x <= 1.:
      return ()
    height = SHIPPED.height if SHIPPED.height is not None else 1.3  # stop_go's camera height without one in the file
    return ((bearing, math.degrees(math.atan2(ASSUMED_HEAD_M - height, x)), 0.7),)
  return heads


def test_a_head_at_the_next_junction_is_no_signal_here_and_one_over_this_junction_is(tmp_path):
  """The review of 2026-11-08: _head_seen took any class-9 head above the horizon within SIGNAL_SPAN_M of the path, however
  far up the road: the next junction's heads made this stop sign a signal (its stop-sign rules no handback, its notice
  dropped, and with signal_controls its stop driven on their green). Now a head is this junction's only from its stop point
  to SIGNAL_PAST_M past its far side, ranged from its elevation: one 40 m past the line (the next junction's) is no signal
  head here, so route 29's skewed stop sign hands back as a stop sign; one over the far lanes is, as on route 29."""
  far = ssim(tmp_path, heads=head_at(40.))
  far.tp.params = None
  run_to_line(far)
  far.run(3.)
  assert not far.tp.signal_junctions.seen
  assert took_control(far.plans()) and '21 deg off straight on' in took_control(far.plans())[-1]
  here = ssim(tmp_path, heads=head_at(14.))
  here.tp.params = None
  run_to_line(here)
  here.run(14.)
  assert here.tp.signal_junctions.matches({'osmNodeId': 0, 'lat': LAT, 'lon': LON})
  assert not took_control(here.plans())


def test_a_colour_read_at_a_junction_never_seen_changes_nothing(tmp_path):
  """A camera colour near a stop sign whose junction no head was seen at this drive: still a stop sign (a signal of the next
  junction read through it is not this junction's)."""
  sim = ssim(tmp_path, info=vinfo())
  camera(sim, 'green')
  run_to_line(sim)
  assert not any(str(p.mapControl.kind) == 'signal' for p in sim.plans())
  assert sim.v < 0.05 and abs(S_LINE - sim.s) < 6.  # stopped at the stop sign's line
