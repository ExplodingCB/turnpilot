"""junction_watch.py (STOP_SIGNS.md 1.3, 4.3; SG3b): the stop sweep asked for at every stop and yield map_controls has engaged,
and step S3's evidence: per held control, one turnpilot.junction_evidence log event with every cross track's first-track
range, lane, view, lighting row and whether it stopped at its line, and turnpilot.stop_visibility each second while the
sweep runs. Nothing here acts."""
import datetime
import math
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.selfdrive.turnpilot import junction_watch as jw
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.control_tracker import TrackedControl
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.tests.test_stop_evidence import track
from openpilot.selfdrive.turnpilot.tests.test_visibility import sectors, vis_file
from openpilot.selfdrive.turnpilot.junction_watch import (JunctionWatch, STOP_SWEEP_M, SWEEP_PAST_M, detector_view, junction_from_info,
                                                          sweep_control)

NOON = datetime.datetime(2026, 10, 28, 16, 0, tzinfo=datetime.UTC).timestamp()  # 12:00 EDT in Indianapolis: the sun south
MPH = sr.MPH


def info(limit_mph=25.):
  v = limit_mph * MPH

  def leg(bearing, rel, role, way):
    return {'bearing': bearing, 'relBearing': rel, 'role': role, 'roadClass': 'residential', 'use': 'road', 'inbound': True,
            'outbound': True, 'lanesIn': 1, 'lanesOut': 1, 'valhallaControl': 'stop', 'tier': 'surveyed', 'assumedLimit': v,
            'wayId': way, 'bikeLane': False, 'shoulderM': -1.}
  return {'status': 'ok', 'osmNodeId': 99, 'lat': 39.8, 'lon': -86.15, 'stopPointDistance': 13., 'stopPointSource': 'default',
          'headingIn': 0., 'maneuver': 'straight', 'turnAngle': 0., 'roundabout': False, 'divided': False, 'railDistance': -1.,
          'sidecar': True, 'suspected': False, 'surveyed': True,
          'approaches': [leg(270., -90., 'left', 11), leg(90., 90., 'right', 12), leg(0., 0., 'opposite', 13)]}


def control(d, kind='stop', engaged=True, held=False, key=(0, 1)):
  return TrackedControl(key, kind, d, engaged=engaged, held=held, junction_info=info() if kind != 'signal' else None)


def test_sweep_control_from_30_m_before_an_engaged_stop_or_yield_until_passed():
  assert sweep_control([control(STOP_SWEEP_M + 1.)]) is None
  assert sweep_control([control(STOP_SWEEP_M - 1.)]).d == STOP_SWEEP_M - 1.
  assert sweep_control([control(5., kind='yield')]) is not None
  assert sweep_control([control(5., engaged=False)]) is None       # map_controls hasn't engaged it
  assert sweep_control([control(5., kind='signal')]) is None       # signals never
  assert sweep_control([control(-SWEEP_PAST_M + 0.1)]) is not None and sweep_control([control(-SWEEP_PAST_M - 0.1)]) is None
  near, far = control(8., key=(0, 2)), control(20., key=(0, 3))
  assert sweep_control([far, near]) is near


def test_the_lefts_own_view_outranks_the_sweep():
  # the unprotected left's road camera from its go to its commit (CRITIQUE REQUIRED 3) and its held roadAndWide: the sweep
  # would suspend the far crop the left needs
  assert detector_view('road', True) == 'road' and detector_view('roadAndWide', True) == 'roadAndWide'
  assert detector_view('auto', True) == 'stopSweep' and detector_view('auto', False) == 'auto'


def test_junction_from_info_reads_the_dict_navd_sent():
  j = junction_from_info('stop', info())
  assert j.osm_node_id == 99 and j.stop_point_distance == 13. and [a.role for a in j.approaches] == ['left', 'right', 'opposite']
  assert junction_from_info('stop', None).status == 'failed'


def run_watch(watch, frames, car_d, dt=0.05, hz=5., wall=NOON, t0=100., stand_from=0):
  """Frames of a held stop: our car standing at the stop point; a car on the left lane at car_d(t) seen by the left side
  view at hz, the four views' runs each 0.2 s."""
  vis = vis_file()
  ln = next(x for x in jw.watch_lanes(junction_from_info('stop', info()), vis.setback or 1.9) if x.role == 'left')
  for k in range(frames):
    t = t0 + k * dt
    runs = []
    if k % 4 == 0:  # the held cycle: a view a step, each at 5 Hz
      for i, v in enumerate(('full', 'sideLeft', 'road', 'sideRight')):
        tv = t + 0.05 * i
        runs.append((v, tv, RawFrame([], 'road' if v == 'road' else 'wide', tv, v if v.startswith('side') else 'full'),
                     sectors(v) if v.startswith('side') else None))
    watch.observe_runs(runs, available=True)
    trk = []
    d = car_d(t - t0)
    if d is not None and k % int(round(1. / (hz * dt))) == 0:
      trk = [SimpleNamespace(**{**vars(track(7, t, d, ln)), 'hits': 5, 'span_seen': (55., 57.)})]
    watch.update(control(0., held=True) if k >= stand_from else control(10., held=False), 0., trk, vis, None, wall)


def test_one_junction_evidence_event_per_held_control_with_each_cross_tracks_first_track():
  watch = JunctionWatch()
  with mock.patch.object(jw.cloudlog, 'event') as event:
    # a car through at 8 m/s from 40 m down the left lane, first matched 4 s into our stand
    run_watch(watch, 200, lambda t: 40. - 8. * (t - 4.) if 4. <= t <= 8.5 else None)
    watch.update(None, 0., [], vis_file(), None, NOON)  # passed: the sweep ends
    watch.update(None, 0., [], vis_file(), None, NOON)
  evs = [c for c in event.call_args_list if c.args[0] == "turnpilot.junction_evidence"]
  assert len(evs) == 1
  e = evs[0].kwargs
  assert e['osmNodeId'] == 99 and e['approachBearingDeg'] == 0. and e['kind'] == 'stop' and e['setbackMeasured']
  assert e['standS'] > 9. and e['lighting'] == 'day'
  assert {(x['role'], x['index']) for x in e['lanes']} == {('left', 1), ('right', 1), ('opposite', 1)}  # every approach's
  (car,) = e['cars']
  ln = next(x for x in jw.watch_lanes(junction_from_info('stop', info()), 1.9) if x.role == 'left')
  psi = math.degrees(math.atan2(40., (ln.s_min + ln.s_max) / 2.))
  assert car['trackId'] == 7 and car['role'] == 'left' and car['index'] == 1 and car['view'] == 'sideLeft' and car['cls'] == 'car'
  # no survey entry given: the map's angle allowance (3.5 deg), and the pessimistic distance end
  assert e['angleBoundDeg'] == sr.angle_bound(False)
  assert car['firstRangeM'] == pytest.approx(ln.s_min * math.tan(math.radians(psi - sr.angle_bound(False))), abs=1e-3)
  assert car['needM'] == pytest.approx(sr.reach_needed_allway(25. * MPH), abs=1e-3)
  assert car['approaching'] is True and car['throughAtSpeed'] is True and car['stoppedAtLine'] is False
  assert car['maxGapInsideS'] == pytest.approx(0.2, abs=0.01) and car['settledS'] == pytest.approx(4., abs=0.2)
  assert car['lighting'] == 'day'


def test_stop_visibility_is_logged_each_second_while_the_sweep_runs():
  watch = JunctionWatch()
  with mock.patch.object(jw.cloudlog, 'event') as event:
    run_watch(watch, 60, lambda t: None)
  vis = [c.kwargs for c in event.call_args_list if c.args[0] == "turnpilot.stop_visibility"]
  assert 2 <= len(vis) <= 4
  lanes = {(x['role'], x['index']): x for x in vis[-1]['lanes']}
  assert set(lanes) >= {('left', 1), ('right', 1)}
  assert lanes[('left', 1)]['reachM'] > 0. and lanes[('left', 1)]['why'] in ('range', 'frameEdge')
  assert vis[-1]['osmNodeId'] == 99


def test_no_clock_no_lighting_row():
  watch = JunctionWatch()
  with mock.patch.object(jw.cloudlog, 'event') as event:
    run_watch(watch, 40, lambda t: None, wall=None)
    watch.update(None, 0., [], vis_file(), None, None)
  (e,) = [c.kwargs for c in event.call_args_list if c.args[0] == "turnpilot.junction_evidence"]
  assert e['lighting'] == 'unknown'
