import math

import numpy as np

# no beamngpy here: these run without the game's client library installed
from openpilot.tools.sim.bridge.beamng import chase_track as ct
from openpilot.tools.sim.bridge.beamng.pose import beamng_quat


def held_log(v=10.0, yaw=0.3, t_end=6.0, lag=0.0):
  """A 20 Hz pose log like route_drive's: a car at v m/s heading yaw, whose IMU value changes on every second or third row."""
  fwd, up = (math.cos(yaw), math.sin(yaw), 0.0), (0.0, 0.0, 1.0)
  q = beamng_quat(fwd, up)
  rows, t_val = [], 0.0
  for k in range(int(t_end * 20)):
    t = k * 0.05
    if k % 5 in (0, 2):  # a new IMU batch on rows 0 and 2 of every 5: held for 2, then 3 rows
      t_val = t - lag
    s = v * t_val
    rows.append([1000.0 + t, t, s * fwd[0], s * fwd[1], 10.0, *q])
  return rows, fwd, q


class TestRotation:
  def test_inverse_of_beamng_quat(self):
    for yaw in (0.0, 0.7, -2.5, math.pi):
      for pitch in (0.0, 0.1):
        fwd = np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), math.sin(pitch)])
        up = np.array([-math.cos(yaw) * math.sin(pitch), -math.sin(yaw) * math.sin(pitch), math.cos(pitch)])
        m = ct.rot_matrix(beamng_quat(fwd, up))
        assert np.allclose(m[:, 1], -fwd, atol=1e-6)  # the vehicle's -y is forward
        assert np.allclose(m[:, 2], up, atol=1e-6)
        assert np.allclose(m.T @ m, np.eye(3), atol=1e-6)

  def test_teleport_pos_is_behind_and_below_the_imu(self):
    q = beamng_quat((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))  # facing east: vehicle x is north (its left), -y east
    p = ct.teleport_pos((10.0, 0.0, 5.0), q)
    ox, oy, oz = ct.IMU_OFFSET
    assert np.allclose(p, (10.0 + oy, -ox, 5.0 - oz))


class TestTrack:
  def test_kept_rows_drop_held_values(self):
    rows, _, _ = held_log(t_end=1.0)
    arr = np.array([[r[0], *r[2:9]] for r in rows])
    kept = ct.kept_rows(arr)
    assert len(kept) == 8  # 20 rows, a new value on 2 of every 5
    assert all(not np.array_equal(a[1:], b[1:]) for a, b in zip(kept, kept[1:], strict=False))

  def test_standing_keeps_both_ends(self):
    rows = [[1000.0 + 0.05 * k, 0.05 * k, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0] for k in range(40)]
    rows += [[1002.0 + 0.05 * k, 2.0 + 0.05 * k, 1.0 + 0.5 * (k + 1), 2.0, 3.0, 0.0, 0.0, 0.0, 1.0] for k in range(10)]
    kept = ct.kept_rows(np.array([[r[0], *r[2:9]] for r in rows]))
    assert kept[0, 0] == 1000.0 and kept[1, 0] == 1000.0 + 0.05 * 39  # standing: first and last row of the hold
    segs, _, _ = ct.ego_track(rows, lag=0.0)
    assert abs(ct.ego_at(segs, 1001.5)[0] - 1.0) < 1e-6  # still standing just before it moves off

  def test_smooth_motion_on_the_game_clock(self):
    lag = 0.16
    rows, fwd, q = held_log(lag=lag)
    segs, t0, t1 = ct.ego_track(rows, lag=lag)
    assert len(segs) == 1
    errs = []
    for t in np.arange(t0 + 0.5, t1 - 0.5, 0.05):
      p = ct.ego_at(segs, float(t))
      s_true = 10.0 * (t - 1000.0)  # where the car really was at game time t
      errs.append(np.linalg.norm(p[:3] - np.array([s_true * fwd[0], s_true * fwd[1], 10.0])))
      assert np.allclose(np.abs(p[3:] @ np.array(q)), 1.0, atol=1e-6)
    assert max(errs) < 0.15  # m at 10 m/s: under 15 ms of the true time, where the held rows are up to 150 ms behind

  def test_teleport_splits_segments(self):
    rows, _, _ = held_log(t_end=2.0)
    for r in rows[20:]:
      r[2] += 500.0
    segs, _, _ = ct.ego_track(rows, lag=0.0)
    assert len(segs) == 2
    assert ct.ego_at(segs, segs[1][0, 0] - 0.01)[0] < 100.0  # just before the jump: still on the old side, not halfway

  def test_actor_at(self):
    rows = [[0.0, 0.0, 0.0, 1.0, 3.0], [1.0, 2.0, 0.0, 1.0, -3.0]]
    x, y, z, h = ct.actor_at(rows, 0.5)
    assert abs(x - 1.0) < 1e-9 and abs(z - 1.0) < 1e-9
    assert abs(abs(h) - math.pi) < 0.2  # the short way round, through pi, not through 0
    assert ct.actor_at(rows, -1.0) is None and ct.actor_at(rows, 2.0) is None


def lit_run():
  """A run with a forced light (the director's light_events) and the oracle's readings of another light, and the stalk."""
  rows = []
  for k in range(100):  # 5 s at 20 Hz from wall 1000
    w = 1000. + 0.05 * k
    lights = [['trafficLight 9', 'green' if w < 1002. else 'yellow', 80. - k]] if k % 5 == 0 else None
    rows.append({'wall': round(w, 3), 'lamp': 'left' if 1001. <= w < 1003. else '', 'blinker': '', 'lights': lights})
  return {'rows': rows, 'light_events': [[1000.2, 'trafficLight 62', 'red'], [1000.2, 'trafficLight 60', 'green'],
                                         [1003.5, 'trafficLight 62', 'green'], [1003.5, 'trafficLight 60', 'red']]}


def test_the_light_timeline_takes_the_forced_events_and_the_oracles_changes():
  tl = ct.light_timeline(lit_run())
  assert tl['trafficLight 62'] == [(1000.2, 'red'), (1003.5, 'green')]
  assert tl['trafficLight 60'] == [(1000.2, 'green'), (1003.5, 'red')]
  # the oracle's: its first reading and each change, at the row's wall less its latency
  assert tl['trafficLight 9'] == [(1000. - ct.ORACLE_LAG_S, 'green'), (round(1002. - ct.ORACLE_LAG_S, 3), 'yellow')]
  assert ct.state_at(tl['trafficLight 62'], 1000.1) is None  # before it was forced: the game's own
  assert ct.state_at(tl['trafficLight 62'], 1002.0) == 'red' and ct.state_at(tl['trafficLight 62'], 1004.0) == 'green'


def test_a_forced_instance_takes_only_its_events():
  run = lit_run()
  run['rows'][0]['lights'] = [['trafficLight 62', 'green', 30.]]  # the oracle read it before the force landed
  assert ct.light_timeline(run)['trafficLight 62'] == [(1000.2, 'red'), (1003.5, 'green')]


def test_the_lamp_timeline_is_carstate_s_blinkers_else_the_stalk():
  rows = lit_run()['rows']
  assert ct.lamp_timeline(rows) == [(1000.0, ''), (1001.0, 'left'), (1003.0, '')]
  old = [{'wall': r['wall'], 'blinker': r['lamp']} for r in rows]  # rows from before carState's lamp was recorded
  assert ct.lamp_timeline(old) == [(1000.0, ''), (1001.0, 'left'), (1003.0, '')]
  assert ct.lamp_at(ct.lamp_timeline(rows), 1002.) == 'left' and ct.lamp_at([], 1002.) == ''


def test_the_replay_sets_each_change_once_and_puts_things_back():
  sets, lamps = [], []
  rp = ct.Replay(ct.light_timeline(lit_run()), ct.lamp_timeline(lit_run()['rows']),
                 set_light=lambda i, s: sets.append((i, s)), set_lamp=lambda side: lamps.append(side))
  for k in range(100):
    rp.apply(1000. + 0.05 * k)
  assert sets.count(('trafficLight 62', 'red')) == 1 and sets.count(('trafficLight 62', 'green')) == 1
  assert ('trafficLight 9', 'yellow') in sets
  assert lamps == ['', 'left', '']
  assert rp.instances() == ['trafficLight 60', 'trafficLight 62', 'trafficLight 9']
  sets.clear()
  rp.close(green=True)
  assert sorted(sets) == [('trafficLight 60', 'green'), ('trafficLight 62', 'green'), ('trafficLight 9', 'green')]
  assert lamps[-1] == ''
