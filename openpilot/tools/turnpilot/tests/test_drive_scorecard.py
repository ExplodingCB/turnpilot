"""drive_scorecard's lane change request and 'check the light' scoring, on synthetic per-frame records."""
from openpilot.tools.turnpilot import drive_scorecard as sc

V = 10.


def rec(t, **kw):
  r = {'t': t, 'odo': V * t, 'sim_en': True, 'v': V, 'a': 0., 'en': True, 'gas': False, 'brake': False, 'steer': False, 'lb': False,
       'rb': False, 'lbs': False, 'rbs': False, 'lead': None, 'yaw': 0., 'cam': None, 'camst': None, 'nav': None, 'state': 'approaching',
       'desire': 0, 'lc': 'none', 'offer': 'none', 'abort': 'none', 'want': 0, 'cap': 30., 'acap': 0., 'stop': False, 'prompt': 'none',
       'alert': 'none', 'reason': '', 'md': 0., 'mtype': 'turn', 'mmod': 'left', 'fast': False, 'skip': '', 'fault': 'none',
       'handback': 'none', 'sigp': 'none', 'mck': 'none', 'mca': 'none', 'mcd': 0., 'mcl': 'unknown', 'mcs': 'none', 'uls': 'none',
       'ulwhy': '', 'ulclear': 0., 'ulview': False, 'ulchime': False, 'sgs': 'none', 'sgr': '', 'sgchime': False, 'names': [], 'ev': [],
       'lcp': 'none', 'sim_sig': 'none'}
  r.update(kw)
  return r


def left_turn(frames, kind='none'):
  """A left the car reaches (and turns at) on the last frames, the planner's frames as given: [(seconds, record fields)]."""
  recs, t = [], 0.
  for seconds, kw in frames:
    for _ in range(round(seconds / sc.DT)):
      recs.append(rec(t, **kw))
      t += sc.DT
  n = len(recs)
  for r in recs[-40:]:
    r['yaw'] = 1.
  track = {'d': {i: (n - 60 - i) * V * sc.DT for i in range(n)}}
  row = {'type': 'turn', 'side': 'left', 'label': 'turn left', 'kind': kind, 'track': track, 'controls': [], 'start': 0, 'pass': n - 60,
         'end': n - 1}
  return sc.score_row(recs, row, ())


REQUEST = {'offer': 'left', 'lc': 'left', 'want': 1}
SKIP = {'fast': True, 'skip': 'laneBlocked'}


def test_a_request_held_a_changes_length_is_the_rows_divergence():
  held = left_turn([(2., {}), (sc.LC_COMPLETE_S + 1., REQUEST), (4., SKIP)])
  assert (held['verdict'], held['diverged_why'], held['skip']) == ('would-drive', 'lane change requested', '')
  assert held['lane_request']['held_s'] == sc.LC_COMPLETE_S and held['lane_request']['desire_helper_would_start']
  assert held['lane_request']['ended'] == 'held'


def test_a_request_dropped_sooner_changes_no_verdict():
  short = left_turn([(2., {}), (3., REQUEST), (0.5, {'abort': 'lineNotBroken'}), (4., SKIP)])
  assert (short['verdict'], short['skip'], short['diverged_why']) == ('skipped', 'laneBlocked', '')
  assert abs(short["lane_request"]["held_s"] - 3.) < 0.15 and short['lane_request']['ended'] == 'lineNotBroken'
  assert short['lane_request']['desire_helper_would_start']


def test_withheld_frames_with_the_offer_up_dont_end_a_hold_but_a_long_gap_does():
  gap = {'offer': 'left', 'want': 1}
  bridged = left_turn([(2., {}), (3., REQUEST), (sc.LC_GAP_S - 0.1, gap), (3., REQUEST), (4., SKIP)])
  assert bridged['verdict'] == 'would-drive' and bridged['diverged_why'] == 'lane change requested'
  broken = left_turn([(2., {}), (3., REQUEST), (sc.LC_GAP_S + 0.5, gap), (2., REQUEST), (4., SKIP)])
  assert broken['verdict'] == 'skipped' and broken['lane_request']['ended'] == 'withheld'


def test_a_request_in_the_blind_spot_never_starts():
  blind = left_turn([(2., {}), (3., {**REQUEST, 'lbs': True}), (4., SKIP)])
  assert not blind['lane_request']['desire_helper_would_start']


def test_driving_on_where_the_car_stood_long_at_a_signal_says_check_the_light():
  stood = {'v': 0., 'en': True}
  long_stand = left_turn([(2., {}), (6., stood), (5., {})], kind='signal')
  assert long_stand['verdict'] == 'would-drive' and long_stand['went_before_car'] and long_stand['check_light']
  short_stand = left_turn([(2., {}), (2., stood), (5., {})], kind='signal')
  assert short_stand['went_before_car'] and not short_stand['check_light']
  no_signal = left_turn([(2., {}), (6., stood), (5., {})], kind='stop')
  assert no_signal['went_before_car'] and not no_signal['check_light']
