"""sideview.SectorQuality's frozen (the review of 2026-11-08): a stuck feed, the wide frame's luma repeated bit for bit, reads
frozen in every sector of the view, whatever else a sector would read, so the stop-sign decider can count its runs as none (the
camera fault, H1); a band clipped black in a live feed (an unlit night stand: no noise lifts it) reads the dark it is, never
frozen, which under TurnPilotStopSignGo=vision waited with no bound."""
import numpy as np

from openpilot.selfdrive.turnpilot.detector import sideview as sv
from openpilot.selfdrive.turnpilot.detector.sideview import SectorQuality
from openpilot.selfdrive.turnpilot.detector.tests.test_sideview import geometry, textured_frame


def qualities(q, geom, frame, t):
  return {(name, s[0]): s[2] for name in ('sideLeft', 'sideRight') for s in q.update(geom, name, frame, t)}


def test_a_stuck_feed_reads_frozen_in_every_sector_even_one_that_would_read_washed():
  geom = geometry()
  q = SectorQuality()
  sat = geom.maps['sideRight'].sectors[3]
  frame = textured_frame(geom, np.random.default_rng(7))
  frame[sat.idx] = 235  # full range 255: washed while the feed lives
  for k in range(7):  # 5 Hz a side, 1.2 s of the same frame
    got = qualities(q, geom, frame.copy(), 0.2 * k)
    if k == 0:
      assert got[('sideRight', sat.inner)] == 'washed'
  assert set(got.values()) == {'frozen'}, got


def test_a_live_feed_refilled_in_the_same_buffer_never_reads_stuck():
  # camerad refills its buffers in place: the sample is a copy, or the feed would always equal itself
  geom = geometry()
  q = SectorQuality()
  rng = np.random.default_rng(3)
  frame = textured_frame(geom, rng)
  for k in range(8):
    frame[:] = textured_frame(geom, rng)
    got = qualities(q, geom, frame, 0.2 * k)
  assert set(got.values()) == {'ok'}, got


def test_a_band_clipped_black_in_a_live_feed_reads_dark_or_blocked_never_frozen_and_a_still_lit_one_still_frozen():
  geom = geometry()
  q = SectorQuality()
  rng = np.random.default_rng(5)
  left = geom.maps['sideLeft'].sectors
  black, still = left[2], left[4]
  stuck = None
  for k in range(7):
    frame = textured_frame(geom, rng)
    frame[black.idx] = 16  # limited-range black: full range 0, the same every frame
    if stuck is None:
      stuck = frame[still.idx].copy()
    frame[still.idx] = stuck  # lit (40-200) and unchanging: no sensor noise, unexplained
    got = qualities(q, geom, frame, 0.2 * k)
  assert got[('sideLeft', black.inner)] in ('dark', 'blocked'), got[('sideLeft', black.inner)]
  assert got[('sideLeft', still.inner)] == 'frozen'
  others = [v for key, v in got.items() if key not in (('sideLeft', black.inner), ('sideLeft', still.inner))]
  assert others and all(v == 'ok' for v in others)


def test_a_gap_between_sweeps_starts_the_stuck_clock_over():
  geom = geometry()
  q = SectorQuality()
  frame = textured_frame(geom, np.random.default_rng(9))
  for t in (0., 0.2, 0.4, 0.6, 0.8 + sv.GAP_S + 0.5, 1.0 + sv.GAP_S + 0.5):
    got = qualities(q, geom, frame.copy(), t)
  assert 'frozen' not in got.values()  # 0.2 s of the same frame since the new sweep began
  for t in (1.2, 1.4, 1.6, 1.8, 2.0):
    got = qualities(q, geom, frame.copy(), t + sv.GAP_S + 0.5)
  assert set(got.values()) == {'frozen'}
