"""CPU-only checks of the tinygrad runtime's host side (no weights, no GPU)."""
import numpy as np

from openpilot.selfdrive.alpamayo.tg import host, preproc
from openpilot.selfdrive.alpamayo.tg.alpamayo_tg import AlpamayoTG  # noqa: F401  (import must not need a GPU)

TPL = preproc.Template(__file__.rsplit("/", 1)[0] + "/template.npz")


def _frames(cams_set):
  cams, ts = [], []
  for c in cams_set:
    for o in ([-2e6, -1e6, -5e5, -1e5, 0] if c in (1, 6) else [-1e6, -1e5, 0]):
      cams.append(c); ts.append(int(30e6 + o))
  return np.array(cams), np.array(ts)


def test_prompt_lengths():
  # token counts checked against NVIDIA's processor (test_preproc.py in tinygrad_port)
  for cams_set, nav, n in (((0, 1, 2, 3, 5, 6), "Turn left in 30m", 3261), ((1, 6), "Turn left in 30m", 2031)):
    cams, ts = _frames(cams_set)
    p = preproc.prepare_prompt(TPL, cams, ts, (540, 960), nav)
    assert len(p["input_ids"]) == n
    assert int(p["mm_token_type_ids"].sum()) == sum(int(t * h * w) // 4 for t, h, w in p["image_grid_thw"])


def test_all_nav_texts_tokenized():
  for nav in ["Continue straight"] + [f"Turn {d} in {k}m" for d in ("left", "right") for k in range(1, 41)]:
    TPL.chunk(f"\nNavigation input is: {nav}\n{TPL.driving_task}")


def test_decode_and_history():
  assert TPL.decode([124549, 11135, 3174, 4286, 1561, 135084]) == "<NOMINAL>\n<|traj_future_start|>"
  xyz = np.zeros((20, 3), np.float32); xyz[:, 0] = np.arange(-19, 1) * 1.5
  tok = host.history_tokens(xyz)
  assert len(tok) == 57 and tok.min() >= host.TRAJ_IDS["history_id0"]


def test_action_to_traj_straight():
  xyz = np.zeros((20, 3), np.float32); xyz[:, 0] = np.arange(-19, 1) * 1.5  # 15 m/s straight
  rot = np.tile(np.eye(3, dtype=np.float32), (20, 1, 1))
  a = np.zeros((64, 2), np.float32)
  a[:, 0] = -host.ACCEL_MEAN / host.ACCEL_STD; a[:, 1] = -host.KAPPA_MEAN / host.KAPPA_STD  # zero accel, zero curvature
  out, _ = host.action_to_traj(a, xyz, rot)
  assert abs(out[-1, 0] - 15 * 6.4) < 0.1 and abs(out[-1, 1]) < 1e-3
