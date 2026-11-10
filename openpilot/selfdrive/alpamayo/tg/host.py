"""Host-side (numpy) parts of the Alpamayo 2 Edge port.

Everything here is cheap and runs on the comma CPU: history token fusion, M-RoPE
position ids, positional-embedding resampling, CoT sampling and action_to_traj.
"""
from __future__ import annotations
import json, struct
import numpy as np

# ---- checkpoint constants (config.json of nvidia/Alpamayo2-Edge rev f8951d0) ----
TRAJ_IDS = dict(future_end=135086, future_id0=132072, future_pad=135088, future_start=135084,
                history_end=135079, history_id0=131072, history_pad=135087, history_start=135077)
HISTORY_VOCAB, FUTURE_VOCAB = 1000, 3000
IMAGE_TOKEN_ID = 19
BLOCKED_EOS = [11]
MROPE_SECTION = (24, 20, 20)
ROPE_THETA = 1e8
HEAD_DIM = 128
SPATIAL_MERGE = 2


def bf16_round(v):
  """Round float32 values to bfloat16 (round-to-nearest-even), returned as float32."""
  u = np.asarray(v, np.float32).view(np.uint32).astype(np.uint64)
  u = ((u + 0x7FFF + ((u >> 16) & 1)) >> 16) << 16
  return u.astype(np.uint32).view(np.float32)


# The reference loads the checkpoint in BF16, which also casts the non-persistent action-space
# buffers to BF16. Mirror that so trajectories match the official path (fp32 values differ by 2 cm
# at 6.4 s on the parity sample). Set BF16_BUFFERS=False for the config's exact fp32 values.
BF16_BUFFERS = True
_ACC = (0.02902694707164455, 0.6810426736454882, 0.0002692167976330542, 0.026148280660833106)
ACCEL_MEAN, ACCEL_STD, KAPPA_MEAN, KAPPA_STD = (float(bf16_round(v)) if BF16_BUFFERS else v for v in _ACC)
DT = 0.1
V_LAMBDA, V_RIDGE = 1e-6, 1e-4
SLIP_GRADIENT = 2.5e-3


# ---------------- safetensors mmap (embedding table stays on host, lazily paged) ----------------
def safetensors_header(path: str) -> tuple[dict, int]:
  with open(path, "rb") as f:
    n = struct.unpack("<Q", f.read(8))[0]
    return json.loads(f.read(n)), 8 + n


def mmap_bf16(path: str, key: str) -> np.memmap:
  hdr, base = safetensors_header(path)
  meta = hdr[key]
  assert meta["dtype"] == "BF16", meta
  s, e = meta["data_offsets"]
  return np.memmap(path, dtype=np.uint16, mode="r", offset=base + s, shape=tuple(meta["shape"]))


def bf16_to_f32(u16: np.ndarray) -> np.ndarray:
  return (np.asarray(u16).astype(np.uint32) << 16).view(np.float32)


# ---------------- history tokens ----------------
def history_tokens(hist_xyz: np.ndarray) -> np.ndarray:
  """DeltaTrajectoryTokenizer(pad_origin_at_beginning=False), 20 poses -> 57 ids."""
  xyz = np.asarray(hist_xyz, np.float32).reshape(-1, 3)
  d = xyz[1:] - xyz[:-1]
  lo, hi = np.array([-4, -4, -10], np.float32), np.array([4, 4, 10], np.float32)
  d = (d - lo) / (hi - lo)
  tok = np.clip(np.round(d * (HISTORY_VOCAB - 1)).astype(np.int64), 0, HISTORY_VOCAB - 1)
  return tok.reshape(-1) + TRAJ_IDS["history_id0"]


def fuse_history(input_ids: np.ndarray, hist_xyz: np.ndarray) -> np.ndarray:
  ids = np.array(input_ids, np.int64).reshape(-1).copy()
  slots = np.nonzero(ids == TRAJ_IDS["history_pad"])[0]
  tok = history_tokens(hist_xyz)
  assert len(slots) == len(tok), (len(slots), len(tok))
  ids[slots] = tok
  return ids


# ---------------- M-RoPE ----------------
def rope_index(mm_token_type_ids: np.ndarray, image_grid_thw: np.ndarray) -> tuple[np.ndarray, int]:
  """Port of Cosmos3EdgeModel.get_rope_index for one unpadded sequence of images + text."""
  types = np.asarray(mm_token_type_ids).reshape(-1)
  grids = iter(np.asarray(image_grid_thw).tolist())
  chunks, cur = [], 0
  i, L = 0, len(types)
  while i < L:
    j = i
    while j < L and types[j] == types[i]: j += 1
    if types[i] == 0:
      chunks.append(np.broadcast_to(np.arange(j - i) + cur, (3, j - i)))
      cur += j - i
    else:
      t, h, w = next(grids)
      h2, w2 = h // SPATIAL_MERGE, w // SPATIAL_MERGE
      T, Hh, Ww = np.meshgrid(np.arange(t), np.arange(h2) + cur, np.arange(w2) + cur, indexing="ij")
      p = np.stack([T.reshape(-1) + cur, Hh.reshape(-1), Ww.reshape(-1)])
      assert p.shape[1] == j - i, "image group length does not match its grid"
      chunks.append(p)
      cur += max(h, w) // SPATIAL_MERGE
    i = j
  pos = np.concatenate(chunks, 1).astype(np.int64)
  return pos, int(pos.max() + 1 - L)


def mrope_axis_select() -> np.ndarray:
  """Which of (T,H,W) drives each of the 64 inverse frequencies (interleaved layout)."""
  sel = np.zeros(HEAD_DIM // 2, np.int64)
  for dim, off in ((1, 1), (2, 2)):
    sel[off:MROPE_SECTION[dim] * 3:3] = dim
  return sel


def rope_cos_sin(pos3: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """pos3: (3, L) -> cos, sin (L, 128) float32, matching Cosmos3EdgeTextRotaryEmbedding."""
  inv = 1.0 / (ROPE_THETA ** (np.arange(0, HEAD_DIM, 2, dtype=np.float32) / HEAD_DIM))
  freqs = pos3[:, :, None].astype(np.float32) * inv[None, None, :].astype(np.float32)  # (3, L, 64)
  sel = mrope_axis_select()
  f = freqs[sel, :, np.arange(len(sel))].T  # (L, 64): frequency i takes axis sel[i]
  emb = np.concatenate([f, f], -1)
  return np.cos(emb).astype(np.float32), np.sin(emb).astype(np.float32)


# ---------------- vision positional embeddings ----------------
def _aa_weights(n_in: int, n_out: int) -> np.ndarray:
  """torch F.interpolate(mode='bilinear', antialias=True, align_corners=False) 1-D weight matrix."""
  scale = n_in / n_out
  support = scale if scale >= 1.0 else 1.0
  invscale = 1.0 / scale if scale >= 1.0 else 1.0
  W = np.zeros((n_out, n_in), np.float64)
  for i in range(n_out):
    center = scale * (i + 0.5)
    xmin = max(int(center - support + 0.5), 0)
    xmax = min(int(center + support + 0.5), n_in)
    w = np.array([max(0.0, 1.0 - abs((j + xmin - center + 0.5) * invscale)) for j in range(xmax - xmin)])
    tot = w.sum()
    if tot > 0: w = w / tot
    W[i, xmin:xmax] = w
  return W


def vision_pos_embed(pos_weight: np.ndarray, grid_thw: np.ndarray) -> np.ndarray:
  """pos_weight (256, C) -> packed (sum(t*h*w), C) in block-major 2x2 merge order."""
  n = int(round(pos_weight.shape[0] ** 0.5))
  grid = pos_weight.reshape(n, n, -1).astype(np.float64)
  out = []
  for t, h, w in np.asarray(grid_thw).tolist():
    r = np.einsum("ih,hwc->iwc", _aa_weights(n, h), grid)
    r = np.einsum("jw,iwc->ijc", _aa_weights(n, w), r)
    m = SPATIAL_MERGE
    r = r.reshape(h // m, m, w // m, m, -1).transpose(0, 2, 1, 3, 4).reshape(h * w, -1)
    out.append(np.tile(r, (t, 1)))
  return np.concatenate(out, 0).astype(np.float32)


# ---------------- CoT sampling ----------------
def mask_logits(logits: np.ndarray) -> np.ndarray:
  x = np.array(logits, np.float32).reshape(-1).copy()
  x[TRAJ_IDS["history_id0"]:TRAJ_IDS["history_id0"] + HISTORY_VOCAB] = -np.inf
  x[TRAJ_IDS["future_id0"]:TRAJ_IDS["future_id0"] + FUTURE_VOCAB] = -np.inf
  x[BLOCKED_EOS] = -np.inf
  return x


def sample_token(logits: np.ndarray, rng: np.random.Generator, temperature=0.6, top_p=0.98) -> int:
  """HF order: logits processors, temperature, top-p (keep >=1), multinomial."""
  x = mask_logits(logits) / temperature
  order = np.argsort(x, kind="stable")  # ascending, like HF
  p = np.exp(x[order] - x[order[-1]]); p /= p.sum()
  remove = np.cumsum(p) <= (1 - top_p)
  remove[-1] = False
  x[order[remove]] = -np.inf
  q = np.exp(x - x.max()); q /= q.sum()
  return int(rng.choice(len(q), p=q))


# ---------------- action space decode (UnicycleAccelCurvatureCourseActionSpace) ----------------
def _wrap(a): return np.arctan2(np.sin(a), np.cos(a))
def _unwrap(phi): d = _wrap(np.diff(phi)); return np.concatenate([phi[:1], phi[:1] + np.cumsum(d)])
def _yaw(rot): return np.arctan2(rot[..., 1, 0], rot[..., 0, 0])


def _v_from_history(xyz, rot):
  dxy = xyz[1:, :2] - xyz[:-1, :2]
  theta = _unwrap(_yaw(rot))
  N = dxy.shape[0]
  g = (2 / DT * dxy).reshape(-1)
  A = np.zeros((2 * N, N + 1))
  c, s = np.cos(theta), np.sin(theta)
  r = np.arange(N)
  A[2 * r, r], A[2 * r, r + 1] = c[:-1], c[1:]
  A[2 * r + 1, r], A[2 * r + 1, r + 1] = s[:-1], s[1:]
  D = np.zeros((N + 1 - 3, N + 1))
  rr = np.arange(N + 1 - 3)
  D[rr, rr], D[rr, rr + 1], D[rr, rr + 2], D[rr, rr + 3] = -1, 3, -3, 1
  lhs = A.T @ A + (V_LAMBDA / DT ** 6) * D.T @ D + V_RIDGE * np.eye(N + 1)
  return np.linalg.solve(lhs, A.T @ g)


def _course_offset(chord_xy, yaw, chord_dt):
  cy, sy = np.cos(yaw), np.sin(yaw)
  along = cy * chord_xy[0] + sy * chord_xy[1]
  across = cy * chord_xy[1] - sy * chord_xy[0]
  off = np.arctan2(across * np.sign(along), abs(along))
  if abs(off) > 0.05: off = 0.0
  speed = np.linalg.norm(chord_xy) / chord_dt
  return off * np.clip((speed - 0.5) / (1.0 - 0.5), 0.0, 1.0)


def action_to_traj(action: np.ndarray, hist_xyz: np.ndarray, hist_rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """action (64, 2) normalized accel/curvature -> xyz (64, 3), rot (64, 3, 3) in the t0 rig frame."""
  a = np.asarray(action, np.float64)
  xyz = np.asarray(hist_xyz, np.float64).reshape(-1, 3)
  rot = np.asarray(hist_rot, np.float64).reshape(-1, 3, 3)
  accel = a[:, 0] * ACCEL_STD + ACCEL_MEAN
  kappa = a[:, 1] * KAPPA_STD + KAPPA_MEAN
  v0 = _v_from_history(xyz, rot)[-1]
  last_yaw = _unwrap(_yaw(rot[-2:]))
  theta0 = _course_offset(xyz[-1, :2] - xyz[-2, :2], last_yaw.mean(), DT)
  vel = np.concatenate([[v0], v0 + np.cumsum(accel * DT)])
  theta = np.concatenate([[theta0], theta0 + np.cumsum(kappa * vel[:-1] * DT) + np.cumsum(kappa * accel * 0.5 * DT ** 2)])
  h = 0.5 * DT
  x = np.cumsum(vel[:-1] * np.cos(theta[:-1]) * h) + np.cumsum(vel[1:] * np.cos(theta[1:]) * h)
  y = np.cumsum(vel[:-1] * np.sin(theta[:-1]) * h) + np.cumsum(vel[1:] * np.sin(theta[1:]) * h)
  out_xyz = np.stack([x, y, np.full_like(x, xyz[-1, 2])], -1)
  lat = vel * np.gradient(theta, DT)
  yaw = theta - theta0 + SLIP_GRADIENT * (lat - lat[0])
  cy, sy = np.cos(yaw[1:]), np.sin(yaw[1:])
  out_rot = np.zeros((len(cy), 3, 3))
  out_rot[:, 0, 0], out_rot[:, 0, 1], out_rot[:, 1, 0], out_rot[:, 1, 1], out_rot[:, 2, 2] = cy, -sy, sy, cy, 1
  return out_xyz.astype(np.float32), out_rot.astype(np.float32)
