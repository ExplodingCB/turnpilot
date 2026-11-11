"""Torch-free input preparation for Alpamayo 2 Edge (numpy + tinygrad only).

Replaces helper.prepare_model_inputs on the comma:
  * canonical frame order (camera, then timestamp) and relative time labels
  * prompt ids from template.npz (built once on the PC by build_template.py)
  * uint8 bicubic antialiased resize matching torchvision.transforms.functional.resize on uint8
    tensors (float32 interpolation, clamp, round), then (x-127.5)/127.5 and the processor's
    block-major patch layout (bh, bw, mh, mw, ph, pw, c)
"""
from __future__ import annotations
import functools, json
import numpy as np

PATCH, MERGE = 16, 2
FRONT_CAMS = (1, 6)
# fixed resized sizes for 540x960 inputs (smart_resize with the checkpoint pixel bounds)
SIZE_FOR_540x960 = {"front": (320, 576), "other": (224, 416)}
FRONT_BOUNDS, OTHER_BOUNDS = (163840, 196608), (81920, 98304)


def smart_resize(h: int, w: int, factor: int, min_pixels: int, max_pixels: int) -> tuple[int, int]:
  """transformers.models.qwen2_vl.image_processing_qwen2_vl.smart_resize."""
  import math
  hb, wb = round(h / factor) * factor, round(w / factor) * factor
  if hb * wb > max_pixels:
    beta = math.sqrt((h * w) / max_pixels)
    hb, wb = max(factor, math.floor(h / beta / factor) * factor), max(factor, math.floor(w / beta / factor) * factor)
  elif hb * wb < min_pixels:
    beta = math.sqrt(min_pixels / (h * w))
    hb, wb = math.ceil(h * beta / factor) * factor, math.ceil(w * beta / factor) * factor
  return hb, wb


def target_size(cam: int, h: int, w: int) -> tuple[int, int]:
  lo, hi = FRONT_BOUNDS if cam in FRONT_CAMS else OTHER_BOUNDS
  return smart_resize(h, w, PATCH * MERGE, lo, hi)


# ---------------- resize weights (torch upsample_bicubic2d_aa, a=-0.5, align_corners=False) ----------------
def _cubic(x: np.ndarray, a: float = -0.5) -> np.ndarray:
  x = np.abs(x)
  return np.where(x < 1, ((a + 2) * x - (a + 3)) * x * x + 1,
                  np.where(x < 2, (((x - 5) * x + 8) * x - 4) * a, 0.0))


@functools.lru_cache(maxsize=16)
def aa_bicubic_weights(n_in: int, n_out: int) -> np.ndarray:
  """Dense (n_out, n_in) float32 weight matrix, same taps/normalization as ATen's AA kernel."""
  scale = n_in / n_out
  support = 2.0 * scale if scale >= 1.0 else 2.0
  invscale = 1.0 / scale if scale >= 1.0 else 1.0
  W = np.zeros((n_out, n_in), np.float32)
  for i in range(n_out):
    center = scale * (i + 0.5)
    xmin = max(int(center - support + 0.5), 0)
    xmax = min(int(center + support + 0.5), n_in)
    j = np.arange(xmax - xmin, dtype=np.float64)
    w = _cubic((j + xmin - center + 0.5) * invscale)
    tot = w.sum()
    W[i, xmin:xmax] = (w / tot if tot != 0 else w).astype(np.float32)
  return W


def resize_uint8_np(img: np.ndarray, h: int, w: int) -> np.ndarray:
  """img (H, W, 3) uint8 -> (h, w, 3) uint8. Reference implementation (float32, horizontal then vertical)."""
  Wh, Ww = aa_bicubic_weights(img.shape[0], h), aa_bicubic_weights(img.shape[1], w)
  x = img.astype(np.float32)
  x = np.einsum("ow,hwc->hoc", Ww, x)  # horizontal pass
  x = np.einsum("oh,hwc->owc", Wh, x)  # vertical pass
  return np.round(np.clip(x, 0, 255)).astype(np.uint8)


def patchify_np(img: np.ndarray) -> np.ndarray:
  """img (h, w, 3) uint8 -> (h/16*w/16, 768) float32 normalized, processor layout."""
  h, w, _ = img.shape
  gh, gw = h // PATCH, w // PATCH
  x = img.reshape(gh // MERGE, MERGE, PATCH, gw // MERGE, MERGE, PATCH, 3).transpose(0, 3, 1, 4, 2, 5, 6)
  return ((x.reshape(gh * gw, PATCH * PATCH * 3).astype(np.float32) - 127.5) / 127.5).astype(np.float32)


def pixel_values_np(images: np.ndarray, cams: list[int]) -> tuple[np.ndarray, np.ndarray]:
  out, grids = [], []
  for img, c in zip(images, cams):
    h, w = target_size(c, img.shape[0], img.shape[1])
    out.append(patchify_np(resize_uint8_np(img, h, w)))
    grids.append((1, h // PATCH, w // PATCH))
  return np.concatenate(out), np.array(grids, np.int64)


def pixel_values_tg(images, cams: list[int], device: str | None = None):
  """Same as pixel_values_np but on a tinygrad device; images: Tensor or ndarray (N, H, W, 3) uint8.
  Groups frames by target size and resizes with two fp32 matmuls per group. Returns (Tensor, grids)."""
  from tinygrad import Tensor, dtypes
  x = images if isinstance(images, Tensor) else Tensor(np.ascontiguousarray(images), device=device)
  N, H, W, _ = x.shape
  sizes = [target_size(c, H, W) for c in cams]
  parts: dict[int, Tensor] = {}
  for hw in dict.fromkeys(sizes):
    idx = [i for i, s in enumerate(sizes) if s == hw]
    h, w = hw
    Wh = Tensor(aa_bicubic_weights(H, h), device=x.device)
    Ww = Tensor(aa_bicubic_weights(W, w), device=x.device)
    g = Tensor.stack(*[x[i] for i in idx]).cast(dtypes.float32)          # (n, H, W, 3)
    g = g.permute(0, 3, 1, 2)                                              # (n, 3, H, W)
    g = g.matmul(Ww.T)                                                     # horizontal: (n, 3, H, w)
    g = Wh.matmul(g)                                                       # vertical:   (n, 3, h, w)
    g = g.clip(0, 255).round()
    gh, gw = h // PATCH, w // PATCH
    g = g.reshape(len(idx), 3, gh // MERGE, MERGE, PATCH, gw // MERGE, MERGE, PATCH).permute(0, 2, 5, 3, 6, 4, 7, 1)
    g = ((g - 127.5) / 127.5).reshape(len(idx), gh * gw, PATCH * PATCH * 3)
    for j, i in enumerate(idx): parts[i] = g[j]
  grids = np.array([(1, h // PATCH, w // PATCH) for h, w in sizes], np.int64)
  return Tensor.cat(*[parts[i] for i in range(N)]), grids


# ---------------- prompt assembly ----------------
class Template:
  def __init__(self, path: str):
    z = np.load(path)
    keys = json.loads(str(z["chunk_keys"]))
    ids, offs = z["chunk_ids"], z["chunk_offsets"]
    self.chunks = {k: ids[offs[i]:offs[i + 1]].tolist() for i, k in enumerate(keys)}
    self.sp = json.loads(str(z["specials"]))
    self.cam_names = {int(k): v for k, v in json.loads(str(z["camera_names"])).items()}
    self.system_prompt, self.driving_task = str(z["system_prompt"]), str(z["driving_task"])
    self.navs = json.loads(str(z["navs"]))
    self.vocab_bytes, self.vocab_offsets = z["vocab_bytes"].tobytes(), z["vocab_offsets"]

  def chunk(self, s: str) -> list[int]:
    if s not in self.chunks: raise KeyError(f"text chunk not in template: {s!r}")
    return self.chunks[s]

  def build(self, cams_sorted: list[int], labels: list[str], n_img_tokens: list[int], nav_text: str,
            history_tokens: int = 57) -> tuple[np.ndarray, np.ndarray]:
    """Returns (input_ids with history placeholders, mm_token_type_ids)."""
    sp = self.sp
    ids = self.chunk("\n") + [sp["<|im_start|>"]] + self.chunk(f"system\n{self.system_prompt}") + [sp["<|im_end|>"]] + \
      self.chunk("\n") + [sp["<|im_start|>"]]
    prev = None
    for k, (c, lab, n) in enumerate(zip(cams_sorted, labels, n_img_tokens)):
      if c != prev: ids += self.chunk(("user\n" if k == 0 else "") + f"{self.cam_names[c]}: {lab}")
      else: ids += self.chunk(lab)
      ids += [sp["<|vision_start|>"]] + [sp["<|image_pad|>"]] * n + [sp["<|vision_end|>"]]
      prev = c
    ids += self.chunk("\n") + [sp["<|traj_history_start|>"]] + [sp["<|traj_history|>"]] * history_tokens + \
      [sp["<|traj_history_end|>"]]
    ids += self.chunk(f"\nNavigation input is: {nav_text}\n{self.driving_task}")
    ids += [sp["<|im_end|>"]] + self.chunk("\n") + [sp["<|im_start|>"]] + self.chunk("assistant\n") + [sp["<think>"], sp["</think>"]]
    ids = np.array(ids, np.int64)
    return ids, (ids == sp["<|image_pad|>"]).astype(np.int64)

  def decode(self, ids) -> str:
    return b"".join(self.vocab_bytes[self.vocab_offsets[i]:self.vocab_offsets[i + 1]] for i in ids).decode("utf-8", "replace")


def time_labels(ts_sorted: np.ndarray) -> list[str]:
  d = np.asarray(ts_sorted, np.int64) - int(ts_sorted[-1])
  return [f"<t={x / 1_000_000:z.1f}s>" for x in d.tolist()]


def canonical_order(cams: np.ndarray, ts: np.ndarray) -> np.ndarray:
  """sample.canonical_frame_order: stable sort by time, then stable sort by camera."""
  by_t = np.argsort(np.asarray(ts), kind="stable")
  return by_t[np.argsort(np.asarray(cams)[by_t], kind="stable")]


def prepare_prompt(template: Template, camera_ids, timestamps_us, hw: tuple[int, int], nav_text: str) -> dict:
  """Prompt only (no pixels): order, sorted cams, grids, input_ids (history placeholders), mm types."""
  cams, ts = np.asarray(camera_ids, np.int64), np.asarray(timestamps_us, np.int64)
  order = canonical_order(cams, ts)
  cams_s, ts_s = cams[order].tolist(), ts[order]
  sizes = [target_size(c, hw[0], hw[1]) for c in cams_s]
  grids = np.array([(1, h // PATCH, w // PATCH) for h, w in sizes], np.int64)
  n_tok = [int(t * h * w) // (MERGE * MERGE) for t, h, w in grids]
  input_ids, mm = template.build(cams_s, time_labels(ts_s), n_tok, nav_text)
  return dict(input_ids=input_ids, mm_token_type_ids=mm, image_grid_thw=grids, order=order, cams=cams_s, sizes=sizes)


def prepare(template: Template, images: np.ndarray, camera_ids, timestamps_us, nav_text: str, device: str | None = None,
            use_tinygrad: bool = True) -> dict:
  """images (N,H,W,3) uint8 in any order -> dict(input_ids (placeholders), mm_token_type_ids, pixel_values, image_grid_thw, order)."""
  cams, ts = np.asarray(camera_ids, np.int64), np.asarray(timestamps_us, np.int64)
  order = canonical_order(cams, ts)
  cams_s, ts_s = cams[order].tolist(), ts[order]
  imgs = np.asarray(images)[order]
  if use_tinygrad: pv, grids = pixel_values_tg(imgs, cams_s, device)
  else: pv, grids = pixel_values_np(imgs, cams_s)
  n_tok = [int(t * h * w) // (MERGE * MERGE) for t, h, w in grids]
  input_ids, mm = template.build(cams_s, time_labels(ts_s), n_tok, nav_text)
  return dict(input_ids=input_ids, mm_token_type_ids=mm, pixel_values=pv, image_grid_thw=grids, order=order)
