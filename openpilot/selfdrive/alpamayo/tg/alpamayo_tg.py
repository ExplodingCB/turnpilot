"""tinygrad port of NVIDIA Alpamayo 2 Edge inference (nvidia/Alpamayo2-Edge rev f8951d0).

GPU part: SigLIP2 vision tower + projector, Cosmos3-Edge text model (prefill + KV-cached decode),
and the 512-wide flow-matching action expert. Host part (host.py): token fusion, M-RoPE positions,
CoT sampling, action_to_traj.

Precision: weights are kept in their stored BF16. Matmul inputs are rounded to CDT (default bf16,
env ALPA_CDT=bf16|fp16|fp32) and accumulated in fp32. Activations between ops are ADT (env ALPA_ACT,
default bf16 like NVIDIA's BF16 path; fp32 is more accurate but slower). Norm statistics, softmax and
RoPE are computed in fp32. The KV cache is stored in CDT.

Speed structure follows comma's MIT-licensed port (openpilot branch alpamayo2-edge-egpu,
tools/alpamayo/model.py): every layer is realized on its own so identical layers share kernels and
fusion stays local, causal masks are built from aranges instead of an L x L tensor, KV caches have a
fixed capacity, and the vision tower runs in image batches.

Fixed shapes for TinyJit: one JIT per (vision grid layout), one for the prompt length, one decode
step JIT with a symbolic position, one expert step JIT with a symbolic prefix length.
"""
from __future__ import annotations
import collections, hashlib, math, os, time
os.environ.setdefault("TC_OPT", "2")  # let tensor cores pad odd dims (comma's benchmark sets the same)
from dataclasses import dataclass, field
import numpy as np
from tinygrad import Tensor, TinyJit, UOp, dtypes, Device
from tinygrad.nn.state import safe_load

try: from . import host  # package import (turnpilot selfdrive/alpamayo/tg)
except ImportError: import host

F32 = dtypes.float32
_DT = {"bf16": dtypes.bfloat16, "fp16": dtypes.float16, "fp32": dtypes.float32}
CDT = _DT[os.getenv("ALPA_CDT", "bf16")]
ADT = _DT[os.getenv("ALPA_ACT", "bf16")]
VIS_PAD = int(os.getenv("ALPA_VIS_PAD", "16"))  # vision patch-count padding multiple
VISION_CACHE = int(os.getenv("ALPA_VISION_CACHE", "128"))  # encoded frames kept on device (0 disables)

# ---- architecture (config.json) ----
V_LAYERS, V_DIM, V_HEADS, V_FF, V_EPS = 27, 1152, 16, 4304, 1e-6
T_LAYERS, T_DIM, T_HEADS, T_KV, T_HD, T_FF, T_EPS, VOCAB = 28, 2048, 16, 8, 128, 9216, 1e-5, 135168
E_DIM, E_FF = 512, 3840
N_WP, A_DIM = 64, 2
FOURIER_FEATS, FOURIER_MAXF = 20, 100.0


_TC_CACHE: dict[str, bool] = {}
def has_tc(device: str) -> bool:
  """True if the device's renderer has CDT->fp32 tensor cores. Without them, a CDT dot rounds each
  product to CDT before the fp32 sum (1.7e-3 rel error on a 2048-deep matmul on CPU), so we upcast the
  inputs instead (exact products; also faster on CPU). With them, the CDT form maps to WMMA/MMA
  (32 TFLOPS on the 5080 vs 3.6 for the fp32 form)."""
  if device not in _TC_CACHE:
    try: tcs = Device[device].renderer.tensor_cores
    except Exception: tcs = []
    _TC_CACHE[device] = CDT != F32 and any(tc.dtype_in == CDT and tc.dtype_out == F32 for tc in tcs)
  return _TC_CACHE[device]


def mdot(a: Tensor, b: Tensor) -> Tensor:
  """a @ b with CDT-rounded inputs, fp32 accumulation and exact products on every backend."""
  a, b = a.cast(CDT), b.cast(CDT)
  if has_tc(a.device if isinstance(a.device, str) else a.device[0]): return a.matmul(b, dtype=F32)
  return a.cast(F32).matmul(b.cast(F32))


def mm(x: Tensor, w: Tensor, b: Tensor | None = None) -> Tensor:
  """x (..., in) @ w(out, in).T + b; fp32 accumulation, returns ADT."""
  y = mdot(x, w.T)
  return (y + b.cast(F32) if b is not None else y).cast(ADT)


def layernorm(x: Tensor, w: Tensor, b: Tensor, eps: float) -> Tensor:
  return (x.cast(F32).layernorm(eps=eps) * w.cast(F32) + b.cast(F32)).cast(ADT)


def rmsnorm(x: Tensor, w: Tensor, eps: float) -> Tensor:
  x = x.cast(F32)
  return (x * (x.square().mean(-1, keepdim=True) + eps).rsqrt() * w.cast(F32)).cast(ADT)


def causal_mask(q_start, q_len: int, k_len) -> Tensor:
  """(q_len, k_len) additive mask from aranges (no materialized L x L tensor input)."""
  return (Tensor.arange(k_len).reshape(1, -1) > (Tensor.arange(q_len) + q_start).reshape(-1, 1)).where(-math.inf, 0.0)


def attention(q: Tensor, k: Tensor, v: Tensor, scale: float, mask: Tensor | None = None) -> Tensor:
  """q (B,H,Tq,D), k/v (B,H,Tk,D) -> (B,H,Tq,D); fp32 scores and softmax, CDT matmul inputs."""
  s = mdot(q, k.transpose(-1, -2)) * scale
  if mask is not None: s = s + mask
  return mdot(s.softmax(-1), v).cast(ADT)


def rotate_half(x: Tensor) -> Tensor:
  h = x.shape[-1] // 2
  return (-x[..., h:]).cat(x[..., :h], dim=-1)


def rope(x: Tensor, cos: Tensor, sin: Tensor) -> Tensor:
  x = x.cast(F32)
  return (x * cos + rotate_half(x) * sin).cast(ADT)


@dataclass
class Layout:
  """Static shapes of one prompt layout (fixed per camera resolution, nav length and history)."""
  grids: list[tuple[int, int, int]]
  prompt_len: int
  max_gen: int = 256

  @property
  def n_patches(self) -> int: return sum(t * h * w for t, h, w in self.grids)
  @property
  def max_ctx(self) -> int: return ((self.prompt_len + self.max_gen + 255) // 256) * 256  # fixed capacity


class Alpamayo2EdgeTG:
  def __init__(self, model_dir: str, device: str | None = None, emb_on_host: bool = True):
    self.device = device or Device.DEFAULT
    self.dir = model_dir
    # tinygrad's default 30 s device wait is too short for whole-model JIT replays on slow devices (CPU)
    to = float(os.getenv("ALPA_WAIT_TIMEOUT_MS", "600000"))
    for d in {self.device, "CPU"}:
      try: Device[d].wait_timeout_ms = to
      except Exception: pass
    self.w: dict[str, Tensor] = {}
    files = sorted(f for f in os.listdir(model_dir) if f.endswith(".safetensors"))
    self.emb_on_host = emb_on_host
    emb_key = "vlm.model.language_model.embed_tokens.weight"
    for f in files:
      path = os.path.join(model_dir, f)
      hdr, _ = host.safetensors_header(path)
      for k in hdr:
        if k == "__metadata__": continue
        arr = host.mmap_bf16(path, k)
        if k == "vlm.model.visual.embeddings.position_embedding.weight": self.pos_weight = host.bf16_to_f32(arr)
        if k == emb_key and emb_on_host:
          self.emb_host = arr
          continue
        # numpy uint16 -> device, then reinterpret as bf16 (much faster than tinygrad's DISK path)
        # realize one tensor at a time: big batched copies can trip tinygrad's 30 s device wait timeout
        self.w[k] = Tensor(np.ascontiguousarray(arr), device=self.device).bitcast(dtypes.bfloat16).contiguous().realize()
    self._jits: dict = {}
    self._vis_jits: dict = {}
    self._vis_pe: dict = {}
    self.layout: Layout | None = None
    # fourier frequencies, bf16-rounded like the reference's BF16 buffer
    fr = np.logspace(0, math.log10(FOURIER_MAXF), FOURIER_FEATS // 2).astype(np.float32)
    if host.BF16_BUFFERS: fr = host.bf16_round(fr)
    self.fourier = Tensor(fr, device=self.device).realize()

  def nbytes(self) -> int: return sum(t.nbytes() for t in self.w.values())

  # ---------------- layout / buffers ----------------
  def setup(self, layout: Layout):
    if self.layout == layout: return
    self.layout = layout
    # the last N_WP slots are the expert's scratch for its own action K/V (written each Euler step)
    self.cache = [Tensor.zeros(2, 1, T_KV, layout.max_ctx + N_WP, T_HD, dtype=CDT, device=self.device).contiguous().realize()
                  for _ in range(T_LAYERS)]
    self._jits = dict(prefill=TinyJit(self._prefill), decode=TinyJit(self._decode), expert=TinyJit(self._expert_step))

  # ---------------- vision ----------------
  # One image per call, one JIT per grid size. Images are independent in the vision tower (attention
  # is per image), so each encoded frame can be cached and reused by later plans: a frame seen at
  # t0 is the -0.5 s frame of a plan 0.5 s later. Constant filler slots are encoded once.
  def encode_image(self, pv: Tensor, grid: tuple[int, int, int], want_hidden: bool = False):
    """pv (t*h*w, 768) for one image -> (embeds (t*h*w/4, 2048) ADT, hidden or None), fresh buffers."""
    key = tuple(grid)
    if key not in self._vis_jits:
      self._vis_jits[key] = TinyJit(self._vision1)
      pe = host.vision_pos_embed(self.pos_weight, [grid])
      npad = -(-len(pe) // VIS_PAD) * VIS_PAD - len(pe)
      self._vis_pe[key] = Tensor(np.concatenate([pe, np.zeros((npad, V_DIM), np.float32)]), device=self.device).realize()
    # pad the patch count to a multiple of 64 so matmuls tile onto tensor cores (14x26=364 ran 4.5x
    # slower than 20x36=720); pad keys are masked and pad rows dropped before the 2x2 merge
    n = pv.shape[0]
    npad = self._vis_pe[key].shape[0] - n
    pvp = pv.cast(F32).pad(((0, npad), (0, 0))) if npad else pv.cast(F32)
    hid, emb = self._vis_jits[key](pvp.contiguous().realize(), self._vis_pe[key], n)
    # JIT outputs are reused buffers: copy before the next call overwrites them
    return emb.clone().realize(), (hid.clone().realize() if want_hidden else None)

  def _vision1(self, pixel_values: Tensor, pos_embed: Tensor, n_real: int):
    w = self.w
    p = "vlm.model.visual."
    g = (mm(pixel_values, w[p + "embeddings.patch_embedding.weight"], w[p + "embeddings.patch_embedding.bias"]) + pos_embed).cast(ADT)
    n = g.shape[0]
    g = g.reshape(1, n, V_DIM).realize()
    kmask = (Tensor.arange(n) >= n_real).where(-math.inf, 0.0).reshape(1, 1, 1, n) if n_real < n else None
    for l in range(V_LAYERS):
      q = f"{p}encoder.layers.{l}."
      h = layernorm(g, w[q + "layer_norm1.weight"], w[q + "layer_norm1.bias"], V_EPS)
      hd = V_DIM // V_HEADS
      qs = mm(h, w[q + "self_attn.q_proj.weight"], w[q + "self_attn.q_proj.bias"]).reshape(1, n, V_HEADS, hd).transpose(1, 2)
      ks = mm(h, w[q + "self_attn.k_proj.weight"], w[q + "self_attn.k_proj.bias"]).reshape(1, n, V_HEADS, hd).transpose(1, 2)
      vs = mm(h, w[q + "self_attn.v_proj.weight"], w[q + "self_attn.v_proj.bias"]).reshape(1, n, V_HEADS, hd).transpose(1, 2)
      a = attention(qs, ks, vs, hd ** -0.5, kmask).transpose(1, 2).reshape(1, n, V_DIM)
      g = (g + mm(a, w[q + "self_attn.out_proj.weight"], w[q + "self_attn.out_proj.bias"])).realize()
      h = layernorm(g, w[q + "layer_norm2.weight"], w[q + "layer_norm2.bias"], V_EPS)
      h = mm(h, w[q + "mlp.fc1.weight"], w[q + "mlp.fc1.bias"]).cast(F32).gelu("tanh").cast(ADT)
      g = (g + mm(h, w[q + "mlp.fc2.weight"], w[q + "mlp.fc2.bias"])).realize()
    vis = layernorm(g, w[p + "post_layernorm.weight"], w[p + "post_layernorm.bias"], V_EPS).reshape(n, V_DIM)[:n_real].realize()
    # projector: per-patch LN, 2x2 merge, MLP with exact GELU
    q = "vlm.model.projector."
    y = layernorm(vis, w[q + "norm.weight"], w[q + "norm.bias"], 1e-6).reshape(-1, 4 * V_DIM)
    y = mm(y, w[q + "linear_fc1.weight"], w[q + "linear_fc1.bias"]).cast(F32).gelu("none").cast(ADT)
    y = mm(y, w[q + "linear_fc2.weight"], w[q + "linear_fc2.bias"])
    return vis.contiguous().realize(), y.contiguous().realize()

  # ---------------- text model ----------------
  def _text_layer(self, l: int, x: Tensor, cos: Tensor, sin: Tensor, start: int | UOp, mask: Tensor | None,
                  prefix: str = "vlm.model.language_model.", kv_len: int | UOp | None = None) -> Tensor:
    w = self.w
    q = f"{prefix}layers.{l}."
    B, T, _ = x.shape
    h = rmsnorm(x, w[q + "input_layernorm.weight"], T_EPS)
    qs = mm(h, w[q + "self_attn.q_proj.weight"]).reshape(B, T, T_HEADS, T_HD).transpose(1, 2)
    ks = mm(h, w[q + "self_attn.k_proj.weight"]).reshape(B, T, T_KV, T_HD).transpose(1, 2)
    vs = mm(h, w[q + "self_attn.v_proj.weight"]).reshape(B, T, T_KV, T_HD).transpose(1, 2)
    qs, ks = rope(qs, cos, sin), rope(ks, cos, sin)
    c = self.cache[l]
    c[:, :, :, start:start + T, :].assign(Tensor.stack(ks, vs).cast(c.dtype)).realize()
    # kv_len="full": attend over the whole fixed-capacity cache with a caller mask (static shapes, so
    # kernels optimize; symbolic-length reads were 10x slower), else over [0, start+T)
    end = c.shape[3] if kv_len == "full" else start + T if kv_len is None else kv_len
    kk = c[0, :, :, 0:end, :].repeat_interleave(T_HEADS // T_KV, dim=1)
    vv = c[1, :, :, 0:end, :].repeat_interleave(T_HEADS // T_KV, dim=1)
    a = attention(qs, kk, vv, T_HD ** -0.5, mask).transpose(1, 2).reshape(B, T, T_HEADS * T_HD)
    x = (x + mm(a, w[q + "self_attn.o_proj.weight"])).realize()
    h = rmsnorm(x, w[q + "post_attention_layernorm.weight"], T_EPS)
    h = mm(h, w[q + "mlp.fc1.weight"]).cast(F32).relu().square().cast(ADT)
    return (x + mm(h, w[q + "mlp.fc2.weight"])).realize()

  def _prefill(self, txt_emb: Tensor, img_emb: Tensor, cos: Tensor, sin: Tensor, sel: Tensor):
    """sel: (L, 1) one-hot of the last real prompt token (the prompt may be right-padded so that
    nav texts of different token lengths share one JIT; pad K/V land after the prompt and are
    overwritten by decode before anything attends to them)."""
    L = txt_emb.shape[0]
    # interleave text and image spans (static layout, so this is just views + one concat)
    x = Tensor.cat(*[(img_emb if kind else txt_emb)[a:a + n] for kind, a, n in self.spans]).reshape(1, L, T_DIM).cast(ADT).realize()
    mask = causal_mask(0, L, L).reshape(1, 1, L, L)
    cos, sin = cos.reshape(1, 1, L, T_HD), sin.reshape(1, 1, L, T_HD)
    for l in range(T_LAYERS): x = self._text_layer(l, x, cos, sin, 0, mask)
    last = (x[0] * sel).sum(0, keepdim=True)
    h = rmsnorm(last, self.w["vlm.model.language_model.norm.weight"], T_EPS)
    return h.reshape(1, T_DIM).cast(F32).contiguous(), mm(h, self.w["vlm.lm_head.weight"]).reshape(1, VOCAB).cast(F32).contiguous()

  def _decode(self, emb: Tensor, cos: Tensor, sin: Tensor, mask: Tensor, start: UOp):
    """mask: (cap,) additive, 0 for cache slots <= start."""
    x = emb.reshape(1, 1, T_DIM).cast(ADT)
    cos, sin = cos.reshape(1, 1, 1, T_HD), sin.reshape(1, 1, 1, T_HD)
    mask = mask.reshape(1, 1, 1, -1)
    for l in range(T_LAYERS): x = self._text_layer(l, x, cos, sin, start, mask, kv_len="full")
    h = rmsnorm(x, self.w["vlm.model.language_model.norm.weight"], T_EPS)
    return mm(h, self.w["vlm.lm_head.weight"]).reshape(1, VOCAB).cast(F32).contiguous()

  # ---------------- expert ----------------
  def _fourier(self, x: Tensor) -> Tensor:
    arg = x.unsqueeze(-1) * self.fourier * (2 * math.pi)
    return arg.sin().cat(arg.cos(), dim=-1) * math.sqrt(2)

  def _expert_step(self, xa: Tensor, t: Tensor, cos: Tensor, sin: Tensor, mask: Tensor) -> Tensor:
    """One flow-matching velocity evaluation. xa (1,64,2) fp32, t (1,1) fp32,
    mask (cap+64,) additive: 0 for the CoT prefix and the 64 action slots, -inf elsewhere."""
    w = self.w
    p = "expert.action_in_proj."
    feats = self._fourier(xa[..., 0]).cat(self._fourier(xa[..., 1]), dim=-1)  # (1,64,40)
    tf = self._fourier(t).reshape(1, 1, FOURIER_FEATS).expand(1, N_WP, FOURIER_FEATS)
    h = feats.cat(tf, dim=-1)
    h = mm(h, w[p + "encoder.trunk.0.weight"], w[p + "encoder.trunk.0.bias"]).cast(F32).silu().cast(ADT)
    h = mm(rmsnorm(h, w[p + "encoder.trunk.2.weight"], 1e-5), w[p + "encoder.trunk.3.weight"], w[p + "encoder.trunk.3.bias"]).cast(F32).silu().cast(ADT)
    h = mm(rmsnorm(h, w[p + "encoder.trunk.5.weight"], 1e-5), w[p + "encoder.trunk.6.weight"], w[p + "encoder.trunk.6.bias"])
    x = layernorm(h, w[p + "norm.weight"], w[p + "norm.bias"], 1e-5)
    cos, sin = cos.reshape(1, 1, N_WP, T_HD), sin.reshape(1, 1, N_WP, T_HD)
    mask = mask.reshape(1, 1, 1, -1)
    for l in range(T_LAYERS):
      x = self._expert_layer(l, x, cos, sin, mask)
    x = rmsnorm(x, w["expert.expert.norm.weight"], T_EPS)
    return mm(x, w["expert.action_out_proj.weight"], w["expert.action_out_proj.bias"]).cast(F32).contiguous()

  def _expert_layer(self, l, x, cos, sin, mask):
    # noncausal attention over the fixed-capacity cache whose last 64 slots hold this step's action K/V; the mask keeps
    # only the CoT prefix plus those 64 slots (the reference crops the cache to the prefix).
    w = self.w
    q = f"expert.expert.layers.{l}."
    h = rmsnorm(x, w[q + "input_layernorm.weight"], T_EPS)
    qs = mm(h, w[q + "self_attn.q_proj.weight"]).reshape(1, N_WP, T_HEADS, T_HD).transpose(1, 2)
    ks = mm(h, w[q + "self_attn.k_proj.weight"]).reshape(1, N_WP, T_KV, T_HD).transpose(1, 2)
    vs = mm(h, w[q + "self_attn.v_proj.weight"]).reshape(1, N_WP, T_KV, T_HD).transpose(1, 2)
    qs, ks = rope(qs, cos, sin), rope(ks, cos, sin)
    c = self.cache[l]
    cap = c.shape[3] - N_WP
    c[:, :, :, cap:, :].assign(Tensor.stack(ks, vs).cast(c.dtype)).realize()
    kk = c[0].repeat_interleave(T_HEADS // T_KV, dim=1)
    vv = c[1].repeat_interleave(T_HEADS // T_KV, dim=1)
    a = attention(qs, kk, vv, T_HD ** -0.5, mask).transpose(1, 2).reshape(1, N_WP, T_HEADS * T_HD)
    x = (x + mm(a, w[q + "self_attn.o_proj.weight"])).realize()
    h = rmsnorm(x, w[q + "post_attention_layernorm.weight"], T_EPS)
    h = mm(h, w[q + "mlp.fc1.weight"]).cast(F32).relu().square().cast(ADT)
    return (x + mm(h, w[q + "mlp.fc2.weight"])).realize()

  # ---------------- host glue ----------------
  def embed(self, ids: np.ndarray) -> Tensor:
    ids = np.asarray(ids).reshape(-1)
    if self.emb_on_host:
      return Tensor(host.bf16_to_f32(self.emb_host[ids]), device=self.device).reshape(1, len(ids), T_DIM)
    return self.w["vlm.model.language_model.embed_tokens.weight"][Tensor(ids.astype(np.int32), device=self.device)].cast(F32).reshape(1, len(ids), T_DIM)

  def infer(self, input_ids: np.ndarray, mm_token_type_ids: np.ndarray, pixel_values, image_grid_thw,
            hist_xyz: np.ndarray, hist_rot: np.ndarray, noise: np.ndarray, rng: np.random.Generator | None = None,
            forced_ids: list[int] | None = None, pad_to: int | None = None, n_steps: int = 10, max_gen: int = 256,
            record: dict | None = None, timing: dict | None = None) -> dict:
    """Core inference from processor-format inputs.

    input_ids: prompt ids with <|traj_history|> placeholders (history is fused here).
    pixel_values: (Np, 768) normalized patches (ndarray or device Tensor), or a list of per-image
    embeddings already produced by encode_image (in prompt order). noise: (64, 2) initial
    diffusion noise. forced_ids: teacher-forced CoT ending in <|traj_future_start|>. pad_to: right-pad
    the prompt to this length so different nav texts reuse one prefill JIT.
    Returns dict(action (64,2) normalized, xyz, rot, gen_ids)."""
    sync = lambda: Device[self.device].synchronize()
    log = (lambda *a: print("[alpa]", *a, flush=True)) if os.getenv("ALPA_VERBOSE") else (lambda *a: None)
    tm = timing if timing is not None else {}
    t0 = time.perf_counter()
    ids = host.fuse_history(input_ids, hist_xyz)
    grids = [tuple(int(v) for v in g) for g in np.asarray(image_grid_thw).tolist()]
    L = len(ids)
    # right-pad to a multiple of 64 so prefill matmuls tile onto tensor cores (L=3261 = 3 x 1087 does not)
    Lp = -(-max(L, pad_to or 0) // 64) * 64
    pos3, delta = host.rope_index(mm_token_type_ids, grids)
    if Lp > L:  # pads continue the text positions; they are never attended by real tokens
      extra = np.arange(1, Lp - L + 1) + pos3[:, -1:].max()
      pos3 = np.concatenate([pos3, np.broadcast_to(extra, (3, Lp - L))], 1)
      ids = np.concatenate([ids, np.zeros(Lp - L, np.int64)])
    cos, sin = host.rope_cos_sin(pos3)
    is_img = ids == host.IMAGE_TOKEN_ID
    spans, i, n_img = [], 0, 0
    while i < Lp:
      j = i
      while j < Lp and is_img[j] == is_img[i]: j += 1
      spans.append((True, n_img, j - i) if is_img[i] else (False, i, j - i))
      if is_img[i]: n_img += j - i
      i = j
    layout = Layout(grids, Lp, max_gen)
    if self.layout != layout or getattr(self, "spans", None) != spans:
      if self.layout is not None: log("layout changed: rebuilding JITs (costs compile time and memory)")
      self.layout = None
      self.spans = spans
      self.setup(layout)
    dev = self.device
    txt = self.embed(np.where(is_img, 0, ids)).reshape(Lp, T_DIM)
    sel = np.zeros((Lp, 1), np.float32); sel[L - 1] = 1
    host_args = (Tensor(cos, device=dev), Tensor(sin, device=dev), Tensor(sel, device=dev))
    Tensor.realize(txt, *host_args)
    sync(); tm["host_prep"] = time.perf_counter() - t0
    log("host prep done", tm["host_prep"])

    t1 = time.perf_counter()
    hidden = []
    if isinstance(pixel_values, list): embs = pixel_values
    else:
      pv = pixel_values if isinstance(pixel_values, Tensor) else Tensor(np.asarray(pixel_values, np.float32), device=dev)
      embs, off = [], 0
      for g in grids:
        n = g[0] * g[1] * g[2]
        e, h = self.encode_image(pv[off:off + n], g, want_hidden=record is not None)
        embs.append(e); hidden.append(h); off += n
    img_emb = Tensor.cat(*embs).contiguous().realize() if len(embs) > 1 else embs[0]
    sync(); tm["vision"] = time.perf_counter() - t1
    log("vision done", tm["vision"])
    t1 = time.perf_counter()
    last_h, logits = self._jits["prefill"](txt, img_emb, *host_args)
    lg = logits.numpy()[0]
    tm["prefill"] = time.perf_counter() - t1
    log("prefill done", tm["prefill"])
    if record is not None:
      if hidden: record["vision_hidden"] = np.concatenate([h.float().numpy() for h in hidden])
      record["image_embeds"] = img_emb.float().numpy()
      record["prefill_last_hidden"] = last_h.numpy(); record["prefill_logits"] = lg.copy()

    # CoT decode (positions continue from the real prompt end, overwriting any pads)
    cap = self.layout.max_ctx
    v_start = UOp.variable("start_pos", 1, cap - 1)
    anchor = host.TRAJ_IDS["future_start"]
    gen, pos = [], L
    rng = rng if rng is not None else np.random.default_rng(43)
    t1 = time.perf_counter()
    while True:
      nxt = forced_ids[len(gen)] if forced_ids is not None else host.sample_token(lg, rng)
      gen.append(nxt)
      c, s = host.rope_cos_sin(np.full((3, 1), pos + delta, np.int64))
      dmask = np.where(np.arange(cap + N_WP) <= pos, 0.0, -np.inf).astype(np.float32)
      lg = self._jits["decode"](self.embed([nxt]), Tensor(c, device=dev), Tensor(s, device=dev), Tensor(dmask, device=dev),
                                v_start.bind(pos)).numpy()[0]
      pos += 1
      log("decoded", nxt)
      if nxt == anchor or len(gen) >= max_gen: break
    tm["decode"] = time.perf_counter() - t1
    tm["n_gen"] = len(gen)
    if gen[-1] != anchor: raise RuntimeError(f"CoT did not emit <|traj_future_start|> in {len(gen)} tokens; first ids {gen[:32]}")
    prefix = pos  # cache holds prompt + generated tokens (anchor included)

    # expert flow matching
    epos = np.broadcast_to(np.arange(N_WP) + prefix + delta, (3, N_WP))
    ec, es = host.rope_cos_sin(epos)
    ec_t, es_t = Tensor(ec, device=dev).realize(), Tensor(es, device=dev).realize()
    emask = Tensor(np.concatenate([np.where(np.arange(cap) < prefix, 0.0, -np.inf), np.zeros(N_WP)]).astype(np.float32), device=dev).realize()
    x = np.asarray(noise, np.float32).reshape(1, N_WP, A_DIM)
    ts = np.linspace(0.0, 1.0, n_steps + 1, dtype=np.float32)
    outs = []
    t1 = time.perf_counter()
    for i in range(n_steps):
      vel = self._jits["expert"](Tensor(x, device=dev), Tensor(ts[i:i + 1].reshape(1, 1), device=dev), ec_t, es_t, emask).numpy()
      outs.append(vel)
      log("expert step", i)
      x = x + (ts[i + 1] - ts[i]) * vel
    tm["expert"] = time.perf_counter() - t1
    if record is not None: record["expert_out"] = np.stack(outs)
    xyz, rot = host.action_to_traj(x[0], hist_xyz, hist_rot)
    tm["total"] = time.perf_counter() - t0
    return dict(action=x[0], xyz=xyz, rot=rot, gen_ids=gen)

  def run(self, tok: dict, hist_xyz, hist_rot, noise, rng=None, forced_ids=None, record=None, timing=None, **kw):
    """Back-compat wrapper over infer() taking a processor-style dict. Returns xyz, rot, gen_ids."""
    o = self.infer(tok["input_ids"], tok["mm_token_type_ids"], tok["pixel_values"], tok["image_grid_thw"], hist_xyz, hist_rot,
                   noise, rng=rng, forced_ids=forced_ids, record=record, timing=timing, **kw)
    return o["xyz"], o["rot"], o["gen_ids"]


class AlpamayoTG:
  """Comma-side entry point: raw camera frames in, trajectory out. numpy + tinygrad only.

    m = AlpamayoTG(weights_dir, device="USB+AMD:LLVM")
    out = m.run(images_uint8[N,540,960,3], camera_ids[N], timestamps_us[N], ego_xyz[20,3], ego_rot[20,3,3],
                "Turn left in 30m", noise=None, seed=43, frame_keys=None)
    out: xyz[64,3], rot[64,3,3], accel[64] (m/s^2), kappa[64] (1/m), cot_text, gen_ids, timings

  Camera sets: the 6-camera profile (0,1,2,3,5,6 with 3/5/3/3/3/5 frames) or front+tele only
  (1 and 6, 5 frames each). `cameras=2` (env ALPA_CAMERAS) drops everything but slots 1 and 6 from a
  6-camera call. Frames may be in any order; they are sorted by (camera, time) and labelled relative
  to the last one like the reference. Nav text must be one of template.navs ("Continue straight",
  "Turn left in {N}m", "Turn right in {N}m", N=1..40). noise=None draws N(0,1) from numpy with `seed`.

  Vision cache: each frame's embeddings are kept on the device (LRU, ALPA_VISION_CACHE entries) and
  reused when the same frame appears again. frame_keys[i] (any hashable, e.g. (camera, frame_id,
  calibration version)) identifies frames; without keys the image bytes are hashed.
  diffusion_steps (env ALPA_DIFFUSION_STEPS, default 10) sets the Euler steps.
  """
  def __init__(self, weights_dir: str, device: str | None = None, template: str | None = None, emb_on_host: bool = True,
               cameras: int | None = None, diffusion_steps: int | None = None, vision_cache: int | None = None):
    try: from . import preproc
    except ImportError: import preproc
    self.pp = preproc
    self.tpl = preproc.Template(template or os.path.join(os.path.dirname(os.path.abspath(__file__)), "template.npz"))
    self.model = Alpamayo2EdgeTG(weights_dir, device, emb_on_host=emb_on_host)
    self.device = self.model.device
    self.cameras = int(cameras or os.getenv("ALPA_CAMERAS", "6"))
    self.steps = int(diffusion_steps or os.getenv("ALPA_DIFFUSION_STEPS", "10"))
    self.cache_size = VISION_CACHE if vision_cache is None else vision_cache
    self.vcache: collections.OrderedDict = collections.OrderedDict()
    # pad prompts to the longest nav chunk so every nav text shares one prefill JIT
    navc = [len(v) for k, v in self.tpl.chunks.items() if k.startswith("\nNavigation input is: ")]
    self.nav_pad = max(navc)

  def _frame_key(self, img: np.ndarray, cam: int, key) -> tuple:
    if key is not None: return ("k", cam, key)
    return ("h", cam, img.shape, hashlib.blake2b(np.ascontiguousarray(img).data, digest_size=16).digest())

  def run(self, images, camera_ids, timestamps_us, ego_xyz, ego_rot, nav_text: str, noise=None, seed: int = 43,
          forced_ids=None, frame_keys=None) -> dict:
    tm: dict = {}
    t0 = time.perf_counter()
    images, camera_ids, timestamps_us = np.asarray(images), np.asarray(camera_ids), np.asarray(timestamps_us)
    if self.cameras == 2:
      keep = np.isin(camera_ids, (1, 6))
      images, camera_ids, timestamps_us = images[keep], camera_ids[keep], timestamps_us[keep]
      if frame_keys is not None: frame_keys = [k for k, m in zip(frame_keys, keep) if m]
    p = self.pp.prepare_prompt(self.tpl, camera_ids, timestamps_us, images.shape[1:3], nav_text)
    nav_len = len(self.tpl.chunk(f"\nNavigation input is: {nav_text}\n{self.tpl.driving_task}"))
    tm["preprocess"] = time.perf_counter() - t0
    # vision with the per-frame cache
    t1 = time.perf_counter()
    embs, hits = [], 0
    for j, i in enumerate(p["order"]):
      grid = tuple(int(v) for v in p["image_grid_thw"][j])
      key = self._frame_key(images[i], p["cams"][j], None if frame_keys is None else frame_keys[i]) + (grid,)
      if self.cache_size and key in self.vcache:
        self.vcache.move_to_end(key); embs.append(self.vcache[key]); hits += 1
        continue
      pv, _ = self.pp.pixel_values_tg(images[i:i + 1], [p["cams"][j]], self.device)
      e, _ = self.model.encode_image(pv, grid)
      embs.append(e)
      if self.cache_size:
        self.vcache[key] = e
        while len(self.vcache) > self.cache_size: self.vcache.popitem(last=False)
    Device[self.device].synchronize()
    tm["vision_encode"] = time.perf_counter() - t1
    tm["vision_cache_hits"] = hits
    tm["vision_new_frames"] = len(embs) - hits
    if noise is None: noise = np.random.default_rng(seed).standard_normal((N_WP, A_DIM)).astype(np.float32)
    o = self.model.infer(p["input_ids"], p["mm_token_type_ids"], embs, p["image_grid_thw"],
                         np.asarray(ego_xyz, np.float32), np.asarray(ego_rot, np.float32), noise,
                         rng=np.random.default_rng(seed), forced_ids=forced_ids, n_steps=self.steps,
                         pad_to=len(p["input_ids"]) + self.nav_pad - nav_len, timing=tm)
    tm["total"] = time.perf_counter() - t0
    a = o["action"]
    text = self.tpl.decode(o["gen_ids"])
    cot = text.split("<|traj_future_start|>", 1)[0].strip()
    return dict(xyz=o["xyz"], rot=o["rot"], accel=a[:, 0] * host.ACCEL_STD + host.ACCEL_MEAN,
                kappa=a[:, 1] * host.KAPPA_STD + host.KAPPA_MEAN, cot_text=cot, gen_ids=o["gen_ids"], timings=tm)
