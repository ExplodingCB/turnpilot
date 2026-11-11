"""Chestnut self-test run by alpamayod before it loads the model.

Round-trips random buffers of awkward sizes (around 4 KB pages, 1 MB ring and
chunk boundaries) to the device and back, then times bulk copies. comma's own
Alpamayo branch saw the chestnut USB path corrupt data at buffer boundaries, so
the model only loads if every byte comes back intact.
"""
from __future__ import annotations

import time

import numpy as np

SIZES = (1, 4095, 4096, 4097, (64 << 10) + 3, (1 << 20) - 1, (1 << 20) + 1, (16 << 20) + 5)
BULK = 64 << 20


def usb_check(dev: str, repeats: int = 2) -> dict:
  from tinygrad import Tensor, dtypes
  rng = np.random.default_rng(0)
  res: dict = {"dev": dev, "ok": True, "bad": []}
  st = time.monotonic()
  for r in range(repeats):
    for n in SIZES:
      a = rng.integers(0, 256, n, dtype=np.uint8)
      b = Tensor(a, device=dev, dtype=dtypes.uint8).realize().numpy()
      if not np.array_equal(a, b):
        res["ok"] = False
        res["bad"].append({"size": n, "rep": r, "first_diff": int(np.argmax(a != b)), "n_diff": int((a != b).sum())})
  a = rng.integers(0, 256, BULK, dtype=np.uint8)
  t0 = time.monotonic()
  t = Tensor(a, device=dev, dtype=dtypes.uint8).realize()
  t1 = time.monotonic()
  b = t.numpy()
  t2 = time.monotonic()
  if not np.array_equal(a, b):
    res["ok"] = False
    res["bad"].append({"size": BULK, "rep": "bulk", "n_diff": int((a != b).sum())})
  res["up_MBps"] = round(BULK / 1e6 / max(t1 - t0, 1e-6), 1)
  res["down_MBps"] = round(BULK / 1e6 / max(t2 - t1, 1e-6), 1)
  res["check_s"] = round(time.monotonic() - st, 1)
  return res
