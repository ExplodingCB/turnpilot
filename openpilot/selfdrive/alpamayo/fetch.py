#!/usr/bin/env python3
"""alpamayo_fetch: download Alpamayo 2 Edge weights from Hugging Face onto the device.

Runs while Alpamayo is switched on, onroad or offroad, only on unmetered networks.
Resumable (HTTP Range into *.part), each file checked against its LFS sha256, then
renamed into place. Progress goes to the AlpamayoStatus param for the settings UI
and alpamayod.
"""
from __future__ import annotations

import hashlib
import os
import time
import urllib.request

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

REPO = "nvidia/Alpamayo2-Edge"
REV = "b92b6aa"  # byte-identical to the weights validated on the PC (NOTES.md)
WEIGHTS_DIR = os.environ.get("ALPAMAYO_WEIGHTS", "/data/models/alpamayo2_edge")
# only the safetensors are read on device; tokenizer and chat template live in tg/template.npz
FILES = {
  "00000.safetensors": (4275716192, "d13c1a427218baf7b7d365098c0f5496c0e5e6a6bf662836b8eebbe7eb7cfe7f"),
  "00001.safetensors": (1026726076, "cfe424a7320ce729cd927b5ceaab4323a62443b50f461a2f5c0dd0b10634e0b5"),
}
TOTAL = sum(s for s, _ in FILES.values())
CHUNK = 8 << 20


def weights_ready(d: str = WEIGHTS_DIR) -> bool:
  return all(os.path.isfile(p := os.path.join(d, f)) and os.path.getsize(p) == s for f, (s, _) in FILES.items())


def set_status(params: Params, text: str) -> None:
  params.put("AlpamayoStatus", text)


def _have_bytes() -> int:
  n = 0
  for f, (s, _) in FILES.items():
    p = os.path.join(WEIGHTS_DIR, f)
    n += s if os.path.isfile(p) and os.path.getsize(p) == s else (os.path.getsize(p + ".part") if os.path.isfile(p + ".part") else 0)
  return n


def _sha256(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as fh:
    while b := fh.read(CHUNK):
      h.update(b)
  return h.hexdigest()


def _download(params: Params, name: str, size: int, sha: str, ok_to_run) -> bool:
  dst = os.path.join(WEIGHTS_DIR, name)
  part = dst + ".part"
  url = f"https://huggingface.co/{REPO}/resolve/{REV}/{name}"
  last = 0.0
  while (have := os.path.getsize(part) if os.path.exists(part) else 0) < size:
    if not ok_to_run():
      return False
    req = urllib.request.Request(url, headers={"Range": f"bytes={have}-", "User-Agent": "openpilot-alpamayo-fetch"})
    try:
      with urllib.request.urlopen(req, timeout=30) as r, open(part, "ab") as out:
        if have and r.status != 206:  # server ignored the range: start over
          out.truncate(0)
        while b := r.read(CHUNK):
          out.write(b)
          now = time.monotonic()
          if now - last > 2:
            last = now
            set_status(params, f"downloading {100 * _have_bytes() / TOTAL:.0f}%")
            if not ok_to_run():
              return False
    except Exception as e:
      cloudlog.warning(f"alpamayo_fetch: {name}: {e}")
      set_status(params, f"download paused ({type(e).__name__}), retrying")
      time.sleep(10)
  set_status(params, "checking download")
  if _sha256(part) != sha:
    cloudlog.error(f"alpamayo_fetch: {name} checksum mismatch, deleting")
    os.remove(part)
    return False
  os.replace(part, dst)
  return True


def main():
  params = Params()
  sm = messaging.SubMaster(["deviceState"])
  os.makedirs(WEIGHTS_DIR, exist_ok=True)

  def ok_to_run() -> bool:
    sm.update(0)
    return sm.seen["deviceState"] and not sm["deviceState"].networkMetered and \
      str(sm["deviceState"].networkType) not in ("none",)

  while not weights_ready():
    free = os.statvfs(WEIGHTS_DIR).f_bavail * os.statvfs(WEIGHTS_DIR).f_frsize
    if free < TOTAL - _have_bytes() + (2 << 30):  # keep 2 GB for drive logs
      set_status(params, f"not enough storage: need {(TOTAL - _have_bytes()) / 1e9 + 2:.1f} GB free")
      time.sleep(60)
      continue
    if not ok_to_run():
      set_status(params, "waiting for Wi-Fi to download (5.3 GB)")
      time.sleep(5)
      continue
    for name, (size, sha) in FILES.items():
      if not (os.path.isfile(p := os.path.join(WEIGHTS_DIR, name)) and os.path.getsize(p) == size):
        _download(params, name, size, sha, ok_to_run)
  if (params.get("AlpamayoStatus") or "").startswith(("downloading", "checking", "waiting", "download")):
    set_status(params, "downloaded")
  cloudlog.warning("alpamayo_fetch: weights ready")
  while True:  # stay up quietly; the manager stops us when Alpamayo is switched off
    time.sleep(60)


if __name__ == "__main__":
  main()
