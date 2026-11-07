#!/usr/bin/env python3
"""List and download the drive logs the cars sent our map server (mapserver/logstore.py).

  fetch_logs.py                   list every car's segments on the server
  fetch_logs.py <route> [-o dir]  download every file of a route, e.g. 00000011--24ea9deaa5

The key and the server's certificate are read from ~/turnpilot_data/mapserver/client/ (key, cert.pem), never printed.
Files land in <dir>/<dongle>/<segment>/ (default ~/turnpilot_data/logs), and a file already there with the same size is skipped.
Every file the server lists for a segment comes down: qlog.zst, rlog.zst and, from 2026-11-06, qcamera.ts, the segment's
small road video (526x330 H.264 at 20 fps; play it with ffplay or mpv, or pull frames with ffmpeg), so the night lights, a
curb or a bus lane can be looked at beside the logs.
"""
import argparse
import os
from pathlib import Path

import requests

CLIENT = Path.home() / 'turnpilot_data/mapserver/client'
HOST = os.environ.get('TURNPILOT_MAPS_URL', 'https://136.113.85.41')


def segment_order(seg: str):
  route, num = seg.rsplit('--', 1)
  return route, int(num)


def fetch(s: requests.Session, host: str, route: str | None, out: Path) -> list[Path]:
  """With a route, download its files the server holds into out/<dongle>/<segment>/ and return them; with none, list."""
  listing = s.get(f'{host}/v1/logs', timeout=30)
  listing.raise_for_status()
  got = []
  for dongle, segs in listing.json().items():
    for seg in sorted(segs, key=segment_order):
      if route is None:
        print(dongle, seg, ' '.join(f"{n} {size / 1e6:.1f} MB" for n, size in sorted(segs[seg].items())))
        continue
      if not seg.startswith(route + '--'):
        continue
      for name, size in segs[seg].items():
        dst = out / dongle / seg / name
        if dst.is_file() and dst.stat().st_size == size:
          continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        with s.get(f'{host}/v1/logs/{dongle}/{seg}/{name}', stream=True, timeout=60) as r:
          r.raise_for_status()
          tmp = dst.with_name(dst.name + '.part')
          with open(tmp, 'wb') as f:
            for chunk in r.iter_content(1 << 20):
              f.write(chunk)
          tmp.replace(dst)
        print('got', dst)
        got.append(dst)
  return got


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('route', nargs='?')
  ap.add_argument('-o', '--out', type=Path, default=Path.home() / 'turnpilot_data/logs')
  args = ap.parse_args()
  s = requests.Session()
  s.headers['Authorization'] = f"Bearer {(CLIENT / 'key').read_text().strip()}"
  s.verify = str(CLIENT / 'cert.pem')
  fetch(s, HOST, args.route, args.out)


if __name__ == '__main__':
  main()
