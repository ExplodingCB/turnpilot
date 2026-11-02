#!/usr/bin/env python3
"""List and download the drive logs the cars sent our map server (mapserver/logstore.py).

  fetch_logs.py                   list every car's segments on the server
  fetch_logs.py <route> [-o dir]  download every file of a route, e.g. 00000011--24ea9deaa5

The key and the server's certificate are read from ~/turnpilot_data/mapserver/client/ (key, cert.pem), never printed.
Files land in <dir>/<dongle>/<segment>/ (default ~/turnpilot_data/logs), and a file already there with the same size is skipped.
"""
import argparse
import os
from pathlib import Path

import requests

CLIENT = Path.home() / 'turnpilot_data/mapserver/client'
HOST = os.environ.get('TURNPILOT_MAPS_URL', 'https://136.113.85.41')


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('route', nargs='?')
  ap.add_argument('-o', '--out', type=Path, default=Path.home() / 'turnpilot_data/logs')
  args = ap.parse_args()
  s = requests.Session()
  s.headers['Authorization'] = f"Bearer {(CLIENT / 'key').read_text().strip()}"
  s.verify = str(CLIENT / 'cert.pem')
  listing = s.get(f'{HOST}/v1/logs', timeout=30)
  listing.raise_for_status()
  logs = listing.json()
  for dongle, segs in logs.items():
    for seg in sorted(segs, key=lambda x: (x.rsplit('--', 1)[0], int(x.rsplit('--', 1)[1]))):
      if args.route is None:
        print(dongle, seg, ' '.join(f"{n} {size / 1e6:.1f} MB" for n, size in sorted(segs[seg].items())))
        continue
      if not seg.startswith(args.route + '--'):
        continue
      for name, size in segs[seg].items():
        dst = args.out / dongle / seg / name
        if dst.is_file() and dst.stat().st_size == size:
          continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        with s.get(f'{HOST}/v1/logs/{dongle}/{seg}/{name}', stream=True, timeout=60) as r:
          r.raise_for_status()
          tmp = dst.with_name(dst.name + '.part')
          with open(tmp, 'wb') as f:
            for chunk in r.iter_content(1 << 20):
              f.write(chunk)
          tmp.replace(dst)
        print('got', dst)


if __name__ == '__main__':
  main()
