#!/usr/bin/env python3
"""Replay comma four road video through the detector's graphs and count the cars and people found with TurnPilotNativeDetect
off (the full pass and today's far crop) and on (the full pass and the native strip, or the two native strips of full).

  DEV=METAL replay_native.py SEGMENT_DIR [SEGMENT_DIR ...] --count 30 --step 40 --out found.json

Each SEGMENT_DIR holds a comma four segment's fcamera.hevc and rlog.zst (its calibration aims the far crop and the strip at
the horizon straight ahead, as FarCrop.aim does at bearing 0). Every sampled frame runs every pass, each TinyJit from the
ONNX (the graphs the committed pkls hold): the full pass (graph.make_detector), the far crop (make_far_detector 640x384), the
native strip (1344x384) and the native top and bottom strips (1344x512, rows 0 and 248). A schedule's objects on a frame are
the union of its passes' boxes, boxes of one class group overlapping by IoU 0.5 one object: cars (car, bus, truck) and people.
On the car the full mode's strips alternate, one an odd step, so a frame here is two of its odd steps.
"""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

import numpy as np

CARS, PEOPLE = (2, 5, 7), (0,)
SCHEDULES = ('off', 'strip', 'full')


def iou(a, b) -> float:
  x0, y0 = max(a[0], b[0]), max(a[1], b[1])
  x1, y1 = min(a[2], b[2]), min(a[3], b[3])
  inter = max(0., x1 - x0) * max(0., y1 - y0)
  return inter / max((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter, 1e-6)


def group(cls: int) -> str | None:
  return 'car' if cls in CARS else 'person' if cls in PEOPLE else None


def union(*passes) -> list[tuple[np.ndarray, str]]:
  """(box, group) of every object any pass found, one per IoU-0.5 cluster of a class group, the earlier pass's box kept."""
  out: list[tuple[np.ndarray, str]] = []
  for dets in passes:
    for d in dets:
      g = group(int(d[5]))
      if g is not None and not any(o[1] == g and iou(o[0], d[:4]) >= 0.5 for o in out):
        out.append((np.asarray(d[:4]).copy(), g))
  return out


def count(passes_by_schedule: dict, full_pass) -> tuple[dict, dict]:
  """Per schedule, the objects found on one frame and those of them the full pass alone missed."""
  base = union(full_pass)
  found, beyond = {}, {}
  for s, passes in passes_by_schedule.items():
    objs = union(full_pass, *passes)
    found[s] = Counter(g for _, g in objs)
    beyond[s] = Counter(g for b, g in objs if not any(o[1] == g and iou(o[0], b) >= 0.5 for o in base))
  return found, beyond


def calibration(rlog: Path) -> list[float]:
  from openpilot.tools.lib.logreader import LogReader
  for m in LogReader(str(rlog)):
    if m.which() in ('liveCalibration', 'extrinsicsCalibration'):
      cal = getattr(m, m.which())
      if len(cal.rpyCalib) == 3 and str(cal.calStatus) == 'calibrated':
        return list(cal.rpyCalib)
  return [0., 0., 0.]


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('segments', nargs='+', type=Path)
  ap.add_argument('--count', type=int, default=30, help='frames per segment')
  ap.add_argument('--step', type=int, default=40, help='frames between the sampled ones (20 fps)')
  ap.add_argument('--out', type=Path)
  args = ap.parse_args()
  from tinygrad import Tensor, TinyJit
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.selfdrive.turnpilot.detector.compile_detector import compile_context
  from openpilot.selfdrive.turnpilot.detector.decoder import decode, decode_far
  from openpilot.selfdrive.turnpilot.detector.graph import FAR_CROP_SIZE, NATIVE_SIZES, crop_at, crop_views, make_detector, make_far_detector
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import FarCrop, native_strips
  from openpilot.selfdrive.turnpilot.detector.replay_route import camerad_nv12, video_frames
  from openpilot.selfdrive.turnpilot.scene.ranging import road_lens
  cam_w, cam_h = 1344, 760
  sizes = NATIVE_SIZES[(cam_w, cam_h)]
  lens = road_lens(DEVICE_CAMERAS[('mici', 'os04c10')].narrow_road)
  t0 = time.perf_counter()
  with compile_context():
    lb, detect = make_detector(cam_w, cam_h)
    full = TinyJit(detect, prune=True)
    graphs = {k: TinyJit(make_far_detector(cam_w, cam_h, 'yolo26n', size), prune=True)
              for k, size in (('far', FAR_CROP_SIZE), ('strip', sizes['strip']), ('half', sizes['full']))}

  def run_crop(graph, frame, crop):
    (lo, ln), (co, cn) = crop_views(crop)
    return decode_far(graph(luma=Tensor(frame[lo:lo + ln]).realize(), chroma=Tensor(frame[co:co + cn]).realize()).numpy(), crop)

  top, bottom = native_strips(cam_w, cam_h, 'full')
  found = {s: Counter() for s in SCHEDULES}
  beyond = {s: Counter() for s in SCHEDULES}
  per_pass = {s: Counter() for s in ('full_pass', 'far', 'strip', 'top', 'bottom')}
  frames = 0
  for seg in args.segments:
    rpy = calibration(seg / 'rlog.zst')
    u, v = lens.project(FarCrop.view_ray(0., rpy))
    far_crop, strip_crop = crop_at(cam_w, cam_h, float(u), float(v)), crop_at(cam_w, cam_h, float(u), float(v), sizes['strip'])
    w, h, nv12 = video_frames(str(seg / 'fcamera.hevc'), 0, args.count, args.step, 'nv12')
    assert (w, h) == (cam_w, cam_h), f"{seg}: {w}x{h} frames, not a comma four's"
    for img in nv12:
      frame = camerad_nv12(img, w, h)
      f = decode(full(input_frame=Tensor(frame).realize()).numpy(), lb)
      p = {'far': run_crop(graphs['far'], frame, far_crop), 'strip': run_crop(graphs['strip'], frame, strip_crop),
           'top': run_crop(graphs['half'], frame, top), 'bottom': run_crop(graphs['half'], frame, bottom)}
      fd, bd = count({'off': (p['far'],), 'strip': (p['strip'],), 'full': (p['top'], p['bottom'])}, f)
      for s in SCHEDULES:
        found[s].update(fd[s])
        beyond[s].update(bd[s])
      for k, dets in (('full_pass', f), *p.items()):
        per_pass[k].update(g for _, g in union(dets))
      frames += 1
    took = time.perf_counter() - t0
    print(f"{seg}: horizon row {v:.0f}, far crop {far_crop.rect}, strip {strip_crop.rect}; {frames} frames, {took:.0f} s", flush=True)
  row = {'frames': frames, 'step': args.step, 'segments': [str(s) for s in args.segments], 'found': {s: dict(c) for s, c in found.items()},
         'beyond_full_pass': {s: dict(c) for s, c in beyond.items()}, 'per_pass': {s: dict(c) for s, c in per_pass.items()}}
  print(json.dumps(row))
  if args.out:
    args.out.write_text(json.dumps(row, indent=1) + '\n')


if __name__ == '__main__':
  main()
