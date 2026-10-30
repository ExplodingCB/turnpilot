#!/usr/bin/env python3
"""Compile the TurnPilot detector into one TinyJit pickle per camera size, the way modeld's warp and driving pkls are built.

  DEV=USB+AMD:LLVM python3 compile_detector.py --retargetable             # chestnut attached: big_detector_{w}x{h}_tinygrad.pkl
  DEV=METAL HCQ_RUNTIME_DEV=CPU:CLANG JIT=2 python3 compile_detector.py   # mac (JIT=2 as in modeld's SConscript)

--retargetable drops the host-side compiled programs, which modeld rebuilds on load (it calls lower_and_compile), so a
pkl compiled on the Mac with the chestnut attached also loads on the comma four.

Run with the repo root and tinygrad_repo on PYTHONPATH. The jit takes one camera's NV12 buffer (input_frame, uint8
[stride * (y_height + uv_height)]) and returns YOLO26's (1, 300, 6). modeld calls it with views into the two frames it
already uploaded, so the self-test does the same: both frames in one device buffer, views alternating, each output
checked against an unjitted run, before and after the pickle round trip.

--far compiles the far crop (graph.py make_far_detector: far_detector_{w}x{h}_tinygrad.pkl, big_ on the chestnut), which
takes two views of a frame, its luma rows and its chroma rows (graph.crop_views), so one pkl serves every aim. Its
self-test runs both frames at two crops each (a corner and the far one), alternating, against unjitted runs.

  DEV=USB+AMD:LLVM python3 compile_detector.py --far --retargetable --sizes 1344x760

--side compiles the stop sweep's side views (graph.py make_side_detector: side_detector_{w}x{h}_tinygrad.pkl, big_ on the
chestnut), which take the wide frame's view and a side view's maps (sideview.SideMap: luma_idx, chroma_idx, weights) as
inputs, so one pkl serves both views and any calibration. Its self-test runs both frames through both views' maps for the
demo route's calibration, alternating, against unjitted runs, before and after the pickle round trip.

  DEV=USB+AMD:LLVM python3 compile_detector.py --side --retargetable --sizes 1344x760
"""
import argparse
import hashlib
import time
from pathlib import Path

import numpy as np

from openpilot.selfdrive.turnpilot.detector.graph import (DEFAULT_MODEL, FAR_CROP_SIZE, INPUT_SIZES, MODELS_DIR, SIDE_INPUT_SIZE, Crop,
                                                         crop_views, detector_pkl_path, far_detector_pkl_path, make_detector,
                                                         make_far_detector, make_side_detector, onnx_path, side_detector_pkl_path,
                                                         side_input_specs)
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info


def frame_views(frames: np.ndarray):
  """Both frames in one device buffer, as modeld packs them, and a uint8 view of each."""
  from tinygrad import Tensor, dtypes
  from tinygrad.uop.ops import UOp
  packed = np.zeros(256 + frames.size, dtype=np.uint8)  # offset like modeld's packed npy inputs
  packed[256:] = frames.reshape(-1)
  buf = Tensor(packed).realize()._buffer()
  size = frames.shape[1]
  return [Tensor(UOp.from_buffer(buf.view(size, 256 + i * size).ensure_allocated(), dtypes.uint8)) for i in range(len(frames))]


def compile_context():
  """The context modeld's own pkls are compiled in (tinygrad examples/openpilot/helpers.benchmark). Kernels are
  optimized when the jit captures them, so outside it the detector misses the AMD tensor core opts."""
  from tinygrad import Context, Device
  return Context(OPENPILOT_HACKS=1, **{'AMD': {'TC_OPT': 2, 'TC_MIN_GLOBALS': 32}}.get(Device.DEFAULT, {}))


def compile_one(cam_w: int, cam_h: int, model: str, out: Path, runs: int, retargetable: bool) -> None:
  with compile_context():
    _compile_one(cam_w, cam_h, model, out, runs, retargetable)


def _compile_one(cam_w: int, cam_h: int, model: str, out: Path, runs: int, retargetable: bool) -> None:
  from tinygrad import Device, Tensor, TinyJit
  from tinygrad.engine.realize import lower_and_compile
  from tinygrad_repo.examples.openpilot.helpers import dump_pickle, load_pickle, make_retargetable
  stride, y_height, uv_height, _ = get_nv12_info(cam_w, cam_h)
  frame_size = stride * (y_height + uv_height)
  lb, detect = make_detector(cam_w, cam_h, model)

  rng = np.random.default_rng(42)
  frames = rng.integers(0, 256, (2, frame_size), dtype=np.uint8)
  expected = [detect(Tensor(f).realize()).numpy() for f in frames]
  assert not np.array_equal(expected[0], expected[1]), "detector output doesn't depend on the frame"

  run = TinyJit(detect, prune=True)
  views = frame_views(frames)

  def check(fn, label):
    for i in (0, 1, 0, 1):
      np.testing.assert_allclose(fn(input_frame=views[i]).numpy(), expected[i], rtol=1e-4, atol=1e-3, err_msg=f'{label} frame {i}')

  check(run, 'jit')
  times = []
  for _ in range(runs):
    Device.default.synchronize()
    st = time.perf_counter()
    run(input_frame=views[len(times) % 2]).realize()
    Device.default.synchronize()
    times.append((time.perf_counter() - st) * 1e3)
  print(f'{model} {cam_w}x{cam_h} -> {lb.in_w}x{lb.in_h} on {Device.DEFAULT}: min {min(times):.2f} ms, median {np.median(times):.2f} ms')

  if retargetable:
    make_retargetable(run)
  specs = {'input_frame': ((frame_size,), np.dtype(np.uint8).str, Device.DEFAULT)}
  metadata = {'model': model, 'onnx_sha256': hashlib.sha256(onnx_path(model).read_bytes()).hexdigest(),
              'camera_size': (cam_w, cam_h), 'input_size': (lb.in_w, lb.in_h)}
  dump_pickle({'metadata': metadata, 'run': run, 'input_specs': specs,
               'output_specs': {'output': ((1, 300, 6), np.dtype(np.float32).str, Device.DEFAULT)}}, out)

  loaded = load_pickle(out, out_of_band=True)
  loaded['run'].captured._linear = lower_and_compile(loaded['run'].captured._linear)
  check(loaded['run'], 'pickle')
  print(f'wrote {out} ({out.stat().st_size / 1e6:.1f} MB)', flush=True)


def compile_far(cam_w: int, cam_h: int, model: str, out: Path, runs: int, retargetable: bool) -> None:
  with compile_context():
    _compile_far(cam_w, cam_h, model, out, runs, retargetable)


def _compile_far(cam_w: int, cam_h: int, model: str, out: Path, runs: int, retargetable: bool) -> None:
  from tinygrad import Device, Tensor, TinyJit, dtypes
  from tinygrad.engine.realize import lower_and_compile
  from tinygrad.uop.ops import UOp
  from tinygrad_repo.examples.openpilot.helpers import dump_pickle, load_pickle, make_retargetable
  stride, y_height, uv_height, _ = get_nv12_info(cam_w, cam_h)
  frame_size = stride * (y_height + uv_height)
  w, h = FAR_CROP_SIZE
  detect = make_far_detector(cam_w, cam_h, model)
  crops = [Crop(cam_w, cam_h, 0, 0), Crop(cam_w, cam_h, (cam_w - w) // 64 * 32, (cam_h - h) // 4 * 2)]

  rng = np.random.default_rng(43)
  frames = rng.integers(0, 256, (2, frame_size), dtype=np.uint8)

  def host_views(f, crop):
    (lo, ln), (co, cn) = crop_views(crop)
    return Tensor(f[lo:lo + ln]).realize(), Tensor(f[co:co + cn]).realize()
  cases = [(i, crop) for i in (0, 1) for crop in crops]
  expected = [detect(*host_views(frames[i], crop)).numpy() for i, crop in cases]
  assert not np.array_equal(expected[0], expected[1]), "far detector output doesn't depend on where the crop is"

  packed = np.zeros(256 + frames.size, dtype=np.uint8)  # both frames in one device buffer, as modeld packs them
  packed[256:] = frames.reshape(-1)
  buf = Tensor(packed).realize()._buffer()

  def device_views(i, crop):
    (lo, ln), (co, cn) = crop_views(crop)
    base = 256 + i * frame_size
    return tuple(Tensor(UOp.from_buffer(buf.view(n, base + o).ensure_allocated(), dtypes.uint8)) for o, n in ((lo, ln), (co, cn)))
  views = [device_views(i, crop) for i, crop in cases]
  run = TinyJit(detect, prune=True)

  def check(fn, label):
    for k in (0, 3, 1, 2, 0, 3, 1, 2):
      np.testing.assert_allclose(fn(luma=views[k][0], chroma=views[k][1]).numpy(), expected[k], rtol=1e-4, atol=1e-3,
                                 err_msg=f'{label} frame {cases[k][0]} crop {cases[k][1].x0},{cases[k][1].y0}')

  check(run, 'jit')
  times = []
  for _ in range(runs):
    Device.default.synchronize()
    st = time.perf_counter()
    luma, chroma = views[len(times) % len(views)]
    run(luma=luma, chroma=chroma).realize()
    Device.default.synchronize()
    times.append((time.perf_counter() - st) * 1e3)
  print(f'far {model} {cam_w}x{cam_h} crop {w}x{h} on {Device.DEFAULT}: min {min(times):.2f} ms, median {np.median(times):.2f} ms')

  if retargetable:
    make_retargetable(run)
  (_, ln), (_, cn) = crop_views(crops[0])
  specs = {'luma': ((ln,), np.dtype(np.uint8).str, Device.DEFAULT), 'chroma': ((cn,), np.dtype(np.uint8).str, Device.DEFAULT)}
  metadata = {'model': model, 'onnx_sha256': hashlib.sha256(onnx_path(model).read_bytes()).hexdigest(), 'kind': 'far',
              'camera_size': (cam_w, cam_h), 'input_size': (w, h)}
  dump_pickle({'metadata': metadata, 'run': run, 'input_specs': specs,
               'output_specs': {'output': ((1, 300, 6), np.dtype(np.float32).str, Device.DEFAULT)}}, out)

  loaded = load_pickle(out, out_of_band=True)
  loaded['run'].captured._linear = lower_and_compile(loaded['run'].captured._linear)
  check(loaded['run'], 'pickle')
  print(f'wrote {out} ({out.stat().st_size / 1e6:.1f} MB)', flush=True)


# the demo route's comma four calibration (5beb9b58bd12b691/0000010a--a51155e496): the side views' self-test maps
SIDE_TEST_RPY, SIDE_TEST_EULER = (-0.0004, 0.16485, 0.004535), (-2.1e-05, 0.00551, -0.00276)


def compile_side(cam_w: int, cam_h: int, model: str, out: Path, runs: int, retargetable: bool) -> None:
  with compile_context():
    _compile_side(cam_w, cam_h, model, out, runs, retargetable)


def _compile_side(cam_w: int, cam_h: int, model: str, out: Path, runs: int, retargetable: bool) -> None:
  from tinygrad import Device, Tensor, TinyJit
  from tinygrad.engine.realize import lower_and_compile
  from tinygrad_repo.examples.openpilot.helpers import dump_pickle, load_pickle, make_retargetable
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.selfdrive.turnpilot.detector.sideview import SIDE_VIEWS, compute_geometry
  from openpilot.selfdrive.turnpilot.scene.ranging import Lens, WIDE_FISHEYE_F
  stride, y_height, uv_height, _ = get_nv12_info(cam_w, cam_h)
  frame_size = stride * (y_height + uv_height)
  detect = make_side_detector(cam_w, cam_h, model)
  wide = next((c.wide_road for c in DEVICE_CAMERAS.values() if (c.wide_road.width, c.wide_road.height) == (cam_w, cam_h)), None)
  assert wide is not None and (cam_w, cam_h) in WIDE_FISHEYE_F, f"no wide fisheye of {cam_w}x{cam_h}"
  geom = compute_geometry(Lens(cam_w, cam_h, WIDE_FISHEYE_F[(cam_w, cam_h)], fisheye=True), SIDE_TEST_RPY, SIDE_TEST_EULER, cam_w, cam_h)
  maps = {name: tuple(Tensor(a).realize() for a in (geom.maps[name].luma_idx, geom.maps[name].chroma_idx, geom.maps[name].weights))
          for name in SIDE_VIEWS}

  rng = np.random.default_rng(44)
  frames = rng.integers(0, 256, (2, frame_size), dtype=np.uint8)
  cases = [(i, name) for i in (0, 1) for name in SIDE_VIEWS]
  expected = [detect(Tensor(frames[i]).realize(), *maps[name]).numpy() for i, name in cases]
  assert not np.array_equal(expected[0], expected[1]), "side detector output doesn't depend on the view"

  run = TinyJit(detect, prune=True)
  views = frame_views(frames)

  def check(fn, label):
    for k in (0, 3, 1, 2, 0, 3, 1, 2):
      i, name = cases[k]
      luma, chroma, weights = maps[name]
      np.testing.assert_allclose(fn(input_frame=views[i], luma_idx=luma, chroma_idx=chroma, weights=weights).numpy(), expected[k],
                                 rtol=1e-4, atol=1e-3, err_msg=f'{label} frame {i} {name}')

  check(run, 'jit')
  times = []
  for _ in range(runs):
    i, name = cases[len(times) % len(cases)]
    luma, chroma, weights = maps[name]
    Device.default.synchronize()
    st = time.perf_counter()
    run(input_frame=views[i], luma_idx=luma, chroma_idx=chroma, weights=weights).realize()
    Device.default.synchronize()
    times.append((time.perf_counter() - st) * 1e3)
  w, h = SIDE_INPUT_SIZE
  print(f'side {model} {cam_w}x{cam_h} view {w}x{h} on {Device.DEFAULT}: min {min(times):.2f} ms, median {np.median(times):.2f} ms')

  if retargetable:
    make_retargetable(run)
  metadata = {'model': model, 'onnx_sha256': hashlib.sha256(onnx_path(model).read_bytes()).hexdigest(), 'kind': 'side',
              'camera_size': (cam_w, cam_h), 'input_size': (w, h)}
  dump_pickle({'metadata': metadata, 'run': run, 'input_specs': side_input_specs(cam_w, cam_h, Device.DEFAULT),
               'output_specs': {'output': ((1, 300, 6), np.dtype(np.float32).str, Device.DEFAULT)}}, out)

  loaded = load_pickle(out, out_of_band=True)
  loaded['run'].captured._linear = lower_and_compile(loaded['run'].captured._linear)
  check(loaded['run'], 'pickle')
  print(f'wrote {out} ({out.stat().st_size / 1e6:.1f} MB)', flush=True)


def main():
  from tinygrad import Device
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--model', default=DEFAULT_MODEL, help='ONNX in selfdrive/turnpilot/models: yolo26n or yolo26s')
  ap.add_argument('--sizes', nargs='+', default=[f'{w}x{h}' for w, h in INPUT_SIZES], help='camera frame sizes, WxH')
  ap.add_argument('--out', type=Path, default=MODELS_DIR)
  ap.add_argument('--benchmark-runs', type=int, default=20)
  ap.add_argument('--retargetable', action='store_true', help='rebuild host programs on load (for a pkl built off the device)')
  ap.add_argument('--far', action='store_true', help='the far crop (graph.make_far_detector) instead of the full frame')
  ap.add_argument('--side', action='store_true', help='the side views (graph.make_side_detector) instead of the full frame')
  args = ap.parse_args()
  chestnut = 'AMD' in Device.DEFAULT
  for size in args.sizes:
    w, h = map(int, size.split('x'))
    if args.side:
      compile_side(w, h, args.model, side_detector_pkl_path(w, h, chestnut, args.out), args.benchmark_runs, args.retargetable)
    elif args.far:
      compile_far(w, h, args.model, far_detector_pkl_path(w, h, chestnut, args.out), args.benchmark_runs, args.retargetable)
    else:
      compile_one(w, h, args.model, detector_pkl_path(w, h, chestnut, args.out), args.benchmark_runs, args.retargetable)


if __name__ == '__main__':
  main()
