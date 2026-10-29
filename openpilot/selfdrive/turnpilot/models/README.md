# TurnPilot models

YOLO26 object detectors from Ultralytics, exported to ONNX for the detector in `../detector/`.

The weights and code are AGPL-3.0. Models trained with Ultralytics code are AGPL-3.0 too, so nothing that uses these files can go upstream.

| File | Source weights | Weights sha256 | ONNX size | ONNX sha256 |
| --- | --- | --- | --- | --- |
| `yolo26n.onnx` | https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt | `9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef` | 9.8 MB | `65b2e24edea581f11f74b0e598904dfe343356809c3ce6f3f90daeb16e4c4e4f` |
| `yolo26s.onnx` | https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26s.pt | `646f8bc3fe0a656803d95c294f7852321748cb29d13466a1af8862e2db384a1b` | 38.1 MB | `a7679e007e7eccd6c61ad6cbe579cfc84fd893690e75866bde5c4af4fc6ce102` |

Both files were exported on 2026-10-26 with ultralytics 8.4.190, torch 2.14.1 and onnx 1.23.2 by `export_yolo26.py`:

- opset 17, `simplify=False`
- `nms=False`, which keeps YOLO26's NMS-free head. `output0` is `(1, 300, 6)`: x0, y0, x1, y1 in input pixels, then confidence and the COCO class id.
- `dynamic=True`, so one file serves every camera. `compile_detector.py` fixes the input size for each camera: 640x384 for a 1344x760 frame and 640x416 for a 1928x1208 frame. Through tinygrad, the outputs match onnxruntime on fixed-size exports to within 1.3e-4.
- The input is `images`, `(1, 3, H, W)` float32 RGB in the range 0 to 1.

The files are plain git blobs, not LFS, because `.gitattributes` exempts `openpilot/selfdrive/turnpilot/**`. upstream's `scripts/lint/lint.sh` limits non-LFS files under `openpilot/` to 120 KB, so these files fail its `check_added_large_files` step, as any detector would. Both are well under GitHub's 50 MB warning.

The compiled detectors (`detector_{w}x{h}_tinygrad.pkl` and `big_detector_{w}x{h}_tinygrad.pkl` for the chestnut) are built here by `../detector/compile_detector.py`, and so are the far crops (`far_detector_{w}x{h}_tinygrad.pkl`, `big_` on the chestnut, with `--far`) and the stop sweep's side views (`side_detector_{w}x{h}_tinygrad.pkl`, with `--side`). They are gitignored, except `big_detector_1344x760_tinygrad.pkl`, `big_far_detector_1344x760_tinygrad.pkl` and `big_side_detector_1344x760_tinygrad.pkl`: the car needs them and the installer can't build them, so they're plain blobs in git.

| File | Model | Built | Size | sha256 |
| --- | --- | --- | --- | --- |
| `big_detector_1344x760_tinygrad.pkl` | yolo26n, 640x384 input | 2026-10-27 on the Mac with the chestnut, `--retargetable`, tinygrad_repo 02e6981 | 20.5 MB | `f4f17f788e86b2b893472908df94ca5d10eca2e857389a3921c4d0eca82a0f11` |
| `big_far_detector_1344x760_tinygrad.pkl` | yolo26n, the far crop: a 640x384 window of the road frame at native resolution | 2026-10-28 on the Mac with the chestnut, `--far --retargetable`, graph of 5ae4a1b | 20.5 MB | `fb8a799ee96f81f81b7c1b62cbb432e0f4bf822ce98eb8a4b41d2c93644337ed` |
| `big_side_detector_1344x760_tinygrad.pkl` | yolo26n, the side views: a 768x512 level pinhole at yaw 75 deg left or right, gathered from the wide frame through maps that are inputs | 2026-10-28 on the Mac with the chestnut, `--side --retargetable`, graph of 6a8b635 | 26.7 MB | `00bafa9f4c47fb2c12b57746b4e47173f42dc991233bec89e263d524f5e985f0` |

Rebuild it with the chestnut attached and nothing else holding it:

```sh
PYTHONPATH=$PWD:$PWD/tinygrad_repo DEV=USB+AMD:LLVM python -m openpilot.selfdrive.turnpilot.detector.compile_detector --retargetable --sizes 1344x760
```

Rebuild the far crop the same way, adding `--far`, and the side views adding `--side`.

Measured on the chestnut on 2026-10-27, at 1344x760:

| Model | Alone | In modeld beside the big model | pkl |
| --- | --- | --- | --- |
| yolo26n | 5.9 ms | 7.9 ms; big model 25-27 ms; no frame drops even at every frame | 20.5 MB |
| yolo26s | 10.1 ms | 13.3 ms at every 2nd frame, no frame drops | 56.2 MB, over GitHub's 50 MB limit, so not used |

Both run in float32. The chestnut's tensor cores (RDNA4 WMMA) take half, bfloat16, fp8 and int8, not float32, so the compile context with `TC_OPT=2` changes nothing for these models (5.92 ms with and without it); a half-precision export is the way to use them.

The far crop, measured on the chestnut on 2026-10-28 (`gate2_modeld --detector`, 600 frames of fake 1344x760 os04c10 cameras): alone 6.05 ms median (min 5.91; `compile_detector --far`, its self-test against unjitted runs passing before and after the pickle round trip). In modeld beside the big model and the full pass, on the frames between the full pass's runs: 7.89 ms median (p95 8.16, max 9.03) at 10 Hz; the full pass 7.88 ms at 10 Hz; the big model 25.52 ms median (p95 26.72, max 30.00); no frame dropped. Without the far crop on the same day: the big model 26.51 ms, the full pass 8.08 ms. Its own budget in modeld is `FAR_BUDGET_S` (12 ms).

The side views (STOP_SIGNS.md section 1.3, SG3a's go/no-go), measured on the chestnut on 2026-10-28 with nothing else on it, the Mac the host: alone 7.00 ms median (min 6.56; `compile_detector --side`, its self-test of both frames through both views' maps against unjitted runs passing before and after the pickle round trip; on the Mac's METAL 6.03 ms). In modeld (`gate2_modeld --detector --sweep stopSweep --v-ego 0`, 600 frames of fake 1344x760 cameras): every model frame runs a view at 20 Hz, the side views on the odd steps at 5 Hz each, 9.04 and 9.06 ms median (p95 9.35, max 9.64), their sectors' quality 0.29 ms (on the Mac's CPU; the comma four's is unmeasured), the full wide pass and the road camera 7.9 ms at 5 Hz each, the big model 25.12 ms median (p95 26.38, max 27.26), no frame dropped. So an odd step is about 35 ms (37 ms at the maxima) of the 45 ms `STEP_BUDGET_S`: the side views fit, and `SIDE_VIEW_V_MAX` stays 2.0 m/s, since they cost no model frame. A first run of the same (both views' maps uploaded in one step, 27 ms, since split to one view a step) gave 9.28 and 9.30 ms beside a 26.18 ms model (max 27.56). The crossing cycle (`--sweep stopSweepGo`): the full wide pass 7.77 ms at 10 Hz, the side views 9.03 ms at 5 Hz each, the big model 25.12 ms, no frame dropped. At 5 m/s (`--v-ego 5`) the odd steps idle: the forward views 7.93 ms at 5 Hz each, 10 Hz in all, no side view, the big model 25.25 ms. Without a sweep the same day: the full pass and the far crop 7.75 and 7.78 ms at 10 Hz each, the big model 25.09 ms. Uploading a view's maps (on a calibration change) takes 13.3-13.5 ms on an odd step that runs no view. Their own budget in modeld is `SIDE_BUDGET_S` (15 ms).
