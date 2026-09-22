# ZED 2i – Ubuntu 22.04 (no CUDA) – Capture & Line-Scan Vision

This repo in this folder src\tests\zed-2i contains small, dependency-light Python tools for working with a **ZED 2i** over **V4L2** using OpenCV only. We capture, crop, and analyze a thin horizontal **line-scan** through a Region of Interest (ROI) to:

* count **RED**/**GREEN** objects and report their positions,
* track a **hand** across the same axis,
* run both **objects + hand** online from the live camera.

Everything runs CPU-only on Ubuntu 22.04.

---

## 0) Requirements

```bash
sudo apt update
sudo apt install -y python3-opencv v4l-utils
```

If OpenCV was compiled **without Qt**, avoid functions like `cv2.displayOverlay` (we removed them once we hit the error).

---

## 1) Devices & ZED nodes (V4L2)

Use `v4l2-ctl --list-devices` to find ZED nodes. Example:

```
ZED 2i: ZED 2i (usb-0000:07:00.4-2):
  /dev/video4
  /dev/video5
```

* `/dev/video4` – stereo **side-by-side** stream (use `--half right` to select the RIGHT eye).
* `/dev/video5` – (on some configurations) mono node may not expose formats; we ignore it and stick to `/dev/video4`.

---

## 2) Scripts Overview

* `zed_preview.py`
  Quick preview from a device or video path (no overlays that require Qt).
* `zed_record.py`
  Record to disk. Press **b** to begin, **q** to stop (MJPG/XVID suggested).
* `zed_line_count.py` / `zed_line_count_robust.py`
  Count RED/GREEN by line-scan on **videos**. The robust version ignores low-saturation/low-value pixels and adds 1-D morphological cleanup and hysteresis.
* `zed_line_centers.py`
  For **videos**: count + compute the **center x** for each object; optional CSV; overlay.
* `zed_line_centers_console.py`
  Console logs only (optionally **only when things change** with a pixel threshold). Can output **cm** instead of pixels.
* `zed_hand_tracking.py`
  For **videos**: line-scan **hand** tracker using a robust **skin mask (YCrCb+HSV)**; logs px + cm; optional CSV/overlay.
* `zed_online.py`
  **Live** from `/dev/video*` with three modes: `objects`, `hand`, or `both`. Handles SxS splitting, FPS/FOURCC selection, per-ROI cm conversion, and change-filtered logging.

---

## 3) ROI & Coordinate Conventions

* ROI format: `x,y,w,h` in **pixels**, relative to the selected frame/half.
* **x=0** is the **left** of the ROI; conversion to **cm** is reported **from the RIGHT edge** (right→left).
* We use:
  `cm = ( (W - 1 - x_px) * roi_width_cm / (W - 1) )`
  so the rightmost pixel center ≈ `0.0 cm`, leftmost ≈ `roi_width_cm`.

**Correct ROIs you’re using live:**

* **Objects:** `--roi-objects 383,400,242,100` with `--roi-width-cm-objects 60`
* **Hand:** `--roi-hand 383,380,242,50` with `--roi-width-cm-hand 60`

---

## 4) Offline (videos)

### Count + centers (and overlay)

```bash
python3 zed_line_centers.py --in sls_r/zed_RIGHT_sls_r.avi \
  --roi 383,400,242,100 --show --out annotated.mp4 --csv centers.csv
```

### Console logging only (with change threshold, cm output)

```bash
python3 zed_line_centers_console.py --in zed_RIGHT_sls_r.avi \
  --roi 383,400,242,100 --roi-width-cm 60 \
  --only-changes --change-th 5
```

### Hand tracker (videos)

```bash
python3 zed_hand_tracking.py --in zed_RIGHT_sls_r.avi \
  --roi 383,380,242,50 --roi-width-cm 60 \
  --only-changes --change-th 6 --show --csv hand_centers.csv
```

---

## 5) Online (live camera)

Open camera, pick SxS half, and run pipelines in real time.

### Live objects

```bash
python3 zed_online.py --dev /dev/video4 --size 2560x720 --fps 60 --fmt MJPG \
  --half right --mode objects \
  --roi-objects 607,400,235,100 --roi-width-cm-objects 60 \
  --only-changes --change-th 5 --show
```

### Live hand

```bash
python3 zed_online.py --dev /dev/video4 --size 2560x720 --fps 60 --fmt MJPG \
  --half right --mode hand \
  --roi-hand 607,400,235,100 --roi-width-cm-hand 60 \
  --only-changes --change-th 6 --show
```

### Live both (objects + hand)

```bash
python3 zed_online.py --dev /dev/video4 --size 2560x720 --fps 60 --fmt MJPG \
  --half right --mode both \
  --roi-objects 607,400,235,100 --roi-width-cm-objects 60 \
  --roi-hand    607,350,215,110  --roi-width-cm-hand 60 \
  --only-changes --change-th 6 --show
```

**If frames fail to open:** try `--fmt YUYV` or `--size 1280x720`.

---

## 6) How the line-scan works (short version)

* We crop to an ROI and extract a thin **horizontal band** through its center.
* For **objects**:

  * Convert to HSV and build color masks:

    * RED: `H∈[0,10]∪[170,179], S≥80, V≥50`
    * GREEN: `H∈[35,90], S≥40, V≥40`
  * Ignore pixels with low saturation/value (white labels / dark background).
  * Compute **per-column** fraction of red/green in the band; smooth along x.
  * Apply thresholds/hysteresis + **1-D morphological closing** to form contiguous segments.
  * Count segments (runs) and compute their **centers**.
* For the **hand**:

  * Robust **skin** mask = YCrCb window **AND** HSV constraints; morphological clean-up.
  * Same per-column fraction → threshold → closing → largest segment → **center**.

**Change-filtered logging**: we only print when counts change or any center shifts by more than `--change-th` pixels.

---

## 7) Tunables (common flags)

* `--band` *(objects: 11; hand: 21–31)* – band height in px (larger = smoother, slower).
* `--smooth` *(\~5–9)* – 1-D moving average along x.
* `--frac-th` – minimum per-column color/skin fraction to accept.
* `--close-w` *(odd)* – 1-D closing window to bridge tiny gaps.
* `--gap-fill` – merge small “none” gaps between same-color columns.
* `--s-min`, `--v-min` – ignore low-saturation / low-value pixels.
* `--only-changes`, `--change-th` – console logs only when there’s a meaningful change.

---

## 8) Troubleshooting & Tips

* **No preview?** The minimal builds sometimes lack Qt; we use basic `imshow`. Avoid `displayOverlay`.
* **Frame drops at 60 FPS?** Try `--fmt MJPG` (lower USB bandwidth) or downscale: `--size 1280x720`.
* **Wrong half?** ZED delivers side-by-side at `/dev/video4`; use `--half right` to analyze the right image.
* **Color drift** (lighting changes): slightly widen HSV ranges and/or bump `--smooth` and `--band`.
* **Whites on objects** causing flicker: the robust pipeline ignores **low-S/V** and merges tiny gaps.

---

## 9) Example console outputs

```
OBJ  RED:2 [12.48, 38.22] cm | GREEN:1 [55.73] cm
HAND 24.15 cm (x=118px)
```

Positions are **centimeters from the RIGHT edge** of each ROI.

---

## 10) Notes from the journey

* Identified ZED devices with `v4l2-ctl`; selected the **RIGHT** view via `--half right`.
* Avoided Qt-dependent UI; built portable OpenCV windows only.
* Designed **line-scan** detectors to be robust to rails’ white labels:

  * low-sat/value suppression,
  * 1-D closing + gap merging,
  * margin/hysteresis between colors,
  * change-aware console logging to reduce noise.
* Added pixel→cm mapping with **right→left** orientation and your **60 cm / 242 px** calibration.
* Built a **live** tool with modes: `objects`, `hand`, `both` (separate ROIs, separate cm scaling).


