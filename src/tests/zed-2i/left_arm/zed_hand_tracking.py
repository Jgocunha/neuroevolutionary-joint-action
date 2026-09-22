#!/usr/bin/env python3
"""
zed_hand_tracking.py

Track a HAND across the width of a cropped ROI (right->left axis), using a thin
center band line-scan and a robust skin-color mask (YCrCb + HSV).

Outputs per-frame logs with the hand center in pixels and centimeters from the **right** edge.
Also supports optional preview, annotated video, and CSV export.
"""
import cv2, numpy as np, argparse, os, csv
from time import sleep

def parse_roi(s):
    x,y,w,h = [int(v) for v in s.split(",")]
    return (x,y,w,h)

def crop(img, roi):
    x,y,w,h = roi
    H,W = img.shape[:2]
    x = max(0, min(x, W-1)); y = max(0, min(y, H-1))
    w = max(1, min(w, W-x));  h = max(1, min(h, H-y))
    return img[y:y+h, x:x+w]

def moving_avg(x, k):
    if k <= 1: return x
    k = int(k)
    pad = (k-1)//2
    xp = np.pad(x, (pad, k-1-pad), mode='edge')
    kernel = np.ones(k, dtype=np.float32)/k
    return np.convolve(xp, kernel, mode='valid')

def bin_dilate_1d(a, k):
    k = int(k)
    c = np.convolve(a.astype(np.uint8), np.ones(k, dtype=np.uint8), mode='same')
    return c > 0

def bin_erode_1d(a, k):
    k = int(k)
    c = np.convolve(a.astype(np.uint8), np.ones(k, dtype=np.uint8), mode='same')
    return c >= k

def bin_close_1d(a, k):
    return bin_erode_1d(bin_dilate_1d(a, k), k)

def merge_small_gaps_1d(labels_bool, gap_fill=7):
    a = labels_bool.astype(np.uint8)
    n = len(a); i = 0
    out = a.copy()
    while i < n:
        if out[i] == 0:
            i += 1; continue
        j = i+1
        while j < n and out[j] == 1: j += 1
        k = j
        while k < n and out[k] == 0: k += 1
        if 0 < (k - j) <= gap_fill and k < n:
            out[j:k] = 1
            j = k
        i = j
    return out.astype(bool)

def px_to_cm_from_right(x_px: int, roi_width_px: int, roi_width_cm: float, inclusive: bool = True) -> float:
    if roi_width_px <= 1:
        return 0.0
    denom = (roi_width_px - 1) if inclusive else float(roi_width_px)
    return ( (roi_width_px - 1 - x_px) * (roi_width_cm / denom) )

def skin_mask_ycrcb_hsv(bgr, s_min=40, v_min=40):
    ycrcb = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
    Y, Cr, Cb = cv2.split(ycrcb)
    m1 = (Cr >= 135) & (Cr <= 180) & (Cb >= 85) & (Cb <= 135)

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    m2 = (S >= s_min) & (S <= 200) & (V >= v_min)

    mask = (m1 & m2).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    return mask

def hand_center_on_band(roi_bgr, band_h=21, frac_th=0.25, smooth=7,
                        close_w=11, gap_fill=9, s_min=40, v_min=40):
    H,W = roi_bgr.shape[:2]
    y0 = max(0, H//2 - band_h//2)
    band = roi_bgr[y0:y0+band_h, :]

    m = skin_mask_ycrcb_hsv(band, s_min=s_min, v_min=v_min)

    frac = np.sum(m, axis=0) / (band_h * 255.0)
    if smooth > 1:
        frac = moving_avg(frac, smooth)
    W2 = min(W, len(frac))
    frac = frac[:W2]

    cols = frac >= frac_th
    if close_w > 1:
        cols = bin_close_1d(cols, close_w)
    if gap_fill > 0:
        cols = merge_small_gaps_1d(cols, gap_fill=gap_fill)

    runs = []
    i = 0
    while i < W2:
        if not cols[i]:
            i += 1; continue
        j = i+1
        while j < W2 and cols[j]: j += 1
        runs.append((i,j))
        i = j

    if not runs:
        return False, -1, y0, band_h, cols, runs

    xs, xe = max(runs, key=lambda p: p[1]-p[0])
    x_center = int((xs + xe - 1) / 2)
    return True, x_center, y0, band_h, cols, runs

def main():
    ap = argparse.ArgumentParser(description="Track hand center along ROI width using a center line-scan.")
    ap.add_argument("--in", dest="inp", default="zed_RIGHT_slsl_h.avi")
    ap.add_argument("--roi", default="383,380,242,50", help="x,y,w,h for hand crop")
    ap.add_argument("--roi-width-cm", type=float, default=60.0,
                    help="Real-world width of the ROI in cm (right edge = 0 cm)")
    ap.add_argument("--only-changes", action="store_true",
                    help="Print only when center changes by more than --change-th pixels")
    ap.add_argument("--change-th", type=int, default=6, help="Pixel motion threshold for logging")
    ap.add_argument("--show", action="store_true", help="Optional preview window")
    ap.add_argument("--out", default="", help="Optional annotated output video path")
    ap.add_argument("--csv", default="", help="Optional CSV to store per-frame hand center (cm)")
    ap.add_argument("--fps", type=float, default=60.0, help="Fallback FPS for writing")

    ap.add_argument("--band", type=int, default=21, help="Center band height (px)")
    ap.add_argument("--frac-th", type=float, default=0.25, help="Min skin fraction per column")
    ap.add_argument("--smooth", type=int, default=7, help="1D smoothing window across columns")
    ap.add_argument("--close-w", type=int, default=11, help="1D closing window (odd) to fill gaps")
    ap.add_argument("--gap-fill", type=int, default=9, help="Merge 0-gaps <= this many columns")
    ap.add_argument("--s-min", type=int, default=40, help="Min saturation for skin")
    ap.add_argument("--v-min", type=int, default=40, help="Min value/brightness for skin")

    args = ap.parse_args()

    if not os.path.exists(args.inp):
        raise SystemExit(f"Input not found: {args.inp}")

    roi = parse_roi(args.roi)

    cap = cv2.VideoCapture(args.inp)
    if not cap.isOpened():
        raise SystemExit(f"Failed to open: {args.inp}")

    fps_src = cap.get(cv2.CAP_PROP_FPS) or args.fps
    if fps_src != fps_src:
        fps_src = args.fps

    writer = None
    if args.out:
        ok, f0 = cap.read()
        if not ok: raise SystemExit("Could not read first frame.")
        c0 = crop(f0, roi)
        H0, W0 = c0.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*("mp4v" if args.out.lower().endswith(".mp4") else "MJPG"))
        writer = cv2.VideoWriter(args.out, fourcc, fps_src, (W0,H0))
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    csv_writer = None; csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["frame_index", "has_hand", "x_px", "x_cm_from_right"])

    prev_x = None
    frame_idx = 0

    while True:
        ok, frame = cap.read()
        if not ok: break
        frame_idx += 1

        roi_bgr = crop(frame, roi)
        has_hand, x_c, y0, band_h, cols, runs = hand_center_on_band(
            roi_bgr, band_h=args.band, frac_th=args.frac_th, smooth=args.smooth,
            close_w=args.close_w, gap_fill=args.gap_fill, s_min=args.s_min, v_min=args.v_min
        )

        W = roi[2]
        if has_hand:
            x_cm = px_to_cm_from_right(x_c, W, args.roi_width_cm)
            msg = f"frame {frame_idx:05d} | HAND x={x_c}px  ->  {x_cm:.2f} cm from RIGHT"
        else:
            msg = f"frame {frame_idx:05d} | HAND: none"

        log_it = True
        if args.only_changes:
            if prev_x is None:
                log_it = has_hand  # only log when first seen
            else:
                log_it = (has_hand != (prev_x is not None)) or (has_hand and abs(x_c - prev_x) > args.change_th)
        if log_it:
            print(msg, flush=True)
        prev_x = (x_c if has_hand else None)

        if csv_writer is not None:
            if has_hand:
                csv_writer.writerow([frame_idx, 1, x_c, f"{x_cm:.3f}"])
            else:
                csv_writer.writerow([frame_idx, 0, "", ""])

        if args.show or writer is not None:
            vis = roi_bgr.copy()
            cv2.rectangle(vis, (0, y0), (vis.shape[1]-1, y0+band_h-1), (200,200,200), 1)
            if has_hand:
                y_center = y0 + band_h//2
                cv2.circle(vis, (x_c, y_center), 6, (0, 215, 255), -1)
                cv2.putText(vis, f"{x_cm:.1f}cm", (x_c+6, y_center-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(vis, f"{x_cm:.1f}cm", (x_c+6, y_center-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1, cv2.LINE_AA)
            if writer is not None:
                writer.write(vis)
            if args.show:
                cv2.imshow("hand ROI", vis)
                if (cv2.waitKey(1) & 0xFF) in (27, ord('q')): break
        sleep(0.02)  # be nice to the CPU

    cap.release()
    if writer is not None: writer.release()
    if csv_file is not None: csv_file.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
