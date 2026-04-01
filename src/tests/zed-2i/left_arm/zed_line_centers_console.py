#!/usr/bin/env python3
"""
zed_line_centers_console.py

Like zed_line_centers.py, but prints per-frame object centers to the console.
Overlays/windows are optional (use --show if you still want a preview).

Each frame prints:
  frame <idx> | RED:<count> [(x,y), ...] | GREEN:<count> [(x,y), ...]

Extras:
  --only-changes   Print only when the list of centers changes vs previous frame.
  --csv <path>     Also write centers to CSV.

Run:
  python3 zed_line_centers_console.py --in llss_h/zed_RIGHT_llss_h.avi --roi 375,350,300,100
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

def mask_hsv(hsv, ranges):
    m = None
    for h1,h2,s1,s2,v1,v2 in ranges:
        lower = np.array([h1,s1,v1], np.uint8)
        upper = np.array([h2,s2,v2], np.uint8)
        k = cv2.inRange(hsv, lower, upper)
        m = k if m is None else cv2.bitwise_or(m, k)
    return m

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

def merge_small_gaps(labels, gap_fill=7):
    n = len(labels)
    out = labels.copy()
    i = 0
    while i < n:
        if out[i] == 0:
            i += 1; continue
        c = out[i]
        j = i+1
        while j < n and out[j] == c:
            j += 1
        k = j
        while k < n and out[k] == 0:
            k += 1
        if 0 < (k - j) <= gap_fill and k < n and out[k] == c:
            out[j:k] = c
            j = k
        i = j
    return out

def classify_band(roi_bgr, band_h=11, frac_th=0.35, smooth=7, margin=0.05,
                  close_w=9, gap_fill=7, s_min=60, v_min=40,
                  red_ranges=((0,10,80,255,50,255), (170,179,80,255,50,255)),
                  green_ranges=((35,90,40,255,40,255),)):
    H,W = roi_bgr.shape[:2]
    y0 = max(0, H//2 - band_h//2)
    band = roi_bgr[y0:y0+band_h, :]

    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
    sat = hsv[:,:,1]; val = hsv[:,:,2]
    valid = (sat >= s_min) & (val >= v_min)

    m_red   = mask_hsv(hsv, red_ranges).astype(np.uint8)
    m_green = mask_hsv(hsv, green_ranges).astype(np.uint8)
    m_red   = cv2.bitwise_and(m_red,   m_red,   mask=valid.astype(np.uint8)*255)
    m_green = cv2.bitwise_and(m_green, m_green, mask=valid.astype(np.uint8)*255)

    valid_counts = np.maximum(np.sum(valid, axis=0).astype(np.float32), 1.0) * 255.0
    r_frac = np.sum(m_red,   axis=0) / valid_counts
    g_frac = np.sum(m_green, axis=0) / valid_counts

    if smooth > 1:
        r_frac = moving_avg(r_frac, smooth)
        g_frac = moving_avg(g_frac, smooth)

    W2 = min(W, len(r_frac))
    r_frac = r_frac[:W2]; g_frac = g_frac[:W2]

    red_cols   = (r_frac >= frac_th) & (r_frac >= g_frac + margin)
    green_cols = (g_frac >= frac_th) & (g_frac >= r_frac + margin)

    if close_w > 1:
        red_cols   = bin_close_1d(red_cols,   close_w)
        green_cols = bin_close_1d(green_cols, close_w)

    conflict = red_cols & green_cols
    if np.any(conflict):
        prefer_red = r_frac >= g_frac
        red_cols[conflict]   = prefer_red[conflict]
        green_cols[conflict] = ~prefer_red[conflict]

    labels = np.zeros(W2, dtype=np.uint8)
    labels[red_cols]   = 1
    labels[green_cols] = 2

    if gap_fill > 0:
        labels = merge_small_gaps(labels, gap_fill=gap_fill)

    return labels, (y0, band_h)

def find_runs(labels, min_run=10):
    runs = []
    n = len(labels)
    i = 0
    while i < n:
        if labels[i] == 0:
            i += 1; continue
        c = labels[i]
        j = i+1
        while j < n and labels[j] == c:
            j += 1
        if (j - i) >= min_run:
            runs.append((c, i, j))
        i = j
    return runs

def centers_from_runs(runs, y0, band_h):
    y_center = y0 + band_h // 2
    centers = []
    for c, xs, xe in runs:
        x_center = int((xs + xe - 1) / 2)
        centers.append((c, x_center, y_center))
    # sort left->right
    centers.sort(key=lambda t: t[1])
    return centers

def centers_to_lr_lists(centers):
    """Split and sort centers left->right. Returns (reds_x, greens_x)."""
    reds   = sorted([x for (col,x,y) in centers if col == 1])
    greens = sorted([x for (col,x,y) in centers if col == 2])
    return reds, greens

def centers_changed(prev_lr, curr_lr, tol_px):
    """Return True if counts differ or any matched center moves > tol_px."""
    if prev_lr is None:
        return True
    prev_r, prev_g = prev_lr
    curr_r, curr_g = curr_lr
    # count change = definite change
    if len(prev_r) != len(curr_r) or len(prev_g) != len(curr_g):
        return True
    # same counts → compare left-to-right pairs
    for a, b in zip(prev_r, curr_r):
        if abs(a - b) > tol_px:
            return True
    for a, b in zip(prev_g, curr_g):
        if abs(a - b) > tol_px:
            return True
    return False

def px_to_cm_from_right(x_px: int, roi_width_px: int, roi_width_cm: float, inclusive: bool = True) -> float:
    """
    Convert a pixel x (0=left) to cm from the RIGHT edge of the ROI.
    If inclusive=True, the rightmost pixel center maps to ~0.0 cm.
    """
    if roi_width_px <= 1:
        return 0.0
    # Use W-1 so leftmost pixel center ≈ roi_width_cm, rightmost ≈ 0.0
    denom = (roi_width_px - 1) if inclusive else float(roi_width_px)
    return ( (roi_width_px - 1 - x_px) * (roi_width_cm / denom) )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", default="zed_RIGHT_slsl_r.avi")
    ap.add_argument("--roi", default="383,400,242,100", help="x,y,w,h")
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--show", action="store_true", help="optional preview")
    ap.add_argument("--out", default="", help="optional annotated video path")
    ap.add_argument("--csv", default="", help="optional CSV path for per-frame centers")
    ap.add_argument("--only_changes", action="store_true", help="print only when the centers list changes")
    ap.add_argument("--change-th", type=int, default=5,
                help="Only log when any center moves by > this many pixels (or count changes)")
    ap.add_argument("--roi_width_cm", type=float, default=60.0,
                help="Real-world width of the ROI in centimeters (right edge = 0 cm)")
    
    
    # robustness & band params
    ap.add_argument("--band", type=int, default=11)
    ap.add_argument("--frac-th", type=float, default=0.35)
    ap.add_argument("--smooth", type=int, default=7)
    ap.add_argument("--min-run", type=int, default=10)
    ap.add_argument("--margin", type=float, default=0.05)
    ap.add_argument("--close-w", type=int, default=9)
    ap.add_argument("--gap-fill", type=int, default=7)
    ap.add_argument("--s-min", type=int, default=60)
    ap.add_argument("--v-min", type=int, default=40)

    # HSV (OpenCV H in 0..179)
    ap.add_argument("--red1",  default="0,10,80,255,50,255")
    ap.add_argument("--red2",  default="170,179,80,255,50,255")
    ap.add_argument("--green", default="35,90,40,255,40,255")
    args = ap.parse_args()

    if not os.path.exists(args.inp):
        raise SystemExit(f"Input not found: {args.inp}")

    roi = parse_roi(args.roi)
    parse = lambda s: tuple(int(v) for v in s.split(","))
    red_ranges = [parse(args.red1), parse(args.red2)]
    green_ranges = [parse(args.green)]

    cap = cv2.VideoCapture(args.inp)
    if not cap.isOpened():
        raise SystemExit(f"Failed to open: {args.inp}")

    fps_src = cap.get(cv2.CAP_PROP_FPS) or args.fps
    if fps_src != fps_src:
        fps_src = args.fps

    # Optional writer if --out
    writer = None
    if args.out:
        ok, f0 = cap.read()
        if not ok: raise SystemExit("Could not read first frame.")
        c0 = crop(f0, roi)
        H0, W0 = c0.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*("mp4v" if args.out.lower().endswith(".mp4") else "MJPG"))
        writer = cv2.VideoWriter(args.out, fourcc, fps_src, (W0,H0))
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    # Optional CSV
    csv_writer = None; csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["frame_index", "type", "x", "y"])  # type: R/G

    prev_centers_repr = None
    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok: break
        frame_idx += 1
        c = crop(frame, roi)

        labels, (y0, band_h) = classify_band(
            c, band_h=args.band, frac_th=args.frac_th, smooth=args.smooth,
            margin=args.margin, close_w=args.close_w, gap_fill=args.gap_fill,
            s_min=args.s_min, v_min=args.v_min,
            red_ranges=red_ranges, green_ranges=green_ranges
        )
        runs = find_runs(labels, min_run=args.min_run)
        centers = centers_from_runs(runs, y0, band_h)

        # Assemble printable summary in *centimeters from right edge*
        roi_w = roi[2]
        reds_px   = [(x,y) for (col,x,y) in centers if col==1]
        greens_px = [(x,y) for (col,x,y) in centers if col==2]

        reds_cm   = [ round(px_to_cm_from_right(x, roi_w, args.roi_width_cm), 2) for (x,_) in reds_px ]
        greens_cm = [ round(px_to_cm_from_right(x, roi_w, args.roi_width_cm), 2) for (x,_) in greens_px ]

        msg = (f"frame {frame_idx:05d} | "
            f"RED:{len(reds_cm)} {reds_cm} cm | "
            f"GREEN:{len(greens_cm)} {greens_cm} cm")

        # Tolerance-aware change detection (still in pixels)
        curr_lr = centers_to_lr_lists(centers)  # ([red_x...], [green_x...])

        if args.only_changes:
            if centers_changed(prev_centers_repr, curr_lr, args.change_th):
                print(msg, flush=True)
                prev_centers_repr = curr_lr
        else:
            print(msg, flush=True)
            prev_centers_repr = curr_lr  # keep tracking anyway

        # CSV
        if csv_writer is not None:
            for col, x, y in centers:
                csv_writer.writerow([frame_idx, "R" if col==1 else "G", x, y])

        # Optional show / write
        if args.show or writer is not None:
            # For preview, draw small markers but keep console as the primary output
            vis = c.copy()
            y_center = y0 + band_h // 2
            for col, x, y in centers:
                color = (0,0,255) if col==1 else (0,255,0)
                cv2.circle(vis, (x, y_center), 5, color, -1)
            if writer is not None:
                writer.write(vis)
            if args.show:
                cv2.imshow("preview", vis)
                if (cv2.waitKey(1) & 0xFF) in (27, ord('q')): break
        
        sleep(0.01)  # be nice to the CPU

    cap.release()
    if writer is not None: writer.release()
    if csv_file is not None: csv_file.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
