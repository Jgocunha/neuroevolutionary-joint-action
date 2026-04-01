#!/usr/bin/env python3
"""
zed_line_count.py

Counts RED and GREEN objects by analyzing a 1D band across the CENTER of a given ROI.
Rather than detecting elongated rectangles, it classifies each column in a thin
horizontal band as red/green/none and counts runs (segments) along the width.

Usage:
  python3 zed_line_count.py --in llss_h/zed_RIGHT_llss_h.avi --roi 375,350,300,100 --show
  python3 zed_line_count.py --in sls_r/zed_RIGHT_sls_r.avi --roi 375,350,300,100 --out annotated.mp4 --show
"""
import cv2, numpy as np, argparse, os
from collections import deque, Counter

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

def run_counts(labels, min_run=8):
    """labels: 0 none, 1 red, 2 green. Returns (#red, #green) and run segments."""
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
        length = j - i
        if length >= min_run:
            runs.append((c, i, j))  # inclusive start, exclusive end
        i = j
    r = sum(1 for c,_,_ in runs if c == 1)
    g = sum(1 for c,_,_ in runs if c == 2)
    return r, g, runs

def classify_band(roi_bgr, band_h=9, frac_th=0.4, smooth=5,
                  red_ranges=((0,10,80,255,50,255), (170,179,80,255,50,255)),
                  green_ranges=((40,85,50,255,40,255),)):
    H,W = roi_bgr.shape[:2]
    y0 = max(0, H//2 - band_h//2)
    band = roi_bgr[y0:y0+band_h, :]

    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
    m_red   = mask_hsv(hsv, red_ranges)
    m_green = mask_hsv(hsv, green_ranges)

    # per-column fractions in [0..1]
    col_sum = float(band_h) * 255.0
    r_frac = np.sum(m_red,   axis=0) / col_sum
    g_frac = np.sum(m_green, axis=0) / col_sum

    # smooth a bit to reduce noise
    if smooth > 1:
        r_frac = moving_avg(r_frac, smooth)
        g_frac = moving_avg(g_frac, smooth)

    # classify: 1 red, 2 green, 0 none
    labels = np.zeros(W, dtype=np.uint8)
    W2 = min(W, len(r_frac))
    labels = labels[:W2]
    r_frac = r_frac[:W2]
    g_frac = g_frac[:W2]

    for x in range(W2):
        r, g = r_frac[x], g_frac[x]
        if r >= frac_th and r >= g:
            labels[x] = 1
        elif g >= frac_th and g > r:
            labels[x] = 2
        else:
            labels[x] = 0

    return labels, (y0, band_h)

def make_overlay(roi_bgr, labels, band_info, runs):
    y0, bh = band_info
    H,W = roi_bgr.shape[:2]
    vis = roi_bgr.copy()
    # draw band
    cv2.rectangle(vis, (0, y0), (W-1, y0+bh-1), (200,200,200), 1)
    # color strip at top showing labels
    strip = np.zeros((8, W, 3), dtype=np.uint8)
    strip[0:, labels==1] = (0,0,255)   # red
    strip[0:, labels==2] = (0,255,0)   # green
    vis[0:8, :W] = cv2.addWeighted(vis[0:8, :W], 0.3, strip, 0.7, 0)

    # mark accepted runs
    for c, xs, xe in runs:
        color = (0,0,255) if c==1 else (0,255,0)
        cv2.rectangle(vis, (xs, 0), (xe-1, H-1), color, 2)
    return vis

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", default="zed_RIGHT_sls_r.avi")
    ap.add_argument("--roi", default="375,350,300,100", help="x,y,w,h")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--out", default="", help="optional annotated video path")
    ap.add_argument("--fps", type=float, default=60.0)

    # band/seg params
    ap.add_argument("--band", type=int, default=30, help="center band height in pixels")
    ap.add_argument("--frac-th", type=float, default=0.1, help="min fraction per column to classify")
    ap.add_argument("--smooth", type=int, default=5, help="1D smoothing window over per-column fractions")
    ap.add_argument("--min-run", type=int, default=8, help="min consecutive columns for a single object")

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

    writer = None
    if args.out:
        ok, f0 = cap.read()
        if not ok: raise SystemExit("Could not read first frame.")
        c0 = crop(f0, roi)
        H0, W0 = c0.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*("mp4v" if args.out.lower().endswith(".mp4") else "MJPG"))
        writer = cv2.VideoWriter(args.out, fourcc, fps_src, (W0,H0))
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    hist = deque(maxlen=30)

    while True:
        ok, frame = cap.read()
        if not ok: break
        c = crop(frame, roi)

        labels, band_info = classify_band(
            c, band_h=args.band, frac_th=args.frac_th, smooth=args.smooth,
            red_ranges=red_ranges, green_ranges=green_ranges
        )
        r_cnt, g_cnt, runs = run_counts(labels, min_run=args.min_run)
        hist.append((r_cnt, g_cnt))

        vis = make_overlay(c, labels, band_info, runs)
        cv2.putText(vis, f"RED:{r_cnt}  GREEN:{g_cnt}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(vis, f"RED:{r_cnt}  GREEN:{g_cnt}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0),     1, cv2.LINE_AA)

        if writer is not None:
            writer.write(vis)

        if args.show:
            cv2.imshow("ROI + line-scan", vis)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')): break

    cap.release()
    if writer is not None: writer.release()
    cv2.destroyAllWindows()

    if hist:
        r_final = Counter([x for x,_ in hist]).most_common(1)[0][0]
        g_final = Counter([y for _,y in hist]).most_common(1)[0][0]
        print(f"FINAL COUNTS  RED={r_final}  GREEN={g_final}")
    else:
        print("No frames processed.")

if __name__ == "__main__":
    main()
