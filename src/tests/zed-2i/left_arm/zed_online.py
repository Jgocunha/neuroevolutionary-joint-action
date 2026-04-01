#!/usr/bin/env python3
"""
zed_online.py

Live processing directly from a ZED 2i V4L2 node using OpenCV.
Modes:
  - mode=objects : count RED/GREEN objects using center-band line scan; log positions in cm (right->left)
  - mode=hand    : track a hand using a skin mask + line scan; log center in cm (right->left)
  - mode=both    : run objects + hand simultaneously (they may use different ROIs)

Examples
--------
# Stereo SxS at 2560x720, use RIGHT half
python3 zed_online.py --dev /dev/video4 --size 2560x720 --fps 60 --fmt MJPG --half right \
  --mode both \
  --roi-objects 375,350,300,100 --roi-width-cm-objects 60 \
  --roi-hand    340,300,350,110 --roi-width-cm-hand 60 \
  --only-changes --change-th 6 --show

Dependencies
------------
sudo apt update && sudo apt install -y python3-opencv v4l-utils
"""
import cv2, numpy as np, argparse, os, time, sys

# ---------------------------- device open ----------------------------

CANDIDATE_SIZES = [(2560,720),(1280,720),(1920,1080)]
CANDIDATE_FMTS  = ["MJPG","YUYV"]

def open_camera(dev, size, fps, fmt_prefer=None):
    """Open /dev/video* robustly; return (cap, actual_w, actual_h, fmt)."""
    sizes = [tuple(map(int, size.split("x")))] if size else CANDIDATE_SIZES
    fmts  = CANDIDATE_FMTS[:]
    if fmt_prefer and fmt_prefer in fmts:
        fmts.remove(fmt_prefer); fmts.insert(0, fmt_prefer)

    last_err = "unknown"
    for fmt in fmts:
        for (w,h) in sizes:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                last_err = f"could not open {dev}"
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, float(fps))
            ok, frame = cap.read()
            if ok and frame is not None:
                H, W = frame.shape[:2]
                return cap, W, H, fmt
            cap.release()
            last_err = f"opened {dev} but couldn't grab at {fmt} {w}x{h}@{fps}"
    raise SystemExit(f"Failed to open camera: {last_err}")

# ------------------------- common helpers ---------------------------
def crop_roi(img, roi):
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

def merge_small_gaps(labels, gap_fill=7):
    n = len(labels); out = labels.copy(); i = 0
    while i < n:
        if out[i] == 0: i += 1; continue
        c = out[i]; j = i+1
        while j < n and out[j] == c: j += 1
        k = j
        while k < n and out[k] == 0: k += 1
        if 0 < (k - j) <= gap_fill and k < n and out[k] == c:
            out[j:k] = c; j = k
        i = j
    return out

def px_to_cm_from_right(x_px, roi_width_px, roi_width_cm, inclusive=True):
    if roi_width_px <= 1: return 0.0
    denom = (roi_width_px - 1) if inclusive else float(roi_width_px)
    return ( (roi_width_px - 1 - x_px) * (roi_width_cm / denom) )

# ------------------------ objects: line-scan ------------------------
def mask_hsv(hsv, ranges):
    m = None
    for h1,h2,s1,s2,v1,v2 in ranges:
        lower = np.array([h1,s1,v1], np.uint8)
        upper = np.array([h2,s2,v2], np.uint8)
        k = cv2.inRange(hsv, lower, upper)
        m = k if m is None else cv2.bitwise_or(m, k)
    return m

def classify_band_objects(roi_bgr, band_h, frac_th, smooth, margin, close_w, gap_fill,
                          red_ranges, green_ranges, s_min, v_min):
    H,W = roi_bgr.shape[:2]
    y0 = max(0, H//2 - band_h//2)
    band = roi_bgr[y0:y0+band_h, :]

    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
    S = hsv[:,:,1]; V = hsv[:,:,2]
    valid = (S >= s_min) & (V >= v_min)

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

    # runs
    runs = []
    i=0
    while i<W2:
        if labels[i]==0: i+=1; continue
        c=labels[i]; j=i+1
        while j<W2 and labels[j]==c: j+=1
        runs.append((c,i,j)); i=j

    # centers
    centers = []
    for c,xs,xe in runs:
        xc = int((xs+xe-1)/2)
        centers.append((c,xc,y0+band_h//2))
    centers.sort(key=lambda t:t[1])  # left->right
    return labels, (y0,band_h), runs, centers

# ------------------------ hand: line-scan --------------------------
def skin_mask_ycrcb_hsv(bgr, s_min=40, v_min=40):
    ycrcb = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
    _Y, Cr, Cb = cv2.split(ycrcb)
    m1 = (Cr >= 135) & (Cr <= 180) & (Cb >= 85) & (Cb <= 135)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    _H, S, V = cv2.split(hsv)
    m2 = (S >= s_min) & (S <= 200) & (V >= v_min)
    mask = (m1 & m2).astype(np.uint8)*255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,k, iterations=2)
    return mask

def hand_center_on_band(roi_bgr, band_h, frac_th, smooth, close_w, gap_fill, s_min, v_min):
    H,W = roi_bgr.shape[:2]
    y0 = max(0, H//2 - band_h//2)
    band = roi_bgr[y0:y0+band_h, :]

    m = skin_mask_ycrcb_hsv(band, s_min=s_min, v_min=v_min)
    frac = np.sum(m, axis=0) / (band_h*255.0)
    if smooth>1: frac = moving_avg(frac, smooth)
    W2=min(W,len(frac)); frac=frac[:W2]
    cols = frac>=frac_th
    if close_w>1: cols=bin_close_1d(cols, close_w)

    # gather runs
    runs=[]; i=0
    while i<W2:
        if not cols[i]: i+=1; continue
        j=i+1
        while j<W2 and cols[j]: j+=1
        runs.append((i,j)); i=j
    if not runs: return False, -1, y0, band_h

    xs,xe = max(runs,key=lambda p:p[1]-p[0])
    xc = int((xs+xe-1)/2)
    return True, xc, y0, band_h

# ------------------------------- main ------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/video4", help="V4L2 device path")
    ap.add_argument("--size", default="2560x720", help="Request WxH (e.g., 2560x720)")
    ap.add_argument("--fps", type=int, default=60, help="Requested FPS")
    ap.add_argument("--fmt", default="MJPG", help="Capture FOURCC (MJPG or YUYV)")
    ap.add_argument("--half", choices=["none","left","right"], default="right",
                    help="If SxS, which half to use (right recommended for RIGHT eye)")
    ap.add_argument("--mode", choices=["objects","hand","both"], default="objects")

    # ROI parameters
    ap.add_argument("--roi", default="375,350,300,100", help="Fallback x,y,w,h if specific ROI not provided")
    ap.add_argument("--roi-objects", default="", help="x,y,w,h for objects (overrides --roi)")
    ap.add_argument("--roi-hand",    default="", help="x,y,w,h for hand (overrides --roi)")
    ap.add_argument("--roi-width-cm", type=float, default=60.0, help="Default real ROI width in cm (for cm output, right edge = 0)")
    ap.add_argument("--roi-width-cm-objects", type=float, default=None, help="Real width cm for objects ROI (defaults to --roi-width-cm)")
    ap.add_argument("--roi-width-cm-hand",    type=float, default=None, help="Real width cm for hand ROI (defaults to --roi-width-cm)")

    # Change detection and preview
    ap.add_argument("--only-changes", action="store_true", help="Log only when values move by > --change-th px or counts change")
    ap.add_argument("--change-th", type=int, default=5, help="Pixel threshold for change detection")
    ap.add_argument("--show", action="store_true", help="Preview window")
    args = ap.parse_args()

    # open camera
    cap, W, H, fmt_used = open_camera(args.dev, args.size, args.fps, fmt_prefer=args.fmt)
    print(f"Opened {args.dev} @ {W}x{H} {fmt_used} (requested {args.size}@{args.fps})")

    # select half if needed
    half = args.half

    # parse ROIs
    def parse_roi(s): return tuple(int(v) for v in s.split(","))
    roi_fallback = parse_roi(args.roi)
    roi_obj = parse_roi(args.roi_objects) if args.roi_objects else roi_fallback
    roi_hand= parse_roi(args.roi_hand)    if args.roi_hand    else roi_fallback

    # real widths
    rwc_default = args.roi_width_cm
    rwc_obj  = args.roi_width_cm_objects if args.roi_width_cm_objects is not None else rwc_default
    rwc_hand = args.roi_width_cm_hand    if args.roi_width_cm_hand    is not None else rwc_default

    # precompute params per mode
    # objects
    band_h_o=11; frac_th_o=0.35; smooth_o=7; margin_o=0.05; close_w_o=9; gap_fill_o=7; s_min_o=60; v_min_o=40
    red_ranges=[(0,10,80,255,50,255),(170,179,80,255,50,255)]
    green_ranges=[(35,90,40,255,40,255)]
    prev_repr=None
    # hand
    band_h_h=21; frac_th_h=0.25; smooth_h=7; close_w_h=11; gap_fill_h=9; s_min_h=40; v_min_h=40
    prev_x=None

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed; exiting.", file=sys.stderr); break

        # SxS split if requested
        if half != "none":
            mid = W//2
            frame = frame[:, :mid] if half=="left" else frame[:, mid:]

        # --- OBJECTS ---
        did_obj = False; obj_msg = None; vis_obj = None
        red_xs = green_xs = ()
        if args.mode in ("objects","both"):
            roi_frame_o = crop_roi(frame, roi_obj)
            labels, (y0o,bho), runs_o, centers_o = classify_band_objects(
                roi_frame_o, band_h_o, frac_th_o, smooth_o, margin_o, close_w_o, gap_fill_o,
                red_ranges, green_ranges, s_min_o, v_min_o
            )
            reds_cm = [ round(px_to_cm_from_right(x, roi_obj[2], rwc_obj), 2) for (c,x,_) in centers_o if c==1 ]
            greens_cm = [ round(px_to_cm_from_right(x, roi_obj[2], rwc_obj), 2) for (c,x,_) in centers_o if c==2 ]

            red_xs   = tuple(sorted([x for (c,x,_) in centers_o if c==1]))
            green_xs = tuple(sorted([x for (c,x,_) in centers_o if c==2]))
            curr_repr = (red_xs, green_xs)

            changed_o = False
            if prev_repr is None: changed_o = True
            else:
                pr, pg = prev_repr
                if len(pr)!=len(red_xs) or len(pg)!=len(green_xs):
                    changed_o = True
                else:
                    for a,b in zip(pr, red_xs):
                        if abs(a-b) > args.change_th: changed_o=True; break
                    if not changed_o:
                        for a,b in zip(pg, green_xs):
                            if abs(a-b) > args.change_th: changed_o=True; break

            obj_msg = f"OBJ  RED:{len(reds_cm)} {reds_cm} cm | GREEN:{len(greens_cm)} {greens_cm} cm"
            if not args.only_changes or changed_o:
                print(obj_msg, flush=True)
                prev_repr = curr_repr
            did_obj = True

            if args.show:
                vis_obj = roi_frame_o.copy()
                cv2.rectangle(vis_obj, (0, y0o), (vis_obj.shape[1]-1, y0o+bho-1), (200,200,200), 1)
                for (c,x,y) in centers_o:
                    color = (0,0,255) if c==1 else (0,255,0)
                    cv2.circle(vis_obj, (x, y), 5, color, -1)
                cv2.putText(vis_obj, "OBJECTS", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(vis_obj, "OBJECTS", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 1, cv2.LINE_AA)

        # --- HAND ---
        did_hand = False; hand_msg = None; vis_hand = None
        if args.mode in ("hand","both"):
            roi_frame_h = crop_roi(frame, roi_hand)
            has_hand, xh, y0h, bhh = hand_center_on_band(
                roi_frame_h, band_h_h, frac_th_h, smooth_h, close_w_h, gap_fill_h, s_min_h, v_min_h
            )
            changed_h = False
            if prev_x is None and has_hand: changed_h = True
            elif (prev_x is not None) and not has_hand: changed_h = True
            elif has_hand and prev_x is not None and abs(xh - prev_x) > args.change_th: changed_h = True

            if has_hand:
                x_cm = px_to_cm_from_right(xh, roi_hand[2], rwc_hand)
                hand_msg = f"HAND {x_cm:.2f} cm (x={xh}px)"
                new_prev_x = xh
            else:
                hand_msg = "HAND none"
                new_prev_x = None

            if not args.only_changes or changed_h:
                print(hand_msg, flush=True)
                prev_x = new_prev_x

            did_hand = True

            if args.show:
                vis_hand = roi_frame_h.copy()
                cv2.rectangle(vis_hand, (0, y0h), (vis_hand.shape[1]-1, y0h+bhh-1), (200,200,200), 1)
                if has_hand:
                    cv2.circle(vis_hand, (xh, y0h+bhh//2), 6, (0,215,255), -1)
                cv2.putText(vis_hand, "HAND", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(vis_hand, "HAND", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 1, cv2.LINE_AA)

        # combined preview
        if args.show:
            if args.mode == "objects":
                cv2.imshow("ZED Online", vis_obj)
            elif args.mode == "hand":
                cv2.imshow("ZED Online", vis_hand)
            else:  # both
                # safely compose
                if vis_obj is None:
                    vis = vis_hand
                elif vis_hand is None:
                    vis = vis_obj
                else:
                    # match heights for hconcat
                    h1, h2 = vis_obj.shape[0], vis_hand.shape[0]
                    if h1 != h2:
                        # resize vis_hand to h1
                        scale = h1 / h2
                        vis_hand = cv2.resize(vis_hand, (int(vis_hand.shape[1]*scale), h1))
                    vis = cv2.hconcat([vis_obj, vis_hand])
                cv2.imshow("ZED Online", vis)

            if (cv2.waitKey(1) & 0xFF) in (27, ord('q')):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
