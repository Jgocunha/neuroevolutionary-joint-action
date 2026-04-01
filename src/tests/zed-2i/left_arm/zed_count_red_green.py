#!/usr/bin/env python3
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

def mask_color_hsv(hsv, ranges):
    m = None
    for h1,h2,s1,s2,v1,v2 in ranges:
        lower = np.array([h1,s1,v1], np.uint8)
        upper = np.array([h2,s2,v2], np.uint8)
        k = cv2.inRange(hsv, lower, upper)
        m = k if m is None else cv2.bitwise_or(m, k)
    return m

def count_objects(bgr, red_ranges, green_ranges,
                  area_min=300, aspect_min=2.0,
                  length_min=50, kernel=5, close_iter=2, open_iter=1):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    m_red   = mask_color_hsv(hsv, red_ranges)
    m_green = mask_color_hsv(hsv, green_ranges)

    # morphology to join label gaps so each rail is one blob
    K = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
    m_red   = cv2.morphologyEx(m_red,   cv2.MORPH_OPEN,  K, iterations=open_iter)
    m_red   = cv2.morphologyEx(m_red,   cv2.MORPH_CLOSE, K, iterations=close_iter)
    m_green = cv2.morphologyEx(m_green, cv2.MORPH_OPEN,  K, iterations=open_iter)
    m_green = cv2.morphologyEx(m_green, cv2.MORPH_CLOSE, K, iterations=close_iter)

    def detect(mask, color):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        keep = []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < area_min:        # too tiny
                continue
            (cx,cy),(w,h),ang = cv2.minAreaRect(c)  # rotated rect
            long_side, short_side = (max(w,h), min(w,h))
            if short_side < 1:      # avoid divide-by-zero
                continue
            aspect = long_side / short_side
            if aspect < aspect_min: # not elongated enough
                continue
            if long_side < length_min:
                continue
            box = cv2.boxPoints(((cx,cy),(w,h),ang)).astype(int)
            cv2.drawContours(bgr, [box], 0, color, 2)
            keep.append(((int(cx),int(cy)), (w,h), ang))
        return keep

    reds   = detect(m_red,   (0,0,255))
    greens = detect(m_green, (0,255,0))
    return len(reds), len(greens), m_red, m_green, bgr

def main():
    ap = argparse.ArgumentParser(description="Count RED vs GREEN rails in a cropped ROI.")
    ap.add_argument("--in", dest="inp", default="llss_h/zed_RIGHT_llss_h.avi")
    ap.add_argument("--out", default="")
    ap.add_argument("--roi", default="375,350,300,100", help="x,y,w,h")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--fps", type=float, default=60.0)

    # thresholds (OpenCV H in [0,179])
    ap.add_argument("--red1",  default="0,10,80,255,50,255")
    ap.add_argument("--red2",  default="170,179,80,255,50,255")
    ap.add_argument("--green", default="40,85,50,255,40,255")

    # shape/morph params
    ap.add_argument("--area-min",   type=int,   default=300)
    ap.add_argument("--aspect-min", type=float, default=2.0)
    ap.add_argument("--length-min", type=float, default=50)
    ap.add_argument("--kernel",     type=int,   default=5)
    ap.add_argument("--close-it",   type=int,   default=2)
    ap.add_argument("--open-it",    type=int,   default=1)
    args = ap.parse_args()

    if not os.path.exists(args.inp):
        raise SystemExit(f"Input not found: {args.inp}")

    roi = parse_roi(args.roi)
    parse = lambda s: tuple(int(v) for v in s.split(","))
    red_ranges   = [parse(args.red1), parse(args.red2)]
    green_ranges = [parse(args.green)]

    cap = cv2.VideoCapture(args.inp)
    if not cap.isOpened():
        raise SystemExit(f"Failed to open: {args.inp}")

    fps_src = cap.get(cv2.CAP_PROP_FPS) or args.fps
    if fps_src != fps_src:  # NaN
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

    hist = deque(maxlen=20)

    while True:
        ok, frame = cap.read()
        if not ok: break
        c = crop(frame, roi)

        r,g, m_r, m_g, ann = count_objects(
            c.copy(), red_ranges, green_ranges,
            area_min=args.area_min, aspect_min=args.aspect_min,
            length_min=args.length_min, kernel=args.kernel,
            close_iter=args.close_it, open_iter=args.open_it
        )
        hist.append((r,g))

        # overlay live counts
        label = f"RED: {r}   GREEN: {g}"
        cv2.putText(ann, label, (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(ann, label, (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0),     1, cv2.LINE_AA)

        if writer is not None:
            writer.write(ann)

        if args.show:
            cv2.imshow("Cropped + detections", ann)
            cv2.imshow("Mask RED", m_r)
            cv2.imshow("Mask GREEN", m_g)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')): break

    cap.release()
    if writer is not None: writer.release()
    cv2.destroyAllWindows()

    if hist:
        from collections import Counter
        r_final = Counter([x for x,_ in hist]).most_common(1)[0][0]
        g_final = Counter([y for _,y in hist]).most_common(1)[0][0]
        print(f"FINAL COUNTS  RED={r_final}  GREEN={g_final}")
    else:
        print("No frames processed.")

if __name__ == "__main__":
    main()
