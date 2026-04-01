#!/usr/bin/env python3
"""
zed_object_type_position.py

Reads a ZED video and crops each frame to a fixed ROI (predefined from sample images).
- Press 'q' to quit.
- Use --show to preview while saving.
- By default, reads 'sls_r/zed_RIGHT_sls_r.avi' from the current working directory.
"""

import cv2
import os
import argparse
from time import sleep

# --- ROI determined from your sample frames ---
ROI_X = 383
ROI_Y = 380
ROI_W = 242
ROI_H = 50
# 383,400,242,100
ROI = (ROI_X, ROI_Y, ROI_W, ROI_H)

def crop_frame(frame, roi):
    x,y,w,h = roi
    H, W = frame.shape[:2]
    # clamp just in case
    x = max(0, min(x, W-1))
    y = max(0, min(y, H-1))
    w = max(1, min(w, W-x))
    h = max(1, min(h, H-y))
    return frame[y:y+h, x:x+w]

def main():
    ap = argparse.ArgumentParser(description="Crop a fixed ROI from a ZED video")
    ap.add_argument("--in", dest="inp", default="zed_RIGHT_slsl_h.avi", help="Input video path")
    ap.add_argument("--out", dest="out", default="cropped_RIGHT_sls_r.mp4", help="Output video path (.mp4 or .avi)")
    ap.add_argument("--show", action="store_true", help="Show preview while processing")
    ap.add_argument("--codec", default="mp4v", help="Output FOURCC (mp4v, XVID, MJPG, etc.)")
    args = ap.parse_args()

    if not os.path.exists(args.inp):
        raise SystemExit(f"Input not found: {args.inp}")

    cap = cv2.VideoCapture(args.inp)
    if not cap.isOpened():
        raise SystemExit(f"Failed to open video: {args.inp}")

    # Get FPS and total frames if available
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps != fps:  # NaN check
        fps = 60.0  # fallback

    # Prime one frame to know size
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Could not read first frame")

    roi_frame = crop_frame(frame, ROI)
    h, w = roi_frame.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    writer = cv2.VideoWriter(args.out, fourcc, fps, (w, h))
    if not writer.isOpened():
        raise SystemExit(f"Failed to open writer: {args.out}")

    # Rewind to frame 0
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    print(f"Cropping ROI x={ROI_X} y={ROI_Y} w={ROI_W} h={ROI_H}")
    print(f"Input: {args.inp}  FPS: {fps:.2f}   Output: {args.out} (493x363, codec={args.codec})")
    print("Press 'q' in the preview window to stop early.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        roi_frame = crop_frame(frame, ROI)
        writer.write(roi_frame)

        if args.show:
            vis = roi_frame.copy()
            cv2.imshow("Cropped ROI", vis)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
        sleep(0.01)  # be nice to the CPU

    cap.release()
    writer.release()
    cv2.destroyAllWindows()
    print("Done.")

if __name__ == "__main__":
    main()
