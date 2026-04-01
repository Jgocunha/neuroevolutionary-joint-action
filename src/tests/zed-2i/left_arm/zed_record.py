#!/usr/bin/env python3
import cv2, argparse, time, os, datetime

def parse_size(s):
    if not s: return None
    w, h = s.lower().split("x")
    return int(w), int(h)

def timestamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

def hud(img, text):
    org = (10, 24)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0),     1, cv2.LINE_AA)
    return img

def open_capture(dev, size, fps, fourcc_cap):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if fourcc_cap:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc_cap))
    if size:
        w, h = size
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, float(fps))
    if not cap.isOpened():
        raise SystemExit(f"Failed to open {dev}")
    return cap

def make_writer(path, size, fps, fourcc_out):
    w, h = size
    writer = cv2.VideoWriter(
        path,
        cv2.VideoWriter_fourcc(*fourcc_out),
        float(fps),
        (w, h)
    )
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open writer: {path}")
    return writer

def main():
    ap = argparse.ArgumentParser(description="ZED 2i recorder: press 'b' to begin, 'q' to quit.")
    ap.add_argument("--dev", default="/dev/video4", help="Video device, e.g., /dev/video4")
    ap.add_argument("--size", default="", help="Capture WxH (e.g., 2560x720 for SxS, 1920x1080 for mono)")
    ap.add_argument("--fps", type=int, default=60, help="Target FPS (writer uses this)")
    ap.add_argument("--fourcc", default="MJPG", help="Capture FOURCC (MJPG or YUYV recommended)")
    ap.add_argument("--out-fourcc", default="MJPG", help="Output FOURCC (e.g., MJPG, XVID, mp4v)")
    ap.add_argument("--outdir", default=".", help="Directory to save files")
    ap.add_argument("--basename", default="zed", help="Base name for output files")
    ap.add_argument("--split", action="store_true", help="If SxS, save LEFT and RIGHT to separate files")
    args = ap.parse_args()

    size = parse_size(args.size) if args.size else None
    cap = open_capture(args.dev, size, args.fps, args.fourcc)

    # Query actual capture size (device may override your request)
    ret, frame = cap.read()
    if not ret:
        raise SystemExit("Could not read an initial frame.")
    H, W = frame.shape[:2]

    is_sxs = (W / max(H, 1.0)) > 3.2  # simple heuristic for side-by-side
    left_writer = right_writer = mono_writer = None
    recording = False

    # Prepare names but only create writers when user presses 'b'
    os.makedirs(args.outdir, exist_ok=True)

    # FPS meter
    last_t = time.time()
    frames = 0
    measured_fps = 0.0

    print("Ready. Press 'b' to begin recording, 'q' to quit, 's' to snapshot.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed; exiting.")
            break

        frames += 1
        now = time.time()
        if now - last_t >= 1.0:
            measured_fps = frames / (now - last_t)
            frames = 0
            last_t = now

        # If recording, write to the right file(s)
        if recording:
            if is_sxs and args.split:
                mid = W // 2
                left  = frame[:, :mid]
                right = frame[:, mid:]
                left_writer.write(left)
                right_writer.write(right)
            else:
                mono_writer.write(frame)

        # Preview with HUD
        status = "REC" if recording else "IDLE"
        info = f"{status}  {args.dev}  {W}x{H}  ~{measured_fps:.1f} FPS  [cap:{args.fourcc} out:{args.out_fourcc}]"
        if is_sxs:
            mid = W // 2
            left  = frame[:, :mid]
            right = frame[:, mid:]
            cv2.imshow("ZED - LEFT (preview)", hud(left.copy(), info))
            cv2.imshow("ZED - RIGHT (preview)", hud(right.copy(), info))
        else:
            cv2.imshow("ZED - MONO (preview)", hud(frame.copy(), info))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('b'):
            if not recording:
                ts = timestamp()
                if is_sxs and args.split:
                    # create two writers
                    lw = os.path.join(args.outdir, f"{args.basename}_LEFT_{ts}.avi")
                    rw = os.path.join(args.outdir, f"{args.basename}_RIGHT_{ts}.avi")
                    left_writer  = make_writer(lw,  (W//2, H), args.fps, args.out_fourcc)
                    right_writer = make_writer(rw,  (W//2, H), args.fps, args.out_fourcc)
                    print(f"Recording started: {lw} and {rw}")
                else:
                    out = os.path.join(args.outdir, f"{args.basename}_{ts}.avi")
                    mono_writer = make_writer(out, (W, H), args.fps, args.out_fourcc)
                    print(f"Recording started: {out}")
                recording = True
            else:
                # Already recording; ignore extra 'b' presses
                pass
        elif key == ord('s'):
            ts = timestamp()
            if is_sxs:
                cv2.imwrite(f"left_{ts}.jpg",  frame[:, :W//2])
                cv2.imwrite(f"right_{ts}.jpg", frame[:, W//2:])
                print(f"Saved left_{ts}.jpg and right_{ts}.jpg")
            else:
                cv2.imwrite(f"mono_{ts}.jpg", frame)
                print(f"Saved mono_{ts}.jpg")

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    for w in (left_writer, right_writer, mono_writer):
        if w is not None:
            w.release()
    if recording:
        print("Recording stopped and files closed.")

if __name__ == "__main__":
    main()
