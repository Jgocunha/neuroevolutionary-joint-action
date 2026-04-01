#!/usr/bin/env python3
import cv2, argparse, time

def parse_size(s):
    if not s: return None
    w, h = s.lower().split("x")
    return int(w), int(h)

def put_hud(img, text):
    org = (10, 24)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1, cv2.LINE_AA)
    return img

def main():
    p = argparse.ArgumentParser(description="ZED 2i UVC preview (auto-split SxS).")
    p.add_argument("--dev", default="/dev/video4", help="Video device (e.g., /dev/video4)")
    p.add_argument("--size", default="", help="WxH, e.g., 2560x720 for SxS or 1920x1080 for mono")
    p.add_argument("--fps", type=int, default=30, help="Target FPS (best-effort)")
    p.add_argument("--fourcc", default="MJPG", help="FOURCC (try MJPG or YUYV)")
    args = p.parse_args()

    cap = cv2.VideoCapture(args.dev, cv2.CAP_V4L2)
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    if args.size:
        w, h = parse_size(args.size)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if args.fps:
        cap.set(cv2.CAP_PROP_FPS, float(args.fps))

    if not cap.isOpened():
        raise SystemExit(f"Failed to open {args.dev}")

    last_t = time.time()
    frames = 0
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed; exiting.")
            break

        frames += 1
        now = time.time()
        if now - last_t >= 1.0:
            fps = frames / (now - last_t)
            frames = 0
            last_t = now

        h, w = frame.shape[:2]
        is_sxs = (w / max(h, 1.0)) > 3.2  # wide side-by-side?

        hud = f"{args.dev}  {w}x{h}  ~{fps:.1f} FPS  [{args.fourcc}]   (q/ESC to quit, s to save)"

        if is_sxs:
            mid = w // 2
            left  = frame[:, :mid]
            right = frame[:, mid:]
            left_v  = put_hud(left.copy(),  "LEFT  " + hud)
            right_v = put_hud(right.copy(), "RIGHT " + hud)
            cv2.imshow("ZED 2i - LEFT", left_v)
            cv2.imshow("ZED 2i - RIGHT", right_v)
        else:
            mono_v = put_hud(frame.copy(), "MONO  " + hud)
            cv2.imshow("ZED 2i - MONO", mono_v)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            break
        elif key == ord('s'):
            ts = int(time.time())
            if is_sxs:
                cv2.imwrite(f"left_{ts}.jpg", left)
                cv2.imwrite(f"right_{ts}.jpg", right)
                print(f"Saved left_{ts}.jpg and right_{ts}.jpg")
            else:
                cv2.imwrite(f"mono_{ts}.jpg", frame)
                print(f"Saved mono_{ts}.jpg")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
