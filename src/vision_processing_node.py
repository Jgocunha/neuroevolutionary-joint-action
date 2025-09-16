#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 node: vision_processing_node
- Opens ZED 2i V4L2 node with OpenCV (CPU only)
- Runs BOTH pipelines (objects + hand) with your tuned defaults
- Publishes:
    /scene_objects  (kuka_lbr_iiwa14_marlab/msg/SceneObjects)
    /hand_position  (std_msgs/msg/Float64)
"""
import os, cv2, numpy as np, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from kuka_lbr_iiwa14_marlab.msg import SceneObject, SceneObjects

# ---------------- device opening ----------------
CANDIDATE_SIZES = [(2560,720),(1280,720),(1920,1080)]
CANDIDATE_FMTS  = ["MJPG","YUYV"]

def open_camera(dev, size, fps, fmt_prefer=None):
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
    raise RuntimeError(f"Failed to open camera: {last_err}")

# --------------- common helpers ----------------
def crop_roi(img, roi):
    x,y,w,h = roi
    H,W = img.shape[:2]
    x = max(0, min(x, W-1)); y = max(0, min(y, H-1))
    w = max(1, min(w, W-x));  h = max(1, min(h, H-y))
    return img[y:y+h, x:x+w]

def moving_avg(x, k):
    if k <= 1: return x
    k = int(k); pad = (k-1)//2
    xp = np.pad(x, (pad, k-1-pad), mode='edge')
    kernel = np.ones(k, dtype=np.float32)/k
    return np.convolve(xp, kernel, mode='valid')

def bin_dilate_1d(a, k):
    k = int(k); c = np.convolve(a.astype(np.uint8), np.ones(k, dtype=np.uint8), mode='same'); return c > 0

def bin_erode_1d(a, k):
    k = int(k); c = np.convolve(a.astype(np.uint8), np.ones(k, dtype=np.uint8), mode='same'); return c >= k

def bin_close_1d(a, k): return bin_erode_1d(bin_dilate_1d(a, k), k)

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

# --------------- objects line-scan -------------
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
    if gap_fill > 0: labels = merge_small_gaps(labels, gap_fill=gap_fill)

    runs = []
    i=0
    while i<W2:
        if labels[i]==0: i+=1; continue
        c=labels[i]; j=i+1
        while j<W2 and labels[j]==c: j+=1
        runs.append((c,i,j)); i=j

    centers = []
    for c,xs,xe in runs:
        xc = int((xs+xe-1)/2)
        centers.append((c,xc,y0+band_h//2))
    centers.sort(key=lambda t:t[1])
    return centers

# ---------------- hand line-scan ----------------
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

def hand_center_on_band(roi_bgr, band_h, frac_th, smooth, close_w, s_min, v_min):
    H,W = roi_bgr.shape[:2]
    y0 = max(0, H//2 - band_h//2)
    band = roi_bgr[y0:y0+band_h, :]
    m = skin_mask_ycrcb_hsv(band, s_min=s_min, v_min=v_min)
    frac = np.sum(m, axis=0) / (band_h*255.0)
    if smooth>1: frac = moving_avg(frac, smooth)
    W2=min(W,len(frac)); frac=frac[:W2]
    cols = frac>=frac_th
    if close_w>1: cols=bin_close_1d(cols, close_w)

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

# ------------------ ROS 2 Node ------------------
class VisionProcessingNode(Node):
    def __init__(self):
        super().__init__("vision_processing_node")

        # --- parameters (defaults = your known-good args) ---
        self.declare_parameter("dev", "/dev/video0")
        self.declare_parameter("size", "2560x720")
        self.declare_parameter("fps", 60)
        self.declare_parameter("fmt", "MJPG")
        self.declare_parameter("half", "right")  # none/left/right

        self.declare_parameter("roi_objects", "383,400,242,100")
        self.declare_parameter("roi_width_cm_objects", 60.0)
        self.declare_parameter("roi_hand", "383,380,242,50")
        self.declare_parameter("roi_width_cm_hand", 60.0)

        self.declare_parameter("only_changes", True)
        self.declare_parameter("change_th", 6)
        self.declare_parameter("show", True)

        # objects thresholds
        self.band_h_o=11; self.frac_th_o=0.35; self.smooth_o=7
        self.margin_o=0.05; self.close_w_o=9; self.gap_fill_o=7
        self.s_min_o=60; self.v_min_o=40
        self.red_ranges=[(0,10,80,255,50,255),(170,179,80,255,50,255)]
        self.green_ranges=[(35,90,40,255,40,255)]
        # hand thresholds
        self.band_h_h=21; self.frac_th_h=0.25; self.smooth_h=7
        self.close_w_h=11; self.s_min_h=40; self.v_min_h=40

        # ROS pubs
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
        self.pub_objects = self.create_publisher(SceneObjects, "scene_objects", qos)
        self.pub_hand    = self.create_publisher(Float64,      "hand_position", qos)

        # state for change-filtered publishing
        self.prev_obj_repr = None   # (tuple(red_xs), tuple(green_xs))
        self.prev_hand_cm  = None   # last published hand value in cm (100.0 when not present)

        # open device
        dev   = self.get_parameter("dev").get_parameter_value().string_value
        size  = self.get_parameter("size").get_parameter_value().string_value
        fps   = self.get_parameter("fps").get_parameter_value().integer_value
        fmt   = self.get_parameter("fmt").get_parameter_value().string_value
        self.cap, self.W, self.H, self.fmt_used = open_camera(dev, size, fps, fmt_prefer=fmt)
        self.get_logger().info(f"Opened {dev} @ {self.W}x{self.H} {self.fmt_used}")

        # start thread
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def get_param_str(self, key): return self.get_parameter(key).get_parameter_value().string_value
    def get_param_bool(self, key): return self.get_parameter(key).get_parameter_value().bool_value
    def get_param_int(self, key): return int(self.get_parameter(key).get_parameter_value().integer_value)
    def get_param_float(self, key): return float(self.get_parameter(key).get_parameter_value().double_value)

    def loop(self):
        half = self.get_param_str("half")
        roi_o = tuple(int(v) for v in self.get_param_str("roi_objects").split(","))
        rwc_o = self.get_param_float("roi_width_cm_objects")
        roi_h = tuple(int(v) for v in self.get_param_str("roi_hand").split(","))
        rwc_h = self.get_param_float("roi_width_cm_hand")
        only_changes = self.get_param_bool("only_changes")
        change_th = self.get_param_int("change_th")
        show = self.get_param_bool("show")

        while rclpy.ok() and self.running:
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().error("Frame grab failed; stopping.")
                break

            # SxS split
            if half != "none":
                mid = self.W // 2
                frame = frame[:, :mid] if half=="left" else frame[:, mid:]

            # ---------- OBJECTS ----------
            roi_frame_o = crop_roi(frame, roi_o)
            centers_o = classify_band_objects(
                roi_frame_o, self.band_h_o, self.frac_th_o, self.smooth_o,
                self.margin_o, self.close_w_o, self.gap_fill_o,
                self.red_ranges, self.green_ranges, self.s_min_o, self.v_min_o
            )
            red_xs   = tuple(sorted([x for (c,x,_) in centers_o if c==1]))
            green_xs = tuple(sorted([x for (c,x,_) in centers_o if c==2]))
            changed_o = False
            if self.prev_obj_repr is None:
                changed_o = True
            else:
                pr, pg = self.prev_obj_repr
                if len(pr)!=len(red_xs) or len(pg)!=len(green_xs):
                    changed_o = True
                else:
                    for a,b in zip(pr, red_xs):
                        if abs(a-b) > change_th: changed_o=True; break
                    if not changed_o:
                        for a,b in zip(pg, green_xs):
                            if abs(a-b) > change_th: changed_o=True; break
            if (not only_changes) or changed_o:
                # build and publish SceneObjects (cm from RIGHT)
                msg = SceneObjects()
                objs = []
                for x in red_xs:
                    so = SceneObject()
                    so.type = "s"      # RED → small
                    so.position = float(round(px_to_cm_from_right(x, roi_o[2], rwc_o), 4))
                    objs.append(so)
                for x in green_xs:
                    so = SceneObject()
                    so.type = "l"      # GREEN → large
                    so.position = float(round(px_to_cm_from_right(x, roi_o[2], rwc_o), 4))
                    objs.append(so)
                msg.objects = objs
                self.pub_objects.publish(msg)
                self.prev_obj_repr = (red_xs, green_xs)

            # ---------- HAND ----------
            roi_frame_h = crop_roi(frame, roi_h)
            has_hand, xh, y0h, bhh = hand_center_on_band(
                roi_frame_h, self.band_h_h, self.frac_th_h, self.smooth_h, self.close_w_h,
                self.s_min_h, self.v_min_h
            )

            # Always produce a cm value; when no hand -> 100.0
            cm = float(px_to_cm_from_right(xh, roi_h[2], rwc_h)) if has_hand else 100.0

            # Publish policy: always publish the current cm (simple & robust),
            # or, if you prefer to respect --only_changes, comment the next line and
            # use the block below it instead.
            #self.pub_hand.publish(Float64(data=cm))

            # If you prefer change-filtered publishing for the hand too, use this:
            if not only_changes:
                self.pub_hand.publish(Float64(data=cm))
            else:
                if self.prev_hand_cm is None or abs(cm - self.prev_hand_cm) > 1e-6:
                    self.pub_hand.publish(Float64(data=cm))

            self.prev_hand_cm = cm

            # ---------- preview ----------
            if show:
                vis_o = roi_frame_o.copy()
                y0o = vis_o.shape[0]//2 - self.band_h_o//2
                cv2.rectangle(vis_o, (0,y0o), (vis_o.shape[1]-1, y0o+self.band_h_o-1), (200,200,200), 1)
                for (c,x,y) in centers_o:
                    color = (0,0,255) if c==1 else (0,255,0)
                    cv2.circle(vis_o, (x, y), 5, color, -1)

                vis_h = roi_frame_h.copy()
                y0h = vis_h.shape[0]//2 - self.band_h_h//2
                cv2.rectangle(vis_h, (0,y0h), (vis_h.shape[1]-1, y0h+self.band_h_h-1), (200,200,200), 1)
                if has_hand:
                    cv2.circle(vis_h, (xh, y0h+self.band_h_h//2), 6, (0,215,255), -1)

                # match heights, show side-by-side
                if vis_o.shape[0] != vis_h.shape[0]:
                    scale = vis_o.shape[0] / vis_h.shape[0]
                    vis_h = cv2.resize(vis_h, (int(vis_h.shape[1]*scale), vis_o.shape[0]))
                vis = cv2.hconcat([vis_o, vis_h])
                cv2.imshow("vision_processing_node (objects | hand)", vis)
                if (cv2.waitKey(1) & 0xFF) in (27, ord('q')):
                    self.get_logger().info("Quit requested by user.")
                    break

        self.running = False
        try:
            self.cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    time.sleep(2.5)  # give rclpy a moment
    node = VisionProcessingNode()
    try:
        rclpy.spin(node)
    finally:
        node.running = False
        if node.thread.is_alive():
            node.thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
