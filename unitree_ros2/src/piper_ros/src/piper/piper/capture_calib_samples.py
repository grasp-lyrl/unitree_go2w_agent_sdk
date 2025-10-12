#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hand–eye sample capturer with live QC overlay (convex-hull coverage + smart acceptance).

- Subscribes:  /camera/color/image_raw, /camera/color/camera_info
- Publishes:   /handeye/debug_image  (overlay: corners, axes, hull, metrics)
- Services:    /save (Trigger)      -> saves img_XXX.png + base_T_ee_XXX.npy if PASS
               /shutdown (Trigger)  -> shuts down the node

Acceptance (on /save):
  time_ok = skew_ms <= --max-skew-ms
  high_conf = (corners >= 16 and rms <= 1.0)
  normal    = (corners >= --min-corners and rms <= --max-rms-px and coverage >= --min-coverage)
  PASS if time_ok and (high_conf or normal)
"""
import os, math, argparse
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener


# ---------- OpenCV helpers (API compatibility) ----------
def aruco_detect(gray, dictionary):
    """Works with both new (ArucoDetector) and legacy (detectMarkers) APIs."""
    if hasattr(cv.aruco, "ArucoDetector"):
        params = cv.aruco.DetectorParameters()
        det = cv.aruco.ArucoDetector(dictionary, params)
        return det.detectMarkers(gray)
    return cv.aruco.detectMarkers(gray, dictionary)

def _get_charuco_object_corners(board):
    """
    Returns (N,3) array of chessboard (ChArUco) 3D points (z=0) across OpenCV builds.
    """
    # Preferred (newer OpenCV)
    if hasattr(board, "getChessboardCorners"):
        obj = board.getChessboardCorners()
        return np.asarray(obj, dtype=np.float32)
    # Some builds expose a property
    if hasattr(board, "chessboardCorners"):
        return np.asarray(board.chessboardCorners, dtype=np.float32)
    # Fallback: synthesize from geometry (squaresX-1, squaresY-1)
    if hasattr(board, "getChessboardSize"):
        nx, ny = map(int, board.getChessboardSize())
    else:
        nx = int(getattr(board, "squaresX", 0)) - 1
        ny = int(getattr(board, "squaresY", 0)) - 1
    if hasattr(board, "getSquareLength"):
        sl = float(board.getSquareLength())
    else:
        sl = float(getattr(board, "squareLength", 1.0))
    pts = [[x*sl, y*sl, 0.0] for y in range(ny) for x in range(nx)]
    return np.array(pts, dtype=np.float32)

def charuco_pose(img_bgr, K, dist, board, dictionary, min_corners=12):
    """
    Returns (ok, rvec, tvec, n_corners, reproj_rms_px, coverage_frac, overlay_img).
    Coverage is area(convex hull of detected Charuco corners) / image area.
    """
    H, W = img_bgr.shape[:2]
    gray = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)
    corners, ids, _ = aruco_detect(gray, dictionary)
    overlay = img_bgr.copy()

    if ids is None or len(ids) == 0:
        return False, None, None, 0, 1e9, 0.0, overlay

    try:
        cv.aruco.drawDetectedMarkers(overlay, corners, ids)
    except Exception:
        pass

    ok, ch_corners, ch_ids = cv.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if not ok or ch_ids is None:
        return False, None, None, 0, 1e9, 0.0, overlay

    n_c = int(ch_ids.size)
    if n_c < min_corners:
        # still draw hull for visual feedback, but report not-ok
        xys = ch_corners.reshape(-1, 2).astype(np.float32)
        if len(xys) >= 3:
            hull = cv.convexHull(xys)
            cv.polylines(overlay, [hull.astype(np.int32)], True, (0, 255, 0), 2)
            coverage = float(cv.contourArea(hull) / (W * H))
        else:
            coverage = 0.0
        cv.putText(overlay, f"corners={n_c}", (10, 24), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2, cv.LINE_AA)
        return False, None, None, n_c, 1e9, coverage, overlay

    ok, rvec, tvec = cv.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, dist, None, None)
    if not ok:
        return False, None, None, n_c, 1e9, 0.0, overlay

    # Reprojection RMS (px)
    obj_all = _get_charuco_object_corners(board)      # (N_all,3)
    obj_pts = obj_all[ch_ids.squeeze()]               # select observed corners
    proj, _ = cv.projectPoints(obj_pts, rvec, tvec, K, dist)
    rms = float(np.linalg.norm(proj.squeeze() - ch_corners.squeeze(), axis=1).mean())

    # Coverage: convex hull of observed corners / image area
    xys = ch_corners.reshape(-1, 2).astype(np.float32)
    hull = cv.convexHull(xys)
    hull_area = float(cv.contourArea(hull)) if hull is not None and len(hull) >= 3 else 0.0
    coverage = hull_area / float(W * H)

    # Draw axes + hull
    try:
        cv.drawFrameAxes(overlay, K, dist, rvec, tvec, 0.05)
    except Exception:
        pass
    if hull is not None and len(hull) >= 3:
        cv.polylines(overlay, [hull.astype(np.int32)], True, (0, 255, 0), 2)

    # Metrics text
    cv.putText(overlay, f"corners={n_c}  rms={rms:.2f}px  cov={coverage*100:.0f}%",
               (10, 24), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2, cv.LINE_AA)

    return True, rvec, tvec, n_c, rms, coverage, overlay

def tf_to_mat44(t):
    q = t.transform.rotation; p = t.transform.translation
    # Normalize quaternion
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) or 1.0
    x, y, z, w = q.x/n, q.y/n, q.z/n, q.w/n
    R = np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3,:3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T

def fmt_stamp(stamp):
    # stamp is a builtin_interfaces/Time
    return f"{stamp.sec}.{stamp.nanosec:09d}"



# ---------- Node ----------
class HandEyeCapture(Node):
    def __init__(self, args):
        super().__init__('handeye_capture')
        self.base, self.ee, self.out = args.base, args.ee, args.out
        os.makedirs(self.out, exist_ok=True)

        # QC thresholds
        self.min_corners  = args.min_corners
        self.max_rms_px   = args.max_rms_px
        self.min_cov      = args.min_coverage
        self.max_skew_ms  = args.max_skew_ms

        # ChArUco
        self.dictionary = cv.aruco.getPredefinedDictionary(args.dict_id)
        self.board = cv.aruco.CharucoBoard(
            (args.squares_x, args.squares_y), args.square, args.marker, self.dictionary
        )

        self.bridge = CvBridge()
        self.last = None       # (img_bgr, stamp)
        self.K = None; self.dist = None
        self.seq = 0

        # TF
        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self)

        # ROS I/O
        self.sub_img = self.create_subscription(Image, args.image_topic, self.on_img, 10)
        self.sub_ci  = self.create_subscription(CameraInfo, args.ci_topic, self.on_ci, 10)
        self.pub_dbg = self.create_publisher(Image, '/handeye/debug_image', 1)

        self.create_service(Trigger, 'save', self.on_save)
        self.create_service(Trigger, 'shutdown', self.on_shutdown)

        self.get_logger().info(
            "Ready.\n"
            " - View /handeye/debug_image in rqt_image_view to monitor quality.\n"
            " - Save a sample:    ros2 service call /save std_srvs/srv/Trigger {}\n"
            " - Shutdown the node: ros2 service call /shutdown std_srvs/srv/Trigger {}"
        )

    # --- Callbacks ---
    def on_ci(self, msg: CameraInfo):
        self.K  = np.array(msg.k, dtype=float).reshape(3,3)
        self.dist = np.array(msg.d, dtype=float)
        path = os.path.join(self.out, "intrinsics.npz")
        if not os.path.exists(path):
            np.savez(path, K=self.K, dist=self.dist)
            self.get_logger().info(f"Saved intrinsics to {path}")

    def on_img(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last = (img, msg.header.stamp)
        if self.K is None:
            return
        # Live overlay (corners/RMS/coverage)
        ok, rvec, tvec, n_c, rms, cov, overlay = charuco_pose(
            img, self.K, self.dist, self.board, self.dictionary, self.min_corners
        )
        # Draw simple banner area
        cv.rectangle(overlay, (0,0), (overlay.shape[1], 36), (0,0,0), -1)
        label = f"corners={n_c}  rms={rms:.2f}px  cov={cov*100:.0f}%"
        color = (0,255,0) if ok else (0,0,255)
        cv.putText(overlay, label, (10, 26), cv.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv.LINE_AA)
        dbg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        dbg.header.stamp = msg.header.stamp
        self.pub_dbg.publish(dbg)

    def on_save(self, req, resp):
        if self.last is None:
            resp.success = False; resp.message = "No image yet."
            return resp
        if self.K is None or self.dist is None:
            resp.success = False; resp.message = "No CameraInfo yet."
            return resp

        img, stamp = self.last
        ok, rvec, tvec, n_c, rms, cov, overlay = charuco_pose(
            img, self.K, self.dist, self.board, self.dictionary, self.min_corners
        )

        # Lookup TF at image timestamp
        try:
            t = self.buf.lookup_transform(self.base, self.ee, Time.from_msg(stamp),
                                          timeout=Duration(seconds=0.5))
        except Exception as e:
            resp.success = False; resp.message = f"TF lookup failed: {e}"
            return resp

        # Time skew (ms)
        tf_ns  = t.header.stamp.sec*1_000_000_000 + t.header.stamp.nanosec
        img_ns = stamp.sec*1_000_000_000 + stamp.nanosec
        skew_ms = abs(tf_ns - img_ns)/1e6

        # Print both timestamps for verification
        self.get_logger().info(
            f"[TIMING] img={fmt_stamp(stamp)}  tf={fmt_stamp(t.header.stamp)}  skew={skew_ms:.1f} ms"
        )


        # --- Acceptance logic ---
        time_ok  = (skew_ms <= self.max_skew_ms)
        hi_conf  = (n_c >= 16 and rms <= 1.0)
        normal   = (n_c >= self.min_corners and rms <= self.max_rms_px and cov >= self.min_cov)
        pass_frame = time_ok and (hi_conf or normal)

        reasons = []
        if not time_ok:            reasons.append(f"skew {skew_ms:.1f}ms>{self.max_skew_ms:.1f}ms")
        if not (hi_conf or normal):
            if n_c < self.min_corners: reasons.append(f"corners {n_c}<{self.min_corners}")
            if rms  > self.max_rms_px:  reasons.append(f"rms {rms:.2f}px>{self.max_rms_px:.2f}")
            if cov  < self.min_cov:     reasons.append(f"coverage {cov*100:.0f}%<{self.min_cov*100:.0f}%")

        # Banner with pass/fail & skew
        banner = ("PASS" if pass_frame else "REJECT: " + "; ".join(reasons)) + f" | skew={skew_ms:.1f}ms"
        color  = (0,255,0) if pass_frame else (0,0,255)
        cv.rectangle(overlay, (0,0), (overlay.shape[1], 36), (0,0,0), -1)
        cv.putText(overlay, banner, (10, 26), cv.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv.LINE_AA)
        dbg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'); dbg.header.stamp = stamp
        self.pub_dbg.publish(dbg)

        if not pass_frame:
            resp.success = False; resp.message = banner
            return resp

        # Save sample
        T_base_ee = tf_to_mat44(t)
        fn = f"{self.seq:03d}"
        cv.imwrite(os.path.join(self.out, f"img_{fn}.png"), img)
        np.save(os.path.join(self.out, f"base_T_ee_{fn}.npy"), T_base_ee)
        self.seq += 1
        resp.success = True; resp.message = f"Saved sample {fn} (corners={n_c}, rms={rms:.2f}px, cov={cov*100:.0f}%, skew={skew_ms:.1f}ms)"
        return resp

    def on_shutdown(self, req, resp):
        resp.success = True; resp.message = "Shutting down."
        rclpy.get_global_executor().create_task(self._shutdown_async())
        return resp

    async def _shutdown_async(self):
        await rclpy.shutdown()


# ---------- main ----------
def main():
    ap = argparse.ArgumentParser()
    # ROS topics & frames
    ap.add_argument('--base', default='base_link')
    ap.add_argument('--ee',   default='link6')
    ap.add_argument('--image-topic', default='/camera/color/image_raw')
    ap.add_argument('--ci-topic',    default='/camera/color/camera_info')
    ap.add_argument('--out', default='/home/unitree/unitree_ros2/src/piper_ros/samples_charuco')
    # ChArUco geometry (meters)
    ap.add_argument('--squares-x', type=int, default=5)
    ap.add_argument('--squares-y', type=int, default=7)
    ap.add_argument('--square', type=float, default=0.0367, help='square length (m) — measured printed')
    ap.add_argument('--marker', type=float, default=0.0269, help='marker length (m) — measured printed')
    # Dictionary (default: 5x5_1000)
    default_dict = int(getattr(cv.aruco, "DICT_5X5_1000", 10))
    ap.add_argument('--dict-id', type=int, default=default_dict)
    # QC thresholds
    ap.add_argument('--min-corners', type=int, default=12)
    ap.add_argument('--max-rms-px',  type=float, default=1.5)
    ap.add_argument('--min-coverage',type=float, default=0.40)
    ap.add_argument('--max-skew-ms', type=float, default=50.0)

    args = ap.parse_args()
    rclpy.init()
    node = HandEyeCapture(args)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
