#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, math, argparse, csv, numpy as np, cv2 as cv
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

# ---------- helpers ----------
def aruco_detect(gray, dictionary):
    if hasattr(cv.aruco, "ArucoDetector"):
        params = cv.aruco.DetectorParameters()
        det = cv.aruco.ArucoDetector(dictionary, params)
        return det.detectMarkers(gray)
    return cv.aruco.detectMarkers(gray, dictionary)

def board_obj_corners(board):
    if hasattr(board, "getChessboardCorners"):
        return np.asarray(board.getChessboardCorners(), np.float32)
    if hasattr(board, "chessboardCorners"):
        return np.asarray(board.chessboardCorners, np.float32)
    nx = int(getattr(board, "squaresX", 0)) - 1
    ny = int(getattr(board, "squaresY", 0)) - 1
    sl = float(getattr(board, "squareLength", 1.0))
    pts = [[x*sl, y*sl, 0.0] for y in range(ny) for x in range(nx)]
    return np.array(pts, np.float32)

def tf_to_mat44(t):
    q = t.transform.rotation; p = t.transform.translation
    n = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) or 1.0
    x,y,z,w = q.x/n, q.y/n, q.z/n, q.w/n
    R = np.array([[1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                  [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
                  [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]], float)
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = [p.x, p.y, p.z]
    return T

def inv_T(T):
    R = T[:3,:3]; t = T[:3,3:4]
    Ti = np.eye(4); Ti[:3,:3] = R.T; Ti[:3,3:4] = -R.T @ t
    return Ti

def rot_angle_deg(R):
    a = (np.trace(R)-1)/2.0
    a = float(np.clip(a, -1.0, 1.0))
    return np.degrees(np.arccos(a))

def fmt_stamp(stamp): return f"{stamp.sec}.{stamp.nanosec:09d}"

def rmat_to_rvec(R):
    rvec, _ = cv.Rodrigues(R)
    return rvec.reshape(3)

# ---------- node ----------
class VerifyHandEye(Node):
    def __init__(self, args):
        super().__init__("verify_handeye_charuco")
        self.base = args.base
        self.ee   = args.ee
        self.cam  = args.camera
        self.max_skew_ms = args.max_skew_ms
        self.bridge = CvBridge()
        self.K = None; self.dist = None

        # TF
        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self)

        # Charuco
        self.dictionary = cv.aruco.getPredefinedDictionary(args.dict_id)
        self.board = cv.aruco.CharucoBoard((args.squares_x, args.squares_y),
                                           args.square, args.marker, self.dictionary)
        self.obj_full = board_obj_corners(self.board)

        # IO
        self.create_subscription(CameraInfo, args.ci_topic, self.on_ci, 10)
        self.create_subscription(Image, args.image_topic, self.on_img, 10)

        # Stats
        self.hist = deque(maxlen=args.window)

        # CSV
        self.csv_path = args.csv
        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
        self._csv_file = open(self.csv_path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            "img_stamp","tf_stamp","skew_ms",
            "corners","charuco_rms_px","coverage",
            "base_board_tx","base_board_ty","base_board_tz",
            "base_board_rvec_x","base_board_rvec_y","base_board_rvec_z",
            "t_std_x","t_std_y","t_std_z","rot_med_deg","reproj_rms_px"
        ])
        self._csv_file.flush()

        self.get_logger().info(
            f"Verify node ready. Logging to {self.csv_path}\n"
            f"Show the ChArUco to the camera and move the wrist around."
        )

    def on_ci(self, msg: CameraInfo):
        self.K = np.array(msg.k, float).reshape(3,3)
        self.dist = np.array(msg.d, float)

    def get_charuco_corners(self, img_bgr):
        gray = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detect(gray, self.dictionary)
        if ids is None or len(ids)==0: return False, None, None
        ok, ch_corners, ch_ids = cv.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
        if not ok or ch_ids is None or ch_ids.size < 6: return False, None, None
        return True, ch_corners, ch_ids

    def detect_charuco(self, img_bgr):
        ok, ch_corners, ch_ids = self.get_charuco_corners(img_bgr)
        if not ok:
            return False, None, None, 0, 1e9, 0.0, None, None
        okp, rvec, tvec = cv.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, self.board, self.K, self.dist, None, None)
        if not okp:
            return False, None, None, int(ch_ids.size), 1e9, 0.0, None, None
        # RMS on observed
        obj = self.obj_full[ch_ids.squeeze()]
        proj, _ = cv.projectPoints(obj, rvec, tvec, self.K, self.dist)
        rms = float(np.sqrt(np.mean(np.sum((proj.squeeze()-ch_corners.squeeze())**2, axis=1))))
        # coverage
        H,W = img_bgr.shape[:2]
        xys = ch_corners.reshape(-1,2).astype(np.float32)
        hull = cv.convexHull(xys) if len(xys)>=3 else None
        cov = (cv.contourArea(hull)/(W*H)) if (hull is not None and len(hull)>=3) else 0.0
        return True, rvec, tvec, int(ch_ids.size), rms, cov, ch_corners, ch_ids

    def on_img(self, msg: Image):
        if self.K is None: return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        ok, rvec, tvec, n, rms, cov, ch_corners, ch_ids = self.detect_charuco(img)
        if not ok:
            return

        # TF at image time
        try:
            t_base_ee = self.buf.lookup_transform(self.base, self.ee, Time.from_msg(msg.header.stamp),
                                                  timeout=Duration(seconds=0.25))
            t_ee_cam  = self.buf.lookup_transform(self.ee, self.cam, Time.from_msg(msg.header.stamp),
                                                  timeout=Duration(seconds=0.25))
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # skew
        tf_ns = t_base_ee.header.stamp.sec*1_000_000_000 + t_base_ee.header.stamp.nanosec
        img_ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        skew_ms = abs(tf_ns - img_ns)/1e6
        if skew_ms > self.max_skew_ms:
            self.get_logger().warn(f"[TIMING] img={fmt_stamp(msg.header.stamp)} tf={fmt_stamp(t_base_ee.header.stamp)} skew={skew_ms:.1f}ms > {self.max_skew_ms}ms")
            return
        else:
            self.get_logger().info(f"[TIMING] img={fmt_stamp(msg.header.stamp)} tf={fmt_stamp(t_base_ee.header.stamp)} skew={skew_ms:.1f}ms")

        # transforms
        T_base_ee  = tf_to_mat44(t_base_ee)
        T_ee_cam   = tf_to_mat44(t_ee_cam)
        T_cam_board = np.eye(4); T_cam_board[:3,:3], _ = cv.Rodrigues(rvec); T_cam_board[:3,3] = tvec.ravel()

        # inferred board in base
        T_base_board = T_base_ee @ T_ee_cam @ T_cam_board
        self.hist.append(T_base_board.copy())

        # window stats
        trans = np.stack([H[:3,3] for H in self.hist], axis=0)
        mean_t = np.mean(trans, axis=0)
        std_t = np.std(trans, axis=0)
        R0 = self.hist[0][:3,:3]
        angs = [rot_angle_deg(R0.T @ H[:3,:3]) for H in self.hist]
        med_ang = float(np.median(angs))

        # reprojection via TF-predicted camera pose
        T_base_cam = T_base_ee @ T_ee_cam
        T_cam_board_tf = inv_T(T_base_cam) @ T_base_board
        R_tf = T_cam_board_tf[:3,:3]; t_tf = T_cam_board_tf[:3,3:4]
        if ch_corners is not None:
            sel = self.obj_full[ch_ids.squeeze()]
            proj_sel, _ = cv.projectPoints(sel, cv.Rodrigues(R_tf)[0], t_tf, self.K, self.dist)
            rp_rms = float(np.sqrt(np.mean(np.sum((proj_sel.squeeze()-ch_corners.squeeze())**2, axis=1))))
        else:
            rp_rms = float('nan')

        rvec_base_board = rmat_to_rvec(T_base_board[:3,:3])

        # console + CSV
        self.get_logger().info(
            f"[VERIFY] N={len(self.hist)} t_std(m)={std_t.round(4).tolist()} rot_med={med_ang:.2f}° rpRMS={rp_rms:.2f}px"
        )

        self._csv.writerow([
            fmt_stamp(msg.header.stamp),
            fmt_stamp(t_base_ee.header.stamp),
            f"{skew_ms:.3f}",
            n, f"{rms:.3f}", f"{cov:.3f}",
            f"{T_base_board[0,3]:.6f}", f"{T_base_board[1,3]:.6f}", f"{T_base_board[2,3]:.6f}",
            f"{rvec_base_board[0]:.6f}", f"{rvec_base_board[1]:.6f}", f"{rvec_base_board[2]:.6f}",
            f"{std_t[0]:.6f}", f"{std_t[1]:.6f}", f"{std_t[2]:.6f}",
            f"{med_ang:.6f}", f"{rp_rms:.6f}"
        ])
        self._csv_file.flush()

    def destroy_node(self):
        try:
            if hasattr(self, "_csv_file"):
                self._csv_file.flush(); self._csv_file.close()
        finally:
            super().destroy_node()

# ---------- main ----------
def main():
    ap = argparse.ArgumentParser(description="Verify link6->camera TF using a ChArUco board (with CSV logging).")
    ap.add_argument('--base', default='base_link')
    ap.add_argument('--ee',   default='link6')
    ap.add_argument('--camera', default='camera')
    ap.add_argument('--image-topic', default='/camera/color/image_raw')
    ap.add_argument('--ci-topic',    default='/camera/color/camera_info')
    ap.add_argument('--max-skew-ms', type=float, default=50.0)
    ap.add_argument('--window', type=int, default=30, help='rolling window for stability stats')
    ap.add_argument('--csv', type=str, default='handeye_verify.csv', help='output CSV path')

    # Charuco params (meters)
    ap.add_argument('--squares-x', type=int, default=5)
    ap.add_argument('--squares-y', type=int, default=7)
    ap.add_argument('--square', type=float, default=0.0367)
    ap.add_argument('--marker', type=float, default=0.0269)
    default_dict = int(getattr(cv.aruco, "DICT_5X5_1000", 10))
    ap.add_argument('--dict-id', type=int, default=default_dict)

    args = ap.parse_args()
    rclpy.init()
    node = VerifyHandEye(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
