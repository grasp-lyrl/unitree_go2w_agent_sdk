#!/usr/bin/env python3
import os, math, argparse, numpy as np, cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

def quat_to_R(q):
    # normalize
    n = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) or 1.0
    x,y,z,w = q.x/n, q.y/n, q.z/n, q.w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)

def pose_to_T(p: Pose):
    T = np.eye(4, dtype=float)
    T[:3,:3] = quat_to_R(p.orientation)
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T

def aruco_detect(gray, dictionary):
    if hasattr(cv.aruco, "ArucoDetector"):
        params = cv.aruco.DetectorParameters()
        det = cv.aruco.ArucoDetector(dictionary, params)
        return det.detectMarkers(gray)
    return cv.aruco.detectMarkers(gray, dictionary)

def charuco_pose(img_bgr, K, dist, board, dictionary, min_corners=16):
    H, W = img_bgr.shape[:2]
    gray = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)
    corners, ids, _ = aruco_detect(gray, dictionary)
    if ids is None or len(ids) == 0:
        return False, None, None, 0, 0.0
    ok, ch_corners, ch_ids = cv.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if not ok or ch_ids is None or len(ch_ids) < min_corners:
        return False, None, None, len(ch_ids) if ch_ids is not None else 0, 0.0
    ok_pose, rvec, tvec = cv.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, dist, None, None)
    if not ok_pose:
        return False, None, None, len(ch_ids), 0.0
    # coverage (convex hull / image area)
    xys = ch_corners.reshape(-1,2).astype(np.float32)
    hull = cv.convexHull(xys)
    cov = (cv.contourArea(hull) / float(W*H)) if hull is not None and len(hull) >= 3 else 0.0
    return True, rvec, tvec, int(ch_ids.size), cov

class Capture(Node):
    def __init__(self, args):
        super().__init__('handeye_capture_endpose')
        self.args = args
        os.makedirs(args.out, exist_ok=True)
        self.bridge = CvBridge()
        self.K = None; self.dist = None
        self.seq = 0
        self.save_next_image = False

        # latest end_pose + arrival time + previous for motion check
        self.last_pose = None
        self.last_pose_time = self.get_clock().now()
        self.prev_pose = None

        # subs
        self.create_subscription(CameraInfo, args.ci_topic, self.on_ci, 10)
        self.create_subscription(Pose,       args.ee_pose_topic, self.on_pose, 50)
        self.create_subscription(Image,      args.image_topic,   self.on_img,  10)

        # /save and /shutdown services
        self.create_service(Trigger, 'save', self.srv_save)
        self.create_service(Trigger, 'shutdown', self.srv_shutdown)

        # Charuco
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
        self.board = cv.aruco.CharucoBoard(
            (args.squares_x, args.squares_y), args.square, args.marker, self.dictionary
        )

        self.get_logger().info("Ready. Call /save to record on the *next* image.")

    def on_ci(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=float).reshape(3,3)
        self.dist = np.array(msg.d, dtype=float)
        np.savez(os.path.join(self.args.out, "intrinsics.npz"), K=self.K, dist=self.dist)

    def on_pose(self, msg: Pose):
        self.prev_pose = self.last_pose
        self.last_pose = msg
        self.last_pose_time = self.get_clock().now()

    def motion_ok(self, p0: Pose, p1: Pose, a_deg=0.5, t_m=0.002):
        if p0 is None or p1 is None: return True
        R0, R1 = quat_to_R(p0.orientation), quat_to_R(p1.orientation)
        dR = R0.T @ R1
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR)-1)/2))))
        dp = np.linalg.norm(np.array([p1.position.x-p0.position.x,
                                      p1.position.y-p0.position.y,
                                      p1.position.z-p0.position.z]))
        return (ang <= a_deg) and (dp <= t_m)

    def on_img(self, msg: Image):
        if self.K is None or self.dist is None: return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.save_next_image:
            return

        # freshness & settled
        age_ms = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e6
        if self.last_pose is None or age_ms > self.args.max_pose_age_ms:
            self.get_logger().warn(f"Reject: end_pose too old ({age_ms:.0f} ms).")
            self.save_next_image = False
            return
        if not self.motion_ok(self.prev_pose, self.last_pose):
            self.get_logger().warn("Reject: end_pose still moving.")
            self.save_next_image = False
            return

        ok, rvec, tvec, n_c, cov = charuco_pose(img, self.K, self.dist, self.board,
                                                self.dictionary, self.args.min_corners)
        if not ok or cov < self.args.min_cov:
            self.get_logger().warn(f"Reject: charuco (corners={n_c}, cov={cov*100:.0f}%).")
            self.save_next_image = False
            return

        # Build base_T_ee from Pose (flange in base_link)
        T_base_ee = pose_to_T(self.last_pose)

        # Save pair
        fn = f"{self.seq:03d}"
        cv.imwrite(os.path.join(self.args.out, f"img_{fn}.png"), img)
        np.save(os.path.join(self.args.out, f"base_T_ee_{fn}.npy"), T_base_ee)
        # also save cam_T_board from this image so you don't have to re-detect later:
        T_cam_board = np.eye(4); T_cam_board[:3,:3] = cv.Rodrigues(rvec)[0]; T_cam_board[:3,3] = tvec.ravel()
        np.save(os.path.join(self.args.out, f"cam_T_board_{fn}.npy"), T_cam_board)

        self.get_logger().info(f"Saved sample {fn} (corners={n_c}, cov={cov*100:.0f}%, age={age_ms:.0f}ms)")
        self.seq += 1
        self.save_next_image = False

    def srv_save(self, req, resp):
        self.save_next_image = True
        resp.success = True; resp.message = "Will save on next image."
        return resp

    def srv_shutdown(self, req, resp):
        resp.success = True; resp.message = "Shutting down."
        rclpy.get_global_executor().create_task(self._shutdown_async())
        return resp

    async def _shutdown_async(self): await rclpy.shutdown()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image-topic', default='/camera/color/image_raw')
    ap.add_argument('--ci-topic',    default='/camera/color/camera_info')
    ap.add_argument('--ee-pose-topic', default='/end_pose')  # Pose (flange in base_link)
    ap.add_argument('--out', default='/home/unitree/unitree_ros2/src/piper_ros/samples_charuco')
    ap.add_argument('--squares-x', type=int, default=5)
    ap.add_argument('--squares-y', type=int, default=7)
    ap.add_argument('--square', type=float, default=0.0367)
    ap.add_argument('--marker', type=float, default=0.0269)
    ap.add_argument('--min-corners', type=int, default=16)
    ap.add_argument('--min-cov', type=float, default=0.50)
    ap.add_argument('--max-pose-age-ms', type=float, default=200.0)
    args = ap.parse_args()
    rclpy.init(); node = Capture(args); rclpy.spin(node)

if __name__ == "__main__":
    main()
