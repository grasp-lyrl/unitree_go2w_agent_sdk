#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Solve eye-in-hand with OpenCV Charuco detections.
Inputs per sample i:
  - image:  samples/img_XXX.png
  - base_T_ee: samples/base_T_ee_XXX.npy  (4x4, gripper in base at the image time)
Also requires: samples/intrinsics.npz  (K, dist) from /camera/color/camera_info

Outputs:
  - samples/gripper_T_cam.npy   (4x4, camera pose expressed in gripper frame)
  - metrics printed to console
"""
import argparse, glob, os, math
import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R 

# ---------- utilities ----------
def aruco_detect(gray, dictionary):
    """Compatibility with both old and new OpenCV APIs."""
    if hasattr(cv.aruco, "ArucoDetector"):
        params = cv.aruco.DetectorParameters()
        det = cv.aruco.ArucoDetector(dictionary, params)
        return det.detectMarkers(gray)
    if hasattr(cv.aruco, "detectMarkers"):
        return cv.aruco.detectMarkers(gray, dictionary)
    raise RuntimeError("cv2.aruco not available. Install opencv-contrib-python.")

def estimate_target_T_cam(img_bgr, K, dist, board, dictionary, min_corners=8):
    gray = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)
    corners, ids, _ = aruco_detect(gray, dictionary)
    if ids is None or len(ids) == 0:
        return None
    ok, ch_corners, ch_ids = cv.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if not ok or ch_ids is None or len(ch_ids) < min_corners:
        return None
    ok, rvec, tvec = cv.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, dist, None, None)
    if not ok:
        return None
    R, _ = cv.Rodrigues(rvec); t = tvec.reshape(3,1)
    return R, t, len(ch_ids)

def invert_Rt(R, t):
    Rt = R.T
    tt = -Rt @ t
    return Rt, tt

def rot_angle_deg(R):
    a = (np.trace(R) - 1.0) * 0.5
    a = float(np.clip(a, -1.0, 1.0))
    return np.degrees(np.arccos(a))

def pairwise_AX_XB_rot_residuals(Rg2b_list, Rt2c_list, R_c2g):
    """Return per-frame average rotational residual (deg) from AX=XB over all pairs."""
    n = len(Rg2b_list)
    if n < 3:
        return np.full(n, np.nan)
    # For each pair (i,j), define motions:
    # A_ij = R_gb_j * R_gb_i^T
    # B_ij = R_tc_j^T * R_tc_i
    # Check: A_ij * R_cg ?= R_cg * B_ij
    R_cg = R_c2g.T
    res_sum = np.zeros(n, dtype=float)
    res_cnt = np.zeros(n, dtype=int)
    for i in range(n):
        Ri = Rg2b_list[i]; Si = Rt2c_list[i]
        for j in range(i+1, n):
            Rj = Rg2b_list[j]; Sj = Rt2c_list[j]
            A = Rj @ Ri.T
            B = Sj.T @ Si
            M = A @ R_cg @ B.T @ R_cg.T
            ang = rot_angle_deg(M)
            res_sum[i] += ang; res_cnt[i] += 1
            res_sum[j] += ang; res_cnt[j] += 1
    res = res_sum / np.maximum(res_cnt, 1)
    return res

def solve_handeye(R_g2b, t_g2b, R_t2c, t_t2c, method):
    R_c2g, t_c2g = cv.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
    # Want gripper_T_cam:
    R_g2c, t_g2c = invert_Rt(R_c2g, t_c2g)
    T = np.eye(4, dtype=float)
    T[:3,:3] = R_g2c; T[:3,3] = t_g2c.squeeze()
    return T, R_c2g, t_c2g

def mad_mask(vals, k=2.5):
    m = np.nanmedian(vals)
    mad = np.nanmedian(np.abs(vals - m)) + 1e-9
    return np.abs(vals - m) <= (k * mad)

def solve_variant(label, Rg2b, tg2b, Rt2c, tt2c):
    T_g2c, R_c2g, t_c2g = solve_handeye(Rg2b, tg2b, Rt2c, tt2c, cv.CALIB_HAND_EYE_PARK)
    per = pairwise_AX_XB_rot_residuals(Rg2b, Rt2c, R_c2g)
    med = float(np.nanmedian(per))
    print(f"[variant:{label}] median AX=XB rot residual: {med:.3f} deg")
    euler = R.from_matrix(R_c2g).as_euler('xyz', degrees=False)
    print(t_c2g,euler)
    return med, T_g2c, R_c2g, t_c2g, per

# ---------- main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', required=True, help='Directory with images & base_T_ee_*.npy & intrinsics.npz')
    ap.add_argument('--squares-x', type=int, default=5)
    ap.add_argument('--squares-y', type=int, default=7)
    ap.add_argument('--square', type=float, default=0.0367, help='measured square length (m)')
    ap.add_argument('--marker', type=float, default=0.0269, help='measured marker length (m)')
    ap.add_argument('--min-corners', type=int, default=10, help='min ChArUco corners to accept a frame')
    ap.add_argument('--reject-outliers', action='store_true')
    args = ap.parse_args()

    # Load intrinsics
    Kdist = np.load(os.path.join(args.samples, "intrinsics.npz"))
    K, dist = Kdist['K'], Kdist['dist']

    # Charuco
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
    board = cv.aruco.CharucoBoard((args.squares_x, args.squares_y), args.square, args.marker, dictionary)

    # Gather samples with proper pairing
    img_paths = sorted(glob.glob(os.path.join(args.samples, "img_*.png")))
    R_g2b, t_g2b, R_t2c, t_t2c, kept_idx, corner_counts = [], [], [], [], [], []
    for p in img_paths:
        idx = int(os.path.splitext(os.path.basename(p))[0].split('_')[-1])
        T_base_ee = np.load(os.path.join(args.samples, f"base_T_ee_{idx:03d}.npy"))
        img = cv.imread(p)
        if img is None:
            print(f"[skip] cannot read {p}")
            continue
        est = estimate_target_T_cam(img, K, dist, board, dictionary, min_corners=args.min_corners)
        if est is None:
            print(f"[skip] Charuco not detected or too few corners: {p}")
            continue
        R_tc, t_tc, n_corners = est
        # only append when BOTH are valid → keep indices aligned
        R_g2b.append(T_base_ee[:3,:3])
        t_g2b.append(T_base_ee[:3,3:4])
        R_t2c.append(R_tc)
        t_t2c.append(t_tc)
        kept_idx.append(idx)
        corner_counts.append(n_corners)

    n = len(R_t2c)
    if n < 6:
        raise RuntimeError(f"Not enough valid paired samples ({n}). Capture more views with strong wrist rotations.")

    print(f"Using {n} paired frames. Corner counts (min/median/max): "
          f"{min(corner_counts)}/{int(np.median(corner_counts))}/{max(corner_counts)}")

    # Try multiple methods and pick the lowest AX=XB residual (rotation)
    methods = [
        ("TSAI", cv.CALIB_HAND_EYE_TSAI),
        ("PARK", cv.CALIB_HAND_EYE_PARK),
        ("DANIILIDIS", cv.CALIB_HAND_EYE_DANIILIDIS),
        ("HORAUD", cv.CALIB_HAND_EYE_HORAUD),
    ]
    candidates = []
    for name, m in methods:
        try:
            T_g2c, R_c2g, t_c2g = solve_handeye(R_g2b, t_g2b, R_t2c, t_t2c, m)
            per_frame_res = pairwise_AX_XB_rot_residuals(R_g2b, R_t2c, R_c2g)
            med_res = float(np.nanmedian(per_frame_res))
            candidates.append((med_res, name, T_g2c, R_c2g, t_c2g, per_frame_res))
            print(f"[{name}] median AX=XB rot residual: {med_res:.3f} deg, "
                  f"translation ≈ {np.linalg.norm(T_g2c[:3,3]):.3f} m")
        except cv.error as e:
            print(f"[{name}] failed: {e}")

    if not candidates:
        raise RuntimeError("All hand-eye methods failed. Check inputs.")

    # Optional outlier rejection based on residuals, then re-solve with best method
    candidates.sort(key=lambda x: x[0])
    best_med, best_name, T_g2c, R_c2g, t_c2g, per_frame_res = candidates[0]
    keep_mask = np.ones(n, dtype=bool)
    if args.reject_outliers:
        mask = mad_mask(per_frame_res, k=2.5)
        dropped = np.count_nonzero(~mask)
        if dropped > 0 and (n - dropped) >= 6:
            keep_mask = mask
            print(f"Rejecting {dropped} outlier frames by AX=XB residual; re-solving with {n - dropped} frames.")
            # Re-solve with the same best method
            Rg2b_f = [R for R,k in zip(R_g2b, keep_mask) if k]
            tg2b_f = [t for t,k in zip(t_g2b, keep_mask) if k]
            Rt2c_f = [R for R,k in zip(R_t2c, keep_mask) if k]
            tt2c_f = [t for t,k in zip(t_t2c, keep_mask) if k]
            T_g2c, R_c2g, t_c2g = solve_handeye(Rg2b_f, tg2b_f, Rt2c_f, tt2c_f,
                                                dict(methods)[best_name])
            per_frame_res = pairwise_AX_XB_rot_residuals(Rg2b_f, Rt2c_f, R_c2g)
            best_med = float(np.nanmedian(per_frame_res))

    # Final result
    print("\nBest method:", best_name)
    print(f"Median AX=XB rot residual: {best_med:.3f} deg")
    print("gripper_T_cam =\n", T_g2c)

    # Save
    out_path = os.path.join(args.samples, "gripper_T_cam.npy")
    np.save(out_path, T_g2c)
    print(f"Saved: {out_path}")
    out_path = os.path.join(args.samples, "cam_T_gripper.npy")
    T = np.eye(4, dtype=float)
    T[:3,:3] = R_c2g; T[:3,3] = t_c2g.squeeze()
    np.save(out_path, T)

    # Quick sanity: camera pose in base for first kept frame
    first_idx = kept_idx[np.argmax(keep_mask)] if keep_mask.any() else kept_idx[0]
    T_base_ee0 = np.load(os.path.join(args.samples, f"base_T_ee_{first_idx:03d}.npy"))
    T_base_cam0 = T_base_ee0 @ T_g2c
    print("Example camera position (sample", first_idx, ") in base (m):", T_base_cam0[:3,3])

    # Report translation vs expected physical offset
    print(f"Estimated camera offset wrt wrist (m): {T_g2c[:3,3]}  |  norm: {np.linalg.norm(T_g2c[:3,3]):.3f}")

    
    eulerCG = R.from_matrix(R_c2g).as_euler('xyz', degrees=False)
    eulerGC = R.from_matrix(T_g2c[:3, :3]).as_euler('xyz', degrees=False)
    new = np.array([
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0]
    ])

    eulerCG = R.from_matrix(new.T)
    #np.set_printoptions(suppress=True, precision=6) 
    pitch_offset = R.from_euler('y', 10, degrees=True)
    eulerCG =  eulerCG * pitch_offset
    eulerCG = eulerCG.as_quat()

    print(f"TF Gripper->Camera:", T_g2c[:3, -1], eulerGC)
    print(f"TF Camera->Gripper:", t_c2g.tolist(), eulerCG)
    # Build the four variants correctly (if you flip a direction, also flip t)

if __name__ == "__main__":
    main()
