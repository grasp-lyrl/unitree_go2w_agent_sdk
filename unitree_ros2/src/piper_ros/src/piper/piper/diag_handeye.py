#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse, glob, os, math
import numpy as np
import cv2 as cv

def aruco_detect(gray, dictionary):
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
    n = len(Rg2b_list)
    if n < 3:
        return np.full(n, np.nan)
    R_cg = R_c2g.T
    res_sum = np.zeros(n); res_cnt = np.zeros(n, dtype=int)
    for i in range(n):
        Ri = Rg2b_list[i]; Si = Rt2c_list[i]
        for j in range(i+1, n):
            Rj = Rg2b_list[j]; Sj = Rt2c_list[j]
            A = Rj @ Ri.T            # gripper motion
            B = Sj.T @ Si            # camera motion about target
            M = A @ R_cg @ B.T @ R_cg.T
            ang = rot_angle_deg(M)
            res_sum[i] += ang; res_cnt[i] += 1
            res_sum[j] += ang; res_cnt[j] += 1
    return res_sum / np.maximum(res_cnt, 1)

def solve_handeye(R_g2b, t_g2b, R_t2c, t_t2c, method):
    R_c2g, t_c2g = cv.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
    R_g2c, t_g2c = invert_Rt(R_c2g, t_c2g)   # want gripper_T_cam
    T = np.eye(4, dtype=float); T[:3,:3] = R_g2c; T[:3,3] = t_g2c.squeeze()
    return T, R_c2g, t_c2g

def mad_mask(vals, k=2.5):
    m = np.nanmedian(vals); mad = np.nanmedian(np.abs(vals - m)) + 1e-9
    return np.abs(vals - m) <= (k * mad)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', required=True)
    ap.add_argument('--squares-x', type=int, default=5)
    ap.add_argument('--squares-y', type=int, default=7)
    ap.add_argument('--square', type=float, default=0.035)
    ap.add_argument('--marker', type=float, default=0.024)
    ap.add_argument('--min-corners', type=int, default=10)
    ap.add_argument('--reject-outliers', action='store_true')
    args = ap.parse_args()

    # Load intrinsics
    Kdist = np.load(os.path.join(args.samples, "intrinsics.npz"))
    K, dist = Kdist['K'], Kdist['dist']
    print(f"K=\n{K}\nD={dist}")

    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
    board = cv.aruco.CharucoBoard((args.squares_x, args.squares_y), args.square, args.marker, dictionary)

    # Pairing
    img_paths = sorted(glob.glob(os.path.join(args.samples, "img_*.png")))
    R_g2b, t_g2b, R_t2c, t_t2c, kept_idx, wh_list, corner_counts = [], [], [], [], [], [], []
    for p in img_paths:
        idx = int(os.path.splitext(os.path.basename(p))[0].split('_')[-1])
        T_base_ee = np.load(os.path.join(args.samples, f"base_T_ee_{idx:03d}.npy"))
        img = cv.imread(p)
        if img is None: print(f"[skip] cannot read {p}"); continue
        h,w = img.shape[:2]; wh_list.append((w,h))
        est = estimate_target_T_cam(img, K, dist, board, dictionary, min_corners=args.min_corners)
        if est is None:
            print(f"[skip] Charuco not detected or too few corners: {p}")
            continue
        R_tc, t_tc, n_c = est
        R_g2b.append(T_base_ee[:3,:3]); t_g2b.append(T_base_ee[:3,3:4])
        R_t2c.append(R_tc); t_t2c.append(t_tc)
        kept_idx.append(idx); corner_counts.append(n_c)

    if not kept_idx:
        raise RuntimeError("No valid paired frames.")
    sizes = set(wh_list)
    print(f"Image sizes seen: {sizes}")
    if len(sizes) > 1:
        print("WARNING: mixed image resolutions; re-capture with a single resolution.")
    n = len(R_t2c)
    print(f"Using {n} paired frames. Corner counts min/median/max: "
          f"{min(corner_counts)}/{int(np.median(corner_counts))}/{max(corner_counts)}")

    # Motion diversity: wrist
    def motion_angles(R_list):
        angs = []
        for i in range(1,len(R_list)):
            dR = R_list[i] @ R_list[i-1].T
            angs.append(rot_angle_deg(dR))
        return np.array(angs) if angs else np.array([0.0])
    wrist_rot = motion_angles(R_g2b)
    print(f"Wrist rotation between consecutive frames: mean={wrist_rot.mean():.1f}°, "
          f"max={wrist_rot.max():.1f}°")

    methods = [
        ("TSAI", cv.CALIB_HAND_EYE_TSAI),
        ("PARK", cv.CALIB_HAND_EYE_PARK),
        ("DANIILIDIS", cv.CALIB_HAND_EYE_DANIILIDIS),
        ("HORAUD", cv.CALIB_HAND_EYE_HORAUD),
    ]

    def try_one(direction_name, Rgb, tgb):
        print(f"\n=== Direction: {direction_name} ===")
        cands = []
        for name, m in methods:
            try:
                T_g2c, R_c2g, t_c2g = solve_handeye(Rgb, tgb, R_t2c, t_t2c, m)
                res = pairwise_AX_XB_rot_residuals(Rgb, R_t2c, R_c2g)
                med = float(np.nanmedian(res))
                print(f"[{name}] median AX=XB rot residual: {med:.3f}°, "
                      f"||t||={np.linalg.norm(T_g2c[:3,3]):.3f} m, t={T_g2c[:3,3]}")
                cands.append((med, name, T_g2c, R_c2g, t_c2g, res))
            except cv.error as e:
                print(f"[{name}] failed: {e}")
        if not cands:
            return None
        cands.sort(key=lambda x: x[0])
        best = cands[0]
        # Optional outlier rejection
        if args.reject_outliers:
            mask = mad_mask(best[5], k=2.5)
            dropped = np.count_nonzero(~mask)
            if dropped > 0 and (n - dropped) >= 6:
                print(f"Rejecting {dropped} outliers; re-solving with {n - dropped} frames.")
                Rgb_f = [R for R,k in zip(Rgb, mask) if k]
                tgb_f = [t for t,k in zip(tgb, mask) if k]
                Rt2c_f= [R for R,k in zip(R_t2c, mask) if k]
                tt2c_f= [t for t,k in zip(t_t2c, mask) if k]
                # re-run same best method name
                method = dict(methods)[best[1]]
                T_g2c2, R_c2g2, t_c2g2 = solve_handeye(Rgb_f, tgb_f, Rt2c_f, tt2c_f, method)
                res2 = pairwise_AX_XB_rot_residuals(Rgb_f, Rt2c_f, R_c2g2)
                print(f"[{best[1]}] post-reject median residual: {np.nanmedian(res2):.3f}°, "
                      f"||t||={np.linalg.norm(T_g2c2[:3,3]):.3f} m, t={T_g2c2[:3,3]}")
                return (np.nanmedian(res2), best[1], T_g2c2, R_c2g2, t_c2g2, mask)
        return best

    # Direction A: what you have (gripper in base)
    best_A = try_one("gripper2base", R_g2b, t_g2b)

    # Direction B: invert to base2gripper (some stacks require this)
    R_b2g = [R.T for R in R_g2b]
    t_b2g = [-(R.T @ t) for R, t in zip(R_g2b, t_g2b)]
    best_B = try_one("base2gripper (inverted)", R_b2g, t_b2g)

    # Pick the better one
    picks = [b for b in [best_A, best_B] if b is not None]
    if not picks:
        raise RuntimeError("All solves failed.")
    picks.sort(key=lambda x: x[0])
    med_res, name, T_g2c, R_c2g, t_c2g, mask = picks[0]
    print("\n=== PICKED ===")
    print(f"Method={name}, median residual={med_res:.3f}°, t={T_g2c[:3,3]}, ||t||={np.linalg.norm(T_g2c[:3,3]):.3f} m")
    np.save(os.path.join(args.samples, "gripper_T_cam.npy"), T_g2c)
    print("Saved:", os.path.join(args.samples, "gripper_T_cam.npy"))

    # Board fixed check: compute base_T_target for each kept frame; stddev should be small if fixed
    keep_mask = mask if isinstance(mask, np.ndarray) else np.ones(n, bool)
    Rt = [R for R,k in zip(R_t2c, keep_mask) if k]
    tt = [t for t,k in zip(t_t2c, keep_mask) if k]
    Rg = [R for R,k in zip(R_g2b, keep_mask) if k]
    tg = [t for t,k in zip(t_g2b, keep_mask) if k]

    # cam_T_target = inverse(target_T_cam)
    cam_T_target = [np.block([[R.T, -R.T @ t],[0,0,0,1]]) for R,t in zip(Rt, tt)]
    gripper_T_cam = T_g2c
    base_T_target_list = []
    kept_idx = [i for i,k in enumerate(keep_mask) if k]
    for i,(Rgb, tgb, cTt) in enumerate(zip(Rg, tg, cam_T_target)):
        base_T_gripper = np.eye(4); base_T_gripper[:3,:3]=Rgb; base_T_gripper[:3,3]=tgb.squeeze()
        base_T_target = base_T_gripper @ gripper_T_cam @ cTt
        base_T_target_list.append(base_T_target)
    pts = np.array([T[:3,3] for T in base_T_target_list])
    mean = pts.mean(axis=0); std = pts.std(axis=0)
    print(f"Base_T_target translation mean (m): {mean}, stddev (m): {std}  (want std ≪ 0.01 if board fixed)")

    # Quick hint if things are still bad
    if np.linalg.norm(T_g2c[:3,3]) > 0.3 or med_res > 5.0 or np.max(std) > 0.02:
        print("\n>>> DIAGNOSIS HINTS:")
        print("- Ensure the ChArUco board was RIGID relative to the world during ALL captures.")
        print("- Verify --square/--marker are in METERS and measured accurately.")
        print("- Confirm images and CameraInfo are SAME resolution; re-capture intrinsics if not.")
        print("- Increase wrist ROTATION diversity (yaw/pitch/roll), not just translations.")
        print("- If the camera is on the wrist, use the wrist link as EE (your link6).")
        print("- Keep the robot base perfectly still during capture.")
        print("- If using depth alignment later, make sure 3D points are in the COLOR camera frame you calibrated.")

if __name__ == "__main__":
    main()
