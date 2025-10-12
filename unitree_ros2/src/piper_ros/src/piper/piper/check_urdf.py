#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, math, argparse
import numpy as np
from piper_sdk import C_PiperInterface_V2

def rpy_xyz_to_T(rx_deg, ry_deg, rz_deg, x_mm, y_mm, z_mm):
    """SDK says rx,ry,rz are degrees (XYZ order), position in mm → 4x4 in meters."""
    roll  = math.radians(rx_deg)
    pitch = math.radians(ry_deg)
    yaw   = math.radians(rz_deg)
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rm = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,              cp*sr,              cp*cr]
    ], dtype=float)
    T = np.eye(4)
    T[:3,:3] = Rm
    T[:3, 3] = [x_mm/1000.0, y_mm/1000.0, z_mm/1000.0]
    return T

def se3_log(T):
    R, p = T[:3,:3], T[:3,3]
    tr = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    theta = math.acos(tr)
    if theta < 1e-9:
        return np.zeros(6)
    w_hat = (R - R.T) * (theta / (2.0*math.sin(theta)))
    w = np.array([w_hat[2,1], w_hat[0,2], w_hat[1,0]])
    A = np.eye(3) - 0.5*w_hat + ((1.0/(theta**2)) - (1.0+math.cos(theta))/(2.0*theta*math.sin(theta))) * (w_hat @ w_hat)
    v = A @ p
    return np.r_[v, w]

def parse_fk_to_Ts(fk):
    """
    Accept common SDK formats and return list of 6 transforms [T_base_link1..T_base_link6].
    Each pose is [x,y,z, rx,ry,rz] with x,y,z in mm, r* in deg.
    """
    # Case: list of 6 sublists, each len 6
    if isinstance(fk, (list, tuple)) and len(fk) == 6 and all(isinstance(x, (list, tuple)) and len(x) == 6 for x in fk):
        return [rpy_xyz_to_T(rx, ry, rz, x, y, z) for (x,y,z,rx,ry,rz) in fk]

    # Case: flattened 36
    if isinstance(fk, (list, tuple)) and len(fk) == 36:
        arr = np.array(fk, dtype=float).reshape(6, 6)
        return [rpy_xyz_to_T(rx, ry, rz, x, y, z) for (x,y,z,rx,ry,rz) in arr]

    # Case: only EE pose (len 6)
    if isinstance(fk, (list, tuple)) and len(fk) == 6:
        x,y,z,rx,ry,rz = fk
        Tee = rpy_xyz_to_T(rx, ry, rz, x, y, z)
        return [None, None, None, None, None, Tee]

    # Unknown
    return None

def snap_cardinal(vec_joint, tol=0.25):
    """Snap a unit vector to nearest ±X/±Y/±Z; returns (axis_str, snapped_vec)."""
    if np.linalg.norm(vec_joint) < 1e-9:
        return None, None
    v = vec_joint / np.linalg.norm(vec_joint)
    i = int(np.argmax(np.abs(v)))
    sign = 1.0 if v[i] >= 0 else -1.0
    snapped = np.zeros(3); snapped[i] = sign
    axis_str = f"{int(snapped[0])} {int(snapped[1])} {int(snapped[2])}"
    # optional sanity: if not close to cardinal, warn (means joint frame/origin likely wrong)
    if abs(v[i]) < (1.0 - tol):
        print(f"[warn] measured axis {v} not close to cardinal in joint frame; check joint <origin rpy> upstream.")
    return axis_str, snapped

def main():
    ap = argparse.ArgumentParser(description="Verify URDF joint axis/sign from Piper SDK FK (per-link).")
    ap.add_argument("--joint", type=int, default=2, help="Joint index to verify (1..6). Child link is link<joint>.")
    ap.add_argument("--mode",  type=str, default="feedback", choices=["feedback","control"], help="FK source")
    ap.add_argument("--hz",    type=float, default=50.0, help="Sampling rate")
    ap.add_argument("--min_deg", type=float, default=1.0, help="Minimum delta angle (deg) to trigger a print")
    args = ap.parse_args()

    jidx = max(1, min(6, args.joint))      # clamp 1..6
    period = 1.0 / args.hz

    piper = C_PiperInterface_V2(dh_is_offset=1)
    piper.ConnectPort()
    piper.EnableFkCal()

    prev_Ts = None
    print(f"Watching joint{jidx} (child: link{jidx}) — jog this joint by +Δ to test sign/axis.")
    while True:
        fk = piper.GetFK(args.mode)
        Ts = parse_fk_to_Ts(fk)
        if Ts is None or Ts[jidx-1] is None:
            if prev_Ts is None:
                print("[info] FK format unknown or only EE pose available; waiting...")
            time.sleep(period)
            continue

        if prev_Ts is not None:
            T_prev = prev_Ts[jidx-1]
            T_curr = Ts[jidx-1]
            dT = np.linalg.inv(T_prev) @ T_curr
            Xi = se3_log(dT)
            w = Xi[3:]
            w_norm = float(np.linalg.norm(w))
            ang_deg = math.degrees(w_norm)

            if ang_deg >= args.min_deg:
                R_prev = T_prev[:3,:3]
                w_unit = w / (w_norm + 1e-12)

                # 1) Axis in joint (child) frame
                a_joint = R_prev.T @ w_unit
                axis_str, snapped = snap_cardinal(a_joint)

                # 2) Dot test in base vs +Z of joint (what +Z looks like right now)
                a_base_plusZ = R_prev @ np.array([0.0, 0.0, 1.0])
                dot_plusZ = float(w_unit @ a_base_plusZ)

                print("\n=== SAMPLE ===")
                print(f"Δθ ≈ {ang_deg:.2f} deg  |  ω(base) = {np.round(w_unit,4)}")
                print(f"a_measured_in_joint = {np.round(a_joint,4)}  →  suggest <axis xyz=\"{axis_str}\">")
                print(f"dot(ω, +Z_in_base_now) = {dot_plusZ:+.3f}  →  "
                      f"{'use 0 0 1' if dot_plusZ>0 else 'use 0 0 -1'} if you expect Z-axis joint")
                print("Right-hand rule: +Δ should rotate child along +axis direction.")

        prev_Ts = Ts
        time.sleep(period)

if __name__ == "__main__":
    main()
