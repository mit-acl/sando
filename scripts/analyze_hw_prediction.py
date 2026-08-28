#!/usr/bin/env python3
# Copyright (c) 2026 MIT Aerospace Controls Laboratory. All rights reserved.
"""Hardware CV-vs-CA obstacle-prediction comparison from a recorded flight bag
+ motion-capture ground truth.

This runs the tracker's documented 9-state CONSTANT-ACCELERATION EKF on the
recorded obstacle detections (the same measurement stream the deployed tracker
saw), then rolls out BOTH predictions from the identical estimated state:

    CV: p(t+tau) = p + v_hat*tau
    CA: p(t+tau) = p + v_hat*tau + 0.5*a_hat*tau^2

with (v_hat, a_hat) from the EKF (a_hat is jointly estimated, not differentiated).
Both are scored against the mocap obstacle position at t+tau.

Fidelity is VALIDATED by comparing the reconstructed v_hat to the deployed
tracker's published velocity (read from predicted_trajs: the linear poly coeff
== pwp.coeff.c). A small mismatch means the offline EKF reproduces the deployed
estimator, so the reconstructed a_hat (and the CA roll-out) are trustworthy.

Usage:
    analyze_hw_prediction.py <bag> --gt-pose-topic /RR04_tower/world [--ns PX01]
"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

try:
    from rosbag2_py import (SequentialReader, StorageOptions, ConverterOptions,
                            StorageFilter)
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    _OK = True
except Exception as exc:  # pragma: no cover
    _OK = False
    _ERR = exc


def detect_storage_id(bag):
    meta = os.path.join(bag, "metadata.yaml")
    if os.path.exists(meta) and "mcap" in open(meta).read():
        return "mcap"
    return "sqlite3"


def _reader(bag, topic, storage_id):
    so = StorageOptions(uri=bag, storage_id=storage_id)
    co = ConverterOptions(input_serialization_format="cdr",
                          output_serialization_format="cdr")
    r = SequentialReader(); r.open(so, co)
    try:
        r.set_filter(StorageFilter(topics=[topic]))
    except Exception:
        pass
    return r


def read_predicted(bag, topic, storage_id):
    """-> list of (t, pos[3], v_recorded[3], id). Velocity/position are read
    from the pwp cubic coeffs (c=velocity, d=position) so the result is robust
    to the poly_coeffs degree (the deployed bag uses a 6-coeff quintic)."""
    T = get_message("dynus_interfaces/msg/DynTraj")   # load dynus typesupport first
    r = _reader(bag, topic, storage_id)
    out = []
    while r.has_next():
        tp, d, _ = r.read_next()
        if tp != topic:
            continue
        m = deserialize_message(d, T)
        t = m.poly_start_time if m.poly_start_time > 0 else (
            m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if len(m.pwp.coeff_x) > 0:
            cx, cy, cz = m.pwp.coeff_x[0], m.pwp.coeff_y[0], m.pwp.coeff_z[0]
            vel = np.array([cx.c, cy.c, cz.c])
            pos = np.array([cx.d, cy.d, cz.d])
        else:  # fallback: linear coeff is 2nd-to-last (highest-power-first)
            vel = np.array([m.poly_coeffs_x[-2], m.poly_coeffs_y[-2], m.poly_coeffs_z[-2]])
            pos = np.array([m.pos.x, m.pos.y, m.pos.z])
        out.append((t, pos, vel, int(m.id)))
    out.sort(key=lambda r: r[0])
    return out


def read_pose_gt(bag, topic, storage_id):
    T = get_message("geometry_msgs/msg/PoseStamped")
    r = _reader(bag, topic, storage_id)
    rows = []
    while r.has_next():
        tp, d, _ = r.read_next()
        if tp != topic:
            continue
        m = deserialize_message(d, T)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rows.append((t, m.pose.position.x, m.pose.position.y, m.pose.position.z))
    rows.sort(key=lambda r: r[0])
    t = np.array([r[0] for r in rows])
    xyz = np.array([[r[1], r[2], r[3]] for r in rows])
    keep = np.concatenate([np.diff(t) > 1e-9, [True]]) if len(t) > 1 else [True]
    return t[keep], xyz[keep]


def gt_at(gt_t, gt_xyz, q):
    valid = (q >= gt_t[0]) & (q <= gt_t[-1])
    out = np.full((len(q), 3), np.nan)
    if valid.any():
        qq = q[valid]
        for k in range(3):
            out[valid, k] = np.interp(qq, gt_t, gt_xyz[:, k])
    return out, valid


class CAEKF:
    """9-state constant-acceleration adaptive EKF (port of obstacle_tracker.cc:
    ekfPredict + aekfUpdate). State = [x y z vx vy vz ax ay az]."""
    def __init__(self, z0, t0, alpha=0.90):
        self.x = np.zeros(9); self.x[:3] = z0
        self.P = np.eye(9)
        self.Q = np.eye(9) * 0.01
        self.R = np.eye(3) * 0.1
        self.t = t0
        self.alpha = alpha
        self.H = np.zeros((3, 9)); self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0

    def step(self, z, t):
        dt = t - self.t
        if dt < 1e-6 or dt > 1.0:      # clamp pathological gaps
            dt = 0.1
        F = np.eye(9)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        F[0, 6] = F[1, 7] = F[2, 8] = 0.5 * dt * dt
        F[3, 6] = F[4, 7] = F[5, 8] = dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        H = self.H
        d = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ d
        eps = z - H @ self.x
        self.R = self.alpha * self.R + (1 - self.alpha) * (np.outer(eps, eps) + H @ self.P @ H.T)
        self.Q = self.alpha * self.Q + (1 - self.alpha) * (K @ np.outer(d, d) @ K.T)
        self.P = (np.eye(9) - K @ H) @ self.P
        self.t = t


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--ns", default="PX01")
    ap.add_argument("--pred-topic", default=None)
    ap.add_argument("--gt-pose-topic", required=True)
    ap.add_argument("--horizon", type=float, default=3.0)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--match-dist", type=float, default=1.5)
    ap.add_argument("--label", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    if not _OK:
        print(f"ERROR: rosbag2_py unavailable ({_ERR})", file=sys.stderr)
        return 2

    bag = args.bag.rstrip("/")
    sid = detect_storage_id(bag)
    pred_topic = args.pred_topic or f"/{args.ns}/predicted_trajs"
    label = args.label or os.path.basename(bag)

    preds = read_predicted(bag, pred_topic, sid)
    gt_t, gt_xyz = read_pose_gt(bag, args.gt_pose_topic, sid)
    print(f"[INFO] {label}: {len(preds)} preds, {len(gt_t)} mocap GT samples")

    # Clean obstacle measurement stream: nearest-GT detection per time, in order.
    raw = []
    for t, pos, vrec, tid in preds:
        g, ok = gt_at(gt_t, gt_xyz, np.array([t]))
        if ok[0]:
            dist = np.linalg.norm(pos - g[0])
            if dist <= args.match_dist:
                raw.append((t, pos, vrec, dist))
    raw.sort(key=lambda r: r[0])
    obs = []  # dedup near-simultaneous detections, keep nearest GT
    for t, pos, vrec, dist in raw:
        if obs and (t - obs[-1][0]) < 5e-3:
            if dist < obs[-1][3]:
                obs[-1] = (t, pos, vrec, dist)
        else:
            obs.append((t, pos, vrec, dist))
    print(f"[INFO] {len(obs)} obstacle detections (<= {args.match_dist} m from mocap)")
    if len(obs) < 20:
        print("ERROR: too few obstacle detections", file=sys.stderr)
        return 2

    t_all = np.array([o[0] for o in obs])
    pos_all = np.array([o[1] for o in obs])
    vrec_all = np.array([o[2] for o in obs])

    # Run the 9-state CA-EKF on the obstacle centroid stream
    ekf = CAEKF(pos_all[0], t_all[0])
    v_ekf = np.zeros_like(pos_all)
    a_ekf = np.zeros_like(pos_all)
    for k in range(len(obs)):
        ekf.step(pos_all[k], t_all[k])
        v_ekf[k] = ekf.x[3:6]
        a_ekf[k] = ekf.x[6:9]

    # VALIDATION: reconstructed velocity vs deployed published velocity
    vdiff = np.linalg.norm(v_ekf - vrec_all, axis=1)
    print(f"[VALIDATION] |v_ekf - v_deployed|: median={np.median(vdiff):.3f} "
          f"mean={np.mean(vdiff):.3f} m/s   "
          f"(|v_ekf| mean={np.mean(np.linalg.norm(v_ekf,axis=1)):.3f}, "
          f"|v_deployed| mean={np.mean(np.linalg.norm(vrec_all,axis=1)):.3f})")
    print(f"[INFO] |a_ekf| median={np.median(np.linalg.norm(a_ekf,axis=1)):.2f} "
          f"p90={np.percentile(np.linalg.norm(a_ekf,axis=1),90):.2f} m/s^2")

    # Roll out CV and CA from the SAME EKF state; score vs mocap GT.
    taus = np.arange(0.0, args.horizon + 1e-9, args.dt)
    cv_e = [[] for _ in taus]
    ca_e = [[] for _ in taus]
    for k in range(len(obs)):
        z, v, a = pos_all[k], v_ekf[k], a_ekf[k]
        g, valid = gt_at(gt_t, gt_xyz, t_all[k] + taus)
        cv = z[None, :] + np.outer(taus, v)
        ca = cv + 0.5 * np.outer(taus**2, a)
        for i in range(len(taus)):
            if valid[i]:
                cv_e[i].append(np.linalg.norm(cv[i] - g[i]))
                ca_e[i].append(np.linalg.norm(ca[i] - g[i]))

    def rms(e):
        return np.array([np.sqrt(np.mean(np.square(x))) if x else np.nan for x in e])
    cv_rms, ca_rms = rms(cv_e), rms(ca_e)
    n = np.array([len(x) for x in cv_e])

    print(f"\n=== {label}: CV vs CA prediction RMS error [m] vs mocap GT ===")
    print(f"{'tau[s]':>6} {'CV':>7} {'CA':>7} {'CA-CV':>8} {'better':>7} {'n':>7}")
    for probe in (0.5, 1.0, 1.5, 2.0, 3.0):
        if probe > args.horizon + 1e-6:
            continue
        i = min(int(round(probe / args.dt)), len(taus) - 1)
        if np.isnan(cv_rms[i]):
            continue
        better = "CA" if ca_rms[i] < cv_rms[i] else "CV"
        print(f"{taus[i]:>6.2f} {cv_rms[i]:>7.3f} {ca_rms[i]:>7.3f} "
              f"{ca_rms[i]-cv_rms[i]:>8.3f} {better:>7} {n[i]:>7}")

    out_dir = args.out or os.path.join(bag, "hw_prediction_accuracy")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "error_vs_horizon.csv"), "w") as f:
        f.write("horizon_s,cv_rms,ca_rms,n\n")
        for i, tau in enumerate(taus):
            f.write(f"{tau:.3f},{cv_rms[i]:.4f},{ca_rms[i]:.4f},{n[i]}\n")

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(taus, cv_rms, "-o", ms=3, color="tab:blue", label="CV (pos+v·t)")
    ax.plot(taus, ca_rms, "-o", ms=3, color="tab:red", label="CA (pos+v·t+½a·t²)")
    ax.set_xlabel("look-ahead τ [s]"); ax.set_ylabel("RMS position error vs mocap [m]")
    ax.set_title(f"HW {label}: CV vs CA obstacle prediction")
    ax.grid(True, alpha=0.3); ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "error_vs_horizon.png"), dpi=150)
    plt.close(fig)
    print(f"[OK] outputs in {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
