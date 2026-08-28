#!/usr/bin/env python3
# Copyright (c) 2026 MIT Aerospace Controls Laboratory. All rights reserved.
"""Compare constant-velocity (CV) vs constant-acceleration (CA) obstacle
prediction accuracy from a ``prediction-eval`` rosbag.

The ``prediction-eval`` mode of ``run_sim.py`` records, in a single run:
  * ``/NX01/predicted_trajs_cv`` - CV roll-out  p(u)=pos+vel*u
  * ``/NX01/predicted_trajs_ca`` - CA roll-out  p(u)=pos+vel*u+0.5*acc*u^2
both emitted from the SAME EKF state (so the comparison is perfectly paired),
plus the ground-truth obstacle trajectory on ``/trajs_ground_truth``.

For every prediction emitted at model time ``t0`` (``poly_start_time``), this
script reconstructs the predicted obstacle position at look-ahead times
``t0 + tau`` (tau in [0, horizon]) and compares it to the ground-truth obstacle
position at the same absolute time, interpolated from ``/trajs_ground_truth``.
It then aggregates the position error as a function of the look-ahead horizon
and writes a CSV plus plots.

Usage:
    python3 analyze_prediction_accuracy.py <bag_path> [--ns NX01] [--horizon 2.0]

Notes on time base: both the tracker (mapper) and the ground-truth node are
launched with ``use_sim_time:=true`` so their stamps share the Gazebo /clock.
``poly_start_time`` (predictions) and ``header.stamp`` (ground truth) are
therefore directly comparable. The script prints a warning if their time ranges
do not overlap (the usual cause is a missing ``use_sim_time``).
"""

import argparse
import os
import sys

import numpy as np

# Headless plotting
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

try:
    from rosbag2_py import (
        SequentialReader,
        StorageOptions,
        ConverterOptions,
        StorageFilter,
    )
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    _ROSBAG_OK = True
except Exception as exc:  # pragma: no cover
    _ROSBAG_OK = False
    _IMPORT_ERR = exc


DYNTRAJ_TYPE = "dynus_interfaces/msg/DynTraj"


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------
def detect_storage_id(bag_path):
    """Best-effort storage backend detection from metadata.yaml."""
    meta = os.path.join(bag_path, "metadata.yaml")
    if os.path.exists(meta):
        try:
            with open(meta) as f:
                text = f.read()
            if "mcap" in text:
                return "mcap"
            if "sqlite3" in text:
                return "sqlite3"
        except Exception:
            pass
    # Fall back on file extensions
    for fn in os.listdir(bag_path) if os.path.isdir(bag_path) else []:
        if fn.endswith(".mcap"):
            return "mcap"
        if fn.endswith(".db3"):
            return "sqlite3"
    return "sqlite3"


def list_topics(bag_path, storage_id):
    so = StorageOptions(uri=bag_path, storage_id=storage_id)
    co = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(so, co)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_dyntraj_topics(bag_path, topics, storage_id):
    """Read the given DynTraj topics. Returns {topic: [(stamp_sec, msg), ...]}."""
    msg_type = get_message(DYNTRAJ_TYPE)
    so = StorageOptions(uri=bag_path, storage_id=storage_id)
    co = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(so, co)
    try:
        reader.set_filter(StorageFilter(topics=list(topics)))
    except Exception:
        pass

    out = {t: [] for t in topics}
    wanted = set(topics)
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(data, msg_type)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out[topic].append((stamp, recv_ns * 1e-9, msg))
    return out


# ---------------------------------------------------------------------------
# Ground-truth handling
# ---------------------------------------------------------------------------
def build_ground_truth(gt_records):
    """From [(stamp, recv, msg)] pick the single dominant obstacle and return
    sorted arrays (t, xyz) using sim-time stamps."""
    by_id = {}
    for stamp, _recv, msg in gt_records:
        if getattr(msg, "is_agent", False):
            continue
        if stamp <= 0.0:  # stamp not yet valid (clock not received)
            continue
        by_id.setdefault(msg.id, []).append(
            (stamp, msg.pos.x, msg.pos.y, msg.pos.z)
        )
    if not by_id:
        return None, None, None
    # Dominant obstacle = most samples
    obs_id = max(by_id, key=lambda k: len(by_id[k]))
    rows = sorted(by_id[obs_id], key=lambda r: r[0])
    t = np.array([r[0] for r in rows])
    xyz = np.array([[r[1], r[2], r[3]] for r in rows])
    # De-duplicate identical timestamps (keep last) for np.interp monotonicity
    keep = np.concatenate([np.diff(t) > 1e-9, [True]])
    return obs_id, t[keep], xyz[keep]


def read_pose_topic(bag_path, topic, storage_id):
    """Read a geometry_msgs/PoseStamped GT topic (e.g. mocap /RR04_tower/world)
    -> sorted (t, xyz) using the header stamp (shares the bag /clock with the
    replay)."""
    msg_type = get_message("geometry_msgs/msg/PoseStamped")
    so = StorageOptions(uri=bag_path, storage_id=storage_id)
    co = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(so, co)
    try:
        reader.set_filter(StorageFilter(topics=[topic]))
    except Exception:
        pass
    rows = []
    while reader.has_next():
        tp, data, _ = reader.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, msg_type)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            continue
        rows.append((stamp, m.pose.position.x, m.pose.position.y, m.pose.position.z))
    if len(rows) < 2:
        return None, None, None
    rows.sort(key=lambda r: r[0])
    t = np.array([r[0] for r in rows])
    xyz = np.array([[r[1], r[2], r[3]] for r in rows])
    keep = np.concatenate([np.diff(t) > 1e-9, [True]])
    return "(pose-gt)", t[keep], xyz[keep]


def gt_at(gt_t, gt_xyz, query):
    """Interpolate ground-truth position at absolute times ``query`` (array).
    Returns (xyz[N,3], valid_mask) where valid = inside the observed window."""
    valid = (query >= gt_t[0]) & (query <= gt_t[-1])
    out = np.full((len(query), 3), np.nan)
    if valid.any():
        q = query[valid]
        out[valid, 0] = np.interp(q, gt_t, gt_xyz[:, 0])
        out[valid, 1] = np.interp(q, gt_t, gt_xyz[:, 1])
        out[valid, 2] = np.interp(q, gt_t, gt_xyz[:, 2])
    return out, valid


# ---------------------------------------------------------------------------
# Prediction handling
# ---------------------------------------------------------------------------
def pred_pos(msg, tau):
    """Evaluate a DynTraj poly_coeffs (highest-power-first) at relative time
    ``tau`` (array). Returns xyz[len(tau), 3]."""
    cx = np.asarray(msg.poly_coeffs_x, dtype=float)
    cy = np.asarray(msg.poly_coeffs_y, dtype=float)
    cz = np.asarray(msg.poly_coeffs_z, dtype=float)
    return np.stack(
        [np.polyval(cx, tau), np.polyval(cy, tau), np.polyval(cz, tau)], axis=1
    )


def accumulate_errors(pred_records, gt_t, gt_xyz, taus, match_dist):
    """For each prediction, compute ||pred - gt|| at each look-ahead in ``taus``.
    Returns errors[len(taus)] -> list, plus per-axis abs-error lists and the
    list of (t0, msg) actually used (for snapshots)."""
    errs = [[] for _ in taus]
    ax_errs = [[] for _ in taus]  # (dx,dy,dz) abs
    used = []
    for _stamp, _recv, msg in pred_records:
        t0 = float(msg.poly_start_time)
        if t0 <= 0.0:
            continue
        # Single-obstacle gating: prediction start (tau=0) must be near GT.
        p0 = pred_pos(msg, np.array([0.0]))[0]
        g0, ok0 = gt_at(gt_t, gt_xyz, np.array([t0]))
        if not ok0[0] or np.linalg.norm(p0 - g0[0]) > match_dist:
            continue
        q = t0 + taus
        g, valid = gt_at(gt_t, gt_xyz, q)
        p = pred_pos(msg, taus)
        d = p - g
        used.append((t0, msg))
        for i in range(len(taus)):
            if valid[i]:
                errs[i].append(float(np.linalg.norm(d[i])))
                ax_errs[i].append(np.abs(d[i]))
    return errs, ax_errs, used


def stats(per_tau):
    """mean, rms, p50, p90, n per look-ahead bin."""
    mean = np.array([np.mean(e) if e else np.nan for e in per_tau])
    rms = np.array(
        [np.sqrt(np.mean(np.square(e))) if e else np.nan for e in per_tau]
    )
    p50 = np.array([np.percentile(e, 50) if e else np.nan for e in per_tau])
    p90 = np.array([np.percentile(e, 90) if e else np.nan for e in per_tau])
    n = np.array([len(e) for e in per_tau])
    return mean, rms, p50, p90, n


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="Path to the rosbag directory")
    ap.add_argument("--ns", default="NX01", help="Drone namespace (default: NX01)")
    ap.add_argument("--gt-topic", default=None,
                    help="Ground-truth DynTraj topic (default: /trajs_ground_truth, "
                         "falling back to /trajs)")
    ap.add_argument("--gt-pose-topic", default=None,
                    help="Ground-truth geometry_msgs/PoseStamped topic (e.g. mocap "
                         "/RR04_tower/world). Overrides --gt-topic when set.")
    ap.add_argument("--cv-topic", default=None,
                    help="CV prediction topic (default: /<ns>/predicted_trajs_cv)")
    ap.add_argument("--ca-topic", default=None,
                    help="CA prediction topic (default: /<ns>/predicted_trajs_ca)")
    ap.add_argument("--horizon", type=float, default=None,
                    help="Max look-ahead [s] (default: auto from messages)")
    ap.add_argument("--dt", type=float, default=0.1,
                    help="Look-ahead sampling step [s] (default: 0.1)")
    ap.add_argument("--match-dist", type=float, default=2.0,
                    help="Max dist [m] from GT to accept a prediction (rejects "
                         "ghost tracks; single-obstacle assumption) (default: 2.0)")
    ap.add_argument("--gt-z-offset", type=float, default=0.0,
                    help="Add this [m] to the GT z (e.g. -0.8 to move a head-top "
                         "mocap marker down to the detected body centroid)")
    ap.add_argument("--snapshots", type=int, default=6,
                    help="Number of XY snapshot panels (default: 6)")
    ap.add_argument("--out", default=None,
                    help="Output directory (default: <bag>/prediction_accuracy)")
    args = ap.parse_args()

    if not _ROSBAG_OK:
        print(f"ERROR: rosbag2_py unavailable ({_IMPORT_ERR}). "
              "Source your ROS 2 + workspace setup.bash first.", file=sys.stderr)
        return 2

    bag = args.bag.rstrip("/")
    if not os.path.isdir(bag):
        print(f"ERROR: bag path is not a directory: {bag}", file=sys.stderr)
        return 2

    storage_id = detect_storage_id(bag)
    topics_in_bag = list_topics(bag, storage_id)

    cv_topic = args.cv_topic or f"/{args.ns}/predicted_trajs_cv"
    ca_topic = args.ca_topic or f"/{args.ns}/predicted_trajs_ca"
    use_pose_gt = bool(args.gt_pose_topic)
    if use_pose_gt:
        gt_topic = args.gt_pose_topic
    elif args.gt_topic:
        gt_topic = args.gt_topic
    elif "/trajs_ground_truth" in topics_in_bag:
        gt_topic = "/trajs_ground_truth"
    else:
        gt_topic = "/trajs"

    for label, tp in [("CV", cv_topic), ("CA", ca_topic), ("ground truth", gt_topic)]:
        if tp not in topics_in_bag:
            print(f"ERROR: {label} topic '{tp}' not found in bag.\n"
                  f"Available topics:\n  " + "\n  ".join(sorted(topics_in_bag)),
                  file=sys.stderr)
            return 2

    print(f"[INFO] bag={bag} (storage={storage_id})")
    print(f"[INFO] GT={gt_topic}  CV={cv_topic}  CA={ca_topic}")

    data = read_dyntraj_topics(bag, [cv_topic, ca_topic], storage_id)
    if use_pose_gt:
        obs_id, gt_t, gt_xyz = read_pose_topic(bag, gt_topic, storage_id)
    else:
        gtdata = read_dyntraj_topics(bag, [gt_topic], storage_id)
        obs_id, gt_t, gt_xyz = build_ground_truth(gtdata[gt_topic])
    if gt_xyz is not None and args.gt_z_offset != 0.0:
        gt_xyz = gt_xyz.copy()
        gt_xyz[:, 2] += args.gt_z_offset
        print(f"[INFO] applied GT z-offset {args.gt_z_offset:+.2f} m")
    if gt_t is None or len(gt_t) < 2:
        print("ERROR: no usable ground-truth obstacle messages (need >=2 with a "
              "valid sim-time stamp). Was use_sim_time set and /clock running?",
              file=sys.stderr)
        return 2
    print(f"[INFO] ground-truth obstacle id={obs_id}, "
          f"{len(gt_t)} samples, t in [{gt_t[0]:.2f}, {gt_t[-1]:.2f}] s")

    cv_rec = data[cv_topic]
    ca_rec = data[ca_topic]
    print(f"[INFO] {len(cv_rec)} CV predictions, {len(ca_rec)} CA predictions")
    if not cv_rec or not ca_rec:
        print("ERROR: no prediction messages on one of the topics.", file=sys.stderr)
        return 2

    # Time-overlap sanity check
    pred_t0 = np.array([m.poly_start_time for _s, _r, m in cv_rec if m.poly_start_time > 0])
    if pred_t0.size:
        lo, hi = pred_t0.min(), pred_t0.max()
        if hi < gt_t[0] or lo > gt_t[-1]:
            print("WARNING: prediction times "
                  f"[{lo:.2f}, {hi:.2f}] do NOT overlap ground-truth times "
                  f"[{gt_t[0]:.2f}, {gt_t[-1]:.2f}].\n"
                  "         Predictions and ground truth are likely on different "
                  "clocks (use_sim_time mismatch). Results will be empty.",
                  file=sys.stderr)

    # Look-ahead grid
    horizon = args.horizon
    if horizon is None:
        hs = [float(m.poly_end_time - m.poly_start_time)
              for _s, _r, m in cv_rec if m.poly_end_time > m.poly_start_time]
        horizon = float(np.median(hs)) if hs else 2.0
    taus = np.arange(0.0, horizon + 1e-9, args.dt)
    print(f"[INFO] look-ahead horizon={horizon:.2f}s, {len(taus)} samples @ {args.dt}s")

    cv_errs, cv_ax, cv_used = accumulate_errors(cv_rec, gt_t, gt_xyz, taus, args.match_dist)
    ca_errs, ca_ax, ca_used = accumulate_errors(ca_rec, gt_t, gt_xyz, taus, args.match_dist)

    cv_mean, cv_rms, cv_p50, cv_p90, cv_n = stats(cv_errs)
    ca_mean, ca_rms, ca_p50, ca_p90, ca_n = stats(ca_errs)

    if np.all(cv_n == 0):
        print("ERROR: no prediction/ground-truth pairs matched. Check "
              "--match-dist and the time-overlap warning above.", file=sys.stderr)
        return 2

    out_dir = args.out or os.path.join(bag, "prediction_accuracy")
    os.makedirs(out_dir, exist_ok=True)

    # --- CSV ---
    csv_path = os.path.join(out_dir, "error_vs_horizon.csv")
    with open(csv_path, "w") as f:
        f.write("horizon_s,cv_mean_m,cv_rms_m,cv_p50_m,cv_p90_m,cv_n,"
                "ca_mean_m,ca_rms_m,ca_p50_m,ca_p90_m,ca_n\n")
        for i, tau in enumerate(taus):
            f.write(f"{tau:.3f},{cv_mean[i]:.4f},{cv_rms[i]:.4f},{cv_p50[i]:.4f},"
                    f"{cv_p90[i]:.4f},{cv_n[i]},{ca_mean[i]:.4f},{ca_rms[i]:.4f},"
                    f"{ca_p50[i]:.4f},{ca_p90[i]:.4f},{ca_n[i]}\n")
    print(f"[OK] wrote {csv_path}")

    # --- Summary table ---
    print("\n=== CV vs CA prediction error (RMS, meters) ===")
    print(f"{'horizon[s]':>10} {'CV_rms':>9} {'CA_rms':>9} {'CA-CV':>9} {'better':>8} {'n':>6}")
    for probe in [0.5, 1.0, 1.5, 2.0]:
        if probe > horizon + 1e-6:
            continue
        i = int(round(probe / args.dt))
        i = min(i, len(taus) - 1)
        cvv, cav = cv_rms[i], ca_rms[i]
        if np.isnan(cvv) or np.isnan(cav):
            continue
        better = "CA" if cav < cvv else "CV"
        print(f"{taus[i]:>10.2f} {cvv:>9.3f} {cav:>9.3f} {cav - cvv:>9.3f} "
              f"{better:>8} {cv_n[i]:>6}")

    # --- Plot 1: error vs horizon ---
    _plot_error_vs_horizon(taus, cv_mean, cv_rms, cv_p90, ca_mean, ca_rms, ca_p90,
                           os.path.join(out_dir, "error_vs_horizon.png"))
    print(f"[OK] wrote {os.path.join(out_dir, 'error_vs_horizon.png')}")

    # --- Plot 2: per-axis RMS ---
    _plot_per_axis(taus, cv_ax, ca_ax,
                   os.path.join(out_dir, "error_per_axis.png"))
    print(f"[OK] wrote {os.path.join(out_dir, 'error_per_axis.png')}")

    # --- Plot 3: XY snapshots ---
    _plot_snapshots(cv_used, ca_used, gt_t, gt_xyz, taus, args.snapshots,
                    os.path.join(out_dir, "xy_snapshots.png"))
    print(f"[OK] wrote {os.path.join(out_dir, 'xy_snapshots.png')}")

    print(f"\n[DONE] outputs in {out_dir}")
    return 0


def _plot_error_vs_horizon(taus, cv_mean, cv_rms, cv_p90, ca_mean, ca_rms, ca_p90, path):
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(taus, cv_rms, "-o", ms=3, color="tab:blue", label="CV  RMS")
    ax.plot(taus, ca_rms, "-o", ms=3, color="tab:red", label="CA  RMS")
    ax.plot(taus, cv_mean, "--", color="tab:blue", alpha=0.6, label="CV  mean")
    ax.plot(taus, ca_mean, "--", color="tab:red", alpha=0.6, label="CA  mean")
    ax.fill_between(taus, cv_mean, cv_p90, color="tab:blue", alpha=0.10)
    ax.fill_between(taus, ca_mean, ca_p90, color="tab:red", alpha=0.10)
    ax.set_xlabel("prediction look-ahead $\\tau$ [s]")
    ax.set_ylabel("position error $\\|p_{pred}-p_{gt}\\|$ [m]")
    ax.set_title("CV vs CA obstacle-prediction error vs look-ahead horizon")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _plot_per_axis(taus, cv_ax, ca_ax, path):
    def ax_rms(per_tau, k):
        return np.array([
            np.sqrt(np.mean(np.square([d[k] for d in e]))) if e else np.nan
            for e in per_tau
        ])

    fig, axes = plt.subplots(1, 3, figsize=(13, 4), sharey=True)
    for k, name in enumerate(["x", "y", "z"]):
        axes[k].plot(taus, ax_rms(cv_ax, k), "-o", ms=3, color="tab:blue", label="CV")
        axes[k].plot(taus, ax_rms(ca_ax, k), "-o", ms=3, color="tab:red", label="CA")
        axes[k].set_title(f"{name}-axis RMS error")
        axes[k].set_xlabel("look-ahead $\\tau$ [s]")
        axes[k].grid(True, alpha=0.3)
    axes[0].set_ylabel("abs error [m]")
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _plot_snapshots(cv_used, ca_used, gt_t, gt_xyz, taus, n_panels, path):
    # Index CA predictions by t0 so we can pair them with CV at the same emit time.
    ca_by_t0 = {round(t0, 3): msg for t0, msg in ca_used}
    paired = []
    for t0, cv_msg in cv_used:
        ca_msg = ca_by_t0.get(round(t0, 3))
        if ca_msg is not None:
            paired.append((t0, cv_msg, ca_msg))
    if not paired:
        # Nothing paired; emit an empty figure with a note.
        fig = plt.figure(figsize=(6, 3))
        fig.text(0.5, 0.5, "no paired CV/CA snapshots", ha="center")
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return

    idx = np.linspace(0, len(paired) - 1, min(n_panels, len(paired))).astype(int)
    cols = min(3, len(idx))
    rows = int(np.ceil(len(idx) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4.2 * rows), squeeze=False)
    for ax in axes.flat:
        ax.axis("off")

    for panel, j in enumerate(idx):
        t0, cv_msg, ca_msg = paired[j]
        q = t0 + taus
        g, valid = gt_at(gt_t, gt_xyz, q)
        cvp = pred_pos(cv_msg, taus)
        cap = pred_pos(ca_msg, taus)

        ax = axes.flat[panel]
        ax.axis("on")
        if valid.any():
            ax.plot(g[valid, 0], g[valid, 1], "-", color="k", lw=2.5,
                    label="ground truth")
            ax.plot(g[valid, 0][-1], g[valid, 1][-1], "k*", ms=11)
        ax.plot(cvp[:, 0], cvp[:, 1], "-", color="tab:blue", lw=1.8, label="CV pred")
        ax.plot(cap[:, 0], cap[:, 1], "-", color="tab:red", lw=1.8, label="CA pred")
        ax.plot(cvp[0, 0], cvp[0, 1], "o", color="green", ms=7, label="start")

        # error at final horizon (if GT covers it)
        if valid[-1]:
            ecv = np.linalg.norm(cvp[-1] - g[-1])
            eca = np.linalg.norm(cap[-1] - g[-1])
            title = f"t0={t0:.1f}s  @{taus[-1]:.1f}s: CV={ecv:.2f} CA={eca:.2f} m"
        else:
            title = f"t0={t0:.1f}s"
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        if panel == 0:
            ax.legend(fontsize=8, loc="best")

    fig.suptitle("Top-down (XY): predicted vs ground-truth obstacle path", y=1.0)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
