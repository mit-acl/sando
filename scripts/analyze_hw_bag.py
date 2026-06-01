#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
Analyze hardware ROS2 bag data from SANDO flights.

Produces:
  1. Computation time statistics (avg +/- std) printed as a table and saved as CSV
  2. History plot of position, velocity, acceleration, jerk (PDF)
  3. (Optional) LaTeX table summarizing multiple test runs

Usage:
  # Single bag:
  python3 analyze_hw_bag.py /path/to/bag_folder

  # All bags in a directory:
  python3 analyze_hw_bag.py /path/to/parent_folder

  # Custom velocity/acceleration/jerk limits:
  python3 analyze_hw_bag.py /path/to/bag_folder --v_max 5.0 --a_max 20.0 --j_max 100.0

  # Generate LaTeX table for hw static tests:
  python3 analyze_hw_bag.py /path/to/hw/static --generate_table --table_output /path/to/tables/hw_static.table
"""

import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------


def is_ros2_bag(path):
    """Check if a directory looks like a ROS2 bag (contains metadata.yaml)."""
    return os.path.isdir(path) and os.path.exists(os.path.join(path, "metadata.yaml"))


def read_bag_topics(bag_path, topic_type_pairs, storage_id="sqlite3"):
    """Read specified topics from a bag. Returns {topic: [msgs]}."""
    type_map = {t: get_message(mt) for t, mt in topic_type_pairs}
    topic_names = [t for t, _ in topic_type_pairs]

    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    try:
        reader.set_filter(StorageFilter(topics=topic_names))
    except Exception:
        pass

    out = {t: [] for t in topic_names}
    while reader.has_next():
        topic, serialized_msg, _ = reader.read_next()
        if topic in type_map:
            msg = deserialize_message(serialized_msg, type_map[topic])
            out[topic].append(msg)
    return out


def get_topic_type(bag_path, topic_name, storage_id="sqlite3"):
    """Return the message type string for ``topic_name`` from the bag's
    metadata, or ``None`` if the topic is absent."""
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    for t in reader.get_all_topics_and_types():
        if t.name == topic_name:
            return t.type
    return None


def discover_namespace(bag_path, storage_id="sqlite3"):
    """Discover the robot namespace from bag topics (e.g., /PX03)."""
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topics = reader.get_all_topics_and_types()
    for t in topics:
        if t.name.endswith("/goal"):
            # e.g., /PX03/goal -> /PX03
            return t.name.rsplit("/goal", 1)[0]
        if t.name.endswith("/computation_times"):
            return t.name.rsplit("/computation_times", 1)[0]
    # fallback: find any namespaced topic
    for t in topics:
        parts = t.name.strip("/").split("/")
        if len(parts) >= 2:
            return "/" + parts[0]
    return ""


# ---------------------------------------------------------------------------
# Computation time statistics
# ---------------------------------------------------------------------------


def compute_time_stats(comp_msgs):
    """Compute avg and std for every computation-time field published by
    ``dynus_interfaces/msg/ComputationTimes``. Sub-fields of HGP are indented
    in the printed report so it is obvious what rolls up into what."""
    # Wrapper-stage breakdown is printed in `Total Replanning` order to make the
    # gap obvious: sum(housekeeping, global_planning, local_outer, append,
    # final_housekeeping) ~= total. Sub-stages of local_outer (cvx_decomp + local_traj)
    # are indented; the difference local_outer - cvx_decomp - local_traj is the
    # parallel-sweep coordination overhead.
    fields = [
        ("total_replanning_ms", "Total Replanning [ms]"),
        ("housekeeping_ms", "  Housekeeping [ms]"),
        ("global_planning_ms", "  Global Planning [ms]"),
        ("update_map_ms", "    Update Map [ms]"),
        ("hgp_static_jps_ms", "    HGP Static JPS [ms]"),
        ("hgp_check_path_ms", "    HGP Check Path [ms]"),
        ("hgp_dynamic_astar_ms", "    HGP Dynamic A* [ms]"),
        ("hgp_recover_path_ms", "    HGP Recover Path [ms]"),
        ("local_outer_ms", "  Local Outer (STSFC+MIQP+sweep) [ms]"),
        ("cvx_decomp_ms", "    Safety Corridor [ms]"),
        ("local_traj_ms", "    Local Traj. [ms]"),
        ("append_ms", "  Append to Plan [ms]"),
        ("final_housekeeping_ms", "  Final Housekeeping [ms]"),
        ("safe_paths_ms", "Safe Paths [ms]"),
        ("safety_check_ms", "Safety Check [ms]"),
        ("yaw_sequence_ms", "Yaw Sequence [ms]"),
        ("yaw_fitting_ms", "Yaw Fitting [ms]"),
    ]

    data = {display: [] for _, display in fields}
    for msg in comp_msgs:
        # Skip zero entries (before flight starts or after goal reached)
        if msg.total_replanning_ms == 0.0:
            continue
        # Only include successful replanning results
        if not msg.result:
            continue
        for attr, display in fields:
            # Default to 0.0 for fields that may not exist in older bags
            # (e.g., before the wrapper-stage breakdown was added).
            data[display].append(getattr(msg, attr, 0.0))

    stats = {}
    for _, display in fields:
        arr = np.array(data[display])
        if len(arr) > 0:
            stats[display] = {"avg": np.mean(arr), "std": np.std(arr)}
        else:
            stats[display] = {"avg": 0.0, "std": 0.0}
    return stats


def print_comp_stats(stats, bag_name):
    """Pretty-print computation time statistics."""
    print(f"\n{'=' * 60}")
    print(f"Computation Times: {bag_name}")
    print(f"{'=' * 60}")
    print(f"{'Metric':<30} {'Avg':>10} {'Std':>10}")
    print(f"{'-' * 50}")
    for name, s in stats.items():
        print(f"{name:<30} {s['avg']:>10.3f} {s['std']:>10.3f}")
    print()


def save_comp_stats_csv(stats, save_path):
    """Save computation time stats as CSV."""
    with open(save_path, "w") as f:
        f.write("Metric,Avg,Std\n")
        for name, s in stats.items():
            f.write(f"{name},{s['avg']:.4f},{s['std']:.4f}\n")
    print(f"Saved computation stats to {save_path}")


# ---------------------------------------------------------------------------
# Hardware metrics for reviewer comments (R5.9 clearance / tracking error /
# replanning failures / obstacle estimation error; R5.10 obstacle peak speed)
# ---------------------------------------------------------------------------


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _msg_stamp_sec(m):
    """Representative timestamp for a ROS message. Handles top-level header
    (most messages) and tf2_msgs/TFMessage (use first transform's header)."""
    if hasattr(m, "header") and hasattr(m.header, "stamp"):
        return _stamp_sec(m.header.stamp)
    if hasattr(m, "transforms") and m.transforms:
        return _stamp_sec(m.transforms[0].header.stamp)
    return None


def extract_pose_array(msgs):
    """Extract (t, xyz) arrays from messages that carry a pose with a header.
    Handles both geometry_msgs/PoseStamped (``m.pose.position``) and
    geometry_msgs/PoseWithCovarianceStamped (``m.pose.pose.position``).
    Returns (None, None) if empty."""
    if not msgs:
        return None, None
    t = np.array([_stamp_sec(m.header.stamp) for m in msgs])
    p_list = []
    for m in msgs:
        # PoseWithCovarianceStamped: m.pose.pose.position
        # PoseStamped:               m.pose.position
        pose = getattr(m.pose, "pose", m.pose) if hasattr(m.pose, "pose") else m.pose
        pos = pose.position
        p_list.append([pos.x, pos.y, pos.z])
    return t, np.array(p_list)


def extract_goal_pos_array(msgs):
    """Extract (t, xyz) commanded reference from dynus_interfaces/Goal messages."""
    if not msgs:
        return None, None
    t = np.array([_stamp_sec(m.header.stamp) for m in msgs])
    p = np.array([[m.p.x, m.p.y, m.p.z] for m in msgs])
    return t, p


def extract_pose_from_tf(tf_msgs, child_frame, parent_frames):
    """Extract (t, xyz) drone pose from a stream of tf2_msgs/TFMessage messages.

    Selects transforms whose ``child_frame_id == child_frame`` and whose
    ``header.frame_id`` is in ``parent_frames``. The translation component of
    such a transform is the child frame's position expressed in the parent
    frame (here: the drone's world-frame pose).
    """
    if not tf_msgs or not child_frame:
        return None, None
    parents = set(parent_frames or [])
    t_list = []
    p_list = []
    for m in tf_msgs:
        for tr in m.transforms:
            if tr.child_frame_id == child_frame and (
                not parents or tr.header.frame_id in parents
            ):
                t_list.append(_stamp_sec(tr.header.stamp))
                tx = tr.transform.translation
                p_list.append([tx.x, tx.y, tx.z])
                break  # one drone-pose per /tf message expected
    if not t_list:
        return None, None
    return np.array(t_list), np.array(p_list)


def extract_setpoint_trajectory_pos_array(msgs):
    """Extract (t, xyz) from trajectory_msgs/MultiDOFJointTrajectory messages
    (e.g., /<ns>/mavros/setpoint_trajectory/local).

    Uses the first point's first transform translation. Skips messages with no
    points or no transforms. Returns (None, None) if no usable messages.
    """
    if not msgs:
        return None, None
    t_list = []
    p_list = []
    for m in msgs:
        if not m.points:
            continue
        pt = m.points[0]
        if not pt.transforms:
            continue
        tr = pt.transforms[0].translation
        t_list.append(_stamp_sec(m.header.stamp))
        p_list.append([tr.x, tr.y, tr.z])
    if not t_list:
        return None, None
    return np.array(t_list), np.array(p_list)


def compute_flight_window(
    goal_msgs, pose_msgs, num_returns=5, near_m=0.5, away_m=2.0
):
    """Detect the active-flight time window.

    Start = timestamp of the first ``/goal`` message.
    End   = timestamp at which the drone has returned to its flight-start
            position for the ``num_returns``-th time (hysteresis: must first
            travel beyond ``away_m`` then come back within ``near_m`` of the
            start to count as one return).

    Returns ``(start_t, end_t, n_returns_observed)``. Returns ``(None, None, 0)``
    if a window cannot be determined.
    """
    if not goal_msgs or not pose_msgs:
        return None, None, 0
    start_t = _stamp_sec(goal_msgs[0].header.stamp)
    pose_t, pose_p = extract_pose_array(pose_msgs)
    if pose_t is None or len(pose_t) == 0:
        return None, None, 0
    order = np.argsort(pose_t)
    pose_t = pose_t[order]
    pose_p = pose_p[order]
    sel = pose_t >= start_t
    if not np.any(sel):
        return start_t, None, 0
    t_post = pose_t[sel]
    p_post = pose_p[sel]
    start_pos = p_post[0]

    state = "at_start"
    returns = 0
    end_t = None
    for ti, pi in zip(t_post, p_post):
        d = float(np.linalg.norm(pi - start_pos))
        if state == "at_start" and d > away_m:
            state = "away"
        elif state == "away" and d < near_m:
            returns += 1
            state = "at_start"
            if returns >= num_returns:
                end_t = float(ti)
                break
    if end_t is None:
        end_t = float(t_post[-1])
    return float(start_t), end_t, returns


def filter_by_time(msgs, t_lo, t_hi):
    """Keep messages whose representative timestamp is in [t_lo, t_hi].
    Pass-through if either bound is None. Handles both header-bearing messages
    and tf2_msgs/TFMessage."""
    if t_lo is None or t_hi is None:
        return list(msgs)
    out = []
    for m in msgs:
        t = _msg_stamp_sec(m)
        if t is None:
            out.append(m)  # cannot timestamp — keep
        elif t_lo <= t <= t_hi:
            out.append(m)
    return out


def _drone_near_any_goal(pos_xyz, goals, radius_m):
    """True if pos is within ``radius_m`` of any of the provided 3D goals."""
    for g in goals:
        if float(np.linalg.norm(pos_xyz - np.asarray(g, dtype=float))) < radius_m:
            return True
    return False


def filter_msgs_not_near_goal(msgs, pose_t, pose_p, goals, radius_m):
    """Drop messages whose timestamp falls in a period when the drone (interpolated
    from ``pose_t``, ``pose_p``) is within ``radius_m`` of any of ``goals``.

    Rationale: when the agent is within the goal-stop radius the planner stops
    publishing fresh setpoints, so tracking error / replan stats from those
    intervals are not representative of active flight.
    """
    if not goals or not msgs or pose_t is None or len(pose_t) == 0:
        return list(msgs)
    order = np.argsort(pose_t)
    pt = np.asarray(pose_t)[order]
    pp = np.asarray(pose_p)[order]
    kept = []
    for m in msgs:
        t = _msg_stamp_sec(m)
        if t is None:
            kept.append(m)
            continue
        # Out-of-pose-range messages: keep (no info to mask on)
        if t < pt[0] or t > pt[-1]:
            kept.append(m)
            continue
        drone_p = np.array(
            [np.interp(t, pt, pp[:, axis]) for axis in range(3)]
        )
        if not _drone_near_any_goal(drone_p, goals, radius_m):
            kept.append(m)
    return kept


def extract_obstacle_estimates(msgs):
    """Flatten DynTraj messages to [(t_sec, id, pos_xyz)]. Skips is_agent==True."""
    out = []
    for m in msgs:
        if getattr(m, "is_agent", False):
            continue
        t = _stamp_sec(m.header.stamp)
        out.append((t, int(m.id), np.array([m.pos.x, m.pos.y, m.pos.z])))
    return out


def interp_xyz(t_src, p_src, t_query):
    """Per-axis linear interpolation; clamps queries outside the source range."""
    if t_src is None or len(t_src) == 0:
        return None
    order = np.argsort(t_src)
    t_src = np.asarray(t_src)[order]
    p_src = np.asarray(p_src)[order]
    t_q = np.clip(np.asarray(t_query), t_src[0], t_src[-1])
    out = np.empty((len(t_q), 3))
    for axis in range(3):
        out[:, axis] = np.interp(t_q, t_src, p_src[:, axis])
    return out


def compute_replan_failures(comp_msgs):
    """Replanning success/failure stats from computation_times messages."""
    if not comp_msgs:
        return {
            "total_replans": 0,
            "num_failures": 0,
            "failure_rate_pct": 0.0,
            "longest_failure_streak": 0,
        }
    results = [bool(m.result) for m in comp_msgs]
    total = len(results)
    fails = sum(1 for r in results if not r)
    longest = cur = 0
    for r in results:
        if not r:
            cur += 1
            longest = max(longest, cur)
        else:
            cur = 0
    return {
        "total_replans": total,
        "num_failures": fails,
        "failure_rate_pct": 100.0 * fails / total if total else 0.0,
        "longest_failure_streak": longest,
    }


def compute_tracking_error(goal_t, goal_p, pose_t, pose_p):
    """Tracking error: commanded position (goal) vs onboard pose estimate."""
    if goal_t is None or pose_t is None or len(goal_t) == 0 or len(pose_t) == 0:
        return None
    pose_at_goal = interp_xyz(pose_t, pose_p, goal_t)
    err = goal_p - pose_at_goal
    err_norm = np.linalg.norm(err, axis=1)
    per_axis_rms = np.sqrt(np.mean(err ** 2, axis=0))
    per_axis_max = np.max(np.abs(err), axis=0)
    return {
        "rms_pos_err_m": float(np.sqrt(np.mean(err_norm ** 2))),
        "max_pos_err_m": float(np.max(err_norm)),
        "mean_pos_err_m": float(np.mean(err_norm)),
        "rms_x": float(per_axis_rms[0]),
        "rms_y": float(per_axis_rms[1]),
        "rms_z": float(per_axis_rms[2]),
        "max_x": float(per_axis_max[0]),
        "max_y": float(per_axis_max[1]),
        "max_z": float(per_axis_max[2]),
        "n_samples": int(len(goal_t)),
    }


def compute_clearance(drone_t, drone_p, mocap_data):
    """Center-to-center clearance: drone pose vs each mocap obstacle over time."""
    if drone_t is None or len(drone_t) == 0 or not mocap_data:
        return None
    overall_min = float("inf")
    overall_min_obst = None
    overall_min_time = None
    per_obstacle = {}
    t0 = float(drone_t[0])
    for name, (t_obst, p_obst) in mocap_data.items():
        if t_obst is None or len(t_obst) == 0:
            continue
        p_obst_at_drone = interp_xyz(t_obst, p_obst, drone_t)
        dist = np.linalg.norm(drone_p - p_obst_at_drone, axis=1)
        min_i = int(np.argmin(dist))
        per_obstacle[name] = {
            "min_clearance_m": float(dist[min_i]),
            "min_time_s": float(drone_t[min_i] - t0),
            "mean_clearance_m": float(np.mean(dist)),
            "p5_clearance_m": float(np.percentile(dist, 5)),
        }
        if dist[min_i] < overall_min:
            overall_min = float(dist[min_i])
            overall_min_obst = name
            overall_min_time = float(drone_t[min_i] - t0)
    return {
        "overall_min_clearance_m": overall_min if overall_min != float("inf") else None,
        "overall_min_obstacle": overall_min_obst,
        "overall_min_time_s": overall_min_time,
        "per_obstacle": per_obstacle,
    }


def compute_obstacle_estimation_error(est_records, mocap_data):
    """Pair each AEKF estimate (predicted_trajs.pos) to the nearest mocap obstacle
    at the estimate's timestamp, and compute the position error."""
    if not est_records or not mocap_data:
        return None
    names = list(mocap_data.keys())
    errs = []
    assoc_counts = {n: 0 for n in names}
    for t_e, _id, p_est in est_records:
        best_d = float("inf")
        best_name = None
        best_truth = None
        for n in names:
            t_m, p_m = mocap_data[n]
            if t_m is None or len(t_m) == 0:
                continue
            if t_e < t_m[0] or t_e > t_m[-1]:
                continue  # outside mocap coverage
            p_truth = interp_xyz(t_m, p_m, np.array([t_e]))[0]
            d = float(np.linalg.norm(p_est - p_truth))
            if d < best_d:
                best_d = d
                best_name = n
                best_truth = p_truth
        if best_name is not None:
            errs.append((t_e, best_name, p_est, best_truth, p_est - best_truth))
            assoc_counts[best_name] += 1
    if not errs:
        return None
    err_vec = np.array([e[4] for e in errs])
    err_norm = np.linalg.norm(err_vec, axis=1)
    per_axis_max = np.max(np.abs(err_vec), axis=0)
    all_zero = all(np.allclose(e[2], 0.0) for e in errs)
    return {
        "n_samples": int(len(errs)),
        "all_estimates_zero": bool(all_zero),
        "est_err_mean_m": float(np.mean(err_norm)),
        "est_err_max_m": float(np.max(err_norm)),
        "est_err_p95_m": float(np.percentile(err_norm, 95)),
        "max_x": float(per_axis_max[0]),
        "max_y": float(per_axis_max[1]),
        "max_z": float(per_axis_max[2]),
        "assoc_counts": assoc_counts,
    }


def extract_twist_array(msgs):
    """Extract (t, vxyz) from TwistStamped messages. Returns (None, None) if empty."""
    if not msgs:
        return None, None
    t = np.array([_stamp_sec(m.header.stamp) for m in msgs])
    v = np.array(
        [[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for m in msgs]
    )
    return t, v


def compute_obstacle_speed_from_twist(twist_data):
    """Compute ||v|| stats directly from mocap TwistStamped messages.

    Preferred over differentiating pose because it avoids the timestamp-jitter
    artifact that produces spurious speed spikes.
    """
    out = {}
    for name, (t, v) in twist_data.items():
        if t is None or len(t) == 0:
            continue
        speed = np.linalg.norm(v, axis=1)
        if speed.size == 0:
            continue
        out[name] = {
            "peak_speed_mps": float(np.max(speed)),
            "peak_p99_speed_mps": float(np.percentile(speed, 99)),
            "p95_speed_mps": float(np.percentile(speed, 95)),
            "mean_speed_mps": float(np.mean(speed)),
            "n_samples": int(speed.size),
            "source": "twist",
        }
    return out


def compute_obstacle_peak_speed(mocap_data, min_dt_s=0.005):
    """Numerically differentiate each mocap obstacle's position; report ||v|| stats.

    Used as a fallback when no twist topic is provided. Samples whose adjacent
    timestamp gap is below ``min_dt_s`` (default 5 ms) are discarded because
    near-zero dt amplifies mocap timestamp noise into spurious speed spikes.
    The robust ``peak_p99_speed_mps`` is the recommended summary; the raw
    ``peak_speed_mps`` is also reported for transparency.
    """
    out = {}
    for name, (t, p) in mocap_data.items():
        if t is None or len(t) < 2:
            continue
        order = np.argsort(t)
        t_s = np.asarray(t)[order]
        p_s = np.asarray(p)[order]
        dt = np.diff(t_s)
        valid = dt >= min_dt_s
        if not np.any(valid):
            continue
        v = np.diff(p_s, axis=0)[valid] / dt[valid][:, None]
        speed = np.linalg.norm(v, axis=1)
        if speed.size == 0:
            continue
        out[name] = {
            "peak_speed_mps": float(np.max(speed)),
            "peak_p99_speed_mps": float(np.percentile(speed, 99)),
            "p95_speed_mps": float(np.percentile(speed, 95)),
            "mean_speed_mps": float(np.mean(speed)),
            "n_samples": int(speed.size),
            "min_dt_s": float(min_dt_s),
            "source": "differentiated_pose",
        }
    return out


def print_metrics(metrics, bag_name):
    """Pretty-print all hardware metrics."""
    print(f"\n{'=' * 60}")
    print(f"Hardware metrics: {bag_name}")
    print(f"{'=' * 60}")

    rf = metrics.get("replan_failures")
    if rf is not None:
        print("\n[Replanning failures]")
        print(f"  Total replans       : {rf['total_replans']}")
        print(f"  Failures            : {rf['num_failures']}")
        print(f"  Failure rate        : {rf['failure_rate_pct']:.2f} %")
        print(f"  Longest streak      : {rf['longest_failure_streak']}")

    te = metrics.get("tracking_error")
    if te is not None:
        src = te.get("reference_source", "goal")
        ref_label = "setpoint_trajectory/local" if src == "setpoint_trajectory" else "/goal"
        actual_label = te.get("actual_source", "onboard /mavros pose")
        print(f"\n[Tracking error  (cmd {ref_label} vs {actual_label})]")
        print(
            f"  RMS  position error : {te['rms_pos_err_m']*100:7.2f} cm   ({te['rms_pos_err_m']:.4f} m)"
        )
        print(
            f"  Max  position error : {te['max_pos_err_m']*100:7.2f} cm   ({te['max_pos_err_m']:.4f} m)"
        )
        print(
            f"  Mean position error : {te['mean_pos_err_m']*100:7.2f} cm   ({te['mean_pos_err_m']:.4f} m)"
        )
        print(
            f"  Per-axis RMS  (x,y,z): {te['rms_x']*100:6.2f}, {te['rms_y']*100:6.2f}, {te['rms_z']*100:6.2f} cm"
        )
        print(
            f"  Per-axis max  (x,y,z): {te['max_x']*100:6.2f}, {te['max_y']*100:6.2f}, {te['max_z']*100:6.2f} cm"
        )
        print(f"  (n = {te['n_samples']} commanded samples)")

    cl = metrics.get("clearance")
    if cl is not None and cl.get("overall_min_clearance_m") is not None:
        drone_src = cl.get("drone_source", "onboard /mavros pose")
        print(f"\n[Clearance: drone ({drone_src}) vs mocap obstacles (center-to-center)]")
        print(
            f"  Overall MIN clearance : {cl['overall_min_clearance_m']:.3f} m"
            f"  (obstacle '{cl['overall_min_obstacle']}' at t = {cl['overall_min_time_s']:.2f} s)"
        )
        for name, s in cl["per_obstacle"].items():
            print(
                f"    {name:25s}: min = {s['min_clearance_m']:.3f} m"
                f"   mean = {s['mean_clearance_m']:.3f} m   p5 = {s['p5_clearance_m']:.3f} m"
            )

    oe = metrics.get("obstacle_est_error")
    if oe is not None:
        print("\n[Obstacle estimation error  (AEKF predicted_trajs.pos vs mocap truth)]")
        if oe["all_estimates_zero"]:
            print(
                "  !! WARNING: all predicted_trajs.pos == (0,0,0). The publisher likely "
                "does not populate msg.pos in this bag; the estimation-error numbers below "
                "reflect the mocap obstacle position, not the AEKF estimate."
            )
        print(f"  n samples           : {oe['n_samples']}")
        print(
            f"  Mean Euclidean error: {oe['est_err_mean_m']*100:7.2f} cm   ({oe['est_err_mean_m']:.4f} m)"
        )
        print(
            f"  Max  Euclidean error: {oe['est_err_max_m']*100:7.2f} cm   ({oe['est_err_max_m']:.4f} m)"
        )
        print(f"  P95  Euclidean error: {oe['est_err_p95_m']*100:7.2f} cm")
        print(
            f"  Per-axis max  (x,y,z): {oe['max_x']*100:6.2f}, {oe['max_y']*100:6.2f}, {oe['max_z']*100:6.2f} cm"
        )
        print(f"  Association counts  : {oe['assoc_counts']}")

    sp = metrics.get("obstacle_speed")
    if sp:
        src = next(iter(sp.values())).get("source", "?")
        print(
            f"\n[Mocap obstacle Euclidean speed]  (source: {src})"
        )
        for name, s in sp.items():
            print(
                f"    {name:25s}: peak = {s['peak_speed_mps']:.3f} m/s"
                f"   p99 = {s['peak_p99_speed_mps']:.3f}"
                f"   p95 = {s['p95_speed_mps']:.3f}"
                f"   mean = {s['mean_speed_mps']:.3f}"
            )
    print()


def save_metrics_csv(metrics, save_path):
    """Save all hardware metrics as a flat Metric,Value,Unit CSV."""
    rows = []
    rf = metrics.get("replan_failures")
    if rf is not None:
        rows.append(("total_replans", rf["total_replans"], ""))
        rows.append(("replan_failures", rf["num_failures"], ""))
        rows.append(("replan_failure_rate", f"{rf['failure_rate_pct']:.4f}", "%"))
        rows.append(("longest_failure_streak", rf["longest_failure_streak"], ""))
    te = metrics.get("tracking_error")
    if te is not None:
        rows.append(("tracking_rms_m", f"{te['rms_pos_err_m']:.4f}", "m"))
        rows.append(("tracking_max_m", f"{te['max_pos_err_m']:.4f}", "m"))
        rows.append(("tracking_mean_m", f"{te['mean_pos_err_m']:.4f}", "m"))
        rows.append(("tracking_rms_x_m", f"{te['rms_x']:.4f}", "m"))
        rows.append(("tracking_rms_y_m", f"{te['rms_y']:.4f}", "m"))
        rows.append(("tracking_rms_z_m", f"{te['rms_z']:.4f}", "m"))
        rows.append(("tracking_max_x_m", f"{te['max_x']:.4f}", "m"))
        rows.append(("tracking_max_y_m", f"{te['max_y']:.4f}", "m"))
        rows.append(("tracking_max_z_m", f"{te['max_z']:.4f}", "m"))
        rows.append(("tracking_n_samples", te["n_samples"], ""))
    cl = metrics.get("clearance")
    if cl is not None and cl.get("overall_min_clearance_m") is not None:
        rows.append(("min_clearance_m", f"{cl['overall_min_clearance_m']:.4f}", "m"))
        rows.append(("min_clearance_obstacle", cl["overall_min_obstacle"] or "", ""))
        rows.append(("min_clearance_time_s", f"{cl['overall_min_time_s']:.3f}", "s"))
        for name, s in cl["per_obstacle"].items():
            safe = name.strip("/").replace("/", "_")
            rows.append((f"clearance_{safe}_min_m", f"{s['min_clearance_m']:.4f}", "m"))
            rows.append(
                (f"clearance_{safe}_mean_m", f"{s['mean_clearance_m']:.4f}", "m")
            )
            rows.append((f"clearance_{safe}_p5_m", f"{s['p5_clearance_m']:.4f}", "m"))
    oe = metrics.get("obstacle_est_error")
    if oe is not None:
        rows.append(("est_err_mean_m", f"{oe['est_err_mean_m']:.4f}", "m"))
        rows.append(("est_err_max_m", f"{oe['est_err_max_m']:.4f}", "m"))
        rows.append(("est_err_p95_m", f"{oe['est_err_p95_m']:.4f}", "m"))
        rows.append(("est_err_max_x_m", f"{oe['max_x']:.4f}", "m"))
        rows.append(("est_err_max_y_m", f"{oe['max_y']:.4f}", "m"))
        rows.append(("est_err_max_z_m", f"{oe['max_z']:.4f}", "m"))
        rows.append(("est_err_n_samples", oe["n_samples"], ""))
        rows.append(("est_all_zero_warning", int(oe["all_estimates_zero"]), ""))
    sp = metrics.get("obstacle_speed")
    if sp:
        for name, s in sp.items():
            safe = name.strip("/").replace("/", "_")
            rows.append((f"speed_{safe}_peak_mps", f"{s['peak_speed_mps']:.4f}", "m/s"))
            rows.append(
                (f"speed_{safe}_peak_p99_mps", f"{s['peak_p99_speed_mps']:.4f}", "m/s")
            )
            rows.append((f"speed_{safe}_p95_mps", f"{s['p95_speed_mps']:.4f}", "m/s"))
            rows.append((f"speed_{safe}_mean_mps", f"{s['mean_speed_mps']:.4f}", "m/s"))
    if not rows:
        return
    with open(save_path, "w") as f:
        f.write("Metric,Value,Unit\n")
        for name, val, unit in rows:
            f.write(f"{name},{val},{unit}\n")
    print(f"Saved hardware metrics to {save_path}")


# ---------------------------------------------------------------------------
# Plotting (matches unc_benchmark_data_analysis.ipynb style)
# ---------------------------------------------------------------------------


def setup_plot_style(use_tex=False):
    """Configure matplotlib to match the notebook style."""
    if use_tex:
        plt.rcParams.update(
            {
                "text.usetex": True,
                "text.latex.preamble": (
                    r"\usepackage{amsmath}\usepackage{bm}"
                    r"\newcommand{\vect}[1]{\bm{#1}}"
                ),
            }
        )
    else:
        plt.rcParams.update({"text.usetex": False})
        candidates = [
            "Times New Roman",
            "Nimbus Roman",
            "TeX Gyre Termes",
            "Times",
            "CMU Serif",
            "DejaVu Serif",
        ]
        avail = {f.name for f in fm.fontManager.ttflist}
        chosen = next((c for c in candidates if c in avail), "DejaVu Serif")
        plt.rcParams["font.family"] = "serif"
        plt.rcParams["font.serif"] = [chosen]

    plt.rcParams["font.size"] = 18


def vect_label(sym, use_tex=False):
    return rf"$\vect{{{sym}}}$" if use_tex else rf"$\mathbf{{{sym}}}$"


def plot_history(
    t,
    p,
    v,
    a,
    j,
    save_path,
    v_max=5.0,
    a_max=20.0,
    j_max=100.0,
    use_tex=False,
    tol_abs=0.001,
    p_ylim=None,
    v_ylim=None,
    a_ylim=None,
    j_ylim=None,
):
    """
    Plot 4-row stacked history: position, velocity, acceleration, jerk.
    Style matches unc_benchmark_data_analysis.ipynb.

    Optional ylim overrides (tuples): p_ylim, v_ylim, a_ylim, j_ylim.
    If not provided, defaults are used.
    """
    setup_plot_style(use_tex)

    base_font = 26
    title_font = 28
    label_font = 26
    tick_font = 22
    legend_font = 28

    width = 10.0
    phi = (1 + 5**0.5) / 2
    height = (width / phi) * 1.3  # taller for 4 subplots

    fig, (ax_p, ax_v, ax_a, ax_j) = plt.subplots(
        4,
        1,
        sharex=True,
        figsize=(width, height),
        gridspec_kw=dict(hspace=0.35),
    )

    right_margin = 0.9

    def plot_xyz(ax, y, lim, ylabel, title, ylim_range=None):
        lx = ax.plot(t, y[:, 0], lw=2.2, label="x")[0]
        ly = ax.plot(t, y[:, 1], lw=2.2, label="y")[0]
        lz = ax.plot(t, y[:, 2], lw=2.2, label="z")[0]
        lines = [lx, ly, lz]

        lim_handle = None
        if lim is not None:
            lim = float(lim)
            lim_eff = lim + float(tol_abs)
            lim_handle = ax.axhline(+lim, ls=":", lw=1.8, color="k", label="limit")
            ax.axhline(-lim, ls=":", lw=1.8, color="k", label="_nolegend_")

        if ylim_range is not None:
            ax.set_ylim(ylim_range[0], ylim_range[1])
        elif lim is not None:
            yabs = np.max(np.abs(y)) if y.size else 1.0
            ymax = max(1.15 * yabs, 1.15 * float(lim))
            ax.set_ylim(-ymax, ymax)

        # Place y-ticks at the limit values (and 0) so readers see the bounds
        if lim is not None:
            ax.set_yticks([-lim, 0, lim])

        ax.set_ylabel(ylabel, fontsize=label_font)
        ax.set_title(title, fontsize=title_font)
        ax.grid(True, ls="--", alpha=0.35)
        ax.tick_params(axis="both", labelsize=tick_font)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        return lines, lim_handle

    # Position (no limits)
    p_lines, _ = plot_xyz(
        ax_p,
        p,
        None,
        rf"{vect_label('p', use_tex)} [m]",
        "Position",
        ylim_range=p_ylim or (-5, 17),
    )
    # Velocity
    v_lines, _ = plot_xyz(
        ax_v,
        v,
        v_max,
        rf"{vect_label('v', use_tex)} [m/s]",
        "Velocity",
        ylim_range=v_ylim or (-v_max * 1.1, v_max * 1.1),
    )
    # Acceleration
    a_lines, _ = plot_xyz(
        ax_a,
        a,
        a_max,
        rf"{vect_label('a', use_tex)} [m/s$^2$]",
        "Acceleration",
        ylim_range=a_ylim or (-a_max * 1.1, a_max * 1.1),
    )
    # Jerk
    j_lines, j_lim = plot_xyz(
        ax_j,
        j,
        j_max,
        rf"{vect_label('j', use_tex)} [m/s$^3$]",
        "Jerk",
        ylim_range=j_ylim or (-j_max * 1.1, j_max * 1.1),
    )

    ax_j.set_xlabel("Time [s]", fontsize=label_font)
    ax_j.tick_params(axis="x", labelsize=tick_font)

    # Legend
    handles = [j_lines[0], j_lines[1], j_lines[2]]
    labels = ["x", "y", "z"]
    if j_lim is not None:
        handles.append(j_lim)
        labels.append("limit")

    plt.tight_layout(rect=[0.0, 0.12, 1.0, 1.0])
    fig.legend(
        handles,
        labels,
        loc="lower center",
        bbox_to_anchor=(0.5, -0.08),
        frameon=False,
        fontsize=legend_font,
        ncol=len(labels),
        borderaxespad=0.0,
        handlelength=2.6,
        columnspacing=2.0,
    )

    os.makedirs(
        os.path.dirname(save_path) if os.path.dirname(save_path) else ".", exist_ok=True
    )
    fig.savefig(save_path, bbox_inches="tight", dpi=300)
    plt.close(fig)
    print(f"Saved history plot to {save_path}")


# ---------------------------------------------------------------------------
# Processing a single bag
# ---------------------------------------------------------------------------


def process_bag(bag_path, args):
    """Process a single ROS2 bag: compute stats and generate plot."""
    bag_name = os.path.basename(os.path.normpath(bag_path))
    print(f"\nProcessing bag: {bag_path}")

    ns = discover_namespace(bag_path)
    print(f"  Detected namespace: {ns}")

    goal_topic = f"{ns}/goal"
    comp_topic = f"{ns}/computation_times"
    pose_topic = f"{ns}/mavros/local_position/pose"
    pred_topic = f"{ns}/predicted_trajs"
    setpoint_topic = f"{ns}/mavros/setpoint_trajectory/local"

    topic_type_pairs = [
        (goal_topic, "dynus_interfaces/msg/Goal"),
        (comp_topic, "dynus_interfaces/msg/ComputationTimes"),
        (pose_topic, "geometry_msgs/msg/PoseStamped"),
        (pred_topic, "dynus_interfaces/msg/DynTraj"),
        (setpoint_topic, "trajectory_msgs/msg/MultiDOFJointTrajectory"),
    ]
    obstacle_mocap_topics = list(getattr(args, "obstacle_mocap_topics", []) or [])
    for mt in obstacle_mocap_topics:
        topic_type_pairs.append((mt, "geometry_msgs/msg/PoseStamped"))
    drone_mocap_topic = getattr(args, "drone_mocap_topic", None)
    if drone_mocap_topic:
        detected_type = (
            get_topic_type(bag_path, drone_mocap_topic)
            or "geometry_msgs/msg/PoseStamped"
        )
        topic_type_pairs.append((drone_mocap_topic, detected_type))
    drone_tf_frame = getattr(args, "drone_tf_frame", None)
    drone_tf_parents = list(getattr(args, "drone_tf_parents", ["world", "map"]) or [])
    if drone_tf_frame:
        topic_type_pairs.append(("/tf", "tf2_msgs/msg/TFMessage"))
    obstacle_mocap_twist_topics = list(
        getattr(args, "obstacle_mocap_twist_topics", []) or []
    )
    for tt in obstacle_mocap_twist_topics:
        topic_type_pairs.append((tt, "geometry_msgs/msg/TwistStamped"))

    data = read_bag_topics(bag_path, topic_type_pairs)

    # --- Flight-window mask (active flight only): first /goal -> N-th return to start ---
    apply_mask = not getattr(args, "no_mask", False)
    if apply_mask:
        n_back = int(getattr(args, "num_back_and_forth", 5))
        near_m = float(getattr(args, "mask_near_m", 0.5))
        away_m = float(getattr(args, "mask_away_m", 2.0))
        t_lo, t_hi, n_obs = compute_flight_window(
            data.get(goal_topic, []),
            data.get(pose_topic, []),
            num_returns=n_back,
            near_m=near_m,
            away_m=away_m,
        )
        if t_lo is None or t_hi is None:
            print("  WARNING: could not determine flight window; reporting on full bag.")
            t_lo, t_hi = None, None
        else:
            if n_obs < n_back:
                print(
                    f"  WARNING: only {n_obs} of {n_back} requested returns observed; "
                    "ending the window at the last pose sample."
                )
            print(
                f"  Flight window: [{t_lo:.3f}, {t_hi:.3f}] s "
                f"(duration {t_hi - t_lo:.1f} s, returns observed = {n_obs})"
            )
    else:
        print("  Flight-window mask disabled (--no_mask).")
        t_lo, t_hi = None, None

    # Apply the mask to every topic; downstream computations use the filtered lists.
    for topic in list(data.keys()):
        data[topic] = filter_by_time(data[topic], t_lo, t_hi)

    # --- Goal-region mask: exclude intervals when the drone is within the
    #     planner's goal-stop radius of any commanded goal point (those
    #     intervals are not active flight; the planner stops updating
    #     setpoints, so tracking-error and replan-fail stats from them
    #     are not representative). ---
    raw_goal_points = list(getattr(args, "goal_point", []) or [])
    apply_goal_mask = bool(raw_goal_points) and not getattr(args, "no_goal_mask", False)
    if apply_goal_mask:
        goal_radius = float(getattr(args, "goal_mask_radius_m", 0.6))
        pose_t_now, pose_p_now = extract_pose_array(data.get(pose_topic, []))
        if pose_t_now is not None and len(pose_t_now) > 0:
            order = np.argsort(pose_t_now)
            pose_t_now = pose_t_now[order]
            pose_p_now = pose_p_now[order]
            near_flags = np.array(
                [
                    _drone_near_any_goal(p, raw_goal_points, goal_radius)
                    for p in pose_p_now
                ]
            )
            frac = float(near_flags.mean()) if len(near_flags) else 0.0
            print(
                f"  Goal-region mask: excluding ~{frac * 100:.1f}% of "
                f"active-flight samples within {goal_radius:.2f} m of "
                f"{len(raw_goal_points)} goal point(s)."
            )
            for topic in list(data.keys()):
                data[topic] = filter_msgs_not_near_goal(
                    data[topic], pose_t_now, pose_p_now, raw_goal_points, goal_radius
                )
        else:
            print("  WARNING: goal-region mask requested but no pose samples available; skipping.")
    elif raw_goal_points and getattr(args, "no_goal_mask", False):
        print("  Goal-region mask disabled (--no_goal_mask).")

    # --- Computation time stats ---
    comp_msgs = data[comp_topic]
    if comp_msgs:
        stats = compute_time_stats(comp_msgs)
        print_comp_stats(stats, bag_name)
        csv_path = os.path.join(bag_path, f"comp_stats_{bag_name}.csv")
        save_comp_stats_csv(stats, csv_path)
    else:
        print("  No computation_times messages found.")

    # --- Hardware metrics (R5.9 clearance / tracking / replan failures /
    #     obstacle estimation error; R5.10 obstacle peak speed) ---
    metrics = {}

    if comp_msgs:
        metrics["replan_failures"] = compute_replan_failures(comp_msgs)

    pose_t, pose_p = extract_pose_array(data.get(pose_topic, []))

    # If a ground-truth drone-pose source is provided (either a dedicated
    # PoseStamped topic via --drone_mocap_topic, or a /tf child frame via
    # --drone_tf_frame), use it as the *true* drone position for tracking
    # error and clearance. Order of preference: drone_mocap_topic > tf > onboard.
    drone_truth_t, drone_truth_p = (None, None)
    drone_actual_source = "onboard /mavros pose"
    if drone_mocap_topic:
        drone_truth_t, drone_truth_p = extract_pose_array(
            data.get(drone_mocap_topic, [])
        )
        if drone_truth_t is not None and len(drone_truth_t) > 0:
            drone_actual_source = f"mocap {drone_mocap_topic}"
        else:
            print(
                f"  WARNING: --drone_mocap_topic {drone_mocap_topic} has no messages; "
                "falling through to /tf or onboard pose."
            )
    if (drone_truth_t is None or len(drone_truth_t) == 0) and drone_tf_frame:
        drone_truth_t, drone_truth_p = extract_pose_from_tf(
            data.get("/tf", []), drone_tf_frame, drone_tf_parents
        )
        if drone_truth_t is not None and len(drone_truth_t) > 0:
            drone_actual_source = (
                f"/tf ({'|'.join(drone_tf_parents)} -> {drone_tf_frame})"
            )
        else:
            print(
                f"  WARNING: no /tf transform found with child={drone_tf_frame} "
                f"and parent in {drone_tf_parents}; falling back to onboard pose."
            )
    actual_t = drone_truth_t if (drone_truth_t is not None and len(drone_truth_t) > 0) else pose_t
    actual_p = drone_truth_p if (drone_truth_p is not None and len(drone_truth_p) > 0) else pose_p

    # Reference signal for tracking error: setpoint_trajectory (the smooth Bezier
    # stream actually sent to the flight controller) is preferred over /goal
    # (per-replan output that contains replanning discontinuities).
    ref_kind = getattr(args, "tracking_reference", "setpoint_trajectory")
    if ref_kind == "setpoint_trajectory":
        ref_t, ref_p = extract_setpoint_trajectory_pos_array(
            data.get(setpoint_topic, [])
        )
        if ref_t is None or len(ref_t) == 0:
            print(
                f"  WARNING: --tracking_reference=setpoint_trajectory but "
                f"{setpoint_topic} has no usable messages; falling back to {goal_topic}."
            )
            ref_kind = "goal"
            ref_t, ref_p = extract_goal_pos_array(data.get(goal_topic, []))
    else:
        ref_t, ref_p = extract_goal_pos_array(data.get(goal_topic, []))

    # Tracking error: always use the onboard pose, NOT mocap. The reference
    # (setpoint_trajectory / goal) is published in the DLIO/mavros local
    # frame; mocap drone pose lives in world frame, offset by the static
    # world->init_pose transform. Comparing across frames produces a constant
    # ~3.7 m bias in our setup, not a meaningful tracking-error.
    if ref_t is not None and pose_t is not None:
        te = compute_tracking_error(ref_t, ref_p, pose_t, pose_p)
        if te is not None:
            te["reference_source"] = ref_kind
            te["actual_source"] = "onboard /mavros pose"
            metrics["tracking_error"] = te
    elif pose_t is None and obstacle_mocap_topics:
        print(f"  WARNING: no onboard /mavros pose messages; skipping tracking error.")

    mocap_data = {}
    for mt in obstacle_mocap_topics:
        t_m, p_m = extract_pose_array(data.get(mt, []))
        if t_m is not None and len(t_m) > 0:
            mocap_data[mt] = (t_m, p_m)
        else:
            print(f"  WARNING: mocap topic {mt} has no messages in this bag.")

    if actual_t is not None and mocap_data:
        cl = compute_clearance(actual_t, actual_p, mocap_data)
        if cl is not None:
            cl["drone_source"] = drone_actual_source
            metrics["clearance"] = cl

    est_records = extract_obstacle_estimates(data.get(pred_topic, []))
    if est_records and mocap_data:
        oe = compute_obstacle_estimation_error(est_records, mocap_data)
        if oe is not None:
            metrics["obstacle_est_error"] = oe
    elif obstacle_mocap_topics and not est_records:
        print(f"  WARNING: no messages on {pred_topic}; skipping obstacle estimation error.")

    twist_data = {}
    for tt in obstacle_mocap_twist_topics:
        t_v, v_v = extract_twist_array(data.get(tt, []))
        if t_v is not None and len(t_v) > 0:
            twist_data[tt] = (t_v, v_v)
        else:
            print(f"  WARNING: mocap twist topic {tt} has no messages in this bag.")

    if twist_data:
        sp = compute_obstacle_speed_from_twist(twist_data)
        if sp:
            metrics["obstacle_speed"] = sp
    elif mocap_data:
        sp = compute_obstacle_peak_speed(mocap_data)
        if sp:
            metrics["obstacle_speed"] = sp

    if metrics:
        print_metrics(metrics, bag_name)
        save_metrics_csv(metrics, os.path.join(bag_path, f"metrics_{bag_name}.csv"))

    # --- History plot ---
    goal_msgs = data[goal_topic]
    if not goal_msgs:
        print("  No goal messages found. Skipping plot.")
        return

    # Extract arrays
    t_abs, p, v, a, j = [], [], [], [], []
    for m in goal_msgs:
        tt = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t_abs.append(tt)
        p.append([m.p.x, m.p.y, m.p.z])
        v.append([m.v.x, m.v.y, m.v.z])
        a.append([m.a.x, m.a.y, m.a.z])
        j.append([m.j.x, m.j.y, m.j.z])

    t_abs = np.array(t_abs)
    p = np.array(p)
    v = np.array(v)
    a = np.array(a)
    j = np.array(j)

    # Shift time to start at 0
    t = t_abs - t_abs[0]

    save_path = os.path.join(bag_path, f"history_{bag_name}.pdf")
    plot_history(
        t,
        p,
        v,
        a,
        j,
        save_path,
        v_max=args.v_max,
        a_max=args.a_max,
        j_max=args.j_max,
        use_tex=args.use_tex,
    )


# ---------------------------------------------------------------------------
# LaTeX table generation for hw tests
# ---------------------------------------------------------------------------

# Static test configurations: test_name -> (v_max, a_max, j_max)
HW_STATIC_TESTS = {
    "test0": (1.0, 5.0, 10.0),
    "test1": (2.0, 5.0, 10.0),
    "test2": (3.0, 5.0, 10.0),
    "test3": (4.0, 5.0, 10.0),
    "test4": (5.0, 5.0, 10.0),
    "test5": (6.0, 5.0, 10.0),
}

# Dynamic test configurations: test_name -> (description,)
# All dynamic tests use v_max=2.0 m/s
HW_DYNAMIC_TESTS = [
    ("test1", "1 obst, line"),
    ("test1-take3", "1 obst, circle (take 3)"),
    ("test1-take4", "1 obst, circle (take 4)"),
    ("test2", "1 obst, figure 8"),
    ("test4", "5 obsts, line-ish (run 1)"),
    ("test5", "5 obsts, line-ish (run 2)"),
    ("test6", "5 obsts, line-ish (run 3)"),
]

# Dynamic round2 test configurations: test_name -> (obst_type, num_obst, obst_traj, v_max, a_max, j_max)
HW_DYNAMIC_ROUND2_TESTS = [
    # Exp 7-10: single obstacle, different trajectories
    ("test28", "1 Dyn.", "Line", 2.0, 5.0, 7.5),
    ("test29", "1 Dyn.", "Circle", 2.0, 5.0, 7.5),
    ("test30", "1 Dyn.", "Fig. Eight", 2.0, 5.0, 7.5),
    ("test31", "1 Dyn.", "Person", 2.0, 5.0, 7.5),
    # Exp 11-13: five dynamic obstacles
    ("test14", "5 Dyn.", "Line", 2.0, 5.0, 7.5),
    ("test15", "5 Dyn.", "Line", 2.0, 5.0, 7.5),
    ("test16", "5 Dyn.", "Line", 2.0, 5.0, 7.5),
    # Exp 14-16: dynamic + static, varying v_max
    ("test18", r"\makecell{5 Dyn. \\ \& Static}", "Line", 2.0, 5.0, 7.5),
    ("test23", r"\makecell{5 Dyn. \\ \& Static}", "Line", 3.0, 5.0, 7.5),
    ("test24", r"\makecell{5 Dyn. \\ \& Static}", "Line", 4.0, 5.0, 7.5),
]


def find_bag_in_test_dir(test_dir):
    """Find the single bag folder inside a test directory."""
    for d in sorted(os.listdir(test_dir)):
        candidate = os.path.join(test_dir, d)
        if is_ros2_bag(candidate):
            return candidate
    return None


def load_comp_stats_from_csv(csv_path):
    """Load computation time stats from a previously saved CSV."""
    stats = {}
    with open(csv_path, "r") as f:
        next(f)  # skip header
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 3:
                stats[parts[0]] = {"avg": float(parts[1]), "std": float(parts[2])}
    return stats


def compute_comp_stats_from_bag(bag_path, ns):
    """Read computation_times from bag and compute stats."""
    comp_topic = f"{ns}/computation_times"
    topic_type_pairs = [(comp_topic, "dynus_interfaces/msg/ComputationTimes")]
    data = read_bag_topics(bag_path, topic_type_pairs)
    comp_msgs = data[comp_topic]
    if comp_msgs:
        return compute_time_stats(comp_msgs)
    return None


def get_comp_stats(bag_path, ns):
    """Get computation stats by reading from the bag (applies result filter)."""
    return compute_comp_stats_from_bag(bag_path, ns)


def format_comp_cells(comp_stats):
    """Format computation time stats into table cells."""
    comp_keys = [
        "Total Replanning [ms]",
        "Global Planning [ms]",
        "Safety Corridor [ms]",
        "Local Traj. [ms]",
    ]
    cells = []
    for key in comp_keys:
        if comp_stats and key in comp_stats:
            avg = comp_stats[key]["avg"]
            std = comp_stats[key]["std"]
            cells.append(f"${avg:.1f} \\pm {std:.1f}$")
        else:
            cells.append("--")
    return cells


def build_comp_time_table(rows, row_header_col, caption, label, row_header_unit=None):
    """Build a LaTeX table with computation time columns.

    rows: list of dicts with 'row_label' (str) and 'comp_stats' (dict).
    row_header_col: LaTeX header for the first column.
    row_header_unit: optional unit string to show on a second line (e.g. '[m/s]').
    """
    lines = []
    lines.append(r"\begin{table}")
    lines.append(f"  \\caption{{{caption}}}")
    lines.append(f"  \\label{{{label}}}")
    lines.append(r"  \centering")
    lines.append(r"  \renewcommand{\arraystretch}{1.2}")
    lines.append(r"  \resizebox{\columnwidth}{!}{")
    lines.append(r"    \begin{tabular}{c c c c c c}")
    lines.append(r"      \toprule")
    lines.append(
        r"      \multirow{2}{*}[-0.4ex]{\textbf{Exp.}}"
        r" & \multirow{2}{*}[-0.4ex]{"
        + (
            rf"\makecell{{{row_header_col} \\ {{{row_header_unit}}}}}"
            if row_header_unit
            else row_header_col
        )
        + r"}"
        r" & \multicolumn{4}{c}{\textbf{Computation Time}}"
        r" \\"
    )
    lines.append(r"      \cmidrule(lr){3-6}")
    lines.append(
        r"      & "
        r"& $T_{\mathrm{replan}}$ [ms]"
        r" & $T_{\mathrm{global}}$ [ms]"
        r" & $T_{\mathrm{SSFC}}$ [ms]"
        r" & $T_{\mathrm{opt}}$ [ms]"
        r" \\"
    )
    lines.append(r"      \midrule")

    for i, row in enumerate(rows, start=1):
        cells = format_comp_cells(row["comp_stats"])
        cell_str = " & ".join(cells)
        lines.append(f"      {i} & {row['row_label']} & {cell_str} \\\\")

    lines.append(r"      \bottomrule")
    lines.append(r"    \end{tabular}")
    lines.append(r"  }")
    lines.append(r"  \vspace{-1.0em}")
    lines.append(r"\end{table}")
    return "\n".join(lines) + "\n"


def save_table(table_str, table_output):
    """Write LaTeX table string to file."""
    os.makedirs(os.path.dirname(table_output), exist_ok=True)
    with open(table_output, "w") as f:
        f.write(table_str)
    basename = os.path.basename(table_output)
    print(f"\nSaved LaTeX table to {table_output}")
    print(f"Include in paper with: \\input{{tables/{basename}}}")


def extract_goal_arrays(bag_path, ns):
    """Extract time, position, velocity, acceleration, jerk arrays from goal messages."""
    goal_topic = f"{ns}/goal"
    topic_type_pairs = [(goal_topic, "dynus_interfaces/msg/Goal")]
    data = read_bag_topics(bag_path, topic_type_pairs)
    goal_msgs = data[goal_topic]

    if not goal_msgs:
        return None

    t_abs = np.array(
        [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in goal_msgs]
    )
    p = np.array([[m.p.x, m.p.y, m.p.z] for m in goal_msgs])
    v = np.array([[m.v.x, m.v.y, m.v.z] for m in goal_msgs])
    a = np.array([[m.a.x, m.a.y, m.a.z] for m in goal_msgs])
    j = np.array([[m.j.x, m.j.y, m.j.z] for m in goal_msgs])
    t = t_abs - t_abs[0]

    return t, p, v, a, j


def generate_hw_static_table(static_dir, table_output):
    """Generate a LaTeX table and history plots for hw static tests (test0-test5)."""
    rows = []

    for test_name in sorted(HW_STATIC_TESTS.keys()):
        v_max, a_max, j_max = HW_STATIC_TESTS[test_name]
        test_dir = os.path.join(static_dir, test_name)
        if not os.path.isdir(test_dir):
            print(f"  WARNING: {test_dir} not found, skipping")
            continue

        bag_path = find_bag_in_test_dir(test_dir)
        if bag_path is None:
            print(f"  WARNING: No bag found in {test_dir}, skipping")
            continue

        print(f"  Processing {test_name} (v_max={v_max}) -> {bag_path}")

        ns = discover_namespace(bag_path)
        comp_stats = get_comp_stats(bag_path, ns)

        rows.append(
            {
                "row_label": f"{v_max:.1f}",
                "comp_stats": comp_stats,
            }
        )

        # Generate history plot
        goal_data = extract_goal_arrays(bag_path, ns)
        if goal_data is not None:
            t, p, v_arr, a_arr, j_arr = goal_data
            plot_path = os.path.join(bag_path, f"history_{test_name}.pdf")
            plot_history(
                t,
                p,
                v_arr,
                a_arr,
                j_arr,
                plot_path,
                v_max=v_max,
                a_max=a_max,
                j_max=j_max,
                p_ylim=(0, 20),
                v_ylim=(-v_max * 1.1, v_max * 1.1),
                a_ylim=(-a_max * 1.1, a_max * 1.1),
                j_ylim=(-j_max * 1.1, j_max * 1.1),
            )
        else:
            print(f"    No goal messages found for {test_name}, skipping plot.")

    if not rows:
        print("No data collected. Cannot generate table.")
        return

    caption = (
        "Hardware flight computation times in static environments with increasing velocity limits. "
        "All flights use $a_{\\max}=5$ m/s$^2$ and $j_{\\max}=10$ m/s$^3$."
    )
    table_str = build_comp_time_table(
        rows,
        row_header_col=r"$v_{\max}$",
        row_header_unit=r"[m/s]",
        caption=caption,
        label="tab:hw_static",
    )
    save_table(table_str, table_output)


def generate_hw_dynamic_table(dynamic_dir, table_output):
    """Generate a LaTeX table for hw dynamic tests."""
    rows = []

    for test_name, description in HW_DYNAMIC_TESTS:
        test_dir = os.path.join(dynamic_dir, test_name)
        if not os.path.isdir(test_dir):
            print(f"  WARNING: {test_dir} not found, skipping")
            continue

        bag_path = find_bag_in_test_dir(test_dir)
        if bag_path is None:
            print(f"  WARNING: No bag found in {test_dir}, skipping")
            continue

        print(f"  Processing {test_name} ({description}) -> {bag_path}")

        ns = discover_namespace(bag_path)
        comp_stats = get_comp_stats(bag_path, ns)

        rows.append(
            {
                "row_label": description,
                "comp_stats": comp_stats,
            }
        )

    if not rows:
        print("No data collected. Cannot generate table.")
        return

    caption = (
        "Hardware flight computation times in dynamic environments. "
        "All flights use $v_{\\max}=2$ m/s, $a_{\\max}=5$ m/s$^2$, and $j_{\\max}=10$ m/s$^3$."
    )
    table_str = build_comp_time_table(
        rows,
        row_header_col=r"\textbf{Scenario}",
        caption=caption,
        label="tab:hw_dynamic",
    )
    save_table(table_str, table_output)


def generate_hw_dynamic_static_table(dynamic_static_dir, table_output):
    """Generate a LaTeX table for hw dynamic+static tests (test7 only)."""
    test_name = "test7"
    test_dir = os.path.join(dynamic_static_dir, test_name)
    if not os.path.isdir(test_dir):
        print(f"  WARNING: {test_dir} not found")
        return

    bag_path = find_bag_in_test_dir(test_dir)
    if bag_path is None:
        print(f"  WARNING: No bag found in {test_dir}")
        return

    print(f"  Processing {test_name} -> {bag_path}")

    ns = discover_namespace(bag_path)
    comp_stats = get_comp_stats(bag_path, ns)

    rows = [
        {
            "row_label": "5 obsts + static",
            "comp_stats": comp_stats,
        }
    ]

    caption = (
        "Hardware flight computation times in combined dynamic and static environments. "
        "All flights use $v_{\\max}=2$ m/s, $a_{\\max}=5$ m/s$^2$, and $j_{\\max}=10$ m/s$^3$."
    )
    table_str = build_comp_time_table(
        rows,
        row_header_col=r"\textbf{Scenario}",
        caption=caption,
        label="tab:hw_dynamic_static",
    )
    save_table(table_str, table_output)


def build_dynamic_round2_table(rows, caption, label):
    """Build a LaTeX table with multirow grouping for repeated values."""
    lines = []
    lines.append(r"\begin{table}")
    lines.append(f"  \\caption{{{caption}}}")
    lines.append(f"  \\label{{{label}}}")
    lines.append(r"  \centering")
    lines.append(r"  \renewcommand{\arraystretch}{1.2}")
    lines.append(r"  \resizebox{\columnwidth}{!}{")
    lines.append(r"    \begin{tabular}{c c c c c c c c}")
    lines.append(r"      \toprule")
    lines.append(
        r"      \multirow{2}{*}[-0.4ex]{\textbf{Exp.}}"
        r" & \multirow{2}{*}[-0.4ex]{\makecell{\textbf{Obst.} \\ \textbf{Type}}}"
        r" & \multirow{2}{*}[-0.4ex]{\makecell{\textbf{Obst.} \\ \textbf{Traj.}}}"
        r" & \multirow{2}{*}[-0.4ex]{\makecell{$v_{\max}$ \\ {[m/s]}}}"
        r" & \multicolumn{4}{c}{\textbf{Computation Time [ms]}}"
        r" \\"
    )
    lines.append(r"      \cmidrule(lr){5-8}")
    lines.append(
        r"      & & & "
        r"& $T_{\mathrm{replan}}$"
        r" & $T_{\mathrm{global}}$"
        r" & $T_{\mathrm{STSFC}}$"
        r" & $T_{\mathrm{opt}}$"
        r" \\"
    )
    lines.append(r"      \midrule")

    # Split rows into groups separated by midrules
    midrule_after = {10, 13}
    groups = []
    current_group = []
    for i, row in enumerate(rows, start=7):
        current_group.append((i, row))
        if i in midrule_after or i == len(rows) + 6:
            groups.append(current_group)
            current_group = []
    if current_group:
        groups.append(current_group)

    for gi, group in enumerate(groups):
        n = len(group)
        for j, (exp_num, row) in enumerate(group):
            cells = format_comp_cells(row["comp_stats"])
            cell_str = " & ".join(cells)

            # Determine if this column should use multirow (first in group)
            # or be empty (subsequent in group with same value)
            obst_type_vals = [r["obst_type"] for _, r in group]
            obst_traj_vals = [r["obst_traj"] for _, r in group]
            v_max_vals = [r["v_max"] for _, r in group]

            if j == 0:
                # First row: use multirow if all values in group are the same
                if all(v == obst_type_vals[0] for v in obst_type_vals):
                    obst_type_str = f"\\multirow{{{n}}}{{*}}{{{row['obst_type']}}}"
                else:
                    obst_type_str = row["obst_type"]

                if all(v == obst_traj_vals[0] for v in obst_traj_vals):
                    obst_traj_str = f"\\multirow{{{n}}}{{*}}{{{row['obst_traj']}}}"
                else:
                    obst_traj_str = row["obst_traj"]

                if all(v == v_max_vals[0] for v in v_max_vals):
                    v_max_str = f"\\multirow{{{n}}}{{*}}{{{row['v_max']:.1f}}}"
                else:
                    v_max_str = f"{row['v_max']:.1f}"
            else:
                # Subsequent rows: empty if multirow, otherwise show value
                obst_type_str = (
                    ""
                    if all(v == obst_type_vals[0] for v in obst_type_vals)
                    else row["obst_type"]
                )
                obst_traj_str = (
                    ""
                    if all(v == obst_traj_vals[0] for v in obst_traj_vals)
                    else row["obst_traj"]
                )
                v_max_str = (
                    ""
                    if all(v == v_max_vals[0] for v in v_max_vals)
                    else f"{row['v_max']:.1f}"
                )

            lines.append(
                f"      {exp_num} & {obst_type_str} & {obst_traj_str} & {v_max_str} & {cell_str} \\\\"
            )

        if gi < len(groups) - 1:
            lines.append(r"      \midrule")

    lines.append(r"      \bottomrule")
    lines.append(r"    \end{tabular}")
    lines.append(r"  }")
    lines.append(r"  \vspace{-1.0em}")
    lines.append(r"\end{table}")
    return "\n".join(lines) + "\n"


def generate_hw_dynamic_round2(dynamic_dir, table_output):
    """Generate a LaTeX table and history plots for hw dynamic round2 tests."""
    rows = []

    for test_name, obst_type, obst_traj, v_max, a_max, j_max in HW_DYNAMIC_ROUND2_TESTS:
        test_dir = os.path.join(dynamic_dir, test_name)
        if not os.path.isdir(test_dir):
            print(f"  WARNING: {test_dir} not found, skipping")
            continue

        bag_path = find_bag_in_test_dir(test_dir)
        if bag_path is None:
            print(f"  WARNING: No bag found in {test_dir}, skipping")
            continue

        print(f"  Processing {test_name} ({obst_type}, v_max={v_max}) -> {bag_path}")

        ns = discover_namespace(bag_path)
        comp_stats = get_comp_stats(bag_path, ns)

        rows.append(
            {
                "obst_type": obst_type,
                "obst_traj": obst_traj,
                "v_max": v_max,
                "comp_stats": comp_stats,
            }
        )

        # Generate history plot
        goal_data = extract_goal_arrays(bag_path, ns)
        if goal_data is not None:
            t, p, v_arr, a_arr, j_arr = goal_data
            plot_path = os.path.join(bag_path, f"history_{test_name}.pdf")
            # Adjust position y-axis based on goal position
            if test_name in ("test28", "test29", "test30", "test31"):
                p_ylim_val = (-2, 12)
            else:
                p_ylim_val = (0, 20)
            plot_history(
                t,
                p,
                v_arr,
                a_arr,
                j_arr,
                plot_path,
                v_max=v_max,
                a_max=a_max,
                j_max=j_max,
                p_ylim=p_ylim_val,
                v_ylim=(-v_max * 1.1, v_max * 1.1),
                a_ylim=(-a_max * 1.1, a_max * 1.1),
                j_ylim=(-j_max * 1.1, j_max * 1.1),
            )
        else:
            print(f"    No goal messages found for {test_name}, skipping plot.")

    if not rows:
        print("No data collected. Cannot generate table.")
        return

    caption = (
        "Hardware flight computation times in dynamic environments. "
        "All flights use $a_{\\max}=5$ m/s$^2$ and $j_{\\max}=10$ m/s$^3$."
    )
    table_str = build_dynamic_round2_table(
        rows,
        caption=caption,
        label="tab:hw_dynamic_round2",
    )
    save_table(table_str, table_output)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Analyze SANDO hardware ROS2 bag data."
    )
    parser.add_argument(
        "path", help="Path to a bag folder or parent directory containing bags"
    )
    parser.add_argument("--v_max", type=float, default=5.0, help="Velocity limit [m/s]")
    parser.add_argument(
        "--a_max", type=float, default=20.0, help="Acceleration limit [m/s^2]"
    )
    parser.add_argument("--j_max", type=float, default=100.0, help="Jerk limit [m/s^3]")
    parser.add_argument(
        "--use_tex", action="store_true", help="Use LaTeX for text rendering"
    )
    parser.add_argument(
        "--obstacle_mocap_topics",
        nargs="*",
        default=[],
        help="Mocap topic name(s) (geometry_msgs/PoseStamped) for ground-truth "
        "obstacle positions, e.g. /RR04_tower/world. Required for clearance and "
        "obstacle-estimation-error metrics; also used for obstacle speed when "
        "--obstacle_mocap_twist_topics is not provided (via differentiation).",
    )
    parser.add_argument(
        "--drone_mocap_topic",
        type=str,
        default=None,
        help="Mocap topic carrying the ground-truth drone pose. Accepts either "
        "geometry_msgs/PoseStamped (e.g. /PX01/world) or PoseWithCovarianceStamped "
        "(e.g. /PX01/mavros/vision_pose/pose_cov, which is mavros's external-pose "
        "ingress and typically carries the mocap stream). The message type is "
        "auto-detected from the bag's metadata. When provided, the script uses "
        "this as the drone's true position for both clearance and tracking-error "
        "'actual' signal, instead of the onboard /mavros/local_position/pose "
        "(which is DLIO-fused and can carry localization drift).",
    )
    parser.add_argument(
        "--drone_tf_frame",
        type=str,
        default=None,
        help='Alternative ground-truth drone-pose source: child_frame_id (e.g. '
        '"PX01") to look up in /tf. Selects /tf transforms whose '
        'child_frame_id matches and whose parent is in --drone_tf_parents. '
        'Used when mocap broadcasts drone pose only via /tf rather than as a '
        'dedicated PoseStamped topic. --drone_mocap_topic takes precedence if '
        'both are provided.',
    )
    parser.add_argument(
        "--drone_tf_parents",
        nargs="*",
        default=["world", "map"],
        help="Allowed parent frame_ids for --drone_tf_frame (default: world map).",
    )
    parser.add_argument(
        "--obstacle_mocap_twist_topics",
        nargs="*",
        default=[],
        help="Mocap twist topic name(s) (geometry_msgs/TwistStamped) carrying "
        "obstacle linear velocity directly, e.g. /RR04/mocap/twist. When provided, "
        "obstacle Euclidean speed is computed from these messages (preferred; "
        "avoids the timestamp-jitter spike caused by differentiating pose).",
    )
    parser.add_argument(
        "--num_back_and_forth",
        type=int,
        default=5,
        help="Number of returns to the start position used to detect the end of "
        "the active-flight window (default 5; the drone is commanded to fly back "
        "and forth this many times).",
    )
    parser.add_argument(
        "--mask_near_m",
        type=float,
        default=0.5,
        help="Hysteresis threshold [m] for detecting a return to the start position "
        "(default 0.5).",
    )
    parser.add_argument(
        "--mask_away_m",
        type=float,
        default=2.0,
        help="Hysteresis threshold [m] for confirming the drone has left the start "
        "position before a return can be counted (default 2.0).",
    )
    parser.add_argument(
        "--no_mask",
        action="store_true",
        help="Disable the flight-window mask and report metrics over the entire bag.",
    )
    parser.add_argument(
        "--goal_point",
        action="append",
        default=[],
        type=lambda s: tuple(
            float(x) for x in s.replace("[", "").replace("]", "").replace("(", "").replace(")", "").split(",")
        ),
        help='Single commanded goal point (3D, world frame) as "x,y,z". '
        'Repeat the flag to pass multiple goals, e.g. '
        '--goal_point "6.0,0.0,0.85" --goal_point "-4.0,0.0,0.85". Used '
        'together with --goal_mask_radius_m to exclude time intervals when '
        'the drone has entered the planner\'s goal-stop radius (during which '
        'the planner stops issuing fresh setpoints, so tracking-error and '
        'replan-fail stats from those intervals are not representative).',
    )
    parser.add_argument(
        "--goal_mask_radius_m",
        type=float,
        default=0.6,
        help="Radius [m] within which the drone is considered to be at a goal "
        "for the goal-region mask (default 0.6).",
    )
    parser.add_argument(
        "--no_goal_mask",
        action="store_true",
        help="Disable the goal-region mask even when --goal_points is provided.",
    )
    parser.add_argument(
        "--tracking_reference",
        choices=["goal", "setpoint_trajectory"],
        default="setpoint_trajectory",
        help="Reference signal used as the commanded position for tracking-error "
        "computation. 'setpoint_trajectory' (default) uses "
        "/<ns>/mavros/setpoint_trajectory/local, the smooth Bezier samples actually "
        "streamed to the flight controller (recommended; isolates controller "
        "tracking from replanning discontinuities). 'goal' uses /<ns>/goal, the "
        "per-replan commanded goal.",
    )
    parser.add_argument(
        "--generate_table",
        type=str,
        choices=["static", "dynamic", "dynamic_static", "dynamic_round2"],
        help="Generate LaTeX table: 'static', 'dynamic', 'dynamic_static', or 'dynamic_round2'",
    )
    parser.add_argument(
        "--table_output",
        type=str,
        default=None,
        help="Output path for the LaTeX table (auto-set if not provided)",
    )
    parser.add_argument(
        "--tables_dir",
        type=str,
        default=None,
        help="Directory for LaTeX table output (required when using --generate_table without --table_output)",
    )
    args = parser.parse_args()

    path = os.path.abspath(args.path)

    if args.generate_table:
        tables_dir = args.tables_dir
        if not tables_dir and not args.table_output:
            parser.error("--tables_dir is required when using --generate_table without --table_output")
        if args.generate_table == "static":
            output = args.table_output or os.path.join(tables_dir, "hw_static.tex")
            generate_hw_static_table(path, output)
        elif args.generate_table == "dynamic":
            output = args.table_output or os.path.join(tables_dir, "hw_dynamic.tex")
            generate_hw_dynamic_table(path, output)
        elif args.generate_table == "dynamic_static":
            output = args.table_output or os.path.join(
                tables_dir, "hw_dynamic_static.tex"
            )
            generate_hw_dynamic_static_table(path, output)
        elif args.generate_table == "dynamic_round2":
            output = args.table_output or os.path.join(
                tables_dir, "hw_dynamic_round2.tex"
            )
            generate_hw_dynamic_round2(path, output)
        return

    if is_ros2_bag(path):
        # Single bag
        process_bag(path, args)
    elif os.path.isdir(path):
        # Directory of bags
        bags = sorted(
            [
                os.path.join(path, d)
                for d in os.listdir(path)
                if is_ros2_bag(os.path.join(path, d))
            ]
        )
        if not bags:
            print(f"No ROS2 bags found in {path}")
            sys.exit(1)
        print(f"Found {len(bags)} bag(s) in {path}")
        for bag in bags:
            process_bag(bag, args)
    else:
        print(f"Error: {path} is not a valid bag or directory")
        sys.exit(1)


if __name__ == "__main__":
    main()
