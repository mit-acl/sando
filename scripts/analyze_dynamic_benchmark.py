#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) Anonymous Author
# Anonymous Institution
# All Rights Reserved
# Authors: Anonymous
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
SANDO Dynamic Benchmark Analyzer

Analyzes benchmark data from dynamic obstacle benchmarks and generates:
1. Statistical summary (console output)
2. CSV summary file
3. LaTeX table for paper

Usage:
    # Analyze all data in a directory
    python3 analyze_dynamic_benchmark.py --data-dir benchmark_data/default/20260204_092346

    # With custom output names
    python3 analyze_dynamic_benchmark.py --data-dir benchmark_data/default/20260204_092346 \
        --output-name my_results \
        --latex-name my_table.tex

    # Analyze multiple configurations
    python3 analyze_dynamic_benchmark.py --data-dir benchmark_data/default/*/benchmark_*.csv
"""

import argparse
import glob
import re
import sys
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import pandas as pd
import math

# ROS2 bag reading
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    HAS_ROSBAG = True
except ImportError:
    HAS_ROSBAG = False
    print("Warning: rosbag2_py not available, bag-based collision analysis disabled")


class BoundingBox:
    """Axis-aligned bounding box for collision detection"""

    def __init__(self, min_x, max_x, min_y, max_y, min_z, max_z):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.min_z = min_z
        self.max_z = max_z

    @classmethod
    def from_center_and_half_extents(cls, cx, cy, cz, hx, hy, hz):
        return cls(cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz)

    def intersects(self, other):
        """Check if this AABB intersects with another AABB"""
        return not (
            self.max_x < other.min_x
            or self.min_x > other.max_x
            or self.max_y < other.min_y
            or self.min_y > other.max_y
            or self.max_z < other.min_z
            or self.min_z > other.max_z
        )

    def distance_to(self, other):
        """Calculate minimum distance between two AABBs"""
        if self.intersects(other):
            return 0.0
        dx = max(0.0, max(self.min_x, other.min_x) - min(self.max_x, other.max_x))
        dy = max(0.0, max(self.min_y, other.min_y) - min(self.max_y, other.max_y))
        dz = max(0.0, max(self.min_z, other.min_z) - min(self.max_z, other.max_z))
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def interpolate_position(
    times: np.ndarray, positions: np.ndarray, t_query: float
) -> np.ndarray:
    """Interpolate position at a given time"""
    if len(times) == 0:
        return np.zeros(3)
    if len(times) == 1:
        return positions[0]

    if t_query <= times[0]:
        return positions[0]
    if t_query >= times[-1]:
        return positions[-1]

    # Linear interpolation
    idx = np.searchsorted(times, t_query)
    if idx >= len(times):
        return positions[-1]

    t0, t1 = times[idx - 1], times[idx]
    p0, p1 = positions[idx - 1], positions[idx]

    alpha = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
    return p0 + alpha * (p1 - p0)


def load_benchmark_data(data_pattern: str) -> pd.DataFrame:
    """Load benchmark CSV files matching pattern

    By default, only loads the most recent (largest) CSV file to avoid
    counting trials from partial/interrupted runs.
    """

    # Handle directory input
    data_path = Path(data_pattern)
    if data_path.is_dir():
        # Look for benchmark_*.csv files in directory (any config name prefix)
        pattern = str(data_path / "benchmark_*.csv")
    else:
        pattern = data_pattern

    csv_files = glob.glob(pattern)

    # Filter out summary files
    csv_files = [f for f in csv_files if "summary" not in Path(f).name.lower()]

    if not csv_files:
        print(f"ERROR: No CSV files found matching: {pattern}")
        print(
            "       Make sure you're pointing to a directory with benchmark_*.csv files"
        )
        sys.exit(1)

    # Filter out any summary files that might match
    csv_files = [f for f in csv_files if "summary" not in Path(f).name.lower()]

    if not csv_files:
        print("ERROR: No valid benchmark CSV files found (only found summary files)")
        sys.exit(1)

    # Sort by modification time and get the most recent file
    csv_files_sorted = sorted(csv_files, key=lambda f: Path(f).stat().st_mtime)
    most_recent_file = csv_files_sorted[-1]

    print(f"Found {len(csv_files)} benchmark CSV file(s)")
    print(
        f"Loading most recent file (to avoid counting partial runs): {Path(most_recent_file).name}"
    )

    # Load only the most recent file
    df = pd.read_csv(most_recent_file)
    print(f"\nTotal trials loaded: {len(df)}")

    # Check if data looks valid
    if "path_length" in df.columns:
        valid_data = (df["path_length"] > 0).sum()
        if valid_data == 0:
            print("\n⚠️  WARNING: All trials have path_length=0.0")
            print("   This suggests no trajectory data was collected.")
            print(
                "   The benchmark may have crashed or not published odometry data.\n"
            )

    return df


def load_computation_data(data_dir: Path) -> dict:
    """Load computation time data from num_*.csv files in csv/ subdirectory

    Returns dict mapping trial_id -> computation stats
    """
    csv_dir = data_dir / "csv"
    if not csv_dir.exists():
        print(f"  Warning: No csv/ subdirectory found at {csv_dir}")
        return {}

    # Find all num_*.csv files
    num_files = sorted(csv_dir.glob("num_*.csv"))
    if not num_files:
        print(f"  Warning: No num_*.csv files found in {csv_dir}")
        return {}

    print(f"  Loading computation data from {len(num_files)} num_*.csv file(s)...")

    computation_stats = {}

    for num_file in num_files:
        # Extract trial number from filename (num_0.csv -> trial 0)
        trial_id = int(num_file.stem.split("_")[1])

        try:
            # Read the CSV file
            comp_df = pd.read_csv(num_file)

            # Filter for successful replans (Result=1)
            successful_replans = comp_df[comp_df["Result"] == 1]

            if len(successful_replans) > 0:
                # Compute statistics for this trial
                stats = {
                    "num_replans": len(successful_replans),
                    "avg_replanning_time": successful_replans[
                        "Total replanning time [ms]"
                    ].mean(),
                    "max_replanning_time": successful_replans[
                        "Total replanning time [ms]"
                    ].max(),
                    "total_replanning_time": successful_replans[
                        "Total replanning time [ms]"
                    ].sum(),
                    "avg_global_planning_time": successful_replans[
                        "Global Planning Time [ms]"
                    ].mean(),
                    "avg_local_traj_time": successful_replans[
                        "Local Traj Time [ms]"
                    ].mean(),
                }

                # Check if CVX Decomposition Time column exists (might be called "CVX Decomposition Time" or "SFC Corridor Time")
                if "CVX Decomposition Time [ms]" in comp_df.columns:
                    stats["avg_sfc_corridor_time"] = successful_replans[
                        "CVX Decomposition Time [ms]"
                    ].mean()
                elif "SFC Corridor Time [ms]" in comp_df.columns:
                    stats["avg_sfc_corridor_time"] = successful_replans[
                        "SFC Corridor Time [ms]"
                    ].mean()
                else:
                    stats["avg_sfc_corridor_time"] = 0.0

                computation_stats[trial_id] = stats
            else:
                # No successful replans
                computation_stats[trial_id] = {
                    "num_replans": 0,
                    "avg_replanning_time": 0.0,
                    "max_replanning_time": 0.0,
                    "total_replanning_time": 0.0,
                    "avg_global_planning_time": 0.0,
                    "avg_sfc_corridor_time": 0.0,
                    "avg_local_traj_time": 0.0,
                }

        except Exception as e:
            print(f"    Warning: Failed to load {num_file.name}: {e}")
            continue

    print(f"  Loaded computation data for {len(computation_stats)} trial(s)")
    return computation_stats


def recompute_metrics_from_bag(
    bag_path: Path,
    goal_pos: Tuple[float, float, float],
    start_pos: Tuple[float, float, float] = (0.0, 0.0, 2.0),
    dist_threshold: float = 0.5,
    speed_threshold: float = 0.1,
) -> Dict:
    """Recompute travel time and path length from rosbag /NX01/goal messages.

    Travel time: from first Goal command to first timestamp where
    distance to goal < dist_threshold AND speed < speed_threshold.

    Path length: sum of Euclidean distances between consecutive positions,
    computed only up to goal arrival. If the bag recording started late
    (first position far from known start), the gap distance is prepended.

    Args:
        bag_path: Path to rosbag directory
        goal_pos: Goal position (x, y, z)
        start_pos: Known start position (x, y, z) for gap detection
        dist_threshold: Distance to goal to consider reached (m)
        speed_threshold: Speed threshold to consider stopped (m/s)

    Returns:
        Dict with 'travel_time' and 'path_length' (None values if goal not reached)
    """
    default_result = {"travel_time": None, "path_length": None}

    if not HAS_ROSBAG:
        print(f"    Warning: Cannot read bag {bag_path}, rosbag2_py not available")
        return default_result

    if not bag_path.exists():
        print(f"    Warning: Bag not found at {bag_path}")
        return default_result

    # Setup bag reader
    storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    goal_msg_type = get_message("sando_interfaces/msg/Goal")

    timestamps = []
    positions = []
    velocities = []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == "/NX01/goal":
            msg = deserialize_message(data, goal_msg_type)
            timestamps.append(t / 1e9)  # Convert to seconds
            positions.append((msg.p.x, msg.p.y, msg.p.z))
            velocities.append((msg.v.x, msg.v.y, msg.v.z))

    del reader

    if len(timestamps) == 0:
        print("    Warning: No /NX01/goal messages found in bag")
        return default_result

    # Detect incomplete bag: check if first recorded position is far from known start
    sx, sy, sz = start_pos
    first_pos = positions[0]
    gap_dist = math.sqrt(
        (first_pos[0] - sx) ** 2 + (first_pos[1] - sy) ** 2 + (first_pos[2] - sz) ** 2
    )

    has_gap = gap_dist > 0.5
    if has_gap:
        print(f"    Note: Bag starts {gap_dist:.1f}m from start (gap detected)")

    # Find first movement index (vel > 0.01 m/s)
    move_start_idx = 0
    for i in range(len(velocities)):
        vx, vy, vz = velocities[i]
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > 0.01:
            move_start_idx = i
            break

    # Find goal arrival index
    gx, gy, gz = goal_pos
    goal_idx = None
    for i in range(len(timestamps)):
        px, py, pz = positions[i]
        vx, vy, vz = velocities[i]
        dist_to_goal = math.sqrt((px - gx) ** 2 + (py - gy) ** 2 + (pz - gz) ** 2)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if dist_to_goal < dist_threshold and speed < speed_threshold:
            goal_idx = i
            break

    # Compute path length up to goal arrival (or end of bag if goal not reached)
    end_idx = goal_idx if goal_idx is not None else len(positions) - 1
    path_length = gap_dist  # prepend gap from start to first recorded position
    for i in range(1, end_idx + 1):
        dx = positions[i][0] - positions[i - 1][0]
        dy = positions[i][1] - positions[i - 1][1]
        dz = positions[i][2] - positions[i - 1][2]
        path_length += math.sqrt(dx * dx + dy * dy + dz * dz)

    # Compute travel time from first movement to goal arrival
    # For gap bags, travel_time is unreliable (gap time estimation is inaccurate
    # due to acceleration), so return None to keep the original CSV value
    if has_gap:
        travel_time = None  # keep original CSV value for gap bags
        if goal_idx is None:
            print(
                f"    Warning: Goal not reached in bag (dist_threshold={dist_threshold}m, speed_threshold={speed_threshold}m/s)"
            )
    elif goal_idx is not None:
        travel_time = timestamps[goal_idx] - timestamps[move_start_idx]
    else:
        travel_time = None
        print(
            f"    Warning: Goal not reached in bag (dist_threshold={dist_threshold}m, speed_threshold={speed_threshold}m/s)"
        )

    return {"travel_time": travel_time, "path_length": path_length}


def recompute_violations_from_bag(
    bag_path: Path,
    vel_limit: float = 5.0,
    acc_limit: float = 20.0,
    jerk_limit: float = 100.0,
    tolerance: float = 1e-3,
) -> Dict:
    """Recompute constraint violation counts from rosbag /NX01/goal messages.

    Uses Linf norm: a violation at a single timestep occurs when ANY axis
    component exceeds the limit + tolerance.

    Args:
        bag_path: Path to rosbag directory
        vel_limit: Velocity limit (m/s)
        acc_limit: Acceleration limit (m/s^2)
        jerk_limit: Jerk limit (m/s^3)
        tolerance: Tolerance added to limits before flagging violation

    Returns:
        Dict with vel_violation_count, vel_total, acc_violation_count, acc_total,
        jerk_violation_count, jerk_total
    """
    default_result = {
        "vel_violation_count": 0,
        "vel_total": 0,
        "acc_violation_count": 0,
        "acc_total": 0,
        "jerk_violation_count": 0,
        "jerk_total": 0,
    }

    if not HAS_ROSBAG:
        print(f"    Warning: Cannot read bag {bag_path}, rosbag2_py not available")
        return default_result

    if not bag_path.exists():
        print(f"    Warning: Bag not found at {bag_path}")
        return default_result

    # Setup bag reader
    storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    goal_msg_type = get_message("sando_interfaces/msg/Goal")

    vel_violation_count = 0
    acc_violation_count = 0
    jerk_violation_count = 0
    total_samples = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == "/NX01/goal":
            msg = deserialize_message(data, goal_msg_type)
            total_samples += 1

            # Velocity: Linf check
            vx, vy, vz = abs(msg.v.x), abs(msg.v.y), abs(msg.v.z)
            if max(vx, vy, vz) > vel_limit + tolerance:
                vel_violation_count += 1

            # Acceleration: Linf check
            ax, ay, az = abs(msg.a.x), abs(msg.a.y), abs(msg.a.z)
            if max(ax, ay, az) > acc_limit + tolerance:
                acc_violation_count += 1

            # Jerk: Linf check
            jx, jy, jz = abs(msg.j.x), abs(msg.j.y), abs(msg.j.z)
            if max(jx, jy, jz) > jerk_limit + tolerance:
                jerk_violation_count += 1

    del reader

    if total_samples == 0:
        print(
            "    Warning: No /NX01/goal messages found in bag for violation analysis"
        )
        return default_result

    return {
        "vel_violation_count": vel_violation_count,
        "vel_total": total_samples,
        "acc_violation_count": acc_violation_count,
        "acc_total": total_samples,
        "jerk_violation_count": jerk_violation_count,
        "jerk_total": total_samples,
    }


def analyze_collision_from_bag(
    bag_path: Path, drone_bbox: Tuple[float, float, float]
) -> Dict:
    """Analyze collisions from rosbag data

    Args:
        bag_path: Path to rosbag directory
        drone_bbox: Drone half-extents (hx, hy, hz)

    Returns:
        Dictionary with collision statistics
    """
    if not HAS_ROSBAG:
        print(f"  Warning: Cannot analyze bag {bag_path}, rosbag2_py not available")
        return {
            "collision_count": 0,
            "min_distance": float("inf"),
            "collision_free_ratio": 1.0,
        }

    if not bag_path.exists():
        print(f"  Warning: Bag not found at {bag_path}")
        return {
            "collision_count": 0,
            "min_distance": float("inf"),
            "collision_free_ratio": 1.0,
        }

    print(f"  Analyzing collision from bag: {bag_path.name}")

    # Setup bag reader
    storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Extract data
    agent_trajectory = []  # [(time, x, y, z), ...]
    obstacle_trajectories = {}  # {obs_id: [(time, x, y, z), ...]}

    # Message types
    goal_msg_type = get_message("sando_interfaces/msg/Goal")
    tf_msg_type = get_message("tf2_msgs/msg/TFMessage")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        timestamp = t / 1e9  # Convert to seconds

        if topic == "/NX01/goal":
            msg = deserialize_message(data, goal_msg_type)
            agent_trajectory.append((timestamp, msg.p.x, msg.p.y, msg.p.z))

        elif topic == "/tf":
            msg = deserialize_message(data, tf_msg_type)
            for transform in msg.transforms:
                frame_id = transform.child_frame_id
                # Look for obstacle frames (typically "obstacle_N" or similar)
                if "obstacle" in frame_id.lower() or frame_id.startswith("obs_"):
                    trans = transform.transform.translation
                    if frame_id not in obstacle_trajectories:
                        obstacle_trajectories[frame_id] = []
                    obstacle_trajectories[frame_id].append(
                        (timestamp, trans.x, trans.y, trans.z)
                    )

    del reader

    if len(agent_trajectory) == 0:
        print("    Warning: No agent trajectory found in bag")
        return {
            "collision_count": 0,
            "min_distance": float("inf"),
            "collision_free_ratio": 1.0,
        }

    # Convert to numpy arrays for efficient processing
    agent_times = np.array([t for t, x, y, z in agent_trajectory])
    agent_positions = np.array([[x, y, z] for t, x, y, z in agent_trajectory])

    obstacle_data = {}
    for obs_id, traj in obstacle_trajectories.items():
        if len(traj) > 0:
            times = np.array([t for t, x, y, z in traj])
            positions = np.array([[x, y, z] for t, x, y, z in traj])
            obstacle_data[obs_id] = {"times": times, "positions": positions}

    print(
        f"    Loaded {len(agent_positions)} agent positions, {len(obstacle_data)} obstacles"
    )

    # Collision checking (point mass - no drone bounding box)
    # NOTE: URDF has collision box at 1.0m but visual mesh scaled to 0.8m
    # The /tf frames come from the visual center, so we use visual size (0.8m)
    # This matches what's published in /tf and what the user sees
    obs_half_extents = (0.4, 0.4, 0.4)  # Visual size: 0.8m cubes

    collisions = 0
    min_distance = float("inf")
    min_dist_info = None  # Track which obstacle/time gave min distance
    collision_free_segments = 0

    for i, (t, px, py, pz) in enumerate(
        zip(
            agent_times,
            agent_positions[:, 0],
            agent_positions[:, 1],
            agent_positions[:, 2],
        )
    ):
        segment_collision_free = True
        for obs_id, obs_data in obstacle_data.items():
            # Interpolate obstacle position at this time
            obs_pos = interpolate_position(obs_data["times"], obs_data["positions"], t)

            # Pre-filter: only consider obstacles within reasonable proximity (5m center-to-center)
            center_dist = np.sqrt(
                (px - obs_pos[0]) ** 2 + (py - obs_pos[1]) ** 2 + (pz - obs_pos[2]) ** 2
            )
            if center_dist > 5.0:
                continue  # Skip obstacles too far away

            # Point-to-AABB distance (Euclidean)
            # Find closest point on obstacle box to drone center
            closest_x = np.clip(
                px, obs_pos[0] - obs_half_extents[0], obs_pos[0] + obs_half_extents[0]
            )
            closest_y = np.clip(
                py, obs_pos[1] - obs_half_extents[1], obs_pos[1] + obs_half_extents[1]
            )
            closest_z = np.clip(
                pz, obs_pos[2] - obs_half_extents[2], obs_pos[2] + obs_half_extents[2]
            )

            # Euclidean distance from drone center to closest point on obstacle surface
            distance = np.sqrt(
                (px - closest_x) ** 2 + (py - closest_y) ** 2 + (pz - closest_z) ** 2
            )

            # Also calculate per-axis distances for debugging
            dist_x = max(0.0, abs(px - obs_pos[0]) - obs_half_extents[0])
            dist_y = max(0.0, abs(py - obs_pos[1]) - obs_half_extents[1])
            dist_z = max(0.0, abs(pz - obs_pos[2]) - obs_half_extents[2])

            if distance < min_distance:
                min_distance = distance
                min_dist_info = {
                    "obstacle": obs_id,
                    "time": t,
                    "agent_pos": (px, py, pz),
                    "obs_pos": tuple(obs_pos),
                    "distance": distance,
                    "closest_point": (closest_x, closest_y, closest_z),
                    "axis_distances": (dist_x, dist_y, dist_z),
                }

            # Check for collision: point mass inside obstacle AABB
            if (
                obs_pos[0] - obs_half_extents[0]
                <= px
                <= obs_pos[0] + obs_half_extents[0]
                and obs_pos[1] - obs_half_extents[1]
                <= py
                <= obs_pos[1] + obs_half_extents[1]
                and obs_pos[2] - obs_half_extents[2]
                <= pz
                <= obs_pos[2] + obs_half_extents[2]
            ):
                segment_collision_free = False
                collisions += 1

        if segment_collision_free:
            collision_free_segments += 1

    collision_free_ratio = (
        collision_free_segments / len(agent_positions)
        if len(agent_positions) > 0
        else 1.0
    )

    result = {
        "collision_count": collisions,
        "min_distance": min_distance if min_distance != float("inf") else 0.0,
        "collision_free_ratio": collision_free_ratio,
        "unique_obstacles": len(obstacle_data),
    }

    print(f"    Collisions: {collisions}, Min distance: {result['min_distance']:.3f}m")
    if min_dist_info:
        gap_cm = min_dist_info["distance"] * 100  # Convert to cm
        obs_size = 2 * obs_half_extents[0]  # Total obstacle size
        dist_x, dist_y, dist_z = min_dist_info["axis_distances"]
        closest_pt = min_dist_info["closest_point"]

        print("    Min distance details:")
        print(f"      Obstacle: {min_dist_info['obstacle']}")
        print(f"      Time: {min_dist_info['time']:.2f}s")
        print(
            f"      Drone center: ({min_dist_info['agent_pos'][0]:.2f}, {min_dist_info['agent_pos'][1]:.2f}, {min_dist_info['agent_pos'][2]:.2f})"
        )
        print(
            f"      Closest point on obstacle: ({closest_pt[0]:.2f}, {closest_pt[1]:.2f}, {closest_pt[2]:.2f})"
        )
        print(f"      Euclidean distance: {gap_cm:.1f}cm")
        print(
            f"      Distance per axis: X={dist_x * 100:.1f}cm, Y={dist_y * 100:.1f}cm, Z={dist_z * 100:.1f}cm"
        )
        print(
            f"      Note: Point mass (no drone bbox) distance to {obs_size * 100:.0f}cm obstacle surface."
        )
    return result


def analyze_collision_from_trajs_bag(
    bag_path: Path,
    drone_bbox: Tuple[float, float, float],
    trajs_topic: str = "/trajs_ground_truth",
) -> Dict:
    """Analyze collisions from rosbag using DynTraj messages on a ground truth topic.

    Unlike analyze_collision_from_bag() which reads /tf frames, this reads DynTraj
    messages directly, extracting per-obstacle bounding boxes from msg.bbox.
    This is needed for the unknown_dynamic benchmark where obstacle sizes vary
    (dynamic 0.8^3, vertical 0.4x0.4x4.0, horizontal 0.4x4.0x0.4).

    Args:
        bag_path: Path to rosbag directory
        drone_bbox: Drone half-extents (hx, hy, hz) - not used (point mass)
        trajs_topic: Topic name for DynTraj ground truth messages

    Returns:
        Dictionary with collision statistics
    """
    default_result = {
        "collision_count": 0,
        "min_distance": float("inf"),
        "collision_free_ratio": 1.0,
        "unique_obstacles": 0,
    }

    if not HAS_ROSBAG:
        print(f"  Warning: Cannot analyze bag {bag_path}, rosbag2_py not available")
        return default_result

    if not bag_path.exists():
        print(f"  Warning: Bag not found at {bag_path}")
        return default_result

    print(f"  Analyzing collision from bag (trajs): {bag_path.name}")

    # Setup bag reader
    storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Extract data
    agent_trajectory = []  # [(time, x, y, z), ...]
    obstacle_trajectories = {}  # {obs_id: {'times': [], 'positions': [], 'half_extents': (hx,hy,hz)}}

    # Message types
    goal_msg_type = get_message("sando_interfaces/msg/Goal")
    dyntraj_msg_type = get_message("sando_interfaces/msg/DynTraj")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        timestamp = t / 1e9  # Convert to seconds

        if topic == "/NX01/goal":
            msg = deserialize_message(data, goal_msg_type)
            agent_trajectory.append((timestamp, msg.p.x, msg.p.y, msg.p.z))

        elif topic == trajs_topic:
            msg = deserialize_message(data, dyntraj_msg_type)
            if msg.is_agent:
                continue

            obs_id = f"obstacle_{msg.id}"

            # Extract per-obstacle half-extents from DynTraj.bbox
            if len(msg.bbox) >= 3:
                half_extents = (
                    float(msg.bbox[0]) / 2.0,
                    float(msg.bbox[1]) / 2.0,
                    float(msg.bbox[2]) / 2.0,
                )
            else:
                half_extents = (0.4, 0.4, 0.4)  # Default: 0.8m cube

            if obs_id not in obstacle_trajectories:
                obstacle_trajectories[obs_id] = {
                    "times": [],
                    "positions": [],
                    "half_extents": half_extents,
                }

            obstacle_trajectories[obs_id]["times"].append(timestamp)
            obstacle_trajectories[obs_id]["positions"].append(
                [msg.pos.x, msg.pos.y, msg.pos.z]
            )

    del reader

    if len(agent_trajectory) == 0:
        print("    Warning: No agent trajectory found in bag")
        return default_result

    # Convert to numpy arrays for efficient processing
    agent_times = np.array([t for t, x, y, z in agent_trajectory])
    agent_positions = np.array([[x, y, z] for t, x, y, z in agent_trajectory])

    obstacle_data = {}
    for obs_id, traj in obstacle_trajectories.items():
        if len(traj["times"]) > 0:
            obstacle_data[obs_id] = {
                "times": np.array(traj["times"]),
                "positions": np.array(traj["positions"]),
                "half_extents": traj["half_extents"],
            }

    print(
        f"    Loaded {len(agent_positions)} agent positions, {len(obstacle_data)} obstacles"
    )

    # Collision checking (point mass - no drone bounding box)
    collisions = 0
    min_distance = float("inf")
    collision_free_segments = 0

    for i, (t, px, py, pz) in enumerate(
        zip(
            agent_times,
            agent_positions[:, 0],
            agent_positions[:, 1],
            agent_positions[:, 2],
        )
    ):
        segment_collision_free = True
        for obs_id, obs_data in obstacle_data.items():
            # Interpolate obstacle position at this time
            obs_pos = interpolate_position(obs_data["times"], obs_data["positions"], t)

            # Pre-filter: skip obstacles too far away
            center_dist = np.sqrt(
                (px - obs_pos[0]) ** 2 + (py - obs_pos[1]) ** 2 + (pz - obs_pos[2]) ** 2
            )
            if center_dist > 5.0:
                continue

            # Per-obstacle half-extents
            hx, hy, hz = obs_data["half_extents"]

            # Point-to-AABB distance
            closest_x = np.clip(px, obs_pos[0] - hx, obs_pos[0] + hx)
            closest_y = np.clip(py, obs_pos[1] - hy, obs_pos[1] + hy)
            closest_z = np.clip(pz, obs_pos[2] - hz, obs_pos[2] + hz)

            distance = np.sqrt(
                (px - closest_x) ** 2 + (py - closest_y) ** 2 + (pz - closest_z) ** 2
            )
            min_distance = min(min_distance, distance)

            # Check collision: point inside obstacle AABB
            if (
                obs_pos[0] - hx <= px <= obs_pos[0] + hx
                and obs_pos[1] - hy <= py <= obs_pos[1] + hy
                and obs_pos[2] - hz <= pz <= obs_pos[2] + hz
            ):
                segment_collision_free = False
                collisions += 1

        if segment_collision_free:
            collision_free_segments += 1

    collision_free_ratio = (
        collision_free_segments / len(agent_positions)
        if len(agent_positions) > 0
        else 1.0
    )

    result = {
        "collision_count": collisions,
        "min_distance": min_distance if min_distance != float("inf") else 0.0,
        "collision_free_ratio": collision_free_ratio,
        "unique_obstacles": len(obstacle_data),
    }

    print(f"    Collisions: {collisions}, Min distance: {result['min_distance']:.3f}m")
    return result


def analyze_collision_from_static_obstacles(
    bag_path: Path, drone_bbox: Tuple[float, float, float], obstacle_csv_path: Path
) -> Dict:
    """Analyze collisions against static cylindrical obstacles from CSV

    Args:
        bag_path: Path to rosbag directory
        drone_bbox: Drone half-extents (hx, hy, hz)
        obstacle_csv_path: Path to obstacle parameters CSV file

    Returns:
        Dictionary with collision statistics
    """
    default_result = {
        "collision_count": 0,
        "min_distance": float("inf"),
        "collision_free_ratio": 1.0,
        "unique_obstacles": 0,
    }

    if not HAS_ROSBAG:
        print(f"  Warning: Cannot analyze bag {bag_path}, rosbag2_py not available")
        return default_result

    if not bag_path.exists():
        print(f"  Warning: Bag not found at {bag_path}")
        return default_result

    if not obstacle_csv_path.exists():
        print(f"  Warning: Obstacle CSV not found at {obstacle_csv_path}")
        return default_result

    print(f"  Analyzing static collisions from bag: {bag_path.name}")

    # Load obstacles from CSV
    obs_df = pd.read_csv(obstacle_csv_path)
    obstacles = []
    for _, row in obs_df.iterrows():
        obstacles.append(
            {
                "id": int(row["id"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "radius": float(row["radius"]),
                "height": float(row["height"]),
            }
        )
    print(f"    Loaded {len(obstacles)} static obstacles from {obstacle_csv_path.name}")

    # Read drone trajectory from bag
    storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    goal_msg_type = get_message("sando_interfaces/msg/Goal")
    agent_trajectory = []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == "/NX01/goal":
            msg = deserialize_message(data, goal_msg_type)
            agent_trajectory.append((msg.p.x, msg.p.y, msg.p.z))

    del reader

    if len(agent_trajectory) == 0:
        print("    Warning: No agent trajectory found in bag")
        return default_result

    print(f"    Loaded {len(agent_trajectory)} agent positions")

    # Collision checking: drone as point mass vs cylinders
    # drone_radius = 0 (point mass assumption)
    collisions = 0
    collision_free_segments = 0
    min_distance = float("inf")
    obstacles_hit = set()

    for px, py, pz in agent_trajectory:
        segment_collision_free = True

        for obs in obstacles:
            cx, cy, cz = obs["x"], obs["y"], obs["z"]
            radius = obs["radius"]
            height = obs["height"]

            # Horizontal distance from point to cylinder center
            horiz_dist = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)
            horiz_clearance = horiz_dist - radius

            # Vertical extent check (point vs cylinder)
            obs_z_min = cz - height / 2.0
            obs_z_max = cz + height / 2.0

            vert_inside = pz > obs_z_min and pz < obs_z_max

            # Track minimum horizontal clearance (clamped to 0)
            dist = max(0.0, horiz_clearance)
            if dist < min_distance:
                min_distance = dist

            # Collision: point inside cylinder (horizontal AND vertical)
            if horiz_clearance < 0 and vert_inside:
                segment_collision_free = False
                collisions += 1
                obstacles_hit.add(obs["id"])

        if segment_collision_free:
            collision_free_segments += 1

    total_segments = len(agent_trajectory)
    collision_free_ratio = (
        collision_free_segments / total_segments if total_segments > 0 else 1.0
    )

    result = {
        "collision_count": collisions,
        "min_distance": min_distance if min_distance != float("inf") else 0.0,
        "collision_free_ratio": collision_free_ratio,
        "unique_obstacles": len(obstacles_hit),
    }

    print(
        f"    Collisions: {collisions}, Unique obstacles hit: {len(obstacles_hit)}, "
        f"Min distance: {result['min_distance']:.3f}m, Collision-free ratio: {collision_free_ratio:.3f}"
    )
    return result


def merge_computation_data(df: pd.DataFrame, computation_stats: dict) -> pd.DataFrame:
    """Merge computation statistics into the main benchmark dataframe"""
    if not computation_stats:
        return df

    # Update each row with computation data
    for idx, row in df.iterrows():
        trial_id = row["trial_id"]
        if trial_id in computation_stats:
            stats = computation_stats[trial_id]
            df.at[idx, "num_replans"] = stats["num_replans"]
            df.at[idx, "avg_replanning_time"] = stats["avg_replanning_time"]
            df.at[idx, "max_replanning_time"] = stats["max_replanning_time"]
            df.at[idx, "total_replanning_time"] = (
                stats["total_replanning_time"] / 1000.0
            )  # Convert ms to s
            df.at[idx, "avg_global_planning_time"] = stats["avg_global_planning_time"]
            df.at[idx, "avg_sfc_corridor_time"] = stats["avg_sfc_corridor_time"]
            df.at[idx, "avg_local_traj_time"] = stats["avg_local_traj_time"]

    return df


def compute_statistics(df: pd.DataFrame, require_collision_free: bool = True) -> dict:
    """Compute comprehensive statistics from benchmark data

    Args:
        df: Benchmark dataframe
        require_collision_free: Legacy parameter, no longer used.
                                Success ALWAYS requires goal_reached AND collision_count==0.
    """

    stats = {}

    # Total trials
    stats["total_trials"] = len(df)

    # Treat trials with flight_travel_time > 50s as timeout/failure
    TIMEOUT_THRESHOLD = 100.0
    if "flight_travel_time" in df.columns:
        over_timeout = df["flight_travel_time"] > TIMEOUT_THRESHOLD
        n_over = over_timeout.sum()
        if n_over > 0:
            print(
                f"  Marking {n_over} trial(s) with flight_travel_time > {TIMEOUT_THRESHOLD}s as timeout/failure"
            )
            df.loc[over_timeout, "goal_reached"] = False
            df.loc[over_timeout, "timeout_reached"] = True

    # Success metrics: always require goal_reached AND collision_free
    stats["success_rate"] = (
        (df["goal_reached"]) & (df["collision_count"] == 0)
    ).mean() * 100
    stats["timeout_rate"] = df["timeout_reached"].mean() * 100
    stats["collision_rate"] = df["collision"].mean() * 100

    # Collision metrics (computed on ALL trials, before filtering for success)
    if "collision_count" in df.columns:
        stats["collision_count_total"] = int(df["collision_count"].sum())
        stats["collision_count_mean"] = df["collision_count"].mean()

        # Collision-free ratio
        stats["collision_free_rate"] = (df["collision_count"] == 0).mean() * 100

        # Among trials with collisions
        trials_with_coll = df[df["collision_count"] > 0]
        if len(trials_with_coll) > 0:
            if "collision_penetration_max" in trials_with_coll.columns:
                stats["collision_penetration_max_avg"] = trials_with_coll[
                    "collision_penetration_max"
                ].mean()
            if "collision_unique_obstacles" in trials_with_coll.columns:
                stats["collision_unique_obstacles_avg"] = trials_with_coll[
                    "collision_unique_obstacles"
                ].mean()

    # Minimum distance to obstacles (computed on ALL trials)
    if "min_distance_to_obstacles" in df.columns:
        values = df["min_distance_to_obstacles"].dropna()
        # Filter out inf values
        values = values[values != float("inf")]
        if len(values) > 0:
            stats["min_distance_to_obstacles_min"] = values.min()
            stats["min_distance_to_obstacles_max"] = values.max()
            stats["min_distance_to_obstacles_mean"] = values.mean()
            stats["min_distance_to_obstacles_std"] = values.std()
        else:
            stats["min_distance_to_obstacles_mean"] = "N/A"
    else:
        stats["min_distance_to_obstacles_mean"] = "N/A (not tracked)"

    # Filter successful trials for performance metrics
    # Always require goal_reached AND collision_free
    successful = df[(df["goal_reached"] == True) & (df["collision_count"] == 0)]
    n_success = len(successful)

    if n_success == 0:
        print("WARNING: No successful trials found!")
        return stats

    stats["n_successful"] = n_success

    # Computation time metrics (convert to ms if needed)
    for metric in [
        "avg_local_traj_time",
        "avg_global_planning_time",
        "avg_sfc_corridor_time",
        "avg_replanning_time",
    ]:
        if metric in successful.columns:
            values = successful[metric].dropna()
            if len(values) > 0:
                stats[f"{metric}_min"] = values.min()
                stats[f"{metric}_max"] = values.max()
                stats[f"{metric}_mean"] = values.mean()
                stats[f"{metric}_std"] = values.std()

    # Travel time
    if "flight_travel_time" in successful.columns:
        values = successful["flight_travel_time"].dropna()
        if len(values) > 0:
            stats["flight_travel_time_min"] = values.min()
            stats["flight_travel_time_max"] = values.max()
            stats["flight_travel_time_mean"] = values.mean()
            stats["flight_travel_time_std"] = values.std()

    # Path length
    if "path_length" in successful.columns:
        values = successful["path_length"].dropna()
        if len(values) > 0:
            stats["path_length_min"] = values.min()
            stats["path_length_max"] = values.max()
            stats["path_length_mean"] = values.mean()
            stats["path_length_std"] = values.std()

    # Path efficiency
    if "path_efficiency" in successful.columns:
        values = successful["path_efficiency"].dropna()
        if len(values) > 0:
            stats["path_efficiency_mean"] = values.mean()

    # Jerk smoothness (RMS)
    if "jerk_rms" in successful.columns:
        values = successful["jerk_rms"].dropna()
        if len(values) > 0:
            stats["jerk_rms_min"] = values.min()
            stats["jerk_rms_max"] = values.max()
            stats["jerk_rms_mean"] = values.mean()
            stats["jerk_rms_std"] = values.std()

    # Jerk integral
    if "jerk_integral" in successful.columns:
        values = successful["jerk_integral"].dropna()
        if len(values) > 0:
            stats["jerk_integral_mean"] = values.mean()

    # Constraint violations (rates among successful trials)
    for viol_type in ["sfc", "vel", "acc", "jerk"]:
        count_col = f"{viol_type}_violation_count"
        total_col = f"{viol_type}_violation_total"
        if count_col in successful.columns and total_col in successful.columns:
            total_violations = successful[count_col].sum()
            total_samples = successful[total_col].sum()
            if total_samples > 0:
                viol_rate = total_violations / total_samples * 100.0
            else:
                viol_rate = 0.0
            stats[f"{viol_type}_violation_rate"] = viol_rate

            # Average count when violations occur
            trials_with_viol = successful[successful[count_col] > 0]
            if len(trials_with_viol) > 0:
                stats[f"{viol_type}_violation_avg_count"] = trials_with_viol[
                    count_col
                ].mean()

    # Number of replans
    if "num_replans" in successful.columns:
        values = successful["num_replans"].dropna()
        if len(values) > 0:
            stats["num_replans_mean"] = values.mean()

    return stats


def print_statistics(stats: dict):
    """Print statistics in a formatted way"""

    print("\n" + "=" * 80)
    print("BENCHMARK ANALYSIS RESULTS")
    print("=" * 80)

    print(f"\n{'OVERVIEW':-^80}")
    print(f"  Total trials: {stats.get('total_trials', 0)}")
    print(f"  Successful trials: {stats.get('n_successful', 0)}")
    print(f"  Success rate: {stats.get('success_rate', 0):.1f}%")
    print(f"  Timeout rate: {stats.get('timeout_rate', 0):.1f}%")
    print(f"  Collision rate: {stats.get('collision_rate', 0):.1f}%")

    print(f"\n{'COMPUTATION TIME (ms)':-^80}")
    for metric in [
        "avg_local_traj_time",
        "avg_global_planning_time",
        "avg_sfc_corridor_time",
        "avg_replanning_time",
    ]:
        min_key = f"{metric}_min"
        max_key = f"{metric}_max"
        mean_key = f"{metric}_mean"
        std_key = f"{metric}_std"

        if mean_key in stats:
            label = metric.replace("avg_", "").replace("_", " ").title()
            print(f"  {label}:")
            print(f"    Min: {stats.get(min_key, 0):.2f} ms")
            print(f"    Max: {stats.get(max_key, 0):.2f} ms")
            print(f"    Mean: {stats.get(mean_key, 0):.2f} ms")
            print(f"    Std: {stats.get(std_key, 0):.2f} ms")

    print(f"\n{'PERFORMANCE METRICS':-^80}")
    if "flight_travel_time_mean" in stats:
        print("  Travel Time:")
        print(f"    Min: {stats.get('flight_travel_time_min', 0):.2f} s")
        print(f"    Max: {stats.get('flight_travel_time_max', 0):.2f} s")
        print(f"    Mean: {stats.get('flight_travel_time_mean', 0):.2f} s")
        print(f"    Std: {stats.get('flight_travel_time_std', 0):.2f} s")

    if "path_length_mean" in stats:
        print("  Path Length:")
        print(f"    Min: {stats.get('path_length_min', 0):.2f} m")
        print(f"    Max: {stats.get('path_length_max', 0):.2f} m")
        print(f"    Mean: {stats.get('path_length_mean', 0):.2f} m")
        print(f"    Std: {stats.get('path_length_std', 0):.2f} m")

    if "path_efficiency_mean" in stats:
        print(f"  Path Efficiency: {stats['path_efficiency_mean']:.3f}")

    print(f"\n{'SMOOTHNESS METRICS':-^80}")
    if "jerk_rms_mean" in stats:
        print("  Jerk RMS:")
        print(f"    Min: {stats.get('jerk_rms_min', 0):.2f} m/s³")
        print(f"    Max: {stats.get('jerk_rms_max', 0):.2f} m/s³")
        print(f"    Mean: {stats.get('jerk_rms_mean', 0):.2f} m/s³")
        print(f"    Std: {stats.get('jerk_rms_std', 0):.2f} m/s³")

    if "jerk_integral_mean" in stats:
        print(f"  Jerk Integral: {stats['jerk_integral_mean']:.2f}")

    print(f"\n{'CONSTRAINT VIOLATIONS':-^80}")
    for viol_type in ["sfc", "vel", "acc", "jerk"]:
        rate_key = f"{viol_type}_violation_rate"
        if rate_key in stats:
            label = viol_type.upper()
            rate = stats[rate_key]
            print(f"  {label} Violation Rate: {rate:.1f}%")

            avg_count_key = f"{viol_type}_violation_avg_count"
            if avg_count_key in stats:
                print(f"    Avg violations when occurs: {stats[avg_count_key]:.1f}")

    print(f"\n{'COLLISION METRICS':-^80}")
    print(f"  Collision-free rate: {stats.get('collision_free_rate', 0):.1f}%")
    print(f"  Total collision events: {stats.get('collision_count_total', 0):.0f}")
    print(f"  Mean collisions per trial: {stats.get('collision_count_mean', 0):.2f}")
    if "collision_penetration_max_avg" in stats:
        print(
            f"  Avg max penetration (when collisions occur): {stats['collision_penetration_max_avg']:.4f} m"
        )
    if "collision_unique_obstacles_avg" in stats:
        print(
            f"  Avg unique obstacles hit: {stats['collision_unique_obstacles_avg']:.1f}"
        )

    print(f"\n{'OTHER METRICS':-^80}")
    if "num_replans_mean" in stats:
        print(f"  Average replans per trial: {stats['num_replans_mean']:.1f}")

    print(f"\n{'MINIMUM DISTANCE TO OBSTACLES':-^80}")
    if "min_distance_to_obstacles_mean" in stats:
        mean_val = stats["min_distance_to_obstacles_mean"]
        if isinstance(mean_val, str):
            print(f"  {mean_val}")
        else:
            print(f"  Min: {stats.get('min_distance_to_obstacles_min', 0):.3f} m")
            print(f"  Max: {stats.get('min_distance_to_obstacles_max', 0):.3f} m")
            print(f"  Mean: {stats.get('min_distance_to_obstacles_mean', 0):.3f} m")
            print(f"  Std: {stats.get('min_distance_to_obstacles_std', 0):.3f} m")
    else:
        print("  N/A (not tracked)")

    print("\n" + "=" * 80 + "\n")


def save_statistics_csv(stats: dict, output_path: Path):
    """Save statistics to CSV file"""

    # Create DataFrame from stats
    df = pd.DataFrame([stats])

    # Save to CSV
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_path, index=False)

    print(f"✓ Statistics saved to CSV: {output_path}")


def generate_latex_table(
    stats: dict,
    config_name: str = "default",
    case_name: str = "Unknown",
    existing_file: Path = None,
    table_type: str = "dynamic",
) -> str:
    """Generate LaTeX table with benchmark results, updating only matching case+SANDO row if table exists.

    Args:
        stats: Dictionary of computed statistics
        config_name: Configuration name for caption
        case_name: Case name (Easy, Medium, Hard)
        existing_file: Path to existing .tex file to update in-place
        table_type: 'dynamic' for dynamic obstacle table, 'static' for static forest table
    """

    # Common stats
    success_rate = stats.get("success_rate", 0)
    travel_time = stats.get("flight_travel_time_mean", 0)
    path_length = stats.get("path_length_mean", 0)
    jerk_integral = stats.get("jerk_integral_mean", 0)
    vel_viol = stats.get("vel_violation_rate", 0)
    acc_viol = stats.get("acc_violation_rate", 0)
    jerk_viol = stats.get("jerk_violation_rate", 0)

    if table_type == "static":
        # Static table columns: Env & Algorithm & Constr(2cols) & R_succ & T_opt_total & T_replan_total & T_trav & L_path & S_jerk & rho_vel & rho_acc & rho_jerk
        total_opt_time = stats.get("avg_local_traj_time_mean", 0)
        total_replan_time = stats.get("avg_replanning_time_mean", 0)

        # Format 9 data values (after Constr. columns)
        data_values = (
            f"{{{success_rate:.1f}}} & {{{total_opt_time:.1f}}} & {{{total_replan_time:.1f}}} & "
            f"\\best{{{travel_time:.1f}}} & {{{path_length:.1f}}} & \\best{{{jerk_integral:.1f}}} & "
            f"{{{vel_viol:.1f}}} & {{{acc_viol:.1f}}} & {{{jerk_viol:.1f}}} \\\\"
        )

        # SANDO row (5th in block, no multirow): 2 constraint columns
        sando_row = f"       & SANDO & Hard & $L_\\infty$ & {data_values}"
    elif table_type == "unknown_dynamic":
        # Unknown dynamic table: no Algorithm column, just Env | data...
        per_opt_time = stats.get("avg_local_traj_time_mean", 0)
        total_replan_time = stats.get("avg_replanning_time_mean", 0)
        cvx_decomp_time = stats.get("avg_sfc_corridor_time_mean", 0)
        min_distance = stats.get("min_distance_to_obstacles_mean", 0)
        min_dist_str = (
            min_distance if isinstance(min_distance, str) else f"{min_distance:.2f}"
        )

        data_values = (
            f"{success_rate:.1f} & {per_opt_time:.1f} & {total_replan_time:.1f} & {cvx_decomp_time:.1f} & "
            f"{travel_time:.1f} & {path_length:.1f} & {jerk_integral:.1f} & {min_dist_str} & "
            f"{vel_viol:.1f} & {acc_viol:.1f} & {jerk_viol:.1f} \\\\"
        )

        sando_row = f"      {case_name} & {data_values}"
    else:
        # Dynamic table columns: Env & Algorithm(2cols) & R_succ & T_per_opt & T_trav & L_path & S_jerk & d_min & rho_vel & rho_acc & rho_jerk
        per_opt_time = stats.get("avg_local_traj_time_mean", 0)
        min_distance = stats.get("min_distance_to_obstacles_mean", 0)
        min_dist_str = (
            min_distance if isinstance(min_distance, str) else f"{min_distance:.2f}"
        )

        data_values = (
            f"{success_rate:.1f} & {per_opt_time:.1f} & "
            f"{travel_time:.1f} & {path_length:.1f} & {jerk_integral:.1f} & {min_dist_str} & "
            f"{vel_viol:.1f} & {acc_viol:.1f} & {jerk_viol:.1f} \\\\"
        )

        sando_row = f"      & \\multicolumn{{2}}{{c}}{{SANDO}} & {data_values}"

    # --- Try to update existing table ---
    if existing_file and existing_file.exists():
        print(f"  Found existing table, updating {case_name} + SANDO row...")
        try:
            content = existing_file.read_text()
            lines = content.split("\n")

            # Track which case block we're in by watching for \multirow{...}{...}{CaseName}
            updated_lines = []
            row_updated = False
            current_case = None
            valid_cases = {"Easy", "Medium", "Hard"}

            for line in lines:
                stripped = line.strip()

                # Track current case from multirow markers (only environment names, not algorithms)
                multirow_matches = re.findall(
                    r"\\multirow\{[^}]*\}\{[^}]*\}\{(\w+)\}", line
                )
                for match in multirow_matches:
                    if match in valid_cases:
                        current_case = match

                # Case 1: SANDO row on same line as multirow (old broken dynamic table format)
                if (
                    case_name in line
                    and "SANDO" in line
                    and "&" in line
                    and "multirow" in line
                ):
                    # Replace with correct format (no multirow, use multicolumn for SANDO)
                    updated_lines.append(sando_row)
                    print(
                        f"  Updated {case_name} + SANDO row (fixed multirow→multicolumn format)"
                    )
                    row_updated = True

                # Case 2: SANDO row in a separate line (static table format)
                elif (
                    current_case == case_name
                    and "SANDO" in line
                    and "&" in line
                    and "\\\\" in line
                    and "multirow" not in line
                ):
                    updated_lines.append(sando_row)
                    print(f"  Updated {case_name} + SANDO row (multi-algorithm format)")
                    row_updated = True

                # Case 3: unknown_dynamic format - rows start with case name directly (no SANDO, no multirow)
                elif (
                    table_type == "unknown_dynamic"
                    and stripped.startswith(case_name)
                    and "&" in stripped
                    and "\\\\" in stripped
                ):
                    updated_lines.append(sando_row)
                    print(f"  Updated {case_name} row (unknown_dynamic format)")
                    row_updated = True

                else:
                    updated_lines.append(line)

            if not row_updated:
                print(
                    f"  Warning: No matching {case_name} + SANDO row found, appending before \\bottomrule..."
                )
                for i in range(len(updated_lines) - 1, -1, -1):
                    if "\\bottomrule" in updated_lines[i]:
                        updated_lines.insert(i, sando_row)
                        break

            return "\n".join(updated_lines)
        except Exception as e:
            print(
                f"  Warning: Could not update existing table ({e}), generating new one..."
            )

    # --- Generate new table ---
    if table_type == "static":
        return _generate_new_static_table(case_name, sando_row, data_values)
    elif table_type == "unknown_dynamic":
        return _generate_new_unknown_dynamic_table(case_name, sando_row, data_values)
    else:
        return _generate_new_dynamic_table(case_name, sando_row, data_values)


def _generate_new_static_table(case_name: str, sando_row: str, data_values: str) -> str:
    """Generate a new static forest benchmark LaTeX table with competitor placeholders"""

    dashes = "{-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\\\"

    cases = ["Easy", "Medium", "Hard"]
    latex = []
    latex.append("\\begin{table*}")
    latex.append(
        "  \\caption{Benchmark results against state-of-the-art methods in static environments. "
        "SANDO outperforms the other methods in terms of travel time and achieves a 100\\% success rate. "
        "Since SUPER performs global path planning for both exploratory and safe trajectories, "
        "we list the corresponding computation times as {Exploratory | Safe} in the Global Path Planning Computation Time column.}"
    )
    latex.append("  \\label{tab:static_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\resizebox{\\textwidth}{!}{")
    latex.append("    \\begin{tabular}{c c c c c c c c c c c c c}")
    latex.append("      \\toprule")
    latex.append("      \\multirow{2}{*}[-0.4ex]{\\textbf{Env}}")
    latex.append("      & \\multirow{2}{*}[-0.4ex]{\\textbf{Algorithm}}")
    latex.append(
        "      & \\multicolumn{2}{c}{\\multirow{2}{*}[-0.4ex]{\\textbf{Constr.}}}"
    )
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Success}}")
    latex.append("      & \\multicolumn{2}{c}{\\textbf{Computation Time}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Performance}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}")
    latex.append("      \\\\")
    latex.append("      \\cmidrule(lr){5-5}")
    latex.append("      \\cmidrule(lr){6-7}")
    latex.append("      \\cmidrule(lr){8-10}")
    latex.append("      \\cmidrule(lr){11-13}")
    latex.append("      &&&&")
    latex.append("      $R_{\\mathrm{succ}}$ [\\%]")
    latex.append("      & $T^{\\mathrm{total}}_{\\mathrm{opt}}$ [ms]")
    latex.append("      & $T^{\\mathrm{total}}_{\\mathrm{replan}}$ [ms]")
    latex.append("      & $T_{\\mathrm{trav}}$ [s]")
    latex.append("      & $L_{\\mathrm{path}}$ [m]")
    latex.append("      & $S_{\\mathrm{jerk}}$ [m/s$^{2}$]")
    latex.append("      & $\\rho_{\\mathrm{vel}}$ [\\%]")
    latex.append("      & $\\rho_{\\mathrm{acc}}$ [\\%]")
    latex.append("      & $\\rho_{\\mathrm{jerk}}$ [\\%]")
    latex.append("      \\\\")
    latex.append("      \\midrule")

    for i, case in enumerate(cases):
        if i > 0:
            latex.append("")
            latex.append("      \\midrule")
            latex.append("")

        # 5 rows per env: EGO-Swarm2, SUPER (L2), SUPER (Linf), BASELINE, SANDO
        latex.append(
            f"      \\multirow{{5}}{{*}}{{{case}}} & EGO-Swarm2 & Soft & $L_\\infty$ & {dashes}"
        )
        latex.append(
            f"       & \\multirow{{2}}{{*}}{{SUPER}} & Soft & $L_2$ & {dashes}"
        )
        latex.append(f"       & & Soft & $L_\\infty$ & {dashes}")
        latex.append(f"       & BASELINE & Hard & $L_\\infty$ & {dashes}")

        # SANDO row — fill with data if this is the matching case, otherwise placeholder
        if case == case_name:
            latex.append(f"       & SANDO & Hard & $L_\\infty$ & {data_values}")
        else:
            latex.append(f"       & SANDO & Hard & $L_\\infty$ & {dashes}")

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table*}")

    return "\n".join(latex)


def _generate_new_dynamic_table(
    case_name: str, sando_row: str, data_values: str
) -> str:
    """Generate a new dynamic obstacle benchmark LaTeX table"""

    latex = []
    latex.append("\\begin{table*}")
    latex.append(
        "  \\caption{Dynamic obstacle benchmarking results: SANDO performance with moving obstacles. "
        "We report success rate, computation time, flight performance, smoothness, safety, and constraint violation metrics.}"
    )
    latex.append("  \\label{tab:dynamic_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\resizebox{\\textwidth}{!}{")
    latex.append("    \\begin{tabular}{c c c c c c c c c c c c}")
    latex.append("      \\toprule")

    latex.append("      \\multirow{2}{*}[-0.4em]{\\textbf{Env}}")
    latex.append(
        "      & \\multicolumn{2}{c}{\\multirow{2}{*}[-0.4em]{\\textbf{Algorithm}}}"
    )
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Success}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Comp. Time}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Performance}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Safety}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}")
    latex.append("      \\\\")

    latex.append("      \\cmidrule(lr){4-4}")
    latex.append("      \\cmidrule(lr){5-5}")
    latex.append("      \\cmidrule(lr){6-8}")
    latex.append("      \\cmidrule(lr){9-9}")
    latex.append("      \\cmidrule(lr){10-12}")

    latex.append("      &&&")
    latex.append("      $R_{\\mathrm{succ}}$ [\\%] &")
    latex.append("      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &")
    latex.append("      $T_{\\mathrm{trav}}$ [s] &")
    latex.append("      $L_{\\mathrm{path}}$ [m] &")
    latex.append("      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &")
    latex.append("      $d_{\\mathrm{min}}$ [m] &")
    latex.append("      $\\rho_{\\mathrm{vel}}$ [\\%] &")
    latex.append("      $\\rho_{\\mathrm{acc}}$ [\\%] &")
    latex.append("      $\\rho_{\\mathrm{jerk}}$ [\\%]")
    latex.append("      \\\\")
    latex.append("      \\midrule")

    latex.append(sando_row)

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table*}")

    return "\n".join(latex)


def _generate_new_unknown_dynamic_table(
    case_name: str, sando_row: str, data_values: str
) -> str:
    """Generate a new unknown dynamic obstacle benchmark LaTeX table.

    Columns:
    Env | R_succ | T_per_opt | T_replan | T_cvx | T_trav | L_path | S_jerk | d_min | rho_vel | rho_acc | rho_jerk
    """

    dashes = "{-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\\\"

    cases = ["Easy", "Medium", "Hard"]
    latex = []
    latex.append("\\begin{table*}")
    latex.append(
        "  \\caption{Benchmark results in unknown dynamic environments. "
        "SANDO navigates using only pointcloud sensing (no ground truth obstacle trajectories). "
        "We report success rate, computation time, flight performance, smoothness, safety, and constraint violation metrics.}"
    )
    latex.append("  \\label{tab:unknown_dynamic_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\resizebox{\\textwidth}{!}{")
    latex.append("    \\begin{tabular}{c c c c c c c c c c c c}")
    latex.append("      \\toprule")

    latex.append("      \\multirow{2}{*}[-0.4em]{\\textbf{Env}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Success}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Comp. Time}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Performance}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Safety}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}")
    latex.append("      \\\\")

    latex.append("      \\cmidrule(lr){2-2}")
    latex.append("      \\cmidrule(lr){3-5}")
    latex.append("      \\cmidrule(lr){6-8}")
    latex.append("      \\cmidrule(lr){9-9}")
    latex.append("      \\cmidrule(lr){10-12}")

    latex.append("      &")
    latex.append("      $R_{\\mathrm{succ}}$ [\\%] &")
    latex.append("      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &")
    latex.append("      $T_{\\mathrm{replan}}$ [ms] &")
    latex.append("      $T_{\\mathrm{STSFC}}$ [ms] &")
    latex.append("      $T_{\\mathrm{trav}}$ [s] &")
    latex.append("      $L_{\\mathrm{path}}$ [m] &")
    latex.append("      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &")
    latex.append("      $d_{\\mathrm{min}}$ [m] &")
    latex.append("      $\\rho_{\\mathrm{vel}}$ [\\%] &")
    latex.append("      $\\rho_{\\mathrm{acc}}$ [\\%] &")
    latex.append("      $\\rho_{\\mathrm{jerk}}$ [\\%]")
    latex.append("      \\\\")
    latex.append("      \\midrule")

    for case in cases:
        if case == case_name:
            latex.append(sando_row)
        else:
            latex.append(f"      {case} & {dashes}")

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table*}")

    return "\n".join(latex)


def analyze_single_case(
    data_dir: Path,
    output_name: str,
    config_name: str,
    latex_output: Path,
    table_type: str = "dynamic",
    goal_pos: Tuple[float, float, float] = (105.0, 0.0, 2.0),
):
    """Analyze a single case directory and update LaTeX table"""

    # Extract case name from directory (easy_, medium_, hard_)
    dir_name = data_dir.name
    if dir_name.startswith("easy_"):
        case_name = "Easy"
    elif dir_name.startswith("medium_"):
        case_name = "Medium"
    elif dir_name.startswith("hard_"):
        case_name = "Hard"
    else:
        case_name = "Unknown"  # Fallback for old directory names

    # Load data
    print("=" * 80)
    print(f"ANALYZING {case_name.upper()} CASE")
    print("=" * 80)
    print(f"\nLoading data from: {data_dir}")
    print(f"Case: {case_name}\n")

    df = load_benchmark_data(str(data_dir))

    # Load computation data from num_*.csv files
    if data_dir.is_dir():
        print("\nLoading computation time data...")
        computation_stats = load_computation_data(data_dir)
        if computation_stats:
            df = merge_computation_data(df, computation_stats)
            print("  ✓ Computation data merged successfully\n")
        else:
            print("  No computation data found (num_*.csv files)\n")

    # Recompute path length from rosbags (with gap detection for late-start bags)
    # Note: travel_time is NOT overwritten — the CSV value from the live monitor
    # is more accurate because it measures actual drone state (via goal_reached callback),
    # whereas bag-based timing measures the commanded trajectory which arrives earlier.
    if data_dir.is_dir():
        bags_dir = data_dir / "bags"
        if bags_dir.exists() and HAS_ROSBAG:
            print(f"Recomputing path length from rosbags (goal={goal_pos})...")

            for idx, row in df.iterrows():
                trial_id = row["trial_id"]
                bag_path = bags_dir / f"trial_{trial_id}"

                if bag_path.exists():
                    metrics = recompute_metrics_from_bag(bag_path, goal_pos)

                    if metrics["path_length"] is not None:
                        df.at[idx, "path_length"] = metrics["path_length"]

                    csv_tt = row["flight_travel_time"]
                    pl = metrics["path_length"]
                    if pl is not None:
                        print(
                            f"  Trial {trial_id}: travel_time={csv_tt:.2f}s (CSV), path_length={pl:.2f}m (bag)"
                        )
                    else:
                        print(f"  Trial {trial_id}: no bag data")
                else:
                    print(f"  Warning: Bag not found for trial {trial_id}")

            print()

    # Analyze collisions from rosbags
    if data_dir.is_dir():
        bags_dir = data_dir / "bags"
        if bags_dir.exists() and HAS_ROSBAG:
            # Default drone bbox (can be loaded from sando.yaml if needed)
            drone_bbox = (0.1, 0.1, 0.1)  # half-extents

            if table_type == "unknown_dynamic":
                # Unknown dynamic collision analysis using DynTraj messages from /trajs_ground_truth
                print("\nAnalyzing collisions from rosbags (trajs_ground_truth)...")

                for idx, row in df.iterrows():
                    trial_id = row["trial_id"]
                    bag_path = bags_dir / f"trial_{trial_id}"

                    if bag_path.exists():
                        collision_result = analyze_collision_from_trajs_bag(
                            bag_path, drone_bbox, trajs_topic="/trajs_ground_truth"
                        )
                        df.at[idx, "collision_count"] = collision_result[
                            "collision_count"
                        ]
                        df.at[idx, "min_distance_to_obstacles"] = collision_result[
                            "min_distance"
                        ]
                        df.at[idx, "collision_free_ratio"] = collision_result[
                            "collision_free_ratio"
                        ]
                        df.at[idx, "collision"] = (
                            collision_result["collision_count"] > 0
                        )
                    else:
                        print(f"  Warning: Bag not found for trial {trial_id}")

                print("  Collision analysis complete (trajs_ground_truth)\n")

            elif table_type == "static":
                # Static obstacle collision analysis using CSV obstacle data
                case_csv_map = {
                    "Easy": "easy_forest_obstacle_parameters.csv",
                    "Medium": "medium_forest_obstacle_parameters.csv",
                    "Hard": "hard_forest_obstacle_parameters.csv",
                }
                csv_filename = case_csv_map.get(case_name)
                if csv_filename:
                    sando_pkg_dir = Path(__file__).parent.parent
                    obstacle_csv_path = (
                        sando_pkg_dir / "benchmark_data" / "static" / csv_filename
                    )
                    print(
                        f"\nAnalyzing static collisions from rosbags (obstacles: {obstacle_csv_path.name})..."
                    )

                    for idx, row in df.iterrows():
                        trial_id = row["trial_id"]
                        bag_path = bags_dir / f"trial_{trial_id}"

                        if bag_path.exists():
                            collision_result = analyze_collision_from_static_obstacles(
                                bag_path, drone_bbox, obstacle_csv_path
                            )
                            df.at[idx, "collision_count"] = collision_result[
                                "collision_count"
                            ]
                            df.at[idx, "min_distance_to_obstacles"] = collision_result[
                                "min_distance"
                            ]
                            df.at[idx, "collision_free_ratio"] = collision_result[
                                "collision_free_ratio"
                            ]
                            df.at[idx, "collision_unique_obstacles"] = collision_result[
                                "unique_obstacles"
                            ]
                            df.at[idx, "collision"] = (
                                collision_result["collision_count"] > 0
                            )
                        else:
                            print(f"  Warning: Bag not found for trial {trial_id}")

                    print("  ✓ Static collision analysis complete\n")
                else:
                    print(
                        f"\n  Warning: No obstacle CSV mapping for case '{case_name}', skipping collision analysis\n"
                    )
            else:
                # Dynamic obstacle collision analysis using /tf frames from bag
                print("\nAnalyzing collisions from rosbags...")

                for idx, row in df.iterrows():
                    trial_id = row["trial_id"]
                    bag_path = bags_dir / f"trial_{trial_id}"

                    if bag_path.exists():
                        collision_result = analyze_collision_from_bag(
                            bag_path, drone_bbox
                        )
                        df.at[idx, "collision_count"] = collision_result[
                            "collision_count"
                        ]
                        df.at[idx, "min_distance_to_obstacles"] = collision_result[
                            "min_distance"
                        ]
                        df.at[idx, "collision_free_ratio"] = collision_result[
                            "collision_free_ratio"
                        ]
                        df.at[idx, "collision"] = (
                            collision_result["collision_count"] > 0
                        )
                    else:
                        print(f"  Warning: Bag not found for trial {trial_id}")

                print("  ✓ Collision analysis complete\n")
        elif bags_dir.exists() and not HAS_ROSBAG:
            print(
                "\n  Warning: Bags found but rosbag2_py not available, skipping collision analysis\n"
            )

    # Recompute constraint violations from rosbags
    if data_dir.is_dir():
        bags_dir = data_dir / "bags"
        if bags_dir.exists() and HAS_ROSBAG:
            print("Recomputing constraint violations from rosbags...")

            for idx, row in df.iterrows():
                trial_id = row["trial_id"]
                bag_path = bags_dir / f"trial_{trial_id}"

                if bag_path.exists():
                    viol_result = recompute_violations_from_bag(bag_path)
                    df.at[idx, "vel_violation_count"] = viol_result[
                        "vel_violation_count"
                    ]
                    df.at[idx, "vel_violation_total"] = viol_result["vel_total"]
                    df.at[idx, "acc_violation_count"] = viol_result[
                        "acc_violation_count"
                    ]
                    df.at[idx, "acc_violation_total"] = viol_result["acc_total"]
                    df.at[idx, "jerk_violation_count"] = viol_result[
                        "jerk_violation_count"
                    ]
                    df.at[idx, "jerk_violation_total"] = viol_result["jerk_total"]
                    print(
                        f"  Trial {trial_id}: vel={viol_result['vel_violation_count']}/{viol_result['vel_total']}, "
                        f"acc={viol_result['acc_violation_count']}/{viol_result['acc_total']}, "
                        f"jerk={viol_result['jerk_violation_count']}/{viol_result['jerk_total']}"
                    )
                else:
                    print(f"  Warning: Bag not found for trial {trial_id}")

            print("  Constraint violation recomputation complete\n")

    # Compute statistics
    print("Computing statistics...")
    stats = compute_statistics(df, require_collision_free=(table_type != "static"))

    # Print results
    print_statistics(stats)

    # Save CSV
    if data_dir.is_dir():
        output_dir = data_dir
    else:
        output_dir = data_dir.parent

    csv_output = output_dir / f"{output_name}.csv"
    save_statistics_csv(stats, csv_output)

    # Generate and save LaTeX table
    print("\nUpdating LaTeX table...")
    latex_code = generate_latex_table(
        stats,
        config_name,
        case_name=case_name,
        existing_file=latex_output,
        table_type=table_type,
    )
    latex_output.parent.mkdir(parents=True, exist_ok=True)
    latex_output.write_text(latex_code)

    print(f"✓ LaTeX table updated for {case_name} case")

    # Summary
    print("\n" + "=" * 80)
    print(f"{case_name.upper()} CASE ANALYSIS COMPLETE")
    print("=" * 80)
    print("\nGenerated files:")
    print(f"  CSV: {csv_output}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Analyze SANDO dynamic benchmark data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "--data-dir",
        type=str,
        required=True,
        help="Path to benchmark data directory or CSV file pattern",
    )

    parser.add_argument(
        "--output-name",
        type=str,
        default="benchmark_summary",
        help="Output filename prefix (default: benchmark_summary)",
    )

    parser.add_argument(
        "--latex-name",
        type=str,
        default="dynamic_benchmark.tex",
        help="LaTeX table filename (default: dynamic_benchmark.tex)",
    )

    parser.add_argument(
        "--latex-dir",
        type=str,
        required=True,
        help="Directory for LaTeX table output (e.g., /path/to/paper/tables)",
    )

    parser.add_argument(
        "--config-name",
        type=str,
        default="default",
        help="Configuration name for table caption (default: default)",
    )

    parser.add_argument(
        "--all-cases",
        action="store_true",
        help="Analyze all cases (easy, medium, hard) from parent directory",
    )

    parser.add_argument(
        "--table-type",
        type=str,
        choices=["dynamic", "static", "unknown_dynamic"],
        default="dynamic",
        help="LaTeX table format: dynamic (default), static (for static forest benchmarks), "
        "or unknown_dynamic (for unknown dynamic obstacle benchmarks)",
    )

    parser.add_argument(
        "--goal-pos",
        type=float,
        nargs=3,
        default=[105.0, 0.0, 2.0],
        metavar=("X", "Y", "Z"),
        help="Goal position for travel time recomputation (default: 105.0 0.0 2.0)",
    )

    args = parser.parse_args()

    # Auto-set latex-name based on table-type if user didn't override
    if (
        args.table_type == "unknown_dynamic"
        and args.latex_name == "dynamic_benchmark.tex"
    ):
        args.latex_name = "unknown_dynamic_sim.tex"

    # Determine which cases to analyze
    if args.all_cases:
        # Find all case directories in parent
        base_dir = Path(args.data_dir)
        if base_dir.is_file():
            base_dir = base_dir.parent

        # Look for easy_*, medium_*, hard_* directories (exclude files like CSVs)
        case_dirs = []
        for pattern in ["easy_*", "medium_*", "hard_*"]:
            matching = sorted([p for p in base_dir.glob(pattern) if p.is_dir()])
            if matching:
                case_dirs.append(matching[-1])  # Use most recent

        if not case_dirs:
            print(
                f"Error: No case directories (easy_*, medium_*, hard_*) found in {base_dir}"
            )
            return

        print("=" * 80)
        print("SANDO DYNAMIC BENCHMARK ANALYZER - ALL CASES")
        print("=" * 80)
        print(f"\nFound {len(case_dirs)} case(s) to analyze:")
        for d in case_dirs:
            print(f"  - {d.name}")
        print()

        # Analyze each case
        latex_output = (
            Path(args.latex_dir) / args.latex_name
        )
        goal_pos = tuple(args.goal_pos)
        for case_dir in case_dirs:
            analyze_single_case(
                case_dir,
                args.output_name,
                args.config_name,
                latex_output,
                table_type=args.table_type,
                goal_pos=goal_pos,
            )

        print("\n" + "=" * 80)
        print("ALL CASES ANALYSIS COMPLETE")
        print("=" * 80)
        print(f"\nLaTeX table updated: {latex_output}")
        print(f"  Include in paper: \\input{{{latex_output.name}}}\n")

    else:
        # Single case analysis
        latex_output = (
            Path(args.latex_dir) / args.latex_name
        )
        goal_pos = tuple(args.goal_pos)
        analyze_single_case(
            Path(args.data_dir),
            args.output_name,
            args.config_name,
            latex_output,
            table_type=args.table_type,
            goal_pos=goal_pos,
        )


if __name__ == "__main__":
    main()
