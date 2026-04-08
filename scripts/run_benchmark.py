#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
SANDO Benchmarking Script

This script runs multiple simulations with different configurations and collects
comprehensive performance metrics including:
- Flight success (goal reached)
- Computation times (global planner, corridor generator, local planner, total replanning)
- Flight travel time
- Path length
- Smoothness (jerk integral)
- Constraint violations (SFC, velocity, acceleration, jerk)

Usage:
    # Basic benchmark with default settings
    python3 scripts/run_benchmark.py --setup-bash install/setup.bash

    # Custom number of trials and obstacles
    python3 scripts/run_benchmark.py --setup-bash install/setup.bash --num-trials 10 --num-obstacles 50

    # Specify output directory
    python3 scripts/run_benchmark.py --setup-bash install/setup.bash --output-dir custom_benchmark_data
"""

import argparse
import csv
import datetime
import json
import math
import os
import signal
import subprocess
import time
import yaml
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Empty
from dynus_interfaces.msg import DynTraj, Goal


def update_num_p_in_yaml(yaml_path: str, num_p: int):
    """Update the num_P parameter in a sando.yaml file using text replacement.

    This preserves comments and formatting by doing a regex substitution
    rather than re-serializing the YAML.

    Args:
        yaml_path: Path to sando.yaml config file
        num_p: New value for num_P
    """
    import re

    with open(yaml_path, "r") as f:
        content = f.read()

    # Replace num_P value while preserving the comment
    new_content = re.sub(r"(num_P:\s*)\d+", rf"\g<1>{num_p}", content)

    with open(yaml_path, "w") as f:
        f.write(new_content)

    print(f"Updated num_P to {num_p} in {yaml_path}")


def set_num_p_everywhere(num_p: int):
    """Update num_P in both src and install copies of sando.yaml.

    Args:
        num_p: New value for num_P
    """
    script_dir = Path(__file__).parent
    src_yaml = script_dir.parent / "config" / "sando.yaml"
    # Find the install yaml relative to the workspace
    ws_dir = script_dir.parent.parent.parent  # sando_ws
    install_yaml = (
        ws_dir / "install" / "sando" / "share" / "sando" / "config" / "sando.yaml"
    )

    for yaml_path in [src_yaml, install_yaml]:
        if yaml_path.exists():
            update_num_p_in_yaml(str(yaml_path), num_p)
        else:
            print(f"Warning: {yaml_path} not found, skipping")


def load_sando_params_from_yaml(yaml_path: str) -> dict:
    """Load SANDO parameters from sando.yaml

    Args:
        yaml_path: Path to sando.yaml config file

    Returns:
        Dictionary with keys: drone_bbox (half-extents), v_max, a_max, j_max, global_planner

    Note:
        drone_bbox in sando.yaml contains FULL sizes, so we divide by 2 to get half-extents
    """
    defaults = {
        "drone_bbox": (0.1, 0.1, 0.1),  # half-extents
        "v_max": 5.0,
        "a_max": 10.0,
        "j_max": 50.0,
        "global_planner": "astar_heat",
    }

    try:
        with open(yaml_path, "r") as f:
            config = yaml.safe_load(f)

        # Look for parameters in sando_node.ros__parameters
        params = None
        if "sando_node" in config and "ros__parameters" in config["sando_node"]:
            params = config["sando_node"]["ros__parameters"]
        elif "sando" in config and "ros__parameters" in config["sando"]:
            params = config["sando"]["ros__parameters"]

        if params is None:
            print(f"Warning: ros__parameters not found in {yaml_path}, using defaults")
            return defaults

        result = {}

        # Load drone_bbox (convert from full size to half-extents)
        if "drone_bbox" in params:
            bbox_full = params["drone_bbox"]
            result["drone_bbox"] = (
                bbox_full[0] / 2.0,
                bbox_full[1] / 2.0,
                bbox_full[2] / 2.0,
            )
            print(
                f"Loaded drone_bbox from yaml: {bbox_full} (full) -> {result['drone_bbox']} (half-extents)"
            )
        else:
            result["drone_bbox"] = defaults["drone_bbox"]
            print(
                f"Warning: drone_bbox not found, using default {result['drone_bbox']}"
            )

        # Load constraint limits
        for key in ["v_max", "a_max", "j_max"]:
            if key in params:
                result[key] = float(params[key])
                print(f"Loaded {key} from yaml: {result[key]}")
            else:
                result[key] = defaults[key]
                print(f"Warning: {key} not found, using default {result[key]}")

        # Load global planner
        if "global_planner" in params:
            result["global_planner"] = str(params["global_planner"])
            print(f"Loaded global_planner from yaml: {result['global_planner']}")
        else:
            result["global_planner"] = defaults["global_planner"]
            print(
                f"Warning: global_planner not found, using default {result['global_planner']}"
            )

        return result

    except Exception as e:
        print(f"Warning: Failed to load parameters from {yaml_path}: {e}")
        print(f"Using defaults: {defaults}")
        return defaults


def check_lingering_processes() -> bool:
    """Check if any sando ROS nodes are still running (excludes benchmark script)"""
    try:
        # Check for specific ROS executables, not Python scripts
        result = subprocess.run(
            [
                "pgrep",
                "-x",
                "fake_sim|dynamic_forest_node|sando_node|rviz2|goal_sender",
            ],
            capture_output=True,
            text=True,
        )
        # pgrep returns 0 if processes found, 1 if not found
        return result.returncode == 0
    except:
        return False


def kill_all_sando_processes():
    """Aggressively kill all sando-related ROS processes (but not benchmark script)"""
    # Kill tmux session first (this kills everything inside tmux)
    subprocess.run(
        ["tmux", "kill-session", "-t", "sando_sim"],
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
    )

    # Kill specific ROS node executables by exact name
    for process_name in [
        "rviz2",
        "fake_sim",
        "dynamic_forest_node",
        "sando_node",
        "goal_sender",
        "gzserver",
        "gzclient",
        "ruby",
    ]:
        subprocess.run(
            ["pkill", "-9", "-x", process_name],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
        )

    # Kill run_sim.py but NOT run_benchmark.py
    subprocess.run(
        ["pkill", "-9", "-f", "run_sim.py"],
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
    )

    # Kill tmuxp
    subprocess.run(
        ["pkill", "-9", "tmuxp"], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
    )

    # Kill ros2 launch processes (but be specific)
    for pattern in ["ros2 launch sando", "ros2 launch.*rviz", "ros2 launch.*onboard"]:
        subprocess.run(
            ["pkill", "-9", "-f", pattern],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
        )

    # Kill processes with /NX01/ namespace (ROS nodes)
    subprocess.run(
        ["pkill", "-9", "-f", "/NX01/"],
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
    )

    # Wait for processes to die
    time.sleep(1)


def interpolate_obstacle_position(
    times: List[float], positions: List[List[float]], t_query: float
) -> Tuple[float, float, float]:
    """Interpolate obstacle position at a given time

    Args:
        times: List of timestamps
        positions: List of [x, y, z] positions
        t_query: Time to query

    Returns:
        Interpolated (x, y, z) position
    """
    import numpy as np

    if len(times) == 0:
        return (0.0, 0.0, 0.0)

    if len(times) == 1:
        return tuple(positions[0])

    # Find nearest time or interpolate
    times_arr = np.array(times)
    if t_query <= times_arr[0]:
        return tuple(positions[0])
    if t_query >= times_arr[-1]:
        return tuple(positions[-1])

    # Linear interpolation
    idx = np.searchsorted(times_arr, t_query)
    if idx >= len(times):
        return tuple(positions[-1])

    t0, t1 = times[idx - 1], times[idx]
    p0, p1 = positions[idx - 1], positions[idx]

    alpha = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0

    x = p0[0] + alpha * (p1[0] - p0[0])
    y = p0[1] + alpha * (p1[1] - p0[1])
    z = p0[2] + alpha * (p1[2] - p0[2])

    return (x, y, z)


@dataclass
class BenchmarkMetrics:
    """Container for all benchmark metrics"""

    # Trial metadata
    trial_id: int = 0
    seed: int = 0
    num_obstacles: int = 100
    dynamic_ratio: float = 0.65
    start_x: float = 0.0
    start_y: float = 0.0
    start_z: float = 2.0
    goal_x: float = 100.0
    goal_y: float = 0.0
    goal_z: float = 2.0

    # Success metrics
    flight_success: bool = False
    goal_reached: bool = False
    timeout_reached: bool = False
    collision: bool = False

    # Time metrics (all in seconds)
    flight_travel_time: float = 0.0
    total_replanning_time: float = 0.0
    avg_replanning_time: float = 0.0
    max_replanning_time: float = 0.0
    num_replans: int = 0

    # Computation time breakdown (milliseconds)
    avg_global_planning_time: float = 0.0
    avg_sfc_corridor_time: float = 0.0
    avg_local_traj_time: float = 0.0

    # Path metrics
    path_length: float = 0.0
    straight_line_distance: float = 0.0
    path_efficiency: float = 0.0  # path_length / straight_line_distance

    # Smoothness metrics
    jerk_integral: float = 0.0
    jerk_rms: float = 0.0
    avg_velocity: float = 0.0
    max_velocity: float = 0.0
    avg_acceleration: float = 0.0
    max_acceleration: float = 0.0

    # Constraint violations
    sfc_violation_count: int = 0
    sfc_max_violation: float = 0.0
    vel_violation_count: int = 0
    vel_max_excess: float = 0.0
    acc_violation_count: int = 0
    acc_max_excess: float = 0.0
    jerk_violation_count: int = 0
    jerk_max_excess: float = 0.0

    # Collision metrics (from post-processing)
    collision_count: int = 0
    collision_penetration_max: float = 0.0
    collision_penetration_avg: float = 0.0
    collision_unique_obstacles: int = 0
    collision_free_ratio: float = 1.0
    min_distance_to_obstacles: float = float(
        "inf"
    )  # Minimum distance throughout trajectory

    # Additional info
    notes: str = ""


class BenchmarkMonitor(Node):
    """ROS2 node to monitor simulation and collect metrics"""

    def __init__(
        self,
        namespace: str = "NX01",
        v_max: float = 2.0,
        a_max: float = 2.0,
        j_max: float = 3.0,
        commanded_start: tuple = None,
        commanded_goal: tuple = None,
        trajs_topic: str = "/trajs",
    ):
        super().__init__("benchmark_monitor")

        self.namespace = namespace
        self.v_max = v_max
        self.a_max = a_max
        self.j_max = j_max

        # State tracking
        self.goal_reached = False
        self.goal_reached_signal = (
            False  # True when /goal_reached msg received but conditions not yet met
        )
        self.start_time = None
        self.end_time = None
        self.start_pos = None
        self.goal_pos = None

        # Commanded positions (ground truth for straight line calculation)
        self.commanded_start = list(commanded_start) if commanded_start else None
        self.commanded_goal = list(commanded_goal) if commanded_goal else None

        # Data collection
        self.positions = []
        self.velocities = []
        self.accelerations = []
        self.jerks = []
        self.timestamps = []
        self.replan_times = []
        self.obstacles = {}  # Store latest obstacle info for collision checking
        self.obstacle_trajectories = {}  # Track obstacle positions over time {obs_id: {'times': [], 'positions': [], 'bbox': []}}

        # Create QoS profile for critical topics
        critical_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # QoS profile with large depth to avoid dropping messages during startup
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
        )

        # Subscribers
        self.sub_goal = self.create_subscription(
            Goal, f"/{namespace}/goal", self.goal_callback, goal_qos
        )
        self.get_logger().info(f"Subscribed to /{namespace}/goal")

        self.sub_goal_reached = self.create_subscription(
            Empty,
            f"/{namespace}/goal_reached",
            self.goal_reached_callback,
            critical_qos,
        )
        self.get_logger().info(f"Subscribed to /{namespace}/goal_reached")

        self.sub_point_g = self.create_subscription(
            PointStamped, f"/{namespace}/point_G_term", self.point_g_callback, 10
        )

        self.sub_trajs = self.create_subscription(
            DynTraj, trajs_topic, self.trajs_callback, 10
        )

        self.get_logger().info(f"Benchmark monitor initialized for {namespace}")

    def goal_callback(self, msg: Goal):
        """Track goal (commanded trajectory with p, v, a, j) for all metrics"""
        # Use message header timestamp for accurate timing (avoids callback scheduling delays)
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # Start the clock on the first Goal message received (first command sent)
        if self.start_time is None:
            self.start_time = msg_time
            self.start_pos = [msg.p.x, msg.p.y, msg.p.z]
            self.get_logger().info(
                f"First Goal command received at position: {self.start_pos}"
            )

        # Always collect data (don't stop when goal is reached - we need data for metrics!)
        # Collect position, velocity, acceleration, and jerk directly from Goal message
        self.positions.append([msg.p.x, msg.p.y, msg.p.z])
        self.velocities.append([msg.v.x, msg.v.y, msg.v.z])
        self.accelerations.append([msg.a.x, msg.a.y, msg.a.z])
        self.jerks.append([msg.j.x, msg.j.y, msg.j.z])
        self.timestamps.append(msg_time)

    def goal_reached_callback(self, msg: Empty):
        """Mark goal_reached signal received.

        When the planner publishes goal_reached, trust the signal if the drone
        is within proximity of the goal. The planner may stop publishing /goal
        commands after this, so we cannot rely on the speed check (the last
        commanded velocity may be non-zero from the deceleration phase).
        """
        self.get_logger().info(
            f"goal_reached_callback triggered! Current state: {self.goal_reached}"
        )
        if not self.goal_reached:
            # Trust the planner's signal — only verify proximity (not speed)
            # because the planner may stop publishing commands after goal_reached,
            # leaving the last velocity non-zero.
            if self._check_arrival_conditions(require_speed_check=False):
                self.goal_reached = True
                if self.timestamps:
                    self.end_time = self.timestamps[-1]
                else:
                    self.end_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(
                    "Goal reached (planner signal + proximity < 1.0m)!"
                )
            else:
                # Signal received but drone not near goal yet; keep checking
                self.goal_reached_signal = True
                self.get_logger().info(
                    "goal_reached signal received, waiting for proximity condition..."
                )

    def _check_arrival_conditions(self, require_speed_check: bool = True) -> bool:
        """Check if drone is near goal.

        Args:
            require_speed_check: If True, also require speed < 0.1 m/s.
                                 Set False when planner signal is received (planner may
                                 stop publishing commands, leaving last velocity non-zero).
        """
        goal = self.commanded_goal if self.commanded_goal else self.goal_pos
        if goal is None or len(self.positions) == 0 or len(self.velocities) == 0:
            return False

        # Check distance to goal
        pos = self.positions[-1]
        dx = pos[0] - goal[0]
        dy = pos[1] - goal[1]
        dz = pos[2] - goal[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if not require_speed_check:
            return dist < 1.0  # Relaxed threshold when planner confirms goal reached

        # Check velocity magnitude
        vel = self.velocities[-1]
        speed = math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)

        return dist < 0.5 and speed < 0.1

    def point_g_callback(self, msg: PointStamped):
        """Store goal position"""
        if self.goal_pos is None:
            self.goal_pos = [msg.point.x, msg.point.y, msg.point.z]

    def trajs_callback(self, msg: DynTraj):
        """Collect obstacle information for collision checking from /trajs topic"""
        # Skip agent trajectories, only process obstacles
        if msg.is_agent:
            return

        obs_id = f"obstacle_{msg.id}"
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Extract bbox (DynTraj.bbox contains FULL sizes, need to divide by 2 for half-extents)
        if len(msg.bbox) >= 3:
            half_extents = [
                float(msg.bbox[0]) / 2.0,
                float(msg.bbox[1]) / 2.0,
                float(msg.bbox[2]) / 2.0,
            ]
        else:
            # Default if bbox not provided (0.8m cube -> 0.4m half-extents)
            half_extents = [0.4, 0.4, 0.4]

        # Store latest obstacle data (for metadata)
        self.obstacles[obs_id] = {
            "id": obs_id,
            "position": [msg.pos.x, msg.pos.y, msg.pos.z],
            "half_extents": half_extents,
            "is_dynamic": msg.mode == "analytic" and len(msg.function) > 0,
        }

        # Track obstacle positions over time for accurate collision checking
        if obs_id not in self.obstacle_trajectories:
            self.obstacle_trajectories[obs_id] = {
                "times": [],
                "positions": [],
                "half_extents": half_extents,
                "is_dynamic": msg.mode == "analytic" and len(msg.function) > 0,
            }

        # Append current position and time
        self.obstacle_trajectories[obs_id]["times"].append(current_time)
        self.obstacle_trajectories[obs_id]["positions"].append(
            [msg.pos.x, msg.pos.y, msg.pos.z]
        )

    def compute_metrics(self) -> BenchmarkMetrics:
        """Compute all metrics from collected data"""
        metrics = BenchmarkMetrics()

        # Success
        metrics.goal_reached = self.goal_reached
        metrics.flight_success = self.goal_reached

        # Time
        if self.start_time and self.end_time:
            metrics.flight_travel_time = self.end_time - self.start_time

        # Path length
        if len(self.positions) >= 2:
            path_length = 0.0
            for i in range(1, len(self.positions)):
                dx = self.positions[i][0] - self.positions[i - 1][0]
                dy = self.positions[i][1] - self.positions[i - 1][1]
                dz = self.positions[i][2] - self.positions[i - 1][2]
                path_length += math.sqrt(dx * dx + dy * dy + dz * dz)
            metrics.path_length = path_length

            # Debug: Check if we captured the full trajectory
            first_pos = self.positions[0]
            last_pos = self.positions[-1]
            self.get_logger().info(f"Trajectory data: {len(self.positions)} samples")
            self.get_logger().info(
                f"  First position: [{first_pos[0]:.2f}, {first_pos[1]:.2f}, {first_pos[2]:.2f}]"
            )
            self.get_logger().info(
                f"  Last position:  [{last_pos[0]:.2f}, {last_pos[1]:.2f}, {last_pos[2]:.2f}]"
            )
            if self.commanded_start:
                dist_from_start = math.sqrt(
                    (first_pos[0] - self.commanded_start[0]) ** 2
                    + (first_pos[1] - self.commanded_start[1]) ** 2
                    + (first_pos[2] - self.commanded_start[2]) ** 2
                )
                self.get_logger().info(
                    f"  Distance from commanded start: {dist_from_start:.2f}m"
                )
                if dist_from_start > 1.0:
                    self.get_logger().warn(
                        f"WARNING: First captured position is {dist_from_start:.2f}m from start! Data may be missing."
                    )

            # Efficiency (use commanded positions for accurate straight line distance)
            start_for_calc = (
                self.commanded_start if self.commanded_start else self.start_pos
            )
            goal_for_calc = (
                self.commanded_goal if self.commanded_goal else self.goal_pos
            )

            if start_for_calc and goal_for_calc:
                dx = goal_for_calc[0] - start_for_calc[0]
                dy = goal_for_calc[1] - start_for_calc[1]
                dz = goal_for_calc[2] - start_for_calc[2]
                metrics.straight_line_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                if metrics.straight_line_distance > 0:
                    metrics.path_efficiency = (
                        path_length / metrics.straight_line_distance
                    )

                # Debug output
                self.get_logger().info(
                    f"Path metrics: length={path_length:.2f}m, straight_line={metrics.straight_line_distance:.2f}m, efficiency={metrics.path_efficiency:.3f}"
                )
                if path_length < metrics.straight_line_distance:
                    self.get_logger().warn(
                        f"WARNING: Path length ({path_length:.2f}m) < straight line ({metrics.straight_line_distance:.2f}m)! Check data collection."
                    )

        # Velocity stats
        if len(self.velocities) > 0:
            vel_norms = [
                math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) for v in self.velocities
            ]
            metrics.avg_velocity = sum(vel_norms) / len(vel_norms)
            metrics.max_velocity = max(vel_norms)

            # Velocity violations (component-wise, with tolerance for numerical errors)
            tolerance = 1e-3  # 1mm/s tolerance
            for v in self.velocities:
                for component in v:
                    if abs(component) > self.v_max + tolerance:
                        metrics.vel_violation_count += 1
                        excess = abs(component) - self.v_max
                        metrics.vel_max_excess = max(metrics.vel_max_excess, excess)

        # Acceleration stats and violations
        if len(self.accelerations) > 0:
            acc_norms = [
                math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2) for a in self.accelerations
            ]
            metrics.avg_acceleration = sum(acc_norms) / len(acc_norms)
            metrics.max_acceleration = max(acc_norms)

            # Acceleration violations (component-wise, with tolerance for numerical errors)
            tolerance = 1e-3  # 1mm/s^2 tolerance
            for a in self.accelerations:
                for component in a:
                    if abs(component) > self.a_max + tolerance:
                        metrics.acc_violation_count += 1
                        excess = abs(component) - self.a_max
                        metrics.acc_max_excess = max(metrics.acc_max_excess, excess)

            # Jerk metrics (directly from Goal message - no finite difference needed!)
            if len(self.jerks) > 0:
                # Compute jerk norms
                jerk_norms = []
                for jerk in self.jerks:
                    jerk_norm = math.sqrt(jerk[0] ** 2 + jerk[1] ** 2 + jerk[2] ** 2)
                    jerk_norms.append(jerk_norm)

                # Jerk violations (component-wise, per-axis, with tolerance for numerical errors)
                tolerance = 1e-3  # 1mm/s^3 tolerance
                for jerk in self.jerks:
                    for component in jerk:
                        if abs(component) > self.j_max + tolerance:
                            metrics.jerk_violation_count += 1
                            excess = abs(component) - self.j_max
                            metrics.jerk_max_excess = max(
                                metrics.jerk_max_excess, excess
                            )

                # Jerk integral (L1 norm) and RMS
                if jerk_norms and len(self.timestamps) > 1:
                    metrics.jerk_integral = (
                        sum(jerk_norms)
                        * (self.timestamps[-1] - self.timestamps[0])
                        / len(jerk_norms)
                    )
                    metrics.jerk_rms = math.sqrt(
                        sum(j**2 for j in jerk_norms) / len(jerk_norms)
                    )

        return metrics


def run_single_trial(
    trial_id: int,
    seed: int,
    num_obstacles: int,
    dynamic_ratio: float,
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    setup_bash: str,
    timeout: float = 100.0,
    obs_x_range: Tuple[float, float] = (5.0, 100.0),
    obs_y_range: Tuple[float, float] = (-6.0, 6.0),
    obs_z_range: Tuple[float, float] = (0.5, 4.5),
    visualize: bool = False,
    data_file: Optional[str] = None,
    mode: str = "rviz-only",
    env: Optional[str] = None,
    trajs_topic: str = "/trajs",
    obstacles_json_file: Optional[str] = None,
) -> BenchmarkMetrics:
    """Run a single simulation trial and collect metrics"""

    print(
        f"\nTrial {trial_id}: seed={seed}, obstacles={num_obstacles}, dynamic_ratio={dynamic_ratio}"
    )

    # Thorough cleanup before starting new trial
    print("  Pre-trial cleanup...")
    kill_all_sando_processes()
    print("  Cleanup complete")

    # Load parameters from sando.yaml
    yaml_path = Path(__file__).parent.parent / "config" / "sando.yaml"
    sando_params = load_sando_params_from_yaml(str(yaml_path))

    # Use yaml parameters instead of command-line args
    v_max_actual = sando_params["v_max"]
    a_max_actual = sando_params["a_max"]
    j_max_actual = sando_params["j_max"]
    global_planner = sando_params.get("global_planner", "astar_heat")

    print("Using parameters from sando.yaml:")
    print(f"  global_planner: {global_planner}")
    print(f"  v_max: {v_max_actual} m/s")
    print(f"  a_max: {a_max_actual} m/s²")
    print(f"  j_max: {j_max_actual} m/s³")
    print(f"  drone_bbox: {sando_params['drone_bbox']} (half-extents)\n")

    # Set ROS_DOMAIN_ID to match the simulation (run_sim.py uses 7)
    os.environ["ROS_DOMAIN_ID"] = "20"

    # Initialize ROS2 (check if already initialized to avoid errors)
    try:
        if not rclpy.ok():
            rclpy.init()
    except:
        # Context might exist but not be ok(), try to init anyway
        try:
            rclpy.init()
        except:
            pass  # Already initialized

    # Create monitor node (pass commanded start/goal for accurate straight line calculation)
    monitor = BenchmarkMonitor(
        namespace="NX01",
        v_max=v_max_actual,
        a_max=a_max_actual,
        j_max=j_max_actual,
        commanded_start=start,
        commanded_goal=goal,
        trajs_topic=trajs_topic,
    )

    # Build run_sim.py command
    run_sim_path = Path(__file__).parent / "run_sim.py"

    if mode == "gazebo":
        cmd = [
            "python3",
            str(run_sim_path),
            "--mode",
            "gazebo",
            "--setup-bash",
            setup_bash,
            "--start",
            str(start[0]),
            str(start[1]),
            str(start[2]),
            "--goal",
            str(goal[0]),
            str(goal[1]),
            str(goal[2]),
            "--no-goal-sender",  # Disable automatic goal sending - we'll send manually after rosbag starts
        ]

        if env:
            cmd.extend(["--env", env])

        # Add benchmark data file if specified
        if data_file:
            cmd.extend(["--data-file", data_file])
            cmd.append("--use-benchmark")
            cmd.extend(["--global-planner", global_planner])

        # Headless for benchmarking
        if not visualize:
            cmd.append("--no-gazebo-gui")
            cmd.append("--no-rviz")

    elif mode == "gazebo-dynamic":
        cmd = [
            "python3",
            str(run_sim_path),
            "--mode",
            "gazebo-dynamic",
            "--trajs-topic",
            trajs_topic,
            "--d435",
            "--setup-bash",
            setup_bash,
            "--start",
            str(start[0]),
            str(start[1]),
            str(start[2]),
            "--goal",
            str(goal[0]),
            str(goal[1]),
            str(goal[2]),
            "--num-obstacles",
            str(num_obstacles),
            "--dynamic-ratio",
            str(dynamic_ratio),
            "--obs-x-range",
            str(obs_x_range[0]),
            str(obs_x_range[1]),
            "--obs-y-range",
            str(obs_y_range[0]),
            str(obs_y_range[1]),
            "--obs-z-range",
            str(obs_z_range[0]),
            str(obs_z_range[1]),
            "--seed",
            str(seed),
            "--no-goal-sender",
        ]

        if env:
            cmd.extend(["--env", env])

        # Add benchmark data file if specified
        if data_file:
            cmd.extend(["--data-file", data_file])
            cmd.append("--use-benchmark")
            cmd.extend(["--global-planner", global_planner])

        # Headless for benchmarking
        if not visualize:
            cmd.append("--no-gazebo-gui")
            cmd.append("--no-rviz")

    else:  # rviz-only (default)
        cmd = [
            "python3",
            str(run_sim_path),
            "--mode",
            "rviz-only",
            "--setup-bash",
            setup_bash,
            "--start",
            str(start[0]),
            str(start[1]),
            str(start[2]),
            "--goal",
            str(goal[0]),
            str(goal[1]),
            str(goal[2]),
            "--num-obstacles",
            str(num_obstacles),
            "--dynamic-ratio",
            str(dynamic_ratio),
            "--obs-x-range",
            str(obs_x_range[0]),
            str(obs_x_range[1]),
            "--obs-y-range",
            str(obs_y_range[0]),
            str(obs_y_range[1]),
            "--obs-z-range",
            str(obs_z_range[0]),
            str(obs_z_range[1]),
            "--seed",
            str(seed),
            "--no-goal-sender",  # Disable automatic goal sending - we'll send manually after rosbag starts
        ]

        # Use shared obstacle JSON config if provided
        if obstacles_json_file:
            cmd.extend(["--obstacles-json-file", obstacles_json_file])

        # Add benchmark data file if specified
        if data_file:
            cmd.extend(["--data-file", data_file])
            cmd.append("--use-benchmark")
            cmd.extend(["--global-planner", global_planner])

        # Add --no-rviz unless visualize is True
        if not visualize:
            cmd.append("--no-rviz")

    print(f"Launching simulation: {' '.join(cmd)}")

    # Launch simulation in background
    sim_process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid,  # Create new process group for clean termination
    )

    # Wait for simulation to initialize (nodes to start up)
    if mode in ("gazebo", "gazebo-dynamic"):
        print("  Waiting for simulation to initialize (20s for Gazebo)...")
        time.sleep(20)
    else:
        print("  Waiting for simulation to initialize (5s)...")
        time.sleep(5)

    # Start rosbag recording BEFORE sending goal
    bag_dir = Path(data_file).parent.parent / "bags" if data_file else None
    bag_process = None
    if bag_dir:
        bag_dir.mkdir(exist_ok=True)
        bag_name = f"trial_{trial_id}"
        bag_path = str(bag_dir / bag_name)

        # Topics to record: state, tf, trajs for collision analysis
        record_topics = [
            f"/{monitor.namespace}/goal",
            f"/{monitor.namespace}/state",
            f"/{monitor.namespace}/goal_reached",
            "/tf",
            "/tf_static",
            trajs_topic,
        ]

        bag_cmd = ["ros2", "bag", "record", "-o", bag_path] + record_topics
        print(f"  Starting rosbag recording: {bag_name}")

        # Set ROS_DOMAIN_ID for bag recording
        bag_env = os.environ.copy()
        bag_env["ROS_DOMAIN_ID"] = "20"

        bag_process = subprocess.Popen(
            bag_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=bag_env
        )
        time.sleep(
            10
        )  # Wait for the bag recorder to be fully ready before sending the goal

    # NOW send the goal immediately after rosbag is ready
    # The planner will start immediately and we'll capture everything
    print(f"  Sending goal to {monitor.namespace}: [{goal[0]}, {goal[1]}, {goal[2]}]")
    goal_pub = monitor.create_publisher(
        PoseStamped, f"/{monitor.namespace}/term_goal", 10
    )
    time.sleep(0.3)  # Brief wait for publisher connection

    # Send goal multiple times to ensure it's received (same as goal_sender.py)
    for _ in range(3):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = monitor.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(goal[0])
        goal_msg.pose.position.y = float(goal[1])
        goal_msg.pose.position.z = float(goal[2])
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        goal_pub.publish(goal_msg)
        time.sleep(0.3)

    print("  Goal sent! Planner is now running, capturing all trajectory data...")

    # Check if topics are being published
    print("  Checking for ROS topics...")
    available_topics = monitor.get_topic_names_and_types()
    goal_topic = f"/{monitor.namespace}/goal"
    goal_reached_topic = f"/{monitor.namespace}/goal_reached"

    goal_found = any(goal_topic in topic for topic, _ in available_topics)
    goal_reached_found = any(
        goal_reached_topic in topic for topic, _ in available_topics
    )

    print(f"    - {goal_topic}: {'✓ Found' if goal_found else '✗ NOT FOUND'}")
    print(
        f"    - {goal_reached_topic}: {'✓ Found' if goal_reached_found else '✗ NOT FOUND'}"
    )

    if not goal_found:
        print("    WARNING: Goal topic not found! Available topics with 'goal':")
        for topic, types in available_topics:
            if "goal" in topic.lower():
                print(f"      - {topic}")

    # Monitor simulation
    start_monitor_time = time.time()
    goal_reached_logged = False
    last_status_time = 0  # Track last status print time

    try:
        while time.time() - start_monitor_time < timeout:
            # Spin the monitor node to process callbacks
            rclpy.spin_once(monitor, timeout_sec=0.05)

            # If goal_reached signal was received but conditions not yet met, keep checking
            # Use relaxed check (no speed requirement) since planner already confirmed goal reached
            if monitor.goal_reached_signal and not monitor.goal_reached:
                if monitor._check_arrival_conditions(require_speed_check=False):
                    monitor.goal_reached = True
                    if monitor.timestamps:
                        monitor.end_time = monitor.timestamps[-1]
                    else:
                        monitor.end_time = time.time()
                    monitor.get_logger().info(
                        "Goal reached (planner signal + proximity < 1.0m)!"
                    )

            # Check if goal reached
            if monitor.goal_reached:
                if not goal_reached_logged:
                    elapsed = time.time() - start_monitor_time
                    positions_so_far = len(monitor.positions)
                    print(
                        f"✓ Goal reached after {elapsed:.2f}s! (collected {positions_so_far} position samples)"
                    )
                    print("  Collecting final data for 3 more seconds...")
                    goal_reached_logged = True

                # Wait longer to collect final trajectory points and obstacle data
                time.sleep(3.0)
                print(
                    f"  Trial complete, moving to next trial... (total samples: {len(monitor.positions)})"
                )
                break

            # Check if simulation process died
            if sim_process.poll() is not None:
                print("⚠ Simulation process terminated unexpectedly")
                # Capture and print error output
                stdout, stderr = sim_process.communicate()
                if stderr:
                    print(
                        f"  Error output: {stderr.decode('utf-8', errors='ignore')[:500]}"
                    )
                if stdout:
                    print(f"  Output: {stdout.decode('utf-8', errors='ignore')[:500]}")
                break

            # Periodic status update every 10 seconds (only print once per 10s interval)
            elapsed = time.time() - start_monitor_time
            current_status_interval = int(elapsed / 10)
            if current_status_interval > last_status_time and int(elapsed) > 0:
                last_status_time = current_status_interval
                n_positions = len(monitor.positions)
                n_obstacles = len(monitor.obstacle_trajectories)
                if n_positions > 0:
                    pos = monitor.positions[-1]
                    print(
                        f"  [{int(elapsed)}s] Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), Samples: {n_positions}, Obstacles: {n_obstacles}"
                    )
                else:
                    print(
                        f"  [{int(elapsed)}s] WARNING: No position data collected yet! Obstacles tracked: {n_obstacles}"
                    )

        # Check timeout
        elapsed = time.time() - start_monitor_time
        if elapsed >= timeout and not monitor.goal_reached:
            print(f"✗ Timeout reached after {elapsed:.1f}s (goal not reached)")

    finally:
        # Compute metrics
        metrics = monitor.compute_metrics()
        metrics.trial_id = trial_id
        metrics.seed = seed
        metrics.num_obstacles = num_obstacles
        metrics.dynamic_ratio = dynamic_ratio
        metrics.start_x, metrics.start_y, metrics.start_z = start
        metrics.goal_x, metrics.goal_y, metrics.goal_z = goal
        metrics.timeout_reached = (
            time.time() - start_monitor_time >= timeout
        ) and not monitor.goal_reached

        print(
            f"  Metrics: goal_reached={metrics.goal_reached}, timeout={metrics.timeout_reached}, positions_collected={len(monitor.positions)}"
        )

        # Collision checking skipped here - done in post-processing by analyze_benchmark (C++)

        # Cleanup
        print("  Post-trial cleanup...")

        # Stop rosbag recording
        if bag_process:
            print("  Stopping rosbag recording...")
            bag_process.terminate()
            try:
                bag_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bag_process.kill()
            print(f"  Rosbag saved to: {bag_path}")

        # First, destroy ROS node and shutdown rclpy
        try:
            monitor.destroy_node()
        except Exception as e:
            print(f"    Warning: Error destroying monitor node: {e}")

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"    Warning: Error shutting down rclpy: {e}")

        # Kill simulation process group
        try:
            os.killpg(os.getpgid(sim_process.pid), signal.SIGTERM)
            time.sleep(1)
            try:
                os.killpg(os.getpgid(sim_process.pid), signal.SIGKILL)
            except:
                pass
        except:
            pass

        # Comprehensive cleanup using helper function
        kill_all_sando_processes()
        print("  Cleanup complete")

        # Print trial summary (will be updated by caller with run number)
        # This placeholder will be replaced by the main loop

        return metrics


def save_results(
    metrics_list: List[BenchmarkMetrics],
    output_dir: Path,
    config_name: str,
    verbose: bool = True,
):
    """Save benchmark results to CSV"""
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"benchmark_{config_name}_{timestamp}.csv"
    json_path = output_dir / f"benchmark_{config_name}_{timestamp}.json"

    # Save as CSV
    if metrics_list:
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=asdict(metrics_list[0]).keys())
            writer.writeheader()
            for metrics in metrics_list:
                writer.writerow(asdict(metrics))
        if verbose:
            print(f"Results saved to: {csv_path}")

    # Save as JSON for easier parsing
    with open(json_path, "w") as f:
        json.dump([asdict(m) for m in metrics_list], f, indent=2)
    if verbose:
        print(f"Results saved to: {json_path}")


def main():
    parser = argparse.ArgumentParser(
        description="SANDO Benchmark Runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "--setup-bash",
        "-s",
        type=str,
        required=True,
        help="Path to setup.bash (required)",
    )

    parser.add_argument(
        "--num-trials",
        "-n",
        type=int,
        default=5,
        help="Number of trials to run (default: 5)",
    )

    parser.add_argument(
        "--cases",
        type=str,
        nargs="+",
        choices=["easy", "medium", "hard", "all"],
        default=["all"],
        help="Difficulty cases to run: easy (50 obs), medium (100 obs), hard (200 obs), or all (default: all)",
    )

    parser.add_argument(
        "--num-obstacles",
        type=int,
        default=None,
        help="[DEPRECATED] Number of obstacles - use --cases instead",
    )

    parser.add_argument(
        "--dynamic-ratio",
        type=float,
        default=0.65,
        help="Ratio of dynamic obstacles (default: 0.65)",
    )

    parser.add_argument(
        "--start",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 2.0],
        help="Start position (default: 0 0 2)",
    )

    parser.add_argument(
        "--goal",
        type=float,
        nargs=3,
        default=[105.0, 0.0, 2.0],
        help="Goal position (default: 105 0 2)",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=50.0,
        help="Timeout per trial in seconds (default: 50)",
    )

    parser.add_argument(
        "--output-dir",
        "-o",
        type=str,
        default=None,
        help="Output directory for results (default: benchmark_data/YYYYMMDD_HHMMSS)",
    )

    parser.add_argument(
        "--config-name",
        type=str,
        default="default",
        help="Configuration name for output files (default: default)",
    )

    parser.add_argument(
        "--start-seed", type=int, default=0, help="Starting seed value (default: 0)"
    )

    parser.add_argument(
        "--obstacles-json-dir",
        type=str,
        default=None,
        help="Directory containing shared obstacle JSON configs "
        "(files named obstacles_seed{N}.json). When set, obstacles are "
        "loaded from these files instead of being generated per-trial.",
    )

    parser.add_argument(
        "--v-max",
        type=float,
        default=None,
        help="[DEPRECATED] Maximum velocity - now loaded from sando.yaml",
    )

    parser.add_argument(
        "--a-max",
        type=float,
        default=None,
        help="[DEPRECATED] Maximum acceleration - now loaded from sando.yaml",
    )

    parser.add_argument(
        "--j-max",
        type=float,
        default=None,
        help="[DEPRECATED] Maximum jerk - now loaded from sando.yaml",
    )

    parser.add_argument(
        "--visualize",
        "--viz",
        action="store_true",
        help="Show RViz visualization during benchmark (default: headless)",
    )

    parser.add_argument(
        "--mode",
        type=str,
        choices=["rviz-only", "gazebo", "gazebo-dynamic"],
        default="rviz-only",
        help="Simulation mode: rviz-only (procedural obstacles), gazebo (static world files), "
        "or gazebo-dynamic (Gazebo + dynamic obstacles, unknown environment) [default: rviz-only]",
    )

    parser.add_argument(
        "--env",
        type=str,
        default=None,
        help="Environment name override for gazebo mode (default: auto-mapped from case)",
    )

    parser.add_argument(
        "--num-p-values",
        type=int,
        nargs="+",
        default=None,
        help="List of num_P values to sweep over (e.g., --num-p-values 2 3). "
        "Creates P_<N> subfolders in the output directory for each value.",
    )

    args = parser.parse_args()

    # Map cases to obstacle counts
    case_obstacles = {"easy": 50, "medium": 100, "hard": 200}

    # Map cases to gazebo environment names (for --mode gazebo)
    case_environments = {
        "easy": "easy_forest",
        "medium": "medium_forest",
        "hard": "hard_forest",
    }

    # Determine which cases to run
    if "all" in args.cases:
        cases_to_run = ["easy", "medium", "hard"]
    else:
        cases_to_run = [c for c in args.cases if c != "all"]

    # Determine num_P sweep values (default: no sweep, use whatever is in sando.yaml)
    num_p_values = args.num_p_values if args.num_p_values else [None]

    print(f"\n{'=' * 80}")
    print("SANDO BENCHMARK")
    print(f"{'=' * 80}")
    print(f"Configuration: {args.config_name}")
    print(f"Mode: {args.mode}")
    print(f"Cases to run: {', '.join(cases_to_run)}")
    print(f"Number of trials per case: {args.num_trials}")
    if args.num_p_values:
        print(f"num_P sweep: {args.num_p_values}")
    if args.mode == "rviz-only":
        print(f"Dynamic ratio: {args.dynamic_ratio}")
    print(f"Timeout: {args.timeout}s")
    print(f"{'=' * 80}\n")

    # Read original num_P so we can restore it after the sweep
    original_num_p = None
    if args.num_p_values:
        src_yaml = Path(__file__).parent.parent / "config" / "sando.yaml"
        if src_yaml.exists():
            with open(src_yaml, "r") as f:
                _cfg = yaml.safe_load(f)
            _params = None
            if "sando_node" in _cfg and "ros__parameters" in _cfg["sando_node"]:
                _params = _cfg["sando_node"]["ros__parameters"]
            elif "sando" in _cfg and "ros__parameters" in _cfg["sando"]:
                _params = _cfg["sando"]["ros__parameters"]
            if _params:
                original_num_p = _params.get("num_P")

    try:
        # Outer loop: sweep over num_P values
        for num_p in num_p_values:
            if num_p is not None:
                print(f"\n{'#' * 80}")
                print(f"# SETTING num_P = {num_p}")
                print(f"{'#' * 80}\n")
                set_num_p_everywhere(num_p)

            # Run benchmarks for each case
            for case in cases_to_run:
                # Determine obstacle count, environment, and trajs_topic for this case
                trajs_topic = "/trajs"  # default

                if args.mode == "gazebo":
                    num_obstacles = 0  # Static world, no procedural obstacles
                    env_name = args.env if args.env else case_environments[case]
                elif args.mode == "gazebo-dynamic":
                    num_obstacles = case_obstacles[case]
                    env_name = args.env if args.env else "empty_wo_ground"
                    trajs_topic = "/trajs_ground_truth"
                else:
                    num_obstacles = case_obstacles[case]
                    env_name = None

                # Determine output directory for this case
                if args.output_dir:
                    if num_p is not None:
                        output_dir = Path(args.output_dir) / f"P_{num_p}" / case
                    else:
                        output_dir = Path(args.output_dir) / case
                else:
                    # Default to benchmark_data with case and timestamp
                    base_dir = (
                        Path(__file__).parent.parent
                        / "benchmark_data"
                        / args.config_name
                    )
                    if num_p is not None:
                        base_dir = base_dir / f"P_{num_p}"
                    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    output_dir = base_dir / f"{case}_{timestamp}"

                if args.mode == "gazebo":
                    print(f"\n{'=' * 80}")
                    print(
                        f"RUNNING {case.upper()} CASE (gazebo: {env_name})"
                        + (f" [num_P={num_p}]" if num_p is not None else "")
                    )
                    print(f"{'=' * 80}")
                else:
                    print(f"\n{'=' * 80}")
                    print(
                        f"RUNNING {case.upper()} CASE ({num_obstacles} obstacles)"
                        + (f" [num_P={num_p}]" if num_p is not None else "")
                    )
                    print(f"{'=' * 80}")
                print(f"Output directory: {output_dir}")
                print(f"{'=' * 80}\n")

                # Create CSV directory for SANDO benchmark data
                csv_dir = output_dir / "csv"
                csv_dir.mkdir(parents=True, exist_ok=True)

                # Start timing the benchmark
                benchmark_start_time = time.time()
                benchmark_start_datetime = datetime.datetime.now()

                # Run trials
                metrics_list = []
                for i in range(args.num_trials):
                    seed = args.start_seed + i

                    # Create data file path for this trial
                    data_file = str(csv_dir / f"num_{i}.csv")

                    # Resolve shared obstacle JSON file if provided
                    obs_json = None
                    if args.obstacles_json_dir:
                        obs_json = os.path.join(
                            args.obstacles_json_dir, f"obstacles_seed{seed}.json"
                        )
                        if not os.path.exists(obs_json):
                            print(f"WARNING: Obstacle JSON not found: {obs_json}")
                            obs_json = None

                    try:
                        metrics = run_single_trial(
                            trial_id=i,
                            seed=seed,
                            num_obstacles=num_obstacles,
                            dynamic_ratio=args.dynamic_ratio,
                            start=tuple(args.start),
                            goal=tuple(args.goal),
                            setup_bash=args.setup_bash,
                            timeout=args.timeout,
                            visualize=args.visualize,
                            data_file=data_file,
                            mode=args.mode,
                            env=env_name,
                            trajs_topic=trajs_topic,
                            obstacles_json_file=obs_json,
                        )
                        metrics_list.append(metrics)

                        # Print trial summary
                        status = (
                            "success"
                            if metrics.goal_reached
                            else ("timeout" if metrics.timeout_reached else "failed")
                        )
                        print(f"\n{'=' * 80}")
                        print(f"Run {i + 1} / {args.num_trials} : {status}")
                        print(f"{'=' * 80}\n")

                        # Save intermediate results (silently)
                        save_results(
                            metrics_list, output_dir, args.config_name, verbose=False
                        )

                        # Wait before next trial to ensure clean shutdown
                        if i < args.num_trials - 1:
                            print("Waiting 3 seconds before starting next trial...\n")
                            time.sleep(3)

                    except KeyboardInterrupt:
                        print("\n\nBenchmark interrupted by user")
                        raise
                    except Exception as e:
                        print(f"\nError in trial {i}: {e}")
                        import traceback

                        traceback.print_exc()
                        continue

                # Calculate benchmark duration and end time for this case
                benchmark_end_datetime = datetime.datetime.now()
                benchmark_duration = time.time() - benchmark_start_time

                # Final save for this case
                if metrics_list:
                    save_results(
                        metrics_list, output_dir, args.config_name, verbose=True
                    )

                    # Print overall summary for this case
                    total_trials = len(metrics_list)
                    successful_trials = sum(1 for m in metrics_list if m.goal_reached)

                    print(f"\n{'=' * 80}")
                    print(
                        f"BENCHMARK SUMMARY - {case.upper()} CASE"
                        + (f" [num_P={num_p}]" if num_p is not None else "")
                    )
                    print(f"{'=' * 80}")
                    print(
                        f"Started: {benchmark_start_datetime.strftime('%Y-%m-%d %H:%M:%S')}"
                    )
                    print(
                        f"Ended: {benchmark_end_datetime.strftime('%Y-%m-%d %H:%M:%S')}"
                    )
                    print(f"Duration: {benchmark_duration:.1f}s")
                    print(f"Total trials: {total_trials}")
                    print(
                        f"Success rate: {successful_trials}/{total_trials} ({100 * successful_trials / total_trials:.1f}%)"
                    )
                    print(f"Data saved to: {output_dir}")
                    print(f"{'=' * 80}\n")
                else:
                    print(f"\nNo trials completed successfully for {case} case.")

    except KeyboardInterrupt:
        print("\n\nBenchmark interrupted by user")
    finally:
        # Restore original num_P if we changed it
        if original_num_p is not None:
            print(f"\nRestoring original num_P = {original_num_p}")
            set_num_p_everywhere(original_num_p)

    print(f"\n{'=' * 80}")
    print("ALL BENCHMARKS COMPLETE")
    print(f"{'=' * 80}\n")


if __name__ == "__main__":
    main()
