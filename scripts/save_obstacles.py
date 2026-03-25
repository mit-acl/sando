#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
SANDO Obstacle Data Saver

Subscribes to /trajs topic and saves obstacle information to JSON.
This data can be used later for collision checking.

Usage:
    # Save obstacles to file
    python3 scripts/save_obstacles.py --output obstacles.json --duration 5.0
"""

import argparse
import json
import sys
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from dynus_interfaces.msg import DynTraj


class ObstacleSaver(Node):
    """ROS2 node to capture and save obstacle information from /trajs"""

    def __init__(self, traj_topic: str = "/trajs"):
        super().__init__("obstacle_saver")

        self.obstacles = {}  # Dict to store unique obstacles

        # Subscribe to DynTraj messages
        self.sub_trajs = self.create_subscription(
            DynTraj, traj_topic, self.traj_callback, 10
        )

        self.get_logger().info(f"Subscribed to {traj_topic}")
        self.get_logger().info("Collecting obstacle data...")

    def traj_callback(self, msg: DynTraj):
        """Process DynTraj messages"""
        # Skip agent trajectories, only process obstacles
        if msg.is_agent:
            return

        # Create unique ID from obstacle ID
        obs_id = f"obstacle_{msg.id}"

        # Extract bbox (DynTraj.bbox contains FULL sizes, convert to half-extents)
        if len(msg.bbox) >= 3:
            half_extents = [msg.bbox[0] / 2.0, msg.bbox[1] / 2.0, msg.bbox[2] / 2.0]
        else:
            # Default if bbox not provided (0.8m cube -> 0.4m half-extents)
            half_extents = [0.4, 0.4, 0.4]

        # Extract obstacle data
        obs_data = {
            "id": obs_id,
            "position": [msg.pos.x, msg.pos.y, msg.pos.z],
            "half_extents": half_extents,
            "is_dynamic": msg.mode == "analytic" and len(msg.function) > 0,
            "obstacle_id": msg.id,
        }

        # Add analytical expressions if available
        if len(msg.function) >= 3:
            obs_data["function"] = {
                "x": msg.function[0],
                "y": msg.function[1],
                "z": msg.function[2],
            }
        if len(msg.velocity) >= 3:
            obs_data["velocity"] = {
                "vx": msg.velocity[0],
                "vy": msg.velocity[1],
                "vz": msg.velocity[2],
            }

        # Store or update obstacle
        self.obstacles[obs_id] = obs_data

    def get_obstacle_data(self) -> Dict:
        """Get collected obstacle data"""
        return {
            "obstacles": list(self.obstacles.values()),
            "num_obstacles": len(self.obstacles),
            "num_dynamic": sum(1 for o in self.obstacles.values() if o["is_dynamic"]),
            "num_static": sum(
                1 for o in self.obstacles.values() if not o["is_dynamic"]
            ),
        }


def main():
    parser = argparse.ArgumentParser(
        description="Save obstacle data from ROS topics",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "--output", "-o", type=str, required=True, help="Output JSON file path"
    )

    parser.add_argument(
        "--topic",
        "-t",
        type=str,
        default="/trajs",
        help="DynTraj topic (default: /trajs)",
    )

    parser.add_argument(
        "--duration",
        "-d",
        type=float,
        default=5.0,
        help="Duration to collect data in seconds (default: 5.0)",
    )

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    # Create saver node
    saver = ObstacleSaver(marker_topic=args.topic)

    # Collect data for specified duration
    print(f"Collecting obstacle data for {args.duration}s...")
    start_time = time.time()

    while time.time() - start_time < args.duration:
        rclpy.spin_once(saver, timeout_sec=0.1)

    # Get collected data
    data = saver.get_obstacle_data()

    print(f"\nCollected {data['num_obstacles']} obstacles:")
    print(f"  Dynamic: {data['num_dynamic']}")
    print(f"  Static: {data['num_static']}")

    # Save to JSON
    with open(args.output, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\nObstacle data saved to: {args.output}")

    # Cleanup
    saver.destroy_node()
    rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
