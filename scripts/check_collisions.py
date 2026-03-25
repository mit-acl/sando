#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
SANDO Collision Checker

Post-processes benchmark data to detect collisions between drone trajectory
and obstacles, accounting for both drone and obstacle bounding boxes.

This script can work in two modes:
1. Rosbag mode: Replay rosbag with TF data to detect collisions
2. CSV mode: Use saved trajectory CSV with obstacle data JSON

Usage:
    # Check collisions from rosbag
    python3 scripts/check_collisions.py \
        --rosbag path/to/benchmark.bag \
        --trajectory benchmark_data/trial_0.csv \
        --drone-bbox 0.3 0.3 0.15

    # Check collisions from saved obstacle data
    python3 scripts/check_collisions.py \
        --trajectory benchmark_data/trial_0.csv \
        --obstacles benchmark_data/obstacles.json \
        --drone-bbox 0.3 0.3 0.15
"""

import argparse
import json
import math
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Tuple, Dict, Optional

import pandas as pd
import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from tf2_ros import Buffer, TransformListener
    from geometry_msgs.msg import TransformStamped
    from visualization_msgs.msg import MarkerArray, Marker
    import rosbag2_py

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available. Rosbag mode will not work.")


@dataclass
class BoundingBox:
    """Axis-aligned bounding box"""

    min_x: float
    min_y: float
    min_z: float
    max_x: float
    max_y: float
    max_z: float

    @staticmethod
    def from_center_and_half_extents(
        cx: float, cy: float, cz: float, hx: float, hy: float, hz: float
    ) -> "BoundingBox":
        """Create AABB from center position and half-extents"""
        return BoundingBox(
            min_x=cx - hx,
            max_x=cx + hx,
            min_y=cy - hy,
            max_y=cy + hy,
            min_z=cz - hz,
            max_z=cz + hz,
        )

    def intersects(self, other: "BoundingBox", margin: float = 0.0) -> bool:
        """Check if this AABB intersects with another (with optional safety margin)"""
        return (
            self.min_x - margin <= other.max_x + margin
            and self.max_x + margin >= other.min_x - margin
            and self.min_y - margin <= other.max_y + margin
            and self.max_y + margin >= other.min_y - margin
            and self.min_z - margin <= other.max_z + margin
            and self.max_z + margin >= other.min_z - margin
        )

    def penetration_depth(self, other: "BoundingBox") -> float:
        """Calculate penetration depth if boxes intersect"""
        if not self.intersects(other):
            return 0.0

        # Calculate overlap in each dimension
        overlap_x = min(self.max_x, other.max_x) - max(self.min_x, other.min_x)
        overlap_y = min(self.max_y, other.max_y) - max(self.min_y, other.min_y)
        overlap_z = min(self.max_z, other.max_z) - max(self.min_z, other.min_z)

        # Return minimum overlap (penetration depth)
        return min(overlap_x, overlap_y, overlap_z)

    def distance_to(self, other: "BoundingBox") -> float:
        """
        Calculate minimum distance between two AABBs

        Returns:
            0.0 if boxes intersect, otherwise minimum Euclidean distance between boxes
        """
        if self.intersects(other):
            return 0.0

        # Calculate separation distance in each axis
        # If boxes overlap in an axis, distance in that axis is 0
        dx = max(0.0, max(self.min_x, other.min_x) - min(self.max_x, other.max_x))
        dy = max(0.0, max(self.min_y, other.min_y) - min(self.max_y, other.max_y))
        dz = max(0.0, max(self.min_z, other.min_z) - min(self.max_z, other.max_z))

        # Euclidean distance
        return math.sqrt(dx * dx + dy * dy + dz * dz)


@dataclass
class Obstacle:
    """Obstacle with position and bounding box"""

    id: str
    position: Tuple[float, float, float]
    half_extents: Tuple[float, float, float]  # (hx, hy, hz)
    is_dynamic: bool = False
    velocity: Optional[Tuple[float, float, float]] = None

    def get_bbox_at_time(self, t: float, ref_time: float = 0.0) -> BoundingBox:
        """Get bounding box at specific time (for dynamic obstacles)"""
        px, py, pz = self.position

        if self.is_dynamic and self.velocity:
            dt = t - ref_time
            px += self.velocity[0] * dt
            py += self.velocity[1] * dt
            pz += self.velocity[2] * dt

        return BoundingBox.from_center_and_half_extents(
            px, py, pz, self.half_extents[0], self.half_extents[1], self.half_extents[2]
        )


@dataclass
class CollisionEvent:
    """Record of a collision"""

    time: float
    drone_position: Tuple[float, float, float]
    obstacle_id: str
    penetration_depth: float
    obstacle_position: Tuple[float, float, float]
    obstacle_half_extents: Tuple[float, float, float]


class CollisionChecker:
    """Check collisions between drone trajectory and obstacles"""

    def __init__(
        self, drone_half_extents: Tuple[float, float, float], safety_margin: float = 0.0
    ):
        """
        Args:
            drone_half_extents: Half-extents of drone bbox (hx, hy, hz)
            safety_margin: Additional safety margin around obstacles (meters)
        """
        self.drone_hx, self.drone_hy, self.drone_hz = drone_half_extents
        self.safety_margin = safety_margin

    def check_trajectory(
        self, trajectory: pd.DataFrame, obstacles: List[Obstacle], dt: float = 0.01
    ) -> Tuple[List[CollisionEvent], Dict]:
        """
        Check trajectory against obstacles

        Args:
            trajectory: DataFrame with columns [t, x, y, z, ...]
            obstacles: List of Obstacle objects
            dt: Time step for trajectory sampling

        Returns:
            (collisions, stats): List of collision events and summary statistics
        """
        collisions = []
        collision_free_segments = 0
        total_segments = 0

        # Process trajectory
        for idx, row in trajectory.iterrows():
            t = row["t"]
            px, py, pz = row["x"], row["y"], row["z"]

            # Create drone bounding box
            drone_bbox = BoundingBox.from_center_and_half_extents(
                px, py, pz, self.drone_hx, self.drone_hy, self.drone_hz
            )

            # Check against all obstacles
            segment_collision_free = True
            for obs in obstacles:
                obs_bbox = obs.get_bbox_at_time(t)

                if drone_bbox.intersects(obs_bbox, self.safety_margin):
                    segment_collision_free = False
                    penetration = drone_bbox.penetration_depth(obs_bbox)

                    collision = CollisionEvent(
                        time=t,
                        drone_position=(px, py, pz),
                        obstacle_id=obs.id,
                        penetration_depth=penetration,
                        obstacle_position=obs.position,
                        obstacle_half_extents=obs.half_extents,
                    )
                    collisions.append(collision)

            if segment_collision_free:
                collision_free_segments += 1
            total_segments += 1

        # Compute statistics
        stats = {
            "total_collisions": len(collisions),
            "unique_obstacles_hit": len(set(c.obstacle_id for c in collisions)),
            "collision_free_segments": collision_free_segments,
            "total_segments": total_segments,
            "collision_free_ratio": collision_free_segments / total_segments
            if total_segments > 0
            else 1.0,
            "max_penetration": max(
                (c.penetration_depth for c in collisions), default=0.0
            ),
            "avg_penetration": sum(c.penetration_depth for c in collisions)
            / len(collisions)
            if collisions
            else 0.0,
        }

        return collisions, stats


def load_obstacles_from_json(json_path: str) -> List[Obstacle]:
    """Load obstacles from JSON file"""
    with open(json_path, "r") as f:
        data = json.load(f)

    obstacles = []
    for obs_data in data["obstacles"]:
        obs = Obstacle(
            id=obs_data["id"],
            position=tuple(obs_data["position"]),
            half_extents=tuple(obs_data["half_extents"]),
            is_dynamic=obs_data.get("is_dynamic", False),
            velocity=tuple(obs_data["velocity"]) if "velocity" in obs_data else None,
        )
        obstacles.append(obs)

    return obstacles


def load_obstacles_from_markers(markers: MarkerArray) -> List[Obstacle]:
    """Extract obstacle information from MarkerArray message"""
    obstacles = []

    for marker in markers.markers:
        # Extract position
        pos = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)

        # Extract half-extents from scale (scale is full size, so divide by 2)
        half_extents = (
            marker.scale.x / 2.0,
            marker.scale.y / 2.0,
            marker.scale.z / 2.0,
        )

        # Determine if dynamic based on namespace or ID pattern
        is_dynamic = "dynamic" in marker.ns or "dyn" in marker.ns.lower()

        obs = Obstacle(
            id=f"{marker.ns}_{marker.id}",
            position=pos,
            half_extents=half_extents,
            is_dynamic=is_dynamic,
        )
        obstacles.append(obs)

    return obstacles


def load_drone_bbox_from_yaml(yaml_path: str) -> Tuple[float, float, float]:
    """Load drone bounding box from sando.yaml

    Note: drone_bbox in sando.yaml contains FULL sizes, so we divide by 2 to get half-extents

    Returns:
        Tuple of (hx, hy, hz) half-extents in meters
    """
    with open(yaml_path, "r") as f:
        config = yaml.safe_load(f)

    # Look for drone_bbox in the config (contains FULL sizes)
    drone_bbox_full = None
    if "sando" in config and "ros__parameters" in config["sando"]:
        params = config["sando"]["ros__parameters"]
        if "drone_bbox" in params:
            drone_bbox_full = params["drone_bbox"]

    if drone_bbox_full is None:
        print(
            "Warning: drone_bbox not found in YAML, using default full size [0.6, 0.6, 0.3]"
        )
        drone_bbox_full = [0.6, 0.6, 0.3]

    # Convert from full size to half-extents
    half_extents = (
        drone_bbox_full[0] / 2.0,
        drone_bbox_full[1] / 2.0,
        drone_bbox_full[2] / 2.0,
    )
    return half_extents


def save_collision_report(
    collisions: List[CollisionEvent],
    stats: Dict,
    output_path: str,
    trajectory_path: str,
):
    """Save collision analysis report"""
    report = {
        "trajectory_file": trajectory_path,
        "statistics": stats,
        "collision_events": [asdict(c) for c in collisions],
    }

    # Save JSON
    json_path = Path(output_path).with_suffix(".json")
    with open(json_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"Collision report saved to: {json_path}")

    # Save summary text
    txt_path = Path(output_path).with_suffix(".txt")
    with open(txt_path, "w") as f:
        f.write("=" * 80 + "\n")
        f.write("COLLISION ANALYSIS REPORT\n")
        f.write("=" * 80 + "\n")
        f.write(f"\nTrajectory: {trajectory_path}\n")
        f.write("\nStatistics:\n")
        for key, value in stats.items():
            f.write(f"  {key}: {value}\n")

        f.write(f"\n{'=' * 80}\n")
        f.write(f"Collision Events ({len(collisions)} total):\n")
        f.write(f"{'=' * 80}\n\n")

        for i, c in enumerate(collisions[:100]):  # Limit to first 100 for readability
            f.write(
                f"[{i + 1}] t={c.time:.3f}s, obstacle={c.obstacle_id}, "
                f"penetration={c.penetration_depth:.4f}m\n"
            )
            f.write(
                f"    Drone pos: ({c.drone_position[0]:.2f}, "
                f"{c.drone_position[1]:.2f}, {c.drone_position[2]:.2f})\n"
            )
            f.write(
                f"    Obs pos: ({c.obstacle_position[0]:.2f}, "
                f"{c.obstacle_position[1]:.2f}, {c.obstacle_position[2]:.2f})\n"
            )
            f.write(
                f"    Obs size: ({c.obstacle_half_extents[0]:.2f}, "
                f"{c.obstacle_half_extents[1]:.2f}, {c.obstacle_half_extents[2]:.2f})\n\n"
            )

        if len(collisions) > 100:
            f.write(f"... and {len(collisions) - 100} more collision events\n")

    print(f"Collision summary saved to: {txt_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Check collisions in SANDO benchmark data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "--trajectory",
        "-t",
        type=str,
        required=True,
        help="Path to trajectory CSV file from benchmark",
    )

    parser.add_argument(
        "--obstacles", "-o", type=str, default=None, help="Path to obstacles JSON file"
    )

    parser.add_argument(
        "--rosbag",
        "-b",
        type=str,
        default=None,
        help="Path to rosbag with obstacle TF data (future support)",
    )

    parser.add_argument(
        "--drone-bbox",
        type=float,
        nargs=3,
        metavar=("HX", "HY", "HZ"),
        default=None,
        help="Drone bounding box half-extents (default: from sando.yaml)",
    )

    parser.add_argument(
        "--sando-yaml", type=str, default=None, help="Path to sando.yaml config file"
    )

    parser.add_argument(
        "--safety-margin",
        type=float,
        default=0.0,
        help="Additional safety margin around obstacles (meters, default: 0.0)",
    )

    parser.add_argument(
        "--output",
        "-out",
        type=str,
        default=None,
        help="Output path for collision report (default: auto-generated)",
    )

    args = parser.parse_args()

    # Load trajectory
    print(f"Loading trajectory: {args.trajectory}")
    try:
        traj_df = pd.read_csv(args.trajectory, comment="#")
        print(f"  Loaded {len(traj_df)} trajectory points")
    except Exception as e:
        print(f"Error loading trajectory: {e}")
        return 1

    # Load drone bounding box
    if args.drone_bbox:
        drone_half_extents = tuple(args.drone_bbox)
        print(f"Using drone bbox from args: {drone_half_extents}")
    elif args.sando_yaml:
        drone_half_extents = load_drone_bbox_from_yaml(args.sando_yaml)
        print(f"Loaded drone bbox from YAML: {drone_half_extents}")
    else:
        # Try to find sando.yaml in standard location
        yaml_path = Path(__file__).parent.parent / "config" / "sando.yaml"
        if yaml_path.exists():
            drone_half_extents = load_drone_bbox_from_yaml(str(yaml_path))
            print(f"Loaded drone bbox from default YAML: {drone_half_extents}")
        else:
            drone_half_extents = (0.3, 0.3, 0.15)
            print(f"Warning: Using default drone bbox: {drone_half_extents}")

    # Load obstacles
    if args.obstacles:
        print(f"Loading obstacles from JSON: {args.obstacles}")
        obstacles = load_obstacles_from_json(args.obstacles)
        print(f"  Loaded {len(obstacles)} obstacles")
    elif args.rosbag:
        print("Error: Rosbag mode not yet implemented")
        print("Please provide obstacles via --obstacles JSON file")
        return 1
    else:
        print("Error: Must provide --obstacles JSON file or --rosbag")
        return 1

    # Create collision checker
    checker = CollisionChecker(
        drone_half_extents=drone_half_extents, safety_margin=args.safety_margin
    )

    # Check collisions
    print(f"\nChecking collisions (safety margin: {args.safety_margin}m)...")
    collisions, stats = checker.check_trajectory(traj_df, obstacles)

    # Print results
    print(f"\n{'=' * 80}")
    print("COLLISION ANALYSIS RESULTS")
    print(f"{'=' * 80}")
    print(f"Total collision events: {stats['total_collisions']}")
    print(f"Unique obstacles hit: {stats['unique_obstacles_hit']}")
    print(
        f"Collision-free segments: {stats['collision_free_segments']} / {stats['total_segments']}"
    )
    print(f"Collision-free ratio: {stats['collision_free_ratio'] * 100:.2f}%")
    if stats["total_collisions"] > 0:
        print(f"Max penetration depth: {stats['max_penetration']:.4f}m")
        print(f"Avg penetration depth: {stats['avg_penetration']:.4f}m")
    print(f"{'=' * 80}\n")

    # Save report
    if args.output:
        output_path = args.output
    else:
        # Auto-generate output path
        traj_path = Path(args.trajectory)
        output_path = traj_path.parent / f"{traj_path.stem}_collision_report"

    save_collision_report(collisions, stats, str(output_path), args.trajectory)

    # Return exit code based on collisions
    return 0 if stats["total_collisions"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
