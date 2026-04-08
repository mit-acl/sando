#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""Publish a point cloud with fixed cylindrical obstacles for benchmarking."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct

OBSTACLES = [
    # (x, y, radius)
    (-1.0, 2.0, 0.5),
    (-1.0, -2.0, 0.5),
    (1.0, 0.0, 0.5),
]

RESOLUTION = 0.1
HEIGHT = 3.0  # cylinder height in meters


def make_cylinder_points(cx, cy, radius, resolution, height):
    """Generate point cloud points for a filled cylinder."""
    pts = []
    n = int(np.ceil(radius / resolution))
    h_steps = int(np.ceil(height / resolution))
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            dx = ix * resolution
            dy = iy * resolution
            if dx * dx + dy * dy <= radius * radius:
                for iz in range(-2, h_steps):
                    pts.append((cx + dx, cy + dy, (iz + 0.5) * resolution))
    return pts


def points_to_pc2(points, frame_id="map", stamp=None):
    """Convert list of (x,y,z) tuples to PointCloud2."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("fff", *p) for p in points)
    return msg


class FixedObstaclesPublisher(Node):
    def __init__(self):
        super().__init__("fixed_obstacles_publisher")

        # Build all obstacle points once
        all_pts = []
        for cx, cy, r in OBSTACLES:
            all_pts.extend(make_cylinder_points(cx, cy, r, RESOLUTION, HEIGHT))
        self.get_logger().info(
            f"Generated {len(all_pts)} points for {len(OBSTACLES)} obstacles"
        )
        self.cloud_msg = points_to_pc2(all_pts)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            PointCloud2, "/map_generator/global_cloud", qos
        )
        self.timer = self.create_timer(
            1.0 / 50.0, self.publish
        )  # 50 Hz like random_forest

    def publish(self):
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud_msg)


def main():
    rclpy.init()
    node = FixedObstaclesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
