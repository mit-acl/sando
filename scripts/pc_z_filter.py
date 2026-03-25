#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""Filter pointcloud by max z height."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PCFilter(Node):
    def __init__(self):
        super().__init__("pc_z_filter")
        self.declare_parameter("z_max", 2.0)
        self.declare_parameter("input_topic", "/cloud_in")
        self.declare_parameter("output_topic", "/cloud_filtered")

        z_max = self.get_parameter("z_max").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.z_max = z_max
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.cb, 10)
        self.get_logger().info(
            f"Filtering {input_topic} -> {output_topic}, z_max={z_max}"
        )

    def cb(self, msg):
        points = list(point_cloud2.read_points(msg, field_names=None, skip_nans=True))
        filtered = [p for p in points if p[2] <= self.z_max]
        if filtered:
            out = point_cloud2.create_cloud(msg.header, msg.fields, filtered)
        else:
            out = point_cloud2.create_cloud(msg.header, msg.fields, [])
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(PCFilter())


if __name__ == "__main__":
    main()
