#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) Anonymous Author
# Anonymous Institution
# All Rights Reserved
# Authors: Anonymous
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
Relay node that republishes markers/polyhedrons with lifetime=0 (forever).
Useful when playing back bags so markers don't disappear on pause.

Usage:
  ros2 run sando relay_forever.py --ros-args -p namespace:=PX03
  # or directly:
  python3 relay_forever.py --ros-args -p namespace:=PX03
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import MarkerArray
from decomp_ros_msgs.msg import PolyhedronArray


class RelayForever(Node):
    def __init__(self):
        super().__init__("relay_forever")
        self.declare_parameter("namespace", "PX03")
        ns = self.get_parameter("namespace").get_parameter_value().string_value

        # tracked_obstacles
        self.sub_tracked = self.create_subscription(
            MarkerArray, f"/{ns}/tracked_obstacles", self.cb_tracked, 10
        )
        self.pub_tracked = self.create_publisher(
            MarkerArray, f"/{ns}/tracked_obstacles_forever", 10
        )

        # cluster_bounding_boxes
        self.sub_cluster = self.create_subscription(
            MarkerArray, f"/{ns}/cluster_bounding_boxes", self.cb_cluster, 10
        )
        self.pub_cluster = self.create_publisher(
            MarkerArray, f"/{ns}/cluster_bounding_boxes_forever", 10
        )

        # poly_safe
        self.sub_poly = self.create_subscription(
            PolyhedronArray, f"/{ns}/poly_safe", self.cb_poly, 10
        )
        self.pub_poly = self.create_publisher(
            PolyhedronArray, f"/{ns}/poly_safe_forever", 10
        )

        self.get_logger().info(f"Relaying /{ns}/ topics with lifetime=forever")

    def _zero_marker_lifetime(self, msg):
        for marker in msg.markers:
            marker.lifetime = Duration(seconds=0).to_msg()
        return msg

    def cb_tracked(self, msg):
        self.pub_tracked.publish(self._zero_marker_lifetime(msg))

    def cb_cluster(self, msg):
        self.pub_cluster.publish(self._zero_marker_lifetime(msg))

    def cb_poly(self, msg):
        msg.lifetime.sec = 0
        msg.lifetime.nanosec = 0
        self.pub_poly.publish(msg)


def main():
    rclpy.init()
    node = RelayForever()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
