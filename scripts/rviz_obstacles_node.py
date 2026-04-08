#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
RViz-only Dynamic Obstacles Node (inspired by MADER's dynamic_corridor.py)

This node creates dynamic and static obstacles that:
1. Publish trajectory information to /trajs (for planner collision avoidance)
2. Visualize obstacles as markers in RViz (no Gazebo needed)
3. Much lighter than full Gazebo simulation

Usage:
    ros2 run sando rviz_obstacles_node --ros-args \
        -p total_num_obs:=50 \
        -p dynamic_ratio:=0.65 \
        -p x_min:=2.0 -p x_max:=75.0 \
        -p y_min:=-3.0 -p y_max:=3.0 \
        -p z_min:=1.0 -p z_max:=2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import random
import math

from dynus_interfaces.msg import DynTraj
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Colors
COLOR_STATIC = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Blue
COLOR_DYNAMIC = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red


class RVizObstaclesNode(Node):
    def __init__(self):
        super().__init__("rviz_obstacles_node")

        # Declare parameters
        self.declare_parameter("total_num_obs", 50)
        self.declare_parameter("dynamic_ratio", 0.65)
        self.declare_parameter("x_min", 2.0)
        self.declare_parameter("x_max", 75.0)
        self.declare_parameter("y_min", -3.0)
        self.declare_parameter("y_max", 3.0)
        self.declare_parameter("z_min", 1.0)
        self.declare_parameter("z_max", 2.0)
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("slower_min", 1.1)
        self.declare_parameter("slower_max", 1.1)
        self.declare_parameter("bbox_dynamic", [0.8, 0.8, 0.8])
        self.declare_parameter("bbox_static_vert", [0.4, 0.4, 4.0])
        self.declare_parameter("bbox_static_horiz", [0.4, 8.0, 0.4])
        self.declare_parameter("percentage_vert", 0.35)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("seed", 0)

        # Get parameters
        self.total_num_obs = self.get_parameter("total_num_obs").value
        self.dynamic_ratio = self.get_parameter("dynamic_ratio").value
        self.x_min = self.get_parameter("x_min").value
        self.x_max = self.get_parameter("x_max").value
        self.y_min = self.get_parameter("y_min").value
        self.y_max = self.get_parameter("y_max").value
        self.z_min = self.get_parameter("z_min").value
        self.z_max = self.get_parameter("z_max").value
        self.scale = self.get_parameter("scale").value
        self.slower_min = self.get_parameter("slower_min").value
        self.slower_max = self.get_parameter("slower_max").value
        self.bbox_dynamic = self.get_parameter("bbox_dynamic").value
        self.bbox_static_vert = self.get_parameter("bbox_static_vert").value
        self.bbox_static_horiz = self.get_parameter("bbox_static_horiz").value
        self.percentage_vert = self.get_parameter("percentage_vert").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.publish_markers = self.get_parameter("publish_markers").value
        self.publish_tf = self.get_parameter("publish_tf").value
        seed = self.get_parameter("seed").value

        # Set seed
        random.seed(seed)

        # Calculate number of dynamic and static obstacles
        self.num_dyn_objects = int(self.dynamic_ratio * self.total_num_obs)
        self.num_stat_objects = self.total_num_obs - self.num_dyn_objects

        self.get_logger().info(
            f"Creating {self.num_dyn_objects} dynamic and {self.num_stat_objects} static obstacles"
        )

        # Initialize obstacle properties
        self.x_all = []
        self.y_all = []
        self.z_all = []
        self.offset_all = []
        self.slower = []
        self.types = []  # "dynamic", "static_vert", "static_horiz"
        self.bboxes = []

        # Create dynamic obstacles
        for i in range(self.num_dyn_objects):
            self.x_all.append(random.uniform(self.x_min, self.x_max))
            self.y_all.append(random.uniform(self.y_min, self.y_max))
            self.z_all.append(random.uniform(self.z_min, self.z_max))
            self.offset_all.append(random.uniform(-2 * math.pi, 2 * math.pi))
            self.slower.append(random.uniform(self.slower_min, self.slower_max))
            self.types.append("dynamic")
            self.bboxes.append(self.bbox_dynamic)

        # Create static obstacles
        for i in range(self.num_stat_objects):
            if i < self.percentage_vert * self.num_stat_objects:
                # Vertical obstacle
                bbox_i = self.bbox_static_vert
                self.z_all.append(bbox_i[2] / 2.0)
                self.types.append("static_vert")
            else:
                # Horizontal obstacle
                bbox_i = self.bbox_static_horiz
                self.z_all.append(random.uniform(0.0, 3.0))
                self.types.append("static_horiz")

            self.x_all.append(
                random.uniform(self.x_min - self.scale, self.x_max + self.scale)
            )
            self.y_all.append(
                random.uniform(self.y_min - self.scale, self.y_max + self.scale)
            )
            self.offset_all.append(random.uniform(-2 * math.pi, 2 * math.pi))
            self.slower.append(random.uniform(self.slower_min, self.slower_max))
            self.bboxes.append(bbox_i)

        # Create publishers
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_traj = self.create_publisher(DynTraj, "/trajs", self.total_num_obs)

        if self.publish_markers:
            self.pub_shapes_static = self.create_publisher(
                Marker, "/shapes_static", qos_latched
            )
            self.pub_shapes_dynamic = self.create_publisher(
                Marker, "/shapes_dynamic", qos_latched
            )

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

        self.get_logger().info(
            f"RViz Obstacles Node started with {self.total_num_obs} obstacles"
        )
        self.get_logger().info(f"Publishing at {publish_rate_hz} Hz")

    def trefoil(self, x, y, z, scale_x, scale_y, scale_z, offset, slower):
        """Trefoil knot trajectory."""
        t = self.get_clock().now().nanoseconds / 1e9
        tt = t / slower + offset

        x_pos = scale_x * (math.sin(tt) + 2 * math.sin(2 * tt)) + x
        y_pos = scale_y * (math.cos(tt) - 2 * math.cos(2 * tt)) + y
        z_pos = scale_z * (-math.sin(3 * tt)) + z

        # Velocity (derivatives)
        inv_slower = 1.0 / slower
        vx = scale_x * inv_slower * (math.cos(tt) + 4 * math.cos(2 * tt))
        vy = scale_y * inv_slower * (-math.sin(tt) + 4 * math.sin(2 * tt))
        vz = -scale_z * 3 * inv_slower * math.cos(3 * tt)

        return x_pos, y_pos, z_pos, vx, vy, vz

    def wave_in_z(self, x, y, z, scale, offset, slower):
        """Wave motion in Z direction (for static obstacles)."""
        t = self.get_clock().now().nanoseconds / 1e9
        tt = t / slower + offset

        x_pos = x
        y_pos = y
        z_pos = scale * (-math.sin(tt)) + z

        inv_slower = 1.0 / slower
        vx = 0.0
        vy = 0.0
        vz = -scale * inv_slower * math.cos(tt)

        return x_pos, y_pos, z_pos, vx, vy, vz

    def timer_callback(self):
        now = self.get_clock().now()

        # Markers for visualization
        if self.publish_markers:
            marker_static = Marker()
            marker_static.header.frame_id = "world"
            marker_static.header.stamp = now.to_msg()
            marker_static.type = Marker.CUBE_LIST
            marker_static.action = Marker.ADD
            marker_static.color = COLOR_STATIC
            marker_static.pose.orientation.w = 1.0

            marker_dynamic = Marker()
            marker_dynamic.header.frame_id = "world"
            marker_dynamic.header.stamp = now.to_msg()
            marker_dynamic.type = Marker.CUBE_LIST
            marker_dynamic.action = Marker.ADD
            marker_dynamic.color = COLOR_DYNAMIC
            marker_dynamic.pose.orientation.w = 1.0

        # Publish each obstacle
        for i in range(self.total_num_obs):
            bbox_i = self.bboxes[i]
            s = self.scale

            if self.types[i] == "dynamic":
                x, y, z, vx, vy, vz = self.trefoil(
                    self.x_all[i],
                    self.y_all[i],
                    self.z_all[i],
                    s,
                    s,
                    s,
                    self.offset_all[i],
                    self.slower[i],
                )
            else:
                x, y, z, vx, vy, vz = self.wave_in_z(
                    self.x_all[i],
                    self.y_all[i],
                    self.z_all[i],
                    s,
                    self.offset_all[i],
                    1.0,
                )

            # Publish DynTraj message
            traj_msg = DynTraj()
            traj_msg.header.stamp = now.to_msg()
            traj_msg.header.frame_id = "world"
            traj_msg.is_agent = False
            traj_msg.id = 4000 + i  # Start from 4000 to avoid conflicts with agents
            traj_msg.pos.x = x
            traj_msg.pos.y = y
            traj_msg.pos.z = z
            traj_msg.bbox = bbox_i

            # Velocity as string expressions (for analytical trajectory representation)
            # Format: velocity = [vx_expr, vy_expr, vz_expr]
            t_var = "(t)"  # Use t as the time variable
            if self.types[i] == "dynamic":
                # For dynamic obstacles, provide analytical velocity expressions
                tt_expr = f"({t_var}/{self.slower[i]}+{self.offset_all[i]})"
                vx_expr = (
                    f"{s / 6.0}/{self.slower[i]}*(cos({tt_expr})+4*cos(2*{tt_expr}))"
                )
                vy_expr = (
                    f"{s / 5.0}/{self.slower[i]}*(-sin({tt_expr})+4*sin(2*{tt_expr}))"
                )
                vz_expr = f"-{3 * s / 2.0}/{self.slower[i]}*cos(3*{tt_expr})"
            else:
                # For static obstacles with wave motion
                tt_expr = f"({t_var}/1.0+{self.offset_all[i]})"
                vx_expr = "0"
                vy_expr = "0"
                vz_expr = f"-{s}*cos({tt_expr})"

            traj_msg.velocity = [vx_expr, vy_expr, vz_expr]

            self.pub_traj.publish(traj_msg)

            # TF
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = "world"
                t.child_frame_id = f"obstacle_{traj_msg.id}"
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = z
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)

            # Add to marker
            if self.publish_markers:
                point = Point()
                point.x = x
                point.y = y
                point.z = z

                if self.types[i] == "dynamic":
                    marker_dynamic.points.append(point)
                    marker_dynamic.scale.x = bbox_i[0]
                    marker_dynamic.scale.y = bbox_i[1]
                    marker_dynamic.scale.z = bbox_i[2]
                else:
                    marker_static.points.append(point)
                    marker_static.scale.x = bbox_i[0]
                    marker_static.scale.y = bbox_i[1]
                    marker_static.scale.z = bbox_i[2]

        # Publish markers
        if self.publish_markers:
            self.pub_shapes_dynamic.publish(marker_dynamic)
            self.pub_shapes_static.publish(marker_static)


def main(args=None):
    rclpy.init(args=args)
    node = RVizObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
