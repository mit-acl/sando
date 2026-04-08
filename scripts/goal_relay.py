#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalRelay(Node):
    """Relay RViz '2D Nav Goal' clicks to the SANDO planner's term_goal topic.

    RViz publishes on /goal_pose (z=0 for 2D goals). This node overrides z
    with a configurable flight altitude and republishes on /NX01/term_goal.
    """

    def __init__(self):
        super().__init__("goal_relay")
        self.default_goal_z = self.declare_parameter("default_goal_z", 2.0).value
        self.namespace = self.declare_parameter("agent_namespace", "NX01").value

        self.sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.pub = self.create_publisher(
            PoseStamped, f"/{self.namespace}/term_goal", 10
        )

        self.get_logger().info(
            f"Goal relay active: /goal_pose -> /{self.namespace}/term_goal "
            f"(z={self.default_goal_z})"
        )

    def goal_callback(self, msg: PoseStamped):
        msg.pose.position.z = self.default_goal_z
        self.pub.publish(msg)
        self.get_logger().info(
            f"Relayed goal: ({msg.pose.position.x:.1f}, "
            f"{msg.pose.position.y:.1f}, {self.default_goal_z})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
