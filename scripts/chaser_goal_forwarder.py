#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

"""Forward one agent's position as another agent's goal.

Used in adversarial hover-avoidance testing: a "chaser" SANDO agent
continuously receives the "evader" agent's current position as its
navigation goal.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from dynus_interfaces.msg import State


class ChaserGoalForwarder(Node):
    def __init__(self):
        super().__init__("chaser_goal_forwarder")

        # Parameters
        self.evader_ns = self.declare_parameter("evader_ns", "NX01").value
        self.chaser_ns = self.declare_parameter("chaser_ns", "NX02").value
        self.rate_hz = self.declare_parameter("rate_hz", 2.0).value

        # State from evader
        self._evader_state = None
        self._state_received = False
        self.create_subscription(
            State,
            f"/{self.evader_ns}/state",
            self._state_cb,
            10,
        )

        # Goal publisher for chaser
        self._pub = self.create_publisher(
            PoseStamped,
            f"/{self.chaser_ns}/term_goal",
            10,
        )

        # Timer to publish at fixed rate
        self._count = 0
        self.create_timer(1.0 / self.rate_hz, self._publish_goal)

        self.get_logger().info(
            f"[chaser_goal_forwarder] Subscribing to /{self.evader_ns}/state"
        )
        self.get_logger().info(
            f"[chaser_goal_forwarder] Publishing to /{self.chaser_ns}/term_goal "
            f"at {self.rate_hz} Hz"
        )

    def _state_cb(self, msg: State):
        if not self._state_received:
            self.get_logger().info(
                f"[chaser_goal_forwarder] First state received from /{self.evader_ns}: "
                f"({msg.pos.x:.2f}, {msg.pos.y:.2f}, {msg.pos.z:.2f})"
            )
            self._state_received = True
        self._evader_state = msg

    def _publish_goal(self):
        if self._evader_state is None:
            self._count += 1
            if self._count % 10 == 0:
                self.get_logger().warn(
                    f"[chaser_goal_forwarder] No state from /{self.evader_ns} yet "
                    f"(waited {self._count / self.rate_hz:.0f}s)"
                )
            return
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self._evader_state.pos.x
        goal.pose.position.y = self._evader_state.pos.y
        goal.pose.position.z = self._evader_state.pos.z
        goal.pose.orientation.w = 1.0
        self._pub.publish(goal)
        self.get_logger().info(
            f"[chaser_goal_forwarder] Published goal to /{self.chaser_ns}: "
            f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}, {goal.pose.position.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ChaserGoalForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
