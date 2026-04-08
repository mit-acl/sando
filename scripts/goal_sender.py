#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright (c) Anonymous Author
#  * Anonymous Institution
#  * All Rights Reserved
#  * Authors: Anonymous
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from time import sleep


class GoalSender(Node):
    def __init__(self):

        super().__init__("goal_sender")

        # Parameters
        ## Get the list of agents and corresponding goals
        self.list_agents = self.declare_parameter(
            "list_agents", value=["NX01", "NX02"]
        ).value
        self.list_goals = self.declare_parameter(
            "list_goals", value=["[2.0, 2.0]", "[-2.0, 2.0]"]
        ).value

        # Since the list_goals parameter is a list of strings, we need to parse it as a list of lists
        self.list_goals = [yaml.safe_load(goal) for goal in self.list_goals]

        ## Get the z value for the goals
        self.default_goal_z = self.declare_parameter("default_goal_z", value=3.0).value

        # Publisher
        self.pub_goals = {}
        for agent in self.list_agents:
            self.pub_goals[agent] = self.create_publisher(
                PoseStamped, f"/{agent}/term_goal", 10
            )

        # Send the goals (publish a few times to ensure delivery, then stop)
        sleep(1)  # wait for subscribers to connect
        for _ in range(3):
            for agent, goal in zip(self.list_agents, self.list_goals):
                self.send_goal(agent, goal)
            sleep(0.3)

        for agent, goal in zip(self.list_agents, self.list_goals):
            print(f"Goal sent for {agent}: {goal}")

    def send_goal(self, agent, goal):

        # Create a goal message
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal[0])
        msg.pose.position.y = float(goal[1])
        msg.pose.position.z = self.default_goal_z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        # Publish the goal
        self.pub_goals[agent].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
