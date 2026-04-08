#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # namespace
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="quadrotor", description="Namespace of the nodes"
    )

    def launch_setup(context, *args, **kwargs):

        namespace = LaunchConfiguration("namespace").perform(context)

        node = Node(
            package="sando",
            executable="convert_odom_to_state",
            name="convert_odom_to_state",
            namespace=namespace,
            remappings=[
                ("odom", "odom/sample"),  # Remap incoming Odometry topic
                ("state", "state"),  # Remap outgoing State topic
            ],
            output="screen",
        )

        return [node]

    return LaunchDescription([namespace_arg, OpaqueFunction(function=launch_setup)])
