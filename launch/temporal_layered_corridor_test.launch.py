#!/usr/bin/env python3
# /* ----------------------------------------------------------------------------
#  * Copyright (c) Anonymous Author
#  * Anonymous Institution
#  * All Rights Reserved
#  * Authors: Anonymous
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sando",
                executable="temporal_layered_corridor_test_node",
                name="temporal_layered_corridor_test_node",
                output="screen",
                parameters=[
                    {"obst_max_vel": 0.2},
                ],
                # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
            ),
        ]
    )
