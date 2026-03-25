#!/usr/bin/env python3
"""
RViz-Only Simulation Launch File

This launch file creates a lightweight simulation environment without Gazebo:
1. Launches simulator node (no Gazebo, just fake sensing)
2. Starts RViz for visualization
3. Spawns dynamic/static obstacles (RViz markers only, no physics)

Usage:
    ros2 launch sando rviz_only.launch.py num_obstacles:=50
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Declare arguments
    args = [
        DeclareLaunchArgument(
            "num_obstacles", default_value="50", description="Total number of obstacles"
        ),
        DeclareLaunchArgument(
            "dynamic_ratio",
            default_value="0.65",
            description="Ratio of dynamic obstacles (0.0-1.0)",
        ),
        DeclareLaunchArgument("x_min", default_value="5.0"),
        DeclareLaunchArgument("x_max", default_value="100.0"),
        DeclareLaunchArgument("y_min", default_value="-6.0"),
        DeclareLaunchArgument("y_max", default_value="6.0"),
        DeclareLaunchArgument("z_min", default_value="0.5"),
        DeclareLaunchArgument("z_max", default_value="4.5"),
        DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument(
            "obstacles_json_file",
            default_value="",
            description="Path to shared benchmark obstacle JSON config",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Publish TF for dynamic obstacles",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("sando"), "rviz", "sando.rviz"
            ),
            description="Path to RViz config file",
        ),
    ]

    # RViz node for visualization
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config"),
            "--ros-args",
            "--log-level",
            "error",
        ],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Include dyn_obstacles.launch.py (skip Gazebo, let rviz_only handle RViz)
    dyn_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sando"), "launch", "dyn_obstacles.launch.py"]
            )
        ),
        launch_arguments={
            "skip_gazebo": "true",
            "use_rviz": "false",  # RViz already launched above
            "num_obstacles": LaunchConfiguration("num_obstacles"),
            "dynamic_ratio": LaunchConfiguration("dynamic_ratio"),
            "x_min": LaunchConfiguration("x_min"),
            "x_max": LaunchConfiguration("x_max"),
            "y_min": LaunchConfiguration("y_min"),
            "y_max": LaunchConfiguration("y_max"),
            "z_min": LaunchConfiguration("z_min"),
            "z_max": LaunchConfiguration("z_max"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "seed": LaunchConfiguration("seed"),
            "publish_markers": "true",
            "publish_tf": LaunchConfiguration("publish_tf"),
            "launch_forest_node": "true",
            "obstacles_json_file": LaunchConfiguration("obstacles_json_file"),
        }.items(),
    )

    return LaunchDescription(
        args
        + [
            rviz,
            dyn_obstacles,
        ]
    )
