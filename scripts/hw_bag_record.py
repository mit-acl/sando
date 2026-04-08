# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
import os
import subprocess
import argparse


def record_ros2_bag(bag_name, bag_path, agents, topics=None):

    # Define the topics template that is common across all agents
    base_topics = [
        "/agent_initial_guess_pos",
        "/agent_pos",
        "/agent_pos_array",
        "/camera_info",
        "/depth_camera/free_cells_vis_array",
        "/depth_camera/occupied_cells_vis_array",
        "/drone_marker",
        "/drone_marker_array",
        "/dummy_traj_pos",
        "/goal",
        "/image_raw",
        "/joint_states",
        "/hgp_path_marker",
        "/free_hgp_path_marker",
        "/lidar/free_cells_vis_array",
        "/lidar/occupied_cells_vis_array",
        "/mode",
        "/own_traj",
        "/point_G",
        "/point_G_term",
        "/poly",
        "/projected_map",
        "/robot_description",
        "/state",
        "/term_goal",
        "/traj",
        "/traj_committed_colored",
        "/mid360_PointCloud2",
        "/d435/color/image_raw",
        "/d435/color/image_raw/compressed",
        "/d435/depth/color/points",
        "/d435/depth/image_raw",
        # "/d435/depth/image_raw/compressed",
        # "/d435/depth/image_raw/compressedDepth",
        "/d435/color/camera_info",
        "/d435/depth/camera_info",
        "/dynamic_map_marker",
        "/actual_traj",
        "/tracked_obstacles",
        "/cluster_bounding_boxes",
        "/yaw_output",
        "/predicted_trajs",
    ]

    # Static topics (not agent-specific)
    static_topics = [
        "/clicked_point",
        "/clock",
        "/dummy_traj",
        "/initialpose",
        "/parameter_events",
        # "/performance_metrics",
        # "/plug/link_states_plug",
        # "/plug/model_states_plug",
        "/trajs",
        "/rosout",
        "/tf",
        "/tf_static",
    ]

    # Generate topics for all agents
    all_topics = []
    for agent in agents:
        for topic in base_topics:
            all_topics.append(f"/{agent}{topic}")

    # Add static topics (non-agent-specific topics)
    all_topics.extend(static_topics)

    # Use provided topics if specified, otherwise default to generated topics
    if topics is None:
        topics = all_topics

    # Build the ros2 bag record command
    command = ["ros2", "bag", "record", "-o", os.path.join(bag_path, bag_name)] + topics

    # Execute the command
    try:
        subprocess.run(command, check=True)
        print(f"Recording started for bag: {bag_name} at path: {bag_path}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to start recording: {e}")


# Main function
if __name__ == "__main__":
    # Create arguments for record_ros2_bag
    parser = argparse.ArgumentParser(description="Record a ROS2 bag")
    parser.add_argument("--bag_number", type=int, help="Bag number to record")
    parser.add_argument(
        "--bag_path",
        type=str,
        help="Path to save the bag",
        default=os.path.expanduser("~/data/sando"),
    )
    # parser.add_argument("--agents", nargs="+", help="List of agents to record", default=["NX01", "NX02", "NX03"])
    parser.add_argument(
        "--agents", nargs="+", help="List of agents to record", default=["BD01"]
    )
    args = parser.parse_args()

    # Customize bag name and path as needed
    # bag_name = "num_" + str(args.bag_number)
    bag_name = "hw_" + str(args.bag_number)
    bag_path = args.bag_path

    # Convert string list (eg. "['NX01', 'NX02']") to list (eg. ['NX01', 'NX02'])
    if args.agents:
        args.agents = (
            args.agents[0]
            .replace("[", "")
            .replace("]", "")
            .replace("'", "")
            .split(", ")
        )

    print("Bag name:", bag_name)
    print("Bag path:", bag_path)
    print("Agents:", args.agents)

    # List of agents
    agents = args.agents

    record_ros2_bag(bag_name, bag_path, agents)
