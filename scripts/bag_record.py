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


def record_ros2_bag(
    bag_name, bag_path, agents, use_hardware=False, video=False, topics=None
):

    # Per-agent topics common to both sim and hardware (prefixed with /{agent})
    base_topics = [
        # --- Planner core ---
        "/state",
        "/goal",
        "/term_goal",
        "/traj",
        # --- Trajectory visualization ---
        "/traj_committed_colored",
        "/traj_subopt_colored",
        "/actual_traj",
        # --- Global planner paths ---
        "/hgp_path_marker",
        "/original_hgp_path_marker",
        # --- Safe corridors ---
        "/poly_safe",
        # --- Waypoints ---
        "/point_G",
        "/point_A",
        "/point_E",
        "/point_G_term",
        # --- Hover avoidance ---
        "/hover_avoidance_viz",
        # --- Mapping (from global_mapper) ---
        "/occupancy_grid",
        "/unknown_grid",
        "/dynamic_grid",
        "/heat_cloud",
        # --- Obstacle tracking ---
        "/tracked_obstacles",
        "/cluster_bounding_boxes",
        "/predicted_trajs",
        # --- Sensors ---
        "/d435/color/image_raw",
        # --- HUD ---
        "/vel_text",
        "/drone_marker",
        # --- Computation times ---
        "/computation_times",
    ]

    # Hardware-only per-agent topics
    hw_topics = [
        "/livox/lidar",
        "/mavros/setpoint_trajectory/local",
        "/mavros/local_position/pose",
        "/mavros/vision_pose/pose_cov",
    ]

    # Simulation-only per-agent topics
    sim_topics = [
        "/mid360_PointCloud2",
    ]

    # Global topics (not agent-specific)
    static_topics = [
        "/tf",
        "/tf_static",
        "/trajs",
        "/clock",
    ]

    # Simulation-only global topics
    sim_static_topics = [
        "/clicked_point",
        "/parameter_events",
        "/rosout",
        "/map_generator/global_cloud",
    ]

    # Lightweight video-friendly topic set (visualization only, no data topics)
    video_topics = [
        "/traj_committed_colored",
        "/traj_subopt_colored",
        "/actual_traj",
        "/hgp_path_marker",
        "/original_hgp_path_marker",
        "/poly_safe",
        "/point_G",
        "/point_A",
        "/point_E",
        "/term_goal",
        "/hover_avoidance_viz",
        "/occupancy_grid",
        "/unknown_grid",
        "/dynamic_grid",
        "/heat_cloud",
        "/tracked_obstacles",
        "/cluster_bounding_boxes",
        "/uncertainty_spheres",
        "/vel_text",
        "/fov",
        "/drone_marker",
    ]

    video_static_topics = [
        "/tf",
        "/tf_static",
        "/shapes_dynamic_mesh",
    ]

    # Build per-agent topic list
    if video:
        agent_topics = video_topics
    else:
        agent_topics = base_topics + (hw_topics if use_hardware else sim_topics)

    # Generate topics for all agents
    all_topics = []
    for agent in agents:
        for topic in agent_topics:
            all_topics.append(f"/{agent}{topic}")

    # Add global topics
    if video:
        all_topics.extend(video_static_topics)
    else:
        all_topics.extend(static_topics)
        if not use_hardware:
            all_topics.extend(sim_static_topics)

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
    parser = argparse.ArgumentParser(description="Record a ROS2 bag")
    parser.add_argument(
        "--bag_number",
        type=int,
        default=None,
        help="Bag number to record (legacy, produces num_N name)",
    )
    parser.add_argument(
        "--bag_name", type=str, default=None, help="Bag name (e.g. 20260222_170122)"
    )
    parser.add_argument(
        "--bag_path",
        type=str,
        help="Path to save the bag",
        required=True,
    )
    parser.add_argument(
        "--agents",
        nargs="+",
        help="List of agents to record",
        # default=['NX01', 'NX02', 'NX03', 'NX04', 'NX05', 'NX06', 'NX07', 'NX08', 'NX09', 'NX10'],
        default=["NX01"],
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Use hardware topic set (livox/lidar instead of mid360_PointCloud2, etc.)",
    )
    parser.add_argument(
        "--video",
        action="store_true",
        help="Record only visualization topics for video replay (lightweight, no data topics)",
    )
    args = parser.parse_args()

    if args.bag_name:
        bag_name = args.bag_name
    elif args.bag_number is not None:
        bag_name = "num_" + str(args.bag_number)
    else:
        from datetime import datetime

        bag_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = args.bag_path

    # NO string conversion needed – args.agents is already a list of strings
    agents = args.agents

    print("Bag name:", bag_name)
    print("Bag path:", bag_path)
    print("Agents:", agents)
    print("Hardware:", args.hardware)
    print("Video:", args.video)

    record_ros2_bag(
        bag_name, bag_path, agents, use_hardware=args.hardware, video=args.video
    )
