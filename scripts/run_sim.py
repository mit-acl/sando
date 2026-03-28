#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
SANDO Simulation Launcher

This script provides a unified interface to launch SANDO simulations in three modes:
1. Multi-agent simulation with fake sensing (multiagent)
2. Single-agent simulation with Gazebo and ACL mapper (gazebo)
3. Single-agent RViz-only simulation with dynamic obstacles (rviz-only) - LIGHTWEIGHT!

Usage:
    # Multi-agent fake simulation (10 agents in a circle)
    python3 scripts/run_sim.py --mode multiagent --setup-bash install/setup.bash

    # Single-agent Gazebo simulation with default goal
    python3 scripts/run_sim.py --mode gazebo --setup-bash install/setup.bash

    # Single-agent Gazebo with custom goal
    python3 scripts/run_sim.py --mode gazebo --setup-bash install/setup.bash --goal 100 50 3

    # Custom number of agents for multiagent mode
    python3 scripts/run_sim.py --mode multiagent --setup-bash install/setup.bash --num-agents 5

    # Custom environment for Gazebo mode
    python3 scripts/run_sim.py --mode gazebo --setup-bash install/setup.bash --env easy_forest

    # RViz-only mode (lightweight, no Gazebo) with 50 obstacles
    python3 scripts/run_sim.py --mode rviz-only --setup-bash install/setup.bash

    # RViz-only with custom obstacles
    python3 scripts/run_sim.py --mode rviz-only --setup-bash install/setup.bash --num-obstacles 100 --dynamic-ratio 0.7
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile
import time
import yaml
from pathlib import Path


# Source rviz config (not the install copy)
RVIZ_CONFIG = Path(__file__).resolve().parent.parent / "rviz" / "sando.rviz"


def find_setup_bash(args_setup_bash: str = None) -> Path:
    """Find setup.bash path. Requires explicit --setup-bash argument."""
    if not args_setup_bash:
        print(
            "[ERROR] --setup-bash is required. Please specify the path to setup.bash",
            file=sys.stderr,
        )
        print(
            "  Example: python3 run_sim.py --mode gazebo --setup-bash install/setup.bash",
            file=sys.stderr,
        )
        sys.exit(1)

    path = Path(args_setup_bash)
    if path.exists():
        return path

    print(f"[ERROR] Specified setup.bash not found: {args_setup_bash}", file=sys.stderr)
    sys.exit(1)


def workspace_source_commands(ros_domain_id: int) -> list:
    """Generate shell commands to cleanly source the SANDO workspace in tmux panes.

    Clears stale ROS/colcon environment from .bashrc before sourcing the target
    workspace, preventing library conflicts when multiple workspaces are installed.
    """
    return [
        # Reset ROS overlay so only the target workspace is active
        'unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH 2>/dev/null',
        'source /opt/ros/humble/setup.bash',
        '. "$SETUP_BASH"',
        f"export ROS_DOMAIN_ID={ros_domain_id}",
    ]


def generate_multiagent_positions(
    num_agents: int, radius: float = 10.0, z: float = 1.0
):
    """Generate agent positions in a circle formation."""
    agents = []
    for i in range(num_agents):
        angle = 2 * math.pi * i / num_agents
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        # Yaw points toward center (opposite of position angle)
        yaw_deg = math.degrees(angle + math.pi)
        # Normalize to [-180, 180]
        if yaw_deg > 180:
            yaw_deg -= 360
        agents.append(
            {
                "namespace": f"NX{i + 1:02d}",
                "x": round(x, 3),
                "y": round(y, 3),
                "z": z,
                "yaw": round(yaw_deg, 1),
            }
        )
    return agents


def generate_multiagent_yaml(
    setup_bash: Path, agents: list, ros_domain_id: int = 20
) -> str:
    """Generate YAML for multi-agent fake simulation."""
    panes = []

    # Base station (simulator)
    panes.append({"shell_command": ["ros2 launch sando simulator.launch.py"]})

    # Agent panes
    for agent in agents:
        panes.append(
            {
                "shell_command": [
                    "sleep 10",
                    f"ros2 launch sando onboard_sando.launch.py namespace:={agent['namespace']} "
                    f"x:={agent['x']} y:={agent['y']} z:={agent['z']} yaw:={agent['yaw']} "
                    f"publish_odom:=true odom_topic:=odom",
                ]
            }
        )

    # Goal monitor
    panes.append(
        {"shell_command": ["sleep 20", "ros2 launch sando goal_monitor.launch.py"]}
    )

    yaml_content = {
        "session_name": "sando_sim",
        "windows": [
            {
                "window_name": "main",
                "layout": "tiled",
                "shell_command_before": workspace_source_commands(ros_domain_id),
                "panes": panes,
            }
        ],
    }

    return yaml.dump(yaml_content, default_flow_style=False, sort_keys=False)


def generate_rviz_only_yaml(
    setup_bash: Path,
    goal: tuple,
    start_pos: tuple = (0, 0, 3.0),
    start_yaw: float = 0.0,
    ros_domain_id: int = 20,
    num_obstacles: int = 50,
    dynamic_ratio: float = 0.65,
    x_min: float = 5.0,
    x_max: float = 100.0,
    y_min: float = -6.0,
    y_max: float = 6.0,
    z_min: float = 0.5,
    z_max: float = 4.5,
    seed: int = 0,
    data_file: str = None,
    use_benchmark: bool = False,
    global_planner: str = "astar_heat",
    send_goal: bool = True,
    use_rviz: bool = True,
    environment_assumption: str = "",
    publish_obstacle_tf: bool = True,
    with_goal_relay: bool = False,
    obstacles_json_file: str = None,
    skip_initial_yawing: bool = False,
) -> str:
    """Generate YAML for RViz-only simulation (no Gazebo, lightweight)."""
    goal_x, goal_y, goal_z = goal
    start_x, start_y, start_z = start_pos

    panes = [
        # RViz + obstacles visualization
        {
            "shell_command": [
                f"ros2 launch sando rviz_only.launch.py "
                f"num_obstacles:={num_obstacles} "
                f"dynamic_ratio:={dynamic_ratio} "
                f"x_min:={x_min} x_max:={x_max} "
                f"y_min:={y_min} y_max:={y_max} "
                f"z_min:={z_min} z_max:={z_max} "
                f"publish_rate_hz:=100.0 "
                f"seed:={seed} "
                f"use_rviz:={str(use_rviz).lower()} "
                f"publish_tf:={str(publish_obstacle_tf).lower()} "
                f"rviz_config:={RVIZ_CONFIG}"
                + (
                    f" obstacles_json_file:={obstacles_json_file}"
                    if obstacles_json_file
                    else ""
                )
            ]
        },
        # Onboard agent NX01 (with rviz_only mode - no point cloud)
        {
            "shell_command": [
                "sleep 3",
                f"ros2 launch sando onboard_sando.launch.py namespace:=NX01 "
                f"x:={start_x} y:={start_y} z:={start_z} yaw:={start_yaw} "
                f"sim_env:=rviz_only "
                f"publish_odom:=true "
                f"odom_topic:=odom "
                + ("skip_initial_yawing:=true " if skip_initial_yawing else "")
                + (
                    f"environment_assumption:={environment_assumption} "
                    if environment_assumption
                    else ""
                )
                + (
                    f"use_benchmark:=true data_file:={data_file} global_planner:={global_planner} "
                    if use_benchmark and data_file
                    else ""
                ),
            ]
        },
    ]

    # Add goal sender pane only if send_goal is True
    if send_goal:
        panes.append(
            {
                "shell_command": [
                    "sleep 8",
                    f"ros2 launch sando goal_sender.launch.py list_agents:=\"['NX01']\" list_goals:=\"['[{goal_x}, {goal_y}, {goal_z}]']\"",
                ]
            }
        )

    # Add goal relay pane for interactive mode (RViz click-to-goal)
    if with_goal_relay:
        panes.append(
            {
                "shell_command": [
                    "sleep 8",
                    f"ros2 run sando goal_relay.py --ros-args -p default_goal_z:={goal_z}",
                ]
            }
        )

    yaml_content = {
        "session_name": "sando_sim",
        "windows": [
            {
                "window_name": "main",
                "layout": "tiled",
                "shell_command_before": workspace_source_commands(ros_domain_id),
                "panes": panes,
            }
        ],
    }

    return yaml.dump(yaml_content, default_flow_style=False, sort_keys=False)


def _trefoil_expr(x0, y0, z0, sx, sy, sz, offset, slower):
    """Trefoil knot position + velocity expression strings (matches dyn_obstacles.launch.py)."""
    tt = f"t/{slower}+{offset}"
    x_str = f"{sx / 6.0}*(sin({tt})+2*sin(2*{tt}))+{x0}"
    y_str = f"{sy / 5.0}*(cos({tt})-2*cos(2*{tt}))+{y0}"
    z_str = f"{sz / 2.0}*(-sin(3*{tt}))+{z0}"
    inv_s = f"(1/{slower})"
    vx_str = f"{sx / 6.0}*{inv_s}*(cos({tt})+4*cos(2*{tt}))"
    vy_str = f"{sy / 5.0}*{inv_s}*(-sin({tt})+4*sin(2*{tt}))"
    vz_str = f"-{3 * sz / 2.0}*{inv_s}*cos(3*{tt})"
    return x_str, y_str, z_str, vx_str, vy_str, vz_str


def _generate_obstacle_json(
    num_obstacles: int,
    seed: int,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    dynamic_ratio: float = 1.0,
    exclusion_zone: tuple = None,
) -> list:
    """Generate obstacle metadata list (same logic as dyn_obstacles.launch.py)."""
    import random as _rng

    _rng.seed(seed)

    scale_range = [[2.0, 4.0], [2.0, 4.0], [2.0, 4.0]]
    offset_range = [0.0, 3.0]
    slower_range = [4.0, 6.0]
    bbox_dynamic = [0.8, 0.8, 0.8]
    bbox_static_vert = [0.4, 0.4, 4.0]
    bbox_static_horiz = [0.4, 4.0, 0.4]
    percentage_vert = 0.35

    num_dynamic = int(num_obstacles * dynamic_ratio)
    num_static = num_obstacles - num_dynamic

    obstacles = []
    for i in range(num_obstacles):
        is_dynamic = i < num_dynamic

        # Sample position, rejecting points inside the exclusion zone
        while True:
            x = x_min + (x_max - x_min) * _rng.random()
            y = y_min + (y_max - y_min) * _rng.random()
            z = z_min + (z_max - z_min) * _rng.random()
            if exclusion_zone is None:
                break
            ex_min, ex_max, ey_min, ey_max = exclusion_zone
            if not (ex_min <= x <= ex_max and ey_min <= y <= ey_max):
                break

        if is_dynamic:
            sx = _rng.uniform(*scale_range[0])
            sy = _rng.uniform(*scale_range[1])
            sz = _rng.uniform(*scale_range[2])
            offset = _rng.uniform(*offset_range)
            slower = _rng.uniform(*slower_range)

            x_str, y_str, z_str, vx_str, vy_str, vz_str = _trefoil_expr(
                x, y, z, sx, sy, sz, offset, slower
            )
            bbox = bbox_dynamic
        else:
            static_idx = i - num_dynamic
            is_vertical = static_idx < (num_static * percentage_vert)

            if is_vertical:
                z = bbox_static_vert[2] / 2.0
                bbox = bbox_static_vert
            else:
                num_horiz = num_static - int(num_static * percentage_vert)
                horiz_idx = static_idx - int(num_static * percentage_vert)
                if horiz_idx < num_horiz / 2:
                    bbox = bbox_static_horiz  # [0.4, 4.0, 0.4] long on Y
                else:
                    bbox = [4.0, 0.4, 0.4]  # long on X

            x_str = str(x)
            y_str = str(y)
            z_str = str(z)
            vx_str = "0.0"
            vy_str = "0.0"
            vz_str = "0.0"
            sx = sy = sz = 0.0
            offset = slower = 0.0

        obstacles.append(
            {
                "name": f"obstacle_{i}",
                "x0": x,
                "y0": y,
                "z0": z,
                "scale_x": sx,
                "scale_y": sy,
                "scale_z": sz,
                "offset": offset,
                "slower": slower,
                "traj_x": x_str,
                "traj_y": y_str,
                "traj_z": z_str,
                "traj_vx": vx_str,
                "traj_vy": vy_str,
                "traj_vz": vz_str,
                "size_x": bbox[0],
                "size_y": bbox[1],
                "size_z": bbox[2],
            }
        )
    return obstacles


def _inject_world_plugin(base_world_path: str, json_path: str, output_path: str):
    """Read a .world file and inject the dynamic obstacles world plugin."""
    with open(base_world_path) as f:
        content = f.read()

    plugin_xml = (
        f'\n    <plugin name="dyn_obs" filename="libdynamic_obstacles_world_plugin.so">'
        f"\n      <json_path>{json_path}</json_path>"
        f"\n    </plugin>\n  "
    )
    content = content.replace("</world>", plugin_xml + "</world>")

    with open(output_path, "w") as f:
        f.write(content)


def generate_gazebo_dynamic_yaml(
    setup_bash: Path,
    goal: tuple,
    env: str = "easy_forest",
    start_pos: tuple = (0, 0, 3.0),
    start_yaw: float = 0.0,
    ros_domain_id: int = 20,
    use_rviz: bool = True,
    use_gazebo_gui: bool = False,
    num_dyn_obstacles: int = 10,
    dynamic_ratio: float = 0.65,
    seed: int = 0,
    x_min: float = 5.0,
    x_max: float = 100.0,
    y_min: float = -6.0,
    y_max: float = 6.0,
    z_min: float = 0.5,
    z_max: float = 4.5,
    send_goal: bool = True,
    publish_trajs: bool = True,
    trajs_topic: str = "/trajs",
    depth_topic: str = "d435/depth/color/points",
    data_file: str = None,
    use_benchmark: bool = False,
    global_planner: str = "astar_heat",
) -> str:
    """Generate YAML for Gazebo + dynamic obstacles (unknown environment).

    Static obstacles come from the Gazebo world file (pointcloud via ACL mapper).

    When ground truth is disabled (publish_trajs=False):
      - A WorldPlugin spawns + moves ALL obstacles in Gazebo from a single plugin
        (no per-model plugins, no RSP nodes, no spawn_entity overhead).
      - dynamic_forest_node still runs for RViz markers and TF.

    When ground truth is enabled (publish_trajs=True):
      - Obstacles are RViz markers only (skip Gazebo), dynamic_forest_node
        publishes /trajs for the planner.
    """
    import json as _json

    goal_x, goal_y, goal_z = goal
    start_x, start_y, start_z = start_pos

    # --- Generate obstacle JSON ---
    obstacles = _generate_obstacle_json(
        num_dyn_obstacles,
        seed,
        x_min,
        x_max,
        y_min,
        y_max,
        z_min,
        z_max,
        dynamic_ratio=dynamic_ratio,
    )
    obstacles_json_str = _json.dumps(obstacles)

    if not publish_trajs:
        # Write JSON for the WorldPlugin to read
        json_path = "/tmp/sando_obstacles.json"
        with open(json_path, "w") as f:
            f.write(obstacles_json_str)

        # Generate temp world file with the WorldPlugin injected
        # Resolve base world path from the install directory
        env_to_world = {
            "easy_forest": "easy_forest.world",
            "medium_forest": "medium_forest.world",
            "hard_forest": "hard_forest.world",
            "empty": "empty.world",
            "empty_wo_ground": "empty_wo_ground.world",
        }
        world_file = env_to_world.get(env, f"{env}.world")
        install_dir = setup_bash.resolve().parent
        base_world = install_dir / "sando" / "share" / "sando" / "worlds" / world_file
        temp_world = "/tmp/sando_world.world"
        _inject_world_plugin(str(base_world), json_path, temp_world)

        panes = [
            # Gazebo with static world + WorldPlugin (spawns + moves dynamic obstacles)
            {
                "shell_command": [
                    f"ros2 launch sando base_sando.launch.py use_dyn_obs:=false "
                    f"use_gazebo_gui:={str(use_gazebo_gui).lower()} "
                    f"use_rviz:={str(use_rviz).lower()} env:={env} "
                    f"world_file:={temp_world} "
                    f"rviz_config:={RVIZ_CONFIG}"
                ]
            },
            # dynamic_forest_node for RViz markers, TF, and ground truth on a separate topic
            # use_sim_time:=true so node.now() returns Gazebo /clock (same as WorldPlugin's SimTime)
            # publish_trajs:=true but on trajs_topic (e.g. /trajs_ground_truth) so planner doesn't see it
            {
                "shell_command": [
                    "sleep 3",
                    f"ros2 launch sando dyn_obstacles.launch.py "
                    f"skip_gazebo:=true "
                    f"obstacles_json_file:={json_path} "
                    f"num_obstacles:={num_dyn_obstacles} "
                    f"dynamic_ratio:={dynamic_ratio} "
                    f"publish_rate_hz:=100.0 "
                    f"seed:={seed} "
                    f"use_sim_time:=true "
                    f"publish_markers:=true "
                    f"publish_tf:=true "
                    f"publish_trajs:=true "
                    f"trajs_topic:={trajs_topic} "
                    f"launch_forest_node:=true",
                ]
            },
        ]
    else:
        # Ground truth mode: RViz markers only (no Gazebo obstacles)
        panes = [
            {
                "shell_command": [
                    f"ros2 launch sando base_sando.launch.py use_dyn_obs:=false "
                    f"use_gazebo_gui:={str(use_gazebo_gui).lower()} "
                    f"use_rviz:={str(use_rviz).lower()} env:={env} "
                    f"rviz_config:={RVIZ_CONFIG}"
                ]
            },
            {
                "shell_command": [
                    "sleep 3",
                    f"ros2 launch sando dyn_obstacles.launch.py "
                    f"skip_gazebo:=true "
                    f"num_obstacles:={num_dyn_obstacles} "
                    f"dynamic_ratio:={dynamic_ratio} "
                    f"x_min:={x_min} x_max:={x_max} "
                    f"y_min:={y_min} y_max:={y_max} "
                    f"z_min:={z_min} z_max:={z_max} "
                    f"publish_rate_hz:=100.0 "
                    f"seed:={seed} "
                    f"publish_markers:=true "
                    f"publish_tf:=true "
                    f"publish_trajs:=true "
                    f"trajs_topic:={trajs_topic} "
                    f"launch_forest_node:=true",
                ]
            },
        ]

    # Common panes for both modes
    panes.extend(
        [
            # ACL mapper
            {
                "shell_command": [
                    "sleep 10",
                    f"ros2 launch global_mapper_ros global_mapper_node.launch.py quad:=NX01 depth_pointcloud_topic:={depth_topic}",
                ]
            },
            # Onboard agent NX01
            {
                "shell_command": [
                    "sleep 10",
                    f"ros2 launch sando onboard_sando.launch.py namespace:=NX01 "
                    f"x:={start_x} y:={start_y} z:={start_z} yaw:={start_yaw} "
                    + ("use_benchmark:=true " if use_benchmark else "")
                    + (f"data_file:={data_file} " if data_file else "")
                    + (f"global_planner:={global_planner} " if use_benchmark else ""),
                ]
            },
        ]
    )

    if send_goal:
        panes.append(
            {
                "shell_command": [
                    "sleep 20",
                    f"ros2 launch sando goal_sender.launch.py list_agents:=\"['NX01']\" list_goals:=\"['[{goal_x}, {goal_y}, {goal_z}]']\"",
                ]
            }
        )

    yaml_content = {
        "session_name": "sando_sim",
        "windows": [
            {
                "window_name": "main",
                "layout": "tiled",
                "shell_command_before": workspace_source_commands(ros_domain_id),
                "panes": panes,
            }
        ],
    }

    return yaml.dump(yaml_content, default_flow_style=False, sort_keys=False)


def generate_hover_test_yaml(
    setup_bash: Path,
    start_pos: tuple = (0, 0, 2.0),
    ros_domain_id: int = 20,
    use_rviz: bool = True,
    obs_offset: float = 5.0,
    obs_scale: float = 4.0,
    obs_slower: float = 2.0,
) -> str:
    """Generate YAML for hover avoidance test (empty world + 1 trefoil obstacle).

    The goal equals the start position so the drone immediately reaches
    GOAL_REACHED.  A single trefoil-knot obstacle orbits near the drone,
    triggering the HOVER_AVOIDING state when it passes close.
    """
    import json as _json

    start_x, start_y, start_z = start_pos
    goal_x, goal_y, goal_z = start_x, start_y, start_z  # goal == start

    # Trefoil obstacles orbiting near the drone from 3 directions, phase-shifted
    import math

    sx, sy, sz = obs_scale, obs_scale, obs_scale / 2.0
    num_obstacles = 3
    obstacles = []
    for i in range(num_obstacles):
        angle = 2.0 * math.pi * i / num_obstacles  # 0°, 120°, 240°
        phase = 2.0 * math.pi * i / num_obstacles  # stagger arrivals
        ox = start_x + obs_offset * math.cos(angle)
        oy = start_y + obs_offset * math.sin(angle)
        oz = start_z
        x_s, y_s, z_s, vx_s, vy_s, vz_s = _trefoil_expr(
            ox, oy, oz, sx, sy, sz, phase, obs_slower
        )
        obstacles.append(
            {
                "name": f"hover_test_obs_{i + 1}",
                "x0": ox,
                "y0": oy,
                "z0": oz,
                "scale_x": sx,
                "scale_y": sy,
                "scale_z": sz,
                "offset": phase,
                "slower": obs_slower,
                "traj_x": x_s,
                "traj_y": y_s,
                "traj_z": z_s,
                "traj_vx": vx_s,
                "traj_vy": vy_s,
                "traj_vz": vz_s,
                "size_x": 0.8,
                "size_y": 0.8,
                "size_z": 0.8,
            }
        )

    json_path = "/tmp/sando_hover_test_obstacles.json"
    with open(json_path, "w") as f:
        _json.dump(obstacles, f)

    panes = []

    # Pane 1: RViz (standalone, since rviz_only.launch.py doesn't forward obstacles_json_file)
    if use_rviz:
        panes.append({"shell_command": [f"rviz2 -d {RVIZ_CONFIG}"]})

    # Pane 2: Obstacle publisher (dynamic_forest_node with pre-generated JSON)
    panes.append(
        {
            "shell_command": [
                "sleep 2",
                f"ros2 launch sando dyn_obstacles.launch.py "
                f"skip_gazebo:=true "
                f"obstacles_json_file:={json_path} "
                f"publish_rate_hz:=100.0 "
                f"publish_trajs:=true "
                f"publish_markers:=true "
                f"publish_tf:=true "
                f"launch_forest_node:=true",
            ]
        }
    )

    # Pane 3: SANDO agent (rviz_only mode)
    panes.append(
        {
            "shell_command": [
                "sleep 3",
                f"ros2 launch sando onboard_sando.launch.py namespace:=NX01 "
                f"x:={start_x} y:={start_y} z:={start_z} "
                f"sim_env:=rviz_only "
                f"publish_odom:=true "
                f"odom_topic:=odom",
            ]
        }
    )

    # Pane 4: One-shot goal sender (publishes once then exits)
    goal_yaml = (
        f"'{{header: {{frame_id: map}}, "
        f"pose: {{position: {{x: {goal_x}, y: {goal_y}, z: {goal_z}}}, "
        f"orientation: {{w: 1.0}}}}}}'"
    )
    panes.append(
        {
            "shell_command": [
                "sleep 8",
                f"ros2 topic pub --once /NX01/term_goal geometry_msgs/msg/PoseStamped {goal_yaml} && "
                f"echo '[hover-test] Goal sent once to ({goal_x}, {goal_y}, {goal_z})'",
            ]
        }
    )

    yaml_content = {
        "session_name": "sando_sim",
        "windows": [
            {
                "window_name": "main",
                "layout": "tiled",
                "shell_command_before": workspace_source_commands(ros_domain_id),
                "panes": panes,
            }
        ],
    }

    return yaml.dump(yaml_content, default_flow_style=False, sort_keys=False)


def generate_adversarial_test_yaml(
    setup_bash: Path,
    evader_start: tuple = (0, 0, 2.0),
    chaser_start: tuple = (8, 0, 2.0),
    evader_v_max: float = 5.0,
    chaser_v_max: float = 1.0,
    ros_domain_id: int = 20,
    use_rviz: bool = True,
) -> str:
    """Generate YAML for adversarial hover-avoidance test.

    Two SANDO agents:
      - NX01 (evader): hovers at start, hover avoidance enabled, high v_max.
      - NX02 (chaser): continuously navigates toward NX01's position, low v_max.

    A chaser_goal_forwarder node forwards NX01's state as NX02's goal.
    Both agents share trajectories on /trajs so NX01 sees NX02 approaching.
    """
    ex, ey, ez = evader_start
    cx, cy, cz = chaser_start

    panes = []

    # Pane 1: RViz
    if use_rviz:
        panes.append({"shell_command": [f"rviz2 -d {RVIZ_CONFIG}"]})

    # Pane 2: Evader (NX01) — hovers at start, hover avoidance on, high v_max
    panes.append(
        {
            "shell_command": [
                "sleep 2",
                f"ros2 launch sando onboard_sando.launch.py namespace:=NX01 "
                f"x:={ex} y:={ey} z:={ez} "
                f"sim_env:=rviz_only "
                f"publish_odom:=true "
                f"odom_topic:=odom "
                f"v_max:={evader_v_max}",
            ]
        }
    )

    # Pane 3: Chaser (NX02) — follows evader, low v_max, hover avoidance OFF
    panes.append(
        {
            "shell_command": [
                "sleep 2",
                f"ros2 launch sando onboard_sando.launch.py namespace:=NX02 "
                f"x:={cx} y:={cy} z:={cz} "
                f"sim_env:=rviz_only "
                f"publish_odom:=true "
                f"odom_topic:=odom "
                f"v_max:={chaser_v_max} "
                f"hover_avoidance_enabled:=false "
                f"ignore_other_trajs:=true",
            ]
        }
    )

    # Pane 4: One-shot goal for evader (goal == start so it enters GOAL_REACHED)
    evader_goal_yaml = (
        f"'{{header: {{frame_id: map}}, "
        f"pose: {{position: {{x: {ex}, y: {ey}, z: {ez}}}, "
        f"orientation: {{w: 1.0}}}}}}'"
    )
    panes.append(
        {
            "shell_command": [
                "sleep 4",
                f"ros2 topic pub --once /NX01/term_goal geometry_msgs/msg/PoseStamped {evader_goal_yaml} && "
                f"echo '[adversarial-test] Evader goal sent to ({ex}, {ey}, {ez})'",
            ]
        }
    )

    # Pane 5: Chaser goal forwarder (NX01 position -> NX02 goal)
    panes.append(
        {
            "shell_command": [
                "sleep 6",
                "ros2 run sando chaser_goal_forwarder.py "
                "--ros-args "
                "-p evader_ns:=NX01 "
                "-p chaser_ns:=NX02 "
                "-p rate_hz:=2.0",
            ]
        }
    )

    yaml_content = {
        "session_name": "sando_sim",
        "windows": [
            {
                "window_name": "main",
                "layout": "tiled",
                "shell_command_before": workspace_source_commands(ros_domain_id),
                "panes": panes,
            }
        ],
    }

    return yaml.dump(yaml_content, default_flow_style=False, sort_keys=False)


def generate_gazebo_yaml(
    setup_bash: Path,
    goal: tuple,
    env: str = "hard_forest",
    start_pos: tuple = (0, 0, 3.0),
    start_yaw: float = 0.0,
    ros_domain_id: int = 20,
    use_rviz: bool = True,
    use_gazebo_gui: bool = True,
    use_dyn_obs: bool = False,
    use_mapper: bool = True,
    data_file: str = None,
    use_benchmark: bool = False,
    global_planner: str = "astar_heat",
    send_goal: bool = True,
    environment_assumption: str = "",
    depth_topic: str = "mid360_PointCloud2",
) -> str:
    """Generate YAML for single-agent Gazebo simulation."""
    goal_x, goal_y, goal_z = goal
    start_x, start_y, start_z = start_pos

    # Default environment assumption based on env type
    static_envs = {"easy_forest", "medium_forest", "hard_forest"}
    if not environment_assumption:
        environment_assumption = "static" if env in static_envs else "dynamic"

    panes = [
        # Base station with Gazebo
        {
            "shell_command": [
                f"ros2 launch sando base_sando.launch.py use_dyn_obs:={str(use_dyn_obs).lower()} "
                f"use_gazebo_gui:={str(use_gazebo_gui).lower()} use_rviz:={str(use_rviz).lower()} env:={env} "
                f"rviz_config:={RVIZ_CONFIG}"
            ]
        }
    ]

    # ACL mapper (optional)
    if use_mapper:
        static_envs = {"easy_forest", "medium_forest", "hard_forest"}
        param_file = (
            "static_global_mapper.yaml" if env in static_envs else "global_mapper.yaml"
        )
        panes.append(
            {
                "shell_command": [
                    "sleep 10",
                    f"ros2 launch global_mapper_ros global_mapper_node.launch.py quad:=NX01 depth_pointcloud_topic:={depth_topic} param_file:={param_file}",
                ]
            }
        )

    # Onboard agent NX01
    panes.append(
        {
            "shell_command": [
                "sleep 10",
                f"ros2 launch sando onboard_sando.launch.py namespace:=NX01 x:={start_x} y:={start_y} z:={start_z} yaw:={start_yaw} "
                + (
                    f"environment_assumption:={environment_assumption} "
                    if environment_assumption
                    else ""
                )
                + (
                    f"use_benchmark:=true data_file:={data_file} global_planner:={global_planner} "
                    if use_benchmark and data_file
                    else ""
                ),
            ]
        }
    )

    # Goal sender (conditional on send_goal)
    if send_goal:
        panes.append(
            {
                "shell_command": [
                    "sleep 20",
                    f"ros2 launch sando goal_sender.launch.py list_agents:=\"['NX01']\" list_goals:=\"['[{goal_x}, {goal_y}, {goal_z}]']\"",
                ]
            }
        )

    yaml_content = {
        "session_name": "sando_sim",
        "windows": [
            {
                "window_name": "main",
                "layout": "tiled",
                "shell_command_before": workspace_source_commands(ros_domain_id),
                "panes": panes,
            }
        ],
    }

    return yaml.dump(yaml_content, default_flow_style=False, sort_keys=False)


# ---------------------------------------------------------------------------
# Benchmark-record mode: constants and helpers
# ---------------------------------------------------------------------------

BENCHMARK_CONFIGS = [
    # (name, mode, env, num_obstacles, dynamic_ratio)
    # Static: Gazebo worlds with obstacles baked into the world file
    ("static_easy", "gazebo", "easy_forest", 0, 0.0),
    ("static_medium", "gazebo", "medium_forest", 0, 0.0),
    ("static_hard", "gazebo", "hard_forest", 0, 0.0),
    # Dynamic: rviz-only with procedurally generated obstacles (ground truth)
    ("dynamic_easy", "rviz-only", None, 50, 0.65),
    ("dynamic_medium", "rviz-only", None, 100, 0.65),
    ("dynamic_hard", "rviz-only", None, 200, 0.65),
    # Unknown dynamic: Gazebo + dynamic obstacles, pointcloud only (no ground truth to planner)
    ("unknown_dynamic_easy", "gazebo-dynamic", "empty_wo_ground", 50, 0.65),
    ("unknown_dynamic_medium", "gazebo-dynamic", "empty_wo_ground", 100, 0.65),
    ("unknown_dynamic_hard", "gazebo-dynamic", "empty_wo_ground", 200, 0.65),
]

RVIZ_RECORD_TOPICS = [
    # Global
    "/map_generator/global_cloud",
    "/shapes_dynamic_mesh",
    "/trajs",
    "/tf",
    "/tf_static",
    # NX01 Planning
    "/NX01/drone_marker",
    "/NX01/traj_committed_colored",
    "/NX01/traj_subopt_colored",
    "/NX01/actual_traj",
    "/NX01/hgp_path_marker",
    "/NX01/free_hgp_path_marker",
    "/NX01/original_hgp_path_marker",
    "/NX01/local_global_path_after_push_marker",
    "/NX01/poly_safe",
    "/NX01/poly_whole",
    "/NX01/cp",
    "/NX01/p_points",
    "/NX01/point_A",
    "/NX01/point_E",
    "/NX01/point_G",
    "/NX01/fov",
    "/NX01/vel_text",
    "/NX01/term_goal",
    "/NX01/frontiers",
    "/NX01/static_push_points",
    # NX01 Mapping
    "/NX01/occupancy_grid",
    "/NX01/unknown_grid",
    "/NX01/dynamic_grid",
    "/NX01/static_map_marker",
    "/NX01/dynamic_map_marker",
    "/NX01/free_map_marker",
    "/NX01/unknown_map_marker",
    "/NX01/mid360_PointCloud2",
    # NX01 Tracking & Prediction
    "/NX01/tracked_obstacles",
    "/NX01/cluster_bounding_boxes",
    "/NX01/uncertainty_spheres",
    "/NX01/predicted_trajs",
    # NX01 State (for monitoring)
    "/NX01/goal",
    "/NX01/goal_reached",
]


def kill_all_sando_processes():
    """Kill all sando-related processes for a clean slate between runs.

    Mirrors the cleanup logic from run_benchmark.py.
    """
    subprocess.run(
        ["tmux", "kill-session", "-t", "sando_sim"],
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
    )

    for process_name in [
        "rviz2",
        "fake_sim",
        "dynamic_forest_node",
        "sando_node",
        "goal_sender",
        "gzserver",
        "gzclient",
        "obstacle_tracker_node",
    ]:
        subprocess.run(
            ["pkill", "-9", "-x", process_name],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
        )

    subprocess.run(
        ["pkill", "-9", "tmuxp"], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
    )

    for pattern in ["ros2 launch sando", "ros2 bag record"]:
        subprocess.run(
            ["pkill", "-9", "-f", pattern],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
        )

    time.sleep(1)


def send_goal_via_rclpy(namespace: str, goal: tuple) -> None:
    """Send a goal to /{namespace}/term_goal using rclpy.

    Caller must ensure rclpy is already initialised.  Publishes 3 times
    with 0.3 s gaps to ensure delivery (same pattern as run_benchmark.py).
    """
    import rclpy
    from geometry_msgs.msg import PoseStamped

    node = rclpy.create_node("benchmark_record_goal_sender")
    pub = node.create_publisher(PoseStamped, f"/{namespace}/term_goal", 10)
    time.sleep(0.5)  # Wait for publisher to connect

    for _ in range(3):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x = float(goal[0])
        msg.pose.position.y = float(goal[1])
        msg.pose.position.z = float(goal[2])
        msg.pose.orientation.w = 1.0
        pub.publish(msg)
        time.sleep(0.3)

    node.destroy_node()


def wait_for_goal_reached(namespace: str, timeout: float) -> bool:
    """Wait for /{namespace}/goal_reached using rclpy, with timeout.

    Caller must ensure rclpy is already initialised.

    Returns True if goal was reached, False on timeout.
    """
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from std_msgs.msg import Empty

    node = rclpy.create_node("benchmark_record_monitor")
    goal_reached = False

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    def _on_goal_reached(msg):
        nonlocal goal_reached
        goal_reached = True

    node.create_subscription(Empty, f"/{namespace}/goal_reached", _on_goal_reached, qos)

    start = time.time()
    try:
        while time.time() - start < timeout and not goal_reached:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()

    return goal_reached


def run_benchmark_record(
    setup_bash: Path,
    ros_domain_id: int = 20,
    timeout: float = 60.0,
    use_rviz: bool = True,
    cases: set = None,
    difficulties: set = None,
) -> None:
    """Run all 6 benchmark configs sequentially, recording rosbags for each.

    Static configs (easy/medium/hard) use Gazebo with world files.
    Dynamic configs (easy/medium/hard) use rviz-only with procedural obstacles.

    For each configuration:
      1. Clean up lingering processes
      2. Generate + launch sim (gazebo or rviz-only, no goal sender)
      3. Start ros2 bag record
      4. Send goal via rclpy
      5. Wait for goal_reached or timeout
      6. Stop bag, tear down sim
    """
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    namespace = "NX01"
    start_pos = (0, 0, 2)
    goal = (105, 0, 2)
    seed = 0

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = (
        Path(__file__).parent.parent
        / "benchmark_data"
        / "visualization_bags"
        / timestamp
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"[INFO] Benchmark bags will be saved to: {output_dir}")

    os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
    rclpy.init()

    # Persistent node — stays alive across all configs so DDS discovery
    # is already established when we publish goals (mirrors BenchmarkMonitor
    # pattern from run_benchmark.py).
    node = rclpy.create_node("benchmark_recorder")

    if cases is None:
        cases = {"static", "dynamic"}
    if difficulties is None:
        difficulties = {"easy", "medium", "hard"}
    configs = [
        c
        for c in BENCHMARK_CONFIGS
        if (
            ("static" in cases and c[1] == "gazebo")
            or ("dynamic" in cases and c[1] == "rviz-only")
            or ("unknown_dynamic" in cases and c[1] == "gazebo-dynamic")
        )
        and any(d in c[0] for d in difficulties)
    ]
    print(
        f"[INFO] Running {len(configs)} configs "
        f"(cases: {', '.join(sorted(cases))}, "
        f"difficulties: {', '.join(sorted(difficulties))})"
    )

    env = os.environ.copy()
    env["SETUP_BASH"] = str(setup_bash)

    for i, (config_name, mode, gazebo_env, num_obstacles, dynamic_ratio) in enumerate(
        configs
    ):
        print(f"\n{'=' * 60}")
        print(
            f"[{i + 1}/{len(configs)}] Config: {config_name}  "
            f"(mode={mode}, env={gazebo_env}, obstacles={num_obstacles}, "
            f"dynamic_ratio={dynamic_ratio})"
        )
        print(f"{'=' * 60}")

        # 1. Cleanup
        print("[INFO] Cleaning up previous processes...")
        kill_all_sando_processes()

        # 2. Generate tmux YAML (no goal sender — we send goal manually)
        env_assumption = "static" if mode == "gazebo" else "dynamic"
        if mode == "gazebo":
            yaml_content = generate_gazebo_yaml(
                setup_bash,
                goal=goal,
                env=gazebo_env,
                start_pos=start_pos,
                ros_domain_id=ros_domain_id,
                use_rviz=use_rviz,
                use_gazebo_gui=False,
                use_dyn_obs=False,
                send_goal=False,
                environment_assumption=env_assumption,
            )
        elif mode == "gazebo-dynamic":
            yaml_content = generate_gazebo_dynamic_yaml(
                setup_bash,
                goal=goal,
                env=gazebo_env,
                start_pos=start_pos,
                ros_domain_id=ros_domain_id,
                use_rviz=use_rviz,
                use_gazebo_gui=False,
                num_dyn_obstacles=num_obstacles,
                dynamic_ratio=dynamic_ratio,
                seed=seed,
                send_goal=False,
                publish_trajs=False,
                trajs_topic="/trajs_ground_truth",
                depth_topic="d435/depth/color/points",
                use_benchmark=True,
            )
        else:  # rviz-only
            yaml_content = generate_rviz_only_yaml(
                setup_bash,
                goal=goal,
                start_pos=start_pos,
                ros_domain_id=ros_domain_id,
                num_obstacles=num_obstacles,
                dynamic_ratio=dynamic_ratio,
                seed=seed,
                send_goal=False,
                use_rviz=use_rviz,
                environment_assumption=env_assumption,
                publish_obstacle_tf=False,
            )

        # 3. Launch via tmuxp (detached)
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            f.write(yaml_content)
            temp_yaml = f.name

        try:
            print("[INFO] Launching simulation...")
            subprocess.run(["tmuxp", "load", "-d", temp_yaml], env=env, check=True)
        finally:
            if os.path.exists(temp_yaml):
                os.unlink(temp_yaml)

        # 4. Wait for sim to initialise (Gazebo needs longer).
        #    Spin the persistent node during the wait so DDS discovers
        #    the new sim's subscribers as they come up.
        if mode in ("gazebo", "gazebo-dynamic"):
            init_wait = 20
        else:
            init_wait = 10
        print(f"[INFO] Waiting {init_wait} s for simulation to initialise...")
        wait_end = time.time() + init_wait
        while time.time() < wait_end:
            rclpy.spin_once(node, timeout_sec=0.1)

        # 5. Start bag recording
        bag_dir = output_dir / config_name
        bag_dir.mkdir(parents=True, exist_ok=True)
        bag_path = str(bag_dir / "recording")

        bag_env = os.environ.copy()
        bag_env["ROS_DOMAIN_ID"] = str(ros_domain_id)

        record_topics = list(RVIZ_RECORD_TOPICS)
        if mode == "gazebo":
            record_topics.append("/NX01/d435/color/image_raw")
        elif mode == "gazebo-dynamic":
            record_topics.append("/NX01/d435/color/image_raw")
            record_topics.append("/trajs_ground_truth")

        print(f"[INFO] Starting rosbag recording -> {bag_path}")
        bag_proc = subprocess.Popen(
            ["ros2", "bag", "record", "-o", bag_path] + record_topics,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=bag_env,
        )
        # Let recorder stabilise while spinning for DDS discovery
        bag_wait_end = time.time() + 5
        while time.time() < bag_wait_end:
            rclpy.spin_once(node, timeout_sec=0.1)

        # 6. Send goal (using persistent node — already discovered by sim)
        print(f"[INFO] Sending goal {goal} to /{namespace}/term_goal")
        goal_pub = node.create_publisher(PoseStamped, f"/{namespace}/term_goal", 10)
        # Brief spin to let publisher match with subscriber
        pub_wait_end = time.time() + 1.0
        while time.time() < pub_wait_end:
            rclpy.spin_once(node, timeout_sec=0.1)

        for _ in range(3):
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x = float(goal[0])
            msg.pose.position.y = float(goal[1])
            msg.pose.position.z = float(goal[2])
            msg.pose.orientation.w = 1.0
            goal_pub.publish(msg)
            time.sleep(0.3)

        # 7. Wait for goal reached or timeout
        print(f"[INFO] Waiting for goal_reached (timeout={timeout}s)...")
        goal_reached = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        def _on_goal_reached(msg):
            nonlocal goal_reached
            goal_reached = True

        goal_sub = node.create_subscription(
            Empty, f"/{namespace}/goal_reached", _on_goal_reached, qos
        )

        start_time = time.time()
        while time.time() - start_time < timeout and not goal_reached:
            rclpy.spin_once(node, timeout_sec=0.1)

        if goal_reached:
            print("[INFO] Goal reached! Recording 3 s of extra data...")
            time.sleep(3)
        else:
            print(f"[WARN] Timeout after {timeout}s — stopping recording")

        # Cleanup per-config ROS entities
        node.destroy_publisher(goal_pub)
        node.destroy_subscription(goal_sub)

        # 8. Stop bag recorder
        bag_proc.terminate()
        try:
            bag_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bag_proc.kill()
            bag_proc.wait()

        # 9. Tear down sim
        kill_all_sando_processes()
        time.sleep(2)

        status = "REACHED" if goal_reached else "TIMEOUT"
        print(f"[INFO] {config_name} done ({status}). Bag: {bag_path}")

    node.destroy_node()
    rclpy.shutdown()

    print(f"\n{'=' * 60}")
    print("[INFO] All benchmark recordings complete!")
    print(f"[INFO] Bags saved to: {output_dir}")
    print(f"{'=' * 60}")


def main():
    parser = argparse.ArgumentParser(
        description="SANDO Simulation Launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "--mode",
        "-m",
        choices=[
            "multiagent",
            "gazebo",
            "gazebo-dynamic",
            "rviz-only",
            "interactive",
            "hover-test",
            "adversarial-test",
            "benchmark-record",
            "static",
            "dynamic",
            "unknown_dynamic",
        ],
        default="gazebo",
        help="Simulation mode. Simplified demo modes: static, dynamic, unknown_dynamic, interactive "
        "(use with --difficulty). Advanced modes: multiagent, gazebo, gazebo-dynamic, rviz-only, "
        "hover-test, adversarial-test, benchmark-record. [default: gazebo]",
    )

    parser.add_argument(
        "--difficulty",
        "-d",
        choices=["easy", "medium", "hard"],
        default="medium",
        help="Difficulty level for demo modes (static/dynamic/unknown_dynamic). "
        "easy=50 obstacles, medium=100, hard=200. [default: medium]",
    )

    parser.add_argument(
        "--setup-bash",
        "-s",
        type=str,
        required=True,
        help="Path to setup.bash (required)",
    )

    parser.add_argument(
        "--goal",
        "-g",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=[105.0, 0.0, 2.0],
        help="Goal position for gazebo mode (default: 105.0 0.0 2.0)",
    )

    parser.add_argument(
        "--start",
        "-p",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=[0.0, 0.0, 2.0],
        help="Start position for gazebo mode (default: 0.0 0.0 2.0)",
    )

    parser.add_argument(
        "--start-yaw",
        type=float,
        default=0.0,
        help="Start yaw in radians for gazebo mode (default: 0.0)",
    )

    parser.add_argument(
        "--num-agents",
        "-n",
        type=int,
        default=10,
        help="Number of agents for multiagent mode (default: 10)",
    )

    parser.add_argument(
        "--radius",
        "-r",
        type=float,
        default=10.0,
        help="Circle radius for multiagent formation (default: 10.0)",
    )

    parser.add_argument(
        "--env",
        "-e",
        type=str,
        default="hard_forest",
        help="Gazebo environment (default: hard_forest). Options: easy_forest, medium_forest, hard_forest, empty, etc.",
    )

    parser.add_argument(
        "--ros-domain-id", type=int, default=20, help="ROS_DOMAIN_ID (default: 20)"
    )

    parser.add_argument(
        "--rviz", action="store_true", default=True, help="Enable RViz (default: True)"
    )

    parser.add_argument("--no-rviz", action="store_true", help="Disable RViz")

    parser.add_argument(
        "--gazebo-gui",
        action="store_true",
        default=False,
        help="Enable Gazebo GUI (default: False)",
    )

    parser.add_argument(
        "--no-gazebo-gui", action="store_true", help="Disable Gazebo GUI"
    )

    parser.add_argument(
        "--dyn-obs",
        action="store_true",
        help="Enable dynamic obstacles (default: False)",
    )

    parser.add_argument(
        "--no-mapper", action="store_true", help="Disable ACL mapper in gazebo mode"
    )

    parser.add_argument(
        "--num-obstacles",
        type=int,
        default=100,
        help="Number of obstacles for rviz-only mode (default: 50)",
    )

    parser.add_argument(
        "--dynamic-ratio",
        type=float,
        default=0.65,
        help="Ratio of dynamic obstacles (0.0-1.0) for rviz-only mode (default: 0.65)",
    )

    parser.add_argument(
        "--obs-x-range",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=[5.0, 100.0],
        help="X range for obstacles in rviz-only mode (default: 5.0 100.0)",
    )

    parser.add_argument(
        "--obs-y-range",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=[-6.0, 6.0],
        help="Y range for obstacles in rviz-only mode (default: -8.0 8.0)",
    )

    parser.add_argument(
        "--obs-z-range",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=[0.5, 4.5],
        help="Z range for obstacles in rviz-only mode (default: 0.5 4.5)",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Random seed for obstacle generation (default: 0)",
    )

    parser.add_argument(
        "--obstacles-json-file",
        type=str,
        default=None,
        help="Path to shared benchmark obstacle JSON config. "
        "When set, overrides seed-based obstacle generation.",
    )

    parser.add_argument(
        "--data-file",
        type=str,
        default=None,
        help="Path to save benchmark data CSV (enables use_benchmark)",
    )

    parser.add_argument(
        "--use-benchmark",
        action="store_true",
        help="Enable benchmark mode (computation time logging)",
    )

    parser.add_argument(
        "--global-planner",
        type=str,
        default="astar_heat",
        help="Global planner algorithm (default: astar_heat)",
    )

    parser.add_argument(
        "--no-goal-sender",
        action="store_true",
        help="Disable automatic goal sender (for benchmark mode with manual goal sending)",
    )

    parser.add_argument(
        "--with-goal-relay",
        action="store_true",
        help="Add a goal relay pane that forwards RViz 2D Nav Goal clicks to the planner (interactive mode)",
    )

    parser.add_argument(
        "--ground-truth",
        action="store_true",
        help="Enable ground truth /trajs publishing for dynamic obstacles (gazebo-dynamic mode, default: disabled)",
    )

    parser.add_argument(
        "--d435",
        action="store_true",
        help="Use D435 depth camera (d435/depth/color/points) instead of mid360 lidar for pointcloud",
    )

    parser.add_argument(
        "--trajs-topic",
        type=str,
        default="/trajs",
        help="Topic name for DynTraj publishing (default: /trajs). Use /trajs_ground_truth for unknown_dynamic benchmark.",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the generated YAML without launching",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="Per-config timeout in seconds for benchmark-record mode (default: 60.0)",
    )

    parser.add_argument(
        "--benchmark-cases",
        type=str,
        nargs="+",
        choices=["static", "dynamic", "unknown_dynamic", "all"],
        default=["all"],
        help="Which benchmark-record cases to run: static (Gazebo), dynamic (rviz-only), "
        "unknown_dynamic (Gazebo + dynamic obs, pointcloud only), or all (default: all)",
    )

    parser.add_argument(
        "--benchmark-difficulties",
        type=str,
        nargs="+",
        choices=["easy", "medium", "hard", "all"],
        default=["all"],
        help="Which difficulties to run: easy, medium, hard, or all (default: all)",
    )

    args = parser.parse_args()

    # Find setup.bash path
    setup_bash = find_setup_bash(args.setup_bash)
    print(f"[INFO] Using setup.bash: {setup_bash}")

    # Benchmark-record mode: run all 6 configs and exit
    if args.mode == "benchmark-record":
        use_rviz = args.rviz and not args.no_rviz
        cases = set(args.benchmark_cases)
        if "all" in cases:
            cases = {"static", "dynamic", "unknown_dynamic"}
        difficulties = set(args.benchmark_difficulties)
        if "all" in difficulties:
            difficulties = {"easy", "medium", "hard"}
        run_benchmark_record(
            setup_bash=setup_bash,
            ros_domain_id=args.ros_domain_id,
            timeout=args.timeout,
            use_rviz=use_rviz,
            cases=cases,
            difficulties=difficulties,
        )
        return

    # ---- Simplified demo modes: map to internal modes ----
    DIFFICULTY_OBSTACLES = {"easy": 50, "medium": 100, "hard": 200}
    STATIC_ENVS = {
        "easy": "easy_forest",
        "medium": "medium_forest",
        "hard": "hard_forest",
    }

    if args.mode == "static":
        args.mode = "gazebo"
        args.env = STATIC_ENVS[args.difficulty]
        args.goal = [105.0, 0.0, 2.0]
        print(f"[INFO] Demo: static {args.difficulty} ({args.env})")
    elif args.mode == "dynamic":
        args.mode = "rviz-only"
        args.num_obstacles = DIFFICULTY_OBSTACLES[args.difficulty]
        args.dynamic_ratio = 0.65
        args.goal = [105.0, 0.0, 2.0]
        print(
            f"[INFO] Demo: dynamic {args.difficulty} ({args.num_obstacles} obstacles)"
        )
    elif args.mode == "unknown_dynamic":
        args.mode = "gazebo-dynamic"
        args.env = "empty_wo_ground"
        args.num_obstacles = DIFFICULTY_OBSTACLES[args.difficulty]
        args.dynamic_ratio = 0.65
        args.goal = [105.0, 0.0, 2.0]
        print(
            f"[INFO] Demo: unknown_dynamic {args.difficulty} ({args.num_obstacles} obstacles)"
        )

    # Determine sim_env and generate YAML
    if args.mode == "multiagent":
        agents = generate_multiagent_positions(args.num_agents, args.radius)
        yaml_content = generate_multiagent_yaml(setup_bash, agents, args.ros_domain_id)
        print(
            f"[INFO] Mode: Multi-agent simulation with {args.num_agents} agents (fake_sim)"
        )
    elif args.mode == "rviz-only":
        use_rviz = args.rviz and not args.no_rviz
        yaml_content = generate_rviz_only_yaml(
            setup_bash,
            goal=tuple(args.goal),
            start_pos=tuple(args.start),
            start_yaw=args.start_yaw,
            ros_domain_id=args.ros_domain_id,
            num_obstacles=args.num_obstacles,
            dynamic_ratio=args.dynamic_ratio,
            x_min=args.obs_x_range[0],
            x_max=args.obs_x_range[1],
            y_min=args.obs_y_range[0],
            y_max=args.obs_y_range[1],
            z_min=args.obs_z_range[0],
            z_max=args.obs_z_range[1],
            seed=args.seed,
            data_file=args.data_file,
            use_benchmark=args.use_benchmark or (args.data_file is not None),
            global_planner=args.global_planner,
            send_goal=not args.no_goal_sender,
            use_rviz=use_rviz,
            with_goal_relay=args.with_goal_relay,
            obstacles_json_file=args.obstacles_json_file,
        )
        num_dyn = int(args.num_obstacles * args.dynamic_ratio)
        num_stat = args.num_obstacles - num_dyn
        print("[INFO] Mode: RViz-only simulation (no Gazebo)")
        print(
            f"[INFO] Obstacles: {args.num_obstacles} total ({num_dyn} dynamic, {num_stat} static)"
        )
        if args.obstacles_json_file:
            print(f"[INFO] Obstacles loaded from: {args.obstacles_json_file}")
        print(
            f"[INFO] Start: ({args.start[0]}, {args.start[1]}, {args.start[2]}) yaw={args.start_yaw}"
        )
        print(f"[INFO] Goal: ({args.goal[0]}, {args.goal[1]}, {args.goal[2]})")
        print(f"[INFO] Seed: {args.seed}")
    elif args.mode == "interactive":
        import json as _json

        use_rviz = args.rviz and not args.no_rviz
        num_obs = args.num_obstacles
        dyn_ratio = args.dynamic_ratio
        # Pre-generate obstacles with exclusion zone around the origin
        obstacles = _generate_obstacle_json(
            num_obs,
            args.seed,
            x_min=-15.0,
            x_max=15.0,
            y_min=-15.0,
            y_max=15.0,
            z_min=0.5,
            z_max=4.5,
            dynamic_ratio=dyn_ratio,
            exclusion_zone=(-3.0, 3.0, -3.0, 3.0),
        )
        json_path = "/tmp/sando_interactive_obstacles.json"
        with open(json_path, "w") as f:
            _json.dump(obstacles, f)
        yaml_content = generate_rviz_only_yaml(
            setup_bash,
            goal=(0, 0, 2.0),  # dummy, user clicks goals
            start_pos=tuple(args.start),
            start_yaw=args.start_yaw,
            ros_domain_id=args.ros_domain_id,
            num_obstacles=num_obs,
            dynamic_ratio=dyn_ratio,
            x_min=-15.0,
            x_max=15.0,
            y_min=-15.0,
            y_max=15.0,
            z_min=0.5,
            z_max=4.5,
            seed=args.seed,
            send_goal=False,
            use_rviz=use_rviz,
            with_goal_relay=True,
            obstacles_json_file=json_path,
            skip_initial_yawing=True,
        )
        num_dyn = int(num_obs * dyn_ratio)
        num_stat = num_obs - num_dyn
        print("[INFO] Mode: Interactive (click goals in RViz)")
        print("[INFO] Arena: 30x30m, exclusion zone: 6x6m center")
        print(
            f"[INFO] Obstacles: {num_obs} total ({num_dyn} dynamic, {num_stat} static)"
        )
        print("[INFO] Use RViz '2D Nav Goal' to send goals")
    elif args.mode == "gazebo-dynamic":
        use_rviz = args.rviz and not args.no_rviz
        use_gazebo_gui = args.gazebo_gui and not args.no_gazebo_gui
        yaml_content = generate_gazebo_dynamic_yaml(
            setup_bash,
            goal=tuple(args.goal),
            env=args.env,
            start_pos=tuple(args.start),
            start_yaw=args.start_yaw,
            ros_domain_id=args.ros_domain_id,
            use_rviz=use_rviz,
            use_gazebo_gui=use_gazebo_gui,
            num_dyn_obstacles=args.num_obstacles,
            dynamic_ratio=args.dynamic_ratio,
            seed=args.seed,
            x_min=args.obs_x_range[0],
            x_max=args.obs_x_range[1],
            y_min=args.obs_y_range[0],
            y_max=args.obs_y_range[1],
            z_min=args.obs_z_range[0],
            z_max=args.obs_z_range[1],
            send_goal=not args.no_goal_sender,
            publish_trajs=args.ground_truth,
            trajs_topic=args.trajs_topic,
            data_file=args.data_file,
            use_benchmark=args.use_benchmark or (args.data_file is not None),
            global_planner=args.global_planner,
        )
        num_dyn = int(args.num_obstacles * args.dynamic_ratio)
        num_stat = args.num_obstacles - num_dyn
        print("[INFO] Mode: Gazebo + dynamic obstacles (unknown environment)")
        print(f"[INFO] Depth sensor: {'D435' if args.d435 else 'mid360 lidar'}")
        print(f"[INFO] Static world: {args.env} (pointcloud + ACL mapper)")
        print(
            f"[INFO] Obstacles: {args.num_obstacles} total ({num_dyn} dynamic, {num_stat} static)"
        )
        print(
            f"[INFO] Ground truth /trajs: {'enabled' if args.ground_truth else 'disabled'}"
        )
        print(
            f"[INFO] Start: ({args.start[0]}, {args.start[1]}, {args.start[2]}) yaw={args.start_yaw}"
        )
        print(f"[INFO] Goal: ({args.goal[0]}, {args.goal[1]}, {args.goal[2]})")
        print(f"[INFO] Seed: {args.seed}")
    elif args.mode == "hover-test":
        use_rviz = args.rviz and not args.no_rviz
        yaml_content = generate_hover_test_yaml(
            setup_bash,
            start_pos=tuple(args.start),
            ros_domain_id=args.ros_domain_id,
            use_rviz=use_rviz,
        )
        print("[INFO] Mode: Hover avoidance test (empty world + 3 trefoil obstacles)")
        print(f"[INFO] Start/Goal: ({args.start[0]}, {args.start[1]}, {args.start[2]})")
        print(
            "[INFO] Obstacles: 3 trefoil knots orbiting from 3 directions (120 deg apart)"
        )
    elif args.mode == "adversarial-test":
        use_rviz = args.rviz and not args.no_rviz
        yaml_content = generate_adversarial_test_yaml(
            setup_bash,
            evader_start=tuple(args.start),
            ros_domain_id=args.ros_domain_id,
            use_rviz=use_rviz,
        )
        print("[INFO] Mode: Adversarial hover-avoidance test")
        print(
            f"[INFO] Evader (NX01): start=({args.start[0]}, {args.start[1]}, {args.start[2]}), v_max=5.0"
        )
        print(f"[INFO] Chaser (NX02): start=(8, 0, {args.start[2]}), v_max=1.0")
        print("[INFO] Chaser continuously follows evader's position")
    else:  # gazebo
        use_rviz = args.rviz and not args.no_rviz
        use_gazebo_gui = args.gazebo_gui and not args.no_gazebo_gui
        use_mapper = not args.no_mapper
        yaml_content = generate_gazebo_yaml(
            setup_bash,
            goal=tuple(args.goal),
            env=args.env,
            start_pos=tuple(args.start),
            start_yaw=args.start_yaw,
            ros_domain_id=args.ros_domain_id,
            use_rviz=use_rviz,
            use_gazebo_gui=use_gazebo_gui,
            use_dyn_obs=args.dyn_obs,
            use_mapper=use_mapper,
            data_file=args.data_file,
            use_benchmark=args.use_benchmark or (args.data_file is not None),
            global_planner=args.global_planner,
            send_goal=not args.no_goal_sender,
            depth_topic="d435/depth/color/points"
            if args.d435
            else "mid360_PointCloud2",
        )
        print("[INFO] Mode: Single-agent Gazebo simulation")
        print(f"[INFO] Depth sensor: {'D435' if args.d435 else 'mid360 lidar'}")
        print(f"[INFO] Environment: {args.env}")
        print(
            f"[INFO] Start: ({args.start[0]}, {args.start[1]}, {args.start[2]}) yaw={args.start_yaw}"
        )
        print(f"[INFO] Goal: ({args.goal[0]}, {args.goal[1]}, {args.goal[2]})")
        print(
            f"[INFO] RViz: {use_rviz}, Gazebo GUI: {use_gazebo_gui}, Dynamic Obs: {args.dyn_obs}, Mapper: {use_mapper}"
        )

    if args.dry_run:
        print("\n[DRY RUN] Generated YAML:")
        print("-" * 60)
        print(yaml_content)
        print("-" * 60)
        return

    # Write temporary YAML file and launch with tmuxp
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(yaml_content)
        temp_yaml_path = f.name

    try:
        print("[INFO] Launching simulation with tmuxp...")
        env = os.environ.copy()
        env["SETUP_BASH"] = str(setup_bash)

        # Use detached mode if not in a terminal (e.g., running from benchmark script)
        # or if --use-benchmark flag is set
        tmuxp_cmd = ["tmuxp", "load", temp_yaml_path]
        if not sys.stdout.isatty() or args.use_benchmark:
            tmuxp_cmd.insert(2, "-d")  # Add detach flag
            print("[INFO] Running in detached mode (no terminal or benchmark mode)")
        else:
            print("[INFO] Attach to session: tmux attach -t sando_sim")

        subprocess.run(tmuxp_cmd, env=env, check=True)

        # In benchmark mode with detached session, keep the script running
        # so the parent benchmark process can monitor ROS topics
        if not sys.stdout.isatty() or args.use_benchmark:
            print(
                "[INFO] Simulation launched. Monitoring will be done by parent process..."
            )
            print("[INFO] Keeping script alive for monitoring...")
            # Keep the script running - the benchmark will monitor via ROS topics
            # and will kill this process when done
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("[INFO] Monitoring interrupted")

    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to launch simulation: {e}", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print(
            "[ERROR] tmuxp not found. Install with: pip install tmuxp", file=sys.stderr
        )
        sys.exit(1)
    finally:
        # Clean up temp file
        if os.path.exists(temp_yaml_path):
            os.unlink(temp_yaml_path)


if __name__ == "__main__":
    main()
