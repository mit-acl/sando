"""Onboard SANDO launch: planner node, fake sim, and optional obstacle tracker for hardware or simulation."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml
from math import radians


def convert_str_to_bool(str):
    return True if (str == "true" or str == "True" or str == 1 or str == "1") else False


def generate_launch_description():

    # Declare launch arguments

    # initial position and yaw of the quadrotor
    x_arg = DeclareLaunchArgument(
        "x", default_value="20.0", description="Initial x position of the quadrotor"
    )
    y_arg = DeclareLaunchArgument(
        "y", default_value="9.0", description="Initial y position of the quadrotor"
    )
    z_arg = DeclareLaunchArgument(
        "z", default_value="2.0", description="Initial z position of the quadrotor"
    )
    yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="180", description="Initial yaw angle of the quadrotor"
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="NX01", description="Namespace of the nodes"
    )  # namespace
    use_obstacle_tracker_arg = DeclareLaunchArgument(
        "use_obstacle_tracker",
        default_value="false",
        description="Flag to indicate whether to start the obstacle tracker node",
    )  # flag to indicate whether to start the obstacle tracker node
    data_file_arg = DeclareLaunchArgument(
        "data_file",
        default_value="",
        description="File name to store data",
    )  # file name to store data
    global_planner_arg = DeclareLaunchArgument(
        "global_planner", default_value="sjps", description="Global planner to use"
    )  # global planner
    use_benchmark_arg = DeclareLaunchArgument(
        "use_benchmark",
        default_value="false",
        description="Flag to indicate whether to use the global planner benchmark",
    )  # global planner benchmark
    use_hardware_arg = DeclareLaunchArgument(
        "use_hardware",
        default_value="false",
        description="Flag to indicate whether to use hardware or simulation",
    )  # flag to indicte if this is hardware or simulation
    use_onboard_localization_arg = DeclareLaunchArgument(
        "use_onboard_localization",
        default_value="false",
        description="Use onboard localization (DLIO odom) instead of Vicon",
    )
    sim_env_arg = DeclareLaunchArgument(
        "sim_env",
        default_value="",
        description="Simulation environment (gazebo, fake_sim). Empty string uses value from config file.",
    )  # override sim_env from config
    environment_assumption_arg = DeclareLaunchArgument(
        "environment_assumption",
        default_value="",
        description="Override environment_assumption from config (static, dynamic, dynamic_worst_case). Empty string uses value from config file.",
    )
    publish_odom_arg = DeclareLaunchArgument("publish_odom", default_value="true")
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic", default_value="visual_slam/odom"
    )
    odom_frame_id_arg = DeclareLaunchArgument("odom_frame_id", default_value="map")
    v_max_arg = DeclareLaunchArgument(
        "v_max",
        default_value="",
        description="Override v_max from config. Empty string uses value from config file.",
    )
    hover_avoidance_enabled_arg = DeclareLaunchArgument(
        "hover_avoidance_enabled",
        default_value="",
        description="Override hover_avoidance_enabled from config (true/false). Empty string uses value from config file.",
    )
    ignore_other_trajs_arg = DeclareLaunchArgument(
        "ignore_other_trajs",
        default_value="",
        description="Override ignore_other_trajs from config (true/false). Empty string uses value from config file.",
    )
    skip_initial_yawing_arg = DeclareLaunchArgument(
        "skip_initial_yawing",
        default_value="",
        description="Skip YAWING state and go directly to TRAVELING (true/false). Empty string uses value from config file.",
    )

    # Need to be the same as simulartor.launch.py
    map_size_x_arg = DeclareLaunchArgument("map_size_x", default_value="20.0")
    map_size_y_arg = DeclareLaunchArgument("map_size_y", default_value="20.0")
    map_size_z_arg = DeclareLaunchArgument("map_size_z", default_value="6.0")
    odometry_topic_arg = DeclareLaunchArgument(
        "odometry_topic", default_value="visual_slam/odom"
    )

    # Opaque function to launch nodes
    def launch_setup(context, *args, **kwargs):

        x = LaunchConfiguration("x").perform(context)
        y = LaunchConfiguration("y").perform(context)
        z = LaunchConfiguration("z").perform(context)
        yaw = LaunchConfiguration("yaw").perform(context)
        namespace = LaunchConfiguration("namespace").perform(context)
        use_obstacle_tracker = convert_str_to_bool(
            LaunchConfiguration("use_obstacle_tracker").perform(context)
        )
        data_file = LaunchConfiguration("data_file").perform(context)
        global_planner = LaunchConfiguration("global_planner").perform(context)
        use_benchmark = convert_str_to_bool(
            LaunchConfiguration("use_benchmark").perform(context)
        )
        use_hardware = convert_str_to_bool(
            LaunchConfiguration("use_hardware").perform(context)
        )
        use_onboard_localization = convert_str_to_bool(
            LaunchConfiguration("use_onboard_localization").perform(context)
        )
        sim_env_override = LaunchConfiguration("sim_env").perform(context)
        environment_assumption_override = LaunchConfiguration(
            "environment_assumption"
        ).perform(context)
        publish_odom = convert_str_to_bool(
            LaunchConfiguration("publish_odom").perform(context)
        )
        odom_topic = LaunchConfiguration("odom_topic").perform(context)
        odom_frame_id = LaunchConfiguration("odom_frame_id").perform(context)
        v_max_override = LaunchConfiguration("v_max").perform(context)
        hover_avoidance_enabled_override = LaunchConfiguration(
            "hover_avoidance_enabled"
        ).perform(context)
        ignore_other_trajs_override = LaunchConfiguration("ignore_other_trajs").perform(
            context
        )
        skip_initial_yawing_override = LaunchConfiguration(
            "skip_initial_yawing"
        ).perform(context)
        base_frame_id = namespace + "/base_link"
        map_size_x = float(LaunchConfiguration("map_size_x").perform(context))
        map_size_y = float(LaunchConfiguration("map_size_y").perform(context))
        map_size_z = float(LaunchConfiguration("map_size_z").perform(context))
        odometry_topic = LaunchConfiguration("odometry_topic").perform(context)

        # The path to the urdf file
        urdf_path = PathJoinSubstitution(
            [FindPackageShare("sando"), "urdf", "quadrotor.urdf.xacro"]
        )
        if use_hardware:
            parameters_path = os.path.join(
                get_package_share_directory("sando"),
                "config",
                "sando_hw_quadrotor.yaml",
            )
        else:
            parameters_path = os.path.join(
                get_package_share_directory("sando"), "config", "sando.yaml"
            )

        # Get the dict of parameters from the yaml file
        with open(parameters_path, "r") as file:
            parameters = yaml.safe_load(file)

        # Extract specific node parameters
        parameters = parameters["sando_node"]["ros__parameters"]

        # Override sim_env if provided (hardware configs may not have sim_env)
        if sim_env_override:
            parameters["sim_env"] = sim_env_override
        elif "sim_env" not in parameters:
            parameters["sim_env"] = ""

        # Override environment_assumption if provided
        if environment_assumption_override:
            parameters["environment_assumption"] = environment_assumption_override

        # Override v_max if provided
        if v_max_override:
            parameters["v_max"] = float(v_max_override)

        # Override hover_avoidance_enabled if provided
        if hover_avoidance_enabled_override:
            parameters["hover_avoidance_enabled"] = convert_str_to_bool(
                hover_avoidance_enabled_override
            )

        # Override ignore_other_trajs if provided
        if ignore_other_trajs_override:
            parameters["ignore_other_trajs"] = convert_str_to_bool(
                ignore_other_trajs_override
            )

        # Override skip_initial_yawing if provided
        if skip_initial_yawing_override:
            parameters["skip_initial_yawing"] = convert_str_to_bool(
                skip_initial_yawing_override
            )

        # Update parameters for benchmarking
        parameters["file_path"] = data_file
        parameters["use_benchmark"] = bool(use_benchmark)
        if use_benchmark:
            parameters["global_planner"] = global_planner

        # Create a Sando node
        sando_node = Node(
            package="sando",
            executable="sando",
            name="sando_node",
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            parameters=[parameters],
            # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
            arguments=["--ros-args", "--log-level", "error"],
        )

        # Robot state publisher node
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            namespace=namespace,
            parameters=[
                {
                    "robot_description": ParameterValue(
                        Command(
                            [
                                "xacro ",
                                urdf_path,
                                " namespace:=",
                                namespace,
                                " d435_range_max_depth:=",
                                str(parameters["depth_camera_depth_max"]),
                            ]
                        ),
                        value_type=str,
                    ),
                    "use_sim_time": False,
                    "frame_prefix": namespace + "/",
                }
            ],
            arguments=["--ros-args", "--log-level", "error"],
        )

        # Spawn entity node for Gazebo
        # Get the start position and yaw from the parameters
        yaw = str(radians(float(yaw)))
        spawn_entity_node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_entity",
            namespace=namespace,
            parameters=[
                {
                    "use_sim_time": False,
                }
            ],
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                namespace,
                "-x",
                x,
                "-y",
                y,
                "-z",
                z,
                "-Y",
                yaw,
            ],
        )

        # Create an obstacle tracker node
        obstacle_tracker_node = Node(
            package="sando",
            executable="obstacle_tracker_node",
            namespace=namespace,
            name="obstacle_tracker_node",
            emulate_tty=True,
            parameters=[parameters],
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            output="screen",
            remappings=[("point_cloud", "d435/depth/color/points")],
        )

        # Convert pose and twist (from Vicon) to state
        pose_twist_to_state_node = Node(
            package="sando",
            executable="convert_vicon_to_state",
            name="convert_vicon_to_state",
            namespace=namespace,
            remappings=[
                ("world", "world"),  # Remap incoming PoseStamped topic
                ("twist", "twist"),  # Remap incoming TwistStamped topic
                ("state", "state"),  # Remap outgoing State topic
            ],
            emulate_tty=True,
            output="screen",
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            # arguments=['--ros-args', '--log-level', 'error']
        )

        # Convert odom (from DLIO) to global-frame state and pose via TF2
        odom_to_global_state_node = Node(
            package="sando",
            executable="odom_to_global_state",
            name="odom_to_global_state",
            namespace=namespace,
            remappings=[
                ("odom", "dlio/odom_node/odom"),
                ("state", "state"),
                ("global_pose", "global_pose"),
            ],
            emulate_tty=True,
            output="screen",
        )

        # When using ground robot, we don't need to send the exact state to gazebo - the state will be taken care of by wheel controllers
        # send_state_to_gazebo = False if use_ground_robot else True
        # Create a fake sim node
        fake_sim_node = Node(
            package="sando",
            executable="fake_sim",
            name="fake_sim",
            namespace=namespace,
            emulate_tty=True,
            parameters=[
                {
                    "start_pos": [float(x), float(y), float(z)],
                    "start_yaw": float(yaw),
                    "send_state_to_gazebo": parameters["sim_env"] == "gazebo",
                    "visual_level": parameters["visual_level"],
                    "publish_odom": publish_odom,
                    "odom_topic": odom_topic,
                    "odom_frame_id": odom_frame_id,
                    "base_frame_id": base_frame_id,
                }
            ],
            # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
            output="screen",
        )

        camera_file = os.path.join(
            get_package_share_directory("local_sensing"), "config", "camera.yaml"
        )

        pcl_render_node = Node(
            package="local_sensing",
            executable="pcl_render_node",
            namespace=namespace,
            name="pcl_render_node",
            output="screen",
            parameters=[
                {"sensing_horizon": 5.0},
                {"sensing_rate": 30.0},
                {"estimation_rate": 30.0},
                {"map/x_size": map_size_x},
                {"map/y_size": map_size_y},
                {"map/z_size": map_size_z},
                camera_file,
            ],
            remappings=[
                ("global_map", "/map_generator/global_cloud"),
                ("odometry", odometry_topic),
                ("depth", "pcl_render_node/depth"),
            ],
            # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
        )

        # Return launch description
        nodes_to_start = [sando_node]
        if use_hardware and use_onboard_localization:
            nodes_to_start.append(odom_to_global_state_node)
        elif use_hardware:
            nodes_to_start.append(pose_twist_to_state_node)
        else:
            nodes_to_start.append(fake_sim_node)
            if parameters["sim_env"] == "gazebo":
                nodes_to_start.append(robot_state_publisher_node)
                nodes_to_start.append(spawn_entity_node)
            elif parameters["sim_env"] == "fake_sim":
                nodes_to_start.append(pcl_render_node)
        nodes_to_start.append(obstacle_tracker_node) if use_obstacle_tracker else None

        return nodes_to_start

    # Create launch description
    return LaunchDescription(
        [
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            namespace_arg,
            use_obstacle_tracker_arg,
            data_file_arg,
            global_planner_arg,
            use_benchmark_arg,
            use_hardware_arg,
            use_onboard_localization_arg,
            sim_env_arg,
            environment_assumption_arg,
            publish_odom_arg,
            odom_topic_arg,
            odom_frame_id_arg,
            map_size_x_arg,
            map_size_y_arg,
            map_size_z_arg,
            odometry_topic_arg,
            v_max_arg,
            hover_avoidance_enabled_arg,
            ignore_other_trajs_arg,
            skip_initial_yawing_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
