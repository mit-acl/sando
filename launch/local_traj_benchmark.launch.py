from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare all launch arguments that can be overridden from command line
    return LaunchDescription(
        [
            # Visualization parameters
            DeclareLaunchArgument("visualize", default_value="true"),
            DeclareLaunchArgument("solve_delay_sec", default_value="0.0"),
            DeclareLaunchArgument("playback_period_sec", default_value="0.0"),
            # Benchmark parameters
            DeclareLaunchArgument("use_single_threaded", default_value="false"),
            DeclareLaunchArgument("planner_names", default_value='["sando"]'),
            DeclareLaunchArgument("num_N_list", default_value="[4,5,6]"),
            DeclareLaunchArgument("factor_initial_list", default_value="[2.2,1.7,1.5]"),
            DeclareLaunchArgument("factor_final_list", default_value="[3.8,2.2,1.9]"),
            DeclareLaunchArgument("using_variable_elimination", default_value="true"),
            DeclareLaunchArgument("output_dir_override", default_value=""),
            DeclareLaunchArgument("per_case_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument("max_gurobi_comp_time_sec", default_value="0.5"),
            DeclareLaunchArgument("factor_constant_step_size", default_value="0.1"),
            DeclareLaunchArgument("use_dynamic_factor", default_value="false"),
            DeclareLaunchArgument(
                "dynamic_factor_initial_mean_list", default_value="[1.5,1.5,1.5]"
            ),
            DeclareLaunchArgument("dynamic_factor_k_radius", default_value="0.4"),
            Node(
                package="sando",
                executable="local_traj_benchmark_node",
                name="local_traj_benchmark_node",
                output="screen",
                parameters=[
                    {
                        "visualize": LaunchConfiguration("visualize"),
                        "solve_delay_sec": LaunchConfiguration("solve_delay_sec"),
                        "playback_period_sec": LaunchConfiguration(
                            "playback_period_sec"
                        ),
                        "use_single_threaded": LaunchConfiguration(
                            "use_single_threaded"
                        ),
                        "planner_names": LaunchConfiguration("planner_names"),
                        "num_N_list": LaunchConfiguration("num_N_list"),
                        "factor_initial_list": LaunchConfiguration(
                            "factor_initial_list"
                        ),
                        "factor_final_list": LaunchConfiguration("factor_final_list"),
                        "using_variable_elimination": LaunchConfiguration(
                            "using_variable_elimination"
                        ),
                        "output_dir_override": LaunchConfiguration(
                            "output_dir_override"
                        ),
                        "per_case_timeout_sec": LaunchConfiguration(
                            "per_case_timeout_sec"
                        ),
                        "max_gurobi_comp_time_sec": LaunchConfiguration(
                            "max_gurobi_comp_time_sec"
                        ),
                        "factor_constant_step_size": LaunchConfiguration(
                            "factor_constant_step_size"
                        ),
                        "use_dynamic_factor": LaunchConfiguration("use_dynamic_factor"),
                        "dynamic_factor_initial_mean_list": LaunchConfiguration(
                            "dynamic_factor_initial_mean_list"
                        ),
                        "dynamic_factor_k_radius": LaunchConfiguration(
                            "dynamic_factor_k_radius"
                        ),
                    }
                ],
            ),
        ]
    )
