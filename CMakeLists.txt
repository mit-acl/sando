cmake_minimum_required(VERSION 3.8)
project(sando)

# Set C++ standard to C++17 (required for std::scoped_lock)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Performance optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -funroll-loops")
# Link-time optimization for better cross-TU optimization
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# avoid Found-related error (ref: https://stackoverflow.com/questions/18327197/opencv-is-considered-to-be-not-found)
set(decomp_rviz_plugins_FOUND 1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED) # Note: if you put this line later on, it gives an error
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(decomp_rviz_plugins REQUIRED)
find_package(decomp_test_node REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(dynus_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nlohmann_json REQUIRED)

# Gurobi (optional): needs a license to run. Without it, only the DAQP backend is available.
option(SANDO_WITH_GUROBI "Build the Gurobi trajectory solver backend if Gurobi is found" ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
if(SANDO_WITH_GUROBI)
  find_package(GUROBI QUIET)
endif()
if(SANDO_WITH_GUROBI AND GUROBI_FOUND)
  message(STATUS "Trajectory solver backends: gurobi, daqp")
else()
  message(STATUS "Trajectory solver backends: daqp (Gurobi not used)")
  set(GUROBI_FOUND FALSE)
endif()

# DAQP: fetched at a fixed release. To use a local checkout (e.g., when building offline), pass
# -DFETCHCONTENT_SOURCE_DIR_DAQP=/path/to/daqp
include(FetchContent)
FetchContent_Declare(
  daqp
  GIT_REPOSITORY https://github.com/darnstrom/daqp.git
  GIT_TAG 44a429a31ebb9e9eea3a12a5d84d3eaeec8c7938  # v0.11.0
)
FetchContent_GetProperties(daqp)
if(NOT daqp_POPULATED)
  FetchContent_Populate(daqp)
  add_subdirectory(${daqp_SOURCE_DIR} ${daqp_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# OpenMP
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# include directories
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS} ${BOOST_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Trajectory solver backends
set(SANDO_SOLVER_SOURCES src/sando/trajectory_solver.cpp src/sando/daqp_solver.cpp)
if(GUROBI_FOUND)
  list(APPEND SANDO_SOLVER_SOURCES src/sando/gurobi_solver.cpp)
endif()
add_library(sando_trajectory_solver STATIC ${SANDO_SOLVER_SOURCES})
set_target_properties(sando_trajectory_solver PROPERTIES POSITION_INDEPENDENT_CODE ON)
# DAQP is private: its generically named headers (utils.h, types.h, ...) stay out of other targets
target_link_libraries(sando_trajectory_solver PRIVATE daqp)
ament_target_dependencies(sando_trajectory_solver PUBLIC rclcpp visualization_msgs Eigen3 decomp_ros_msgs sensor_msgs std_msgs decomp_rviz_plugins decomp_test_node)
if(GUROBI_FOUND)
  FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9].so) #files that are start with libgurobi and end with number.so
  set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};$ENV{GUROBI_HOME}/lib/" )
  target_include_directories(sando_trajectory_solver PUBLIC ${GUROBI_INCLUDE_DIRS})
  target_link_libraries(sando_trajectory_solver PUBLIC ${GUROBI_LIBRARIES})
  target_compile_definitions(sando_trajectory_solver PRIVATE SANDO_HAS_GUROBI)
endif()

# add executables
add_executable(sando src/sando/sando_node.cpp src/sando/sando.cpp src/hgp/hgp_manager.cpp src/hgp/utils.cpp src/hgp/hgp_planner.cpp src/hgp/graph_search.cpp src/sando/utils.cpp)
target_link_libraries(sando sando_trajectory_solver ${PCL_LIBRARIES})
ament_target_dependencies(sando rclcpp example_interfaces dynus_interfaces visualization_msgs Eigen3 decomp_ros_msgs sensor_msgs std_msgs decomp_rviz_plugins decomp_test_node tf2_geometry_msgs tf2 tf2_ros tf2_eigen std_srvs pcl_ros)

# Add the fake_sim executable
add_executable(fake_sim src/sando/fake_sim.cpp)
target_link_libraries(fake_sim)
ament_target_dependencies(fake_sim rclcpp rclcpp_components visualization_msgs dynus_interfaces sensor_msgs gazebo_msgs sensor_msgs tf2 tf2_ros nav_msgs)

# Add the obstacle tracker
add_executable(obstacle_tracker_node src/sando/obstacle_tracker_node.cpp src/hgp/utils.cpp src/sando/utils.cpp)
target_link_libraries(obstacle_tracker_node ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_tracker_node rclcpp rclcpp_components visualization_msgs Eigen3 dynus_interfaces std_msgs gazebo_msgs sensor_msgs tf2 tf2_ros pcl_conversions tf2_geometry_msgs tf2_eigen)

# Add goal to cmd_vel converter
add_executable(convert_goal_to_cmd_vel src/sando/convert_goal_to_cmd_vel.cpp)
target_link_libraries(convert_goal_to_cmd_vel)
ament_target_dependencies(convert_goal_to_cmd_vel rclcpp example_interfaces dynus_interfaces visualization_msgs Eigen3 sensor_msgs std_msgs decomp_rviz_plugins decomp_test_node tf2_geometry_msgs tf2 tf2_ros tf2_eigen)

# Add odom to state converter
add_executable(convert_odom_to_state src/sando/convert_odom_to_state.cpp)
target_link_libraries(convert_odom_to_state)
ament_target_dependencies(convert_odom_to_state rclcpp example_interfaces dynus_interfaces visualization_msgs Eigen3 sensor_msgs std_msgs decomp_rviz_plugins decomp_test_node tf2_geometry_msgs tf2 tf2_ros tf2_eigen)

# Add odom to global state converter (local->global frame via TF2)
add_executable(odom_to_global_state src/sando/odom_to_global_state.cpp)
target_link_libraries(odom_to_global_state)
ament_target_dependencies(odom_to_global_state rclcpp dynus_interfaces Eigen3 tf2 tf2_ros nav_msgs geometry_msgs)

# Add vicon to state converter
add_executable(convert_vicon_to_state src/sando/convert_vicon_to_state.cpp)
target_link_libraries(convert_vicon_to_state)
ament_target_dependencies(convert_vicon_to_state rclcpp example_interfaces dynus_interfaces visualization_msgs Eigen3 decomp_ros_msgs sensor_msgs std_msgs decomp_rviz_plugins decomp_test_node tf2_geometry_msgs tf2 tf2_ros tf2_eigen)

# Add the convert velodyne executable
add_executable(convert_velodyne_to_ros_time src/sando/convert_velodyne_to_ros_time.cpp)
target_link_libraries(convert_velodyne_to_ros_time)
ament_target_dependencies(convert_velodyne_to_ros_time rclcpp rclcpp_components visualization_msgs dynus_interfaces sensor_msgs gazebo_msgs sensor_msgs tf2 tf2_ros nav_msgs)

# For dynamic obstacles in Gazebo (per-model plugin, legacy)
add_library(move_model SHARED src/sim/move_model.cpp)
target_link_libraries(move_model ${GAZEBO_LIBRARIES})
ament_target_dependencies(move_model rclcpp tf2 tf2_ros geometry_msgs)

# World plugin: single plugin manages ALL dynamic obstacles (replaces N move_model instances)
add_library(dynamic_obstacles_world_plugin SHARED src/sim/dynamic_obstacles_world_plugin.cpp)
target_link_libraries(dynamic_obstacles_world_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(dynamic_obstacles_world_plugin)

# For gazebo plugins (IMU)
add_library(imu_plugin SHARED src/sim/gazebo_ros_imu_sensor.cpp)
target_link_libraries(imu_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(imu_plugin sensor_msgs gazebo_ros)

# For dynamic forest node
add_executable(dynamic_forest_node src/sim/dynamic_forest_node.cpp)
ament_target_dependencies(dynamic_forest_node
  rclcpp
  geometry_msgs
  gazebo_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  dynus_interfaces
)

# For benchmark SFC generation
add_executable(corridor_generator_node src/sando/corridor_generator_node.cpp src/hgp/hgp_manager.cpp src/hgp/utils.cpp src/hgp/hgp_planner.cpp src/hgp/graph_search.cpp src/sando/utils.cpp
)
target_link_libraries(corridor_generator_node sando_trajectory_solver ${PCL_LIBRARIES})
ament_target_dependencies(corridor_generator_node rclcpp example_interfaces dynus_interfaces visualization_msgs Eigen3 decomp_ros_msgs sensor_msgs std_msgs decomp_rviz_plugins decomp_test_node tf2_geometry_msgs tf2 tf2_ros tf2_eigen std_srvs pcl_ros)

add_executable(local_traj_benchmark_node src/sando/local_traj_benchmark.cpp src/sando/sando.cpp src/hgp/hgp_manager.cpp src/hgp/utils.cpp src/hgp/hgp_planner.cpp src/hgp/graph_search.cpp src/sando/utils.cpp)
target_link_libraries(local_traj_benchmark_node sando_trajectory_solver ${PCL_LIBRARIES})
ament_target_dependencies(local_traj_benchmark_node rclcpp example_interfaces dynus_interfaces visualization_msgs Eigen3 decomp_ros_msgs sensor_msgs std_msgs decomp_rviz_plugins decomp_test_node tf2_geometry_msgs tf2 tf2_ros tf2_eigen std_srvs pcl_ros)

add_executable(visualize_local_trajs_node src/sando/visualize_local_trajs.cpp src/sando/sando.cpp src/hgp/hgp_manager.cpp src/hgp/utils.cpp src/hgp/hgp_planner.cpp src/hgp/graph_search.cpp src/sando/utils.cpp)
target_link_libraries(visualize_local_trajs_node sando_trajectory_solver ${PCL_LIBRARIES})
ament_target_dependencies(visualize_local_trajs_node rclcpp example_interfaces dynus_interfaces visualization_msgs Eigen3 decomp_ros_msgs sensor_msgs std_msgs decomp_rviz_plugins decomp_test_node tf2_geometry_msgs tf2 tf2_ros tf2_eigen std_srvs pcl_ros)

add_executable(temporal_layered_corridor_test_node src/sando/temporal_layered_corridor_test_node.cpp src/sando/sando.cpp src/hgp/hgp_manager.cpp src/hgp/utils.cpp src/hgp/hgp_planner.cpp src/hgp/graph_search.cpp src/sando/utils.cpp)
target_link_libraries(temporal_layered_corridor_test_node sando_trajectory_solver ${PCL_LIBRARIES})
ament_target_dependencies(temporal_layered_corridor_test_node rclcpp visualization_msgs std_msgs geometry_msgs Eigen3 decomp_util decomp_ros_msgs decomp_rviz_plugins decomp_test_node tf2 tf2_ros tf2_geometry_msgs tf2_eigen std_srvs pcl_ros pcl_conversions sensor_msgs dynus_interfaces)

# Benchmark analysis tool (C++ port of analyze_dynamic_benchmark.py)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
add_executable(analyze_benchmark src/tools/analyze_benchmark.cpp)
ament_target_dependencies(analyze_benchmark rclcpp dynus_interfaces tf2_msgs rosbag2_cpp rosbag2_storage)
target_link_libraries(analyze_benchmark nlohmann_json::nlohmann_json)

# Bag converter tool (lifetime adjustment + pointcloud z-filter)
add_executable(convert_bag src/tools/convert_bag.cpp)
ament_target_dependencies(convert_bag rclcpp visualization_msgs sensor_msgs decomp_ros_msgs rosbag2_cpp rosbag2_storage)

# install for launch and config files
install(DIRECTORY
launch
config
rviz
worlds
urdf
meshes
DESTINATION share/${PROJECT_NAME}
)

# install for executables
install(TARGETS
  sando
  fake_sim
  obstacle_tracker_node
  convert_goal_to_cmd_vel
  convert_odom_to_state
  odom_to_global_state
  convert_vicon_to_state
  convert_velodyne_to_ros_time
  dynamic_forest_node
  corridor_generator_node
  local_traj_benchmark_node
  visualize_local_trajs_node
  temporal_layered_corridor_test_node
  analyze_benchmark
  convert_bag
  DESTINATION lib/${PROJECT_NAME}
)

# install for libraries
install(TARGETS
  daqp
  move_model
  imu_plugin
  dynamic_obstacles_world_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Python modules (none — scripts are installed directly below)

# install Python executables
install(PROGRAMS
  scripts/goal_sender.py
  scripts/goal_relay.py
  scripts/goal_monitor_node.py
  scripts/generate_random_forest.py
  scripts/rviz_obstacles_node.py
  scripts/chaser_goal_forwarder.py
  benchmarking/dynamic_obstacle_benchmark_node.py
  benchmarking/unc_benchmark_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_daqp_solver test/test_daqp_solver.cpp)
  target_link_libraries(test_daqp_solver sando_trajectory_solver)
  add_test(NAME test_daqp_solver COMMAND test_daqp_solver)
endif()

ament_package()
