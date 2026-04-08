/* ----------------------------------------------------------------------------
 * Copyright (c) Anonymous Author
 * Anonymous Institution
 * All Rights Reserved
 * Authors: Anonymous
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

// C++ standard library
#include <algorithm>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <execution>

// Eigen
#include <Eigen/Dense>

// PCL
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"  // pcl::SAC_SAMPLE_SIZE // NOLINT
#include <pcl/sample_consensus/model_types.h>
#pragma GCC diagnostic pop
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

// Third-party (TF2, message_filters, PCL conversions)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

// ROS 2 messages
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ROS 2 core
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pcl_ros/transforms.hpp"
#include "std_srvs/srv/empty.hpp"

// Interface messages
#include "sando_interfaces/msg/computation_times.hpp"
#include "sando_interfaces/msg/dyn_traj.hpp"
#include "sando_interfaces/msg/dyn_traj_array.hpp"
#include "sando_interfaces/msg/goal.hpp"
#include "sando_interfaces/msg/pn_adaptation.hpp"
#include "sando_interfaces/msg/state.hpp"
#include "sando_interfaces/msg/yaw_output.hpp"

// Project headers
#include "hgp/utils.hpp"
#include "sando/sando.hpp"
#include "sando/sando_type.hpp"
#include "sando/utils.hpp"

// Synchronization policy for dual point cloud subscriptions
using MySyncPolicy = message_filters::sync_policies::
    ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
using Sync = message_filters::Synchronizer<MySyncPolicy>;

// ROS2 component (run-time composition)
// common practice is to use namespace for components
namespace sando {

using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;

/** @brief ROS 2 node wrapper for the SANDO trajectory planner.
 *
 *  Manages subscriptions, publishers, timers, and visualization for the
 *  planning pipeline.
 */
class SANDO_NODE : public rclcpp::Node {
 public:
  /** @brief Construct the SANDO node, declare parameters, and set up ROS interfaces. */
  SANDO_NODE();

  /** @brief Destructor; logs benchmark data if enabled. */
  ~SANDO_NODE();

 private:
  // Callbacks
  /** @brief Runs the replanning pipeline on a timer and publishes results. */
  void replanCallback();
  /** @brief Handles incoming dynamic trajectory messages from other agents. */
  void trajCallback(const sando_interfaces::msg::DynTraj::SharedPtr msg);
  /** @brief Handles incoming state messages and initializes the planner on first reception. */
  void stateCallback(const sando_interfaces::msg::State::SharedPtr msg);
  /** @brief Handles incoming terminal goal messages from the user or goal sender. */
  void terminalGoalCallback(const geometry_msgs::msg::PoseStamped& msg);
  /** @brief Handles synchronized occupied and unknown point cloud map updates. */
  void mapCallback(
      const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_map_ros,
      const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_unk_ros);
  /** @brief Handles occupancy map updates from the fake simulator. */
  void occupancyMapCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& map_msg);
  /** @brief Checks whether the robot has reached the terminal goal and publishes the event. */
  void goalReachedCheckCallback();
  /** @brief Converts a DynTraj ROS message into an internal DynTraj representation. */
  void convertDynTrajMsg2DynTraj(
      const sando_interfaces::msg::DynTraj& msg,
      std::shared_ptr<DynTraj>& traj,
      double current_time);
  /** @brief Removes expired dynamic obstacle trajectories that are no longer relevant. */
  void cleanUpOldTrajsCallback();
  /** @brief Retrieves the initial pose from TF on hardware startup. */
  void getInitialPoseHwCallback();

  // Others
  /** @brief Declares all ROS 2 parameters with default values. */
  void declareParameters();
  /** @brief Reads declared ROS 2 parameters and applies them to the internal config. */
  void setParameters();
  /** @brief Logs all current parameter values for debugging. */
  void printParameters();
  /** @brief Creates a marker array from a vector of 3D points for RViz visualization. */
  void createMarkerArrayFromVec_Vec3f(
      const vec_Vec3f& occupied_cells,
      const std_msgs::msg::ColorRGBA& color,
      int namespace_id,
      double scale,
      visualization_msgs::msg::MarkerArray* marker_array);
  /** @brief Clears a marker array by publishing delete-all markers. */
  void clearMarkerArray(
      visualization_msgs::msg::MarkerArray& path_marker,
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher);
  /** @brief Runs the fake simulation loop for testing without Gazebo. */
  void runSim();
  /** @brief Prints computation times for the latest replan cycle to the console. */
  void printComputationTime(bool result);
  /** @brief Publishes computation times as a ROS message for external monitoring. */
  void publishComputationTimes(bool result);
  /** @brief Records benchmark data for the current replan cycle. */
  void recordData(bool result);
  /** @brief Writes accumulated benchmark data to a log file. */
  void logData();
  /** @brief Resets all computation time accumulators to zero. */
  void setComputationTimesToZero();
  /** @brief Constructs the field-of-view visualization marker from sensor parameters. */
  void constructFOVMarker();
  /** @brief Retrieves planning results from the SANDO planner after a replan. */
  void retrieveData();
  /** @brief Retrieves data needed for visualization and topic publishing after a replan. */
  void retrieveDataForVisualizationAndTopics();

  // Functions to publish
  /** @brief Publishes the global path as a marker array for RViz. */
  void publishGlobalPath();
  /** @brief Publishes the free (obstacle-cleared) portion of the global path. */
  void publishFreeGlobalPath();
  /** @brief Publishes the convex decomposition polyhedra for corridor visualization. */
  void publishPoly();
  /** @brief Publishes the committed and sub-optimal local trajectories. */
  void publishTraj();
  /** @brief Publishes the robot's own trajectory for multi-agent coordination. */
  void publishOwnTraj();
  /** @brief Publishes the actual trajectory history as a colored line strip. */
  void publishActualTraj();
  /** @brief Publishes the goal setpoints along the planned trajectory. */
  void publishGoal();
  /** @brief Publishes point G, the projected terminal goal on the planning horizon. */
  void publishPointG() const;
  /** @brief Publishes point E, the local trajectory endpoint. */
  void publishPointE() const;
  /** @brief Publishes point A, the starting point for the current replan. */
  void publishPointA() const;
  /** @brief Publishes the current robot state as a point marker. */
  void publishCurrentState(const RobotState& state) const;
  /** @brief Publishes a robot state as a PointStamped message on the given publisher. */
  void publishState(
      const RobotState& data,
      const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr& publisher) const;
  /** @brief Publishes the field-of-view visualization marker. */
  void publishFOV();
  /** @brief Publishes the local trajectory control points for debugging. */
  void publisCps();
  /** @brief Publishes static push point markers used for path deformation. */
  void publishStaticPushPoints();
  /** @brief Publishes the local portion of the global path used for trajectory optimization. */
  void publishLocalGlobalPath();
  /** @brief Publishes a text marker showing the current velocity at the robot's position. */
  void publishVelocityInText(const Eigen::Vector3d& position, double velocity);
  /** @brief Publishes the dynamic obstacle heat map as a colored point cloud. */
  void publishDynamicHeatCloud();
  /** @brief Publishes the occupied voxel map as a point cloud. */
  void publishOccupiedCloud();
  /** @brief Publishes visualization markers for the hover avoidance maneuver. */
  void publishHoverAvoidanceViz();

  // Timers for callback
  rclcpp::TimerBase::SharedPtr timer_replanning_;
  rclcpp::TimerBase::SharedPtr timer_goal_;
  rclcpp::TimerBase::SharedPtr timer_update_tmap_;
  rclcpp::TimerBase::SharedPtr timer_goal_reached_check_;
  rclcpp::TimerBase::SharedPtr timer_cleanup_old_trajs_;
  rclcpp::TimerBase::SharedPtr timer_trajs_update_for_tmap_;
  rclcpp::TimerBase::SharedPtr timer_check_plan_safety_;
  rclcpp::TimerBase::SharedPtr timer_initial_pose_;
  rclcpp::TimerBase::SharedPtr timer_hover_avoidance_viz_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_1_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_2_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_3_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_4_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_5_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_6_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_7_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_8_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mu_9_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_1_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_2_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_3_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_4_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_5_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_6_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_7_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_8_;
  rclcpp::CallbackGroup::SharedPtr cb_group_re_9_;
  rclcpp::CallbackGroup::SharedPtr cb_group_map_;
  rclcpp::CallbackGroup::SharedPtr cb_group_replan_;
  rclcpp::CallbackGroup::SharedPtr cb_group_goal_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_hgp_path_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_original_hgp_path_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_free_hgp_path_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_local_global_path_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_local_global_path_after_push_marker_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_free_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_unknown_map_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_static_map_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dynamic_map_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_free_map_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_unknown_map_marker_;
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_whole_;
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_safe_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_committed_colored_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_subopt_colored_;
  rclcpp::Publisher<sando_interfaces::msg::DynTraj>::SharedPtr pub_own_traj_;
  rclcpp::Publisher<sando_interfaces::msg::Goal>::SharedPtr pub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_G_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_E_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_G_term_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_A_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_current_state_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_goal_reached_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_actual_traj_;
  rclcpp::Publisher<sando_interfaces::msg::YawOutput>::SharedPtr pub_yaw_output_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_fov_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cp_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_static_push_points_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_p_points_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vel_text_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_heat_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_occupied_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_hover_avoidance_viz_;
  rclcpp::Publisher<sando_interfaces::msg::ComputationTimes>::SharedPtr pub_computation_times_;

  // Subscribers
  rclcpp::Subscription<sando_interfaces::msg::DynTraj>::SharedPtr sub_traj_;
  rclcpp::Subscription<sando_interfaces::msg::DynTraj>::SharedPtr sub_predicted_traj_;
  rclcpp::Subscription<sando_interfaces::msg::State>::SharedPtr sub_state_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_terminal_goal_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_fake_sim_occupancy_map_;

  // Time synchronizer
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> occup_grid_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> unknown_grid_sub_;
  std::shared_ptr<Sync> sync_;

  // Visualization
  visualization_msgs::msg::MarkerArray hgp_path_marker_;
  visualization_msgs::msg::MarkerArray original_hgp_path_marker_;
  visualization_msgs::msg::MarkerArray hgp_free_path_marker_;
  visualization_msgs::msg::MarkerArray hgp_local_global_path_marker_;
  visualization_msgs::msg::MarkerArray hgp_local_global_path_after_push_marker_;
  visualization_msgs::msg::MarkerArray traj_committed_colored_;
  visualization_msgs::msg::MarkerArray traj_subopt_colored_;
  visualization_msgs::msg::Marker marker_fov_;

  // Mutex
  std::mutex cloud_callback_mutex_;  // Mutex for cloud callback

  // Parameters
  int id_;
  std::string ns_;
  std::string id_str_;
  Parameters par_;
  double final_g_ = 0.0;                   // only for debugging
  bool verbose_computation_time_ = false;  // only for debugging
  bool local_traj_comp_verbose_ = false;   // only for debugging
  int marker_fov_id_ = 0;

  // SANDO pointer
  std::shared_ptr<SANDO> sando_ptr_;

  // Global Path Benchmarking
  std::string file_path_;  // only for benchmarking
  std::vector<std::tuple<
      bool,    // Result
      double,  // Cost
      double,  // Total replanning time
      double,  // Global planning time
      double,  // CVX decomposition time
      double,  // Local trajectory time
      double,  // Safe paths time
      double,  // Safety Check time
      double,  // Yaw sequence time
      double,  // Yaw fitting time
      double,  // Static JPS time in HGP
      double,  // Check path time in HGP
      double,  // Dynamic A* time in HGP
      double   // Recover path time in HGP
      >>
      global_path_benchmark_;  // only for benchmarking

  // debug
  double replan_last_time_called_ = 0.0;

  // Record computation time
  double global_planning_time_ = 0.0;
  double hgp_static_jps_time_ = 0.0;
  double hgp_check_path_time_ = 0.0;
  double hgp_dynamic_astar_time_ = 0.0;
  double hgp_recover_path_time_ = 0.0;
  double cvx_decomp_time_ = 0.0;
  double local_traj_computation_time_ = 0.0;
  double safe_paths_time_ = 0.0;
  double safety_check_time_ = 0.0;
  double yaw_sequence_time_ = 0.0;
  double yaw_fitting_time_ = 0.0;
  double replanning_computation_time_ = 0.0;
  double successful_factor_ = 0.0;
  double current_time_for_debug_ = 0.0;

  // Visualization
  vec_E<Polyhedron<3>> poly_whole_;
  vec_E<Polyhedron<3>> poly_safe_;
  std::vector<RobotState> goal_setpoints_;
  std::vector<std::vector<RobotState>> list_subopt_goal_setpoints_;
  int actual_traj_id_ = 0;

  // Yaw Optimization Debugging
  std::vector<double> optimal_yaw_sequence_;
  std::vector<double> yaw_control_points_;
  std::vector<double> yaw_knots_;

  // Local trajectory debugging
  std::vector<Eigen::Matrix<double, 3, 4>> cps_;

  // Static push points and p points
  vec_Vecf<3> static_push_points_;
  vec_Vecf<3> p_points_;
  int static_push_points_id_ = 0;
  int p_points_id_ = 0;

  // Trajectory sharing
  PieceWisePol pwp_to_share_;  // Piecewise polynomial

  // Flags
  bool state_initialized_ = false;           // State initialized
  bool replan_timer_started_ = false;        // Replan timer started
  bool publish_actual_traj_called_ = false;  // Publish actual trajectory called
  bool use_benchmark_ = false;               // Use benchmark
  int last_subopt_count_{0};                 // tracks how many subopt strips we drew last time

  // Visualization frame: "map" in sim, "world" in hardware global-frame mode
  std::string viz_frame_ = "map";

  // D435 parameters
  std::string d435_depth_frame_id_;
  std::string lidar_frame_id_;
  std::string d435_camera_info_topic_;

  // TF2 buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // initial pose (for hardware)
  bool initial_pose_received_ = false;
  std::string initial_pose_topic_;
  geometry_msgs::msg::TransformStamped init_pose_transform_stamped_;

  // Timer to make sure we don't sample point cloud too often
  rclcpp::Time last_lidar_callback_time_;
  rclcpp::Time last_depth_camera_callback_time_;

  // ---- Actual trajectory history (for smooth RViz line) ----
  std::vector<RobotState> actual_traj_hist_;
  bool actual_traj_initialized_ = false;

  // For ground-robot velocity approximation
  Eigen::Vector3d actual_traj_prev_pos_{0.0, 0.0, 0.0};
  double actual_traj_prev_time_ = 0.0;

  // Optional: hard caps / visualization knobs
  size_t actual_traj_max_hist_ = 4000;       // max stored states
  size_t actual_traj_max_points_vis_ = 300;  // max points shown in RViz after downsampling
  double actual_traj_line_width_ = 0.15;     // meters
};

}  // namespace sando