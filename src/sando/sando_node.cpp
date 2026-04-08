/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <sando/sando_node.hpp>

// ----------------------------------------------------------------------------

SANDO_NODE::SANDO_NODE() : Node("sando_node") {
  // Get id from ns
  ns_ = this->get_namespace();
  ns_ = ns_.substr(ns_.find_last_of("/") + 1);
  id_str_ = ns_.substr(
      ns_.size() - 2);  // ns is like NX01, so we get the last two characters and convert to int
  id_ = std::stoi(id_str_);

  // Declare, set, and print parameters
  this->declareParameters();
  this->setParameters();
  this->printParameters();

  // Qos policy settings
  rclcpp::QoS critical_qos(rclcpp::KeepLast(10));
  critical_qos.reliable().durability_volatile();

  // Create callbackgroup
  this->cb_group_mu_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_5_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_6_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_7_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_8_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_9_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_re_1_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_2_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_3_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_4_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_5_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_6_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_7_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_8_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_9_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_map_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_replan_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_goal_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Options for callback group
  rclcpp::SubscriptionOptions options_re_1;
  options_re_1.callback_group = this->cb_group_re_1_;
  rclcpp::SubscriptionOptions options_re_2;
  options_re_2.callback_group = this->cb_group_re_2_;
  rclcpp::SubscriptionOptions options_map;
  options_map.callback_group = this->cb_group_map_;

  // Visulaization publishers
  pub_dynamic_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "dynamic_occupied_grid", 10);  // visual level 2 (no longer used)
  pub_static_map_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "static_map_marker", 10);  // visual level 2
  pub_dynamic_map_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "dynamic_map_marker", 10);  // visual level 2
  pub_free_map_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "free_map_marker", 10);  // visual level 2
  pub_unknown_map_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "unknown_map_marker", 10);  // visual level 2
  pub_free_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "free_grid", 10);  // visual level 2 (no longer used)
  pub_unknown_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "unknown_grid", 10);  // visual level 2 (no longer used)
  pub_hgp_path_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "hgp_path_marker", 10);  // visual level 1
  pub_original_hgp_path_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "original_hgp_path_marker", 10);  // visual level 1
  pub_free_hgp_path_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "free_hgp_path_marker", 10);  // visual level 1
  pub_local_global_path_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "local_global_path_marker", 10);  // visual level 1
  pub_local_global_path_after_push_marker_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "local_global_path_after_push_marker", 10);  // visual level 1
  pub_poly_whole_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(
      "poly_whole", 10);  // visual level 1
  pub_poly_safe_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(
      "poly_safe", 10);  // visual level 1
  pub_traj_committed_colored_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "traj_committed_colored", 10);  // visual level 1
  pub_traj_subopt_colored_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "traj_subopt_colored", 10);  // visual level 1
  pub_setpoint_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "setpoint_vis",
      10);  // visual level 1
  pub_actual_traj_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "actual_traj", 10);                                                         // visual level 1
  pub_fov_ = this->create_publisher<visualization_msgs::msg::Marker>("fov", 10);  // visual level 1
  pub_cp_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("cp", 10);  // visual level 1
  pub_static_push_points_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "static_push_points", 10);  // visual level 1
  pub_p_points_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "p_points", 10);  // visual level 1
  pub_point_A_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("point_A", 10);  // visual level 1
  pub_point_G_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("point_G", 10);  // visual level 1
  pub_point_E_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("point_E", 10);  // visual level 1
  pub_point_G_term_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "point_G_term", 10);  // visual level 1
  pub_current_state_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "point_current_state", 10);  // visual level 1
  pub_vel_text_ =
      this->create_publisher<visualization_msgs::msg::Marker>("vel_text", 10);  // visual level 1
  pub_dynamic_heat_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("heat_cloud", 10);
  pub_occupied_cloud_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("sando_occupied_cloud", 10);
  pub_hover_avoidance_viz_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("hover_avoidance_viz", 10);
  pub_computation_times_ =
      this->create_publisher<dynus_interfaces::msg::ComputationTimes>("computation_times", 10);

  // Debug publishers
  pub_yaw_output_ = this->create_publisher<dynus_interfaces::msg::YawOutput>("yaw_output", 10);

  // Essential publishers
  pub_own_traj_ = this->create_publisher<dynus_interfaces::msg::DynTraj>("/trajs", critical_qos);
  pub_goal_ = this->create_publisher<dynus_interfaces::msg::Goal>("goal", critical_qos);
  pub_goal_reached_ = this->create_publisher<std_msgs::msg::Empty>("goal_reached", critical_qos);

  // Subscribers
  if (!par_.ignore_other_trajs)
    sub_traj_ = this->create_subscription<dynus_interfaces::msg::DynTraj>(
        "/trajs", critical_qos, std::bind(&SANDO_NODE::trajCallback, this, std::placeholders::_1),
        options_re_1);
  sub_predicted_traj_ = this->create_subscription<dynus_interfaces::msg::DynTraj>(
      "predicted_trajs", critical_qos,
      std::bind(&SANDO_NODE::trajCallback, this, std::placeholders::_1), options_re_1);
  sub_state_ = this->create_subscription<dynus_interfaces::msg::State>(
      "state", critical_qos, std::bind(&SANDO_NODE::stateCallback, this, std::placeholders::_1),
      options_re_1);
  sub_terminal_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "term_goal", critical_qos,
      std::bind(&SANDO_NODE::terminalGoalCallback, this, std::placeholders::_1));

  // Timer for callback
  timer_replanning_ = this->create_wall_timer(
      10ms, std::bind(&SANDO_NODE::replanCallback, this), this->cb_group_replan_);
  timer_goal_ = this->create_wall_timer(
      std::chrono::duration<double>(par_.dc), std::bind(&SANDO_NODE::publishGoal, this),
      this->cb_group_goal_);
  if (use_benchmark_)
    timer_goal_reached_check_ = this->create_wall_timer(
        100ms, std::bind(&SANDO_NODE::goalReachedCheckCallback, this), this->cb_group_re_3_);
  timer_cleanup_old_trajs_ = this->create_wall_timer(
      500ms, std::bind(&SANDO_NODE::cleanUpOldTrajsCallback, this), this->cb_group_mu_5_);
  if (par_.hover_avoidance_enabled)
    timer_hover_avoidance_viz_ = this->create_wall_timer(
        33ms, std::bind(&SANDO_NODE::publishHoverAvoidanceViz, this), this->cb_group_mu_6_);
  if (par_.use_hardware)
    timer_initial_pose_ = this->create_wall_timer(
        100ms, std::bind(&SANDO_NODE::getInitialPoseHwCallback, this), this->cb_group_mu_9_);

  // Stop the timer for callback
  if (timer_replanning_) timer_replanning_->cancel();
  if (timer_goal_) timer_goal_->cancel();
  if (!use_benchmark_ && timer_goal_reached_check_) timer_goal_reached_check_->cancel();

  // Initialize the SANDO object
  sando_ptr_ = std::make_shared<SANDO>(par_);

  // Initialize the tf2 buffer and listener
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);

  // Initialize the d435 depth frame ID and camera
  d435_depth_frame_id_ = ns_ + "/" + ns_ + "_d435_depth_optical_frame";
  lidar_frame_id_ = ns_ + "/NX01_livox";

  // Construct the FOV marker
  constructFOVMarker();

  // Initialize the initial pose topic name
  initial_pose_topic_ = ns_ + "/init_pose";

  if (par_.sim_env == "fake_sim") {
    std::string topic_name = "sensor_point_cloud";
    if (par_.use_global_pc) topic_name = "/map_generator/global_cloud";

    sub_fake_sim_occupancy_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
        std::bind(&SANDO_NODE::occupancyMapCallback, this, std::placeholders::_1), options_map);
  } else if (par_.sim_env == "rviz_only") {
    // In rviz_only mode, initialize map with empty point cloud (no sensor simulation)
    // This initializes the HGP manager so planner can start
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_map_pc(new pcl::PointCloud<pcl::PointXYZ>());
    sando_ptr_->updateOccupancyMapPtr(empty_map_pc);
    RCLCPP_INFO(this->get_logger(), "[rviz_only] Initialized map with empty point cloud");
  } else {
    // Synchronize the occupancy grid and unknown grid
    occup_grid_sub_.subscribe(this, "occupancy_grid", rmw_qos_profile_sensor_data, options_map);
    unknown_grid_sub_.subscribe(this, "unknown_grid", rmw_qos_profile_sensor_data, options_map);
    sync_.reset(new Sync(MySyncPolicy(10), occup_grid_sub_, unknown_grid_sub_));
    sync_->registerCallback(
        std::bind(&SANDO_NODE::mapCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
}

// ----------------------------------------------------------------------------

SANDO_NODE::~SANDO_NODE() {
  // release the memory
  sando_ptr_.reset();
}

// ----------------------------------------------------------------------------

void SANDO_NODE::declareParameters() {
  // Sim enviroment
  this->declare_parameter("sim_env", "fake_sim");
  this->declare_parameter("use_global_pc", true);

  // UAV or Ground robot
  this->declare_parameter("vehicle_type", "uav");
  this->declare_parameter("provide_goal_in_global_frame", false);
  this->declare_parameter("state_already_in_global_frame", false);
  this->declare_parameter("use_hardware", false);

  // Flight mode
  this->declare_parameter("flight_mode", "terminal_goal");

  // Visual
  this->declare_parameter("visual_level", 1);

  // Global planner parameters
  this->declare_parameter("file_path", "");
  this->declare_parameter("use_benchmark", false);
  this->declare_parameter("start_yaw", -90.0);
  this->declare_parameter("global_planner", "sjps");
  this->declare_parameter("global_planner_verbose", false);
  this->declare_parameter("global_planner_heuristic_weight", 1.0);
  this->declare_parameter("factor_hgp", 1.0);
  this->declare_parameter("inflation_hgp", 0.5);
  this->declare_parameter("x_min", -100.0);
  this->declare_parameter("x_max", 100.0);
  this->declare_parameter("y_min", -100.0);
  this->declare_parameter("y_max", 100.0);
  this->declare_parameter("z_min", 0.0);
  this->declare_parameter("z_max", 5.0);
  this->declare_parameter("hgp_timeout_duration_ms", 1000);
  this->declare_parameter("max_num_expansion", 10000);
  this->declare_parameter("use_free_start", false);
  this->declare_parameter("free_start_factor", 1.0);
  this->declare_parameter("use_free_goal", false);
  this->declare_parameter("free_goal_factor", 1.0);
  this->declare_parameter("max_dist_vertexes", 5.0);
  this->declare_parameter("w_unknown", 1.0);
  this->declare_parameter("w_align", 60.0);
  this->declare_parameter("decay_len_cells", 20.0);
  this->declare_parameter("w_side", 0.2);
  this->declare_parameter("heat_weight", 5.0);

  // Heat map parameters
  this->declare_parameter("use_heat_map", true);
  this->declare_parameter("dynamic_heat_enabled", true);
  this->declare_parameter("dynamic_as_occupied_current", true);
  this->declare_parameter("dynamic_as_occupied_future", false);
  this->declare_parameter("use_only_curr_pos_for_dynamic_obst", false);
  this->declare_parameter("heat_alpha0", 1.0);
  this->declare_parameter("heat_alpha1", 2.0);
  this->declare_parameter("heat_p", 2);
  this->declare_parameter("heat_q", 2);
  this->declare_parameter("heat_tau_ratio", 0.5);
  this->declare_parameter("heat_gamma", 0.0);
  this->declare_parameter("heat_Hmax", 10.0);
  this->declare_parameter("dyn_base_inflation_m", 0.5);
  this->declare_parameter("dyn_heat_tube_radius_m", 2.0);
  this->declare_parameter("heat_num_samples", 15);
  this->declare_parameter("static_heat_enabled", true);
  this->declare_parameter("static_heat_alpha", 2.0);
  this->declare_parameter("static_heat_p", 2);
  this->declare_parameter("static_heat_Hmax", 50.0);
  this->declare_parameter("static_heat_rmax_m", 1.0);
  this->declare_parameter("static_heat_default_radius_m", 0.5);
  this->declare_parameter("static_heat_boundary_only", true);
  this->declare_parameter("static_heat_apply_on_unknown", false);
  this->declare_parameter("static_heat_exclude_dynamic", true);
  this->declare_parameter("use_soft_cost_obstacles", false);
  this->declare_parameter("obstacle_soft_cost", 100.0);

  // LOS post processing parameters
  this->declare_parameter("los_cells", 3);
  this->declare_parameter(
      "min_len",
      0.5);  // [m] minimum length between two waypoints after post processing
  this->declare_parameter("min_turn", 10.0);  // [deg] minimum turn angle after post processing

  // Path push visualization parameters
  this->declare_parameter("use_state_update", true);
  this->declare_parameter("use_random_color_for_global_path", false);
  this->declare_parameter("use_path_push_for_visualization", false);

  // Decomposition parameters
  this->declare_parameter("environment_assumption", "static");
  this->declare_parameter("sfc_size", std::vector<float>{2.0, 2.0, 2.0});
  this->declare_parameter("min_dist_from_agent_to_traj", 6.0);
  this->declare_parameter("use_shrinked_box", false);
  this->declare_parameter("shrinked_box_size", 0.2);

  // Map parameters
  this->declare_parameter("map_buffer", 6.0);
  this->declare_parameter("center_shift_factor", 0.5);
  this->declare_parameter("initial_wdx", 1.0);
  this->declare_parameter("initial_wdy", 1.0);
  this->declare_parameter("initial_wdz", 1.0);
  this->declare_parameter("min_wdx", 10.0);
  this->declare_parameter("min_wdy", 10.0);
  this->declare_parameter("min_wdz", 2.0);
  this->declare_parameter("sando_map_res", 0.1);

  // Communication delay parameters
  this->declare_parameter("use_comm_delay_inflation", true);
  this->declare_parameter("comm_delay_inflation_alpha", 0.2);
  this->declare_parameter("comm_delay_inflation_max", 0.5);
  this->declare_parameter("comm_delay_filter_alpha", 0.8);

  // Simulation parameters
  this->declare_parameter("depth_camera_depth_max", 10.0);
  this->declare_parameter("fov_visual_depth", 10.0);
  this->declare_parameter("fov_visual_x_deg", 10.0);
  this->declare_parameter("fov_visual_y_deg", 10.0);

  // Optimization parameters
  this->declare_parameter("horizon", 20.0);
  this->declare_parameter("dc", 0.01);
  this->declare_parameter("dynamic_constraint_type", "Linf");
  this->declare_parameter("v_max", 1.0);
  this->declare_parameter("a_max", 1.0);
  this->declare_parameter("j_max", 1.0);
  this->declare_parameter("jerk_weight", 1.0);
  this->declare_parameter("dynamic_weight", 1.0);
  this->declare_parameter("time_weight", 1.0);
  this->declare_parameter("pos_anchor_weight", 1.0);
  this->declare_parameter("stat_weight", 1.0);
  this->declare_parameter("dyn_constr_bodyrate_weight", 1.0);
  this->declare_parameter("dyn_constr_tilt_weight", 1.0);
  this->declare_parameter("dyn_constr_thrust_weight", 1.0);
  this->declare_parameter("dyn_constr_vel_weight", 1.0);
  this->declare_parameter("dyn_constr_acc_weight", 1.0);
  this->declare_parameter("dyn_constr_jerk_weight", 1.0);
  this->declare_parameter("num_dyn_obst_samples", 10);
  this->declare_parameter("planner_Co", 0.5);
  this->declare_parameter("planner_Cw", 1.0);
  this->declare_parameter("verbose_computation_time", false);
  this->declare_parameter("local_traj_comp_verbose", false);
  this->declare_parameter("drone_bbox", std::vector<double>{0.5, 0.5, 0.5});
  this->declare_parameter("goal_radius", 0.5);
  this->declare_parameter("goal_seen_radius", 2.0);

  // SANDO specific parameters
  this->declare_parameter("num_P", 3);
  this->declare_parameter("num_N", 6);
  this->declare_parameter("use_dynamic_factor", false);
  this->declare_parameter("dynamic_factor_k_radius", 0.3);
  this->declare_parameter("dynamic_factor_initial_mean", 0.5);
  this->declare_parameter("factor_initial", 1.0);
  this->declare_parameter("factor_final", 5.0);
  this->declare_parameter("factor_constant_step_size", 0.1);
  this->declare_parameter("obst_max_vel", 0.5);
  this->declare_parameter("obst_position_error", 0.0);
  this->declare_parameter("inflate_unknown_boundary", true);
  this->declare_parameter("max_gurobi_comp_time_sec", 0.05);
  this->declare_parameter("jerk_smooth_weight", 1.0e+1);

  // Dynamic obstacles parameters
  this->declare_parameter("traj_lifetime", 10.0);

  // Dynamic k_value parameters
  this->declare_parameter("num_replanning_before_adapt", 10);
  this->declare_parameter("default_k_value", 150);
  this->declare_parameter("alpha_k_value_filtering", 0.8);
  this->declare_parameter("k_value_factor", 1.2);

  // Yaw-related parameters
  this->declare_parameter("alpha_filter_dyaw", 0.8);
  this->declare_parameter("w_max", 0.5);
  this->declare_parameter("w_max_yawing", 0.5);
  this->declare_parameter("skip_initial_yawing", false);
  this->declare_parameter("yaw_spinning_threshold", 10);
  this->declare_parameter("yaw_spinning_dyaw", 0.1);

  // Simulation env parameters
  this->declare_parameter("force_goal_z", true);
  this->declare_parameter("default_goal_z", 2.5);

  // Debug flag
  this->declare_parameter("debug_verbose", false);

  // Trajectory sharing
  this->declare_parameter("ignore_other_trajs", false);

  // Hover avoidance parameters
  this->declare_parameter("hover_avoidance_enabled", false);
  this->declare_parameter("hover_avoidance_2d", true);
  this->declare_parameter("hover_avoidance_d_trigger", 4.0);
  this->declare_parameter("hover_avoidance_h", 3.0);
  this->declare_parameter("hover_avoidance_min_repulsion_norm", 0.01);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::setParameters() {
  // Set the parameters

  // Sim enviroment
  par_.sim_env = this->get_parameter("sim_env").as_string();
  par_.use_global_pc = this->get_parameter("use_global_pc").as_bool();

  // Vehicle type (UAV, Wheeled Robit, or Quadruped)
  par_.vehicle_type = this->get_parameter("vehicle_type").as_string();
  par_.provide_goal_in_global_frame = this->get_parameter("provide_goal_in_global_frame").as_bool();
  par_.state_already_in_global_frame =
      this->get_parameter("state_already_in_global_frame").as_bool();
  par_.use_hardware = this->get_parameter("use_hardware").as_bool();

  // Visualization frame: use "world" when operating in global frame (hardware), "map" otherwise
  viz_frame_ = (par_.use_hardware && par_.state_already_in_global_frame) ? "world" : "map";

  // Flight mode
  par_.flight_mode = this->get_parameter("flight_mode").as_string();

  // Visual level
  par_.visual_level = this->get_parameter("visual_level").as_int();

  // Global Planner parameters
  file_path_ = this->get_parameter("file_path").as_string();
  use_benchmark_ = this->get_parameter("use_benchmark").as_bool();
  par_.global_planner = this->get_parameter("global_planner").as_string();
  par_.global_planner_verbose = this->get_parameter("global_planner_verbose").as_bool();
  par_.global_planner_heuristic_weight =
      this->get_parameter("global_planner_heuristic_weight").as_double();
  par_.factor_hgp = this->get_parameter("factor_hgp").as_double();
  par_.inflation_hgp = this->get_parameter("inflation_hgp").as_double();
  par_.x_min = this->get_parameter("x_min").as_double();
  par_.x_max = this->get_parameter("x_max").as_double();
  par_.y_min = this->get_parameter("y_min").as_double();
  par_.y_max = this->get_parameter("y_max").as_double();
  par_.z_min = this->get_parameter("z_min").as_double();
  par_.z_max = this->get_parameter("z_max").as_double();
  par_.hgp_timeout_duration_ms = this->get_parameter("hgp_timeout_duration_ms").as_int();
  par_.max_num_expansion = this->get_parameter("max_num_expansion").as_int();
  par_.use_free_start = this->get_parameter("use_free_start").as_bool();
  par_.free_start_factor = this->get_parameter("free_start_factor").as_double();
  par_.use_free_goal = this->get_parameter("use_free_goal").as_bool();
  par_.free_goal_factor = this->get_parameter("free_goal_factor").as_double();
  par_.max_dist_vertexes = this->get_parameter("max_dist_vertexes").as_double();
  par_.w_unknown = this->get_parameter("w_unknown").as_double();
  par_.w_align = this->get_parameter("w_align").as_double();
  par_.decay_len_cells = this->get_parameter("decay_len_cells").as_double();
  par_.w_side = this->get_parameter("w_side").as_double();
  par_.heat_weight = this->get_parameter("heat_weight").as_double();

  // Heat map parameters
  par_.use_heat_map = this->get_parameter("use_heat_map").as_bool();
  par_.dynamic_heat_enabled = this->get_parameter("dynamic_heat_enabled").as_bool();
  par_.dynamic_as_occupied_current = this->get_parameter("dynamic_as_occupied_current").as_bool();
  par_.dynamic_as_occupied_future = this->get_parameter("dynamic_as_occupied_future").as_bool();
  par_.use_only_curr_pos_for_dynamic_obst =
      this->get_parameter("use_only_curr_pos_for_dynamic_obst").as_bool();
  par_.heat_alpha0 = this->get_parameter("heat_alpha0").as_double();
  par_.heat_alpha1 = this->get_parameter("heat_alpha1").as_double();
  par_.heat_p = this->get_parameter("heat_p").as_int();
  par_.heat_q = this->get_parameter("heat_q").as_int();
  par_.heat_tau_ratio = this->get_parameter("heat_tau_ratio").as_double();
  par_.heat_gamma = this->get_parameter("heat_gamma").as_double();
  par_.heat_Hmax = this->get_parameter("heat_Hmax").as_double();
  par_.dyn_base_inflation_m = this->get_parameter("dyn_base_inflation_m").as_double();
  par_.dyn_heat_tube_radius_m = this->get_parameter("dyn_heat_tube_radius_m").as_double();
  par_.heat_num_samples = this->get_parameter("heat_num_samples").as_int();
  par_.static_heat_enabled = this->get_parameter("static_heat_enabled").as_bool();
  par_.static_heat_alpha = this->get_parameter("static_heat_alpha").as_double();
  par_.static_heat_p = this->get_parameter("static_heat_p").as_int();
  par_.static_heat_Hmax = this->get_parameter("static_heat_Hmax").as_double();
  par_.static_heat_rmax_m = this->get_parameter("static_heat_rmax_m").as_double();
  par_.static_heat_default_radius_m =
      this->get_parameter("static_heat_default_radius_m").as_double();
  par_.static_heat_boundary_only = this->get_parameter("static_heat_boundary_only").as_bool();
  par_.static_heat_apply_on_unknown = this->get_parameter("static_heat_apply_on_unknown").as_bool();
  par_.static_heat_exclude_dynamic = this->get_parameter("static_heat_exclude_dynamic").as_bool();
  par_.use_soft_cost_obstacles = this->get_parameter("use_soft_cost_obstacles").as_bool();
  par_.obstacle_soft_cost = this->get_parameter("obstacle_soft_cost").as_double();

  // LOS post processing parameters
  par_.los_cells = this->get_parameter("los_cells").as_int();
  par_.min_len = this->get_parameter("min_len").as_double();
  par_.min_turn = this->get_parameter("min_turn").as_double();

  // Path push visualization parameters
  par_.use_state_update = this->get_parameter("use_state_update").as_bool();
  par_.use_random_color_for_global_path =
      this->get_parameter("use_random_color_for_global_path").as_bool();
  par_.use_path_push_for_visualization =
      this->get_parameter("use_path_push_for_visualization").as_bool();

  // Static obstacle push parameters

  // Decomposition parameters
  par_.environment_assumption = this->get_parameter("environment_assumption").as_string();
  if (par_.environment_assumption != "static" && par_.environment_assumption != "dynamic" &&
      par_.environment_assumption != "dynamic_worst_case") {
    RCLCPP_ERROR(
        this->get_logger(),
        "Invalid environment_assumption: '%s'. Must be 'static', 'dynamic', or "
        "'dynamic_worst_case'.",
        par_.environment_assumption.c_str());
    rclcpp::shutdown();
    return;
  }
  par_.sfc_size = this->get_parameter("sfc_size").as_double_array();
  par_.min_dist_from_agent_to_traj = this->get_parameter("min_dist_from_agent_to_traj").as_double();
  par_.use_shrinked_box = this->get_parameter("use_shrinked_box").as_bool();
  par_.shrinked_box_size = this->get_parameter("shrinked_box_size").as_double();

  // Map parameters
  par_.map_buffer = this->get_parameter("map_buffer").as_double();
  par_.center_shift_factor = this->get_parameter("center_shift_factor").as_double();
  par_.initial_wdx = this->get_parameter("initial_wdx").as_double();
  par_.initial_wdy = this->get_parameter("initial_wdy").as_double();
  par_.initial_wdz = this->get_parameter("initial_wdz").as_double();
  par_.min_wdx = this->get_parameter("min_wdx").as_double();
  par_.min_wdy = this->get_parameter("min_wdy").as_double();
  par_.min_wdz = this->get_parameter("min_wdz").as_double();
  par_.res = this->get_parameter("sando_map_res").as_double();

  // Communication delay parameters
  par_.use_comm_delay_inflation = this->get_parameter("use_comm_delay_inflation").as_bool();
  par_.comm_delay_inflation_alpha = this->get_parameter("comm_delay_inflation_alpha").as_double();
  par_.comm_delay_inflation_max = this->get_parameter("comm_delay_inflation_max").as_double();
  par_.comm_delay_filter_alpha = this->get_parameter("comm_delay_filter_alpha").as_double();

  // Simulation parameters
  par_.depth_camera_depth_max = this->get_parameter("depth_camera_depth_max").as_double();
  par_.fov_visual_depth = this->get_parameter("fov_visual_depth").as_double();
  par_.fov_visual_x_deg = this->get_parameter("fov_visual_x_deg").as_double();
  par_.fov_visual_y_deg = this->get_parameter("fov_visual_y_deg").as_double();

  // Optimization parameters
  par_.horizon = this->get_parameter("horizon").as_double();
  par_.dc = this->get_parameter("dc").as_double();
  par_.dynamic_constraint_type = this->get_parameter("dynamic_constraint_type").as_string();
  par_.v_max = this->get_parameter("v_max").as_double();
  par_.a_max = this->get_parameter("a_max").as_double();
  par_.j_max = this->get_parameter("j_max").as_double();
  verbose_computation_time_ = this->get_parameter("verbose_computation_time").as_bool();
  local_traj_comp_verbose_ = this->get_parameter("local_traj_comp_verbose").as_bool();
  par_.drone_bbox = this->get_parameter("drone_bbox").as_double_array();
  par_.drone_radius = par_.drone_bbox[0] / 2.0;
  par_.goal_radius = this->get_parameter("goal_radius").as_double();
  par_.goal_seen_radius = this->get_parameter("goal_seen_radius").as_double();

  // SANDO specific parameters
  par_.num_P = this->get_parameter("num_P").as_int();
  par_.num_N = this->get_parameter("num_N").as_int();
  par_.use_dynamic_factor = this->get_parameter("use_dynamic_factor").as_bool();
  par_.dynamic_factor_k_radius = this->get_parameter("dynamic_factor_k_radius").as_double();
  par_.dynamic_factor_initial_mean = this->get_parameter("dynamic_factor_initial_mean").as_double();
  par_.factor_initial = this->get_parameter("factor_initial").as_double();
  par_.factor_final = this->get_parameter("factor_final").as_double();
  par_.factor_constant_step_size = this->get_parameter("factor_constant_step_size").as_double();
  par_.obst_max_vel = this->get_parameter("obst_max_vel").as_double();
  par_.obst_position_error = this->get_parameter("obst_position_error").as_double();
  par_.inflate_unknown_boundary = this->get_parameter("inflate_unknown_boundary").as_bool();
  par_.max_gurobi_comp_time_sec = this->get_parameter("max_gurobi_comp_time_sec").as_double();
  par_.jerk_smooth_weight = this->get_parameter("jerk_smooth_weight").as_double();

  // Dynamic obstacles parameters
  par_.traj_lifetime = this->get_parameter("traj_lifetime").as_double();

  // Dynamic k_value parameters
  par_.num_replanning_before_adapt = this->get_parameter("num_replanning_before_adapt").as_int();
  par_.default_k_value = this->get_parameter("default_k_value").as_int();
  par_.alpha_k_value_filtering = this->get_parameter("alpha_k_value_filtering").as_double();
  par_.k_value_factor = this->get_parameter("k_value_factor").as_double();

  // Yaw-related parameters
  par_.alpha_filter_dyaw = this->get_parameter("alpha_filter_dyaw").as_double();
  par_.w_max = this->get_parameter("w_max").as_double();
  par_.w_max_yawing = this->get_parameter("w_max_yawing").as_double();
  par_.skip_initial_yawing = this->get_parameter("skip_initial_yawing").as_bool();
  par_.yaw_spinning_threshold = this->get_parameter("yaw_spinning_threshold").as_int();
  par_.yaw_spinning_dyaw = this->get_parameter("yaw_spinning_dyaw").as_double();

  // Simulation env parameters
  par_.force_goal_z = this->get_parameter("force_goal_z").as_bool();
  par_.default_goal_z = this->get_parameter("default_goal_z").as_double();

  if (par_.default_goal_z <= par_.z_min) {
    RCLCPP_ERROR(this->get_logger(), "Default goal z is lower than the ground level");
  }

  if (par_.default_goal_z >= par_.z_max) {
    RCLCPP_ERROR(this->get_logger(), "Default goal z is higher than the max level");
  }

  // Debug flag
  par_.debug_verbose = this->get_parameter("debug_verbose").as_bool();

  // Trajectory sharing
  par_.ignore_other_trajs = this->get_parameter("ignore_other_trajs").as_bool();

  // Hover avoidance parameters
  par_.hover_avoidance_enabled = this->get_parameter("hover_avoidance_enabled").as_bool();
  par_.hover_avoidance_2d = this->get_parameter("hover_avoidance_2d").as_bool();
  par_.hover_avoidance_d_trigger = this->get_parameter("hover_avoidance_d_trigger").as_double();
  par_.hover_avoidance_h = this->get_parameter("hover_avoidance_h").as_double();
  par_.hover_avoidance_min_repulsion_norm =
      this->get_parameter("hover_avoidance_min_repulsion_norm").as_double();
}

// ----------------------------------------------------------------------------

void SANDO_NODE::printParameters() {
  // Print the parameters

  // Sim enviroment
  RCLCPP_INFO(this->get_logger(), "Sim Enviroment: %s", par_.sim_env.c_str());
  RCLCPP_INFO(this->get_logger(), "Use Global Point Cloud: %d", par_.use_global_pc);

  // Vehicle type (UAV, Wheeled Robit, or Quadruped)
  RCLCPP_INFO(this->get_logger(), "Vehicle Type: %d", par_.vehicle_type);
  RCLCPP_INFO(
      this->get_logger(), "Provide Goal in Global Frame: %d", par_.provide_goal_in_global_frame);
  RCLCPP_INFO(this->get_logger(), "Use Hardware: %d", par_.use_hardware);

  // Flight mode
  RCLCPP_INFO(this->get_logger(), "Flight Mode: %s", par_.flight_mode.c_str());

  // Visual
  RCLCPP_INFO(this->get_logger(), "Visual Level: %d", par_.visual_level);

  // HGP parameters
  RCLCPP_INFO(this->get_logger(), "File Path: %s", file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Perform Benchmark?: %d", use_benchmark_);
  RCLCPP_INFO(this->get_logger(), "Global Planner: %s", par_.global_planner.c_str());
  RCLCPP_INFO(this->get_logger(), "Global Planner Verbose: %d", par_.global_planner_verbose);
  RCLCPP_INFO(
      this->get_logger(), "Global Planner Huristic Weight: %f",
      par_.global_planner_heuristic_weight);
  RCLCPP_INFO(this->get_logger(), "Factor HGP: %f", par_.factor_hgp);
  RCLCPP_INFO(this->get_logger(), "Inflation HGP: %f", par_.inflation_hgp);
  RCLCPP_INFO(this->get_logger(), "X Min: %f", par_.x_min);
  RCLCPP_INFO(this->get_logger(), "X Max: %f", par_.x_max);
  RCLCPP_INFO(this->get_logger(), "Y Min: %f", par_.y_min);
  RCLCPP_INFO(this->get_logger(), "Y Max: %f", par_.y_max);
  RCLCPP_INFO(this->get_logger(), "Z Ground: %f", par_.z_min);
  RCLCPP_INFO(this->get_logger(), "Z Max: %f", par_.z_max);
  RCLCPP_INFO(this->get_logger(), "HGP Timeout Duration: %d", par_.hgp_timeout_duration_ms);
  RCLCPP_INFO(this->get_logger(), "Use Free Start?: %d", par_.use_free_start);
  RCLCPP_INFO(this->get_logger(), "Free Start Factor: %f", par_.free_start_factor);
  RCLCPP_INFO(this->get_logger(), "Use Free Goal?: %d", par_.use_free_goal);
  RCLCPP_INFO(this->get_logger(), "Free Goal Factor: %f", par_.free_goal_factor);
  RCLCPP_INFO(this->get_logger(), "max_dist_vertexes: %f", par_.max_dist_vertexes);
  RCLCPP_INFO(this->get_logger(), "w_unknown: %f", par_.w_unknown);
  RCLCPP_INFO(this->get_logger(), "w_align: %f", par_.w_align);
  RCLCPP_INFO(this->get_logger(), "decay_len_cells: %f", par_.decay_len_cells);
  RCLCPP_INFO(this->get_logger(), "w_side: %f", par_.w_side);
  RCLCPP_INFO(this->get_logger(), "heat_weight: %f", par_.heat_weight);

  // LOS post processing parameters
  RCLCPP_INFO(this->get_logger(), "LOS Cells: %d", par_.los_cells);
  RCLCPP_INFO(this->get_logger(), "Min Len: %f", par_.min_len);
  RCLCPP_INFO(this->get_logger(), "Min Turn: %f", par_.min_turn);

  // Path push visualization parameters
  RCLCPP_INFO(this->get_logger(), "Use State Update?: %d", par_.use_state_update);
  RCLCPP_INFO(
      this->get_logger(), "Use Random Color for Global Path?: %d",
      par_.use_random_color_for_global_path);
  RCLCPP_INFO(
      this->get_logger(), "Use Path Push for Paper?: %d", par_.use_path_push_for_visualization);

  // Static obstacle push parameters
  RCLCPP_INFO(
      this->get_logger(), "Environment Assumption: %s", par_.environment_assumption.c_str());
  RCLCPP_INFO(
      this->get_logger(), "Local Box Size: (%f, %f, %f)", par_.sfc_size[0], par_.sfc_size[1],
      par_.sfc_size[2]);
  RCLCPP_INFO(
      this->get_logger(), "Min Dist from Agent to Traj: %f", par_.min_dist_from_agent_to_traj);
  RCLCPP_INFO(this->get_logger(), "Use Shrinked Box: %d", par_.use_shrinked_box);
  RCLCPP_INFO(this->get_logger(), "Shrinked Box Size: %f", par_.shrinked_box_size);

  // Map parameters
  RCLCPP_INFO(this->get_logger(), "Local Map Buffer: %f", par_.map_buffer);
  RCLCPP_INFO(this->get_logger(), "Center Shift Factor: %f", par_.center_shift_factor);
  RCLCPP_INFO(this->get_logger(), "initial_wdx: %f", par_.initial_wdx);
  RCLCPP_INFO(this->get_logger(), "initial_wdy: %f", par_.initial_wdy);
  RCLCPP_INFO(this->get_logger(), "initial_wdz: %f", par_.initial_wdz);
  RCLCPP_INFO(this->get_logger(), "min_wdx: %f", par_.min_wdx);
  RCLCPP_INFO(this->get_logger(), "min_wdy: %f", par_.min_wdy);
  RCLCPP_INFO(this->get_logger(), "min_wdz: %f", par_.min_wdz);
  RCLCPP_INFO(this->get_logger(), "Res: %f", par_.res);

  // Communication delay parameters
  RCLCPP_INFO(this->get_logger(), "Use Comm Delay Inflation: %d", par_.use_comm_delay_inflation);
  RCLCPP_INFO(
      this->get_logger(), "Comm Delay Inflation Alpha: %f", par_.comm_delay_inflation_alpha);
  RCLCPP_INFO(this->get_logger(), "Comm Delay Inflation Max: %f", par_.comm_delay_inflation_max);
  RCLCPP_INFO(this->get_logger(), "Comm Delay Filter Alpha: %f", par_.comm_delay_filter_alpha);

  // Simulation parameters
  RCLCPP_INFO(this->get_logger(), "D435 Depth Max: %f", par_.depth_camera_depth_max);
  RCLCPP_INFO(this->get_logger(), "FOV Visual Depth: %f", par_.fov_visual_depth);
  RCLCPP_INFO(this->get_logger(), "FOV Visual X Deg: %f", par_.fov_visual_x_deg);
  RCLCPP_INFO(this->get_logger(), "FOV Visual Y Deg: %f", par_.fov_visual_y_deg);

  // Optimization parameters
  RCLCPP_INFO(this->get_logger(), "Horizon: %f", par_.horizon);
  RCLCPP_INFO(this->get_logger(), "DC: %f", par_.dc);
  RCLCPP_INFO(this->get_logger(), "V Max: %f", par_.v_max);
  RCLCPP_INFO(this->get_logger(), "A Max: %f", par_.a_max);
  RCLCPP_INFO(this->get_logger(), "J Max: %f", par_.j_max);
  RCLCPP_INFO(this->get_logger(), "Verbose Computation Time: %d", verbose_computation_time_);
  RCLCPP_INFO(this->get_logger(), "Local Traj Comp Verbose: %d", local_traj_comp_verbose_);
  RCLCPP_INFO(
      this->get_logger(), "Drone Bbox: (%f, %f, %f)", par_.drone_bbox[0], par_.drone_bbox[1],
      par_.drone_bbox[2]);
  RCLCPP_INFO(this->get_logger(), "Goal Radius: %f", par_.goal_radius);
  RCLCPP_INFO(this->get_logger(), "Goal Seen Radius: %f", par_.goal_seen_radius);

  // SANDO specific parameters
  RCLCPP_INFO(this->get_logger(), "Num P: %d", par_.num_P);
  RCLCPP_INFO(this->get_logger(), "Num N: %d", par_.num_N);
  RCLCPP_INFO(this->get_logger(), "Use Dynamic Factor: %d", par_.use_dynamic_factor);
  RCLCPP_INFO(this->get_logger(), "Dynamic Factor K Radius: %f", par_.dynamic_factor_k_radius);
  RCLCPP_INFO(
      this->get_logger(), "Dynamic Factor Initial Mean: %f", par_.dynamic_factor_initial_mean);
  RCLCPP_INFO(this->get_logger(), "Factor Initial: %f", par_.factor_initial);
  RCLCPP_INFO(this->get_logger(), "Factor Final: %f", par_.factor_final);
  RCLCPP_INFO(this->get_logger(), "Factor Constant Step Size: %f", par_.factor_constant_step_size);
  RCLCPP_INFO(this->get_logger(), "Obst Max Vel: %f", par_.obst_max_vel);
  RCLCPP_INFO(this->get_logger(), "Obst Position Error: %f", par_.obst_position_error);
  RCLCPP_INFO(this->get_logger(), "Max Gurobi Comp Time Sec: %f", par_.max_gurobi_comp_time_sec);
  RCLCPP_INFO(this->get_logger(), "Jerk Smooth Weight: %f", par_.jerk_smooth_weight);

  // Dynamic obstacles parameters
  RCLCPP_INFO(this->get_logger(), "Traj Lifetime: %f", par_.traj_lifetime);

  // Dynamic k_value parameters
  RCLCPP_INFO(
      this->get_logger(), "Num Replanning Before Adapt: %d", par_.num_replanning_before_adapt);
  RCLCPP_INFO(this->get_logger(), "Default K Value End: %d", par_.default_k_value);
  RCLCPP_INFO(this->get_logger(), "Alpha K Value: %f", par_.alpha_k_value_filtering);
  RCLCPP_INFO(this->get_logger(), "K Value Inflation: %f", par_.k_value_factor);

  // Yaw-related parameters
  RCLCPP_INFO(this->get_logger(), "Alpha Filter Dyaw: %f", par_.alpha_filter_dyaw);
  RCLCPP_INFO(this->get_logger(), "W Max: %f", par_.w_max);
  RCLCPP_INFO(this->get_logger(), "Yaw Spinning Threshold: %d", par_.yaw_spinning_threshold);
  RCLCPP_INFO(this->get_logger(), "Yaw Spinning Dyaw: %f", par_.yaw_spinning_dyaw);

  // Simulation env parameters
  RCLCPP_INFO(this->get_logger(), "Force Goal Z: %d", par_.force_goal_z);
  RCLCPP_INFO(this->get_logger(), "Default Goal Z: %f", par_.default_goal_z);

  // Debug flag
  RCLCPP_INFO(this->get_logger(), "Debug Verbose: %d", par_.debug_verbose);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::cleanUpOldTrajsCallback() {
  // Get current time
  double current_time = this->now().seconds();

  // Clean up old trajs
  sando_ptr_->cleanUpOldTrajs(current_time);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::trajCallback(const dynus_interfaces::msg::DynTraj::SharedPtr msg) {
  // Filter out its own traj
  if (msg->id == id_) return;

  // Get current time
  double current_time = this->now().seconds();

  // Get DynTraj from the message
  auto traj = std::make_shared<DynTraj>();
  convertDynTrajMsg2DynTraj(*msg, traj, current_time);

  // Pass the DynTraj to sando.cpp
  sando_ptr_->addTraj(traj, current_time);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::stateCallback(const dynus_interfaces::msg::State::SharedPtr msg) {
  if (par_.use_state_update) {
    RobotState current_state;
    current_state.setPos(msg->pos.x, msg->pos.y, msg->pos.z);
    current_state.setVel(msg->vel.x, msg->vel.y, msg->vel.z);
    current_state.setAccel(0.0, 0.0, 0.0);
    double roll, pitch, yaw;
    quaternion2Euler(msg->quat, roll, pitch, yaw);
    current_state.setYaw(yaw);
    sando_ptr_->updateState(current_state);

    // publish the state
    publishCurrentState(current_state);

    // Publish the velocity in text
    if (par_.visual_level >= 1) publishVelocityInText(current_state.pos, current_state.vel.norm());
  }

  if (!state_initialized_) {
    // If we don't use state update, we need to initialize the state
    if (!par_.use_state_update) {
      RobotState current_state;
      current_state.setPos(msg->pos.x, msg->pos.y, msg->pos.z);
      current_state.setVel(msg->vel.x, msg->vel.y, msg->vel.z);
      current_state.setAccel(0.0, 0.0, 0.0);
      double roll, pitch, yaw;
      quaternion2Euler(msg->quat, roll, pitch, yaw);
      current_state.setYaw(yaw);
      current_state.t = this->now().seconds();
      sando_ptr_->updateState(current_state);
    }
    RCLCPP_INFO(this->get_logger(), "State initialized");
    state_initialized_ = true;
    timer_goal_->reset();
  }

  if (par_.visual_level >= 1) publishActualTraj();
}

// ----------------------------------------------------------------------------

void SANDO_NODE::replanCallback() {
  // Get the current time as double
  double current_time = this->now().seconds();

  // Set computation times to zero
  setComputationTimesToZero();

  // Replan
  auto [replanning_result, hgp_result] =
      sando_ptr_->replan(replanning_computation_time_, current_time);

  // Get computation time (used to find point A) - note this value is not updated in the replan
  // function
  if (replanning_result) {
    // Get the replanning computation time
    replanning_computation_time_ = this->now().seconds() - current_time;
    if (par_.debug_verbose)
      printf("Total Replanning: %f ms\n", replanning_computation_time_ * 1000.0);
  }

  // To share trajectory with other agents
  if (replanning_result) publishOwnTraj();

  // For visualization of global path
  if (hgp_result && par_.visual_level >= 1) publishGlobalPath();

  // For visualization of free global path
  if (hgp_result && par_.visual_level >= 1) publishFreeGlobalPath();

  // For visualization of local_global_path and local_global_path_after_push_
  if (hgp_result && par_.visual_level >= 1) publishLocalGlobalPath();

  if (hgp_result && par_.visual_level >= 2) {
    publishDynamicHeatCloud();
    publishOccupiedCloud();
  }

  // For visualization of the local trajectory
  if (replanning_result && par_.visual_level >= 1) publishTraj();

  // For visualization of the safe corridor
  if (hgp_result && par_.visual_level >= 1) publishPoly();

  // For visualization of point G and point A
  if (replanning_result && par_.visual_level >= 1) {
    publishPointG();
    publishPointE();
    publishPointA();
  }

  // For visualization of control points
  if (replanning_result && par_.visual_level >= 1) publisCps();

  // For visualization of static push points and P points
  if (replanning_result && par_.visual_level >= 1) {
    sando_ptr_->getStaticPushPoints(static_push_points_);
    publishStaticPushPoints();
  }

  // Always retrieve data so we can publish computation times
  retrieveData();

  // Publish computation times topic
  publishComputationTimes(replanning_result);

  // Verbose computation time to the terminal
  if (verbose_computation_time_) printComputationTime(replanning_result);

  // Verbose only local trajectory computation time
  if (local_traj_comp_verbose_ && !verbose_computation_time_)
    std::cout << "Local Traj Time [ms]: " << local_traj_computation_time_ << std::endl;

  if (use_benchmark_) recordData(replanning_result);

  // Usually this is done is goal callback but becuase we don't call that in push path test, we need
  // to call it here
  if (par_.use_path_push_for_visualization) publishFOV();
}

// ----------------------------------------------------------------------------

void SANDO_NODE::terminalGoalCallback(const geometry_msgs::msg::PoseStamped& msg) {
  // Set the terminal goal
  RobotState G_term;
  double goal_z;

  // If force_goal_z is true, set the goal_z to default_goal_z
  if (par_.force_goal_z)
    goal_z = par_.default_goal_z;
  else
    goal_z = msg.pose.position.z;

  // Check if the goal_z is within the limits
  if (goal_z < par_.z_min || goal_z > par_.z_max) {
    RCLCPP_ERROR(this->get_logger(), "Goal z is out of bounds: %f", goal_z);
    return;
  }

  // Set the terminal goal
  G_term.setPos(msg.pose.position.x, msg.pose.position.y, goal_z);

  // Update the terminal goal
  sando_ptr_->setTerminalGoal(G_term);

  // Publish the term goal for visualization
  publishState(G_term, pub_point_G_term_);

  // Start replanning
  timer_replanning_->reset();
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishVelocityInText(const Eigen::Vector3d& position, double velocity) {
  // Set velocity's precision to 2 decimal points
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << velocity << "m/s";
  std::string text = oss.str();

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = viz_frame_;
  marker.header.stamp = this->get_clock()->now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.ns = "velocity";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.text = text;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z() + 5.0;
  marker.pose.orientation.w = 1.0;
  pub_vel_text_->publish(marker);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::goalReachedCheckCallback() {
  if (sando_ptr_->goalReachedCheck()) {
    logData();
    pub_goal_reached_->publish(std_msgs::msg::Empty());
  }
}

// ----------------------------------------------------------------------------

void SANDO_NODE::getInitialPoseHwCallback() {
  // First find the transformation matrix from map to camera
  try {
    init_pose_transform_stamped_ =
        tf2_buffer_->lookupTransform("map", initial_pose_topic_, tf2::TimePointZero);

    // Print out the initial pose
    RCLCPP_INFO(
        this->get_logger(), "Initial pose received: (%f, %f, %f)",
        init_pose_transform_stamped_.transform.translation.x,
        init_pose_transform_stamped_.transform.translation.y,
        init_pose_transform_stamped_.transform.translation.z);

    // Push the initial pose to sando
    sando_ptr_->setInitialPose(init_pose_transform_stamped_);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  // flag
  if (!initial_pose_received_) {
    initial_pose_received_ = true;
    timer_initial_pose_->cancel();
  }
}

// ----------------------------------------------------------------------------

void SANDO_NODE::convertDynTrajMsg2DynTraj(
    const dynus_interfaces::msg::DynTraj& msg,
    std::shared_ptr<DynTraj>& traj,
    double current_time) {
  // Inflate bbox using drone_bbox
  // We need to use the obstacle's bbox as well as ego drone's bbox
  traj->bbox << msg.bbox[0] / 2.0 + par_.drone_bbox[0] / 2.0,
      msg.bbox[1] / 2.0 + par_.drone_bbox[1] / 2.0, msg.bbox[2] / 2.0 + par_.drone_bbox[2] / 2.0;

  // Get id
  traj->id = msg.id;

  // Check if we should skip future trajectory information (for fair comparison)
  bool skip_future_traj = par_.use_only_curr_pos_for_dynamic_obst && !msg.is_agent;

  // Get pwp (skip if only using current position for obstacles)
  if (!skip_future_traj) {
    traj->pwp = sando_utils::convertPwpMsg2Pwp(msg.pwp);
    traj->mode = DynTraj::Mode::Piecewise;  // default to PWP; overridden below if analytic compiles
  } else {
    // Create stationary trajectory at current position for fair comparison
    // Use current position from msg.pos and create zero velocity trajectory
    traj->traj_x = std::to_string(msg.pos.x);
    traj->traj_y = std::to_string(msg.pos.y);
    traj->traj_z = std::to_string(msg.pos.z);
    traj->traj_vx = "0.0";
    traj->traj_vy = "0.0";
    traj->traj_vz = "0.0";

    // Compile as stationary point
    if (traj->compileAnalytic()) {
      traj->mode = DynTraj::Mode::Analytic;
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "Failed to compile stationary trajectory for obstacle id=%d at pos=[%.2f, %.2f, %.2f]",
          msg.id, msg.pos.x, msg.pos.y, msg.pos.z);
    }
  }

  // Get covariances (skip if only using current position for obstacles)
  if (!msg.is_agent && !skip_future_traj) {
    traj->ekf_cov_p = sando_utils::convertCovMsg2Cov(msg.ekf_cov_p);  // ekf cov
    traj->ekf_cov_q = sando_utils::convertCovMsg2Cov(msg.ekf_cov_q);  // ekf cov
    traj->poly_cov = sando_utils::convertCovMsg2Cov(msg.poly_cov);    // future traj cov
  }

  // Get analytical functions (skip if only using current position for obstacles)
  if (!skip_future_traj) {
    if (msg.function.size() == 3) {
      traj->traj_x = msg.function[0];
      traj->traj_y = msg.function[1];
      traj->traj_z = msg.function[2];
    }

    if (msg.velocity.size() == 3) {
      traj->traj_vx = msg.velocity[0];
      traj->traj_vy = msg.velocity[1];
      traj->traj_vz = msg.velocity[2];
    }

    if (msg.function.size() == 3 && msg.velocity.size() == 3) {
      if (traj->compileAnalytic()) {
        // Change the mode only when we successfully compiled the analytic trajectory
        traj->mode = DynTraj::Mode::Analytic;
      } else {
        RCLCPP_ERROR(
            this->get_logger(), "Failed to compile analytic traj id=%d, falling back to zeros.",
            traj->id);
        // leave mode as whatever it was (Piecewise/Quintic),
        // or explicitly set a safe default here
      }
    }
  }

  // Record received time
  traj->time_received = current_time;

  // Store actual current position from message
  traj->current_pos << msg.pos.x, msg.pos.y, msg.pos.z;

  // Get is_agent
  traj->is_agent = msg.is_agent;

  // Get terminal goal
  if (traj->is_agent) traj->goal << msg.goal[0], msg.goal[1], msg.goal[2];

  // Get communication delay
  if (traj->is_agent && par_.use_comm_delay_inflation) {
    // Get the delay (current time - msg time)
    traj->communication_delay =
        this->now().seconds() - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);

    // Sanity check - if the delay is negative, set it to 0 - send warning message: it's probably
    // due to the clock synchronization issue
    if (traj->communication_delay < 0) {
      traj->communication_delay = 0;
      RCLCPP_WARN(this->get_logger(), "Communication delay is negative. Setting it to 0.");
    }
  }
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publisCps() {
  // Retrieve control points
  sando_ptr_->retrieveCPs(cps_);

  // Create a marker array
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(cps_.size());

  // Loop through the control points (std::vector<Eigen::Matrix<double, 3, 4>>)
  const size_t num_segments = cps_.size();
  for (size_t seg = 0; seg < num_segments; ++seg) {
    // Create a marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = viz_frame_;
    marker.header.stamp = this->now();
    marker.ns = "cp";
    marker.id = seg;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;

    // Set different colors for different segments
    if (seg % 3 == 0) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else if (seg % 3 == 1) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }

    auto cp = cps_[seg];

    // Loop through the control points for each segment
    for (int i = 0; i < cp.cols(); i++) {
      geometry_msgs::msg::Point point;
      point.x = cp(0, i);
      point.y = cp(1, i);
      point.z = cp(2, i);
      marker.points.push_back(point);
    }

    // Add the marker to the marker array
    marker_array.markers[seg] = marker;
  }

  // Publish
  pub_cp_->publish(marker_array);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishStaticPushPoints() {
  // Create a marker array
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = viz_frame_;
  marker.header.stamp = this->now();
  marker.ns = "static_push_points";
  marker.id = static_push_points_id_;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 3.0;
  marker.scale.y = 3.0;
  marker.scale.z = 3.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.925;
  marker.color.b = 1.0;

  // Loop through the static push points
  const size_t num_points = static_push_points_.size();
  for (size_t idx = 0; idx < num_points; ++idx) {
    geometry_msgs::msg::Point point;
    point.x = static_push_points_[idx](0);
    point.y = static_push_points_[idx](1);
    point.z = static_push_points_[idx](2);
    marker.points.push_back(point);
  }

  // Add the single marker to the marker array
  marker_array.markers.push_back(marker);

  // Publish
  pub_static_push_points_->publish(marker_array);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::setComputationTimesToZero() {
  final_g_ = 0.0;
  global_planning_time_ = 0.0;
  hgp_static_jps_time_ = 0.0;
  hgp_check_path_time_ = 0.0;
  hgp_dynamic_astar_time_ = 0.0;
  hgp_recover_path_time_ = 0.0;
  cvx_decomp_time_ = 0.0;
  local_traj_computation_time_ = 0.0;
  safe_paths_time_ = 0.0;
  safety_check_time_ = 0.0;
  yaw_sequence_time_ = 0.0;
  yaw_fitting_time_ = 0.0;
}

// ----------------------------------------------------------------------------

void SANDO_NODE::retrieveData() {
  sando_ptr_->retrieveData(
      final_g_, global_planning_time_, hgp_static_jps_time_, hgp_check_path_time_,
      hgp_dynamic_astar_time_, hgp_recover_path_time_, cvx_decomp_time_,
      local_traj_computation_time_, safety_check_time_, safe_paths_time_, yaw_sequence_time_,
      yaw_fitting_time_, successful_factor_);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::printComputationTime(bool result) {
  // Print the computation times
  RCLCPP_INFO(this->get_logger(), "Planner: %s", par_.global_planner.c_str());
  RCLCPP_INFO(this->get_logger(), "Result: %d", result);
  RCLCPP_INFO(this->get_logger(), "Cost (final node's g): %f", final_g_);
  RCLCPP_INFO(
      this->get_logger(), "Total replanning time [ms]: %f", replanning_computation_time_ * 1000.0);
  RCLCPP_INFO(this->get_logger(), "Global Planning Time [ms]: %f", global_planning_time_);
  RCLCPP_INFO(this->get_logger(), "CVX Decomposition Time [ms]: %f", cvx_decomp_time_);
  RCLCPP_INFO(this->get_logger(), "Local Traj Time [ms]: %f", local_traj_computation_time_);
  RCLCPP_INFO(this->get_logger(), "Safe Paths Time [ms]: %f", safe_paths_time_);
  RCLCPP_INFO(this->get_logger(), "Safety Check Time [ms]: %f", safety_check_time_);
  RCLCPP_INFO(this->get_logger(), "Yaw Sequence Time [ms]: %f", yaw_sequence_time_);
  RCLCPP_INFO(this->get_logger(), "Yaw Fitting Time [ms]: %f", yaw_fitting_time_);
  RCLCPP_INFO(this->get_logger(), "------------------------");
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishComputationTimes(bool result) {
  dynus_interfaces::msg::ComputationTimes msg;
  msg.header.stamp = this->now();
  msg.result = result;
  msg.successful_factor = successful_factor_;
  msg.total_replanning_ms = replanning_computation_time_ * 1000.0;
  msg.global_planning_ms = global_planning_time_;
  msg.hgp_static_jps_ms = hgp_static_jps_time_;
  msg.hgp_check_path_ms = hgp_check_path_time_;
  msg.hgp_dynamic_astar_ms = hgp_dynamic_astar_time_;
  msg.hgp_recover_path_ms = hgp_recover_path_time_;
  msg.cvx_decomp_ms = cvx_decomp_time_;
  msg.local_traj_ms = local_traj_computation_time_;
  msg.safe_paths_ms = safe_paths_time_;
  msg.safety_check_ms = safety_check_time_;
  msg.yaw_sequence_ms = yaw_sequence_time_;
  msg.yaw_fitting_ms = yaw_fitting_time_;
  pub_computation_times_->publish(msg);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::recordData(bool result) {
  // Record all the data into global_path_benchmark_
  std::tuple<
      bool, double, double, double, double, double, double, double, double, double, double, double,
      double, double>
      data;

  data = std::make_tuple(
      result, final_g_, replanning_computation_time_, global_planning_time_, cvx_decomp_time_,
      local_traj_computation_time_, safe_paths_time_, safety_check_time_, yaw_sequence_time_,
      yaw_fitting_time_, 0.0, 0.0, 0.0, 0.0);

  global_path_benchmark_.push_back(data);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::logData() {
  std::ofstream log_file(file_path_);  // Open the file in overwrite mode
  if (log_file.is_open()) {
    // Header
    log_file << "Planner,Result,Cost (final node's g),Total replanning time [ms],Global Planning "
                "Time [ms],CVX Decomposition Time [ms],Local Traj Time [ms],Safe Paths Time "
                "[ms],Safety Check Time [ms],Yaw Sequence Time [ms],Yaw Fitting Time [ms]\n";

    // Data
    for (const auto& row : global_path_benchmark_) {
      log_file << par_.global_planner << "," << std::get<0>(row) << "," << std::get<1>(row) << ","
               << std::get<2>(row) * 1000.0 << "," << std::get<3>(row) << "," << std::get<4>(row)
               << "," << std::get<5>(row) << "," << std::get<6>(row) << "," << std::get<7>(row)
               << "," << std::get<8>(row) << "," << std::get<9>(row) << "\n";
    }

    log_file.close();
  }
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishPointG() const {
  // get projected goal (G)
  RobotState G;
  sando_ptr_->getG(G);

  // Publish the goal for visualization
  publishState(G, pub_point_G_);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishPointE() const {
  // get projected goal (E)
  RobotState E;
  sando_ptr_->getE(E);

  // Publish the goal for visualization
  publishState(E, pub_point_E_);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishPointA() const {
  // get projected goal (A)
  RobotState A;
  sando_ptr_->getA(A);

  // Publish the goal for visualization
  publishState(A, pub_point_A_);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishCurrentState(const RobotState& state) const {
  // Publish the goal for visualization
  publishState(state, pub_current_state_);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishState(
    const RobotState& data,
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr& publisher) const {
  geometry_msgs::msg::PointStamped p;
  p.header.frame_id = viz_frame_;
  p.header.stamp = this->now();
  p.point = eigen2point(data.pos);
  publisher->publish(p);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishOwnTraj() {
  // Get the piecewise polynomial trajectory to share
  sando_ptr_->getPieceWisePol(pwp_to_share_);

  // Create the message
  dynus_interfaces::msg::DynTraj msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = viz_frame_;
  msg.bbox.push_back(par_.drone_bbox[0]);
  msg.bbox.push_back(par_.drone_bbox[1]);
  msg.bbox.push_back(par_.drone_bbox[2]);
  msg.id = id_;
  msg.mode = "pwp";
  msg.pwp = sando_utils::convertPwp2PwpMsg(pwp_to_share_);
  msg.is_agent = true;

  // Set current position so other agents can track this agent's actual location
  RobotState current_state;
  sando_ptr_->getState(current_state);
  msg.pos.x = current_state.pos.x();
  msg.pos.y = current_state.pos.y();
  msg.pos.z = current_state.pos.z();

  // Get the terminal goal
  RobotState G;
  sando_ptr_->getG(G);
  msg.goal.push_back(G.pos(0));
  msg.goal.push_back(G.pos(1));
  msg.goal.push_back(G.pos(2));

  // Publish the trajectory
  pub_own_traj_->publish(msg);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishActualTraj() {
  if (!pub_actual_traj_) return;

  // Get current state
  RobotState current_state;
  sando_ptr_->getState(current_state);
  const Eigen::Vector3d current_pos = current_state.pos;

  // If state not initialized yet
  if (current_pos.norm() < 1e-2) return;

  const auto now = this->now();
  const double tnow = now.seconds();

  // Initialize on first valid sample
  if (!actual_traj_initialized_) {
    actual_traj_prev_pos_ = current_pos;
    actual_traj_prev_time_ = tnow;

    // Ensure velocity is reasonable even on first point
    if (par_.vehicle_type != "uav") current_state.vel.setZero();

    actual_traj_hist_.clear();
    actual_traj_hist_.push_back(current_state);

    actual_traj_initialized_ = true;
    return;  // wait for second sample to draw a line
  }

  // Velocity handling:
  // - UAV: assume current_state.vel already valid from estimator/sim
  // - non-UAV: approximate velocity from position difference (TF-based state publisher case)
  if (par_.vehicle_type != "uav") {
    const double dt = tnow - actual_traj_prev_time_;
    if (dt > 1e-3)
      current_state.vel = (current_pos - actual_traj_prev_pos_) / dt;
    else
      current_state.vel.setZero();
  }

  // Update prev for next call (do this *after* computing vel)
  actual_traj_prev_pos_ = current_pos;
  actual_traj_prev_time_ = tnow;

  // Append to history only if it moved enough (optional but helps reduce visual noise)
  // You can tune eps; this prevents dense identical points from clogging the strip.
  const double eps = 1e-3;
  if (!actual_traj_hist_.empty()) {
    const Eigen::Vector3d last_pos = actual_traj_hist_.back().pos;
    if ((current_pos - last_pos).norm() < eps) {
      // Still update the last sample's velocity (so color can reflect speed changes)
      actual_traj_hist_.back().vel = current_state.vel;
    } else {
      actual_traj_hist_.push_back(current_state);
    }
  } else {
    actual_traj_hist_.push_back(current_state);
  }

  // Bound history (prevents RViz lag / “outdated” visuals)
  if (actual_traj_hist_.size() > actual_traj_max_hist_) {
    const size_t overflow = actual_traj_hist_.size() - actual_traj_max_hist_;
    actual_traj_hist_.erase(actual_traj_hist_.begin(), actual_traj_hist_.begin() + overflow);
  }

  // Publish as a single persistent colored LINE_STRIP marker
  // NOTE: par_.v_max is per-axis; for speed magnitude scaling you may prefer sqrt(3)*v_max.
  const double vmax_for_color = par_.v_max;  // or: std::sqrt(3.0) * par_.v_max;

  visualization_msgs::msg::MarkerArray ma;

  ma = stateVector2ColoredLineStripMarkerArray(
      actual_traj_hist_,
      /*id=*/1,
      /*ns=*/"actual_traj_" + id_str_,
      /*max_value=*/vmax_for_color,
      /*stamp=*/now,
      /*line_width=*/actual_traj_line_width_,
      /*max_points_vis=*/actual_traj_max_points_vis_);

  pub_actual_traj_->publish(ma);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishGoal() {
  // On hardware, don't publish until initial pose transform is acquired
  if (par_.use_hardware && !initial_pose_received_) return;

  // On hardware, don't publish goal setpoints when hovering at goal — stops
  // the MAVROS bridge from continuously feeding PX4's position controller,
  // which causes oscillation around the goal position.
  if (par_.use_hardware && sando_ptr_->getDroneStatus() == DroneStatus::GOAL_REACHED) return;

  // Initialize the goal
  RobotState next_goal;

  // Get the next goal
  if (sando_ptr_->getNextGoal(next_goal) && par_.use_state_update) {
    // Publish the goal (actual setpoint)
    dynus_interfaces::msg::Goal quadGoal;
    quadGoal.header.stamp = this->now();
    quadGoal.header.frame_id = viz_frame_;
    quadGoal.p = eigen2rosvector(next_goal.pos);
    quadGoal.v = eigen2rosvector(next_goal.vel);
    quadGoal.a = eigen2rosvector(next_goal.accel);
    quadGoal.j = eigen2rosvector(next_goal.jerk);
    quadGoal.yaw = next_goal.yaw;
    quadGoal.dyaw = next_goal.dyaw;
    pub_goal_->publish(quadGoal);

    // Publish the goal (setpoint) for visualization
    if (par_.visual_level >= 1) publishState(next_goal, pub_setpoint_);
  }

  // Publish FOV
  if (par_.visual_level >= 1) publishFOV();
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishPoly() {
  // retrieve the polyhedra
  sando_ptr_->retrievePolytopes(poly_whole_, poly_safe_);

  // For whole trajectory
  if (!poly_whole_.empty()) {
    decomp_ros_msgs::msg::PolyhedronArray poly_whole_msg =
        DecompROS::polyhedron_array_to_ros(poly_whole_);
    poly_whole_msg.header.stamp = this->now();
    poly_whole_msg.header.frame_id = viz_frame_;
    poly_whole_msg.lifetime = rclcpp::Duration::from_seconds(1.0);
    pub_poly_whole_->publish(poly_whole_msg);
  }

  // For safe trajectory
  if (!poly_safe_.empty()) {
    decomp_ros_msgs::msg::PolyhedronArray poly_safe_msg =
        DecompROS::polyhedron_array_to_ros(poly_safe_);
    poly_safe_msg.header.stamp = this->now();
    poly_safe_msg.header.frame_id = viz_frame_;
    poly_safe_msg.lifetime = rclcpp::Duration::from_seconds(1.0);
    pub_poly_safe_->publish(poly_safe_msg);
  }
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishTraj() {
  auto now = this->now();

  // 1) DELETEALL on both topics
  {
    visualization_msgs::msg::MarkerArray clear_msg;
    visualization_msgs::msg::Marker clear_m;
    clear_m.header.frame_id = viz_frame_;
    clear_m.header.stamp = now;
    clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_msg.markers.push_back(clear_m);

    pub_traj_committed_colored_->publish(clear_msg);
    pub_traj_subopt_colored_->publish(clear_msg);
  }

  // 2) Publish the committed (best) trajectory
  sando_ptr_->retrieveGoalSetpoints(goal_setpoints_);
  {
    auto committed_ma = stateVector2ColoredMarkerArray(
        goal_setpoints_,
        /*type=*/1, par_.v_max, now);
    pub_traj_committed_colored_->publish(committed_ma);
  }

  // 3) Publish all sub-optimal trajectories
  sando_ptr_->retrieveListSubOptGoalSetpoints(list_subopt_goal_setpoints_);
  visualization_msgs::msg::MarkerArray subopt_ma;
  for (int i = 0; i < (int)list_subopt_goal_setpoints_.size(); ++i) {
    auto single = stateVector2ColoredMarkerArray(
        list_subopt_goal_setpoints_[i],
        /*type=*/i + 2, par_.v_max, now);
    // append all markers from this one:
    subopt_ma.markers.insert(subopt_ma.markers.end(), single.markers.begin(), single.markers.end());
  }
  pub_traj_subopt_colored_->publish(subopt_ma);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishGlobalPath() {
  int global_path_color = RED;
  int original_global_path_color = ORANGE;

  // Generate random integer from 1 to 10 to generate random color
  if (par_.use_random_color_for_global_path) global_path_color = rand() % 10 + 1;

  // Get global_path
  vec_Vecf<3> global_path;
  sando_ptr_->getGlobalPath(global_path);

  if (!global_path.empty()) {
    // Publish global_path (thin line + dots)
    clearMarkerArray(hgp_path_marker_, pub_hgp_path_marker_);

    pathLineDotsToMarkerArray(
        global_path, &hgp_path_marker_, color(global_path_color),
        /*line_width=*/0.03,    // meters
        /*dot_diameter=*/0.06,  // meters
        /*base_id=*/50000,
        /*frame_id=*/viz_frame_,
        /*lifetime_sec=*/1.0);

    pub_hgp_path_marker_->publish(hgp_path_marker_);
  }

  // Get the original global path
  vec_Vecf<3> original_global_path;
  sando_ptr_->getOriginalGlobalPath(original_global_path);

  if (!original_global_path.empty()) {
    // Publish original_global_path
    clearMarkerArray(original_hgp_path_marker_, pub_original_hgp_path_marker_);

    pathLineDotsToMarkerArray(
        original_global_path, &original_hgp_path_marker_, color(original_global_path_color),
        /*line_width=*/0.03,    // meters
        /*dot_diameter=*/0.06,  // meters
        /*base_id=*/60000,
        /*frame_id=*/viz_frame_,
        /*lifetime_sec=*/1.0);

    pub_original_hgp_path_marker_->publish(original_hgp_path_marker_);
  }
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishFreeGlobalPath() {
  // Get free_global_path
  vec_Vecf<3> free_global_path;
  sando_ptr_->getFreeGlobalPath(free_global_path);

  if (free_global_path.empty()) return;

  // Publish free_global_path
  clearMarkerArray(hgp_free_path_marker_, pub_free_hgp_path_marker_);
  vectorOfVectors2MarkerArray(free_global_path, &hgp_free_path_marker_, color(GREEN));
  pub_free_hgp_path_marker_->publish(hgp_free_path_marker_);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishLocalGlobalPath() {
  // Get the local global path and local global path after push
  vec_Vecf<3> local_global_path;
  vec_Vecf<3> local_global_path_after_push;
  sando_ptr_->getLocalGlobalPath(local_global_path, local_global_path_after_push);

  if (!local_global_path.empty()) {
    // Publish local_global_path
    clearMarkerArray(hgp_local_global_path_marker_, pub_local_global_path_marker_);
    vectorOfVectors2MarkerArray(local_global_path, &hgp_local_global_path_marker_, color(BLUE));
    pub_local_global_path_marker_->publish(hgp_local_global_path_marker_);
  }

  if (!local_global_path_after_push.empty()) {
    // Publish local_global_path_after_push
    clearMarkerArray(
        hgp_local_global_path_after_push_marker_, pub_local_global_path_after_push_marker_);
    vectorOfVectors2MarkerArray(
        local_global_path_after_push, &hgp_local_global_path_after_push_marker_, color(ORANGE));
    pub_local_global_path_after_push_marker_->publish(hgp_local_global_path_after_push_marker_);
  }
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishDynamicHeatCloud() {
  if (!pub_dynamic_heat_cloud_) return;

  auto map_util = sando_ptr_->getMapUtilSharedPtr();
  if (!map_util) return;

  const bool any_heat = map_util->dynamicHeatEnabled() || map_util->staticHeatEnabled();

  if (!any_heat) return;

  // Use new efficient API - only iterates over voxels with heat > threshold
  const float heat_threshold = 0.05f;
  const vec_Vecf<3> heat_cloud = map_util->getHeatCloud(heat_threshold);

  if (heat_cloud.empty()) return;

  const float max_heat = map_util->getMaxHeat();

  // Build PointCloud2 message
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = viz_frame_;
  msg.header.stamp = this->now();

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(heat_cloud.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");

  for (size_t i = 0; i < heat_cloud.size(); ++i) {
    const Vec3f& pt = heat_cloud[i];
    *iter_x = pt.x();
    *iter_y = pt.y();
    *iter_z = pt.z();

    const Veci<3> idx = map_util->floatToInt(pt);
    const float h = map_util->getHeat(idx.x(), idx.y(), idx.z());
    *iter_i = (max_heat > 1e-6f) ? (h / max_heat) : 0.0f;

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_i;
  }

  pub_dynamic_heat_cloud_->publish(msg);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishOccupiedCloud() {
  if (!pub_occupied_cloud_) return;

  auto map_util = sando_ptr_->getMapUtilSharedPtr();
  if (!map_util) return;

  // -------- Tunables --------
  const int stride = 1;              // 1 = every voxel (we want to see all occupied cells)
  const size_t max_points = 500000;  // hard cap for safety
  // --------------------------

  const auto dim = map_util->getDim();  // Veci<3>
  const int nx = dim(0);
  const int ny = dim(1);
  const int nz = dim(2);

  // Use Eigen-aligned vector type
  vec_Vec3f pts;
  pts.reserve(50000);

  for (int x = 0; x < nx; x += stride) {
    for (int y = 0; y < ny; y += stride) {
      for (int z = 0; z < nz; z += stride) {
        const int idx = map_util->getIndex(Veci<3>(x, y, z));
        if (!map_util->isOccupied(idx)) continue;

        const Vec3f p = map_util->intToFloat(Veci<3>(x, y, z));
        pts.push_back(p);

        if (pts.size() >= max_points) goto BUILD_OCC_MSG;
      }
    }
  }

BUILD_OCC_MSG:
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = viz_frame_;
  msg.header.stamp = this->now();

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

  for (size_t k = 0; k < pts.size(); ++k, ++iter_x, ++iter_y, ++iter_z) {
    const auto& p = pts[k];
    *iter_x = static_cast<float>(p(0));
    *iter_y = static_cast<float>(p(1));
    *iter_z = static_cast<float>(p(2));
  }

  pub_occupied_cloud_->publish(msg);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishHoverAvoidanceViz() {
  visualization_msgs::msg::MarkerArray ma;

  double current_time = this->now().seconds();
  int drone_status = sando_ptr_->getDroneStatus();
  double d_trigger = sando_ptr_->getHoverAvoidanceDTrigger();

  // --- Danger spheres around each obstacle ---
  std::vector<std::shared_ptr<DynTraj>> trajs;
  sando_ptr_->getTrajs(trajs);

  if (!trajs.empty()) {
    static int viz_dbg_count = 0;
  }

  for (size_t i = 0; i < trajs.size(); ++i) {
    // Use the agent's actual reported position (updated each msg) rather than
    // eval(current_time) which can return stale endpoint for expired PWP trajectories.
    Eigen::Vector3d p_obs = trajs[i]->current_pos;

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = viz_frame_;
    sphere.header.stamp = this->now();
    sphere.ns = "danger_sphere";
    sphere.id = static_cast<int>(i);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = p_obs.x();
    sphere.pose.position.y = p_obs.y();
    sphere.pose.position.z = p_obs.z();
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = d_trigger * 2.0;  // diameter
    sphere.scale.y = d_trigger * 2.0;
    sphere.scale.z = d_trigger * 2.0;
    sphere.color.r = 1.0;
    sphere.color.g = 0.0;
    sphere.color.b = 0.0;
    sphere.color.a = 0.12;
    sphere.lifetime = rclcpp::Duration::from_seconds(0.1);
    ma.markers.push_back(sphere);
  }

  // --- Delete stale danger sphere markers ---
  // If there are fewer obstacles than before, delete old markers
  for (size_t i = trajs.size(); i < trajs.size() + 10; ++i) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = viz_frame_;
    del.header.stamp = this->now();
    del.ns = "danger_sphere";
    del.id = static_cast<int>(i);
    del.action = visualization_msgs::msg::Marker::DELETE;
    ma.markers.push_back(del);
  }

  // --- Hover position marker (orange sphere, 0.3m radius) ---
  bool show_hover =
      (drone_status == DroneStatus::HOVER_AVOIDING || drone_status == DroneStatus::GOAL_REACHED);
  Eigen::Vector3d p_hover = sando_ptr_->getHoverPos();

  visualization_msgs::msg::Marker hover_marker;
  hover_marker.header.frame_id = viz_frame_;
  hover_marker.header.stamp = this->now();
  hover_marker.ns = "hover_pos";
  hover_marker.id = 0;
  hover_marker.type = visualization_msgs::msg::Marker::SPHERE;
  hover_marker.pose.orientation.w = 1.0;
  hover_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

  if (show_hover && p_hover.norm() > 1e-9) {
    hover_marker.action = visualization_msgs::msg::Marker::ADD;
    hover_marker.pose.position.x = p_hover.x();
    hover_marker.pose.position.y = p_hover.y();
    hover_marker.pose.position.z = p_hover.z();
    hover_marker.scale.x = 0.4;  // diameter = 2 * 0.2m
    hover_marker.scale.y = 0.4;
    hover_marker.scale.z = 0.4;
    hover_marker.color.r = 1.0;
    hover_marker.color.g = 0.5;
    hover_marker.color.b = 0.0;
    hover_marker.color.a = 0.9;
  } else {
    hover_marker.action = visualization_msgs::msg::Marker::DELETE;
  }
  ma.markers.push_back(hover_marker);

  pub_hover_avoidance_viz_->publish(ma);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::createMarkerArrayFromVec_Vec3f(
    const vec_Vec3f& occupied_cells,
    const std_msgs::msg::ColorRGBA& color,
    int namespace_id,
    double scale,
    visualization_msgs::msg::MarkerArray* marker_array) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = viz_frame_;
  marker.header.stamp = this->now();
  marker.ns = "namespace_" + std::to_string(namespace_id);
  marker.id = 0;
  marker.type =
      visualization_msgs::msg::Marker::CUBE_LIST;  // Each point will be visualized as a cube
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = par_.res;
  marker.scale.y = par_.res;
  marker.scale.z = par_.res;
  marker.color = color;

  for (const auto& cell : occupied_cells) {
    geometry_msgs::msg::Point point;
    point.x = cell(0);
    point.y = cell(1);
    point.z = cell(2);
    marker.points.push_back(point);
  }

  marker_array->markers.push_back(marker);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::clearMarkerArray(
    visualization_msgs::msg::MarkerArray& path_marker,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher) {
  // If the marker array is empty, return
  if (path_marker.markers.size() == 0) return;

  // Clear the marker array
  int id_begin = path_marker.markers[0].id;

  for (int i = 0; i < path_marker.markers.size(); i++) {
    visualization_msgs::msg::Marker m;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::DELETE;
    m.id = i + id_begin;
    path_marker.markers[i] = m;
  }

  publisher->publish(path_marker);
  path_marker.markers.clear();
}

// ----------------------------------------------------------------------------

void SANDO_NODE::constructFOVMarker() {
  marker_fov_.header.stamp = this->now();
  marker_fov_.header.frame_id = d435_depth_frame_id_;
  marker_fov_.ns = "marker_fov";
  marker_fov_.id = marker_fov_id_++;
  marker_fov_.frame_locked = true;
  marker_fov_.type = marker_fov_.LINE_LIST;
  marker_fov_.action = marker_fov_.ADD;
  marker_fov_.pose = sando_utils::identityGeometryMsgsPose();

  double delta_y = par_.fov_visual_depth * fabs(tan((par_.fov_visual_x_deg * M_PI / 180) / 2.0));
  double delta_z = par_.fov_visual_depth * fabs(tan((par_.fov_visual_y_deg * M_PI / 180) / 2.0));

  geometry_msgs::msg::Point v0 = eigen2point(Eigen::Vector3d(0.0, 0.0, 0.0));
  geometry_msgs::msg::Point v1 =
      eigen2point(Eigen::Vector3d(-delta_y, delta_z, par_.fov_visual_depth));
  geometry_msgs::msg::Point v2 =
      eigen2point(Eigen::Vector3d(delta_y, delta_z, par_.fov_visual_depth));
  geometry_msgs::msg::Point v3 =
      eigen2point(Eigen::Vector3d(delta_y, -delta_z, par_.fov_visual_depth));
  geometry_msgs::msg::Point v4 =
      eigen2point(Eigen::Vector3d(-delta_y, -delta_z, par_.fov_visual_depth));

  marker_fov_.points.clear();

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v1);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v2);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v3);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v4);

  // Line
  marker_fov_.points.push_back(v1);
  marker_fov_.points.push_back(v2);

  // Line
  marker_fov_.points.push_back(v2);
  marker_fov_.points.push_back(v3);

  // Line
  marker_fov_.points.push_back(v3);
  marker_fov_.points.push_back(v4);

  // Line
  marker_fov_.points.push_back(v4);
  marker_fov_.points.push_back(v1);

  marker_fov_.scale.x = 0.03;
  marker_fov_.scale.y = 0.00001;
  marker_fov_.scale.z = 0.00001;
  marker_fov_.color.a = 1.0;
  marker_fov_.color.r = 0.0;
  marker_fov_.color.g = 1.0;
  marker_fov_.color.b = 0.0;
}

// ----------------------------------------------------------------------------

void SANDO_NODE::publishFOV() {
  marker_fov_.header.stamp = this->now();
  pub_fov_->publish(marker_fov_);
  return;
}

// ----------------------------------------------------------------------------

void SANDO_NODE::mapCallback(
    const sensor_msgs::msg::PointCloud2::ConstPtr& map_msg,
    const sensor_msgs::msg::PointCloud2::ConstPtr& unk_msg) {
  // use PCL’s own Ptr (boost::shared_ptr)
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*map_msg, *map_pc);

  pcl::PointCloud<pcl::PointXYZ>::Ptr unk_pc(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*unk_msg, *unk_pc);

  sando_ptr_->updateMapPtr(map_pc, unk_pc);
}

// ----------------------------------------------------------------------------

void SANDO_NODE::occupancyMapCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& map_msg) {
  // use PCL’s own Ptr (boost::shared_ptr)
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*map_msg, *map_pc);

  sando_ptr_->updateOccupancyMapPtr(map_pc);

  // If we use global point cloud, we don't need to update the map ever
  if (par_.use_global_pc) {
    // stop the subscription
    sub_fake_sim_occupancy_map_.reset();
  }
}

// ----------------------------------------------------------------------------

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Initialize multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // add node to executor
  auto node = std::make_shared<sando::SANDO_NODE>();
  executor.add_node(node);

  // spin
  executor.spin();

  rclcpp::shutdown();
  return 0;
}