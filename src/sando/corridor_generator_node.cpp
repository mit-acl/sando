/* ----------------------------------------------------------------------------
 * Copyright (c) Anonymous Author
 * Anonymous Institution
 * All Rights Reserved
 * Authors: Anonymous
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

// Your project headers (adjust include paths to match your repo)
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sando/gurobi_solver.hpp>
#include <sando/utils.hpp>
#include "hgp/hgp_manager.hpp"
#include "hgp/termcolor.hpp"
#include "hgp/utils.hpp"
#include "sando/sando_type.hpp"
#include "timer.hpp"
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace sando;

// Type‐aliases
using Vec3 = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;
using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

static Vec3f vec3FromStd(const std::vector<double>& v, const char* name) {
  if (v.size() != 3) throw std::runtime_error(std::string("Expected ") + name + " to have size 3.");
  return Vec3f(v[0], v[1], v[2]);
}

static std::vector<Vec3f> goalsFromFlat(const std::vector<double>& flat) {
  if (flat.empty()) return {};
  if (flat.size() % 3 != 0) throw std::runtime_error("goals_flat must have length multiple of 3.");

  std::vector<Vec3f> goals;
  goals.reserve(flat.size() / 3);
  for (size_t i = 0; i < flat.size(); i += 3)
    goals.emplace_back(flat[i + 0], flat[i + 1], flat[i + 2]);
  return goals;
}

static vec_Vec3f pclToVec3f(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  vec_Vec3f out;
  out.reserve(cloud.size());
  for (const auto& p : cloud.points) out.emplace_back(p.x, p.y, p.z);
  return out;
}

// Cumulative end-time per segment for dynamic-obstacle inflation.
// If you don’t care about dynamic inflation for corridor generation, this is still fine.
static std::vector<double> computeSegEndTimesFromPath(
    const vec_Vecf<3>& path, double nominal_speed_mps, double min_dt = 1e-3) {
  std::vector<double> seg_end_times;
  if (path.size() < 2) return seg_end_times;
  const size_t num_seg = path.size() - 1;

  seg_end_times.reserve(num_seg);

  double t = 0.0;
  const double v = std::max(nominal_speed_mps, 1e-3);

  for (size_t i = 0; i < num_seg; ++i) {
    const double dist = (path[i + 1] - path[i]).norm();
    const double dt = std::max(dist / v, min_dt);
    t += dt;
    seg_end_times.push_back(t);  // cumulative time at end of segment i
  }
  return seg_end_times;
}

// Dependency-free binary serialization for reuse.
// File format (little-endian):
//   magic[8]="MYSCO2\0\0"
//   u32 version=1
//   start(3*double), goal(3*double)
//   u32 num_path_pts; path_pts[num_path_pts](3*double)
//   u32 num_seg; seg_end_times[num_seg](double)
//   for each seg:
//     u32 num_planes
//     A[num_planes][3] (double)
//     b[num_planes] (double)
static void saveCorridorBinary(
    const fs::path& filepath,
    const Vec3f& start,
    const Vec3f& goal,
    const vec_Vecf<3>& path,
    const std::vector<double>& seg_end_times,
    const std::vector<LinearConstraint3D>& l_constraints) {
  std::ofstream ofs(filepath, std::ios::binary);
  if (!ofs) throw std::runtime_error("Failed to open file for writing: " + filepath.string());

  const char magic[8] = {'M', 'Y', 'S', 'C', 'O', '2', '\0', '\0'};
  const uint32_t version = 1;

  auto writeU32 = [&](uint32_t v) { ofs.write(reinterpret_cast<const char*>(&v), sizeof(v)); };
  auto writeD = [&](double v) { ofs.write(reinterpret_cast<const char*>(&v), sizeof(v)); };

  ofs.write(magic, 8);
  writeU32(version);

  // start / goal
  writeD(start.x());
  writeD(start.y());
  writeD(start.z());
  writeD(goal.x());
  writeD(goal.y());
  writeD(goal.z());

  // path
  writeU32(static_cast<uint32_t>(path.size()));
  for (const auto& p : path) {
    writeD(p.x());
    writeD(p.y());
    writeD(p.z());
  }

  // seg_end_times
  const uint32_t num_seg = static_cast<uint32_t>(seg_end_times.size());
  writeU32(num_seg);
  for (double t : seg_end_times) writeD(t);

  // constraints
  if (l_constraints.size() != num_seg)
    throw std::runtime_error("l_constraints size mismatch with seg_end_times.");

  for (uint32_t i = 0; i < num_seg; ++i) {
    const auto& A = l_constraints[i].A_;
    const auto& b = l_constraints[i].b_;

    if (A.cols() != 3) throw std::runtime_error("Constraint A must have 3 columns.");

    if (A.rows() != b.size()) throw std::runtime_error("Constraint A rows must match b size.");

    const uint32_t num_planes = static_cast<uint32_t>(A.rows());
    writeU32(num_planes);

    // A row-major
    for (uint32_t r = 0; r < num_planes; ++r)
      for (int c = 0; c < 3; ++c) writeD(A(r, c));

    // b
    for (uint32_t r = 0; r < num_planes; ++r) writeD(b(r));
  }

  ofs.flush();
}

class CorridorGeneratorNode final : public rclcpp::Node {
 public:
  CorridorGeneratorNode() : Node("corridor_generator_node") {
    declare_parameter<std::string>("map_topic", "/map_generator/global_cloud");
    declare_parameter<std::vector<double>>("start", {-3.0, 0.0, 1.0});

    // Map window to read into VoxelMapUtil (make this cover all your goals for fairness)
    declare_parameter<std::vector<double>>("map_center", {0.0, 0.0, 1.0});
    declare_parameter<double>("wdx", 20.0);
    declare_parameter<double>("wdy", 20.0);
    declare_parameter<double>("wdz", 10.0);

    // Output
    declare_parameter<std::string>("output_dir", "");
    declare_parameter<std::string>("output_prefix", "sfc");

    // Planner / decomp essentials
    declare_parameter<std::string>("global_planner", "sjps");
    declare_parameter<bool>("global_planner_verbose", false);
    declare_parameter<double>("global_planner_heuristic_weight", 1.0);
    declare_parameter<int>("hgp_timeout_duration_ms", 100000);

    // Corridor generation / inflation params
    declare_parameter<double>("res", 0.15);
    declare_parameter<double>("factor_hgp", 1.0);
    declare_parameter<double>("inflation_hgp", 0.45);
    declare_parameter<double>("drone_radius", 0.1);
    declare_parameter<double>("obst_max_vel", 0.0);  // set >0 only if you want dynamic inflation
    declare_parameter<double>("corridor_nominal_speed", 2.0);  // used to compute seg_end_times
    declare_parameter<double>("v_max", 2.0);
    declare_parameter<double>("a_max", 3.0);
    declare_parameter<double>("j_max", 10.0);

    // Decomp tuning
    declare_parameter<std::vector<double>>("sfc_size", {1.5, 1.5, 1.5});
    declare_parameter<bool>("use_shrinked_box", false);
    declare_parameter<double>("shrinked_box_size", 0.2);
    declare_parameter<double>("max_dist_vertexes", 10.0);
    declare_parameter<int>("num_P", 3);  // max polytopes (same as live planner)

    // Static heat map parameters (align with sando.yaml defaults)
    declare_parameter<bool>("static_heat_enabled", true);
    declare_parameter<double>("static_heat_alpha", 1.0);
    declare_parameter<int>("static_heat_p", 2);
    declare_parameter<double>("static_heat_Hmax", 5.0);
    declare_parameter<double>("static_heat_rmax_m", 1.0);
    declare_parameter<double>("static_heat_default_radius_m", 0.5);
    declare_parameter<bool>("static_heat_boundary_only", true);
    declare_parameter<bool>("static_heat_apply_on_unknown", false);
    declare_parameter<bool>("static_heat_exclude_dynamic", true);
    declare_parameter<double>("heat_weight", 10.0);
    declare_parameter<bool>("use_soft_cost_obstacles", false);
    declare_parameter<double>("obstacle_soft_cost", 0.0);

    // Heat kernel parameters (used by astar_heat even for static-only mode)
    declare_parameter<double>("heat_alpha0", 1.0);
    declare_parameter<double>("heat_alpha1", 2.0);
    declare_parameter<int>("heat_p", 2);
    declare_parameter<int>("heat_q", 2);
    declare_parameter<double>("heat_tau_ratio", 0.5);
    declare_parameter<double>("heat_gamma", 0.0);
    declare_parameter<double>("heat_Hmax", 10.0);
    declare_parameter<double>("dyn_base_inflation_m", 0.5);
    declare_parameter<double>("dyn_heat_tube_radius_m", 2.0);

    // Dynamic heat (disabled for benchmark — no dynamic obstacles)
    declare_parameter<bool>("dynamic_heat_enabled", false);
    declare_parameter<bool>("dynamic_as_occupied_current", false);
    declare_parameter<bool>("dynamic_as_occupied_future", false);

    // Map bounds for VoxelMapUtil ctor
    declare_parameter<double>("x_min", -100.0);
    declare_parameter<double>("x_max", 100.0);
    declare_parameter<double>("y_min", -100.0);
    declare_parameter<double>("y_max", 100.0);
    declare_parameter<double>("z_min", 0.0);
    declare_parameter<double>("z_max", 2.0);

    // For publisher
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("poly_topic", "/NX01/poly_safe");
    declare_parameter<std::string>("path_topic", "/NX01/hgp_path_marker");
    declare_parameter<bool>("publish_ellipsoids", false);
    declare_parameter<std::string>("ellip_topic", "/sando/sfc_ellip");
    declare_parameter<bool>("keep_alive", true);             // keep node running for RViz
    declare_parameter<double>("republish_period_sec", 1.0);  // 0 disables periodic republish

    // Read parameters
    map_topic_ = get_parameter("map_topic").as_string();
    output_dir_ = get_parameter("output_dir").as_string();
    output_prefix_ = get_parameter("output_prefix").as_string();

    start_ = vec3FromStd(get_parameter("start").as_double_array(), "start");
    goals_.clear();
    for (double y = -3.0; y <= 3.0 + 1e-3; y += 0.1) {
      goals_.emplace_back(3.5, y, 1.0);
    }
    if (goals_.empty()) {
      RCLCPP_WARN(get_logger(), "No goals provided (goals_flat is empty). Node will idle.");
    }

    map_center_ = vec3FromStd(get_parameter("map_center").as_double_array(), "map_center");
    wdx_ = get_parameter("wdx").as_double();
    wdy_ = get_parameter("wdy").as_double();
    wdz_ = get_parameter("wdz").as_double();

    corridor_nominal_speed_ = get_parameter("corridor_nominal_speed").as_double();

    // Fill the subset of `parameters` that HGPManager and cvxEllipsoidDecomp use.
    par_.res = get_parameter("res").as_double();
    par_.factor_hgp = get_parameter("factor_hgp").as_double();
    par_.inflation_hgp = get_parameter("inflation_hgp").as_double();
    par_.drone_radius = get_parameter("drone_radius").as_double();
    par_.obst_max_vel = get_parameter("obst_max_vel").as_double();

    par_.sfc_size =
        get_parameter("sfc_size").as_double_array();  // should be vector<double> in your struct
    par_.use_shrinked_box = get_parameter("use_shrinked_box").as_bool();
    par_.shrinked_box_size = get_parameter("shrinked_box_size").as_double();
    par_.max_dist_vertexes = get_parameter("max_dist_vertexes").as_double();
    num_P_ = get_parameter("num_P").as_int();

    par_.x_min = get_parameter("x_min").as_double();
    par_.x_max = get_parameter("x_max").as_double();
    par_.y_min = get_parameter("y_min").as_double();
    par_.y_max = get_parameter("y_max").as_double();
    par_.z_min = get_parameter("z_min").as_double();
    par_.z_max = get_parameter("z_max").as_double();

    global_planner_ = get_parameter("global_planner").as_string();
    global_planner_verbose_ = get_parameter("global_planner_verbose").as_bool();
    weight_ = get_parameter("global_planner_heuristic_weight").as_double();
    hgp_timeout_ms_ = get_parameter("hgp_timeout_duration_ms").as_int();

    v_max_ = get_parameter("v_max").as_double();
    a_max_ = get_parameter("a_max").as_double();
    j_max_ = get_parameter("j_max").as_double();

    // Static heat map parameters
    par_.static_heat_enabled = get_parameter("static_heat_enabled").as_bool();
    par_.static_heat_alpha = get_parameter("static_heat_alpha").as_double();
    par_.static_heat_p = get_parameter("static_heat_p").as_int();
    par_.static_heat_Hmax = get_parameter("static_heat_Hmax").as_double();
    par_.static_heat_rmax_m = get_parameter("static_heat_rmax_m").as_double();
    par_.static_heat_default_radius_m = get_parameter("static_heat_default_radius_m").as_double();
    par_.static_heat_boundary_only = get_parameter("static_heat_boundary_only").as_bool();
    par_.static_heat_apply_on_unknown = get_parameter("static_heat_apply_on_unknown").as_bool();
    par_.static_heat_exclude_dynamic = get_parameter("static_heat_exclude_dynamic").as_bool();
    par_.heat_weight = get_parameter("heat_weight").as_double();
    par_.use_soft_cost_obstacles = get_parameter("use_soft_cost_obstacles").as_bool();
    par_.obstacle_soft_cost = get_parameter("obstacle_soft_cost").as_double();

    // Heat kernel parameters
    par_.heat_alpha0 = get_parameter("heat_alpha0").as_double();
    par_.heat_alpha1 = get_parameter("heat_alpha1").as_double();
    par_.heat_p = get_parameter("heat_p").as_int();
    par_.heat_q = get_parameter("heat_q").as_int();
    par_.heat_tau_ratio = get_parameter("heat_tau_ratio").as_double();
    par_.heat_gamma = get_parameter("heat_gamma").as_double();
    par_.heat_Hmax = get_parameter("heat_Hmax").as_double();
    par_.dyn_base_inflation_m = get_parameter("dyn_base_inflation_m").as_double();
    par_.dyn_heat_tube_radius_m = get_parameter("dyn_heat_tube_radius_m").as_double();

    // Dynamic heat (disabled for benchmark)
    par_.dynamic_heat_enabled = get_parameter("dynamic_heat_enabled").as_bool();
    par_.dynamic_as_occupied_current = get_parameter("dynamic_as_occupied_current").as_bool();
    par_.dynamic_as_occupied_future = get_parameter("dynamic_as_occupied_future").as_bool();

    // Init HGPManager with parameters
    hgp_.setParameters(par_);

    // Subscribe to point cloud
    sub_map_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        map_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
        std::bind(&CorridorGeneratorNode::mapCb, this, std::placeholders::_1));

    // Periodic trigger to run once map is ready
    timer_ = create_wall_timer(200ms, std::bind(&CorridorGeneratorNode::tick, this));

    // For publisher
    frame_id_ = get_parameter("frame_id").as_string();
    poly_topic_ = get_parameter("poly_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    keep_alive_ = get_parameter("keep_alive").as_bool();
    republish_period_sec_ = get_parameter("republish_period_sec").as_double();

    auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    pub_poly_ = create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(poly_topic_, qos_latched);
    pub_path_ = create_publisher<visualization_msgs::msg::MarkerArray>(path_topic_, qos_latched);

    if (republish_period_sec_ > 1e-6) {
      repub_timer_ = create_wall_timer(
          std::chrono::duration<double>(republish_period_sec_),
          std::bind(&CorridorGeneratorNode::republishCachedMsgs, this));
    }
  }

 private:
  void mapCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_cloud_ = cloud;
      map_received_ = true;
    }

    // Update HGP voxel map with a fixed window for fairness
    // Dynamic obstacles: pass empty here unless you explicitly want inflation-in-map-update.
    vec_Vecf<3> obst_pos_empty;
    vec_Vecf<3> obst_bbox_empty;  // Empty bbox vector (no dynamic obstacles)
    const double traj_max_time = 0.0;

    // Create an empty point cloud for unknowns
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unk(new pcl::PointCloud<pcl::PointXYZ>());

    hgp_.updateMap(
        wdx_, wdy_, wdz_, map_center_, cloud, cloud_unk, obst_pos_empty, obst_bbox_empty,
        traj_max_time);

    // Also store occupied vector for decomp obstacle set
    hgp_.updateVecOccupied(pclToVec3f(*cloud));
  }

  void tick() {
    if (done_) return;
    if (!map_received_) return;
    if (goals_.empty()) return;
    if (!hgp_.isMapInitialized()) return;

    try {
      runAllGoalsOnce();
      done_ = true;
      RCLCPP_INFO(get_logger(), "All corridors generated.");

      timer_->cancel();  // stop calling tick()

      if (!keep_alive_) {
        RCLCPP_INFO(get_logger(), "keep_alive:=false -> shutting down.");
        rclcpp::shutdown();
      } else {
        RCLCPP_INFO(get_logger(), "keep_alive:=true -> staying alive for RViz.");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed: %s", e.what());
      rclcpp::shutdown();
    }
  }

  void runAllGoalsOnce() {
    fs::create_directories(output_dir_);

    // Setup planner snapshot (locks current map_util_ into map_util_for_planning_)
    hgp_.setupHGPPlanner(
        global_planner_, global_planner_verbose_, par_.res, v_max_, a_max_, j_max_, hgp_timeout_ms_,
        /*max_num_expansion*/ 10000,
        /*w_unknown*/ 0.0, /*w_align*/ 0.0, /*decay_len_cells*/ 100.0, /*w_side*/ 0.0,
        /*los_cells*/ 0, /*min_len*/ 1.0, /*min_turn*/ 0.0);

    // Base obstacle set for decomposition: occupied (or unknown+occupied if you maintain it)
    vec_Vec3f base_uo;
    hgp_.getVecOccupied(base_uo);

    for (size_t gi = 0; gi < goals_.size(); ++gi) {
      const Vec3f goal = goals_[gi];

      // Direction hint (magnitude irrelevant; used as direction)
      Vec3f dir = goal - start_;
      if (dir.norm() > 1e-6)
        dir = dir / dir.norm();
      else
        dir = Vec3f(1.0, 0.0, 0.0);

      double final_g = 0.0;
      vec_Vecf<3> path;
      vec_Vecf<3> raw_path;

      const double tnow = now().seconds();
      const bool ok = hgp_.solveHGP(start_, dir, goal, final_g, weight_, tnow, path, raw_path);

      if (!ok || path.size() < 2) {
        RCLCPP_WARN(get_logger(), "Goal %zu: global planner failed. Skipping.", gi);
        continue;
      }

      // Snap endpoints to exact start/goal (undo voxel discretization)
      path.front() = start_;
      path.back() = goal;

      // Trim global path to at most (num_P + 1) waypoints, but always
      // keep the goal as the last point so the corridor reaches it.
      if (num_P_ > 0 && path.size() > static_cast<size_t>(num_P_ + 1)) {
        path.resize(num_P_ + 1);
        path.back() = goal;
      }

      // Remove any interior waypoint whose adjacent segment is shorter
      // than 2m. This eliminates tiny degenerate corridors anywhere in
      // the path (not just at the tail).
      {
        const double min_seg_len = 1.5;
        bool changed = true;
        while (changed && path.size() > 2) {
          changed = false;
          for (size_t i = 1; i + 1 < path.size(); ++i) {
            double seg_before = (path[i] - path[i - 1]).norm();
            double seg_after = (path[i + 1] - path[i]).norm();
            if (seg_before < min_seg_len || seg_after < min_seg_len) {
              path.erase(path.begin() + static_cast<long>(i));
              changed = true;
              break;  // restart scan
            }
          }
        }
      }

      // Segment end times for per-segment dynamic inflation
      const auto seg_end_times = computeSegEndTimesFromPath(path, corridor_nominal_speed_);

      // Convex decomposition -> linear constraints + polys
      EllipsoidDecomp3D ellip;
      std::vector<LinearConstraint3D> l_constraints;
      vec_E<Polyhedron<3>> poly_out;

      vec_Vecf<3> obst_pos_empty;  // set this if you want dynamic obstacle inflation in corridor
      vec_Vecf<3> obst_bbox_empty;
      const bool decomp_ok = hgp_.cvxEllipsoidDecomp(
          ellip, path, base_uo, obst_pos_empty, obst_bbox_empty, seg_end_times, l_constraints,
          poly_out);

      if (!decomp_ok) {
        RCLCPP_WARN(get_logger(), "Goal %zu: cvxEllipsoidDecomp failed. Skipping.", gi);
        continue;
      }

      publishCorridorAndPath(gi, path, poly_out, ellip);

      // Save
      const fs::path out =
          fs::path(output_dir_) / (output_prefix_ + "_g" + pad_int(gi, 3) + ".mysco2");

      saveCorridorBinary(out, start_, goal, path, seg_end_times, l_constraints);

      // Print segment details
      std::string seg_info;
      for (size_t s = 0; s + 1 < path.size(); ++s) {
        double len = (path[s + 1] - path[s]).norm();
        seg_info += std::to_string(len).substr(0, 4) + "m";
        if (s + 2 < path.size()) seg_info += ", ";
      }
      RCLCPP_INFO(
          get_logger(), "Goal %zu: %zu segments [%s]", gi, l_constraints.size(), seg_info.c_str());
    }
  }

  static std::string pad_int(int v, int width) {
    std::ostringstream oss;
    oss << std::setw(width) << std::setfill('0') << v;
    return oss.str();
  }

  void publishCorridorAndPath(
      size_t goal_idx,
      const vec_Vecf<3>& path,
      const vec_E<Polyhedron<3>>& poly_out,
      const EllipsoidDecomp3D& ellip) {
    // 1) Path — use pathLineDotsToMarkerArray (same as live planner)
    //    Use unique base_id per goal so markers accumulate in RViz.
    visualization_msgs::msg::MarkerArray path_msg;
    pathLineDotsToMarkerArray(
        path, &path_msg, color(RED),
        /*line_width=*/0.03,
        /*dot_diameter=*/0.06,
        /*base_id=*/static_cast<int>(goal_idx) * 1000,
        /*frame_id=*/frame_id_,
        /*lifetime_sec=*/0.0);  // 0 = forever

    // 2) Polyhedra corridor (decomp_rviz_plugins-compatible)
    decomp_ros_msgs::msg::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_out);
    poly_msg.header.stamp = now();
    poly_msg.header.frame_id = frame_id_;
    poly_msg.lifetime = rclcpp::Duration::from_seconds(0.0);  // forever

    // Publish immediately
    pub_path_->publish(path_msg);
    pub_poly_->publish(poly_msg);

    // Accumulate into cached messages for periodic republish
    for (const auto& m : path_msg.markers) cached_path_msg_.markers.push_back(m);
    for (const auto& p : poly_msg.polyhedrons) cached_poly_msg_.polyhedrons.push_back(p);
    cached_poly_msg_.header = poly_msg.header;
    cached_poly_msg_.lifetime = poly_msg.lifetime;
    have_cached_path_ = true;
    have_cached_poly_ = true;
  }

  void republishCachedMsgs() {
    if (!done_) return;  // only republish after generation

    if (have_cached_path_) pub_path_->publish(cached_path_msg_);
    if (have_cached_poly_) pub_poly_->publish(cached_poly_msg_);
  }

 private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex mtx_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;
  bool map_received_{false};
  bool done_{false};

  // Config
  std::string map_topic_;
  std::string output_dir_;
  std::string output_prefix_;

  Vec3f start_;
  std::vector<Vec3f> goals_;

  Vec3f map_center_;
  double wdx_{200.0}, wdy_{200.0}, wdz_{10.0};

  double corridor_nominal_speed_{2.0};
  int num_P_{3};  // max number of polytopes (matches live planner)

  // Planner config
  Parameters par_;
  HGPManager hgp_;

  std::string global_planner_{"sjps"};
  bool global_planner_verbose_{false};
  double weight_{1.0};
  int hgp_timeout_ms_{1000};

  double v_max_{2.0}, a_max_{3.0}, j_max_{10.0};

  // RViz publishing
  std::string frame_id_{"world"};
  std::string poly_topic_{"/sando/sfc_poly"};
  std::string path_topic_{"/sando/sfc_path"};
  bool keep_alive_{true};
  double republish_period_sec_{1.0};

  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_path_;
  rclcpp::Publisher<decomp_ros_msgs::msg::EllipsoidArray>::SharedPtr pub_ellip_;
  rclcpp::TimerBase::SharedPtr repub_timer_;

  // Cached messages
  visualization_msgs::msg::MarkerArray cached_path_msg_;
  decomp_ros_msgs::msg::PolyhedronArray cached_poly_msg_;
  decomp_ros_msgs::msg::EllipsoidArray cached_ellip_msg_;
  bool have_cached_path_{false};
  bool have_cached_poly_{false};
  bool have_cached_ellip_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorridorGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
