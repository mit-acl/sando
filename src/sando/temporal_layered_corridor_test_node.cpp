/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_rviz_plugins/data_ros_utils.hpp>  // DecompROS::polyhedron_array_to_ros
#include <geometry_msgs/msg/point.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hgp/hgp_manager.hpp"
#include "sando/gurobi_solver.hpp"
#include "sando/sando_type.hpp"
#include "sando/utils.hpp"

using namespace std::chrono_literals;
using namespace sando;

using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;

// ------------------------- small helpers -------------------------

static Vec3f vec3FromParam(const std::vector<double>& v, const std::string& name) {
  if (v.size() != 3) throw std::runtime_error("Parameter '" + name + "' must have size 3.");
  return Vec3f(v[0], v[1], v[2]);
}

static vec_Vecf<3> vec3ListFromFlat(const std::vector<double>& flat, const std::string& name) {
  if (flat.empty()) return {};
  if (flat.size() % 3 != 0)
    throw std::runtime_error("Parameter '" + name + "' must have length multiple of 3.");

  vec_Vecf<3> out;
  out.reserve(flat.size() / 3);
  for (size_t i = 0; i < flat.size(); i += 3) {
    Vec3f p(flat[i + 0], flat[i + 1], flat[i + 2]);
    out.push_back(p);
  }
  return out;
}

static RobotState makeStateFromPos(const Vec3f& p) {
  RobotState s;
  s.pos = Vec3d(p.x(), p.y(), p.z());
  s.vel = Vec3d::Zero();
  s.accel = Vec3d::Zero();
  return s;
}

static std_msgs::msg::ColorRGBA colorRGBA(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

static visualization_msgs::msg::Marker makeSphere(const std::string& frame_id, int id,
                                                  const Vec3f& p, double diameter,
                                                  const std_msgs::msg::ColorRGBA& c,
                                                  const std::string& ns, rclcpp::Time stamp) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = p.x();
  m.pose.position.y = p.y();
  m.pose.position.z = p.z();
  m.pose.orientation.w = 1.0;
  m.scale.x = diameter;
  m.scale.y = diameter;
  m.scale.z = diameter;
  m.color = c;
  m.lifetime = rclcpp::Duration::from_seconds(1.0);
  return m;
}

static visualization_msgs::msg::Marker makeLineStrip(const std::string& frame_id, int id,
                                                     const std::vector<Vec3f>& pts,
                                                     double line_width,
                                                     const std_msgs::msg::ColorRGBA& c,
                                                     const std::string& ns, rclcpp::Time stamp) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = line_width;
  m.color = c;
  m.lifetime = rclcpp::Duration::from_seconds(1.0);

  m.points.reserve(pts.size());
  for (const auto& p : pts) {
    geometry_msgs::msg::Point q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    m.points.push_back(q);
  }
  return m;
}

static void pathLineDotsToMarkerArray(const vec_Vecf<3>& traj,
                                      visualization_msgs::msg::MarkerArray* m_array,
                                      const std_msgs::msg::ColorRGBA& color,
                                      double line_width = 0.03, double dot_diameter = 0.06,
                                      int base_id = 50000, const std::string& frame_id = "map",
                                      double lifetime_sec = 1.0,
                                      const rclcpp::Time& stamp = rclcpp::Clock().now()) {
  if (!m_array || traj.empty()) return;

  // ---------- LINE_STRIP ----------
  visualization_msgs::msg::Marker line;
  line.header.frame_id = frame_id;
  line.header.stamp = stamp;
  line.ns = "global_path_line";
  line.id = base_id + 1;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.lifetime = rclcpp::Duration::from_seconds(lifetime_sec);

  line.scale.x = line_width;
  line.color = color;
  line.pose.orientation.w = 1.0;

  line.points.reserve(traj.size());
  for (const auto& it : traj) line.points.push_back(eigen2point(it));
  m_array->markers.push_back(line);

  // ---------- SPHERE_LIST (dots) ----------
  visualization_msgs::msg::Marker dots;
  dots.header.frame_id = frame_id;
  dots.header.stamp = stamp;
  dots.ns = "global_path_dots";
  dots.id = base_id + 2;
  dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  dots.action = visualization_msgs::msg::Marker::ADD;
  dots.lifetime = rclcpp::Duration::from_seconds(lifetime_sec);

  dots.scale.x = dot_diameter;
  dots.scale.y = dot_diameter;
  dots.scale.z = dot_diameter;

  dots.color = color;
  dots.pose.orientation.w = 1.0;

  dots.points.reserve(traj.size());
  for (const auto& it : traj) dots.points.push_back(eigen2point(it));
  m_array->markers.push_back(dots);
}

static visualization_msgs::msg::MarkerArray stateVector2ColoredMarkerArray(
    const std::vector<RobotState>& data, int type, double max_value, const rclcpp::Time& stamp,
    const std::string& frame_id = "map") {
  visualization_msgs::msg::MarkerArray marker_array;
  if (data.empty()) return marker_array;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "traj_committed_colored";
  m.id = type;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.pose.orientation.w = 1.0;

  m.scale.x = 0.15;  // thickness

  const size_t total_sample_number = 100;
  const size_t step = std::max<size_t>(1, data.size() / total_sample_number);
  const size_t segs = (data.size() > step) ? (data.size() / step) : 0;

  m.points.reserve(2 * segs);
  m.colors.reserve(2 * segs);

  geometry_msgs::msg::Point p_last;
  p_last.x = data[0].pos(0);
  p_last.y = data[0].pos(1);
  p_last.z = data[0].pos(2);

  for (size_t i = 0; i < data.size(); i += step) {
    geometry_msgs::msg::Point p;
    p.x = data[i].pos(0);
    p.y = data[i].pos(1);
    p.z = data[i].pos(2);

    const double vel = data[i].vel.norm();
    const std_msgs::msg::ColorRGBA c = getColorJet(vel, 0, max_value);

    m.points.push_back(p_last);
    m.colors.push_back(c);
    m.points.push_back(p);
    m.colors.push_back(c);

    p_last = p;
  }

  marker_array.markers.push_back(m);
  return marker_array;
}

// Compute max(Ax-b) (<=0 means inside all halfspaces for that polytope)
static double maxViolationOnePoly(const LinearConstraint3D& lc, const Vec3f& p) {
  if (lc.A_.rows() == 0) return 0.0;
  Eigen::VectorXd d = lc.A_ * p - lc.b_;
  return d.maxCoeff();
}

// For a given time-layer n, evaluate corridor union violation: min_p max(A_p x - b_p)
static double unionViolationAtLayer(const std::vector<LinearConstraint3D>& polys_at_n,
                                    const Vec3f& p, int* best_poly = nullptr) {
  double best = std::numeric_limits<double>::infinity();
  int best_i = -1;
  for (int i = 0; i < (int)polys_at_n.size(); ++i) {
    const double v = maxViolationOnePoly(polys_at_n[i], p);
    if (v < best) {
      best = v;
      best_i = i;
    }
  }
  if (best_poly) *best_poly = best_i;
  if (!std::isfinite(best)) best = 0.0;
  return best;
}

// ------------------------- node -------------------------

class TemporalLayeredCorridorTestNode final : public rclcpp::Node {
 public:
  TemporalLayeredCorridorTestNode() : Node("temporal_layered_corridor_test_node") {
    // ---------------- parameters ----------------
    declare_parameter<std::string>("frame_id", "map");

    declare_parameter<std::vector<double>>("start", {-4.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>("goal", {4.0, 0.0, 1.0});

    // Spatial corridor pieces (global segments) and time layers (local segments)
    declare_parameter<int>("P_global", 3);
    declare_parameter<int>("N_local", 6);

    // Timing used for time layers (and solver DT computation)
    declare_parameter<double>("initial_dt", 1.0);
    declare_parameter<double>("factor", 1.0);
    declare_parameter<double>("factor_initial", 1.0);
    declare_parameter<double>("factor_final", 2.0);

    // Sampling for post-check (and solver goal setpoints)
    declare_parameter<double>("dc", 0.01);

    // Dynamic limits (solver)
    declare_parameter<double>("x_min", -10.0);
    declare_parameter<double>("x_max", 10.0);
    declare_parameter<double>("y_min", -10.0);
    declare_parameter<double>("y_max", 10.0);
    declare_parameter<double>("z_min", 0.0);
    declare_parameter<double>("z_max", 5.0);

    declare_parameter<double>("v_max", 2.0);
    declare_parameter<double>("a_max", 5.0);
    declare_parameter<double>("j_max", 10.0);
    declare_parameter<double>("w_max", 0.5);

    declare_parameter<double>("max_gurobi_comp_time_sec", 5.0);
    declare_parameter<double>("jerk_smooth_weight", 1.0e+1);

    declare_parameter<bool>("using_variable_elimination", true);
    declare_parameter<bool>("debug_verbose", true);

    // Corridor generation params (HGPManager / EllipsoidDecomp)
    declare_parameter<double>("res", 0.15);
    declare_parameter<double>("factor_hgp", 1.0);
    declare_parameter<double>("inflation_hgp", 0.0);
    declare_parameter<double>("drone_radius", 0.1);
    declare_parameter<double>("obst_max_vel", 0.1);  // affects obstacle_to_vec inflation r = v*t
    declare_parameter<std::vector<double>>("sfc_size", {4.0, 4.0, 3.0});
    declare_parameter<bool>("use_shrinked_box", false);
    declare_parameter<double>("shrinked_box_size", 0.2);
    declare_parameter<double>("max_dist_vertexes", 2.0);

    // Obstacles
    // Dynamic obstacle centers (flat list: [x0,y0,z0, x1,y1,z1, ...])
    declare_parameter<std::vector<double>>("dynamic_obstacles_flat", {0.0, 0.0, 1.0});
    // Optional static obstacle points (flat list). Leave empty if you only want dynamic.
    declare_parameter<std::vector<double>>("static_obstacles_flat", std::vector<double>{});

    // Visualization / playback
    declare_parameter<std::string>("topic_poly_layer", "/sando/poly_time_layer");
    declare_parameter<std::string>("topic_markers", "/sando/temporal_test_markers");
    declare_parameter<bool>("cycle_layers", true);
    declare_parameter<double>("cycle_period_sec", 0.4);
    declare_parameter<int>("fixed_layer", 0);  // used when cycle_layers=false

    // Map window for voxelization
    declare_parameter<double>("wdx", 20.0);
    declare_parameter<double>("wdy", 20.0);
    declare_parameter<double>("wdz", 6.0);
    declare_parameter<std::vector<double>>("map_center", {0.0, 0.0, 1.0});

    // HGP global planner params
    declare_parameter<std::string>("global_planner", "sjps");  // or "jps" depending on your code
    declare_parameter<bool>("global_planner_verbose", false);
    declare_parameter<double>("hgp_timeout_ms", 50.0);
    declare_parameter<double>("global_planner_heuristic_weight", 1.0);

    // Publish the global path to the same topic as your real stack
    declare_parameter<std::string>("hgp_path_topic", "/NX01/hgp_path_marker");

    declare_parameter<std::string>("traj_colored_topic", "/NX01/traj_committed_colored");

    // ---------------- read parameters ----------------
    frame_id_ = get_parameter("frame_id").as_string();

    start_ = vec3FromParam(get_parameter("start").as_double_array(), "start");
    goal_ = vec3FromParam(get_parameter("goal").as_double_array(), "goal");

    P_global_ = get_parameter("P_global").as_int();
    N_local_ = get_parameter("N_local").as_int();

    initial_dt_ = get_parameter("initial_dt").as_double();
    factor_ = get_parameter("factor").as_double();
    factor_initial_ = get_parameter("factor_initial").as_double();
    factor_final_ = get_parameter("factor_final").as_double();
    dc_ = get_parameter("dc").as_double();

    poly_topic_ = get_parameter("topic_poly_layer").as_string();
    markers_topic_ = get_parameter("topic_markers").as_string();

    cycle_layers_ = get_parameter("cycle_layers").as_bool();
    cycle_period_sec_ = get_parameter("cycle_period_sec").as_double();
    fixed_layer_ = get_parameter("fixed_layer").as_int();

    // Build parameters struct used by both HGPManager and SolverGurobi
    par_.num_N = N_local_;
    par_.dc = dc_;

    par_.x_min = get_parameter("x_min").as_double();
    par_.x_max = get_parameter("x_max").as_double();
    par_.y_min = get_parameter("y_min").as_double();
    par_.y_max = get_parameter("y_max").as_double();
    par_.z_min = get_parameter("z_min").as_double();
    par_.z_max = get_parameter("z_max").as_double();

    par_.v_max = get_parameter("v_max").as_double();
    par_.a_max = get_parameter("a_max").as_double();
    par_.j_max = get_parameter("j_max").as_double();
    par_.w_max = get_parameter("w_max").as_double();

    par_.max_gurobi_comp_time_sec = get_parameter("max_gurobi_comp_time_sec").as_double();
    par_.jerk_smooth_weight = get_parameter("jerk_smooth_weight").as_double();
    par_.using_variable_elimination = get_parameter("using_variable_elimination").as_bool();
    par_.debug_verbose = get_parameter("debug_verbose").as_bool();

    // Corridor params (HGP)
    par_.res = get_parameter("res").as_double();
    par_.factor_hgp = get_parameter("factor_hgp").as_double();
    par_.inflation_hgp = get_parameter("inflation_hgp").as_double();
    par_.drone_radius = get_parameter("drone_radius").as_double();
    par_.obst_max_vel = get_parameter("obst_max_vel").as_double();

    {
      auto lbs = get_parameter("sfc_size").as_double_array();
      if (lbs.size() != 3) throw std::runtime_error("sfc_size must be size 3");
      par_.sfc_size = {lbs[0], lbs[1], lbs[2]};
    }
    par_.use_shrinked_box = get_parameter("use_shrinked_box").as_bool();
    par_.shrinked_box_size = get_parameter("shrinked_box_size").as_double();
    par_.max_dist_vertexes = get_parameter("max_dist_vertexes").as_double();

    // Obstacles
    obst_pos_ = vec3ListFromFlat(get_parameter("dynamic_obstacles_flat").as_double_array(),
                                 "dynamic_obstacles_flat");
    // Initialize bbox with default half-extents for each obstacle
    obst_bbox_.clear();
    obst_bbox_.resize(obst_pos_.size(), Vecf<3>(0.4, 0.4, 0.4));
    base_uo_ = vec3ListFromFlat(get_parameter("static_obstacles_flat").as_double_array(),
                                "static_obstacles_flat");

    wdx_ = get_parameter("wdx").as_double();
    wdy_ = get_parameter("wdy").as_double();
    wdz_ = get_parameter("wdz").as_double();
    map_center_ = vec3FromParam(get_parameter("map_center").as_double_array(), "map_center");

    global_planner_ = get_parameter("global_planner").as_string();
    global_planner_verbose_ = get_parameter("global_planner_verbose").as_bool();
    hgp_timeout_ms_ = get_parameter("hgp_timeout_ms").as_double();
    global_planner_hweight_ = get_parameter("global_planner_heuristic_weight").as_double();

    hgp_path_topic_ = get_parameter("hgp_path_topic").as_string();

    traj_colored_topic_ = get_parameter("traj_colored_topic").as_string();

    if (P_global_ <= 0 || N_local_ <= 0)
      throw std::runtime_error("P_global and N_local must be positive.");
    if (!(initial_dt_ > 0.0) || !(factor_ > 0.0) || !(dc_ > 0.0))
      throw std::runtime_error("initial_dt, factor, dc must be > 0.");

    // ---------------- publishers ----------------
    rclcpp::QoS qos_poly(rclcpp::KeepLast(1));
    qos_poly.reliable().transient_local();

    pub_poly_layer_ =
        create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(poly_topic_, qos_poly);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 10);

    rclcpp::QoS qos_latched(rclcpp::KeepLast(1));
    qos_latched.reliable().transient_local();
    pub_hgp_path_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(hgp_path_topic_, qos_latched);
    pub_traj_colored_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(traj_colored_topic_, 10);

    // ---------------- run once: build path, corridor, solve ----------------
    buildGlobalPath_();
    buildTimeLayers_();
    buildTemporalCorridor_();
    solveTrajectory_();
    buildVizCache_();
    runPostCheck_();

    // ---------------- publish loop ----------------
    const double period = std::max(0.05, cycle_period_sec_);
    timer_ = create_wall_timer(std::chrono::duration<double>(period),
                               std::bind(&TemporalLayeredCorridorTestNode::publishOnce_, this));
  }

 private:
  // Build global path with P_global segments (P_global+1 points)
  void buildGlobalPath_() {
    // 1) Convert static obstacles into a point cloud for updateMap()
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.reserve(base_uo_.size());

    for (const auto& p : base_uo_) {
      pcl::PointXYZ q;
      q.x = p.x();
      q.y = p.y();
      q.z = p.z();
      cloud->points.push_back(q);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    // 2) Initialize HGP manager
    hgp_.setParameters(par_);

    // 3) Update map with *dynamic obstacles* (this is the key part)
    // Use planning horizon as traj_max_time for spatio-temporal occupancy.
    const double traj_max_time = (initial_dt_ * factor_final_) * static_cast<double>(N_local_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>());
    vec_Vecf<3> empty_obst_bbox;  // Empty bbox vector (no dynamic obstacles in this test)
    hgp_.updateMap(wdx_, wdy_, wdz_, map_center_, cloud, empty_pclptr_unk, obst_pos_,
                   empty_obst_bbox, traj_max_time);

    // 4) Setup planner snapshot
    hgp_.setupHGPPlanner(global_planner_, global_planner_verbose_, par_.res, par_.v_max, par_.a_max,
                         par_.j_max, hgp_timeout_ms_,
                         /*max_num_expansion*/ 10000,
                         /*w_unknown*/ 0.0, /*w_align*/ 0.0, /*decay_len_cells*/ 100.0,
                         /*w_side*/ 0.0,
                         /*los_cells*/ 0, /*min_len*/ 0.5, /*min_turn*/ 0.0);

    // 5) Direction hint (same pattern as your corridor_generator_node)
    Vec3f dir = goal_ - start_;
    if (dir.norm() > 1e-6)
      dir = dir / dir.norm();
    else
      dir = Vec3f(1.0, 0.0, 0.0);

    // 6) Solve HGP
    double final_g = 0.0;
    vec_Vecf<3> path;
    vec_Vecf<3> raw_path;

    const double tnow = now().seconds();
    const bool ok =
        hgp_.solveHGP(start_, dir, goal_, final_g, global_planner_hweight_, tnow, path, raw_path);

    if (!ok || path.size() < 2)
      throw std::runtime_error("HGP global planner failed to find a path.");

    global_path_ = path;  // <-- use the real path for corridor + optimization
  }

  // Build time layers: t_end[n] = (n+1) * dt_layer
  void buildTimeLayers_() {
    const double dt_layer = initial_dt_ * factor_;
    time_end_times_.clear();
    time_end_times_.reserve((size_t)N_local_);
    for (int n = 0; n < N_local_; ++n) time_end_times_.push_back((double)(n + 1) * dt_layer);
  }

  void buildTemporalCorridor_() {
    // HGP manager parameter wiring
    hgp_.setParameters(par_);

    // Ellipsoid decomp worker
    EllipsoidDecomp3D ellip;

    // Generate [N][P] corridor constraints + polyhedra
    const bool ok = hgp_.cvxEllipsoidDecompTimeLayered(ellip, global_path_, base_uo_, obst_pos_,
                                                       obst_bbox_, time_end_times_,
                                                       l_constraints_by_time_,  // [N][P]
                                                       poly_out_by_time_        // [N][P]
    );

    if (!ok) throw std::runtime_error("cvxEllipsoidDecompTimeLayered failed.");
  }

  void solveTrajectory_() {
    // Solver setup
    solver_ = std::make_shared<SolverGurobi>();
    solver_->setPlannerName("SANDO");
    solver_->initializeSolver(par_);
    solver_->setT0(0.0);
    solver_->setInitialDt(initial_dt_);

    // Boundary conditions
    const RobotState x0 = makeStateFromPos(start_);
    RobotState xf;
    xf.setPos(global_path_[(size_t)P_global_ - 1].x(), global_path_[(size_t)P_global_ - 1].y(),
              global_path_[(size_t)P_global_ - 1].z());
    solver_->setX0(x0);
    solver_->setXf(xf);

    // Temporal-layered corridor (your new API)
    solver_->setPolytopesTimeLayered(l_constraints_by_time_);

    bool gurobi_error = false;
    double grb_ms = 0.0;

    const bool use_single_thread = true;
    const bool ok =
        solver_->generateNewTrajectory(gurobi_error, grb_ms, factor_, use_single_thread);

    if (!ok || gurobi_error) {
      std::ostringstream oss;
      oss << "generateNewTrajectory failed. ok=" << ok << " gurobi_error=" << gurobi_error;
      throw std::runtime_error(oss.str());
    }

    // Extract goal setpoints for visualization & post-check
    goal_setpoints_.clear();
    solver_->fillGoalSetPoints();
    solver_->getGoalSetpoints(goal_setpoints_);
  }

  void buildVizCache_() {
    // Prebuild PolyhedronArray per time layer
    poly_msgs_.clear();
    poly_msgs_.resize((size_t)N_local_);

    for (int n = 0; n < N_local_; ++n) {
      // Convert poly_out_by_time_[n] (vector of P Polyhedron<3>) to ROS msg
      auto msg = DecompROS::polyhedron_array_to_ros(poly_out_by_time_[n]);
      msg.header.frame_id = frame_id_;
      msg.header.stamp = now();
      msg.lifetime = rclcpp::Duration::from_seconds(1.0);
      poly_msgs_[(size_t)n] = msg;
    }

    // Global path marker
    global_path_line_.clear();
    for (const auto& p : global_path_) global_path_line_.push_back(p);

    // Trajectory marker points from goal setpoints
    traj_line_.clear();
    traj_line_.reserve(goal_setpoints_.size());
    for (const auto& s : goal_setpoints_) {
      Vec3f p(s.pos.x(), s.pos.y(), s.pos.z());
      traj_line_.push_back(p);
    }
  }

  void runPostCheck_() {
    // Sampled post-check: for each setpoint at time t = k*dc, map to layer n=floor(t/dt_layer)
    // and compute union violation min_p max(Ap-b).
    const double dt_layer = initial_dt_ * factor_;

    double worst = -1e9;
    double t_at_worst = 0.0;
    Vec3f p_at_worst(0, 0, 0);
    int best_poly = -1;
    int n_at_worst = -1;

    for (size_t k = 0; k < goal_setpoints_.size(); ++k) {
      const double t = (double)k * dc_;
      int n = (int)std::floor(t / dt_layer);
      n = std::max(0, std::min(N_local_ - 1, n));

      const auto& s = goal_setpoints_[k];
      Vec3f p(s.pos.x(), s.pos.y(), s.pos.z());

      int best = -1;
      const double v = unionViolationAtLayer(l_constraints_by_time_[(size_t)n], p, &best);

      if (v > worst) {
        worst = v;
        t_at_worst = t;
        p_at_worst = p;
        best_poly = best;
        n_at_worst = n;
      }
    }

    RCLCPP_INFO(get_logger(),
                "Post-check (sampled @dc=%.3f): worst union violation = %.6f at t=%.3f (layer "
                "n=%d, best_poly=%d) p=(%.3f,%.3f,%.3f). "
                "Interpretation: <=0 means inside corridor union for that layer.",
                dc_, worst, t_at_worst, n_at_worst, best_poly, p_at_worst.x(), p_at_worst.y(),
                p_at_worst.z());
  }

  void publishOnce_() {
    const rclcpp::Time stamp = now();

    int n_show = 0;
    if (cycle_layers_) {
      n_show = layer_idx_;
      layer_idx_ = (layer_idx_ + 1) % std::max(1, N_local_);
    } else {
      n_show = std::max(0, std::min(N_local_ - 1, fixed_layer_));
    }

    // Publish polyhedra for this time layer
    auto poly_msg = poly_msgs_[(size_t)n_show];
    poly_msg.header.stamp = stamp;
    pub_poly_layer_->publish(poly_msg);

    // Markers: start/goal, global path, trajectory, obstacle reachable sets for this layer
    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(8 + obst_pos_.size());

    // Start / goal
    ma.markers.push_back(makeSphere(frame_id_, 1, start_, 0.20, colorRGBA(0.0f, 1.0f, 0.0f, 1.0f),
                                    "start_goal", stamp));
    ma.markers.push_back(makeSphere(frame_id_, 2, goal_, 0.20, colorRGBA(1.0f, 0.0f, 0.0f, 1.0f),
                                    "start_goal", stamp));

    // Reachable set spheres for dynamic obstacles at time layer n_show
    // r = obst_max_vel * t_end[n]
    const double t_end = time_end_times_[(size_t)n_show];
    const double r = std::max(0.0, par_.obst_max_vel * t_end);
    const double diameter = 2.0 * r;

    for (size_t i = 0; i < obst_pos_.size(); ++i) {
      // Draw center (small)
      ma.markers.push_back(makeSphere(frame_id_, 1000 + (int)i, obst_pos_[i], 0.16,
                                      colorRGBA(1.0f, 0.6f, 0.0f, 1.0f), "dyn_obst_center", stamp));

      // Draw reachable sphere (translucent)
      if (r > 1e-6) {
        ma.markers.push_back(makeSphere(frame_id_, 2000 + (int)i, obst_pos_[i], diameter,
                                        colorRGBA(1.0f, 0.6f, 0.0f, 0.12f), "dyn_obst_reachable",
                                        stamp));
      }
    }

    // A small text marker showing the layer index
    {
      visualization_msgs::msg::Marker txt;
      txt.header.frame_id = frame_id_;
      txt.header.stamp = stamp;
      txt.ns = "layer_text";
      txt.id = 999;
      txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x = start_.x();
      txt.pose.position.y = start_.y();
      txt.pose.position.z = start_.z() + 0.6;
      txt.pose.orientation.w = 1.0;
      txt.scale.z = 0.35;
      txt.color = colorRGBA(1.0f, 1.0f, 1.0f, 1.0f);
      std::ostringstream oss;
      oss << "time layer n=" << n_show << "  t_end=" << std::fixed << std::setprecision(2) << t_end;
      txt.text = oss.str();
      txt.lifetime = rclcpp::Duration::from_seconds(1.0);
      ma.markers.push_back(txt);
    }

    pub_markers_->publish(ma);

    // Publish committed trajectory as colored LINE_LIST to /NX01/traj_committed_colored

    auto traj_ma = stateVector2ColoredMarkerArray(goal_setpoints_,
                                                  /*type*/ 0,
                                                  /*max_value*/ par_.v_max, stamp, frame_id_);
    pub_traj_colored_->publish(traj_ma);

    // Publish global path to /NX01/hgp_path_marker as thin line + dots
    visualization_msgs::msg::MarkerArray hgp_ma;
    pathLineDotsToMarkerArray(global_path_, &hgp_ma, colorRGBA(0.0f, 1.0f, 0.0f, 1.0f),
                              /*line_width*/ 0.03,
                              /*dot_diameter*/ 0.06,
                              /*base_id*/ 50000, frame_id_,
                              /*lifetime_sec*/ 1.0, stamp);

    pub_hgp_path_->publish(hgp_ma);
  }

 private:
  // Parameters
  std::string frame_id_;
  std::string poly_topic_;
  std::string markers_topic_;

  double wdx_{20.0};
  double wdy_{20.0};
  double wdz_{6.0};
  Vec3f map_center_{0, 0, 1};
  std::string global_planner_;
  bool global_planner_verbose_{false};
  double hgp_timeout_ms_{50.0};
  double global_planner_hweight_{1.0};
  std::string hgp_path_topic_;
  double factor_initial_{1.0};
  double factor_final_{1.0};

  Vec3f start_{0, 0, 0};
  Vec3f goal_{0, 0, 0};

  int P_global_{3};
  int N_local_{6};

  double initial_dt_{0.5};
  double factor_{2.0};
  double dc_{0.01};

  bool cycle_layers_{true};
  double cycle_period_sec_{0.4};
  int fixed_layer_{0};

  // Core structs
  Parameters par_;
  HGPManager hgp_;
  std::shared_ptr<SolverGurobi> solver_;

  // Inputs
  vec_Vecf<3> obst_pos_;   // dynamic obstacle centers
  vec_Vecf<3> obst_bbox_;  // dynamic obstacle bbox half-extents
  vec_Vec3f base_uo_;      // static obstacle points

  // Generated
  vec_Vecf<3> global_path_;             // P+1 points
  std::vector<double> time_end_times_;  // size N

  std::vector<std::vector<LinearConstraint3D>> l_constraints_by_time_;  // [N][P]
  std::vector<vec_E<Polyhedron<3>>> poly_out_by_time_;                  // [N][P] for RViz

  std::vector<decomp_ros_msgs::msg::PolyhedronArray> poly_msgs_;  // [N]
  std::vector<RobotState> goal_setpoints_;

  // Viz cache
  std::vector<Vec3f> global_path_line_;
  std::vector<Vec3f> traj_line_;

  // ROS
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_layer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_hgp_path_;

  std::string traj_colored_topic_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_colored_;

  rclcpp::TimerBase::SharedPtr timer_;

  int layer_idx_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TemporalLayeredCorridorTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
