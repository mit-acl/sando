/* ----------------------------------------------------------------------------
 * Copyright (c) Anonymous Author
 * Anonymous Institution
 * All Rights Reserved
 * Authors: Anonymous
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "sando/sando.hpp"
#include <chrono>
#include <fstream>

using namespace sando;
using namespace termcolor;

typedef timer::Timer MyTimer;

// ----------------------------------------------------------------------------

SANDO::SANDO(Parameters par) : par_(par) {
  // Set up hgp_manager
  hgp_manager_.setParameters(par_);

  // Compute factors_ for time allocation
  if (par_.use_dynamic_factor) {
    // Dynamic factor search
    num_dynamic_factors_ =
        static_cast<int>((2 * par_.dynamic_factor_k_radius) / par_.factor_constant_step_size) + 1;
    factors_.reserve(num_dynamic_factors_);
    for (int i = 0; i < num_dynamic_factors_; i++) {
      double factor = par_.dynamic_factor_initial_mean - par_.dynamic_factor_k_radius +
                      i * par_.factor_constant_step_size;
      if (factor >= par_.factor_initial && factor <= par_.factor_final) factors_.push_back(factor);
    }
  } else {
    // Constant factor search
    num_dynamic_factors_ =
        static_cast<int>(
            (par_.factor_final - par_.factor_initial) / par_.factor_constant_step_size) +
        1;
    factors_.reserve(num_dynamic_factors_);
    for (int i = 0; i < num_dynamic_factors_; i++) {
      double factor = par_.factor_initial + i * par_.factor_constant_step_size;
      factors_.push_back(factor);
    }
  }

  // Set up optimization solver for whole trajectory.
  // Hybrid threading: distribute CPU cores across external factor threads
  // so each Gurobi instance gets multiple internal threads rather than
  // all threads competing for all cores.
  const int num_cores = static_cast<int>(std::thread::hardware_concurrency());
  const int grb_threads_per_solver = std::max(1, num_cores / std::max(1, num_dynamic_factors_));
  whole_traj_solver_ptrs_.reserve(num_dynamic_factors_);
  for (int i = 0; i < num_dynamic_factors_; i++) {
    auto solver = std::make_shared<SolverGurobi>();
    solver->setGurobiThreads(grb_threads_per_solver);
    solver->initializeSolver(par_);
    whole_traj_solver_ptrs_.push_back(solver);
  }

  // Set up decomp ellip workers for each thread
  ellip_workers_.resize(whole_traj_solver_ptrs_.size());

  // Pre-compute the worst initial_dt * par_.num_N (this is the worst case time allocated for the
  // whole trajectory)
  auto tmp_traj_solver_ptr = std::make_shared<SolverGurobi>();
  tmp_traj_solver_ptr->initializeSolver(par_);
  tmp_traj_solver_ptr->resetToNominalState();
  RobotState tmp_start_state, tmp_end_state;
  tmp_end_state.setPos(par_.num_P * par_.max_dist_vertexes, 0.0, 0.0);
  tmp_traj_solver_ptr->setX0(tmp_start_state);
  tmp_traj_solver_ptr->setXf(tmp_end_state);
  worst_traj_time_ = tmp_traj_solver_ptr->getInitialDt() * par_.num_N;

  // Set up basis converter
  BasisConverter basis_converter;
  A_rest_pos_basis_ = basis_converter.getArestMinvo();  // Use Minvo basis
  A_rest_pos_basis_inverse_ = A_rest_pos_basis_.inverse();

  // Parameters
  v_max_3d_ = Eigen::Vector3d(par_.v_max, par_.v_max, par_.v_max);
  v_max_ = par_.v_max;
  a_max_3d_ = Eigen::Vector3d(par_.a_max, par_.a_max, par_.a_max);
  j_max_3d_ = Eigen::Vector3d(par_.j_max, par_.j_max, par_.j_max);

  // Initialize the state
  changeDroneStatus(DroneStatus::GOAL_REACHED);

  // Initialize the map size
  wdx_ = par_.initial_wdx;
  wdy_ = par_.initial_wdy;
  wdz_ = par_.initial_wdz;

  // Map resolution
  map_res_ = par_.res;
}

// ----------------------------------------------------------------------------

void SANDO::startAdaptKValue() {
  // Compute the average computation time
  const size_t num_samples = store_computation_times_.size();
  for (const auto& comp_time : store_computation_times_) {
    est_comp_time_ += comp_time;
  }
  est_comp_time_ = est_comp_time_ / num_samples;

  // Start k_value adaptation
  use_adapt_k_value_ = true;
}

// ----------------------------------------------------------------------------

void SANDO::computeG(const RobotState& A, const RobotState& G_term, double horizon) {
  // Initialize the result
  RobotState local_G;

  // Compute pos for G
  local_G.pos = sando_utils::projectPointToSphere(A.pos, G_term.pos, horizon);

  // Compute yaw for G
  Eigen::Vector3d dir = (G_term.pos - local_G.pos).normalized();
  local_G.yaw = atan2(dir[1], dir[0]);

  // Set G
  setG(local_G);
}

// ----------------------------------------------------------------------------

bool SANDO::needReplan(
    const RobotState& local_state,
    const RobotState& local_G_term,
    const RobotState& last_plan_state) {
  // Compute the distance to the terminal goal
  double dist_to_term_G = (local_state.pos - local_G_term.pos).norm();
  double dist_from_last_plan_state_to_term_G = (last_plan_state.pos - local_G_term.pos).norm();

  // Check velocity magnitude to ensure drone is moving slowly enough
  double vel_magnitude = local_state.vel.norm();
  const double max_goal_velocity = 0.1;  // [m/s] Maximum velocity when reaching goal

  // Hover avoidance: allow replanning when GOAL_REACHED or HOVER_AVOIDING
  // Must be checked before the GOAL_REACHED early return so the drone can
  // detect nearby obstacles while hovering at the goal.
  if (par_.hover_avoidance_enabled && (drone_status_ == DroneStatus::GOAL_REACHED ||
                                       drone_status_ == DroneStatus::HOVER_AVOIDING)) {
    return true;
  }

  if (dist_to_term_G < par_.goal_radius && vel_magnitude < max_goal_velocity) {
    if (par_.hover_avoidance_enabled) {
      p_hover_ = local_G_term.pos;
      changeDroneStatus(DroneStatus::HOVER_AVOIDING);
      return true;  // allow replan loop to run checkHoverAvoidance
    } else {
      changeDroneStatus(DroneStatus::GOAL_REACHED);
      p_hover_ = local_G_term.pos;
      return false;
    }
  }

  // Don't plan if drone is not traveling
  // IMPORTANT: this must come BEFORE the GOAL_SEEN check below, otherwise
  // the GOAL_SEEN distance check would overwrite YAWING status and trigger
  // replanning (causing the drone to move instead of yawing in place).
  if (drone_status_ == DroneStatus::GOAL_REACHED || drone_status_ == DroneStatus::YAWING)
    return false;

  if (dist_to_term_G < par_.goal_seen_radius) {
    changeDroneStatus(
        DroneStatus::GOAL_SEEN);  // This triggers to use the hard final state constraint
  }

  if (drone_status_ == DroneStatus::GOAL_SEEN &&
      dist_from_last_plan_state_to_term_G < par_.goal_radius)
    return false;

  return true;
}

// ----------------------------------------------------------------------------

bool SANDO::findAandAtime(
    RobotState& A, double& A_time, double current_time, double last_replaning_computation_time) {
  mtx_plan_.lock();
  int plan_size = plan_.size();
  mtx_plan_.unlock();

  if (plan_size == 0) {
    std::cout << bold << red << "plan_size == 0" << reset << std::endl;
    return false;
  }

  if (par_.use_state_update) {
    // Change k_value dynamically
    // To get stable results, we will use a default value of k_value until we have enough
    // computation time
    if (!use_adapt_k_value_) {
      // Use default k_value
      k_value_ = std::max((int)plan_size - par_.default_k_value, 0);

      // Store computation times
      if (num_replanning_ != 1)  // Don't store the very first computation time (because we don't
                                 // have a previous computation time)
        store_computation_times_.push_back(last_replaning_computation_time);
    } else {
      // Computation time filtering
      est_comp_time_ = par_.alpha_k_value_filtering * last_replaning_computation_time +
                       (1 - par_.alpha_k_value_filtering) * est_comp_time_;

      // Get state number based on est_comp_time_ and dc
      k_value_ =
          std::max((int)plan_size - (int)(par_.k_value_factor * est_comp_time_ / par_.dc), 0);
    }

    // Check if k_value_ is valid
    if (plan_size - 1 - k_value_ < 0 || plan_size - 1 - k_value_ >= plan_size) {
      k_value_ =
          plan_size - 1;  // If k_value_ is larger than the plan size, we set it to the last state
    }

    // Get A
    mtx_plan_.lock();
    A = plan_[plan_size - 1 - k_value_];
    mtx_plan_.unlock();

    // Get A_time
    A_time = current_time +
             (plan_size - 1 - k_value_) *
                 par_.dc;  // time to A from current_pos is (plan_size - 1 - k_value_) * par_.dc;
  } else  // If we don't update state - this is for global planner benchmarking purposes
  {
    // Get state
    getState(A);
    A_time = current_time;
  }

  // Check if A is within the map (especially for z)
  if ((A.pos[2] < par_.z_min || A.pos[2] > par_.z_max) ||
      (A.pos[0] < par_.x_min || A.pos[0] > par_.x_max) ||
      (A.pos[1] < par_.y_min || A.pos[1] > par_.y_max)) {
    printf("A (%f, %f, %f) is out of the map\n", A.pos[0], A.pos[1], A.pos[2]);
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool SANDO::checkIfPointOccupied(const Vec3f& point) {
  // Check if the point is free
  return hgp_manager_.checkIfPointOccupied(point);
}

// ----------------------------------------------------------------------------

bool SANDO::checkIfPointFree(const Vec3f& point) {
  // Check if the point is free
  return hgp_manager_.checkIfPointFree(point);
}

// ----------------------------------------------------------------------------

void SANDO::findSafeSubGoal(vec_Vecf<3>& global_path) {
  // Keep the original global path
  vec_Vecf<3> original_global_path = global_path;

  // Reset goal path
  global_path.clear();

  if (original_global_path.empty()) return;

  // Initialize it with the start point
  global_path.push_back(original_global_path[0]);

  // Kd-tree search parameters
  const int k = 1;  // nearest neighbor
  std::vector<int> pointIdxNKNSearch(k);
  std::vector<float> pointNKNSquaredDistance(k);

  // Sampling parameters (TODO: make these parameters configurable)
  const double sample_dist = 0.1;  // [m] distance between two samples along the trajectory

  // Inflation radius for unknown space (max extent)
  const double r_inflate = par_.obst_max_vel * traj_max_time_;  // [m]
  const double thr_orig = par_.drone_radius;                    // [m]
  const double thr_infl = par_.drone_radius + r_inflate;        // [m]
  const double thr_orig2 = thr_orig * thr_orig;
  const double thr_infl2 = thr_infl * thr_infl;

  // Mutex lock (KD-tree shared)
  std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);

  // Helper: returns true if pt is within (unknown KD-tree distance) <= threshold^2.
  auto isWithinUnknown = [&](const Eigen::Vector3d& pt, double thr2) -> bool {
    pcl::PointXYZ searchPoint(pt(0), pt(1), pt(2));
    if (kdtree_unk_.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) >
        0) {
      return static_cast<double>(pointNKNSquaredDistance[0]) < thr2;
    }
    return false;
  };

  // Helper: backtrack from a hit location (segment i, arc-length s_hit along that segment)
  // until outside inflated unknown. Returns the backtracked safe point.
  auto backtrackToOutsideInflated = [&](int seg_i, double s_hit) -> Eigen::Vector3d {
    // We will walk backward in steps of sample_dist along the polyline.
    int i = seg_i;
    if (i < 0) i = 0;
    if (i >= static_cast<int>(original_global_path.size()) - 1)
      i = static_cast<int>(original_global_path.size()) - 2;

    Eigen::Vector3d A = original_global_path[i];
    Eigen::Vector3d B = original_global_path[i + 1];

    Eigen::Vector3d d = B - A;
    double L = d.norm();
    if (L < 1e-9) return A;  // degenerate segment

    Eigen::Vector3d dir = d / L;

    // Clamp s to [0, L]
    double s = std::min(std::max(0.0, s_hit), L);

    // Start from the hit point
    Eigen::Vector3d pt = A + dir * s;

    // If we're already outside inflated unknown, keep it (shouldn't happen in your described flow)
    if (!isWithinUnknown(pt, thr_infl2)) return pt;

    // Walk backward until outside inflated unknown or we reach the start.
    // This can cross segment boundaries if inflation is large.
    while (true) {
      // Step backward on current segment
      s -= sample_dist;

      if (s >= 0.0) {
        pt = A + dir * s;
      } else {
        // Need to go to previous segment
        i -= 1;
        if (i < 0) {
          // We reached the very beginning; return the start point (best we can do)
          return original_global_path.front();
        }

        // New segment [i, i+1]
        A = original_global_path[i];
        B = original_global_path[i + 1];
        d = B - A;
        L = d.norm();
        if (L < 1e-9) {
          // Skip degenerate segment
          s = 0.0;
          pt = A;
          continue;
        }
        dir = d / L;

        // We crossed into previous segment: set s at its end (B) plus leftover negative s
        // Example: if s was -0.03, we start at L - 0.03 on the previous segment.
        s = L + s;  // s is negative here
        if (s < 0.0) s = 0.0;
        if (s > L) s = L;

        pt = A + dir * s;
      }

      // Check inflated condition
      if (!isWithinUnknown(pt, thr_infl2)) return pt;
    }
  };

  // Loop through the global path and check for intersection with original unknown (NOT inflated)
  const int M = static_cast<int>(original_global_path.size());
  for (int i = 0; i < M - 1; i++) {
    Eigen::Vector3d current_gp = original_global_path[i];
    Eigen::Vector3d next_gp = original_global_path[i + 1];

    Eigen::Vector3d dir = next_gp - current_gp;
    double dist = dir.norm();
    if (dist < 1e-9) {
      // Degenerate; just continue
      continue;
    }
    dir /= dist;

    // Sample points along the line segment
    const int num_samples = static_cast<int>(dist / sample_dist);

    for (int j = 0; j <= num_samples; j++) {
      Eigen::Vector3d sample_point = current_gp + dir * (sample_dist * j);

      // Detect intersection with original unknown (same as before, but squared distance)
      if (isWithinUnknown(sample_point, thr_orig2)) {
        // Found first contact with original unknown -> now backtrack until outside inflated unknown
        const double s_hit = sample_dist * j;
        Eigen::Vector3d safe_pt = backtrackToOutsideInflated(i, s_hit);

        // Ensure we don't add duplicates
        if ((safe_pt - global_path.back()).norm() > 1e-6) global_path.push_back(safe_pt);

        return;  // Stop: this is the new last global path point
      }
    }

    // No unknown intersection on this segment; keep the next waypoint
    global_path.push_back(next_gp);
  }
}

// ----------------------------------------------------------------------------

void SANDO::computeMapSize(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
  // Get local_A
  RobotState local_A;
  getA(local_A);

  // Increase the effective buffer size based on the number of HGP failures.
  double dynamic_buffer = par_.map_buffer;

  // Increase the effective buffer size based on velocity.
  double dynamic_buffer_x = dynamic_buffer;
  double dynamic_buffer_y = dynamic_buffer;
  double dynamic_buffer_z = dynamic_buffer;

  // Compute the distance to the terminal goal for each axis.
  double dist_x = std::abs(min_pos[0] - max_pos[0]);
  double dist_y = std::abs(min_pos[1] - max_pos[1]);
  double dist_z = std::abs(min_pos[2] - max_pos[2]);

  // Update the map size based on the min and max positions.
  wdx_ = std::max(dist_x + 2 * dynamic_buffer_x, par_.min_wdx);
  wdy_ = std::max(dist_y + 2 * dynamic_buffer_y, par_.min_wdy);
  wdz_ = std::max(dist_z + 2 * dynamic_buffer_z, par_.min_wdz);

  // Compute the base map center as the midpoint between the min and max positions.
  map_center_ = (min_pos + max_pos) / 2.0;
}

// ----------------------------------------------------------------------------

bool SANDO::checkPointWithinMap(const Eigen::Vector3d& point) const {
  // Check if the point is within the map boundaries for each axis
  return (std::abs(point[0] - map_center_[0]) <= wdx_ / 2.0) &&
         (std::abs(point[1] - map_center_[1]) <= wdy_ / 2.0) &&
         (std::abs(point[2] - map_center_[2]) <= wdz_ / 2.0);
}

// ----------------------------------------------------------------------------

void SANDO::getStaticPushPoints(vec_Vecf<3>& static_push_points) {
  static_push_points = static_push_points_;
}

// ----------------------------------------------------------------------------

void SANDO::getLocalGlobalPath(
    vec_Vecf<3>& local_global_path, vec_Vecf<3>& local_global_path_after_push) {
  local_global_path = local_global_path_;
  local_global_path_after_push = local_global_path_after_push_;
}

// ----------------------------------------------------------------------------

void SANDO::getGlobalPath(vec_Vecf<3>& global_path) {
  mtx_global_path_.lock();
  global_path = global_path_;
  mtx_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getOriginalGlobalPath(vec_Vecf<3>& original_global_path) {
  mtx_original_global_path_.lock();
  original_global_path = original_global_path_;
  mtx_original_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getFreeGlobalPath(vec_Vecf<3>& free_global_path) {
  free_global_path = free_global_path_;
}

// ----------------------------------------------------------------------------

void SANDO::resetData() {
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

  poly_out_whole_.clear();
  poly_out_safe_.clear();
  goal_setpoints_.clear();
  pwp_to_share_.clear();
  optimal_yaw_sequence_.clear();
  yaw_control_points_.clear();
  yaw_knots_.clear();
  cps_.clear();
}

// ----------------------------------------------------------------------------

void SANDO::retrieveData(
    double& final_g,
    double& global_planning_time,
    double& hgp_static_jps_time,
    double& hgp_check_path_time,
    double& hgp_dynamic_astar_time,
    double& hgp_recover_path_time,
    double& cvx_decomp_time,
    double& local_traj_computatoin_time,
    double& safety_check_time,
    double& safe_paths_time,
    double& yaw_sequence_time,
    double& yaw_fitting_time,
    double& successful_factor) {
  final_g = final_g_;
  global_planning_time = global_planning_time_;
  hgp_static_jps_time = hgp_static_jps_time_;
  hgp_check_path_time = hgp_check_path_time_;
  hgp_dynamic_astar_time = hgp_dynamic_astar_time_;
  hgp_recover_path_time = hgp_recover_path_time_;
  cvx_decomp_time = cvx_decomp_time_;
  local_traj_computatoin_time = local_traj_computation_time_;
  safe_paths_time = safe_paths_time_;
  safety_check_time = safety_check_time_;
  yaw_sequence_time = yaw_sequence_time_;
  yaw_fitting_time = yaw_fitting_time_;
  successful_factor = successful_factor_;
}

// ----------------------------------------------------------------------------

void SANDO::retrievePolytopes(
    vec_E<Polyhedron<3>>& poly_out_whole, vec_E<Polyhedron<3>>& poly_out_safe) {
  poly_out_whole = poly_out_whole_;
  poly_out_safe = poly_out_safe_;
}

// ----------------------------------------------------------------------------

void SANDO::retrieveGoalSetpoints(std::vector<RobotState>& goal_setpoints) {
  goal_setpoints = goal_setpoints_;
}

// ----------------------------------------------------------------------------

void SANDO::retrieveListSubOptGoalSetpoints(
    std::vector<std::vector<RobotState>>& list_subopt_goal_setpoints) {
  list_subopt_goal_setpoints = list_subopt_goal_setpoints_;
}

// ----------------------------------------------------------------------------

void SANDO::retrieveCPs(std::vector<Eigen::Matrix<double, 3, 4>>& cps) { cps = cps_; }

// ----------------------------------------------------------------------------

std::tuple<bool, bool> SANDO::replan(double last_replaning_computation_time, double current_time) {
  /* -------------------- Housekeeping -------------------- */

  MyTimer timer_housekeeping(true);

  // Reset Data
  resetData();

  // Check if we need to replan
  if (!checkReadyToReplan()) {
    std::cout << bold << red << "Planner is not ready to replan" << reset << std::endl;
    return std::make_tuple(false, false);
  }

  // Get states we need
  RobotState local_state, local_G_term, last_plan_state;
  getState(local_state);
  getGterm(local_G_term);
  getLastPlanState(last_plan_state);

  // Check if we need to replan based on the distance to the terminal goal
  if (!needReplan(local_state, local_G_term, last_plan_state)) return std::make_tuple(false, false);

  // Hover avoidance: check obstacles and potentially set evasion goal
  if (par_.hover_avoidance_enabled && (drone_status_ == DroneStatus::GOAL_REACHED ||
                                       drone_status_ == DroneStatus::HOVER_AVOIDING)) {
    if (!checkHoverAvoidance(current_time))
      return std::make_tuple(false, false);  // no avoidance needed, stay hovering
  }

  if (par_.debug_verbose)
    std::cout << "Housekeeping: " << timer_housekeeping.getElapsedMicros() / 1000.0 << " ms"
              << std::endl;

  /* -------------------- Global Planning -------------------- */

  MyTimer timer_global(true);
  vec_Vecf<3> global_path;
  if (!generateGlobalPath(global_path, current_time, last_replaning_computation_time)) {
    if (par_.debug_verbose)
      std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms"
                << std::endl;
    return std::make_tuple(false, false);
  }
  if (par_.debug_verbose)
    std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms"
              << std::endl;

  /* -------------------- Local Trajectory Optimization -------------------- */

  MyTimer timer_local(true);
  if (!planLocalTrajectory(global_path, last_replaning_computation_time)) {
    if (par_.debug_verbose)
      std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0
                << " ms" << std::endl;
    return std::make_tuple(false, true);
  }
  if (par_.debug_verbose)
    std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0
              << " ms" << std::endl;

  /* -------------------- Append to Plan -------------------- */

  MyTimer timer_append(true);
  if (!appendToPlan()) {
    if (par_.debug_verbose)
      std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms"
                << std::endl;
    return std::make_tuple(false, true);
  }
  if (par_.debug_verbose)
    std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms"
              << std::endl;

  /* -------------------- Final Housekeeping -------------------- */

  MyTimer timer_final(true);

  if (par_.debug_verbose)
    printf("\033[1;32mReplanning succeeded (factor=%.2f)\033[0m\n", successful_factor_);

  // Reset the replanning failure count
  replanning_failure_count_ = 0;
  if (par_.debug_verbose)
    std::cout << "Final Housekeeping: " << timer_final.getElapsedMicros() / 1000.0 << " ms"
              << std::endl;

  return std::make_tuple(true, true);
}

// ----------------------------------------------------------------------------

bool SANDO::generateGlobalPath(
    vec_Vecf<3>& global_path, double current_time, double last_replaning_computation_time) {
  // Get G and G_term
  RobotState local_G, local_G_term;
  getG(local_G);
  getGterm(local_G_term);

  // Declare local variables
  RobotState local_A;
  double A_time;

  // Find A and A_time
  if (!findAandAtime(local_A, A_time, current_time, last_replaning_computation_time)) {
    replanning_failure_count_++;
    return false;
  }

  // Set A and A_time
  setA(local_A);
  setA_time(A_time);

  // Compute G
  computeG(local_A, local_G_term, par_.horizon);

  // Update Map
  if (par_.sim_env == "fake_sim" || par_.sim_env == "rviz_only") {
    updateOccupancyMap(current_time);
  } else {
    updateMap(current_time);
  }

  // Set up the HGP planner (since updateVmax() needs to be called after setupHGPPlanner, we use
  // v_max_ from the last replan)
  hgp_manager_.setupHGPPlanner(
      par_.global_planner, par_.global_planner_verbose, map_res_, v_max_, par_.a_max, par_.j_max,
      par_.hgp_timeout_duration_ms, par_.max_num_expansion, par_.w_unknown, par_.w_align,
      par_.decay_len_cells, par_.w_side, par_.los_cells, par_.min_len, par_.min_turn);

  // Free start and goal if necessary
  if (par_.use_free_start) hgp_manager_.freeStart(local_A.pos, par_.free_start_factor);
  if (par_.use_free_goal) hgp_manager_.freeGoal(local_G.pos, par_.free_goal_factor);

  // Debug
  if (par_.debug_verbose) std::cout << "Solving HGP" << std::endl;

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav") {
    local_A.pos[2] = 1.0;
    local_G.pos[2] = 1.0;
  }

  // 1) Build a direction hint from the *previous* global path
  vec_Vecf<3> prev_global;
  getGlobalPath(prev_global);  // last successful global path

  Eigen::Vector3d dir_hint = (local_G.pos - local_A.pos).normalized();
  if (prev_global.size() >= 2) {
    Eigen::Vector3d s0 = prev_global[0];
    Eigen::Vector3d s1 = prev_global[1];
    Eigen::Vector3d seg = s1 - s0;
    if (seg.norm() > 1e-8) {
      dir_hint = seg.normalized();
    }
  } else {
    dir_hint = local_G.pos - local_A.pos;
    if (dir_hint.norm() > 1e-8) dir_hint.normalize();
  }

  // Keep ground robots planar
  if (par_.vehicle_type != "uav") dir_hint[2] = 0.0;

  // 2) Use this as the "start_vel" argument (magnitude doesn't matter; we use the direction)
  Vec3f start_dir_hint(dir_hint.x(), dir_hint.y(), dir_hint.z());

  // Solve HGP
  vec_Vecf<3> raw_global_path;
  if (!hgp_manager_.solveHGP(
          local_A.pos, start_dir_hint, local_G.pos, final_g_, par_.global_planner_heuristic_weight,
          A_time, global_path, raw_global_path)) {
    if (par_.debug_verbose)
      std::cout << bold << red << "HGP did not find a solution" << reset << std::endl;
    hgp_failure_count_++;
    replanning_failure_count_++;
    return false;
  }

  // Log replan details to file (only when debug_verbose is enabled)
  if (par_.debug_verbose) {
    static const std::string path = "/tmp/sando_goal_log.txt";
    std::ofstream f(path, std::ios::app);
    if (f.is_open()) {
      static auto t0 = std::chrono::steady_clock::now();
      double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      f << std::fixed << std::setprecision(3) << "[" << t << "s] REPLAN #" << num_replanning_
        << "\n"
        << "  A:          (" << local_A.pos.x() << ", " << local_A.pos.y() << ", "
        << local_A.pos.z() << ")\n"
        << "  G:          (" << local_G.pos.x() << ", " << local_G.pos.y() << ", "
        << local_G.pos.z() << ")\n"
        << "  G_term:     (" << local_G_term.pos.x() << ", " << local_G_term.pos.y() << ", "
        << local_G_term.pos.z() << ")\n"
        << "  dir_hint:   (" << dir_hint.x() << ", " << dir_hint.y() << ", " << dir_hint.z()
        << ")\n"
        << "  global_path (" << global_path.size() << " pts):";
      for (size_t i = 0; i < std::min(global_path.size(), (size_t)5); i++)
        f << " (" << global_path[i][0] << "," << global_path[i][1] << "," << global_path[i][2]
          << ")";
      if (global_path.size() > 5) f << " ...";
      f << "\n\n";
      f.close();
    }
  }

  // use this for map resizing
  mtx_global_path_.lock();
  global_path_ = global_path;
  mtx_global_path_.unlock();

  // For visualization
  mtx_original_global_path_.lock();
  original_global_path_ = raw_global_path;
  mtx_original_global_path_.unlock();

  // Make sure global path does not exceed (num_P + 1)
  if (global_path.size() > par_.num_P + 1) {
    // Trim the global path
    global_path.resize(par_.num_P + 1);
  }

  // Find global path with safe sub goal
  findSafeSubGoal(global_path);

  // Debug
  if (par_.debug_verbose) std::cout << "global_path.size(): " << global_path.size() << std::endl;

  // Get computation time
  hgp_manager_.getComputationTime(
      global_planning_time_, hgp_static_jps_time_, hgp_check_path_time_, hgp_dynamic_astar_time_,
      hgp_recover_path_time_);

  return true;
}

// ----------------------------------------------------------------------------

bool SANDO::planLocalTrajectory(vec_Vecf<3>& global_path, double last_replaning_computation_time) {
  // Get local_A, local_G and A_time
  RobotState local_A, local_G, local_E;
  double A_time;
  getA(local_A);
  getG(local_G);
  getA_time(A_time);

  // If the global path has exactly 2 points, subdivide the longest segment
  // by inserting midpoints until we reach at least 3 points.
  while (global_path.size() == 2) {
    // Find the midpoint of the single segment
    Eigen::Vector3d mid = (global_path[0] + global_path[1]) / 2.0;
    global_path.insert(global_path.begin() + 1, mid);
  }

  // If the global path's size is still < 3, we cannot proceed
  if (global_path.empty() || global_path.size() < 3) {
    replanning_failure_count_++;
    return false;
  }

  // Initialize flag
  bool optimization_succeeded = false;

  // Set local_E
  if (drone_status_ == DroneStatus::GOAL_REACHED || drone_status_ == DroneStatus::GOAL_SEEN ||
      drone_status_ == DroneStatus::HOVER_AVOIDING)
    local_E = local_G;
  else
    local_E.pos = global_path.back();

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav") {
    local_A.pos[2] = 1.0;
    local_E.pos[2] = 1.0;
  }

  /*
   * Parallelized Local Trajectory Optimization
   */

  // Reset whole trajectory planners to nominal state
  for (auto& solver : whole_traj_solver_ptrs_) solver->resetToNominalState();

  // Get the base map vector
  vec_Vec3f base_map;
  if (par_.sim_env == "gazebo" || par_.sim_env == "hardware") {
    hgp_manager_.getVecUnknownOccupied(base_map);
  } else if (par_.sim_env == "fake_sim") {
    hgp_manager_.getVecOccupied(base_map);
  }

  // Filter out floor voxels at z_min from base_map before corridor decomposition.
  // The floor is already enforced by the solver's z_min constraint and the
  // ellipsoid decomp's set_z_min_and_max(). Including floor voxels in the obstacle
  // set causes the ellipsoid to shrink in all axes (ellipsoidal coupling), producing
  // unnecessarily skinny corridors.
  {
    const float z_floor_thresh = static_cast<float>(par_.z_min) + static_cast<float>(par_.res);
    const size_t before = base_map.size();
    base_map.erase(
        std::remove_if(
            base_map.begin(), base_map.end(),
            [z_floor_thresh](const Vec3f& pt) { return pt.z() <= z_floor_thresh; }),
        base_map.end());
    if (par_.debug_verbose && base_map.size() != before)
      std::cout << "[replan] Filtered " << (before - base_map.size())
                << " floor voxels (z<=" << z_floor_thresh << ") from base_map" << std::endl;
  }

  // Get obst_pos and obst_bbox
  vec_Vecf<3> obst_pos;
  vec_Vecf<3> obst_bbox;
  {
    std::lock_guard<std::mutex> lk(mtx_obst_pos_);
    obst_pos = obst_pos_;
    obst_bbox = obst_bbox_;
  }
  // Compute an initial dt for the local trajectory optimization
  whole_traj_solver_ptrs_[0]->setX0(local_A);
  whole_traj_solver_ptrs_[0]->setXf(local_E);
  double initial_dt = whole_traj_solver_ptrs_[0]->getInitialDt();

  // Compute sub goal vector once
  std::vector<double> sub_goal;
  sub_goal.push_back(local_G.pos[0]);
  sub_goal.push_back(local_G.pos[1]);
  sub_goal.push_back(local_G.pos[2]);

  // Pre-compute convex decomposition if environment is static or dynamic_worst_case
  std::vector<LinearConstraint3D> shared_spatial_constraints;
  vec_E<Polyhedron<3>> shared_spatial_poly_out;
  bool use_precomputed_constraints =
      (par_.environment_assumption == "static" ||
       par_.environment_assumption == "dynamic_worst_case");

  if (use_precomputed_constraints) {
    const size_t P = (global_path.size() >= 2) ? (global_path.size() - 1) : 0;

    std::vector<double> seg_end_times;
    if (par_.environment_assumption == "dynamic_worst_case") {
      // Worst-case inflation: set ALL segment end times to the maximum possible time horizon
      // across all factor threads. This inflates every obstacle by obst_max_vel * max_time,
      // producing the most conservative corridors (ablation baseline).
      const double max_time_horizon =
          static_cast<double>(par_.num_N) * initial_dt * factors_.back();
      seg_end_times.assign(P, max_time_horizon);
    } else {
      // Static environment: compute seg_end_times based on worst-case trajectory time per spatial
      // segment
      seg_end_times = computeWorstSegEndTimesPoly(initial_dt, factors_[0], P);
    }

    // Run spatial convex decomposition once before threading
    if (!hgp_manager_.cvxEllipsoidDecomp(
            ellip_workers_[0], global_path, base_map, obst_pos, obst_bbox, seg_end_times,
            shared_spatial_constraints, shared_spatial_poly_out)) {
      std::cout << bold << red
                << "Precomputed spatial convex decomposition failed for static environment" << reset
                << std::endl;
      return false;
    }

    // Save the whole polytopes for visualization (available even if local traj optimization fails)
    poly_out_whole_ = shared_spatial_poly_out;
  }

  // Shared flag: when one thread succeeds, all others abort early
  auto any_thread_succeeded = std::make_shared<std::atomic<bool>>(false);

  std::vector<std::future<std::tuple<bool, double, double, double, vec_E<Polyhedron<3>>>>> futures;
  futures.reserve(factors_.size());

  // Time the parallel optimization section
  auto parallel_opt_start = std::chrono::steady_clock::now();

  for (size_t i = 0; i < factors_.size(); ++i) {
    const double factor = factors_[i];  // corresponding factor for solver i

    futures.push_back(std::async(
        std::launch::async,
        [this, i, factor, &global_path, local_A, local_E, sub_goal, A_time, initial_dt, &obst_pos,
         &obst_bbox, &base_map, use_precomputed_constraints, &shared_spatial_constraints,
         &shared_spatial_poly_out,
         any_thread_succeeded]() -> std::tuple<bool, double, double, double, vec_E<Polyhedron<3>>> {
          try {
            // Early exit: another thread already found a solution
            if (any_thread_succeeded->load(std::memory_order_relaxed))
              return {false, 0.0, 0.0, factor, vec_E<Polyhedron<3>>{}};

            double thread_gurobi_time = 0.0;
            double thread_convx_decomp_time = 0.0;
            vec_E<Polyhedron<3>> thread_poly_out_safe;

            // Per-worker decomp util (no sharing across worker index)
            EllipsoidDecomp3D& ellip = this->ellip_workers_[i];

            const bool result = generateLocalTrajectory(
                ellip, global_path, local_A, local_E, sub_goal, A_time, thread_gurobi_time,
                thread_convx_decomp_time, whole_traj_solver_ptrs_[i], factor, initial_dt, obst_pos,
                obst_bbox,
                base_map,  // base_uo snapshot
                thread_poly_out_safe,
                use_precomputed_constraints ? &shared_spatial_constraints : nullptr,
                use_precomputed_constraints ? &shared_spatial_poly_out : nullptr);

            // Signal other threads to stop
            if (result) any_thread_succeeded->store(true, std::memory_order_relaxed);

            return {
                result, thread_gurobi_time, thread_convx_decomp_time, factor, thread_poly_out_safe};
          } catch (const std::exception& ex) {
            std::cerr << "Exception in async task with factor " << factor << ": " << ex.what()
                      << std::endl;
            return {false, 0.0, 0.0, factor, vec_E<Polyhedron<3>>{}};
          }
        }));
  }

  // Poll futures for first success instead of blocking sequentially.
  // This ensures we react immediately when ANY thread finishes successfully,
  // rather than waiting for earlier (by index) threads to complete first.
  std::vector<bool> vec_optimization_succeeded;
  std::vector<std::vector<RobotState>> vec_goal_setpoints;
  std::vector<PieceWisePol> vec_pwp_to_share;
  std::vector<std::vector<Eigen::Matrix<double, 3, 4>>> vec_cps;
  std::vector<double> vec_gurobi_times;
  std::vector<double> vec_convx_decomp_times;
  std::vector<vec_E<Polyhedron<3>>> vec_poly_out_safe;

  const size_t num_factors = factors_.size();
  vec_optimization_succeeded.resize(num_factors, false);
  vec_goal_setpoints.resize(num_factors);
  vec_pwp_to_share.resize(num_factors);
  vec_cps.resize(num_factors);
  vec_gurobi_times.resize(num_factors, 0.0);
  vec_convx_decomp_times.resize(num_factors, 0.0);
  vec_poly_out_safe.resize(num_factors);

  std::vector<bool> collected(num_factors, false);
  size_t num_collected = 0;
  int winner_index = -1;

  // Poll until we find a winner or all futures are collected
  while (num_collected < num_factors) {
    for (size_t i = 0; i < num_factors; ++i) {
      if (collected[i]) continue;

      // Non-blocking check: is this future ready?
      if (futures[i].wait_for(std::chrono::microseconds(0)) != std::future_status::ready) continue;

      // Collect the result
      auto
          [result, thread_gurobi_time, thread_convx_decomp_time, thread_factor,
           thread_poly_out_safe] = futures[i].get();
      collected[i] = true;
      num_collected++;

      // Save polytopes for visualization even if the optimizer failed
      if (poly_out_safe_.empty() && !thread_poly_out_safe.empty())
        poly_out_safe_ = thread_poly_out_safe;

      if (!result) continue;

      // First success — immediately stop all other solvers
      for (size_t j = 0; j < num_factors; ++j) {
        if (j == i) continue;
        try {
          whole_traj_solver_ptrs_[j]->stopExecution();
        } catch (const std::exception& e) {
          std::cout << "it's likely that the solver has gurobi error and already released the "
                       "gurobi environment"
                    << std::endl;
          std::cerr << e.what() << '\n';
        }
      }

      // Get results from the successful solver
      whole_traj_solver_ptrs_[i]->fillGoalSetPoints();
      whole_traj_solver_ptrs_[i]->getGoalSetpoints(vec_goal_setpoints[i]);
      whole_traj_solver_ptrs_[i]->getPieceWisePol(vec_pwp_to_share[i]);
      whole_traj_solver_ptrs_[i]->getControlPoints(vec_cps[i]);  // Bezier control points
      vec_gurobi_times[i] = thread_gurobi_time;
      vec_convx_decomp_times[i] = thread_convx_decomp_time;
      vec_poly_out_safe[i] = thread_poly_out_safe;
      vec_optimization_succeeded[i] = true;
      winner_index = static_cast<int>(i);
    }

    // If we found a winner, still drain remaining futures (they should exit fast
    // due to stopExecution + any_thread_succeeded flag)
    if (winner_index >= 0 && num_collected < num_factors) {
      for (size_t i = 0; i < num_factors; ++i) {
        if (!collected[i]) {
          futures[i].get();  // These should return quickly since solvers were stopped
          collected[i] = true;
          num_collected++;
        }
      }
      break;
    }

    // Brief yield to avoid busy-spin
    std::this_thread::yield();
  }

  // Measure wall-clock time for the parallel optimization only
  auto parallel_opt_end = std::chrono::steady_clock::now();
  double parallel_opt_ms =
      std::chrono::duration<double, std::milli>(parallel_opt_end - parallel_opt_start).count();

  // Find the first successful optimization
  int successful_index = -1;
  for (size_t i = 0; i < vec_optimization_succeeded.size(); ++i) {
    if (vec_optimization_succeeded[i]) {
      optimization_succeeded = true;
      goal_setpoints_ = vec_goal_setpoints[i];
      pwp_to_share_ = vec_pwp_to_share[i];
      cps_ = vec_cps[i];
      local_traj_computation_time_ = parallel_opt_ms;
      cvx_decomp_time_ = vec_convx_decomp_times[i];
      successful_factor_ = factors_[i];
      poly_out_safe_ = vec_poly_out_safe[i];
      successful_index = i;
      break;  // Exit the loop after the first success
    }
  }

  if (optimization_succeeded) {
    // update list_subopt_goal_setpoints_ (vec_goal_setpoints without the successful one)
    list_subopt_goal_setpoints_.clear();
    list_subopt_goal_setpoints_.reserve(vec_goal_setpoints.size() - 1);
    for (size_t i = 0; i < vec_goal_setpoints.size(); ++i) {
      if (i != successful_index && !vec_goal_setpoints[i].empty()) {
        list_subopt_goal_setpoints_.push_back(std::move(vec_goal_setpoints[i]));
      }
    }

    // update the factors_ vector
    if (par_.use_dynamic_factor) {
      // Save the successful factor BEFORE clearing
      double successful_factor = factors_[successful_index];

      // clear factors_ first
      factors_.clear();
      factors_.reserve(num_dynamic_factors_);

      // Set the successful factor to be the mean of k-radius factors
      for (int i = 0; i < num_dynamic_factors_; i++) {
        double factor =
            successful_factor - par_.dynamic_factor_k_radius + i * par_.factor_constant_step_size;
        if (factor >= par_.factor_initial && factor <= par_.factor_final)
          factors_.push_back(factor);
      }

      if (!dynamic_factor_inital_sucess_) dynamic_factor_inital_sucess_ = true;
    }
  } else {
    // if the optimization failed, we increase the factors_ for next replanning
    if (par_.use_dynamic_factor) {
      // compute current mean of the factor window
      double current_mean = 0.0;
      for (size_t i = 0; i < factors_.size(); i++) current_mean += factors_[i];
      current_mean /= static_cast<double>(factors_.size());

      if (current_mean + par_.factor_constant_step_size > par_.factor_final) {
        // reset factors back to the initial window
        factors_.clear();
        factors_.reserve(num_dynamic_factors_);
        for (int i = 0; i < num_dynamic_factors_; i++) {
          double factor = par_.dynamic_factor_initial_mean - par_.dynamic_factor_k_radius +
                          i * par_.factor_constant_step_size;
          if (factor >= par_.factor_initial && factor <= par_.factor_final)
            factors_.push_back(factor);
        }
      } else {
        // shift all the factors in factors_ by factor_constant_step_size
        for (size_t i = 0; i < factors_.size(); i++) {
          factors_[i] = factors_[i] + par_.factor_constant_step_size;
        }
        // remove any factors that exceed factor_final
        factors_.erase(
            std::remove_if(
                factors_.begin(), factors_.end(),
                [this](double f) { return f > par_.factor_final; }),
            factors_.end());
      }
    }
  }

  return optimization_succeeded;
}

// ----------------------------------------------------------------------------

void SANDO::getPieceWisePol(PieceWisePol& pwp) { pwp = pwp_to_share_; }

// ----------------------------------------------------------------------------

std::vector<double> SANDO::computeWorstSegEndTimesPoly(
    double initial_dt, double factor, size_t num_seg) {
  std::vector<double> seg_end_times;
  seg_end_times.reserve(num_seg);

  if (num_seg == 0) return seg_end_times;

  const int P = std::max(0, par_.num_P);
  if (P <= 0) {
    // Fallback: still produce valid per-segment times
    const double dt = initial_dt * factor;
    double t_acc = 0.0;
    for (size_t i = 0; i < num_seg; ++i) {
      t_acc += dt;
      seg_end_times.push_back(t_acc);
    }
    return seg_end_times;
  }

  // Assign segments to polytopes under your rule, but using num_seg (NOT par_.num_N-1).
  // Number of last polytopes that can be guaranteed 1 segment:
  const int max_last_ones = P - 1;
  const int min_one = std::min<int>(max_last_ones, static_cast<int>(num_seg));

  std::vector<int> segments_per_poly(P, 0);

  // Last min_one polytopes get 1 segment each
  for (int k = 0; k < min_one; ++k) {
    const int p = (P - 1) - k;
    segments_per_poly[p] = 1;
  }

  // First polytope gets the remainder
  const int assigned_to_last = min_one;
  const int first_segments = static_cast<int>(num_seg) - assigned_to_last;
  if (first_segments > 0) segments_per_poly[0] += first_segments;

  // Convert to per-segment cumulative end times
  const double dt = initial_dt * factor;
  double t_acc = 0.0;

  size_t produced = 0;
  for (int p = 0; p < P && produced < num_seg; ++p) {
    const int k = segments_per_poly[p];
    for (int s = 0; s < k && produced < num_seg; ++s) {
      t_acc += dt;
      seg_end_times.push_back(t_acc);
      ++produced;
    }
  }

  // Safety fallback: if anything went odd, pad to length num_seg
  while (seg_end_times.size() < num_seg) {
    t_acc += dt;
    seg_end_times.push_back(t_acc);
  }

  return seg_end_times;
}

// ----------------------------------------------------------------------------

bool SANDO::generateLocalTrajectory(
    EllipsoidDecomp3D& ellip,
    const vec_Vecf<3>& global_path,
    const RobotState& local_A,
    const RobotState& local_E,
    const std::vector<double>& sub_goal,
    double A_time,
    double& gurobi_computation_time,
    double& cvx_decomp_time,
    std::shared_ptr<SolverGurobi>& whole_traj_solver_ptr,
    double factor,
    double initial_dt,
    const vec_Vecf<3>& obst_pos,
    const vec_Vecf<3>& obst_bbox,
    const vec_Vec3f& base_uo,
    vec_E<Polyhedron<3>>& poly_out_safe,
    const std::vector<LinearConstraint3D>* precomputed_spatial_constraints,
    const vec_E<Polyhedron<3>>* precomputed_spatial_poly_out) {
  // P: spatial corridor pieces (global segments)
  const size_t P = (global_path.size() >= 2) ? (global_path.size() - 1) : 0;
  if (P == 0) return false;

  // N: local trajectory segments (time layers)
  const size_t N = static_cast<size_t>(par_.num_N);
  if (N == 0) return false;

  // Local time layers: end time of local segment n
  // NOTE: this matches your solver's uniform dt assumption (dt = initial_dt * factor).
  const double dt_layer = initial_dt * factor;
  std::vector<double> time_end_times;
  time_end_times.reserve(N);
  for (size_t n = 0; n < N; ++n)
    time_end_times.push_back((static_cast<double>(n) + 1.0) * dt_layer);

  // Timer for computing the safe corridor
  MyTimer cvx_decomp_timer(true);

  // Check if we have precomputed spatial constraints (static environment)
  bool use_spatial_only =
      (precomputed_spatial_constraints != nullptr && precomputed_spatial_poly_out != nullptr);

  // Declare constraints outside if block so they're available for solver setup
  std::vector<std::vector<LinearConstraint3D>> l_constraints_by_time;
  std::vector<vec_E<Polyhedron<3>>> poly_out_by_time;  // [N][P]

  if (use_spatial_only) {
    // Static environment: use precomputed spatial-only constraints
    // Copy the spatial polytopes for visualization
    poly_out_safe = *precomputed_spatial_poly_out;
    cvx_decomp_time = 0.0;  // No decomposition time since we're using precomputed
  } else {
    // Dynamic environment: compute time-layered constraints for this thread
    if (!hgp_manager_.cvxEllipsoidDecompTimeLayered(
            ellip, global_path, base_uo, obst_pos, obst_bbox, time_end_times, l_constraints_by_time,
            poly_out_by_time)) {
      poly_out_safe.clear();
      return false;
    }

    cvx_decomp_time = cvx_decomp_timer.getElapsedMicros() / 1000.0;

    // Build poly_out_safe from all time layers for visualization
    poly_out_safe.clear();
    poly_out_safe.reserve(N * P);
    for (size_t n = 0; n < N; ++n) {
      for (size_t p = 0; p < P; ++p) {
        poly_out_safe.emplace_back(poly_out_by_time[n][p]);
      }
    }
  }

  // Initialize the solver.
  whole_traj_solver_ptr->setX0(local_A);  // Initial condition
  whole_traj_solver_ptr->setXf(local_E);  // Final condition

  // Set polytopes based on environment type
  if (use_spatial_only) {
    // Static environment: use spatial-only polytopes
    whole_traj_solver_ptr->setPolytopes(*precomputed_spatial_constraints);
  } else {
    // Dynamic environment: use time-layered polytopes
    whole_traj_solver_ptr->setPolytopesTimeLayered(l_constraints_by_time);
  }
  whole_traj_solver_ptr->setT0(A_time);             // Initial time (kept as-is)
  whole_traj_solver_ptr->setInitialDt(initial_dt);  // Initial dt

  // Solve the optimization problem.
  bool gurobi_error_detected = false;
  bool gurobi_result = whole_traj_solver_ptr->generateNewTrajectory(
      gurobi_error_detected, gurobi_computation_time, factor);

  // If a Gurobi error occurred, reset the solver and return.
  if (gurobi_error_detected) {
    whole_traj_solver_ptr = std::make_shared<SolverGurobi>();
    whole_traj_solver_ptr->initializeSolver(par_);
    return false;
  }

  // If no solution is found, return.
  if (!gurobi_result) {
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool SANDO::appendToPlan() {
  if (par_.debug_verbose)
    std::cout << "goal_setpoints_.size(): " << goal_setpoints_.size() << std::endl;

  // mutex lock
  mtx_plan_.lock();

  // get the size of the plan and plan_safe_paths
  int plan_size = plan_.size();

  // If the plan size is less than k_value_, which means we already passed point A, we cannot use
  // this plan
  if (plan_size < k_value_) {
    if (par_.debug_verbose)
      std::cout << bold << red << "(plan_size - k_value_) = " << (plan_size - k_value_) << " < 0"
                << reset << std::endl;
    k_value_ = std::max(1, plan_size - 1);  // Decrease k_value_ to plan_size - 1 but at least 1
  } else  // If the plan size is greater than k_value_, which means we haven't passed point A yet,
          // we can use this plan
  {
    plan_.erase(plan_.end() - k_value_, plan_.end());
    plan_.insert(plan_.end(), goal_setpoints_.begin(), goal_setpoints_.end());
  }

  // mutex unlock
  mtx_plan_.unlock();

  // k_value adaptation initialization
  if (!got_enough_replanning_) {
    if (store_computation_times_.size() < par_.num_replanning_before_adapt) {
      num_replanning_++;
    } else {
      startAdaptKValue();
      got_enough_replanning_ = true;
    }
  }

  return true;
}

// ----------------------------------------------------------------------------

void SANDO::getGterm(RobotState& G_term) {
  mtx_G_term_.lock();
  G_term = G_term_;
  mtx_G_term_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::setGterm(const RobotState& G_term) {
  mtx_G_term_.lock();
  G_term_ = G_term;
  mtx_G_term_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getG(RobotState& G) {
  mtx_G_.lock();
  G = G_;
  mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getE(RobotState& E) {
  mtx_E_.lock();
  E = E_;
  mtx_E_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::setG(const RobotState& G) {
  mtx_G_.lock();
  G_ = G;
  mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getA(RobotState& A) {
  mtx_A_.lock();
  A = A_;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::setA(const RobotState& A) {
  mtx_A_.lock();
  A_ = A;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getA_time(double& A_time) {
  mtx_A_time_.lock();
  A_time = A_time_;
  mtx_A_time_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::setA_time(double A_time) {
  mtx_A_time_.lock();
  A_time_ = A_time;
  mtx_A_time_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getState(RobotState& state) {
  mtx_state_.lock();
  state = state_;
  mtx_state_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getLastPlanState(RobotState& state) {
  mtx_plan_.lock();
  if (plan_.empty()) {
    mtx_plan_.unlock();
    getState(state);  // fallback to current state when no plan exists
    return;
  }
  state = plan_.back();
  mtx_plan_.unlock();
}

// ----------------------------------------------------------------------------

void SANDO::getTrajs(std::vector<std::shared_ptr<DynTraj>>& out) {
  std::lock_guard<std::mutex> lock(mtx_trajs_);
  out = trajs_;  // copies shared_ptr only, not expressions
}

// ----------------------------------------------------------------------------

void SANDO::cleanUpOldTrajs(double current_time) {
  std::lock_guard<std::mutex> lock(mtx_trajs_);

  // remove_if moves all “expired” to the end, then erase() chops them off
  trajs_.erase(
      std::remove_if(
          trajs_.begin(), trajs_.end(),
          [&](const std::shared_ptr<DynTraj>& t) {
            return (current_time - t->time_received) > par_.traj_lifetime;
          }),
      trajs_.end());
}

// ----------------------------------------------------------------------------

void SANDO::addTraj(std::shared_ptr<DynTraj> new_traj, double current_time) {
  // Always update existing trajectories (to keep time_received fresh and data current).
  // Only apply map/horizon filtering when adding a brand-new trajectory.
  {
    std::lock_guard<std::mutex> lock(mtx_trajs_);
    auto it = std::find_if(trajs_.begin(), trajs_.end(), [&](const std::shared_ptr<DynTraj>& t) {
      return t && t->id == new_traj->id;
    });

    if (it != trajs_.end()) {
      *it = new_traj;  // always update existing trajectory
      return;
    }
  }

  // New trajectory: only add if currently within map and horizon
  Eigen::Vector3d p = new_traj->eval(current_time);
  if (!checkPointWithinMap(p)) return;
  if ((p - state_.pos).norm() > par_.horizon) return;

  {
    std::lock_guard<std::mutex> lock(mtx_trajs_);
    trajs_.push_back(new_traj);
  }
}

// ----------------------------------------------------------------------------

void SANDO::updateState(RobotState data) {
  // If we are doing hardware and provide goal in global frame (e.g. vicon), we need to transform
  // the goal to the local frame

  if (par_.use_hardware && par_.provide_goal_in_global_frame &&
      !par_.state_already_in_global_frame) {
    // Apply transformation to position
    Eigen::Vector4d homo_pos(data.pos[0], data.pos[1], data.pos[2], 1.0);
    Eigen::Vector4d global_pos = init_pose_transform_ * homo_pos;
    data.pos = Eigen::Vector3d(global_pos[0], global_pos[1], global_pos[2]);

    // Apply rotation to velocity
    data.vel = init_pose_transform_rotation_ * data.vel;

    // Apply rotation to accel
    data.accel = init_pose_transform_rotation_ * data.accel;

    // Apply rotation to jerk
    data.jerk = init_pose_transform_rotation_ * data.jerk;

    // Apply yaw
    data.yaw += yaw_init_offset_;
  }

  mtx_state_.lock();
  state_ = data;
  mtx_state_.unlock();

  if (state_initialized_ == false || drone_status_ == DroneStatus::YAWING) {
    // create temporary state
    RobotState tmp;
    if (drone_status_ == DroneStatus::YAWING) {
      // During YAWING, command the FIXED start position so PX4 actively
      // pulls the drone back if it drifts. Using the drifting current
      // position would let the drone wander and change desired_yaw.
      tmp.pos = yaw_start_pos_;
    } else {
      tmp.pos = data.pos;
    }
    tmp.yaw = data.yaw;

    // Only seed previous_yaw_ on first-ever state.
    // During YAWING, previous_yaw_ is seeded once by setTerminalGoal() and
    // then driven forward by yaw() each goal tick.  Resetting it here every
    // state callback would collapse the step to a tiny w_max*dc delta that
    // PX4 ignores, so the drone never starts rotating.
    if (!state_initialized_) previous_yaw_ = data.yaw;

    // Push the state to the plan
    mtx_plan_.lock();
    plan_.clear();
    plan_.push_back(tmp);
    mtx_plan_.unlock();

    // Update Point A
    setA(tmp);

    // Update Point G
    setG(tmp);

    // Update the flag
    state_initialized_ = true;
  }
}

// ----------------------------------------------------------------------------

bool SANDO::getNextGoal(RobotState& next_goal) {
  // Check if the planner is initialized.
  // During YAWING we only need state + terminal goal (no map required — just rotating in place).
  if (drone_status_ == DroneStatus::YAWING) {
    if (!state_initialized_ || !terminal_goal_initialized_) return false;
  } else if (!checkReadyToReplan()) {
    return false;
  }

  // Pop the front of the plan
  next_goal.setZero();

  // If the plan is empty, return false
  mtx_plan_.lock();  // Lock the mutex
  auto local_plan = plan_;
  mtx_plan_.unlock();  // Unlock the mutex

  // Get the next goal
  next_goal = local_plan.front();

  // If there's more than one goal setpoint, pop the front
  if (local_plan.size() > 1) {
    mtx_plan_.lock();
    plan_.pop_front();
    mtx_plan_.unlock();
  }

  // ---- Yaw computation (BEFORE frame transform — everything in global frame) ----
  if (!(drone_status_ == DroneStatus::GOAL_REACHED)) {
    // Get the desired yaw
    // If the planner keeps failing, just keep spinning
    if (replanning_failure_count_ > par_.yaw_spinning_threshold &&
        drone_status_ != DroneStatus::HOVER_AVOIDING) {
      next_goal.yaw = previous_yaw_ + par_.yaw_spinning_dyaw * par_.dc;
      next_goal.dyaw = par_.yaw_spinning_dyaw;
      previous_yaw_ = next_goal.yaw;
    } else {
      // If the local_plan is small just use the previous yaw with no dyaw
      // Exception: during YAWING we always need to call getDesiredYaw (plan is
      // intentionally kept at 1 entry by updateState)
      if (local_plan.size() < 5 && drone_status_ != DroneStatus::YAWING &&
          drone_status_ != DroneStatus::HOVER_AVOIDING) {
        next_goal.yaw = previous_yaw_;
        next_goal.dyaw = 0.0;
      } else {
        // next_goal.vel is still in GLOBAL frame here, matching previous_yaw_
        getDesiredYaw(next_goal);
      }
    }

    next_goal.dyaw = std::clamp(next_goal.dyaw, -par_.w_max, par_.w_max);
  } else {
    next_goal.yaw = previous_yaw_;
    next_goal.dyaw = 0.0;
  }

  // ---- Frame transform (global → local for MAVROS) ----
  if (par_.use_hardware && par_.provide_goal_in_global_frame && init_pose_set_) {
    // Convert position from global to local frame
    Eigen::Vector4d homo_pos(next_goal.pos[0], next_goal.pos[1], next_goal.pos[2], 1.0);
    Eigen::Vector4d local_pos = init_pose_transform_inv_ * homo_pos;

    // Apply inverse rotation to velocity, accel, jerk
    Eigen::Vector3d local_vel = init_pose_transform_rotation_inv_ * next_goal.vel;
    Eigen::Vector3d local_accel = init_pose_transform_rotation_inv_ * next_goal.accel;
    Eigen::Vector3d local_jerk = init_pose_transform_rotation_inv_ * next_goal.jerk;

    next_goal.pos = Eigen::Vector3d(local_pos[0], local_pos[1], local_pos[2]);
    next_goal.vel = local_vel;
    next_goal.accel = local_accel;
    next_goal.jerk = local_jerk;

    // Convert yaw from global to local frame and wrap to [-pi, pi]
    next_goal.yaw -= yaw_init_offset_;
    sando_utils::angle_wrap(next_goal.yaw);
  }

  return true;
}

// ----------------------------------------------------------------------------

void SANDO::getDesiredYaw(RobotState& next_goal) {
  // YAWING/HOVER_AVOIDING: closed-loop diff from actual drone yaw (state_.yaw).
  // TRAVELING/GOAL_SEEN: open-loop diff from commanded reference (previous_yaw_).

  double desired_yaw = 0.0;

  switch (drone_status_) {
    case DroneStatus::YAWING: {
      mtx_G_term_.lock();
      RobotState G_term = G_term_;
      mtx_G_term_.unlock();
      // Use the fixed yaw-start position (not the potentially drifted
      // next_goal.pos) so that desired_yaw stays stable during rotation.
      desired_yaw = atan2(G_term.pos[1] - yaw_start_pos_[1], G_term.pos[0] - yaw_start_pos_[0]);
      break;
    }
    case DroneStatus::HOVER_AVOIDING: {
      // Face toward the hover position (p_hover_), unless too close (atan2 unstable)
      double dx = p_hover_.x() - next_goal.pos[0];
      double dy = p_hover_.y() - next_goal.pos[1];
      double dist_xy = std::sqrt(dx * dx + dy * dy);
      if (dist_xy < 0.3) {
        // Too close — hold current yaw
        next_goal.yaw = previous_yaw_;
        next_goal.dyaw = 0.0;
        return;
      }
      desired_yaw = atan2(dy, dx);
      break;
    }
    case DroneStatus::TRAVELING:
    case DroneStatus::GOAL_SEEN: {
      double speed_xy =
          std::sqrt(next_goal.vel[0] * next_goal.vel[0] + next_goal.vel[1] * next_goal.vel[1]);
      if (speed_xy < 0.01) {
        next_goal.yaw = previous_yaw_;
        next_goal.dyaw = 0.0;
        return;
      }
      desired_yaw = atan2(next_goal.vel[1], next_goal.vel[0]);
      break;
    }
    case DroneStatus::GOAL_REACHED:
      next_goal.dyaw = 0.0;
      next_goal.yaw = previous_yaw_;
      return;
  }

  if (drone_status_ == DroneStatus::YAWING) {
    // Closed-loop yaw from actual drone heading.
    RobotState local_state;
    getState(local_state);
    double diff = desired_yaw - local_state.yaw;
    sando_utils::angle_wrap(diff);

    // Convergence check: transition when within ~17 deg of target
    if (std::fabs(diff) < 0.3) {
      changeDroneStatus(DroneStatus::TRAVELING);
    }

    // Timeout: if yawing for > 5 seconds and roughly facing the right way, transition
    double yaw_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - yaw_start_time_).count();
    if (yaw_elapsed > 10.0 && std::fabs(diff) < 1.0) {
      changeDroneStatus(DroneStatus::TRAVELING);
    }

    // Step from previous_yaw_ (commanded reference) toward desired_yaw.
    // Using previous_yaw_ instead of local_state.yaw ensures the quaternion
    // marches steadily toward the target — PX4 tracks the quaternion position,
    // not the dyaw feedforward, so the commanded yaw must lead the drone.
    double diff_cmd = desired_yaw - previous_yaw_;
    sando_utils::angle_wrap(diff_cmd);
    double max_step = par_.w_max_yawing * par_.dc;
    double step = std::clamp(diff_cmd, -max_step, max_step);
    next_goal.yaw = previous_yaw_ + step;
    next_goal.dyaw = step / par_.dc;
    previous_yaw_ = next_goal.yaw;
  } else if (drone_status_ == DroneStatus::HOVER_AVOIDING) {
    // Open-loop yaw toward hover position (same pattern as YAWING).
    double diff_cmd = desired_yaw - previous_yaw_;
    sando_utils::angle_wrap(diff_cmd);
    double max_step = par_.w_max_yawing * par_.dc;
    double step = std::clamp(diff_cmd, -max_step, max_step);
    next_goal.yaw = previous_yaw_ + step;
    next_goal.dyaw = step / par_.dc;
    previous_yaw_ = next_goal.yaw;
  } else {
    // TRAVELING/GOAL_SEEN: open-loop from commanded reference (previous_yaw_)
    // for smooth convergence without overshoot.
    double diff = desired_yaw - previous_yaw_;
    sando_utils::angle_wrap(diff);
    yaw(diff, next_goal);
  }
}

// ----------------------------------------------------------------------------

void SANDO::yaw(double diff, RobotState& next_goal) {
  // Filter the yaw ANGLE directly instead of the rate.
  // Each step covers a fraction of the remaining error → exponential
  // convergence with zero overshoot. PX4's attitude controller handles
  // the actual rate tracking (just like RC controller yaw).
  double step = (1.0 - par_.alpha_filter_dyaw) * diff;

  // Clamp step size to enforce max yaw rate
  double max_step = par_.w_max * par_.dc;
  step = std::clamp(step, -max_step, max_step);

  next_goal.yaw = previous_yaw_ + step;
  next_goal.dyaw = step / par_.dc;  // feedforward rate for PX4
  previous_yaw_ = next_goal.yaw;
}

// ----------------------------------------------------------------------------

void SANDO::logGoalEvent(
    const std::string& event,
    const RobotState& drone,
    const RobotState& goal,
    const Eigen::Vector3d& G_projected) {
  if (!par_.debug_verbose) return;
  static const std::string path = "/tmp/sando_goal_log.txt";
  std::ofstream f(path, std::ios::app);
  if (!f.is_open()) return;

  auto now = std::chrono::steady_clock::now();
  static auto t0 = now;
  double t = std::chrono::duration<double>(now - t0).count();

  f << std::fixed << std::setprecision(3) << "[" << t << "s] " << event << "\n"
    << "  drone_pos:  (" << drone.pos.x() << ", " << drone.pos.y() << ", " << drone.pos.z() << ")\n"
    << "  drone_vel:  (" << drone.vel.x() << ", " << drone.vel.y() << ", " << drone.vel.z() << ")\n"
    << "  drone_yaw:  " << drone.yaw << "\n"
    << "  term_goal:  (" << goal.pos.x() << ", " << goal.pos.y() << ", " << goal.pos.z() << ")\n"
    << "  G_project:  (" << G_projected.x() << ", " << G_projected.y() << ", " << G_projected.z()
    << ")\n"
    << "  status:     " << static_cast<int>(drone_status_)
    << " (0=YAWING,1=TRAVELING,2=GOAL_SEEN,3=GOAL_REACHED)\n"
    << "\n";
  f.close();
}

void SANDO::setTerminalGoal(const RobotState& term_goal) {
  // Ignore duplicate goals — the goal_sender re-publishes every 2s for reliability,
  // but re-triggering YAWING clears the plan and stops the drone mid-flight.
  if (terminal_goal_initialized_) {
    RobotState current_gterm;
    getGterm(current_gterm);
    if ((current_gterm.pos - term_goal.pos).norm() < 0.1) return;  // same goal, skip
  }

  // Get the state
  RobotState local_state;
  getState(local_state);

  // If the drone is already in TRAVELING or GOAL_SEEN state (i.e. mid-flight),
  // smoothly update the terminal goal without stopping.  The replanning timer
  // will pick up the new goal on the next cycle and replan toward it.
  if (terminal_goal_initialized_ &&
      (drone_status_ == DroneStatus::TRAVELING || drone_status_ == DroneStatus::GOAL_SEEN)) {
    setGterm(term_goal);
    p_hover_ = term_goal.pos;

    // Project the terminal goal to the sphere for the next replan
    mtx_G_.lock();
    G_.pos = sando_utils::projectPointToSphere(local_state.pos, term_goal.pos, par_.horizon);
    mtx_G_.unlock();

    logGoalEvent("SMOOTH_UPDATE (mid-flight)", local_state, term_goal, G_.pos);

    // Go back to TRAVELING if we were in GOAL_SEEN (since we have a new goal now)
    if (drone_status_ == DroneStatus::GOAL_SEEN) changeDroneStatus(DroneStatus::TRAVELING);

    return;
  }

  // First goal or drone is in YAWING/GOAL_REACHED: full initialization
  // Re-initialize plan from current state so that the first replan's
  // A point reflects the actual drone position (not the stale position
  // from when state_initialized_ was first set, e.g. z=0 on the ground).
  {
    RobotState tmp;
    tmp.pos = local_state.pos;
    tmp.vel = local_state.vel;
    tmp.accel = local_state.accel;
    tmp.yaw = local_state.yaw;

    mtx_plan_.lock();
    plan_.clear();
    plan_.push_back(tmp);
    mtx_plan_.unlock();

    setA(tmp);
    setG(tmp);
  }

  // Set the terminal goal
  setGterm(term_goal);
  p_hover_ = term_goal.pos;

  // Reset previous_yaw_ to actual drone yaw so the first getDesiredYaw
  // during YAWING computes diff from the correct reference
  previous_yaw_ = local_state.yaw;

  // Reset replanning failure count so YAWING uses getDesiredYaw (not spinning mode)
  replanning_failure_count_ = 0;

  // Project the terminal goal to the sphere
  mtx_G_.lock();
  G_.pos = sando_utils::projectPointToSphere(local_state.pos, term_goal.pos, par_.horizon);
  mtx_G_.unlock();

  // Store the position and time where yawing starts so the drone holds this
  // point (not the continuously-drifting state) during rotation.
  yaw_start_pos_ = local_state.pos;
  yaw_start_time_ = std::chrono::steady_clock::now();

  // Start with YAWING: rotate to face terminal goal before planning
  // In interactive/sim mode, skip YAWING and go directly to TRAVELING
  if (par_.skip_initial_yawing) {
    changeDroneStatus(DroneStatus::TRAVELING);
    logGoalEvent("FULL_INIT (TRAVELING, skip_yaw)", local_state, term_goal, G_.pos);
  } else {
    changeDroneStatus(DroneStatus::YAWING);
    logGoalEvent("FULL_INIT (YAWING)", local_state, term_goal, G_.pos);
  }

  if (!terminal_goal_initialized_) terminal_goal_initialized_ = true;
}

// ----------------------------------------------------------------------------

void SANDO::changeDroneStatus(int new_status) {
  if (new_status == drone_status_) return;

  std::cout << "Changing DroneStatus from ";

  switch (drone_status_) {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
    case DroneStatus::HOVER_AVOIDING:
      std::cout << bold << "status_=HOVER_AVOIDING" << reset;
      break;
  }

  std::cout << " to ";

  switch (new_status) {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
    case DroneStatus::HOVER_AVOIDING:
      std::cout << bold << "status_=HOVER_AVOIDING" << reset;
      break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}

// ----------------------------------------------------------------------------

bool SANDO::checkReadyToReplan() {
  bool map_init = hgp_manager_.isMapInitialized();
  bool kdtree_ok = !par_.use_hardware || kdtree_map_initialized_;

  return state_initialized_ && terminal_goal_initialized_ && map_init && kdtree_ok;
}

// ----------------------------------------------------------------------------

void SANDO::updateMapPtr(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_map,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_unk) {
  // 1) Atomically store the incoming clouds
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    pclptr_map_ = pclptr_map;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_unk_);
    pclptr_unk_ = pclptr_unk;
  }

  // Build the KD-tree from the incoming occupied cloud so that
  // kdtree_map_initialized_ becomes true even before the first replan.
  // Without this, checkReadyToReplan() would block forever on hardware
  // because updateMap() (which also builds the kdtree) only runs inside
  // the replan loop — a chicken-and-egg problem.
  if (pclptr_map && !pclptr_map->points.empty() && !kdtree_map_initialized_) {
    std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
    kdtree_map_.setInputCloud(pclptr_map);
    kdtree_map_initialized_ = true;
  }

  if (!hgp_manager_.isMapInitialized()) {
    updateMap(0.0);
  }
}

// ----------------------------------------------------------------------------

void SANDO::updateMap(double current_time) {
  // Update the map size
  RobotState local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // Get dynamic obstacles' positions, bboxes, and traj_max_time
  vec_Vecf<3> obst_pos;
  vec_Vecf<3> obst_bbox;
  std::vector<vec_Vecf<3>> pred_samples;
  std::vector<float> pred_times;

  traj_max_time_ = computeObstPosAndTrajMaxTimeForMapUpdate(
      obst_pos, obst_bbox, pred_samples, pred_times, current_time);

  hgp_manager_.setDynamicPredictedSamples(pred_samples, pred_times);

  // time the map update
  MyTimer timer_map(true);

  // 2) map update
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    std::lock_guard<std::mutex> lk2(mtx_pclptr_unk_);

    hgp_manager_.updateMap(
        wdx_, wdy_, wdz_, map_center_, pclptr_map_, pclptr_unk_, obst_pos, obst_bbox,
        traj_max_time_);

    if (par_.debug_verbose)
      std::cout << "Map update time: " << timer_map.getElapsedMicros() / 1000.0 << " ms"
                << std::endl;

    // 3) Known‐space KD‐tree
    if (pclptr_map_ && !pclptr_map_->points.empty()) {
      std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
      kdtree_map_.setInputCloud(pclptr_map_);
      kdtree_map_initialized_ = true;
      hgp_manager_.updateVecOccupied(pclptr_to_vec(pclptr_map_));
    } else {
      RCLCPP_WARN(
          rclcpp::get_logger("sando"),
          "updateMap: member pclptr_map_ was null or empty; skipping KD-tree update");
    }
  }

  // 4) Unknown‐space KD‐tree
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_unk_);
    if (pclptr_unk_ && !pclptr_unk_->points.empty()) {
      std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);
      kdtree_unk_.setInputCloud(pclptr_unk_);
      kdtree_unk_initialized_ = true;
      // merge known into unknown vector
      hgp_manager_.updateVecUnknownOccupied(pclptr_to_vec(pclptr_unk_));
      hgp_manager_.insertVecOccupiedToVecUnknownOccupied();
    } else {
      RCLCPP_WARN(
          rclcpp::get_logger("sando"),
          "updateMap: member pclptr_unk_ was null or empty; skipping KD‐tree update");
    }
  }
}

// ----------------------------------------------------------------------------

void SANDO::updateOccupancyMapPtr(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_map) {
  // store the incoming clouds
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    pclptr_map_ = pclptr_map;
  }

  if (!hgp_manager_.isMapInitialized()) {
    updateOccupancyMap(0.0);
  }
}

// ----------------------------------------------------------------------------

void SANDO::updateOccupancyMap(double current_time) {
  // Update the map size
  RobotState local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // Get dynamic obstacles' positions, bboxes, and traj_max_time
  vec_Vecf<3> obst_pos;
  vec_Vecf<3> obst_bbox;
  std::vector<vec_Vecf<3>> pred_samples;
  std::vector<float> pred_times;

  traj_max_time_ = computeObstPosAndTrajMaxTimeForMapUpdate(
      obst_pos, obst_bbox, pred_samples, pred_times, current_time);

  hgp_manager_.setDynamicPredictedSamples(pred_samples, pred_times);

  // 2) map update (unlocked)
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>());

    hgp_manager_.updateMap(
        wdx_, wdy_, wdz_, map_center_, pclptr_map_, empty_pclptr_unk, obst_pos, obst_bbox,
        traj_max_time_);

    // 3) Known‐space KD‐tree
    if (pclptr_map_ && !pclptr_map_->points.empty()) {
      std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
      kdtree_map_.setInputCloud(pclptr_map_);
      kdtree_map_initialized_ = true;
      hgp_manager_.updateVecOccupied(pclptr_to_vec(pclptr_map_));
    } else {
      RCLCPP_WARN(
          rclcpp::get_logger("sando"),
          "updateMap: member pclptr_map_ was null or empty; skipping KD-tree update");
    }
  }
}

// ----------------------------------------------------------------------------

double SANDO::computeObstPosAndTrajMaxTimeForMapUpdate(
    vec_Vecf<3>& obst_pos,
    vec_Vecf<3>& obst_bbox,                  // bbox of each obstacle
    std::vector<vec_Vecf<3>>& pred_samples,  // [K][M]
    std::vector<float>& pred_times,          // [M], relative times from now
    double current_time) {
  obst_pos.clear();
  obst_bbox.clear();
  pred_samples.clear();
  pred_times.clear();

  std::vector<std::shared_ptr<DynTraj>> local_trajs;
  getTrajs(local_trajs);

  // 1) Filter obstacles and build obst_pos and obst_bbox in a consistent order
  std::vector<std::shared_ptr<DynTraj>> selected_trajs;
  selected_trajs.reserve(local_trajs.size());

  for (const auto& traj : local_trajs) {
    Eigen::Vector3d p = traj->eval(current_time);
    bool in_map = checkPointWithinMap(p);
    double dist = (p - state_.pos).norm();
    bool in_horizon = dist <= par_.horizon;
    if (!in_map || !in_horizon) {
      continue;
    }

    obst_pos.push_back(p);
    obst_bbox.push_back(traj->bbox);  // Extract bbox from DynTraj
    selected_trajs.push_back(traj);
  }

  // Update obst_pos_ and obst_bbox_
  {
    std::lock_guard<std::mutex> lock(mtx_obst_pos_);
    obst_pos_ = obst_pos;
    obst_bbox_ = obst_bbox;
  }

  // 2) Horizon for map update (your existing “worst possible”)
  const double Th = worst_traj_time_ * factors_.back();  // [s]
  if (!(Th > 0.0) || selected_trajs.empty()) return Th;

  // 3) Build time samples between [0, Th]
  const double dt = 0.5;  // [s]
  int M = static_cast<int>(std::ceil(Th / dt)) + 1;
  // keep it bounded for cost (heat-map build is O(#voxels * K * M))
  M = std::max(5, std::min(M, 10));

  pred_times.resize(M);
  for (int j = 0; j < M; ++j) {
    const double a = (M == 1) ? 0.0 : (double)j / (double)(M - 1);
    pred_times[j] = static_cast<float>(a * Th);  // relative time from now
  }

  // 4) Sample each obstacle trajectory at (current_time + pred_times[j])
  pred_samples.resize(selected_trajs.size());
  for (size_t k = 0; k < selected_trajs.size(); ++k) {
    pred_samples[k].resize(M);
    for (int j = 0; j < M; ++j) {
      const double t_abs = current_time + (double)pred_times[j];
      Eigen::Vector3d pk = selected_trajs[k]->eval(t_abs);

      // NOTE: We do NOT drop samples outside the map, because the heat map
      // will simply have no effect there. But we must avoid NaNs.
      if (!std::isfinite(pk.x()) || !std::isfinite(pk.y()) || !std::isfinite(pk.z())) {
        // fallback: use current position (safe default)
        pk = selected_trajs[k]->eval(current_time);
      }

      pred_samples[k][j] = pk;
    }
  }

  return Th;
}

// ----------------------------------------------------------------------------

std::shared_ptr<sando::VoxelMapUtil> SANDO::getMapUtilSharedPtr() {
  return hgp_manager_.getMapUtilSharedPtr();
}

// ----------------------------------------------------------------------------

void SANDO::setInitialPose(const geometry_msgs::msg::TransformStamped& init_pose) {
  init_pose_ = init_pose;

  // Extract and normalize quaternion
  Eigen::Quaterniond q(
      init_pose_.transform.rotation.w, init_pose_.transform.rotation.x,
      init_pose_.transform.rotation.y, init_pose_.transform.rotation.z);
  q.normalize();

  Eigen::Vector3d t(
      init_pose_.transform.translation.x, init_pose_.transform.translation.y,
      init_pose_.transform.translation.z);

  // Rotation matrix and its transpose (= inverse for orthogonal matrices)
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix3d R_T = R.transpose();

  // Forward transform: local → global  [R | t; 0 0 0 1]
  init_pose_transform_ = Eigen::Matrix4d::Identity();
  init_pose_transform_.block<3, 3>(0, 0) = R;
  init_pose_transform_.block<3, 1>(0, 3) = t;

  // Inverse transform: global → local  [R^T | -R^T * t; 0 0 0 1]
  // Using the analytic SE(3) inverse instead of general matrix inverse
  init_pose_transform_inv_ = Eigen::Matrix4d::Identity();
  init_pose_transform_inv_.block<3, 3>(0, 0) = R_T;
  init_pose_transform_inv_.block<3, 1>(0, 3) = -R_T * t;

  // Store rotation matrices
  init_pose_transform_rotation_ = R;
  init_pose_transform_rotation_inv_ = R_T;

  // Yaw offset
  yaw_init_offset_ = std::atan2(R(1, 0), R(0, 0));

  // Sanity check: M_inv * init_pos should give (0,0,0) in local frame
  Eigen::Vector4d t_homo(t.x(), t.y(), t.z(), 1.0);
  Eigen::Vector4d local_origin = init_pose_transform_inv_ * t_homo;
  double sanity_err = local_origin.head<3>().norm();
  if (sanity_err < 0.1) {
    std::cout << bold << green << "****** [SANDO] READY TO FLY ******" << reset << std::endl;
  } else {
    std::cout << "\033[1;31m"
              << "****** [SANDO] TRANSFORM SANITY CHECK FAILED ******"
              << "\033[0m" << std::endl;
    std::cout << "\033[1;31m"
              << "inv * init_pos = (" << local_origin[0] << ", " << local_origin[1] << ", "
              << local_origin[2] << ") [should be ~(0,0,0), err=" << sanity_err << "]"
              << "\033[0m" << std::endl;
  }

  init_pose_set_ = true;
}

// ----------------------------------------------------------------------------

// Apply the initial pose transformation to the pwp
void SANDO::applyInitiPoseTransform(PieceWisePol& pwp) {
  // Loop thru the intervals
  for (int i = 0; i < pwp.coeff_x.size(); i++) {
    // Loop thru a, b, c, and d
    for (int j = 0; j < 4; j++) {
      Eigen::Vector4d coeff;
      coeff[0] = pwp.coeff_x[i][j];
      coeff[1] = pwp.coeff_y[i][j];
      coeff[2] = pwp.coeff_z[i][j];
      coeff[3] = 1.0;

      // Apply multiplication
      coeff = init_pose_transform_ * coeff;

      // cout agent frame pose
      pwp.coeff_x[i][j] = coeff[0];
      pwp.coeff_y[i][j] = coeff[1];
      pwp.coeff_z[i][j] = coeff[2];
    }
  }
}

// ----------------------------------------------------------------------------

// Apply the inverse of initial pose transformation to the pwp
void SANDO::applyInitiPoseInverseTransform(PieceWisePol& pwp) {
  // Loop thru the intervals
  for (int i = 0; i < pwp.coeff_x.size(); i++) {
    // Loop thru a, b, c, and d
    for (int j = 0; j < 4; j++) {
      Eigen::Vector4d coeff;
      coeff[0] = pwp.coeff_x[i][j];
      coeff[1] = pwp.coeff_y[i][j];
      coeff[2] = pwp.coeff_z[i][j];
      coeff[3] = 1.0;

      // Apply multiplication
      coeff = init_pose_transform_inv_ * coeff;

      pwp.coeff_x[i][j] = coeff[0];
      pwp.coeff_y[i][j] = coeff[1];
      pwp.coeff_z[i][j] = coeff[2];
    }
  }
}

// ----------------------------------------------------------------------------

bool SANDO::goalReachedCheck() {
  if (checkReadyToReplan() && (drone_status_ == DroneStatus::GOAL_REACHED ||
                               drone_status_ == DroneStatus::HOVER_AVOIDING)) {
    return true;
  }
  return false;
}

// ----------------------------------------------------------------------------

bool SANDO::checkHoverAvoidance(double current_time) {
  RobotState local_state;
  getState(local_state);

  // Get current obstacle positions from trajs_
  std::vector<std::shared_ptr<DynTraj>> local_trajs;
  getTrajs(local_trajs);

  // Helper: check if a point is within d_trigger of any obstacle.
  // When lookahead=true, samples over a future time window to catch periodic orbits
  // (used for evasion goal validation). When lookahead=false, only checks the
  // obstacle's current position (used for return-to-hover decisions so the drone
  // returns as soon as the obstacle is visually clear).
  const double lookahead_window = 15.0;  // seconds into the future
  const double lookahead_step = 0.5;     // sampling interval
  auto isPointThreatened = [&](const Eigen::Vector3d& pt, bool lookahead = true) -> bool {
    for (size_t i = 0; i < local_trajs.size(); ++i) {
      // Always check the agent's actual reported position first
      if ((pt - local_trajs[i]->current_pos).norm() < par_.hover_avoidance_d_trigger) return true;

      // If lookahead enabled, also sample the predicted trajectory.
      // For PWP trajectories (agents), limit to the trajectory's valid time range
      // to avoid using stale endpoint extrapolation as a "future prediction".
      // For analytic trajectories (obstacles), use the full lookahead window.
      if (lookahead) {
        double t_end_lookahead = current_time + lookahead_window;
        if (local_trajs[i]->mode == DynTraj::Mode::Piecewise &&
            !local_trajs[i]->pwp.times.empty()) {
          t_end_lookahead = std::min(t_end_lookahead, local_trajs[i]->pwp.times.back());
        }
        for (double t = current_time; t <= t_end_lookahead; t += lookahead_step) {
          Eigen::Vector3d p_obs = local_trajs[i]->eval(t);
          if ((pt - p_obs).norm() < par_.hover_avoidance_d_trigger) return true;
        }
      }
    }
    return false;
  };

  // Compute repulsion vector from drone's current position
  Eigen::Vector3d n_total = Eigen::Vector3d::Zero();
  double closest_dist = 1e9;
  for (const auto& traj : local_trajs) {
    // Use the agent's actual reported position for repulsion
    Eigen::Vector3d p_obs = traj->current_pos;
    Eigen::Vector3d r_i = local_state.pos - p_obs;
    double dist_i = r_i.norm();
    closest_dist = std::min(closest_dist, dist_i);

    if (dist_i < par_.hover_avoidance_d_trigger && dist_i > 1e-6) {
      double w_i = 1.0 / (dist_i * dist_i);
      n_total += w_i * (r_i / dist_i);
    }
  }

  if (n_total.norm() > par_.hover_avoidance_min_repulsion_norm) {
    // Store hover position once when first entering avoidance
    if (drone_status_ != DroneStatus::HOVER_AVOIDING) {
      RobotState temp_gterm;
      getGterm(temp_gterm);
      p_hover_ = temp_gterm.pos;
      changeDroneStatus(DroneStatus::HOVER_AVOIDING);
    }

    // Compute evasion goal — push away from current position (not p_hover_)
    Eigen::Vector3d direction = n_total.normalized();

    // 2D mode: zero out vertical component so avoidance stays at current altitude
    if (par_.hover_avoidance_2d) direction.z() = 0.0;

    // Re-normalize after zeroing z (guard against degenerate case)
    if (direction.norm() < 1e-6)
      direction = Eigen::Vector3d(1.0, 0.0, 0.0);
    else
      direction.normalize();

    Eigen::Vector3d p_evasion = local_state.pos + par_.hover_avoidance_h * direction;

    // In 2D mode keep the drone's current altitude; otherwise clamp to safe range
    if (par_.hover_avoidance_2d)
      p_evasion.z() = local_state.pos.z();
    else
      p_evasion.z() = std::max(par_.z_min + 0.5, std::min(p_evasion.z(), par_.z_max - 0.5));

    // Reject evasion goal if it's still inside an obstacle's d_trigger
    if (isPointThreatened(p_evasion, true)) {
      // Try rotated directions to escape
      const std::vector<double> angles = {M_PI / 6, -M_PI / 6, M_PI / 3, -M_PI / 3,
                                          M_PI / 2, -M_PI / 2, M_PI};
      bool found_safe = false;
      for (double angle : angles) {
        double cos_a = std::cos(angle), sin_a = std::sin(angle);
        Eigen::Vector3d rotated_dir(
            direction.x() * cos_a - direction.y() * sin_a,
            direction.x() * sin_a + direction.y() * cos_a, direction.z());
        Eigen::Vector3d candidate =
            local_state.pos + par_.hover_avoidance_h * rotated_dir.normalized();
        if (par_.hover_avoidance_2d)
          candidate.z() = local_state.pos.z();
        else
          candidate.z() = std::max(par_.z_min + 0.5, std::min(candidate.z(), par_.z_max - 0.5));
        if (!isPointThreatened(candidate, true) && !checkIfPointOccupied(Vec3f(candidate))) {
          p_evasion = candidate;
          found_safe = true;
          break;
        }
      }
      if (!found_safe) return false;  // can't find safe evasion point, stay put
    }

    // Also reject if evasion point hits a static obstacle
    if (checkIfPointOccupied(Vec3f(p_evasion))) {
      const std::vector<double> angles = {M_PI / 6,  -M_PI / 6, M_PI / 3,
                                          -M_PI / 3, M_PI / 2,  -M_PI / 2};
      bool found_free = false;
      for (double angle : angles) {
        double cos_a = std::cos(angle), sin_a = std::sin(angle);
        Eigen::Vector3d rotated_dir(
            direction.x() * cos_a - direction.y() * sin_a,
            direction.x() * sin_a + direction.y() * cos_a, direction.z());
        Eigen::Vector3d candidate =
            local_state.pos + par_.hover_avoidance_h * rotated_dir.normalized();
        if (par_.hover_avoidance_2d)
          candidate.z() = local_state.pos.z();
        else
          candidate.z() = std::max(par_.z_min + 0.5, std::min(candidate.z(), par_.z_max - 0.5));
        if (!checkIfPointOccupied(Vec3f(candidate)) && !isPointThreatened(candidate, true)) {
          p_evasion = candidate;
          found_free = true;
          break;
        }
      }
      if (!found_free) return false;
    }

    // Set evasion goal
    RobotState evasion_goal;
    evasion_goal.setPos(p_evasion.x(), p_evasion.y(), p_evasion.z());
    setGterm(evasion_goal);

    mtx_G_.lock();
    G_.pos = sando_utils::projectPointToSphere(local_state.pos, p_evasion, par_.horizon);
    mtx_G_.unlock();

    return true;  // continue with replanning
  } else if (drone_status_ == DroneStatus::HOVER_AVOIDING) {
    // Obstacles cleared from drone's current position.
    // Only return to p_hover_ if it's also clear of all obstacles.
    if (isPointThreatened(p_hover_, false)) {
      // p_hover_ is still unsafe — stay at current position and keep waiting.
      // Do NOT overwrite p_hover_; the obstacle will eventually move away.
      return false;
    }

    // Safe to return to original hover position
    RobotState hover_goal;
    hover_goal.setPos(p_hover_.x(), p_hover_.y(), p_hover_.z());
    setGterm(hover_goal);

    mtx_G_.lock();
    G_.pos = sando_utils::projectPointToSphere(local_state.pos, p_hover_, par_.horizon);
    mtx_G_.unlock();

    // Stay in HOVER_AVOIDING — drone is back at hover position but keeps monitoring
    return true;
  }

  // No obstacles nearby — stay in HOVER_AVOIDING (ready to dodge)
  return false;
}