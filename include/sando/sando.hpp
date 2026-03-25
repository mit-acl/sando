/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <math.h>
#include <pcl/kdtree/kdtree.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/StdVector>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <future>
#include <mutex>
#include <sando/gurobi_solver.hpp>
#include <sando/utils.hpp>
#include <thread>
#include <vector>

#include "hgp/hgp_manager.hpp"
#include "hgp/termcolor.hpp"
#include "sando/sando_type.hpp"
#include "timer.hpp"

enum { MAP = 0, UNKNOWN_MAP = 1 };
enum { RETURN_LAST_VERTEX = 0, RETURN_INTERSECTION = 1 };

using namespace sando;
using namespace termcolor;

// Type‐aliases
using Vec3 = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;
using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

// ------------------------------------------
// 2) The aligned‐allocator for local blocks:
// ------------------------------------------
// your PlaneBlock is still:
using PlaneBlock = std::pair<Eigen::Matrix<double, Eigen::Dynamic, 3>, Eigen::VectorXd>;

// replace your old ConstraintBlocks with:

// 1) an aligned‐allocator for a vector of PlaneBlock
using AlignedPlaneBlockVec = std::vector<PlaneBlock, Eigen::aligned_allocator<PlaneBlock>>;

// 2) then your top‐level blocks is a vector of those
using ConstraintBlocks =
    std::vector<AlignedPlaneBlockVec, Eigen::aligned_allocator<AlignedPlaneBlockVec>>;

enum DroneStatus { YAWING = 0, TRAVELING = 1, GOAL_SEEN = 2, GOAL_REACHED = 3, HOVER_AVOIDING = 4 };

class SANDO {
 public:
  // Methods

  /** @brief Constructs the SANDO planner and initializes solvers, factors, and map parameters.
   *  @param par Configuration parameters for the planner.
   */
  SANDO(Parameters par);

  /** @brief Determines whether replanning is needed based on distance to the terminal goal and drone status.
   *  @return True if replanning should be triggered.
   */
  bool needReplan(const RobotState& local_state, const RobotState& local_G_term,
                  const RobotState& last_plan_state);

  /** @brief Finds the starting state A and its timestamp for the next replan cycle.
   *  @param A Output starting state.
   *  @param A_time Output timestamp corresponding to state A.
   *  @param current_time Current ROS time.
   *  @param last_replaning_computation_time Computation time of the previous replan.
   *  @return True if a valid A was found within map bounds.
   */
  bool findAandAtime(RobotState& A, double& A_time, double current_time,
                     double last_replaning_computation_time);

  /** @brief Checks whether a point is occupied in the voxel map.
   *  @return True if the point is occupied.
   */
  bool checkIfPointOccupied(const Vec3f& point);

  /** @brief Checks whether a point is free (unoccupied) in the voxel map.
   *  @return True if the point is free.
   */
  bool checkIfPointFree(const Vec3f& point);

  /** @brief Truncates the global path at the first intersection with unknown space, backtracking to a safe point.
   *  @param global_path Global path to modify in place.
   */
  void findSafeSubGoal(vec_Vecf<3>& global_path);

  /** @brief Runs the full replanning pipeline: global path planning, local trajectory optimization, and plan appending.
   *  @param last_replaning_computation_time Computation time of the previous replan cycle.
   *  @param current_time Current ROS time.
   *  @return Tuple of (replan_succeeded, local_traj_attempted).
   */
  std::tuple<bool, bool> replan(double last_replaning_computation_time, double current_time);

  /** @brief Enables adaptive k-value computation based on accumulated replanning computation times. */
  void startAdaptKValue();

  /** @brief Gets the terminal goal state.
   *  @param G_term Output terminal goal state.
   */
  void getGterm(RobotState& G_term);

  /** @brief Sets the terminal goal state.
   *  @param G_term Terminal goal state to set.
   */
  void setGterm(const RobotState& G_term);

  /** @brief Gets the current subgoal projected onto the planning horizon sphere.
   *  @param G Output subgoal state.
   */
  void getG(RobotState& G);

  /** @brief Gets the local trajectory endpoint state E.
   *  @param E Output endpoint state.
   */
  void getE(RobotState& E);

  /** @brief Sets the subgoal state.
   *  @param G Subgoal state to set.
   */
  void setG(const RobotState& G);

  /** @brief Gets the starting state A used for the current replan.
   *  @param A Output starting state.
   */
  void getA(RobotState& A);

  /** @brief Sets the starting state A for global planning.
   *  @param A Starting state to set.
   */
  void setA(const RobotState& A);

  /** @brief Gets the timestamp of the starting state A.
   *  @param A_time Output timestamp.
   */
  void getA_time(double& A_time);

  /** @brief Sets the timestamp of the starting state A.
   *  @param A_time Timestamp to set.
   */
  void setA_time(double A_time);

  /** @brief Gets the current drone state (thread-safe).
   *  @param state Output current state.
   */
  void getState(RobotState& state);

  /** @brief Gets the list of tracked dynamic obstacle trajectories (thread-safe).
   *  @param out Output vector of shared pointers to dynamic trajectories.
   */
  void getTrajs(std::vector<std::shared_ptr<DynTraj>>& out);

  /** @brief Gets the last state in the current plan, or the current state if the plan is empty.
   *  @param state Output last plan state.
   */
  void getLastPlanState(RobotState& state);

  /** @brief Removes dynamic trajectories that have exceeded their lifetime.
   *  @param current_time Current ROS time.
   */
  void cleanUpOldTrajs(double current_time);

  /** @brief Adds a new dynamic trajectory or updates an existing one by matching trajectory ID.
   *  @param new_traj Shared pointer to the new trajectory.
   *  @param current_time Current ROS time used for map/horizon filtering.
   */
  void addTraj(std::shared_ptr<DynTraj> new_traj, double current_time);

  /** @brief Updates the drone state, applying frame transforms for hardware if configured.
   *  @param data New state data from odometry or Vicon.
   */
  void updateState(RobotState data);

  /** @brief Retrieves the next goal setpoint from the plan and computes the desired yaw.
   *  @param next_goal Output goal state with position, velocity, and yaw.
   *  @return True if a valid goal was retrieved.
   */
  bool getNextGoal(RobotState& next_goal);

  /** @brief Checks whether all prerequisites for replanning are satisfied (state, goal, map initialized).
   *  @return True if the planner is ready to replan.
   */
  bool checkReadyToReplan();

  /** @brief Sets a new terminal goal, triggering yawing or smooth mid-flight update as appropriate.
   *  @param term_goal Desired terminal goal state.
   */
  void setTerminalGoal(const RobotState& term_goal);

  /** @brief Logs a goal-related event to /tmp/sando_goal_log.txt for debugging.
   *  @param event Description of the event.
   *  @param drone Current drone state.
   *  @param goal Terminal goal state.
   *  @param G_projected Projected subgoal position on the horizon sphere.
   */
  void logGoalEvent(const std::string& event, const RobotState& drone, const RobotState& goal,
                    const Eigen::Vector3d& G_projected);

  /** @brief Changes the drone's flight status and logs the transition.
   *  @param new_status New DroneStatus enum value.
   */
  void changeDroneStatus(int new_status);

  /** @brief Computes the desired yaw angle and rate for the next goal based on drone status.
   *  @param next_goal Goal state to update with computed yaw and dyaw.
   */
  void getDesiredYaw(RobotState& next_goal);

  /** @brief Applies filtered yaw stepping toward a desired yaw difference, clamped by w_max.
   *  @param diff Angle difference to the desired yaw.
   *  @param next_goal Goal state to update with computed yaw and dyaw.
   */
  void yaw(double diff, RobotState& next_goal);

  /** @brief Computes the subgoal G by projecting the terminal goal onto a sphere of the given horizon radius around A.
   *  @param A Starting state.
   *  @param G_term Terminal goal state.
   *  @param horizon Projection sphere radius.
   */
  void computeG(const RobotState& A, const RobotState& G_term, double horizon);

  /** @brief Checks whether the drone has reached the goal or is in hover avoidance mode.
   *  @return True if the goal is reached or hover avoidance is active.
   */
  bool goalReachedCheck();

  /** @brief Checks for nearby dynamic obstacles during hover and computes an evasion goal if needed.
   *  @param current_time Current ROS time for evaluating obstacle trajectories.
   *  @return True if avoidance is needed and replanning should continue.
   */
  bool checkHoverAvoidance(double current_time);

  /** @brief Returns the stored hover position.
   *  @return Hover position as a 3D vector.
   */
  Eigen::Vector3d getHoverPos() const { return p_hover_; }

  /** @brief Returns the current drone status enum value.
   *  @return DroneStatus integer value.
   */
  int getDroneStatus() const { return drone_status_; }

  /** @brief Returns the distance trigger threshold for hover avoidance.
   *  @return Trigger distance in meters.
   */
  double getHoverAvoidanceDTrigger() const { return par_.hover_avoidance_d_trigger; }

  /** @brief Computes the local voxel map dimensions and center from the bounding positions.
   *  @param min_pos Minimum corner of the bounding region.
   *  @param max_pos Maximum corner of the bounding region.
   */
  void computeMapSize(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos);

  /** @brief Checks whether a point lies within the current local map boundaries.
   *  @return True if the point is inside the map.
   */
  bool checkPointWithinMap(const Eigen::Vector3d& point) const;

  /** @brief Gets the static push points used for path deformation around obstacles.
   *  @param static_push_points Output vector of push points.
   */
  void getStaticPushPoints(vec_Vecf<3>& static_push_points);

  /** @brief Gets the local segment of the global path before and after static push deformation.
   *  @param local_global_path Output path before push.
   *  @param local_global_path_after_push Output path after push.
   */
  void getLocalGlobalPath(vec_Vecf<3>& local_global_path,
                          vec_Vecf<3>& local_global_path_after_push);

  /** @brief Gets the current global path (thread-safe).
   *  @param global_path Output global path.
   */
  void getGlobalPath(vec_Vecf<3>& global_path);

  /** @brief Gets the original (unmodified) global path for visualization (thread-safe).
   *  @param original_global_path Output original global path.
   */
  void getOriginalGlobalPath(vec_Vecf<3>& original_global_path);

  /** @brief Gets the free (obstacle-cleared) global path.
   *  @param free_global_path Output free global path.
   */
  void getFreeGlobalPath(vec_Vecf<3>& free_global_path);

  /** @brief Computes worst-case cumulative segment end times for corridor inflation using polytope assignment.
   *  @param initial_dt Base time step for each segment.
   *  @param factor Time scaling factor.
   *  @param num_seg Number of spatial segments.
   *  @return Vector of cumulative end times, one per segment.
   */
  std::vector<double> computeWorstSegEndTimesPoly(double initial_dt, double factor, size_t num_seg);

  /** @brief Generates a local trajectory by performing convex decomposition and Gurobi optimization.
   *  @param ellip Ellipsoid decomposition worker (per-thread).
   *  @param global_path Global waypoints defining the spatial corridor seeds.
   *  @param local_A Starting state.
   *  @param local_E End state.
   *  @param sub_goal Subgoal position as [x, y, z].
   *  @param A_time Timestamp of the starting state.
   *  @param gurobi_computation_time Output Gurobi solve time in milliseconds.
   *  @param cvx_decomp_time Output convex decomposition time in milliseconds.
   *  @param whole_traj_solver_ptr Gurobi solver instance for this thread.
   *  @param factor Time scaling factor for this optimization attempt.
   *  @param initial_dt Base time step.
   *  @param obst_pos Current positions of dynamic obstacles.
   *  @param obst_bbox Bounding box half-extents of dynamic obstacles.
   *  @param base_uo Obstacle voxel positions for corridor decomposition.
   *  @param poly_out_safe Output safe corridor polytopes for visualization.
   *  @param precomputed_spatial_constraints Optional precomputed spatial constraints for static environments.
   *  @param precomputed_spatial_poly_out Optional precomputed spatial polytopes for static environments.
   *  @return True if the trajectory optimization succeeded.
   */
  bool generateLocalTrajectory(
      EllipsoidDecomp3D& ellip, const vec_Vecf<3>& global_path, const RobotState& local_A,
      const RobotState& local_E, const std::vector<double>& sub_goal, double A_time,
      double& gurobi_computation_time, double& cvx_decomp_time,
      std::shared_ptr<SolverGurobi>& whole_traj_solver_ptr, double factor, double initial_dt,
      const vec_Vecf<3>& obst_pos, const vec_Vecf<3>& obst_bbox, const vec_Vec3f& base_uo,
      vec_E<Polyhedron<3>>& poly_out_safe,
      const std::vector<LinearConstraint3D>* precomputed_spatial_constraints = nullptr,
      const vec_E<Polyhedron<3>>* precomputed_spatial_poly_out = nullptr);

  /** @brief Resets all internal timing, polytope, and trajectory data for a new replan cycle. */
  void resetData();

  /** @brief Retrieves timing and factor data from the most recent successful replan. */
  void retrieveData(double& final_g, double& global_planning_time, double& hgp_static_jps_time,
                    double& hgp_check_path_time, double& hgp_dynamic_astar_time,
                    double& hgp_recover_path_time, double& cvx_decomp_time,
                    double& local_traj_computatoin_time, double& safety_check_time,
                    double& safe_paths_time, double& yaw_sequence_time, double& yaw_fitting_time,
                    double& successful_factor);

  /** @brief Retrieves the whole and safe corridor polytopes from the last replan. */
  void retrievePolytopes(vec_E<Polyhedron<3>>& poly_out_whole, vec_E<Polyhedron<3>>& poly_out_safe);

  /** @brief Retrieves the goal setpoints generated by the last successful trajectory optimization. */
  void retrieveGoalSetpoints(std::vector<RobotState>& goal_setpoints);

  /** @brief Retrieves the list of sub-optimal goal setpoints from non-winning solver threads. */
  void retrieveListSubOptGoalSetpoints(std::vector<std::vector<RobotState>>& list_subopt_goal_setpoints);

  /** @brief Retrieves the Bezier control points from the last successful trajectory. */
  void retrieveCPs(std::vector<Eigen::Matrix<double, 3, 4>>& cps);

  /** @brief Runs the heatmap based global path planner (HGP) from state A to subgoal G, updating the global path.
   *  @param global_path Output global path waypoints.
   *  @param current_time Current ROS time.
   *  @param last_replaning_computation_time Computation time of the previous replan.
   *  @return True if a valid global path was found.
   */
  bool generateGlobalPath(vec_Vecf<3>& global_path, double current_time,
                          double last_replaning_computation_time);

  /** @brief Pushes the global path away from obstacles to produce a free path.
   *  @param global_path Global path to deform in place.
   *  @param free_global_path Output obstacle-free path.
   *  @param current_time Current ROS time.
   *  @return True if the push succeeded.
   */
  bool pushPath(vec_Vecf<3>& global_path, vec_Vecf<3>& free_global_path, double current_time);

  /** @brief Plans the local trajectory by running parallel Gurobi optimizations across multiple time-scaling factors.
   *  @param global_path Global path waypoints to use as corridor seeds.
   *  @param last_replaning_computation_time Computation time of the previous replan.
   *  @return True if at least one factor produced a feasible trajectory.
   */
  bool planLocalTrajectory(vec_Vecf<3>& global_path, double last_replaning_computation_time);

  /** @brief Appends the latest trajectory setpoints to the plan deque and manages k-value adaptation.
   *  @return True if the plan was successfully updated.
   */
  bool appendToPlan();

  /** @brief Sets the initial pose transform used for converting between local and global frames on hardware.
   *  @param init_pose Transform from local to global frame.
   */
  void setInitialPose(const geometry_msgs::msg::TransformStamped& init_pose);

  /** @brief Applies the initial pose forward transform (local to global) to a piecewise polynomial trajectory.
   *  @param pwp Piecewise polynomial to transform in place.
   */
  void applyInitiPoseTransform(PieceWisePol& pwp);

  /** @brief Applies the initial pose inverse transform (global to local) to a piecewise polynomial trajectory.
   *  @param pwp Piecewise polynomial to transform in place.
   */
  void applyInitiPoseInverseTransform(PieceWisePol& pwp);

  /** @brief Stores incoming point cloud pointers for occupancy and unknown maps, bootstrapping the KD-tree if needed.
   *  @param pclptr_map Occupied point cloud.
   *  @param pclptr_unk Unknown point cloud.
   */
  void updateMapPtr(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_map,
                    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_unk);

  /** @brief Updates the voxel map, KD-trees, and dynamic obstacle heat maps from stored point cloud data.
   *  @param current_time Current ROS time.
   */
  void updateMap(double current_time);

  /** @brief Stores an incoming occupancy-only point cloud pointer and triggers initial map update if needed.
   *  @param pclptr_map Occupied point cloud.
   */
  void updateOccupancyMapPtr(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_map);

  /** @brief Updates the occupancy-only voxel map and KD-tree (used for fake_sim and rviz_only modes).
   *  @param current_time Current ROS time.
   */
  void updateOccupancyMap(double current_time);

  /** @brief Computes current obstacle positions, bounding boxes, and predicted trajectory samples for map update.
   *  @param obst_pos Output current positions of dynamic obstacles.
   *  @param obst_bbox Output bounding box half-extents of each obstacle.
   *  @param pred_samples Output predicted position samples, indexed [obstacle][time_step].
   *  @param pred_times Output relative time offsets for each sample step.
   *  @param current_time Current ROS time.
   *  @return Maximum trajectory time horizon used for map inflation.
   */
  double computeObstPosAndTrajMaxTimeForMapUpdate(
      vec_Vecf<3>& obst_pos,
      vec_Vecf<3>& obst_bbox,                  // bbox of each obstacle
      std::vector<vec_Vecf<3>>& pred_samples,  // [K][M]
      std::vector<float>& pred_times,          // [M], relative times from now
      double current_time);

  /** @brief Gets the current piecewise polynomial trajectory for sharing with other agents.
   *  @param pwp Output piecewise polynomial.
   */
  void getPieceWisePol(PieceWisePol& pwp);

  /** @brief Returns a shared pointer to the internal voxel map utility.
   *  @return Shared pointer to VoxelMapUtil.
   */
  std::shared_ptr<sando::VoxelMapUtil> getMapUtilSharedPtr();

 private:
  // Parameters
  Parameters par_;          // Parameters of the planner
  HGPManager hgp_manager_;  // HGP Manager
  std::vector<std::shared_ptr<SolverGurobi>>
      whole_traj_solver_ptrs_;                   // L-BFGS solver pointer for the whole trajectory
  std::vector<std::shared_ptr<DynTraj>> trajs_;  // Dynamic trajectory
  Eigen::Vector3d v_max_3d_;                     // Maximum velocity
  Eigen::Vector3d a_max_3d_;                     // Maximum acceleration
  Eigen::Vector3d j_max_3d_;                     // Maximum jerk
  double v_max_;                                 // Maximum speed
  double max_dist_vertexes_;                     // Maximum velocity
  std::vector<double> factors_;                  // Factors for time allocation
  int num_dynamic_factors_;
  bool dynamic_factor_inital_sucess_ = false;
  double worst_traj_time_ = 0.0;  // Worst case trajectory time for pre-computation
  double traj_max_time_ = 0.0;    // Maximum trajectory time for map update

  // Flags
  bool state_initialized_ = false;          // State initialized
  bool terminal_goal_initialized_ = false;  // Terminal goal initialized
  bool use_adapt_k_value_ = false;          // Use adapt k value
  bool kdtree_map_initialized_ = false;     // Kd-tree for the map initialized
  bool kdtree_unk_initialized_ = false;     // Kd-tree for the map initialized

  // Data
  double final_g_ = 0.0;
  double global_planning_time_ = 0.0;
  double hgp_static_jps_time_ = 0.0;
  double hgp_check_path_time_ = 0.0;
  double hgp_dynamic_astar_time_ = 0.0;
  double hgp_recover_path_time_ = 0.0;
  double cvx_decomp_time_ = 0.0;
  double successful_factor_ = 0.0;
  double local_traj_computation_time_ = 0.0;
  double safe_paths_time_ = 0.0;
  double safety_check_time_ = 0.0;
  double yaw_sequence_time_ = 0.0;
  double yaw_fitting_time_ = 0.0;
  vec_E<Polyhedron<3>> poly_out_whole_;
  vec_E<Polyhedron<3>> poly_out_safe_;
  std::vector<RobotState> goal_setpoints_;
  std::vector<double> optimal_yaw_sequence_;
  std::vector<double> yaw_control_points_;
  std::vector<double> yaw_knots_;
  std::vector<Eigen::Matrix<double, 3, 4>> cps_;
  std::vector<Eigen::VectorXd> list_z_subopt_;
  std::vector<std::vector<Eigen::Vector3d>> list_initial_guess_wps_subopt_;
  std::vector<std::vector<RobotState>> list_subopt_goal_setpoints_;

  // Basis
  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_;
  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_inverse_;

  // Replanning-related variables
  RobotState state_;                                     // State for the drone
  RobotState G_;                                         // This goal is always inside of the map
  RobotState A_;                                         // Starting point of the drone
  double A_time_;                                   // Time of the starting point
  RobotState E_;                                         // The goal point of actual trajectory
  RobotState G_term_;                                    // Terminal goal
  std::deque<RobotState> plan_;                          // Plan for the drone
  std::deque<std::vector<RobotState>> plan_safe_paths_;  // Indicate if the state has a safe path
  double previous_yaw_ = 0.0;                       // Previous yaw
  double prev_dyaw_ = 0.0;                          // Previous dyaw
  double dyaw_filtered_ = 0.0;                      // Filtered dyaw
  PieceWisePol pwp_to_share_;                       // Piecewise polynomial to share
  vec_Vecf<3> obst_pos_;
  vec_Vecf<3> obst_bbox_;  // Bbox half-extents for each obstacle

  // Drone status
  int drone_status_ =
      DroneStatus::GOAL_REACHED;  // status_ can be TRAVELING, GOAL_SEEN, GOAL_REACHED

  // YAWING state: fixed position the drone should hold while rotating
  Eigen::Vector3d yaw_start_pos_ = Eigen::Vector3d::Zero();
  std::chrono::steady_clock::time_point yaw_start_time_;  // for YAWING timeout

  // Hover avoidance
  Eigen::Vector3d p_hover_ = Eigen::Vector3d::Zero();  // stored hover position
  bool hover_avoidance_active_ = false;                // whether we're currently avoiding

  // Mutex
  std::mutex mtx_plan_;                  // Mutex for the plan_
  std::mutex mtx_state_;                 // Mutex for the state_
  std::mutex mtx_G_;                     // Mutex for the G_
  std::mutex mtx_A_;                     // Mutex for the A_
  std::mutex mtx_A_time_;                // Mutex for the A_time_
  std::mutex mtx_G_term_;                // Mutex for the G_term_
  std::mutex mtx_E_;                     // Mutex for the E_
  std::mutex mtx_trajs_;                 // Mutex for the trajs_
  std::mutex mtx_solve_hgp_;             // Mutex for the solveHGP
  std::mutex mtx_global_path_;           // Mutex for the global_path_
  std::mutex mtx_original_global_path_;  // Mutex for the original_global_path_
  std::mutex mtx_kdtree_map_;            // Mutex for the map_
  std::mutex mtx_kdtree_unk_;            // Mutex for the unknown map_
  std::mutex mtx_obst_pos_;              // Mutex for the obst_pos_
  std::mutex mtx_pclptr_map_;            // Mutex for the pclptr_map_
  std::mutex mtx_pclptr_unk_;            // Mutex for the pclptr
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclptr_map_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclptr_unk_;

  // Map resolution
  double map_res_;

  // Counter for replanning failure
  int replanning_failure_count_ = 0;

  // Map size
  bool map_size_initialized_ = false;  // Map size initialized
  int hgp_failure_count_ = 0;          // HGP failure count used for map size adaptation

  // Communication delay
  std::unordered_map<int, double> comm_delay_map_;

  // Dynamic k value
  int num_replanning_ = 0;                       // Number of replanning
  bool got_enough_replanning_ = false;           // Check if tot enough replanning
  int k_value_ = 0;                              // k value
  std::vector<double> store_computation_times_;  // Store computation times
  double est_comp_time_ = 0.0;                   // Computation time estimation

  // Map size
  double wdx_, wdy_, wdz_;         // Width of the map
  Vec3f map_center_;               // Center of the map
  double current_dynamic_buffer_;  // Current dynamic buffer

  // Global path
  vec_Vecf<3> global_path_;
  vec_Vecf<3> original_global_path_;  // For visualization
  vec_Vecf<3> free_global_path_;

  // Static push
  vec_Vecf<3> static_push_points_;
  vec_Vecf<3> p_points_;
  vec_Vecf<3> local_global_path_;
  vec_Vecf<3> local_global_path_after_push_;

  // Initial pose
  geometry_msgs::msg::TransformStamped init_pose_;
  Eigen::Matrix4d init_pose_transform_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d init_pose_transform_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix4d init_pose_transform_inv_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d init_pose_transform_rotation_inv_ = Eigen::Matrix3d::Identity();
  double yaw_init_offset_ = 0.0;
  bool init_pose_set_ = false;

  // Safe corridor
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>,
              Eigen::aligned_allocator<Eigen::Matrix<double, Eigen::Dynamic, 3>>>
      A_stat_;
  std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> b_stat_;

  // kd-tree for the map
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_map_;  // kdtree of the point cloud of the occuppancy grid
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_unk_;  // kdtree of the point cloud of the unknown grid

  // Store data
  Eigen::VectorXd zopt_;
  double fopt_;

  // decomp ellip workers for each thread
  std::vector<EllipsoidDecomp3D> ellip_workers_;
};
