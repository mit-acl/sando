/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

// Convex Decomposition includes
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>

#include <decomp_rviz_plugins/data_ros_utils.hpp>

// Map includes
#include <hgp/read_map.hpp>

// HGP includes
#include <hgp/data_utils.hpp>
#include <hgp/hgp_planner.hpp>
#include <hgp/utils.hpp>

// Other includes
#include <Eigen/Dense>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "timer.hpp"

// prefix
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class HGPManager {
 public:
  HGPManager();

  /** @brief Configure the manager with planner parameters.
   *  @param par Parameter struct containing map, planner, and vehicle settings.
   */
  void setParameters(const Parameters& par);

  /** @brief Initialize the HGP planner with global planning configuration.
   *  @param global_planner Name of the global planner algorithm (e.g., "astar_heat", "sjps").
   *  @param global_planner_verbose Enable verbose output for the global planner.
   *  @param res Map resolution in meters.
   *  @param v_max Maximum velocity.
   *  @param a_max Maximum acceleration.
   *  @param j_max Maximum jerk.
   *  @param timeout_duration_ms Planning timeout in milliseconds.
   *  @param max_num_expansion Maximum number of node expansions allowed.
   *  @param w_unknown Cost weight for traversing unknown cells.
   *  @param w_align Alignment penalty weight for start-direction bias.
   *  @param decay_len_cells E-folding distance for alignment decay in cells.
   *  @param w_side Side tie-break weight for handedness preference.
   *  @param los_cells Line-of-sight check distance in cells.
   *  @param min_len Minimum segment length for path simplification.
   *  @param min_turn Minimum turn angle for path simplification.
   */
  void setupHGPPlanner(const std::string& global_planner, bool global_planner_verbose, double res,
                       double v_max, double a_max, double j_max, int timeout_duration_ms,
                       int max_num_expansion, double w_unknown, double w_align,
                       double decay_len_cells, double w_side, int los_cells = 3,
                       double min_len = 0.5, double min_turn = 10.0);

  /** @brief Retrieve all occupied cells from the map (thread-safe).
   *  @param occupied_cells Output vector of occupied cell positions.
   */
  void getOccupiedCells(vec_Vecf<3>& occupied_cells);

  /** @brief Retrieve all free cells from the map (thread-safe).
   *  @param free_cells Output vector of free cell positions.
   */
  void getFreeCells(vec_Vecf<3>& free_cells);

  /** @brief Retrieve occupied cells in the vicinity of a path for convex decomposition.
   *  @param occupied_cells Output vector of occupied cell positions.
   *  @param path Reference path used to define the vicinity region.
   *  @param use_for_safe_path If true, includes unknown cells as occupied.
   */
  void getOccupiedCellsForCvxDecomp(vec_Vecf<3>& occupied_cells, const vec_Vecf<3>& path,
                                    bool use_for_safe_path);

  /** @brief Retrieve dynamic occupied, free, and unknown cells for visualization.
   *  @param occupied_cells Output vector of occupied cell positions.
   *  @param free_cells Output vector of free cell positions.
   *  @param unknown_cells Output vector of unknown cell positions.
   *  @param current_time Current simulation time for dynamic obstacle state.
   */
  void getDynamicOccupiedCellsForVis(vec_Vecf<3>& occupied_cells, vec_Vecf<3>& free_cells,
                                     vec_Vecf<3>& unknown_cells, double current_time);

  /** @brief Update the voxel map from point cloud data and dynamic obstacle state.
   *  @param wdx Map window width in x (meters).
   *  @param wdy Map window width in y (meters).
   *  @param wdz Map window width in z (meters).
   *  @param center_map Center of the map window in world coordinates.
   *  @param pclptr Point cloud of occupied observations.
   *  @param pclptr_unk Point cloud of unknown-space observations.
   *  @param obst_pos Current positions of dynamic obstacles.
   *  @param obst_bbox Bounding box half-extents of dynamic obstacles.
   *  @param traj_max_time Maximum trajectory time horizon for dynamic inflation.
   */
  void updateMap(double wdx, double wdy, double wdz, const Vec3f& center_map,
                 const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr,
                 const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_unk,
                 const vec_Vecf<3>& obst_pos, const vec_Vecf<3>& obst_bbox, double traj_max_time);

  /** @brief Clear occupied voxels around the start position to ensure a valid planning origin.
   *  @param start_sent Start position (may be snapped to free space).
   *  @param factor Multiplier for the clearing radius relative to map resolution.
   */
  void freeStart(Vec3f& start_sent, double factor);

  /** @brief Clear occupied voxels around the goal position to ensure a valid planning target.
   *  @param goal_sent Goal position (may be snapped to free space).
   *  @param factor Multiplier for the clearing radius relative to map resolution.
   */
  void freeGoal(Vec3f& goal_sent, double factor);

  /** @brief Check whether a point lies in an occupied voxel.
   *  @param point Query position in world coordinates.
   *  @return True if the point is occupied.
   */
  bool checkIfPointOccupied(const Vec3f& point);

  /** @brief Run the HGP global planner from start to goal.
   *  @param start_sent Start position in world coordinates.
   *  @param start_vel Current velocity at start.
   *  @param goal_sent Goal position in world coordinates.
   *  @param final_g Output g-cost of the found path.
   *  @param weight Heuristic weight for the planner.
   *  @param current_time Current simulation time for dynamic cost evaluation.
   *  @param path Output simplified path.
   *  @param raw_path Output raw path before simplification.
   *  @return True if a valid path was found.
   */
  bool solveHGP(const Vec3f& start_sent, const Vec3f& start_vel, const Vec3f& goal_sent,
                double& final_g, double weight, double current_time, vec_Vecf<3>& path,
                vec_Vecf<3>& raw_path);

  /** @brief Truncate a path to include only waypoints in free space.
   *  @param path Input path to check.
   *  @param free_path Output path truncated at the first non-free waypoint.
   *  @return True if the free path has at least two waypoints.
   */
  bool checkIfPathInFree(const vec_Vecf<3>& path, vec_Vecf<3>& free_path);

  /** @brief Retrieve computation time breakdowns from the last planning call.
   *  @param global_planning_time Total global planning time (seconds).
   *  @param hgp_static_jps_time Time spent in static JPS (seconds).
   *  @param hgp_check_path_time Time spent checking path validity (seconds).
   *  @param hgp_dynamic_astar_time Time spent in dynamic A* (seconds).
   *  @param hgp_recover_path_time Time spent recovering the path (seconds).
   */
  void getComputationTime(double& global_planning_time, double& hgp_static_jps_time,
                          double& hgp_check_path_time, double& hgp_dynamic_astar_time,
                          double& hgp_recover_path_time);

  /** @brief Compute convex ellipsoid decomposition per path segment with time-varying obstacles.
   *  @param ellip Per-worker ellipsoid decomposition utility.
   *  @param path Global path waypoints (size P+1 yields P segments).
   *  @param base_uo Snapshot of static unknown+occupied points.
   *  @param obst_pos Dynamic obstacle positions.
   *  @param obst_bbox Dynamic obstacle bounding box half-extents.
   *  @param seg_end_times End time for each segment (size P).
   *  @param l_constraints Output linear constraints per segment.
   *  @param poly_out Output polyhedra per segment.
   *  @return True if decomposition succeeded for all segments.
   */
  bool cvxEllipsoidDecomp(
      EllipsoidDecomp3D& ellip,  // per-worker decomp util
      const vec_Vecf<3>& path,
      const vec_Vec3f& base_uo,  // snapshot of (unknown+occupied) or (occupied-only)
      const vec_Vecf<3>& obst_pos, const vec_Vecf<3>& obst_bbox,
      const std::vector<double>& seg_end_times, std::vector<LinearConstraint3D>& l_constraints,
      vec_E<Polyhedron<3>>& poly_out);

  /** @brief Compute time-layered convex decomposition for temporal safety corridors.
   *  @param ellip Per-worker ellipsoid decomposition utility.
   *  @param path Global path waypoints (size P+1).
   *  @param base_uo Snapshot of static unknown+occupied points.
   *  @param obst_pos Dynamic obstacle positions.
   *  @param obst_bbox Dynamic obstacle bounding box half-extents.
   *  @param time_end_times End times for each time layer (size N).
   *  @param l_constraints_by_time Output linear constraints indexed [N][P].
   *  @param poly_out_by_time Output polyhedra indexed [N][P].
   *  @return True if decomposition succeeded for all time layers and segments.
   */
  bool cvxEllipsoidDecompTimeLayered(
      EllipsoidDecomp3D& ellip, const vec_Vecf<3>& path, const vec_Vec3f& base_uo,
      const vec_Vecf<3>& obst_pos, const vec_Vecf<3>& obst_bbox,
      const std::vector<double>& time_end_times,
      std::vector<std::vector<LinearConstraint3D>>& l_constraints_by_time,
      std::vector<vec_E<Polyhedron<3>>>& poly_out_by_time);

  /** @brief Set predicted trajectory samples for dynamic obstacles used in heat map computation.
   *  @param pred_samples Predicted position samples per obstacle, aligned with pred_times.
   *  @param pred_times Time stamps corresponding to the predicted samples.
   */
  void setDynamicPredictedSamples(const std::vector<vec_Vecf<3>>& pred_samples,
                                  const std::vector<float>& pred_times);

  /** @brief Inflate dynamic obstacles and unknown boundary voxels into a point set for decomposition.
   *  @param pts Input/output point set; inflated points are appended.
   *  @param obst_pos Dynamic obstacle positions.
   *  @param obst_bbox Dynamic obstacle bounding box half-extents.
   *  @param traj_max_time Time horizon for motion-based inflation radius.
   */
  void obstacle_to_vec(vec_Vec3f& pts, const vec_Vecf<3>& obst_pos, const vec_Vecf<3>& obst_bbox,
                       double traj_max_time);

  /** @brief Check whether a point lies in a free voxel.
   *  @param point Query position in world coordinates.
   *  @return True if the point is free.
   */
  bool checkIfPointFree(const Vec3f& point) const;

  /** @brief Refresh the internal read map utility from the current map state. */
  void updateReadMapUtil();

  /** @brief Project each waypoint of a path to its closest free voxel.
   *  @param path Input path with potentially non-free waypoints.
   *  @param free_path Output path with all waypoints pushed to free space.
   */
  void pushPathIntoFreeSpace(const vec_Vecf<3>& path, vec_Vecf<3>& free_path);

  /** @brief Find the closest free voxel to a given point.
   *  @param point Query position in world coordinates.
   *  @param closest_free_point Output position of the nearest free voxel.
   */
  void findClosestFreePoint(const Vec3f& point, Vec3f& closest_free_point);

  /** @brief Count the number of unknown cells in the current map.
   *  @return Number of unknown cells.
   */
  int countUnknownCells() const;

  /** @brief Get the total number of cells in the voxel map.
   *  @return Total cell count.
   */
  int getTotalNumCells() const;

  /** @brief Update the maximum velocity used by the planner.
   *  @param v_max New maximum velocity.
   */
  void updateVmax(double v_max);

  /** @brief Remove redundant waypoints from a path using the underlying planner logic. */
  void cleanUpPath(vec_Vecf<3>& path);

  /** @brief Check whether the voxel map has been initialized with sensor data.
   *  @return True if the map is initialized.
   */
  bool isMapInitialized() const;

  /** @brief Check whether a point has any non-free neighbor in the voxel grid.
   *  @param point Query position in world coordinates.
   *  @return True if at least one neighbor is non-free.
   */
  bool checkIfPointHasNonFreeNeighbour(const Vec3f& point) const;

  /** @brief Get a thread-safe copy of the occupied point vector.
   *  @param vec_o Output vector of occupied points.
   */
  void getVecOccupied(vec_Vec3f& vec_o);

  /** @brief Update the internal occupied point vector (thread-safe).
   *  @param vec_o New occupied point vector.
   */
  void updateVecOccupied(const vec_Vec3f& vec_o);

  /** @brief Get a thread-safe copy of the unknown+occupied point vector.
   *  @param vec_uo Output vector of unknown and occupied points.
   */
  void getVecUnknownOccupied(vec_Vec3f& vec_uo);

  /** @brief Update the internal unknown+occupied point vector (thread-safe).
   *  @param vec_uo New unknown+occupied point vector.
   */
  void updateVecUnknownOccupied(const vec_Vec3f& vec_uo);

  /** @brief Append the occupied point vector into the unknown+occupied point vector (thread-safe). */
  void insertVecOccupiedToVecUnknownOccupied();

  /** @brief Get a thread-safe shared pointer to the underlying voxel map utility.
   *  @return Shared pointer to the VoxelMapUtil instance.
   */
  std::shared_ptr<sando::VoxelMapUtil> getMapUtilSharedPtr();

  // Helper: construct Veci<3> from 3 ints.
  static inline Veci<3> idxs_to_veci3(int x, int y, int z) {
    Veci<3> v;
    v << x, y, z;
    return v;
  }

  std::shared_ptr<sando::VoxelMapUtil> map_util_;
  std::shared_ptr<sando::VoxelMapUtil> map_util_for_planning_;
  std::unique_ptr<HGPPlanner> planner_ptr_;

 private:
  vec_Vec3f vec_o_;   // Vector that contains the occupied points
  vec_Vec3f vec_uo_;  // Vector that contains the unkown and occupied points

  // Mutex
  std::mutex mtx_map_util_;
  std::mutex mtx_vec_o_;
  std::mutex mtx_vec_uo_;

  // Convex decomposition
  std::vector<float> sfc_size_;

  // Parameters
  Parameters par_;
  double weight_;
  double res_, drone_radius_;
  double v_max_, a_max_, j_max_;
  Eigen::Vector3d v_max_3d_, a_max_3d_, j_max_3d_;
  double max_dist_vertexes_ = 2.0;
  bool use_shrinked_box_ = false;
  double shrinked_box_size_ = 0.0;

  // Constants
  const Eigen::Vector3d unitX_ = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d unitY_ = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d unitZ_ = Eigen::Vector3d::UnitZ();

  // Flats
  bool map_initialized_ = false;
};
