/* ----------------------------------------------------------------------------
 * Copyright (c) Anonymous Author
 * Anonymous Institution
 * All Rights Reserved
 * Authors: Anonymous
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file hgp_planner.h
 * @brief HGPPlanner
 */
#pragma once

#include <hgp/data_type.hpp>
#include <hgp/graph_search.hpp>
#include <hgp/map_util.hpp>

/**
 * @brief Abstract base for planning
 */
class HGPPlanner {
 public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug mode
   */
  HGPPlanner(
      std::string global_planner,
      bool verbose,
      double v_max,
      double a_max,
      double j_max,
      int hgp_timeout_duration_ms,
      double w_unknown,
      double w_align,
      double decay_len_cells,
      double w_side,
      int los_cells = 3,
      double min_len = 0.5,
      double min_turn = 10.0);

  /** @brief Set the voxel map utility used for collision checking.
   *  @param map_util Shared pointer to a 3D voxel map utility.
   */
  void setMapUtil(const std::shared_ptr<sando::MapUtil<3>>& map_util);
  /**
   * @brief Status of the planner
   *
   * 0 --- exit normally;
   * -1 --- no path found;
   * 1, 2 --- start or goal is not free.
   */
  int status();
  /** @brief Get the post-processed path.
   *  @return Cleaned-up waypoint path.
   */
  vec_Vecf<3> getPath();

  /** @brief Get the raw path directly from the graph search.
   *  @return Unmodified waypoint path.
   */
  vec_Vecf<3> getRawPath();

  /** @brief Set the planning timeout duration.
   *  @param timeout_duration_ms Timeout in milliseconds.
   */
  void setTimeOutDurationMs(int timeout_duration_ms);

  /** @brief Remove collinear intermediate waypoints from a path.
   *  @param path Input waypoint path.
   *  @return Path with redundant collinear points removed.
   */
  vec_Vecf<3> removeLinePts(const vec_Vecf<3>& path);

  /** @brief Remove unnecessary corner waypoints from a path.
   *  @param path Input waypoint path.
   *  @return Simplified path.
   */
  vec_Vecf<3> removeCornerPts(const vec_Vecf<3>& path);

  /** @brief Apply all path cleanup steps (collinear and corner removal) in place.
   *  @param path Waypoint path to clean up.
   */
  void cleanUpPath(vec_Vecf<3>& path);

  /** @brief Run the global path planner from start to goal.
   *  @param start Start position.
   *  @param start_vel Start velocity for alignment heuristic.
   *  @param goal Goal position.
   *  @param final_g Output cost of the found path.
   *  @param current_time Current simulation/wall time.
   *  @param eps Heuristic weight for A* search.
   *  @return True if a valid path was found.
   */
  bool plan(
      const Vecf<3>& start,
      const Vecf<3>& start_vel,
      const Vecf<3>& goal,
      double& final_g,
      double current_time,
      decimal_t eps = 1);

  /** @brief Get the nodes in the open set after planning.
   *  @return Positions of open-set nodes.
   */
  vec_Vecf<3> getOpenSet() const;

  /** @brief Get the nodes in the closed set after planning.
   *  @return Positions of closed-set nodes.
   */
  vec_Vecf<3> getCloseSet() const;

  /** @brief Get all explored nodes after planning.
   *  @return Positions of all explored nodes.
   */
  vec_Vecf<3> getAllSet() const;

  /** @brief Get the total initial-guess planning time.
   *  @return Time in seconds.
   */
  double getInitialGuessPlanningTime();

  /** @brief Get the static JPS planning time.
   *  @return Time in seconds.
   */
  double getStaticJPSPlanningTime();

  /** @brief Get the path-checking time.
   *  @return Time in seconds.
   */
  double getCheckPathTime();

  /** @brief Get the dynamic A* planning time.
   *  @return Time in seconds.
   */
  double getDynamicAstarTime();

  /** @brief Get the path recovery time.
   *  @return Time in seconds.
   */
  double getRecoverPathTime();

  /** @brief Update the maximum velocity used by the planner.
   *  @param v_max New maximum velocity.
   */
  void updateVmax(double v_max);

  /** @brief Set the maximum number of node expansions for graph search.
   *  @param max_expand Maximum node expansions.
   */
  void setMaxExpand(int max_expand) { max_expand_ = max_expand; }

  /** @brief Shorten a path using line-of-sight checks with inflated capsule collision tests.
   *  @param in Input waypoint path.
   *  @param inflate_radius_cells Inflation radius in voxel cells.
   *  @return Shortened path.
   */
  vec_Vecf<3> shortCutByLoS(const vec_Vecf<3>& in, int inflate_radius_cells) const;

  /** @brief Check line-of-sight between two points using an inflated capsule collision test.
   *  @param a Start point.
   *  @param b End point.
   *  @param inflate_radius_cells Inflation radius in voxel cells.
   *  @return True if the line segment is collision-free.
   */
  bool lineOfSightCapsule(const Vecf<3>& a, const Vecf<3>& b, int inflate_radius_cells) const;

  /** @brief Merge consecutive edges shorter than a minimum length.
   *  @param in Input waypoint path.
   *  @param min_len Minimum edge length.
   *  @return Path with short edges collapsed.
   */
  vec_Vecf<3> collapseShortEdges(const vec_Vecf<3>& in, double min_len) const;

  /** @brief Remove waypoints that create turns below a minimum angle threshold.
   *  @param in Input waypoint path.
   *  @param min_turn_deg Minimum turn angle in degrees.
   *  @param min_seg_len Minimum segment length to consider.
   *  @return Filtered path.
   */
  vec_Vecf<3> angleSpacingFilter(
      const vec_Vecf<3>& in, double min_turn_deg, double min_seg_len) const;

 protected:
  // Assume using 3D voxel map for all 2d and 3d planning
  std::shared_ptr<sando::VoxelMapUtil> map_util_;
  // The planner
  std::shared_ptr<sando::GraphSearch> graph_search_;
  // Raw path from planner
  vec_Vecf<3> raw_path_;
  // Modified path for future usage
  vec_Vecf<3> path_;
  // Flag indicating the success of planning
  int status_ = 0;
  // Enabled for printing info
  bool planner_verbose_;

  // Parameters
  std::string global_planner_;

  // For benchmarking time recording
  double global_planning_time_ = 0.0;
  double hgp_static_jps_time_ = 0.0;
  double hgp_check_path_time_ = 0.0;
  double hgp_dynamic_astar_time_ = 0.0;
  double hgp_recover_path_time_ = 0.0;

  // time out duration
  int hgp_timeout_duration_ms_ = 1000;

  // max node expansion
  int max_expand_ = 10000;

  // max values
  double v_max_;
  double a_max_;
  double j_max_;

  // alignment
  double w_align_ = 60.0;          // weight for alignment with start velocity
  double decay_len_cells_ = 20.0;  // decay length in cells for alignment
  double w_side_ = 0.2;            // weight for side tie-break

  // unknown cell cost weight
  double w_unknown_ = 1.0;

  // LOS post processing
  int los_cells_ = 3;       // number of cells for inflation in LoS
  double min_len_ = 0.5;    // minimum length of edges
  double min_turn_ = 10.0;  // minimum turn angle in degrees
};
