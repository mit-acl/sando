/* ----------------------------------------------------------------------------
 * Copyright (c) Anonymous Author
 * Anonymous Institution
 * All Rights Reserved
 * Authors: Anonymous
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "hgp/hgp_manager.hpp"
#include <mutex>

/// The type of map data Tmap is defined as a 1D array
using Tmap = std::vector<char>;
using namespace sando;
using namespace termcolor;

typedef timer::Timer MyTimer;

HGPManager::HGPManager() {}

void HGPManager::setParameters(const Parameters& par) {
  // Get the parameter
  par_ = par;

  // Set the parameters
  res_ = par.res;
  drone_radius_ = par.drone_radius;
  max_dist_vertexes_ = par.max_dist_vertexes;
  use_shrinked_box_ = par.use_shrinked_box;
  shrinked_box_size_ = par.shrinked_box_size;
  // sfc_size_ is std::vector<float> but par.sfc_size is std::vector<double>
  sfc_size_ = {
      static_cast<float>(par.sfc_size[0]), static_cast<float>(par.sfc_size[1]),
      static_cast<float>(par.sfc_size[2])};

  // shared pointer to the map util for actual planning
  map_util_ = std::make_shared<sando::VoxelMapUtil>(
      par.factor_hgp * par.res, par.x_min, par.x_max, par.y_min, par.y_max, par.z_min, par.z_max,
      par.inflation_hgp, par.obst_max_vel);

  // ---------------- Global-planner configuration: YAML-driven heat map parameters ----------------

  // Control hard vs soft dynamic obstacle representation
  map_util_->setDynamicAsOccupiedCurrentPos(par.dynamic_as_occupied_current);
  map_util_->setDynamicAsOccupiedFuturePos(par.dynamic_as_occupied_future);

  // Dynamic heat configuration
  map_util_->setDynamicHeatEnabled(par.dynamic_heat_enabled);
  map_util_->setDynamicHeatParams(
      par.heat_alpha0, par.heat_alpha1, par.heat_p, par.heat_q, par.heat_tau_ratio, par.heat_gamma,
      par.heat_Hmax, par.dyn_base_inflation_m);
  map_util_->setDynHeatTubeRadius(par.dyn_heat_tube_radius_m);
  map_util_->setHeatWeight(par.heat_weight);

  // Print heat parameters for verification
  std::cout << "=== HGP Heat Parameters Applied ===" << std::endl;
  std::cout << "  heat_weight: " << par.heat_weight << std::endl;
  std::cout << "  heat_alpha0: " << par.heat_alpha0 << ", heat_alpha1: " << par.heat_alpha1
            << std::endl;
  std::cout << "  heat_Hmax: " << par.heat_Hmax << ", static_heat_Hmax: " << par.static_heat_Hmax
            << std::endl;
  std::cout << "  dyn_heat_tube_radius_m: " << par.dyn_heat_tube_radius_m << std::endl;
  std::cout << "  static_heat_alpha: " << par.static_heat_alpha
            << ", static_heat_rmax_m: " << par.static_heat_rmax_m << std::endl;
  std::cout << "  dynamic_heat_enabled: " << par.dynamic_heat_enabled
            << ", static_heat_enabled: " << par.static_heat_enabled << std::endl;
  std::cout << "===================================" << std::endl;

  // Static heat configuration
  map_util_->setStaticHeatEnabled(par.static_heat_enabled);
  map_util_->setStaticHeatParams(
      par.static_heat_alpha, par.static_heat_p, par.static_heat_Hmax, par.static_heat_rmax_m,
      par.static_heat_boundary_only, par.static_heat_apply_on_unknown,
      par.static_heat_exclude_dynamic);

  // Static heat radius function
  map_util_->setStaticHeatRadiusFunction(
      [default_r = par.static_heat_default_radius_m](const Eigen::Vector3f&) { return default_r; },
      par.static_heat_default_radius_m);

  // Soft-cost obstacle mode
  map_util_->setSoftCostObstacles(par.use_soft_cost_obstacles, par.obstacle_soft_cost);
}

// ----------------------------------------------------------------------------

void HGPManager::setDynamicPredictedSamples(
    const std::vector<vec_Vecf<3>>& pred_samples, const std::vector<float>& pred_times) {
  std::lock_guard<std::mutex> lock(mtx_map_util_);
  if (map_util_) map_util_->setDynamicPredictedSamples(pred_samples, pred_times);
}

std::shared_ptr<sando::VoxelMapUtil> HGPManager::getMapUtilSharedPtr() {
  std::lock_guard<std::mutex> lock(mtx_map_util_);
  return map_util_;
}

void HGPManager::cleanUpPath(vec_Vecf<3>& path) { planner_ptr_->cleanUpPath(path); }

void HGPManager::setupHGPPlanner(
    const std::string& global_planner,
    bool global_planner_verbose,
    double res,
    double v_max,
    double a_max,
    double j_max,
    int hgp_timeout_duration_ms,
    int max_num_expansion,
    double w_unknown,
    double w_align,
    double decay_len_cells,
    double w_side,
    int los_cells,
    double min_len,
    double min_turn) {
  // Get the parameters
  v_max_ = v_max;
  v_max_3d_ = Eigen::Vector3d(v_max, v_max, v_max);
  a_max_ = a_max;
  a_max_3d_ = Eigen::Vector3d(a_max, a_max, a_max);
  j_max_ = j_max;
  j_max_3d_ = Eigen::Vector3d(j_max, j_max, j_max);

  // Create the HGP planner
  planner_ptr_ = std::unique_ptr<HGPPlanner>(new HGPPlanner(
      global_planner, global_planner_verbose, v_max, a_max, j_max, hgp_timeout_duration_ms,
      w_unknown, w_align, decay_len_cells, w_side, los_cells, min_len, min_turn));

  // Set max node expansion
  planner_ptr_->setMaxExpand(max_num_expansion);

  // Create the map_util_for_planning
  // This is the beginning of the planning, so we fetch the map_util_ and don't update it for the
  // entire planning process (updating while planning makes the planner slower)
  mtx_map_util_.lock();
  map_util_for_planning_ = std::make_shared<sando::VoxelMapUtil>(*map_util_);
  mtx_map_util_.unlock();
}

void HGPManager::updateVmax(double v_max) {
  // Update the maximum velocity
  v_max_ = v_max;
  v_max_3d_ = Eigen::Vector3d(v_max, v_max, v_max);
  planner_ptr_->updateVmax(v_max);
}

void HGPManager::freeStart(Vec3f& start, double factor) {
  // Set start free
  Veci<3> start_int = map_util_for_planning_->floatToInt(start);
  map_util_for_planning_->setFreeVoxelAndSurroundings(start_int, factor * res_);
}

void HGPManager::freeGoal(Vec3f& goal, double factor) {
  // Set goal free
  Veci<3> goal_int = map_util_for_planning_->floatToInt(goal);
  map_util_for_planning_->setFreeVoxelAndSurroundings(goal_int, factor * res_);
}

bool HGPManager::checkIfPointOccupied(const Vec3f& point) {
  // Use planning map if available, otherwise fall back to the base map.
  // map_util_for_planning_ is only created inside setupHGPPlanner/solveHGP,
  // so callers outside the planning pipeline (e.g. checkHoverAvoidance) need
  // the fallback.
  const auto& mu = map_util_for_planning_ ? map_util_for_planning_ : map_util_;
  if (!mu) return false;  // map not yet initialized

  Veci<3> point_int = mu->floatToInt(point);
  return mu->isOccupied(point_int);
}

// Sample along [p0, p1] at a safe step to ensure we don't skip thin obstacles.
// Uses the occupancy from the (already inflated) planning map.
inline bool isSegmentFree(
    const sando::VoxelMapUtil& map, const Vec3f& p0, const Vec3f& p1, const double sample_step) {
  const Vec3f d = p1 - p0;
  const double L = d.norm();
  if (L <= 1e-6) return true;

  const Vec3f dir = d / L;
  // March from start to end, including the goal voxel.
  for (double s = 0.0; s <= L; s += sample_step) {
    const Vec3f q = p0 + s * dir;
    const Veci<3> qi = map.floatToInt(q);
    if (map.isOccupied(qi)) return false;
  }
  // Ensure exact endpoint is also checked (if s stepped past)
  const Veci<3> q_end = map.floatToInt(p1);
  if (map.isOccupied(q_end)) return false;

  return true;
}

// Greedily collapse a path into maximal collision-free segments.
// This mirrors the "generate a long segment if it’s collision free" behavior.
inline void collapseIntoLongSegments(
    const sando::VoxelMapUtil& map,
    double res,
    vec_Vecf<3>& path_inout,
    double sample_step = -1.0) {
  if (path_inout.size() <= 2) return;

  const double step = (sample_step > 0.0) ? sample_step : 0.5 * res;

  vec_Vecf<3> simplified;
  simplified.reserve(path_inout.size());
  simplified.push_back(path_inout.front());  // keep start

  size_t anchor = 0;      // current segment start index
  size_t j = anchor + 1;  // candidate end

  while (j < path_inout.size()) {
    size_t last_good = anchor + 1;

    // Extend j as far as LoS holds
    while (j < path_inout.size() && isSegmentFree(map, path_inout[anchor], path_inout[j], step)) {
      last_good = j;
      ++j;
    }

    // Commit the farthest valid endpoint
    simplified.push_back(path_inout[last_good]);

    // Start next segment from there
    anchor = last_good;
    j = anchor + 1;
  }

  // Make sure we end exactly at the original goal
  simplified.back() = path_inout.back();

  path_inout.swap(simplified);
}

bool HGPManager::solveHGP(
    const Vec3f& start_sent,
    const Vec3f& start_vel,
    const Vec3f& goal_sent,
    double& final_g,
    double weight,
    double current_time,
    vec_Vecf<3>& path,
    vec_Vecf<3>& raw_path) {
  {
    std::lock_guard<std::mutex> lock(mtx_map_util_);
    map_util_for_planning_ = std::make_shared<sando::VoxelMapUtil>(*map_util_);
  }

  // Set start and goal
  Eigen::Vector3d start(start_sent(0), start_sent(1), start_sent(2));
  Eigen::Vector3d goal(goal_sent(0), goal_sent(1), goal_sent(2));

  // Set collision checking function
  planner_ptr_->setMapUtil(map_util_for_planning_);

  // HGP Plan
  bool result = false;

  // Attempt to plan
  result = planner_ptr_->plan(start, start_vel, goal, final_g, current_time, weight);

  // If there is a solution
  if (result) {
    path = planner_ptr_->getPath();
    raw_path = planner_ptr_->getRawPath();
  }

  // Add more vertices if necessary
  sando_utils::createMoreVertexes(path, max_dist_vertexes_);

  // Final cleanup: merge any vertices closer than min_len
  // (can arise from createMoreVertexes remainders or post-processing steps after
  // collapseShortEdges)
  path = planner_ptr_->collapseShortEdges(path, par_.min_len);

  return result;
}

bool HGPManager::checkIfPathInFree(const vec_Vecf<3>& path, vec_Vecf<3>& free_path) {
  // Initialize result
  free_path.clear();
  free_path.reserve(path.size());
  free_path.push_back(path[0]);

  // Plan only in free space if required
  for (size_t i = 1; i < path.size(); i++) {
    Veci<3> path_int = map_util_for_planning_->floatToInt(path[i]);
    if (map_util_for_planning_->isFree(path_int)) {
      free_path.push_back(path[i]);
    } else {
      break;
    }
  }

  if (free_path.size() <= 1) return false;

  return true;
}

void HGPManager::pushPathIntoFreeSpace(const vec_Vecf<3>& path, vec_Vecf<3>& free_path) {
  // Initialize result
  free_path.clear();
  free_path.reserve(path.size());
  free_path.push_back(path[0]);

  // Plan only in free space if required
  for (size_t i = 1; i < path.size(); i++) {
    Vec3f free_point;
    map_util_for_planning_->findClosestFreePoint(path[i], free_point);
    free_path.push_back(free_point);
  }
}

bool HGPManager::checkIfPointFree(const Vec3f& point) const {
  const auto& mu = map_util_for_planning_ ? map_util_for_planning_ : map_util_;
  if (!mu) return true;  // map not yet initialized, assume free

  Veci<3> point_int = mu->floatToInt(point);
  return mu->isFree(point_int);
}

bool HGPManager::checkIfPointHasNonFreeNeighbour(const Vec3f& point) const {
  // Check if the point has an occupied neighbour
  Veci<3> point_int = map_util_for_planning_->floatToInt(point);
  return map_util_for_planning_->checkIfPointHasNonFreeNeighbour(point_int);
}

void HGPManager::getOccupiedCells(vec_Vecf<3>& occupied_cells) {
  // Get the occupied cells
  std::lock_guard<std::mutex> lock(mtx_map_util_);
  occupied_cells = map_util_->getOccupiedCloud();
}

void HGPManager::getFreeCells(vec_Vecf<3>& free_cells) {
  // Get the free cells
  std::lock_guard<std::mutex> lock(mtx_map_util_);
  free_cells = map_util_->getFreeCloud();
}

void HGPManager::getComputationTime(
    double& global_planning_time,
    double& hgp_static_jps_time,
    double& hgp_check_path_time,
    double& hgp_dynamic_astar_time,
    double& hgp_recover_path_time) {
  // Get the computation time
  global_planning_time = planner_ptr_->getInitialGuessPlanningTime();
  hgp_static_jps_time = planner_ptr_->getStaticJPSPlanningTime();
  hgp_check_path_time = planner_ptr_->getCheckPathTime();
  hgp_dynamic_astar_time = planner_ptr_->getDynamicAstarTime();
  hgp_recover_path_time = planner_ptr_->getRecoverPathTime();
}

void HGPManager::getVecOccupied(vec_Vec3f& vec_o) {
  std::lock_guard<std::mutex> lock(mtx_vec_o_);
  vec_o = vec_o_;
}

void HGPManager::updateVecOccupied(const vec_Vec3f& vec_o) {
  std::lock_guard<std::mutex> lock(mtx_vec_o_);
  vec_o_ = vec_o;
}

void HGPManager::getVecUnknownOccupied(vec_Vec3f& vec_uo) {
  std::lock_guard<std::mutex> lock(mtx_vec_uo_);
  vec_uo = vec_uo_;
}

void HGPManager::updateVecUnknownOccupied(const vec_Vec3f& vec_uo) {
  std::lock_guard<std::mutex> lock(mtx_vec_uo_);
  vec_uo_ = vec_uo;
}

void HGPManager::insertVecOccupiedToVecUnknownOccupied() {
  std::scoped_lock lock(mtx_vec_uo_, mtx_vec_o_);
  vec_uo_.insert(vec_uo_.end(), vec_o_.begin(), vec_o_.end());
}

bool HGPManager::cvxEllipsoidDecomp(
    EllipsoidDecomp3D& ellip,
    const vec_Vecf<3>& path,
    const vec_Vec3f& base_uo,
    const vec_Vecf<3>& obst_pos,
    const vec_Vecf<3>& obst_bbox,
    const std::vector<double>& seg_end_times,
    std::vector<LinearConstraint3D>& l_constraints,
    vec_E<Polyhedron<3>>& poly_out) {
  if (path.size() < 2) return false;

  const size_t num_seg = path.size() - 1;

  if (seg_end_times.size() != num_seg) {
    std::cout << "cvxEllipsoidDecomp: seg_end_times size mismatch. Expected " << num_seg << ", got "
              << seg_end_times.size() << std::endl;
    return false;
  }

  // Configure ellipsoid-decomp settings on the per-worker instance
  ellip.set_local_bbox(Vec3f(sfc_size_[0], sfc_size_[1], sfc_size_[2]));
  ellip.set_z_min_and_max(par_.z_min, par_.z_max);
  ellip.set_inflate_distance(drone_radius_);

  // Outputs
  l_constraints.clear();
  l_constraints.resize(num_seg);

  poly_out.clear();
  poly_out.resize(num_seg);

  vec_Vecf<3> seg_path;
  seg_path.reserve(2);

  for (size_t i = 0; i < num_seg; ++i) {
    const double traj_max_time = seg_end_times[i];
    if (!(traj_max_time > 0.0)) {
      std::cout << "cvxEllipsoidDecomp: non-positive seg_end_times[" << i << "]=" << traj_max_time
                << std::endl;
      return false;
    }

    // Build per-segment obstacle set = base_uo + inflated dynamic obstacle points
    vec_Vec3f vec_uo = base_uo;  // copy snapshot
    obstacle_to_vec(vec_uo, obst_pos, obst_bbox, traj_max_time);

    ellip.set_obs(vec_uo);

    seg_path.clear();
    seg_path.push_back(path[i]);
    seg_path.push_back(path[i + 1]);

    bool ok = true;
    ellip.dilate(seg_path, ok);
    if (!ok) {
      std::cout << "cvxEllipsoidDecomp: dilate failed at segment " << i << std::endl;
      return false;
    }

    if (use_shrinked_box_) ellip.shrink_polyhedrons(shrinked_box_size_);

    auto polys = ellip.get_polyhedrons();
    if (polys.size() != 1) {
      std::cout << "cvxEllipsoidDecomp: expected 1 polyhedron for segment " << i << ", got "
                << polys.size() << std::endl;
      return false;
    }

    poly_out[i] = polys[0];

    const auto pt_inside = (path[i] + path[i + 1]) / 2.0;
    LinearConstraint3D cs(pt_inside, poly_out[i].hyperplanes(), poly_out[i]);

    if (cs.A_.hasNaN() || cs.b_.hasNaN()) {
      std::cout << "cvxEllipsoidDecomp: A_ or b_ has NaN at segment " << i << std::endl;
      return false;
    }

    l_constraints[i] = cs;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool HGPManager::cvxEllipsoidDecompTimeLayered(
    EllipsoidDecomp3D& ellip,
    const vec_Vecf<3>& path,                    // global path (size = P+1)
    const vec_Vec3f& base_uo,                   // static+unknown occupied snapshot
    const vec_Vecf<3>& obst_pos,                // dynamic obstacle positions
    const vec_Vecf<3>& obst_bbox,               // dynamic obstacle bbox half-extents
    const std::vector<double>& time_end_times,  // size = N (local segment time layers)
    std::vector<std::vector<LinearConstraint3D>>& l_constraints_by_time,  // [N][P]
    std::vector<vec_E<Polyhedron<3>>>& poly_out_by_time                   // [N][P]
) {
  if (path.size() < 2) return false;

  const size_t P = path.size() - 1;        // number of spatial segments
  const size_t N = time_end_times.size();  // number of time layers

  if (P == 0 || N == 0) return false;

  // Configure ellipsoid-decomp settings on the per-worker instance (same as single-layer)
  ellip.set_local_bbox(Vec3f(sfc_size_[0], sfc_size_[1], sfc_size_[2]));
  ellip.set_z_min_and_max(par_.z_min, par_.z_max);
  ellip.set_inflate_distance(drone_radius_);

  // Allocate outputs
  l_constraints_by_time.clear();
  l_constraints_by_time.resize(N);
  for (size_t n = 0; n < N; ++n) l_constraints_by_time[n].resize(P);

  poly_out_by_time.clear();
  poly_out_by_time.resize(N);
  for (size_t n = 0; n < N; ++n) poly_out_by_time[n].resize(P);

  // Precompute per-time-layer obstacle sets (base_uo + inflated dynamic obstacle points)
  // This avoids calling obstacle_to_vec inside the inner P-loop.
  std::vector<vec_Vec3f> uo_by_time;
  uo_by_time.resize(N);

  for (size_t n = 0; n < N; ++n) {
    const double tmax = time_end_times[n];
    if (!(tmax > 0.0)) {
      std::cout << "cvxEllipsoidDecompTimeLayered: non-positive time_end_times[" << n
                << "]=" << tmax << std::endl;
      return false;
    }

    uo_by_time[n] = base_uo;  // copy snapshot
    obstacle_to_vec(uo_by_time[n], obst_pos, obst_bbox, tmax);
  }

  vec_Vecf<3> seg_path;
  seg_path.reserve(2);

  for (size_t n = 0; n < N; ++n) {
    // Set per-time-layer obstacle set once
    ellip.set_obs(uo_by_time[n]);

    for (size_t p = 0; p < P; ++p) {
      seg_path.clear();
      seg_path.push_back(path[p]);
      seg_path.push_back(path[p + 1]);

      bool ok = true;
      ellip.dilate(seg_path, ok);
      if (!ok) {
        std::cout << "cvxEllipsoidDecompTimeLayered: dilate failed at (n=" << n << ", p=" << p
                  << ")" << std::endl;
        return false;
      }

      if (use_shrinked_box_) ellip.shrink_polyhedrons(shrinked_box_size_);

      auto polys = ellip.get_polyhedrons();
      if (polys.size() != 1) {
        std::cout << "cvxEllipsoidDecompTimeLayered: expected 1 polyhedron at (n=" << n
                  << ", p=" << p << "), got " << polys.size() << std::endl;
        return false;
      }

      poly_out_by_time[n][p] = polys[0];

      const auto pt_inside = (path[p] + path[p + 1]) / 2.0;
      LinearConstraint3D cs(
          pt_inside, poly_out_by_time[n][p].hyperplanes(), poly_out_by_time[n][p]);

      if (cs.A_.hasNaN() || cs.b_.hasNaN()) {
        std::cout << "cvxEllipsoidDecompTimeLayered: A_ or b_ has NaN at (n=" << n << ", p=" << p
                  << ")" << std::endl;
        return false;
      }

      l_constraints_by_time[n][p] = cs;
    }
  }

  return true;
}

// ----------------------------------------------------------------------------

namespace {
struct Offset3i {
  int ix{0}, iy{0}, iz{0};
  int n2{0};  // ix^2 + iy^2 + iz^2
};

struct VoxelKey {
  int x{0}, y{0}, z{0};
  bool operator==(const VoxelKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const noexcept {
    // 64-bit mix (cheap, decent distribution)
    std::size_t h = static_cast<std::size_t>(k.x);
    h = h * 1315423911u + static_cast<std::size_t>(k.y);
    h = h * 1315423911u + static_cast<std::size_t>(k.z);
    return h;
  }
};

inline const std::vector<Offset3i>& sphereOffsetsCached(int m) {
  static std::mutex mtx;
  static std::unordered_map<int, std::vector<Offset3i>> cache;

  std::lock_guard<std::mutex> lk(mtx);
  auto it = cache.find(m);
  if (it != cache.end()) return it->second;

  std::vector<Offset3i> offs;
  offs.reserve(
      static_cast<std::size_t>(2 * m + 1) * static_cast<std::size_t>(2 * m + 1) *
      static_cast<std::size_t>(2 * m + 1));

  const int m2 = m * m;
  for (int ix = -m; ix <= m; ++ix) {
    for (int iy = -m; iy <= m; ++iy) {
      for (int iz = -m; iz <= m; ++iz) {
        const int n2 = ix * ix + iy * iy + iz * iz;
        if (n2 > m2) continue;
        offs.push_back(Offset3i{ix, iy, iz, n2});
      }
    }
  }

  auto [it2, _] = cache.emplace(m, std::move(offs));
  return it2->second;
}

inline bool isUnknownVoxel(const sando::VoxelMapUtil& map, const Veci<3>& idx) {
  // Unknown := neither free nor occupied.
  return (!map.isFree(idx)) && (!map.isOccupied(idx));
}

}  // namespace

// ----------------------------------------------------------------------------

void HGPManager::obstacle_to_vec(
    vec_Vec3f& pts,
    const vec_Vecf<3>& obst_pos,
    const vec_Vecf<3>& obst_bbox,
    double traj_max_time) {
  // Inflate radius around unknown boundary and dynamic obstacles.
  // Unknown boundary is extracted from the *current contents* of pts (assumed to include unknown
  // voxels). Dynamic obstacles are provided separately in obst_pos.

  const double res = par_.factor_hgp * par_.res;
  const double r = par_.obst_max_vel * traj_max_time +
                   par_.obst_position_error;  // [m] motion + estimation error

  if (!(r > 0.0) || !(res > 0.0)) return;

  // Grid half-width in cells
  const int m = static_cast<int>(std::ceil(r / res));
  if (m <= 0) return;

  // ------------------------------------------------------------------------
  // Offsets: pre-filter to the true radius to remove the hot-loop branch.
  // sphereOffsetsCached(m) already gives offsets within m cells, but r can be slightly smaller.
  // ------------------------------------------------------------------------
  const auto& offs_all = sphereOffsetsCached(m);

  const double r2_over_res2_d = (r * r) / (res * res);
  int max_n2 = static_cast<int>(std::floor(r2_over_res2_d + 1e-9));
  const int m2 = m * m;
  if (max_n2 > m2) max_n2 = m2;
  if (max_n2 < 0) return;

  std::vector<Offset3i> offs_r;
  offs_r.reserve(offs_all.size());
  for (const auto& o : offs_all) {
    if (o.n2 <= max_n2) offs_r.push_back(o);
  }
  if (offs_r.empty()) return;

  // ------------------------------------------------------------------------
  // (A) Inflate unknown space efficiently:
  //     - classify unknown voxels among pts[0:base_sz) using map_util_for_planning_
  //     - extract unknown boundary voxels (6-neighborhood)
  //     - inflate only the boundary voxels
  //
  // Fast path: dense local voxel window (no hashing) if the unknown AABB is reasonable.
  // Fallback: single unordered_map (still faster than set+map+set).
  // Gated by inflate_unknown_boundary parameter (default true).
  // ------------------------------------------------------------------------
  const std::size_t base_sz = pts.size();
  if (base_sz > 0 && par_.inflate_unknown_boundary) {
    // Collect unknown voxel indices (+ representative point) in one pass.
    // We need reps to preserve the same world-frame anchoring as your original (c + offset*res).
    std::vector<Veci<3>> unk_idxs;
    std::vector<Vec3f> unk_reps;
    unk_idxs.reserve(base_sz / 2);
    unk_reps.reserve(base_sz / 2);

    bool have_map = static_cast<bool>(map_util_for_planning_);

    // Track AABB in voxel index space for dense-window decision
    bool aabb_init = false;
    int min_x = 0, min_y = 0, min_z = 0, max_x = 0, max_y = 0, max_z = 0;

    if (have_map) {
      for (std::size_t i = 0; i < base_sz; ++i) {
        const Vec3f& p = pts[i];
        const Veci<3> idx = map_util_for_planning_->floatToInt(p);
        if (!isUnknownVoxel(*map_util_for_planning_, idx)) continue;

        unk_idxs.push_back(idx);
        unk_reps.push_back(p);

        if (!aabb_init) {
          aabb_init = true;
          min_x = max_x = idx(0);
          min_y = max_y = idx(1);
          min_z = max_z = idx(2);
        } else {
          min_x = std::min(min_x, idx(0));
          max_x = std::max(max_x, idx(0));
          min_y = std::min(min_y, idx(1));
          max_y = std::max(max_y, idx(1));
          min_z = std::min(min_z, idx(2));
          max_z = std::max(max_z, idx(2));
        }
      }
    } else {
      // Fallback: treat everything in pts as "unknown" and voxelize by rounding in res-sized grid.
      // This is best-effort; prefer map_util_for_planning_ if available.
      for (std::size_t i = 0; i < base_sz; ++i) {
        const Vec3f& p = pts[i];
        Veci<3> idx;
        idx << static_cast<int>(std::llround(p.x() / res)),
            static_cast<int>(std::llround(p.y() / res)),
            static_cast<int>(std::llround(p.z() / res));

        unk_idxs.push_back(idx);
        unk_reps.push_back(p);

        if (!aabb_init) {
          aabb_init = true;
          min_x = max_x = idx(0);
          min_y = max_y = idx(1);
          min_z = max_z = idx(2);
        } else {
          min_x = std::min(min_x, idx(0));
          max_x = std::max(max_x, idx(0));
          min_y = std::min(min_y, idx(1));
          max_y = std::max(max_y, idx(1));
          min_z = std::min(min_z, idx(2));
          max_z = std::max(max_z, idx(2));
        }
      }
    }

    if (!unk_idxs.empty()) {
      // Dense-window thresholds (tune as needed).
      // These are chosen to avoid pathological allocations while covering typical local-window
      // maps.
      constexpr std::size_t kMaxDenseCells = 2'000'000;      // ~2 MB for 1-byte mask
      constexpr std::size_t kMaxDenseInflCells = 8'000'000;  // inflated window mask

      const int dx = (max_x - min_x + 1);
      const int dy = (max_y - min_y + 1);
      const int dz = (max_z - min_z + 1);

      auto safeMul3 = [](std::size_t a, std::size_t b, std::size_t c) -> std::size_t {
        // conservative overflow guard
        if (a == 0 || b == 0 || c == 0) return 0;
        if (a > (std::numeric_limits<std::size_t>::max() / b))
          return std::numeric_limits<std::size_t>::max();
        std::size_t ab = a * b;
        if (ab > (std::numeric_limits<std::size_t>::max() / c))
          return std::numeric_limits<std::size_t>::max();
        return ab * c;
      };

      const std::size_t vol = safeMul3(
          static_cast<std::size_t>(dx), static_cast<std::size_t>(dy), static_cast<std::size_t>(dz));

      const int dx2 = dx + 2 * m;
      const int dy2 = dy + 2 * m;
      const int dz2 = dz + 2 * m;
      const std::size_t vol2 = safeMul3(
          static_cast<std::size_t>(dx2), static_cast<std::size_t>(dy2),
          static_cast<std::size_t>(dz2));

      const bool use_dense = (vol > 0 && vol <= kMaxDenseCells) &&
                             (vol2 > 0 && vol2 <= kMaxDenseInflCells) &&
                             (dx > 0 && dy > 0 && dz > 0) && (dx2 > 0 && dy2 > 0 && dz2 > 0);

      if (use_dense) {
        // Dense masks
        std::vector<unsigned char> unk_mask(vol, 0);
        std::vector<unsigned char> infl_mask(vol2, 0);

        // Representative point per unique unknown voxel (stored as SoA for speed / alignment
        // safety)
        std::vector<float> repx(vol, 0.0f), repy(vol, 0.0f), repz(vol, 0.0f);

        auto lin = [dx, dy](int x, int y, int z) -> std::size_t {
          return static_cast<std::size_t>(x) +
                 static_cast<std::size_t>(dx) *
                     (static_cast<std::size_t>(y) +
                      static_cast<std::size_t>(dy) * static_cast<std::size_t>(z));
        };

        auto lin2 = [dx2, dy2](int x, int y, int z) -> std::size_t {
          return static_cast<std::size_t>(x) +
                 static_cast<std::size_t>(dx2) *
                     (static_cast<std::size_t>(y) +
                      static_cast<std::size_t>(dy2) * static_cast<std::size_t>(z));
        };

        // Unique unknown voxels list in window coordinates [0..dx-1], etc.
        std::vector<Veci<3>> uniq;
        uniq.reserve(std::min<std::size_t>(unk_idxs.size(), vol));

        for (std::size_t i = 0; i < unk_idxs.size(); ++i) {
          const auto& idx = unk_idxs[i];
          const int x = idx(0) - min_x;
          const int y = idx(1) - min_y;
          const int z = idx(2) - min_z;

          if (x < 0 || x >= dx || y < 0 || y >= dy || z < 0 || z >= dz) continue;

          const std::size_t L = lin(x, y, z);
          if (!unk_mask[L]) {
            unk_mask[L] = 1;
            uniq.push_back(
                idxs_to_veci3(x, y, z));  // if you don't have this helper, see note below

            // store representative world-frame point for this voxel
            repx[L] = unk_reps[i].x();
            repy[L] = unk_reps[i].y();
            repz[L] = unk_reps[i].z();
          }
        }

        if (!uniq.empty()) {
          // Boundary voxels
          std::vector<Veci<3>> boundary;
          boundary.reserve(uniq.size() / 2);

          for (const auto& uvw : uniq) {
            const int x = uvw(0), y = uvw(1), z = uvw(2);

            // If any 6-neighbor is missing, it's boundary.
            bool is_b = false;

            // neighbor checks with bounds
            if (x + 1 >= dx || !unk_mask[lin(x + 1, y, z)])
              is_b = true;
            else if (x - 1 < 0 || !unk_mask[lin(x - 1, y, z)])
              is_b = true;
            else if (y + 1 >= dy || !unk_mask[lin(x, y + 1, z)])
              is_b = true;
            else if (y - 1 < 0 || !unk_mask[lin(x, y - 1, z)])
              is_b = true;
            else if (z + 1 >= dz || !unk_mask[lin(x, y, z + 1)])
              is_b = true;
            else if (z - 1 < 0 || !unk_mask[lin(x, y, z - 1)])
              is_b = true;

            if (is_b) boundary.push_back(uvw);
          }

          if (!boundary.empty()) {
            // Reserve a reasonable amount to reduce reallocations.
            // Dedup happens via infl_mask, so output size is bounded by vol2.
            pts.reserve(pts.size() + std::min<std::size_t>(boundary.size() * offs_r.size(), vol2));

            for (const auto& uvw : boundary) {
              const int bx = uvw(0);
              const int by = uvw(1);
              const int bz = uvw(2);

              const std::size_t Lb = lin(bx, by, bz);
              const float cx = repx[Lb];
              const float cy = repy[Lb];
              const float cz = repz[Lb];

              // Inflate in the expanded window with margin m
              const int base_x2 = bx + m;
              const int base_y2 = by + m;
              const int base_z2 = bz + m;

              for (const auto& o : offs_r) {
                const int nx2 = base_x2 + o.ix;
                const int ny2 = base_y2 + o.iy;
                const int nz2 = base_z2 + o.iz;

                // bounds in inflated window (should hold, but keep safe)
                if (nx2 < 0 || nx2 >= dx2 || ny2 < 0 || ny2 >= dy2 || nz2 < 0 || nz2 >= dz2)
                  continue;

                const std::size_t Li = lin2(nx2, ny2, nz2);
                if (infl_mask[Li]) continue;
                infl_mask[Li] = 1;

                Vec3f q;
                q << (cx + static_cast<float>(o.ix * res)), (cy + static_cast<float>(o.iy * res)),
                    (cz + static_cast<float>(o.iz * res));
                pts.emplace_back(q);
              }
            }
          }
        }
      } else {
        // -----------------------
        // Fallback: hashing path
        // -----------------------
        std::unordered_map<VoxelKey, Vec3f, VoxelKeyHash> unk_rep;
        unk_rep.reserve(unk_idxs.size());
        unk_rep.max_load_factor(0.7f);

        for (std::size_t i = 0; i < unk_idxs.size(); ++i) {
          const auto& idx = unk_idxs[i];
          VoxelKey k{idx(0), idx(1), idx(2)};
          // keep first representative
          if (unk_rep.find(k) == unk_rep.end()) unk_rep.emplace(k, unk_reps[i]);
        }

        if (!unk_rep.empty()) {
          std::vector<VoxelKey> boundary;
          boundary.reserve(unk_rep.size() / 2);

          for (const auto& kv : unk_rep) {
            const VoxelKey& k = kv.first;

            const VoxelKey nbrs[6] = {
                VoxelKey{k.x + 1, k.y, k.z}, VoxelKey{k.x - 1, k.y, k.z},
                VoxelKey{k.x, k.y + 1, k.z}, VoxelKey{k.x, k.y - 1, k.z},
                VoxelKey{k.x, k.y, k.z + 1}, VoxelKey{k.x, k.y, k.z - 1},
            };

            bool is_b = false;
            for (const auto& nb : nbrs) {
              if (unk_rep.find(nb) == unk_rep.end()) {
                is_b = true;
                break;
              }
            }
            if (is_b) boundary.push_back(k);
          }

          if (!boundary.empty()) {
            // Dedup inflated voxels (still hashing here)
            std::unordered_set<VoxelKey, VoxelKeyHash> added;
            added.max_load_factor(0.7f);
            // Reserve conservatively; avoid huge allocations
            added.reserve(std::min<std::size_t>(boundary.size() * 32, 2'000'000ULL));

            pts.reserve(pts.size() + boundary.size() * std::min<std::size_t>(offs_r.size(), 128));

            for (const auto& bk : boundary) {
              auto itp = unk_rep.find(bk);
              if (itp == unk_rep.end()) continue;

              const Vec3f& c = itp->second;

              for (const auto& o : offs_r) {
                const VoxelKey nk{bk.x + o.ix, bk.y + o.iy, bk.z + o.iz};
                if (!added.insert(nk).second) continue;

                Vec3f q;
                q << (c.x() + static_cast<float>(o.ix * res)),
                    (c.y() + static_cast<float>(o.iy * res)),
                    (c.z() + static_cast<float>(o.iz * res));
                pts.emplace_back(q);
              }
            }
          }
        }
      }
    }
  }

  // ------------------------------------------------------------------------
  // (B) Inflate dynamic obstacles (bbox-aware)
  // ------------------------------------------------------------------------
  if (obst_pos.empty()) return;

  for (size_t k = 0; k < obst_pos.size(); ++k) {
    const auto& O = obst_pos[k];
    const double ox = O.x();
    const double oy = O.y();
    const double oz = O.z();

    // Get bbox half-extents for this obstacle
    double hx = 0.4, hy = 0.4, hz = 0.4;  // default half-extents
    if (k < obst_bbox.size()) {
      hx = obst_bbox[k].x();
      hy = obst_bbox[k].y();
      hz = obst_bbox[k].z();
    }

    // Inflated bbox half-extents (inflate by motion radius r)
    const double hx_inf = hx + r;
    const double hy_inf = hy + r;
    const double hz_inf = hz + r;

    // Grid bounds in cells
    const int mx = static_cast<int>(std::ceil(hx_inf / res));
    const int my = static_cast<int>(std::ceil(hy_inf / res));
    const int mz = static_cast<int>(std::ceil(hz_inf / res));

    // Reserve space for this obstacle (rough estimate)
    pts.reserve(pts.size() + (2 * mx + 1) * (2 * my + 1) * (2 * mz + 1));

    // Generate grid points within the inflated bbox
    for (int ix = -mx; ix <= mx; ++ix) {
      const double dx = ix * res;

      for (int iy = -my; iy <= my; ++iy) {
        const double dy = iy * res;

        for (int iz = -mz; iz <= mz; ++iz) {
          const double dz = iz * res;

          // Check if point is within inflated bbox
          if (std::abs(dx) > hx_inf || std::abs(dy) > hy_inf || std::abs(dz) > hz_inf) continue;

          Vec3f p;
          p << static_cast<float>(ox + dx), static_cast<float>(oy + dy),
              static_cast<float>(oz + dz);
          pts.emplace_back(p);
        }
      }
    }
  }
}

void HGPManager::updateMap(
    double wdx,
    double wdy,
    double wdz,
    const Vec3f& center_map,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_unk,
    const vec_Vecf<3>& obst_pos,
    const vec_Vecf<3>& obst_bbox,
    double traj_max_time) {
  // Get the current time to see the computation time for readmap
  auto start_time = std::chrono::high_resolution_clock::now();

  mtx_map_util_.lock();
  map_util_->readMap(
      pclptr, pclptr_unk, (int)(wdx / res_), (int)(wdy / res_), (int)(wdz / res_), center_map,
      par_.z_min, par_.z_max, par_.inflation_hgp, obst_pos, obst_bbox, traj_max_time);
  mtx_map_util_.unlock();

  // Get the elapsed time for reading the map
  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  if (!map_initialized_) {
    map_initialized_ = true;
  }
}

bool HGPManager::isMapInitialized() const { return map_initialized_; }

void HGPManager::findClosestFreePoint(const Vec3f& point, Vec3f& closest_free_point) {
  mtx_map_util_.lock();
  map_util_->findClosestFreePoint(point, closest_free_point);
  mtx_map_util_.unlock();
}

int HGPManager::countUnknownCells() const { return map_util_for_planning_->countUnknownCells(); }

int HGPManager::getTotalNumCells() const { return map_util_for_planning_->getTotalNumCells(); }