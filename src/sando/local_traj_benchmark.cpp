/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <filesystem>
#include <fstream>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sando/gurobi_solver.hpp>
#include <sando/utils.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hgp/hgp_manager.hpp"
#include "hgp/termcolor.hpp"
#include "sando/sando_type.hpp"
#include "timer.hpp"

namespace fs = std::filesystem;
using namespace std::chrono;

using namespace sando;

using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;

struct DynFactorCaseRecord {
  size_t case_idx{0};
  bool success{false};
  double factor_used{0.0};
  double window_lo{0.0};
  double window_hi{0.0};
};

struct DynFactorConfigReport {
  std::string planner_name;
  int num_N{0};
  double initial_window_lo{0.0};
  double initial_window_hi{0.0};
  std::string csv_out;
  std::vector<DynFactorCaseRecord> records;
};

struct BenchResult {
  std::string file;
  bool success{false};
  bool gurobi_error{false};

  double per_opt_runtime_ms{0.0};    // from GRB Runtime * 1000
  double total_opt_runtime_ms{0.0};  // end-to-end wall time for this case
  double total_traj_time_sec{0.0};
  std::string status;

  Vec3d start{0, 0, 0};
  Vec3d goal{0, 0, 0};

  double factor_used{0.0};
  double cost_value{-100000.0};

  // -------- Constraint violation metrics (sampled at dt=dc) --------
  // Safe corridor (union of polytopes): for each sample, compute min_i max(A_i p - b_i).
  // max over samples of that value is reported here. <= 0 means "inside at least one polytope".
  double corridor_max_min_violation{
      0.0};  // max_t min_poly max(Ap-b) (positive means outside corridor union)
  double corridor_t_at_max{0.0};
  Vec3d corridor_p_at_max{0, 0, 0};
  int corridor_best_poly_idx_at_max{-1};

  // Dynamic constraints
  double v_max_observed{0.0};
  double v_max_excess{0.0};  // max( ||v|| - v_max ) (axis-wise in current code)
  double v_t_at_max{0.0};

  double a_max_observed{0.0};
  double a_max_excess{0.0};  // max( ||a|| - a_max )
  double a_t_at_max{0.0};

  double j_max_observed{0.0};
  double j_max_excess{0.0};  // max( ||j|| - j_max )
  double j_t_at_max{0.0};

  // Convenience flags (derived from the above using tolerances)
  bool corridor_violated{false};
  bool v_violated{false};
  bool a_violated{false};
  bool j_violated{false};

  // Per-sample violation counts (for rate = count / total * 100)
  int corridor_violation_count{0};
  int v_violation_count{0};
  int a_violation_count{0};
  int j_violation_count{0};
  int violation_total_samples{0};

  // -------- Smoothness metrics (sampled at dt=dc; jerk via finite-diff of accel) --------
  // S_jerk    = ∫_0^T ||j(t)|| dt
  // Sbar_jerk = sqrt( (1/T) ∫_0^T ||j(t)||^2 dt )   (RMS jerk; time-normalized)
  double jerk_smoothness_l1{0.0};
  double jerk_rms{0.0};

  double traj_length_m{0.0};  // ∫ ||v(t)|| dt

  // Visualization payloads (cached)
  decomp_ros_msgs::msg::PolyhedronArray poly_msg;
  visualization_msgs::msg::MarkerArray opt_traj_ma;     // committed colored
  visualization_msgs::msg::MarkerArray global_path_ma;  // marker array path
};

// ------------------------ helpers ------------------------

static void writeU32(std::ofstream& ofs, uint32_t v) {
  ofs.write(reinterpret_cast<const char*>(&v), sizeof(v));
}
static void writeD(std::ofstream& ofs, double v) {
  ofs.write(reinterpret_cast<const char*>(&v), sizeof(v));
}
static uint32_t readU32(std::ifstream& ifs) {
  uint32_t v;
  ifs.read(reinterpret_cast<char*>(&v), sizeof(v));
  if (!ifs) throw std::runtime_error("Corrupt .mysco2 (u32).");
  return v;
}
static double readD(std::ifstream& ifs) {
  double v;
  ifs.read(reinterpret_cast<char*>(&v), sizeof(v));
  if (!ifs) throw std::runtime_error("Corrupt .mysco2 (double).");
  return v;
}

static void fillStateFromPos(RobotState& s, const Vec3d& p) {
  s.pos = p;
  s.vel = Eigen::Vector3d::Zero();
  s.accel = Eigen::Vector3d::Zero();
}

// Build Polyhedron<3> from A x <= b
static Polyhedron<3> polyFromHalfspacesSeeded(Eigen::MatrixXd A, Eigen::VectorXd b,
                                              const Vec3f& seed, double eps,
                                              bool* did_flip_any = nullptr) {
  if (did_flip_any) *did_flip_any = false;

  // Ensure seed satisfies all inequalities by flipping rows that violate it.
  for (int r = 0; r < A.rows(); ++r) {
    const double v = A.row(r).dot(seed) - b(r);
    if (v > eps) {
      A.row(r) *= -1.0;
      b(r) *= -1.0;
      if (did_flip_any) *did_flip_any = true;
    }
  }

  Polyhedron<3> poly;
  for (int r = 0; r < A.rows(); ++r) {
    Vec3f a = A.row(r).transpose();
    const double norm = a.norm();
    if (norm < 1e-12) continue;

    // Inequality: a^T x <= b
    const Vec3f n = a / norm;
    const double d = b(r) / norm;
    const Vec3f p0 = n * d;

    // Hyperplane(point, normal)
    poly.add(Hyperplane<3>(p0, n));
  }

  return poly;
}

// Load one .mysco2 file and reconstruct:
// - path (vec_Vecf<3>)
// - seg_end_times
// - poly_out (vec_E<Polyhedron<3>>)
// - l_constraints (std::vector<LinearConstraint3D>)
static void loadMysco2(const fs::path& file, Vec3d& start, Vec3d& goal, vec_Vecf<3>& path,
                       std::vector<double>& seg_end_times, vec_E<Polyhedron<3>>& poly_out,
                       std::vector<LinearConstraint3D>& l_constraints, double poly_seed_eps,
                       bool debug_poly_check) {
  std::ifstream ifs(file, std::ios::binary);
  if (!ifs) throw std::runtime_error("Failed to open: " + file.string());

  char magic[8];
  ifs.read(magic, 8);
  if (!ifs) throw std::runtime_error("Corrupt .mysco2 (magic): " + file.string());

  const std::string m(magic, magic + 8);
  if (m.rfind("MYSCO2", 0) != 0) throw std::runtime_error("Bad magic in: " + file.string());

  const uint32_t version = readU32(ifs);
  if (version != 1) throw std::runtime_error("Unsupported .mysco2 version in: " + file.string());

  const double a0 = readD(ifs);
  const double a1 = readD(ifs);
  const double a2 = readD(ifs);

  const double g0 = readD(ifs);
  const double g1 = readD(ifs);
  const double g2 = readD(ifs);

  start = Vec3d(a0, a1, a2);
  goal = Vec3d(g0, g1, g2);

  const uint32_t num_path_pts = readU32(ifs);
  path.clear();
  path.reserve(num_path_pts);
  for (uint32_t i = 0; i < num_path_pts; ++i) {
    Vec3f p;
    const double px0 = readD(ifs);
    const double px1 = readD(ifs);
    const double px2 = readD(ifs);
    p << px0, px1, px2;
    path.push_back(p);
  }

  const uint32_t num_seg = readU32(ifs);
  seg_end_times.resize(num_seg);
  for (uint32_t i = 0; i < num_seg; ++i) seg_end_times[i] = readD(ifs);

  if (path.size() < 2 || (path.size() - 1) != num_seg)
    throw std::runtime_error("File inconsistent: path.size()-1 != num_seg in " + file.string());

  poly_out.clear();
  poly_out.resize(num_seg);

  l_constraints.clear();
  l_constraints.resize(num_seg);

  for (uint32_t si = 0; si < num_seg; ++si) {
    const uint32_t mplanes = readU32(ifs);

    Eigen::MatrixXd A(mplanes, 3);
    Eigen::VectorXd b(mplanes);

    for (uint32_t r = 0; r < mplanes; ++r)
      for (int c = 0; c < 3; ++c) A(r, c) = readD(ifs);

    for (uint32_t r = 0; r < mplanes; ++r) b(r) = readD(ifs);

    const Vec3f pt_inside = (path[si] + path[si + 1]) / 2.0;

    // Make seed-consistent halfspaces
    Eigen::MatrixXd A_fix = A;
    Eigen::VectorXd b_fix = b;
    for (int r = 0; r < A_fix.rows(); ++r) {
      const double v = A_fix.row(r).dot(pt_inside) - b_fix(r);
      if (v > poly_seed_eps) {
        A_fix.row(r) *= -1.0;
        b_fix(r) *= -1.0;
      }
    }

    poly_out[si] = polyFromHalfspacesSeeded(A_fix, b_fix, pt_inside, poly_seed_eps, nullptr);

    LinearConstraint3D lc;
    lc.A_ = A_fix;
    lc.b_ = b_fix;
    l_constraints[si] = lc;

    const double max_v = (lc.A_ * pt_inside - lc.b_).maxCoeff();
    if (max_v > 1e-4) {
      std::cerr << "[SFC LOAD] WARNING seg=" << si << " seed violates A_fix x<=b_fix by " << max_v
                << " file=" << file.string() << std::endl;
    }
  }
}

// ------------------------ constraint checks (sample at dt=dc) ------------------------

template <typename T, typename = void>
struct has_member_pos : std::false_type {};
template <typename T>
struct has_member_pos<T, std::void_t<decltype(std::declval<const T&>().pos)>> : std::true_type {};

template <typename T, typename = void>
struct has_member_vel : std::false_type {};
template <typename T>
struct has_member_vel<T, std::void_t<decltype(std::declval<const T&>().vel)>> : std::true_type {};

template <typename T, typename = void>
struct has_member_accel : std::false_type {};
template <typename T>
struct has_member_accel<T, std::void_t<decltype(std::declval<const T&>().accel)>> : std::true_type {
};

static inline Vec3d getPosSafe(const RobotState& s) {
  if constexpr (has_member_pos<RobotState>::value) return s.pos;
  return Vec3d::Zero();
}

static inline Vec3d getVelSafe(const RobotState& s) {
  if constexpr (has_member_vel<RobotState>::value) return s.vel;
  return Vec3d::Zero();
}

static inline Vec3d getAccelSafe(const RobotState& s) {
  if constexpr (has_member_accel<RobotState>::value) return s.accel;
  return Vec3d::Zero();
}

static inline double maxHalfspaceViolation(const LinearConstraint3D& lc, const Vec3f& p) {
  if (lc.A_.rows() == 0) return 0.0;
  Eigen::VectorXd d = lc.A_ * p - lc.b_;
  return d.maxCoeff();
}

struct ConstraintReport {
  double corridor_max_min_violation{0.0};
  double corridor_t_at_max{0.0};
  Vec3d corridor_p_at_max{0, 0, 0};
  int corridor_best_poly_idx_at_max{-1};

  double v_max_observed{0.0};
  double v_max_excess{0.0};
  double v_t_at_max{0.0};

  double a_max_observed{0.0};
  double a_max_excess{0.0};
  double a_t_at_max{0.0};

  double j_max_observed{0.0};
  double j_max_excess{0.0};
  double j_t_at_max{0.0};

  bool corridor_violated{false};
  bool v_violated{false};
  bool a_violated{false};
  bool j_violated{false};

  // Per-sample violation counts (for rate = count / total * 100)
  int corridor_violation_count{0};
  int v_violation_count{0};
  int a_violation_count{0};
  int j_violation_count{0};
  int violation_total_samples{0};

  // -------- Smoothness metrics (sampled at dt=dc; jerk via finite-diff of accel) --------
  // S_jerk    = ∫_0^T ||j(t)|| dt
  // Sbar_jerk = sqrt( (1/T) ∫_0^T ||j(t)||^2 dt )   (RMS jerk; time-normalized)
  double jerk_smoothness_l1{0.0};
  double jerk_rms{0.0};

  double traj_length_m{0.0};  // ∫ ||v(t)|| dt
};

static ConstraintReport analyzeConstraintsSampled(
    const std::vector<RobotState>& samples, const std::vector<LinearConstraint3D>& l_constraints,
    double dc, double v_max, double a_max, double j_max, double corridor_tol = 1e-6,
    double dyn_tol = 1e-6) {
  ConstraintReport rep;

  if (samples.empty()) return rep;

  const int num_poly = static_cast<int>(l_constraints.size());
  const bool have_polys = (num_poly > 0);

  auto maxAbsComponent = [](const Vec3d& x) -> double { return x.cwiseAbs().maxCoeff(); };

  // Corridor union
  if (have_polys) {
    double worst = -std::numeric_limits<double>::infinity();
    double worst_t = 0.0;
    Vec3d worst_p(0, 0, 0);
    int worst_best_idx = -1;

    for (size_t k = 0; k < samples.size(); ++k) {
      const double t = static_cast<double>(k) * dc;
      const Vec3d p_d = getPosSafe(samples[k]);
      const Vec3f p = p_d.cast<double>();

      double best = std::numeric_limits<double>::infinity();
      int best_idx = -1;

      for (int i = 0; i < num_poly; ++i) {
        const double v = maxHalfspaceViolation(l_constraints[i], p);
        if (v < best) {
          best = v;
          best_idx = i;
        }
      }

      if (best > corridor_tol) ++rep.corridor_violation_count;

      if (best > worst) {
        worst = best;
        worst_t = t;
        worst_p = p_d;
        worst_best_idx = best_idx;
      }
    }

    rep.corridor_max_min_violation = worst;
    rep.corridor_t_at_max = worst_t;
    rep.corridor_p_at_max = worst_p;
    rep.corridor_best_poly_idx_at_max = worst_best_idx;
    rep.corridor_violated = (worst > corridor_tol);
  }

  // Build v,a,j
  std::vector<Vec3d> vel(samples.size(), Vec3d::Zero());
  std::vector<Vec3d> acc(samples.size(), Vec3d::Zero());
  std::vector<Vec3d> jerk(samples.size(), Vec3d::Zero());

  if constexpr (has_member_vel<RobotState>::value) {
    for (size_t k = 0; k < samples.size(); ++k) vel[k] = samples[k].vel;
  } else {
    if (samples.size() >= 2 && dc > 0.0) {
      vel[0] = Vec3d::Zero();
      for (size_t k = 1; k < samples.size(); ++k)
        vel[k] = (getPosSafe(samples[k]) - getPosSafe(samples[k - 1])) / dc;
    }
  }

  double length_m = 0.0;

  if (samples.size() >= 2 && dc > 0.0) {
    // If you have velocity vectors vel[k]
    for (size_t k = 1; k < samples.size(); ++k) {
      const double s0 = vel[k - 1].norm();
      const double s1 = vel[k].norm();
      length_m += 0.5 * (s0 + s1) * dc;  // trapezoidal rule
    }
  }

  // store
  rep.traj_length_m = length_m;

  if constexpr (has_member_accel<RobotState>::value) {
    for (size_t k = 0; k < samples.size(); ++k) acc[k] = samples[k].accel;
  } else {
    if (samples.size() >= 2 && dc > 0.0) {
      acc[0] = Vec3d::Zero();
      for (size_t k = 1; k < samples.size(); ++k) acc[k] = (vel[k] - vel[k - 1]) / dc;
    }
  }

  if constexpr (has_member_accel<RobotState>::value == false) {
    for (size_t k = 1; k < samples.size(); ++k) jerk[k] = samples[k].jerk;
  } else {
    if (samples.size() >= 2 && dc > 0.0) {
      jerk[0] = Vec3d::Zero();
      for (size_t k = 1; k < samples.size(); ++k) jerk[k] = (acc[k] - acc[k - 1]) / dc;
    }
  }

  constexpr double buf = 1e-6;

  double vmax_obs = 0.0, vmax_ex = -std::numeric_limits<double>::infinity(), vmax_t = 0.0;
  double amax_obs = 0.0, amax_ex = -std::numeric_limits<double>::infinity(), amax_t = 0.0;
  double jmax_obs = 0.0, jmax_ex = -std::numeric_limits<double>::infinity(), jmax_t = 0.0;

  for (size_t k = 0; k < samples.size(); ++k) {
    const double t = static_cast<double>(k) * dc;

    const double vcomp_max = maxAbsComponent(vel[k]);
    vmax_obs = std::max(vmax_obs, vcomp_max);
    const double vex = vcomp_max - (v_max + buf);
    if (vex > dyn_tol) ++rep.v_violation_count;
    if (vex > vmax_ex) {
      vmax_ex = vex;
      vmax_t = t;
    }

    const double acomp_max = maxAbsComponent(acc[k]);
    amax_obs = std::max(amax_obs, acomp_max);
    const double aex = acomp_max - (a_max + buf);
    if (aex > dyn_tol) ++rep.a_violation_count;
    if (aex > amax_ex) {
      amax_ex = aex;
      amax_t = t;
    }

    const double jcomp_max = maxAbsComponent(jerk[k]);
    jmax_obs = std::max(jmax_obs, jcomp_max);
    const double jex = jcomp_max - (j_max + buf);
    if (jex > dyn_tol) ++rep.j_violation_count;
    if (jex > jmax_ex) {
      jmax_ex = jex;
      jmax_t = t;
    }
  }

  rep.violation_total_samples = static_cast<int>(samples.size());

  rep.v_max_observed = vmax_obs;
  rep.v_max_excess = std::max(0.0, vmax_ex);
  rep.v_t_at_max = vmax_t;
  rep.v_violated = (vmax_ex > dyn_tol);

  rep.a_max_observed = amax_obs;
  rep.a_max_excess = std::max(0.0, amax_ex);
  rep.a_t_at_max = amax_t;
  rep.a_violated = (amax_ex > dyn_tol);

  rep.j_max_observed = jmax_obs;
  rep.j_max_excess = std::max(0.0, jmax_ex);
  rep.j_t_at_max = jmax_t;
  rep.j_violated = (jmax_ex > dyn_tol);

  double jerk_l1 = 0.0;      // ∫ ||j|| dt   (approx)
  double jerk_l2_int = 0.0;  // ∫ ||j||^2 dt (approx)

  if (samples.size() >= 2 && dc > 0.0) {
    for (size_t k = 1; k < samples.size(); ++k) {
      // Smoothness metrics use Euclidean norm.
      const double jnorm = jerk[k].norm();
      jerk_l1 += jnorm * dc;
      jerk_l2_int += (jnorm * jnorm) * dc;
    }
  }

  // Smoothness results.
  // Use the sampled trajectory duration implied by samples + dc for numerical consistency.
  const double T =
      (samples.size() >= 2 && dc > 0.0) ? (static_cast<double>(samples.size() - 1) * dc) : 0.0;
  rep.jerk_smoothness_l1 = jerk_l1;
  rep.jerk_rms = (T > 0.0) ? std::sqrt(jerk_l2_int / T) : 0.0;

  return rep;
}

// ------------------------ trajectory dump helpers (NEW) ------------------------

static inline bool ensureDir(const fs::path& p) {
  std::error_code ec;
  if (fs::exists(p, ec)) return fs::is_directory(p, ec);
  return fs::create_directories(p, ec);
}

static inline std::string sanitizeFilename(std::string s) {
  for (char& c : s) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (!(std::isalnum(uc) || c == '-' || c == '_' || c == '.')) c = '_';
  }
  return s;
}

// Dump goal_setpoints (sampled at dt=dc) to CSV.
// If traj_dump_dt > dc, downsample by stride = round(traj_dump_dt/dc), clamped >= 1.
static void dumpTrajectoryCsvV1(const fs::path& out_csv, const std::string& planner_name,
                                const std::string& case_file_basename, const std::string& frame_id,
                                const std::vector<RobotState>& samples, double dc,
                                double traj_dump_dt_requested, double factor_used,
                                double cost_value, double total_traj_time_sec) {
  if (samples.empty()) return;

  if (dc <= 0.0) dc = 0.01;

  int stride = 1;
  if (traj_dump_dt_requested > 0.0) {
    const double ratio = traj_dump_dt_requested / dc;
    const long long s = llround(ratio);
    stride = static_cast<int>(std::max<long long>(1LL, s));
  }
  const double dump_dt = stride * dc;

  std::ofstream ofs(out_csv);
  if (!ofs) throw std::runtime_error("Failed to open traj csv for write: " + out_csv.string());

  ofs << "# traj_format: sando_local_traj_csv_v1\n";
  ofs << "# planner_name: " << planner_name << "\n";
  ofs << "# case_file: " << case_file_basename << "\n";
  ofs << "# frame_id: " << frame_id << "\n";
  ofs << "# dc_sec: " << std::fixed << std::setprecision(9) << dc << "\n";
  ofs << "# dump_dt_sec: " << std::fixed << std::setprecision(9) << dump_dt << "\n";
  ofs << "# factor_used: " << std::fixed << std::setprecision(9) << factor_used << "\n";
  ofs << "# cost_value: " << std::fixed << std::setprecision(9) << cost_value << "\n";
  ofs << "# total_traj_time_sec: " << std::fixed << std::setprecision(9) << total_traj_time_sec
      << "\n";
  ofs << "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz\n";

  ofs << std::fixed << std::setprecision(9);

  Vec3d prev_a = getAccelSafe(samples.front());
  for (size_t k = 0; k < samples.size(); k += static_cast<size_t>(stride)) {
    const double t = static_cast<double>(k) * dc;

    const Vec3d p = getPosSafe(samples[k]);
    const Vec3d v = getVelSafe(samples[k]);
    const Vec3d a = getAccelSafe(samples[k]);

    Vec3d j = Vec3d::Zero();
    if (k > 0 && dc > 0.0) j = (a - prev_a) / dc;

    // Update prev_a for next jerk computation (even if downsampling)
    prev_a = a;

    ofs << t << "," << p.x() << "," << p.y() << "," << p.z() << "," << v.x() << "," << v.y() << ","
        << v.z() << "," << a.x() << "," << a.y() << "," << a.z() << "," << j.x() << "," << j.y()
        << "," << j.z() << "\n";
  }

  ofs.flush();
}

// ------------------------ node ------------------------

class LocalTrajBenchmarkNode final : public rclcpp::Node {
 public:
  LocalTrajBenchmarkNode() : Node("local_traj_benchmark_node") {
    // I/O + visualization
    declare_parameter<std::string>("sfc_dir", "");
    declare_parameter<std::string>("file_ext", ".mysco2");
    declare_parameter<std::string>("frame_id", "map");

    declare_parameter<std::string>("poly_topic", "/NX01/poly_safe");
    declare_parameter<std::string>("traj_committed_topic", "/NX01/traj_committed_colored");
    declare_parameter<std::string>("hgp_path_topic", "/NX01/hgp_path_marker");

    declare_parameter<bool>("visualize", true);
    declare_parameter<double>("playback_period_sec", 0.1);
    declare_parameter<double>("solve_delay_sec",
                              0.0);  // delay between solving each case (for visualization)
    declare_parameter<bool>("latched", true);

    // NEW: trajectory dump settings
    declare_parameter<bool>("traj_dump_enable", true);
    declare_parameter<std::string>("traj_dump_root_dir", "");
    declare_parameter<double>("traj_dump_dt", -1.0);

    // Solver control
    declare_parameter<double>("assumed_last_replan_time_sec", 0.05);

    // Minimal subset of sando.yaml that SolverGurobi needs
    declare_parameter<int>("num_N", 6);
    declare_parameter<double>("dc", 0.01);

    declare_parameter<double>("x_min", -10.0);
    declare_parameter<double>("x_max", 10.0);
    declare_parameter<double>("y_min", -10.0);
    declare_parameter<double>("y_max", 10.0);
    declare_parameter<double>("z_min", 0.0);
    declare_parameter<double>("z_max", 5.0);

    declare_parameter<double>("v_max", 1.0);
    declare_parameter<double>("a_max", 2.0);
    declare_parameter<double>("j_max", 3.0);
    declare_parameter<std::string>("dynamic_constraint_type", "Linf");

    declare_parameter<double>("factor_constant_step_size", 0.1);

    declare_parameter<double>("w_max", 0.5);

    declare_parameter<double>("max_gurobi_comp_time_sec", 0.5);
    declare_parameter<double>("per_case_timeout_sec", 1.0);  // timeout per optimization case
    declare_parameter<double>("jerk_smooth_weight", 1.0e+1);

    declare_parameter<bool>("debug_verbose", false);

    declare_parameter<double>("poly_seed_eps", 1e-6);
    declare_parameter<bool>("debug_poly_check", true);

    declare_parameter<std::vector<std::string>>("planner_names", std::vector<std::string>{});
    declare_parameter<std::vector<int64_t>>("num_N_list", std::vector<int64_t>{});
    declare_parameter<std::vector<double>>("factor_initial_list",
                                           std::vector<double>{2.0, 1.0, 1.0});
    declare_parameter<std::vector<double>>("factor_final_list", std::vector<double>{4.0, 3.0, 2.0});

    declare_parameter<bool>("use_dynamic_factor", false);
    declare_parameter<std::vector<double>>("dynamic_factor_initial_mean_list",
                                           std::vector<double>{1.5, 1.5, 1.5});
    declare_parameter<double>("dynamic_factor_k_radius", 0.4);

    declare_parameter<std::string>("planner_name", "SANDO");
    declare_parameter<bool>("use_single_threaded", false);

    declare_parameter<bool>("using_variable_elimination", true);
    declare_parameter<std::string>("output_dir_override", "");
    declare_parameter<std::string>("benchmark_data_dir", "");

    // Read params
    std::vector<std::string> planner_names = this->get_parameter("planner_names").as_string_array();
    std::vector<int64_t> num_N_list_64 = this->get_parameter("num_N_list").as_integer_array();

    if (planner_names.empty())
      planner_names.push_back(this->get_parameter("planner_name").as_string());

    std::vector<int> num_N_list;
    num_N_list.reserve(num_N_list_64.size());
    for (auto v : num_N_list_64) num_N_list.push_back(static_cast<int>(v));

    if (num_N_list.empty()) num_N_list.push_back(this->get_parameter("num_N").as_int());

    std::vector<double> factor_initial_list =
        this->get_parameter("factor_initial_list").as_double_array();
    std::vector<double> factor_final_list =
        this->get_parameter("factor_final_list").as_double_array();

    use_dynamic_factor_ = this->get_parameter("use_dynamic_factor").as_bool();
    std::vector<double> dynamic_factor_initial_mean_list =
        this->get_parameter("dynamic_factor_initial_mean_list").as_double_array();
    dynamic_factor_k_radius_ = this->get_parameter("dynamic_factor_k_radius").as_double();

    use_single_threaded_ = get_parameter("use_single_threaded").as_bool();
    sfc_dir_ = get_parameter("sfc_dir").as_string();
    file_ext_ = get_parameter("file_ext").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    poly_topic_ = get_parameter("poly_topic").as_string();
    traj_committed_topic_ = get_parameter("traj_committed_topic").as_string();
    hgp_path_topic_ = get_parameter("hgp_path_topic").as_string();

    visualize_ = get_parameter("visualize").as_bool();
    playback_period_sec_ = get_parameter("playback_period_sec").as_double();
    solve_delay_sec_ = get_parameter("solve_delay_sec").as_double();
    latched_ = get_parameter("latched").as_bool();

    assumed_last_replan_time_sec_ = get_parameter("assumed_last_replan_time_sec").as_double();

    // Dump params (NEW)
    traj_dump_enable_ = get_parameter("traj_dump_enable").as_bool();
    traj_dump_root_dir_ = get_parameter("traj_dump_root_dir").as_string();
    traj_dump_dt_ = get_parameter("traj_dump_dt").as_double();

    // Fill parameters struct for SolverGurobi
    par_.dc = get_parameter("dc").as_double();

    par_.x_min = get_parameter("x_min").as_double();
    par_.x_max = get_parameter("x_max").as_double();
    par_.y_min = get_parameter("y_min").as_double();
    par_.y_max = get_parameter("y_max").as_double();
    par_.z_min = get_parameter("z_min").as_double();
    par_.z_max = get_parameter("z_max").as_double();

    par_.v_max = get_parameter("v_max").as_double();
    par_.a_max = get_parameter("a_max").as_double();
    par_.j_max = get_parameter("j_max").as_double();
    par_.dynamic_constraint_type = get_parameter("dynamic_constraint_type").as_string();

    par_.factor_constant_step_size = get_parameter("factor_constant_step_size").as_double();

    par_.w_max = get_parameter("w_max").as_double();

    par_.max_gurobi_comp_time_sec = get_parameter("max_gurobi_comp_time_sec").as_double();
    per_case_timeout_sec_ = get_parameter("per_case_timeout_sec").as_double();
    par_.jerk_smooth_weight = get_parameter("jerk_smooth_weight").as_double();
    par_.debug_verbose = get_parameter("debug_verbose").as_bool();

    par_.using_variable_elimination = get_parameter("using_variable_elimination").as_bool();

    poly_seed_eps_ = get_parameter("poly_seed_eps").as_double();
    debug_poly_check_ = get_parameter("debug_poly_check").as_bool();

    std::string output_dir_override = get_parameter("output_dir_override").as_string();
    benchmark_data_dir_ = get_parameter("benchmark_data_dir").as_string();

    for (const auto& planner_name : planner_names) {
      dyn_factor_reports_.clear();

      for (int idx = 0; idx < (int)num_N_list.size(); ++idx) {
        int num_N = num_N_list[idx];
        par_.factor_initial = factor_initial_list[idx];
        par_.factor_final = factor_final_list[idx];

        factors_.clear();
        dynamic_factor_initial_success_ = false;

        if (use_dynamic_factor_) {
          // Dynamic k-factor window approach
          double initial_mean = dynamic_factor_initial_mean_list[idx];
          dynamic_factor_initial_mean_ = initial_mean;
          int num_factors =
              static_cast<int>((2 * dynamic_factor_k_radius_) / par_.factor_constant_step_size) + 1;
          for (int i = 0; i < num_factors; i++) {
            double f = initial_mean - dynamic_factor_k_radius_ + i * par_.factor_constant_step_size;
            if (f >= 1.0) factors_.push_back(f);
          }
        } else {
          // Fixed factor range
          for (double f = par_.factor_initial; f <= par_.factor_final + 1e-6;
               f += par_.factor_constant_step_size) {
            factors_.push_back(f);
          }
        }

        planner_name_ = planner_name;
        par_.num_N = num_N;

        // Determine output directory and filename
        std::string base_dir;
        std::string filename_suffix = "";

        if (!output_dir_override.empty()) {
          // Use override directory (e.g., for VE comparison)
          base_dir = output_dir_override;
          // Add VE status to filename when using override
          filename_suffix = par_.using_variable_elimination ? "_with_ve" : "_without_ve";
        } else {
          // Normal mode: single_thread or multi_thread subdirectories
          std::string thread_string = use_single_threaded_ ? "single_thread" : "multi_thread";
          base_dir = benchmark_data_dir_ + "/" + thread_string;
        }

        csv_out_ = base_dir + "/" + planner_name_ + "_" + std::to_string(par_.num_N) +
                   filename_suffix + "_benchmark.csv";

        // Timing profiling log (only for multi-threaded)
        timing_log_path_ = base_dir + "/" + planner_name_ + "_" + std::to_string(par_.num_N) +
                           filename_suffix + "_timing_log.csv";
        // Clear previous log
        if (!use_single_threaded_) {
          std::ofstream(timing_log_path_, std::ios::trunc);
        }

        // NEW: derive dump directory for this run
        if (traj_dump_enable_) {
          fs::path root;
          if (!traj_dump_root_dir_.empty())
            root = fs::path(traj_dump_root_dir_);
          else
            root = fs::path(csv_out_).parent_path() / "traj_dump";

          traj_dump_run_dir_ =
              (root / (planner_name_ + "_N" + std::to_string(par_.num_N))).string();

          if (!ensureDir(fs::path(traj_dump_run_dir_))) {
            RCLCPP_WARN(get_logger(),
                        "Failed to create traj dump dir: %s (disabling dump for this run)",
                        traj_dump_run_dir_.c_str());
            traj_dump_enable_this_run_ = false;
          } else {
            traj_dump_enable_this_run_ = true;
            RCLCPP_INFO(get_logger(), "Trajectory dump enabled. dir=%s dt_req=%.4f (dc=%.4f)",
                        traj_dump_run_dir_.c_str(), traj_dump_dt_, par_.dc);
          }
        } else {
          traj_dump_enable_this_run_ = false;
        }

        RCLCPP_INFO(get_logger(),
                    "Benchmarking planner=%s num_N=%d factors=[%.2f .. %.2f] step=%.2f (%s) cases "
                    "in %s (output %s) using %s",
                    planner_name_.c_str(), par_.num_N, factors_.front(), factors_.back(),
                    par_.factor_constant_step_size,
                    use_dynamic_factor_ ? "dynamic k-factor" : "fixed range", sfc_dir_.c_str(),
                    csv_out_.c_str(), use_single_threaded_ ? "single thread" : "multiple threads");

        // Create one solver per factor (persistent, reused across cases)
        // For dynamic factor mode, allocate for max possible window size
        // (window may grow if factors shift above 1.0 after recentering)
        size_t max_num_factors = factors_.size();
        if (use_dynamic_factor_) {
          max_num_factors =
              static_cast<size_t>((2 * dynamic_factor_k_radius_) / par_.factor_constant_step_size) +
              1;
        }
        whole_traj_solver_ptrs_.clear();
        whole_traj_solver_ptrs_.reserve(max_num_factors);

        // Hybrid threading: distribute CPU cores across external factor
        // threads so each Gurobi instance gets multiple internal threads.
        // This avoids the pathology where each Gurobi model is limited to
        // 1 thread while many factors compete for CPU resources.
        const int num_cores = static_cast<int>(std::thread::hardware_concurrency());
        const int grb_threads_per_solver =
            use_single_threaded_ ? 0  // 0 = Gurobi auto (all cores)
                                 : std::max(1, num_cores / static_cast<int>(max_num_factors));

        RCLCPP_INFO(get_logger(),
                    "Hybrid threading: %d cores, %zu factors, %d Gurobi threads/solver", num_cores,
                    max_num_factors, grb_threads_per_solver);

        for (size_t i = 0; i < max_num_factors; ++i) {
          auto s = std::make_shared<SolverGurobi>();
          s->setPlannerName(planner_name_);
          if (!use_single_threaded_) s->setGurobiThreads(grb_threads_per_solver);
          s->initializeSolver(par_);
          whole_traj_solver_ptrs_.push_back(s);
        }

        // Workers
        ellip_workers_.resize(whole_traj_solver_ptrs_.size());

        // Publishers
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliable();
        if (latched_) qos.transient_local();

        pub_poly_ = create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(poly_topic_, qos);
        pub_traj_committed_colored_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(traj_committed_topic_, 10);
        pub_hgp_path_marker_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(hgp_path_topic_, 10);

        // Record initial window for dynamic factor report
        dyn_factor_case_records_.clear();
        double initial_window_lo = factors_.empty() ? 0.0 : factors_.front();
        double initial_window_hi = factors_.empty() ? 0.0 : factors_.back();

        // Load + solve
        loadAll();
        solveAll();
        writeCsv();

        // Finalize factor config report
        {
          DynFactorConfigReport cfg_report;
          cfg_report.planner_name = planner_name_;
          cfg_report.num_N = par_.num_N;
          cfg_report.initial_window_lo = initial_window_lo;
          cfg_report.initial_window_hi = initial_window_hi;
          cfg_report.csv_out = csv_out_;
          cfg_report.records = dyn_factor_case_records_;
          dyn_factor_reports_.push_back(cfg_report);
        }

        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "Config complete! Results saved to:");
        RCLCPP_INFO(get_logger(), "  %s", csv_out_.c_str());
        RCLCPP_INFO(get_logger(), "========================================");
      }

      // Write factor report for this planner (all N configs)
      if (!dyn_factor_reports_.empty()) {
        writeFactorReport();
      }
    }

    // All configs complete - exit node
    RCLCPP_INFO(get_logger(), "\n========================================");
    RCLCPP_INFO(get_logger(), "ALL BENCHMARKS COMPLETE!");
    RCLCPP_INFO(get_logger(), "========================================");

    // Only create playback timer if playback_period_sec > 0 (for post-solve review)
    if (visualize_ && playback_period_sec_ > 0.0) {
      RCLCPP_INFO(get_logger(),
                  "Starting post-solve playback (period=%.2fs). Press Ctrl+C to exit.",
                  playback_period_sec_);
      playback_timer_ = create_wall_timer(std::chrono::duration<double>(playback_period_sec_),
                                          std::bind(&LocalTrajBenchmarkNode::publishNext, this));
    } else {
      RCLCPP_INFO(get_logger(), "Benchmarks complete. Exiting in 0.5 seconds...");
      // Use a one-shot timer to exit the process cleanly
      shutdown_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
        RCLCPP_INFO(get_logger(), "Exiting process.");
        std::exit(0);  // Clean exit - launch will handle the rest
      });
    }
  }

 private:
  void loadAll() {
    results_.clear();

    if (!fs::exists(sfc_dir_)) throw std::runtime_error("sfc_dir does not exist: " + sfc_dir_);

    std::vector<fs::path> files;
    for (const auto& ent : fs::directory_iterator(sfc_dir_)) {
      if (!ent.is_regular_file()) continue;
      const auto p = ent.path();
      if (p.extension() == file_ext_) files.push_back(p);
    }

    std::sort(files.begin(), files.end());

    for (const auto& f : files) {
      BenchResult r;
      r.file = f.string();
      results_.push_back(std::move(r));
    }
  }

  void maybeDumpTrajectory(const std::string& case_fname, const std::vector<RobotState>& goal_setpoints,
                           const BenchResult& r) {
    if (!traj_dump_enable_this_run_) return;

    const std::string base = sanitizeFilename("traj_" + planner_name_ + "_N" +
                                              std::to_string(par_.num_N) + "__" + case_fname);
    const fs::path out_csv = fs::path(traj_dump_run_dir_) / (base + ".csv");

    dumpTrajectoryCsvV1(out_csv, planner_name_, case_fname, frame_id_, goal_setpoints, par_.dc,
                        traj_dump_dt_, r.factor_used, r.cost_value, r.total_traj_time_sec);
  }

  // Per-thread timing breakdown for profiling multi-threaded overhead
  struct ThreadTiming {
    double thread_start_ms{0.0};    // time from cumul_t0 to thread body start
    double setup_ms{0.0};           // setX0 + setXf + setInitialDt + setT0 + setPolytopes
    double generate_ms{0.0};        // generateNewTrajectory total (constraint setup + Gurobi solve)
    double gurobi_runtime_ms{0.0};  // Gurobi's internal solve time only
    double thread_total_ms{0.0};    // total time inside thread body
    double factor{0.0};
    bool success{false};
    std::string msg;
    // Sub-step breakdown from inside generateNewTrajectory
    double findDT_ms{0.0};
    double setX_ms{0.0};
    double polytopes_ms{0.0};
    double dynamic_ms{0.0};
    double objective_ms{0.0};
    double mapsize_ms{0.0};
    double callOptimizer_ms{0.0};
    double postsolve_ms{0.0};
  };

  void writeTimingLog(const std::string& log_path, size_t case_idx, const std::string& planner_name,
                      double cumul_total_ms, double poll_first_ready_ms, double poll_success_ms,
                      const std::vector<ThreadTiming>& timings) {
    // Append mode — header written once
    bool write_header = !fs::exists(log_path) || fs::file_size(log_path) == 0;
    std::ofstream ofs(log_path, std::ios::app);
    if (!ofs.is_open()) return;
    if (write_header) {
      ofs << "case_idx,planner,factor,success,"
             "thread_start_ms,setup_ms,generate_ms,gurobi_runtime_ms,thread_total_ms,"
             "findDT_ms,setX_ms,polytopes_ms,dynamic_ms,objective_ms,mapsize_ms,callOptimizer_ms,"
             "postsolve_ms,"
             "poll_first_ready_ms,poll_success_ms,cumul_total_ms\n";
    }
    for (const auto& t : timings) {
      ofs << case_idx << "," << planner_name << "," << std::fixed << std::setprecision(3)
          << t.factor << "," << (t.success ? 1 : 0) << "," << t.thread_start_ms << "," << t.setup_ms
          << "," << t.generate_ms << "," << t.gurobi_runtime_ms << "," << t.thread_total_ms << ","
          << t.findDT_ms << "," << t.setX_ms << "," << t.polytopes_ms << "," << t.dynamic_ms << ","
          << t.objective_ms << "," << t.mapsize_ms << "," << t.callOptimizer_ms << ","
          << t.postsolve_ms << "," << poll_first_ready_ms << "," << poll_success_ms << ","
          << cumul_total_ms << "\n";
    }
  }

  void solveAll() {
    using ThreadRet = std::tuple<bool, bool, double, double, std::string, ThreadTiming>;
    // (success, gurobi_error, per_opt_runtime_ms, factor, msg, timing)

    auto maxViolation = [](const LinearConstraint3D& lc, const Vec3f& p) -> double {
      if (lc.A_.rows() == 0) return 0.0;
      Eigen::VectorXd d = lc.A_ * p - lc.b_;
      return d.maxCoeff();
    };

    auto abStats = [](const LinearConstraint3D& lc, double& min_row_norm, double& max_row_norm,
                      double& min_b, double& max_b) {
      min_row_norm = std::numeric_limits<double>::infinity();
      max_row_norm = 0.0;
      min_b = std::numeric_limits<double>::infinity();
      max_b = -std::numeric_limits<double>::infinity();

      for (int r = 0; r < lc.A_.rows(); ++r) {
        const double rn = lc.A_.row(r).norm();
        min_row_norm = std::min(min_row_norm, rn);
        max_row_norm = std::max(max_row_norm, rn);
      }
      for (int r = 0; r < lc.b_.size(); ++r) {
        min_b = std::min(min_b, lc.b_(r));
        max_b = std::max(max_b, lc.b_(r));
      }

      if (!std::isfinite(min_row_norm)) min_row_norm = 0.0;
      if (!std::isfinite(min_b)) min_b = 0.0;
      if (!std::isfinite(max_b)) max_b = 0.0;
    };

    auto applyConstraintReport = [](BenchResult& out, const ConstraintReport& rep) {
      out.corridor_max_min_violation = rep.corridor_max_min_violation;
      out.corridor_t_at_max = rep.corridor_t_at_max;
      out.corridor_p_at_max = rep.corridor_p_at_max;
      out.corridor_best_poly_idx_at_max = rep.corridor_best_poly_idx_at_max;

      out.v_max_observed = rep.v_max_observed;
      out.v_max_excess = rep.v_max_excess;
      out.v_t_at_max = rep.v_t_at_max;

      out.a_max_observed = rep.a_max_observed;
      out.a_max_excess = rep.a_max_excess;
      out.a_t_at_max = rep.a_t_at_max;

      out.j_max_observed = rep.j_max_observed;
      out.j_max_excess = rep.j_max_excess;
      out.j_t_at_max = rep.j_t_at_max;

      out.corridor_violated = rep.corridor_violated;
      out.v_violated = rep.v_violated;
      out.a_violated = rep.a_violated;
      out.j_violated = rep.j_violated;

      out.corridor_violation_count = rep.corridor_violation_count;
      out.v_violation_count = rep.v_violation_count;
      out.a_violation_count = rep.a_violation_count;
      out.j_violation_count = rep.j_violation_count;
      out.violation_total_samples = rep.violation_total_samples;

      out.jerk_smoothness_l1 = rep.jerk_smoothness_l1;
      out.jerk_rms = rep.jerk_rms;

      out.traj_length_m = rep.traj_length_m;
    };

    for (size_t case_idx = 0; case_idx < results_.size(); ++case_idx) {
      auto& r = results_[case_idx];
      const std::string fname = fs::path(r.file).filename().string();

      try {
        Vec3d start, goal;
        vec_Vecf<3> path;
        std::vector<double> seg_end_times;
        vec_E<Polyhedron<3>> poly_out;
        std::vector<LinearConstraint3D> l_constraints;

        loadMysco2(r.file, start, goal, path, seg_end_times, poly_out, l_constraints,
                   poly_seed_eps_, debug_poly_check_);

        r.start = start;
        r.goal = goal;

        // Cache corridor polyhedra for RViz
        {
          auto msg = DecompROS::polyhedron_array_to_ros(poly_out);
          msg.header.frame_id = frame_id_;
          msg.header.stamp = now();
          msg.lifetime = rclcpp::Duration::from_seconds(1.0);
          r.poly_msg = msg;
        }

        // Global path marker array
        {
          r.global_path_ma.markers.clear();
          vectorOfVectors2MarkerArray(path, &r.global_path_ma, color(RED));
        }

        const int num_seg = static_cast<int>(l_constraints.size());

        if (num_seg > par_.num_N) {
          r.success = false;
          r.status = "SKIP: polytopes(" + std::to_string(num_seg) + ") > num_N(" +
                     std::to_string(par_.num_N) + ")";
          const auto t1 = steady_clock::now();
          r.total_opt_runtime_ms = 1000000.0;
          continue;
        }

        if (num_seg <= 0) throw std::runtime_error("No segments/constraints loaded.");

        // Sanity checks
        const Vec3f p_start = path.front();
        const Vec3f p_goal = path.back();

        const double v_start = maxViolation(l_constraints.front(), p_start);
        const double v_goal = maxViolation(l_constraints.back(), p_goal);

        int bad_mid_count = 0;
        for (int i = 0; i < num_seg; ++i) {
          const Vec3f mid = 0.5 * (path[i] + path[i + 1]);
          const double vm = maxViolation(l_constraints[i], mid);
          if (vm > 1e-6) bad_mid_count++;
        }

        // If start/goal/mids violate, skip
        if (v_start > 1e-5 || v_goal > 1e-5 || bad_mid_count > 0) {
          r.success = false;
          r.gurobi_error = false;
          r.status = "BAD_CONSTRAINTS: start/goal/mid violates corridor (see logs)";
          const auto t1 = steady_clock::now();
          r.total_opt_runtime_ms = 1000000.0;
          continue;
        }

        // Reset all solvers
        for (auto& s : whole_traj_solver_ptrs_) s->resetToNominalState();

        // Reset factor window to initial for each case (independent benchmarking)
        if (use_dynamic_factor_) {
          factors_.clear();
          int num_factors =
              static_cast<int>((2 * dynamic_factor_k_radius_) / par_.factor_constant_step_size) + 1;
          for (int i = 0; i < num_factors; i++) {
            double f = dynamic_factor_initial_mean_ - dynamic_factor_k_radius_ +
                       i * par_.factor_constant_step_size;
            if (f >= 1.0) factors_.push_back(f);
          }
        }

        // Setup states
        RobotState A, E;
        fillStateFromPos(A, start);
        fillStateFromPos(E, goal);

        // initial_dt
        whole_traj_solver_ptrs_[0]->setX0(A);
        whole_traj_solver_ptrs_[0]->setXf(E);
        const double initial_dt = whole_traj_solver_ptrs_[0]->getInitialDt();

        if (use_single_threaded_) {
          auto solver = whole_traj_solver_ptrs_[0];
          solver->resetToNominalState();
          solver->setX0(A);
          solver->setXf(E);
          solver->setInitialDt(initial_dt);
          solver->setT0(0.0);
          solver->setPolytopes(l_constraints);

          bool gurobi_error = false;
          double gurobi_ms = 0.0;

          const auto t0 = steady_clock::now();
          const bool ok = solver->generateNewTrajectory(gurobi_error, gurobi_ms, factors_[0], true);
          const auto t1 = steady_clock::now();
          r.total_opt_runtime_ms = 1e3 * duration<double>(t1 - t0).count();

          r.per_opt_runtime_ms = gurobi_ms;
          r.factor_used = solver->getFactorThatWorked();
          r.cost_value = solver->getObjectiveValue();

          // Check for timeout
          if (r.total_opt_runtime_ms > per_case_timeout_sec_ * 1000.0) {
            RCLCPP_WARN(get_logger(), "Case timeout after %.3f seconds",
                        r.total_opt_runtime_ms / 1000.0);
            r.success = false;
            r.gurobi_error = false;
            r.status = "TIMEOUT (exceeded " + std::to_string(per_case_timeout_sec_) + "s)";
          } else if (!ok) {
            r.success = false;
            r.gurobi_error = gurobi_error;
            r.status = gurobi_error ? "GRB_ERROR" : "NO_SOLUTION";
          } else {
            solver->getTotalTrajTime(r.total_traj_time_sec);
            solver->fillGoalSetPoints();

            std::vector<RobotState> goal_setpoints;
            solver->getGoalSetpoints(goal_setpoints);

            const auto crep = analyzeConstraintsSampled(goal_setpoints, l_constraints, par_.dc,
                                                        par_.v_max, par_.a_max, par_.j_max);
            applyConstraintReport(r, crep);

            r.opt_traj_ma =
                stateVector2ColoredMarkerArray(goal_setpoints, /*type=*/1, par_.v_max, this->now());
            RCLCPP_INFO(get_logger(),
                        "Created opt_traj_ma with %zu markers from %zu goal_setpoints",
                        r.opt_traj_ma.markers.size(), goal_setpoints.size());
            r.success = true;
            r.gurobi_error = false;
            r.status = "OK (factor=" + std::to_string(r.factor_used) + ")";

            // NEW: dump trajectory
            maybeDumpTrajectory(fname, goal_setpoints, r);
          }
        } else {
          // Multi-threaded solve with retry on failure (expand window)
          const auto cumul_t0 = steady_clock::now();
          bool case_solved = false;
          bool case_timed_out = false;
          size_t last_error_count = 0;
          size_t last_non_error_fail_count = 0;
          std::string last_error_msg;
          std::string last_non_error_msg;
          double final_poll_success_ms = 0.0;  // clocked at the moment a solution is found

          while (!case_solved && !case_timed_out && !factors_.empty()) {
            // Check cumulative timeout before launching a new attempt
            {
              double elapsed_sec = duration<double>(steady_clock::now() - cumul_t0).count();
              if (elapsed_sec > per_case_timeout_sec_) {
                case_timed_out = true;
                break;
              }
            }

            // Reset solvers before each attempt
            for (size_t i = 0; i < factors_.size(); ++i)
              whole_traj_solver_ptrs_[i]->resetToNominalState();

            // Shared state for condition-variable-based notification
            std::mutex cv_mtx;
            std::condition_variable cv_done;
            // completed_results[i] is set once thread i finishes
            std::vector<std::optional<ThreadRet>> completed_results(factors_.size(), std::nullopt);
            std::atomic<size_t> num_completed{0};

            // Launch async workers with current factors_
            const auto opt_start = steady_clock::now();  // for fair total_opt timing
            std::vector<std::future<void>> futures;
            futures.reserve(factors_.size());

            for (size_t i = 0; i < factors_.size(); ++i) {
              const double factor = factors_[i];
              auto solver = whole_traj_solver_ptrs_[i];

              futures.push_back(std::async(
                  std::launch::async,
                  [i, solver, &l_constraints, A, E, initial_dt, factor, cumul_t0, &cv_mtx, &cv_done,
                   &completed_results, &num_completed]() {
                    ThreadTiming tt;
                    tt.factor = factor;
                    const auto thr_start = steady_clock::now();
                    tt.thread_start_ms = 1e3 * duration<double>(thr_start - cumul_t0).count();

                    ThreadRet ret;
                    try {
                      solver->setX0(A);
                      solver->setXf(E);
                      solver->setInitialDt(initial_dt);
                      solver->setT0(0.0);
                      solver->setPolytopes(l_constraints);

                      const auto after_setup = steady_clock::now();
                      tt.setup_ms = 1e3 * duration<double>(after_setup - thr_start).count();

                      bool gurobi_error = false;
                      double per_opt_runtime_ms = 0.0;
                      const bool ok =
                          solver->generateNewTrajectory(gurobi_error, per_opt_runtime_ms, factor);
                      const bool success = ok && (!gurobi_error);

                      const auto after_gen = steady_clock::now();
                      tt.generate_ms = 1e3 * duration<double>(after_gen - after_setup).count();
                      tt.gurobi_runtime_ms = per_opt_runtime_ms;
                      tt.thread_total_ms = 1e3 * duration<double>(after_gen - thr_start).count();
                      tt.success = success;
                      tt.msg = success ? "SUCCESS" : (gurobi_error ? "GRB_ERROR" : "NO_SOLUTION");

                      // Copy solver sub-step breakdown
                      const auto& sb = solver->last_solve_timing_;
                      tt.findDT_ms = sb.findDT_ms;
                      tt.setX_ms = sb.setX_ms;
                      tt.polytopes_ms = sb.polytopes_ms;
                      tt.dynamic_ms = sb.dynamic_ms;
                      tt.objective_ms = sb.objective_ms;
                      tt.mapsize_ms = sb.mapsize_ms;
                      tt.callOptimizer_ms = sb.callOptimizer_ms;
                      tt.postsolve_ms = sb.postsolve_ms;

                      std::string msg = tt.msg;
                      ret = {success, gurobi_error, per_opt_runtime_ms, factor, msg, tt};
                    } catch (const std::exception& e) {
                      tt.thread_total_ms =
                          1e3 * duration<double>(steady_clock::now() - thr_start).count();
                      tt.msg = std::string("EXCEPTION: ") + e.what();
                      ret = {false, true, 0.0, factor, tt.msg, tt};
                    } catch (...) {
                      tt.thread_total_ms =
                          1e3 * duration<double>(steady_clock::now() - thr_start).count();
                      tt.msg = "UNKNOWN_EXCEPTION";
                      ret = {false, true, 0.0, factor, tt.msg, tt};
                    }

                    // Signal completion via condition variable
                    {
                      std::lock_guard<std::mutex> lk(cv_mtx);
                      completed_results[i] = std::move(ret);
                      num_completed.fetch_add(1, std::memory_order_release);
                    }
                    cv_done.notify_one();
                  }));
            }

            // Collect results + timing
            last_error_count = 0;
            last_non_error_fail_count = 0;
            last_error_msg.clear();
            last_non_error_msg.clear();
            std::vector<ThreadTiming> attempt_timings(futures.size());
            double poll_first_ready_ms = 0.0;
            double poll_success_ms = 0.0;
            bool poll_first_logged = false;

            if (planner_name_ == "sando" || planner_name_ == "faster_star") {
              // Iterate futures sequentially (lowest factor first).
              // On first success, stop all other solvers, then drain remaining futures.
              // This matches the sando.cpp live-planner pattern.
              int best_idx = -1;

              for (size_t i = 0; i < futures.size(); ++i) {
                // Wait for this thread to complete
                futures[i].get();

                // Result was stored via condition variable
                if (!completed_results[i].has_value()) continue;

                auto& [succ, gurobi_error, gurobi_ms, factor, msg, tt] =
                    completed_results[i].value();
                attempt_timings[i] = tt;

                if (!poll_first_logged) {
                  poll_first_ready_ms =
                      1e3 * duration<double>(steady_clock::now() - cumul_t0).count();
                  poll_first_logged = true;
                }

                if (!succ) {
                  if (gurobi_error) {
                    ++last_error_count;
                    last_error_msg = msg;
                  } else {
                    ++last_non_error_fail_count;
                    last_non_error_msg = msg;
                  }
                  continue;
                }

                // First success — stop all other solvers
                best_idx = (int)i;
                r.per_opt_runtime_ms = gurobi_ms;
                r.factor_used = factor;
                poll_success_ms = 1e3 * duration<double>(steady_clock::now() - opt_start).count();

                for (size_t j = 0; j < factors_.size(); ++j) {
                  if (j == i) continue;
                  try {
                    whole_traj_solver_ptrs_[j]->stopExecution();
                  } catch (...) {
                  }
                }
                break;
              }

              // Drain remaining futures so solvers can be reused
              for (size_t i = 0; i < futures.size(); ++i) {
                if (attempt_timings[i].thread_total_ms > 0.0) continue;  // already consumed
                try {
                  futures[i].get();
                  if (completed_results[i].has_value())
                    attempt_timings[i] = std::get<5>(completed_results[i].value());
                } catch (...) {
                }
              }

              if (best_idx >= 0) {
                auto& solver = whole_traj_solver_ptrs_[best_idx];
                solver->getTotalTrajTime(r.total_traj_time_sec);
                solver->fillGoalSetPoints();

                std::vector<RobotState> goal_setpoints;
                solver->getGoalSetpoints(goal_setpoints);

                const auto crep = analyzeConstraintsSampled(goal_setpoints, l_constraints, par_.dc,
                                                            par_.v_max, par_.a_max, par_.j_max);
                applyConstraintReport(r, crep);

                r.opt_traj_ma = stateVector2ColoredMarkerArray(goal_setpoints,
                                                               /*type=*/1, par_.v_max, this->now());
                RCLCPP_INFO(get_logger(),
                            "Created opt_traj_ma with %zu markers from %zu goal_setpoints",
                            r.opt_traj_ma.markers.size(), goal_setpoints.size());

                r.success = true;
                r.gurobi_error = false;
                r.cost_value = solver->getObjectiveValue();
                r.status = "OK (factor=" + std::to_string(r.factor_used) + ")";
                case_solved = true;
                final_poll_success_ms = poll_success_ms;

                maybeDumpTrajectory(fname, goal_setpoints, r);
              }

              // Check cumulative timeout
              {
                double elapsed_sec = duration<double>(steady_clock::now() - cumul_t0).count();
                if (elapsed_sec > per_case_timeout_sec_) case_timed_out = true;
              }
            }

            // Write timing log for this attempt
            {
              double cumul_ms = 1e3 * duration<double>(steady_clock::now() - cumul_t0).count();
              writeTimingLog(timing_log_path_, case_idx, planner_name_, cumul_ms,
                             poll_first_ready_ms, poll_success_ms, attempt_timings);
            }

            // If this attempt failed and dynamic factor is enabled, shift window and retry
            if (!case_solved && !case_timed_out && use_dynamic_factor_) {
              // Shift window up by one step
              double current_max = factors_.back();
              if (current_max + par_.factor_constant_step_size > par_.factor_final + 1e-9) {
                // Can't shift further — give up
                RCLCPP_WARN(get_logger(),
                            "[Case %zu] All factor windows exhausted up to factor_final=%.2f",
                            case_idx, par_.factor_final);
                break;
              }

              for (auto& f : factors_) f += par_.factor_constant_step_size;
              // Remove factors that exceed factor_final
              factors_.erase(
                  std::remove_if(factors_.begin(), factors_.end(),
                                 [this](double f) { return f > par_.factor_final + 1e-9; }),
                  factors_.end());

              if (factors_.empty()) break;

              RCLCPP_INFO(get_logger(),
                          "[Case %zu] Retry: shifted window to [%.2f .. %.2f] (%zu factors)",
                          case_idx, factors_.front(), factors_.back(), factors_.size());
            } else if (!case_solved) {
              // No retry for fixed-range mode or timeout
              break;
            }
          }  // end retry while loop

          // Compute total elapsed time:
          // For successful cases, use final_poll_success_ms (clocked at the
          // moment a solution was found) so post-processing / future-draining
          // overhead is excluded.  For failed cases, use wall-clock to capture
          // the full time spent attempting all factors.
          r.total_opt_runtime_ms =
              (case_solved && final_poll_success_ms > 0.0)
                  ? final_poll_success_ms
                  : 1e3 * duration<double>(steady_clock::now() - cumul_t0).count();

          if (!case_solved) {
            r.success = false;
            const bool any_non_error_fail = (last_non_error_fail_count > 0);
            const bool only_errors = (last_error_count > 0 && last_non_error_fail_count == 0);

            if (case_timed_out || r.total_opt_runtime_ms > per_case_timeout_sec_ * 1000.0) {
              RCLCPP_WARN(get_logger(), "Case timeout after %.3f seconds",
                          r.total_opt_runtime_ms / 1000.0);
              r.gurobi_error = only_errors;
              r.status = "TIMEOUT (exceeded " + std::to_string(per_case_timeout_sec_) + "s)";
            } else if (any_non_error_fail) {
              r.gurobi_error = false;
              r.status = "NO_SOLUTION: " +
                         (last_non_error_msg.empty() ? "NO_SOLUTION" : last_non_error_msg);
            } else {
              r.gurobi_error = only_errors;
              r.status =
                  only_errors
                      ? ("GRB_ERROR: " + (last_error_msg.empty() ? "GRB_ERROR" : last_error_msg))
                      : "NO_SOLUTION";
            }
          }
        }
      } catch (const std::exception& e) {
        r.success = false;
        r.status = std::string("EXCEPTION: ") + e.what();
      }

      // Record for factor report (always, not just dynamic factor mode)
      {
        DynFactorCaseRecord rec;
        rec.case_idx = case_idx;
        rec.success = r.success;
        rec.factor_used = r.factor_used;
        rec.window_lo = factors_.empty() ? 0.0 : factors_.front();
        rec.window_hi = factors_.empty() ? 0.0 : factors_.back();
        dyn_factor_case_records_.push_back(rec);
      }

      // Publish visualization immediately if enabled
      if (visualize_) {
        publishCase(case_idx);
        rclcpp::spin_some(this->get_node_base_interface());  // process callbacks

        // Add delay if specified (allows visualization to be seen)
        if (solve_delay_sec_ > 0.0) {
          std::this_thread::sleep_for(std::chrono::duration<double>(solve_delay_sec_));
        }
      }
    }
  }

  void writeCsv() const {
    std::ofstream ofs(csv_out_);
    if (!ofs) {
      RCLCPP_WARN(get_logger(), "Failed to open csv_out: %s", csv_out_.c_str());
      return;
    }

    ofs << "planner_name,file,success,status,gurobi_error,per_opt_runtime_ms,total_opt_runtime_ms,"
           "factor_used,cost_value,total_traj_time_sec,"
           "corridor_max_min_violation,corridor_t_at_max,corridor_px,corridor_py,corridor_pz,"
           "corridor_best_poly_idx,corridor_violated,"
           "v_max_observed,v_max_excess,v_t_at_max,v_violated,"
           "a_max_observed,a_max_excess,a_t_at_max,a_violated,"
           "j_max_observed,j_max_excess,j_t_at_max,j_violated,"
           "corridor_violation_count,v_violation_count,a_violation_count,j_violation_count,"
           "violation_total_samples,"
           "jerk_smoothness_l1,jerk_rms,"
           "traj_length_m,"
           "start_x,start_y,start_z,goal_x,goal_y,goal_z\n";

    for (const auto& r : results_) {
      ofs << planner_name_ << "," << fs::path(r.file).filename().string() << ","
          << (r.success ? 1 : 0) << ","
          << "\"" << r.status << "\""
          << "," << (r.gurobi_error ? 1 : 0) << "," << std::fixed << std::setprecision(3)
          << r.per_opt_runtime_ms << "," << r.total_opt_runtime_ms << "," << r.factor_used << ","
          << r.cost_value << "," << r.total_traj_time_sec << "," << std::setprecision(9)
          << r.corridor_max_min_violation << "," << r.corridor_t_at_max << ","
          << r.corridor_p_at_max.x() << "," << r.corridor_p_at_max.y() << ","
          << r.corridor_p_at_max.z() << "," << r.corridor_best_poly_idx_at_max << ","
          << (r.corridor_violated ? 1 : 0) << "," << r.v_max_observed << "," << r.v_max_excess
          << "," << r.v_t_at_max << "," << (r.v_violated ? 1 : 0) << "," << r.a_max_observed << ","
          << r.a_max_excess << "," << r.a_t_at_max << "," << (r.a_violated ? 1 : 0) << ","
          << r.j_max_observed << "," << r.j_max_excess << "," << r.j_t_at_max << ","
          << (r.j_violated ? 1 : 0) << "," << r.corridor_violation_count << ","
          << r.v_violation_count << "," << r.a_violation_count << "," << r.j_violation_count << ","
          << r.violation_total_samples << "," << r.jerk_smoothness_l1 << "," << r.jerk_rms << ","
          << r.traj_length_m << "," << std::setprecision(3) << r.start.x() << "," << r.start.y()
          << "," << r.start.z() << "," << r.goal.x() << "," << r.goal.y() << "," << r.goal.z()
          << "\n";
    }

    ofs.flush();
    RCLCPP_INFO(get_logger(), "Wrote CSV: %s", csv_out_.c_str());
  }

  void writeFactorReport() const {
    // Write to benchmark_data/<planner>_factor_report.txt
    std::string report_path = benchmark_data_dir_ + "/" +
                              dyn_factor_reports_.front().planner_name + "_factor_report.txt";

    std::ofstream ofs(report_path);
    if (!ofs) {
      RCLCPP_WARN(get_logger(), "Failed to open factor report: %s", report_path.c_str());
      return;
    }

    for (const auto& cfg : dyn_factor_reports_) {
      ofs << "=== " << cfg.planner_name << " N=" << cfg.num_N << " (initial: [" << std::fixed
          << std::setprecision(2) << cfg.initial_window_lo << " .. " << cfg.initial_window_hi
          << "], output " << cfg.csv_out << ") ===\n";

      if (use_dynamic_factor_) {
        ofs << "  Case    Result    Factor      Window After\n";
      } else {
        ofs << "  Case    Result    Factor\n";
      }
      ofs << "--------------------------------------------------\n";

      int success_count = 0;
      int fail_count = 0;
      double factor_min = std::numeric_limits<double>::max();
      double factor_max = 0.0;
      double factor_sum = 0.0;

      for (const auto& rec : cfg.records) {
        ofs << std::setw(6) << rec.case_idx << "   ";
        if (rec.success) {
          ofs << "SUCCESS   " << std::fixed << std::setprecision(2) << std::setw(8)
              << rec.factor_used;
          if (use_dynamic_factor_) {
            ofs << "  [" << rec.window_lo << " .. " << rec.window_hi << "]";
          }
          ofs << "\n";
          success_count++;
          factor_min = std::min(factor_min, rec.factor_used);
          factor_max = std::max(factor_max, rec.factor_used);
          factor_sum += rec.factor_used;
        } else {
          ofs << "   FAIL      --";
          if (use_dynamic_factor_) {
            ofs << "    [" << std::fixed << std::setprecision(2) << rec.window_lo << " .. "
                << rec.window_hi << "]";
          }
          ofs << "\n";
          fail_count++;
        }
      }

      int total = success_count + fail_count;
      ofs << "\nTotal: " << total << " cases, " << success_count << " success, " << fail_count
          << " fail\n";
      if (success_count > 0) {
        ofs << "Factor range used: [" << std::fixed << std::setprecision(2) << factor_min << ", "
            << factor_max << "], mean=" << (factor_sum / success_count) << "\n";
      }
      ofs << "\n\n";
    }

    ofs.flush();
    RCLCPP_INFO(get_logger(), "Wrote factor report: %s", report_path.c_str());
  }

  void publishCase(size_t idx) {
    if (idx >= results_.size()) {
      RCLCPP_WARN(get_logger(), "publishCase: idx %zu out of range (size=%zu)", idx,
                  results_.size());
      return;
    }

    if (!pub_poly_ || !pub_traj_committed_colored_ || !pub_hgp_path_marker_) {
      RCLCPP_WARN(get_logger(), "publishCase: Publishers not initialized!");
      return;
    }

    auto& r = results_[idx];
    auto stamp = now();

    // Clear the trajectory topic
    {
      visualization_msgs::msg::MarkerArray clear_msg;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = stamp;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      clear_msg.markers.push_back(m);
      pub_traj_committed_colored_->publish(clear_msg);
      pub_hgp_path_marker_->publish(clear_msg);
    }

    // Publish global path markers
    for (auto& mk : r.global_path_ma.markers) {
      mk.header.frame_id = frame_id_;
      mk.header.stamp = stamp;
    }
    pub_hgp_path_marker_->publish(r.global_path_ma);
    RCLCPP_INFO(get_logger(), "Published global path with %zu markers",
                r.global_path_ma.markers.size());

    // Publish committed traj markers (if success)
    if (r.success) {
      if (r.opt_traj_ma.markers.empty()) {
        RCLCPP_WARN(get_logger(), "Case %zu: SUCCESS but opt_traj_ma is EMPTY!", idx);
      } else {
        for (auto& mk : r.opt_traj_ma.markers) {
          mk.header.frame_id = frame_id_;
          mk.header.stamp = stamp;
        }
        pub_traj_committed_colored_->publish(r.opt_traj_ma);
        RCLCPP_INFO(get_logger(), "Published trajectory with %zu markers",
                    r.opt_traj_ma.markers.size());
      }
    } else {
      RCLCPP_WARN(get_logger(), "Case %zu: FAILED (no trajectory to publish) - status: %s", idx,
                  r.status.c_str());
    }

    // publish corridor polyhedra
    r.poly_msg.header.stamp = stamp;
    r.poly_msg.header.frame_id = frame_id_;
    pub_poly_->publish(r.poly_msg);
    RCLCPP_INFO(get_logger(), "Published %zu polyhedra", r.poly_msg.polyhedrons.size());

    RCLCPP_INFO(get_logger(), "[%zu/%zu] %s: %s", idx + 1, results_.size(),
                fs::path(r.file).filename().string().c_str(), r.status.c_str());
  }

  void publishNext() {
    publishCase(play_idx_);
    play_idx_ = (play_idx_ + 1) % results_.size();
  }

 private:
  std::string planner_name_{"sando"};

  // I/O
  std::string sfc_dir_;
  std::string benchmark_data_dir_;
  std::string file_ext_;
  std::string frame_id_;
  std::string poly_topic_;
  std::string traj_committed_topic_;
  std::string hgp_path_topic_;
  std::string csv_out_;
  std::string timing_log_path_;

  bool use_single_threaded_{false};

  bool visualize_{true};
  double playback_period_sec_{1.0};
  double solve_delay_sec_{0.0};       // delay between solving each case
  double per_case_timeout_sec_{1.0};  // timeout for each optimization case
  bool latched_{true};

  double assumed_last_replan_time_sec_{0.05};

  Parameters par_;

  std::vector<BenchResult> results_;
  size_t play_idx_{0};

  std::vector<double> factors_;
  std::vector<std::shared_ptr<SolverGurobi>> whole_traj_solver_ptrs_;

  bool use_dynamic_factor_{false};
  double dynamic_factor_k_radius_{0.4};
  double dynamic_factor_initial_mean_{0.0};
  bool dynamic_factor_initial_success_{false};

  // Dynamic factor report accumulation (across N configs for same planner)
  std::vector<DynFactorConfigReport> dyn_factor_reports_;
  std::vector<DynFactorCaseRecord> dyn_factor_case_records_;

  double poly_seed_eps_{1e-6};
  bool debug_poly_check_{true};

  std::vector<EllipsoidDecomp3D> ellip_workers_;

  // NEW: trajectory dump control
  bool traj_dump_enable_{true};
  std::string traj_dump_root_dir_;
  double traj_dump_dt_{-1.0};

  bool traj_dump_enable_this_run_{false};
  std::string traj_dump_run_dir_;

  // ROS
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_committed_colored_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_hgp_path_marker_;
  rclcpp::TimerBase::SharedPtr playback_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalTrajBenchmarkNode>());
  rclcpp::shutdown();
  return 0;
}
