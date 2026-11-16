/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <sando/daqp_solver.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

extern "C" {
#include <api.h>
}

namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Constraints that do not depend on z are checked with the same tolerance as DAQP's primal_tol
constexpr double kFeasibilityTol = DAQP_DEFAULT_PRIM_TOL;
constexpr double kZeroRowTol = 1e-12;

// Suboptimality that the branch-and-bound allows, relative to a lower bound on the objective. Without
// it, many nodes tie with the incumbent and cannot be pruned, since the objective does not depend on
// the binaries. The scale is a floor on that lower bound, in the units of the objective.
constexpr double kRelativeSuboptimality = 1e-6;
constexpr double kMinObjectiveScale = 1e-3;
constexpr int kIterationLimit = 200000;

// Functionals of the normalized coefficients (an, bn, cn, dn) of a segment.
// As in SolverGurobi, the constraints are on the Bezier control points of the segment: a polynomial
// is contained in the convex hull of its control points.
const Eigen::RowVector4d kPosCP[4] = {
    {0.0, 0.0, 0.0, 1.0},
    {0.0, 0.0, 1.0 / 3.0, 1.0},
    {0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0},
    {1.0, 1.0, 1.0, 1.0}};
const Eigen::RowVector4d kVelCP[3] = {{0.0, 0.0, 1.0, 0.0}, {0.0, 1.0, 1.0, 0.0}, {3.0, 2.0, 1.0, 0.0}};
const Eigen::RowVector4d kAccelCP[2] = {{0.0, 2.0, 0.0, 0.0}, {6.0, 2.0, 0.0, 0.0}};
const Eigen::RowVector4d kJerkCP = {6.0, 0.0, 0.0, 0.0};

}  // namespace

void SolverDAQP::initializeSolver(const Parameters& par) {
  setCommonParameters_(par);

  if (N_ < 4)
    throw std::invalid_argument(
        "SolverDAQP requires num_N >= 4, got " + std::to_string(N_) + ".");
  if (!usingFaster_() && dynamic_constraint_type_ != "Linf" && dynamic_constraint_type_ != "L1")
    throw std::invalid_argument(
        "SolverDAQP only supports the linear dynamic constraints \"Linf\" and \"L1\", got \"" +
        dynamic_constraint_type_ + "\". Use the Gurobi backend for \"L2\".");

  // FASTER has no bounds on its free coefficients
  if (usingFaster_()) using_variable_elimination_ = false;

  num_free_ = N_ - 3;
  n_ = 3 * num_free_;
}

void SolverDAQP::functionalToRow_(
    int segment,
    int axis,
    const Eigen::RowVector4d& functional,
    Eigen::RowVectorXd& row,
    double& offset) const {
  row = Eigen::RowVectorXd::Zero(n_);
  row.segment(axis * num_free_, num_free_).noalias() =
      functional * G_.block(4 * segment, 0, 4, num_free_);
  offset = functional.dot(g_.col(axis).segment<4>(4 * segment));
}

void SolverDAQP::addRow_(
    std::vector<Eigen::RowVectorXd>& rows,
    std::vector<double>& lower,
    std::vector<double>& upper,
    bool& infeasible,
    const Eigen::RowVectorXd& row,
    double row_lower,
    double row_upper) const {
  // Control points of the first/last segment can be fully determined by the boundary conditions
  if (row.cwiseAbs().maxCoeff() < kZeroRowTol) {
    if (row_lower > kFeasibilityTol || row_upper < -kFeasibilityTol) infeasible = true;
    return;
  }
  rows.push_back(row);
  lower.push_back(row_lower);
  upper.push_back(row_upper);
}

static void toRowSet(
    const std::vector<Eigen::RowVectorXd>& rows,
    const std::vector<double>& lower,
    const std::vector<double>& upper,
    int n,
    bool infeasible,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& A,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u,
    bool& set_infeasible) {
  const int m = static_cast<int>(rows.size());
  A.resize(m, n);
  l.resize(m);
  u.resize(m);
  for (int i = 0; i < m; ++i) {
    A.row(i) = rows[i];
    l(i) = lower[i];
    u(i) = upper[i];
  }
  set_infeasible = infeasible;
}

void SolverDAQP::buildReducedProblem_() {
  // Unknowns: normalized coefficients an = a*T^3, bn = b*T^2, cn = c*T, dn = d per segment, which
  // makes the equations below independent of the magnitude of the segment times.
  const int num_coeffs = 4 * N_;
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(num_coeffs, num_coeffs);
  Eigen::MatrixXd rhs_z = Eigen::MatrixXd::Zero(num_coeffs, num_free_);
  g_.resize(num_coeffs, 3);
  Eigen::MatrixXd rhs_bc = Eigen::MatrixXd::Zero(num_coeffs, 3);

  const double T_first = dt_.front();
  const double T_last = dt_.back();
  int row = 0;

  // Initial conditions
  M(row, 3) = 1.0;
  for (int axis = 0; axis < 3; ++axis) rhs_bc(row, axis) = x0_[axis];
  ++row;
  M(row, 2) = 1.0;
  for (int axis = 0; axis < 3; ++axis) rhs_bc(row, axis) = x0_[axis + 3] * T_first;
  ++row;
  M(row, 1) = 2.0;
  for (int axis = 0; axis < 3; ++axis) rhs_bc(row, axis) = x0_[axis + 6] * T_first * T_first;
  ++row;

  // Final conditions
  const int last = 4 * (N_ - 1);
  M.block<1, 4>(row, last) << 1.0, 1.0, 1.0, 1.0;
  for (int axis = 0; axis < 3; ++axis) rhs_bc(row, axis) = xf_[axis];
  ++row;
  M.block<1, 4>(row, last) << 3.0, 2.0, 1.0, 0.0;
  for (int axis = 0; axis < 3; ++axis) rhs_bc(row, axis) = xf_[axis + 3] * T_last;
  ++row;
  M.block<1, 4>(row, last) << 6.0, 2.0, 0.0, 0.0;
  for (int axis = 0; axis < 3; ++axis) rhs_bc(row, axis) = xf_[axis + 6] * T_last * T_last;
  ++row;

  // Continuity of position, velocity and acceleration
  for (int s = 0; s + 1 < N_; ++s) {
    const double ratio = dt_[s] / dt_[s + 1];
    const int cur = 4 * s;
    const int next = 4 * (s + 1);
    M.block<1, 4>(row, cur) << 1.0, 1.0, 1.0, 1.0;
    M(row, next + 3) = -1.0;
    ++row;
    M.block<1, 4>(row, cur) << 3.0, 2.0, 1.0, 0.0;
    M(row, next + 2) = -ratio;
    ++row;
    M.block<1, 4>(row, cur) << 6.0, 2.0, 0.0, 0.0;
    M(row, next + 1) = -2.0 * ratio * ratio;
    ++row;
  }

  // Free parameters: the position at the start of the segments 3, ..., N-1
  for (int k = 0; k < num_free_; ++k) {
    M(row, 4 * (3 + k) + 3) = 1.0;
    rhs_z(row, k) = 1.0;
    ++row;
  }

  const Eigen::PartialPivLU<Eigen::MatrixXd> lu(M);
  G_ = lu.solve(rhs_z);
  g_ = lu.solve(rhs_bc);

  buildObjective_();
  buildDynamicAndMapRows_();
  buildCorridorRows_();

  // Bounds on the free parameters (same as the variable bounds in SolverGurobi::createVars)
  z_lower_ = Eigen::VectorXd::Constant(n_, -DAQP_INF);
  z_upper_ = Eigen::VectorXd::Constant(n_, DAQP_INF);
  if (using_variable_elimination_) {
    const double margin_xy = 5.0;
    const double margin_z = 2.0;
    const double lb[3] = {x_min_ - margin_xy, y_min_ - margin_xy, z_min_ - margin_z};
    const double ub[3] = {x_max_ + margin_xy, y_max_ + margin_xy, z_max_ + margin_z};
    for (int axis = 0; axis < 3; ++axis) {
      z_lower_.segment(axis * num_free_, num_free_).setConstant(lb[axis]);
      z_upper_.segment(axis * num_free_, num_free_).setConstant(ub[axis]);
    }
  }
}

void SolverDAQP::buildObjective_() {
  // The jerk of a segment is 6*an/T^3
  const double T_ref = dt_.front();
  const double T_ref3 = T_ref * T_ref * T_ref;
  objective_scale_ = jerk_smooth_weight_ / (T_ref3 * T_ref3);

  H_ = RowMatrix::Zero(n_, n_);
  f_ = Eigen::VectorXd::Zero(n_);
  objective_offset_ = 0.0;

  Eigen::RowVectorXd row;
  double offset = 0.0;
  for (int s = 0; s < N_; ++s) {
    const double ratio3 = (T_ref / dt_[s]) * (T_ref / dt_[s]) * (T_ref / dt_[s]);
    const double weight = ratio3 * ratio3;
    for (int axis = 0; axis < 3; ++axis) {
      functionalToRow_(s, axis, kJerkCP, row, offset);
      H_.noalias() += 2.0 * weight * row.transpose() * row;
      f_.noalias() += 2.0 * weight * offset * row.transpose();
      objective_offset_ += weight * offset * offset;
    }
  }
}

void SolverDAQP::buildDynamicAndMapRows_() {
  std::vector<Eigen::RowVectorXd> rows;
  std::vector<double> lower, upper;
  bool infeasible = false;

  Eigen::RowVectorXd row;
  double offset = 0.0;

  // All rows are scaled to meters: |v| <= v_max is written as |v*T| <= v_max*T, and so on
  const auto add_axis = [&](int s, int axis, const Eigen::RowVector4d& functional, double limit) {
    functionalToRow_(s, axis, functional, row, offset);
    addRow_(rows, lower, upper, infeasible, row, -limit - offset, limit - offset);
  };
  // |x| + |y| + |z| <= limit as four double-sided constraints (the faces of an octahedron)
  const auto add_l1 = [&](int s, const Eigen::RowVector4d& functional, double limit) {
    static const double signs[4][2] = {{1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}};
    for (const auto& sign : signs) {
      Eigen::RowVectorXd combined = Eigen::RowVectorXd::Zero(n_);
      double combined_offset = 0.0;
      const double coeff[3] = {1.0, sign[0], sign[1]};
      for (int axis = 0; axis < 3; ++axis) {
        functionalToRow_(s, axis, functional, row, offset);
        combined += coeff[axis] * row;
        combined_offset += coeff[axis] * offset;
      }
      addRow_(
          rows, lower, upper, infeasible, combined, -limit - combined_offset,
          limit - combined_offset);
    }
  };

  const bool original_faster = usingFaster_() && planner_name_ != "safe_faster";
  const bool use_l1 = !usingFaster_() && dynamic_constraint_type_ == "L1";
  const double map_lower[3] = {x_min_, y_min_, z_min_};
  const double map_upper[3] = {x_max_, y_max_, z_max_};

  for (int s = 0; s < N_; ++s) {
    const double T = dt_[s];
    const double v_lim = v_max_ * T;
    const double a_lim = a_max_ * T * T;
    const double j_lim = j_max_ * T * T * T;

    if (use_l1) {
      for (const auto& cp : kVelCP) add_l1(s, cp, v_lim);
      for (const auto& cp : kAccelCP) add_l1(s, cp, a_lim);
      add_l1(s, kJerkCP, j_lim);
    } else {
      for (int axis = 0; axis < 3; ++axis) {
        if (original_faster) {
          // Original FASTER: only constrains the first control point per segment
          add_axis(s, axis, kVelCP[0], v_lim);
          add_axis(s, axis, kAccelCP[0], a_lim);
        } else {
          for (const auto& cp : kVelCP) add_axis(s, axis, cp, v_lim);
          for (const auto& cp : kAccelCP) add_axis(s, axis, cp, a_lim);
        }
        add_axis(s, axis, kJerkCP, j_lim);
      }
    }

    for (int axis = 0; axis < 3; ++axis) {
      for (const auto& cp : kPosCP) {
        functionalToRow_(s, axis, cp, row, offset);
        addRow_(
            rows, lower, upper, infeasible, row, map_lower[axis] - offset, map_upper[axis] - offset);
      }
    }
  }

  toRowSet(
      rows, lower, upper, n_, infeasible, common_rows_.A, common_rows_.lower, common_rows_.upper,
      common_rows_.infeasible);
}

void SolverDAQP::buildCorridorRows_() {
  corridor_.assign(N_, {});
  if (!hasPolytopes_()) return;

  const int P = numSpatialPolys_();
  std::vector<Eigen::RowVectorXd> rows;
  std::vector<double> lower, upper;
  Eigen::RowVectorXd row(n_);
  double offset = 0.0;

  for (int s = 0; s < N_; ++s) {
    corridor_[s].reserve(P);
    // Precompute the map from z to the control points of the segment
    Eigen::RowVectorXd cp_rows[4][3];
    double cp_offsets[4][3];
    for (int k = 0; k < 4; ++k)
      for (int axis = 0; axis < 3; ++axis)
        functionalToRow_(s, axis, kPosCP[k], cp_rows[k][axis], cp_offsets[k][axis]);

    for (int p = 0; p < P; ++p) {
      const auto& poly = polyAt_(s, p);
      const Eigen::MatrixXd A = poly.A();
      const Eigen::VectorXd b = poly.b();
      if (b.rows() == 0) continue;  // Empty polytope (invalid decomposition) cannot be selected

      rows.clear();
      lower.clear();
      upper.clear();
      bool infeasible = false;
      for (int k = 0; k < 4; ++k) {
        for (int i = 0; i < b.rows(); ++i) {
          row.setZero();
          offset = 0.0;
          for (int axis = 0; axis < 3; ++axis) {
            row += A(i, axis) * cp_rows[k][axis];
            offset += A(i, axis) * cp_offsets[k][axis];
          }
          addRow_(rows, lower, upper, infeasible, row, -DAQP_INF, b(i) - offset);
        }
      }
      // A polytope that excludes a fixed control point can never be selected for this segment
      if (infeasible) continue;

      RowSet set;
      toRowSet(rows, lower, upper, n_, infeasible, set.A, set.lower, set.upper, set.infeasible);
      corridor_[s].push_back(std::move(set));
    }
  }
}

double SolverDAQP::suboptimalityTolerance_() const {
  // Lower bound on the objective: its unconstrained minimum (the objective is a sum of squares)
  const Eigen::VectorXd z = H_.ldlt().solve(-f_);
  const double unconstrained = z.allFinite() ? 0.5 * f_.dot(z) + objective_offset_ : 0.0;
  return kRelativeSuboptimality * std::max(unconstrained, kMinObjectiveScale);
}

SolverDAQP::SolveStatus SolverDAQP::solveMIQP_(Eigen::VectorXd& z, double& cost) {
  int num_binaries = 0;
  for (int s = 0; hasPolytopes_() && s < N_; ++s) {
    if (corridor_[s].empty()) return SolveStatus::Infeasible;
    num_binaries += static_cast<int>(corridor_[s].size());
  }

  // Box that contains z for all trajectories that satisfy the dynamic limits and the map bounds. A
  // tight box gives small big-M coefficients. All control points are in the map, and if all velocity
  // control points are limited, the position is within reach from the initial position.
  // (Original FASTER only limits the first velocity control point of a segment.)
  Eigen::VectorXd box_lower(n_), box_upper(n_);
  const bool velocity_limited = !usingFaster_() || planner_name_ == "safe_faster";
  const double map_lower[3] = {x_min_, y_min_, z_min_};
  const double map_upper[3] = {x_max_, y_max_, z_max_};
  for (int axis = 0; axis < 3; ++axis) {
    double time = std::accumulate(dt_.begin(), dt_.begin() + 3, 0.0);
    for (int k = 0; k < num_free_; ++k) {
      const int i = axis * num_free_ + k;
      box_lower(i) = map_lower[axis];
      box_upper(i) = map_upper[axis];
      if (velocity_limited) {
        box_lower(i) = std::max(box_lower(i), x0_[axis] - v_max_ * time);
        box_upper(i) = std::min(box_upper(i), x0_[axis] + v_max_ * time);
      }
      time += dt_[3 + k];
    }
  }

  // Big-M form of "b = 1 => row*z <= upper". A row with M = 0 holds on the whole box, so it is left out.
  struct BigMRow {
    int binary;
    const RowSet* set;
    int index;
    double M;
  };
  std::vector<BigMRow> big_m_rows;
  int binary = 0;
  for (int s = 0; hasPolytopes_() && s < N_; ++s) {
    for (const RowSet& set : corridor_[s]) {
      for (int i = 0; i < set.A.rows(); ++i) {
        double largest = 0.0;
        for (int j = 0; j < n_; ++j)
          largest += std::max(set.A(i, j) * box_lower(j), set.A(i, j) * box_upper(j));
        if (largest > set.upper(i)) big_m_rows.push_back({binary, &set, i, largest - set.upper(i)});
      }
      ++binary;
    }
  }

  // Variables: x = {z, b}
  const int num_vars = n_ + num_binaries;
  const int num_common = static_cast<int>(common_rows_.A.rows());
  const int num_big_m = static_cast<int>(big_m_rows.size());
  const int num_rows = num_common + num_big_m + (num_binaries > 0 ? N_ : 0);
  const int m = num_vars + num_rows;

  H_miqp_ = RowMatrix::Zero(num_vars, num_vars);
  H_miqp_.topLeftCorner(n_, n_) = H_;
  f_miqp_ = Eigen::VectorXd::Zero(num_vars);
  f_miqp_.head(n_) = f_;
  A_miqp_ = RowMatrix::Zero(num_rows, num_vars);
  lower_miqp_.resize(m);
  upper_miqp_.resize(m);
  sense_miqp_.assign(m, 0);

  lower_miqp_.head(n_) = z_lower_;
  upper_miqp_.head(n_) = z_upper_;
  lower_miqp_.segment(n_, num_binaries).setZero();
  upper_miqp_.segment(n_, num_binaries).setOnes();
  for (int i = n_; i < num_vars; ++i) sense_miqp_[i] = DAQP_BINARY;

  int row = 0;
  if (num_common > 0) {
    A_miqp_.topLeftCorner(num_common, n_) = common_rows_.A;
    lower_miqp_.segment(num_vars, num_common) = common_rows_.lower;
    upper_miqp_.segment(num_vars, num_common) = common_rows_.upper;
    row = num_common;
  }
  for (const BigMRow& big_m : big_m_rows) {
    A_miqp_.row(row).head(n_) = big_m.set->A.row(big_m.index);
    A_miqp_(row, n_ + big_m.binary) = big_m.M;
    lower_miqp_(num_vars + row) = -DAQP_INF;
    upper_miqp_(num_vars + row) = big_m.set->upper(big_m.index) + big_m.M;
    ++row;
  }
  // Each segment is in at least one polytope
  binary = n_;
  for (int s = 0; num_binaries > 0 && s < N_; ++s) {
    for (size_t c = 0; c < corridor_[s].size(); ++c) A_miqp_(row, binary++) = 1.0;
    lower_miqp_(num_vars + row) = 1.0;
    upper_miqp_(num_vars + row) = DAQP_INF;
    ++row;
  }

  x_miqp_.resize(num_vars);
  lam_miqp_.resize(m);

  DAQPProblem qp{};
  qp.n = num_vars;
  qp.m = m;
  qp.ms = num_vars;
  qp.H = H_miqp_.data();
  qp.f = f_miqp_.data();
  qp.A = A_miqp_.data();
  qp.bupper = upper_miqp_.data();
  qp.blower = lower_miqp_.data();
  qp.sense = sense_miqp_.data();
  qp.nh = 0;

  DAQPSettings settings;
  daqp_default_settings(&settings);
  settings.time_limit = max_comp_time_sec_;
  settings.iter_limit = kIterationLimit;
  // Not rel_subopt: it is relative to the objective without objective_offset_, which can be orders
  // of magnitude larger than the objective itself
  settings.abs_subopt = suboptimalityTolerance_();

  DAQPResult result{};
  result.x = x_miqp_.data();
  result.lam = lam_miqp_.data();
  daqp_quadprog(&result, &qp, &settings);
  node_count_ = result.nodes;

  switch (result.exitflag) {
    case DAQP_EXIT_OPTIMAL:
    case DAQP_EXIT_CONSTRAINED_POINT:
    // The objective is singular in the binaries, which DAQP regularizes. It reports this flag if it
    // cannot rule out that the regularization affected which binaries were chosen.
    case DAQP_EXIT_OPTIMAL_INEXACT:
      z = x_miqp_.head(n_);
      cost = 0.5 * z.dot(H_ * z) + f_.dot(z) + objective_offset_;
      return SolveStatus::Optimal;
    case DAQP_EXIT_INFEASIBLE:
      return SolveStatus::Infeasible;
    // As for Gurobi, a solution that has not been proven to be optimal is not accepted
    case DAQP_EXIT_TIMELIMIT:
    case DAQP_EXIT_FEASIBLE:
      return SolveStatus::Stopped;
    default:
      return SolveStatus::Failed;
  }
}

void SolverDAQP::extractCoefficients_(const Eigen::VectorXd& z) {
  x_double_.assign(3, std::vector<double>(4 * N_));
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::VectorXd normalized =
        G_ * z.segment(axis * num_free_, num_free_) + g_.col(axis);
    for (int s = 0; s < N_; ++s) {
      const double T = dt_[s];
      x_double_[axis][4 * s + 0] = normalized(4 * s + 0) / (T * T * T);
      x_double_[axis][4 * s + 1] = normalized(4 * s + 1) / (T * T);
      x_double_[axis][4 * s + 2] = normalized(4 * s + 2) / T;
      x_double_[axis][4 * s + 3] = normalized(4 * s + 3);
    }
  }
}

SolverDAQP::SolveStatus SolverDAQP::solveCurrentAllocation_(double& computation_time_ms) {
  const auto t_start = Clock::now();
  buildReducedProblem_();
  const auto t_built = Clock::now();
  last_solve_timing_.setX_ms = std::chrono::duration<double, std::milli>(t_built - t_start).count();

  node_count_ = 0;

  // DAQP cannot be interrupted, so a stop only has an effect before and after the solve
  Eigen::VectorXd z;
  double cost = 0.0;
  SolveStatus status = SolveStatus::Infeasible;
  if (should_terminate_ || max_comp_time_sec_ <= 0.0)
    status = SolveStatus::Stopped;
  else if (!common_rows_.infeasible)
    status = solveMIQP_(z, cost);
  if (should_terminate_) status = SolveStatus::Stopped;

  computation_time_ms = elapsedMs(t_built);
  last_solve_timing_.callOptimizer_ms = computation_time_ms;
  if (status != SolveStatus::Optimal) return status;

  const auto t_post = Clock::now();
  objective_value_ = objective_scale_ * cost;
  extractCoefficients_(z);
  initializeGoalSetpoints();
  last_solve_timing_.postsolve_ms = elapsedMs(t_post);
  return SolveStatus::Optimal;
}

bool SolverDAQP::generateNewTrajectory(
    bool& solver_error_detected,
    double& computation_time,
    double factor,
    bool use_single_thread) {
  if (!use_single_thread) {
    last_solve_timing_ = SolveTimingBreakdown{};
    if (should_terminate_) return false;
    const auto t0 = Clock::now();
    findDT(factor);
    last_solve_timing_.findDT_ms = elapsedMs(t0);
    const SolveStatus status = solveCurrentAllocation_(computation_time);
    if (status == SolveStatus::Failed) solver_error_detected = true;
    return status == SolveStatus::Optimal;
  }

  // Sequential sweep over the factors
  solver_error_detected = false;
  computation_time = 0.0;
  factor_that_worked_ = 0.0;
  bool solved = false;
  bool failed = false;
  for (double f = factor_initial_; f <= factor_final_ + 1e-9 && !solved && !should_terminate_;
       f += factor_constant_step_size_) {
    findDT(f);
    const SolveStatus status = solveCurrentAllocation_(computation_time);
    if (status == SolveStatus::Failed) failed = true;
    if (status == SolveStatus::Stopped) break;
    if (status == SolveStatus::Optimal) {
      solved = true;
      factor_that_worked_ = f;
    }
  }
  solver_error_detected = failed && !solved;
  should_terminate_ = false;
  return solved;
}
