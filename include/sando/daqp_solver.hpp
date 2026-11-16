/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <sando/trajectory_solver.hpp>
#include <vector>

/**
 * @brief License-free trajectory optimizer based on the MIQP support of the solver DAQP.
 *
 * Solves the same problem as SolverGurobi: minimize the squared jerk subject to the dynamic limits,
 * the map bounds and, for each segment, all control points being in at least one polytope.
 *
 * The cubic coefficients are eliminated numerically, which leaves the problem in the free
 * parameters z = {d3, ..., d_(N-1)} per axis (see buildReducedProblem_). The choice of polytopes is
 * modeled with one binary variable per segment and polytope and big-M constraints (see solveMIQP_).
 *
 * A solve cannot be interrupted: stopExecution only prevents a solve from starting and its result
 * from being used. The running time of a solve is bounded by the time limit.
 *
 * The L2 dynamic constraints are not supported, since they are not linear.
 */
class SolverDAQP : public TrajectorySolver {
 public:
  /** @throws std::invalid_argument if the configuration cannot be handled by this backend. */
  void initializeSolver(const Parameters& par) override;

  bool generateNewTrajectory(
      bool& solver_error_detected,
      double& computation_time,
      double factor,
      bool use_single_thread = false) override;

  void stopExecution() override { should_terminate_ = true; }
  void resetToNominalState() override { should_terminate_ = false; }

  /** @brief Number of branch-and-bound nodes in the last solve. */
  int getLastNodeCount() const { return node_count_; }

 private:
  using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  // Linear constraints lower <= A * z <= upper
  struct RowSet {
    RowMatrix A;
    Eigen::VectorXd lower, upper;
    bool infeasible{false};  // Contains a constraint that does not depend on z and that is violated
  };

  enum class SolveStatus { Optimal, Infeasible, Stopped, Failed };

  // Solve for one time allocation (dt_ must be set)
  SolveStatus solveCurrentAllocation_(double& computation_time_ms);

  void buildReducedProblem_();
  void buildDynamicAndMapRows_();
  void buildCorridorRows_();
  void buildObjective_();

  // Maps a linear functional of the normalized coefficients of a segment to a row in z
  void functionalToRow_(
      int segment,
      int axis,
      const Eigen::RowVector4d& functional,
      Eigen::RowVectorXd& row,
      double& offset) const;

  void addRow_(
      std::vector<Eigen::RowVectorXd>& rows,
      std::vector<double>& lower,
      std::vector<double>& upper,
      bool& infeasible,
      const Eigen::RowVectorXd& row,
      double row_lower,
      double row_upper) const;

  SolveStatus solveMIQP_(Eigen::VectorXd& z, double& cost);
  double suboptimalityTolerance_() const;

  void extractCoefficients_(const Eigen::VectorXd& z);

  std::atomic<bool> should_terminate_{false};

  // Normalized coefficients (a*T^3, b*T^2, c*T, d) of all segments: G_ * z_axis + g_.col(axis)
  int num_free_{0};  // Number of free parameters per axis
  int n_{0};         // Total number of decision variables
  Eigen::MatrixXd G_;
  Eigen::MatrixXd g_;

  // The objective w*sum||jerk||^2 is objective_scale_ * (0.5 * z'*H*z + f'*z + objective_offset_),
  // where the scale makes the QP independent of the magnitude of the jerk
  double objective_scale_{1.0};
  double objective_offset_{0.0};
  RowMatrix H_;
  Eigen::VectorXd f_;
  Eigen::VectorXd z_lower_, z_upper_;

  RowSet common_rows_;                         // Dynamic limits and map bounds
  std::vector<std::vector<RowSet>> corridor_;  // [segment][polytope]; empty polytopes are skipped

  int node_count_{0};

  // Buffers for the MIQP
  RowMatrix H_miqp_, A_miqp_;
  Eigen::VectorXd f_miqp_, lower_miqp_, upper_miqp_, x_miqp_, lam_miqp_;
  std::vector<int> sense_miqp_;
};
