/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <decomp_geometry/polyhedron.h>

#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <sando/sando_type.hpp>
#include <string>
#include <unsupported/Eigen/Polynomials>
#include <vector>

#include "hgp/termcolor.hpp"
#include "timer.hpp"

using namespace termcolor;
typedef timer::Timer MyTimer;

/**
 * @brief Backend-independent interface of the local trajectory optimizer.
 *
 * Holds the problem data (boundary conditions, time allocation, safe corridor) and everything that
 * can be computed from the solved polynomial coefficients. A backend only has to fill x_double_ in
 * generateNewTrajectory(), with the coefficients of segment i for an axis stored as
 * x_double_[axis][4 * i + {0, 1, 2, 3}] = {a, b, c, d}.
 */
class TrajectorySolver {
 public:
  // Sub-step timing breakdown from last generateNewTrajectory call (ms)
  struct SolveTimingBreakdown {
    double findDT_ms{0.0};
    double setX_ms{0.0};
    double polytopes_ms{0.0};
    double dynamic_ms{0.0};
    double objective_ms{0.0};
    double mapsize_ms{0.0};
    double callOptimizer_ms{0.0};
    double postsolve_ms{0.0};
  };
  SolveTimingBreakdown last_solve_timing_;

  TrajectorySolver() = default;
  virtual ~TrajectorySolver() = default;
  TrajectorySolver(const TrajectorySolver&) = delete;
  TrajectorySolver& operator=(const TrajectorySolver&) = delete;

  /** @brief Initialize the solver with planner parameters (segment count, limits, bounds, etc.).
   *  @param par Planner parameters struct.
   */
  virtual void initializeSolver(const Parameters& par) = 0;

  /** @brief Generate a new trajectory for a given dynamic factor.
   *  @param solver_error_detected Set to true if the backend fails for numerical/internal reasons.
   *  @param computation_time Set to the optimizer time in milliseconds on success.
   *  @param factor Dynamic factor that scales the time allocation per segment.
   *  @param use_single_thread If true, use sequential factor sweeping instead.
   *  @return True if the optimization found an optimal solution.
   */
  virtual bool generateNewTrajectory(
      bool& solver_error_detected,
      double& computation_time,
      double factor,
      bool use_single_thread = false) = 0;

  /** @brief Abort the current optimization. May be called from another thread. */
  virtual void stopExecution() = 0;

  /** @brief Reset the termination flag so the solver can run again after a stop. */
  virtual void resetToNominalState() = 0;

  /** @brief Set the number of threads used per solver (no-op for single-threaded backends).
   *  @param num_threads Number of threads.
   */
  virtual void setNumThreads(int /*num_threads*/) {}

  /** @brief Set the planner name to select the formulation (e.g., "SANDO" or "FASTER").
   *  @param name Planner identifier string.
   */
  void setPlannerName(const std::string& name);

  /** @brief Set the initial state (position, velocity, acceleration) for the trajectory.
   *  @param data Initial state.
   */
  void setX0(const RobotState& data);

  /** @brief Set the trajectory start time.
   *  @param t0 Start time in seconds.
   */
  void setT0(double t0);

  /** @brief Set the final (goal) state for the trajectory.
   *  @param data Final state.
   */
  void setXf(const RobotState& data);

  /** @brief Set the desired final yaw direction.
   *  @param yawf Final yaw angle in radians.
   */
  void setDirf(double yawf);

  /** @brief Retrieve the total trajectory duration.
   *  @param total_traj_time Output parameter set to the sum of all segment durations.
   */
  void getTotalTrajTime(double& total_traj_time);

  /** @brief Allocate and initialize the goal setpoints vector based on total trajectory time. */
  void initializeGoalSetpoints();

  /** @brief Set time-invariant safety corridor polytopes.
   *  @param polytopes Vector of linear constraints defining convex polytopes.
   */
  void setPolytopes(std::vector<LinearConstraint3D> polytopes);

  /** @brief Set time-layered safety corridor polytopes (one set per trajectory segment).
   *  @param polytopes_by_time Outer vector indexed by time step, inner by polytope index.
   */
  void setPolytopesTimeLayered(
      const std::vector<std::vector<LinearConstraint3D>>& polytopes_by_time);

  /** @brief Compute uniform time allocations for all segments given a dynamic factor.
   *  @param factor Scaling factor applied to the base time step.
   */
  void findDT(double factor);

  /** @brief Compute an initial time step estimate from the distance and dynamic limits.
   *  @return Estimated per-segment time step in seconds.
   */
  double getInitialDt();

  /** @brief Evaluate the solved trajectory at regular intervals to populate goal setpoints. */
  void fillGoalSetPoints();

  /** @brief Extract initial and final position, velocity, and acceleration for a given axis.
   *  @param P0 Output initial position.
   *  @param V0 Output initial velocity.
   *  @param A0 Output initial acceleration.
   *  @param Pf Output final position.
   *  @param Vf Output final velocity.
   *  @param Af Output final acceleration.
   *  @param axis Axis index (0=x, 1=y, 2=z).
   */
  void getInitialAndFinalConditions(
      double& P0, double& V0, double& A0, double& Pf, double& Vf, double& Af, int axis);

  /** @brief Find which trajectory segment contains a given absolute time and the local offset.
   *  @param time_in_whole_traj Time offset from trajectory start.
   *  @param interval_idx Output segment index.
   *  @param dt_interval Output time offset within the segment.
   */
  void findIntervalIdxAndDt(double time_in_whole_traj, int& interval_idx, double& dt_interval);

  /** @brief Extract the solved trajectory as a piecewise polynomial with time knots and
   * coefficients.
   *  @param pwp Output piecewise polynomial struct populated with times and per-axis coefficients.
   */
  void getPieceWisePol(PieceWisePol& pwp);

  /** @brief Get Bezier position control points for all segments (post-optimization).
   *  @param cps Output vector of 3x4 matrices, one per segment.
   */
  void getControlPoints(std::vector<Eigen::Matrix<double, 3, 4>>& cps);

  /** @brief Retrieve the goal setpoints computed from the solved trajectory.
   *  @param goal_setpoints Output vector of evenly-spaced state samples along the trajectory.
   */
  void getGoalSetpoints(std::vector<RobotState>& goal_setpoints);

  /** @brief Set the initial per-segment time step used as the base for factor scaling.
   *  @param initial_dt Base time step in seconds.
   */
  void setInitialDt(double initial_dt);

  /** @brief Get the dynamic factor value that produced the last successful solution.
   *  @return Factor value, or 0 if no solution has been found.
   */
  double getFactorThatWorked();

  /** @brief Get the objective function value from the last successful optimization.
   *  @return Objective value, or NaN if no solution has been found.
   */
  double getObjectiveValue() const { return objective_value_; }

  /** @brief Evaluate position numerically at a given segment and time (post-optimization). */
  inline double getPosDouble(int t, double tau, int ii) const;
  /** @brief Evaluate velocity numerically at a given segment and time (post-optimization). */
  inline double getVelDouble(int t, double tau, int ii) const;
  /** @brief Evaluate acceleration numerically at a given segment and time (post-optimization). */
  inline double getAccelDouble(int t, double tau, int ii) const;
  /** @brief Evaluate jerk numerically at a given segment and time (post-optimization). */
  inline double getJerkDouble(int t, double tau, int ii) const;

  /** @brief Get the normalized cubic coefficient as a numeric value (post-optimization). */
  inline double getAnDouble(int t, int ii) const;
  /** @brief Get the normalized quadratic coefficient as a numeric value (post-optimization). */
  inline double getBnDouble(int t, int ii) const;
  /** @brief Get the normalized linear coefficient as a numeric value (post-optimization). */
  inline double getCnDouble(int t, int ii) const;
  /** @brief Get the normalized constant coefficient as a numeric value (post-optimization). */
  inline double getDnDouble(int t, int ii) const;

  /** @brief Get the 0th position control point (3D) as numeric values for segment t. */
  inline std::vector<double> getCP0Double(int t) const;
  /** @brief Get the 1st position control point (3D) as numeric values for segment t. */
  inline std::vector<double> getCP1Double(int t) const;
  /** @brief Get the 2nd position control point (3D) as numeric values for segment t. */
  inline std::vector<double> getCP2Double(int t) const;
  /** @brief Get the 3rd position control point (3D) as numeric values for segment t. */
  inline std::vector<double> getCP3Double(int t) const;

  /** @brief Get Bezier position control points as a 3x4 numeric matrix for segment t
   * (post-optimization).
   *  @return 3x4 matrix where each column is a control point.
   */
  inline Eigen::Matrix<double, 3, 4> getPosControlPointsDouble(int t) const;

  double objective_value_{std::numeric_limits<double>::quiet_NaN()};
  std::vector<RobotState> goal_setpoints_;
  std::vector<double> dt_;  // time step found by the solver
  double total_traj_time_;
  double factor_that_worked_ = 0;
  int N_ = 6;

 protected:
  /** @brief Store the parameters shared by all backends. */
  void setCommonParameters_(const Parameters& par);

  bool usingFaster_() const;
  const LinearConstraint3D& polyAt_(int t, int p) const;
  bool hasPolytopes_() const;
  int numSpatialPolys_() const;

  std::string planner_name_{"SANDO"};  // "SANDO" or "FASTER"

  // parameters
  double xf_[3 * 3];
  double dirf_[2];
  double x0_[3 * 3];
  double t0_;
  std::string dynamic_constraint_type_;  // "Linf", "L1", or "L2"
  double v_max_;
  double a_max_;
  double j_max_;
  double dc_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
  double initial_dt_;
  bool using_variable_elimination_ = true;
  double max_comp_time_sec_ = 1.0;

  // Flags
  bool debug_verbose_;

  // Solved polynomial coefficients
  std::vector<std::vector<double>> x_double_;

  std::vector<LinearConstraint3D> polytopes_;

  // Optimization weights
  double jerk_smooth_weight_ = 10.0;

  double factor_initial_ = 0.6;
  double factor_final_ = 2.0;
  double factor_constant_step_size_ = 0.1;
  double w_max_ = 1;

  // Time-layered corridor support
  bool use_time_layered_polytopes_{false};
  int P_spatial_{0};
  std::vector<LinearConstraint3D> polytopes_time_layered_;  // flattened: [t * P_spatial_ + p]
};

/**
 * @brief Create a trajectory solver.
 * @param backend "gurobi", "daqp", or "auto" (Gurobi if it is available and licensed, else DAQP).
 * @throws std::invalid_argument if the backend is unknown or SANDO was built without it.
 * @throws std::runtime_error if "gurobi" is requested, but it cannot be started (e.g., no license).
 */
std::shared_ptr<TrajectorySolver> makeTrajectorySolver(const std::string& backend);

// ---------------------------------------------------------------------------------------------
// Inline evaluation of the solved polynomials
// ---------------------------------------------------------------------------------------------

inline double TrajectorySolver::getPosDouble(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  return x_double_[axis][base_index] * tau * tau * tau +
         x_double_[axis][base_index + 1] * tau * tau + x_double_[axis][base_index + 2] * tau +
         x_double_[axis][base_index + 3];
}

inline double TrajectorySolver::getVelDouble(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  // For the numeric (double) version we assume the solved (numeric) values are stored in x_double_
  return 3 * x_double_[axis][base_index] * tau * tau + 2 * x_double_[axis][base_index + 1] * tau +
         x_double_[axis][base_index + 2];
}

inline double TrajectorySolver::getAccelDouble(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  return 6 * x_double_[axis][base_index] * tau + 2 * x_double_[axis][base_index + 1];
}

inline double TrajectorySolver::getJerkDouble(int interval, double tau, int axis) const {
  return 6 * x_double_[axis][4 * interval];
}

inline double TrajectorySolver::getAnDouble(int interval, int axis) const {
  return x_double_[axis][4 * interval] * dt_[interval] * dt_[interval] * dt_[interval];
}

inline double TrajectorySolver::getBnDouble(int interval, int axis) const {
  return x_double_[axis][4 * interval + 1] * dt_[interval] * dt_[interval];
}

inline double TrajectorySolver::getCnDouble(int interval, int axis) const {
  return x_double_[axis][4 * interval + 2] * dt_[interval];
}

inline double TrajectorySolver::getDnDouble(int interval, int axis) const {
  return x_double_[axis][4 * interval + 3];
}

inline std::vector<double> TrajectorySolver::getCP0Double(int interval) const {
  std::vector<double> cp = {
      getPosDouble(interval, 0, 0), getPosDouble(interval, 0, 1), getPosDouble(interval, 0, 2)};
  return cp;
}

inline std::vector<double> TrajectorySolver::getCP1Double(int interval) const {
  double cpx = (getCnDouble(interval, 0) + 3 * getDnDouble(interval, 0)) / 3;
  double cpy = (getCnDouble(interval, 1) + 3 * getDnDouble(interval, 1)) / 3;
  double cpz = (getCnDouble(interval, 2) + 3 * getDnDouble(interval, 2)) / 3;
  std::vector<double> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<double> TrajectorySolver::getCP2Double(int interval) const {
  double cpx =
      (getBnDouble(interval, 0) + 2 * getCnDouble(interval, 0) + 3 * getDnDouble(interval, 0)) / 3;
  double cpy =
      (getBnDouble(interval, 1) + 2 * getCnDouble(interval, 1) + 3 * getDnDouble(interval, 1)) / 3;
  double cpz =
      (getBnDouble(interval, 2) + 2 * getCnDouble(interval, 2) + 3 * getDnDouble(interval, 2)) / 3;
  std::vector<double> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<double> TrajectorySolver::getCP3Double(int interval) const {
  std::vector<double> cp = {
      getPosDouble(interval, dt_[interval], 0), getPosDouble(interval, dt_[interval], 1),
      getPosDouble(interval, dt_[interval], 2)};
  return cp;
}

inline Eigen::Matrix<double, 3, 4> TrajectorySolver::getPosControlPointsDouble(int interval) const {
  // Compute the normalized coefficient matrix Pn (for u ∈ [0,1])
  Eigen::Matrix<double, 3, 4> Pn;
  std::vector<double> cp0 = getCP0Double(interval);
  std::vector<double> cp1 = getCP1Double(interval);
  std::vector<double> cp2 = getCP2Double(interval);
  std::vector<double> cp3 = getCP3Double(interval);
  Pn << cp0[0], cp1[0], cp2[0], cp3[0], cp0[1], cp1[1], cp2[1], cp3[1], cp0[2], cp1[2], cp2[2],
      cp3[2];
  return Pn;
}
