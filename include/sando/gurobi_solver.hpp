/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <sando/sando_type.hpp>
#include <sando/trajectory_solver.hpp>
#include "gurobi_c++.h"
#include <type_traits>

enum ConstraintType { POSITION, VELOCITY, ACCELERATION, JERK };

class MyCallback : public GRBCallback {
 public:
  std::atomic<bool> should_terminate_;
  MyCallback() : should_terminate_(false) {}

 protected:
  void callback();
};

class SolverGurobi : public TrajectorySolver {
 public:
  /** @brief Construct the solver, initialize the Gurobi model and basis converter. */
  SolverGurobi();
  ~SolverGurobi() override;

  /** @brief Set the number of threads Gurobi uses for optimization.
   *  @param num_threads Number of threads.
   */
  void setNumThreads(int num_threads) override;

  /** @brief Initialize the solver with planner parameters (segment count, limits, bounds, etc.).
   *  @param par Planner parameters struct.
   */
  void initializeSolver(const Parameters& par) override;

  /** @brief Generate a new trajectory for a given dynamic factor.
   *  @param gurobi_error_detected Set to true if a Gurobi exception occurs.
   *  @param gurobi_computation_time Set to the Gurobi solve time in milliseconds on success.
   *  @param factor Dynamic factor that scales the time allocation per segment.
   *  @param use_single_thread If true, use sequential factor sweeping instead.
   *  @return True if the optimization found a feasible solution.
   */
  bool generateNewTrajectory(
      bool& gurobi_error_detected,
      double& gurobi_computation_time,
      double factor,
      bool use_single_thread = false) override;

  /** @brief Try generating a trajectory by sweeping factors from initial to final.
   *  @param gurobi_error_detected Set to true if a Gurobi exception occurs.
   *  @param gurobi_computation_time_ms Set to the Gurobi solve time in milliseconds on success.
   *  @param factor_that_worked Set to the first factor value that produced a feasible solution.
   *  @return True if any factor in the sweep produced a feasible solution.
   */
  bool generateNewTrajectorySequentialFactors(
      bool& gurobi_error_detected, double& gurobi_computation_time_ms, double& factor_that_worked);

  /** @brief Run the Gurobi optimizer on the current model and check for optimality.
   *  @return True if the optimizer found an optimal solution.
   */
  bool callOptimizer();

  /** @brief Signal the Gurobi callback to abort the current optimization. */
  void stopExecution() override;

  /** @brief Reset the termination flag so the solver can run again after a stop. */
  void resetToNominalState() override;

  /** @brief Add polytope containment constraints to the Gurobi model via MIQP indicator
   * constraints. */
  void setPolytopesConstraints();

  /** @brief Create binary variables and per-segment safe corridor constraints for all polytopes. */
  void setPolyConsts();

  /** @brief Constrain all MINVO control points to lie within the map boundaries. */
  void setMapSizeConstraints();

  /** @brief Set the Gurobi objective to minimize the squared jerk norm. */
  void setObjective();

  /** @brief Add final-state equality constraints (position, velocity, acceleration) to the model.
   */
  void setConstraintsXf();

  /** @brief Add initial-state equality constraints (position, velocity, acceleration) to the model.
   */
  void setConstraintsX0();

  /** @brief Add position, velocity, and acceleration continuity constraints between adjacent
   * segments. */
  void setContinuityConstraints();

  /** @brief Check whether the solved trajectory satisfies velocity, acceleration, and jerk limits.
   *  @param is_dyn_constraints_satisfied Set to false if any MINVO control point violates limits.
   */
  void checkDynamicViolation(bool& is_dyn_constraints_satisfied);

  /** @brief Check whether all trajectory segments are contained within at least one polytope.
   *  @param is_collision_free_corridor_satisfied Set to false if any segment violates the corridor.
   */
  void checkCollisionViolation(bool& is_collision_free_corridor_satisfied);

  /** @brief Add MIQP indicator constraints requiring segment t to lie in at least one polytope.
   *  @param t Trajectory segment index.
   */
  void createSafeCorridorConstraintsForPolytopeAtleastOne(int t);

  /** @brief Add velocity, acceleration, and jerk constraints using the configured norm type
   * (Linf/L1/L2). */
  void setDynamicConstraints();

  /** @brief Create the Gurobi decision variables (free control point parameters d3, d4, ...). */
  void createVars();

  /** @brief Compute all polynomial coefficients as GRBLinExpr from free variables via variable
   * elimination. */
  void setX();

  /** @brief Remove all Gurobi variables from the model and clear coefficient storage. */
  void removeVars();

  /** @brief Compute all polynomial coefficients symbolically for N=4 segments via variable
   * elimination. */
  void computeDependentCoefficientsN4();

  /** @brief Compute all polynomial coefficients symbolically for N=5 segments via variable
   * elimination. */
  void computeDependentCoefficientsN5();

  /** @brief Compute all polynomial coefficients symbolically for N=6 segments via variable
   * elimination. */
  void computeDependentCoefficientsN6();

  /** @brief Extract numeric polynomial coefficients after optimization for N=4 segments. */
  void getDependentCoefficientsN4Double();

  /** @brief Extract numeric polynomial coefficients after optimization for N=5 segments. */
  void getDependentCoefficientsN5Double();

  /** @brief Extract numeric polynomial coefficients after optimization for N=6 segments. */
  void getDependentCoefficientsN6Double();

  /** @brief Add distance-based soft constraints to the model using stored obstacle samples. */
  void setDistanceConstraints();

  /** @brief Find the index in a time vector whose value is closest to the query time.
   *  @param t Query time.
   *  @param index Output closest index.
   *  @param time Vector of time values to search.
   */
  void findClosestIndexFromTime(const double t, int& index, const std::vector<double>& time);

  /** @brief Evaluate position as a symbolic Gurobi expression at a given segment and time.
   *  @param t Segment index.
   *  @param tau Local time within the segment.
   *  @param ii Axis index (0=x, 1=y, 2=z).
   *  @return Symbolic linear expression for position.
   */
  inline GRBLinExpr getPos(int t, double tau, int ii) const;

  /** @brief Evaluate velocity as a symbolic Gurobi expression at a given segment and time.
   *  @param t Segment index.
   *  @param tau Local time within the segment.
   *  @param ii Axis index (0=x, 1=y, 2=z).
   *  @return Symbolic linear expression for velocity.
   */
  inline GRBLinExpr getVel(int t, double tau, int ii) const;

  /** @brief Evaluate acceleration as a symbolic Gurobi expression at a given segment and time.
   *  @param t Segment index.
   *  @param tau Local time within the segment.
   *  @param ii Axis index (0=x, 1=y, 2=z).
   *  @return Symbolic linear expression for acceleration.
   */
  inline GRBLinExpr getAccel(int t, double tau, int ii) const;

  /** @brief Evaluate jerk as a symbolic Gurobi expression at a given segment and time.
   *  @param t Segment index.
   *  @param tau Local time within the segment.
   *  @param ii Axis index (0=x, 1=y, 2=z).
   *  @return Symbolic linear expression for jerk.
   */
  inline GRBLinExpr getJerk(int t, double tau, int ii) const;

  /** @brief Get the cubic coefficient 'a' as a symbolic expression for a segment and axis. */
  inline GRBLinExpr getA(int t, int ii) const;
  /** @brief Get the quadratic coefficient 'b' as a symbolic expression for a segment and axis. */
  inline GRBLinExpr getB(int t, int ii) const;
  /** @brief Get the linear coefficient 'c' as a symbolic expression for a segment and axis. */
  inline GRBLinExpr getC(int t, int ii) const;
  /** @brief Get the constant coefficient 'd' as a symbolic expression for a segment and axis. */
  inline GRBLinExpr getD(int t, int ii) const;

  /** @brief Get the normalized cubic coefficient as a symbolic expression for a segment and axis.
   */
  inline GRBLinExpr getAn(int t, int ii) const;
  /** @brief Get the normalized quadratic coefficient as a symbolic expression for a segment and
   * axis. */
  inline GRBLinExpr getBn(int t, int ii) const;
  /** @brief Get the normalized linear coefficient as a symbolic expression for a segment and axis.
   */
  inline GRBLinExpr getCn(int t, int ii) const;
  /** @brief Get the normalized constant coefficient as a symbolic expression for a segment and
   * axis. */
  inline GRBLinExpr getDn(int t, int ii) const;

  /** @brief Get the 0th MINVO position control point (3D) as symbolic expressions for segment t. */
  inline std::vector<GRBLinExpr> getCP0(int t) const;
  /** @brief Get the 1st MINVO position control point (3D) as symbolic expressions for segment t. */
  inline std::vector<GRBLinExpr> getCP1(int t) const;
  /** @brief Get the 2nd MINVO position control point (3D) as symbolic expressions for segment t. */
  inline std::vector<GRBLinExpr> getCP2(int t) const;
  /** @brief Get the 3rd MINVO position control point (3D) as symbolic expressions for segment t. */
  inline std::vector<GRBLinExpr> getCP3(int t) const;

  /** @brief Get all velocity control points as symbolic expressions for a segment and axis. */
  inline std::vector<GRBLinExpr> getVelCP(int interval, int axis) const;
  /** @brief Get all acceleration control points as symbolic expressions for a segment and axis. */
  inline std::vector<GRBLinExpr> getAccelCP(int interval, int axis) const;
  /** @brief Get all jerk control points as symbolic expressions for a segment and axis. */
  inline std::vector<GRBLinExpr> getJerkCP(int interval, int axis) const;

  /** @brief Get all MINVO position control points as symbolic expressions for segment t.
   *  @return 3-by-4 nested vector (outer: axis, inner: control point index).
   */
  inline std::vector<std::vector<GRBLinExpr>> getMinvoPosControlPoints(int t) const;

  /** @brief Get all MINVO velocity control points as symbolic expressions for segment t.
   *  @return 3-by-3 nested vector.
   */
  inline std::vector<std::vector<GRBLinExpr>> getMinvoVelControlPoints(int t) const;

  /** @brief Get all MINVO acceleration control points as symbolic expressions for segment t.
   *  @return 3-by-2 nested vector.
   */
  inline std::vector<std::vector<GRBLinExpr>> getMinvoAccelControlPoints(int t) const;

  /** @brief Get all MINVO jerk control points as symbolic expressions for segment t.
   *  @return 3-by-1 nested vector.
   */
  inline std::vector<std::vector<GRBLinExpr>> getMinvoJerkControlPoints(int t) const;

  /** @brief Get MINVO position control points as a 3x4 numeric matrix for segment t
   * (post-optimization).
   *  @return 3x4 matrix where each column is a control point.
   */
  inline Eigen::Matrix<double, 3, 4> getMinvoPosControlPointsDouble(int t) const;

  /** @brief Get MINVO velocity control points as a 3x3 numeric matrix for segment t
   * (post-optimization).
   *  @return 3x3 matrix where each column is a control point.
   */
  inline Eigen::Matrix<double, 3, 3> getMinvoVelControlPointsDouble(int t) const;

  /** @brief Get MINVO acceleration control points as a 3x2 numeric matrix for segment t
   * (post-optimization).
   *  @return 3x2 matrix where each column is a control point.
   */
  inline Eigen::Matrix<double, 3, 2> getMinvoAccelControlPointsDouble(int t) const;

  /** @brief Get MINVO jerk control points as a 3x1 numeric vector for segment t
   * (post-optimization).
   *  @return 3x1 matrix (single control point).
   */
  inline Eigen::Matrix<double, 3, 1> getMinvoJerkControlPointsDouble(int t) const;

  /** @brief Check whether a control point depends on any free decision variable.
   *  @param type Constraint type (POSITION, VELOCITY, ACCELERATION, or JERK).
   *  @param seg Segment index.
   *  @param cp Control point index within the segment.
   *  @return True if the control point expression depends on free variables.
   */
  bool controlPointDepends(ConstraintType type, int seg, int cp);

  /** @brief Check whether a control point depends on the d3 free variable (N=4 case).
   *  @param type Constraint type.
   *  @param seg Segment index.
   *  @param cp Control point index.
   *  @return True if the control point depends on d3.
   */
  bool controlPointDependsOnD3(ConstraintType type, int seg, int cp);

  /** @brief Check whether a control point depends on d3 or d4 free variables (N=5 case).
   *  @param type Constraint type.
   *  @param seg Segment index.
   *  @param cp Control point index.
   *  @return True if the control point depends on d3 or d4.
   */
  bool controlPointDependsOnD3OrD4(ConstraintType type, int seg, int cp);

  /** @brief Check whether a control point depends on d3, d4, or d5 free variables (N=6 case).
   *  @param type Constraint type.
   *  @param seg Segment index.
   *  @param cp Control point index.
   *  @return True if the control point depends on d3, d4, or d5.
   */
  bool controlPointDependsOnD3OrD4OrD5(ConstraintType type, int seg, int cp);

  /** @brief Get MINVO position control points for all segments (post-optimization).
   *  @param cps Output vector of 3x4 matrices, one per segment.
   */
  void getMinvoControlPoints(std::vector<Eigen::Matrix<double, 3, 4>>& cps);

  int trials_ = 0;
  int file_t_ = 0;
  MyCallback cb_;

 protected:
  std::vector<std::vector<GRBVar>> x_faster_vars_;  // [axis][4*N] coefficient vars for FASTER
  void createVarsFaster_();
  void setXFaster_();
  void getCoefficientsDoubleFaster_();
  void setDynamicConstraintsFaster_();
  void setDynamicConstraintsSafeFaster_();

  // parameters
  double cost_;
  std::vector<DynTraj> trajs_;  // Dynamic trajectory
  PieceWisePol pwp_;
  vec_Vecf<3> global_path_;
  std::vector<float> sfc_size_;

  // Basis converter
  BasisConverter basis_converter_;
  std::vector<std::vector<double>> M_be2mv_;
  Eigen::Matrix<double, 4, 4> A_pos_mv_rest_inv_;
  Eigen::Matrix<double, 3, 3> A_vel_mv_rest_inv_;
  Eigen::Matrix<double, 2, 2> A_accel_mv_rest_inv_;

  int N_of_polytopes_ = 3;

  // Shared Gurobi environment (singleton) — avoids repeated license token
  // acquisition when multiple SolverGurobi instances are created.
  // Thread-safe initialization via C++11 static local.
  static GRBEnv* getSharedEnv() {
    static GRBEnv* e = []() {
      GRBEnv* env = new GRBEnv(true);
      env->set(GRB_IntParam_OutputFlag, 0);
      env->set(GRB_IntParam_LogToConsole, 0);
      env->start();
      return env;
    }();
    return e;
  }
  GRBModel m_ = GRBModel(*getSharedEnv());

  std::vector<GRBConstr> at_least_1_pol_cons_;     // Constraints at least in one polytope
  std::vector<GRBConstr> polytopes_cons_;          // for SANDO
  std::vector<GRBGenConstr> miqp_polytopes_cons_;  // for MIQP
  std::vector<GRBConstr> continuity_cons_;
  std::vector<GRBConstr> init_cons_;
  std::vector<GRBConstr> final_cons_;
  std::vector<GRBConstr> map_cons_;
  std::vector<GRBConstr> dyn_cons_;
  std::vector<GRBQConstr> dyn_qcons_;  // Quadratic constraints (for norm-based mode)

  std::vector<std::vector<GRBVar>> b_;  // binary variables (only used by the MIQP)
  std::vector<std::vector<GRBLinExpr>> x_;
  std::vector<std::vector<GRBLinExpr>> p_cp_;
  std::vector<std::vector<GRBLinExpr>> v_cp_;
  std::vector<std::vector<GRBLinExpr>> a_cp_;
  std::vector<std::vector<GRBLinExpr>> j_cp_;
  std::vector<GRBVar> d3_;
  std::vector<GRBVar> d4_;
  std::vector<GRBVar> d5_;
  std::vector<GRBVar> d6_;
  std::vector<GRBVar> d7_;
  std::vector<GRBVar> d8_;

  vec_Vecf<3> samples_;           // Samples along the rescue path
  vec_Vecf<3> samples_penalize_;  // Samples along the rescue path

  std::vector<double> dist_near_obs_;

};