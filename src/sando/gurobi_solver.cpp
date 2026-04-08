/* ----------------------------------------------------------------------------
 * Copyright (c) Anonymous Author
 * Anonymous Institution
 * All Rights Reserved
 * Authors: Anonymous
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <sando/gurobi_solver.hpp>
#include <sando/gurobi_solver_utils.hpp>

void MyCallback::callback() {  // This function is called periodically along the optimization
                               // process.
  //  It is called several times more after terminating the program
  if (should_terminate_ == true) {
    GRBCallback::abort();  // This function only does effect when inside the function callback() of
                           // this class terminated_ = true;
  }
}

void SolverGurobi::stopExecution() { cb_.should_terminate_ = true; }

void SolverGurobi::resetToNominalState() { cb_.should_terminate_ = false; }

SolverGurobi::SolverGurobi() {
  // Model
  m_.set(GRB_StringAttr_ModelName, "planning");

  // Debug
  m_.set(GRB_IntParam_OutputFlag, 0);  // 0: no output, 1: output

  // Set the callback
  m_.setCallback(&cb_);  // The callback will be called periodically along the optimization

  // Get basis converter (Q_{MINVO} = M_{BE2MV} * Q_{BEZIER})
  // Get std version of the basis converter
  M_be2mv_ = eigenMatrix2std(basis_converter_.getMinvoPosConverterFromBezier());
  A_pos_mv_rest_inv_ = basis_converter_.A_pos_mv_rest_inv;
  A_vel_mv_rest_inv_ = basis_converter_.A_vel_mv_rest_inv;
  A_accel_mv_rest_inv_ = basis_converter_.A_accel_mv_rest_inv;
}

SolverGurobi::~SolverGurobi() {}

bool SolverGurobi::usingBaseline_() const {
  return (
      planner_name_ == "BASELINE" || planner_name_ == "original_baseline" ||
      planner_name_ == "safe_baseline");
}

void SolverGurobi::setPlannerName(const std::string& name) { planner_name_ = name; }

void SolverGurobi::setGurobiThreads(int num_threads) { m_.set(GRB_IntParam_Threads, num_threads); }

void SolverGurobi::createVarsBaseline_() {
  // Create cubic coefficients per interval, per axis: a,b,c,d for each interval
  // Stored in the same flattened layout your getPos/getVel/getAccel expect:
  // x_[axis][4*interval + 0..3]
  x_baseline_vars_.clear();
  x_baseline_vars_.resize(3);

  for (int axis = 0; axis < 3; ++axis) {
    x_baseline_vars_[axis].reserve(4 * N_);
    for (int interval = 0; interval < N_; ++interval) {
      const std::string s = "_ax" + std::to_string(axis) + "_k" + std::to_string(interval);
      x_baseline_vars_[axis].push_back(
          m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "a" + s));
      x_baseline_vars_[axis].push_back(
          m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "b" + s));
      x_baseline_vars_[axis].push_back(
          m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "c" + s));
      x_baseline_vars_[axis].push_back(
          m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d" + s));
    }
  }
}

void SolverGurobi::setXBaseline_() {
  // Ensure x_ is a GRBLinExpr view into the vars
  x_.clear();
  x_.resize(3);

  for (int axis = 0; axis < 3; ++axis) {
    x_[axis].reserve(4 * N_);
    for (int i = 0; i < 4 * N_; ++i) x_[axis].push_back(GRBLinExpr(x_baseline_vars_[axis][i]));
  }
}

void SolverGurobi::getCoefficientsDoubleBaseline_() {
  x_double_.clear();
  x_double_.resize(3);

  for (int axis = 0; axis < 3; ++axis) {
    x_double_[axis].resize(4 * N_);
    for (int i = 0; i < 4 * N_; ++i)
      x_double_[axis][i] = x_baseline_vars_[axis][i].get(GRB_DoubleAttr_X);
  }
}

void SolverGurobi::setDynamicConstraintsBaseline_() {
  // Remove previous dynamic constraints (reuse same dyn_cons_ vector)
  if (!dyn_cons_.empty()) {
    for (auto& c : dyn_cons_) m_.remove(c);
    dyn_cons_.clear();
  }

  // Original BASELINE: only constrain first control point per segment
  for (int segment = 0; segment < N_; ++segment) {
    for (int axis = 0; axis < 3; ++axis) {
      dyn_cons_.push_back(m_.addConstr(
          getVel(segment, 0, axis) <= v_max_,
          "MaxVel_t" + std::to_string(segment) + "_axis_" + std::to_string(axis)));
      dyn_cons_.push_back(m_.addConstr(
          getVel(segment, 0, axis) >= -v_max_,
          "MinVel_t" + std::to_string(segment) + "_axis_" + std::to_string(axis)));

      dyn_cons_.push_back(m_.addConstr(
          getAccel(segment, 0, axis) <= a_max_,
          "MaxAccel_t" + std::to_string(segment) + "_axis_" + std::to_string(axis)));
      dyn_cons_.push_back(m_.addConstr(
          getAccel(segment, 0, axis) >= -a_max_,
          "MinAccel_t" + std::to_string(segment) + "_axis_" + std::to_string(axis)));

      dyn_cons_.push_back(m_.addConstr(
          getJerk(segment, 0, axis) <= j_max_,
          "MaxJerk_t" + std::to_string(segment) + "_axis_" + std::to_string(axis)));
      dyn_cons_.push_back(m_.addConstr(
          getJerk(segment, 0, axis) >= -j_max_,
          "MinJerk_t" + std::to_string(segment) + "_axis_" + std::to_string(axis)));
    }
  }
}

void SolverGurobi::setDynamicConstraintsSafeBaseline_() {
  // Remove previous dynamic constraints
  if (!dyn_cons_.empty()) {
    for (auto& c : dyn_cons_) m_.remove(c);
    dyn_cons_.clear();
  }

  // Safe BASELINE: constrain ALL control points per segment (like SANDO Linf)
  for (int segment = 0; segment < N_; ++segment) {
    for (int axis = 0; axis < 3; ++axis) {
      // --- Velocity constraints (3 control points) ---
      std::vector<GRBLinExpr> vel_cps = getVelCP(segment, axis);
      for (size_t i = 0; i < vel_cps.size(); ++i) {
        dyn_cons_.push_back(m_.addConstr(
            vel_cps[i] <= v_max_, "SafeMaxVel_seg" + std::to_string(segment) + "_ax" +
                                      std::to_string(axis) + "_cp" + std::to_string(i)));
        dyn_cons_.push_back(m_.addConstr(
            vel_cps[i] >= -v_max_, "SafeMinVel_seg" + std::to_string(segment) + "_ax" +
                                       std::to_string(axis) + "_cp" + std::to_string(i)));
      }

      // --- Acceleration constraints (2 control points) ---
      std::vector<GRBLinExpr> accel_cps = getAccelCP(segment, axis);
      for (size_t i = 0; i < accel_cps.size(); ++i) {
        dyn_cons_.push_back(m_.addConstr(
            accel_cps[i] <= a_max_, "SafeMaxAccel_seg" + std::to_string(segment) + "_ax" +
                                        std::to_string(axis) + "_cp" + std::to_string(i)));
        dyn_cons_.push_back(m_.addConstr(
            accel_cps[i] >= -a_max_, "SafeMinAccel_seg" + std::to_string(segment) + "_ax" +
                                         std::to_string(axis) + "_cp" + std::to_string(i)));
      }

      // --- Jerk constraints (1 control point) ---
      std::vector<GRBLinExpr> jerk_cps = getJerkCP(segment, axis);
      for (size_t i = 0; i < jerk_cps.size(); ++i) {
        dyn_cons_.push_back(m_.addConstr(
            jerk_cps[i] <= j_max_, "SafeMaxJerk_seg" + std::to_string(segment) + "_ax" +
                                       std::to_string(axis) + "_cp" + std::to_string(i)));
        dyn_cons_.push_back(m_.addConstr(
            jerk_cps[i] >= -j_max_, "SafeMinJerk_seg" + std::to_string(segment) + "_ax" +
                                        std::to_string(axis) + "_cp" + std::to_string(i)));
      }
    }
  }
}

void SolverGurobi::initializeSolver(const Parameters& par) {
  N_ = par.num_N;
  dc_ = par.dc;
  x_min_ = par.x_min;
  x_max_ = par.x_max;
  y_min_ = par.y_min;
  y_max_ = par.y_max;
  z_min_ = par.z_min;
  z_max_ = par.z_max;
  dynamic_constraint_type_ = par.dynamic_constraint_type;
  v_max_ = par.v_max;
  a_max_ = par.a_max;
  j_max_ = par.j_max;
  factor_initial_ = par.factor_initial;
  factor_final_ = par.factor_final;
  factor_constant_step_size_ = par.factor_constant_step_size;
  w_max_ = par.w_max;
  debug_verbose_ = par.debug_verbose;
  jerk_smooth_weight_ = par.jerk_smooth_weight;
  using_variable_elimination_ =
      par.using_variable_elimination;  // benchmark param for SANDO with/without var elimination

  // Time limit
  m_.set(GRB_DoubleParam_TimeLimit, par.max_gurobi_comp_time_sec);

  if (usingBaseline_() || !using_variable_elimination_) {
    createVarsBaseline_();
    using_variable_elimination_ = false;  // BASELINE does not use variable elimination
  } else {
    createVars();  // your current variable-elimination free vars (d3/d4/...)
  }
}

void SolverGurobi::setT0(double t0) { t0_ = t0; }

void SolverGurobi::createVars() {
  // Conservative position bounds for "free" position-like parameters.
  // Widened beyond map bounds to avoid restricting feasible solutions near edges.
  const double margin_xy = 5.0;
  const double margin_z = 2.0;

  const double xmin = x_min_ - margin_xy, xmax = x_max_ + margin_xy;
  const double ymin = y_min_ - margin_xy, ymax = y_max_ + margin_xy;
  const double zmin = z_min_ - margin_z, zmax = z_max_ + margin_z;

  // Utility lambda to create bounded vars per axis.
  auto add_pos_like_var = [&](const std::string& name, int axis) -> GRBVar {
    if (axis == 0) return m_.addVar(xmin, xmax, 0.0, GRB_CONTINUOUS, name);
    if (axis == 1) return m_.addVar(ymin, ymax, 0.0, GRB_CONTINUOUS, name);
    return m_.addVar(zmin, zmax, 0.0, GRB_CONTINUOUS, name);  // axis == 2
  };

  if (N_ >= 4) {
    d3_.clear();
    d3_.reserve(3);
    d3_.push_back(add_pos_like_var("d3x", 0));
    d3_.push_back(add_pos_like_var("d3y", 1));
    d3_.push_back(add_pos_like_var("d3z", 2));
  }

  if (N_ >= 5) {
    d4_.clear();
    d4_.reserve(3);
    d4_.push_back(add_pos_like_var("d4x", 0));
    d4_.push_back(add_pos_like_var("d4y", 1));
    d4_.push_back(add_pos_like_var("d4z", 2));
  }

  if (N_ >= 6) {
    d5_.clear();
    d5_.reserve(3);
    d5_.push_back(add_pos_like_var("d5x", 0));
    d5_.push_back(add_pos_like_var("d5y", 1));
    d5_.push_back(add_pos_like_var("d5z", 2));
  }

  if (N_ >= 7) {
    d6_.clear();
    d6_.reserve(3);
    d6_.push_back(add_pos_like_var("d6x", 0));
    d6_.push_back(add_pos_like_var("d6y", 1));
    d6_.push_back(add_pos_like_var("d6z", 2));
  }

  if (N_ >= 8) {
    d7_.clear();
    d7_.reserve(3);
    d7_.push_back(add_pos_like_var("d7x", 0));
    d7_.push_back(add_pos_like_var("d7y", 1));
    d7_.push_back(add_pos_like_var("d7z", 2));
  }

  if (N_ >= 9) {
    d8_.clear();
    d8_.reserve(3);
    d8_.push_back(add_pos_like_var("d8x", 0));
    d8_.push_back(add_pos_like_var("d8y", 1));
    d8_.push_back(add_pos_like_var("d8z", 2));
  }
}

void SolverGurobi::setX() {
  // Depending on N_, we initialize coefficients as GRBLinExpr
  if (N_ == 4) {
    computeDependentCoefficientsN4();
  } else if (N_ == 5) {
    computeDependentCoefficientsN5();
  } else if (N_ == 6) {
    computeDependentCoefficientsN6();
  } else {
    std::cout << "N should be 6" << std::endl;
  }
}

void SolverGurobi::getInitialAndFinalConditions(
    double& P0, double& V0, double& A0, double& Pf, double& Vf, double& Af, int axis) {
  // Get initial and final conditions
  P0 = x0_[axis];
  V0 = x0_[axis + 3];
  A0 = x0_[axis + 6];
  Pf = xf_[axis];
  Vf = xf_[axis + 3];
  Af = xf_[axis + 6];
}

void SolverGurobi::computeDependentCoefficientsN4() {
  // Clear x_
  x_.clear();

  // Get time allocations
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];

  // Loop thru each axis
  for (int axis = 0; axis < 3; axis++) {
    // Get initial and final conditions
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // Get free parameter for the cubic Bézier control coefficients
    GRBVar d3_var = d3_[axis];

    // C++ expressions for composite cubic Bézier control coefficients (one coordinate) in terms of
    // d3:
    GRBLinExpr a0 =
        d3_var * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) /
            (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) +
             T0 * T1 * T2 * pow(T3, 2)) +
        (-3 * A0 * pow(T0, 2) * pow(T3, 2) - 4 * A0 * T0 * T1 * pow(T3, 2) -
         2 * A0 * T0 * T2 * pow(T3, 2) - A0 * pow(T1, 2) * pow(T3, 2) - A0 * T1 * T2 * pow(T3, 2) -
         2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) -
         2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 -
         6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 +
         6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf +
         6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf) /
            (6 * pow(T0, 3) * pow(T3, 2) + 12 * pow(T0, 2) * T1 * pow(T3, 2) +
             6 * pow(T0, 2) * T2 * pow(T3, 2) + 6 * T0 * pow(T1, 2) * pow(T3, 2) +
             6 * T0 * T1 * T2 * pow(T3, 2));
    GRBLinExpr b0 = (1.0 / 2.0) * A0;
    GRBLinExpr c0 = V0;
    GRBLinExpr d0 = P0;
    GRBLinExpr a1 =
        d3_var *
            (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 -
             2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 -
             3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) -
             pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) +
        (A0 * pow(T0, 3) * pow(T3, 2) + 4 * A0 * pow(T0, 2) * T1 * pow(T3, 2) +
         2 * A0 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * A0 * T0 * pow(T1, 2) * pow(T3, 2) +
         3 * A0 * T0 * T1 * T2 * pow(T3, 2) + A0 * T0 * pow(T2, 2) * pow(T3, 2) +
         2 * Af * pow(T0, 2) * T2 * pow(T3, 2) + Af * pow(T0, 2) * pow(T3, 3) +
         6 * Af * T0 * T1 * T2 * pow(T3, 2) + 3 * Af * T0 * T1 * pow(T3, 3) +
         4 * Af * T0 * pow(T2, 2) * pow(T3, 2) + 3 * Af * T0 * T2 * pow(T3, 3) +
         6 * Af * pow(T1, 2) * T2 * pow(T3, 2) + 3 * Af * pow(T1, 2) * pow(T3, 3) +
         8 * Af * T1 * pow(T2, 2) * pow(T3, 2) + 6 * Af * T1 * T2 * pow(T3, 3) +
         2 * Af * pow(T2, 3) * pow(T3, 2) + 2 * Af * pow(T2, 2) * pow(T3, 3) +
         6 * P0 * T0 * pow(T3, 2) + 12 * P0 * T1 * pow(T3, 2) + 6 * P0 * T2 * pow(T3, 2) +
         6 * Pf * pow(T0, 2) * T2 + 6 * Pf * pow(T0, 2) * T3 + 18 * Pf * T0 * T1 * T2 +
         18 * Pf * T0 * T1 * T3 + 12 * Pf * T0 * pow(T2, 2) + 18 * Pf * T0 * T2 * T3 +
         18 * Pf * pow(T1, 2) * T2 + 18 * Pf * pow(T1, 2) * T3 + 24 * Pf * T1 * pow(T2, 2) +
         36 * Pf * T1 * T2 * T3 + 6 * Pf * pow(T2, 3) + 12 * Pf * pow(T2, 2) * T3 -
         6 * pow(T0, 2) * T2 * T3 * Vf + 4 * pow(T0, 2) * pow(T3, 2) * V0 -
         4 * pow(T0, 2) * pow(T3, 2) * Vf - 18 * T0 * T1 * T2 * T3 * Vf +
         12 * T0 * T1 * pow(T3, 2) * V0 - 12 * T0 * T1 * pow(T3, 2) * Vf -
         12 * T0 * pow(T2, 2) * T3 * Vf + 6 * T0 * T2 * pow(T3, 2) * V0 -
         12 * T0 * T2 * pow(T3, 2) * Vf - 18 * pow(T1, 2) * T2 * T3 * Vf +
         6 * pow(T1, 2) * pow(T3, 2) * V0 - 12 * pow(T1, 2) * pow(T3, 2) * Vf -
         24 * T1 * pow(T2, 2) * T3 * Vf + 6 * T1 * T2 * pow(T3, 2) * V0 -
         24 * T1 * T2 * pow(T3, 2) * Vf - 6 * pow(T2, 3) * T3 * Vf +
         2 * pow(T2, 2) * pow(T3, 2) * V0 - 8 * pow(T2, 2) * pow(T3, 2) * Vf) /
            (6 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 6 * pow(T0, 2) * T1 * T2 * pow(T3, 2) +
             12 * T0 * pow(T1, 3) * pow(T3, 2) + 18 * T0 * pow(T1, 2) * T2 * pow(T3, 2) +
             6 * T0 * T1 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T1, 4) * pow(T3, 2) +
             12 * pow(T1, 3) * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2));
    GRBLinExpr b1 =
        d3_var * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        (-2 * A0 * pow(T0, 2) * pow(T3, 2) - 2 * A0 * T0 * T1 * pow(T3, 2) -
         A0 * T0 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) -
         2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) -
         6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 -
         6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 +
         4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 +
         8 * T2 * pow(T3, 2) * Vf) /
            (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2));
    GRBLinExpr c1 =
        d3_var *
            (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 +
             3 * T0 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        (-A0 * pow(T0, 3) * pow(T3, 2) + A0 * T0 * pow(T1, 2) * pow(T3, 2) +
         A0 * T0 * T1 * T2 * pow(T3, 2) - 2 * Af * T0 * T1 * T2 * pow(T3, 2) -
         Af * T0 * T1 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 2) -
         2 * Af * T0 * T2 * pow(T3, 3) - 6 * P0 * T0 * pow(T3, 2) - 6 * Pf * T0 * T1 * T2 -
         6 * Pf * T0 * T1 * T3 - 6 * Pf * T0 * pow(T2, 2) - 12 * Pf * T0 * T2 * T3 -
         4 * pow(T0, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * T3 * Vf +
         4 * T0 * T1 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 2) * T3 * Vf +
         8 * T0 * T2 * pow(T3, 2) * Vf + 2 * pow(T1, 2) * pow(T3, 2) * V0 +
         2 * T1 * T2 * pow(T3, 2) * V0) /
            (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2));
    GRBLinExpr d1 =
        d3_var *
            (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) +
             2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        (2 * A0 * pow(T0, 3) * T1 * pow(T3, 2) + A0 * pow(T0, 3) * T2 * pow(T3, 2) +
         2 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) +
         2 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 2 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) -
         Af * pow(T0, 2) * T1 * pow(T3, 3) - 2 * Af * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) -
         2 * Af * pow(T0, 2) * T2 * pow(T3, 3) + 12 * P0 * T0 * T1 * pow(T3, 2) +
         6 * P0 * T0 * T2 * pow(T3, 2) + 6 * P0 * pow(T1, 2) * pow(T3, 2) +
         6 * P0 * T1 * T2 * pow(T3, 2) - 6 * Pf * pow(T0, 2) * T1 * T2 -
         6 * Pf * pow(T0, 2) * T1 * T3 - 6 * Pf * pow(T0, 2) * pow(T2, 2) -
         12 * Pf * pow(T0, 2) * T2 * T3 + 6 * pow(T0, 2) * T1 * T2 * T3 * Vf +
         8 * pow(T0, 2) * T1 * pow(T3, 2) * V0 + 4 * pow(T0, 2) * T1 * pow(T3, 2) * Vf +
         6 * pow(T0, 2) * pow(T2, 2) * T3 * Vf + 4 * pow(T0, 2) * T2 * pow(T3, 2) * V0 +
         8 * pow(T0, 2) * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T1, 2) * pow(T3, 2) * V0 +
         6 * T0 * T1 * T2 * pow(T3, 2) * V0) /
            (6 * pow(T0, 2) * pow(T3, 2) + 12 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) +
             6 * pow(T1, 2) * pow(T3, 2) + 6 * T1 * T2 * pow(T3, 2));
    GRBLinExpr a2 =
        d3_var *
            (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 +
             3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) +
             pow(T2, 3) * pow(T3, 2)) +
        (-A0 * pow(T0, 2) * pow(T3, 2) - A0 * T0 * T1 * pow(T3, 2) - 2 * Af * T0 * T1 * pow(T3, 2) -
         4 * Af * T0 * T2 * pow(T3, 2) - Af * T0 * pow(T3, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) -
         8 * Af * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * pow(T3, 3) -
         6 * Af * pow(T2, 2) * pow(T3, 2) - 3 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) -
         6 * Pf * T0 * T1 - 12 * Pf * T0 * T2 - 6 * Pf * T0 * T3 - 6 * Pf * pow(T1, 2) -
         24 * Pf * T1 * T2 - 12 * Pf * T1 * T3 - 18 * Pf * pow(T2, 2) - 18 * Pf * T2 * T3 +
         6 * T0 * T1 * T3 * Vf + 12 * T0 * T2 * T3 * Vf - 4 * T0 * pow(T3, 2) * V0 +
         4 * T0 * pow(T3, 2) * Vf + 6 * pow(T1, 2) * T3 * Vf + 24 * T1 * T2 * T3 * Vf -
         2 * T1 * pow(T3, 2) * V0 + 8 * T1 * pow(T3, 2) * Vf + 18 * pow(T2, 2) * T3 * Vf +
         12 * T2 * pow(T3, 2) * Vf) /
            (6 * T0 * T1 * T2 * pow(T3, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) +
             6 * pow(T2, 3) * pow(T3, 2));
    GRBLinExpr b2 =
        d3_var *
            (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 -
             3 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        (A0 * pow(T0, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T3, 2) + 2 * Af * T0 * T2 * pow(T3, 2) +
         Af * T0 * pow(T3, 3) + 4 * Af * T1 * T2 * pow(T3, 2) + 2 * Af * T1 * pow(T3, 3) +
         4 * Af * pow(T2, 2) * pow(T3, 2) + 3 * Af * T2 * pow(T3, 3) + 6 * P0 * pow(T3, 2) +
         6 * Pf * T0 * T2 + 6 * Pf * T0 * T3 + 12 * Pf * T1 * T2 + 12 * Pf * T1 * T3 +
         12 * Pf * pow(T2, 2) + 18 * Pf * T2 * T3 - 6 * T0 * T2 * T3 * Vf +
         4 * T0 * pow(T3, 2) * V0 - 4 * T0 * pow(T3, 2) * Vf - 12 * T1 * T2 * T3 * Vf +
         2 * T1 * pow(T3, 2) * V0 - 8 * T1 * pow(T3, 2) * Vf - 12 * pow(T2, 2) * T3 * Vf -
         12 * T2 * pow(T3, 2) * Vf) /
            (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) +
             4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2));
    GRBLinExpr c2 =
        d3_var *
            (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 +
             3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        (-A0 * pow(T0, 2) * T2 * pow(T3, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) +
         2 * Af * T0 * T1 * T2 * pow(T3, 2) + Af * T0 * T1 * pow(T3, 3) +
         2 * Af * pow(T1, 2) * T2 * pow(T3, 2) + Af * pow(T1, 2) * pow(T3, 3) -
         2 * Af * pow(T2, 3) * pow(T3, 2) - 2 * Af * pow(T2, 2) * pow(T3, 3) -
         6 * P0 * T2 * pow(T3, 2) + 6 * Pf * T0 * T1 * T2 + 6 * Pf * T0 * T1 * T3 +
         6 * Pf * pow(T1, 2) * T2 + 6 * Pf * pow(T1, 2) * T3 - 6 * Pf * pow(T2, 3) -
         12 * Pf * pow(T2, 2) * T3 - 6 * T0 * T1 * T2 * T3 * Vf - 4 * T0 * T1 * pow(T3, 2) * Vf -
         4 * T0 * T2 * pow(T3, 2) * V0 - 6 * pow(T1, 2) * T2 * T3 * Vf -
         4 * pow(T1, 2) * pow(T3, 2) * Vf - 2 * T1 * T2 * pow(T3, 2) * V0 +
         6 * pow(T2, 3) * T3 * Vf + 8 * pow(T2, 2) * pow(T3, 2) * Vf) /
            (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) +
             4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2));
    GRBLinExpr d2 =
        d3_var *
            (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) +
             T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) -
         4 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 3 * Af * T0 * T1 * T2 * pow(T3, 3) -
         2 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 3) -
         4 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T3, 3) -
         4 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 4 * Af * T1 * pow(T2, 2) * pow(T3, 3) +
         6 * P0 * pow(T2, 2) * pow(T3, 2) - 12 * Pf * T0 * T1 * pow(T2, 2) -
         18 * Pf * T0 * T1 * T2 * T3 - 6 * Pf * T0 * pow(T2, 3) - 12 * Pf * T0 * pow(T2, 2) * T3 -
         12 * Pf * pow(T1, 2) * pow(T2, 2) - 18 * Pf * pow(T1, 2) * T2 * T3 -
         12 * Pf * T1 * pow(T2, 3) - 24 * Pf * T1 * pow(T2, 2) * T3 +
         12 * T0 * T1 * pow(T2, 2) * T3 * Vf + 12 * T0 * T1 * T2 * pow(T3, 2) * Vf +
         6 * T0 * pow(T2, 3) * T3 * Vf + 4 * T0 * pow(T2, 2) * pow(T3, 2) * V0 +
         8 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 12 * pow(T1, 2) * pow(T2, 2) * T3 * Vf +
         12 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 12 * T1 * pow(T2, 3) * T3 * Vf +
         2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 16 * T1 * pow(T2, 2) * pow(T3, 2) * Vf) /
            (6 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) +
             12 * T1 * T2 * pow(T3, 2) + 6 * pow(T2, 2) * pow(T3, 2));
    GRBLinExpr a3 =
        -d3_var / pow(T3, 3) + (1.0 / 2.0) * (Af * pow(T3, 2) + 2 * Pf - 2 * T3 * Vf) / pow(T3, 3);
    GRBLinExpr b3 =
        3 * d3_var / pow(T3, 2) + (-Af * pow(T3, 2) - 3 * Pf + 3 * T3 * Vf) / pow(T3, 2);
    GRBLinExpr c3 = -3 * d3_var / T3 + (1.0 / 2.0) * (Af * pow(T3, 2) + 6 * Pf - 4 * T3 * Vf) / T3;
    GRBLinExpr d3 = d3_var;

    // Fill x_ with the coefficients
    x_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3});
  }
}

void SolverGurobi::computeDependentCoefficientsN5() {
  // Clear x_
  x_.clear();

  // Get time allocations
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];
  double T4 = dt_[4];

  // Loop thru each axis
  for (int axis = 0; axis < 3; axis++) {
    // Get initial and final conditions
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // Get free parameter for the cubic Bézier control coefficients
    GRBVar d3_var = d3_[axis];
    GRBVar d4_var = d4_[axis];

    // C++ expressions for composite cubic Bézier control coefficients
    GRBLinExpr a0 =
        d3_var * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) /
            (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) +
             T0 * T1 * T2 * pow(T3, 2)) +
        d4_var *
            (-2 * T1 * T2 * pow(T3, 2) - 3 * T1 * T2 * T3 * T4 - T1 * T2 * pow(T4, 2) -
             T1 * pow(T3, 3) - 2 * T1 * pow(T3, 2) * T4 - T1 * T3 * pow(T4, 2) -
             2 * pow(T2, 2) * pow(T3, 2) - 3 * pow(T2, 2) * T3 * T4 - pow(T2, 2) * pow(T4, 2) -
             2 * T2 * pow(T3, 3) - 4 * T2 * pow(T3, 2) * T4 - 2 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 3) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        (-3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 4 * A0 * T0 * T1 * T3 * pow(T4, 2) -
         2 * A0 * T0 * T2 * T3 * pow(T4, 2) - A0 * pow(T1, 2) * T3 * pow(T4, 2) -
         A0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) +
         3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) +
         2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) +
         3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) +
         4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 +
         18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 +
         12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) +
         24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf -
         12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf -
         4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf -
         12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf -
         12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 -
         16 * T2 * T3 * pow(T4, 2) * Vf) /
            (6 * pow(T0, 3) * T3 * pow(T4, 2) + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) +
             6 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) +
             6 * T0 * T1 * T2 * T3 * pow(T4, 2));
    GRBLinExpr b0 = (1.0 / 2.0) * A0;
    GRBLinExpr c0 = V0;
    GRBLinExpr d0 = P0;
    GRBLinExpr a1 =
        d3_var *
            (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 -
             2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 -
             3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) -
             pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) +
        d4_var *
            (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 +
             pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) +
             2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) +
             6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 +
             3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) +
             6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) +
             4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 +
             2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) +
             6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 +
             3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) +
             6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) +
             8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 +
             4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) +
             12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) +
             2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) +
             2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 +
             2 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) +
             3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 4) * pow(T3, 2) * pow(T4, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        (A0 * pow(T0, 3) * T3 * pow(T4, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * pow(T4, 2) +
         2 * A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 3 * A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) +
         3 * A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + A0 * T0 * pow(T2, 2) * T3 * pow(T4, 2) -
         4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T0, 2) * T2 * pow(T4, 3) -
         2 * Af * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T0, 2) * T3 * pow(T4, 3) -
         12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 9 * Af * T0 * T1 * T2 * pow(T4, 3) -
         6 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T1 * T3 * pow(T4, 3) -
         8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 3) -
         6 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T2 * T3 * pow(T4, 3) -
         12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T4, 3) -
         6 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T4, 3) -
         16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T4, 3) -
         12 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 12 * Af * T1 * T2 * T3 * pow(T4, 3) -
         4 * Af * pow(T2, 3) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 3) * pow(T4, 3) -
         4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) +
         6 * P0 * T0 * T3 * pow(T4, 2) + 12 * P0 * T1 * T3 * pow(T4, 2) +
         6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 -
         18 * Pf * pow(T0, 2) * T2 * T4 - 6 * Pf * pow(T0, 2) * pow(T3, 2) -
         12 * Pf * pow(T0, 2) * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 -
         54 * Pf * T0 * T1 * T2 * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) -
         36 * Pf * T0 * T1 * T3 * T4 - 24 * Pf * T0 * pow(T2, 2) * T3 -
         36 * Pf * T0 * pow(T2, 2) * T4 - 18 * Pf * T0 * T2 * pow(T3, 2) -
         36 * Pf * T0 * T2 * T3 * T4 - 36 * Pf * pow(T1, 2) * T2 * T3 -
         54 * Pf * pow(T1, 2) * T2 * T4 - 18 * Pf * pow(T1, 2) * pow(T3, 2) -
         36 * Pf * pow(T1, 2) * T3 * T4 - 48 * Pf * T1 * pow(T2, 2) * T3 -
         72 * Pf * T1 * pow(T2, 2) * T4 - 36 * Pf * T1 * T2 * pow(T3, 2) -
         72 * Pf * T1 * T2 * T3 * T4 - 12 * Pf * pow(T2, 3) * T3 - 18 * Pf * pow(T2, 3) * T4 -
         12 * Pf * pow(T2, 2) * pow(T3, 2) - 24 * Pf * pow(T2, 2) * T3 * T4 +
         12 * pow(T0, 2) * T2 * T3 * T4 * Vf + 12 * pow(T0, 2) * T2 * pow(T4, 2) * Vf +
         6 * pow(T0, 2) * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 +
         8 * pow(T0, 2) * T3 * pow(T4, 2) * Vf + 36 * T0 * T1 * T2 * T3 * T4 * Vf +
         36 * T0 * T1 * T2 * pow(T4, 2) * Vf + 18 * T0 * T1 * pow(T3, 2) * T4 * Vf +
         12 * T0 * T1 * T3 * pow(T4, 2) * V0 + 24 * T0 * T1 * T3 * pow(T4, 2) * Vf +
         24 * T0 * pow(T2, 2) * T3 * T4 * Vf + 24 * T0 * pow(T2, 2) * pow(T4, 2) * Vf +
         18 * T0 * T2 * pow(T3, 2) * T4 * Vf + 6 * T0 * T2 * T3 * pow(T4, 2) * V0 +
         24 * T0 * T2 * T3 * pow(T4, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * T4 * Vf +
         36 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 18 * pow(T1, 2) * pow(T3, 2) * T4 * Vf +
         6 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 24 * pow(T1, 2) * T3 * pow(T4, 2) * Vf +
         48 * T1 * pow(T2, 2) * T3 * T4 * Vf + 48 * T1 * pow(T2, 2) * pow(T4, 2) * Vf +
         36 * T1 * T2 * pow(T3, 2) * T4 * Vf + 6 * T1 * T2 * T3 * pow(T4, 2) * V0 +
         48 * T1 * T2 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 3) * T3 * T4 * Vf +
         12 * pow(T2, 3) * pow(T4, 2) * Vf + 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf +
         2 * pow(T2, 2) * T3 * pow(T4, 2) * V0 + 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) /
            (6 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) +
             6 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 12 * T0 * pow(T1, 3) * T3 * pow(T4, 2) +
             18 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) +
             6 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 4) * T3 * pow(T4, 2) +
             12 * pow(T1, 3) * T2 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2));
    GRBLinExpr b1 =
        d3_var * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_var *
            (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) -
             3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) -
             6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        (-2 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 2 * A0 * T0 * T1 * T3 * pow(T4, 2) -
         A0 * T0 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) +
         3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) +
         2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) +
         3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) +
         4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 +
         18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 +
         12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) +
         24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf -
         12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf -
         4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf -
         12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf -
         12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 -
         16 * T2 * T3 * pow(T4, 2) * Vf) /
            (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) +
             2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2));
    GRBLinExpr c1 =
        d3_var *
            (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 +
             3 * T0 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_var *
            (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 -
             3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) -
             6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) -
             6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 -
             3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) -
             12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        (-A0 * pow(T0, 3) * T3 * pow(T4, 2) + A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) +
         A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) +
         3 * Af * T0 * T1 * T2 * pow(T4, 3) + 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
         2 * Af * T0 * T1 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) +
         3 * Af * T0 * pow(T2, 2) * pow(T4, 3) + 4 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) +
         4 * Af * T0 * T2 * T3 * pow(T4, 3) - 6 * P0 * T0 * T3 * pow(T4, 2) +
         12 * Pf * T0 * T1 * T2 * T3 + 18 * Pf * T0 * T1 * T2 * T4 + 6 * Pf * T0 * T1 * pow(T3, 2) +
         12 * Pf * T0 * T1 * T3 * T4 + 12 * Pf * T0 * pow(T2, 2) * T3 +
         18 * Pf * T0 * pow(T2, 2) * T4 + 12 * Pf * T0 * T2 * pow(T3, 2) +
         24 * Pf * T0 * T2 * T3 * T4 - 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 -
         12 * T0 * T1 * T2 * T3 * T4 * Vf - 12 * T0 * T1 * T2 * pow(T4, 2) * Vf -
         6 * T0 * T1 * pow(T3, 2) * T4 * Vf - 8 * T0 * T1 * T3 * pow(T4, 2) * Vf -
         12 * T0 * pow(T2, 2) * T3 * T4 * Vf - 12 * T0 * pow(T2, 2) * pow(T4, 2) * Vf -
         12 * T0 * T2 * pow(T3, 2) * T4 * Vf - 16 * T0 * T2 * T3 * pow(T4, 2) * Vf +
         2 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 2 * T1 * T2 * T3 * pow(T4, 2) * V0) /
            (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) +
             2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2));
    GRBLinExpr d1 =
        d3_var *
            (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) +
             2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_var *
            (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 -
             pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) -
             2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) -
             2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 -
             pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) -
             4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        (2 * A0 * pow(T0, 3) * T1 * T3 * pow(T4, 2) + A0 * pow(T0, 3) * T2 * T3 * pow(T4, 2) +
         2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) +
         2 * A0 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) +
         4 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) +
         3 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 3) +
         2 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) +
         2 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 3) +
         4 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
         3 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) +
         4 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
         4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 12 * P0 * T0 * T1 * T3 * pow(T4, 2) +
         6 * P0 * T0 * T2 * T3 * pow(T4, 2) + 6 * P0 * pow(T1, 2) * T3 * pow(T4, 2) +
         6 * P0 * T1 * T2 * T3 * pow(T4, 2) + 12 * Pf * pow(T0, 2) * T1 * T2 * T3 +
         18 * Pf * pow(T0, 2) * T1 * T2 * T4 + 6 * Pf * pow(T0, 2) * T1 * pow(T3, 2) +
         12 * Pf * pow(T0, 2) * T1 * T3 * T4 + 12 * Pf * pow(T0, 2) * pow(T2, 2) * T3 +
         18 * Pf * pow(T0, 2) * pow(T2, 2) * T4 + 12 * Pf * pow(T0, 2) * T2 * pow(T3, 2) +
         24 * Pf * pow(T0, 2) * T2 * T3 * T4 - 12 * pow(T0, 2) * T1 * T2 * T3 * T4 * Vf -
         12 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * Vf - 6 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * Vf +
         8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * V0 - 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * Vf -
         12 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * Vf -
         12 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * Vf -
         12 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * V0 -
         16 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * V0 +
         6 * T0 * T1 * T2 * T3 * pow(T4, 2) * V0) /
            (6 * pow(T0, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * T3 * pow(T4, 2) +
             6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) +
             6 * T1 * T2 * T3 * pow(T4, 2));
    GRBLinExpr a2 =
        d3_var *
            (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 +
             3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) +
             pow(T2, 3) * pow(T3, 2)) +
        d4_var *
            (-2 * T0 * T1 * pow(T3, 2) - 3 * T0 * T1 * T3 * T4 - T0 * T1 * pow(T4, 2) -
             4 * T0 * T2 * pow(T3, 2) - 6 * T0 * T2 * T3 * T4 - 2 * T0 * T2 * pow(T4, 2) -
             T0 * pow(T3, 3) - 2 * T0 * pow(T3, 2) * T4 - T0 * T3 * pow(T4, 2) -
             2 * pow(T1, 2) * pow(T3, 2) - 3 * pow(T1, 2) * T3 * T4 - pow(T1, 2) * pow(T4, 2) -
             8 * T1 * T2 * pow(T3, 2) - 12 * T1 * T2 * T3 * T4 - 4 * T1 * T2 * pow(T4, 2) -
             2 * T1 * pow(T3, 3) - 4 * T1 * pow(T3, 2) * T4 - 2 * T1 * T3 * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) -
             3 * T2 * pow(T3, 3) - 6 * T2 * pow(T3, 2) * T4 - 3 * T2 * T3 * pow(T4, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T2, 3) * pow(T3, 2) * pow(T4, 2)) +
        (-A0 * pow(T0, 2) * T3 * pow(T4, 2) - A0 * T0 * T1 * T3 * pow(T4, 2) +
         4 * Af * T0 * T1 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * pow(T4, 3) +
         8 * Af * T0 * T2 * T3 * pow(T4, 2) + 6 * Af * T0 * T2 * pow(T4, 3) +
         2 * Af * T0 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T3 * pow(T4, 3) +
         4 * Af * pow(T1, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T1, 2) * pow(T4, 3) +
         16 * Af * T1 * T2 * T3 * pow(T4, 2) + 12 * Af * T1 * T2 * pow(T4, 3) +
         4 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T1 * T3 * pow(T4, 3) +
         12 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 9 * Af * pow(T2, 2) * pow(T4, 3) +
         6 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T2 * T3 * pow(T4, 3) -
         6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T3 + 18 * Pf * T0 * T1 * T4 +
         24 * Pf * T0 * T2 * T3 + 36 * Pf * T0 * T2 * T4 + 6 * Pf * T0 * pow(T3, 2) +
         12 * Pf * T0 * T3 * T4 + 12 * Pf * pow(T1, 2) * T3 + 18 * Pf * pow(T1, 2) * T4 +
         48 * Pf * T1 * T2 * T3 + 72 * Pf * T1 * T2 * T4 + 12 * Pf * T1 * pow(T3, 2) +
         24 * Pf * T1 * T3 * T4 + 36 * Pf * pow(T2, 2) * T3 + 54 * Pf * pow(T2, 2) * T4 +
         18 * Pf * T2 * pow(T3, 2) + 36 * Pf * T2 * T3 * T4 - 12 * T0 * T1 * T3 * T4 * Vf -
         12 * T0 * T1 * pow(T4, 2) * Vf - 24 * T0 * T2 * T3 * T4 * Vf -
         24 * T0 * T2 * pow(T4, 2) * Vf - 6 * T0 * pow(T3, 2) * T4 * Vf -
         4 * T0 * T3 * pow(T4, 2) * V0 - 8 * T0 * T3 * pow(T4, 2) * Vf -
         12 * pow(T1, 2) * T3 * T4 * Vf - 12 * pow(T1, 2) * pow(T4, 2) * Vf -
         48 * T1 * T2 * T3 * T4 * Vf - 48 * T1 * T2 * pow(T4, 2) * Vf -
         12 * T1 * pow(T3, 2) * T4 * Vf - 2 * T1 * T3 * pow(T4, 2) * V0 -
         16 * T1 * T3 * pow(T4, 2) * Vf - 36 * pow(T2, 2) * T3 * T4 * Vf -
         36 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T2 * pow(T3, 2) * T4 * Vf -
         24 * T2 * T3 * pow(T4, 2) * Vf) /
            (6 * T0 * T1 * T2 * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 2) * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 2) * T3 * pow(T4, 2) +
             6 * pow(T2, 3) * T3 * pow(T4, 2));
    GRBLinExpr b2 =
        d3_var *
            (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 -
             3 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_var *
            (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) +
             3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) +
             12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) +
             6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) +
             12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 +
             6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 +
             9 * T2 * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        (A0 * pow(T0, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T3 * pow(T4, 2) -
         4 * Af * T0 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T2 * pow(T4, 3) -
         2 * Af * T0 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T3 * pow(T4, 3) -
         8 * Af * T1 * T2 * T3 * pow(T4, 2) - 6 * Af * T1 * T2 * pow(T4, 3) -
         4 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T1 * T3 * pow(T4, 3) -
         8 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * pow(T2, 2) * pow(T4, 3) -
         6 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T2 * T3 * pow(T4, 3) +
         6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T0 * T2 * T3 - 18 * Pf * T0 * T2 * T4 -
         6 * Pf * T0 * pow(T3, 2) - 12 * Pf * T0 * T3 * T4 - 24 * Pf * T1 * T2 * T3 -
         36 * Pf * T1 * T2 * T4 - 12 * Pf * T1 * pow(T3, 2) - 24 * Pf * T1 * T3 * T4 -
         24 * Pf * pow(T2, 2) * T3 - 36 * Pf * pow(T2, 2) * T4 - 18 * Pf * T2 * pow(T3, 2) -
         36 * Pf * T2 * T3 * T4 + 12 * T0 * T2 * T3 * T4 * Vf + 12 * T0 * T2 * pow(T4, 2) * Vf +
         6 * T0 * pow(T3, 2) * T4 * Vf + 4 * T0 * T3 * pow(T4, 2) * V0 +
         8 * T0 * T3 * pow(T4, 2) * Vf + 24 * T1 * T2 * T3 * T4 * Vf +
         24 * T1 * T2 * pow(T4, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * Vf +
         2 * T1 * T3 * pow(T4, 2) * V0 + 16 * T1 * T3 * pow(T4, 2) * Vf +
         24 * pow(T2, 2) * T3 * T4 * Vf + 24 * pow(T2, 2) * pow(T4, 2) * Vf +
         18 * T2 * pow(T3, 2) * T4 * Vf + 24 * T2 * T3 * pow(T4, 2) * Vf) /
            (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) +
             2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) +
             2 * pow(T2, 2) * T3 * pow(T4, 2));
    GRBLinExpr c2 =
        d3_var *
            (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 +
             3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_var *
            (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 +
             3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) +
             6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 +
             3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) +
             6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) -
             6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 -
             6 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        (-A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - A0 * T0 * T1 * T2 * T3 * pow(T4, 2) -
         4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T1 * T2 * pow(T4, 3) -
         2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T1 * T3 * pow(T4, 3) -
         4 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T4, 3) -
         2 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T1, 2) * T3 * pow(T4, 3) +
         4 * Af * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 3) * pow(T4, 3) +
         4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) -
         6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * T0 * T1 * T2 * T3 - 18 * Pf * T0 * T1 * T2 * T4 -
         6 * Pf * T0 * T1 * pow(T3, 2) - 12 * Pf * T0 * T1 * T3 * T4 -
         12 * Pf * pow(T1, 2) * T2 * T3 - 18 * Pf * pow(T1, 2) * T2 * T4 -
         6 * Pf * pow(T1, 2) * pow(T3, 2) - 12 * Pf * pow(T1, 2) * T3 * T4 +
         12 * Pf * pow(T2, 3) * T3 + 18 * Pf * pow(T2, 3) * T4 + 12 * Pf * pow(T2, 2) * pow(T3, 2) +
         24 * Pf * pow(T2, 2) * T3 * T4 + 12 * T0 * T1 * T2 * T3 * T4 * Vf +
         12 * T0 * T1 * T2 * pow(T4, 2) * Vf + 6 * T0 * T1 * pow(T3, 2) * T4 * Vf +
         8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 4 * T0 * T2 * T3 * pow(T4, 2) * V0 +
         12 * pow(T1, 2) * T2 * T3 * T4 * Vf + 12 * pow(T1, 2) * T2 * pow(T4, 2) * Vf +
         6 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 8 * pow(T1, 2) * T3 * pow(T4, 2) * Vf -
         2 * T1 * T2 * T3 * pow(T4, 2) * V0 - 12 * pow(T2, 3) * T3 * T4 * Vf -
         12 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf -
         16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) /
            (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) +
             2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) +
             2 * pow(T2, 2) * T3 * pow(T4, 2));
    GRBLinExpr d2 =
        d3_var *
            (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) +
             T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_var *
            (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 -
             2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) -
             6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) -
             2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 -
             T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) -
             4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) -
             4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 -
             2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) -
             6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) -
             4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 -
             2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) -
             8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        (A0 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
         A0 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) +
         8 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) +
         6 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 3) +
         6 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) +
         4 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 3) * pow(T4, 3) +
         4 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
         4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) +
         8 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
         6 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) +
         6 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
         6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 8 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) +
         6 * Af * T1 * pow(T2, 3) * pow(T4, 3) +
         8 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
         8 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * pow(T2, 2) * T3 * pow(T4, 2) +
         24 * Pf * T0 * T1 * pow(T2, 2) * T3 + 36 * Pf * T0 * T1 * pow(T2, 2) * T4 +
         18 * Pf * T0 * T1 * T2 * pow(T3, 2) + 36 * Pf * T0 * T1 * T2 * T3 * T4 +
         12 * Pf * T0 * pow(T2, 3) * T3 + 18 * Pf * T0 * pow(T2, 3) * T4 +
         12 * Pf * T0 * pow(T2, 2) * pow(T3, 2) + 24 * Pf * T0 * pow(T2, 2) * T3 * T4 +
         24 * Pf * pow(T1, 2) * pow(T2, 2) * T3 + 36 * Pf * pow(T1, 2) * pow(T2, 2) * T4 +
         18 * Pf * pow(T1, 2) * T2 * pow(T3, 2) + 36 * Pf * pow(T1, 2) * T2 * T3 * T4 +
         24 * Pf * T1 * pow(T2, 3) * T3 + 36 * Pf * T1 * pow(T2, 3) * T4 +
         24 * Pf * T1 * pow(T2, 2) * pow(T3, 2) + 48 * Pf * T1 * pow(T2, 2) * T3 * T4 -
         24 * T0 * T1 * pow(T2, 2) * T3 * T4 * Vf - 24 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * Vf -
         18 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf -
         12 * T0 * pow(T2, 3) * T3 * T4 * Vf - 12 * T0 * pow(T2, 3) * pow(T4, 2) * Vf -
         12 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * V0 -
         16 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 24 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * Vf -
         24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * Vf -
         18 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf -
         24 * T1 * pow(T2, 3) * T3 * T4 * Vf - 24 * T1 * pow(T2, 3) * pow(T4, 2) * Vf -
         24 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * V0 -
         32 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) /
            (6 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T3 * pow(T4, 2) + 12 * T1 * T2 * T3 * pow(T4, 2) +
             6 * pow(T2, 2) * T3 * pow(T4, 2));
    GRBLinExpr a3 =
        (1.0 / 2.0) *
            (-2 * Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 6 * Pf * T3 - 6 * Pf * T4 +
             6 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) /
            (pow(T3, 2) * pow(T4, 2)) -
        d3_var / pow(T3, 3) +
        d4_var * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2)) / (pow(T3, 3) * pow(T4, 2));
    GRBLinExpr b3 =
        (1.0 / 2.0) *
            (4 * Af * T3 * pow(T4, 2) + 3 * Af * pow(T4, 3) + 12 * Pf * T3 + 18 * Pf * T4 -
             12 * T3 * T4 * Vf - 12 * pow(T4, 2) * Vf) /
            (T3 * pow(T4, 2)) +
        3 * d3_var / pow(T3, 2) +
        d4_var * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2));
    GRBLinExpr c3 = (-Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 3 * Pf * T3 - 6 * Pf * T4 +
                     3 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) /
                        pow(T4, 2) -
                    3 * d3_var / T3 +
                    d4_var * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2));
    GRBLinExpr d3 = d3_var;
    GRBLinExpr a4 =
        -d4_var / pow(T4, 3) + (1.0 / 2.0) * (Af * pow(T4, 2) + 2 * Pf - 2 * T4 * Vf) / pow(T4, 3);
    GRBLinExpr b4 =
        3 * d4_var / pow(T4, 2) + (-Af * pow(T4, 2) - 3 * Pf + 3 * T4 * Vf) / pow(T4, 2);
    GRBLinExpr c4 = -3 * d4_var / T4 + (1.0 / 2.0) * (Af * pow(T4, 2) + 6 * Pf - 4 * T4 * Vf) / T4;
    GRBLinExpr d4 = d4_var;

    // Fill x_ with the coefficients
    x_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3, a4, b4, c4, d4});
  }
}

void SolverGurobi::computeDependentCoefficientsN6() {
  // Clear x_
  x_.clear();

  // Get time allocations
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];
  double T4 = dt_[4];
  double T5 = dt_[5];

  // Loop thru each axis
  for (int axis = 0; axis < 3; axis++) {
    // Get initial and final conditions
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // Get free parameter for the cubic Bézier control coefficients
    GRBVar d3_var = d3_[axis];
    GRBVar d4_var = d4_[axis];
    GRBVar d5_var = d5_[axis];

    // C++ expressions for composite cubic Bézier control coefficients
    GRBLinExpr a0 =
        d3_var * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) /
            (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) +
             T0 * T1 * T2 * pow(T3, 2)) +
        d4_var *
            (-2 * T1 * T2 * pow(T3, 2) - 3 * T1 * T2 * T3 * T4 - T1 * T2 * pow(T4, 2) -
             T1 * pow(T3, 3) - 2 * T1 * pow(T3, 2) * T4 - T1 * T3 * pow(T4, 2) -
             2 * pow(T2, 2) * pow(T3, 2) - 3 * pow(T2, 2) * T3 * T4 - pow(T2, 2) * pow(T4, 2) -
             2 * T2 * pow(T3, 3) - 4 * T2 * pow(T3, 2) * T4 - 2 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 3) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 +
             2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) +
             6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) +
             2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 +
             T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) +
             4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) +
             4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 +
             2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) +
             6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) +
             4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 +
             2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) +
             8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 3) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             T0 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) -
         2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) -
         A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) -
         6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) -
         3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) -
         4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) -
         24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 -
         18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 -
         12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 -
         12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 -
         24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 -
         18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 -
         24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 -
         24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 -
         6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf +
         24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf +
         24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf +
         12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf -
         4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf +
         24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 +
         32 * T2 * T3 * T4 * pow(T5, 2) * Vf) /
            (6 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 12 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) +
             6 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) +
             6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2));
    GRBLinExpr b0 = (1.0 / 2.0) * A0;
    GRBLinExpr c0 = V0;
    GRBLinExpr d0 = P0;
    GRBLinExpr a1 =
        d3_var *
            (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 -
             2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 -
             3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) -
             pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) +
        d4_var *
            (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 +
             pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) +
             2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) +
             6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 +
             3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) +
             6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) +
             4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 +
             2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) +
             6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 +
             3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) +
             6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) +
             8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 +
             4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) +
             12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) +
             2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) +
             2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 +
             2 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) +
             3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 4) * pow(T3, 2) * pow(T4, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (-4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 6 * pow(T0, 2) * T2 * T3 * T4 * T5 -
             2 * pow(T0, 2) * T2 * T3 * pow(T5, 2) - 3 * pow(T0, 2) * T2 * pow(T4, 3) -
             6 * pow(T0, 2) * T2 * pow(T4, 2) * T5 - 3 * pow(T0, 2) * T2 * T4 * pow(T5, 2) -
             2 * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 3 * pow(T0, 2) * pow(T3, 2) * T4 * T5 -
             pow(T0, 2) * pow(T3, 2) * pow(T5, 2) - 2 * pow(T0, 2) * T3 * pow(T4, 3) -
             4 * pow(T0, 2) * T3 * pow(T4, 2) * T5 - 2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) -
             12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 -
             6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) -
             18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) -
             6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 -
             3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) -
             12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) -
             8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * T0 * pow(T2, 2) * T3 * T4 * T5 -
             4 * T0 * pow(T2, 2) * T3 * pow(T5, 2) - 6 * T0 * pow(T2, 2) * pow(T4, 3) -
             12 * T0 * pow(T2, 2) * pow(T4, 2) * T5 - 6 * T0 * pow(T2, 2) * T4 * pow(T5, 2) -
             6 * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T2 * pow(T3, 2) * T4 * T5 -
             3 * T0 * T2 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T2 * T3 * pow(T4, 3) -
             12 * T0 * T2 * T3 * pow(T4, 2) * T5 - 6 * T0 * T2 * T3 * T4 * pow(T5, 2) -
             12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 -
             6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) -
             18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) -
             6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 -
             3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) -
             12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) -
             16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 24 * T1 * pow(T2, 2) * T3 * T4 * T5 -
             8 * T1 * pow(T2, 2) * T3 * pow(T5, 2) - 12 * T1 * pow(T2, 2) * pow(T4, 3) -
             24 * T1 * pow(T2, 2) * pow(T4, 2) * T5 - 12 * T1 * pow(T2, 2) * T4 * pow(T5, 2) -
             12 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * T2 * pow(T3, 2) * T4 * T5 -
             6 * T1 * T2 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T2 * T3 * pow(T4, 3) -
             24 * T1 * T2 * T3 * pow(T4, 2) * T5 - 12 * T1 * T2 * T3 * T4 * pow(T5, 2) -
             4 * pow(T2, 3) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * T3 * T4 * T5 -
             2 * pow(T2, 3) * T3 * pow(T5, 2) - 3 * pow(T2, 3) * pow(T4, 3) -
             6 * pow(T2, 3) * pow(T4, 2) * T5 - 3 * pow(T2, 3) * T4 * pow(T5, 2) -
             4 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) * T4 * T5 -
             2 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) - 4 * pow(T2, 2) * T3 * pow(T4, 3) -
             8 * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 4 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T0 * pow(T1, 3) * T3 * pow(T4, 2) * pow(T5, 2) +
             3 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 4) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * pow(T1, 3) * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) +
         2 * A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) +
         3 * A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
         3 * A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) +
         A0 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
         8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) +
         6 * Af * pow(T0, 2) * T2 * T3 * pow(T5, 3) +
         6 * Af * pow(T0, 2) * T2 * pow(T4, 2) * pow(T5, 2) +
         6 * Af * pow(T0, 2) * T2 * T4 * pow(T5, 3) +
         4 * Af * pow(T0, 2) * pow(T3, 2) * T4 * pow(T5, 2) +
         3 * Af * pow(T0, 2) * pow(T3, 2) * pow(T5, 3) +
         4 * Af * pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
         4 * Af * pow(T0, 2) * T3 * T4 * pow(T5, 3) +
         24 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) +
         18 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) +
         18 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) +
         12 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) +
         9 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) +
         12 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) +
         12 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) +
         16 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
         12 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) +
         12 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) +
         12 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) +
         12 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) +
         9 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) +
         12 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
         12 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) +
         24 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) +
         18 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) +
         18 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) +
         18 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) +
         12 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) +
         9 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) +
         12 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
         12 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) +
         32 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
         24 * Af * T1 * pow(T2, 2) * T3 * pow(T5, 3) +
         24 * Af * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) +
         24 * Af * T1 * pow(T2, 2) * T4 * pow(T5, 3) +
         24 * Af * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) +
         18 * Af * T1 * T2 * pow(T3, 2) * pow(T5, 3) +
         24 * Af * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
         24 * Af * T1 * T2 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) +
         6 * Af * pow(T2, 3) * T3 * pow(T5, 3) + 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) +
         6 * Af * pow(T2, 3) * T4 * pow(T5, 3) +
         8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) +
         6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) +
         8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
         8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * T0 * T3 * T4 * pow(T5, 2) +
         12 * P0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T2 * T3 * T4 * pow(T5, 2) +
         24 * Pf * pow(T0, 2) * T2 * T3 * T4 + 36 * Pf * pow(T0, 2) * T2 * T3 * T5 +
         18 * Pf * pow(T0, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T0, 2) * T2 * T4 * T5 +
         12 * Pf * pow(T0, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T0, 2) * pow(T3, 2) * T5 +
         12 * Pf * pow(T0, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T0, 2) * T3 * T4 * T5 +
         72 * Pf * T0 * T1 * T2 * T3 * T4 + 108 * Pf * T0 * T1 * T2 * T3 * T5 +
         54 * Pf * T0 * T1 * T2 * pow(T4, 2) + 108 * Pf * T0 * T1 * T2 * T4 * T5 +
         36 * Pf * T0 * T1 * pow(T3, 2) * T4 + 54 * Pf * T0 * T1 * pow(T3, 2) * T5 +
         36 * Pf * T0 * T1 * T3 * pow(T4, 2) + 72 * Pf * T0 * T1 * T3 * T4 * T5 +
         48 * Pf * T0 * pow(T2, 2) * T3 * T4 + 72 * Pf * T0 * pow(T2, 2) * T3 * T5 +
         36 * Pf * T0 * pow(T2, 2) * pow(T4, 2) + 72 * Pf * T0 * pow(T2, 2) * T4 * T5 +
         36 * Pf * T0 * T2 * pow(T3, 2) * T4 + 54 * Pf * T0 * T2 * pow(T3, 2) * T5 +
         36 * Pf * T0 * T2 * T3 * pow(T4, 2) + 72 * Pf * T0 * T2 * T3 * T4 * T5 +
         72 * Pf * pow(T1, 2) * T2 * T3 * T4 + 108 * Pf * pow(T1, 2) * T2 * T3 * T5 +
         54 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 108 * Pf * pow(T1, 2) * T2 * T4 * T5 +
         36 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 54 * Pf * pow(T1, 2) * pow(T3, 2) * T5 +
         36 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 72 * Pf * pow(T1, 2) * T3 * T4 * T5 +
         96 * Pf * T1 * pow(T2, 2) * T3 * T4 + 144 * Pf * T1 * pow(T2, 2) * T3 * T5 +
         72 * Pf * T1 * pow(T2, 2) * pow(T4, 2) + 144 * Pf * T1 * pow(T2, 2) * T4 * T5 +
         72 * Pf * T1 * T2 * pow(T3, 2) * T4 + 108 * Pf * T1 * T2 * pow(T3, 2) * T5 +
         72 * Pf * T1 * T2 * T3 * pow(T4, 2) + 144 * Pf * T1 * T2 * T3 * T4 * T5 +
         24 * Pf * pow(T2, 3) * T3 * T4 + 36 * Pf * pow(T2, 3) * T3 * T5 +
         18 * Pf * pow(T2, 3) * pow(T4, 2) + 36 * Pf * pow(T2, 3) * T4 * T5 +
         24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 + 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 +
         24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) + 48 * Pf * pow(T2, 2) * T3 * T4 * T5 -
         24 * pow(T0, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T0, 2) * T2 * T3 * pow(T5, 2) * Vf -
         18 * pow(T0, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T0, 2) * T2 * T4 * pow(T5, 2) * Vf -
         12 * pow(T0, 2) * pow(T3, 2) * T4 * T5 * Vf -
         12 * pow(T0, 2) * pow(T3, 2) * pow(T5, 2) * Vf -
         12 * pow(T0, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 -
         16 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * Vf - 72 * T0 * T1 * T2 * T3 * T4 * T5 * Vf -
         72 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 54 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf -
         72 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 36 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf -
         36 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf +
         12 * T0 * T1 * T3 * T4 * pow(T5, 2) * V0 - 48 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf -
         48 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf -
         36 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf -
         36 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf -
         36 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 -
         48 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf - 72 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf -
         72 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 54 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf -
         72 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 36 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf -
         36 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf -
         36 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 -
         48 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 96 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf -
         96 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 72 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf -
         96 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 72 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf -
         72 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 72 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf +
         6 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 - 96 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf -
         24 * pow(T2, 3) * T3 * T4 * T5 * Vf - 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf -
         18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf - 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf -
         24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf -
         24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf -
         24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 -
         32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) /
            (6 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             6 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) +
             12 * T0 * pow(T1, 3) * T3 * T4 * pow(T5, 2) +
             18 * T0 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) +
             6 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
             6 * pow(T1, 4) * T3 * T4 * pow(T5, 2) + 12 * pow(T1, 3) * T2 * T3 * T4 * pow(T5, 2) +
             6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    GRBLinExpr b1 =
        d3_var * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_var *
            (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) -
             3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) -
             6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (12 * T1 * T2 * T3 * pow(T4, 2) + 18 * T1 * T2 * T3 * T4 * T5 +
             6 * T1 * T2 * T3 * pow(T5, 2) + 9 * T1 * T2 * pow(T4, 3) +
             18 * T1 * T2 * pow(T4, 2) * T5 + 9 * T1 * T2 * T4 * pow(T5, 2) +
             6 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T1 * pow(T3, 2) * T4 * T5 +
             3 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T1 * T3 * pow(T4, 3) +
             12 * T1 * T3 * pow(T4, 2) * T5 + 6 * T1 * T3 * T4 * pow(T5, 2) +
             12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 +
             6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) +
             18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) +
             12 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T2 * pow(T3, 2) * T4 * T5 +
             6 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T2 * T3 * pow(T4, 3) +
             24 * T2 * T3 * pow(T4, 2) * T5 + 12 * T2 * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-2 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) -
         A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) -
         6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) -
         3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) -
         4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) -
         24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 -
         18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 -
         12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 -
         12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 -
         24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 -
         18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 -
         24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 -
         24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 -
         6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf +
         24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf +
         24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf +
         12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf -
         4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf +
         24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 +
         32 * T2 * T3 * T4 * pow(T5, 2) * Vf) /
            (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) +
             2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             2 * T1 * T2 * T3 * T4 * pow(T5, 2));
    GRBLinExpr c1 =
        d3_var *
            (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 +
             3 * T0 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_var *
            (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 -
             3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) -
             6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) -
             6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 -
             3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) -
             12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (12 * T0 * T1 * T2 * T3 * pow(T4, 2) + 18 * T0 * T1 * T2 * T3 * T4 * T5 +
             6 * T0 * T1 * T2 * T3 * pow(T5, 2) + 9 * T0 * T1 * T2 * pow(T4, 3) +
             18 * T0 * T1 * T2 * pow(T4, 2) * T5 + 9 * T0 * T1 * T2 * T4 * pow(T5, 2) +
             6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * pow(T3, 2) * T4 * T5 +
             3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T3 * pow(T4, 3) +
             12 * T0 * T1 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T3 * T4 * pow(T5, 2) +
             12 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * T0 * pow(T2, 2) * T3 * T4 * T5 +
             6 * T0 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * T0 * pow(T2, 2) * pow(T4, 3) +
             18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * T0 * pow(T2, 2) * T4 * pow(T5, 2) +
             12 * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T0 * T2 * pow(T3, 2) * T4 * T5 +
             6 * T0 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T0 * T2 * T3 * pow(T4, 3) +
             24 * T0 * T2 * T3 * pow(T4, 2) * T5 + 12 * T0 * T2 * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
         A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) -
         6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) -
         3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) -
         4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) -
         8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) -
         6 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) -
         8 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) -
         8 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) -
         6 * P0 * T0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T2 * T3 * T4 -
         36 * Pf * T0 * T1 * T2 * T3 * T5 - 18 * Pf * T0 * T1 * T2 * pow(T4, 2) -
         36 * Pf * T0 * T1 * T2 * T4 * T5 - 12 * Pf * T0 * T1 * pow(T3, 2) * T4 -
         18 * Pf * T0 * T1 * pow(T3, 2) * T5 - 12 * Pf * T0 * T1 * T3 * pow(T4, 2) -
         24 * Pf * T0 * T1 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * T4 -
         36 * Pf * T0 * pow(T2, 2) * T3 * T5 - 18 * Pf * T0 * pow(T2, 2) * pow(T4, 2) -
         36 * Pf * T0 * pow(T2, 2) * T4 * T5 - 24 * Pf * T0 * T2 * pow(T3, 2) * T4 -
         36 * Pf * T0 * T2 * pow(T3, 2) * T5 - 24 * Pf * T0 * T2 * T3 * pow(T4, 2) -
         48 * Pf * T0 * T2 * T3 * T4 * T5 - 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 +
         24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf +
         18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf +
         12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf +
         12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf +
         24 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         24 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 32 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf +
         2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) /
            (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) +
             2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             2 * T1 * T2 * T3 * T4 * pow(T5, 2));
    GRBLinExpr d1 =
        d3_var *
            (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) +
             2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_var *
            (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 -
             pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) -
             2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) -
             2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 -
             pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) -
             4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (4 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 +
             2 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) + 3 * pow(T0, 2) * T1 * T2 * pow(T4, 3) +
             6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 +
             3 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) +
             2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) +
             3 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 +
             pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 3) +
             4 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 +
             2 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) +
             4 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
             6 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 +
             2 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) +
             3 * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) +
             6 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 +
             3 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) +
             4 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             6 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 +
             2 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 3) +
             8 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 +
             4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) +
        (2 * A0 * pow(T0, 3) * T1 * T3 * T4 * pow(T5, 2) +
         A0 * pow(T0, 3) * T2 * T3 * T4 * pow(T5, 2) +
         2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
         2 * A0 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) -
         8 * Af * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 3) -
         6 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 3) -
         4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) -
         3 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 3) -
         4 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) -
         4 * Af * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 3) -
         8 * Af * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 3) -
         6 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 3) -
         8 * Af * pow(T0, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 3) -
         8 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 3) +
         12 * P0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T0 * T2 * T3 * T4 * pow(T5, 2) +
         6 * P0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * P0 * T1 * T2 * T3 * T4 * pow(T5, 2) -
         24 * Pf * pow(T0, 2) * T1 * T2 * T3 * T4 - 36 * Pf * pow(T0, 2) * T1 * T2 * T3 * T5 -
         18 * Pf * pow(T0, 2) * T1 * T2 * pow(T4, 2) - 36 * Pf * pow(T0, 2) * T1 * T2 * T4 * T5 -
         12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T4 - 18 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T5 -
         12 * Pf * pow(T0, 2) * T1 * T3 * pow(T4, 2) - 24 * Pf * pow(T0, 2) * T1 * T3 * T4 * T5 -
         24 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T5 -
         18 * Pf * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) -
         36 * Pf * pow(T0, 2) * pow(T2, 2) * T4 * T5 - 24 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T4 -
         36 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T5 - 24 * Pf * pow(T0, 2) * T2 * T3 * pow(T4, 2) -
         48 * Pf * pow(T0, 2) * T2 * T3 * T4 * T5 + 24 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 * Vf +
         24 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) * Vf +
         18 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 * Vf +
         24 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) * Vf +
         12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 * Vf +
         12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) * Vf +
         12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 * Vf +
         8 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * V0 +
         16 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * Vf +
         24 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf +
         24 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         18 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf +
         24 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         24 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf +
         24 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf +
         4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * V0 +
         32 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf +
         6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 +
         6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) /
            (6 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) +
             6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             6 * T1 * T2 * T3 * T4 * pow(T5, 2));
    GRBLinExpr a2 =
        d3_var *
            (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 +
             3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) +
             pow(T2, 3) * pow(T3, 2)) +
        d4_var *
            (-2 * T0 * T1 * pow(T3, 2) - 3 * T0 * T1 * T3 * T4 - T0 * T1 * pow(T4, 2) -
             4 * T0 * T2 * pow(T3, 2) - 6 * T0 * T2 * T3 * T4 - 2 * T0 * T2 * pow(T4, 2) -
             T0 * pow(T3, 3) - 2 * T0 * pow(T3, 2) * T4 - T0 * T3 * pow(T4, 2) -
             2 * pow(T1, 2) * pow(T3, 2) - 3 * pow(T1, 2) * T3 * T4 - pow(T1, 2) * pow(T4, 2) -
             8 * T1 * T2 * pow(T3, 2) - 12 * T1 * T2 * T3 * T4 - 4 * T1 * T2 * pow(T4, 2) -
             2 * T1 * pow(T3, 3) - 4 * T1 * pow(T3, 2) * T4 - 2 * T1 * T3 * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) -
             3 * T2 * pow(T3, 3) - 6 * T2 * pow(T3, 2) * T4 - 3 * T2 * T3 * pow(T4, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T2, 3) * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (4 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T1 * T3 * T4 * T5 +
             2 * T0 * T1 * T3 * pow(T5, 2) + 3 * T0 * T1 * pow(T4, 3) +
             6 * T0 * T1 * pow(T4, 2) * T5 + 3 * T0 * T1 * T4 * pow(T5, 2) +
             8 * T0 * T2 * T3 * pow(T4, 2) + 12 * T0 * T2 * T3 * T4 * T5 +
             4 * T0 * T2 * T3 * pow(T5, 2) + 6 * T0 * T2 * pow(T4, 3) +
             12 * T0 * T2 * pow(T4, 2) * T5 + 6 * T0 * T2 * T4 * pow(T5, 2) +
             2 * T0 * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T3, 2) * T4 * T5 +
             T0 * pow(T3, 2) * pow(T5, 2) + 2 * T0 * T3 * pow(T4, 3) +
             4 * T0 * T3 * pow(T4, 2) * T5 + 2 * T0 * T3 * T4 * pow(T5, 2) +
             4 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * T4 * T5 +
             2 * pow(T1, 2) * T3 * pow(T5, 2) + 3 * pow(T1, 2) * pow(T4, 3) +
             6 * pow(T1, 2) * pow(T4, 2) * T5 + 3 * pow(T1, 2) * T4 * pow(T5, 2) +
             16 * T1 * T2 * T3 * pow(T4, 2) + 24 * T1 * T2 * T3 * T4 * T5 +
             8 * T1 * T2 * T3 * pow(T5, 2) + 12 * T1 * T2 * pow(T4, 3) +
             24 * T1 * T2 * pow(T4, 2) * T5 + 12 * T1 * T2 * T4 * pow(T5, 2) +
             4 * T1 * pow(T3, 2) * pow(T4, 2) + 6 * T1 * pow(T3, 2) * T4 * T5 +
             2 * T1 * pow(T3, 2) * pow(T5, 2) + 4 * T1 * T3 * pow(T4, 3) +
             8 * T1 * T3 * pow(T4, 2) * T5 + 4 * T1 * T3 * T4 * pow(T5, 2) +
             12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 +
             6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) +
             18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) +
             6 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 2) * T4 * T5 +
             3 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T2 * T3 * pow(T4, 3) +
             12 * T2 * T3 * pow(T4, 2) * T5 + 6 * T2 * T3 * T4 * pow(T5, 2)) /
            (T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T3 * T4 * pow(T5, 2) -
         8 * Af * T0 * T1 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T3 * pow(T5, 3) -
         6 * Af * T0 * T1 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T4 * pow(T5, 3) -
         16 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T2 * T3 * pow(T5, 3) -
         12 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T2 * T4 * pow(T5, 3) -
         4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) -
         4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T3 * T4 * pow(T5, 3) -
         8 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T5, 3) -
         6 * Af * pow(T1, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T1, 2) * T4 * pow(T5, 3) -
         32 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Af * T1 * T2 * T3 * pow(T5, 3) -
         24 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 24 * Af * T1 * T2 * T4 * pow(T5, 3) -
         8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) -
         8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T1 * T3 * T4 * pow(T5, 3) -
         24 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 18 * Af * pow(T2, 2) * T3 * pow(T5, 3) -
         18 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 18 * Af * pow(T2, 2) * T4 * pow(T5, 3) -
         12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) -
         12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T2 * T3 * T4 * pow(T5, 3) -
         6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T3 * T4 - 36 * Pf * T0 * T1 * T3 * T5 -
         18 * Pf * T0 * T1 * pow(T4, 2) - 36 * Pf * T0 * T1 * T4 * T5 -
         48 * Pf * T0 * T2 * T3 * T4 - 72 * Pf * T0 * T2 * T3 * T5 -
         36 * Pf * T0 * T2 * pow(T4, 2) - 72 * Pf * T0 * T2 * T4 * T5 -
         12 * Pf * T0 * pow(T3, 2) * T4 - 18 * Pf * T0 * pow(T3, 2) * T5 -
         12 * Pf * T0 * T3 * pow(T4, 2) - 24 * Pf * T0 * T3 * T4 * T5 -
         24 * Pf * pow(T1, 2) * T3 * T4 - 36 * Pf * pow(T1, 2) * T3 * T5 -
         18 * Pf * pow(T1, 2) * pow(T4, 2) - 36 * Pf * pow(T1, 2) * T4 * T5 -
         96 * Pf * T1 * T2 * T3 * T4 - 144 * Pf * T1 * T2 * T3 * T5 -
         72 * Pf * T1 * T2 * pow(T4, 2) - 144 * Pf * T1 * T2 * T4 * T5 -
         24 * Pf * T1 * pow(T3, 2) * T4 - 36 * Pf * T1 * pow(T3, 2) * T5 -
         24 * Pf * T1 * T3 * pow(T4, 2) - 48 * Pf * T1 * T3 * T4 * T5 -
         72 * Pf * pow(T2, 2) * T3 * T4 - 108 * Pf * pow(T2, 2) * T3 * T5 -
         54 * Pf * pow(T2, 2) * pow(T4, 2) - 108 * Pf * pow(T2, 2) * T4 * T5 -
         36 * Pf * T2 * pow(T3, 2) * T4 - 54 * Pf * T2 * pow(T3, 2) * T5 -
         36 * Pf * T2 * T3 * pow(T4, 2) - 72 * Pf * T2 * T3 * T4 * T5 +
         24 * T0 * T1 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T3 * pow(T5, 2) * Vf +
         18 * T0 * T1 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T4 * pow(T5, 2) * Vf +
         48 * T0 * T2 * T3 * T4 * T5 * Vf + 48 * T0 * T2 * T3 * pow(T5, 2) * Vf +
         36 * T0 * T2 * pow(T4, 2) * T5 * Vf + 48 * T0 * T2 * T4 * pow(T5, 2) * Vf +
         12 * T0 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf +
         12 * T0 * T3 * pow(T4, 2) * T5 * Vf - 4 * T0 * T3 * T4 * pow(T5, 2) * V0 +
         16 * T0 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T1, 2) * T3 * T4 * T5 * Vf +
         24 * pow(T1, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T1, 2) * pow(T4, 2) * T5 * Vf +
         24 * pow(T1, 2) * T4 * pow(T5, 2) * Vf + 96 * T1 * T2 * T3 * T4 * T5 * Vf +
         96 * T1 * T2 * T3 * pow(T5, 2) * Vf + 72 * T1 * T2 * pow(T4, 2) * T5 * Vf +
         96 * T1 * T2 * T4 * pow(T5, 2) * Vf + 24 * T1 * pow(T3, 2) * T4 * T5 * Vf +
         24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T1 * T3 * pow(T4, 2) * T5 * Vf -
         2 * T1 * T3 * T4 * pow(T5, 2) * V0 + 32 * T1 * T3 * T4 * pow(T5, 2) * Vf +
         72 * pow(T2, 2) * T3 * T4 * T5 * Vf + 72 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         54 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 72 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         36 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         36 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) /
            (6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
             6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) +
             12 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 3) * T3 * T4 * pow(T5, 2));
    GRBLinExpr b2 =
        d3_var *
            (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 -
             3 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_var *
            (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) +
             3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) +
             12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) +
             6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) +
             12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 +
             6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 +
             9 * T2 * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (-12 * T0 * T2 * T3 * pow(T4, 2) - 18 * T0 * T2 * T3 * T4 * T5 -
             6 * T0 * T2 * T3 * pow(T5, 2) - 9 * T0 * T2 * pow(T4, 3) -
             18 * T0 * T2 * pow(T4, 2) * T5 - 9 * T0 * T2 * T4 * pow(T5, 2) -
             6 * T0 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * pow(T3, 2) * T4 * T5 -
             3 * T0 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T3 * pow(T4, 3) -
             12 * T0 * T3 * pow(T4, 2) * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) -
             24 * T1 * T2 * T3 * pow(T4, 2) - 36 * T1 * T2 * T3 * T4 * T5 -
             12 * T1 * T2 * T3 * pow(T5, 2) - 18 * T1 * T2 * pow(T4, 3) -
             36 * T1 * T2 * pow(T4, 2) * T5 - 18 * T1 * T2 * T4 * pow(T5, 2) -
             12 * T1 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * pow(T3, 2) * T4 * T5 -
             6 * T1 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T3 * pow(T4, 3) -
             24 * T1 * T3 * pow(T4, 2) * T5 - 12 * T1 * T3 * T4 * pow(T5, 2) -
             24 * pow(T2, 2) * T3 * pow(T4, 2) - 36 * pow(T2, 2) * T3 * T4 * T5 -
             12 * pow(T2, 2) * T3 * pow(T5, 2) - 18 * pow(T2, 2) * pow(T4, 3) -
             36 * pow(T2, 2) * pow(T4, 2) * T5 - 18 * pow(T2, 2) * T4 * pow(T5, 2) -
             18 * T2 * pow(T3, 2) * pow(T4, 2) - 27 * T2 * pow(T3, 2) * T4 * T5 -
             9 * T2 * pow(T3, 2) * pow(T5, 2) - 18 * T2 * T3 * pow(T4, 3) -
             36 * T2 * T3 * pow(T4, 2) * T5 - 18 * T2 * T3 * T4 * pow(T5, 2)) /
            (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T3 * T4 * pow(T5, 2) +
         8 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T2 * T3 * pow(T5, 3) +
         6 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T2 * T4 * pow(T5, 3) +
         4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) +
         4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T3 * T4 * pow(T5, 3) +
         16 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) + 12 * Af * T1 * T2 * T3 * pow(T5, 3) +
         12 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T1 * T2 * T4 * pow(T5, 3) +
         8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) +
         8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * T1 * T3 * T4 * pow(T5, 3) +
         16 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * pow(T2, 2) * T3 * pow(T5, 3) +
         12 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T2, 2) * T4 * pow(T5, 3) +
         12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) +
         12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T2 * T3 * T4 * pow(T5, 3) +
         6 * P0 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T2 * T3 * T4 + 36 * Pf * T0 * T2 * T3 * T5 +
         18 * Pf * T0 * T2 * pow(T4, 2) + 36 * Pf * T0 * T2 * T4 * T5 +
         12 * Pf * T0 * pow(T3, 2) * T4 + 18 * Pf * T0 * pow(T3, 2) * T5 +
         12 * Pf * T0 * T3 * pow(T4, 2) + 24 * Pf * T0 * T3 * T4 * T5 +
         48 * Pf * T1 * T2 * T3 * T4 + 72 * Pf * T1 * T2 * T3 * T5 +
         36 * Pf * T1 * T2 * pow(T4, 2) + 72 * Pf * T1 * T2 * T4 * T5 +
         24 * Pf * T1 * pow(T3, 2) * T4 + 36 * Pf * T1 * pow(T3, 2) * T5 +
         24 * Pf * T1 * T3 * pow(T4, 2) + 48 * Pf * T1 * T3 * T4 * T5 +
         48 * Pf * pow(T2, 2) * T3 * T4 + 72 * Pf * pow(T2, 2) * T3 * T5 +
         36 * Pf * pow(T2, 2) * pow(T4, 2) + 72 * Pf * pow(T2, 2) * T4 * T5 +
         36 * Pf * T2 * pow(T3, 2) * T4 + 54 * Pf * T2 * pow(T3, 2) * T5 +
         36 * Pf * T2 * T3 * pow(T4, 2) + 72 * Pf * T2 * T3 * T4 * T5 -
         24 * T0 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T2 * T3 * pow(T5, 2) * Vf -
         18 * T0 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T2 * T4 * pow(T5, 2) * Vf -
         12 * T0 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf -
         12 * T0 * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * T3 * T4 * pow(T5, 2) * V0 -
         16 * T0 * T3 * T4 * pow(T5, 2) * Vf - 48 * T1 * T2 * T3 * T4 * T5 * Vf -
         48 * T1 * T2 * T3 * pow(T5, 2) * Vf - 36 * T1 * T2 * pow(T4, 2) * T5 * Vf -
         48 * T1 * T2 * T4 * pow(T5, 2) * Vf - 24 * T1 * pow(T3, 2) * T4 * T5 * Vf -
         24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 24 * T1 * T3 * pow(T4, 2) * T5 * Vf +
         2 * T1 * T3 * T4 * pow(T5, 2) * V0 - 32 * T1 * T3 * T4 * pow(T5, 2) * Vf -
         48 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * pow(T2, 2) * T3 * pow(T5, 2) * Vf -
         36 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * pow(T2, 2) * T4 * pow(T5, 2) * Vf -
         36 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf -
         36 * T2 * T3 * pow(T4, 2) * T5 * Vf - 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) /
            (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) +
             2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) +
             2 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    GRBLinExpr c2 =
        d3_var *
            (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 +
             3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_var *
            (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 +
             3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) +
             6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 +
             3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) +
             6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) -
             6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 -
             6 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (-12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 -
             6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) -
             18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) -
             6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 -
             3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) -
             12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) -
             12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 -
             6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) -
             18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) -
             6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 -
             3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) -
             12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             12 * pow(T2, 3) * T3 * pow(T4, 2) + 18 * pow(T2, 3) * T3 * T4 * T5 +
             6 * pow(T2, 3) * T3 * pow(T5, 2) + 9 * pow(T2, 3) * pow(T4, 3) +
             18 * pow(T2, 3) * pow(T4, 2) * T5 + 9 * pow(T2, 3) * T4 * pow(T5, 2) +
             12 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 18 * pow(T2, 2) * pow(T3, 2) * T4 * T5 +
             6 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 3) +
             24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 12 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) /
            (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) +
         8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) +
         6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) +
         4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) +
         3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) +
         4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) +
         8 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) +
         6 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) +
         6 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) +
         6 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) +
         4 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) +
         3 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) +
         4 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
         4 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T2, 3) * T4 * pow(T5, 3) -
         8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) -
         8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 6 * P0 * T2 * T3 * T4 * pow(T5, 2) +
         24 * Pf * T0 * T1 * T2 * T3 * T4 + 36 * Pf * T0 * T1 * T2 * T3 * T5 +
         18 * Pf * T0 * T1 * T2 * pow(T4, 2) + 36 * Pf * T0 * T1 * T2 * T4 * T5 +
         12 * Pf * T0 * T1 * pow(T3, 2) * T4 + 18 * Pf * T0 * T1 * pow(T3, 2) * T5 +
         12 * Pf * T0 * T1 * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * T3 * T4 * T5 +
         24 * Pf * pow(T1, 2) * T2 * T3 * T4 + 36 * Pf * pow(T1, 2) * T2 * T3 * T5 +
         18 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T1, 2) * T2 * T4 * T5 +
         12 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T1, 2) * pow(T3, 2) * T5 +
         12 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T1, 2) * T3 * T4 * T5 -
         24 * Pf * pow(T2, 3) * T3 * T4 - 36 * Pf * pow(T2, 3) * T3 * T5 -
         18 * Pf * pow(T2, 3) * pow(T4, 2) - 36 * Pf * pow(T2, 3) * T4 * T5 -
         24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 -
         24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * pow(T2, 2) * T3 * T4 * T5 -
         24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf -
         18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf -
         12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf -
         12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf - 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf -
         4 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 24 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf -
         24 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf -
         24 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf -
         12 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf -
         12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf - 16 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf -
         2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 + 24 * pow(T2, 3) * T3 * T4 * T5 * Vf +
         24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf +
         24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf +
         24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf +
         32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) /
            (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) +
             2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) +
             2 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    GRBLinExpr d2 =
        d3_var *
            (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) +
             T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_var *
            (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 -
             2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) -
             6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) -
             2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 -
             T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) -
             4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) -
             4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 -
             2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) -
             6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) -
             4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 -
             2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) -
             8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        d5_var *
            (8 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 +
             4 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T4, 3) +
             12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 +
             6 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) +
             6 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 +
             3 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 3) +
             12 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) +
             4 * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 3) * T3 * T4 * T5 +
             2 * T0 * pow(T2, 3) * T3 * pow(T5, 2) + 3 * T0 * pow(T2, 3) * pow(T4, 3) +
             6 * T0 * pow(T2, 3) * pow(T4, 2) * T5 + 3 * T0 * pow(T2, 3) * T4 * pow(T5, 2) +
             4 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
             6 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 +
             2 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 3) +
             8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 +
             4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
             8 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
             12 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 +
             4 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) +
             6 * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) +
             12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 +
             6 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             9 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 +
             3 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 3) +
             12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 +
             6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 3) * T3 * pow(T4, 2) +
             12 * T1 * pow(T2, 3) * T3 * T4 * T5 + 4 * T1 * pow(T2, 3) * T3 * pow(T5, 2) +
             6 * T1 * pow(T2, 3) * pow(T4, 3) + 12 * T1 * pow(T2, 3) * pow(T4, 2) * T5 +
             6 * T1 * pow(T2, 3) * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
             12 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 +
             4 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 8 * T1 * pow(T2, 2) * T3 * pow(T4, 3) +
             16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 +
             8 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) /
            (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (A0 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
         A0 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         16 * Af * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 3) -
         12 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         12 * Af * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 3) -
         12 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         9 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 3) -
         12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         12 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 3) -
         8 * Af * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 3) * T3 * pow(T5, 3) -
         6 * Af * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 3) * T4 * pow(T5, 3) -
         8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) -
         8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 3) -
         16 * Af * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 3) -
         12 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         12 * Af * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 3) -
         12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         9 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 3) -
         12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         12 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 3) -
         16 * Af * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) -
         12 * Af * T1 * pow(T2, 3) * T3 * pow(T5, 3) -
         12 * Af * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) -
         12 * Af * T1 * pow(T2, 3) * T4 * pow(T5, 3) -
         16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) -
         12 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) -
         16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) -
         16 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 3) +
         6 * P0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 48 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 -
         72 * Pf * T0 * T1 * pow(T2, 2) * T3 * T5 - 36 * Pf * T0 * T1 * pow(T2, 2) * pow(T4, 2) -
         72 * Pf * T0 * T1 * pow(T2, 2) * T4 * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 -
         54 * Pf * T0 * T1 * T2 * pow(T3, 2) * T5 - 36 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) -
         72 * Pf * T0 * T1 * T2 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 3) * T3 * T4 -
         36 * Pf * T0 * pow(T2, 3) * T3 * T5 - 18 * Pf * T0 * pow(T2, 3) * pow(T4, 2) -
         36 * Pf * T0 * pow(T2, 3) * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 -
         36 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 2) -
         48 * Pf * T0 * pow(T2, 2) * T3 * T4 * T5 - 48 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 -
         72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T5 -
         36 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) -
         72 * Pf * pow(T1, 2) * pow(T2, 2) * T4 * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 -
         54 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T5 - 36 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) -
         72 * Pf * pow(T1, 2) * T2 * T3 * T4 * T5 - 48 * Pf * T1 * pow(T2, 3) * T3 * T4 -
         72 * Pf * T1 * pow(T2, 3) * T3 * T5 - 36 * Pf * T1 * pow(T2, 3) * pow(T4, 2) -
         72 * Pf * T1 * pow(T2, 3) * T4 * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 -
         72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T5 - 48 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 2) -
         96 * Pf * T1 * pow(T2, 2) * T3 * T4 * T5 + 48 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf +
         48 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf +
         48 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         36 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf +
         36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         36 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf +
         48 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 3) * T3 * T4 * T5 * Vf +
         24 * T0 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 3) * pow(T4, 2) * T5 * Vf +
         24 * T0 * pow(T2, 3) * T4 * pow(T5, 2) * Vf +
         24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf +
         24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf +
         4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 +
         32 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf +
         48 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf +
         48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         36 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf +
         48 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf +
         36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf +
         48 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf +
         48 * T1 * pow(T2, 3) * T3 * T4 * T5 * Vf + 48 * T1 * pow(T2, 3) * T3 * pow(T5, 2) * Vf +
         36 * T1 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 3) * T4 * pow(T5, 2) * Vf +
         48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf +
         48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf +
         48 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf +
         2 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 +
         64 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) /
            (6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) +
             6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * T1 * T2 * T3 * T4 * pow(T5, 2) +
             6 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    GRBLinExpr a3 =
        (1.0 / 2.0) *
            (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) +
             2 * Af * pow(T4, 2) * pow(T5, 2) + 2 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 +
             18 * Pf * T3 * T5 + 6 * Pf * pow(T4, 2) + 12 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf -
             12 * T3 * pow(T5, 2) * Vf - 6 * pow(T4, 2) * T5 * Vf - 8 * T4 * pow(T5, 2) * Vf) /
            (pow(T3, 2) * T4 * pow(T5, 2)) +
        d5_var *
            (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 3 * pow(T4, 3) -
             6 * pow(T4, 2) * T5 - 3 * T4 * pow(T5, 2)) /
            (pow(T3, 2) * pow(T4, 2) * pow(T5, 2)) -
        d3_var / pow(T3, 3) +
        d4_var * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2)) / (pow(T3, 3) * pow(T4, 2));
    GRBLinExpr b3 =
        (-4 * Af * T3 * T4 * pow(T5, 2) - 3 * Af * T3 * pow(T5, 3) -
         3 * Af * pow(T4, 2) * pow(T5, 2) - 3 * Af * T4 * pow(T5, 3) - 12 * Pf * T3 * T4 -
         18 * Pf * T3 * T5 - 9 * Pf * pow(T4, 2) - 18 * Pf * T4 * T5 + 12 * T3 * T4 * T5 * Vf +
         12 * T3 * pow(T5, 2) * Vf + 9 * pow(T4, 2) * T5 * Vf + 12 * T4 * pow(T5, 2) * Vf) /
            (T3 * T4 * pow(T5, 2)) +
        d5_var *
            (12 * T3 * pow(T4, 2) + 18 * T3 * T4 * T5 + 6 * T3 * pow(T5, 2) + 9 * pow(T4, 3) +
             18 * pow(T4, 2) * T5 + 9 * T4 * pow(T5, 2)) /
            (T3 * pow(T4, 2) * pow(T5, 2)) +
        3 * d3_var / pow(T3, 2) +
        d4_var * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2));
    GRBLinExpr c3 =
        (1.0 / 2.0) *
            (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) +
             4 * Af * pow(T4, 2) * pow(T5, 2) + 4 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 +
             18 * Pf * T3 * T5 + 12 * Pf * pow(T4, 2) + 24 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf -
             12 * T3 * pow(T5, 2) * Vf - 12 * pow(T4, 2) * T5 * Vf - 16 * T4 * pow(T5, 2) * Vf) /
            (T4 * pow(T5, 2)) +
        d5_var *
            (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 6 * pow(T4, 3) -
             12 * pow(T4, 2) * T5 - 6 * T4 * pow(T5, 2)) /
            (pow(T4, 2) * pow(T5, 2)) -
        3 * d3_var / T3 +
        d4_var * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2));
    GRBLinExpr d3 = d3_var;
    GRBLinExpr a4 =
        (1.0 / 2.0) *
            (-2 * Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 6 * Pf * T4 - 6 * Pf * T5 +
             6 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) /
            (pow(T4, 2) * pow(T5, 2)) -
        d4_var / pow(T4, 3) +
        d5_var * (3 * pow(T4, 2) + 3 * T4 * T5 + pow(T5, 2)) / (pow(T4, 3) * pow(T5, 2));
    GRBLinExpr b4 =
        (1.0 / 2.0) *
            (4 * Af * T4 * pow(T5, 2) + 3 * Af * pow(T5, 3) + 12 * Pf * T4 + 18 * Pf * T5 -
             12 * T4 * T5 * Vf - 12 * pow(T5, 2) * Vf) /
            (T4 * pow(T5, 2)) +
        3 * d4_var / pow(T4, 2) +
        d5_var * (-6 * pow(T4, 2) - 9 * T4 * T5 - 3 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2));
    GRBLinExpr c4 = (-Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 3 * Pf * T4 - 6 * Pf * T5 +
                     3 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) /
                        pow(T5, 2) -
                    3 * d4_var / T4 +
                    d5_var * (3 * pow(T4, 2) + 6 * T4 * T5 + 3 * pow(T5, 2)) / (T4 * pow(T5, 2));
    GRBLinExpr d4 = d4_var;
    GRBLinExpr a5 =
        -d5_var / pow(T5, 3) + (1.0 / 2.0) * (Af * pow(T5, 2) + 2 * Pf - 2 * T5 * Vf) / pow(T5, 3);
    GRBLinExpr b5 =
        3 * d5_var / pow(T5, 2) + (-Af * pow(T5, 2) - 3 * Pf + 3 * T5 * Vf) / pow(T5, 2);
    GRBLinExpr c5 = -3 * d5_var / T5 + (1.0 / 2.0) * (Af * pow(T5, 2) + 6 * Pf - 4 * T5 * Vf) / T5;
    GRBLinExpr d5 = d5_var;

    // Fill x_ with the coefficients
    x_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2,
                  a3, b3, c3, d3, a4, b4, c4, d4, a5, b5, c5, d5});
  }
}

void SolverGurobi::getDependentCoefficientsN4Double() {
  // Clear the vector
  x_double_.clear();

  // get the time intervals from the optimization
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];

  for (int axis = 0; axis < 3; axis++) {
    // get the initial and final values from the optimization
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // get free parameters
    double d3_val = d3_[axis].get(GRB_DoubleAttr_X);

    // C++ expressions for composite cubic Bézier control coefficients (one coordinate) in terms of
    // lambda:
    double a0 =
        d3_val * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) /
            (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) +
             T0 * T1 * T2 * pow(T3, 2)) +
        (-3 * A0 * pow(T0, 2) * pow(T3, 2) - 4 * A0 * T0 * T1 * pow(T3, 2) -
         2 * A0 * T0 * T2 * pow(T3, 2) - A0 * pow(T1, 2) * pow(T3, 2) - A0 * T1 * T2 * pow(T3, 2) -
         2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) -
         2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 -
         6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 +
         6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf +
         6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf) /
            (6 * pow(T0, 3) * pow(T3, 2) + 12 * pow(T0, 2) * T1 * pow(T3, 2) +
             6 * pow(T0, 2) * T2 * pow(T3, 2) + 6 * T0 * pow(T1, 2) * pow(T3, 2) +
             6 * T0 * T1 * T2 * pow(T3, 2));
    double b0 = (1.0 / 2.0) * A0;
    double c0 = V0;
    double d0 = P0;
    double a1 =
        d3_val *
            (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 -
             2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 -
             3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) -
             pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) +
        (A0 * pow(T0, 3) * pow(T3, 2) + 4 * A0 * pow(T0, 2) * T1 * pow(T3, 2) +
         2 * A0 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * A0 * T0 * pow(T1, 2) * pow(T3, 2) +
         3 * A0 * T0 * T1 * T2 * pow(T3, 2) + A0 * T0 * pow(T2, 2) * pow(T3, 2) +
         2 * Af * pow(T0, 2) * T2 * pow(T3, 2) + Af * pow(T0, 2) * pow(T3, 3) +
         6 * Af * T0 * T1 * T2 * pow(T3, 2) + 3 * Af * T0 * T1 * pow(T3, 3) +
         4 * Af * T0 * pow(T2, 2) * pow(T3, 2) + 3 * Af * T0 * T2 * pow(T3, 3) +
         6 * Af * pow(T1, 2) * T2 * pow(T3, 2) + 3 * Af * pow(T1, 2) * pow(T3, 3) +
         8 * Af * T1 * pow(T2, 2) * pow(T3, 2) + 6 * Af * T1 * T2 * pow(T3, 3) +
         2 * Af * pow(T2, 3) * pow(T3, 2) + 2 * Af * pow(T2, 2) * pow(T3, 3) +
         6 * P0 * T0 * pow(T3, 2) + 12 * P0 * T1 * pow(T3, 2) + 6 * P0 * T2 * pow(T3, 2) +
         6 * Pf * pow(T0, 2) * T2 + 6 * Pf * pow(T0, 2) * T3 + 18 * Pf * T0 * T1 * T2 +
         18 * Pf * T0 * T1 * T3 + 12 * Pf * T0 * pow(T2, 2) + 18 * Pf * T0 * T2 * T3 +
         18 * Pf * pow(T1, 2) * T2 + 18 * Pf * pow(T1, 2) * T3 + 24 * Pf * T1 * pow(T2, 2) +
         36 * Pf * T1 * T2 * T3 + 6 * Pf * pow(T2, 3) + 12 * Pf * pow(T2, 2) * T3 -
         6 * pow(T0, 2) * T2 * T3 * Vf + 4 * pow(T0, 2) * pow(T3, 2) * V0 -
         4 * pow(T0, 2) * pow(T3, 2) * Vf - 18 * T0 * T1 * T2 * T3 * Vf +
         12 * T0 * T1 * pow(T3, 2) * V0 - 12 * T0 * T1 * pow(T3, 2) * Vf -
         12 * T0 * pow(T2, 2) * T3 * Vf + 6 * T0 * T2 * pow(T3, 2) * V0 -
         12 * T0 * T2 * pow(T3, 2) * Vf - 18 * pow(T1, 2) * T2 * T3 * Vf +
         6 * pow(T1, 2) * pow(T3, 2) * V0 - 12 * pow(T1, 2) * pow(T3, 2) * Vf -
         24 * T1 * pow(T2, 2) * T3 * Vf + 6 * T1 * T2 * pow(T3, 2) * V0 -
         24 * T1 * T2 * pow(T3, 2) * Vf - 6 * pow(T2, 3) * T3 * Vf +
         2 * pow(T2, 2) * pow(T3, 2) * V0 - 8 * pow(T2, 2) * pow(T3, 2) * Vf) /
            (6 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 6 * pow(T0, 2) * T1 * T2 * pow(T3, 2) +
             12 * T0 * pow(T1, 3) * pow(T3, 2) + 18 * T0 * pow(T1, 2) * T2 * pow(T3, 2) +
             6 * T0 * T1 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T1, 4) * pow(T3, 2) +
             12 * pow(T1, 3) * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2));
    double b1 =
        d3_val * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        (-2 * A0 * pow(T0, 2) * pow(T3, 2) - 2 * A0 * T0 * T1 * pow(T3, 2) -
         A0 * T0 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) -
         2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) -
         6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 -
         6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 +
         4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 +
         8 * T2 * pow(T3, 2) * Vf) /
            (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2));
    double c1 =
        d3_val *
            (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 +
             3 * T0 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        (-A0 * pow(T0, 3) * pow(T3, 2) + A0 * T0 * pow(T1, 2) * pow(T3, 2) +
         A0 * T0 * T1 * T2 * pow(T3, 2) - 2 * Af * T0 * T1 * T2 * pow(T3, 2) -
         Af * T0 * T1 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 2) -
         2 * Af * T0 * T2 * pow(T3, 3) - 6 * P0 * T0 * pow(T3, 2) - 6 * Pf * T0 * T1 * T2 -
         6 * Pf * T0 * T1 * T3 - 6 * Pf * T0 * pow(T2, 2) - 12 * Pf * T0 * T2 * T3 -
         4 * pow(T0, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * T3 * Vf +
         4 * T0 * T1 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 2) * T3 * Vf +
         8 * T0 * T2 * pow(T3, 2) * Vf + 2 * pow(T1, 2) * pow(T3, 2) * V0 +
         2 * T1 * T2 * pow(T3, 2) * V0) /
            (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2));
    double d1 =
        d3_val *
            (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) +
             2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        (2 * A0 * pow(T0, 3) * T1 * pow(T3, 2) + A0 * pow(T0, 3) * T2 * pow(T3, 2) +
         2 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) +
         2 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 2 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) -
         Af * pow(T0, 2) * T1 * pow(T3, 3) - 2 * Af * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) -
         2 * Af * pow(T0, 2) * T2 * pow(T3, 3) + 12 * P0 * T0 * T1 * pow(T3, 2) +
         6 * P0 * T0 * T2 * pow(T3, 2) + 6 * P0 * pow(T1, 2) * pow(T3, 2) +
         6 * P0 * T1 * T2 * pow(T3, 2) - 6 * Pf * pow(T0, 2) * T1 * T2 -
         6 * Pf * pow(T0, 2) * T1 * T3 - 6 * Pf * pow(T0, 2) * pow(T2, 2) -
         12 * Pf * pow(T0, 2) * T2 * T3 + 6 * pow(T0, 2) * T1 * T2 * T3 * Vf +
         8 * pow(T0, 2) * T1 * pow(T3, 2) * V0 + 4 * pow(T0, 2) * T1 * pow(T3, 2) * Vf +
         6 * pow(T0, 2) * pow(T2, 2) * T3 * Vf + 4 * pow(T0, 2) * T2 * pow(T3, 2) * V0 +
         8 * pow(T0, 2) * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T1, 2) * pow(T3, 2) * V0 +
         6 * T0 * T1 * T2 * pow(T3, 2) * V0) /
            (6 * pow(T0, 2) * pow(T3, 2) + 12 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) +
             6 * pow(T1, 2) * pow(T3, 2) + 6 * T1 * T2 * pow(T3, 2));
    double a2 =
        d3_val *
            (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 +
             3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) +
             pow(T2, 3) * pow(T3, 2)) +
        (-A0 * pow(T0, 2) * pow(T3, 2) - A0 * T0 * T1 * pow(T3, 2) - 2 * Af * T0 * T1 * pow(T3, 2) -
         4 * Af * T0 * T2 * pow(T3, 2) - Af * T0 * pow(T3, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) -
         8 * Af * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * pow(T3, 3) -
         6 * Af * pow(T2, 2) * pow(T3, 2) - 3 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) -
         6 * Pf * T0 * T1 - 12 * Pf * T0 * T2 - 6 * Pf * T0 * T3 - 6 * Pf * pow(T1, 2) -
         24 * Pf * T1 * T2 - 12 * Pf * T1 * T3 - 18 * Pf * pow(T2, 2) - 18 * Pf * T2 * T3 +
         6 * T0 * T1 * T3 * Vf + 12 * T0 * T2 * T3 * Vf - 4 * T0 * pow(T3, 2) * V0 +
         4 * T0 * pow(T3, 2) * Vf + 6 * pow(T1, 2) * T3 * Vf + 24 * T1 * T2 * T3 * Vf -
         2 * T1 * pow(T3, 2) * V0 + 8 * T1 * pow(T3, 2) * Vf + 18 * pow(T2, 2) * T3 * Vf +
         12 * T2 * pow(T3, 2) * Vf) /
            (6 * T0 * T1 * T2 * pow(T3, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) +
             6 * pow(T2, 3) * pow(T3, 2));
    double b2 =
        d3_val *
            (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 -
             3 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        (A0 * pow(T0, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T3, 2) + 2 * Af * T0 * T2 * pow(T3, 2) +
         Af * T0 * pow(T3, 3) + 4 * Af * T1 * T2 * pow(T3, 2) + 2 * Af * T1 * pow(T3, 3) +
         4 * Af * pow(T2, 2) * pow(T3, 2) + 3 * Af * T2 * pow(T3, 3) + 6 * P0 * pow(T3, 2) +
         6 * Pf * T0 * T2 + 6 * Pf * T0 * T3 + 12 * Pf * T1 * T2 + 12 * Pf * T1 * T3 +
         12 * Pf * pow(T2, 2) + 18 * Pf * T2 * T3 - 6 * T0 * T2 * T3 * Vf +
         4 * T0 * pow(T3, 2) * V0 - 4 * T0 * pow(T3, 2) * Vf - 12 * T1 * T2 * T3 * Vf +
         2 * T1 * pow(T3, 2) * V0 - 8 * T1 * pow(T3, 2) * Vf - 12 * pow(T2, 2) * T3 * Vf -
         12 * T2 * pow(T3, 2) * Vf) /
            (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) +
             4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2));
    double c2 =
        d3_val *
            (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 +
             3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        (-A0 * pow(T0, 2) * T2 * pow(T3, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) +
         2 * Af * T0 * T1 * T2 * pow(T3, 2) + Af * T0 * T1 * pow(T3, 3) +
         2 * Af * pow(T1, 2) * T2 * pow(T3, 2) + Af * pow(T1, 2) * pow(T3, 3) -
         2 * Af * pow(T2, 3) * pow(T3, 2) - 2 * Af * pow(T2, 2) * pow(T3, 3) -
         6 * P0 * T2 * pow(T3, 2) + 6 * Pf * T0 * T1 * T2 + 6 * Pf * T0 * T1 * T3 +
         6 * Pf * pow(T1, 2) * T2 + 6 * Pf * pow(T1, 2) * T3 - 6 * Pf * pow(T2, 3) -
         12 * Pf * pow(T2, 2) * T3 - 6 * T0 * T1 * T2 * T3 * Vf - 4 * T0 * T1 * pow(T3, 2) * Vf -
         4 * T0 * T2 * pow(T3, 2) * V0 - 6 * pow(T1, 2) * T2 * T3 * Vf -
         4 * pow(T1, 2) * pow(T3, 2) * Vf - 2 * T1 * T2 * pow(T3, 2) * V0 +
         6 * pow(T2, 3) * T3 * Vf + 8 * pow(T2, 2) * pow(T3, 2) * Vf) /
            (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) +
             4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2));
    double d2 =
        d3_val *
            (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) +
             T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) -
         4 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 3 * Af * T0 * T1 * T2 * pow(T3, 3) -
         2 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 3) -
         4 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T3, 3) -
         4 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 4 * Af * T1 * pow(T2, 2) * pow(T3, 3) +
         6 * P0 * pow(T2, 2) * pow(T3, 2) - 12 * Pf * T0 * T1 * pow(T2, 2) -
         18 * Pf * T0 * T1 * T2 * T3 - 6 * Pf * T0 * pow(T2, 3) - 12 * Pf * T0 * pow(T2, 2) * T3 -
         12 * Pf * pow(T1, 2) * pow(T2, 2) - 18 * Pf * pow(T1, 2) * T2 * T3 -
         12 * Pf * T1 * pow(T2, 3) - 24 * Pf * T1 * pow(T2, 2) * T3 +
         12 * T0 * T1 * pow(T2, 2) * T3 * Vf + 12 * T0 * T1 * T2 * pow(T3, 2) * Vf +
         6 * T0 * pow(T2, 3) * T3 * Vf + 4 * T0 * pow(T2, 2) * pow(T3, 2) * V0 +
         8 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 12 * pow(T1, 2) * pow(T2, 2) * T3 * Vf +
         12 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 12 * T1 * pow(T2, 3) * T3 * Vf +
         2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 16 * T1 * pow(T2, 2) * pow(T3, 2) * Vf) /
            (6 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) +
             12 * T1 * T2 * pow(T3, 2) + 6 * pow(T2, 2) * pow(T3, 2));
    double a3 =
        -d3_val / pow(T3, 3) + (1.0 / 2.0) * (Af * pow(T3, 2) + 2 * Pf - 2 * T3 * Vf) / pow(T3, 3);
    double b3 = 3 * d3_val / pow(T3, 2) + (-Af * pow(T3, 2) - 3 * Pf + 3 * T3 * Vf) / pow(T3, 2);
    double c3 = -3 * d3_val / T3 + (1.0 / 2.0) * (Af * pow(T3, 2) + 6 * Pf - 4 * T3 * Vf) / T3;
    double d3 = d3_val;

    // Fill x_double_ with the coefficients
    x_double_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3});
  }
}

void SolverGurobi::getDependentCoefficientsN5Double() {
  // Clear the vector
  x_double_.clear();

  // get the time intervals from the optimization
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];
  double T4 = dt_[4];

  for (int axis = 0; axis < 3; axis++) {
    // get the initial and final values from the optimization
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // get free parameters
    double d3_val = d3_[axis].get(GRB_DoubleAttr_X);
    double d4_val = d4_[axis].get(GRB_DoubleAttr_X);

    // C++ Code for the coefficients:
    double a0 =
        d3_val * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) /
            (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) +
             T0 * T1 * T2 * pow(T3, 2)) +
        d4_val *
            (-2 * T1 * T2 * pow(T3, 2) - 3 * T1 * T2 * T3 * T4 - T1 * T2 * pow(T4, 2) -
             T1 * pow(T3, 3) - 2 * T1 * pow(T3, 2) * T4 - T1 * T3 * pow(T4, 2) -
             2 * pow(T2, 2) * pow(T3, 2) - 3 * pow(T2, 2) * T3 * T4 - pow(T2, 2) * pow(T4, 2) -
             2 * T2 * pow(T3, 3) - 4 * T2 * pow(T3, 2) * T4 - 2 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 3) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        (-3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 4 * A0 * T0 * T1 * T3 * pow(T4, 2) -
         2 * A0 * T0 * T2 * T3 * pow(T4, 2) - A0 * pow(T1, 2) * T3 * pow(T4, 2) -
         A0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) +
         3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) +
         2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) +
         3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) +
         4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 +
         18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 +
         12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) +
         24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf -
         12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf -
         4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf -
         12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf -
         12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 -
         16 * T2 * T3 * pow(T4, 2) * Vf) /
            (6 * pow(T0, 3) * T3 * pow(T4, 2) + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) +
             6 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) +
             6 * T0 * T1 * T2 * T3 * pow(T4, 2));
    double b0 = (1.0 / 2.0) * A0;
    double c0 = V0;
    double d0 = P0;
    double a1 =
        d3_val *
            (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 -
             2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 -
             3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) -
             pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) +
        d4_val *
            (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 +
             pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) +
             2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) +
             6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 +
             3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) +
             6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) +
             4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 +
             2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) +
             6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 +
             3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) +
             6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) +
             8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 +
             4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) +
             12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) +
             2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) +
             2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 +
             2 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) +
             3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 4) * pow(T3, 2) * pow(T4, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        (A0 * pow(T0, 3) * T3 * pow(T4, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * pow(T4, 2) +
         2 * A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 3 * A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) +
         3 * A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + A0 * T0 * pow(T2, 2) * T3 * pow(T4, 2) -
         4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T0, 2) * T2 * pow(T4, 3) -
         2 * Af * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T0, 2) * T3 * pow(T4, 3) -
         12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 9 * Af * T0 * T1 * T2 * pow(T4, 3) -
         6 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T1 * T3 * pow(T4, 3) -
         8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 3) -
         6 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T2 * T3 * pow(T4, 3) -
         12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T4, 3) -
         6 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T4, 3) -
         16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T4, 3) -
         12 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 12 * Af * T1 * T2 * T3 * pow(T4, 3) -
         4 * Af * pow(T2, 3) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 3) * pow(T4, 3) -
         4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) +
         6 * P0 * T0 * T3 * pow(T4, 2) + 12 * P0 * T1 * T3 * pow(T4, 2) +
         6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 -
         18 * Pf * pow(T0, 2) * T2 * T4 - 6 * Pf * pow(T0, 2) * pow(T3, 2) -
         12 * Pf * pow(T0, 2) * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 -
         54 * Pf * T0 * T1 * T2 * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) -
         36 * Pf * T0 * T1 * T3 * T4 - 24 * Pf * T0 * pow(T2, 2) * T3 -
         36 * Pf * T0 * pow(T2, 2) * T4 - 18 * Pf * T0 * T2 * pow(T3, 2) -
         36 * Pf * T0 * T2 * T3 * T4 - 36 * Pf * pow(T1, 2) * T2 * T3 -
         54 * Pf * pow(T1, 2) * T2 * T4 - 18 * Pf * pow(T1, 2) * pow(T3, 2) -
         36 * Pf * pow(T1, 2) * T3 * T4 - 48 * Pf * T1 * pow(T2, 2) * T3 -
         72 * Pf * T1 * pow(T2, 2) * T4 - 36 * Pf * T1 * T2 * pow(T3, 2) -
         72 * Pf * T1 * T2 * T3 * T4 - 12 * Pf * pow(T2, 3) * T3 - 18 * Pf * pow(T2, 3) * T4 -
         12 * Pf * pow(T2, 2) * pow(T3, 2) - 24 * Pf * pow(T2, 2) * T3 * T4 +
         12 * pow(T0, 2) * T2 * T3 * T4 * Vf + 12 * pow(T0, 2) * T2 * pow(T4, 2) * Vf +
         6 * pow(T0, 2) * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 +
         8 * pow(T0, 2) * T3 * pow(T4, 2) * Vf + 36 * T0 * T1 * T2 * T3 * T4 * Vf +
         36 * T0 * T1 * T2 * pow(T4, 2) * Vf + 18 * T0 * T1 * pow(T3, 2) * T4 * Vf +
         12 * T0 * T1 * T3 * pow(T4, 2) * V0 + 24 * T0 * T1 * T3 * pow(T4, 2) * Vf +
         24 * T0 * pow(T2, 2) * T3 * T4 * Vf + 24 * T0 * pow(T2, 2) * pow(T4, 2) * Vf +
         18 * T0 * T2 * pow(T3, 2) * T4 * Vf + 6 * T0 * T2 * T3 * pow(T4, 2) * V0 +
         24 * T0 * T2 * T3 * pow(T4, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * T4 * Vf +
         36 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 18 * pow(T1, 2) * pow(T3, 2) * T4 * Vf +
         6 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 24 * pow(T1, 2) * T3 * pow(T4, 2) * Vf +
         48 * T1 * pow(T2, 2) * T3 * T4 * Vf + 48 * T1 * pow(T2, 2) * pow(T4, 2) * Vf +
         36 * T1 * T2 * pow(T3, 2) * T4 * Vf + 6 * T1 * T2 * T3 * pow(T4, 2) * V0 +
         48 * T1 * T2 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 3) * T3 * T4 * Vf +
         12 * pow(T2, 3) * pow(T4, 2) * Vf + 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf +
         2 * pow(T2, 2) * T3 * pow(T4, 2) * V0 + 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) /
            (6 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) +
             6 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 12 * T0 * pow(T1, 3) * T3 * pow(T4, 2) +
             18 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) +
             6 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 4) * T3 * pow(T4, 2) +
             12 * pow(T1, 3) * T2 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2));
    double b1 =
        d3_val * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_val *
            (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) -
             3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) -
             6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        (-2 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 2 * A0 * T0 * T1 * T3 * pow(T4, 2) -
         A0 * T0 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) +
         3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) +
         2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) +
         3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) +
         4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 +
         18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 +
         12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) +
         24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf -
         12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf -
         4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf -
         12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf -
         12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 -
         16 * T2 * T3 * pow(T4, 2) * Vf) /
            (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) +
             2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2));
    double c1 =
        d3_val *
            (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 +
             3 * T0 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_val *
            (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 -
             3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) -
             6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) -
             6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 -
             3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) -
             12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        (-A0 * pow(T0, 3) * T3 * pow(T4, 2) + A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) +
         A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) +
         3 * Af * T0 * T1 * T2 * pow(T4, 3) + 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
         2 * Af * T0 * T1 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) +
         3 * Af * T0 * pow(T2, 2) * pow(T4, 3) + 4 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) +
         4 * Af * T0 * T2 * T3 * pow(T4, 3) - 6 * P0 * T0 * T3 * pow(T4, 2) +
         12 * Pf * T0 * T1 * T2 * T3 + 18 * Pf * T0 * T1 * T2 * T4 + 6 * Pf * T0 * T1 * pow(T3, 2) +
         12 * Pf * T0 * T1 * T3 * T4 + 12 * Pf * T0 * pow(T2, 2) * T3 +
         18 * Pf * T0 * pow(T2, 2) * T4 + 12 * Pf * T0 * T2 * pow(T3, 2) +
         24 * Pf * T0 * T2 * T3 * T4 - 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 -
         12 * T0 * T1 * T2 * T3 * T4 * Vf - 12 * T0 * T1 * T2 * pow(T4, 2) * Vf -
         6 * T0 * T1 * pow(T3, 2) * T4 * Vf - 8 * T0 * T1 * T3 * pow(T4, 2) * Vf -
         12 * T0 * pow(T2, 2) * T3 * T4 * Vf - 12 * T0 * pow(T2, 2) * pow(T4, 2) * Vf -
         12 * T0 * T2 * pow(T3, 2) * T4 * Vf - 16 * T0 * T2 * T3 * pow(T4, 2) * Vf +
         2 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 2 * T1 * T2 * T3 * pow(T4, 2) * V0) /
            (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) +
             2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2));
    double d1 =
        d3_val *
            (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) +
             2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_val *
            (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 -
             pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) -
             2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) -
             2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 -
             pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) -
             4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        (2 * A0 * pow(T0, 3) * T1 * T3 * pow(T4, 2) + A0 * pow(T0, 3) * T2 * T3 * pow(T4, 2) +
         2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) +
         2 * A0 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) +
         4 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) +
         3 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 3) +
         2 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) +
         2 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 3) +
         4 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
         3 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) +
         4 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
         4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 12 * P0 * T0 * T1 * T3 * pow(T4, 2) +
         6 * P0 * T0 * T2 * T3 * pow(T4, 2) + 6 * P0 * pow(T1, 2) * T3 * pow(T4, 2) +
         6 * P0 * T1 * T2 * T3 * pow(T4, 2) + 12 * Pf * pow(T0, 2) * T1 * T2 * T3 +
         18 * Pf * pow(T0, 2) * T1 * T2 * T4 + 6 * Pf * pow(T0, 2) * T1 * pow(T3, 2) +
         12 * Pf * pow(T0, 2) * T1 * T3 * T4 + 12 * Pf * pow(T0, 2) * pow(T2, 2) * T3 +
         18 * Pf * pow(T0, 2) * pow(T2, 2) * T4 + 12 * Pf * pow(T0, 2) * T2 * pow(T3, 2) +
         24 * Pf * pow(T0, 2) * T2 * T3 * T4 - 12 * pow(T0, 2) * T1 * T2 * T3 * T4 * Vf -
         12 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * Vf - 6 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * Vf +
         8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * V0 - 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * Vf -
         12 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * Vf -
         12 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * Vf -
         12 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * V0 -
         16 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * V0 +
         6 * T0 * T1 * T2 * T3 * pow(T4, 2) * V0) /
            (6 * pow(T0, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * T3 * pow(T4, 2) +
             6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) +
             6 * T1 * T2 * T3 * pow(T4, 2));
    double a2 =
        d3_val *
            (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 +
             3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) +
             pow(T2, 3) * pow(T3, 2)) +
        d4_val *
            (-2 * T0 * T1 * pow(T3, 2) - 3 * T0 * T1 * T3 * T4 - T0 * T1 * pow(T4, 2) -
             4 * T0 * T2 * pow(T3, 2) - 6 * T0 * T2 * T3 * T4 - 2 * T0 * T2 * pow(T4, 2) -
             T0 * pow(T3, 3) - 2 * T0 * pow(T3, 2) * T4 - T0 * T3 * pow(T4, 2) -
             2 * pow(T1, 2) * pow(T3, 2) - 3 * pow(T1, 2) * T3 * T4 - pow(T1, 2) * pow(T4, 2) -
             8 * T1 * T2 * pow(T3, 2) - 12 * T1 * T2 * T3 * T4 - 4 * T1 * T2 * pow(T4, 2) -
             2 * T1 * pow(T3, 3) - 4 * T1 * pow(T3, 2) * T4 - 2 * T1 * T3 * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) -
             3 * T2 * pow(T3, 3) - 6 * T2 * pow(T3, 2) * T4 - 3 * T2 * T3 * pow(T4, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T2, 3) * pow(T3, 2) * pow(T4, 2)) +
        (-A0 * pow(T0, 2) * T3 * pow(T4, 2) - A0 * T0 * T1 * T3 * pow(T4, 2) +
         4 * Af * T0 * T1 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * pow(T4, 3) +
         8 * Af * T0 * T2 * T3 * pow(T4, 2) + 6 * Af * T0 * T2 * pow(T4, 3) +
         2 * Af * T0 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T3 * pow(T4, 3) +
         4 * Af * pow(T1, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T1, 2) * pow(T4, 3) +
         16 * Af * T1 * T2 * T3 * pow(T4, 2) + 12 * Af * T1 * T2 * pow(T4, 3) +
         4 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T1 * T3 * pow(T4, 3) +
         12 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 9 * Af * pow(T2, 2) * pow(T4, 3) +
         6 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T2 * T3 * pow(T4, 3) -
         6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T3 + 18 * Pf * T0 * T1 * T4 +
         24 * Pf * T0 * T2 * T3 + 36 * Pf * T0 * T2 * T4 + 6 * Pf * T0 * pow(T3, 2) +
         12 * Pf * T0 * T3 * T4 + 12 * Pf * pow(T1, 2) * T3 + 18 * Pf * pow(T1, 2) * T4 +
         48 * Pf * T1 * T2 * T3 + 72 * Pf * T1 * T2 * T4 + 12 * Pf * T1 * pow(T3, 2) +
         24 * Pf * T1 * T3 * T4 + 36 * Pf * pow(T2, 2) * T3 + 54 * Pf * pow(T2, 2) * T4 +
         18 * Pf * T2 * pow(T3, 2) + 36 * Pf * T2 * T3 * T4 - 12 * T0 * T1 * T3 * T4 * Vf -
         12 * T0 * T1 * pow(T4, 2) * Vf - 24 * T0 * T2 * T3 * T4 * Vf -
         24 * T0 * T2 * pow(T4, 2) * Vf - 6 * T0 * pow(T3, 2) * T4 * Vf -
         4 * T0 * T3 * pow(T4, 2) * V0 - 8 * T0 * T3 * pow(T4, 2) * Vf -
         12 * pow(T1, 2) * T3 * T4 * Vf - 12 * pow(T1, 2) * pow(T4, 2) * Vf -
         48 * T1 * T2 * T3 * T4 * Vf - 48 * T1 * T2 * pow(T4, 2) * Vf -
         12 * T1 * pow(T3, 2) * T4 * Vf - 2 * T1 * T3 * pow(T4, 2) * V0 -
         16 * T1 * T3 * pow(T4, 2) * Vf - 36 * pow(T2, 2) * T3 * T4 * Vf -
         36 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T2 * pow(T3, 2) * T4 * Vf -
         24 * T2 * T3 * pow(T4, 2) * Vf) /
            (6 * T0 * T1 * T2 * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 2) * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 2) * T3 * pow(T4, 2) +
             6 * pow(T2, 3) * T3 * pow(T4, 2));
    double b2 =
        d3_val *
            (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 -
             3 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_val *
            (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) +
             3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) +
             12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) +
             6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) +
             12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 +
             6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 +
             9 * T2 * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        (A0 * pow(T0, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T3 * pow(T4, 2) -
         4 * Af * T0 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T2 * pow(T4, 3) -
         2 * Af * T0 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T3 * pow(T4, 3) -
         8 * Af * T1 * T2 * T3 * pow(T4, 2) - 6 * Af * T1 * T2 * pow(T4, 3) -
         4 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T1 * T3 * pow(T4, 3) -
         8 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * pow(T2, 2) * pow(T4, 3) -
         6 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T2 * T3 * pow(T4, 3) +
         6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T0 * T2 * T3 - 18 * Pf * T0 * T2 * T4 -
         6 * Pf * T0 * pow(T3, 2) - 12 * Pf * T0 * T3 * T4 - 24 * Pf * T1 * T2 * T3 -
         36 * Pf * T1 * T2 * T4 - 12 * Pf * T1 * pow(T3, 2) - 24 * Pf * T1 * T3 * T4 -
         24 * Pf * pow(T2, 2) * T3 - 36 * Pf * pow(T2, 2) * T4 - 18 * Pf * T2 * pow(T3, 2) -
         36 * Pf * T2 * T3 * T4 + 12 * T0 * T2 * T3 * T4 * Vf + 12 * T0 * T2 * pow(T4, 2) * Vf +
         6 * T0 * pow(T3, 2) * T4 * Vf + 4 * T0 * T3 * pow(T4, 2) * V0 +
         8 * T0 * T3 * pow(T4, 2) * Vf + 24 * T1 * T2 * T3 * T4 * Vf +
         24 * T1 * T2 * pow(T4, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * Vf +
         2 * T1 * T3 * pow(T4, 2) * V0 + 16 * T1 * T3 * pow(T4, 2) * Vf +
         24 * pow(T2, 2) * T3 * T4 * Vf + 24 * pow(T2, 2) * pow(T4, 2) * Vf +
         18 * T2 * pow(T3, 2) * T4 * Vf + 24 * T2 * T3 * pow(T4, 2) * Vf) /
            (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) +
             2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) +
             2 * pow(T2, 2) * T3 * pow(T4, 2));
    double c2 =
        d3_val *
            (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 +
             3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_val *
            (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 +
             3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) +
             6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 +
             3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) +
             6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) -
             6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 -
             6 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        (-A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - A0 * T0 * T1 * T2 * T3 * pow(T4, 2) -
         4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T1 * T2 * pow(T4, 3) -
         2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T1 * T3 * pow(T4, 3) -
         4 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T4, 3) -
         2 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T1, 2) * T3 * pow(T4, 3) +
         4 * Af * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 3) * pow(T4, 3) +
         4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) -
         6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * T0 * T1 * T2 * T3 - 18 * Pf * T0 * T1 * T2 * T4 -
         6 * Pf * T0 * T1 * pow(T3, 2) - 12 * Pf * T0 * T1 * T3 * T4 -
         12 * Pf * pow(T1, 2) * T2 * T3 - 18 * Pf * pow(T1, 2) * T2 * T4 -
         6 * Pf * pow(T1, 2) * pow(T3, 2) - 12 * Pf * pow(T1, 2) * T3 * T4 +
         12 * Pf * pow(T2, 3) * T3 + 18 * Pf * pow(T2, 3) * T4 + 12 * Pf * pow(T2, 2) * pow(T3, 2) +
         24 * Pf * pow(T2, 2) * T3 * T4 + 12 * T0 * T1 * T2 * T3 * T4 * Vf +
         12 * T0 * T1 * T2 * pow(T4, 2) * Vf + 6 * T0 * T1 * pow(T3, 2) * T4 * Vf +
         8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 4 * T0 * T2 * T3 * pow(T4, 2) * V0 +
         12 * pow(T1, 2) * T2 * T3 * T4 * Vf + 12 * pow(T1, 2) * T2 * pow(T4, 2) * Vf +
         6 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 8 * pow(T1, 2) * T3 * pow(T4, 2) * Vf -
         2 * T1 * T2 * T3 * pow(T4, 2) * V0 - 12 * pow(T2, 3) * T3 * T4 * Vf -
         12 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf -
         16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) /
            (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) +
             2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) +
             2 * pow(T2, 2) * T3 * pow(T4, 2));
    double d2 =
        d3_val *
            (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) +
             T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_val *
            (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 -
             2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) -
             6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) -
             2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 -
             T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) -
             4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) -
             4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 -
             2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) -
             6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) -
             4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 -
             2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) -
             8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        (A0 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
         A0 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) +
         8 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) +
         6 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 3) +
         6 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) +
         4 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 3) * pow(T4, 3) +
         4 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
         4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) +
         8 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
         6 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) +
         6 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
         6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 8 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) +
         6 * Af * T1 * pow(T2, 3) * pow(T4, 3) +
         8 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
         8 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * pow(T2, 2) * T3 * pow(T4, 2) +
         24 * Pf * T0 * T1 * pow(T2, 2) * T3 + 36 * Pf * T0 * T1 * pow(T2, 2) * T4 +
         18 * Pf * T0 * T1 * T2 * pow(T3, 2) + 36 * Pf * T0 * T1 * T2 * T3 * T4 +
         12 * Pf * T0 * pow(T2, 3) * T3 + 18 * Pf * T0 * pow(T2, 3) * T4 +
         12 * Pf * T0 * pow(T2, 2) * pow(T3, 2) + 24 * Pf * T0 * pow(T2, 2) * T3 * T4 +
         24 * Pf * pow(T1, 2) * pow(T2, 2) * T3 + 36 * Pf * pow(T1, 2) * pow(T2, 2) * T4 +
         18 * Pf * pow(T1, 2) * T2 * pow(T3, 2) + 36 * Pf * pow(T1, 2) * T2 * T3 * T4 +
         24 * Pf * T1 * pow(T2, 3) * T3 + 36 * Pf * T1 * pow(T2, 3) * T4 +
         24 * Pf * T1 * pow(T2, 2) * pow(T3, 2) + 48 * Pf * T1 * pow(T2, 2) * T3 * T4 -
         24 * T0 * T1 * pow(T2, 2) * T3 * T4 * Vf - 24 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * Vf -
         18 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf -
         12 * T0 * pow(T2, 3) * T3 * T4 * Vf - 12 * T0 * pow(T2, 3) * pow(T4, 2) * Vf -
         12 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * V0 -
         16 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 24 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * Vf -
         24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * Vf -
         18 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf -
         24 * T1 * pow(T2, 3) * T3 * T4 * Vf - 24 * T1 * pow(T2, 3) * pow(T4, 2) * Vf -
         24 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * V0 -
         32 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) /
            (6 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T3 * pow(T4, 2) + 12 * T1 * T2 * T3 * pow(T4, 2) +
             6 * pow(T2, 2) * T3 * pow(T4, 2));
    double a3 = (1.0 / 2.0) *
                    (-2 * Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 6 * Pf * T3 - 6 * Pf * T4 +
                     6 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) /
                    (pow(T3, 2) * pow(T4, 2)) -
                d3_val / pow(T3, 3) +
                d4_val * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2)) / (pow(T3, 3) * pow(T4, 2));
    double b3 =
        (1.0 / 2.0) *
            (4 * Af * T3 * pow(T4, 2) + 3 * Af * pow(T4, 3) + 12 * Pf * T3 + 18 * Pf * T4 -
             12 * T3 * T4 * Vf - 12 * pow(T4, 2) * Vf) /
            (T3 * pow(T4, 2)) +
        3 * d3_val / pow(T3, 2) +
        d4_val * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2));
    double c3 = (-Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 3 * Pf * T3 - 6 * Pf * T4 +
                 3 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) /
                    pow(T4, 2) -
                3 * d3_val / T3 +
                d4_val * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2));
    double d3 = d3_val;
    double a4 =
        -d4_val / pow(T4, 3) + (1.0 / 2.0) * (Af * pow(T4, 2) + 2 * Pf - 2 * T4 * Vf) / pow(T4, 3);
    double b4 = 3 * d4_val / pow(T4, 2) + (-Af * pow(T4, 2) - 3 * Pf + 3 * T4 * Vf) / pow(T4, 2);
    double c4 = -3 * d4_val / T4 + (1.0 / 2.0) * (Af * pow(T4, 2) + 6 * Pf - 4 * T4 * Vf) / T4;
    double d4 = d4_val;

    // Fill x_double_ with the coefficients
    x_double_.push_back(
        {a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3, a4, b4, c4, d4});
  }
}

void SolverGurobi::getDependentCoefficientsN6Double() {
  // Clear the vector
  x_double_.clear();

  // get the time intervals from the optimization
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];
  double T4 = dt_[4];
  double T5 = dt_[5];

  for (int axis = 0; axis < 3; axis++) {
    // get the initial and final values from the optimization
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // get free parameters
    double d3_val = d3_[axis].get(GRB_DoubleAttr_X);
    double d4_val = d4_[axis].get(GRB_DoubleAttr_X);
    double d5_val = d5_[axis].get(GRB_DoubleAttr_X);

    // C++ Code for the coefficients:
    double a0 =
        d3_val * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) /
            (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) +
             T0 * T1 * T2 * pow(T3, 2)) +
        d4_val *
            (-2 * T1 * T2 * pow(T3, 2) - 3 * T1 * T2 * T3 * T4 - T1 * T2 * pow(T4, 2) -
             T1 * pow(T3, 3) - 2 * T1 * pow(T3, 2) * T4 - T1 * T3 * pow(T4, 2) -
             2 * pow(T2, 2) * pow(T3, 2) - 3 * pow(T2, 2) * T3 * T4 - pow(T2, 2) * pow(T4, 2) -
             2 * T2 * pow(T3, 3) - 4 * T2 * pow(T3, 2) * T4 - 2 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 3) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) +
             pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 +
             2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) +
             6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) +
             2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 +
             T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) +
             4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) +
             4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 +
             2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) +
             6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) +
             4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 +
             2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) +
             8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 3) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             T0 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) -
         2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) -
         A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) -
         6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) -
         3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) -
         4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) -
         24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 -
         18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 -
         12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 -
         12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 -
         24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 -
         18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 -
         24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 -
         24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 -
         6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf +
         24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf +
         24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf +
         12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf -
         4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf +
         24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 +
         32 * T2 * T3 * T4 * pow(T5, 2) * Vf) /
            (6 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 12 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) +
             6 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) +
             6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2));
    double b0 = (1.0 / 2.0) * A0;
    double c0 = V0;
    double d0 = P0;
    double a1 =
        d3_val *
            (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 -
             2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 -
             3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) -
             pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) +
        d4_val *
            (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 +
             pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) +
             2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) +
             6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 +
             3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) +
             6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) +
             4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 +
             2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) +
             6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 +
             3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) +
             6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) +
             8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 +
             4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) +
             12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) +
             2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) +
             2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 +
             2 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             2 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) +
             3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 4) * pow(T3, 2) * pow(T4, 2) +
             2 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (-4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 6 * pow(T0, 2) * T2 * T3 * T4 * T5 -
             2 * pow(T0, 2) * T2 * T3 * pow(T5, 2) - 3 * pow(T0, 2) * T2 * pow(T4, 3) -
             6 * pow(T0, 2) * T2 * pow(T4, 2) * T5 - 3 * pow(T0, 2) * T2 * T4 * pow(T5, 2) -
             2 * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 3 * pow(T0, 2) * pow(T3, 2) * T4 * T5 -
             pow(T0, 2) * pow(T3, 2) * pow(T5, 2) - 2 * pow(T0, 2) * T3 * pow(T4, 3) -
             4 * pow(T0, 2) * T3 * pow(T4, 2) * T5 - 2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) -
             12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 -
             6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) -
             18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) -
             6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 -
             3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) -
             12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) -
             8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * T0 * pow(T2, 2) * T3 * T4 * T5 -
             4 * T0 * pow(T2, 2) * T3 * pow(T5, 2) - 6 * T0 * pow(T2, 2) * pow(T4, 3) -
             12 * T0 * pow(T2, 2) * pow(T4, 2) * T5 - 6 * T0 * pow(T2, 2) * T4 * pow(T5, 2) -
             6 * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T2 * pow(T3, 2) * T4 * T5 -
             3 * T0 * T2 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T2 * T3 * pow(T4, 3) -
             12 * T0 * T2 * T3 * pow(T4, 2) * T5 - 6 * T0 * T2 * T3 * T4 * pow(T5, 2) -
             12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 -
             6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) -
             18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) -
             6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 -
             3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) -
             12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) -
             16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 24 * T1 * pow(T2, 2) * T3 * T4 * T5 -
             8 * T1 * pow(T2, 2) * T3 * pow(T5, 2) - 12 * T1 * pow(T2, 2) * pow(T4, 3) -
             24 * T1 * pow(T2, 2) * pow(T4, 2) * T5 - 12 * T1 * pow(T2, 2) * T4 * pow(T5, 2) -
             12 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * T2 * pow(T3, 2) * T4 * T5 -
             6 * T1 * T2 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T2 * T3 * pow(T4, 3) -
             24 * T1 * T2 * T3 * pow(T4, 2) * T5 - 12 * T1 * T2 * T3 * T4 * pow(T5, 2) -
             4 * pow(T2, 3) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * T3 * T4 * T5 -
             2 * pow(T2, 3) * T3 * pow(T5, 2) - 3 * pow(T2, 3) * pow(T4, 3) -
             6 * pow(T2, 3) * pow(T4, 2) * T5 - 3 * pow(T2, 3) * T4 * pow(T5, 2) -
             4 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) * T4 * T5 -
             2 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) - 4 * pow(T2, 2) * T3 * pow(T4, 3) -
             8 * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 4 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T0 * pow(T1, 3) * T3 * pow(T4, 2) * pow(T5, 2) +
             3 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 4) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * pow(T1, 3) * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) +
         2 * A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) +
         3 * A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
         3 * A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) +
         A0 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
         8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) +
         6 * Af * pow(T0, 2) * T2 * T3 * pow(T5, 3) +
         6 * Af * pow(T0, 2) * T2 * pow(T4, 2) * pow(T5, 2) +
         6 * Af * pow(T0, 2) * T2 * T4 * pow(T5, 3) +
         4 * Af * pow(T0, 2) * pow(T3, 2) * T4 * pow(T5, 2) +
         3 * Af * pow(T0, 2) * pow(T3, 2) * pow(T5, 3) +
         4 * Af * pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
         4 * Af * pow(T0, 2) * T3 * T4 * pow(T5, 3) +
         24 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) +
         18 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) +
         18 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) +
         12 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) +
         9 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) +
         12 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) +
         12 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) +
         16 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
         12 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) +
         12 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) +
         12 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) +
         12 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) +
         9 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) +
         12 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
         12 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) +
         24 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) +
         18 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) +
         18 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) +
         18 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) +
         12 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) +
         9 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) +
         12 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
         12 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) +
         32 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
         24 * Af * T1 * pow(T2, 2) * T3 * pow(T5, 3) +
         24 * Af * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) +
         24 * Af * T1 * pow(T2, 2) * T4 * pow(T5, 3) +
         24 * Af * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) +
         18 * Af * T1 * T2 * pow(T3, 2) * pow(T5, 3) +
         24 * Af * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
         24 * Af * T1 * T2 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) +
         6 * Af * pow(T2, 3) * T3 * pow(T5, 3) + 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) +
         6 * Af * pow(T2, 3) * T4 * pow(T5, 3) +
         8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) +
         6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) +
         8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
         8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * T0 * T3 * T4 * pow(T5, 2) +
         12 * P0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T2 * T3 * T4 * pow(T5, 2) +
         24 * Pf * pow(T0, 2) * T2 * T3 * T4 + 36 * Pf * pow(T0, 2) * T2 * T3 * T5 +
         18 * Pf * pow(T0, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T0, 2) * T2 * T4 * T5 +
         12 * Pf * pow(T0, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T0, 2) * pow(T3, 2) * T5 +
         12 * Pf * pow(T0, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T0, 2) * T3 * T4 * T5 +
         72 * Pf * T0 * T1 * T2 * T3 * T4 + 108 * Pf * T0 * T1 * T2 * T3 * T5 +
         54 * Pf * T0 * T1 * T2 * pow(T4, 2) + 108 * Pf * T0 * T1 * T2 * T4 * T5 +
         36 * Pf * T0 * T1 * pow(T3, 2) * T4 + 54 * Pf * T0 * T1 * pow(T3, 2) * T5 +
         36 * Pf * T0 * T1 * T3 * pow(T4, 2) + 72 * Pf * T0 * T1 * T3 * T4 * T5 +
         48 * Pf * T0 * pow(T2, 2) * T3 * T4 + 72 * Pf * T0 * pow(T2, 2) * T3 * T5 +
         36 * Pf * T0 * pow(T2, 2) * pow(T4, 2) + 72 * Pf * T0 * pow(T2, 2) * T4 * T5 +
         36 * Pf * T0 * T2 * pow(T3, 2) * T4 + 54 * Pf * T0 * T2 * pow(T3, 2) * T5 +
         36 * Pf * T0 * T2 * T3 * pow(T4, 2) + 72 * Pf * T0 * T2 * T3 * T4 * T5 +
         72 * Pf * pow(T1, 2) * T2 * T3 * T4 + 108 * Pf * pow(T1, 2) * T2 * T3 * T5 +
         54 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 108 * Pf * pow(T1, 2) * T2 * T4 * T5 +
         36 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 54 * Pf * pow(T1, 2) * pow(T3, 2) * T5 +
         36 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 72 * Pf * pow(T1, 2) * T3 * T4 * T5 +
         96 * Pf * T1 * pow(T2, 2) * T3 * T4 + 144 * Pf * T1 * pow(T2, 2) * T3 * T5 +
         72 * Pf * T1 * pow(T2, 2) * pow(T4, 2) + 144 * Pf * T1 * pow(T2, 2) * T4 * T5 +
         72 * Pf * T1 * T2 * pow(T3, 2) * T4 + 108 * Pf * T1 * T2 * pow(T3, 2) * T5 +
         72 * Pf * T1 * T2 * T3 * pow(T4, 2) + 144 * Pf * T1 * T2 * T3 * T4 * T5 +
         24 * Pf * pow(T2, 3) * T3 * T4 + 36 * Pf * pow(T2, 3) * T3 * T5 +
         18 * Pf * pow(T2, 3) * pow(T4, 2) + 36 * Pf * pow(T2, 3) * T4 * T5 +
         24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 + 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 +
         24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) + 48 * Pf * pow(T2, 2) * T3 * T4 * T5 -
         24 * pow(T0, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T0, 2) * T2 * T3 * pow(T5, 2) * Vf -
         18 * pow(T0, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T0, 2) * T2 * T4 * pow(T5, 2) * Vf -
         12 * pow(T0, 2) * pow(T3, 2) * T4 * T5 * Vf -
         12 * pow(T0, 2) * pow(T3, 2) * pow(T5, 2) * Vf -
         12 * pow(T0, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 -
         16 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * Vf - 72 * T0 * T1 * T2 * T3 * T4 * T5 * Vf -
         72 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 54 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf -
         72 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 36 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf -
         36 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf +
         12 * T0 * T1 * T3 * T4 * pow(T5, 2) * V0 - 48 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf -
         48 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf -
         36 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf -
         36 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf -
         36 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 -
         48 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf - 72 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf -
         72 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 54 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf -
         72 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 36 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf -
         36 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf -
         36 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 -
         48 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 96 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf -
         96 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 72 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf -
         96 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 72 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf -
         72 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 72 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf +
         6 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 - 96 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf -
         24 * pow(T2, 3) * T3 * T4 * T5 * Vf - 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf -
         18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf - 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf -
         24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf -
         24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf -
         24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 -
         32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) /
            (6 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             6 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) +
             12 * T0 * pow(T1, 3) * T3 * T4 * pow(T5, 2) +
             18 * T0 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) +
             6 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
             6 * pow(T1, 4) * T3 * T4 * pow(T5, 2) + 12 * pow(T1, 3) * T2 * T3 * T4 * pow(T5, 2) +
             6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    double b1 =
        d3_val * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_val *
            (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) -
             3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) -
             6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (12 * T1 * T2 * T3 * pow(T4, 2) + 18 * T1 * T2 * T3 * T4 * T5 +
             6 * T1 * T2 * T3 * pow(T5, 2) + 9 * T1 * T2 * pow(T4, 3) +
             18 * T1 * T2 * pow(T4, 2) * T5 + 9 * T1 * T2 * T4 * pow(T5, 2) +
             6 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T1 * pow(T3, 2) * T4 * T5 +
             3 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T1 * T3 * pow(T4, 3) +
             12 * T1 * T3 * pow(T4, 2) * T5 + 6 * T1 * T3 * T4 * pow(T5, 2) +
             12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 +
             6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) +
             18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) +
             12 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T2 * pow(T3, 2) * T4 * T5 +
             6 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T2 * T3 * pow(T4, 3) +
             24 * T2 * T3 * pow(T4, 2) * T5 + 12 * T2 * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-2 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) -
         A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) -
         6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) -
         3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) -
         4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) -
         24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 -
         18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 -
         12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 -
         12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 -
         24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 -
         18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 -
         24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 -
         24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 -
         6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf +
         24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf +
         24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf +
         12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf -
         4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf +
         24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 +
         32 * T2 * T3 * T4 * pow(T5, 2) * Vf) /
            (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) +
             2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             2 * T1 * T2 * T3 * T4 * pow(T5, 2));
    double c1 =
        d3_val *
            (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 +
             3 * T0 * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_val *
            (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 -
             3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) -
             6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) -
             6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 -
             3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) -
             12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (12 * T0 * T1 * T2 * T3 * pow(T4, 2) + 18 * T0 * T1 * T2 * T3 * T4 * T5 +
             6 * T0 * T1 * T2 * T3 * pow(T5, 2) + 9 * T0 * T1 * T2 * pow(T4, 3) +
             18 * T0 * T1 * T2 * pow(T4, 2) * T5 + 9 * T0 * T1 * T2 * T4 * pow(T5, 2) +
             6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * pow(T3, 2) * T4 * T5 +
             3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T3 * pow(T4, 3) +
             12 * T0 * T1 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T3 * T4 * pow(T5, 2) +
             12 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * T0 * pow(T2, 2) * T3 * T4 * T5 +
             6 * T0 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * T0 * pow(T2, 2) * pow(T4, 3) +
             18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * T0 * pow(T2, 2) * T4 * pow(T5, 2) +
             12 * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T0 * T2 * pow(T3, 2) * T4 * T5 +
             6 * T0 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T0 * T2 * T3 * pow(T4, 3) +
             24 * T0 * T2 * T3 * pow(T4, 2) * T5 + 12 * T0 * T2 * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
         A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) -
         6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) -
         3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) -
         4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) -
         8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) -
         6 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) -
         8 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) -
         8 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) -
         6 * P0 * T0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T2 * T3 * T4 -
         36 * Pf * T0 * T1 * T2 * T3 * T5 - 18 * Pf * T0 * T1 * T2 * pow(T4, 2) -
         36 * Pf * T0 * T1 * T2 * T4 * T5 - 12 * Pf * T0 * T1 * pow(T3, 2) * T4 -
         18 * Pf * T0 * T1 * pow(T3, 2) * T5 - 12 * Pf * T0 * T1 * T3 * pow(T4, 2) -
         24 * Pf * T0 * T1 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * T4 -
         36 * Pf * T0 * pow(T2, 2) * T3 * T5 - 18 * Pf * T0 * pow(T2, 2) * pow(T4, 2) -
         36 * Pf * T0 * pow(T2, 2) * T4 * T5 - 24 * Pf * T0 * T2 * pow(T3, 2) * T4 -
         36 * Pf * T0 * T2 * pow(T3, 2) * T5 - 24 * Pf * T0 * T2 * T3 * pow(T4, 2) -
         48 * Pf * T0 * T2 * T3 * T4 * T5 - 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 +
         24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf +
         18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf +
         12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf +
         12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf +
         24 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         24 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 32 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf +
         2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) /
            (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) +
             2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             2 * T1 * T2 * T3 * T4 * pow(T5, 2));
    double d1 =
        d3_val *
            (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) +
             2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) /
            (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) +
             pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) +
        d4_val *
            (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 -
             pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) -
             2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) -
             2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 -
             pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) -
             4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) /
            (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) +
             T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) +
             T1 * T2 * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (4 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 +
             2 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) + 3 * pow(T0, 2) * T1 * T2 * pow(T4, 3) +
             6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 +
             3 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) +
             2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) +
             3 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 +
             pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 3) +
             4 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 +
             2 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) +
             4 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
             6 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 +
             2 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) +
             3 * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) +
             6 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 +
             3 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) +
             4 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             6 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 +
             2 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 3) +
             8 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 +
             4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2)) /
            (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) +
        (2 * A0 * pow(T0, 3) * T1 * T3 * T4 * pow(T5, 2) +
         A0 * pow(T0, 3) * T2 * T3 * T4 * pow(T5, 2) +
         2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
         2 * A0 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) -
         8 * Af * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 3) -
         6 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 3) -
         4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) -
         3 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 3) -
         4 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) -
         4 * Af * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 3) -
         8 * Af * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 3) -
         6 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 3) -
         8 * Af * pow(T0, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 3) -
         8 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 3) +
         12 * P0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T0 * T2 * T3 * T4 * pow(T5, 2) +
         6 * P0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * P0 * T1 * T2 * T3 * T4 * pow(T5, 2) -
         24 * Pf * pow(T0, 2) * T1 * T2 * T3 * T4 - 36 * Pf * pow(T0, 2) * T1 * T2 * T3 * T5 -
         18 * Pf * pow(T0, 2) * T1 * T2 * pow(T4, 2) - 36 * Pf * pow(T0, 2) * T1 * T2 * T4 * T5 -
         12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T4 - 18 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T5 -
         12 * Pf * pow(T0, 2) * T1 * T3 * pow(T4, 2) - 24 * Pf * pow(T0, 2) * T1 * T3 * T4 * T5 -
         24 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T5 -
         18 * Pf * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) -
         36 * Pf * pow(T0, 2) * pow(T2, 2) * T4 * T5 - 24 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T4 -
         36 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T5 - 24 * Pf * pow(T0, 2) * T2 * T3 * pow(T4, 2) -
         48 * Pf * pow(T0, 2) * T2 * T3 * T4 * T5 + 24 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 * Vf +
         24 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) * Vf +
         18 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 * Vf +
         24 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) * Vf +
         12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 * Vf +
         12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) * Vf +
         12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 * Vf +
         8 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * V0 +
         16 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * Vf +
         24 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf +
         24 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         18 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf +
         24 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         24 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf +
         24 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf +
         4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * V0 +
         32 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf +
         6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 +
         6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) /
            (6 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) +
             6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             6 * T1 * T2 * T3 * T4 * pow(T5, 2));
    double a2 =
        d3_val *
            (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 +
             3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) +
             pow(T2, 3) * pow(T3, 2)) +
        d4_val *
            (-2 * T0 * T1 * pow(T3, 2) - 3 * T0 * T1 * T3 * T4 - T0 * T1 * pow(T4, 2) -
             4 * T0 * T2 * pow(T3, 2) - 6 * T0 * T2 * T3 * T4 - 2 * T0 * T2 * pow(T4, 2) -
             T0 * pow(T3, 3) - 2 * T0 * pow(T3, 2) * T4 - T0 * T3 * pow(T4, 2) -
             2 * pow(T1, 2) * pow(T3, 2) - 3 * pow(T1, 2) * T3 * T4 - pow(T1, 2) * pow(T4, 2) -
             8 * T1 * T2 * pow(T3, 2) - 12 * T1 * T2 * T3 * T4 - 4 * T1 * T2 * pow(T4, 2) -
             2 * T1 * pow(T3, 3) - 4 * T1 * pow(T3, 2) * T4 - 2 * T1 * T3 * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) -
             3 * T2 * pow(T3, 3) - 6 * T2 * pow(T3, 2) * T4 - 3 * T2 * T3 * pow(T4, 2)) /
            (T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T2, 3) * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (4 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T1 * T3 * T4 * T5 +
             2 * T0 * T1 * T3 * pow(T5, 2) + 3 * T0 * T1 * pow(T4, 3) +
             6 * T0 * T1 * pow(T4, 2) * T5 + 3 * T0 * T1 * T4 * pow(T5, 2) +
             8 * T0 * T2 * T3 * pow(T4, 2) + 12 * T0 * T2 * T3 * T4 * T5 +
             4 * T0 * T2 * T3 * pow(T5, 2) + 6 * T0 * T2 * pow(T4, 3) +
             12 * T0 * T2 * pow(T4, 2) * T5 + 6 * T0 * T2 * T4 * pow(T5, 2) +
             2 * T0 * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T3, 2) * T4 * T5 +
             T0 * pow(T3, 2) * pow(T5, 2) + 2 * T0 * T3 * pow(T4, 3) +
             4 * T0 * T3 * pow(T4, 2) * T5 + 2 * T0 * T3 * T4 * pow(T5, 2) +
             4 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * T4 * T5 +
             2 * pow(T1, 2) * T3 * pow(T5, 2) + 3 * pow(T1, 2) * pow(T4, 3) +
             6 * pow(T1, 2) * pow(T4, 2) * T5 + 3 * pow(T1, 2) * T4 * pow(T5, 2) +
             16 * T1 * T2 * T3 * pow(T4, 2) + 24 * T1 * T2 * T3 * T4 * T5 +
             8 * T1 * T2 * T3 * pow(T5, 2) + 12 * T1 * T2 * pow(T4, 3) +
             24 * T1 * T2 * pow(T4, 2) * T5 + 12 * T1 * T2 * T4 * pow(T5, 2) +
             4 * T1 * pow(T3, 2) * pow(T4, 2) + 6 * T1 * pow(T3, 2) * T4 * T5 +
             2 * T1 * pow(T3, 2) * pow(T5, 2) + 4 * T1 * T3 * pow(T4, 3) +
             8 * T1 * T3 * pow(T4, 2) * T5 + 4 * T1 * T3 * T4 * pow(T5, 2) +
             12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 +
             6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) +
             18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) +
             6 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 2) * T4 * T5 +
             3 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T2 * T3 * pow(T4, 3) +
             12 * T2 * T3 * pow(T4, 2) * T5 + 6 * T2 * T3 * T4 * pow(T5, 2)) /
            (T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T3 * T4 * pow(T5, 2) -
         8 * Af * T0 * T1 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T3 * pow(T5, 3) -
         6 * Af * T0 * T1 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T4 * pow(T5, 3) -
         16 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T2 * T3 * pow(T5, 3) -
         12 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T2 * T4 * pow(T5, 3) -
         4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) -
         4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T3 * T4 * pow(T5, 3) -
         8 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T5, 3) -
         6 * Af * pow(T1, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T1, 2) * T4 * pow(T5, 3) -
         32 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Af * T1 * T2 * T3 * pow(T5, 3) -
         24 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 24 * Af * T1 * T2 * T4 * pow(T5, 3) -
         8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) -
         8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T1 * T3 * T4 * pow(T5, 3) -
         24 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 18 * Af * pow(T2, 2) * T3 * pow(T5, 3) -
         18 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 18 * Af * pow(T2, 2) * T4 * pow(T5, 3) -
         12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) -
         12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T2 * T3 * T4 * pow(T5, 3) -
         6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T3 * T4 - 36 * Pf * T0 * T1 * T3 * T5 -
         18 * Pf * T0 * T1 * pow(T4, 2) - 36 * Pf * T0 * T1 * T4 * T5 -
         48 * Pf * T0 * T2 * T3 * T4 - 72 * Pf * T0 * T2 * T3 * T5 -
         36 * Pf * T0 * T2 * pow(T4, 2) - 72 * Pf * T0 * T2 * T4 * T5 -
         12 * Pf * T0 * pow(T3, 2) * T4 - 18 * Pf * T0 * pow(T3, 2) * T5 -
         12 * Pf * T0 * T3 * pow(T4, 2) - 24 * Pf * T0 * T3 * T4 * T5 -
         24 * Pf * pow(T1, 2) * T3 * T4 - 36 * Pf * pow(T1, 2) * T3 * T5 -
         18 * Pf * pow(T1, 2) * pow(T4, 2) - 36 * Pf * pow(T1, 2) * T4 * T5 -
         96 * Pf * T1 * T2 * T3 * T4 - 144 * Pf * T1 * T2 * T3 * T5 -
         72 * Pf * T1 * T2 * pow(T4, 2) - 144 * Pf * T1 * T2 * T4 * T5 -
         24 * Pf * T1 * pow(T3, 2) * T4 - 36 * Pf * T1 * pow(T3, 2) * T5 -
         24 * Pf * T1 * T3 * pow(T4, 2) - 48 * Pf * T1 * T3 * T4 * T5 -
         72 * Pf * pow(T2, 2) * T3 * T4 - 108 * Pf * pow(T2, 2) * T3 * T5 -
         54 * Pf * pow(T2, 2) * pow(T4, 2) - 108 * Pf * pow(T2, 2) * T4 * T5 -
         36 * Pf * T2 * pow(T3, 2) * T4 - 54 * Pf * T2 * pow(T3, 2) * T5 -
         36 * Pf * T2 * T3 * pow(T4, 2) - 72 * Pf * T2 * T3 * T4 * T5 +
         24 * T0 * T1 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T3 * pow(T5, 2) * Vf +
         18 * T0 * T1 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T4 * pow(T5, 2) * Vf +
         48 * T0 * T2 * T3 * T4 * T5 * Vf + 48 * T0 * T2 * T3 * pow(T5, 2) * Vf +
         36 * T0 * T2 * pow(T4, 2) * T5 * Vf + 48 * T0 * T2 * T4 * pow(T5, 2) * Vf +
         12 * T0 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf +
         12 * T0 * T3 * pow(T4, 2) * T5 * Vf - 4 * T0 * T3 * T4 * pow(T5, 2) * V0 +
         16 * T0 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T1, 2) * T3 * T4 * T5 * Vf +
         24 * pow(T1, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T1, 2) * pow(T4, 2) * T5 * Vf +
         24 * pow(T1, 2) * T4 * pow(T5, 2) * Vf + 96 * T1 * T2 * T3 * T4 * T5 * Vf +
         96 * T1 * T2 * T3 * pow(T5, 2) * Vf + 72 * T1 * T2 * pow(T4, 2) * T5 * Vf +
         96 * T1 * T2 * T4 * pow(T5, 2) * Vf + 24 * T1 * pow(T3, 2) * T4 * T5 * Vf +
         24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T1 * T3 * pow(T4, 2) * T5 * Vf -
         2 * T1 * T3 * T4 * pow(T5, 2) * V0 + 32 * T1 * T3 * T4 * pow(T5, 2) * Vf +
         72 * pow(T2, 2) * T3 * T4 * T5 * Vf + 72 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         54 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 72 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         36 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         36 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) /
            (6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
             6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) +
             12 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 3) * T3 * T4 * pow(T5, 2));
    double b2 =
        d3_val *
            (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 -
             3 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_val *
            (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) +
             3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) +
             12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) +
             6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) +
             12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 +
             6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 +
             9 * T2 * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (-12 * T0 * T2 * T3 * pow(T4, 2) - 18 * T0 * T2 * T3 * T4 * T5 -
             6 * T0 * T2 * T3 * pow(T5, 2) - 9 * T0 * T2 * pow(T4, 3) -
             18 * T0 * T2 * pow(T4, 2) * T5 - 9 * T0 * T2 * T4 * pow(T5, 2) -
             6 * T0 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * pow(T3, 2) * T4 * T5 -
             3 * T0 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T3 * pow(T4, 3) -
             12 * T0 * T3 * pow(T4, 2) * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) -
             24 * T1 * T2 * T3 * pow(T4, 2) - 36 * T1 * T2 * T3 * T4 * T5 -
             12 * T1 * T2 * T3 * pow(T5, 2) - 18 * T1 * T2 * pow(T4, 3) -
             36 * T1 * T2 * pow(T4, 2) * T5 - 18 * T1 * T2 * T4 * pow(T5, 2) -
             12 * T1 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * pow(T3, 2) * T4 * T5 -
             6 * T1 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T3 * pow(T4, 3) -
             24 * T1 * T3 * pow(T4, 2) * T5 - 12 * T1 * T3 * T4 * pow(T5, 2) -
             24 * pow(T2, 2) * T3 * pow(T4, 2) - 36 * pow(T2, 2) * T3 * T4 * T5 -
             12 * pow(T2, 2) * T3 * pow(T5, 2) - 18 * pow(T2, 2) * pow(T4, 3) -
             36 * pow(T2, 2) * pow(T4, 2) * T5 - 18 * pow(T2, 2) * T4 * pow(T5, 2) -
             18 * T2 * pow(T3, 2) * pow(T4, 2) - 27 * T2 * pow(T3, 2) * T4 * T5 -
             9 * T2 * pow(T3, 2) * pow(T5, 2) - 18 * T2 * T3 * pow(T4, 3) -
             36 * T2 * T3 * pow(T4, 2) * T5 - 18 * T2 * T3 * T4 * pow(T5, 2)) /
            (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T3 * T4 * pow(T5, 2) +
         8 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T2 * T3 * pow(T5, 3) +
         6 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T2 * T4 * pow(T5, 3) +
         4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) +
         4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T3 * T4 * pow(T5, 3) +
         16 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) + 12 * Af * T1 * T2 * T3 * pow(T5, 3) +
         12 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T1 * T2 * T4 * pow(T5, 3) +
         8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) +
         8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * T1 * T3 * T4 * pow(T5, 3) +
         16 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * pow(T2, 2) * T3 * pow(T5, 3) +
         12 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T2, 2) * T4 * pow(T5, 3) +
         12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) +
         12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T2 * T3 * T4 * pow(T5, 3) +
         6 * P0 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T2 * T3 * T4 + 36 * Pf * T0 * T2 * T3 * T5 +
         18 * Pf * T0 * T2 * pow(T4, 2) + 36 * Pf * T0 * T2 * T4 * T5 +
         12 * Pf * T0 * pow(T3, 2) * T4 + 18 * Pf * T0 * pow(T3, 2) * T5 +
         12 * Pf * T0 * T3 * pow(T4, 2) + 24 * Pf * T0 * T3 * T4 * T5 +
         48 * Pf * T1 * T2 * T3 * T4 + 72 * Pf * T1 * T2 * T3 * T5 +
         36 * Pf * T1 * T2 * pow(T4, 2) + 72 * Pf * T1 * T2 * T4 * T5 +
         24 * Pf * T1 * pow(T3, 2) * T4 + 36 * Pf * T1 * pow(T3, 2) * T5 +
         24 * Pf * T1 * T3 * pow(T4, 2) + 48 * Pf * T1 * T3 * T4 * T5 +
         48 * Pf * pow(T2, 2) * T3 * T4 + 72 * Pf * pow(T2, 2) * T3 * T5 +
         36 * Pf * pow(T2, 2) * pow(T4, 2) + 72 * Pf * pow(T2, 2) * T4 * T5 +
         36 * Pf * T2 * pow(T3, 2) * T4 + 54 * Pf * T2 * pow(T3, 2) * T5 +
         36 * Pf * T2 * T3 * pow(T4, 2) + 72 * Pf * T2 * T3 * T4 * T5 -
         24 * T0 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T2 * T3 * pow(T5, 2) * Vf -
         18 * T0 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T2 * T4 * pow(T5, 2) * Vf -
         12 * T0 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf -
         12 * T0 * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * T3 * T4 * pow(T5, 2) * V0 -
         16 * T0 * T3 * T4 * pow(T5, 2) * Vf - 48 * T1 * T2 * T3 * T4 * T5 * Vf -
         48 * T1 * T2 * T3 * pow(T5, 2) * Vf - 36 * T1 * T2 * pow(T4, 2) * T5 * Vf -
         48 * T1 * T2 * T4 * pow(T5, 2) * Vf - 24 * T1 * pow(T3, 2) * T4 * T5 * Vf -
         24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 24 * T1 * T3 * pow(T4, 2) * T5 * Vf +
         2 * T1 * T3 * T4 * pow(T5, 2) * V0 - 32 * T1 * T3 * T4 * pow(T5, 2) * Vf -
         48 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * pow(T2, 2) * T3 * pow(T5, 2) * Vf -
         36 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * pow(T2, 2) * T4 * pow(T5, 2) * Vf -
         36 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf -
         36 * T2 * T3 * pow(T4, 2) * T5 * Vf - 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) /
            (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) +
             2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) +
             2 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    double c2 =
        d3_val *
            (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 +
             3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_val *
            (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 +
             3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) +
             6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 +
             3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) +
             6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) -
             6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) -
             6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 -
             6 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (-12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 -
             6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) -
             18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) -
             6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 -
             3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) -
             12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) -
             12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 -
             6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) -
             18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) -
             6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 -
             3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) -
             12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) +
             12 * pow(T2, 3) * T3 * pow(T4, 2) + 18 * pow(T2, 3) * T3 * T4 * T5 +
             6 * pow(T2, 3) * T3 * pow(T5, 2) + 9 * pow(T2, 3) * pow(T4, 3) +
             18 * pow(T2, 3) * pow(T4, 2) * T5 + 9 * pow(T2, 3) * T4 * pow(T5, 2) +
             12 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 18 * pow(T2, 2) * pow(T3, 2) * T4 * T5 +
             6 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 3) +
             24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 12 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) /
            (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (-A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) +
         8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) +
         6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) +
         4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) +
         3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) +
         4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) +
         8 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) +
         6 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) +
         6 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) +
         6 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) +
         4 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) +
         3 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) +
         4 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
         4 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) -
         6 * Af * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * pow(T2, 3) * T4 * pow(T5, 3) -
         8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) -
         8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 6 * P0 * T2 * T3 * T4 * pow(T5, 2) +
         24 * Pf * T0 * T1 * T2 * T3 * T4 + 36 * Pf * T0 * T1 * T2 * T3 * T5 +
         18 * Pf * T0 * T1 * T2 * pow(T4, 2) + 36 * Pf * T0 * T1 * T2 * T4 * T5 +
         12 * Pf * T0 * T1 * pow(T3, 2) * T4 + 18 * Pf * T0 * T1 * pow(T3, 2) * T5 +
         12 * Pf * T0 * T1 * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * T3 * T4 * T5 +
         24 * Pf * pow(T1, 2) * T2 * T3 * T4 + 36 * Pf * pow(T1, 2) * T2 * T3 * T5 +
         18 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T1, 2) * T2 * T4 * T5 +
         12 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T1, 2) * pow(T3, 2) * T5 +
         12 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T1, 2) * T3 * T4 * T5 -
         24 * Pf * pow(T2, 3) * T3 * T4 - 36 * Pf * pow(T2, 3) * T3 * T5 -
         18 * Pf * pow(T2, 3) * pow(T4, 2) - 36 * Pf * pow(T2, 3) * T4 * T5 -
         24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 -
         24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * pow(T2, 2) * T3 * T4 * T5 -
         24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf -
         18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf -
         12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf -
         12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf - 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf -
         4 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 24 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf -
         24 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf -
         24 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf -
         12 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf -
         12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf - 16 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf -
         2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 + 24 * pow(T2, 3) * T3 * T4 * T5 * Vf +
         24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf +
         24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf +
         24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf +
         32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) /
            (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) +
             2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) +
             2 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    double d2 =
        d3_val *
            (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) +
             T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) +
             2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) /
            (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) +
             2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) +
        d4_val *
            (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 -
             2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) -
             6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) -
             2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 -
             T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) -
             4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) -
             4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 -
             2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) -
             6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) -
             4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 -
             2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) -
             8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) /
            (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) +
             pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) +
        d5_val *
            (8 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 +
             4 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T4, 3) +
             12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 +
             6 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) +
             6 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 +
             3 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 3) +
             12 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) +
             4 * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 3) * T3 * T4 * T5 +
             2 * T0 * pow(T2, 3) * T3 * pow(T5, 2) + 3 * T0 * pow(T2, 3) * pow(T4, 3) +
             6 * T0 * pow(T2, 3) * pow(T4, 2) * T5 + 3 * T0 * pow(T2, 3) * T4 * pow(T5, 2) +
             4 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
             6 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 +
             2 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 3) +
             8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 +
             4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
             8 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) +
             12 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 +
             4 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) +
             6 * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) +
             12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 +
             6 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) +
             6 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) +
             9 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 +
             3 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 3) +
             12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 +
             6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 3) * T3 * pow(T4, 2) +
             12 * T1 * pow(T2, 3) * T3 * T4 * T5 + 4 * T1 * pow(T2, 3) * T3 * pow(T5, 2) +
             6 * T1 * pow(T2, 3) * pow(T4, 3) + 12 * T1 * pow(T2, 3) * pow(T4, 2) * T5 +
             6 * T1 * pow(T2, 3) * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) +
             12 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 +
             4 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 8 * T1 * pow(T2, 2) * T3 * pow(T4, 3) +
             16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 +
             8 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) /
            (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) +
             2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) +
             pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) +
        (A0 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) +
         A0 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         16 * Af * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 3) -
         12 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         12 * Af * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 3) -
         12 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         9 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 3) -
         12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         12 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 3) -
         8 * Af * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 3) * T3 * pow(T5, 3) -
         6 * Af * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 3) * T4 * pow(T5, 3) -
         8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) -
         6 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) -
         8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) -
         8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 3) -
         16 * Af * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) -
         12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 3) -
         12 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) -
         12 * Af * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 3) -
         12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) -
         9 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 3) -
         12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) -
         12 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 3) -
         16 * Af * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) -
         12 * Af * T1 * pow(T2, 3) * T3 * pow(T5, 3) -
         12 * Af * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) -
         12 * Af * T1 * pow(T2, 3) * T4 * pow(T5, 3) -
         16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) -
         12 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) -
         16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) -
         16 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 3) +
         6 * P0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 48 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 -
         72 * Pf * T0 * T1 * pow(T2, 2) * T3 * T5 - 36 * Pf * T0 * T1 * pow(T2, 2) * pow(T4, 2) -
         72 * Pf * T0 * T1 * pow(T2, 2) * T4 * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 -
         54 * Pf * T0 * T1 * T2 * pow(T3, 2) * T5 - 36 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) -
         72 * Pf * T0 * T1 * T2 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 3) * T3 * T4 -
         36 * Pf * T0 * pow(T2, 3) * T3 * T5 - 18 * Pf * T0 * pow(T2, 3) * pow(T4, 2) -
         36 * Pf * T0 * pow(T2, 3) * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 -
         36 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 2) -
         48 * Pf * T0 * pow(T2, 2) * T3 * T4 * T5 - 48 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 -
         72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T5 -
         36 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) -
         72 * Pf * pow(T1, 2) * pow(T2, 2) * T4 * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 -
         54 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T5 - 36 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) -
         72 * Pf * pow(T1, 2) * T2 * T3 * T4 * T5 - 48 * Pf * T1 * pow(T2, 3) * T3 * T4 -
         72 * Pf * T1 * pow(T2, 3) * T3 * T5 - 36 * Pf * T1 * pow(T2, 3) * pow(T4, 2) -
         72 * Pf * T1 * pow(T2, 3) * T4 * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 -
         72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T5 - 48 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 2) -
         96 * Pf * T1 * pow(T2, 2) * T3 * T4 * T5 + 48 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf +
         48 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf +
         48 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         36 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf +
         36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         36 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf +
         48 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 3) * T3 * T4 * T5 * Vf +
         24 * T0 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 3) * pow(T4, 2) * T5 * Vf +
         24 * T0 * pow(T2, 3) * T4 * pow(T5, 2) * Vf +
         24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf +
         24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf +
         24 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf +
         4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 +
         32 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf +
         48 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf +
         48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf +
         36 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf +
         48 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf +
         36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf +
         36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf +
         36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf +
         48 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf +
         48 * T1 * pow(T2, 3) * T3 * T4 * T5 * Vf + 48 * T1 * pow(T2, 3) * T3 * pow(T5, 2) * Vf +
         36 * T1 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 3) * T4 * pow(T5, 2) * Vf +
         48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf +
         48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf +
         48 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf +
         2 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 +
         64 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) /
            (6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) +
             6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * T1 * T2 * T3 * T4 * pow(T5, 2) +
             6 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    double a3 =
        (1.0 / 2.0) *
            (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) +
             2 * Af * pow(T4, 2) * pow(T5, 2) + 2 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 +
             18 * Pf * T3 * T5 + 6 * Pf * pow(T4, 2) + 12 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf -
             12 * T3 * pow(T5, 2) * Vf - 6 * pow(T4, 2) * T5 * Vf - 8 * T4 * pow(T5, 2) * Vf) /
            (pow(T3, 2) * T4 * pow(T5, 2)) +
        d5_val *
            (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 3 * pow(T4, 3) -
             6 * pow(T4, 2) * T5 - 3 * T4 * pow(T5, 2)) /
            (pow(T3, 2) * pow(T4, 2) * pow(T5, 2)) -
        d3_val / pow(T3, 3) +
        d4_val * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2)) / (pow(T3, 3) * pow(T4, 2));
    double b3 =
        (-4 * Af * T3 * T4 * pow(T5, 2) - 3 * Af * T3 * pow(T5, 3) -
         3 * Af * pow(T4, 2) * pow(T5, 2) - 3 * Af * T4 * pow(T5, 3) - 12 * Pf * T3 * T4 -
         18 * Pf * T3 * T5 - 9 * Pf * pow(T4, 2) - 18 * Pf * T4 * T5 + 12 * T3 * T4 * T5 * Vf +
         12 * T3 * pow(T5, 2) * Vf + 9 * pow(T4, 2) * T5 * Vf + 12 * T4 * pow(T5, 2) * Vf) /
            (T3 * T4 * pow(T5, 2)) +
        d5_val *
            (12 * T3 * pow(T4, 2) + 18 * T3 * T4 * T5 + 6 * T3 * pow(T5, 2) + 9 * pow(T4, 3) +
             18 * pow(T4, 2) * T5 + 9 * T4 * pow(T5, 2)) /
            (T3 * pow(T4, 2) * pow(T5, 2)) +
        3 * d3_val / pow(T3, 2) +
        d4_val * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2));
    double c3 =
        (1.0 / 2.0) *
            (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) +
             4 * Af * pow(T4, 2) * pow(T5, 2) + 4 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 +
             18 * Pf * T3 * T5 + 12 * Pf * pow(T4, 2) + 24 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf -
             12 * T3 * pow(T5, 2) * Vf - 12 * pow(T4, 2) * T5 * Vf - 16 * T4 * pow(T5, 2) * Vf) /
            (T4 * pow(T5, 2)) +
        d5_val *
            (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 6 * pow(T4, 3) -
             12 * pow(T4, 2) * T5 - 6 * T4 * pow(T5, 2)) /
            (pow(T4, 2) * pow(T5, 2)) -
        3 * d3_val / T3 +
        d4_val * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2));
    double d3 = d3_val;
    double a4 = (1.0 / 2.0) *
                    (-2 * Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 6 * Pf * T4 - 6 * Pf * T5 +
                     6 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) /
                    (pow(T4, 2) * pow(T5, 2)) -
                d4_val / pow(T4, 3) +
                d5_val * (3 * pow(T4, 2) + 3 * T4 * T5 + pow(T5, 2)) / (pow(T4, 3) * pow(T5, 2));
    double b4 =
        (1.0 / 2.0) *
            (4 * Af * T4 * pow(T5, 2) + 3 * Af * pow(T5, 3) + 12 * Pf * T4 + 18 * Pf * T5 -
             12 * T4 * T5 * Vf - 12 * pow(T5, 2) * Vf) /
            (T4 * pow(T5, 2)) +
        3 * d4_val / pow(T4, 2) +
        d5_val * (-6 * pow(T4, 2) - 9 * T4 * T5 - 3 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2));
    double c4 = (-Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 3 * Pf * T4 - 6 * Pf * T5 +
                 3 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) /
                    pow(T5, 2) -
                3 * d4_val / T4 +
                d5_val * (3 * pow(T4, 2) + 6 * T4 * T5 + 3 * pow(T5, 2)) / (T4 * pow(T5, 2));
    double d4 = d4_val;
    double a5 =
        -d5_val / pow(T5, 3) + (1.0 / 2.0) * (Af * pow(T5, 2) + 2 * Pf - 2 * T5 * Vf) / pow(T5, 3);
    double b5 = 3 * d5_val / pow(T5, 2) + (-Af * pow(T5, 2) - 3 * Pf + 3 * T5 * Vf) / pow(T5, 2);
    double c5 = -3 * d5_val / T5 + (1.0 / 2.0) * (Af * pow(T5, 2) + 6 * Pf - 4 * T5 * Vf) / T5;
    double d5 = d5_val;

    // Fill x_double_ with the coefficients
    x_double_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2,
                         a3, b3, c3, d3, a4, b4, c4, d4, a5, b5, c5, d5});
  }
}

void SolverGurobi::removeVars() {
  GRBVar* vars = 0;
  vars = m_.getVars();
  for (int i = 0; i < m_.get(GRB_IntAttr_NumVars); ++i) {
    m_.remove(vars[i]);
  }
  x_.clear();
  x_double_.clear();
}

// Given the time in the whole trajectory, find the interval in which the time is located and the dt
// within the interval
void SolverGurobi::findIntervalIdxAndDt(
    double time_in_whole_traj, int& interval_idx, double& dt_interval) {
  // Find the interval and the time within the interval
  double dt_sum = 0.0;
  for (int interval = 0; interval < N_; interval++) {
    dt_sum += dt_[interval];

    if (time_in_whole_traj < dt_sum) {
      interval_idx = interval;
      break;
    }
  }

  if (interval_idx == 0) {
    dt_interval = time_in_whole_traj;
  } else {
    dt_interval = time_in_whole_traj - (dt_sum - dt_[interval_idx]);
  }
}

void SolverGurobi::setObjective() {
  GRBQuadExpr jerk_smooth_cost = 0.0;

  // Control input cost (jerk)
  for (int t = 0; t < N_; t++) {
    std::vector<GRBLinExpr> ut = {getJerk(t, 0, 0), getJerk(t, 0, 1), getJerk(t, 0, 2)};
    jerk_smooth_cost += GetNorm2(ut);  // ||jerk||^2
  }

  // Objective: minimize jerk smoothness
  GRBQuadExpr obj = jerk_smooth_weight_ * jerk_smooth_cost;

  m_.setObjective(obj, GRB_MINIMIZE);
}

void SolverGurobi::findClosestIndexFromTime(
    const double t, int& index, const std::vector<double>& time) {
  // Find the closest index from the time vector
  double min_diff = std::numeric_limits<double>::max();
  for (int i = 0; i < time.size(); i++) {
    double diff = std::abs(time[i] - t);
    if (diff < min_diff) {
      min_diff = diff;
      index = i;
    }
  }
}

void SolverGurobi::checkDynamicViolation(bool& is_dyn_constraints_satisfied) {
  // Check if the time allocation satisfies the dynamic constraints
  is_dyn_constraints_satisfied = true;
  for (int segment = 0; segment < N_; segment++) {
    /// velocity
    Eigen::Matrix<double, 3, 3> Vn_prime = getMinvoVelControlPointsDouble(segment);

    // for control points (colunmns of Vn_prime)
    for (int i = 0; i < 3; i++) {
      if (Vn_prime.col(i)[0] < -v_max_ || Vn_prime.col(i)[0] > v_max_ ||
          Vn_prime.col(i)[1] < -v_max_ || Vn_prime.col(i)[1] > v_max_ ||
          Vn_prime.col(i)[2] < -v_max_ || Vn_prime.col(i)[2] > v_max_) {
        is_dyn_constraints_satisfied = false;
        break;
      }
    }

    /// acceleration
    Eigen::Matrix<double, 3, 2> Vn_double_prime = getMinvoAccelControlPointsDouble(segment);

    // for control points (colunmns of Vn_double_prime)
    for (int i = 0; i < 2; i++) {
      if (Vn_double_prime.col(i)[0] < -a_max_ || Vn_double_prime.col(i)[0] > a_max_ ||
          Vn_double_prime.col(i)[1] < -a_max_ || Vn_double_prime.col(i)[1] > a_max_ ||
          Vn_double_prime.col(i)[2] < -a_max_ || Vn_double_prime.col(i)[2] > a_max_) {
        is_dyn_constraints_satisfied = false;
        break;
      }
    }

    /// jerk
    Eigen::Matrix<double, 3, 1> Vn_triple_prime = getMinvoJerkControlPointsDouble(segment);

    // for control points (colunmns of Vn_triple_prime)
    if (Vn_triple_prime.col(0)[0] < -j_max_ || Vn_triple_prime.col(0)[0] > j_max_ ||
        Vn_triple_prime.col(0)[1] < -j_max_ || Vn_triple_prime.col(0)[1] > j_max_ ||
        Vn_triple_prime.col(0)[2] < -j_max_ || Vn_triple_prime.col(0)[2] > j_max_) {
      is_dyn_constraints_satisfied = false;
      break;
    }

  }  // end for segment
}

void SolverGurobi::checkCollisionViolation(bool& is_collision_free_corridor_satisfied) {
  is_collision_free_corridor_satisfied = true;
  for (int t = 0; t < N_; t++) {     // Loop through each segment
    bool segment_satisfied = false;  // Will be true if at least one polytope contains this segment

    // Retrieve the closed-form MINVO position control points for segment t.
    // Here we assume the function returns a 3x4 matrix (each column is a control point)
    Eigen::Matrix<double, 3, 4> cps = getMinvoPosControlPointsDouble(t);

    // Check every polytope until one is found that contains the segment
    for (int polytope_idx = 0; polytope_idx < polytopes_.size(); polytope_idx++) {
      // Assume this polytope works until proven otherwise
      bool polytope_satisfied = true;

      // Check every control point in the segment.
      for (int j = 0; j < cps.cols(); j++) {
        if (!polytopes_[polytope_idx].inside(cps.col(j))) {
          polytope_satisfied = false;
          break;  // This polytope fails for this constraint
        }
      }

      // If this polytope satisfied all constraints for all control points, mark segment as
      // satisfied.
      if (polytope_satisfied) {
        segment_satisfied = true;
        break;  // No need to check other polytopes for this segment
      }
    }  // end polytope loop

    // If the segment was not contained in any polytope, flag the overall corridor as unsatisfied.
    if (!segment_satisfied) {
      is_collision_free_corridor_satisfied = false;
      break;  // Early exit: one segment failed, so overall constraint is not satisfied.
    }
  }
}

void SolverGurobi::fillGoalSetPoints() {
  const int N = static_cast<int>(goal_setpoints_.size());
  if (N <= 0) return;

  const double T = total_traj_time_;
  const double eps = 1e-9;  // or 1e-6 if your time units are coarse

  for (int i = 0; i < N; ++i) {
    // plan_ already ends at A, so start at dc_
    double t = (i + 1) * dc_;

    // Clamp strictly inside [0, T]
    if (t >= T) t = std::max(0.0, T - eps);

    int interval_idx = 0;
    double dt_interval = 0.0;
    findIntervalIdxAndDt(t, interval_idx, dt_interval);

    RobotState s;
    s.setTimeStamp(t0_ + t);
    s.setPos(
        getPosDouble(interval_idx, dt_interval, 0), getPosDouble(interval_idx, dt_interval, 1),
        getPosDouble(interval_idx, dt_interval, 2));
    s.setVel(
        getVelDouble(interval_idx, dt_interval, 0), getVelDouble(interval_idx, dt_interval, 1),
        getVelDouble(interval_idx, dt_interval, 2));
    s.setAccel(
        getAccelDouble(interval_idx, dt_interval, 0), getAccelDouble(interval_idx, dt_interval, 1),
        getAccelDouble(interval_idx, dt_interval, 2));
    s.setJerk(
        getJerkDouble(interval_idx, dt_interval, 0), getJerkDouble(interval_idx, dt_interval, 1),
        getJerkDouble(interval_idx, dt_interval, 2));

    goal_setpoints_[i] = s;
  }
}

void SolverGurobi::getGoalSetpoints(std::vector<RobotState>& goal_setpoints) {
  goal_setpoints = goal_setpoints_;
}

bool SolverGurobi::hasPolytopes_() const {
  if (use_time_layered_polytopes_) return (!polytopes_time_layered_.empty() && P_spatial_ > 0);
  return (!polytopes_.empty());
}

int SolverGurobi::numSpatialPolys_() const {
  if (use_time_layered_polytopes_) return P_spatial_;
  return static_cast<int>(polytopes_.size());
}

const LinearConstraint3D& SolverGurobi::polyAt_(int t, int p) const {
  if (!use_time_layered_polytopes_) return polytopes_.at(p);

  const int idx = t * P_spatial_ + p;
  return polytopes_time_layered_.at(idx);
}

void SolverGurobi::setPolytopes(std::vector<LinearConstraint3D> polytopes) {
  // Legacy: time-invariant corridor
  use_time_layered_polytopes_ = false;
  P_spatial_ = 0;
  polytopes_time_layered_.clear();
  polytopes_ = std::move(polytopes);
}

void SolverGurobi::setPolytopesTimeLayered(
    const std::vector<std::vector<LinearConstraint3D>>& polytopes_by_time) {
  // Expect: outer size == N_, inner size == P (constant)
  use_time_layered_polytopes_ = true;
  polytopes_.clear();

  const int T = static_cast<int>(polytopes_by_time.size());
  if (T == 0) {
    P_spatial_ = 0;
    polytopes_time_layered_.clear();
    return;
  }

  P_spatial_ = static_cast<int>(polytopes_by_time.front().size());
  if (P_spatial_ == 0) {
    polytopes_time_layered_.clear();
    return;
  }

  polytopes_time_layered_.clear();
  polytopes_time_layered_.reserve(static_cast<size_t>(T) * static_cast<size_t>(P_spatial_));

  for (int t = 0; t < T; ++t) {
    if (static_cast<int>(polytopes_by_time[t].size()) != P_spatial_)
      throw std::runtime_error("setPolytopesTimeLayered: inconsistent P across time layers");

    for (int p = 0; p < P_spatial_; ++p) polytopes_time_layered_.push_back(polytopes_by_time[t][p]);
  }
}

void SolverGurobi::setMapSizeConstraints() {
  // Remove previous map constraints
  if (!map_cons_.empty()) {
    for (int i = 0; i < map_cons_.size(); i++) {
      m_.remove(map_cons_[i]);
    }
    map_cons_.clear();
  }

  for (int t = 0; t < N_; t++) {
    std::vector<GRBLinExpr> cp0 = getCP0(t);
    std::vector<GRBLinExpr> cp1 = getCP1(t);
    std::vector<GRBLinExpr> cp2 = getCP2(t);
    std::vector<GRBLinExpr> cp3 = getCP3(t);

    // Set the constraints for the map size (x, y, and z)

    // X constraints
    map_cons_.push_back(
        m_.addConstr(cp0[0] <= x_max_, "Map_size_cp0_x_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp1[0] <= x_max_, "Map_size_cp1_x_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp2[0] <= x_max_, "Map_size_cp2_x_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp3[0] <= x_max_, "Map_size_cp3_x_max_t_" + std::to_string(t)));

    map_cons_.push_back(
        m_.addConstr(cp0[0] >= x_min_, "Map_size_cp0_x_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp1[0] >= x_min_, "Map_size_cp1_x_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp2[0] >= x_min_, "Map_size_cp2_x_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp3[0] >= x_min_, "Map_size_cp3_x_min_t_" + std::to_string(t)));

    // Y constraints
    map_cons_.push_back(
        m_.addConstr(cp0[1] <= y_max_, "Map_size_cp0_y_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp1[1] <= y_max_, "Map_size_cp1_y_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp2[1] <= y_max_, "Map_size_cp2_y_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp3[1] <= y_max_, "Map_size_cp3_y_max_t_" + std::to_string(t)));

    map_cons_.push_back(
        m_.addConstr(cp0[1] >= y_min_, "Map_size_cp0_y_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp1[1] >= y_min_, "Map_size_cp1_y_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp2[1] >= y_min_, "Map_size_cp2_y_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp3[1] >= y_min_, "Map_size_cp3_y_min_t_" + std::to_string(t)));

    // Z constraints
    map_cons_.push_back(
        m_.addConstr(cp0[2] <= z_max_, "Map_size_cp0_z_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp1[2] <= z_max_, "Map_size_cp1_z_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp2[2] <= z_max_, "Map_size_cp2_z_max_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp3[2] <= z_max_, "Map_size_cp3_z_max_t_" + std::to_string(t)));

    map_cons_.push_back(
        m_.addConstr(cp0[2] >= z_min_, "Map_size_cp0_z_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp1[2] >= z_min_, "Map_size_cp1_z_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp2[2] >= z_min_, "Map_size_cp2_z_min_t_" + std::to_string(t)));
    map_cons_.push_back(
        m_.addConstr(cp3[2] >= z_min_, "Map_size_cp3_z_min_t_" + std::to_string(t)));
  }
}

void SolverGurobi::setPolytopesConstraints() {
  // Remove previous polytopes constraints
  if (!polytopes_cons_.empty()) {
    for (int i = 0; i < polytopes_cons_.size(); i++) {
      m_.remove(polytopes_cons_[i]);
    }
    polytopes_cons_.clear();
  }

  // Remove previous at_least_1_pol_cons_ constraints
  if (!at_least_1_pol_cons_.empty()) {
    for (int i = 0; i < at_least_1_pol_cons_.size(); i++) {
      m_.remove(at_least_1_pol_cons_[i]);
    }
    at_least_1_pol_cons_.clear();
  }

  // Remove previous binary variables
  if (!b_.empty()) {
    for (int i = 0; i < b_.size(); i++) {
      for (int j = 0; j < b_[i].size(); j++) {
        m_.remove(b_[i][j]);
      }
    }
    b_.clear();
  }

  // Remove previous miqp_polytopes_cons_ constraints
  if (!miqp_polytopes_cons_.empty()) {
    for (int i = 0; i < miqp_polytopes_cons_.size(); i++) {
      m_.remove(miqp_polytopes_cons_[i]);
    }
    miqp_polytopes_cons_.clear();
  }

  // Set polytope constraints (either MIQP or SANDO approach)
  setPolyConsts();
}

void SolverGurobi::setPolyConsts() {
  if (!hasPolytopes_()) return;

  const int P = numSpatialPolys_();

  for (int t = 0; t < N_; t++) {
    std::vector<GRBVar> row;

    // Declare binary variables once for all t (avoid duplicated push_back)
    for (int tt = 0; tt < N_; tt++) {
      std::vector<GRBVar> r;
      r.reserve(P);
      for (int i = 0; i < P; i++) {
        GRBVar variable =
            m_.addVar(0.0, 1.0, 0, GRB_BINARY, "s" + std::to_string(i) + "_" + std::to_string(tt));
        r.push_back(variable);
      }
      b_.push_back(r);
    }
    break;  // IMPORTANT: do not repeat the above N_ times
  }

  // Add constraints per segment
  for (int t = 0; t < N_; t++) {
    createSafeCorridorConstraintsForPolytopeAtleastOne(t);
  }
}

void SolverGurobi::createSafeCorridorConstraintsForPolytopeAtleastOne(int t) {
  GRBLinExpr sum = 0;
  for (int col = 0; col < b_[t].size(); col++) {
    sum = sum + b_[t][col];
  }
  at_least_1_pol_cons_.push_back(
      m_.addConstr(sum >= 1, "At_least_1_pol_t_" + std::to_string(t)));  // at least in one polytope

  std::vector<GRBLinExpr> cp0 = getCP0(t);
  std::vector<GRBLinExpr> cp1 = getCP1(t);
  std::vector<GRBLinExpr> cp2 = getCP2(t);
  std::vector<GRBLinExpr> cp3 = getCP3(t);

  // Loop over the number of polytopes
  const int P = numSpatialPolys_();

  for (int p = 0; p < P; ++p) {
    const auto& poly = polyAt_(t, p);

    Eigen::MatrixXd A1 = poly.A();
    auto bb = poly.b();

    // Empty polytope (invalid decomposition): disable this binary so the
    // solver cannot assign the trajectory to a non-existent corridor.
    if (bb.rows() == 0) {
      at_least_1_pol_cons_.push_back(m_.addConstr(
          b_[t][p] == 0, "empty_poly_t" + std::to_string(t) + "_p" + std::to_string(p)));
      continue;
    }

    std::vector<std::vector<double>> A1std = eigenMatrix2std(A1);
    std::vector<GRBLinExpr> Acp0 = MatrixMultiply(A1std, cp0);
    std::vector<GRBLinExpr> Acp1 = MatrixMultiply(A1std, cp1);
    std::vector<GRBLinExpr> Acp2 = MatrixMultiply(A1std, cp2);
    std::vector<GRBLinExpr> Acp3 = MatrixMultiply(A1std, cp3);

    for (int i = 0; i < bb.rows(); i++) {
      miqp_polytopes_cons_.push_back(m_.addGenConstrIndicator(
          b_[t][p], 1, Acp0[i], GRB_LESS_EQUAL, bb[i],
          "safe_corridor_t" + std::to_string(t) + "_p" + std::to_string(p) + "_face" +
              std::to_string(i) + "_cp0"));
      miqp_polytopes_cons_.push_back(m_.addGenConstrIndicator(
          b_[t][p], 1, Acp1[i], GRB_LESS_EQUAL, bb[i],
          "safe_corridor_t" + std::to_string(t) + "_p" + std::to_string(p) + "_face" +
              std::to_string(i) + "_cp1"));
      miqp_polytopes_cons_.push_back(m_.addGenConstrIndicator(
          b_[t][p], 1, Acp2[i], GRB_LESS_EQUAL, bb[i],
          "safe_corridor_t" + std::to_string(t) + "_p" + std::to_string(p) + "_face" +
              std::to_string(i) + "_cp2"));
      miqp_polytopes_cons_.push_back(m_.addGenConstrIndicator(
          b_[t][p], 1, Acp3[i], GRB_LESS_EQUAL, bb[i],
          "safe_corridor_t" + std::to_string(t) + "_p" + std::to_string(p) + "_face" +
              std::to_string(i) + "_cp3"));
    }
  }
}

void SolverGurobi::setX0(const RobotState& data) {
  Eigen::Vector3d pos = data.pos;
  Eigen::Vector3d vel = data.vel;
  Eigen::Vector3d accel = data.accel;

  x0_[0] = data.pos.x();
  x0_[1] = data.pos.y();
  x0_[2] = data.pos.z();
  x0_[3] = data.vel.x();
  x0_[4] = data.vel.y();
  x0_[5] = data.vel.z();
  x0_[6] = data.accel.x();
  x0_[7] = data.accel.y();
  x0_[8] = data.accel.z();
}

void SolverGurobi::setXf(const RobotState& data) {
  Eigen::Vector3d pos = data.pos;
  Eigen::Vector3d vel = data.vel;
  Eigen::Vector3d accel = data.accel;

  xf_[0] = data.pos.x();
  xf_[1] = data.pos.y();
  xf_[2] = data.pos.z();
  xf_[3] = data.vel.x();
  xf_[4] = data.vel.y();
  xf_[5] = data.vel.z();
  xf_[6] = data.accel.x();
  xf_[7] = data.accel.y();
  xf_[8] = data.accel.z();
}

void SolverGurobi::setDirf(double yawf) {
  dirf_[0] = cos(yawf);
  dirf_[1] = sin(yawf);
}

void SolverGurobi::setConstraintsXf() {
  // Remove previous final constraints
  if (!final_cons_.empty()) {
    for (int i = 0; i < final_cons_.size(); i++) {
      m_.remove(final_cons_[i]);
    }

    final_cons_.clear();
  }

  // Constraint xT==x_final
  for (int i = 0; i < 3; i++) {
    // Final position should be equal to the final position
    final_cons_.push_back(m_.addConstr(
        getPos(N_ - 1, dt_.back(), i) - xf_[i] == 0,
        "FinalPosAxis_" + std::to_string(i)));  // Final position
    // Final velocity and acceleration should be 0
    final_cons_.push_back(m_.addConstr(
        getVel(N_ - 1, dt_.back(), i) - xf_[i + 3] == 0,
        "FinalVelAxis_" + std::to_string(i)));  // Final velocity
    final_cons_.push_back(m_.addConstr(
        getAccel(N_ - 1, dt_.back(), i) - xf_[i + 6] == 0,
        "FinalAccel_" + std::to_string(i)));  // Final acceleration
  }
}

void SolverGurobi::setConstraintsX0() {
  // Remove previous initial constraints
  if (!init_cons_.empty()) {
    for (int i = 0; i < init_cons_.size(); i++) {
      m_.remove(init_cons_[i]);
    }
    init_cons_.clear();
  }

  // Constraint x0==x_initial
  for (int i = 0; i < 3; i++) {
    init_cons_.push_back(m_.addConstr(
        getPos(0, 0, i) == x0_[i],
        "InitialPosAxis_" + std::to_string(i)));  // Initial position
    init_cons_.push_back(m_.addConstr(
        getVel(0, 0, i) == x0_[i + 3],
        "InitialVelAxis_" + std::to_string(i)));  // Initial velocity
    init_cons_.push_back(m_.addConstr(
        getAccel(0, 0, i) == x0_[i + 6],
        "InitialAccelAxis_" + std::to_string(i)));  // Initial acceleration}
  }
}

void SolverGurobi::getTotalTrajTime(double& total_traj_time) { total_traj_time = total_traj_time_; }

void SolverGurobi::initializeGoalSetpoints() {
  const double T = total_traj_time_;
  int size = static_cast<int>(std::ceil(T / dc_));  // ceil, not floor
  size = std::max(size, 2);
  goal_setpoints_.assign(size, RobotState{});
}

void SolverGurobi::setDynamicConstraints() {
  if (usingBaseline_()) {
    if (planner_name_ == "safe_baseline")
      setDynamicConstraintsSafeBaseline_();
    else
      setDynamicConstraintsBaseline_();
    return;
  }

  // Remove previous dynamic constraints
  if (!dyn_cons_.empty()) {
    for (int i = 0; i < dyn_cons_.size(); i++) {
      m_.remove(dyn_cons_[i]);
    }
    dyn_cons_.clear();
  }

  // Remove quadratic constraints (for L2 mode)
  if (!dyn_qcons_.empty()) {
    for (auto& qc : dyn_qcons_) m_.remove(qc);
    dyn_qcons_.clear();
  }

  // Choose constraint type based on parameter
  if (dynamic_constraint_type_ == "Linf") {
    // L-infinity: per-axis constraints on ALL control points
    for (int segment = 0; segment < N_; segment++) {
      for (int axis = 0; axis < 3; axis++) {
        // --- Velocity constraints ---
        std::vector<GRBLinExpr> vel_cps = getVelCP(segment, axis);
        for (int i = 0; i < vel_cps.size(); i++) {
          dyn_cons_.push_back(m_.addConstr(
              vel_cps[i] <= v_max_, "max_vel_Linf_seg" + std::to_string(segment) + "_axis_" +
                                        std::to_string(axis) + "_cp" + std::to_string(i)));
          dyn_cons_.push_back(m_.addConstr(
              vel_cps[i] >= -v_max_, "min_vel_Linf_seg" + std::to_string(segment) + "_axis_" +
                                         std::to_string(axis) + "_cp" + std::to_string(i)));
        }

        // --- Acceleration constraints ---
        std::vector<GRBLinExpr> accel_cps = getAccelCP(segment, axis);
        for (int i = 0; i < accel_cps.size(); i++) {
          dyn_cons_.push_back(m_.addConstr(
              accel_cps[i] <= a_max_, "max_accel_Linf_seg" + std::to_string(segment) + "_axis_" +
                                          std::to_string(axis) + "_cp" + std::to_string(i)));
          dyn_cons_.push_back(m_.addConstr(
              accel_cps[i] >= -a_max_, "min_accel_Linf_seg" + std::to_string(segment) + "_axis_" +
                                           std::to_string(axis) + "_cp" + std::to_string(i)));
        }

        // --- Jerk constraints ---
        std::vector<GRBLinExpr> jerk_cps = getJerkCP(segment, axis);
        for (int i = 0; i < jerk_cps.size(); i++) {
          dyn_cons_.push_back(m_.addConstr(
              jerk_cps[i] <= j_max_, "max_jerk_Linf_seg" + std::to_string(segment) + "_axis_" +
                                         std::to_string(axis) + "_cp" + std::to_string(i)));
          dyn_cons_.push_back(m_.addConstr(
              jerk_cps[i] >= -j_max_, "min_jerk_Linf_seg" + std::to_string(segment) + "_axis_" +
                                          std::to_string(axis) + "_cp" + std::to_string(i)));
        }
      }  // end for axis
    }    // end for segment
  } else if (dynamic_constraint_type_ == "L1") {
    // L1 norm: |vx| + |vy| + |vz| ≤ v_max on control points
    for (int segment = 0; segment < N_; segment++) {
      // Get control points for all three axes
      std::vector<GRBLinExpr> vel_cps_x = getVelCP(segment, 0);
      std::vector<GRBLinExpr> vel_cps_y = getVelCP(segment, 1);
      std::vector<GRBLinExpr> vel_cps_z = getVelCP(segment, 2);

      std::vector<GRBLinExpr> accel_cps_x = getAccelCP(segment, 0);
      std::vector<GRBLinExpr> accel_cps_y = getAccelCP(segment, 1);
      std::vector<GRBLinExpr> accel_cps_z = getAccelCP(segment, 2);

      std::vector<GRBLinExpr> jerk_cps_x = getJerkCP(segment, 0);
      std::vector<GRBLinExpr> jerk_cps_y = getJerkCP(segment, 1);
      std::vector<GRBLinExpr> jerk_cps_z = getJerkCP(segment, 2);

      // For each control point
      for (int i = 0; i < vel_cps_x.size(); i++) {
        // L1 velocity norm: 8 linear constraints (octahedron faces)
        dyn_cons_.push_back(m_.addConstr(
            vel_cps_x[i] + vel_cps_y[i] + vel_cps_z[i] <= v_max_,
            "vel_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_ppp"));
        dyn_cons_.push_back(m_.addConstr(
            vel_cps_x[i] + vel_cps_y[i] - vel_cps_z[i] <= v_max_,
            "vel_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_ppn"));
        dyn_cons_.push_back(m_.addConstr(
            vel_cps_x[i] - vel_cps_y[i] + vel_cps_z[i] <= v_max_,
            "vel_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_pnp"));
        dyn_cons_.push_back(m_.addConstr(
            vel_cps_x[i] - vel_cps_y[i] - vel_cps_z[i] <= v_max_,
            "vel_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_pnn"));
        dyn_cons_.push_back(m_.addConstr(
            -vel_cps_x[i] + vel_cps_y[i] + vel_cps_z[i] <= v_max_,
            "vel_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_npp"));
        dyn_cons_.push_back(m_.addConstr(
            -vel_cps_x[i] + vel_cps_y[i] - vel_cps_z[i] <= v_max_,
            "vel_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_npn"));
        dyn_cons_.push_back(m_.addConstr(
            -vel_cps_x[i] - vel_cps_y[i] + vel_cps_z[i] <= v_max_,
            "vel_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_nnp"));
        dyn_cons_.push_back(m_.addConstr(
            -vel_cps_x[i] - vel_cps_y[i] - vel_cps_z[i] <= v_max_,
            "vel_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_nnn"));
      }

      for (int i = 0; i < accel_cps_x.size(); i++) {
        // L1 acceleration norm: 8 linear constraints
        dyn_cons_.push_back(m_.addConstr(
            accel_cps_x[i] + accel_cps_y[i] + accel_cps_z[i] <= a_max_,
            "acc_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_ppp"));
        dyn_cons_.push_back(m_.addConstr(
            accel_cps_x[i] + accel_cps_y[i] - accel_cps_z[i] <= a_max_,
            "acc_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_ppn"));
        dyn_cons_.push_back(m_.addConstr(
            accel_cps_x[i] - accel_cps_y[i] + accel_cps_z[i] <= a_max_,
            "acc_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_pnp"));
        dyn_cons_.push_back(m_.addConstr(
            accel_cps_x[i] - accel_cps_y[i] - accel_cps_z[i] <= a_max_,
            "acc_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_pnn"));
        dyn_cons_.push_back(m_.addConstr(
            -accel_cps_x[i] + accel_cps_y[i] + accel_cps_z[i] <= a_max_,
            "acc_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_npp"));
        dyn_cons_.push_back(m_.addConstr(
            -accel_cps_x[i] + accel_cps_y[i] - accel_cps_z[i] <= a_max_,
            "acc_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_npn"));
        dyn_cons_.push_back(m_.addConstr(
            -accel_cps_x[i] - accel_cps_y[i] + accel_cps_z[i] <= a_max_,
            "acc_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_nnp"));
        dyn_cons_.push_back(m_.addConstr(
            -accel_cps_x[i] - accel_cps_y[i] - accel_cps_z[i] <= a_max_,
            "acc_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_nnn"));
      }

      for (int i = 0; i < jerk_cps_x.size(); i++) {
        // L1 jerk norm: 8 linear constraints
        dyn_cons_.push_back(m_.addConstr(
            jerk_cps_x[i] + jerk_cps_y[i] + jerk_cps_z[i] <= j_max_,
            "jerk_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_ppp"));
        dyn_cons_.push_back(m_.addConstr(
            jerk_cps_x[i] + jerk_cps_y[i] - jerk_cps_z[i] <= j_max_,
            "jerk_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_ppn"));
        dyn_cons_.push_back(m_.addConstr(
            jerk_cps_x[i] - jerk_cps_y[i] + jerk_cps_z[i] <= j_max_,
            "jerk_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_pnp"));
        dyn_cons_.push_back(m_.addConstr(
            jerk_cps_x[i] - jerk_cps_y[i] - jerk_cps_z[i] <= j_max_,
            "jerk_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_pnn"));
        dyn_cons_.push_back(m_.addConstr(
            -jerk_cps_x[i] + jerk_cps_y[i] + jerk_cps_z[i] <= j_max_,
            "jerk_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_npp"));
        dyn_cons_.push_back(m_.addConstr(
            -jerk_cps_x[i] + jerk_cps_y[i] - jerk_cps_z[i] <= j_max_,
            "jerk_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_npn"));
        dyn_cons_.push_back(m_.addConstr(
            -jerk_cps_x[i] - jerk_cps_y[i] + jerk_cps_z[i] <= j_max_,
            "jerk_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_nnp"));
        dyn_cons_.push_back(m_.addConstr(
            -jerk_cps_x[i] - jerk_cps_y[i] - jerk_cps_z[i] <= j_max_,
            "jerk_L1_seg" + std::to_string(segment) + "_cp" + std::to_string(i) + "_nnn"));
      }
    }  // end for segment
  } else if (dynamic_constraint_type_ == "L2") {
    // L2 norm: √(vx² + vy² + vz²) ≤ v_max on control points
    for (int segment = 0; segment < N_; segment++) {
      // Get control points for all three axes
      std::vector<GRBLinExpr> vel_cps_x = getVelCP(segment, 0);
      std::vector<GRBLinExpr> vel_cps_y = getVelCP(segment, 1);
      std::vector<GRBLinExpr> vel_cps_z = getVelCP(segment, 2);

      std::vector<GRBLinExpr> accel_cps_x = getAccelCP(segment, 0);
      std::vector<GRBLinExpr> accel_cps_y = getAccelCP(segment, 1);
      std::vector<GRBLinExpr> accel_cps_z = getAccelCP(segment, 2);

      std::vector<GRBLinExpr> jerk_cps_x = getJerkCP(segment, 0);
      std::vector<GRBLinExpr> jerk_cps_y = getJerkCP(segment, 1);
      std::vector<GRBLinExpr> jerk_cps_z = getJerkCP(segment, 2);

      // For each control point
      for (int i = 0; i < vel_cps_x.size(); i++) {
        // L2 velocity norm
        GRBQuadExpr vel_norm_sq =
            vel_cps_x[i] * vel_cps_x[i] + vel_cps_y[i] * vel_cps_y[i] + vel_cps_z[i] * vel_cps_z[i];
        dyn_qcons_.push_back(m_.addQConstr(
            vel_norm_sq <= v_max_ * v_max_,
            "vel_L2_seg" + std::to_string(segment) + "_cp" + std::to_string(i)));
      }

      for (int i = 0; i < accel_cps_x.size(); i++) {
        // L2 acceleration norm
        GRBQuadExpr acc_norm_sq = accel_cps_x[i] * accel_cps_x[i] +
                                  accel_cps_y[i] * accel_cps_y[i] + accel_cps_z[i] * accel_cps_z[i];
        dyn_qcons_.push_back(m_.addQConstr(
            acc_norm_sq <= a_max_ * a_max_,
            "acc_L2_seg" + std::to_string(segment) + "_cp" + std::to_string(i)));
      }

      for (int i = 0; i < jerk_cps_x.size(); i++) {
        // L2 jerk norm
        GRBQuadExpr jerk_norm_sq = jerk_cps_x[i] * jerk_cps_x[i] + jerk_cps_y[i] * jerk_cps_y[i] +
                                   jerk_cps_z[i] * jerk_cps_z[i];
        dyn_qcons_.push_back(m_.addQConstr(
            jerk_norm_sq <= j_max_ * j_max_,
            "jerk_L2_seg" + std::to_string(segment) + "_cp" + std::to_string(i)));
      }
    }  // end for segment
  }
}

bool SolverGurobi::controlPointDepends(ConstraintType type, int seg, int cp) {
  if (N_ == 4) {
    return controlPointDependsOnD3(type, seg, cp);
  } else if (N_ == 5) {
    return controlPointDependsOnD3OrD4(type, seg, cp);
  } else if (N_ == 6) {
    return controlPointDependsOnD3OrD4OrD5(type, seg, cp);
  } else {
    std::cerr << "Error: N_ should be 4, 5, or 6." << std::endl;
    return false;
  }
}

// Helper function that returns true if 'expr' contains d3_var with a nonzero coefficient.
// Returns true if the control point of the given type, in segment 'seg' (0-indexed)
// and with control point index 'cp' (also 0-indexed) depends on d3.
bool SolverGurobi::controlPointDependsOnD3(ConstraintType type, int seg, int cp) {
  switch (type) {
    case POSITION:
      // For position control points:
      // Segment 0: CP0, CP1, CP2 are independent, CP3 depends.
      if (seg == 0) {
        if (cp == 0 || cp == 1 || cp == 2)
          return false;
        else if (cp == 3)
          return true;
      }
      // Segment 1 and 2: all control points depend.
      if (seg == 1 || seg == 2) return true;
      // Segment 3: CP0, CP1, CP2 depend, CP3 independent.
      if (seg == 3) {
        if (cp == 3)
          return false;
        else
          return true;
      }
      break;

    case VELOCITY:
      // For velocity control points (three per segment):
      // Segment 0: vCP0 is independent; vCP1 and vCP2 depend.
      if (seg == 0) {
        if (cp == 0)
          return false;
        else
          return true;
      }
      // Segment 1 and 2: all depend.
      if (seg == 1 || seg == 2) return true;
      // Segment 3: vCP2 is independent; vCP0 and vCP1 depend.
      if (seg == 3) {
        if (cp == 2)
          return false;
        else
          return true;
      }
      break;

    case ACCELERATION:
      // For acceleration control points (two per segment):
      // Segment 0: aCP0 is independent; aCP1 depends.
      if (seg == 0) {
        if (cp == 0)
          return false;
        else
          return true;
      }
      // Segment 1 and 2: both depend.
      if (seg == 1 || seg == 2) return true;
      // Segment 3: aCP1 is independent; aCP0 depends.
      if (seg == 3) {
        if (cp == 1)
          return false;
        else
          return true;
      }
      break;

    case JERK:
      // For jerk, there's only one control point per segment, and according to your table all jerk
      // CPs depend on d3.
      return true;
  }
  return false;  // Default: if type is not recognized.
}

bool SolverGurobi::controlPointDependsOnD3OrD4(ConstraintType type, int seg, int cp) {
  switch (type) {
    case POSITION:
      if (seg == 0) {
        // Segment 0: CP0, CP1, CP2 independent; CP3 depends.
        if (cp == 0 || cp == 1 || cp == 2)
          return false;
        else if (cp == 3)
          return true;
      } else if (seg == 1) {
        // Segment 1: all depend on d3 and d4.
        return true;
      } else if (seg == 2) {
        // Segment 2: CP0, CP1, CP2 depend on d3 and d4; CP3 depends on d3.
        return true;
      } else if (seg == 3) {
        // Segment 3: CP0 depends on d3; CP1 and CP2 depend on d3 and d4; CP3 depends on d4.
        return true;
      } else if (seg == 4) {
        // Segment 4: CP0, CP1, CP2 depend on d4; CP3 independent.
        if (cp == 3)
          return false;
        else
          return true;
      }
      break;

    case VELOCITY:
      if (seg == 0) {
        // Segment 0: vCP0 independent; vCP1 and vCP2 depend.
        if (cp == 0)
          return false;
        else
          return true;
      } else if (seg == 1) {
        // Segment 1: all depend.
        return true;
      } else if (seg == 2) {
        // Segment 2: all depend.
        return true;
      } else if (seg == 3) {
        // Segment 3: all depend.
        return true;
      } else if (seg == 4) {
        // Segment 4: vCP0 and vCP1 depend on d4; vCP2 independent.
        if (cp == 2)
          return false;
        else
          return true;
      }
      break;

    case ACCELERATION:
      if (seg == 0) {
        // Segment 0: aCP0 independent; aCP1 depends.
        if (cp == 0)
          return false;
        else
          return true;
      } else if (seg == 1) {
        // Segment 1: both depend.
        return true;
      } else if (seg == 2) {
        // Segment 2: both depend.
        return true;
      } else if (seg == 3) {
        // Segment 3: aCP0 depends on d3 and d4; aCP1 depends on d4.
        if (cp == 0)
          return true;
        else  // cp==1
          return false;
      } else if (seg == 4) {
        // Segment 4: aCP0 depends on d4; aCP1 independent.
        if (cp == 0)
          return true;
        else  // cp==1
          return false;
      }
      break;

    case JERK:
      // For jerk, there is one control point per segment.
      // Segments 0-3: depend on d3 and d4; Segment 4: depends on d4.
      if (seg >= 0 && seg <= 3) return true;
      if (seg == 4) return true;
      break;
  }
  return false;  // Default if type not recognized.
}

bool SolverGurobi::controlPointDependsOnD3OrD4OrD5(ConstraintType type, int seg, int cp) {
  // seg: segment index (0...5) for N=6.
  // cp: control point index.
  switch (type) {
    case POSITION:
      // There are 4 position control points per segment.
      if (seg == 0) {
        // Segment 0:
        //   CP0, CP1, CP2 are independent; CP3 depends on d3, d4, d5.
        if (cp >= 0 && cp <= 2)
          return false;
        else if (cp == 3)
          return true;
      } else if (seg == 1) {
        // Segment 1: all CPs depend on d3, d4, d5.
        return true;
      } else if (seg == 2) {
        // Segment 2: CP0, CP1, CP2 depend on d3, d4, d5; CP3 depends on d3.
        return true;
      } else if (seg == 3) {
        // Segment 3: CP0 depends on d3; CP1 and CP2 depend on d3, d4, d5; CP3 depends on d4.
        return true;
      } else if (seg == 4) {
        // Segment 4: CP0 depends on d4; CP1 and CP2 depend on d4, d5; CP3 depends on d5.
        return true;
      } else if (seg == 5) {
        // Segment 5: CP0, CP1, CP2 depend on d5; CP3 is independent.
        if (cp == 3)
          return false;
        else
          return true;
      }
      break;

    case VELOCITY:
      // There are 3 velocity control points per segment.
      if (seg == 0) {
        // Segment 0: vCP0 is independent; vCP1 and vCP2 depend on d3, d4, d5.
        if (cp == 0)
          return false;
        else
          return true;
      } else if (seg == 1) {
        // Segment 1: all velocity CPs depend on d3, d4, d5.
        return true;
      } else if (seg == 2) {
        // Segment 2: all depend on d3, d4, d5.
        return true;
      } else if (seg == 3) {
        // Segment 3: all depend on d3, d4, d5 (even if vCP2 only depends on d4,d5, it still is
        // free).
        return true;
      } else if (seg == 4) {
        // Segment 4: vCP0 and vCP1 depend on d4, d5; vCP2 is independent.
        if (cp == 2)
          return false;
        else
          return true;
      } else if (seg == 5) {
        // Segment 5: vCP0 and vCP1 depend on d5; vCP2 is independent.
        if (cp == 2)
          return false;
        else
          return true;
      }
      break;

    case ACCELERATION:
      // There are 2 acceleration control points per segment.
      if (seg == 0) {
        // Segment 0: aCP0 is independent; aCP1 depends on d3, d4, d5.
        if (cp == 0)
          return false;
        else
          return true;
      } else if (seg == 1) {
        // Segment 1: both depend on d3, d4, d5.
        return true;
      } else if (seg == 2) {
        // Segment 2: both depend on d3, d4, d5.
        return true;
      } else if (seg == 3) {
        // Segment 3: aCP0 depends on d3, d4, d5; aCP1 depends on d4.
        return true;
      } else if (seg == 4) {
        // Segment 4: aCP0 depends on d4, d5; aCP1 depends on d5.
        return true;
      } else if (seg == 5) {
        // Segment 5: aCP0 depends on d5; aCP1 is independent.
        if (cp == 0)
          return true;
        else
          return false;
      }
      break;

    case JERK:
      // There is one jerk control point per segment.
      // Segment 0–3: depend on d3, d4, d5.
      if (seg >= 0 && seg <= 3) return true;
      // Segment 4: depends on d4, d5.
      if (seg == 4) return true;
      // Segment 5: depends on d5.
      if (seg == 5) return true;
      break;
  }
  return false;  // Default if type not recognized.
}

double SolverGurobi::getFactorThatWorked() { return factor_that_worked_; }

bool SolverGurobi::generateNewTrajectory(
    bool& gurobi_error_detected,
    double& gurobi_computation_time,
    double factor,
    bool use_single_thread) {
  // Use sequential factor sweeping for BASELINE
  if (use_single_thread) {
    double factor_used = factor;
    // ignore the provided `factor` and sweep internally
    const bool ok = generateNewTrajectorySequentialFactors(
        gurobi_error_detected, gurobi_computation_time, factor_used);
    factor_that_worked_ = factor_used;  // store for reporting
    return ok;
  }

  bool solved = false;
  last_solve_timing_ = SolveTimingBreakdown{};

  using clk = std::chrono::steady_clock;
  using dur = std::chrono::duration<double>;

  try {
    if (cb_.should_terminate_) return false;

    auto t0 = clk::now();
    findDT(factor);
    auto t1 = clk::now();
    last_solve_timing_.findDT_ms = 1e3 * dur(t1 - t0).count();

    if (usingBaseline_() || !using_variable_elimination_) {
      setXBaseline_();
      setConstraintsX0();
      setConstraintsXf();
      setContinuityConstraints();
    } else {
      setX();
    }
    auto t2 = clk::now();
    last_solve_timing_.setX_ms = 1e3 * dur(t2 - t1).count();

    setPolytopesConstraints();
    auto t3 = clk::now();
    last_solve_timing_.polytopes_ms = 1e3 * dur(t3 - t2).count();

    setDynamicConstraints();
    auto t4 = clk::now();
    last_solve_timing_.dynamic_ms = 1e3 * dur(t4 - t3).count();

    setObjective();
    auto t5 = clk::now();
    last_solve_timing_.objective_ms = 1e3 * dur(t5 - t4).count();

    setMapSizeConstraints();
    auto t6 = clk::now();
    last_solve_timing_.mapsize_ms = 1e3 * dur(t6 - t5).count();

    if (cb_.should_terminate_) return false;

    solved = callOptimizer();
    auto t7 = clk::now();
    last_solve_timing_.callOptimizer_ms = 1e3 * dur(t7 - t6).count();

    if (solved) {
      gurobi_computation_time = m_.get(GRB_DoubleAttr_Runtime) * 1000;
      initializeGoalSetpoints();

      if (usingBaseline_() || !using_variable_elimination_) {
        getCoefficientsDoubleBaseline_();
      } else {
        if (N_ == 4)
          getDependentCoefficientsN4Double();
        else if (N_ == 5)
          getDependentCoefficientsN5Double();
        else if (N_ == 6)
          getDependentCoefficientsN6Double();
      }
    }
    auto t8 = clk::now();
    last_solve_timing_.postsolve_ms = 1e3 * dur(t8 - t7).count();
  } catch (GRBException&) {
    gurobi_error_detected = true;
  }

  return solved;
}

bool SolverGurobi::generateNewTrajectorySequentialFactors(
    bool& gurobi_error_detected, double& gurobi_computation_time_ms, double& factor_that_worked) {
  bool solved = false;
  gurobi_error_detected = false;
  gurobi_computation_time_ms = 0.0;
  factor_that_worked = 0.0;

  // Timing variables
  using std::chrono::duration;
  using std::chrono::steady_clock;

  double total_findDT_ms = 0.0;
  double total_setX_ms = 0.0;
  double total_constraintsX0_ms = 0.0;
  double total_constraintsXf_ms = 0.0;
  double total_continuity_ms = 0.0;
  double total_polytopes_ms = 0.0;
  double total_dynamic_ms = 0.0;
  double total_objective_ms = 0.0;
  double total_mapsize_ms = 0.0;
  double total_optimizer_ms = 0.0;
  double total_goalsetpoints_ms = 0.0;
  double total_getcoeff_ms = 0.0;
  int num_iterations = 0;

  // factor_initial_, factor_final_, factor_constant_step_size should come from par_
  for (double f = factor_initial_; f <= factor_final_ + 1e-9 && !solved && !cb_.should_terminate_;
       f += factor_constant_step_size_) {
    try {
      findDT(f);

      if (usingBaseline_() || !using_variable_elimination_) {
        setXBaseline_();
      } else {
        setX();
      }

      // BASELINE formulation requires explicit constraints (no elimination)
      setConstraintsX0();
      setConstraintsXf();
      setContinuityConstraints();  // if your BASELINE formulation needs it
      setPolytopesConstraints();
      setDynamicConstraints();  // in BASELINE mode this must constrain all CPs
      setObjective();
      setMapSizeConstraints();
      solved = callOptimizer();
      if (solved) {
        initializeGoalSetpoints();
        factor_that_worked = f;
        gurobi_computation_time_ms = m_.get(GRB_DoubleAttr_Runtime) * 1000.0;

        if (usingBaseline_() || !using_variable_elimination_) {
          getCoefficientsDoubleBaseline_();
        } else {
          if (N_ == 4)
            getDependentCoefficientsN4Double();
          else if (N_ == 5)
            getDependentCoefficientsN5Double();
          else if (N_ == 6)
            getDependentCoefficientsN6Double();
        }
      }
    } catch (GRBException&) {
      gurobi_error_detected = true;
      solved = false;
      break;
    }
  }

  cb_.should_terminate_ = false;
  return solved;
}

void SolverGurobi::setInitialDt(double initial_dt) { initial_dt_ = initial_dt; }

void SolverGurobi::findDT(double factor) {
  // Clear the previous dt
  dt_.clear();

  // BASELINE's approach
  for (int i = 0; i < N_; i++) dt_.push_back(factor * std::max(initial_dt_, 2 * dc_));

  // Compute total_traj_time_, which is used to fill goal_setpoints_
  total_traj_time_ = std::accumulate(dt_.begin(), dt_.end(), 0.0);
}

double SolverGurobi::getInitialDt() {
  double initial_dt = 0;

  float t_vx = 0;
  float t_vy = 0;
  float t_vz = 0;
  float t_ax = 0;
  float t_ay = 0;
  float t_az = 0;
  float t_jx = 0;
  float t_jy = 0;
  float t_jz = 0;

  t_vx = fabs(xf_[0] - x0_[0]) / v_max_;
  t_vy = fabs(xf_[1] - x0_[1]) / v_max_;
  t_vz = fabs(xf_[2] - x0_[2]) / v_max_;

  float jerkx = copysign(1, xf_[0] - x0_[0]) * j_max_;
  float jerky = copysign(1, xf_[1] - x0_[1]) * j_max_;
  float jerkz = copysign(1, xf_[2] - x0_[2]) * j_max_;
  float a0x = x0_[6];
  float a0y = x0_[7];
  float a0z = x0_[8];
  float v0x = x0_[3];
  float v0y = x0_[4];
  float v0z = x0_[5];

  // Solve For JERK
  // polynomial ax3+bx2+cx+d=0 --> coeff=[d c b a]
  Eigen::Vector4d coeff_jx(x0_[0] - xf_[0], v0x, a0x / 2.0, jerkx / 6.0);
  Eigen::Vector4d coeff_jy(x0_[1] - xf_[1], v0y, a0y / 2.0, jerky / 6.0);
  Eigen::Vector4d coeff_jz(x0_[2] - xf_[2], v0z, a0z / 2.0, jerkz / 6.0);

  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_jx(coeff_jx);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_jy(coeff_jy);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_jz(coeff_jz);

  std::vector<double> realRoots_jx;
  std::vector<double> realRoots_jy;
  std::vector<double> realRoots_jz;
  psolve_jx.realRoots(realRoots_jx);
  psolve_jy.realRoots(realRoots_jy);
  psolve_jz.realRoots(realRoots_jz);

  t_jx = MinPositiveElement(realRoots_jx);
  t_jy = MinPositiveElement(realRoots_jy);
  t_jz = MinPositiveElement(realRoots_jz);

  float accelx = copysign(1, xf_[0] - x0_[0]) * a_max_;
  float accely = copysign(1, xf_[1] - x0_[1]) * a_max_;
  float accelz = copysign(1, xf_[2] - x0_[2]) * a_max_;

  // Solve For ACCELERATION
  // polynomial ax2+bx+c=0 --> coeff=[c b a]
  Eigen::Vector3d coeff_ax(x0_[0] - xf_[0], v0x, 0.5 * accelx);
  Eigen::Vector3d coeff_ay(x0_[1] - xf_[1], v0y, 0.5 * accely);
  Eigen::Vector3d coeff_az(x0_[2] - xf_[2], v0z, 0.5 * accelz);

  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_ax(coeff_ax);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_ay(coeff_ay);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_az(coeff_az);

  std::vector<double> realRoots_ax;
  std::vector<double> realRoots_ay;
  std::vector<double> realRoots_az;
  psolve_ax.realRoots(realRoots_ax);
  psolve_ay.realRoots(realRoots_ay);
  psolve_az.realRoots(realRoots_az);

  t_ax = MinPositiveElement(realRoots_ax);
  t_ay = MinPositiveElement(realRoots_ay);
  t_az = MinPositiveElement(realRoots_az);

  initial_dt = std::max({t_vx, t_vy, t_vz, t_ax, t_ay, t_az, t_jx, t_jy, t_jz}) / N_;
  if (initial_dt > 10000)  // happens when there is no solution to the previous eq.
  {
    printf("there is not a solution to the previous equations\n");
    initial_dt = 0;
  }

  return initial_dt;
}

void SolverGurobi::setContinuityConstraints() {
  // Remove the previous continuity constraints
  if (!continuity_cons_.empty()) {
    for (int i = 0; i < continuity_cons_.size(); i++) {
      m_.remove(continuity_cons_[i]);
    }
    continuity_cons_.clear();
  }

  // Continuity constraints
  for (int t = 0; t < N_ - 1; t++)  // From 0 to N_ - 1
  {
    for (int i = 0; i < 3; i++) {
      continuity_cons_.push_back(m_.addConstr(
          getPos(t, dt_[t], i) == getPos(t + 1, 0, i),
          "ContPos_t" + std::to_string(t) + "_axis" + std::to_string(i)));  // Continuity in
                                                                            // position
      continuity_cons_.push_back(m_.addConstr(
          getVel(t, dt_[t], i) == getVel(t + 1, 0, i),
          "ContVel_t" + std::to_string(t) + "_axis" + std::to_string(i)));  // Continuity in
                                                                            // velocity
      continuity_cons_.push_back(m_.addConstr(
          getAccel(t, dt_[t], i) == getAccel(t + 1, 0, i),
          "ContAccel_t" + std::to_string(t) + "_axis" +
              std::to_string(i)));  // Continuity in acceleration
    }
  }
}

bool SolverGurobi::callOptimizer() {
  // House keeping
  bool solved = false;  // set the flag to true
  file_t_++;            // increase the temporal counter

  // Optimize
  m_.optimize();

  int optimstatus = m_.get(GRB_IntAttr_Status);

  // Check if the optimization was successful
  if (optimstatus == GRB_OPTIMAL) {
    solved = true;
    objective_value_ = m_.get(GRB_DoubleAttr_ObjVal);
  } else {
    solved = false;

    if (optimstatus == GRB_INF_OR_UNBD || optimstatus == GRB_INFEASIBLE ||
        optimstatus == GRB_UNBOUNDED) {
      if (debug_verbose_) std::cout << "GUROBI Status: Infeasible or Unbounded" << std::endl;
    }

    if (optimstatus == GRB_NUMERIC) {
    }

    if (optimstatus == GRB_INTERRUPTED) {
    }
  }

  return solved;
}

void SolverGurobi::getPieceWisePol(PieceWisePol& pwp) {
  // Reset the piecewise polynomial
  pwp.clear();

  // Get the times of the piecewise polynomial
  double t = 0.0;
  for (int i = 0; i < N_ + 1; i++) {
    if (i == 0) {
      pwp.times.push_back(t0_);
      continue;
    }

    t = t + dt_[i - 1];

    // Add the time to the piecewise polynomial
    pwp.times.push_back(t0_ + t);
  }

  // Get the coefficients of the piecewise polynomial
  for (int i = 0; i < N_; i++) {
    // Initialize the coefficients
    Eigen::Matrix<double, 4, 1> coeff_x_i;
    Eigen::Matrix<double, 4, 1> coeff_y_i;
    Eigen::Matrix<double, 4, 1> coeff_z_i;

    // a, b, c, d
    for (int j = 0; j < 4; j++) {
      coeff_x_i(j) = x_double_[0][i * 4 + j];
      coeff_y_i(j) = x_double_[1][i * 4 + j];
      coeff_z_i(j) = x_double_[2][i * 4 + j];
    }

    // Add the coefficients to the piecewise polynomial
    pwp.coeff_x.push_back(coeff_x_i);
    pwp.coeff_y.push_back(coeff_y_i);
    pwp.coeff_z.push_back(coeff_z_i);
  }
}

void SolverGurobi::getMinvoControlPoints(std::vector<Eigen::Matrix<double, 3, 4>>& cps) {
  // Clear the control points
  cps.clear();

  for (int t = 0; t < N_; t++) {
    cps.push_back(getMinvoPosControlPointsDouble(t));
  }
}

void SolverGurobi::getControlPoints(std::vector<Eigen::Matrix<double, 3, 4>>& cps) {
  // Clear the control points
  cps.clear();

  for (int t = 0; t < N_; t++) {
    cps.push_back(getPosControlPointsDouble(t));
  }
}

inline GRBLinExpr SolverGurobi::getPos(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  return x_[axis][base_index] * tau * tau * tau + x_[axis][base_index + 1] * tau * tau +
         x_[axis][base_index + 2] * tau + x_[axis][base_index + 3];
}

inline double SolverGurobi::getPosDouble(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  return x_double_[axis][base_index] * tau * tau * tau +
         x_double_[axis][base_index + 1] * tau * tau + x_double_[axis][base_index + 2] * tau +
         x_double_[axis][base_index + 3];
}

inline GRBLinExpr SolverGurobi::getVel(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  return 3 * x_[axis][base_index] * tau * tau + 2 * GRBLinExpr(x_[axis][base_index + 1]) * tau +
         GRBLinExpr(x_[axis][base_index + 2]);
}

inline double SolverGurobi::getVelDouble(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  // For the numeric (double) version we assume the solved (numeric) values are stored in x_double_
  return 3 * x_double_[axis][base_index] * tau * tau + 2 * x_double_[axis][base_index + 1] * tau +
         x_double_[axis][base_index + 2];
}

inline GRBLinExpr SolverGurobi::getAccel(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  return 6 * x_[axis][base_index] * tau + 2 * x_[axis][base_index + 1];
}

inline double SolverGurobi::getAccelDouble(int interval, double tau, int axis) const {
  int base_index = 4 * interval;
  return 6 * x_double_[axis][base_index] * tau + 2 * x_double_[axis][base_index + 1];
}
inline GRBLinExpr SolverGurobi::getJerk(int interval, double tau, int axis) const {
  return 6 * x_[axis][4 * interval];
}

inline double SolverGurobi::getJerkDouble(int interval, double tau, int axis) const {
  return 6 * x_double_[axis][4 * interval];
}

// Coefficient getters: At^3 + Bt^2 + Ct + D  , t \in [0, dt]
inline GRBLinExpr SolverGurobi::getA(int interval, int axis) const {
  return x_[axis][4 * interval];
}

inline GRBLinExpr SolverGurobi::getB(int interval, int axis) const {
  return x_[axis][4 * interval + 1];
}

inline GRBLinExpr SolverGurobi::getC(int interval, int axis) const {
  return x_[axis][4 * interval + 2];
}

inline GRBLinExpr SolverGurobi::getD(int interval, int axis) const {
  return x_[axis][4 * interval + 3];
}

// Coefficients Normalized: At^3 + Bt^2 + Ct + D  , t \in [0, 1]
inline GRBLinExpr SolverGurobi::getAn(int interval, int axis) const {
  // a is always from x_ (the decision variable)
  return x_[axis][4 * interval] * dt_[interval] * dt_[interval] * dt_[interval];
}

inline GRBLinExpr SolverGurobi::getBn(int interval, int axis) const {
  return x_[axis][4 * interval + 1] * dt_[interval] * dt_[interval];
}

inline GRBLinExpr SolverGurobi::getCn(int interval, int axis) const {
  return x_[axis][4 * interval + 2] * dt_[interval];
}

inline GRBLinExpr SolverGurobi::getDn(int interval, int axis) const {
  return x_[axis][4 * interval + 3];
}

// Coefficients Normalized: At^3 + Bt^2 + Ct + D  , t \in [0, 1]
inline double SolverGurobi::getAnDouble(int interval, int axis) const {
  return x_double_[axis][4 * interval] * dt_[interval] * dt_[interval] * dt_[interval];
}

inline double SolverGurobi::getBnDouble(int interval, int axis) const {
  return x_double_[axis][4 * interval + 1] * dt_[interval] * dt_[interval];
}

inline double SolverGurobi::getCnDouble(int interval, int axis) const {
  return x_double_[axis][4 * interval + 2] * dt_[interval];
}

inline double SolverGurobi::getDnDouble(int interval, int axis) const {
  return x_double_[axis][4 * interval + 3];
}

// Control Points (of the splines) getters
inline std::vector<GRBLinExpr> SolverGurobi::getCP0(int interval) const {
  std::vector<GRBLinExpr> cp = {
      getPos(interval, 0, 0), getPos(interval, 0, 1), getPos(interval, 0, 2)};
  return cp;
}

inline std::vector<GRBLinExpr> SolverGurobi::getCP1(int interval) const {
  GRBLinExpr cpx = (getCn(interval, 0) + 3 * getDn(interval, 0)) / 3;
  GRBLinExpr cpy = (getCn(interval, 1) + 3 * getDn(interval, 1)) / 3;
  GRBLinExpr cpz = (getCn(interval, 2) + 3 * getDn(interval, 2)) / 3;
  std::vector<GRBLinExpr> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<GRBLinExpr> SolverGurobi::getCP2(int interval) const {
  GRBLinExpr cpx = (getBn(interval, 0) + 2 * getCn(interval, 0) + 3 * getDn(interval, 0)) / 3;
  GRBLinExpr cpy = (getBn(interval, 1) + 2 * getCn(interval, 1) + 3 * getDn(interval, 1)) / 3;
  GRBLinExpr cpz = (getBn(interval, 2) + 2 * getCn(interval, 2) + 3 * getDn(interval, 2)) / 3;
  std::vector<GRBLinExpr> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<GRBLinExpr> SolverGurobi::getCP3(int interval) const {
  std::vector<GRBLinExpr> cp = {
      getPos(interval, dt_[interval], 0), getPos(interval, dt_[interval], 1),
      getPos(interval, dt_[interval], 2)};
  return cp;
}

inline std::vector<double> SolverGurobi::getCP0Double(int interval) const {
  std::vector<double> cp = {
      getPosDouble(interval, 0, 0), getPosDouble(interval, 0, 1), getPosDouble(interval, 0, 2)};
  return cp;
}

inline std::vector<double> SolverGurobi::getCP1Double(int interval) const {
  double cpx = (getCnDouble(interval, 0) + 3 * getDnDouble(interval, 0)) / 3;
  double cpy = (getCnDouble(interval, 1) + 3 * getDnDouble(interval, 1)) / 3;
  double cpz = (getCnDouble(interval, 2) + 3 * getDnDouble(interval, 2)) / 3;
  std::vector<double> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<double> SolverGurobi::getCP2Double(int interval) const {
  double cpx =
      (getBnDouble(interval, 0) + 2 * getCnDouble(interval, 0) + 3 * getDnDouble(interval, 0)) / 3;
  double cpy =
      (getBnDouble(interval, 1) + 2 * getCnDouble(interval, 1) + 3 * getDnDouble(interval, 1)) / 3;
  double cpz =
      (getBnDouble(interval, 2) + 2 * getCnDouble(interval, 2) + 3 * getDnDouble(interval, 2)) / 3;
  std::vector<double> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<double> SolverGurobi::getCP3Double(int interval) const {
  std::vector<double> cp = {
      getPosDouble(interval, dt_[interval], 0), getPosDouble(interval, dt_[interval], 1),
      getPosDouble(interval, dt_[interval], 2)};
  return cp;
}

inline std::vector<std::vector<GRBLinExpr>> SolverGurobi::getMinvoPosControlPoints(
    int interval) const {
  // Compute the normalized coefficient matrix Pn (for u ∈ [0,1])
  Eigen::Matrix<GRBLinExpr, 3, 4> Pn;
  Pn << getAn(interval, 0), getBn(interval, 0), getCn(interval, 0), getDn(interval, 0),
      getAn(interval, 1), getBn(interval, 1), getCn(interval, 1), getDn(interval, 1),
      getAn(interval, 2), getBn(interval, 2), getCn(interval, 2), getDn(interval, 2);

  // Convert from coefficients of polynomials to MINVO control points using the precomputed
  // conversion matrix.
  Eigen::Matrix<GRBLinExpr, 3, 4> Vn;
  // GRBLinExpr doesn't allow this operation: Vn = Pn * A_pos_mv_rest_inv_;
  for (int i = 0; i < Vn.rows(); i++) {
    for (int j = 0; j < Vn.cols(); j++) {
      Vn(i, j) = 0;
      for (int k = 0; k < Pn.cols(); k++) {
        Vn(i, j) += Pn(i, k) * A_pos_mv_rest_inv_(k, j);
      }
    }
  }

  // Conver the Eigen matrix to a vector of vectors - and each vector is a column of the matrix
  // because that is the control point
  std::vector<std::vector<GRBLinExpr>> Vn_vec;
  for (int j = 0; j < Vn.cols(); j++) {
    std::vector<GRBLinExpr> Vn_col;
    for (int i = 0; i < Vn.rows(); i++) {
      Vn_col.push_back(Vn(i, j));
    }
    Vn_vec.push_back(Vn_col);
  }

  return Vn_vec;
}

inline std::vector<std::vector<GRBLinExpr>> SolverGurobi::getMinvoVelControlPoints(
    int interval) const {
  // Retrieve the duration for segment t.
  double deltaT = dt_[interval];

  // Compute the derivative with respect to the normalized variable u:
  // dp/du = 3*Aₙ*u² + 2*Bₙ*u + Cₙ.
  // Then, using the chain rule, dp/dt = (dp/du) (du/dt).
  // du / dt = 1 / deltaT.
  Eigen::Matrix<GRBLinExpr, 3, 3> Pn_prime;
  Pn_prime << (3 * getAn(interval, 0)) / deltaT, (2 * getBn(interval, 0)) / deltaT,
      getCn(interval, 0) / deltaT, (3 * getAn(interval, 1)) / deltaT,
      (2 * getBn(interval, 1)) / deltaT, getCn(interval, 1) / deltaT,
      (3 * getAn(interval, 2)) / deltaT, (2 * getBn(interval, 2)) / deltaT,
      getCn(interval, 2) / deltaT;

  // Convert the normalized derivative to MINVO velocity control points.
  Eigen::Matrix<GRBLinExpr, 3, 3> Vn_prime;
  // Vn_prime = Pn_prime * A_vel_mv_rest_inv_;
  for (int i = 0; i < Vn_prime.rows(); i++) {
    for (int j = 0; j < Vn_prime.cols(); j++) {
      Vn_prime(i, j) = 0;
      for (int k = 0; k < Pn_prime.cols(); k++) {
        Vn_prime(i, j) += Pn_prime(i, k) * A_vel_mv_rest_inv_(k, j);
      }
    }
  }

  // Conver the Eigen matrix to a vector of vectors - and each vector is a column of the matrix
  // because that is the control point
  std::vector<std::vector<GRBLinExpr>> Vn_prime_vec;
  for (int j = 0; j < Vn_prime.cols(); j++) {
    std::vector<GRBLinExpr> Vn_prime_col;
    for (int i = 0; i < Vn_prime.rows(); i++) {
      Vn_prime_col.push_back(Vn_prime(i, j));
    }
    Vn_prime_vec.push_back(Vn_prime_col);
  }

  return Vn_prime_vec;
}

inline std::vector<std::vector<GRBLinExpr>> SolverGurobi::getMinvoAccelControlPoints(
    int interval) const {
  double deltaT = dt_[interval];
  double deltaT2 = deltaT * deltaT;

  // Compute the second derivative with respect to u:
  // d²p/du² = 6*Aₙ*u + 2*Bₙ.
  // Then, using the chain rule, d²p/dt² = (d²p/du²) (du/dt)².
  Eigen::Matrix<GRBLinExpr, 3, 2> Pn_double_prime;
  Pn_double_prime << (6 * getAn(interval, 0)) / deltaT2, (2 * getBn(interval, 0)) / deltaT2,
      (6 * getAn(interval, 1)) / deltaT2, (2 * getBn(interval, 1)) / deltaT2,
      (6 * getAn(interval, 2)) / deltaT2, (2 * getBn(interval, 2)) / deltaT2;

  // Convert the normalized second derivative to MINVO acceleration control points.
  Eigen::Matrix<GRBLinExpr, 3, 2> Vn_double_prime;
  // Vn_double_prime = Pn_double_prime * A_accel_mv_rest_inv_;
  for (int i = 0; i < Vn_double_prime.rows(); i++) {
    for (int j = 0; j < Vn_double_prime.cols(); j++) {
      Vn_double_prime(i, j) = 0;
      for (int k = 0; k < Pn_double_prime.cols(); k++) {
        Vn_double_prime(i, j) += Pn_double_prime(i, k) * A_accel_mv_rest_inv_(k, j);
      }
    }
  }

  // Conver the Eigen matrix to a vector of vectors - and each vector is a column of the matrix
  // because that is the control point
  std::vector<std::vector<GRBLinExpr>> Vn_double_prime_vec;
  for (int j = 0; j < Vn_double_prime.cols(); j++) {
    std::vector<GRBLinExpr> Vn_double_prime_col;
    for (int i = 0; i < Vn_double_prime.rows(); i++) {
      Vn_double_prime_col.push_back(Vn_double_prime(i, j));
    }
    Vn_double_prime_vec.push_back(Vn_double_prime_col);
  }

  return Vn_double_prime_vec;
}

inline std::vector<std::vector<GRBLinExpr>> SolverGurobi::getMinvoJerkControlPoints(
    int interval) const {
  double deltaT = dt_[interval];
  double deltaT3 = deltaT * deltaT * deltaT;

  // Compute the third derivative with respect to u:
  // d³p/du³ = 6*Aₙ.
  // Then, using the chain rule, d³p/dt³ = (d³p/du³) (du/dt)³.
  Eigen::Matrix<GRBLinExpr, 3, 1> Pn_triple_prime;
  Pn_triple_prime << (6 * getAn(interval, 0)) / deltaT3, (6 * getAn(interval, 1)) / deltaT3,
      (6 * getAn(interval, 2)) / deltaT3;

  // Here we assume that no further conversion is required for jerk.
  Eigen::Matrix<GRBLinExpr, 3, 1> Vn_triple_prime = Pn_triple_prime;

  // Conver the Eigen matrix to a vector of vectors - and each vector is a column of the matrix
  // because that is the control point
  std::vector<std::vector<GRBLinExpr>> Vn_triple_prime_vec;
  for (int j = 0; j < Vn_triple_prime.cols(); j++) {
    std::vector<GRBLinExpr> Vn_triple_prime_col;
    for (int i = 0; i < Vn_triple_prime.rows(); i++) {
      Vn_triple_prime_col.push_back(Vn_triple_prime(i, j));
    }
    Vn_triple_prime_vec.push_back(Vn_triple_prime_col);
  }

  return Vn_triple_prime_vec;
}

inline Eigen::Matrix<double, 3, 4> SolverGurobi::getMinvoPosControlPointsDouble(
    int interval) const {
  // Compute the normalized coefficient matrix Pn (for u ∈ [0,1])
  Eigen::Matrix<double, 3, 4> Pn;
  Pn << getAnDouble(interval, 0), getBnDouble(interval, 0), getCnDouble(interval, 0),
      getDnDouble(interval, 0), getAnDouble(interval, 1), getBnDouble(interval, 1),
      getCnDouble(interval, 1), getDnDouble(interval, 1), getAnDouble(interval, 2),
      getBnDouble(interval, 2), getCnDouble(interval, 2), getDnDouble(interval, 2);

  // Convert from coefficients of polynomials to MINVO control points using the precomputed
  // conversion matrix.
  Eigen::Matrix<double, 3, 4> Vn = Pn * A_pos_mv_rest_inv_;
  return Vn;
}

inline Eigen::Matrix<double, 3, 3> SolverGurobi::getMinvoVelControlPointsDouble(
    int interval) const {
  // Retrieve the duration for segment t.
  double deltaT = dt_[interval];

  // Compute the derivative with respect to the normalized variable u:
  // dp/du = 3*Aₙ*u² + 2*Bₙ*u + Cₙ.
  // Then, using the chain rule, dp/dt = (dp/du) (du/dt).
  // du / dt = 1 / deltaT.
  Eigen::Matrix<double, 3, 3> Pn_prime;
  Pn_prime << (3 * getAnDouble(interval, 0)) / deltaT, (2 * getBnDouble(interval, 0)) / deltaT,
      getCnDouble(interval, 0) / deltaT, (3 * getAnDouble(interval, 1)) / deltaT,
      (2 * getBnDouble(interval, 1)) / deltaT, getCnDouble(interval, 1) / deltaT,
      (3 * getAnDouble(interval, 2)) / deltaT, (2 * getBnDouble(interval, 2)) / deltaT,
      getCnDouble(interval, 2) / deltaT;

  // Convert the normalized derivative to MINVO velocity control points.
  Eigen::Matrix<double, 3, 3> Vn_prime = Pn_prime * A_vel_mv_rest_inv_;
  return Vn_prime;
}

inline Eigen::Matrix<double, 3, 2> SolverGurobi::getMinvoAccelControlPointsDouble(
    int interval) const {
  double deltaT = dt_[interval];
  double deltaT2 = deltaT * deltaT;

  // Compute the second derivative with respect to u:
  // d²p/du² = 6*Aₙ*u + 2*Bₙ.
  // Then, using the chain rule, d²p/dt² = (d²p/du²) (du/dt)².
  Eigen::Matrix<double, 3, 2> Pn_double_prime;
  Pn_double_prime << (6 * getAnDouble(interval, 0)) / deltaT2,
      (2 * getBnDouble(interval, 0)) / deltaT2, (6 * getAnDouble(interval, 1)) / deltaT2,
      (2 * getBnDouble(interval, 1)) / deltaT2, (6 * getAnDouble(interval, 2)) / deltaT2,
      (2 * getBnDouble(interval, 2)) / deltaT2;

  // Convert the normalized second derivative to MINVO acceleration control points.
  Eigen::Matrix<double, 3, 2> Vn_double_prime = Pn_double_prime * A_accel_mv_rest_inv_;
  return Vn_double_prime;
}

inline Eigen::Matrix<double, 3, 1> SolverGurobi::getMinvoJerkControlPointsDouble(
    int interval) const {
  double deltaT = dt_[interval];
  double deltaT3 = deltaT * deltaT * deltaT;

  // Compute the third derivative with respect to u:
  // d³p/du³ = 6*Aₙ.
  // Then, using the chain rule, d³p/dt³ = (d³p/du³) (du/dt)³.
  Eigen::Matrix<double, 3, 1> Pn_triple_prime;
  Pn_triple_prime << (6 * getAnDouble(interval, 0)) / deltaT3,
      (6 * getAnDouble(interval, 1)) / deltaT3, (6 * getAnDouble(interval, 2)) / deltaT3;

  // Here we assume that no further conversion is required for jerk.
  Eigen::Matrix<double, 3, 1> Vn_triple_prime = Pn_triple_prime;
  return Vn_triple_prime;
}

inline Eigen::Matrix<double, 3, 4> SolverGurobi::getPosControlPointsDouble(int interval) const {
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

// Returns the velocity control points (as a quadratic Bézier curve) for a given axis in segment
// 'interval'
inline std::vector<GRBLinExpr> SolverGurobi::getVelCP(int interval, int axis) const {
  // Let T = dt_[interval]
  double T = dt_[interval];
  // The underlying cubic coefficients are stored as:
  // a = x_[axis][4*interval], b = x_[axis][4*interval+1], c = x_[axis][4*interval+2], d =
  // x_[axis][4*interval+3]. The velocity polynomial is: v(t) = 3*a*t^2 + 2*b*t + c.
  // Reparameterizing with u = t/T, the power basis coefficients for v(u) become:
  //   A_v = 3*a*T^2, B_v = 2*b*T, C_v = c.
  // Then the quadratic Bézier control points for v(u) are:
  //   V0 = C_v,
  //   V1 = C_v + B_v/2,
  //   V2 = C_v + B_v + A_v.
  GRBLinExpr A_v = 3 * x_[axis][4 * interval] * T * T;
  GRBLinExpr B_v = 2 * x_[axis][4 * interval + 1] * T;
  GRBLinExpr C_v = x_[axis][4 * interval + 2];

  GRBLinExpr V0 = C_v;
  GRBLinExpr V1 = C_v + B_v / 2;
  GRBLinExpr V2 = C_v + B_v + A_v;

  std::vector<GRBLinExpr> cp = {V0, V1, V2};
  return cp;
}

// Returns the acceleration control points (as a linear Bézier curve) for a given axis in segment
// 'interval'
inline std::vector<GRBLinExpr> SolverGurobi::getAccelCP(int interval, int axis) const {
  // Acceleration: a(t)= 6*a*t + 2*b.
  // With u = t/T, note that:
  // a(0)= 2*b, and a(T)=6*a*T+2*b.
  GRBLinExpr A0_expr = 2 * x_[axis][4 * interval + 1];                        // a(0)
  GRBLinExpr A1_expr = A0_expr + 6 * x_[axis][4 * interval] * dt_[interval];  // a(T)

  std::vector<GRBLinExpr> cp = {A0_expr, A1_expr};
  return cp;
}

// Returns the jerk control point (constant) for a given axis in segment 'interval'
inline std::vector<GRBLinExpr> SolverGurobi::getJerkCP(int interval, int axis) const {
  // Jerk: j(t)= 6*a, which is constant.
  GRBLinExpr J = 6 * x_[axis][4 * interval];
  std::vector<GRBLinExpr> cp = {J};
  return cp;
}