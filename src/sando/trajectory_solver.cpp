/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <math.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <numeric>
#include <sando/trajectory_solver.hpp>
#include <stdexcept>

#include "sando/daqp_solver.hpp"
#ifdef SANDO_HAS_GUROBI
#include "sando/gurobi_solver.hpp"
#endif

std::shared_ptr<TrajectorySolver> makeTrajectorySolver(const std::string& backend) {
  if (backend == "daqp") return std::make_shared<SolverDAQP>();
  if (backend != "auto" && backend != "gurobi")
    throw std::invalid_argument(
        "Unknown trajectory_solver \"" + backend + "\". Valid values: auto, gurobi, daqp.");

#ifdef SANDO_HAS_GUROBI
  try {
    return std::make_shared<SolverGurobi>();
  } catch (const GRBException& e) {
    // Typically a missing license
    if (backend == "gurobi")
      throw std::runtime_error(
          "Could not start Gurobi: " + e.getMessage() +
          " Use trajectory_solver \"daqp\" or \"auto\" to run without a license.");
    static std::once_flag warned;
    std::call_once(warned, [&e]() {
      std::cerr << "Could not start Gurobi (" << e.getMessage() << "). Using DAQP instead."
                << std::endl;
    });
    return std::make_shared<SolverDAQP>();
  }
#else
  if (backend == "gurobi")
    throw std::invalid_argument(
        "trajectory_solver is \"gurobi\", but SANDO was built without Gurobi. Use \"daqp\" or "
        "\"auto\", or rebuild with Gurobi installed.");
  return std::make_shared<SolverDAQP>();
#endif
}

/** @brief Find the smallest positive element in a vector.
 *  @param v Vector of doubles to search.
 *  @return The minimum positive value, or 0 if none exists.
 */
static double MinPositiveElement(std::vector<double> v) {
  std::sort(v.begin(), v.end());  // sorted in ascending order
  double min_value = 0;
  for (int i = 0; i < v.size(); i++) {
    if (v[i] > 0) {
      min_value = v[i];
      break;
    }
  }
  return min_value;
}

void TrajectorySolver::setCommonParameters_(const Parameters& par) {
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
  max_comp_time_sec_ = par.max_gurobi_comp_time_sec;
}

void TrajectorySolver::setPlannerName(const std::string& name) { planner_name_ = name; }

bool TrajectorySolver::usingFaster_() const {
  return (
      planner_name_ == "FASTER" || planner_name_ == "original_faster" ||
      planner_name_ == "safe_faster");
}

void TrajectorySolver::setT0(double t0) { t0_ = t0; }

void TrajectorySolver::getInitialAndFinalConditions(
    double& P0, double& V0, double& A0, double& Pf, double& Vf, double& Af, int axis) {
  // Get initial and final conditions
  P0 = x0_[axis];
  V0 = x0_[axis + 3];
  A0 = x0_[axis + 6];
  Pf = xf_[axis];
  Vf = xf_[axis + 3];
  Af = xf_[axis + 6];
}

void TrajectorySolver::findIntervalIdxAndDt(
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

void TrajectorySolver::fillGoalSetPoints() {
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

void TrajectorySolver::getGoalSetpoints(std::vector<RobotState>& goal_setpoints) {
  goal_setpoints = goal_setpoints_;
}

bool TrajectorySolver::hasPolytopes_() const {
  if (use_time_layered_polytopes_) return (!polytopes_time_layered_.empty() && P_spatial_ > 0);
  return (!polytopes_.empty());
}

int TrajectorySolver::numSpatialPolys_() const {
  if (use_time_layered_polytopes_) return P_spatial_;
  return static_cast<int>(polytopes_.size());
}

const LinearConstraint3D& TrajectorySolver::polyAt_(int t, int p) const {
  if (!use_time_layered_polytopes_) return polytopes_.at(p);

  const int idx = t * P_spatial_ + p;
  return polytopes_time_layered_.at(idx);
}

void TrajectorySolver::setPolytopes(std::vector<LinearConstraint3D> polytopes) {
  // Legacy: time-invariant corridor
  use_time_layered_polytopes_ = false;
  P_spatial_ = 0;
  polytopes_time_layered_.clear();
  polytopes_ = std::move(polytopes);
}

void TrajectorySolver::setPolytopesTimeLayered(
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

void TrajectorySolver::setX0(const RobotState& data) {
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

void TrajectorySolver::setXf(const RobotState& data) {
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

void TrajectorySolver::setDirf(double yawf) {
  dirf_[0] = cos(yawf);
  dirf_[1] = sin(yawf);
}

void TrajectorySolver::getTotalTrajTime(double& total_traj_time) { total_traj_time = total_traj_time_; }

void TrajectorySolver::initializeGoalSetpoints() {
  const double T = total_traj_time_;
  int size = static_cast<int>(std::ceil(T / dc_));  // ceil, not floor
  size = std::max(size, 2);
  goal_setpoints_.assign(size, RobotState{});
}

double TrajectorySolver::getFactorThatWorked() { return factor_that_worked_; }

void TrajectorySolver::setInitialDt(double initial_dt) { initial_dt_ = initial_dt; }

void TrajectorySolver::findDT(double factor) {
  // Clear the previous dt
  dt_.clear();

  // FASTER's approach
  for (int i = 0; i < N_; i++) dt_.push_back(factor * std::max(initial_dt_, 2 * dc_));

  // Compute total_traj_time_, which is used to fill goal_setpoints_
  total_traj_time_ = std::accumulate(dt_.begin(), dt_.end(), 0.0);
}

double TrajectorySolver::getInitialDt() {
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

void TrajectorySolver::getPieceWisePol(PieceWisePol& pwp) {
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

void TrajectorySolver::getControlPoints(std::vector<Eigen::Matrix<double, 3, 4>>& cps) {
  // Clear the control points
  cps.clear();

  for (int t = 0; t < N_; t++) {
    cps.push_back(getPosControlPointsDouble(t));
  }
}
