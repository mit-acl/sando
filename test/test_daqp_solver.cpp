// Tests of the DAQP trajectory solver backend. Only depends on Eigen and DecompUtil.
//
// With a path to a file as the only argument, the problems and their solutions are also written to
// that file, so that they can be cross-checked with another solver.
#include <sando/daqp_solver.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <limits>
#include <random>
#include <thread>
#include <vector>

namespace {

int g_failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      ++g_failures;                                       \
      std::printf("FAILED %s:%d: ", __FILE__, __LINE__);  \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

Parameters makeParameters(int N) {
  Parameters par{};
  par.num_N = N;
  par.num_P = 3;
  par.dc = 0.01;
  par.x_min = -200.0;
  par.x_max = 200.0;
  par.y_min = -200.0;
  par.y_max = 200.0;
  par.z_min = 0.5;
  par.z_max = 6.0;
  par.v_max = 5.0;
  par.a_max = 20.0;
  par.j_max = 100.0;
  par.dynamic_constraint_type = "Linf";
  par.factor_initial = 1.0;
  par.factor_final = 2.5;
  par.factor_constant_step_size = 0.1;
  par.jerk_smooth_weight = 10.0;
  par.using_variable_elimination = true;
  par.max_gurobi_comp_time_sec = 1.0;
  par.debug_verbose = false;
  return par;
}

RobotState makeState(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a) {
  RobotState s;
  s.setPos(p.x(), p.y(), p.z());
  s.setVel(v.x(), v.y(), v.z());
  s.setAccel(a.x(), a.y(), a.z());
  return s;
}

// Axis-aligned box as a polytope
LinearConstraint3D makeBox(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi) {
  Polyhedron3D poly;
  for (int i = 0; i < 3; ++i) {
    Vec3f n = Vec3f::Zero();
    n(i) = 1.0;
    Vec3f pu = 0.5 * (lo + hi);
    pu(i) = hi(i);
    poly.add(Hyperplane3D(pu, n));
    Vec3f pl = 0.5 * (lo + hi);
    pl(i) = lo(i);
    poly.add(Hyperplane3D(pl, -n));
  }
  const Vec3f inside = 0.5 * (lo + hi);
  return LinearConstraint3D(inside, poly.hyperplanes(), poly);
}

// Worst constraint violations of a solved trajectory
struct Violations {
  double corridor{0.0}, dynamics{0.0}, boundary{0.0}, continuity{0.0};
};

Violations computeViolations(
    TrajectorySolver& solver,
    const std::vector<LinearConstraint3D>& polys,
    const Parameters& par,
    const RobotState& x0,
    const RobotState& xf) {
  Violations out;
  PieceWisePol pwp;
  solver.getPieceWisePol(pwp);
  std::vector<Eigen::Matrix<double, 3, 4>> cps;
  solver.getControlPoints(cps);
  const int N = par.num_N;

  double worst_corridor = 0.0;
  for (int s = 0; s < N; ++s) {
    double best_poly = std::numeric_limits<double>::max();
    for (const auto& poly : polys) {
      const Eigen::MatrixXd A = poly.A();
      const Eigen::VectorXd b = poly.b();
      best_poly = std::min(best_poly, ((A * cps[s]).colwise() - b).maxCoeff());
    }
    worst_corridor = std::max(worst_corridor, best_poly);
  }
  out.corridor = worst_corridor;

  const auto coeffs = [&](int s, int axis) -> Eigen::Vector4d {
    return axis == 0 ? pwp.coeff_x[s] : (axis == 1 ? pwp.coeff_y[s] : pwp.coeff_z[s]);
  };
  for (int s = 0; s < N; ++s) {
    const double T = pwp.times[s + 1] - pwp.times[s];
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Vector4d k = coeffs(s, axis);
      const double a = k(0), b = k(1), c = k(2);
      const double v[3] = {c, c + b * T, c + 2 * b * T + 3 * a * T * T};
      for (double vi : v) out.dynamics = std::max(out.dynamics, std::abs(vi) - par.v_max);
      const double acc[2] = {2 * b, 2 * b + 6 * a * T};
      for (double ai : acc) out.dynamics = std::max(out.dynamics, std::abs(ai) - par.a_max);
      out.dynamics = std::max(out.dynamics, std::abs(6 * a) - par.j_max);

      if (s + 1 < N) {
        const Eigen::Vector4d q = coeffs(s + 1, axis);
        out.continuity = std::max(
            {out.continuity, std::abs(a * T * T * T + b * T * T + c * T + k(3) - q(3)),
             std::abs(3 * a * T * T + 2 * b * T + c - q(2)),
             std::abs(6 * a * T + 2 * b - 2 * q(1))});
      }
    }
  }
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector4d k0 = coeffs(0, axis), kf = coeffs(N - 1, axis);
    const double T = pwp.times[N] - pwp.times[N - 1];
    out.boundary = std::max(
        {out.boundary, std::abs(k0(3) - x0.pos(axis)), std::abs(k0(2) - x0.vel(axis)),
         std::abs(2 * k0(1) - x0.accel(axis)),
         std::abs(kf(0) * T * T * T + kf(1) * T * T + kf(2) * T + kf(3) - xf.pos(axis)),
         std::abs(3 * kf(0) * T * T + 2 * kf(1) * T + kf(2) - xf.vel(axis)),
         std::abs(6 * kf(0) * T + 2 * kf(1) - xf.accel(axis))});
  }
  return out;
}

struct Problem {
  Parameters par;
  RobotState x0, xf;
  double initial_dt{0.1}, factor{1.0};
  std::vector<LinearConstraint3D> polys;
};

bool solve(SolverDAQP& solver, const Problem& problem, double* computation_time_ms = nullptr) {
  solver.initializeSolver(problem.par);
  solver.resetToNominalState();
  solver.setX0(problem.x0);
  solver.setXf(problem.xf);
  solver.setT0(0.0);
  solver.setInitialDt(problem.initial_dt);
  solver.setPolytopes(problem.polys);
  bool error = false;
  double ms = 0.0;
  const bool ok = solver.generateNewTrajectory(error, ms, problem.factor);
  CHECK(!error, "unexpected solver error");
  if (computation_time_ms != nullptr) *computation_time_ms = ms;
  return ok;
}

Problem randomProblem(std::mt19937& rng, int N) {
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  Problem problem;
  problem.par = makeParameters(N);

  // A chain of overlapping boxes, with a turn
  const int P = 3;
  Eigen::Vector3d center(0.0, 0.0, 2.5);
  Eigen::Vector3d start = center;
  Eigen::Vector3d end = center;
  for (int p = 0; p < P; ++p) {
    const Eigen::Vector3d half(
        0.6 + 0.8 * unif(rng), 0.6 + 0.8 * unif(rng), 0.4 + 0.6 * unif(rng));
    problem.polys.push_back(makeBox(center - half, center + half));
    end = center + Eigen::Vector3d(0.7 * half.x(), (2.0 * unif(rng) - 1.0) * 0.7 * half.y(), 0.0);
    center = end + Eigen::Vector3d(0.3 * half.x(), 0.0, 0.1 * (unif(rng) - 0.5));
  }
  const double speed = 3.0 * unif(rng);
  problem.x0 = makeState(
      start, Eigen::Vector3d(speed, (unif(rng) - 0.5), 0.0),
      Eigen::Vector3d(4.0 * (unif(rng) - 0.5), 4.0 * (unif(rng) - 0.5), 0.0));
  problem.xf = makeState(end, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  problem.initial_dt = 0.15 + 0.2 * unif(rng);
  return problem;
}

// Exhaustive search over all segment-to-polytope assignments, each solved with a single polytope
// sequence: a segment is fixed to a polytope by giving a time-layered corridor with only that
// polytope for that segment.
double bruteForceCost(const Problem& problem, bool& feasible) {
  const int N = problem.par.num_N;
  const int P = static_cast<int>(problem.polys.size());
  long total = 1;
  for (int i = 0; i < N; ++i) total *= P;
  double best = std::numeric_limits<double>::max();
  feasible = false;
  SolverDAQP solver;
  solver.initializeSolver(problem.par);
  for (long id = 0; id < total; ++id) {
    std::vector<std::vector<LinearConstraint3D>> layers(N);
    long c = id;
    for (int s = 0; s < N; ++s, c /= P) layers[s] = {problem.polys[c % P]};
    solver.setX0(problem.x0);
    solver.setXf(problem.xf);
    solver.setT0(0.0);
    solver.setInitialDt(problem.initial_dt);
    solver.setPolytopesTimeLayered(layers);
    bool error = false;
    double ms = 0.0;
    if (solver.generateNewTrajectory(error, ms, problem.factor) &&
        solver.getObjectiveValue() < best) {
      best = solver.getObjectiveValue();
      feasible = true;
    }
  }
  return best;
}

void testRandomProblemsAgainstBruteForce(std::ofstream* dump) {
  std::mt19937 rng(42);
  int num_feasible = 0;
  double worst_corridor = 0.0, worst_dynamics = 0.0;
  const int num_problems = 300;
  for (int i = 0; i < num_problems; ++i) {
    const int N = 4 + i % 3;
    const Problem problem = randomProblem(rng, N);
    SolverDAQP solver;
    double ms = 0.0;
    const bool ok = solve(solver, problem, &ms);

    bool bf_feasible = false;
    const double bf_cost = bruteForceCost(problem, bf_feasible);
    CHECK(ok == bf_feasible, "problem %d: feasible=%d, brute force=%d", i, ok, bf_feasible);

    if (ok) {
      ++num_feasible;
      const double cost = solver.getObjectiveValue();
      CHECK(
          bf_feasible && std::abs(cost - bf_cost) <= 1e-6 * (1.0 + std::abs(bf_cost)),
          "problem %d: cost %.10g, brute force %.10g", i, cost, bf_cost);
      const Violations v = computeViolations(solver, problem.polys, problem.par, problem.x0, problem.xf);
      worst_corridor = std::max(worst_corridor, v.corridor);
      worst_dynamics = std::max(worst_dynamics, v.dynamics);
      CHECK(v.corridor <= 1e-5, "problem %d: corridor violation %.3e", i, v.corridor);
      CHECK(v.dynamics <= 1e-5, "problem %d: dynamics violation %.3e", i, v.dynamics);
      CHECK(v.boundary <= 1e-7, "problem %d: boundary violation %.3e", i, v.boundary);
      CHECK(v.continuity <= 1e-7, "problem %d: continuity violation %.3e", i, v.continuity);
    }

    if (dump != nullptr) {
      dump->precision(17);
      *dump << "{\"N\":" << N << ",\"dt\":" << problem.initial_dt * problem.factor;
      const auto vec = [&](const Eigen::Vector3d& x) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "[%.17g,%.17g,%.17g]", x(0), x(1), x(2));
        return std::string(buf);
      };
      *dump << ",\"p0\":" << vec(problem.x0.pos) << ",\"v0\":" << vec(problem.x0.vel)
            << ",\"a0\":" << vec(problem.x0.accel) << ",\"pf\":" << vec(problem.xf.pos);
      *dump << ",\"polys\":[";
      for (size_t p = 0; p < problem.polys.size(); ++p) {
        const Eigen::MatrixXd A = problem.polys[p].A();
        const Eigen::VectorXd b = problem.polys[p].b();
        *dump << (p ? "," : "") << "{\"A\":[";
        for (int r = 0; r < A.rows(); ++r) *dump << (r ? "," : "") << vec(A.row(r).transpose());
        *dump << "],\"b\":[";
        char buf[64];
        for (int r = 0; r < b.rows(); ++r) {
          std::snprintf(buf, sizeof(buf), "%s%.17g", r ? "," : "", b(r));
          *dump << buf;
        }
        *dump << "]}";
      }
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.17g", ok ? solver.getObjectiveValue() : -1.0);
      *dump << "],\"feasible\":" << (ok ? "true" : "false") << ",\"cost\":" << buf
            << ",\"ms\":" << ms << "}\n";
    }
  }
  CHECK(num_feasible > num_problems / 10, "too few feasible problems (%d)", num_feasible);
  std::printf(
      "random problems: %d/%d feasible, worst violation: corridor %.2e, dynamics %.2e\n",
      num_feasible, num_problems, worst_corridor, worst_dynamics);
}

// Without active inequality constraints, the solution is a polynomial spline given by the equality
// constraints. A straight rest-to-rest maneuver stays on the line.
void testRestToRestIsStraight() {
  Problem problem;
  problem.par = makeParameters(5);
  problem.x0 = makeState({0, 0, 2}, {0, 0, 0}, {0, 0, 0});
  problem.xf = makeState({4, 2, 3}, {0, 0, 0}, {0, 0, 0});
  problem.initial_dt = 0.5;
  SolverDAQP solver;
  CHECK(solve(solver, problem), "no corridor problem should be feasible");
  std::vector<Eigen::Matrix<double, 3, 4>> cps;
  solver.getControlPoints(cps);
  const Eigen::Vector3d dir = (problem.xf.pos - problem.x0.pos).normalized();
  double off = 0.0;
  for (const auto& cp : cps)
    for (int k = 0; k < 4; ++k) {
      const Eigen::Vector3d d = cp.col(k) - problem.x0.pos;
      off = std::max(off, (d - d.dot(dir) * dir).norm());
    }
  CHECK(off < 1e-8, "rest-to-rest deviates from the line by %.3e", off);
}

void testInfeasibleCases() {
  Problem base;
  base.par = makeParameters(5);
  base.x0 = makeState({0, 0, 2}, {0, 0, 0}, {0, 0, 0});
  base.xf = makeState({4, 0, 2}, {0, 0, 0}, {0, 0, 0});
  base.initial_dt = 0.5;

  {  // The corridor does not contain the goal
    Problem p = base;
    p.polys = {makeBox({-1, -1, 1}, {1, 1, 3})};
    SolverDAQP solver;
    CHECK(!solve(solver, p), "goal outside of the corridor should be infeasible");
  }
  {  // Too little time for the velocity limit
    Problem p = base;
    p.initial_dt = 0.01;
    SolverDAQP solver;
    CHECK(!solve(solver, p), "too short time allocation should be infeasible");
  }
  {  // The initial velocity violates the limit
    Problem p = base;
    p.x0 = makeState({0, 0, 2}, {6, 0, 0}, {0, 0, 0});
    SolverDAQP solver;
    CHECK(!solve(solver, p), "initial velocity above the limit should be infeasible");
  }
  {  // An empty polytope must not be selectable
    Problem p = base;
    p.polys = {LinearConstraint3D{}};
    p.polys[0].A_.resize(0, 3);
    p.polys[0].b_.resize(0);
    SolverDAQP solver;
    CHECK(!solve(solver, p), "corridor with only empty polytopes should be infeasible");
  }
}

void testUnsupportedConfigurationsThrow() {
  Parameters par = makeParameters(5);
  par.dynamic_constraint_type = "L2";
  SolverDAQP l2;
  bool thrown = false;
  try {
    l2.initializeSolver(par);
  } catch (const std::invalid_argument&) {
    thrown = true;
  }
  CHECK(thrown, "L2 should be rejected");

  thrown = false;
  try {
    makeTrajectorySolver("not_a_solver");
  } catch (const std::invalid_argument&) {
    thrown = true;
  }
  CHECK(thrown, "an unknown backend should be rejected");
  CHECK(makeTrajectorySolver("daqp") != nullptr, "daqp should always be available");
  CHECK(makeTrajectorySolver("auto") != nullptr, "auto should always give a solver");
}

void testL1IsTighterThanLinf() {
  Problem p;
  p.par = makeParameters(5);
  p.x0 = makeState({0, 0, 2}, {0, 0, 0}, {0, 0, 0});
  p.xf = makeState({2.0, 2.0, 2.5}, {0, 0, 0}, {0, 0, 0});
  p.initial_dt = 0.3;
  SolverDAQP linf;
  CHECK(solve(linf, p), "Linf problem should be feasible");
  p.par.dynamic_constraint_type = "L1";
  p.par.v_max = 5.0;
  SolverDAQP l1;
  const bool ok = solve(l1, p);
  if (ok) {
    CHECK(
        l1.getObjectiveValue() >= linf.getObjectiveValue() - 1e-9,
        "L1 (%.8g) cannot be cheaper than Linf (%.8g)", l1.getObjectiveValue(),
        linf.getObjectiveValue());
    std::vector<Eigen::Matrix<double, 3, 4>> cps;
    PieceWisePol pwp;
    l1.getPieceWisePol(pwp);
    for (int s = 0; s < 5; ++s) {
      const double v = std::abs(pwp.coeff_x[s](2)) + std::abs(pwp.coeff_y[s](2)) + std::abs(pwp.coeff_z[s](2));
      CHECK(v <= p.par.v_max + 1e-5, "L1 velocity %.8f above the limit", v);
    }
  }
}

void testStopAndTimeLimit() {
  std::mt19937 rng(1);
  Problem problem;
  do {
    problem = randomProblem(rng, 6);
    SolverDAQP probe;
    if (solve(probe, problem)) break;
  } while (true);

  {  // A pending stop is honored, and the solver can be used again after a reset
    SolverDAQP solver;
    solver.initializeSolver(problem.par);
    solver.setX0(problem.x0);
    solver.setXf(problem.xf);
    solver.setT0(0.0);
    solver.setInitialDt(problem.initial_dt);
    solver.setPolytopes(problem.polys);
    solver.stopExecution();
    bool error = false;
    double ms = 0.0;
    CHECK(!solver.generateNewTrajectory(error, ms, problem.factor), "stop was ignored");
    CHECK(!error, "a stop is not an error");
    solver.resetToNominalState();
    CHECK(solver.generateNewTrajectory(error, ms, problem.factor), "no recovery after a reset");
  }
  // Each layer has identical polytopes, and those of the middle segments are out of reach, so
  // that the search has a lot to enumerate before the infeasibility is detected
  Problem hard = problem;
  hard.par.num_N = 12;
  hard.par.max_gurobi_comp_time_sec = 60.0;
  hard.x0 = makeState({0, 0, 1}, {0, 0, 0}, {0, 0, 0});
  hard.xf = makeState({0.1, 0, 1}, {0, 0, 0}, {0, 0, 0});
  std::vector<std::vector<LinearConstraint3D>> hard_layers(hard.par.num_N);
  for (int s = 0; s < hard.par.num_N; ++s) {
    for (int c = 0; c < 8; ++c) {
      const double w = 0.05 * c;
      const bool middle = s == 5 || s == 6;
      hard_layers[s].push_back(
          middle ? makeBox({40 + w, 40, 5}, {50, 50, 6}) : makeBox({-50 + w, -50, 0.5}, {50, 50, 6}));
    }
  }
  const auto setup_hard = [&](SolverDAQP& solver, double time_limit) {
    Problem p = hard;
    p.par.max_gurobi_comp_time_sec = time_limit;
    solver.initializeSolver(p.par);
    solver.setX0(p.x0);
    solver.setXf(p.xf);
    solver.setT0(0.0);
    solver.setInitialDt(0.2);
    solver.setPolytopesTimeLayered(hard_layers);
  };
  {  // The time limit bounds a long solve, and it does not report a solution
    SolverDAQP solver;
    setup_hard(solver, 0.02);
    bool error = false;
    double ms = 0.0;
    const bool ok = solver.generateNewTrajectory(error, ms, 1.0);
    CHECK(!ok, "a solve that reached the time limit should not report a solution");
    CHECK(!error, "a time limit is not an error");
    CHECK(solver.getLastNodeCount() > 1, "the time limit test is not meaningful");
    CHECK(ms < 1000.0, "the time limit did not bound the solve (%.1f ms)", ms);
    std::printf("time limit 20 ms: stopped after %.1f ms and %d nodes\n", ms, solver.getLastNodeCount());
  }
  {  // A solve cannot be interrupted, but a stop during a solve discards its result
    SolverDAQP solver;
    setup_hard(solver, 0.2);
    std::atomic<bool> finished{false};
    bool ok = true;
    bool error = false;
    std::thread worker([&]() {
      double ms = 0.0;
      ok = solver.generateNewTrajectory(error, ms, 1.0);
      finished = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const bool finished_before_stop = finished;
    solver.stopExecution();
    worker.join();
    CHECK(!finished_before_stop, "the solve finished before the stop: the test is not meaningful");
    CHECK(!ok, "a stopped solve should not report a solution");
    CHECK(!error, "a stop is not an error");
  }
  {  // A solve that is out of time does not report a solution
    Problem p = problem;
    p.par.max_gurobi_comp_time_sec = 0.0;
    SolverDAQP solver;
    bool error = false;
    double ms = 0.0;
    solver.initializeSolver(p.par);
    solver.setX0(p.x0);
    solver.setXf(p.xf);
    solver.setT0(0.0);
    solver.setInitialDt(p.initial_dt);
    solver.setPolytopes(p.polys);
    CHECK(!solver.generateNewTrajectory(error, ms, p.factor), "the time limit was ignored");
    CHECK(!error, "a time limit is not an error");
  }
  {  // Sequential sweep over the factors reports the factor that worked
    Problem p = problem;
    SolverDAQP solver;
    solver.initializeSolver(p.par);
    solver.setX0(p.x0);
    solver.setXf(p.xf);
    solver.setT0(0.0);
    solver.setInitialDt(p.initial_dt);
    solver.setPolytopes(p.polys);
    bool error = false;
    double ms = 0.0;
    const bool ok = solver.generateNewTrajectory(error, ms, 0.0, true);
    CHECK(ok, "the sweep should find a solution");
    CHECK(solver.getFactorThatWorked() >= p.par.factor_initial, "no factor reported");
  }
}

void testSetpointsMatchBoundaryConditions() {
  Problem p;
  p.par = makeParameters(5);
  p.x0 = makeState({0, 0, 2}, {1, 0, 0}, {0.5, 0, 0});
  p.xf = makeState({3, 1, 2.5}, {0, 0, 0}, {0, 0, 0});
  p.initial_dt = 0.4;
  SolverDAQP solver;
  CHECK(solve(solver, p), "should be feasible");
  solver.fillGoalSetPoints();
  std::vector<RobotState> setpoints;
  solver.getGoalSetpoints(setpoints);
  CHECK(!setpoints.empty(), "no setpoints");
  const double first = (setpoints.front().pos - p.x0.pos).norm();
  const double last = (setpoints.back().pos - p.xf.pos).norm();
  CHECK(first < 0.1, "first setpoint is %.3f m from the start", first);
  CHECK(last < 1e-3, "last setpoint is %.3e m from the goal", last);
}

}  // namespace

int main(int argc, char** argv) {
  std::ofstream dump_file;
  if (argc > 1) dump_file.open(argv[1]);
  testRestToRestIsStraight();
  testInfeasibleCases();
  testUnsupportedConfigurationsThrow();
  testL1IsTighterThanLinf();
  testStopAndTimeLimit();
  testSetpointsMatchBoundaryConditions();
  testRandomProblemsAgainstBruteForce(argc > 1 ? &dump_file : nullptr);
  if (g_failures == 0) std::printf("all tests passed\n");
  return g_failures == 0 ? 0 : 1;
}
