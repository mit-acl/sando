#!/usr/bin/env python3
"""Cross-check the DAQP backend against Gurobi (through gurobipy) on the random test problems.

The problems are written by the unit test:

    build/sando/test_daqp_solver problems.jsonl
    python3 test/crosscheck_gurobipy.py problems.jsonl

Gurobi solves its own formulation of each problem: the full cubic coefficients of every segment
(no variable elimination) and indicator constraints for the corridor, as SolverGurobi does. The
limits are those of makeParameters in test_daqp_solver.cpp. The size-limited license that comes
with `pip install gurobipy` is enough for these problems.
"""

import json
import sys

import gurobipy as gp
import numpy as np

GRB = gp.GRB

# makeParameters in test_daqp_solver.cpp
V_MAX, A_MAX, J_MAX = 5.0, 20.0, 100.0
BOUNDS = [(-200.0, 200.0), (-200.0, 200.0), (0.5, 6.0)]
# Margin on the map bounds for the start point d of segments 3, 4, ... (as in SolverGurobi)
MARGIN = [5.0, 5.0, 2.0]
WEIGHT = 10.0


def solve(env, p):
    """Returns (status, objective, runtime in ms)."""
    N, T = p["N"], p["dt"]
    P0, V0, A0, Pf = (np.array(p[k]) for k in ("p0", "v0", "a0", "pf"))
    m = gp.Model(env=env)
    m.Params.MIPGap = 1e-9  # The default (1e-4) is looser than the tolerance of the DAQP backend
    c = m.addVars(3, N, 4, lb=-GRB.INFINITY)  # a, b, c, d of each axis and segment

    def pos(ax, s, t):
        return c[ax, s, 0] * t**3 + c[ax, s, 1] * t**2 + c[ax, s, 2] * t + c[ax, s, 3]

    def vel(ax, s, t):
        return 3 * c[ax, s, 0] * t**2 + 2 * c[ax, s, 1] * t + c[ax, s, 2]

    def acc(ax, s, t):
        return 6 * c[ax, s, 0] * t + 2 * c[ax, s, 1]

    def control_points(ax, s):
        a, b, cc, d = (c[ax, s, i] for i in range(4))
        return [d, cc * T / 3 + d, (b * T * T + 2 * cc * T + 3 * d) / 3, a * T**3 + b * T * T + cc * T + d]

    for ax in range(3):
        m.addConstr(pos(ax, 0, 0) == P0[ax])
        m.addConstr(vel(ax, 0, 0) == V0[ax])
        m.addConstr(acc(ax, 0, 0) == A0[ax])
        m.addConstr(pos(ax, N - 1, T) == Pf[ax])
        m.addConstr(vel(ax, N - 1, T) == 0)
        m.addConstr(acc(ax, N - 1, T) == 0)
        for s in range(N - 1):
            m.addConstr(pos(ax, s, T) == pos(ax, s + 1, 0))
            m.addConstr(vel(ax, s, T) == vel(ax, s + 1, 0))
            m.addConstr(acc(ax, s, T) == acc(ax, s + 1, 0))
        for s in range(3, N):
            m.addConstr(c[ax, s, 3] >= BOUNDS[ax][0] - MARGIN[ax])
            m.addConstr(c[ax, s, 3] <= BOUNDS[ax][1] + MARGIN[ax])

    # Linf limits on the control points of velocity, acceleration and jerk, and the map bounds
    for s in range(N):
        for ax in range(3):
            a, b, cc = c[ax, s, 0], c[ax, s, 1], c[ax, s, 2]
            for e in (cc, cc + b * T, cc + 2 * b * T + 3 * a * T * T):
                m.addConstr(e <= V_MAX)
                m.addConstr(e >= -V_MAX)
            for e in (2 * b, 2 * b + 6 * a * T):
                m.addConstr(e <= A_MAX)
                m.addConstr(e >= -A_MAX)
            m.addConstr(6 * a <= J_MAX)
            m.addConstr(6 * a >= -J_MAX)
            for e in control_points(ax, s):
                m.addConstr(e <= BOUNDS[ax][1])
                m.addConstr(e >= BOUNDS[ax][0])

    # Each segment has all its control points in at least one polytope
    polys = p["polys"]
    in_poly = m.addVars(N, len(polys), vtype=GRB.BINARY)
    for s in range(N):
        m.addConstr(gp.quicksum(in_poly[s, q] for q in range(len(polys))) >= 1)
        cps = [control_points(ax, s) for ax in range(3)]
        for q, poly in enumerate(polys):
            for row, rhs in zip(poly["A"], poly["b"]):
                for k in range(4):
                    m.addGenConstrIndicator(
                        in_poly[s, q], 1, row[0] * cps[0][k] + row[1] * cps[1][k] + row[2] * cps[2][k] <= rhs
                    )

    m.setObjective(
        WEIGHT * gp.quicksum((6 * c[ax, s, 0]) * (6 * c[ax, s, 0]) for ax in range(3) for s in range(N)),
        GRB.MINIMIZE,
    )
    m.optimize()
    return m.Status, (m.ObjVal if m.Status == GRB.OPTIMAL else None), m.Runtime * 1e3


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    env = gp.Env(empty=True)
    env.setParam("OutputFlag", 0)
    env.start()

    count = feasible = mismatches = 0
    worst_gap = worst_daqp_gap = 0.0
    times = []
    daqp_times = []
    for i, line in enumerate(open(sys.argv[1])):
        p = json.loads(line)
        status, obj, ms = solve(env, p)
        times.append(ms)
        if "ms" in p:
            daqp_times.append((p["ms"], ms, p["feasible"]))
        count += 1
        gurobi_feasible = status == GRB.OPTIMAL
        if status not in (GRB.OPTIMAL, GRB.INFEASIBLE, GRB.INF_OR_UNBD):
            print(f"problem {i}: unexpected Gurobi status {status}")
            mismatches += 1
        elif gurobi_feasible != p["feasible"]:
            print(f"problem {i}: DAQP feasible={p['feasible']}, Gurobi feasible={gurobi_feasible}")
            mismatches += 1
        elif gurobi_feasible:
            feasible += 1
            gap = (p["cost"] - obj) / max(1.0, abs(obj))  # Positive if DAQP is worse
            worst_daqp_gap = max(worst_daqp_gap, gap)
            gap = abs(gap)
            worst_gap = max(worst_gap, gap)
            if gap > 1e-4:
                print(f"problem {i}: cost DAQP {p['cost']:.10g}, Gurobi {obj:.10g}")
                mismatches += 1

    times.sort()
    print(
        f"{count} problems ({feasible} feasible): {mismatches} mismatches, worst relative cost gap "
        f"{worst_gap:.2e} (DAQP worse by at most {worst_daqp_gap:.2e}), Gurobi solve time median {times[len(times) // 2]:.2f} ms, max {times[-1]:.2f} ms"
    )
    # Gurobi's runtime excludes building the model; DAQP's includes the big-M rows, not the elimination
    for name, sel in (
        ("feasible", [t for t in daqp_times if t[2]]),
        ("infeasible", [t for t in daqp_times if not t[2]]),
    ):
        if sel:
            d = sorted(t[0] for t in sel)
            g = sorted(t[1] for t in sel)
            ratio = sorted(t[1] / max(t[0], 1e-3) for t in sel)
            print(
                f"{name:10s} ({len(sel)}): DAQP median {d[len(d) // 2]:.3f} ms, max {d[-1]:.3f} ms | "
                f"Gurobi median {g[len(g) // 2]:.2f} ms, max {g[-1]:.2f} ms | "
                f"speedup median {ratio[len(ratio) // 2]:.0f}x, min {ratio[0]:.1f}x"
            )
    sys.exit(1 if mismatches else 0)


if __name__ == "__main__":
    main()
