#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""Generate SFC ablation LaTeX table from benchmark data.

Runs analyze_benchmark on 4 data directories (2 velocities x 2 SFC modes)
and produces a LaTeX table with best/worst highlighting per velocity group.
"""

import csv
import subprocess
import sys
import os

# Configuration: (label, velocity, data_dir)
CONFIGS = [
    (
        "Worst-Case",
        2.5,
        os.path.expanduser(
            "~/code/sando_ws/src/sando/benchmark_data/dynamic_worst_case/vel_2.5/hard"
        ),
    ),
    (
        "SANDO2 (STSFC)",
        2.5,
        os.path.expanduser(
            "~/code/sando_ws/src/sando/benchmark_data/dynamic/vel_2.5/hard_20260307_133859"
        ),
    ),
    (
        "Worst-Case",
        5.0,
        os.path.expanduser(
            "~/code/sando_ws/src/sando/benchmark_data/dynamic_worst_case/vel_5/N_3/hard"
        ),
    ),
    (
        "SANDO2 (STSFC)",
        5.0,
        os.path.expanduser(
            "~/code/sando_ws/src/sando/benchmark_data/dynamic/vel_5/N_3/hard_20260306_170520"
        ),
    ),
]

ANALYZE_BIN = os.path.expanduser(
    "~/code/sando_ws/install/sando/lib/sando/analyze_benchmark"
)

OUTPUT_TEX = os.path.expanduser("~/paper_writing/SANDO_paper/tables/sfc_ablation.tex")


def run_analysis(data_dir):
    """Run analyze_benchmark and read stats from benchmark_summary.csv.

    analyze_benchmark writes full-precision values to benchmark_summary.csv.
    Reading from that avoids double-rounding errors from parsing text output.
    """
    cmd = (
        f"source {os.path.expanduser('~/code/sando_ws/install/setup.bash')} && "
        f"{ANALYZE_BIN} --data-dir {data_dir} --table-type dynamic"
    )
    result = subprocess.run(
        ["bash", "-c", cmd], capture_output=True, text=True, timeout=120
    )
    if result.returncode != 0:
        print(f"ERROR running on {data_dir}:")
        print(result.stderr)
        sys.exit(1)

    summary_csv = os.path.join(data_dir, "benchmark_summary.csv")
    with open(summary_csv) as f:
        row = next(csv.DictReader(f))

    return {
        "success_rate": float(row["success_rate"]),
        "comp_time": float(row["avg_local_traj_time_mean"]),
        "travel_time": float(row["flight_travel_time_mean"]),
        "path_length": float(row["path_length_mean"]),
        "jerk_integral": float(row["jerk_integral_mean"]),
        "min_dist": float(row["min_distance_to_obstacles_mean"]),
        "vel_viol": float(row["vel_violation_rate"]),
        "acc_viol": float(row["acc_violation_rate"]),
        "jerk_viol": float(row["jerk_violation_rate"]),
    }


def fmt(val, decimals=1):
    """Format a float value."""
    return f"{val:.{decimals}f}"


def generate_table(rows):
    """Generate LaTeX table with best/worst highlighting per velocity group.

    rows: list of (sfc_mode, velocity, stats_dict)
    """
    metrics = [
        "success_rate",
        "comp_time",
        "travel_time",
        "path_length",
        "jerk_integral",
        "min_dist",
        "vel_viol",
        "acc_viol",
        "jerk_viol",
    ]
    # higher-is-better for these metrics
    higher_better = {"success_rate", "min_dist"}

    # Group by velocity
    vel_groups = {}
    for sfc_mode, vel, stats in rows:
        vel_groups.setdefault(vel, []).append((sfc_mode, stats))

    # Compute best/worst per velocity group
    highlights = {}  # (vel, sfc_mode, metric) -> 'best' | 'worst' | None
    for vel, group in vel_groups.items():
        for metric in metrics:
            vals = [(sfc_mode, stats[metric]) for sfc_mode, stats in group]
            sorted_vals = sorted(vals, key=lambda x: x[1])
            if len(set(v for _, v in vals)) > 1:  # only highlight if values differ
                if metric in higher_better:
                    highlights[(vel, sorted_vals[-1][0], metric)] = "best"
                    highlights[(vel, sorted_vals[0][0], metric)] = "worst"
                else:
                    highlights[(vel, sorted_vals[0][0], metric)] = "best"
                    highlights[(vel, sorted_vals[-1][0], metric)] = "worst"
            else:
                # All same value - mark as best
                for sfc_mode, _ in vals:
                    highlights[(vel, sfc_mode, metric)] = "best"

    def wrap(val_str, vel, sfc_mode, metric):
        key = (vel, sfc_mode, metric)
        h = highlights.get(key)
        if h == "best":
            return f"\\best{{{val_str}}}"
        elif h == "worst":
            return f"\\worst{{{val_str}}}"
        return val_str

    lines = []
    lines.append(r"\begin{table*}")
    lines.append(
        r"  \caption{SFC ablation study. We compare Worst-Case SFC and Spatio-Temporal SFC (STSFC) at different maximum velocities in the Hard dynamic environment. We highlight the \best{best} and \worst{worst} value for each velocity.}"
    )
    lines.append(r"  \label{tab:sfc_ablation}")
    lines.append(r"  \centering")
    lines.append(r"  \renewcommand{\arraystretch}{1.2}")
    lines.append(r"  \resizebox{\textwidth}{!}{")
    lines.append(r"    \begin{tabular}{c c c c c c c c c c c}")
    lines.append(r"      \toprule")
    lines.append(r"      \multirow{2}{*}[-0.4em]{\textbf{$v_{\max}$ [m/s]}}")
    lines.append(r"      & \multirow{2}{*}[-0.4em]{\textbf{SFC Mode}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Success}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Comp. Time}}")
    lines.append(r"      & \multicolumn{3}{c}{\textbf{Performance}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Safety}}")
    lines.append(r"      & \multicolumn{3}{c}{\textbf{Constraint Violation}}")
    lines.append(r"      \\")
    lines.append(r"      \cmidrule(lr){3-3}")
    lines.append(r"      \cmidrule(lr){4-4}")
    lines.append(r"      \cmidrule(lr){5-7}")
    lines.append(r"      \cmidrule(lr){8-8}")
    lines.append(r"      \cmidrule(lr){9-11}")
    lines.append(r"      &&")
    lines.append(r"      $R_{\mathrm{succ}}$ [\%] &")
    lines.append(r"      $T^{\mathrm{per}}_{\mathrm{opt}}$ [ms] &")
    lines.append(r"      $T_{\mathrm{trav}}$ [s] &")
    lines.append(r"      $L_{\mathrm{path}}$ [m] &")
    lines.append(r"      $S_{\mathrm{jerk}}$ [m/s$^{2}$] &")
    lines.append(r"      $d_{\mathrm{min}}$ [m] &")
    lines.append(r"      $\rho_{\mathrm{vel}}$ [\%] &")
    lines.append(r"      $\rho_{\mathrm{acc}}$ [\%] &")
    lines.append(r"      $\rho_{\mathrm{jerk}}$ [\%]")
    lines.append(r"      \\")
    lines.append(r"      \midrule")

    sorted_vels = sorted(vel_groups.keys())
    for vi, vel in enumerate(sorted_vels):
        group = vel_groups[vel]
        n_rows = len(group)
        for ri, (sfc_mode, stats) in enumerate(group):
            parts = []
            if ri == 0:
                parts.append(f"      \\multirow{{{n_rows}}}{{*}}{{{fmt(vel, 1)}}}")
            else:
                parts.append("      ")

            parts.append(f" & {sfc_mode}")
            parts.append(
                f" & {wrap(fmt(stats['success_rate'], 1), vel, sfc_mode, 'success_rate')}"
            )
            parts.append(
                f" & {wrap(fmt(stats['comp_time'], 1), vel, sfc_mode, 'comp_time')}"
            )
            parts.append(
                f" & {wrap(fmt(stats['travel_time'], 1), vel, sfc_mode, 'travel_time')}"
            )
            parts.append(
                f" & {wrap(fmt(stats['path_length'], 1), vel, sfc_mode, 'path_length')}"
            )
            parts.append(
                f" & {wrap(fmt(stats['jerk_integral'], 1), vel, sfc_mode, 'jerk_integral')}"
            )
            parts.append(
                f" & {wrap(fmt(stats['min_dist'], 2), vel, sfc_mode, 'min_dist')}"
            )
            parts.append(
                f" & {wrap(fmt(stats['vel_viol'], 1), vel, sfc_mode, 'vel_viol')}"
            )
            parts.append(
                f" & {wrap(fmt(stats['acc_viol'], 1), vel, sfc_mode, 'acc_viol')}"
            )
            parts.append(
                f" & {wrap(fmt(stats['jerk_viol'], 1), vel, sfc_mode, 'jerk_viol')}"
            )
            parts.append(" \\\\")
            lines.append("".join(parts))

        if vi < len(sorted_vels) - 1:
            lines.append(r"      \midrule")

    lines.append(r"      \bottomrule")
    lines.append(r"    \end{tabular}")
    lines.append(r"  }")
    lines.append(r"  \vspace{-1.0em}")
    lines.append(r"\end{table*}")

    return "\n".join(lines)


def main():
    print("=" * 80)
    print("SFC ABLATION TABLE GENERATOR")
    print("=" * 80)

    rows = []
    for sfc_mode, vel, data_dir in CONFIGS:
        print(f"\nProcessing: {sfc_mode} v_max={vel} ({data_dir})")
        if not os.path.exists(data_dir):
            print(f"  ERROR: Directory not found: {data_dir}")
            sys.exit(1)
        stats = run_analysis(data_dir)
        print(
            f"  Success: {stats['success_rate']}%, Comp: {stats['comp_time']}ms, "
            f"Travel: {stats['travel_time']}s, MinDist: {stats['min_dist']}m"
        )
        rows.append((sfc_mode, vel, stats))

    table = generate_table(rows)

    os.makedirs(os.path.dirname(OUTPUT_TEX), exist_ok=True)
    with open(OUTPUT_TEX, "w") as f:
        f.write(table)

    print(f"\n{'=' * 80}")
    print(f"TABLE GENERATED: {OUTPUT_TEX}")
    print(f"{'=' * 80}\n")
    print(table)


if __name__ == "__main__":
    main()
