#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
Generate LaTeX table: Unknown Dynamic benchmark with Heat Weight and N ablation.

Compares multiple configurations (heat weight, number of segments N) across
easy/medium/hard environments in unknown dynamic obstacle scenarios.

Uses the same analysis pipeline as analyze_dynamic_benchmark.py (including rosbag
collision analysis, computation data merging, etc.) to ensure consistent results.

Usage:
    # Default paths
    python3 generate_unknown_dynamic_table.py

    # Custom base directory
    python3 generate_unknown_dynamic_table.py \
        --base-dir /path/to/benchmark_data/unknown_dynamic

    # Specify output location
    python3 generate_unknown_dynamic_table.py \
        --output /path/to/tables/unknown_dynamic_sim.tex
"""

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple


# Import analysis functions from analyze_dynamic_benchmark.py
SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
from analyze_dynamic_benchmark import (
    load_benchmark_data,
    load_computation_data,
    merge_computation_data,
    compute_statistics,
    analyze_collision_from_trajs_bag,
    recompute_violations_from_bag,
    recompute_metrics_from_bag,
    HAS_ROSBAG,
)

# --------------------------------------------------------------------------
# Default paths
# --------------------------------------------------------------------------
BASE_DIR = None  # Must be set via --base-dir CLI arg
OUTPUT_FILE = None  # Must be set via --output CLI arg

# Configuration directories and their (heat_weight, N) parameters
CONFIGS = [
    ("inflate_unknown_voxels_heat_w_5", 5, 3),
    ("inflate_unknown_voxels_heat_w_10_N_2", 10, 2),
    ("inflate_unknown_voxels_heat_w_10", 10, 3),
]

# Cases in display order
CASES = ["easy", "medium", "hard"]
CASE_LABELS = {"easy": "Easy", "medium": "Medium", "hard": "Hard"}

# Default drone bounding box half-extents
DRONE_BBOX = (0.1, 0.1, 0.1)

# Goal position for path length recomputation
GOAL_POS = (105.0, 0.0, 2.0)


# --------------------------------------------------------------------------
# Data loading
# --------------------------------------------------------------------------
def find_case_dir(base_dir: Path, case: str) -> Optional[Path]:
    """Find the directory for a given case (exact match or glob)."""
    exact = base_dir / case
    if exact.exists():
        return exact
    matching = sorted(base_dir.glob(f"{case}_*"))
    if matching:
        return matching[-1]
    return None


def analyze_case(case_dir: Path, case_label: str) -> Optional[dict]:
    """Run the full analysis pipeline for a single case directory."""
    print(f"  Loading {case_label} from {case_dir.name}...")

    # 1. Load benchmark data
    df = load_benchmark_data(str(case_dir))
    if df is None or df.empty:
        print(f"    Warning: No benchmark data found in {case_dir}")
        return None

    # 2. Load and merge computation data
    if case_dir.is_dir():
        print("    Loading computation time data...")
        comp_stats = load_computation_data(case_dir)
        if comp_stats:
            df = merge_computation_data(df, comp_stats)
            print("    Computation data merged successfully")
        else:
            print("    No computation data found (num_*.csv files)")

    # 3. Recompute path length from rosbags
    if case_dir.is_dir():
        bags_dir = case_dir / "bags"
        if bags_dir.exists() and HAS_ROSBAG:
            print("    Recomputing path length from rosbags...")
            for idx, row in df.iterrows():
                trial_id = row["trial_id"]
                bag_path = bags_dir / f"trial_{trial_id}"
                if bag_path.exists():
                    metrics = recompute_metrics_from_bag(bag_path, GOAL_POS)
                    if metrics["path_length"] is not None:
                        df.at[idx, "path_length"] = metrics["path_length"]

    # 4. Analyze collisions from rosbags (unknown dynamic uses trajs_ground_truth)
    if case_dir.is_dir():
        bags_dir = case_dir / "bags"
        if bags_dir.exists() and HAS_ROSBAG:
            print("    Analyzing collisions from rosbags (trajs_ground_truth)...")
            for idx, row in df.iterrows():
                trial_id = row["trial_id"]
                bag_path = bags_dir / f"trial_{trial_id}"
                if bag_path.exists():
                    collision_result = analyze_collision_from_trajs_bag(
                        bag_path, DRONE_BBOX, trajs_topic="/trajs_ground_truth"
                    )
                    df.at[idx, "collision_count"] = collision_result["collision_count"]
                    df.at[idx, "min_distance_to_obstacles"] = collision_result[
                        "min_distance"
                    ]
                    df.at[idx, "collision_free_ratio"] = collision_result[
                        "collision_free_ratio"
                    ]
                    df.at[idx, "collision"] = collision_result["collision_count"] > 0
                else:
                    print(f"      Warning: Bag not found for trial {trial_id}")
            print("    Collision analysis complete")
        elif bags_dir.exists() and not HAS_ROSBAG:
            print(
                "    Warning: Bags found but rosbag2_py not available, skipping collision analysis"
            )

    # 5. Recompute constraint violations from rosbags
    if case_dir.is_dir():
        bags_dir = case_dir / "bags"
        if bags_dir.exists() and HAS_ROSBAG:
            print("    Recomputing constraint violations from rosbags...")
            for idx, row in df.iterrows():
                trial_id = row["trial_id"]
                bag_path = bags_dir / f"trial_{trial_id}"
                if bag_path.exists():
                    viol_result = recompute_violations_from_bag(bag_path)
                    df.at[idx, "vel_violation_count"] = viol_result[
                        "vel_violation_count"
                    ]
                    df.at[idx, "vel_violation_total"] = viol_result["vel_total"]
                    df.at[idx, "acc_violation_count"] = viol_result[
                        "acc_violation_count"
                    ]
                    df.at[idx, "acc_violation_total"] = viol_result["acc_total"]
                    df.at[idx, "jerk_violation_count"] = viol_result[
                        "jerk_violation_count"
                    ]
                    df.at[idx, "jerk_violation_total"] = viol_result["jerk_total"]
                else:
                    print(f"      Warning: Bag not found for trial {trial_id}")
            print("    Constraint violation analysis complete")

    # 6. Compute statistics
    stats = compute_statistics(df, require_collision_free=True)
    print(
        f"    {stats.get('total_trials', 0)} trials, {stats.get('success_rate', 0):.1f}% success"
    )

    return stats


def load_all_cases(base_dir: Path) -> Dict[str, dict]:
    """Load and analyze all cases in a benchmark directory."""
    results = {}
    for case in CASES:
        case_dir = find_case_dir(base_dir, case)
        if case_dir is None:
            print(f"  Warning: No {case} directory found in {base_dir}")
            continue

        stats = analyze_case(case_dir, CASE_LABELS[case])
        if stats is not None:
            results[case] = stats

    return results


# --------------------------------------------------------------------------
# LaTeX generation
# --------------------------------------------------------------------------

# Columns: (stat_key, latex_header, higher_is_better, precision)
COLUMNS = [
    ("success_rate", r"$R_{\mathrm{succ}}$ [\%]", True, 1),
    ("avg_local_traj_time_mean", r"$T^{\mathrm{per}}_{\mathrm{opt}}$ [ms]", False, 1),
    ("flight_travel_time_mean", r"$T_{\mathrm{trav}}$ [s]", False, 1),
    ("path_length_mean", r"$L_{\mathrm{path}}$ [m]", False, 1),
    ("jerk_integral_mean", r"$S_{\mathrm{jerk}}$ [m/s$^{2}$]", False, 1),
    ("min_distance_to_obstacles_mean", r"$d_{\mathrm{min}}$ [m]", True, 3),
    ("vel_violation_rate", r"$\rho_{\mathrm{vel}}$ [\%]", False, 1),
    ("acc_violation_rate", r"$\rho_{\mathrm{acc}}$ [\%]", False, 1),
    ("jerk_violation_rate", r"$\rho_{\mathrm{jerk}}$ [\%]", False, 1),
]


def format_val(val, best, worst, precision):
    """Format a cell value with \\best / \\worst highlighting."""
    if val is None or (isinstance(val, float) and math.isnan(val)):
        return "{-}"
    if isinstance(val, str):
        return val
    formatted = f"{val:.{precision}f}"
    tol = 10 ** (-precision) * 0.5
    if best is not None and abs(val - best) < tol:
        return rf"\best{{{formatted}}}"
    if worst is not None and abs(val - worst) < tol:
        return rf"\worst{{{formatted}}}"
    return formatted


# Each row is: (case_key, case_label, heat_w, n_seg, stats_dict)
RowTuple = Tuple[str, str, int, int, dict]


def generate_latex(rows: list) -> str:
    """Generate the full LaTeX table.

    Args:
        rows: list of (case_key, case_label, heat_w, n_seg, stats_dict) tuples.
    """

    if not rows:
        return "% ERROR: No data to generate table"

    # Find per-case best/worst for each column
    case_best_worst: Dict[str, Dict[str, Tuple]] = {}
    for case in CASES:
        case_rows = [r for r in rows if r[0] == case]
        bw = {}
        for col_key, _, higher_better, _ in COLUMNS:
            vals = [r[4].get(col_key) for r in case_rows]
            vals = [
                v
                for v in vals
                if v is not None
                and not isinstance(v, str)
                and not (isinstance(v, float) and math.isnan(v))
            ]
            if len(vals) < 2:
                bw[col_key] = (None, None)
            elif higher_better:
                bw[col_key] = (max(vals), min(vals))
            else:
                bw[col_key] = (min(vals), max(vals))
        case_best_worst[case] = bw

    # Build LaTeX
    lines = []
    lines.append(r"\begin{table*}")
    lines.append(
        r"  \caption{Benchmark results in unknown dynamic environments. "
        r"SANDO navigates using only pointcloud sensing (no ground truth obstacle trajectories). "
        r"We compare different heat map weights ($w$) and trajectory segment counts ($N$). "
        r"We highlight the \best{best} and \worst{worst} value for each environment.}"
    )
    lines.append(r"  \label{tab:unknown_dynamic_benchmark}")
    lines.append(r"  \centering")
    lines.append(r"  \renewcommand{\arraystretch}{1.2}")
    lines.append(r"  \resizebox{\textwidth}{!}{")

    n_data_cols = len(COLUMNS)
    # Columns: Env | Heat Weight | N | data columns...
    col_spec = "c c c " + " ".join(["c"] * n_data_cols)
    lines.append(f"    \\begin{{tabular}}{{{col_spec}}}")
    lines.append(r"      \toprule")

    # Header row 1: grouped
    lines.append(r"      \multirow{2}{*}[-0.4em]{\textbf{Env}}")
    lines.append(r"      & \multirow{2}{*}[-0.4em]{\textbf{$w$}}")
    lines.append(r"      & \multirow{2}{*}[-0.4em]{\textbf{$N$}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Success}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Comp. Time}}")
    lines.append(r"      & \multicolumn{3}{c}{\textbf{Performance}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Safety}}")
    lines.append(r"      & \multicolumn{3}{c}{\textbf{Constraint Violation}}")
    lines.append(r"      \\")

    # cmidrules (column indices: 1=Env, 2=w, 3=N, 4..12=data)
    lines.append(r"      \cmidrule(lr){4-4}")
    lines.append(r"      \cmidrule(lr){5-5}")
    lines.append(r"      \cmidrule(lr){6-8}")
    lines.append(r"      \cmidrule(lr){9-9}")
    lines.append(r"      \cmidrule(lr){10-12}")

    # Header row 2: individual column names
    header_parts = ["      &&&"]
    for i, (_, latex_hdr, _, _) in enumerate(COLUMNS):
        sep = " &" if i < len(COLUMNS) - 1 else ""
        header_parts.append(f"\n      {latex_hdr}{sep}")
    lines.append("".join(header_parts))
    lines.append(r"      \\")
    lines.append(r"      \midrule")

    # Data rows
    for ci, case in enumerate(CASES):
        case_rows = [r for r in rows if r[0] == case]
        n_rows_in_case = len(case_rows)

        for ri, (_, label, heat_w, n_seg, stats) in enumerate(case_rows):
            # Env cell (multirow on first row of this case)
            if ri == 0:
                case_cell = rf"\multirow{{{n_rows_in_case}}}{{*}}{{{label}}}"
            else:
                case_cell = ""

            parts = [f"      {case_cell} & {heat_w} & {n_seg}"]

            bw = case_best_worst[case]
            for col_key, _, _, prec in COLUMNS:
                val = stats.get(col_key)
                best, worst = bw.get(col_key, (None, None))
                parts.append(format_val(val, best, worst, prec))

            lines.append(" & ".join(parts) + r" \\")

        # Midrule between cases (not after last)
        if ci < len(CASES) - 1:
            lines.append(r"      \midrule")

    lines.append(r"      \bottomrule")
    lines.append(r"    \end{tabular}")
    lines.append(r"  }")
    lines.append(r"  \vspace{-1.0em}")
    lines.append(r"\end{table*}")

    return "\n".join(lines)


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Generate LaTeX table: Unknown Dynamic benchmark with Heat Weight and N ablation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--base-dir",
        type=str,
        required=True,
        help="Base directory containing config subdirectories",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        required=True,
        help="Output .tex file path",
    )
    args = parser.parse_args()

    base_dir = Path(args.base_dir)
    output_file = Path(args.output)

    print("=" * 80)
    print("UNKNOWN DYNAMIC BENCHMARK TABLE GENERATOR")
    print("=" * 80)

    # Load data for each configuration
    # all_config_data: list of (heat_w, n_seg, case_data_dict)
    all_config_data = []

    for step, (subdir_name, heat_w, n_seg) in enumerate(CONFIGS, 1):
        config_dir = base_dir / subdir_name
        if not config_dir.exists():
            print(f"\n  WARNING: Directory not found: {config_dir}")
            continue

        print(
            f"\n[{step}/{len(CONFIGS)}] Loading w={heat_w}, N={n_seg} from: {config_dir.name}"
        )
        case_data = load_all_cases(config_dir)
        if case_data:
            all_config_data.append((heat_w, n_seg, case_data))

    if not all_config_data:
        print("\nERROR: No data loaded for any configuration.")
        sys.exit(1)

    # Build row tuples: (case_key, case_label, heat_w, n_seg, stats)
    rows = []
    for case in CASES:
        label = CASE_LABELS[case]
        for heat_w, n_seg, case_data in all_config_data:
            if case in case_data:
                rows.append((case, label, heat_w, n_seg, case_data[case]))

    # Generate LaTeX
    print("\nGenerating LaTeX table...")
    latex_code = generate_latex(rows)

    # Save
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(latex_code)

    print(f"\nOutput saved to: {output_file}")
    print(f"Include in paper: \\input{{tables/{output_file.name}}}")

    # Print summary
    print(f"\n{'=' * 80}")
    print("SUMMARY")
    print(f"{'=' * 80}")
    for case in CASES:
        label = CASE_LABELS[case]
        print(f"  {label}:")
        for heat_w, n_seg, case_data in all_config_data:
            sr = case_data.get(case, {}).get("success_rate", "N/A")
            sr_str = f"{sr:.1f}%" if isinstance(sr, (int, float)) else sr
            print(f"    w={heat_w:2d}, N={n_seg}  Success: {sr_str}")
    print(f"{'=' * 80}\n")


if __name__ == "__main__":
    main()
