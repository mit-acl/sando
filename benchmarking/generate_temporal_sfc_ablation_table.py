#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) Anonymous Author
# Anonymous Institution
# All Rights Reserved
# Authors: Anonymous
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
Generate LaTeX ablation table: Temporal SFC vs Worst-Case SFC

Compares the layered temporal safe flight corridor approach (environment_assumption="dynamic")
against the worst-case conservative baseline (environment_assumption="dynamic_worst_case").

Uses the same analysis pipeline as analyze_dynamic_benchmark.py (including rosbag collision
analysis, computation data merging, etc.) to ensure consistent results.

Usage:
    # Default paths (temporal and worst-case auto-detected)
    python3 generate_temporal_sfc_ablation_table.py

    # Custom directories
    python3 generate_temporal_sfc_ablation_table.py \
        --temporal-dir benchmark_data/dynamic_worst_case/temporal \
        --worst-case-dir benchmark_data/dynamic_worst_case/worst_case

    # Specify output location
    python3 generate_temporal_sfc_ablation_table.py \
        --output /path/to/tables/temporal_sfc_ablation.tex
"""

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple


# Import analysis functions from analyze_dynamic_benchmark.py
# so we use the exact same pipeline (rosbag collision analysis, computation data, etc.)
SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
from analyze_dynamic_benchmark import (
    load_benchmark_data,
    load_computation_data,
    merge_computation_data,
    compute_statistics,
    analyze_collision_from_bag,
    recompute_violations_from_bag,
    HAS_ROSBAG,
)

# --------------------------------------------------------------------------
# Default paths
# --------------------------------------------------------------------------
BENCHMARK_ROOT = Path(__file__).resolve().parent.parent / "benchmark_data"
TEMPORAL_DIR = BENCHMARK_ROOT / "dynamic_worst_case" / "temporal"
WORST_CASE_DIR = BENCHMARK_ROOT / "dynamic_worst_case" / "worst_case"
OUTPUT_FILE = None  # Must be set via --output CLI arg

# Cases in display order
CASES = ["easy", "medium", "hard"]
CASE_LABELS = {"easy": "Easy", "medium": "Medium", "hard": "Hard"}

# Default drone bounding box half-extents (same as analyze_dynamic_benchmark.py)
DRONE_BBOX = (0.1, 0.1, 0.1)


# --------------------------------------------------------------------------
# Data loading — delegates to analyze_dynamic_benchmark functions
# --------------------------------------------------------------------------
def find_case_dir(base_dir: Path, case: str) -> Optional[Path]:
    """Find the directory for a given case.

    Checks for exact match first (e.g. 'easy'), then glob pattern (e.g. 'easy_*').
    """
    # Exact match
    exact = base_dir / case
    if exact.exists():
        return exact
    # Glob pattern (e.g. easy_20260303)
    matching = sorted(base_dir.glob(f"{case}_*"))
    if matching:
        return matching[-1]
    return None


def analyze_case(case_dir: Path, case_label: str) -> Optional[dict]:
    """Run the full analysis pipeline for a single case directory.

    This matches what analyze_dynamic_benchmark.py does:
    1. Load benchmark CSV (most recent)
    2. Load and merge computation data from csv/num_*.csv
    3. Analyze collisions from rosbags
    4. Compute statistics
    """
    print(f"  Loading {case_label} from {case_dir.name}...")

    # 1. Load benchmark CSV
    df = load_benchmark_data(str(case_dir))
    if df is None or df.empty:
        print(f"    Warning: No benchmark CSV found in {case_dir}")
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

    # 3. Analyze collisions from rosbags
    if case_dir.is_dir():
        bags_dir = case_dir / "bags"
        if bags_dir.exists() and HAS_ROSBAG:
            print("    Analyzing collisions from rosbags...")
            for idx, row in df.iterrows():
                trial_id = row["trial_id"]
                bag_path = bags_dir / f"trial_{trial_id}"
                if bag_path.exists():
                    collision_result = analyze_collision_from_bag(bag_path, DRONE_BBOX)
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

    # 3b. Recompute constraint violations from rosbags
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

    # 4. Compute statistics
    stats = compute_statistics(df)
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
    formatted = f"{val:.{precision}f}"
    tol = 10 ** (-precision) * 0.5  # half of the last displayed digit
    if best is not None and abs(val - best) < tol:
        return rf"\best{{{formatted}}}"
    if worst is not None and abs(val - worst) < tol:
        return rf"\worst{{{formatted}}}"
    return formatted


# Each row is: (case_key, case_label, sfc_mode, stats_dict)
RowTuple = Tuple[str, str, str, dict]


def generate_latex(rows: list) -> str:
    """Generate the full LaTeX table with Case + SFC Mode columns (no Unk Infl.).

    Args:
        rows: list of (case_key, case_label, sfc_mode, stats_dict) tuples.
    """

    if not rows:
        return "% ERROR: No data to generate table"

    # Find per-case best/worst for each column
    case_best_worst: Dict[str, Dict[str, Tuple]] = {}
    for case in CASES:
        case_rows = [r for r in rows if r[0] == case]
        bw = {}
        for col_key, _, higher_better, _ in COLUMNS:
            vals = [r[3].get(col_key) for r in case_rows]
            vals = [
                v
                for v in vals
                if v is not None and not (isinstance(v, float) and math.isnan(v))
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
        r"  \caption{Ablation study: temporal safe flight corridor (SFC) vs worst-case SFC. "
        r"The temporal approach uses per-layer obstacle inflation $r = v_{\max}^{\mathrm{obs}} \times t_n$, "
        r"while the worst-case baseline inflates all obstacles by the maximum time horizon. "
        r"We highlight the \best{better} and \worst{worse} value for each case.}"
    )
    lines.append(r"  \label{tab:temporal_sfc_ablation}")
    lines.append(r"  \centering")
    lines.append(r"  \renewcommand{\arraystretch}{1.2}")
    lines.append(r"  \resizebox{\textwidth}{!}{")

    n_data_cols = len(COLUMNS)
    # Columns: Case | SFC Mode | data columns...
    col_spec = "c c " + " ".join(["c"] * n_data_cols)
    lines.append(f"    \\begin{{tabular}}{{{col_spec}}}")
    lines.append(r"      \toprule")

    # Header row 1: grouped
    lines.append(r"      \multirow{2}{*}[-0.4em]{\textbf{Case}}")
    lines.append(r"      & \multirow{2}{*}[-0.4em]{\textbf{SFC Mode}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Success}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Comp. Time}}")
    lines.append(r"      & \multicolumn{3}{c}{\textbf{Performance}}")
    lines.append(r"      & \multicolumn{1}{c}{\textbf{Safety}}")
    lines.append(r"      & \multicolumn{3}{c}{\textbf{Constraint Violation}}")
    lines.append(r"      \\")

    # cmidrules (column indices: 1=Case, 2=SFC Mode, 3..11=data)
    lines.append(r"      \cmidrule(lr){3-3}")
    lines.append(r"      \cmidrule(lr){4-4}")
    lines.append(r"      \cmidrule(lr){5-7}")
    lines.append(r"      \cmidrule(lr){8-8}")
    lines.append(r"      \cmidrule(lr){9-11}")

    # Header row 2: individual column names
    header_parts = ["      &&"]
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

        for ri, (_, label, sfc_mode, stats) in enumerate(case_rows):
            # Case cell (multirow on first row of this case)
            if ri == 0:
                case_cell = rf"\multirow{{{n_rows_in_case}}}{{*}}{{{label}}}"
            else:
                case_cell = ""

            parts = [f"      {case_cell} & {sfc_mode}"]

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
        description="Generate LaTeX ablation table: Temporal SFC vs Worst-Case SFC",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--temporal-dir",
        type=str,
        default=str(TEMPORAL_DIR),
        help=f"Directory with temporal SFC benchmark data (default: {TEMPORAL_DIR})",
    )
    parser.add_argument(
        "--worst-case-dir",
        type=str,
        default=str(WORST_CASE_DIR),
        help=f"Directory with worst-case SFC benchmark data (default: {WORST_CASE_DIR})",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        required=True,
        help="Output .tex file path",
    )
    args = parser.parse_args()

    temporal_dir = Path(args.temporal_dir)
    worst_case_dir = Path(args.worst_case_dir)
    output_file = Path(args.output)

    print("=" * 80)
    print("TEMPORAL SFC ABLATION TABLE GENERATOR")
    print("=" * 80)

    # (sfc_mode_label, data_dir)
    config_entries = []

    for sfc_label, base_dir in [
        ("Worst-Case", worst_case_dir),
        ("Temporal", temporal_dir),
    ]:
        if not base_dir.exists():
            print(f"\n  WARNING: Directory not found: {base_dir}")
            continue
        config_entries.append((sfc_label, base_dir))

    if not config_entries:
        print("\nERROR: No data directories found.")
        sys.exit(1)

    # Load data for each config entry
    # all_case_data: list of (sfc_label, case_data_dict)
    all_case_data = []

    for step, (sfc_label, data_dir) in enumerate(config_entries, 1):
        print(f"\n[{step}/{len(config_entries)}] Loading {sfc_label} from: {data_dir}")
        case_data = load_all_cases(data_dir)
        if case_data:
            all_case_data.append((sfc_label, case_data))

    if not all_case_data:
        print("\nERROR: No data loaded for any configuration.")
        sys.exit(1)

    # Build row tuples: (case_key, case_label, sfc_mode, stats)
    rows = []
    for case in CASES:
        label = CASE_LABELS[case]
        for sfc_label, case_data in all_case_data:
            if case in case_data:
                rows.append((case, label, sfc_label, case_data[case]))

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
        for sfc_label, case_data in all_case_data:
            sr = case_data.get(case, {}).get("success_rate", "N/A")
            sr_str = f"{sr:.1f}%" if isinstance(sr, (int, float)) else sr
            print(f"    {sfc_label:12s}  Success: {sr_str}")
    print(f"{'=' * 80}\n")


if __name__ == "__main__":
    main()
