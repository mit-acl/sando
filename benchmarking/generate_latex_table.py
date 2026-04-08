#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) Anonymous Author
# Anonymous Institution
# All Rights Reserved
# Authors: Anonymous
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
Generate LaTeX table from benchmark data for SANDO paper

This script reads the CSV benchmark data and generates a properly formatted
LaTeX table with best/worst highlighting.

Usage:
    python3 generate_latex_table.py
"""

import argparse
import pandas as pd
import numpy as np
import sys
from pathlib import Path

# Configuration — defaults are relative to the package; paper paths require CLI args.
ROOT_PATH = Path(__file__).resolve().parent.parent / "benchmark_data"
OUTPUT_FILE = None  # Must be set via --output CLI arg
VE_OUTPUT_FILE = None  # Must be set via --ve-output CLI arg

# Data files to load
DATA_FILES = {
    # (mode, planner, N) -> filename
    # BASELINE original (only first control point constrained)
    ("single", "baseline_orig", 4): "single_thread/original_baseline_4_benchmark.csv",
    ("single", "baseline_orig", 5): "single_thread/original_baseline_5_benchmark.csv",
    ("single", "baseline_orig", 6): "single_thread/original_baseline_6_benchmark.csv",
    # BASELINE-CP (all control points constrained)
    ("single", "safe_baseline", 4): "single_thread/safe_baseline_4_benchmark.csv",
    ("single", "safe_baseline", 5): "single_thread/safe_baseline_5_benchmark.csv",
    ("single", "safe_baseline", 6): "single_thread/safe_baseline_6_benchmark.csv",
    # SANDO2 single
    ("single", "sando", 4): "single_thread/sando_4_benchmark.csv",
    ("single", "sando", 5): "single_thread/sando_5_benchmark.csv",
    ("single", "sando", 6): "single_thread/sando_6_benchmark.csv",
    # SANDO2 multi
    ("multi", "sando", 4): "multi_thread/sando_4_benchmark.csv",
    ("multi", "sando", 5): "multi_thread/sando_5_benchmark.csv",
    ("multi", "sando", 6): "multi_thread/sando_6_benchmark.csv",
}

# SUPER data: loaded from external CSV files (L2 and Linf norm variants)
# Set via --super-l2-csv and --super-linf-csv CLI args
SUPER_CSV_FILES = {}


def safe_mean(series):
    """Compute mean, handling NaN"""
    s = series.dropna()
    return float(s.mean()) if not s.empty else np.nan


def _load_one_super_csv(csv_path, label):
    """Load a single SUPER benchmark CSV and return a summary dict."""
    print(f"  Loading {label} data from {csv_path}")
    df = pd.read_csv(csv_path)

    df["success"] = pd.to_numeric(df["success"], errors="coerce").fillna(0).astype(int)

    success_rate = safe_mean(df["success"]) * 100
    succ_df = df[df["success"] == 1]

    per_opt_ms = safe_mean(succ_df["per_opt_runtime_ms"])
    total_opt_ms = safe_mean(succ_df["total_opt_runtime_ms"])
    traj_time_s = safe_mean(succ_df["total_traj_time_sec"])
    path_length = safe_mean(succ_df["traj_length_m"])
    jerk_smooth = safe_mean(succ_df["jerk_smoothness_l1"])

    # SUPER CSVs use _pct columns directly (percentage of trajectory samples violating)
    if "corridor_violation_pct" in succ_df.columns:
        sfc_viol = safe_mean(succ_df["corridor_violation_pct"])
        vel_viol = safe_mean(succ_df["v_violation_pct"])
        acc_viol = safe_mean(succ_df["a_violation_pct"])
        jerk_viol = safe_mean(succ_df["j_violation_pct"])
    elif (
        "violation_total_samples" in succ_df.columns
        and "v_violation_count" in succ_df.columns
    ):
        total_samples = succ_df["violation_total_samples"].sum()
        if total_samples > 0:
            sfc_viol = succ_df["corridor_violation_count"].sum() / total_samples * 100.0
            vel_viol = succ_df["v_violation_count"].sum() / total_samples * 100.0
            acc_viol = succ_df["a_violation_count"].sum() / total_samples * 100.0
            jerk_viol = succ_df["j_violation_count"].sum() / total_samples * 100.0
        else:
            sfc_viol = vel_viol = acc_viol = jerk_viol = 0.0
    else:
        sfc_viol = (
            safe_mean(succ_df["corridor_violated"]) * 100
            if "corridor_violated" in succ_df.columns
            else 0.0
        )
        vel_viol = (
            safe_mean(succ_df["v_violated"]) * 100
            if "v_violated" in succ_df.columns
            else 0.0
        )
        acc_viol = (
            safe_mean(succ_df["a_violated"]) * 100
            if "a_violated" in succ_df.columns
            else 0.0
        )
        jerk_viol = (
            safe_mean(succ_df["j_violated"]) * 100
            if "j_violated" in succ_df.columns
            else 0.0
        )

    return {
        "Algorithm": label,
        "Thread": "multi",
        "N": "--",
        "success_rate": success_rate,
        "per_opt_ms": per_opt_ms,
        "total_opt_ms": total_opt_ms,
        "traj_time_s": traj_time_s,
        "path_length": path_length,
        "jerk_smooth": jerk_smooth,
        "sfc_viol": sfc_viol,
        "vel_viol": vel_viol,
        "acc_jerk_viol": max(acc_viol, jerk_viol),
    }


def load_super_data():
    """Load all SUPER benchmark CSVs and return a list of summary dicts."""
    results = []
    for label, csv_path in SUPER_CSV_FILES.items():
        if csv_path.exists():
            results.append(_load_one_super_csv(csv_path, label))
        else:
            print(f"  WARNING: SUPER CSV not found at {csv_path}")
    return results


SUPER_DATA_LIST = load_super_data()


def load_and_process_data():
    """Load all CSV files and compute statistics"""
    rows = []

    for (mode, planner, N), filename in DATA_FILES.items():
        filepath = ROOT_PATH / filename

        if not filepath.exists():
            print(f"WARNING: Missing {filepath}")
            continue

        df = pd.read_csv(filepath)

        # Parse success column
        df["success"] = (
            pd.to_numeric(df["success"], errors="coerce").fillna(0).astype(int)
        )

        # Compute success rate (percentage)
        success_rate = safe_mean(df["success"]) * 100

        # Compute means over successful runs only
        succ_df = df[df["success"] == 1]

        per_opt_ms = safe_mean(succ_df["per_opt_runtime_ms"])
        total_opt_ms = safe_mean(succ_df["total_opt_runtime_ms"])
        traj_time_s = safe_mean(succ_df["total_traj_time_sec"])
        path_length = safe_mean(succ_df["traj_length_m"])
        jerk_smooth = safe_mean(succ_df["jerk_smoothness_l1"])

        # Violation rates: count / total * 100 (consistent with dynamic/static benchmarks)
        # New CSVs have per-sample counts; fall back to binary flags for old CSVs
        if (
            "violation_total_samples" in succ_df.columns
            and "v_violation_count" in succ_df.columns
        ):
            total_samples = succ_df["violation_total_samples"].sum()
            if total_samples > 0:
                sfc_viol = (
                    succ_df["corridor_violation_count"].sum() / total_samples * 100.0
                )
                vel_viol = succ_df["v_violation_count"].sum() / total_samples * 100.0
                acc_viol = succ_df["a_violation_count"].sum() / total_samples * 100.0
                jerk_viol = succ_df["j_violation_count"].sum() / total_samples * 100.0
            else:
                sfc_viol = vel_viol = acc_viol = jerk_viol = 0.0
        else:
            # Legacy fallback: binary per-case flags
            sfc_viol = (
                safe_mean(succ_df["corridor_violated"]) * 100
                if "corridor_violated" in succ_df.columns
                else 0.0
            )
            vel_viol = (
                safe_mean(succ_df["v_violated"]) * 100
                if "v_violated" in succ_df.columns
                else 0.0
            )
            acc_viol = (
                safe_mean(succ_df["a_violated"]) * 100
                if "a_violated" in succ_df.columns
                else 0.0
            )
            jerk_viol = (
                safe_mean(succ_df["j_violated"]) * 100
                if "j_violated" in succ_df.columns
                else 0.0
            )

        # Determine algorithm name
        if planner == "baseline_orig":
            alg_name = "BASELINE"
            alg_variant = "(orig.)"
        elif planner == "safe_baseline":
            alg_name = "BASELINE"
            alg_variant = "(CP)"
        else:  # sando
            alg_name = "SANDO"
            alg_variant = ""

        rows.append(
            {
                "Algorithm": alg_name,
                "Variant": alg_variant,
                "Thread": mode,
                "N": N,
                "success_rate": success_rate,
                "per_opt_ms": per_opt_ms,
                "total_opt_ms": total_opt_ms,
                "traj_time_s": traj_time_s,
                "path_length": path_length,
                "jerk_smooth": jerk_smooth,
                "sfc_viol": sfc_viol,
                "vel_viol": vel_viol,
                "acc_jerk_viol": max(acc_viol, jerk_viol),
            }
        )

    df = pd.DataFrame(rows)

    # Add SUPER data as first rows
    if SUPER_DATA_LIST:
        super_df = pd.DataFrame(SUPER_DATA_LIST)
        df = pd.concat([super_df, df], ignore_index=True)

    return df


def find_best_worst(df, column, higher_is_better=False):
    """Find best and worst values in a column"""
    valid = df[column].dropna()
    if valid.empty:
        return None, None

    if higher_is_better:
        best = valid.max()
        worst = valid.min()
    else:
        best = valid.min()
        worst = valid.max()

    return best, worst


def format_value(val, best, worst, precision=1, force_best_when_all_equal=False):
    """Format value with LaTeX highlighting"""
    if pd.isna(val):
        return "-"

    formatted = f"{val:.{precision}f}"

    # Compare on rounded values so tied display values are all highlighted
    val_r = round(val, precision)
    best_r = round(best, precision) if best is not None else None
    worst_r = round(worst, precision) if worst is not None else None

    # Skip highlighting if best and worst are the same (no meaningful difference)
    # unless force_best_when_all_equal is set (e.g., all-zero violation columns)
    if best_r is not None and worst_r is not None and best_r == worst_r:
        if force_best_when_all_equal:
            return f"\\best{{{formatted}}}"
        return formatted
    if best_r is not None and val_r == best_r:
        return f"\\best{{{formatted}}}"
    elif worst_r is not None and val_r == worst_r:
        return f"\\worst{{{formatted}}}"
    else:
        return formatted


def generate_sando_rows_only(df):
    """Generate only SANDO rows (not full table) for manual insertion"""

    # (column_name, latex_header, higher_is_better, precision, force_best_when_all_equal)
    columns_config = [
        ("success_rate", "$R^{\\mathrm{opt}}_{\\mathrm{succ}}$ [\\%]", True, 1, False),
        ("per_opt_ms", "$T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms]", False, 1, False),
        ("total_opt_ms", "$T^{\\mathrm{total}}_{\\mathrm{opt}}$ [ms]", False, 1, False),
        ("traj_time_s", "$T_{\\mathrm{trav}}$ [s]", False, 1, False),
        ("path_length", "$L_{\\mathrm{path}}$ [m]", False, 1, False),
        ("jerk_smooth", "$S_{\\mathrm{jerk}}$ [m/s$^{2}$]", False, 1, False),
        ("sfc_viol", "$\\rho_{\\mathrm{sfc}}$ [\\%]", False, 1, False),
        ("vel_viol", "$\\rho_{\\mathrm{vel}}$ [\\%]", False, 1, False),
        ("acc_jerk_viol", "$\\rho_{\\mathrm{acc/jerk}}$ [\\%]", False, 1, True),
    ]

    # Filter to SANDO only
    df_sando = df[df["Algorithm"] == "SANDO"].copy()

    if df_sando.empty:
        return "% No SANDO data found"

    # Find best/worst across ALL data (not just SANDO) for fair comparison
    best_worst = {}
    for col_name, _, higher_better, _, _ in columns_config:
        best, worst = find_best_worst(df, col_name, higher_better)
        best_worst[col_name] = (best, worst)

    latex = []
    latex.append("% ========== SANDO ROWS ONLY (copy into main table) ==========")
    latex.append("% Replace existing SANDO rows with these updated values")
    latex.append("")

    # Group by N
    for N_val in sorted(df_sando["N"].unique()):
        df_n = df_sando[df_sando["N"] == N_val].copy()

        # Sort: multi-thread first, then single-thread
        df_n = df_n.sort_values("Thread", ascending=False)  # multi before single

        latex.append(f"% N = {int(N_val)}")

        for idx, row in df_n.iterrows():
            thread = row["Thread"]

            # Build row (no N column - assume it's handled by multirow in main table)
            row_str = f"      SANDO & {thread} &"

            for col_name, _, _, precision, force_best in columns_config:
                val = row[col_name]
                best, worst = best_worst[col_name]
                formatted = format_value(val, best, worst, precision, force_best)
                row_str += f" & {formatted}"

            row_str += " \\\\"
            latex.append(row_str)

        latex.append("")

    return "\n".join(latex)


def generate_latex_table(df):
    """Generate FULL LaTeX table code (use only if table doesn't exist yet)"""

    # Define columns and their properties
    # (column_name, latex_header, higher_is_better, precision)
    # (column_name, latex_header, higher_is_better, precision, force_best_when_all_equal)
    columns_config = [
        ("success_rate", "$R^{\\mathrm{opt}}_{\\mathrm{succ}}$ [\\%]", True, 1, False),
        ("per_opt_ms", "$T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms]", False, 1, False),
        ("total_opt_ms", "$T^{\\mathrm{total}}_{\\mathrm{opt}}$ [ms]", False, 1, False),
        ("traj_time_s", "$T_{\\mathrm{trav}}$ [s]", False, 1, False),
        ("path_length", "$L_{\\mathrm{path}}$ [m]", False, 1, False),
        ("jerk_smooth", "$S_{\\mathrm{jerk}}$ [m/s$^{2}$]", False, 1, False),
        ("sfc_viol", "$\\rho_{\\mathrm{sfc}}$ [\\%]", False, 1, False),
        ("vel_viol", "$\\rho_{\\mathrm{vel}}$ [\\%]", False, 1, False),
        ("acc_jerk_viol", "$\\rho_{\\mathrm{acc/jerk}}$ [\\%]", False, 1, True),
    ]

    # Start building LaTeX
    latex = []
    latex.append("\\begin{table*}")
    latex.append(
        "  \\caption{Local trajectory optimization benchmarking results (computation time, performance, and constraint violation)."
    )
    latex.append(
        "  We mark in \\best{green} the best value in each column and in \\worst{red} the worst value.}"
    )
    latex.append("  \\label{tab:standardized_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\resizebox{\\textwidth}{!}{")
    latex.append("    \\begin{tabular}{c @{\\hspace{4pt}} l c c c c c c c c c c c}")
    latex.append("      \\toprule")

    # Header rows (Algorithm spans 2 columns)
    latex.append(
        "      \\multicolumn{2}{c}{\\multirow{2}{*}[-0.4ex]{\\textbf{Algorithm}}}"
    )
    latex.append("      & \\multirow{2}{*}[-0.4ex]{\\textbf{Thread}}")
    latex.append("      & \\multirow{2}{*}[-0.4ex]{\\textbf{N}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Success}}")
    latex.append("      & \\multicolumn{2}{c}{\\textbf{Computation Time}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Performance}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}")
    latex.append("      \\\\")
    latex.append("      \\cmidrule(lr){5-5}")
    latex.append("      \\cmidrule(lr){6-7}")
    latex.append("      \\cmidrule(lr){8-10}")
    latex.append("      \\cmidrule(lr){11-13}")

    # Column headers (second row)
    latex.append("      &&&&")
    latex.append("      $R^{\\mathrm{opt}}_{\\mathrm{succ}}$ [\\%] &")
    latex.append("      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &")
    latex.append("      $T^{\\mathrm{total}}_{\\mathrm{opt}}$ [ms] &")
    latex.append("      $T_{\\mathrm{trav}}$ [s] &")
    latex.append("      $L_{\\mathrm{path}}$ [m] &")
    latex.append("      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &")
    latex.append("      $\\rho_{\\mathrm{sfc}}${[\\%]} &")
    latex.append("      $\\rho_{\\mathrm{vel}}${[\\%]} &")
    latex.append("      $\\rho_{\\mathrm{acc/jerk}}${[\\%]}")
    latex.append("      \\\\")
    latex.append("      \\midrule")

    # Find best/worst for each column
    best_worst = {}
    for col_name, _, higher_better, _, _ in columns_config:
        best, worst = find_best_worst(df, col_name, higher_better)
        best_worst[col_name] = (best, worst)

    # Handle SUPER rows separately (appear first, before N-grouped rows)
    df_super = df[df["Algorithm"].str.startswith("SUPER")].copy()
    df_rest = df[~df["Algorithm"].str.startswith("SUPER")].copy()

    # Add SUPER rows if they exist
    # SUPER uses both Algorithm columns: col1=SUPER (multirow), col2=variant
    # Thread and N use multirow
    if not df_super.empty:
        n_super = len(df_super)
        # Extract variant labels from algorithm names: "SUPER ($L_2$)" -> "($L_2$)"
        super_variants = []
        for _, row in df_super.iterrows():
            alg = row["Algorithm"]
            variant = alg.replace("SUPER ", "")
            super_variants.append(variant)

        for i, (_, row) in enumerate(df_super.iterrows()):
            if i == 0:
                alg_cell = f"\\multirow{{{n_super}}}{{*}}{{SUPER}}"
                thread_cell = f"\\multirow{{{n_super}}}{{*}}{{multi}}"
                n_cell = f"\\multirow{{{n_super}}}{{*}}{{--}}"
            else:
                alg_cell = ""
                thread_cell = ""
                n_cell = ""

            variant_cell = super_variants[i]

            # Build row - SUPER has combined per/total opt time
            row_str = f"      {alg_cell} & {variant_cell} & {thread_cell} & {n_cell}"

            # Success rate
            val = row["success_rate"]
            best_sr, worst_sr = best_worst["success_rate"]
            row_str += f" & {format_value(val, best_sr, worst_sr, 1)}"

            # Combined per_opt and total_opt (use multicolumn)
            val = row["per_opt_ms"]
            best_po, worst_po = best_worst["per_opt_ms"]
            formatted = format_value(val, best_po, worst_po, 1)
            row_str += f" & \\multicolumn{{2}}{{c}}{{{formatted}}}"

            # Performance metrics
            for col_name in ["traj_time_s", "path_length", "jerk_smooth"]:
                val = row[col_name]
                best, worst = best_worst[col_name]
                formatted = format_value(val, best, worst, 1)
                row_str += f" & {formatted}"

            # Violation rates
            for col_name in ["sfc_viol", "vel_viol", "acc_jerk_viol"]:
                val = row[col_name]
                best, worst = best_worst[col_name]
                force_best = col_name == "acc_jerk_viol"
                formatted = format_value(val, best, worst, 1, force_best)
                row_str += f" & {formatted}"

            row_str += " \\\\"
            latex.append(row_str)

        latex.append("")
        latex.append("      \\midrule")
        latex.append("")

    # Group by N and add data rows for other algorithms
    for N_val in sorted(df_rest["N"].unique()):
        df_n = df_rest[df_rest["N"] == N_val].copy()

        # Sort by: BASELINE (orig.), BASELINE (CP), SANDO single, SANDO multi
        def sort_key(row):
            if row["Algorithm"] == "BASELINE" and row["Variant"] == "(orig.)":
                return (0, 0)
            elif row["Algorithm"] == "BASELINE" and row["Variant"] == "(CP)":
                return (0, 1)
            elif row["Algorithm"] == "SANDO" and row["Thread"] == "single":
                return (1, 0)
            elif row["Algorithm"] == "SANDO" and row["Thread"] == "multi":
                return (1, 1)
            else:
                return (3, 0)

        df_n["sort_key"] = df_n.apply(sort_key, axis=1)
        df_n = df_n.sort_values("sort_key").drop(columns=["sort_key"])

        # Count BASELINE and SANDO rows for multirow
        baseline_rows = df_n[df_n["Algorithm"] == "BASELINE"]
        n_baseline = len(baseline_rows)
        sando_rows = df_n[df_n["Algorithm"] == "SANDO"]
        n_sando = len(sando_rows)

        first_in_group = True
        first_baseline = True
        first_sando = True
        for _, row in df_n.iterrows():
            # Algorithm name
            alg = row["Algorithm"]
            variant = row["Variant"]
            thread = row["Thread"]

            # BASELINE uses col1=BASELINE (multirow), col2=variant, thread=multirow — like SUPER
            if alg == "BASELINE":
                if first_baseline and n_baseline > 1:
                    alg_col1 = f"\\multirow{{{n_baseline}}}{{*}}{{BASELINE}}"
                    thread_cell = f"\\multirow{{{n_baseline}}}{{*}}{{{thread}}}"
                    first_baseline = False
                elif first_baseline:
                    alg_col1 = "BASELINE"
                    thread_cell = thread
                    first_baseline = False
                else:
                    alg_col1 = ""
                    thread_cell = ""
                alg_cell = f"{alg_col1} & {variant}"
            elif alg == "SANDO":
                if first_sando and n_sando > 1:
                    alg_cell = f"\\multicolumn{{2}}{{c}}{{\\multirow{{{n_sando}}}{{*}}{{SANDO2}}}}"
                    first_sando = False
                elif first_sando:
                    alg_cell = "\\multicolumn{2}{c}{SANDO2}"
                    first_sando = False
                else:
                    alg_cell = "\\multicolumn{2}{c}{}"
            else:
                alg_cell = f"\\multicolumn{{2}}{{c}}{{{alg}}}"

            # N (use multirow for first entry of each N)
            if first_in_group:
                n_cell = f"\\multirow{{{len(df_n)}}}{{*}}{{{int(N_val)}}}"
                first_in_group = False
            else:
                n_cell = ""

            # For non-BASELINE rows, thread_cell is just the thread value
            if alg != "BASELINE":
                thread_cell = thread

            # Build row
            row_str = f"      {alg_cell} & {thread_cell} & {n_cell}"

            for col_name, _, _, precision, force_best in columns_config:
                val = row[col_name]
                best, worst = best_worst[col_name]
                formatted = format_value(val, best, worst, precision, force_best)
                row_str += f" & {formatted}"

            row_str += " \\\\"
            latex.append(row_str)

        # Add midrule between N groups (except after last)
        numeric_N = [n for n in df_rest["N"].unique() if isinstance(n, (int, float))]
        if numeric_N and N_val != max(numeric_N):
            latex.append("")
            latex.append("      \\midrule")
            latex.append("")

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table*}")

    return "\n".join(latex)


def load_ve_data():
    """Load variable elimination benchmark data.

    VE=yes rows come from the standardized benchmark (multi_thread/sando_N_benchmark.csv)
    so that both tables share the same data.  VE=no rows come from ve_benchmark/.
    """
    # VE=yes: use SANDO2 (dynamic k-factor) multi-threaded data
    ve_yes_files = {
        N: ROOT_PATH / f"multi_thread/sando_{N}_benchmark.csv" for N in [4, 5, 6]
    }
    # VE=no: dedicated without-VE runs for SANDO2
    ve_no_files = {
        N: ROOT_PATH / f"ve_benchmark/sando_{N}_without_ve_benchmark.csv"
        for N in [4, 5, 6]
    }

    # Build combined file list: (N, ve_flag, filepath)
    file_list = []
    for N, fp in ve_yes_files.items():
        if fp.exists():
            file_list.append((N, "yes", fp))
        else:
            # Fall back to ve_benchmark with_ve file if multi_thread doesn't exist
            fallback = ROOT_PATH / f"ve_benchmark/sando_{N}_with_ve_benchmark.csv"
            if fallback.exists():
                file_list.append((N, "yes", fallback))
            else:
                print(f"WARNING: Missing VE=yes data for N={N}")
    for N, fp in ve_no_files.items():
        if fp.exists():
            file_list.append((N, "no", fp))
        else:
            print(f"WARNING: Missing VE=no data for N={N}")

    if not file_list:
        print("WARNING: No VE benchmark data found")
        return pd.DataFrame()

    rows = []

    for N, ve_flag, csv_file in file_list:
        df = pd.read_csv(csv_file)

        # Parse success column
        df["success"] = (
            pd.to_numeric(df["success"], errors="coerce").fillna(0).astype(int)
        )

        # Compute success rate (percentage)
        success_rate = safe_mean(df["success"]) * 100

        # Compute means over successful runs only
        succ_df = df[df["success"] == 1]

        if succ_df.empty:
            # No successful runs - use NaN
            per_opt_ms = np.nan
            total_opt_ms = np.nan
            traj_time_s = np.nan
            path_length = np.nan
            jerk_smooth = np.nan
            any_viol = np.nan
        else:
            per_opt_ms = safe_mean(succ_df["per_opt_runtime_ms"])
            total_opt_ms = safe_mean(succ_df["total_opt_runtime_ms"])
            traj_time_s = safe_mean(succ_df["total_traj_time_sec"])
            path_length = safe_mean(succ_df["traj_length_m"])
            jerk_smooth = safe_mean(succ_df["jerk_smoothness_l1"])

            # Combined violation rate: count / total * 100 (consistent with dynamic/static benchmarks)
            if (
                "violation_total_samples" in succ_df.columns
                and "v_violation_count" in succ_df.columns
            ):
                total_samples = succ_df["violation_total_samples"].sum()
                if total_samples > 0:
                    sfc_viol = (
                        succ_df["corridor_violation_count"].sum()
                        / total_samples
                        * 100.0
                    )
                    vel_viol = (
                        succ_df["v_violation_count"].sum() / total_samples * 100.0
                    )
                    acc_viol = (
                        succ_df["a_violation_count"].sum() / total_samples * 100.0
                    )
                    jerk_viol = (
                        succ_df["j_violation_count"].sum() / total_samples * 100.0
                    )
                else:
                    sfc_viol = vel_viol = acc_viol = jerk_viol = 0.0
            else:
                sfc_viol = (
                    safe_mean(succ_df["corridor_violated"]) * 100
                    if "corridor_violated" in succ_df.columns
                    else 0.0
                )
                vel_viol = (
                    safe_mean(succ_df["v_violated"]) * 100
                    if "v_violated" in succ_df.columns
                    else 0.0
                )
                acc_viol = (
                    safe_mean(succ_df["a_violated"]) * 100
                    if "a_violated" in succ_df.columns
                    else 0.0
                )
                jerk_viol = (
                    safe_mean(succ_df["j_violated"]) * 100
                    if "j_violated" in succ_df.columns
                    else 0.0
                )

            # Max of all violations
            any_viol = max(sfc_viol, vel_viol, acc_viol, jerk_viol)

        rows.append(
            {
                "N": N,
                "VE": ve_flag,
                "success_rate": success_rate,
                "per_opt_ms": per_opt_ms,
                "total_opt_ms": total_opt_ms,
                "traj_time_s": traj_time_s,
                "path_length": path_length,
                "jerk_smooth": jerk_smooth,
                "any_viol": any_viol,
            }
        )

    return pd.DataFrame(rows)


def generate_ve_latex_table(df):
    """Generate LaTeX table for VE benchmark comparison"""

    if df.empty:
        return "% VE benchmark data not available"

    latex = []
    latex.append("\\begin{table}")
    latex.append(
        "  \\caption{Variable Elimination (VE) Benchmarking Results in Standardized Environment: We highlight the best and worst values for each $N$ in \\best{green} and \\worst{red}, respectively. \\todo{add description}}"
    )
    latex.append("  \\label{tab:variable_elimination_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\resizebox{\\columnwidth}{!}{")
    latex.append("    \\begin{tabular}{c c c c c c c c}")
    latex.append("      \\toprule")

    # Column headers (single row, no sub-labels)
    latex.append("      \\textbf{N}")
    latex.append("      & \\textbf{VE}")
    latex.append("      & $R^{\\mathrm{opt}}_{\\mathrm{succ}}$ [\\%]")
    latex.append("      & \\shortstack{$T^{\\mathrm{per}}_{\\mathrm{opt}}${[ms]}}")
    latex.append("      & \\shortstack{$T_{\\mathrm{trav}}${[s]}}")
    latex.append("      & \\shortstack{$L_{\\mathrm{path}}${[m]}}")
    latex.append("      & \\shortstack{$S_{\\mathrm{jerk}}${[m/s$^{2}$]}}")
    latex.append("      & \\shortstack{$\\rho_{\\mathrm{viol}}${[\\%]}}")
    latex.append("      \\\\")
    latex.append("")
    latex.append("      \\midrule")
    latex.append("")

    # Data rows - group by N
    for N_val in sorted(df["N"].unique()):
        df_n = df[df["N"] == N_val].sort_values("VE", ascending=False)  # yes before no

        # Find best/worst for this N
        def find_best_worst_for_n(col, higher_better=False):
            vals = df_n[col].dropna()
            if vals.empty:
                return None, None
            if higher_better:
                return vals.max(), vals.min()
            else:
                return vals.min(), vals.max()

        best_worst_n = {
            "success_rate": find_best_worst_for_n("success_rate", True),
            "per_opt_ms": find_best_worst_for_n("per_opt_ms", False),
            "traj_time_s": find_best_worst_for_n("traj_time_s", False),
            "path_length": find_best_worst_for_n("path_length", False),
            "jerk_smooth": find_best_worst_for_n("jerk_smooth", False),
            "any_viol": find_best_worst_for_n("any_viol", False),
        }

        first_row = True
        for _, row in df_n.iterrows():
            # N column (multirow for first entry)
            if first_row:
                n_cell = f"\\multirow{{2}}{{*}}{{{int(N_val)}}}"
                first_row = False
            else:
                n_cell = ""

            # VE column
            ve_cell = "\\YesGreen" if row["VE"] == "yes" else "\\NoRed"

            row_str = f"      {n_cell} & {ve_cell}"

            # Success rate - green if 100.0, black otherwise
            val = row["success_rate"]
            if pd.isna(val):
                formatted = "-"
            elif abs(val - 100.0) < 0.01:
                # 100.0 -> green (best)
                formatted = f"\\best{{{val:.1f}}}"
            else:
                # Not 100.0 -> black (no highlighting)
                formatted = f"{val:.1f}"
            row_str += f" & {formatted}"

            # Per opt time
            val = row["per_opt_ms"]
            best, worst = best_worst_n["per_opt_ms"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            # Traj time
            val = row["traj_time_s"]
            best, worst = best_worst_n["traj_time_s"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            # Path length
            val = row["path_length"]
            best, worst = best_worst_n["path_length"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            # Jerk smoothness
            val = row["jerk_smooth"]
            best, worst = best_worst_n["jerk_smooth"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            # Combined violation
            val = row["any_viol"]
            best, worst = best_worst_n["any_viol"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            row_str += " \\\\"
            latex.append(row_str)

        # Add midrule between N groups (except after last)
        if N_val != df["N"].max():
            latex.append("")
            latex.append("      \\midrule")
            latex.append("")

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table}")

    return "\n".join(latex)


UNKNOWN_DYNAMIC_OUTPUT_FILE = None  # Must be set via --ud-output CLI arg


def load_unknown_dynamic_data():
    """Load unknown dynamic benchmark data from P_2 and P_3 folders.

    Reads per-trial benchmark CSVs for success/travel/path/jerk metrics,
    and per-replanning CSVs for optimization timing.

    Returns:
        DataFrame with columns: P, case, success_rate, per_opt_ms,
        travel_time, path_length, jerk_integral, cv_pct
    """
    base = ROOT_PATH / "unknown_dynamic"
    rows = []

    for p_val in [2, 3]:
        p_dir = base / f"P_{p_val}"
        if not p_dir.exists():
            print(f"  WARNING: {p_dir} not found")
            continue

        for case in ["easy", "medium", "hard"]:
            # Find the case directory (e.g., easy_20260309_191424)
            case_dirs = sorted(p_dir.glob(f"{case}_*"))
            if not case_dirs:
                print(f"  WARNING: No {case} directory found in {p_dir}")
                continue
            case_dir = case_dirs[-1]  # Use the latest

            # Load top-level benchmark CSVs (last one has all trials)

            bench_files = sorted(case_dir.glob("benchmark_*.csv"))
            bench_files = [f for f in bench_files if f.stat().st_size > 10]
            if not bench_files:
                print(f"  WARNING: No benchmark CSVs in {case_dir}")
                continue

            final_df = pd.read_csv(bench_files[-1])
            n_trials = len(final_df)
            n_success = int(final_df["goal_reached"].sum())
            success_rate = n_success / n_trials * 100

            succ = final_df[final_df["goal_reached"] == True]
            travel_time = safe_mean(succ["flight_travel_time"])
            path_length = safe_mean(succ["path_length"])
            jerk = safe_mean(succ["jerk_integral"])

            # Combined constraint violation rate
            viol_cols = [
                "sfc_violation_count",
                "vel_violation_count",
                "acc_violation_count",
                "jerk_violation_count",
            ]
            total_viol = sum(succ[c].sum() for c in viol_cols if c in succ.columns)
            # Express as percentage of trials with any violation
            trials_with_viol = (
                (succ[viol_cols].sum(axis=1) > 0).sum()
                if all(c in succ.columns for c in viol_cols)
                else 0
            )
            cv_pct = trials_with_viol / len(succ) * 100 if len(succ) > 0 else 0.0

            # Per-replanning CSVs for optimization time
            csv_dir = case_dir / "csv"
            replan_files = sorted(csv_dir.glob("num_*.csv")) if csv_dir.exists() else []
            replan_files = [f for f in replan_files if f.stat().st_size > 10]
            per_opt_ms = np.nan
            total_replan_ms = np.nan
            cvx_decomp_ms = np.nan
            if replan_files:
                replan_dfs = []
                for f in replan_files:
                    try:
                        replan_dfs.append(pd.read_csv(f))
                    except Exception:
                        pass
                if replan_dfs:
                    replan = pd.concat(replan_dfs, ignore_index=True)
                    replan_succ = replan[replan["Result"] == 1]
                    if not replan_succ.empty:
                        per_opt_ms = safe_mean(replan_succ["Local Traj Time [ms]"])
                        if "Total replanning time [ms]" in replan_succ.columns:
                            total_replan_ms = safe_mean(
                                replan_succ["Total replanning time [ms]"]
                            )
                        if "CVX Decomposition Time [ms]" in replan_succ.columns:
                            cvx_decomp_ms = safe_mean(
                                replan_succ["CVX Decomposition Time [ms]"]
                            )

            rows.append(
                {
                    "P": p_val,
                    "case": case,
                    "success_rate": success_rate,
                    "per_opt_ms": per_opt_ms,
                    "total_replan_ms": total_replan_ms,
                    "cvx_decomp_ms": cvx_decomp_ms,
                    "travel_time": travel_time,
                    "path_length": path_length,
                    "jerk_integral": jerk,
                    "cv_pct": cv_pct,
                }
            )
            print(
                f"  P={p_val} {case}: {n_success}/{n_trials} success, "
                f"opt={per_opt_ms:.1f}ms, replan={total_replan_ms:.1f}ms, "
                f"cvx={cvx_decomp_ms:.1f}ms, trav={travel_time:.1f}s, "
                f"path={path_length:.1f}m, jerk={jerk:.1f}"
            )

    return pd.DataFrame(rows)


def generate_unknown_dynamic_latex_table(df):
    """Generate LaTeX table for unknown dynamic benchmark.

    Produces a table with best/worst highlighting per environment (Easy/Medium/Hard),
    comparing P=2 and P=3 configurations.
    """
    if df.empty:
        return "% No unknown dynamic benchmark data available"

    # Columns to highlight: (key, higher_is_better)
    metric_cols = [
        ("success_rate", True),
        ("per_opt_ms", False),
        ("total_replan_ms", False),
        ("cvx_decomp_ms", False),
        ("travel_time", False),
        ("path_length", False),
        ("jerk_integral", False),
        ("cv_pct", False),
    ]

    latex = []
    latex.append("\\begin{table}")
    latex.append(
        "  \\caption{Benchmark results in unknown dynamic environments. "
        "SANDO navigates using only pointcloud sensing (no ground truth obstacle trajectories). "
        "We highlight the \\best{best} and \\worst{worst} value for each environment.}"
    )
    latex.append("  \\label{tab:unknown_dynamic_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\setlength{\\tabcolsep}{4pt}")
    latex.append("  \\resizebox{\\columnwidth}{!}{")
    latex.append("    \\begin{tabular}{c c c c c c c c c c}")
    latex.append("      \\toprule")
    latex.append("      \\textbf{Env} &")
    latex.append("      \\textbf{$P$} &")
    latex.append("      $R_{\\mathrm{succ}}$\\,[\\%] &")
    latex.append("      $T^{\\mathrm{per}}_{\\mathrm{opt}}$\\,[ms] &")
    latex.append("      $T_{\\mathrm{replan}}$\\,[ms] &")
    latex.append("      $T_{\\mathrm{STSFC}}$\\,[ms] &")
    latex.append("      $T_{\\mathrm{trav}}$\\,[s] &")
    latex.append("      $L_{\\mathrm{path}}$\\,[m] &")
    latex.append("      $S_{\\mathrm{jerk}}$\\,[m/s$^{2}$] &")
    latex.append("      $\\rho_{\\mathrm{cv}}$\\,[\\%]")
    latex.append("      \\\\")
    latex.append("      \\midrule")

    case_labels = {"easy": "Easy", "medium": "Medium", "hard": "Hard"}

    for ci, case in enumerate(["easy", "medium", "hard"]):
        df_case = df[df["case"] == case].copy()
        if df_case.empty:
            continue

        # Sort P=3 first, then P=2 (higher P first)
        df_case = df_case.sort_values("P", ascending=False)

        # Find best/worst for this environment
        bw = {}
        for col, higher_better in metric_cols:
            vals = df_case[col].dropna()
            if vals.empty:
                bw[col] = (None, None)
            elif higher_better:
                bw[col] = (vals.max(), vals.min())
            else:
                bw[col] = (vals.min(), vals.max())

        first_row = True
        for _, row in df_case.iterrows():
            if first_row:
                env_cell = f"\\multirow{{2}}{{*}}{{{case_labels[case]}}}"
                first_row = False
            else:
                env_cell = ""

            p_val = int(row["P"])
            cells = [f"      {env_cell} & {p_val}"]

            for col, _ in metric_cols:
                val = row[col]
                best, worst = bw[col]
                cells.append(format_value(val, best, worst, 1))

            latex.append(" & ".join(cells) + " \\\\")

        # Add midrule between environments (except after last)
        if ci < 2:
            latex.append("      \\midrule")

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table}")

    return "\n".join(latex)


def main():
    """Main function"""
    global ROOT_PATH, OUTPUT_FILE, VE_OUTPUT_FILE, UNKNOWN_DYNAMIC_OUTPUT_FILE, SUPER_CSV_FILES

    parser = argparse.ArgumentParser(description="Generate LaTeX tables from SANDO benchmark data")
    parser.add_argument("--root-path", type=Path, default=ROOT_PATH,
                        help="Root path for benchmark data (default: <package>/benchmark_data)")
    parser.add_argument("--output", type=Path, required=True,
                        help="Output path for standardized benchmark LaTeX table")
    parser.add_argument("--ve-output", type=Path, default=None,
                        help="Output path for VE benchmark LaTeX table")
    parser.add_argument("--ud-output", type=Path, default=None,
                        help="Output path for unknown dynamic benchmark LaTeX table")
    parser.add_argument("--super-l2-csv", type=Path, default=None,
                        help="Path to SUPER L2 benchmark CSV")
    parser.add_argument("--super-linf-csv", type=Path, default=None,
                        help="Path to SUPER Linf benchmark CSV")
    parser.add_argument("--no-ve", action="store_true", help="Skip VE benchmark table")
    parser.add_argument("--no-unknown-dynamic", action="store_true",
                        help="Skip unknown dynamic benchmark table")
    args = parser.parse_args()

    ROOT_PATH = args.root_path
    OUTPUT_FILE = args.output
    VE_OUTPUT_FILE = args.ve_output
    UNKNOWN_DYNAMIC_OUTPUT_FILE = args.ud_output
    if args.super_l2_csv:
        SUPER_CSV_FILES["SUPER ($L_2$)"] = args.super_l2_csv
    if args.super_linf_csv:
        SUPER_CSV_FILES["SUPER ($L_\\infty$)"] = args.super_linf_csv

    skip_ve = args.no_ve or VE_OUTPUT_FILE is None

    print("=" * 80)
    print("SANDO LaTeX Table Generator")
    print("=" * 80)

    # ========== Generate Standardized Benchmark Table ==========
    print("\n[1/3] Generating Standardized Benchmark Table")
    print("-" * 80)

    # Load data
    print("Loading benchmark data...")
    df = load_and_process_data()

    if df.empty:
        print("ERROR: No data loaded. Check that CSV files exist.")
    else:
        print(f"Loaded {len(df)} data rows")
        print("\nData summary:")
        print(df[["Algorithm", "Thread", "N"]].to_string(index=False))

        # Generate full LaTeX table
        print("\nGenerating FULL LaTeX table...")
        latex_code = generate_latex_table(df)

        # Save full table
        OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT_FILE.write_text(latex_code)

        print(f"\n✓ Full LaTeX table saved to: {OUTPUT_FILE}")
        print(f"  Include in paper: \\input{{{OUTPUT_FILE.name}}}")

        # Generate SANDO-only rows
        print("\nGenerating SANDO-only rows...")
        sando_rows = generate_sando_rows_only(df)

        # Save SANDO-only rows
        sando_only_file = OUTPUT_FILE.parent / "sando_rows_only.tex"
        sando_only_file.write_text(sando_rows)

        print(f"\n✓ SANDO-only rows saved to: {sando_only_file}")
        print(f"\n{'=' * 80}")
        print("USAGE INSTRUCTIONS:")
        print("=" * 80)
        print("\nOption 1: Use full table (if starting fresh)")
        print(f"  \\input{{{OUTPUT_FILE.name}}}")
        print("\nOption 2: Update existing table (preserves other planners)")
        print("  1. Open your existing table file")
        print("  2. Find all lines containing 'SANDO'")
        print(f"  3. Replace them with contents from: {sando_only_file.name}")
        print("  4. Make sure multirow{N} values match your table structure")
        print("=" * 80)

    # ========== Generate VE Benchmark Table ==========
    ve_df = pd.DataFrame()
    if skip_ve:
        print("\n[2/3] Skipping VE Benchmark Table (--no-ve)")
    else:
        print("\n[2/3] Generating Variable Elimination Benchmark Table")
        print("-" * 80)

        # Load VE data
        print("Loading VE benchmark data...")
        ve_df = load_ve_data()

        if ve_df.empty:
            print("WARNING: No VE benchmark data found.")
            print("  Run: python3 run_benchmark_suite.py --ve-comparison")
        else:
            print(f"Loaded {len(ve_df)} VE data rows")
            print("\nVE data summary:")
            print(ve_df[["N", "VE"]].to_string(index=False))

            # Generate LaTeX
            print("\nGenerating VE LaTeX table...")
            ve_latex_code = generate_ve_latex_table(ve_df)

            # Save to file
            VE_OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
            VE_OUTPUT_FILE.write_text(ve_latex_code)

            print(f"\n✓ VE LaTeX table saved to: {VE_OUTPUT_FILE}")
            print(f"  Include in paper: \\input{{{VE_OUTPUT_FILE.name}}}")

    # ========== Generate Unknown Dynamic Benchmark Table ==========
    ud_df = pd.DataFrame()
    skip_ud = args.no_unknown_dynamic or UNKNOWN_DYNAMIC_OUTPUT_FILE is None
    if skip_ud:
        print("\n[3/3] Skipping Unknown Dynamic Benchmark Table (--no-unknown-dynamic)")
    else:
        print("\n[3/3] Generating Unknown Dynamic Benchmark Table")
        print("-" * 80)

        print("Loading unknown dynamic benchmark data...")
        ud_df = load_unknown_dynamic_data()

        if ud_df.empty:
            print("WARNING: No unknown dynamic benchmark data found.")
            print(
                "  Run: python3 run_benchmark.py --num-p-values 2 3 --config-name unknown_dynamic ..."
            )
        else:
            print(f"\nLoaded {len(ud_df)} data rows")

            print("\nGenerating Unknown Dynamic LaTeX table...")
            ud_latex_code = generate_unknown_dynamic_latex_table(ud_df)

            UNKNOWN_DYNAMIC_OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
            UNKNOWN_DYNAMIC_OUTPUT_FILE.write_text(ud_latex_code)

            print(
                f"\n✓ Unknown Dynamic LaTeX table saved to: {UNKNOWN_DYNAMIC_OUTPUT_FILE}"
            )
            print(f"  Include in paper: \\input{{{UNKNOWN_DYNAMIC_OUTPUT_FILE.name}}}")

    # ========== Summary ==========
    print("\n" + "=" * 80)
    print("GENERATION COMPLETE")
    print("=" * 80)
    print("\nGenerated files:")
    if not df.empty:
        print(f"  1. {OUTPUT_FILE}")
    if not ve_df.empty:
        print(f"  2. {VE_OUTPUT_FILE}")
    if not ud_df.empty:
        print(f"  3. {UNKNOWN_DYNAMIC_OUTPUT_FILE}")


if __name__ == "__main__":
    main()
