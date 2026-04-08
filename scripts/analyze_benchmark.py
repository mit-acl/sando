#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) Anonymous Author
# Anonymous Institution
# All Rights Reserved
# Authors: Anonymous
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
SANDO Benchmark Analysis Script

Quick analysis and visualization of benchmark results.

Usage:
    # Analyze single run
    python3 scripts/analyze_benchmark.py benchmark_data/20260203_120000/benchmark_default_*.csv

    # Compare multiple configurations
    python3 scripts/analyze_benchmark.py benchmark_data/*/benchmark_*.csv --compare

    # Generate detailed report
    python3 scripts/analyze_benchmark.py benchmark_data/20260203_120000/benchmark_*.csv --report
"""

import argparse
import glob
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


def load_benchmark_data(pattern: str) -> pd.DataFrame:
    """Load benchmark CSV files matching pattern"""
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"No files found matching: {pattern}")

    dfs = []
    for file in files:
        df = pd.read_csv(file)
        df["source_file"] = Path(file).name
        dfs.append(df)

    combined = pd.concat(dfs, ignore_index=True)
    print(f"Loaded {len(combined)} trials from {len(files)} files")
    return combined


def print_summary(df: pd.DataFrame, name: str = "Dataset"):
    """Print summary statistics"""
    print(f"\n{'=' * 80}")
    print(f"SUMMARY: {name}")
    print(f"{'=' * 80}")
    print(f"Total trials: {len(df)}")

    # Success metrics
    success_df = df[df["flight_success"] == True]
    success_rate = len(success_df) / len(df) * 100 if len(df) > 0 else 0
    print(f"Success rate: {len(success_df)}/{len(df)} ({success_rate:.1f}%)")

    if len(success_df) > 0:
        print("\nSuccessful Trials:")
        print(
            f"  Travel time:      {success_df['flight_travel_time'].mean():.2f} ± {success_df['flight_travel_time'].std():.2f}s"
        )
        print(
            f"  Path length:      {success_df['path_length'].mean():.2f} ± {success_df['path_length'].std():.2f}m"
        )
        print(
            f"  Path efficiency:  {success_df['path_efficiency'].mean():.3f} ± {success_df['path_efficiency'].std():.3f}"
        )
        print(
            f"  Avg velocity:     {success_df['avg_velocity'].mean():.2f} ± {success_df['avg_velocity'].std():.2f} m/s"
        )
        print(
            f"  Max velocity:     {success_df['max_velocity'].mean():.2f} ± {success_df['max_velocity'].std():.2f} m/s"
        )

        print("\n  Smoothness:")
        print(
            f"  Jerk integral:    {success_df['jerk_integral'].mean():.2f} ± {success_df['jerk_integral'].std():.2f}"
        )
        print(
            f"  RMS jerk:         {success_df['jerk_rms'].mean():.2f} ± {success_df['jerk_rms'].std():.2f}"
        )

        # Violations
        vel_viol = success_df["vel_violation_count"].sum()
        acc_viol = success_df["acc_violation_count"].sum()
        jerk_viol = success_df["jerk_violation_count"].sum()

        print("\n  Constraint Violations:")
        print(
            f"  Velocity:         {vel_viol} ({vel_viol / len(success_df):.1f} per trial)"
        )
        print(
            f"  Acceleration:     {acc_viol} ({acc_viol / len(success_df):.1f} per trial)"
        )
        print(
            f"  Jerk:             {jerk_viol} ({jerk_viol / len(success_df):.1f} per trial)"
        )

    # Failed trials
    failed_df = df[df["flight_success"] == False]
    if len(failed_df) > 0:
        print(f"\nFailed Trials: {len(failed_df)}")
        timeout_count = failed_df["timeout_reached"].sum()
        print(f"  Timeouts:         {timeout_count}")

    print(f"{'=' * 80}\n")


def plot_results(df: pd.DataFrame, output_path: str = None):
    """Generate comprehensive visualization"""
    success_df = df[df["flight_success"] == True]

    if len(success_df) == 0:
        print("No successful trials to plot")
        return

    # Set style
    sns.set_style("whitegrid")
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

    # 1. Travel time distribution
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.hist(success_df["flight_travel_time"], bins=20, edgecolor="black", alpha=0.7)
    ax1.set_title("Travel Time Distribution", fontweight="bold")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Count")
    ax1.axvline(
        success_df["flight_travel_time"].mean(), color="r", linestyle="--", label="Mean"
    )
    ax1.legend()

    # 2. Path efficiency distribution
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.hist(
        success_df["path_efficiency"],
        bins=20,
        edgecolor="black",
        alpha=0.7,
        color="green",
    )
    ax2.set_title("Path Efficiency Distribution", fontweight="bold")
    ax2.set_xlabel("Efficiency (straight_line / path_length)")
    ax2.set_ylabel("Count")
    ax2.axvline(
        success_df["path_efficiency"].mean(), color="r", linestyle="--", label="Mean"
    )
    ax2.legend()

    # 3. Success rate by seed
    ax3 = fig.add_subplot(gs[0, 2])
    success_by_seed = df.groupby("seed")["flight_success"].mean() * 100
    ax3.bar(success_by_seed.index, success_by_seed.values, edgecolor="black", alpha=0.7)
    ax3.set_title("Success Rate by Seed", fontweight="bold")
    ax3.set_xlabel("Seed")
    ax3.set_ylabel("Success Rate (%)")
    ax3.set_ylim([0, 105])

    # 4. Travel time vs obstacles
    ax4 = fig.add_subplot(gs[1, 0])
    scatter = ax4.scatter(
        success_df["num_obstacles"],
        success_df["flight_travel_time"],
        c=success_df["path_efficiency"],
        cmap="RdYlGn",
        alpha=0.6,
        s=50,
    )
    ax4.set_title("Travel Time vs Number of Obstacles", fontweight="bold")
    ax4.set_xlabel("Number of Obstacles")
    ax4.set_ylabel("Travel Time (s)")
    plt.colorbar(scatter, ax=ax4, label="Path Efficiency")

    # 5. Path length vs efficiency
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.scatter(
        success_df["path_length"], success_df["path_efficiency"], alpha=0.6, s=50
    )
    ax5.set_title("Path Efficiency vs Path Length", fontweight="bold")
    ax5.set_xlabel("Path Length (m)")
    ax5.set_ylabel("Path Efficiency")
    ax5.axhline(1.0, color="r", linestyle="--", alpha=0.5, label="Optimal")
    ax5.legend()

    # 6. Jerk RMS vs path length
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.scatter(
        success_df["path_length"],
        success_df["jerk_rms"],
        alpha=0.6,
        s=50,
        color="purple",
    )
    ax6.set_title("Smoothness vs Path Length", fontweight="bold")
    ax6.set_xlabel("Path Length (m)")
    ax6.set_ylabel("RMS Jerk")

    # 7. Velocity distribution
    ax7 = fig.add_subplot(gs[2, 0])
    ax7.hist(
        success_df["avg_velocity"],
        bins=20,
        alpha=0.5,
        label="Average",
        edgecolor="black",
    )
    ax7.hist(
        success_df["max_velocity"],
        bins=20,
        alpha=0.5,
        label="Maximum",
        edgecolor="black",
    )
    ax7.set_title("Velocity Distribution", fontweight="bold")
    ax7.set_xlabel("Velocity (m/s)")
    ax7.set_ylabel("Count")
    ax7.legend()

    # 8. Constraint violations
    ax8 = fig.add_subplot(gs[2, 1])
    violations = {
        "Velocity": success_df["vel_violation_count"].sum(),
        "Acceleration": success_df["acc_violation_count"].sum(),
        "Jerk": success_df["jerk_violation_count"].sum(),
    }
    colors = ["red" if v > 0 else "green" for v in violations.values()]
    ax8.bar(
        violations.keys(),
        violations.values(),
        color=colors,
        edgecolor="black",
        alpha=0.7,
    )
    ax8.set_title("Total Constraint Violations", fontweight="bold")
    ax8.set_ylabel("Count")

    # 9. Box plot of travel times by dynamic ratio
    ax9 = fig.add_subplot(gs[2, 2])
    if success_df["dynamic_ratio"].nunique() > 1:
        success_df.boxplot(column="flight_travel_time", by="dynamic_ratio", ax=ax9)
        ax9.set_title("Travel Time by Dynamic Ratio", fontweight="bold")
        ax9.set_xlabel("Dynamic Ratio")
        ax9.set_ylabel("Travel Time (s)")
    else:
        # If only one dynamic ratio, show jerk distribution
        ax9.hist(
            success_df["jerk_integral"],
            bins=20,
            edgecolor="black",
            alpha=0.7,
            color="orange",
        )
        ax9.set_title("Jerk Integral Distribution", fontweight="bold")
        ax9.set_xlabel("Jerk Integral")
        ax9.set_ylabel("Count")

    plt.suptitle("SANDO Benchmark Analysis", fontsize=16, fontweight="bold", y=0.995)

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Plot saved to: {output_path}")
    else:
        plt.show()


def generate_report(df: pd.DataFrame, output_path: str):
    """Generate detailed markdown report"""
    success_df = df[df["flight_success"] == True]

    report = [
        "# SANDO Benchmark Report",
        f"\nGenerated: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "\n## Overview",
        f"\n- Total trials: {len(df)}",
        f"- Successful: {len(success_df)} ({len(success_df) / len(df) * 100:.1f}%)",
        f"- Failed: {len(df) - len(success_df)}",
        "\n## Configuration",
        f"\n- Number of obstacles: {df['num_obstacles'].iloc[0]}",
        f"- Dynamic ratio: {df['dynamic_ratio'].iloc[0]}",
        f"- Start position: ({df['start_x'].iloc[0]}, {df['start_y'].iloc[0]}, {df['start_z'].iloc[0]})",
        f"- Goal position: ({df['goal_x'].iloc[0]}, {df['goal_y'].iloc[0]}, {df['goal_z'].iloc[0]})",
    ]

    if len(success_df) > 0:
        report.extend(
            [
                "\n## Performance Metrics (Successful Trials)",
                "\n### Time and Distance",
                "\n| Metric | Mean | Std Dev | Min | Max |",
                "|--------|------|---------|-----|-----|",
                f"| Travel Time (s) | {success_df['flight_travel_time'].mean():.2f} | {success_df['flight_travel_time'].std():.2f} | {success_df['flight_travel_time'].min():.2f} | {success_df['flight_travel_time'].max():.2f} |",
                f"| Path Length (m) | {success_df['path_length'].mean():.2f} | {success_df['path_length'].std():.2f} | {success_df['path_length'].min():.2f} | {success_df['path_length'].max():.2f} |",
                f"| Path Efficiency | {success_df['path_efficiency'].mean():.3f} | {success_df['path_efficiency'].std():.3f} | {success_df['path_efficiency'].min():.3f} | {success_df['path_efficiency'].max():.3f} |",
                "\n### Velocity and Acceleration",
                "\n| Metric | Mean | Std Dev | Min | Max |",
                "|--------|------|---------|-----|-----|",
                f"| Avg Velocity (m/s) | {success_df['avg_velocity'].mean():.2f} | {success_df['avg_velocity'].std():.2f} | {success_df['avg_velocity'].min():.2f} | {success_df['avg_velocity'].max():.2f} |",
                f"| Max Velocity (m/s) | {success_df['max_velocity'].mean():.2f} | {success_df['max_velocity'].std():.2f} | {success_df['max_velocity'].min():.2f} | {success_df['max_velocity'].max():.2f} |",
                f"| Avg Acceleration (m/s²) | {success_df['avg_acceleration'].mean():.2f} | {success_df['avg_acceleration'].std():.2f} | {success_df['avg_acceleration'].min():.2f} | {success_df['avg_acceleration'].max():.2f} |",
                f"| Max Acceleration (m/s²) | {success_df['max_acceleration'].mean():.2f} | {success_df['max_acceleration'].std():.2f} | {success_df['max_acceleration'].min():.2f} | {success_df['max_acceleration'].max():.2f} |",
                "\n### Smoothness",
                "\n| Metric | Mean | Std Dev | Min | Max |",
                "|--------|------|---------|-----|-----|",
                f"| Jerk Integral | {success_df['jerk_integral'].mean():.2f} | {success_df['jerk_integral'].std():.2f} | {success_df['jerk_integral'].min():.2f} | {success_df['jerk_integral'].max():.2f} |",
                f"| RMS Jerk | {success_df['jerk_rms'].mean():.2f} | {success_df['jerk_rms'].std():.2f} | {success_df['jerk_rms'].min():.2f} | {success_df['jerk_rms'].max():.2f} |",
                "\n### Constraint Violations",
                "\n| Constraint | Total Count | Avg per Trial | Max Excess |",
                "|------------|-------------|---------------|------------|",
                f"| Velocity | {success_df['vel_violation_count'].sum()} | {success_df['vel_violation_count'].mean():.1f} | {success_df['vel_max_excess'].max():.3f} |",
                f"| Acceleration | {success_df['acc_violation_count'].sum()} | {success_df['acc_violation_count'].mean():.1f} | {success_df['acc_max_excess'].max():.3f} |",
                f"| Jerk | {success_df['jerk_violation_count'].sum()} | {success_df['jerk_violation_count'].mean():.1f} | {success_df['jerk_max_excess'].max():.3f} |",
            ]
        )

    with open(output_path, "w") as f:
        f.write("\n".join(report))

    print(f"Report saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze SANDO benchmark results",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "data", type=str, help="Path to benchmark CSV file(s) (supports wildcards)"
    )

    parser.add_argument(
        "--plot", action="store_true", help="Generate visualization plots"
    )

    parser.add_argument(
        "--report", action="store_true", help="Generate markdown report"
    )

    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default=None,
        help="Output file path for plot/report",
    )

    args = parser.parse_args()

    # Load data
    df = load_benchmark_data(args.data)

    # Print summary
    print_summary(df)

    # Generate plot
    if args.plot or (not args.report and not args.output):
        output_path = args.output if args.output else "benchmark_analysis.png"
        plot_results(df, output_path)

    # Generate report
    if args.report:
        output_path = args.output if args.output else "benchmark_report.md"
        generate_report(df, output_path)


if __name__ == "__main__":
    main()
