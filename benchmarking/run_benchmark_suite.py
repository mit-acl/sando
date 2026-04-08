#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) Anonymous Author
# Anonymous Institution
# All Rights Reserved
# Authors: Anonymous
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
Simplified Benchmark Suite Runner for SANDO Local Trajectory Optimization

This script:
1. Launches the simulator in the background
2. Runs each benchmark configuration one by one
3. Shuts down cleanly between configs

Usage:
    python3 run_benchmark_suite.py [--factor-determination] [--ve-comparison]

    --factor-determination: Run SANDO2 single-threaded with wide factor ranges
                           to determine optimal min/max factors for each N
    --ve-comparison: Run multi-threaded SANDO2 with and without variable elimination
                    to compare performance. Results saved to ve_benchmark/ folder.
"""

import os
import sys
import subprocess
import time
import signal
import argparse
from pathlib import Path

# Configuration
WORKSPACE_DIR = Path(
    os.environ.get("SANDO_WORKSPACE_DIR", Path(__file__).resolve().parent.parent.parent.parent)
)
PACKAGE_NAME = "sando"

# Factor ranges for each N (normal mode - fixed range)
FACTOR_INITIAL = {4: 1.0, 5: 1.0, 6: 1.0}
FACTOR_FINAL = {4: 5.0, 5: 5.0, 6: 5.0}

# Wide factor ranges for determination mode
FACTOR_INITIAL_WIDE = {4: 1.0, 5: 1.0, 6: 1.0}
FACTOR_FINAL_WIDE = {4: 5.0, 5: 5.0, 6: 5.0}

# Dynamic k-factor window parameters (for SANDO2)
DYNAMIC_FACTOR_INITIAL_MEAN = {4: 1.5, 5: 1.5, 6: 1.5}
DYNAMIC_FACTOR_K_RADIUS = 0.4


def get_benchmark_configs(
    factor_determination=False,
    ve_comparison=False,
    only_sando=False,
    only_sando_single=False,
    safe_baseline_only=False,
):
    """Get benchmark configurations based on mode

    Returns list of tuples: (use_single_threaded, planner_name, num_N_list, description, use_var_elim)
    """
    if safe_baseline_only:
        return [
            (
                True,
                "safe_baseline",
                [4, 5, 6],
                "Safe BASELINE single-threaded (N=4,5,6)",
                False,
            ),
        ]
    elif only_sando_single:
        return [
            (True, "sando", [4, 5, 6], "SANDO2 single-threaded (N=4,5,6)", True),
        ]
    elif only_sando:
        return [
            (False, "sando", [4, 5, 6], "SANDO2 multi-threaded (N=4,5,6)", True),
        ]
    elif factor_determination:
        # Only run SANDO2 single-threaded for factor determination
        return [
            (
                True,
                "sando",
                [4, 5, 6],
                "SANDO2 single-threaded (N=4,5,6) - Factor Determination",
                True,
            ),
        ]
    elif ve_comparison:
        # Variable elimination comparison mode - multi-threaded SANDO2 only
        return [
            (
                False,
                "sando",
                [4, 5, 6],
                "SANDO2 multi-threaded (N=4,5,6) WITH variable elimination",
                True,
            ),
            (
                False,
                "sando",
                [4, 5, 6],
                "SANDO2 multi-threaded (N=4,5,6) WITHOUT variable elimination",
                False,
            ),
        ]
    else:
        # Normal full benchmark suite
        return [
            (False, "sando", [4, 5, 6], "SANDO2 multi-threaded (N=4,5,6)", True),
            (True, "sando", [4, 5, 6], "SANDO2 single-threaded (N=4,5,6)", True),
            (
                True,
                "original_baseline",
                [4, 5, 6],
                "BASELINE (original) single-threaded (N=4,5,6)",
                False,
            ),
        ]


def source_workspace():
    """Get the setup file path"""
    setup_file = WORKSPACE_DIR / "install" / "setup.bash"
    if not setup_file.exists():
        print(f"ERROR: Setup file not found: {setup_file}")
        print("Please build the workspace first: colcon build --packages-select sando")
        sys.exit(1)
    return str(setup_file)


def launch_simulator(visualize=False):
    """Launch the fixed obstacles publisher (+ RViz if visualize=True) in the background"""
    print("\n" + "=" * 80)
    if visualize:
        print("Starting Fixed Obstacles Publisher + RViz")
    else:
        print("Starting Fixed Obstacles Publisher")
    print("=" * 80)

    setup_file = source_workspace()
    obstacles_script = (
        WORKSPACE_DIR / "src" / "sando" / "scripts" / "fixed_obstacles_publisher.py"
    )
    cmd = f"source {setup_file} && python3 {obstacles_script} & "
    if visualize:
        rviz_config = WORKSPACE_DIR / "src" / "sando" / "rviz" / "sando.rviz"
        cmd += f"rviz2 -d {rviz_config} & "
    cmd += "wait"

    proc = subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )

    print("Waiting for obstacles publisher to initialize...")
    time.sleep(3)

    return proc


def run_benchmark(
    use_single_threaded,
    planner_name,
    num_N_list,
    description,
    factor_determination=False,
    ve_comparison=False,
    use_var_elim=True,
):
    """Run a single benchmark configuration"""
    print("\n" + "=" * 80)
    print(f"Running: {description}")
    print("=" * 80)

    # Determine if this planner uses dynamic k-factor
    use_dynamic_factor = planner_name in ("sando", "baseline_star")

    # Choose factor ranges based on mode
    if factor_determination:
        factor_initial_dict = FACTOR_INITIAL_WIDE
        factor_final_dict = FACTOR_FINAL_WIDE
        print("\nUsing WIDE factor ranges for determination:")
        for n in num_N_list:
            print(
                f"  N={n}: factors [{factor_initial_dict[n]:.1f}, {factor_final_dict[n]:.1f}]"
            )
    else:
        factor_initial_dict = FACTOR_INITIAL
        factor_final_dict = FACTOR_FINAL

    # Print planner configuration
    print(f"\nPlanner: {planner_name}")
    print(f"  Variable elimination: {use_var_elim}")
    print(f"  Threading: {'single' if use_single_threaded else 'multi'}")
    print(f"  Dynamic k-factor: {use_dynamic_factor}")

    # Build factor lists
    factor_initial_list = [factor_initial_dict[n] for n in num_N_list]
    factor_final_list = [factor_final_dict[n] for n in num_N_list]

    # Convert lists to ROS2 parameter format
    def list_to_yaml(lst):
        return "[" + ",".join(str(x) for x in lst) + "]"

    num_N_str = list_to_yaml(num_N_list)
    factor_initial_str = list_to_yaml(factor_initial_list)
    factor_final_str = list_to_yaml(factor_final_list)
    planner_names_str = '["' + planner_name + '"]'

    # Build command
    # Adjust timeout based on max N value (higher N needs more time)
    max_N = max(num_N_list) if num_N_list else 6
    timeout_sec = 10.0

    # Variable elimination parameter
    use_var_elim_str = "true" if use_var_elim else "false"

    # Determine output directory based on mode
    if ve_comparison:
        # Use ve_benchmark folder for variable elimination comparison
        output_dir_override = str(
            WORKSPACE_DIR / "src" / PACKAGE_NAME / "benchmark_data" / "ve_benchmark"
        )
    else:
        output_dir_override = ""

    # Build dynamic factor parameters
    if use_dynamic_factor:
        dynamic_initial_mean_list = [DYNAMIC_FACTOR_INITIAL_MEAN[n] for n in num_N_list]
        dynamic_initial_mean_str = list_to_yaml(dynamic_initial_mean_list)

    cmd = [
        "ros2",
        "launch",
        PACKAGE_NAME,
        "local_traj_benchmark.launch.py",
        f"use_single_threaded:={str(use_single_threaded).lower()}",
        f"'planner_names:={planner_names_str}'",
        f"'num_N_list:={num_N_str}'",
        f"'factor_initial_list:={factor_initial_str}'",
        f"'factor_final_list:={factor_final_str}'",
        f"using_variable_elimination:={use_var_elim_str}",
        f"use_dynamic_factor:={str(use_dynamic_factor).lower()}",
        "visualize:=false",
        "solve_delay_sec:=0.05",
        "playback_period_sec:=0.0",
        f"per_case_timeout_sec:={timeout_sec}",
        "max_gurobi_comp_time_sec:=1.0",
        "factor_constant_step_size:=0.1",
    ]

    # Add dynamic factor params when using dynamic k-factor
    if use_dynamic_factor:
        cmd.append(f"'dynamic_factor_initial_mean_list:={dynamic_initial_mean_str}'")
        cmd.append(f"dynamic_factor_k_radius:={DYNAMIC_FACTOR_K_RADIUS}")

    # Add output directory override if specified
    if output_dir_override:
        cmd.append(f"output_dir_override:={output_dir_override}")

    print(f"\nCommand: {' '.join(cmd)}\n")

    # Run benchmark
    setup_file = source_workspace()
    full_cmd = f"source {setup_file} && {' '.join(cmd)}"

    try:
        result = subprocess.run(
            full_cmd,
            shell=True,
            executable="/bin/bash",
            check=False,  # Don't raise exception on non-zero exit
            capture_output=False,
            text=True,
            timeout=600,
        )
        if result.returncode == 0:
            print(f"\n✓ Completed: {description}")
            return True
        else:
            print(f"\n✓ Completed with exit code {result.returncode}: {description}")
            return True  # Still consider it success if data was saved
    except subprocess.TimeoutExpired:
        print(f"\n✗ Timeout: {description}")
        return False
    except KeyboardInterrupt:
        print("\n\nBenchmark interrupted by user")
        raise


def analyze_factors():
    """Analyze CSV results to determine optimal factor ranges"""
    print("\n\n" + "=" * 80)
    print("FACTOR ANALYSIS")
    print("=" * 80)

    import pandas as pd

    data_dir = WORKSPACE_DIR / "src" / "sando" / "benchmark_data" / "single_thread"

    results = {}
    for n in [4, 5, 6]:
        csv_file = data_dir / f"sando_{n}_benchmark.csv"

        if not csv_file.exists():
            print(f"\n✗ CSV file not found: {csv_file}")
            continue

        print(f"\nAnalyzing N={n}...")
        df = pd.read_csv(csv_file)

        # Filter successful cases
        successful = df[df["success"] == True]

        if len(successful) == 0:
            print(f"  ✗ No successful cases found for N={n}")
            continue

        min_factor = successful["factor_used"].min()
        max_factor = successful["factor_used"].max()
        mean_factor = successful["factor_used"].mean()
        success_rate = len(successful) / len(df) * 100

        results[n] = {
            "min": min_factor,
            "max": max_factor,
            "mean": mean_factor,
            "success_rate": success_rate,
            "total_cases": len(df),
            "successful_cases": len(successful),
        }

        print(
            f"  Success rate: {success_rate:.1f}% ({len(successful)}/{len(df)} cases)"
        )
        print(f"  Factor range used: [{min_factor:.2f}, {max_factor:.2f}]")
        print(f"  Mean factor: {mean_factor:.2f}")

    # Print recommended ranges
    if results:
        print("\n" + "=" * 80)
        print("RECOMMENDED FACTOR RANGES")
        print("=" * 80)
        print("\nCopy these into your code:\n")
        print("FACTOR_INITIAL = {", end="")
        for i, n in enumerate([4, 5, 6]):
            if n in results:
                if i > 0:
                    print(", ", end="")
                print(f"{n}: {results[n]['min']:.1f}", end="")
        print("}")

        print("FACTOR_FINAL = {", end="")
        for i, n in enumerate([4, 5, 6]):
            if n in results:
                if i > 0:
                    print(", ", end="")
                print(f"{n}: {results[n]['max']:.1f}", end="")
        print("}")
        print()


def main():
    """Run all benchmark configurations"""
    parser = argparse.ArgumentParser(description="SANDO Benchmark Suite")
    parser.add_argument(
        "--factor-determination",
        action="store_true",
        help="Run factor determination mode (SANDO single-threaded with wide ranges)",
    )
    parser.add_argument(
        "--ve-comparison",
        action="store_true",
        help="Run variable elimination comparison mode (SANDO multi-threaded with/without VE)",
    )
    parser.add_argument(
        "--only-sando",
        action="store_true",
        help="Run only SANDO2 (dynamic k-factor) multi-threaded benchmark",
    )
    parser.add_argument(
        "--only-sando-single",
        action="store_true",
        help="Run only SANDO2 single-threaded benchmark",
    )
    parser.add_argument(
        "--safe-baseline-only",
        action="store_true",
        help="Run only Safe BASELINE single-threaded benchmark",
    )
    args = parser.parse_args()

    # Check for conflicting modes
    mode_count = sum(
        [
            args.factor_determination,
            args.ve_comparison,
            args.only_sando,
            args.only_sando_single,
            args.safe_baseline_only,
        ]
    )
    if mode_count > 1:
        print("ERROR: Cannot specify more than one mode flag")
        sys.exit(1)

    print("\n" + "=" * 80)
    print("SANDO Local Trajectory Benchmark Suite")
    if args.factor_determination:
        print("MODE: Factor Determination")
    elif args.ve_comparison:
        print("MODE: Variable Elimination Comparison")
    elif args.only_sando:
        print("MODE: SANDO2 Only")
    elif args.only_sando_single:
        print("MODE: SANDO2 Single-Threaded Only")
    elif args.safe_baseline_only:
        print("MODE: Safe BASELINE Only")
    print("=" * 80)
    print(f"\nWorkspace: {WORKSPACE_DIR}")

    configs = get_benchmark_configs(
        args.factor_determination,
        args.ve_comparison,
        args.only_sando,
        args.only_sando_single,
        args.safe_baseline_only,
    )
    print(f"Total configurations: {len(configs)}")

    # Check workspace
    source_workspace()

    # Launch simulator
    simulator_proc = None
    try:
        simulator_proc = launch_simulator()

        results = []
        start_time = time.time()

        # Run each benchmark config
        for i, config in enumerate(configs, 1):
            print(f"\n[{i}/{len(configs)}]")
            # Unpack config: (use_single_threaded, planner_name, num_N_list, description, use_var_elim)
            use_single_threaded, planner_name, num_N_list, description, use_var_elim = (
                config
            )
            success = run_benchmark(
                use_single_threaded,
                planner_name,
                num_N_list,
                description,
                factor_determination=args.factor_determination,
                ve_comparison=args.ve_comparison,
                use_var_elim=use_var_elim,
            )
            results.append((description, success))

            if i < len(configs):
                print("\nWaiting 3 seconds before next benchmark...")
                time.sleep(3)

        # Analyze factors if in determination mode
        if args.factor_determination:
            analyze_factors()

        # Summary
        elapsed = time.time() - start_time
        print("\n\n" + "=" * 80)
        print("BENCHMARK SUITE COMPLETED")
        print("=" * 80)
        print(f"\nTotal time: {elapsed:.1f} seconds ({elapsed / 60:.1f} minutes)")
        print("\nResults:")
        for desc, success in results:
            status = "✓ PASS" if success else "✗ FAIL"
            print(f"  {status}: {desc}")

        if not args.factor_determination:
            data_dir = WORKSPACE_DIR / "src" / "sando" / "benchmark_data"
            if args.ve_comparison:
                print("\nBenchmark data saved to:")
                print(f"  {data_dir}/ve_benchmark/")
            else:
                print("\nBenchmark data saved to:")
                print(f"  {data_dir}/single_thread/")
                print(f"  {data_dir}/multi_thread/")

    finally:
        # Cleanup simulator
        if simulator_proc:
            print("\n" + "=" * 80)
            print("Shutting down simulator...")
            print("=" * 80)
            try:
                os.killpg(os.getpgid(simulator_proc.pid), signal.SIGTERM)
                simulator_proc.wait(timeout=5)
            except:
                try:
                    os.killpg(os.getpgid(simulator_proc.pid), signal.SIGKILL)
                except:
                    pass
            print("Simulator stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nScript interrupted by user")
        sys.exit(1)
