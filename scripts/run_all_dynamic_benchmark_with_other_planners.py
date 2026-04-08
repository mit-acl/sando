#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) Anonymous Author
# Anonymous Institution
# All Rights Reserved
# Authors: Anonymous
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""Unified benchmark runner for SANDO, I-MPC, FAPP, and Ego-Swarm2 planners."""

import argparse
import glob
import os
import subprocess
import time

TEST_TIMEOUT = 5  # seconds per command in --test mode

def _get_planner_configs():
    """Build planner configs using environment variables for workspace paths.

    Set these environment variables to override the default working directories:
        SANDO_WORKSPACE_DIR  - SANDO workspace (default: auto-detect from script location)
        IMPC_WORKSPACE_DIR   - I-MPC workspace
        FAPP_WORKSPACE_DIR   - FAPP workspace
        EGO_WORKSPACE_DIR    - EGO-Swarm v2 workspace
    """
    _sando_ws = os.environ.get(
        "SANDO_WORKSPACE_DIR",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")),
    )
    _impc_ws = os.environ.get("IMPC_WORKSPACE_DIR", "")
    _fapp_ws = os.environ.get("FAPP_WORKSPACE_DIR", "")
    _ego_ws = os.environ.get("EGO_WORKSPACE_DIR", "")

    return {
        "sando": {
            "name": "SANDO",
            "key": "sando",
            "working_dir": _sando_ws,
            "benchmark_cmd": (
                "colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando"
                " && . install/setup.bash"
                " && python3 src/sando/scripts/run_benchmark.py"
                " --setup-bash install/setup.bash"
                " --num-trials {num_trials}"
                " --start-seed {seed_start}"
            ),
            "analysis_cmd": (
                ". install/setup.bash"
                " && python3 src/sando/scripts/analyze_dynamic_benchmark.py"
                " --data-dir {data_dir}"
            ),
            "data_base_dir": "src/sando/benchmark_data/default",
            "data_glob_pattern": "*",
        },
        "impc": {
            "name": "I-MPC",
            "key": "impc",
            "working_dir": os.path.join(_impc_ws, "docker") if _impc_ws else "",
            "benchmark_cmd": "make run-benchmark-sweep NUM_TRIALS={num_trials}",
            "analysis_cmd": "make analyze-benchmark-sweep",
            "data_base_dir": "../data",
            "data_glob_pattern": "benchmark_wa*",
        },
        "fapp": {
            "name": "FAPP",
            "key": "fapp",
            "working_dir": os.path.join(_fapp_ws, "docker") if _fapp_ws else "",
            "benchmark_cmd": "make run-benchmark-sweep NUM_TRIALS={num_trials}",
            "analysis_cmd": "make analyze-benchmark-sweep",
            "data_base_dir": "../data",
            "data_glob_pattern": "benchmark_wt*",
        },
        "ego": {
            "name": "EGO-Swarm v2",
            "key": "ego",
            "working_dir": os.path.join(_ego_ws, "docker") if _ego_ws else "",
            "benchmark_cmd": "make run-dynamic-benchmark-sweep NUM_TRIALS={num_trials}",
            "analysis_cmd": "make analyze-dynamic-sweep",
            "data_base_dir": "../data_dynamic",
            "data_glob_pattern": "benchmark_*_wt*",
        },
    }


PLANNERS = _get_planner_configs()

PLANNER_ORDER = ["sando", "impc", "fapp", "ego"]


def find_latest_data_dir(planner, after_time=None):
    """Find the latest data directory for a planner.

    Args:
        planner: Planner config dict.
        after_time: If set, only consider dirs with mtime > after_time.

    Returns:
        Path to the latest data directory, or None if not found.
    """
    base = os.path.join(planner["working_dir"], planner["data_base_dir"])
    pattern = os.path.join(base, planner["data_glob_pattern"])
    candidates = glob.glob(pattern)
    candidates = [c for c in candidates if os.path.isdir(c)]

    if after_time is not None:
        candidates = [c for c in candidates if os.path.getmtime(c) > after_time]

    if not candidates:
        return None

    return max(candidates, key=os.path.getmtime)


def _run_cmd(cmd, cwd, timeout=None):
    """Run a shell command, optionally with a timeout.

    If timeout is set and expires, the process group is killed and
    TimeoutExpired is raised.
    """
    proc = subprocess.run(
        cmd,
        shell=True,
        cwd=cwd,
        check=True,
        timeout=timeout,
        start_new_session=True,
    )


def run_benchmark(planner, num_trials, seed_start, timeout=None):
    """Run the benchmark for a single planner.

    Returns:
        The path to the output data directory.
    """
    before_time = time.time()
    cmd = planner["benchmark_cmd"].format(num_trials=num_trials, seed_start=seed_start)
    print(f"\n>>> Running benchmark: {cmd}")
    print(f">>> Working dir: {planner['working_dir']}\n")
    _run_cmd(cmd, cwd=planner["working_dir"], timeout=timeout)

    data_dir = find_latest_data_dir(planner, after_time=before_time)
    if data_dir is None:
        print(f"WARNING: Could not find new data directory for {planner['name']}")
    return data_dir


def run_analysis(planner, data_dir, timeout=None):
    """Run the analysis for a single planner."""
    if planner["key"] == "sando":
        # SANDO needs --data-dir with path relative to working_dir
        rel_data_dir = os.path.relpath(data_dir, planner["working_dir"])
        cmd = planner["analysis_cmd"].format(data_dir=rel_data_dir)
    else:
        # I-MPC, FAPP, Ego-Swarm2 sweep analysis auto-discovers directories
        cmd = planner["analysis_cmd"]

    print(f"\n>>> Running analysis: {cmd}")
    print(f">>> Working dir: {planner['working_dir']}\n")
    _run_cmd(cmd, cwd=planner["working_dir"], timeout=timeout)


def format_duration(seconds):
    """Format a duration in seconds to a human-readable string."""
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    if h > 0:
        return f"{h}h {m:02d}m {s:02d}s"
    return f"{m}m {s:02d}s"


def main():
    parser = argparse.ArgumentParser(
        description="Run benchmarks and analysis for all planners."
    )
    parser.add_argument(
        "--num-trials", type=int, required=True, help="Number of trials per planner"
    )
    parser.add_argument(
        "--seed-start", type=int, default=0, help="Starting random seed (default: 0)"
    )
    parser.add_argument(
        "--planners",
        nargs="+",
        choices=PLANNER_ORDER,
        default=PLANNER_ORDER,
        help="Subset of planners to run (default: all)",
    )
    parser.add_argument(
        "--skip-benchmark",
        action="store_true",
        help="Skip benchmark phase, run analysis only on latest data",
    )
    parser.add_argument(
        "--skip-analysis",
        action="store_true",
        help="Skip analysis phase, run benchmarks only",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help=f"Test mode: run each command for {TEST_TIMEOUT}s then move on",
    )
    args = parser.parse_args()

    if args.test:
        print(
            f"*** TEST MODE: each command will run for {TEST_TIMEOUT}s then move on ***\n"
        )

    selected = [PLANNERS[k] for k in args.planners]

    # Track results: {key: {benchmark, analysis, time, data_dir}}
    results = {}

    # Phase 1: Benchmarks
    if not args.skip_benchmark:
        print("=" * 66)
        print("                    BENCHMARK PHASE")
        print("=" * 66)
        for planner in selected:
            key = planner["key"]
            print(f"\n{'─' * 66}")
            print(f"  Starting benchmark for {planner['name']}")
            print(f"{'─' * 66}")
            timeout = TEST_TIMEOUT if args.test else None
            t0 = time.time()
            try:
                data_dir = run_benchmark(
                    planner, args.num_trials, args.seed_start, timeout=timeout
                )
                elapsed = time.time() - t0
                results[key] = {
                    "benchmark": "SUCCESS",
                    "time": elapsed,
                    "data_dir": data_dir,
                }
                print(
                    f"\n>>> {planner['name']} benchmark completed in"
                    f" {format_duration(elapsed)}"
                )
            except subprocess.TimeoutExpired:
                elapsed = time.time() - t0
                print(
                    f"\n>>> TEST MODE: {planner['name']} benchmark ran for"
                    f" {format_duration(elapsed)}, moving on"
                )
                results[key] = {
                    "benchmark": "TEST_OK",
                    "time": elapsed,
                    "data_dir": find_latest_data_dir(planner),
                }
            except subprocess.CalledProcessError as e:
                elapsed = time.time() - t0
                results[key] = {
                    "benchmark": "FAILED",
                    "time": elapsed,
                    "data_dir": None,
                }
                print(
                    f"\n>>> {planner['name']} benchmark FAILED (exit code"
                    f" {e.returncode}) after {format_duration(elapsed)}"
                )

    # For --skip-benchmark, find latest data dirs
    if args.skip_benchmark:
        for planner in selected:
            key = planner["key"]
            data_dir = find_latest_data_dir(planner)
            results[key] = {
                "benchmark": "SKIPPED",
                "time": 0,
                "data_dir": data_dir,
            }
            if data_dir:
                print(f"Found existing data for {planner['name']}: {data_dir}")
            else:
                print(f"WARNING: No existing data found for {planner['name']}")

    # Phase 2: Analysis
    if not args.skip_analysis:
        print(f"\n{'=' * 66}")
        print("                    ANALYSIS PHASE")
        print("=" * 66)
        for planner in selected:
            key = planner["key"]
            result = results.get(key, {})
            data_dir = result.get("data_dir")

            if data_dir is None:
                result["analysis"] = "SKIPPED"
                print(
                    f"\n>>> Skipping analysis for {planner['name']} (no data directory)"
                )
                continue

            if result.get("benchmark") == "FAILED":
                result["analysis"] = "SKIPPED"
                print(
                    f"\n>>> Skipping analysis for {planner['name']} (benchmark failed)"
                )
                continue

            print(f"\n{'─' * 66}")
            print(f"  Starting analysis for {planner['name']}")
            print(f"{'─' * 66}")
            timeout = TEST_TIMEOUT if args.test else None
            try:
                run_analysis(planner, data_dir, timeout=timeout)
                result["analysis"] = "SUCCESS"
                print(f"\n>>> {planner['name']} analysis completed")
            except subprocess.TimeoutExpired:
                result["analysis"] = "TEST_OK"
                print(
                    f"\n>>> TEST MODE: {planner['name']} analysis ran for"
                    f" {TEST_TIMEOUT}s, moving on"
                )
            except subprocess.CalledProcessError as e:
                result["analysis"] = "FAILED"
                print(
                    f"\n>>> {planner['name']} analysis FAILED"
                    f" (exit code {e.returncode})"
                )
    else:
        for planner in selected:
            key = planner["key"]
            if key in results:
                results[key]["analysis"] = "SKIPPED"

    # Summary
    print(f"\n{'=' * 66}")
    print("                   BENCHMARK RESULTS SUMMARY")
    print("=" * 66)
    print(f"{'Planner':<14} {'Benchmark':<12} {'Time':<11} {'Analysis':<10} Data Dir")
    print("-" * 66)
    for planner in selected:
        key = planner["key"]
        r = results.get(key, {})
        bm = r.get("benchmark", "N/A")
        elapsed = r.get("time", 0)
        time_str = format_duration(elapsed) if elapsed > 0 else "-"
        analysis = r.get("analysis", "N/A")
        data_dir = r.get("data_dir")
        if data_dir:
            # Show path relative to working_dir for brevity
            try:
                data_dir_display = os.path.relpath(data_dir, planner["working_dir"])
            except ValueError:
                data_dir_display = data_dir
        else:
            data_dir_display = "(no data)"
        print(
            f"{planner['name']:<14} {bm:<12} {time_str:<11} {analysis:<10}"
            f" {data_dir_display}"
        )
    print("=" * 66)
    latex_path = "(set --latex-dir to specify output location)/dynamic_benchmark.tex"
    print(f"LaTeX table: {latex_path}")
    print("=" * 66)


if __name__ == "__main__":
    main()
