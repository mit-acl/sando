#!/usr/bin/env python3
"""Sweep max_gurobi_comp_time_sec over several values and report success rate.

For each requested Gurobi time limit this script:
  1. Rewrites ``max_gurobi_comp_time_sec`` in BOTH the src and install copies of
     sando.yaml (runtime param, no recompile needed).
  2. Runs the dynamic ground-truth benchmark (rviz-only, hard case, N trials).
  3. Relocates the produced ``hard_<timestamp>`` directory into
     ``benchmark_data/dynamic_gt/gurobi_<t>/`` so it matches the existing layout.
  4. Runs analyze_dynamic_benchmark.py on it and parses the success rate and
     per-optimization (Gurobi) time out of the generated LaTeX row.

The original yaml value is always restored at the end. The paper LaTeX table is
NOT touched: each analysis writes to a scratch dir so the sweep does not leave the
paper in a half-updated state. Pick the winner, then run the real analyzer command.

Example:
    python3 src/sando/scripts/sweep_gurobi_comp_time.py \
        --comp-times 0.8 0.75 0.6 0.5 0.4 0.3 0.25 --num-trials 10
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent          # .../src/sando/scripts
PKG_DIR = SCRIPT_DIR.parent                            # .../src/sando
WS_DIR = PKG_DIR.parent.parent                         # .../sando_ws
SRC_YAML = PKG_DIR / "config" / "sando.yaml"
INSTALL_YAML = WS_DIR / "install" / "sando" / "share" / "sando" / "config" / "sando.yaml"
DYNAMIC_GT_DIR = PKG_DIR / "benchmark_data" / "dynamic_gt"

PARAM_RE = re.compile(r"(max_gurobi_comp_time_sec:\s*)[\d.]+")


def read_current_comp_time() -> str:
    m = PARAM_RE.search(SRC_YAML.read_text())
    if not m:
        raise RuntimeError(f"max_gurobi_comp_time_sec not found in {SRC_YAML}")
    return SRC_YAML.read_text()[m.start():m.end()].split(":")[1].strip()


def set_comp_time(value: float):
    for yaml_path in (SRC_YAML, INSTALL_YAML):
        if not yaml_path.exists():
            print(f"  WARNING: {yaml_path} not found, skipping")
            continue
        content = yaml_path.read_text()
        new_content, n = PARAM_RE.subn(rf"\g<1>{value}", content)
        if n == 0:
            raise RuntimeError(f"max_gurobi_comp_time_sec not found in {yaml_path}")
        yaml_path.write_text(new_content)
    print(f"  set max_gurobi_comp_time_sec = {value} (src + install)")


def restore_comp_time(raw_value: str):
    for yaml_path in (SRC_YAML, INSTALL_YAML):
        if not yaml_path.exists():
            continue
        content = yaml_path.read_text()
        content = PARAM_RE.sub(rf"\g<1>{raw_value}", content)
        yaml_path.write_text(content)
    print(f"Restored max_gurobi_comp_time_sec = {raw_value}")


def latest_hard_dir(before: set) -> Path | None:
    """Return the newly created hard_* dir under dynamic_gt that wasn't there before."""
    now = {p for p in DYNAMIC_GT_DIR.glob("hard_*") if p.is_dir()}
    new = sorted(now - before)
    return new[-1] if new else None


def run_benchmark(num_trials: int, timeout: float, log_handle) -> bool:
    cmd = [
        "python3", "src/sando/scripts/run_benchmark.py",
        "--setup-bash", "install/setup.bash",
        "--mode", "rviz-only",
        "--cases", "hard",
        "--config-name", "dynamic_gt",
        "--num-trials", str(num_trials),
        "--start", "0.0", "0.0", "2.0",
        "--goal", "105.0", "0.0", "2.0",
        "--timeout", str(timeout),
    ]
    log_handle.write(f"\n$ {' '.join(cmd)}\n")
    log_handle.flush()
    proc = subprocess.run(cmd, cwd=str(WS_DIR), stdout=log_handle,
                          stderr=subprocess.STDOUT)
    return proc.returncode == 0


def run_analyzer(case_dir: Path, scratch_latex_dir: Path, tag: str,
                 goal_pos, log_handle) -> dict:
    scratch_latex_dir.mkdir(parents=True, exist_ok=True)
    latex_name = f"dynamic_benchmark_{tag}.tex"
    cmd = [
        "python3", "src/sando/scripts/analyze_dynamic_benchmark.py",
        "--data-dir", str(case_dir),
        "--latex-dir", str(scratch_latex_dir),
        "--latex-name", latex_name,
        "--table-type", "dynamic",
        "--goal-pos", str(goal_pos[0]), str(goal_pos[1]), str(goal_pos[2]),
    ]
    log_handle.write(f"\n$ {' '.join(cmd)}\n")
    log_handle.flush()
    proc = subprocess.run(cmd, cwd=str(WS_DIR), stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True)
    log_handle.write(proc.stdout)
    log_handle.flush()

    result = {"success_rate": None, "per_opt_time": None, "raw_row": None}

    # Prefer parsing the generated LaTeX SANDO row:
    #   & \multicolumn{2}{c}{SANDO} & <succ> & <per_opt> & <travel> & ...
    latex_file = scratch_latex_dir / latex_name
    if latex_file.exists():
        for line in latex_file.read_text().splitlines():
            if "SANDO" in line and "multicolumn" in line:
                after = line.split("SANDO}")[-1]
                nums = re.findall(r"[-+]?\d*\.\d+|\d+", after)
                if len(nums) >= 2:
                    result["success_rate"] = float(nums[0])
                    result["per_opt_time"] = float(nums[1])
                    result["raw_row"] = line.strip()
                break

    # Fallback: parse "Success rate: X%" from analyzer stdout.
    if result["success_rate"] is None:
        m = re.search(r"Success rate:\s*([\d.]+)%", proc.stdout)
        if m:
            result["success_rate"] = float(m.group(1))

    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--comp-times", type=float, nargs="+",
                    default=[0.8, 0.75, 0.6, 0.5, 0.4, 0.3, 0.25],
                    help="Gurobi time-limit values to sweep")
    ap.add_argument("--num-trials", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=50.0)
    ap.add_argument("--goal-pos", type=float, nargs=3, default=[105.0, 0.0, 2.0])
    args = ap.parse_args()

    DYNAMIC_GT_DIR.mkdir(parents=True, exist_ok=True)
    scratch_latex_dir = DYNAMIC_GT_DIR / "sweep_tables"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = DYNAMIC_GT_DIR / f"sweep_{stamp}.log"
    summary_path = DYNAMIC_GT_DIR / f"sweep_{stamp}_summary.json"

    original_value = read_current_comp_time()
    print(f"Original max_gurobi_comp_time_sec = {original_value}")
    print(f"Sweeping: {args.comp_times}")
    print(f"Log: {log_path}")

    results = []
    log = open(log_path, "w")
    try:
        for t in args.comp_times:
            tag = f"gurobi_{t}"
            print(f"\n{'=' * 70}\n[{tag}] starting  ({datetime.now():%H:%M:%S})\n{'=' * 70}")
            log.write(f"\n{'=' * 70}\n[{tag}] {datetime.now()}\n{'=' * 70}\n")
            log.flush()

            set_comp_time(t)
            before = {p for p in DYNAMIC_GT_DIR.glob("hard_*") if p.is_dir()}

            ok = run_benchmark(args.num_trials, args.timeout, log)
            if not ok:
                print(f"[{tag}] benchmark run FAILED (see log)")
                results.append({"comp_time": t, "status": "run_failed"})
                continue

            produced = latest_hard_dir(before)
            if produced is None:
                print(f"[{tag}] no hard_* output directory found")
                results.append({"comp_time": t, "status": "no_output"})
                continue

            dest_dir = DYNAMIC_GT_DIR / tag
            dest_dir.mkdir(parents=True, exist_ok=True)
            moved = dest_dir / produced.name
            if moved.exists():
                shutil.rmtree(moved)
            shutil.move(str(produced), str(moved))

            metrics = run_analyzer(moved, scratch_latex_dir, tag,
                                   args.goal_pos, log)
            sr = metrics["success_rate"]
            pot = metrics["per_opt_time"]
            print(f"[{tag}] success_rate={sr}%  per_opt_time={pot}ms  -> {moved}")
            results.append({
                "comp_time": t, "status": "ok", "data_dir": str(moved),
                "success_rate": sr, "per_opt_time": pot,
                "latex_row": metrics["raw_row"],
            })
            summary_path.write_text(json.dumps(results, indent=2))
    finally:
        restore_comp_time(original_value)
        log.close()

    summary_path.write_text(json.dumps(results, indent=2))

    print(f"\n{'=' * 70}\nSWEEP SUMMARY\n{'=' * 70}")
    print(f"{'comp_time':>10} | {'success%':>9} | {'per_opt[ms]':>11} | status")
    print("-" * 55)
    for r in results:
        sr = f"{r.get('success_rate')}" if r.get("success_rate") is not None else "-"
        pot = f"{r.get('per_opt_time')}" if r.get("per_opt_time") is not None else "-"
        print(f"{r['comp_time']:>10} | {sr:>9} | {pot:>11} | {r['status']}")
    print(f"\nSummary JSON: {summary_path}")
    print(f"Full log:     {log_path}")
    print("\nPaper table NOT modified. To commit the winner, run the real analyzer "
          "command pointing --data-dir at that setting's hard_* dir.")


if __name__ == "__main__":
    main()
