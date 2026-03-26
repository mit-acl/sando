#!/bin/bash
#
# Run dynamic environment benchmarks for all 4 planners sequentially.
# Each planner starts only after the previous one completes.
#
# Usage:
#   bash src/sando/scripts/run_all_dynamic_benchmarks.sh
#
# Run from: ~/code/sando_ws

set -e  # Exit on first failure

NUM_TRIALS=${1:-10}  # Default 10, override with first argument

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$HOME/code/sando_ws/benchmark_logs/${TIMESTAMP}"
mkdir -p "$LOG_DIR"

echo "============================================================"
echo "  DYNAMIC ENVIRONMENT BENCHMARK SUITE"
echo "  Started: $(date)"
echo "  Logs: $LOG_DIR"
echo "============================================================"
echo ""

# ── 1. SANDO ──────────────────────────────────────────────────
echo "============================================================"
echo "  [1/4] SANDO - Dynamic Benchmark"
echo "  Directory: ~/code/sando_ws"
echo "  Started: $(date)"
echo "============================================================"

cd "$HOME/code/sando_ws"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando \
  && . install/setup.bash \
  && python3 src/sando/scripts/run_benchmark.py \
       --setup-bash install/setup.bash \
       --mode rviz-only \
       --cases easy medium hard \
       --config-name dynamic \
       --num-trials $NUM_TRIALS \
       --start 0.0 0.0 2.0 \
       --goal 105.0 0.0 2.0 \
       --timeout 100 \
  2>&1 | tee "$LOG_DIR/sando.log"

echo ""
echo "  [1/4] SANDO complete: $(date)"
echo ""

# ── 2. I-MPC ──────────────────────────────────────────────────
echo "============================================================"
echo "  [2/4] I-MPC - Dynamic Benchmark"
echo "  Directory: ~/code/ip-mpc_ws/Intent-MPC/docker"
echo "  Started: $(date)"
echo "============================================================"

cd "$HOME/code/ip-mpc_ws/Intent-MPC/docker"
make run-full-sweep NUM_TRIALS=$NUM_TRIALS \
  2>&1 | tee "$LOG_DIR/impc.log"

echo ""
echo "  [2/4] I-MPC complete: $(date)"
echo ""

# ── 3. FAPP ───────────────────────────────────────────────────
echo "============================================================"
echo "  [3/4] FAPP - Dynamic Benchmark"
echo "  Directory: ~/code/fapp_ws/src/FAPP/docker"
echo "  Started: $(date)"
echo "============================================================"

cd "$HOME/code/fapp_ws/src/FAPP/docker"
make run-benchmark-sweep CASES='easy medium hard' NUM_TRIALS=$NUM_TRIALS \
  2>&1 | tee "$LOG_DIR/fapp.log"

echo ""
echo "  [3/4] FAPP complete: $(date)"
echo ""

# ── 4. EGO-Swarm2 ────────────────────────────────────────────
echo "============================================================"
echo "  [4/4] EGO-Swarm2 - Dynamic Benchmark"
echo "  Directory: ~/code/ego_swarm_v2_ws/src/EGO-Planner-v2/docker"
echo "  Started: $(date)"
echo "============================================================"

cd "$HOME/code/ego_swarm_v2_ws/src/EGO-Planner-v2/docker"
make run-dynamic-benchmark-sweep NUM_TRIALS=$NUM_TRIALS \
  2>&1 | tee "$LOG_DIR/ego_swarm2.log"

echo ""
echo "  [4/4] EGO-Swarm2 complete: $(date)"
echo ""

# ── Summary ───────────────────────────────────────────────────
echo "============================================================"
echo "  ALL BENCHMARKS COMPLETE"
echo "  Finished: $(date)"
echo "  Logs saved to: $LOG_DIR"
echo "============================================================"
