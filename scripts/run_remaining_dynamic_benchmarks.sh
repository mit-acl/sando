#!/bin/bash
# Wait for I-MPC to finish, then run FAPP and EGO-Swarm2

IMPC_PID=1549519
LOG_DIR="$HOME/code/sando_ws/benchmark_logs/$(date +%Y%m%d_%H%M%S)_fapp_ego"
mkdir -p "$LOG_DIR"

echo "============================================================"
echo "  Waiting for I-MPC (PID $IMPC_PID) to finish..."
echo "  Started monitoring: $(date)"
echo "============================================================"

# Wait for I-MPC process to complete
while kill -0 $IMPC_PID 2>/dev/null; do
    sleep 30
done

echo ""
echo "  I-MPC finished: $(date)"
echo ""

# ── FAPP ──────────────────────────────────────────────────────
echo "============================================================"
echo "  [1/2] FAPP - Dynamic Benchmark"
echo "  Directory: ~/code/fapp_ws/src/FAPP/docker"
echo "  Started: $(date)"
echo "============================================================"

cd "$HOME/code/fapp_ws/src/FAPP/docker"
make run-benchmark-sweep CASES='easy medium hard' NUM_TRIALS=10 \
  2>&1 | tee "$LOG_DIR/fapp.log"

echo ""
echo "  [1/2] FAPP complete: $(date)"
echo ""

# ── EGO-Swarm2 ───────────────────────────────────────────────
echo "============================================================"
echo "  [2/2] EGO-Swarm2 - Dynamic Benchmark"
echo "  Directory: ~/code/ego_swarm_v2_ws/src/EGO-Planner-v2/docker"
echo "  Started: $(date)"
echo "============================================================"

cd "$HOME/code/ego_swarm_v2_ws/src/EGO-Planner-v2/docker"
make run-dynamic-benchmark-sweep NUM_TRIALS=10 \
  2>&1 | tee "$LOG_DIR/ego_swarm2.log"

echo ""
echo "  [2/2] EGO-Swarm2 complete: $(date)"
echo ""

echo "============================================================"
echo "  ALL REMAINING BENCHMARKS COMPLETE"
echo "  Finished: $(date)"
echo "  Logs: $LOG_DIR"
echo "============================================================"
