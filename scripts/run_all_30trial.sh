#!/bin/bash
# Re-run ALL unknown-dynamic + horizon-ablation benchmarks at 30 trials / 40 s timeout,
# fresh and self-consistent (old data was deleted). After each config is recorded and
# analyzed, its bags are MOVED to the external drive to keep local disk usage bounded.
#
# Dedup: shared configurations are run ONCE under a canonical path and reused by the
# tables; they are not listed twice here.
#   - L3 P3 (P3,mdv1.0,L3)      == unknown_dynamic/N_5/P_3/hard
#   - fixedP2 mdv1.0 (P2,L2)    == unknown_dynamic/N_5/P_2/hard
#   - fixedP2 mdv1.5 (P2,L3)    == horizon_ablation_L3/N_5/P_2/hard
#
# Resumable: a config whose <cfg_dir>/stats_cache.json already exists is skipped.
#
# Usage: run_all_30trial.sh

set -o pipefail
WS=/home/kkondo/code/sando_ws
SANDO="$WS/src/sando"
cd "$WS"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "$WS/install/setup.bash" >/dev/null 2>&1

TRIALS=30
TIMEOUT=70
EXT=/media/kkondo/kota_elements/sando/benchmark_data
BD="$SANDO/benchmark_data"
LOG="$BD/run_all_30trial.log"
SRC_YAML="$SANDO/config/sando.yaml"
INSTALL_YAML="$WS/install/sando/share/sando/config/sando.yaml"
mkdir -p "$BD"

# (N P mdv case outdir_base)  --  HARD configs first (feed Psweep + disentangle), then easy/medium.
CONFIGS=(
  # --- hard: unknown_dynamic (mdv=1.0) ---
  "5 2 1.0 hard   $BD/unknown_dynamic"
  "5 3 1.0 hard   $BD/unknown_dynamic"
  "5 4 1.0 hard   $BD/unknown_dynamic"
  "6 2 1.0 hard   $BD/unknown_dynamic"
  "6 3 1.0 hard   $BD/unknown_dynamic"
  "6 4 1.0 hard   $BD/unknown_dynamic"
  "6 5 1.0 hard   $BD/unknown_dynamic"
  # --- hard: horizon extras (set mdv) ---
  "5 2 1.5  hard  $BD/horizon_ablation_L3"          # L3 P2  (== fixedP2 mdv1.5)
  "5 4 0.75 hard  $BD/horizon_ablation_L3"          # L3 P4
  "5 2 2.0  hard  $BD/horizon_ablation_fixedP2/mdv_2.0"
  "5 2 2.5  hard  $BD/horizon_ablation_fixedP2/mdv_2.5"
  "5 2 3.0  hard  $BD/horizon_ablation_fixedP2/mdv_3.0"
  # --- easy/medium: unknown_dynamic (mdv=1.0) ---
  "5 2 1.0 easy   $BD/unknown_dynamic"
  "5 2 1.0 medium $BD/unknown_dynamic"
  "5 3 1.0 easy   $BD/unknown_dynamic"
  "5 3 1.0 medium $BD/unknown_dynamic"
  "5 4 1.0 easy   $BD/unknown_dynamic"
  "5 4 1.0 medium $BD/unknown_dynamic"
  "6 2 1.0 easy   $BD/unknown_dynamic"
  "6 2 1.0 medium $BD/unknown_dynamic"
  "6 3 1.0 easy   $BD/unknown_dynamic"
  "6 3 1.0 medium $BD/unknown_dynamic"
  "6 4 1.0 easy   $BD/unknown_dynamic"
  "6 4 1.0 medium $BD/unknown_dynamic"
  "6 5 1.0 easy   $BD/unknown_dynamic"
  "6 5 1.0 medium $BD/unknown_dynamic"
)

set_mdv() {  # $1 = value
  local v="$1"
  for y in "$SRC_YAML" "$INSTALL_YAML"; do
    [ -f "$y" ] && python3 - "$y" "$v" <<'PY'
import re, sys
p, val = sys.argv[1], sys.argv[2]
s = open(p).read()
s = re.sub(r'(max_dist_vertexes:\s*)[0-9.]+', r'\g<1>'+val, s)
open(p, 'w').write(s)
PY
  done
}
trap 'echo "Restoring max_dist_vertexes=1.0"; set_mdv 1.0' EXIT

log(){ echo "$(date '+%F %T') :: $*" | tee -a "$LOG"; }

avail=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
log "==== RUN ALL 30-trial START (trials=$TRIALS timeout=${TIMEOUT}s) :: free ${avail} GB ===="
[ "$avail" -lt 40 ] && { log "ABORT: <40 GB free"; exit 1; }

for entry in "${CONFIGS[@]}"; do
  read -r N P MDV CASE OUTDIR <<< "$entry"
  CFG_DIR="$OUTDIR/N_${N}/P_${P}/${CASE}"
  REL="${CFG_DIR#$BD/}"
  TAG="N${N} P${P} mdv${MDV} ${CASE}"

  if [ -f "$CFG_DIR/stats_cache.json" ]; then
    log "SKIP (already done): $TAG  ->  $CFG_DIR"
    continue
  fi

  af=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
  [ "$af" -lt 40 ] && { log "ABORT before $TAG: <40 GB free ($af)"; exit 1; }

  log "---- START $TAG  ->  $CFG_DIR ----"
  set_mdv "$MDV"
  tmux kill-session -t sando_sim 2>/dev/null
  pkill -9 gzserver 2>/dev/null
  sleep 2

  python3 "$SANDO/scripts/run_benchmark.py" --setup-bash install/setup.bash \
    --mode gazebo-dynamic --cases "$CASE" --config-name unknown_dynamic \
    --num-trials "$TRIALS" --num-n-values "$N" --num-p-values "$P" \
    --output-dir "$OUTDIR" \
    --start 0.0 0.0 2.0 --goal 105.0 0.0 2.0 --timeout "$TIMEOUT" >>"$LOG" 2>&1

  nbags=$(ls -d "$CFG_DIR"/bags/trial_* 2>/dev/null | wc -l)
  log "run done: $TAG  bags=$nbags/$TRIALS"

  # Analyze AFTER the sim is down (no CPU contention) -> writes stats_cache.json locally.
  ros2 run sando analyze_benchmark --data-dir "$CFG_DIR" \
    --latex-dir "$OUTDIR/sweep_tables" --latex-name "N${N}_P${P}_${CASE}_mdv${MDV}.tex" \
    --table-type unknown_dynamic --goal-pos 105.0 0.0 2.0 --recompute >>"$LOG" 2>&1
  [ -f "$CFG_DIR/stats_cache.json" ] || { log "ERROR: no stats_cache for $TAG -- aborting"; exit 1; }

  # Archive bags to external drive (preserve data, free local disk).
  if [ -d "$CFG_DIR/bags" ]; then
    DEST="$EXT/$REL"
    if mkdir -p "$DEST" 2>/dev/null && mv "$CFG_DIR/bags" "$DEST/bags" 2>/dev/null; then
      log "archived bags -> $DEST/bags"
    else
      log "ERROR: external archive FAILED for $TAG (dest=$DEST) -- aborting to preserve data & disk"
      exit 1
    fi
  fi
  log "---- DONE $TAG ----"
done

log "==== RUN ALL 30-trial COMPLETE ===="
