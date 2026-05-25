#!/bin/bash
# Overnight re-run of the P=3,4,5 configs (N=5,6) with a LARGER FIXED sample (30
# trials) to beat down the small-sample noise in the unknown_dynamic hard sweep.
#
# Integrity note: this runs a fixed number of trials once per config and reports
# the honest aggregate. It does NOT delete unfavorable trials or stop early when a
# number "looks good" — that would be cherry-picking. P=2 configs are never touched.
#
# Disk-safety: bags are ~18 GB per 30-trial config. We analyze each config while its
# bags exist, record the table row, then delete that config's bags before the next
# config so peak usage stays ~one config's worth.

# NOTE: no `set -u` — ROS setup.bash references unset vars and would abort sourcing.
WS=/home/kkondo/code/sando_ws
cd "$WS"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "$WS/install/setup.bash" >/dev/null 2>&1

BASE="$WS/src/sando/benchmark_data/unknown_dynamic"
EXT="/media/kkondo/kota_elements/sando/benchmark_data/unknown_dynamic"
OUT="$BASE/sweep_tables"
LOG="$BASE/overnight_psweep.log"
ROWS="$BASE/overnight_rows.txt"
TRIALS=30
mkdir -p "$OUT"
: > "$ROWS"

# Move a hard-dir's bags to the external drive (preserve data), with a delete
# fallback to protect the local disk if the external drive is unavailable/full.
archive_bags() {  # $1 = hard dir
  local d="$1"
  [ -d "$d/bags" ] || return 0
  local rel="${d#$BASE/}"           # e.g. N_5/P_3/hard_20260524_...
  local dest="$EXT/$rel"
  if mkdir -p "$dest" 2>/dev/null && mv "$d/bags" "$dest/bags" 2>/dev/null; then
    echo "$(date) :: archived bags -> $dest/bags" | tee -a "$LOG"
  else
    echo "$(date) :: external archive FAILED for $d; deleting bags to protect local disk" | tee -a "$LOG"
    rm -rf "$d/bags"
  fi
}

echo "==== OVERNIGHT SWEEP START $(date) (trials=$TRIALS) ====" | tee -a "$LOG"

# (N P) configs in priority order: P=3 then P=2 first (the ones we care about),
# then P=4, P=5 last (expected low, OK if they run long).
for cfg in "5 3" "6 3" "5 2" "6 2" "5 4" "6 4" "6 5"; do
  set -- $cfg; N=$1; P=$2
  echo "==== $(date) :: N=$N P=$P START ====" | tee -a "$LOG"
  df -h / | tail -1 | tee -a "$LOG"

  # Safety: clean any stale sim. For P!=2, archive any existing local bags for
  # this config to the external drive first (frees local space, keeps the data).
  # P=2 existing data is left fully in place; we add a fresh 30-trial run alongside.
  tmux kill-session -t sando_sim 2>/dev/null
  pkill -9 gzserver 2>/dev/null
  sleep 2
  if [ "$P" != "2" ]; then
    for hd in "$BASE/N_${N}/P_${P}"/hard_*; do
      [ -d "$hd" ] && archive_bags "$hd"
    done
  fi

  python3 "$WS/src/sando/scripts/run_benchmark.py" --setup-bash install/setup.bash \
    --mode gazebo-dynamic --cases hard --config-name unknown_dynamic \
    --num-trials "$TRIALS" --num-n-values "$N" --num-p-values "$P" \
    --start 0.0 0.0 2.0 --goal 105.0 0.0 2.0 --timeout 100 >> "$LOG" 2>&1

  D=$(ls -dt "$BASE/N_${N}/P_${P}"/hard_* 2>/dev/null | head -1)
  echo "$(date) :: analyzing $D" | tee -a "$LOG"
  ros2 run sando analyze_benchmark --data-dir "$D" --latex-dir "$OUT" \
    --latex-name "overnight_N${N}_P${P}.tex" --table-type unknown_dynamic \
    --goal-pos 105.0 0.0 2.0 >> "$LOG" 2>&1

  ROW=$(grep -hE "Hard &" "$OUT/overnight_N${N}_P${P}.tex" 2>/dev/null | tail -1)
  echo "N=$N P=$P :: $ROW" | tee -a "$ROWS"

  # Archive this run's bags to the external drive now that it's analyzed.
  archive_bags "$D"
  echo "==== $(date) :: N=$N P=$P DONE (bags archived) ====" | tee -a "$LOG"
done

echo "==== OVERNIGHT SWEEP COMPLETE $(date) ====" | tee -a "$LOG"
