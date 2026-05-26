#!/bin/bash
# Horizon-disentanglement sweep (reviewer response, Table IX confound).
#
# The reviewer noted that increasing P in the unknown-dynamic sweep simultaneously
# (a) pushes the local terminal point farther (local_E = global_path.back(), and the
# global path is clipped to num_P+1 vertices spaced max_dist_vertexes apart, so the
# local horizon == num_P * max_dist_vertexes), and (b) grows dt -> grows corridor
# inflation (r = obst_max_vel * traj_max_time + ...), and (c) adds polytopes (MIQP
# complexity). All three are entangled.
#
# This sweep holds the LOCAL HORIZON FIXED at L meters while varying P, by setting
#     max_dist_vertexes = L / P
# so num_P * max_dist_vertexes == L for every P. That fixes the terminal distance,
# dt, and inflation by construction, leaving only MIQP complexity + corridor
# granularity as the residual "P effect" -- which is exactly what we want to isolate.
#
# Usage: horizon_disentangle_sweep.sh [num_trials] [output_subdir]
#   num_trials    default 1  (validation pass; use 10 for the real run)
#   output_subdir default horizon_ablation_L3

set -o pipefail
WS=/home/kkondo/code/sando_ws
SANDO="$WS/src/sando"
cd "$WS"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "$WS/install/setup.bash" >/dev/null 2>&1

TRIALS="${1:-1}"
OUTSUB="${2:-horizon_ablation_L3}"
L=3.0
N=5
PSWEEP=(2 3 4)
OUTDIR="$SANDO/benchmark_data/$OUTSUB"

SRC_YAML="$SANDO/config/sando.yaml"
INSTALL_YAML="$WS/install/sando/share/sando/config/sando.yaml"

# Capture original max_dist_vertexes so we can restore it no matter how we exit.
ORIG_MDV=$(grep -oP 'max_dist_vertexes:\s*\K[0-9.]+' "$SRC_YAML" | head -1)
echo "Original max_dist_vertexes = $ORIG_MDV  (will restore on exit)"

set_mdv() {  # $1 = new value
  local v="$1"
  for y in "$SRC_YAML" "$INSTALL_YAML"; do
    [ -f "$y" ] && python3 - "$y" "$v" <<'PY'
import re, sys
path, val = sys.argv[1], sys.argv[2]
s = open(path).read()
s = re.sub(r'(max_dist_vertexes:\s*)[0-9.]+', r'\g<1>'+val, s)
open(path,'w').write(s)
PY
  done
}

restore_mdv() {
  echo "Restoring max_dist_vertexes = $ORIG_MDV"
  set_mdv "$ORIG_MDV"
}
trap restore_mdv EXIT

echo "==== HORIZON-DISENTANGLE SWEEP  L=$L  N=$N  P=${PSWEEP[*]}  trials=$TRIALS ===="
echo "==== output -> $OUTDIR ===="

for P in "${PSWEEP[@]}"; do
  MDV=$(python3 -c "print(f'{$L/$P:.6f}')")
  echo "==== $(date) :: N=$N P=$P  -> max_dist_vertexes=$MDV  (P*mdv=$L) ===="
  set_mdv "$MDV"

  # Safety: clean any stale sim before each config.
  tmux kill-session -t sando_sim 2>/dev/null
  pkill -9 gzserver 2>/dev/null
  sleep 2

  python3 "$SANDO/scripts/run_benchmark.py" --setup-bash install/setup.bash \
    --mode gazebo-dynamic --cases hard --config-name unknown_dynamic \
    --num-trials "$TRIALS" --num-n-values "$N" --num-p-values "$P" \
    --output-dir "$OUTDIR" \
    --start 0.0 0.0 2.0 --goal 105.0 0.0 2.0 --timeout 100

  # Verify the install yaml actually carries the intended values for this config.
  echo "--- post-run install yaml check (N=$N P=$P) ---"
  grep -nE 'num_P:|num_N:|max_dist_vertexes:' "$INSTALL_YAML"
done

echo "==== SWEEP DONE $(date) ===="
