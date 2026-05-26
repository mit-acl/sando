#!/bin/bash
# Axis B of the horizon-disentanglement study (reviewer response, Table IX confound).
#
# Companion to horizon_disentangle_sweep.sh (Axis A). Axis A held the horizon fixed
# and varied P to isolate MIQP complexity. Axis B does the opposite: it FIXES P (and
# therefore the polytope count / MIQP complexity) and varies max_dist_vertexes to
# sweep the LOCAL HORIZON  L = num_P * max_dist_vertexes.
#
# With P fixed at 2 and max_dist_vertexes in {1.0,1.5,2.0,2.5,3.0}, the local horizon
# L = 2 * mdv = {2,3,4,5,6} m, while the number of polytopes (complexity) stays
# constant. This isolates how much of the performance change comes purely from the
# horizon length (terminal distance -> dt -> corridor inflation conservatism).
#
# Because P and N are fixed, every config would otherwise collide in N_5/P_2/hard, so
# each mdv writes to its own subdir: <base>/mdv_<v>/N_5/P_2/hard.
#
# Usage: horizon_axisB_fixedP_sweep.sh [num_trials] [output_subdir]
#   num_trials    default 1  (validation pass; use 10 for the real run)
#   output_subdir default horizon_ablation_fixedP2

set -o pipefail
WS=/home/kkondo/code/sando_ws
SANDO="$WS/src/sando"
cd "$WS"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "$WS/install/setup.bash" >/dev/null 2>&1

TRIALS="${1:-1}"
OUTSUB="${2:-horizon_ablation_fixedP2}"
N=5
P=2
MDV_SWEEP=(1.0 1.5 2.0 2.5 3.0)
BASE="$SANDO/benchmark_data/$OUTSUB"

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

echo "==== HORIZON AXIS-B SWEEP  N=$N  P=$P (fixed)  mdv=${MDV_SWEEP[*]}  trials=$TRIALS ===="
echo "==== output base -> $BASE ===="

for MDV in "${MDV_SWEEP[@]}"; do
  L=$(python3 -c "print(f'{$P*$MDV:.1f}')")
  OUTDIR="$BASE/mdv_${MDV}"
  echo "==== $(date) :: max_dist_vertexes=$MDV  P=$P -> horizon L=$L m  (out: mdv_${MDV}) ===="
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

  echo "--- post-run install yaml check (mdv target=$MDV; num_P restored by run_benchmark) ---"
  grep -nE 'num_P:|num_N:|max_dist_vertexes:' "$INSTALL_YAML"
done

echo "==== AXIS-B SWEEP DONE $(date) ===="
