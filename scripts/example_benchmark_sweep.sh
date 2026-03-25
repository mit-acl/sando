#!/bin/bash
# Example benchmark sweep script
# This demonstrates how to run systematic benchmarks across multiple configurations

set -e  # Exit on error

SETUP_BASH="install/setup.bash"
OUTPUT_BASE="benchmark_data/sweep_$(date +%Y%m%d_%H%M%S)"

echo "============================================"
echo "SANDO Benchmark Sweep"
echo "============================================"
echo "Output directory: $OUTPUT_BASE"
echo ""

# Create output directory
mkdir -p "$OUTPUT_BASE"

# Function to run a benchmark configuration
run_config() {
    local config_name=$1
    local num_trials=$2
    shift 2
    local extra_args="$@"

    echo ""
    echo "--------------------------------------------"
    echo "Running: $config_name"
    echo "Trials: $num_trials"
    echo "Args: $extra_args"
    echo "--------------------------------------------"

    python3 src/sando/scripts/run_benchmark.py \
        --setup-bash "$SETUP_BASH" \
        --num-trials "$num_trials" \
        --config-name "$config_name" \
        --output-dir "$OUTPUT_BASE" \
        $extra_args

    echo "✓ $config_name complete"
}

# 1. Baseline configuration
run_config "baseline" 10 \
    --num-obstacles 50 \
    --dynamic-ratio 0.65

# 2. Obstacle density sweep
for NUM_OBS in 25 50 75 100; do
    run_config "obstacles_${NUM_OBS}" 10 \
        --num-obstacles "$NUM_OBS" \
        --dynamic-ratio 0.65
done

# 3. Dynamic ratio sweep
for RATIO in 0.0 0.3 0.5 0.7 1.0; do
    RATIO_NAME=$(echo "$RATIO" | sed 's/\./_/g')
    run_config "dynamic_ratio_${RATIO_NAME}" 10 \
        --num-obstacles 50 \
        --dynamic-ratio "$RATIO"
done

# 4. Distance sweep
for DISTANCE in 25 50 75 100; do
    run_config "distance_${DISTANCE}m" 10 \
        --goal "$DISTANCE" 0.0 3.0 \
        --num-obstacles 50 \
        --dynamic-ratio 0.65 \
        --timeout 180.0
done

# 5. Velocity limits sweep
for VMAX in 1.0 2.0 3.0; do
    VMAX_NAME=$(echo "$VMAX" | sed 's/\./_/g')
    run_config "vmax_${VMAX_NAME}" 10 \
        --num-obstacles 50 \
        --dynamic-ratio 0.65 \
        --v-max "$VMAX" \
        --a-max "$VMAX" \
        --j-max "$((VMAX + 1.0))"
done

echo ""
echo "============================================"
echo "Benchmark sweep complete!"
echo "============================================"
echo ""
echo "Generating comparison plots..."

# Generate comparison visualizations
python3 src/sando/scripts/analyze_benchmark.py \
    "$OUTPUT_BASE/*/benchmark_*.csv" \
    --plot \
    --output "$OUTPUT_BASE/comparison_all.png"

echo ""
echo "Results saved to: $OUTPUT_BASE"
echo ""
echo "To analyze individual configurations:"
echo "  python3 src/sando/scripts/analyze_benchmark.py $OUTPUT_BASE/baseline/benchmark_*.csv --plot"
echo ""
echo "To generate detailed report:"
echo "  python3 src/sando/scripts/analyze_benchmark.py $OUTPUT_BASE/*/benchmark_*.csv --report"
echo ""
