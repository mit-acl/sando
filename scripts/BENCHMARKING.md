# SANDO Benchmarking Guide

This guide explains how to use the benchmarking system to evaluate SANDO performance.

## Quick Start

```bash
# Navigate to workspace
cd /home/kkondo/code/dynus_ws

# Run a quick benchmark (5 trials)
python3 src/sando/scripts/run_benchmark.py --setup-bash install/setup.bash

# Analyze results
python3 src/sando/scripts/analyze_benchmark.py benchmark_data/*/benchmark_*.csv --plot
```

## System Architecture

The benchmarking system consists of three main components:

1. **run_benchmark.py** - Orchestrates multiple simulation trials
2. **BenchmarkMonitor (ROS2 node)** - Collects real-time data from simulations
3. **analyze_benchmark.py** - Post-processes and visualizes results

### Data Flow

```
run_benchmark.py
    ├─> Launches run_sim.py (starts simulation)
    ├─> Creates BenchmarkMonitor node
    │   ├─> Subscribes to /NX01/odom
    │   ├─> Subscribes to /NX01/goal_reached
    │   └─> Collects position, velocity, acceleration data
    ├─> Computes metrics from collected data
    └─> Saves to CSV/JSON in benchmark_data/
```

## Metrics Collected

### Flight Performance
- **Success Rate**: Percentage of trials that reached the goal
- **Travel Time**: Time from start to goal
- **Path Length**: Total distance traveled
- **Path Efficiency**: Ratio of straight-line distance to actual path (1.0 = optimal)

### Computational Performance
- **Total Replanning Time**: Sum of all replanning computations
- **Average Replanning Time**: Mean time per replan
- **Global Planning Time**: A* search time
- **SFC Corridor Time**: Convex decomposition time
- **Local Trajectory Time**: Trajectory optimization time

### Motion Quality
- **Jerk Integral**: ∫||j(t)||dt - lower is smoother
- **RMS Jerk**: Root mean square jerk - lower is smoother
- **Velocity Statistics**: Average and maximum velocities
- **Acceleration Statistics**: Average and maximum accelerations

### Safety
- **SFC Violations**: Count and magnitude of safe corridor penetrations
- **Velocity Violations**: Instances exceeding v_max
- **Acceleration Violations**: Instances exceeding a_max
- **Jerk Violations**: Instances exceeding j_max

## Usage Examples

### 1. Basic Benchmark

```bash
# 5 trials with default settings (50 obstacles, dynamic_ratio=0.65)
python3 scripts/run_benchmark.py --setup-bash install/setup.bash
```

Output:
```
benchmark_data/
└── 20260203_143022/
    ├── benchmark_default_20260203_143022.csv
    └── benchmark_default_20260203_143022.json
```

### 2. Dense Obstacle Environment

```bash
# 10 trials with 100 obstacles
python3 scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --num-trials 10 \
    --num-obstacles 100 \
    --dynamic-ratio 0.8 \
    --config-name "dense_env"
```

### 3. Long Distance Navigation

```bash
# Test long-range navigation (100m)
python3 scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --num-trials 20 \
    --goal 100 0 3 \
    --timeout 180.0 \
    --config-name "long_range"
```

### 4. Stress Test with High Dynamics

```bash
# Test with aggressive constraints
python3 scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --num-trials 15 \
    --v-max 3.0 \
    --a-max 3.0 \
    --j-max 5.0 \
    --num-obstacles 75 \
    --config-name "high_dynamics"
```

### 5. Reproducibility Test

```bash
# Run multiple trials with same seed to test determinism
python3 scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --num-trials 5 \
    --start-seed 42 \
    --config-name "reproducibility"
```

## Analyzing Results

### Quick Summary

```bash
# View summary statistics
python3 scripts/analyze_benchmark.py \
    benchmark_data/20260203_143022/benchmark_*.csv
```

### Generate Plots

```bash
# Create visualization plots
python3 scripts/analyze_benchmark.py \
    benchmark_data/20260203_143022/benchmark_*.csv \
    --plot \
    --output results_plot.png
```

### Generate Report

```bash
# Create markdown report
python3 scripts/analyze_benchmark.py \
    benchmark_data/20260203_143022/benchmark_*.csv \
    --report \
    --output results_report.md
```

### Compare Multiple Configurations

```bash
# Compare all benchmark runs
python3 scripts/analyze_benchmark.py \
    benchmark_data/*/benchmark_*.csv \
    --plot \
    --output comparison.png
```

## Custom Analysis with Python

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('benchmark_data/20260203_143022/benchmark_default_20260203_143022.csv')

# Filter successful trials
success = df[df['flight_success'] == True]

# Compute success rate
success_rate = len(success) / len(df) * 100
print(f"Success Rate: {success_rate:.1f}%")

# Analyze travel time
print(f"Avg Travel Time: {success['flight_travel_time'].mean():.2f}s")
print(f"Std Travel Time: {success['flight_travel_time'].std():.2f}s")

# Plot travel time vs obstacles
plt.figure(figsize=(10, 6))
plt.scatter(success['num_obstacles'], success['flight_travel_time'])
plt.xlabel('Number of Obstacles')
plt.ylabel('Travel Time (s)')
plt.title('Performance vs Obstacle Density')
plt.grid(True, alpha=0.3)
plt.savefig('custom_analysis.png')
```

### Statistical Testing

```python
from scipy import stats

# Load two configurations
df1 = pd.read_csv('benchmark_data/config1/benchmark_*.csv')
df2 = pd.read_csv('benchmark_data/config2/benchmark_*.csv')

# Filter successful trials
s1 = df1[df1['flight_success'] == True]['flight_travel_time']
s2 = df2[df2['flight_success'] == True]['flight_travel_time']

# Perform t-test
t_stat, p_value = stats.ttest_ind(s1, s2)
print(f"T-statistic: {t_stat:.4f}")
print(f"P-value: {p_value:.4f}")

if p_value < 0.05:
    print("Statistically significant difference detected!")
else:
    print("No significant difference found.")
```

## Advanced Configuration

### Custom Obstacle Distribution

```bash
python3 scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --num-obstacles 60 \
    --obs-x-range 10.0 80.0 \
    --obs-y-range -10.0 10.0 \
    --obs-z-range 1.0 5.0 \
    --config-name "custom_obstacles"
```

### Batch Processing

Create a shell script for running multiple configurations:

```bash
#!/bin/bash
# batch_benchmark.sh

SETUP_BASH="install/setup.bash"

# Test different obstacle densities
for NUM_OBS in 25 50 75 100; do
    python3 scripts/run_benchmark.py \
        --setup-bash $SETUP_BASH \
        --num-trials 10 \
        --num-obstacles $NUM_OBS \
        --config-name "obstacles_${NUM_OBS}" \
        --output-dir benchmark_data/obstacle_sweep
done

# Analyze all results
python3 scripts/analyze_benchmark.py \
    "benchmark_data/obstacle_sweep/*/benchmark_*.csv" \
    --plot \
    --output benchmark_data/obstacle_sweep/comparison.png
```

## Troubleshooting

### Issue: Trials timeout consistently

**Solution**: Increase timeout or reduce difficulty
```bash
python3 scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --timeout 300.0 \
    --num-obstacles 30
```

### Issue: ROS2 topics not being received

**Check**:
1. Simulation is launching correctly
2. Namespace matches (default: NX01)
3. Topics are being published: `ros2 topic list`

### Issue: Missing computation time data

**Note**: Computation time metrics (global planning, SFC corridor, local trajectory) are not yet published to topics. Future enhancement will add these via a dedicated timing topic.

**Current workaround**: Parse ROS logs for timing information

### Issue: Memory issues with many trials

**Solution**: Run in batches
```bash
# Run 100 trials in batches of 10
for i in {0..9}; do
    python3 scripts/run_benchmark.py \
        --setup-bash install/setup.bash \
        --num-trials 10 \
        --start-seed $((i * 10)) \
        --config-name "batch_${i}" \
        --output-dir benchmark_data/large_benchmark
done
```

## Best Practices

1. **Start small**: Test with 3-5 trials first to validate setup
2. **Use meaningful config names**: Helps organize and identify results
3. **Monitor first trial**: Watch the first trial to catch issues early
4. **Save raw data**: Keep CSV/JSON files for future re-analysis
5. **Document configurations**: Note parameter changes in experiment logs
6. **Use version control**: Track changes to benchmark parameters
7. **Replicate**: Run multiple seeds for statistical significance

## Integration with Jupyter Notebooks

```python
# Load in Jupyter notebook
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load all benchmark data
df = pd.read_csv('benchmark_data/20260203_143022/benchmark_default_*.csv')

# Interactive visualization
%matplotlib inline
sns.set_style("whitegrid")

# Create dashboard
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Success rate
success_rate = df.groupby('seed')['flight_success'].mean() * 100
axes[0,0].bar(success_rate.index, success_rate.values)
axes[0,0].set_title('Success Rate by Seed')
axes[0,0].set_ylabel('Success Rate (%)')

# Travel time distribution
success_df = df[df['flight_success'] == True]
axes[0,1].hist(success_df['flight_travel_time'], bins=20)
axes[0,1].set_title('Travel Time Distribution')
axes[0,1].set_xlabel('Time (s)')

# Path efficiency
axes[1,0].scatter(success_df['path_length'], success_df['path_efficiency'])
axes[1,0].set_title('Path Efficiency vs Length')
axes[1,0].set_xlabel('Path Length (m)')
axes[1,0].set_ylabel('Efficiency')

# Smoothness
axes[1,1].scatter(success_df['jerk_integral'], success_df['jerk_rms'])
axes[1,1].set_title('Smoothness Metrics')
axes[1,1].set_xlabel('Jerk Integral')
axes[1,1].set_ylabel('RMS Jerk')

plt.tight_layout()
plt.show()
```

## Future Enhancements

- [ ] Add rosbag recording for detailed post-analysis
- [ ] Publish computation time metrics to dedicated topic
- [ ] Add obstacle avoidance margin metrics
- [ ] Include map quality metrics (occupancy accuracy)
- [ ] Add multi-agent benchmark support
- [ ] Implement real-time monitoring dashboard
- [ ] Add automatic parameter tuning based on results

## Contributing

To add new metrics:

1. Update `BenchmarkMetrics` dataclass in `run_benchmark.py`
2. Add collection logic in `BenchmarkMonitor` callbacks
3. Update `compute_metrics()` to calculate new metric
4. Add visualization in `analyze_benchmark.py`
5. Update documentation

## Support

For issues or questions:
- Check this documentation
- Review example usage in `/benchmark_data/README.md`
- Examine source code comments
- Open an issue on the SANDO repository
