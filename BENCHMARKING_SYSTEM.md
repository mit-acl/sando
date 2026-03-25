# SANDO Comprehensive Benchmarking System

Complete benchmarking solution for SANDO with collision checking, obstacle tracking, and performance analysis.

## System Components

### 1. Core Benchmarking (`scripts/run_benchmark.py`)
Main benchmarking orchestrator that:
- Launches multiple simulation trials with different configurations
- Monitors flight via ROS2 subscribers
- Collects comprehensive performance metrics
- **Automatically performs collision checking**
- Saves results to CSV and JSON

**Metrics Collected:**
- ✅ Flight success (goal reached)
- ✅ Flight travel time
- ✅ Path length and efficiency
- ✅ Smoothness (jerk integral, RMS jerk)
- ✅ Velocity/acceleration statistics
- ✅ Constraint violations (SFC, velocity, accel, jerk)
- ✅ **Collision detection (automatic with obstacle bbox)**
- ⚠️ Computation times (needs topic publisher integration)

### 2. Collision Checking (`scripts/check_collisions.py`)
Post-flight safety analysis that:
- Performs AABB collision detection
- Accounts for drone bounding box (from sando.yaml)
- Accounts for obstacle bounding boxes (cubes, pillars, walls)
- Supports safety margins
- Generates detailed collision reports

**Features:**
- Automatic integration with benchmarking
- Standalone mode for existing trajectory data
- Penetration depth calculation
- Per-obstacle collision tracking

### 3. Obstacle Data Saver (`scripts/save_obstacles.py`)
Utility to capture obstacle information:
- Subscribes to `/trajs` topic (DynTraj messages)
- Extracts position and bounding box data from obstacle trajectories
- Tracks dynamic obstacle motion over time
- Saves to JSON for collision checking
- Automatically integrated in benchmark system

### 4. Analysis Tools (`scripts/analyze_benchmark.py`)
Visualization and statistical analysis:
- Summary statistics with collision metrics
- 9-subplot comprehensive visualization
- Markdown report generation
- Multi-configuration comparison support

### 5. Example Scripts
- `scripts/example_benchmark_sweep.sh` - Batch benchmarking across configurations
- Demonstrates systematic parameter sweeps

## Quick Start

### Run Benchmark with Full Analysis

```bash
# Navigate to workspace
cd /home/kkondo/code/dynus_ws

# Run 5 trials with collision checking (automatic)
python3 src/sando/scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --num-trials 5

# Analyze results
python3 src/sando/scripts/analyze_benchmark.py \
    benchmark_data/*/benchmark_*.csv \
    --plot
```

### Results Include Collision Data

CSV output automatically includes:
```csv
...,collision,collision_count,collision_penetration_max,collision_penetration_avg,collision_unique_obstacles,collision_free_ratio,...
...,False,0,0.0,0.0,0,1.0,...
...,True,15,0.0523,0.0234,3,0.985,...
```

## Documentation

- **`scripts/BENCHMARKING.md`** - Complete benchmarking guide with examples
- **`scripts/COLLISION_CHECKING.md`** - Detailed collision checking documentation
- **`benchmark_data/README.md`** - Data format and analysis guide

## Workflow

### Automated Workflow (Recommended)

```bash
# 1. Run benchmark - everything automated
python3 scripts/run_benchmark.py --setup-bash install/setup.bash --num-trials 10

# 2. Results automatically include:
#    - Flight metrics
#    - Collision detection
#    - Obstacle data
#    - All in one CSV file

# 3. Analyze
python3 scripts/analyze_benchmark.py benchmark_data/*/benchmark_*.csv --plot
```

### Manual Workflow (Advanced)

```bash
# 1. Save obstacle data during simulation
python3 scripts/save_obstacles.py --output obstacles.json --duration 5.0 --topic /trajs &

# 2. Run simulation
python3 scripts/run_sim.py --mode rviz-only --setup-bash install/setup.bash

# 3. After flight, check collisions
python3 scripts/check_collisions.py \
    --trajectory recorded_trajectory.csv \
    --obstacles obstacles.json \
    --drone-bbox 0.3 0.3 0.15
```

**Note**: The benchmark system now:
- Subscribes to `/trajs` topic published by `dynamic_forest_node`
- Tracks obstacle positions over time for accurate dynamic obstacle collision detection
- Interpolates obstacle positions to match trajectory timestamps
- Automatically loads drone bounding box from `config/sando.yaml`

## Collision Detection Details

### Data Source

The collision checking system uses the `/trajs` topic:
- **Published by**: `dynamic_forest_node` (from `dyn_obstacles.launch.py`)
- **Message type**: `dynus_interfaces/msg/DynTraj`
- **Contains**: Obstacle position, bounding box (half-extents), and analytical trajectory expressions
- **Update rate**: ~50 Hz
- **Dynamic obstacle handling**: Positions are tracked over time and interpolated to match drone trajectory timestamps

### Obstacle Types Supported

1. **Dynamic Cubes** (0.8×0.8×0.8m full size)
   - Half-extents for collision: (0.4, 0.4, 0.4)
   - Trefoil knot motion
   - DynTraj.bbox = [0.8, 0.8, 0.8] (full sizes)

2. **Static Vertical Pillars** (0.4×0.4×4.0m full size)
   - Half-extents for collision: (0.2, 0.2, 2.0)
   - Stationary
   - DynTraj.bbox = [0.4, 0.4, 4.0] (full sizes)

3. **Static Horizontal Walls** (0.4×4.0×0.8m full size)
   - Half-extents for collision: (0.2, 2.0, 0.4)
   - Stationary
   - DynTraj.bbox = [0.4, 4.0, 0.8] (full sizes)

**Important Note**: DynTraj.bbox contains **full sizes** [width, height, depth], not half-extents. The collision checker automatically divides by 2 to compute half-extents for AABB collision detection.

### Drone Bounding Box

Configured in `config/sando.yaml`:
```yaml
sando:
  ros__parameters:
    drone_bbox: [0.6, 0.6, 0.3]  # [width, height, depth] in meters (FULL sizes)
```

**Important**: `drone_bbox` contains **full sizes** [width, height, depth]. The collision checker automatically divides by 2 to get half-extents (0.3, 0.3, 0.15) for AABB collision detection.

### Algorithm

- **Method**: Axis-Aligned Bounding Box (AABB) intersection
- **Penetration**: Minimum overlap distance across all axes
- **Frequency**: Checked at every trajectory sample point

## Example Usage

### Basic Benchmark

```bash
python3 scripts/run_benchmark.py --setup-bash install/setup.bash
```

### Dense Environment Test

```bash
python3 scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --num-trials 20 \
    --num-obstacles 100 \
    --dynamic-ratio 0.8 \
    --config-name "dense_obstacles"
```

### Long Distance Test

```bash
python3 scripts/run_benchmark.py \
    --setup-bash install/setup.bash \
    --num-trials 15 \
    --goal 100 0 3 \
    --timeout 180.0 \
    --config-name "long_range"
```

### Systematic Sweep

```bash
bash scripts/example_benchmark_sweep.sh
```

## Analysis Examples

### Check Success Rate

```python
import pandas as pd

df = pd.read_csv('benchmark_data/20260203_120000/benchmark_default_*.csv')

# Overall success
success_rate = df['flight_success'].mean() * 100
print(f"Success rate: {success_rate:.1f}%")

# Collision-free success
collision_free = df[df['collision_count'] == 0]
collision_free_rate = len(collision_free) / len(df) * 100
print(f"Collision-free rate: {collision_free_rate:.1f}%")

# Success without collisions
safe_success = df[(df['flight_success'] == True) & (df['collision_count'] == 0)]
safe_success_rate = len(safe_success) / len(df) * 100
print(f"Safe success rate: {safe_success_rate:.1f}%")
```

### Compare Collision Severity

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('benchmark_results.csv')

# Filter trials with collisions
collisions = df[df['collision_count'] > 0]

# Plot penetration depth
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.hist(collisions['collision_penetration_max'], bins=20, edgecolor='black')
plt.xlabel('Max Penetration Depth (m)')
plt.ylabel('Frequency')
plt.title('Collision Severity Distribution')
plt.axvline(0.05, color='r', linestyle='--', label='5cm threshold')
plt.legend()

plt.subplot(1, 2, 2)
plt.scatter(collisions['collision_unique_obstacles'],
           collisions['collision_penetration_max'])
plt.xlabel('Number of Obstacles Hit')
plt.ylabel('Max Penetration (m)')
plt.title('Collision Complexity')

plt.tight_layout()
plt.savefig('collision_analysis.png')
```

### Filter Safe Trials

```python
import pandas as pd

df = pd.read_csv('benchmark_results.csv')

# Define safety criteria
PENETRATION_THRESHOLD = 0.01  # 1cm

safe_trials = df[
    (df['collision_count'] == 0) |
    (df['collision_penetration_max'] <= PENETRATION_THRESHOLD)
]

print(f"Safe trials: {len(safe_trials)}/{len(df)}")
print(f"\nSafe trial statistics:")
print(f"  Avg travel time: {safe_trials['flight_travel_time'].mean():.2f}s")
print(f"  Avg path efficiency: {safe_trials['path_efficiency'].mean():.3f}")
print(f"  Avg jerk RMS: {safe_trials['jerk_rms'].mean():.2f}")
```

## File Structure

```
sando/
├── scripts/
│   ├── run_benchmark.py              # Main benchmark orchestrator
│   ├── check_collisions.py           # Collision checker (integrated & standalone)
│   ├── save_obstacles.py             # Obstacle data saver
│   ├── analyze_benchmark.py          # Analysis and visualization
│   ├── example_benchmark_sweep.sh    # Batch benchmarking example
│   ├── BENCHMARKING.md              # Benchmarking guide
│   └── COLLISION_CHECKING.md        # Collision checking guide
├── benchmark_data/
│   ├── README.md                     # Data format documentation
│   └── YYYYMMDD_HHMMSS/             # Timestamped results
│       ├── benchmark_*.csv           # Results CSV
│       ├── benchmark_*.json          # Results JSON
│       └── obstacles.json            # Obstacle data (if saved)
├── config/
│   └── sando.yaml                    # Contains drone_bbox config
└── BENCHMARKING_SYSTEM.md           # This file
```

## Output Files

### Benchmark CSV
Contains all metrics including collision data:
- Flight performance metrics
- Smoothness metrics
- Constraint violations
- **Collision metrics** (count, penetration, unique obstacles, etc.)

### Benchmark JSON
Same data in JSON format for easier programmatic access

### Collision Report (when using standalone checker)
- `*_collision_report.json` - Detailed collision events
- `*_collision_report.txt` - Human-readable summary

## Key Features

✅ **Fully Automated**: Run benchmarks, get collision data automatically
✅ **Bbox-Aware**: Correctly handles different obstacle sizes (cubes, pillars, walls)
✅ **Drone-Aware**: Reads drone size from sando.yaml configuration
✅ **Comprehensive Metrics**: 30+ metrics per trial
✅ **Visualization**: Automated plotting with matplotlib/seaborn
✅ **Statistical Analysis**: Built-in summary statistics
✅ **Batch Processing**: Easy parameter sweep scripts
✅ **Export Formats**: CSV, JSON for maximum compatibility

## Future Enhancements

Potential additions:
- [ ] Real-time collision monitoring during flight
- [ ] Publish computation times to ROS topics
- [ ] Rosbag integration for replay analysis
- [ ] Oriented bounding box (OBB) support
- [ ] Collision heatmap visualization
- [ ] Multi-agent collision detection
- [ ] Parameter optimization based on benchmarks

## Support and Troubleshooting

### Common Issues

1. **No collision data in results**
   - Ensure `/obstacles` topic is publishing
   - Check that `check_collisions.py` is in scripts directory
   - Verify imports work: `python3 -c "from check_collisions import CollisionChecker"`

2. **Incorrect collision detection**
   - Verify drone_bbox in `config/sando.yaml`
   - Check obstacle sizes with `save_obstacles.py`
   - Confirm obstacle topic name (default: `/obstacles`)

3. **Benchmark timeouts**
   - Increase `--timeout` value
   - Reduce obstacle density
   - Check simulation is starting correctly

### Getting Help

1. Check documentation in `scripts/BENCHMARKING.md`
2. Review examples in `scripts/example_benchmark_sweep.sh`
3. Examine collision guide in `scripts/COLLISION_CHECKING.md`
4. Inspect code comments in `scripts/run_benchmark.py`

## Credits

Benchmarking system developed for SANDO (Dynamic UAV Navigation and Uncertainty-aware Safe corridors).
Collision checking uses AABB intersection with bbox-aware obstacle representation.

---

**Ready to use!** The system is fully functional and can be used immediately:

```bash
# Start benchmarking with collision checking now
python3 src/sando/scripts/run_benchmark.py --setup-bash install/setup.bash
```
