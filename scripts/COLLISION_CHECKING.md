# SANDO Collision Checking Guide

This guide explains how to use the collision checking system to verify trajectory safety.

## Overview

The collision checking system performs post-flight analysis to detect collisions between the drone trajectory and obstacles, accounting for:
- **Drone bounding box**: Size of the drone (from `drone_bbox` in sando.yaml)
- **Obstacle bounding boxes**: Different sizes for each obstacle type (cubes, pillars, walls)
- **Dynamic obstacle motion**: Trefoil knot trajectories for moving obstacles
- **Safety margins**: Optional additional clearance requirements

## Quick Start

### Integrated with Benchmarks (Automatic)

The benchmarking system automatically performs collision checking:

```bash
# Run benchmark - collision checking happens automatically
python3 scripts/run_benchmark.py --setup-bash install/setup.bash

# Check results - collision metrics are included
python3 scripts/analyze_benchmark.py benchmark_data/*/benchmark_*.csv
```

Collision metrics in benchmark output:
- `collision`: True if any collisions detected
- `collision_count`: Number of collision events
- `collision_penetration_max`: Maximum penetration depth (meters)
- `collision_penetration_avg`: Average penetration depth
- `collision_unique_obstacles`: Number of different obstacles hit
- `collision_free_ratio`: Ratio of collision-free trajectory segments

### Standalone Collision Checking

For detailed collision analysis on existing trajectory data:

```bash
# Check collisions with saved obstacle data
python3 scripts/check_collisions.py \
    --trajectory path/to/trajectory.csv \
    --obstacles path/to/obstacles.json \
    --drone-bbox 0.3 0.3 0.15

# Or load drone bbox from sando.yaml
python3 scripts/check_collisions.py \
    --trajectory path/to/trajectory.csv \
    --obstacles path/to/obstacles.json \
    --sando-yaml config/sando.yaml
```

## Saving Obstacle Data

If you need to save obstacle data separately (for custom analysis):

```bash
# Save obstacles during simulation
python3 scripts/save_obstacles.py \
    --output obstacles.json \
    --topic /obstacles \
    --duration 5.0
```

This creates a JSON file with all obstacle information:
```json
{
  "obstacles": [
    {
      "id": "static_obs_0",
      "position": [25.0, 3.5, 2.0],
      "half_extents": [0.2, 0.2, 2.0],
      "is_dynamic": false,
      "namespace": "static_obs",
      "marker_id": 0
    },
    {
      "id": "dynamic_obs_0",
      "position": [40.0, 0.0, 3.0],
      "half_extents": [0.2, 0.2, 0.2],
      "is_dynamic": true,
      "namespace": "dynamic_obs",
      "marker_id": 0
    }
  ],
  "num_obstacles": 50,
  "num_dynamic": 32,
  "num_static": 18
}
```

## Obstacle Types and Sizes

### MADER-Style Obstacles (Default Configuration)

1. **Dynamic Cubes** (65% of obstacles)
   - Size: 0.8×0.8×0.8 meters (bbox half-extents: 0.4×0.4×0.4)
   - Motion: Trefoil knot trajectory
   - Use case: Moving obstacles

2. **Static Vertical Pillars** (17.5% of obstacles)
   - Size: 0.8×0.8×4.0 meters (bbox half-extents: 0.4×0.4×2.0)
   - Motion: Stationary
   - Use case: Vertical obstacles (trees, poles)

3. **Static Horizontal Walls** (17.5% of obstacles)
   - Size: 0.8×4.0×0.8 meters (bbox half-extents: 0.4×2.0×0.4)
   - Motion: Stationary
   - Use case: Horizontal obstacles (low-hanging branches)

### Drone Bounding Box

Configured in `config/sando.yaml`:
```yaml
sando:
  ros__parameters:
    drone_bbox: [0.3, 0.3, 0.15]  # [hx, hy, hz] half-extents in meters
```

Typical drone sizes:
- **Small quadcopter**: [0.15, 0.15, 0.08]
- **Medium quadcopter** (default): [0.3, 0.3, 0.15]
- **Large quadcopter**: [0.5, 0.5, 0.2]

## Collision Detection Algorithm

### AABB Intersection Test

The system uses Axis-Aligned Bounding Box (AABB) intersection:

```python
def intersects(drone_bbox, obstacle_bbox):
    return (
        drone.min_x <= obstacle.max_x and drone.max_x >= obstacle.min_x and
        drone.min_y <= obstacle.max_y and drone.max_y >= obstacle.min_y and
        drone.min_z <= obstacle.max_z and drone.max_z >= obstacle.min_z
    )
```

### Penetration Depth

If boxes intersect, penetration depth is calculated as the minimum overlap:

```python
penetration_depth = min(
    overlap_x,  # min(drone.max_x, obs.max_x) - max(drone.min_x, obs.min_x)
    overlap_y,
    overlap_z
)
```

### Safety Margins

Optional safety margin adds clearance around obstacles:

```bash
# Add 10cm safety margin
python3 scripts/check_collisions.py \
    --trajectory traj.csv \
    --obstacles obs.json \
    --safety-margin 0.1
```

## Understanding Collision Reports

### Terminal Output

```
================================================================================
COLLISION ANALYSIS RESULTS
================================================================================
Total collision events: 15
Unique obstacles hit: 3
Collision-free segments: 985 / 1000
Collision-free ratio: 98.50%
Max penetration depth: 0.0523m
Avg penetration depth: 0.0234m
================================================================================
```

### JSON Report

Detailed collision events saved to `*_collision_report.json`:

```json
{
  "trajectory_file": "trajectory.csv",
  "statistics": {
    "total_collisions": 15,
    "unique_obstacles_hit": 3,
    "collision_free_segments": 985,
    "total_segments": 1000,
    "collision_free_ratio": 0.985,
    "max_penetration": 0.0523,
    "avg_penetration": 0.0234
  },
  "collision_events": [
    {
      "time": 5.23,
      "drone_position": [25.4, 3.2, 2.9],
      "obstacle_id": "static_obs_12",
      "penetration_depth": 0.0523,
      "obstacle_position": [25.0, 3.5, 2.0],
      "obstacle_half_extents": [0.2, 0.2, 2.0]
    }
  ]
}
```

### Text Summary

Human-readable summary in `*_collision_report.txt`:

```
================================================================================
COLLISION ANALYSIS REPORT
================================================================================

Trajectory: benchmark_data/trial_0.csv

Statistics:
  total_collisions: 15
  unique_obstacles_hit: 3
  collision_free_segments: 985
  total_segments: 1000
  collision_free_ratio: 0.985
  max_penetration: 0.0523
  avg_penetration: 0.0234

================================================================================
Collision Events (15 total):
================================================================================

[1] t=5.230s, obstacle=static_obs_12, penetration=0.0523m
    Drone pos: (25.40, 3.20, 2.90)
    Obs pos: (25.00, 3.50, 2.00)
    Obs size: (0.20, 0.20, 2.00)
...
```

## Use Cases

### 1. Verify Benchmark Safety

```bash
# Run benchmark with collision checking
python3 scripts/run_benchmark.py --setup-bash install/setup.bash

# Check results
python3 scripts/analyze_benchmark.py benchmark_data/*/benchmark_*.csv

# Filter for collision-free trials
import pandas as pd
df = pd.read_csv('benchmark_data/.../benchmark_*.csv')
safe_trials = df[df['collision_count'] == 0]
print(f"Safe trials: {len(safe_trials)}/{len(df)}")
```

### 2. Debug Collision Issues

```bash
# Run detailed collision check on failed trial
python3 scripts/check_collisions.py \
    --trajectory benchmark_data/trial_5.csv \
    --obstacles benchmark_data/obstacles.json \
    --drone-bbox 0.3 0.3 0.15 \
    --safety-margin 0.0

# Examine collision report
cat benchmark_data/trial_5_collision_report.txt
```

### 3. Test Different Drone Sizes

```bash
# Test with smaller drone
python3 scripts/check_collisions.py \
    --trajectory traj.csv \
    --obstacles obs.json \
    --drone-bbox 0.15 0.15 0.08

# Test with larger drone
python3 scripts/check_collisions.py \
    --trajectory traj.csv \
    --obstacles obs.json \
    --drone-bbox 0.5 0.5 0.2
```

### 4. Validate Safety Margins

```bash
# Check if 5cm margin would prevent collisions
python3 scripts/check_collisions.py \
    --trajectory traj.csv \
    --obstacles obs.json \
    --safety-margin 0.05

# Check if 10cm margin would prevent collisions
python3 scripts/check_collisions.py \
    --trajectory traj.csv \
    --obstacles obs.json \
    --safety-margin 0.10
```

## Integration with Analysis

### Python Analysis with Collision Data

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load benchmark with collision data
df = pd.read_csv('benchmark_data/20260203_120000/benchmark_default_*.csv')

# Analyze collision-free vs collision trials
safe = df[df['collision_count'] == 0]
unsafe = df[df['collision_count'] > 0]

print(f"Safety rate: {len(safe)}/{len(df)} ({100*len(safe)/len(df):.1f}%)")

# Plot collision penetration distribution
plt.figure(figsize=(10, 6))
plt.hist(unsafe['collision_penetration_max'], bins=20, edgecolor='black')
plt.xlabel('Maximum Penetration Depth (m)')
plt.ylabel('Frequency')
plt.title('Collision Severity Distribution')
plt.axvline(0.05, color='r', linestyle='--', label='5cm threshold')
plt.legend()
plt.savefig('collision_analysis.png')

# Correlation between path efficiency and collisions
print(f"Avg efficiency (safe): {safe['path_efficiency'].mean():.3f}")
print(f"Avg efficiency (unsafe): {unsafe['path_efficiency'].mean():.3f}")
```

### Filtering Safe Trajectories

```python
import pandas as pd

df = pd.read_csv('benchmark_results.csv')

# Define safety criteria
safety_margin_threshold = 0.01  # 1cm max penetration considered "safe"

truly_safe = df[
    (df['collision_count'] == 0) |
    (df['collision_penetration_max'] <= safety_margin_threshold)
]

print(f"Strictly safe: {len(df[df['collision_count'] == 0])}")
print(f"Safe with margin: {len(truly_safe)}")
```

## Troubleshooting

### Issue: No obstacles detected

**Cause**: Obstacle topic not publishing or wrong topic name

**Solution**: Check obstacle topic
```bash
# List available topics
ros2 topic list | grep obstacle

# Echo obstacle topic
ros2 topic echo /obstacles --once
```

### Issue: Collision checker reports false positives

**Cause**: Incorrect bounding box sizes

**Solution**: Verify bbox configuration
```bash
# Check drone bbox in config
grep -A 3 "drone_bbox" config/sando.yaml

# Verify obstacle sizes match simulation
python3 scripts/save_obstacles.py --output obs_verify.json --duration 3.0
cat obs_verify.json | jq '.obstacles[0]'
```

### Issue: Missing collision data in benchmark

**Cause**: check_collisions module import failed

**Solution**: Ensure check_collisions.py is in the same directory as run_benchmark.py

```bash
ls -la scripts/check_collisions.py
ls -la scripts/run_benchmark.py
```

## Future Enhancements

- [ ] Support for oriented bounding boxes (OBB) for rotated obstacles
- [ ] Time-varying collision detection for dynamic obstacles
- [ ] Minimum separation distance tracking
- [ ] Collision heatmap visualization
- [ ] Integration with rosbag replay
- [ ] GPU-accelerated collision checking for large datasets

## Best Practices

1. **Always verify obstacle data**: Use `save_obstacles.py` to confirm obstacle info is correct
2. **Match simulation config**: Ensure drone_bbox in collision check matches simulation
3. **Consider safety margins**: Test with 5-10cm margins for real-world deployment
4. **Review collision reports**: Don't just look at counts - check penetration depths
5. **Validate trajectories**: A trajectory with zero collisions isn't necessarily safe - check clearance margins
