# SANDO: Safe Autonomous Trajectory Planning for Dynamic Unknown Environments

*Under review (double-blind). Author identifying material has been removed from this snapshot.*

> Supplementary videos are omitted from this anonymous snapshot.

SANDO plans safe, dynamically-feasible trajectories for UAVs in environments with both static and dynamic obstacles, including previously unseen ones detected at runtime via onboard sensors.

## Quick Start (Docker)

```bash
git clone --recursive <ANONYMIZED_REPO_URL> && cd sando/docker
make build                         # ~15 min first time
make run-interactive               # click goals in RViz!
```

## What You Can Do

SANDO provides **four simulation modes** at three difficulty levels (50 / 100 / 200 obstacles):

| Mode | Description | Engine |
|------|-------------|--------|
| `interactive` | Click goals in RViz, drone navigates around obstacles | RViz-only |
| `static` | Pre-defined static forest | Gazebo |
| `dynamic` | Known dynamic obstacles | RViz-only |
| `unknown_dynamic` | Unknown obstacles detected via pointcloud | Gazebo |

**Docker:**
```bash
make run-interactive                           # click goals in RViz
make run-demo SCENARIO=static_easy             # auto-goal demo
make run-demo SCENARIO=dynamic_hard            # 200 dynamic obstacles
make run-demo SCENARIO=unknown_dynamic_medium  # perception-in-the-loop
```

**Native:**
```bash
python3 src/sando/scripts/run_sim.py -m interactive -s install/setup.bash
python3 src/sando/scripts/run_sim.py -m static -d easy -s install/setup.bash
python3 src/sando/scripts/run_sim.py -m dynamic -d hard -s install/setup.bash
python3 src/sando/scripts/run_sim.py -m unknown_dynamic -d medium -s install/setup.bash
```

## Setup

### Docker (Recommended)

1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/)
2. Clone and build:
   ```bash
   git clone --recursive <ANONYMIZED_REPO_URL>
   cd sando/docker
   make build
   ```
3. Run any simulation mode above

<details>
<summary><b>Docker tips</b></summary>

```bash
make shell                         # debug shell inside container
make run-demo SCENARIO=static_easy GPU=false  # without GPU

# Convenience aliases for all scenarios
make run-static-easy
make run-static-medium
make run-static-hard
make run-dynamic-easy
make run-dynamic-medium
make run-dynamic-hard
make run-unknown-easy
make run-unknown-medium
make run-unknown-hard

# Cleanup
docker builder prune               # remove build caches
docker rm $(docker ps -a -q)       # remove all containers
docker rmi $(docker images -q)     # remove all images
```

</details>

### Native Installation

1. Clone and run setup:
   ```bash
   mkdir -p ~/code/sando_ws/src && cd ~/code/sando_ws/src
   git clone --recursive <ANONYMIZED_REPO_URL>
   cd sando && ./setup.sh
   ```
2. Source and run:
   ```bash
   cd ~/code/sando_ws
   source install/setup.bash
   python3 src/sando/scripts/run_sim.py -m interactive -s install/setup.bash
   ```

### System Requirements

- **OS:** Ubuntu 22.04 (Docker works on any Linux/macOS)
- **ROS:** ROS 2 Humble
- **Optimizer:** [Gurobi](https://www.gurobi.com/) (free academic license available)
- **Hardware:** 4+ CPU cores recommended; GPU optional (Gazebo rendering only)

## Configuration

All planner parameters are in `config/sando.yaml`, organized into three tiers:

| Tier | Description | Examples |
|------|-------------|---------|
| **`[CONFIGURE]`** | Must set for your vehicle/environment | `v_max`, `a_max`, `drone_bbox`, `z_min`/`z_max`, `num_P`/`num_N` |
| **`[TUNE]`** | Adjust for performance trade-offs | `inflation_hgp`, `sfc_size`, `goal_seen_radius`, `heat_alpha0/1` |
| **`[INTERNAL]`** | Safe defaults, change only if needed | Algorithm internals, debug flags |

Hardware-specific config: `config/sando_hw_quadrotor.yaml` (with inline comments explaining differences from simulation defaults).

<details>
<summary><b>Key parameters to tune</b></summary>

| Parameter | Default | Effect |
|-----------|---------|--------|
| `v_max` / `a_max` / `j_max` | 5.0 / 20.0 / 100.0 | Vehicle dynamic limits |
| `num_P` / `num_N` | 3 / 5 | More polytopes/segments = smoother but slower |
| `sfc_size` | [3.0, 3.0, 3.0] | Local planning window size (smaller = faster) |
| `inflation_hgp` | 0.45 | Static obstacle buffer for global planning [m] |
| `dynamic_factor_initial_mean` | 1.5 | Starting time-allocation factor (higher = more conservative) |
| `goal_seen_radius` | 2.0 | Distance to stop replanning (too small = very hard to plan) |
| `heat_alpha0` / `heat_alpha1` | 0.2 / 1.0 | Dynamic obstacle avoidance weights |

</details>

<details>
<summary><b>Benchmarking</b></summary>

### Local Trajectory Benchmarking

```bash
# 1. Generate safety corridors (once)
source install/setup.bash
tmuxp load src/sando/launch/generate_sfc.yaml

# 2. Run benchmarks
cd src/sando/benchmarking
python3 run_benchmark_suite.py                # standard benchmarks
python3 run_benchmark_suite.py --ve-comparison  # variable elimination comparison

# 3. Generate LaTeX tables
python3 generate_latex_table.py --output tables/benchmark.tex
```

### Simulation Benchmarking (Dynamic & Static)

```bash
# Dynamic obstacle benchmark
python3 src/sando/scripts/run_benchmark.py \
  -s install/setup.bash --mode rviz-only \
  --cases easy medium hard --config-name dynamic --num-trials 10

# Static forest benchmark
python3 src/sando/scripts/run_benchmark.py \
  -s install/setup.bash --mode gazebo \
  --cases easy medium hard --config-name static --num-trials 10

# Analyze results
python3 src/sando/scripts/analyze_dynamic_benchmark.py \
  --data-dir src/sando/benchmark_data/dynamic --all-cases \
  --latex-dir /path/to/tables
```

Results are saved to `benchmark_data/` with per-trial CSV, JSON, and ROS bag recordings.

</details>

<details>
<summary><b>Hover avoidance testing</b></summary>

SANDO includes hover avoidance that detects nearby dynamic obstacles when the drone is hovering and autonomously evades them.

```bash
# Hover test — trefoil obstacles orbit near a hovering drone
python3 src/sando/scripts/run_sim.py --mode hover-test -s install/setup.bash

# Adversarial test — chaser drone pursues an evader drone
python3 src/sando/scripts/run_sim.py --mode adversarial-test -s install/setup.bash
```

| Parameter | Description | Default |
|-----------|-------------|---------|
| `hover_avoidance_enabled` | Enable/disable hover avoidance | `false` |
| `hover_avoidance_d_trigger` | Danger radius [m] | `4.0` |
| `hover_avoidance_h` | Evasion distance [m] | `3.0` |

Both modes support `--dry-run` to inspect the generated tmuxp YAML without launching.

</details>

<details>
<summary><b>Architecture overview</b></summary>

### Planning Pipeline

1. **Sensor Input** — Point cloud subscriptions update the voxel grid
2. **HGP Manager** (`include/hgp/`) — Heat map-based global planner: voxel map, A* with heat costs, convex decomposition into safety corridors
3. **SANDO Core** (`include/sando/sando.hpp`) — Planning orchestrator with state machine (YAWING → TRAVELING → GOAL_SEEN → GOAL_REACHED)
4. **Gurobi Solver** (`include/sando/gurobi_solver.hpp`) — Local trajectory optimization using Hermite spline parameterization with dynamic factor adaptation
5. **SANDO Node** (`src/sando/sando_node.cpp`) — ROS 2 node wrapper

### Key Innovations

- **Spatiotemporal safe flight corridors** — convex decomposition that accounts for obstacle motion over time
- **Variable elimination** — reduces QP solve time by analytically eliminating dependent variables
- **Dynamic factor adaptation** — automatically adjusts time-scaling for reliable convergence
- **Dynamic heat maps** — soft-cost global planning that steers away from predicted obstacle trajectories

</details>

## License

BSD 3-Clause License. See [LICENSE](LICENSE) for details.
