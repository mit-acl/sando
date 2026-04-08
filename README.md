# SANDO: Safe Autonomous Trajectory Planning for Dynamic Unknown Environments

If you like this project, please consider starring ⭐ the repo!

**Submitted to the IEEE Transactions on Robotics (T-RO)**

<table>
<tr>
<td width="50%"><b>Spatiotemporal SFC</b><br><video src="https://github.com/user-attachments/assets/7279e010-bde3-4061-9a28-07a145092276" width="100%" autoplay loop muted playsinline controls></video></td>
<td width="50%"><b>Hardware Heatmap</b><br><video src="https://github.com/user-attachments/assets/f9bf40c3-18ec-498c-80f7-e7ff92dcb2a6" width="100%" autoplay loop muted playsinline controls></video></td>
</tr>
<tr>
<td width="50%"><b>Sim Interactive</b><br><video src="https://github.com/user-attachments/assets/6718410a-aa0d-4726-a6ca-605a22e25b2c" width="100%" autoplay loop muted playsinline controls></video></td>
<td width="50%"><b>Sim Static</b><br><video src="https://github.com/user-attachments/assets/61e676a6-4b03-480a-90b3-4dafb19d7198" width="100%" autoplay loop muted playsinline controls></video></td>
</tr>
<tr>
<td width="50%"><b>Sim Dynamic</b><br><video src="https://github.com/user-attachments/assets/f672ad26-3e06-4cbf-99d2-fd6b2eaeac1d" width="100%" autoplay loop muted playsinline controls></video></td>
<td width="50%"><b>Hardware Single Dynamic</b><br><video src="https://github.com/user-attachments/assets/3a6a9805-078e-4b12-bc5e-d3421edce9cf" width="100%" autoplay loop muted playsinline controls></video></td>
</tr>
<tr>
<td width="50%"><b>Hardware Multiple Dynamic</b><br><video src="https://github.com/user-attachments/assets/aac54514-d8fb-4aa7-8304-63ffc71c2bad" width="100%" autoplay loop muted playsinline controls></video></td>
<td width="50%"><b>Hardware Dynamic + Static</b><br><video src="https://github.com/user-attachments/assets/175d183b-60ea-40a3-92ae-1ece3cdf40e6" width="100%" autoplay loop muted playsinline controls></video></td>
</tr>
</table>

SANDO plans safe, dynamically-feasible trajectories for UAVs in environments with both static and dynamic obstacles, including previously unseen ones detected at runtime via onboard sensors.

**Full video:** [https://youtu.be/_T10DJiLQXg](https://youtu.be/_T10DJiLQXg)

**arXiv Paper:** Coming soon!

**ResearchGate Paper:** [https://www.researchgate.net/publication/403632573_SANDO_Safe_Autonomous_Trajectory_Planning_for_Dynamic_Unknown_Environments](https://www.researchgate.net/publication/403632573_SANDO_Safe_Autonomous_Trajectory_Planning_for_Dynamic_Unknown_Environments)

```bibtex
@article{kondo2026sando,
      title={SANDO: Safe Autonomous Trajectory Planning for Dynamic Unknown Environments},
      year={2026},
}
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

1. **Install [Docker](https://docs.docker.com/engine/install/ubuntu/).**

2. **Obtain a Gurobi WLS license.** Free [WLS Academic](https://portal.gurobi.com/iam/licenses/request) for academics. Other license types (named-user, compute server) do not work inside Docker.

3. **Clone the repo and place the license:**
   ```bash
   git clone --recursive https://github.com/mit-acl/sando.git
   cd sando/docker
   cp /path/to/your/gurobi.lic ./gurobi.lic   # required — change the path to your WLS license file
   ```

4. **Build the image** (~15 min first time):
   ```bash
   make build
   ```

5. **Run a simulation:**
   ```bash
   make run-interactive               # click goals in RViz (use "2D Nav Goal" or hit "g")
   make run-demo SCENARIO=static_easy # auto-goal demo
   ```

   See the [What You Can Do](#what-you-can-do) section above for the full list of modes.

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
   git clone --recursive https://github.com/mit-acl/sando.git
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
