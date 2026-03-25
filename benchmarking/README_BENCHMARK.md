# SANDO Local Trajectory Benchmark Suite

This directory contains tools for running and analyzing local trajectory optimization benchmarks for the SANDO paper.

## Quick Start

### Step 1: Run Benchmarks

```bash
cd /home/kkondo/code/dynus_ws
python3 src/sando/benchmarking/run_benchmark_suite.py
```

This will automatically run:
- **SANDO2 multi-threaded** for N = 4, 5, 6
- **SANDO2 single-threaded** for N = 4, 5, 6
- **FASTER (original)** single-threaded for N = 6

Results are saved to:
- `src/sando/benchmark_data/single_thread/`
- `src/sando/benchmark_data/multi_thread/`

**Note:** Each benchmark configuration will run sequentially. The entire suite takes approximately 15-30 minutes depending on your system.

### Step 2: Generate LaTeX Table

After benchmarks complete:

```bash
python3 src/sando/benchmarking/generate_latex_table.py
```

This creates:
- LaTeX table at: `~/paper_writing/SANDO_v3/tables/standardized_benchmark.tex`
- Formatted with `\best{}` and `\worst{}` highlighting

### Step 3: Analyze Data (Optional)

For more detailed analysis and plots:

```bash
jupyter notebook src/sando/benchmarking/local_traj_benchmark.ipynb
```

The notebook will automatically load data from the CSV files.

## File Descriptions

### Scripts

- **`run_benchmark_suite.py`** - Main benchmark runner
  - Launches ROS2 benchmarks with different configurations
  - Handles parameter configuration automatically
  - Shows progress and saves results

- **`generate_latex_table.py`** - LaTeX table generator
  - Reads CSV benchmark data
  - Computes statistics (means, success rates, violations)
  - Generates formatted LaTeX table with best/worst highlighting

### Configuration Files

- **`../launch/local_traj_benchmark.launch.py`** - ROS2 launch file
  - Called automatically by `run_benchmark_suite.py`
  - Contains default parameters (don't need to edit manually)

### Data Files

Benchmark results are saved as CSV files in:
```
src/sando/benchmark_data/
├── single_thread/
│   ├── sando_4_benchmark.csv
│   ├── sando_5_benchmark.csv
│   ├── sando_6_benchmark.csv
│   └── faster_6_benchmark.csv
└── multi_thread/
    ├── sando_4_benchmark.csv
    ├── sando_5_benchmark.csv
    └── sando_6_benchmark.csv
```

Each CSV contains:
- `success` - Whether optimization succeeded (0/1)
- `per_opt_runtime_ms` - Per-optimization computation time
- `total_opt_runtime_ms` - Total computation time including threading overhead
- `total_traj_time_sec` - Travel time along trajectory
- `traj_length_m` - Path length
- `jerk_smoothness_l1` - Jerk smoothness metric
- `corridor_violated`, `v_violated`, `a_violated`, `j_violated` - Constraint violations

## Customization

### Running Custom Configurations

Edit `run_benchmark_suite.py` and modify the `BENCHMARK_CONFIGS` list:

```python
BENCHMARK_CONFIGS = [
    # (use_single_threaded, planner_names, num_N_list, description)
    (False, ["sando"], [4, 5, 6], "SANDO2 multi-threaded"),
    (True, ["sando"], [7, 8], "SANDO2 single-threaded (N=7,8)"),
    # Add more configurations...
]
```

### Modifying Table Format

Edit `generate_latex_table.py`:
- Change `columns_config` to add/remove columns
- Modify `OUTPUT_FILE` path
- Adjust formatting in `generate_latex_table()` function

## LaTeX Table Usage

In your paper, add these macros to your preamble:

```latex
% For best/worst highlighting
\usepackage{xcolor}
\newcommand{\best}[1]{\textcolor{green!70!black}{#1}}
\newcommand{\worst}[1]{\textcolor{red}{#1}}
```

Then include the table:

```latex
\input{tables/standardized_benchmark.tex}
```

## Troubleshooting

### "No data loaded" error
- Make sure you ran `run_benchmark_suite.py` first
- Check that CSV files exist in `src/sando/benchmark_data/`

### ROS2 launch fails
- Ensure workspace is built: `colcon build --packages-select sando`
- Source the workspace: `source install/setup.bash`

### Missing columns in CSV
- The benchmark node may have crashed
- Check ROS2 logs for errors
- Ensure Gurobi is properly configured

## Expected Results

For the paper table, you should see:
- **Success rates** around 77-100%
- **SANDO2 multi** having lowest `total_opt_runtime_ms` (parallelization benefit)
- **SANDO2 single** having competitive `per_opt_runtime_ms`
- **Zero violations** for SANDO (safe corridor)
- **Some velocity violations** for FASTER (original)

## Notes

- The benchmark runs in a simulation environment
- Results may vary slightly between runs due to threading
- For publication-quality results, run multiple times and average
- Make sure no other heavy processes are running during benchmarks
