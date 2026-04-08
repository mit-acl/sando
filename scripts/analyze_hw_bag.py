#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
Analyze hardware ROS2 bag data from SANDO flights.

Produces:
  1. Computation time statistics (avg +/- std) printed as a table and saved as CSV
  2. History plot of position, velocity, acceleration, jerk (PDF)
  3. (Optional) LaTeX table summarizing multiple test runs

Usage:
  # Single bag:
  python3 analyze_hw_bag.py /path/to/bag_folder

  # All bags in a directory:
  python3 analyze_hw_bag.py /path/to/parent_folder

  # Custom velocity/acceleration/jerk limits:
  python3 analyze_hw_bag.py /path/to/bag_folder --v_max 5.0 --a_max 20.0 --j_max 100.0

  # Generate LaTeX table for hw static tests:
  python3 analyze_hw_bag.py /path/to/hw/static --generate_table --table_output /path/to/tables/hw_static.table
"""

import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------


def is_ros2_bag(path):
    """Check if a directory looks like a ROS2 bag (contains metadata.yaml)."""
    return os.path.isdir(path) and os.path.exists(os.path.join(path, "metadata.yaml"))


def read_bag_topics(bag_path, topic_type_pairs, storage_id="sqlite3"):
    """Read specified topics from a bag. Returns {topic: [msgs]}."""
    type_map = {t: get_message(mt) for t, mt in topic_type_pairs}
    topic_names = [t for t, _ in topic_type_pairs]

    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    try:
        reader.set_filter(StorageFilter(topics=topic_names))
    except Exception:
        pass

    out = {t: [] for t in topic_names}
    while reader.has_next():
        topic, serialized_msg, _ = reader.read_next()
        if topic in type_map:
            msg = deserialize_message(serialized_msg, type_map[topic])
            out[topic].append(msg)
    return out


def discover_namespace(bag_path, storage_id="sqlite3"):
    """Discover the robot namespace from bag topics (e.g., /PX03)."""
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topics = reader.get_all_topics_and_types()
    for t in topics:
        if t.name.endswith("/goal"):
            # e.g., /PX03/goal -> /PX03
            return t.name.rsplit("/goal", 1)[0]
        if t.name.endswith("/computation_times"):
            return t.name.rsplit("/computation_times", 1)[0]
    # fallback: find any namespaced topic
    for t in topics:
        parts = t.name.strip("/").split("/")
        if len(parts) >= 2:
            return "/" + parts[0]
    return ""


# ---------------------------------------------------------------------------
# Computation time statistics
# ---------------------------------------------------------------------------


def compute_time_stats(comp_msgs):
    """Compute avg and std for computation time fields."""
    fields = [
        ("total_replanning_ms", "Total Replanning [ms]"),
        ("global_planning_ms", "Global Planning [ms]"),
        ("cvx_decomp_ms", "Safety Corridor [ms]"),
        ("local_traj_ms", "Local Traj. [ms]"),
    ]

    data = {display: [] for _, display in fields}
    for msg in comp_msgs:
        # Skip zero entries (before flight starts or after goal reached)
        if msg.total_replanning_ms == 0.0:
            continue
        # Only include successful replanning results
        if not msg.result:
            continue
        for attr, display in fields:
            data[display].append(getattr(msg, attr))

    stats = {}
    for _, display in fields:
        arr = np.array(data[display])
        if len(arr) > 0:
            stats[display] = {"avg": np.mean(arr), "std": np.std(arr)}
        else:
            stats[display] = {"avg": 0.0, "std": 0.0}
    return stats


def print_comp_stats(stats, bag_name):
    """Pretty-print computation time statistics."""
    print(f"\n{'=' * 60}")
    print(f"Computation Times: {bag_name}")
    print(f"{'=' * 60}")
    print(f"{'Metric':<30} {'Avg':>10} {'Std':>10}")
    print(f"{'-' * 50}")
    for name, s in stats.items():
        print(f"{name:<30} {s['avg']:>10.3f} {s['std']:>10.3f}")
    print()


def save_comp_stats_csv(stats, save_path):
    """Save computation time stats as CSV."""
    with open(save_path, "w") as f:
        f.write("Metric,Avg,Std\n")
        for name, s in stats.items():
            f.write(f"{name},{s['avg']:.4f},{s['std']:.4f}\n")
    print(f"Saved computation stats to {save_path}")


# ---------------------------------------------------------------------------
# Plotting (matches unc_benchmark_data_analysis.ipynb style)
# ---------------------------------------------------------------------------


def setup_plot_style(use_tex=False):
    """Configure matplotlib to match the notebook style."""
    if use_tex:
        plt.rcParams.update(
            {
                "text.usetex": True,
                "text.latex.preamble": (
                    r"\usepackage{amsmath}\usepackage{bm}"
                    r"\newcommand{\vect}[1]{\bm{#1}}"
                ),
            }
        )
    else:
        plt.rcParams.update({"text.usetex": False})
        candidates = [
            "Times New Roman",
            "Nimbus Roman",
            "TeX Gyre Termes",
            "Times",
            "CMU Serif",
            "DejaVu Serif",
        ]
        avail = {f.name for f in fm.fontManager.ttflist}
        chosen = next((c for c in candidates if c in avail), "DejaVu Serif")
        plt.rcParams["font.family"] = "serif"
        plt.rcParams["font.serif"] = [chosen]

    plt.rcParams["font.size"] = 18


def vect_label(sym, use_tex=False):
    return rf"$\vect{{{sym}}}$" if use_tex else rf"$\mathbf{{{sym}}}$"


def plot_history(
    t,
    p,
    v,
    a,
    j,
    save_path,
    v_max=5.0,
    a_max=20.0,
    j_max=100.0,
    use_tex=False,
    tol_abs=0.001,
    p_ylim=None,
    v_ylim=None,
    a_ylim=None,
    j_ylim=None,
):
    """
    Plot 4-row stacked history: position, velocity, acceleration, jerk.
    Style matches unc_benchmark_data_analysis.ipynb.

    Optional ylim overrides (tuples): p_ylim, v_ylim, a_ylim, j_ylim.
    If not provided, defaults are used.
    """
    setup_plot_style(use_tex)

    base_font = 26
    title_font = 28
    label_font = 26
    tick_font = 22
    legend_font = 28

    width = 10.0
    phi = (1 + 5**0.5) / 2
    height = (width / phi) * 1.3  # taller for 4 subplots

    fig, (ax_p, ax_v, ax_a, ax_j) = plt.subplots(
        4,
        1,
        sharex=True,
        figsize=(width, height),
        gridspec_kw=dict(hspace=0.35),
    )

    right_margin = 0.9

    def plot_xyz(ax, y, lim, ylabel, title, ylim_range=None):
        lx = ax.plot(t, y[:, 0], lw=2.2, label="x")[0]
        ly = ax.plot(t, y[:, 1], lw=2.2, label="y")[0]
        lz = ax.plot(t, y[:, 2], lw=2.2, label="z")[0]
        lines = [lx, ly, lz]

        lim_handle = None
        if lim is not None:
            lim = float(lim)
            lim_eff = lim + float(tol_abs)
            lim_handle = ax.axhline(+lim, ls=":", lw=1.8, color="k", label="limit")
            ax.axhline(-lim, ls=":", lw=1.8, color="k", label="_nolegend_")

        if ylim_range is not None:
            ax.set_ylim(ylim_range[0], ylim_range[1])
        elif lim is not None:
            yabs = np.max(np.abs(y)) if y.size else 1.0
            ymax = max(1.15 * yabs, 1.15 * float(lim))
            ax.set_ylim(-ymax, ymax)

        # Place y-ticks at the limit values (and 0) so readers see the bounds
        if lim is not None:
            ax.set_yticks([-lim, 0, lim])

        ax.set_ylabel(ylabel, fontsize=label_font)
        ax.set_title(title, fontsize=title_font)
        ax.grid(True, ls="--", alpha=0.35)
        ax.tick_params(axis="both", labelsize=tick_font)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        return lines, lim_handle

    # Position (no limits)
    p_lines, _ = plot_xyz(
        ax_p,
        p,
        None,
        rf"{vect_label('p', use_tex)} [m]",
        "Position",
        ylim_range=p_ylim or (-5, 17),
    )
    # Velocity
    v_lines, _ = plot_xyz(
        ax_v,
        v,
        v_max,
        rf"{vect_label('v', use_tex)} [m/s]",
        "Velocity",
        ylim_range=v_ylim or (-v_max * 1.1, v_max * 1.1),
    )
    # Acceleration
    a_lines, _ = plot_xyz(
        ax_a,
        a,
        a_max,
        rf"{vect_label('a', use_tex)} [m/s$^2$]",
        "Acceleration",
        ylim_range=a_ylim or (-a_max * 1.1, a_max * 1.1),
    )
    # Jerk
    j_lines, j_lim = plot_xyz(
        ax_j,
        j,
        j_max,
        rf"{vect_label('j', use_tex)} [m/s$^3$]",
        "Jerk",
        ylim_range=j_ylim or (-j_max * 1.1, j_max * 1.1),
    )

    ax_j.set_xlabel("Time [s]", fontsize=label_font)
    ax_j.tick_params(axis="x", labelsize=tick_font)

    # Legend
    handles = [j_lines[0], j_lines[1], j_lines[2]]
    labels = ["x", "y", "z"]
    if j_lim is not None:
        handles.append(j_lim)
        labels.append("limit")

    plt.tight_layout(rect=[0.0, 0.12, 1.0, 1.0])
    fig.legend(
        handles,
        labels,
        loc="lower center",
        bbox_to_anchor=(0.5, -0.08),
        frameon=False,
        fontsize=legend_font,
        ncol=len(labels),
        borderaxespad=0.0,
        handlelength=2.6,
        columnspacing=2.0,
    )

    os.makedirs(
        os.path.dirname(save_path) if os.path.dirname(save_path) else ".", exist_ok=True
    )
    fig.savefig(save_path, bbox_inches="tight", dpi=300)
    plt.close(fig)
    print(f"Saved history plot to {save_path}")


# ---------------------------------------------------------------------------
# Processing a single bag
# ---------------------------------------------------------------------------


def process_bag(bag_path, args):
    """Process a single ROS2 bag: compute stats and generate plot."""
    bag_name = os.path.basename(os.path.normpath(bag_path))
    print(f"\nProcessing bag: {bag_path}")

    ns = discover_namespace(bag_path)
    print(f"  Detected namespace: {ns}")

    goal_topic = f"{ns}/goal"
    comp_topic = f"{ns}/computation_times"

    topic_type_pairs = [
        (goal_topic, "dynus_interfaces/msg/Goal"),
        (comp_topic, "dynus_interfaces/msg/ComputationTimes"),
    ]

    data = read_bag_topics(bag_path, topic_type_pairs)

    # --- Computation time stats ---
    comp_msgs = data[comp_topic]
    if comp_msgs:
        stats = compute_time_stats(comp_msgs)
        print_comp_stats(stats, bag_name)
        csv_path = os.path.join(bag_path, f"comp_stats_{bag_name}.csv")
        save_comp_stats_csv(stats, csv_path)
    else:
        print("  No computation_times messages found.")

    # --- History plot ---
    goal_msgs = data[goal_topic]
    if not goal_msgs:
        print("  No goal messages found. Skipping plot.")
        return

    # Extract arrays
    t_abs, p, v, a, j = [], [], [], [], []
    for m in goal_msgs:
        tt = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t_abs.append(tt)
        p.append([m.p.x, m.p.y, m.p.z])
        v.append([m.v.x, m.v.y, m.v.z])
        a.append([m.a.x, m.a.y, m.a.z])
        j.append([m.j.x, m.j.y, m.j.z])

    t_abs = np.array(t_abs)
    p = np.array(p)
    v = np.array(v)
    a = np.array(a)
    j = np.array(j)

    # Shift time to start at 0
    t = t_abs - t_abs[0]

    save_path = os.path.join(bag_path, f"history_{bag_name}.pdf")
    plot_history(
        t,
        p,
        v,
        a,
        j,
        save_path,
        v_max=args.v_max,
        a_max=args.a_max,
        j_max=args.j_max,
        use_tex=args.use_tex,
    )


# ---------------------------------------------------------------------------
# LaTeX table generation for hw tests
# ---------------------------------------------------------------------------

# Static test configurations: test_name -> (v_max, a_max, j_max)
HW_STATIC_TESTS = {
    "test0": (1.0, 5.0, 10.0),
    "test1": (2.0, 5.0, 10.0),
    "test2": (3.0, 5.0, 10.0),
    "test3": (4.0, 5.0, 10.0),
    "test4": (5.0, 5.0, 10.0),
    "test5": (6.0, 5.0, 10.0),
}

# Dynamic test configurations: test_name -> (description,)
# All dynamic tests use v_max=2.0 m/s
HW_DYNAMIC_TESTS = [
    ("test1", "1 obst, line"),
    ("test1-take3", "1 obst, circle (take 3)"),
    ("test1-take4", "1 obst, circle (take 4)"),
    ("test2", "1 obst, figure 8"),
    ("test4", "5 obsts, line-ish (run 1)"),
    ("test5", "5 obsts, line-ish (run 2)"),
    ("test6", "5 obsts, line-ish (run 3)"),
]

# Dynamic round2 test configurations: test_name -> (obst_type, num_obst, obst_traj, v_max, a_max, j_max)
HW_DYNAMIC_ROUND2_TESTS = [
    # Exp 7-10: single obstacle, different trajectories
    ("test28", "1 Dyn.", "Line", 2.0, 5.0, 7.5),
    ("test29", "1 Dyn.", "Circle", 2.0, 5.0, 7.5),
    ("test30", "1 Dyn.", "Fig. Eight", 2.0, 5.0, 7.5),
    ("test31", "1 Dyn.", "Person", 2.0, 5.0, 7.5),
    # Exp 11-13: five dynamic obstacles
    ("test14", "5 Dyn.", "Line", 2.0, 5.0, 7.5),
    ("test15", "5 Dyn.", "Line", 2.0, 5.0, 7.5),
    ("test16", "5 Dyn.", "Line", 2.0, 5.0, 7.5),
    # Exp 14-16: dynamic + static, varying v_max
    ("test18", r"\makecell{5 Dyn. \\ \& Static}", "Line", 2.0, 5.0, 7.5),
    ("test23", r"\makecell{5 Dyn. \\ \& Static}", "Line", 3.0, 5.0, 7.5),
    ("test24", r"\makecell{5 Dyn. \\ \& Static}", "Line", 4.0, 5.0, 7.5),
]


def find_bag_in_test_dir(test_dir):
    """Find the single bag folder inside a test directory."""
    for d in sorted(os.listdir(test_dir)):
        candidate = os.path.join(test_dir, d)
        if is_ros2_bag(candidate):
            return candidate
    return None


def load_comp_stats_from_csv(csv_path):
    """Load computation time stats from a previously saved CSV."""
    stats = {}
    with open(csv_path, "r") as f:
        next(f)  # skip header
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 3:
                stats[parts[0]] = {"avg": float(parts[1]), "std": float(parts[2])}
    return stats


def compute_comp_stats_from_bag(bag_path, ns):
    """Read computation_times from bag and compute stats."""
    comp_topic = f"{ns}/computation_times"
    topic_type_pairs = [(comp_topic, "dynus_interfaces/msg/ComputationTimes")]
    data = read_bag_topics(bag_path, topic_type_pairs)
    comp_msgs = data[comp_topic]
    if comp_msgs:
        return compute_time_stats(comp_msgs)
    return None


def get_comp_stats(bag_path, ns):
    """Get computation stats by reading from the bag (applies result filter)."""
    return compute_comp_stats_from_bag(bag_path, ns)


def format_comp_cells(comp_stats):
    """Format computation time stats into table cells."""
    comp_keys = [
        "Total Replanning [ms]",
        "Global Planning [ms]",
        "Safety Corridor [ms]",
        "Local Traj. [ms]",
    ]
    cells = []
    for key in comp_keys:
        if comp_stats and key in comp_stats:
            avg = comp_stats[key]["avg"]
            std = comp_stats[key]["std"]
            cells.append(f"${avg:.1f} \\pm {std:.1f}$")
        else:
            cells.append("--")
    return cells


def build_comp_time_table(rows, row_header_col, caption, label, row_header_unit=None):
    """Build a LaTeX table with computation time columns.

    rows: list of dicts with 'row_label' (str) and 'comp_stats' (dict).
    row_header_col: LaTeX header for the first column.
    row_header_unit: optional unit string to show on a second line (e.g. '[m/s]').
    """
    lines = []
    lines.append(r"\begin{table}")
    lines.append(f"  \\caption{{{caption}}}")
    lines.append(f"  \\label{{{label}}}")
    lines.append(r"  \centering")
    lines.append(r"  \renewcommand{\arraystretch}{1.2}")
    lines.append(r"  \resizebox{\columnwidth}{!}{")
    lines.append(r"    \begin{tabular}{c c c c c c}")
    lines.append(r"      \toprule")
    lines.append(
        r"      \multirow{2}{*}[-0.4ex]{\textbf{Exp.}}"
        r" & \multirow{2}{*}[-0.4ex]{"
        + (
            rf"\makecell{{{row_header_col} \\ {{{row_header_unit}}}}}"
            if row_header_unit
            else row_header_col
        )
        + r"}"
        r" & \multicolumn{4}{c}{\textbf{Computation Time}}"
        r" \\"
    )
    lines.append(r"      \cmidrule(lr){3-6}")
    lines.append(
        r"      & "
        r"& $T_{\mathrm{replan}}$ [ms]"
        r" & $T_{\mathrm{global}}$ [ms]"
        r" & $T_{\mathrm{SSFC}}$ [ms]"
        r" & $T_{\mathrm{opt}}$ [ms]"
        r" \\"
    )
    lines.append(r"      \midrule")

    for i, row in enumerate(rows, start=1):
        cells = format_comp_cells(row["comp_stats"])
        cell_str = " & ".join(cells)
        lines.append(f"      {i} & {row['row_label']} & {cell_str} \\\\")

    lines.append(r"      \bottomrule")
    lines.append(r"    \end{tabular}")
    lines.append(r"  }")
    lines.append(r"  \vspace{-1.0em}")
    lines.append(r"\end{table}")
    return "\n".join(lines) + "\n"


def save_table(table_str, table_output):
    """Write LaTeX table string to file."""
    os.makedirs(os.path.dirname(table_output), exist_ok=True)
    with open(table_output, "w") as f:
        f.write(table_str)
    basename = os.path.basename(table_output)
    print(f"\nSaved LaTeX table to {table_output}")
    print(f"Include in paper with: \\input{{tables/{basename}}}")


def extract_goal_arrays(bag_path, ns):
    """Extract time, position, velocity, acceleration, jerk arrays from goal messages."""
    goal_topic = f"{ns}/goal"
    topic_type_pairs = [(goal_topic, "dynus_interfaces/msg/Goal")]
    data = read_bag_topics(bag_path, topic_type_pairs)
    goal_msgs = data[goal_topic]

    if not goal_msgs:
        return None

    t_abs = np.array(
        [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in goal_msgs]
    )
    p = np.array([[m.p.x, m.p.y, m.p.z] for m in goal_msgs])
    v = np.array([[m.v.x, m.v.y, m.v.z] for m in goal_msgs])
    a = np.array([[m.a.x, m.a.y, m.a.z] for m in goal_msgs])
    j = np.array([[m.j.x, m.j.y, m.j.z] for m in goal_msgs])
    t = t_abs - t_abs[0]

    return t, p, v, a, j


def generate_hw_static_table(static_dir, table_output):
    """Generate a LaTeX table and history plots for hw static tests (test0-test5)."""
    rows = []

    for test_name in sorted(HW_STATIC_TESTS.keys()):
        v_max, a_max, j_max = HW_STATIC_TESTS[test_name]
        test_dir = os.path.join(static_dir, test_name)
        if not os.path.isdir(test_dir):
            print(f"  WARNING: {test_dir} not found, skipping")
            continue

        bag_path = find_bag_in_test_dir(test_dir)
        if bag_path is None:
            print(f"  WARNING: No bag found in {test_dir}, skipping")
            continue

        print(f"  Processing {test_name} (v_max={v_max}) -> {bag_path}")

        ns = discover_namespace(bag_path)
        comp_stats = get_comp_stats(bag_path, ns)

        rows.append(
            {
                "row_label": f"{v_max:.1f}",
                "comp_stats": comp_stats,
            }
        )

        # Generate history plot
        goal_data = extract_goal_arrays(bag_path, ns)
        if goal_data is not None:
            t, p, v_arr, a_arr, j_arr = goal_data
            plot_path = os.path.join(bag_path, f"history_{test_name}.pdf")
            plot_history(
                t,
                p,
                v_arr,
                a_arr,
                j_arr,
                plot_path,
                v_max=v_max,
                a_max=a_max,
                j_max=j_max,
                p_ylim=(0, 20),
                v_ylim=(-v_max * 1.1, v_max * 1.1),
                a_ylim=(-a_max * 1.1, a_max * 1.1),
                j_ylim=(-j_max * 1.1, j_max * 1.1),
            )
        else:
            print(f"    No goal messages found for {test_name}, skipping plot.")

    if not rows:
        print("No data collected. Cannot generate table.")
        return

    caption = (
        "Hardware flight computation times in static environments with increasing velocity limits. "
        "All flights use $a_{\\max}=5$ m/s$^2$ and $j_{\\max}=10$ m/s$^3$."
    )
    table_str = build_comp_time_table(
        rows,
        row_header_col=r"$v_{\max}$",
        row_header_unit=r"[m/s]",
        caption=caption,
        label="tab:hw_static",
    )
    save_table(table_str, table_output)


def generate_hw_dynamic_table(dynamic_dir, table_output):
    """Generate a LaTeX table for hw dynamic tests."""
    rows = []

    for test_name, description in HW_DYNAMIC_TESTS:
        test_dir = os.path.join(dynamic_dir, test_name)
        if not os.path.isdir(test_dir):
            print(f"  WARNING: {test_dir} not found, skipping")
            continue

        bag_path = find_bag_in_test_dir(test_dir)
        if bag_path is None:
            print(f"  WARNING: No bag found in {test_dir}, skipping")
            continue

        print(f"  Processing {test_name} ({description}) -> {bag_path}")

        ns = discover_namespace(bag_path)
        comp_stats = get_comp_stats(bag_path, ns)

        rows.append(
            {
                "row_label": description,
                "comp_stats": comp_stats,
            }
        )

    if not rows:
        print("No data collected. Cannot generate table.")
        return

    caption = (
        "Hardware flight computation times in dynamic environments. "
        "All flights use $v_{\\max}=2$ m/s, $a_{\\max}=5$ m/s$^2$, and $j_{\\max}=10$ m/s$^3$."
    )
    table_str = build_comp_time_table(
        rows,
        row_header_col=r"\textbf{Scenario}",
        caption=caption,
        label="tab:hw_dynamic",
    )
    save_table(table_str, table_output)


def generate_hw_dynamic_static_table(dynamic_static_dir, table_output):
    """Generate a LaTeX table for hw dynamic+static tests (test7 only)."""
    test_name = "test7"
    test_dir = os.path.join(dynamic_static_dir, test_name)
    if not os.path.isdir(test_dir):
        print(f"  WARNING: {test_dir} not found")
        return

    bag_path = find_bag_in_test_dir(test_dir)
    if bag_path is None:
        print(f"  WARNING: No bag found in {test_dir}")
        return

    print(f"  Processing {test_name} -> {bag_path}")

    ns = discover_namespace(bag_path)
    comp_stats = get_comp_stats(bag_path, ns)

    rows = [
        {
            "row_label": "5 obsts + static",
            "comp_stats": comp_stats,
        }
    ]

    caption = (
        "Hardware flight computation times in combined dynamic and static environments. "
        "All flights use $v_{\\max}=2$ m/s, $a_{\\max}=5$ m/s$^2$, and $j_{\\max}=10$ m/s$^3$."
    )
    table_str = build_comp_time_table(
        rows,
        row_header_col=r"\textbf{Scenario}",
        caption=caption,
        label="tab:hw_dynamic_static",
    )
    save_table(table_str, table_output)


def build_dynamic_round2_table(rows, caption, label):
    """Build a LaTeX table with multirow grouping for repeated values."""
    lines = []
    lines.append(r"\begin{table}")
    lines.append(f"  \\caption{{{caption}}}")
    lines.append(f"  \\label{{{label}}}")
    lines.append(r"  \centering")
    lines.append(r"  \renewcommand{\arraystretch}{1.2}")
    lines.append(r"  \resizebox{\columnwidth}{!}{")
    lines.append(r"    \begin{tabular}{c c c c c c c c}")
    lines.append(r"      \toprule")
    lines.append(
        r"      \multirow{2}{*}[-0.4ex]{\textbf{Exp.}}"
        r" & \multirow{2}{*}[-0.4ex]{\makecell{\textbf{Obst.} \\ \textbf{Type}}}"
        r" & \multirow{2}{*}[-0.4ex]{\makecell{\textbf{Obst.} \\ \textbf{Traj.}}}"
        r" & \multirow{2}{*}[-0.4ex]{\makecell{$v_{\max}$ \\ {[m/s]}}}"
        r" & \multicolumn{4}{c}{\textbf{Computation Time [ms]}}"
        r" \\"
    )
    lines.append(r"      \cmidrule(lr){5-8}")
    lines.append(
        r"      & & & "
        r"& $T_{\mathrm{replan}}$"
        r" & $T_{\mathrm{global}}$"
        r" & $T_{\mathrm{STSFC}}$"
        r" & $T_{\mathrm{opt}}$"
        r" \\"
    )
    lines.append(r"      \midrule")

    # Split rows into groups separated by midrules
    midrule_after = {10, 13}
    groups = []
    current_group = []
    for i, row in enumerate(rows, start=7):
        current_group.append((i, row))
        if i in midrule_after or i == len(rows) + 6:
            groups.append(current_group)
            current_group = []
    if current_group:
        groups.append(current_group)

    for gi, group in enumerate(groups):
        n = len(group)
        for j, (exp_num, row) in enumerate(group):
            cells = format_comp_cells(row["comp_stats"])
            cell_str = " & ".join(cells)

            # Determine if this column should use multirow (first in group)
            # or be empty (subsequent in group with same value)
            obst_type_vals = [r["obst_type"] for _, r in group]
            obst_traj_vals = [r["obst_traj"] for _, r in group]
            v_max_vals = [r["v_max"] for _, r in group]

            if j == 0:
                # First row: use multirow if all values in group are the same
                if all(v == obst_type_vals[0] for v in obst_type_vals):
                    obst_type_str = f"\\multirow{{{n}}}{{*}}{{{row['obst_type']}}}"
                else:
                    obst_type_str = row["obst_type"]

                if all(v == obst_traj_vals[0] for v in obst_traj_vals):
                    obst_traj_str = f"\\multirow{{{n}}}{{*}}{{{row['obst_traj']}}}"
                else:
                    obst_traj_str = row["obst_traj"]

                if all(v == v_max_vals[0] for v in v_max_vals):
                    v_max_str = f"\\multirow{{{n}}}{{*}}{{{row['v_max']:.1f}}}"
                else:
                    v_max_str = f"{row['v_max']:.1f}"
            else:
                # Subsequent rows: empty if multirow, otherwise show value
                obst_type_str = (
                    ""
                    if all(v == obst_type_vals[0] for v in obst_type_vals)
                    else row["obst_type"]
                )
                obst_traj_str = (
                    ""
                    if all(v == obst_traj_vals[0] for v in obst_traj_vals)
                    else row["obst_traj"]
                )
                v_max_str = (
                    ""
                    if all(v == v_max_vals[0] for v in v_max_vals)
                    else f"{row['v_max']:.1f}"
                )

            lines.append(
                f"      {exp_num} & {obst_type_str} & {obst_traj_str} & {v_max_str} & {cell_str} \\\\"
            )

        if gi < len(groups) - 1:
            lines.append(r"      \midrule")

    lines.append(r"      \bottomrule")
    lines.append(r"    \end{tabular}")
    lines.append(r"  }")
    lines.append(r"  \vspace{-1.0em}")
    lines.append(r"\end{table}")
    return "\n".join(lines) + "\n"


def generate_hw_dynamic_round2(dynamic_dir, table_output):
    """Generate a LaTeX table and history plots for hw dynamic round2 tests."""
    rows = []

    for test_name, obst_type, obst_traj, v_max, a_max, j_max in HW_DYNAMIC_ROUND2_TESTS:
        test_dir = os.path.join(dynamic_dir, test_name)
        if not os.path.isdir(test_dir):
            print(f"  WARNING: {test_dir} not found, skipping")
            continue

        bag_path = find_bag_in_test_dir(test_dir)
        if bag_path is None:
            print(f"  WARNING: No bag found in {test_dir}, skipping")
            continue

        print(f"  Processing {test_name} ({obst_type}, v_max={v_max}) -> {bag_path}")

        ns = discover_namespace(bag_path)
        comp_stats = get_comp_stats(bag_path, ns)

        rows.append(
            {
                "obst_type": obst_type,
                "obst_traj": obst_traj,
                "v_max": v_max,
                "comp_stats": comp_stats,
            }
        )

        # Generate history plot
        goal_data = extract_goal_arrays(bag_path, ns)
        if goal_data is not None:
            t, p, v_arr, a_arr, j_arr = goal_data
            plot_path = os.path.join(bag_path, f"history_{test_name}.pdf")
            # Adjust position y-axis based on goal position
            if test_name in ("test28", "test29", "test30", "test31"):
                p_ylim_val = (-2, 12)
            else:
                p_ylim_val = (0, 20)
            plot_history(
                t,
                p,
                v_arr,
                a_arr,
                j_arr,
                plot_path,
                v_max=v_max,
                a_max=a_max,
                j_max=j_max,
                p_ylim=p_ylim_val,
                v_ylim=(-v_max * 1.1, v_max * 1.1),
                a_ylim=(-a_max * 1.1, a_max * 1.1),
                j_ylim=(-j_max * 1.1, j_max * 1.1),
            )
        else:
            print(f"    No goal messages found for {test_name}, skipping plot.")

    if not rows:
        print("No data collected. Cannot generate table.")
        return

    caption = (
        "Hardware flight computation times in dynamic environments. "
        "All flights use $a_{\\max}=5$ m/s$^2$ and $j_{\\max}=10$ m/s$^3$."
    )
    table_str = build_dynamic_round2_table(
        rows,
        caption=caption,
        label="tab:hw_dynamic_round2",
    )
    save_table(table_str, table_output)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Analyze SANDO hardware ROS2 bag data."
    )
    parser.add_argument(
        "path", help="Path to a bag folder or parent directory containing bags"
    )
    parser.add_argument("--v_max", type=float, default=5.0, help="Velocity limit [m/s]")
    parser.add_argument(
        "--a_max", type=float, default=20.0, help="Acceleration limit [m/s^2]"
    )
    parser.add_argument("--j_max", type=float, default=100.0, help="Jerk limit [m/s^3]")
    parser.add_argument(
        "--use_tex", action="store_true", help="Use LaTeX for text rendering"
    )
    parser.add_argument(
        "--generate_table",
        type=str,
        choices=["static", "dynamic", "dynamic_static", "dynamic_round2"],
        help="Generate LaTeX table: 'static', 'dynamic', 'dynamic_static', or 'dynamic_round2'",
    )
    parser.add_argument(
        "--table_output",
        type=str,
        default=None,
        help="Output path for the LaTeX table (auto-set if not provided)",
    )
    parser.add_argument(
        "--tables_dir",
        type=str,
        default=None,
        help="Directory for LaTeX table output (required when using --generate_table without --table_output)",
    )
    args = parser.parse_args()

    path = os.path.abspath(args.path)

    if args.generate_table:
        tables_dir = args.tables_dir
        if not tables_dir and not args.table_output:
            parser.error("--tables_dir is required when using --generate_table without --table_output")
        if args.generate_table == "static":
            output = args.table_output or os.path.join(tables_dir, "hw_static.tex")
            generate_hw_static_table(path, output)
        elif args.generate_table == "dynamic":
            output = args.table_output or os.path.join(tables_dir, "hw_dynamic.tex")
            generate_hw_dynamic_table(path, output)
        elif args.generate_table == "dynamic_static":
            output = args.table_output or os.path.join(
                tables_dir, "hw_dynamic_static.tex"
            )
            generate_hw_dynamic_static_table(path, output)
        elif args.generate_table == "dynamic_round2":
            output = args.table_output or os.path.join(
                tables_dir, "hw_dynamic_round2.tex"
            )
            generate_hw_dynamic_round2(path, output)
        return

    if is_ros2_bag(path):
        # Single bag
        process_bag(path, args)
    elif os.path.isdir(path):
        # Directory of bags
        bags = sorted(
            [
                os.path.join(path, d)
                for d in os.listdir(path)
                if is_ros2_bag(os.path.join(path, d))
            ]
        )
        if not bags:
            print(f"No ROS2 bags found in {path}")
            sys.exit(1)
        print(f"Found {len(bags)} bag(s) in {path}")
        for bag in bags:
            process_bag(bag, args)
    else:
        print(f"Error: {path} is not a valid bag or directory")
        sys.exit(1)


if __name__ == "__main__":
    main()
