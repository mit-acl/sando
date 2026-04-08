"""Shared constants: colors, positions, and corridor definitions for the SANDO2 animation."""

from manim import *
import numpy as np

# ── Colors ───────────────────────────────────────────────────────────────────
STATIC_OBS_COLOR = "#D4956A"
DYNAMIC_OBS_COLOR = "#F5A623"

CORRIDOR_COLORS = {
    0: "#FF00FF",  # magenta  (n=0)
    1: "#FF0000",  # red      (n=1)
    2: "#00AA00",  # green    (n=2)
    3: "#0000FF",  # blue     (n=3)
}

CORRIDOR_FILL_OPACITY = 0.1
CORRIDOR_STROKE_WIDTH = 2.0

TRAJ_STROKE_WIDTH = 3.5
CTRL_PT_RADIUS = 0.08

SUCCESS_COLOR = "#00AA00"
FAILURE_COLOR = "#CC0000"

# ── 2D Positions (used in Scenes 1-3 and as the base for 3D) ────────────────
# Coordinate system: roughly [-4, 4] x [-3, 3] in Manim units

_SHIFT_X = -1.0  # shift all positions left to balance white space
START_A = np.array([2.2 + _SHIFT_X, -1.8, 0.0])
GOAL_G = np.array([-1.0 + _SHIFT_X, 2.5, 0.0])

# Control points along the initial trajectory (A -> G)
# 4 points = 3 segments (global plan)
CTRL_POINTS = [
    np.array([2.2 + _SHIFT_X, -1.8, 0.0]),  # x0 ≈ A
    np.array([-0.2 + _SHIFT_X, -1.5, 0.0]),  # x1
    np.array([-1.5 + _SHIFT_X, 0.3, 0.0]),  # x2
    np.array([-1.0 + _SHIFT_X, 2.5, 0.0]),  # x3 ≈ G
]

# Smooth trajectory: 4 colored pieces arcing close to the obstacle
# Waypoints (white dots) at wp1, wp2, wp3 divide the path into 4 color segments
# Control points chosen so tangent directions match at each waypoint
# Single smooth trajectory through all key points (guarantees G1 continuity).
# Split into 4 colored pieces at the waypoint indices.
# Pink shorter, green/blue longer: waypoints shifted accordingly.
# Just 5 key points: A, wp1, wp2, wp3, G
# set_points_smoothly creates 4 bezier segments = 4 color pieces, no waviness
SMOOTH_TRAJ_ALL_POINTS = [
    np.array([2.2 + _SHIFT_X, -1.8, 0.0]),  # 0: A
    np.array([0.65 + _SHIFT_X, -1.0, 0.0]),  # 1: wp1
    np.array([-0.55 + _SHIFT_X, 0.15, 0.0]),  # 2: wp2
    np.array([-1.1 + _SHIFT_X, 1.3, 0.0]),  # 3: wp3
    np.array([-1.0 + _SHIFT_X, 2.5, 0.0]),  # 4: G
]
# Indices for the 3 white dots (waypoints)
SMOOTH_TRAJ_DOT_INDICES = [1, 2, 3]
# Color breaks: [0..1]=magenta, [1..2]=red, [2..3]=green, [3..4]=blue
SMOOTH_TRAJ_COLOR_BREAKS = [0, 1, 2, 3, 4]

# Static obstacles (centers + approximate radii for blob shapes)
STATIC_OBS = [
    {"center": np.array([-2.5, 1.5, 0.0]), "rx": 1.0, "ry": 0.8},  # top-left
    {"center": np.array([-2.0, -1.5, 0.0]), "rx": 0.9, "ry": 0.7},  # bottom-left
]

# Dynamic obstacle
DYN_OBS_CENTER_T0 = np.array([0.8 + _SHIFT_X, 0.5, 0.0])
DYN_OBS_RADIUS = 0.7
DYN_OBS_VELOCITY = np.array([-0.6, 0.4, 0.0])  # direction of motion

# Where dynamic obstacle moves to at t1
DYN_OBS_CENTER_T1 = DYN_OBS_CENTER_T0 + np.array([-0.3, -0.25, 0.0])

# ── Replanning at t=t1 ──────────────────────────────────────────────────────
# A' is along the pink trajectory, ~80% toward wp1
# G' is near the original G but slightly offset
GOAL_G_PRIME = np.array([-1.2 + _SHIFT_X, 2.6, 0.0])

# New global plan from A' to G' (3 segments, avoiding the moved obstacle)
# These points go around the left side of the new obstacle position
REPLAN_CTRL_POINTS = None  # Will be set dynamically based on A' position

# ── 3D / Spatio-Temporal Parameters ─────────────────────────────────────────
NUM_TIME_LAYERS = 4  # n = 0, 1, 2, 3
TIME_LAYER_HEIGHT = 1.8  # vertical spacing per time layer in 3D

# Predicted dynamic obstacle radii at each time layer (equally spaced)
DYN_OBS_RADII = [0.5, 0.9, 1.3, 1.7]

# ── 2D Spatial-Only Corridor Vertices (parallelogram polytopes) ─────────────
# Each corridor is a list of 4 vertices (parallelogram) in 2D
SPATIAL_CORRIDORS_2D = [
    # Around x0 → x1
    [
        np.array([3.0, -3.0, 0]),
        np.array([2.0, -3.0, 0]),
        np.array([0.5, -0.3, 0]),
        np.array([1.5, -0.3, 0]),
    ],
    # Around x1 → x2
    [
        np.array([1.5, -1.3, 0]),
        np.array([0.3, -1.3, 0]),
        np.array([-1.0, 1.3, 0]),
        np.array([0.2, 1.3, 0]),
    ],
    # Around x2 → x3
    [
        np.array([0.2, 0.3, 0]),
        np.array([-1.2, 0.3, 0]),
        np.array([-1.2, 3.2, 0]),
        np.array([0.2, 3.2, 0]),
    ],
]

# ── 3D Spatio-Temporal Corridor Definitions ─────────────────────────────────
# Each corridor C[i][j] is defined at a specific time layer n with 2D base vertices.
# In 3D, it becomes a slab between two time layers.


def _make_corridor(base_verts_2d, n_low, n_high):
    """Create a 3D corridor dict from 2D base vertices and time-layer range."""
    return {
        "base_verts": [np.array(v) for v in base_verts_2d],
        "n_low": n_low,
        "n_high": n_high,
    }


# Corridors for t=t0 plan  (C[interval][index])
CORRIDORS_3D_T0 = {
    (0, 0): _make_corridor([[2.0, -3.0], [3.2, -3.0], [3.2, -1.5], [2.0, -1.5]], 0, 1),
    (1, 0): _make_corridor([[1.5, -1.5], [2.5, -1.5], [2.0, -0.2], [1.0, -0.2]], 0, 1),
    (1, 1): _make_corridor([[0.5, -1.2], [1.8, -1.2], [1.2, 0.3], [0.0, 0.3]], 1, 2),
    (2, 1): _make_corridor([[-0.2, -0.2], [1.0, -0.2], [0.5, 1.2], [-0.8, 1.2]], 1, 2),
    (2, 2): _make_corridor([[-1.0, 0.5], [0.3, 0.5], [0.0, 1.8], [-1.3, 1.8]], 2, 3),
    (3, 2): _make_corridor([[-1.2, 1.5], [0.2, 1.5], [0.2, 3.2], [-1.2, 3.2]], 2, 3),
}

# Corridors for t=t1 replan (C'[i][j])
CORRIDORS_3D_T1 = {
    (0, 0): _make_corridor([[1.8, -2.8], [3.0, -2.8], [3.0, -1.3], [1.8, -1.3]], 0, 1),
    (1, 1): _make_corridor([[0.8, -1.5], [2.0, -1.5], [1.5, 0.0], [0.3, 0.0]], 1, 2),
    (2, 1): _make_corridor([[-0.5, -0.5], [0.8, -0.5], [0.3, 1.0], [-1.0, 1.0]], 1, 2),
    (2, 2): _make_corridor([[-1.3, 0.8], [0.0, 0.8], [-0.3, 2.0], [-1.6, 2.0]], 2, 3),
    (3, 2): _make_corridor([[-1.5, 1.8], [0.0, 1.8], [0.0, 3.5], [-1.5, 3.5]], 2, 3),
}

# Replan control points (t1)
CTRL_POINTS_T1 = [
    np.array([2.3, -2.3, 0.0]),
    np.array([1.2, -0.6, 0.0]),
    np.array([-0.3, 1.0, 0.0]),
    np.array([-0.7, 3.0, 0.0]),
]

# ── Camera angles ────────────────────────────────────────────────────────────
CAM_TOP_DOWN = {"phi": 0 * DEGREES, "theta": -90 * DEGREES}
CAM_3D_VIEW = {"phi": 65 * DEGREES, "theta": -60 * DEGREES}
