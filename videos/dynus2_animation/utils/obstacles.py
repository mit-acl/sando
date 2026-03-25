"""Helper functions for drawing obstacles."""

from manim import *
import numpy as np
from .config import STATIC_OBS_COLOR, DYNAMIC_OBS_COLOR


def create_static_obstacle(center, rx, ry, num_points=12):
    """Create an irregular rounded blob for a static obstacle."""
    rng = np.random.default_rng(hash(tuple(center[:2].tolist())) % (2**31))
    angles = np.linspace(0, TAU, num_points, endpoint=False)
    points = []
    for a in angles:
        r = 1.0 + 0.15 * rng.standard_normal()
        x = center[0] + rx * r * np.cos(a)
        y = center[1] + ry * r * np.sin(a)
        points.append([x, y, 0.0])
    blob = Polygon(
        *points,
        color=STATIC_OBS_COLOR,
        fill_color=STATIC_OBS_COLOR,
        fill_opacity=0.6,
        stroke_width=1.5,
    )
    return blob


def create_dynamic_obstacle(center, radius, opacity=0.5):
    """Create a filled circle for the dynamic obstacle."""
    circle = Circle(
        radius=radius,
        color=DYNAMIC_OBS_COLOR,
        fill_color=DYNAMIC_OBS_COLOR,
        fill_opacity=opacity,
        stroke_width=2,
    )
    circle.move_to(center)
    return circle


def create_velocity_arrow(center, velocity, length=0.8):
    """Create a small arrow showing velocity direction."""
    direction = velocity / np.linalg.norm(velocity)
    arrow = Arrow(
        start=center,
        end=center + direction * length,
        color=DYNAMIC_OBS_COLOR,
        stroke_width=3,
        max_tip_length_to_length_ratio=0.25,
    )
    return arrow
