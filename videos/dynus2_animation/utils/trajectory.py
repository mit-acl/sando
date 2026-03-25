"""Helper functions for trajectory curves and control points."""

from manim import *
import numpy as np
from .config import CORRIDOR_COLORS, TRAJ_STROKE_WIDTH, CTRL_PT_RADIUS


def create_trajectory_curve_2d(control_points, color=BLUE, stroke_width=None):
    """Create a smooth 2D curve through control points using CubicBezier or VMobject."""
    if stroke_width is None:
        stroke_width = TRAJ_STROKE_WIDTH
    curve = VMobject(color=color, stroke_width=stroke_width)
    curve.set_points_smoothly([np.array(p) for p in control_points])
    return curve


def create_trajectory_curve_3d(
    control_points, num_time_layers=4, time_layer_height=1.8
):
    """
    Create a 3D trajectory curve with segments colored by time layer.

    Each control point is placed at its corresponding time layer height.
    Returns a VGroup of colored segments.
    """
    # Assign each control point to a time layer
    pts_3d = []
    for idx, pt in enumerate(control_points):
        z = idx * time_layer_height
        pts_3d.append(np.array([pt[0], pt[1], z]))

    segments = VGroup()
    for i in range(len(pts_3d) - 1):
        color = CORRIDOR_COLORS.get(i, BLUE)
        seg = Line3D(
            start=pts_3d[i],
            end=pts_3d[i + 1],
            color=color,
            stroke_width=TRAJ_STROKE_WIDTH,
        )
        segments.add(seg)

    return segments, pts_3d


def create_control_point_dots_3d(pts_3d, labels=None):
    """Create colored dots at 3D control point positions."""
    dots = VGroup()
    for idx, pt in enumerate(pts_3d):
        n = min(idx, len(CORRIDOR_COLORS) - 1)
        color = CORRIDOR_COLORS[n]
        dot = Sphere(radius=CTRL_PT_RADIUS, color=color).move_to(pt)
        dot.set_color(color)
        dots.add(dot)
    return dots


def create_control_point_dots_2d(control_points, labels=None, color=BLUE):
    """Create labeled dots at 2D control point positions."""
    dots = VGroup()
    label_mobjects = VGroup()
    default_labels = [f"x_{i}" for i in range(len(control_points))]
    if labels is None:
        labels = default_labels

    for idx, (pt, lbl) in enumerate(zip(control_points, labels)):
        dot = Dot(point=np.array(pt), radius=CTRL_PT_RADIUS * 1.2, color=color)
        dots.add(dot)
        tex = MathTex(lbl, color=BLACK, font_size=28)
        tex.next_to(dot, RIGHT, buff=0.12)
        label_mobjects.add(tex)

    return dots, label_mobjects
