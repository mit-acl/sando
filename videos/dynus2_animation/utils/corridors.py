"""Helper functions for drawing polytope corridors in 2D and 3D."""

from manim import *
import numpy as np
from .config import (
    CORRIDOR_COLORS,
    CORRIDOR_FILL_OPACITY,
    CORRIDOR_STROKE_WIDTH,
    TIME_LAYER_HEIGHT,
)


def create_2d_corridor(vertices, color=BLUE, fill_opacity=0.15):
    """Create a 2D parallelogram corridor from 4 vertices."""
    poly = Polygon(
        *vertices,
        color=color,
        fill_color=color,
        fill_opacity=fill_opacity,
        stroke_width=CORRIDOR_STROKE_WIDTH,
    )
    return poly


def create_3d_corridor_slab(base_verts_2d, n_low, n_high, time_layer_color_index=None):
    """
    Create a 3D corridor slab (extruded polygon between two time layers).

    Returns a VGroup of Polygon3D faces forming the slab.
    The color is determined by the lower time layer index.
    """
    n_color = time_layer_color_index if time_layer_color_index is not None else n_low
    color = CORRIDOR_COLORS[n_color]

    z_low = n_low * TIME_LAYER_HEIGHT
    z_high = n_high * TIME_LAYER_HEIGHT

    # Bottom and top face vertices
    bottom = [np.array([v[0], v[1], z_low]) for v in base_verts_2d]
    top = [np.array([v[0], v[1], z_high]) for v in base_verts_2d]

    faces = VGroup()

    # Bottom face
    bottom_face = Polygon(
        *bottom,
        color=color,
        fill_color=color,
        fill_opacity=CORRIDOR_FILL_OPACITY,
        stroke_width=CORRIDOR_STROKE_WIDTH,
        stroke_color=color,
    )
    faces.add(bottom_face)

    # Top face
    top_face = Polygon(
        *top,
        color=color,
        fill_color=color,
        fill_opacity=CORRIDOR_FILL_OPACITY,
        stroke_width=CORRIDOR_STROKE_WIDTH,
        stroke_color=color,
    )
    faces.add(top_face)

    # Side faces
    n = len(bottom)
    for i in range(n):
        j = (i + 1) % n
        side = Polygon(
            bottom[i],
            bottom[j],
            top[j],
            top[i],
            color=color,
            fill_color=color,
            fill_opacity=CORRIDOR_FILL_OPACITY * 0.7,
            stroke_width=CORRIDOR_STROKE_WIDTH * 0.5,
            stroke_color=color,
        )
        faces.add(side)

    return faces


def create_corridors_for_time_layer(corridors_dict, target_n):
    """
    From a corridors dict, create all 3D corridor slabs whose n_low == target_n.
    Returns a VGroup.
    """
    group = VGroup()
    for (i, j), corr in corridors_dict.items():
        if corr["n_low"] == target_n:
            slab = create_3d_corridor_slab(
                corr["base_verts"],
                corr["n_low"],
                corr["n_high"],
                time_layer_color_index=target_n,
            )
            group.add(slab)
    return group
