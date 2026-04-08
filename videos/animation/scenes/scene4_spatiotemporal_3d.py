"""
Scene 4: SANDO2's Solution — Spatio-Temporal Safe Flight Corridors

Animation sequence:
1. Dynamic obstacle (double circle) + O^d label
2. A and G points with labels
3. Global plan: lines drawn segment by segment, black dots appear at endpoints
4. Dashed prediction circles + r labels
5. First segment polytopes: pink, red, green, blue (one by one) with C labels
6. Second + third segment polytopes together with C labels
7. Smooth colored trajectory + white waypoint dots
8. Fade non-matching polytopes
9. Bring all polytopes back, shift to 3D view
10. Fade trajectory, separate polytopes vertically by time layer
11. Add n-axis with labels, C[n][p] labels on 3D polytopes
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from manim import *
import numpy as np
from utils.config import (
    DYN_OBS_CENTER_T0,
    DYN_OBS_RADII,
    CORRIDOR_COLORS,
    CORRIDOR_FILL_OPACITY,
    CORRIDOR_STROKE_WIDTH,
    CTRL_POINTS,
    START_A,
    GOAL_G,
    TIME_LAYER_HEIGHT,
    NUM_TIME_LAYERS,
    TRAJ_STROKE_WIDTH,
    SMOOTH_TRAJ_ALL_POINTS,
    SMOOTH_TRAJ_DOT_INDICES,
    SMOOTH_TRAJ_COLOR_BREAKS,
)

# Height per time layer when separating in 3D
LAYER_Z = {0: 1.0, 1: 2.0, 2: 3.0, 3: 4.0}


class SpatioTemporalSFC(ThreeDScene):
    """2D -> 3D transition showing spatio-temporal safe flight corridors."""

    def construct(self):
        Tex.set_default(color=BLACK)
        MathTex.set_default(color=BLACK)

        self.set_camera_orientation(phi=0 * DEGREES, theta=-90 * DEGREES)
        self.camera.background_color = WHITE

        DOT_RADIUS = 0.06
        num_segs = len(CTRL_POINTS) - 1
        DASH_DENSITY = 4

        # ── Build all shared objects ─────────────────────────────────────────
        dyn_obs_group = self._make_double_circle(DYN_OBS_CENTER_T0, 0.12)
        label_od = MathTex(r"\mathcal{O}^d", font_size=28, color=BLACK)
        label_od.move_to(DYN_OBS_CENTER_T0 + np.array([0, 0.35, 0]))
        label_od.set_opacity(0)
        self.add_fixed_in_frame_mobjects(label_od)

        start_dot = Dot(
            point=START_A,
            radius=DOT_RADIUS,
            color=BLACK,
            fill_color=BLACK,
            fill_opacity=1.0,
        )
        goal_dot = Dot(
            point=GOAL_G,
            radius=DOT_RADIUS,
            color=BLACK,
            fill_color=BLACK,
            fill_opacity=1.0,
        )
        label_a = MathTex("A", font_size=32).next_to(start_dot, DOWN, buff=0.12)
        label_g = MathTex("G", font_size=32).next_to(
            goal_dot, UP + LEFT * 0.5, buff=0.12
        )
        label_a.set_opacity(0)
        label_g.set_opacity(0)
        self.add_fixed_in_frame_mobjects(label_a, label_g)

        # ── Step 1 & 2: Show obstacle, A, G ─────────────────────────────────
        label_od.set_opacity(1)
        self.play(
            FadeIn(dyn_obs_group),
            FadeIn(label_od),
            run_time=1.0,
        )
        label_a.set_opacity(1)
        label_g.set_opacity(1)
        self.play(
            FadeIn(start_dot),
            FadeIn(label_a),
            FadeIn(goal_dot),
            FadeIn(label_g),
            run_time=0.8,
        )
        self.wait(0.3)

        # ── Legend box at top-right (appears with global plan) ──────────────
        legend_items = VGroup()
        li_a = MathTex(r"A", r"\text{: Start}", font_size=20, color=BLACK)
        legend_items.add(li_a)
        li_g = MathTex(r"G", r"\text{: Goal}", font_size=20, color=BLACK)
        legend_items.add(li_g)
        # Global path: thin black line icon + text
        gp_line = Line(ORIGIN, RIGHT * 0.35, color=BLACK, stroke_width=1.5)
        gp_text = MathTex(r"\text{: Global path}", font_size=20, color=BLACK)
        gp_text.next_to(gp_line, RIGHT, buff=0.06)
        li_gp = VGroup(gp_line, gp_text)
        legend_items.add(li_gp)
        li_od = MathTex(
            r"\mathcal{O}^d",
            r"\text{: Dynamic obstacle}",
            font_size=20,
            color=BLACK,
        )
        legend_items.add(li_od)
        li_rn = MathTex(
            r"r_n",
            r"\text{: Reachable set radius at layer } n",
            font_size=20,
            color=BLACK,
        )
        legend_items.add(li_rn)
        li_xn = MathTex(
            r"\boldsymbol{x}_n",
            r"\text{: } n\text{-th trajectory piece}",
            font_size=20,
            color=BLACK,
        )
        legend_items.add(li_xn)
        li_cnp = MathTex(
            r"C[n][p]",
            r"\text{: Polytope at layer } n \text{, segment } p",
            font_size=20,
            color=BLACK,
        )
        legend_items.add(li_cnp)
        legend_items.arrange(DOWN, buff=0.1, aligned_edge=LEFT)
        legend_box = SurroundingRectangle(
            legend_items, color=BLACK, buff=0.15, stroke_width=1.5, fill_opacity=0
        )
        legend_all = VGroup(legend_box, legend_items)
        legend_all.to_edge(RIGHT, buff=0.15)
        self.add_fixed_in_frame_mobjects(legend_box, *legend_items)

        # ── Step 3: Global plan ───────────────────────────────────────────
        intermediate_dots = VGroup()
        for idx in range(1, len(CTRL_POINTS) - 1):
            dot = Dot(
                point=CTRL_POINTS[idx],
                radius=DOT_RADIUS,
                color=BLACK,
                fill_color=BLACK,
                fill_opacity=1.0,
            )
            intermediate_dots.add(dot)

        traj_segments_2d = VGroup()
        for i in range(num_segs):
            seg = Line(
                start=np.array(CTRL_POINTS[i]),
                end=np.array(CTRL_POINTS[i + 1]),
                color=BLACK,
                stroke_width=TRAJ_STROKE_WIDTH,
            )
            traj_segments_2d.add(seg)

        # Show legend alongside the first global plan segment
        self.play(Create(traj_segments_2d[0]), FadeIn(legend_all), run_time=0.6)
        self.play(FadeIn(intermediate_dots[0]), run_time=0.3)
        for i in range(1, len(traj_segments_2d)):
            seg = traj_segments_2d[i]
            if i < len(intermediate_dots):
                self.play(Create(seg), run_time=0.6)
                self.play(FadeIn(intermediate_dots[i]), run_time=0.3)
            else:
                self.play(Create(seg), run_time=0.6)
        self.wait(0.3)

        # ── Step 4: Dashed prediction circles + r labels ────────────────────
        DASH_DENSITY = 4
        prediction_circles_2d = VGroup()
        radius_labels_2d = VGroup()
        for n, r in enumerate(DYN_OBS_RADII):
            color = CORRIDOR_COLORS[n]
            circumference = 2 * np.pi * r
            num_dashes = max(12, int(circumference * DASH_DENSITY))
            dashed = DashedVMobject(
                Circle(radius=r, color=color, stroke_width=1.0).move_to(
                    DYN_OBS_CENTER_T0
                ),
                num_dashes=num_dashes,
                dashed_ratio=0.5,
            )
            prediction_circles_2d.add(dashed)
            lbl = MathTex(f"r_{n}", font_size=22, color=color)
            lbl.move_to(DYN_OBS_CENTER_T0 + np.array([r + 0.2, 0, 0]))
            self.add_fixed_in_frame_mobjects(lbl)
            radius_labels_2d.add(lbl)

        self.play(
            LaggedStart(*[FadeIn(c) for c in prediction_circles_2d], lag_ratio=0.2),
            FadeIn(radius_labels_2d),
            run_time=1.2,
        )
        self.wait(0.3)

        # ── Step 5: Build all polytopes + C labels ──────────────────────────
        polytopes_2d = [[None] * NUM_TIME_LAYERS for _ in range(num_segs)]
        c_labels_2d = [[None] * NUM_TIME_LAYERS for _ in range(num_segs)]
        all_polytopes_flat = VGroup()
        all_c_labels_flat = VGroup()

        for seg_idx in range(num_segs):
            p1 = CTRL_POINTS[seg_idx]
            p2 = CTRL_POINTS[seg_idx + 1]
            for n in range(NUM_TIME_LAYERS):
                color = CORRIDOR_COLORS[n]
                r = DYN_OBS_RADII[n]
                verts = self._compute_polytope_verts(
                    p1[:2],
                    p2[:2],
                    DYN_OBS_CENTER_T0[:2],
                    r,
                    far_width=0.4,
                    extend=0.15,
                    seg_idx=seg_idx,
                    layer_idx=n,
                )
                poly = Polygon(
                    *[np.array([*v, 0]) for v in verts],
                    color=color,
                    fill_color=color,
                    fill_opacity=CORRIDOR_FILL_OPACITY,
                    stroke_width=CORRIDOR_STROKE_WIDTH,
                    stroke_color=color,
                )
                poly.set_z_index(seg_idx * 10 + n)
                polytopes_2d[seg_idx][n] = poly
                all_polytopes_flat.add(poly)

                # C[n][seg] label outside the polytope, stacked by layer
                # seg 0: right side (stacked vertically)
                # seg 1: bottom-left (stacked vertically)
                # seg 2: top (stacked horizontally)
                verts_3d = [np.array([*v, 0]) for v in verts]
                c_lbl = MathTex(
                    f"C[{n}][{seg_idx}]",
                    font_size=22,
                    color=color,
                )
                if seg_idx == 0:
                    right_pt = max(verts_3d, key=lambda v: v[0])
                    c_lbl.move_to(right_pt + np.array([0.45, 0.8 - n * 0.3, 0]))
                elif seg_idx == 1:
                    bot_left = min(verts_3d, key=lambda v: v[0] + v[1])
                    c_lbl.move_to(
                        bot_left + np.array([-0.15 - n * 0.05, -0.2 - n * 0.3, 0])
                    )
                else:
                    # Last segment — C[3][2] on left, C[0][2] on right, wide spacing
                    top_pt = max(verts_3d, key=lambda v: v[1])
                    c_lbl.move_to(top_pt + np.array([2.5 - n * 0.7, 0.35, 0]))
                # Don't add to scene yet — will be added when animated
                c_lbl.set_opacity(0)
                self.add_fixed_in_frame_mobjects(c_lbl)
                c_labels_2d[seg_idx][n] = c_lbl
                all_c_labels_flat.add(c_lbl)

        # Step 5a: First segment — polytopes one by one with labels
        for n in range(NUM_TIME_LAYERS):
            c_labels_2d[0][n].set_opacity(1)
            self.play(
                FadeIn(polytopes_2d[0][n]),
                FadeIn(c_labels_2d[0][n]),
                run_time=0.5,
            )

        # Step 5b: Second segment — all together
        for n in range(NUM_TIME_LAYERS):
            c_labels_2d[1][n].set_opacity(1)
        seg1_polys = VGroup(*polytopes_2d[1])
        seg1_labels = VGroup(*[c_labels_2d[1][n] for n in range(NUM_TIME_LAYERS)])
        self.play(FadeIn(seg1_polys), FadeIn(seg1_labels), run_time=0.8)

        # Step 5c: Third segment — all together
        for n in range(NUM_TIME_LAYERS):
            c_labels_2d[2][n].set_opacity(1)
        seg2_polys = VGroup(*polytopes_2d[2])
        seg2_labels = VGroup(*[c_labels_2d[2][n] for n in range(NUM_TIME_LAYERS)])
        self.play(FadeIn(seg2_polys), FadeIn(seg2_labels), run_time=0.8)
        self.wait(0.3)

        # ── Step 6: Smooth colored trajectory + white dots ──────────────────
        full_curve = VMobject()
        full_curve.set_points_smoothly([np.array(p) for p in SMOOTH_TRAJ_ALL_POINTS])

        smooth_curves = VGroup()
        for i in range(len(SMOOTH_TRAJ_COLOR_BREAKS) - 1):
            start_idx = SMOOTH_TRAJ_COLOR_BREAKS[i]
            end_idx = SMOOTH_TRAJ_COLOR_BREAKS[i + 1]
            t_start = start_idx / (len(SMOOTH_TRAJ_ALL_POINTS) - 1)
            t_end = end_idx / (len(SMOOTH_TRAJ_ALL_POINTS) - 1)
            sub = full_curve.copy().pointwise_become_partial(full_curve, t_start, t_end)
            sub.set_color(CORRIDOR_COLORS[i])
            sub.set_stroke(width=TRAJ_STROKE_WIDTH)
            smooth_curves.add(sub)

        smooth_dots = VGroup()
        for idx in SMOOTH_TRAJ_DOT_INDICES:
            pt = SMOOTH_TRAJ_ALL_POINTS[idx]
            dot = Circle(
                radius=0.06,
                color=BLACK,
                fill_color=WHITE,
                fill_opacity=1.0,
                stroke_width=2.0,
            ).move_to(pt)
            smooth_dots.add(dot)

        # Draw trajectory one piece at a time from A to G
        # Dots appear AFTER all pieces are drawn so they sit on top
        for i, curve in enumerate(smooth_curves):
            self.play(Create(curve), run_time=0.5)
        # Now add all waypoint dots on top
        self.play(FadeIn(smooth_dots), run_time=0.4)

        # Add bold x labels using \boldsymbol
        traj_x_labels = VGroup()
        x_label_names = [
            r"\boldsymbol{x}_{0}",
            r"\boldsymbol{x}_{1}",
            r"\boldsymbol{x}_{2}",
            r"\boldsymbol{x}_{3}",
        ]
        x_label_offsets = [
            np.array([-0.2, 0.25, 0]),  # x0: above middle of pink piece
            np.array(
                [-0.05, 0.3, 0]
            ),  # x1: above middle of red piece, tiny bit up-right
            np.array([-0.4, -0.1, 0]),  # x2: left and down
            np.array([-0.35, 0.2, 0]),  # x3: left of blue, a bit up
        ]
        for i in range(len(SMOOTH_TRAJ_COLOR_BREAKS) - 1):
            color = CORRIDOR_COLORS[i]
            t_mid = (SMOOTH_TRAJ_COLOR_BREAKS[i] + SMOOTH_TRAJ_COLOR_BREAKS[i + 1]) / (
                2 * (len(SMOOTH_TRAJ_ALL_POINTS) - 1)
            )
            mid_pt = full_curve.point_from_proportion(t_mid)
            x_lbl = MathTex(x_label_names[i], font_size=30, color=color)
            x_lbl.move_to(mid_pt + x_label_offsets[i])
            self.add_fixed_in_frame_mobjects(x_lbl)
            traj_x_labels.add(x_lbl)

        self.play(FadeIn(traj_x_labels), run_time=0.6)
        self.wait(0.5)

        # ── Step 7: Fade non-matching polytopes + their labels ──────────────
        to_fade = VGroup()
        for n in [1, 2, 3]:
            to_fade.add(polytopes_2d[0][n], c_labels_2d[0][n])
        for n in [0, 2, 3]:
            to_fade.add(polytopes_2d[1][n], c_labels_2d[1][n])
        for n in [0, 1]:
            to_fade.add(polytopes_2d[2][n], c_labels_2d[2][n])

        self.play(FadeOut(to_fade), run_time=1.0)

        # Show MIQP Assignment on the right, vertically stacked with box
        miqp_title = Text("MIQP Assignment", font_size=22, color=BLACK, weight=BOLD)

        miqp_entries = VGroup()
        assignments = [
            (r"x_0 \subseteq C[0][0]", CORRIDOR_COLORS[0]),
            (r"x_1 \subseteq C[1][1]", CORRIDOR_COLORS[1]),
            (r"x_2 \subseteq C[2][2]", CORRIDOR_COLORS[2]),
            (r"x_3 \subseteq C[3][2]", CORRIDOR_COLORS[3]),
        ]
        for i, (tex, color) in enumerate(assignments):
            entry = MathTex(tex, font_size=24, color=color)
            miqp_entries.add(entry)
        miqp_entries.arrange(DOWN, buff=0.15, aligned_edge=LEFT)

        miqp_group = VGroup(miqp_title, miqp_entries)
        miqp_group.arrange(DOWN, buff=0.2)
        miqp_box = SurroundingRectangle(
            miqp_group, color=BLACK, buff=0.2, stroke_width=1.5, fill_opacity=0
        )
        miqp_all = VGroup(miqp_box, miqp_group)
        miqp_all.to_edge(RIGHT, buff=0.15)
        self.add_fixed_in_frame_mobjects(miqp_title, miqp_box, *miqp_entries)

        self.play(FadeOut(legend_all), FadeIn(miqp_all), run_time=0.8)
        self.wait(1.5)

        # ── Step 8: Bring all polytopes back ────────────────────────────────
        self.play(
            FadeIn(to_fade),
            FadeOut(miqp_all),
            FadeOut(traj_x_labels),
            run_time=0.8,
        )
        self.wait(0.3)

        # ── Step 9: Fade out circles, C labels, and 2D fixed labels ─────────
        self.play(
            FadeOut(label_od),
            FadeOut(prediction_circles_2d),
            FadeOut(radius_labels_2d),
            FadeOut(all_c_labels_flat),
            FadeOut(label_a),
            FadeOut(label_g),
            run_time=0.8,
        )
        self.wait(0.3)

        # Add 3D-positioned A and G labels (fixed orientation = upright)
        label_a_3d = MathTex("A", font_size=30, color=BLACK)
        label_a_3d.move_to(np.array([*START_A[:2], 0]) + np.array([0, -0.3, 0]))
        self.add_fixed_orientation_mobjects(label_a_3d)

        label_g_3d = MathTex("G", font_size=30, color=BLACK)
        label_g_3d.move_to(np.array([*GOAL_G[:2], 0]) + np.array([-0.3, 0.2, 0]))
        self.add_fixed_orientation_mobjects(label_g_3d)

        self.play(FadeIn(label_a_3d), FadeIn(label_g_3d), run_time=0.4)

        # Thin the global path for 3D view
        for seg in traj_segments_2d:
            seg.set_stroke(width=1.5)

        # ── Step 10: Quickly move camera to 3D view from right side ──────────
        self.move_camera(
            phi=60 * DEGREES,
            theta=-15 * DEGREES,
            zoom=0.85,
            frame_center=np.array([0, 0, 1.5]),
            run_time=0.6,
            rate_func=smooth,
        )
        # Baseline rotation
        self.begin_ambient_camera_rotation(rate=0.15)

        # Re-add "Temporal View" after 3D transition is complete
        temporal_title_3d = Text(
            "Temporal View", font_size=22, color=BLACK, weight=BOLD
        )
        temporal_title_3d.to_edge(RIGHT, buff=0.5)
        self.add_fixed_in_frame_mobjects(temporal_title_3d)
        self.play(FadeIn(temporal_title_3d), run_time=0.4)

        # ── Step 11: Separate polytopes vertically by time layer ────────────
        layer_groups = [VGroup() for _ in range(NUM_TIME_LAYERS)]
        for seg_idx in range(num_segs):
            for n in range(NUM_TIME_LAYERS):
                layer_groups[n].add(polytopes_2d[seg_idx][n])

        lift_anims = []
        for n in range(NUM_TIME_LAYERS):
            lift_anims.append(layer_groups[n].animate.shift(OUT * LAYER_Z[n]))
        self.play(*lift_anims, run_time=2.0, rate_func=smooth)
        self.wait(0.3)

        # Keep dynamic obstacle on ground (don't fade it)

        # ── Step 12: Add thin axes rotated 90° around z (n) axis ─────────
        axis_origin = np.array([3.5, -2.5, 0.0])
        ax_len = 1.2
        z_top = 4.8

        # Rotated 90° around z: x-axis now points in +y, y-axis in -x
        # Arrow3D with very small thickness for subtle arrows
        z_axis = Arrow3D(
            start=axis_origin,
            end=axis_origin + np.array([0, 0, z_top]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )
        x_axis = Arrow3D(
            start=axis_origin,
            end=axis_origin + np.array([0, ax_len, 0]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )
        y_axis = Arrow3D(
            start=axis_origin,
            end=axis_origin + np.array([-ax_len, 0, 0]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )

        # Labels upright, positioned at rotated axis tips
        n_label_3d = MathTex(r"\text{Time Layer } n", font_size=28, color=BLACK)
        n_label_3d.move_to(axis_origin + np.array([0, 0, z_top + 0.4]))
        self.add_fixed_orientation_mobjects(n_label_3d)

        x_label_3d = MathTex("x", font_size=28, color=BLACK)
        x_label_3d.move_to(axis_origin + np.array([0, ax_len + 0.25, 0]))
        self.add_fixed_orientation_mobjects(x_label_3d)

        y_label_3d = MathTex("y", font_size=28, color=BLACK)
        y_label_3d.move_to(axis_origin + np.array([-ax_len - 0.25, 0, 0]))
        self.add_fixed_orientation_mobjects(y_label_3d)

        # Tick labels — colored by layer, with tick lines
        tick_labels = VGroup()
        tick_lines = VGroup()
        for n_idx in range(NUM_TIME_LAYERS):
            z = LAYER_Z[n_idx]
            color = CORRIDOR_COLORS[n_idx]
            tick_lbl = MathTex(str(n_idx), font_size=26, color=color)
            tick_lbl.move_to(axis_origin + np.array([0.3, 0, z]))
            self.add_fixed_orientation_mobjects(tick_lbl)
            tick_labels.add(tick_lbl)
            tick_line = Line(
                start=axis_origin + np.array([-0.08, 0, z]),
                end=axis_origin + np.array([0.08, 0, z]),
                color=GREY_C,
                stroke_width=0.8,
            )
            tick_lines.add(tick_line)

        self.play(
            FadeIn(z_axis),
            FadeIn(x_axis),
            FadeIn(y_axis),
            FadeIn(n_label_3d),
            FadeIn(x_label_3d),
            FadeIn(y_label_3d),
            FadeIn(tick_labels),
            FadeIn(tick_lines),
            run_time=0.8,
        )
        self.wait(0.5)

        # (C[n][seg] labels in 3D commented out for now)
        c_labels_3d = VGroup()  # placeholder

        # ── Step 14: Dashed prediction circles at each layer ────────────────
        dyn_3d_group = VGroup()
        for n in range(NUM_TIME_LAYERS):
            z = LAYER_Z[n]
            r = DYN_OBS_RADII[n]
            color = CORRIDOR_COLORS[n]
            center_n = np.array([DYN_OBS_CENTER_T0[0], DYN_OBS_CENTER_T0[1], z])

            double_circ = self._make_double_circle(center_n, 0.12)
            circumference = 2 * np.pi * r
            num_dashes = max(12, int(circumference * 4))
            dashed_ring = DashedVMobject(
                Circle(radius=r, color=color, stroke_width=1.0).move_to(center_n),
                num_dashes=num_dashes,
                dashed_ratio=0.5,
            )
            dyn_3d_group.add(VGroup(double_circ, dashed_ring))

        self.play(
            LaggedStart(*[FadeIn(g, scale=0.6) for g in dyn_3d_group], lag_ratio=0.3),
            run_time=1.5,
        )
        self.wait(0.5)

        # ── Step 15: Rotate for 1 second then stop ──────────────────────────
        self.wait(1.0)
        self.stop_ambient_camera_rotation()

        # Fade out dynamic obstacle circles
        self.play(FadeOut(dyn_3d_group), run_time=0.5)
        self.wait(0.3)

        # ── Step 16: Show trajectory pieces + dots + vertical connections ───
        traj_3d_pieces = VGroup()
        traj_3d_labels = VGroup()
        traj_3d_dots = VGroup()
        piece_names = [
            r"\boldsymbol{x}_{0}",
            r"\boldsymbol{x}_{1}",
            r"\boldsymbol{x}_{2}",
            r"\boldsymbol{x}_{3}",
        ]

        # Use actual waypoint positions from config for exact alignment
        # Waypoints are at indices 1, 2, 3 in SMOOTH_TRAJ_ALL_POINTS (between A and G)
        # A = index 0, wp1 = 1, wp2 = 2, wp3 = 3, G = 4
        all_wp_2d = [np.array(p[:2]) for p in SMOOTH_TRAJ_ALL_POINTS]

        # Collect all waypoint 3D positions per layer
        waypoint_3d_positions = []  # list of (start_3d, end_3d) per layer

        for n in range(NUM_TIME_LAYERS):
            color = CORRIDOR_COLORS[n]
            z = LAYER_Z[n]
            t_start = SMOOTH_TRAJ_COLOR_BREAKS[n] / (len(SMOOTH_TRAJ_ALL_POINTS) - 1)
            t_end = SMOOTH_TRAJ_COLOR_BREAKS[n + 1] / (len(SMOOTH_TRAJ_ALL_POINTS) - 1)

            # Trajectory piece at this level
            sub = full_curve.copy().pointwise_become_partial(full_curve, t_start, t_end)
            sub.set_color(color)
            sub.set_stroke(width=TRAJ_STROKE_WIDTH)
            sub.shift(OUT * z)
            traj_3d_pieces.add(sub)

            # Use exact waypoint positions (not curve proportion) for dots
            wp_start_idx = SMOOTH_TRAJ_COLOR_BREAKS[n]
            wp_end_idx = SMOOTH_TRAJ_COLOR_BREAKS[n + 1]
            start_3d = np.array([*all_wp_2d[wp_start_idx], z])
            end_3d = np.array([*all_wp_2d[wp_end_idx], z])
            waypoint_3d_positions.append((start_3d, end_3d))

            # Dots: black-filled at A (n=0 start) and G (n=3 end),
            # white-filled black-edged for all others
            for is_start, pt in [(True, start_3d), (False, end_3d)]:
                if (n == 0 and is_start) or (n == NUM_TIME_LAYERS - 1 and not is_start):
                    # A or G level: black filled
                    dot = Circle(
                        radius=0.06,
                        color=BLACK,
                        fill_color=BLACK,
                        fill_opacity=1.0,
                        stroke_width=1.5,
                    ).move_to(pt)
                else:
                    # Intermediate: white filled, black edged
                    dot = Circle(
                        radius=0.06,
                        color=BLACK,
                        fill_color=WHITE,
                        fill_opacity=1.0,
                        stroke_width=2.0,
                    ).move_to(pt)
                traj_3d_dots.add(dot)

            # Label (created but not added to scene yet)
            t_mid = (t_start + t_end) / 2
            mid_pt = full_curve.point_from_proportion(t_mid) + np.array([0, 0, z])
            # Per-label 3D offsets (camera: phi=60, theta=-15)
            # x0: move down in camera view → -y, -z
            # x1: move right in camera view → +x
            lbl_3d_offsets = [
                np.array([0.6, 0.7, 0.1]),  # x0: down-right in camera view
                np.array([-0.4, 0.3, 0.35]),  # x1: toward x2, slightly up
                np.array([0, 0, 0.3]),  # x2: default above
                np.array([0.3, 0.3, 0.4]),  # x3: right, above trajectory
            ]
            lbl = MathTex(piece_names[n], font_size=26, color=color)
            lbl.move_to(mid_pt + lbl_3d_offsets[n])
            traj_3d_labels.add(lbl)

        # Animate trajectory pieces + labels appearing one by one
        for n in range(NUM_TIME_LAYERS):
            self.add_fixed_orientation_mobjects(traj_3d_labels[n])
            self.play(
                Create(traj_3d_pieces[n]),
                FadeIn(traj_3d_labels[n]),
                run_time=0.5,
            )

        self.play(FadeIn(traj_3d_dots), run_time=0.4)

        # Vertical dotted lines from GROUND to each waypoint at every level
        # For each unique (x,y) waypoint, draw a single dashed line from z=0
        # up to the highest level that uses it
        vertical_lines = VGroup()

        # All unique waypoint (x,y) positions and the z range they span
        # A is used at ground and n=0, wp1 at n=0 end / n=1 start, etc.
        waypoint_xy_z = {}  # (x,y) tuple -> (min_z, max_z)
        for n in range(NUM_TIME_LAYERS):
            s3d, e3d = waypoint_3d_positions[n]
            for pt in [s3d, e3d]:
                key = (round(pt[0], 4), round(pt[1], 4))
                z = pt[2]
                if key not in waypoint_xy_z:
                    waypoint_xy_z[key] = [z, z]
                else:
                    waypoint_xy_z[key][0] = min(waypoint_xy_z[key][0], z)
                    waypoint_xy_z[key][1] = max(waypoint_xy_z[key][1], z)

        for (x, y), (z_min, z_max) in waypoint_xy_z.items():
            ground_pt = np.array([x, y, 0])
            top_pt = np.array([x, y, z_max])
            dashed_line = DashedVMobject(
                Line(start=ground_pt, end=top_pt, color=BLACK, stroke_width=1.0),
                num_dashes=max(6, int((z_max) * 4)),
                dashed_ratio=0.5,
            )
            vertical_lines.add(dashed_line)

        self.play(FadeIn(vertical_lines), run_time=0.8)
        self.wait(2.0)

        # Early exit for STSFCFirstHalf
        if getattr(self, "STOP_AFTER_3D", False):
            return

        # ── Step 17: Reverse 3D → back to 2D view ─────────────────────────
        # Fade out 3D-only elements (trajectories, dots, axes, circles, labels)
        self.play(
            FadeOut(traj_3d_pieces),
            FadeOut(traj_3d_dots),
            FadeOut(traj_3d_labels),
            FadeOut(vertical_lines),
            FadeOut(z_axis),
            FadeOut(x_axis),
            FadeOut(y_axis),
            FadeOut(n_label_3d),
            FadeOut(x_label_3d),
            FadeOut(y_label_3d),
            FadeOut(tick_labels),
            FadeOut(tick_lines),
            FadeOut(c_labels_3d),
            FadeOut(label_a_3d),
            FadeOut(label_g_3d),
            run_time=0.8,
        )

        # Lower polytopes back to ground (reverse the lift)
        lower_anims = []
        for n in range(NUM_TIME_LAYERS):
            lower_anims.append(layer_groups[n].animate.shift(OUT * (-LAYER_Z[n])))
        self.play(*lower_anims, run_time=1.5, rate_func=smooth)

        # Rotate camera back to 2D top-down
        self.move_camera(
            phi=0 * DEGREES,
            theta=-90 * DEGREES,
            zoom=1.0,
            frame_center=ORIGIN,
            run_time=1.5,
            rate_func=smooth,
        )
        self.wait(0.5)

        # Now back in 2D with polytopes, global path, trajectory, dots on screen

        # ── Step 18: Replanning at t = t1 ─────────────────────────────────
        # dyn_obs_group is still on screen from t=t0 (kept on ground)
        # Just make sure it's visible
        self.bring_to_front(dyn_obs_group)

        # Bring back label_a and label_g (re-create as fixed in frame)
        label_a2 = MathTex("A", font_size=32).next_to(start_dot, DOWN, buff=0.12)
        self.add_fixed_in_frame_mobjects(label_a2)
        label_g2 = MathTex("G", font_size=32).next_to(
            goal_dot, UP + LEFT * 0.5, buff=0.12
        )
        self.add_fixed_in_frame_mobjects(label_g2)
        self.play(FadeIn(label_a2), FadeIn(label_g2), run_time=0.4)

        # Bring back prediction circles
        pred_circles_2 = VGroup()
        r_labels_2 = VGroup()
        for n, r in enumerate(DYN_OBS_RADII):
            color = CORRIDOR_COLORS[n]
            circumference = 2 * np.pi * r
            num_dashes = max(12, int(circumference * DASH_DENSITY))
            dashed = DashedVMobject(
                Circle(radius=r, color=color, stroke_width=1.0).move_to(
                    DYN_OBS_CENTER_T0
                ),
                num_dashes=num_dashes,
                dashed_ratio=0.5,
            )
            pred_circles_2.add(dashed)
            lbl = MathTex(f"r_{n}", font_size=22, color=color)
            lbl.move_to(DYN_OBS_CENTER_T0 + np.array([r + 0.2, 0, 0]))
            self.add_fixed_in_frame_mobjects(lbl)
            r_labels_2.add(lbl)

        self.play(FadeIn(pred_circles_2), FadeIn(r_labels_2), run_time=0.8)
        self.wait(0.5)

        # Add "t = t1" box — same position as t0
        t1_text = MathTex(r"t = t_1", font_size=30, color=BLACK)
        t1_rect = SurroundingRectangle(
            t1_text, color=BLACK, buff=0.15, stroke_width=1.5, fill_opacity=0
        )
        t1_box = VGroup(t1_rect, t1_text)
        t1_box.move_to([-3.5, 3.5, 0])
        self.add_fixed_in_frame_mobjects(t1_text, t1_rect)
        self.play(FadeIn(t1_box), run_time=0.5)
        self.wait(0.3)

        # Remove polytopes and global path
        self.play(
            *[
                FadeOut(polytopes_2d[s][n])
                for s in range(num_segs)
                for n in range(NUM_TIME_LAYERS)
            ],
            FadeOut(traj_segments_2d),
            FadeOut(intermediate_dots),
            run_time=0.8,
        )
        self.wait(0.3)

        # A' position: ~80% along the pink piece
        t_a_prime = 0.8 * (
            SMOOTH_TRAJ_COLOR_BREAKS[1] / (len(SMOOTH_TRAJ_ALL_POINTS) - 1)
        )

        # Create a partial pink curve from A to A'
        pink_to_a_prime = full_curve.copy().pointwise_become_partial(
            full_curve, 0, t_a_prime
        )
        # Use the actual end of the partial curve as A' (avoids proportion mismatch)
        a_prime_pos = pink_to_a_prime.get_end()

        moving_dot = Dot(
            point=START_A,
            radius=DOT_RADIUS,
            color=BLACK,
            fill_color=BLACK,
            fill_opacity=1.0,
        )
        self.add(moving_dot)

        # New obstacle position: move left-bottom, slightly inside pink circle
        new_obs_center = DYN_OBS_CENTER_T0 + np.array([-0.3, -0.25, 0])

        # Arrow from old to new position (behind obstacle)
        obs_arrow = Arrow(
            start=DYN_OBS_CENTER_T0,
            end=new_obs_center,
            color=BLACK,
            stroke_width=8,
            max_tip_length_to_length_ratio=0.5,
            buff=0.05,
            tip_length=0.25,
        )
        self.add(obs_arrow)
        self.bring_to_front(dyn_obs_group)

        # O^d label moves with obstacle
        label_od_moving = MathTex(r"\mathcal{O}^d", font_size=28, color=BLACK)
        label_od_moving.move_to(DYN_OBS_CENTER_T0 + np.array([0, 0.35, 0]))
        label_od_moving.set_opacity(0)
        self.add_fixed_in_frame_mobjects(label_od_moving)
        label_od_moving.set_opacity(1)
        self.play(FadeIn(label_od_moving), run_time=0.2)

        # Keep moving_dot always on top
        moving_dot.set_z_index(100)

        # Animate simultaneously: dot moves, obstacle moves, O^d label moves
        self.play(
            MoveAlongPath(moving_dot, pink_to_a_prime, rate_func=smooth),
            dyn_obs_group.animate.move_to(new_obs_center),
            label_od_moving.animate.move_to(new_obs_center + np.array([0, 0.35, 0])),
            run_time=2.0,
        )

        # Make old elements transparent (use stroke opacity for curves to avoid artifacts)
        # Keep waypoint dots on top of the faded trajectory
        for d in smooth_dots:
            d.set_z_index(50)
        old_elements = VGroup(start_dot, goal_dot)
        self.play(
            smooth_curves.animate.set_stroke(opacity=0.1),
            smooth_dots.animate.set_opacity(0.15),
            old_elements.animate.set_opacity(0.1),
            label_a2.animate.set_opacity(0.1),
            label_g2.animate.set_opacity(0.1),
            run_time=0.4,
        )
        old_elements.add(smooth_curves, smooth_dots)  # group them for later reference

        # A' label
        label_a_prime = MathTex("A'", font_size=30, color=BLACK)
        label_a_prime.next_to(moving_dot, DOWN + RIGHT, buff=0.08)
        label_a_prime.set_opacity(0)
        self.add_fixed_in_frame_mobjects(label_a_prime)
        label_a_prime.set_opacity(1)
        self.play(FadeIn(label_a_prime), run_time=0.3)

        # O^d label already at new position
        label_od_new = label_od_moving
        self.wait(0.3)

        # Remove old prediction circles and r labels
        self.play(FadeOut(pred_circles_2), FadeOut(r_labels_2), run_time=0.5)

        # Create new prediction circles centered at new obstacle position
        new_pred_circles = VGroup()
        new_r_labels = VGroup()
        for n, r in enumerate(DYN_OBS_RADII):
            color = CORRIDOR_COLORS[n]
            circumference = 2 * np.pi * r
            num_dashes = max(12, int(circumference * DASH_DENSITY))
            dashed = DashedVMobject(
                Circle(radius=r, color=color, stroke_width=1.0).move_to(new_obs_center),
                num_dashes=num_dashes,
                dashed_ratio=0.5,
            )
            new_pred_circles.add(dashed)
            lbl = MathTex(f"r'_{n}", font_size=22, color=color)
            lbl.move_to(new_obs_center + np.array([r + 0.25, 0, 0]))
            self.add_fixed_in_frame_mobjects(lbl)
            new_r_labels.add(lbl)

        self.play(
            LaggedStart(*[FadeIn(c) for c in new_pred_circles], lag_ratio=0.15),
            FadeIn(new_r_labels),
            run_time=1.0,
        )
        self.wait(0.5)

        # (old elements already made transparent above)

        # ── Step 20: Add G' and replan global path A' → G' ──────────────
        # G' has same displacement from G as A' has from A
        a_to_a_prime = a_prime_pos - np.array(START_A)
        g_prime_pos = np.array(GOAL_G) + a_to_a_prime
        g_prime_dot = Dot(
            point=g_prime_pos,
            radius=DOT_RADIUS,
            color=BLACK,
            fill_color=BLACK,
            fill_opacity=1.0,
        )
        label_g_prime = MathTex("G'", font_size=30, color=BLACK)
        label_g_prime.move_to(g_prime_pos + np.array([0.0, 0.4, 0]))
        self.add_fixed_in_frame_mobjects(label_g_prime)

        self.play(FadeIn(g_prime_dot), FadeIn(label_g_prime), run_time=0.5)
        self.wait(0.3)

        # New global plan from A' to G' (3 segments going around new obstacle)
        replan_pts = [
            a_prime_pos,
            np.array([-2.0, -0.5, 0.0]),  # moved up from -1.3
            np.array([-3.0, 1.2, 0.0]),  # moved up from 0.5
            g_prime_pos,
        ]
        replan_num_segs = len(replan_pts) - 1

        # Draw replan global path segment by segment with intermediate dots
        replan_segments = VGroup()
        replan_inter_dots = VGroup()
        for i in range(replan_num_segs):
            seg = Line(
                start=np.array(replan_pts[i]),
                end=np.array(replan_pts[i + 1]),
                color=BLACK,
                stroke_width=TRAJ_STROKE_WIDTH,
            )
            replan_segments.add(seg)

        for idx in range(1, len(replan_pts) - 1):
            dot = Dot(
                point=replan_pts[idx],
                radius=DOT_RADIUS,
                color=BLACK,
                fill_color=BLACK,
                fill_opacity=1.0,
            )
            replan_inter_dots.add(dot)

        for i, seg in enumerate(replan_segments):
            if i < len(replan_inter_dots):
                self.play(Create(seg), run_time=0.5)
                self.play(FadeIn(replan_inter_dots[i]), run_time=0.2)
            else:
                self.play(Create(seg), run_time=0.5)
        self.wait(0.3)

        # ── Step 21: Generate new polytopes C'[n][seg] ──────────────────
        replan_polytopes = [[None] * NUM_TIME_LAYERS for _ in range(replan_num_segs)]
        replan_c_labels = [[None] * NUM_TIME_LAYERS for _ in range(replan_num_segs)]
        replan_all_polys = VGroup()
        replan_all_labels = VGroup()

        for seg_idx in range(replan_num_segs):
            p1 = replan_pts[seg_idx]
            p2 = replan_pts[seg_idx + 1]
            for n in range(NUM_TIME_LAYERS):
                color = CORRIDOR_COLORS[n]
                r = DYN_OBS_RADII[n]

                # Check if polytope is feasible (segment not too close to circle)
                seg_vec = np.array(p2[:2]) - np.array(p1[:2])
                seg_hat_r = seg_vec / np.linalg.norm(seg_vec)
                n1_r = np.array([-seg_hat_r[1], seg_hat_r[0]])
                mid_r = (np.array(p1[:2]) + np.array(p2[:2])) / 2
                if np.dot(new_obs_center[:2] - mid_r, n1_r) > 0:
                    n_toward_r = n1_r
                else:
                    n_toward_r = -n1_r
                seg_dist = np.dot(new_obs_center[:2] - np.array(p1[:2]), n_toward_r)

                if seg_dist < r + 0.02:
                    # Too close — skip this polytope (e.g. blue for some segments)
                    replan_polytopes[seg_idx][n] = None
                    replan_c_labels[seg_idx][n] = None
                    continue

                # Seg 1: bigger polytopes, rotated CCW
                # Seg 2: all same size as green (r=1.3)
                if seg_idx == 1:
                    fw, ext, sk = 0.55, 0.6, 0.1
                    r_use = r
                elif seg_idx == 2:
                    fw, ext, sk = 0.4, 0.15, 0.0
                    r_use = DYN_OBS_RADII[2]  # green radius for ALL layers
                else:
                    fw, ext, sk = 0.4, 0.15, 0.3
                    r_use = r

                verts = self._compute_polytope_verts(
                    p1[:2],
                    p2[:2],
                    new_obs_center[:2],
                    r_use,
                    far_width=fw,
                    extend=ext,
                    skew=sk,
                    seg_idx=seg_idx + 100,
                    layer_idx=n,
                )

                # Seg 1: make near-side edge nearly vertical (tangent to circle)
                if seg_idx == 1:
                    seg_vec = np.array(p2[:2]) - np.array(p1[:2])
                    seg_h = seg_vec / np.linalg.norm(seg_vec)
                    n_tw = np.array([-seg_h[1], seg_h[0]])
                    mid_v = (np.array(p1[:2]) + np.array(p2[:2])) / 2
                    if np.dot(new_obs_center[:2] - mid_v, n_tw) < 0:
                        n_tw = -n_tw
                    x_tang = new_obs_center[0] - r_use - 0.05
                    slope_off = 0.1
                    bot_y = np.array(p1[:2])[1] - 0.5
                    top_y = np.array(p2[:2])[1] + 0.6
                    verts[4] = np.array([x_tang, bot_y])
                    verts[3] = np.array([x_tang + slope_off, top_y])
                    verts[2] = verts[2] + np.array([0.3, 0])

                poly = Polygon(
                    *[np.array([*v, 0]) for v in verts],
                    color=color,
                    fill_color=color,
                    fill_opacity=CORRIDOR_FILL_OPACITY,
                    stroke_width=CORRIDOR_STROKE_WIDTH,
                    stroke_color=color,
                )
                poly.set_z_index(seg_idx * 10 + n)
                replan_polytopes[seg_idx][n] = poly
                replan_all_polys.add(poly)

                # C' labels — fixed positions decoupled from polytope vertices
                c_lbl = MathTex(
                    f"C'[{n}][{seg_idx}]",
                    font_size=20,
                    color=color,
                )
                if seg_idx == 0:
                    c_lbl.move_to(np.array([0.85, -0.6 - n * 0.25, 0]))
                elif seg_idx == 1:
                    c_lbl.move_to(np.array([-3.5, 0.0 - n * 0.25, 0]))
                else:
                    c_lbl.move_to(np.array([-1.5, 3.5 - n * 0.25, 0]))
                c_lbl.set_opacity(0)
                self.add_fixed_in_frame_mobjects(c_lbl)
                replan_c_labels[seg_idx][n] = c_lbl
                replan_all_labels.add(c_lbl)

        # Animate: first segment one by one, rest together
        for n in range(NUM_TIME_LAYERS):
            if replan_polytopes[0][n] is not None:
                if replan_c_labels[0][n] is not None:
                    replan_c_labels[0][n].set_opacity(1)
                self.play(
                    FadeIn(replan_polytopes[0][n]),
                    *([FadeIn(replan_c_labels[0][n])] if replan_c_labels[0][n] else []),
                    run_time=0.4,
                )

        for seg_idx in range(1, replan_num_segs):
            seg_polys = VGroup(*[p for p in replan_polytopes[seg_idx] if p is not None])
            seg_labels = VGroup()
            for n in range(NUM_TIME_LAYERS):
                if replan_c_labels[seg_idx][n] is not None:
                    replan_c_labels[seg_idx][n].set_opacity(1)
                    seg_labels.add(replan_c_labels[seg_idx][n])
            if len(seg_polys) > 0:
                self.play(FadeIn(seg_polys), FadeIn(seg_labels), run_time=0.6)

        # Fade arrow after polytopes
        self.play(FadeOut(obs_arrow), run_time=0.4)
        self.wait(0.5)

        # ── Smooth trajectory A'→G' ──────────────────────────────────────
        replan_traj_pts = [
            a_prime_pos,  # A'
            np.array([-1.5, -0.6, 0.0]),  # w1' near obstacle
            np.array([-2.4, 0.2, 0.0]),  # w2' close to O^d
            np.array([-2.8, 1.5, 0.0]),  # w3'
            g_prime_pos,  # G'
        ]
        replan_curve = VMobject()
        replan_curve.set_points_smoothly([np.array(p) for p in replan_traj_pts])

        replan_smooth_curves = VGroup()
        for i in range(4):
            ts = i / (len(replan_traj_pts) - 1)
            te = (i + 1) / (len(replan_traj_pts) - 1)
            sub = replan_curve.copy().pointwise_become_partial(replan_curve, ts, te)
            sub.set_color(CORRIDOR_COLORS[i]).set_stroke(width=TRAJ_STROKE_WIDTH)
            replan_smooth_curves.add(sub)

        for curve in replan_smooth_curves:
            self.play(Create(curve), run_time=0.5)

        replan_smooth_dots = VGroup()
        for idx in range(1, len(replan_traj_pts) - 1):
            dot = Circle(
                radius=0.06,
                color=BLACK,
                fill_color=WHITE,
                fill_opacity=1.0,
                stroke_width=2.0,
            )
            dot.move_to(replan_traj_pts[idx])
            replan_smooth_dots.add(dot)
        self.play(FadeIn(replan_smooth_dots), run_time=0.4)

        replan_x_labels = VGroup()
        rxn = [
            r"\boldsymbol{x}'_{0}",
            r"\boldsymbol{x}'_{1}",
            r"\boldsymbol{x}'_{2}",
            r"\boldsymbol{x}'_{3}",
        ]
        rxo = [
            np.array([0.0, -0.3, 0]),
            np.array([0.4, 0.1, 0]),
            np.array([0.35, -0.2, 0]),
            np.array([0.3, -0.2, 0]),
        ]
        for i in range(4):
            tm = (i + 0.5) / (len(replan_traj_pts) - 1)
            mp = replan_curve.point_from_proportion(tm)
            lb = MathTex(rxn[i], font_size=30, color=CORRIDOR_COLORS[i])
            lb.move_to(mp + rxo[i])
            self.add_fixed_in_frame_mobjects(lb)
            replan_x_labels.add(lb)
        self.play(FadeIn(replan_x_labels), run_time=0.5)
        self.wait(0.5)

        # MIQP Assignment
        miqp_title2 = Text("MIQP Assignment", font_size=26, color=BLACK, weight=BOLD)
        miqp_title2.to_edge(DOWN, buff=0.9)
        self.add_fixed_in_frame_mobjects(miqp_title2)
        miqp_entries2 = VGroup()
        for tex, color in [
            (r"x'_0 \subseteq C'[0][0]", CORRIDOR_COLORS[0]),
            (r"x'_1 \subseteq C'[1][1]", CORRIDOR_COLORS[1]),
            (r"x'_2 \subseteq C'[2][1]", CORRIDOR_COLORS[2]),
            (r"x'_3 \subseteq C'[3][2]", CORRIDOR_COLORS[3]),
        ]:
            miqp_entries2.add(MathTex(tex, font_size=26, color=color))
        miqp_entries2.arrange(RIGHT, buff=0.4)
        miqp_entries2.next_to(miqp_title2, DOWN, buff=0.15)
        self.add_fixed_in_frame_mobjects(miqp_entries2)

        # Fade unassigned polytopes
        to_fade_rp = VGroup()
        assigned = {(0, 0), (1, 1), (2, 1), (3, 2)}
        for si in range(replan_num_segs):
            for ni in range(NUM_TIME_LAYERS):
                if (ni, si) not in assigned and replan_polytopes[si][ni] is not None:
                    to_fade_rp.add(replan_polytopes[si][ni])
                if (ni, si) not in assigned and replan_c_labels[si][ni] is not None:
                    to_fade_rp.add(replan_c_labels[si][ni])
        self.play(
            FadeOut(to_fade_rp),
            FadeIn(miqp_title2),
            FadeIn(miqp_entries2),
            run_time=0.8,
        )
        self.wait(1.5)

        # Bring back and transition to 3D
        self.play(
            FadeIn(to_fade_rp),
            FadeOut(miqp_title2),
            FadeOut(miqp_entries2),
            FadeOut(replan_x_labels),
            run_time=0.6,
        )

        # ── Transition to 3D ─────────────────────────────────────────────
        self.play(
            FadeOut(replan_all_labels),
            FadeOut(old_elements),
            FadeOut(label_a2),
            FadeOut(label_g2),
            FadeOut(label_a_prime),
            FadeOut(label_g_prime),
            FadeOut(label_od_new),
            FadeOut(new_pred_circles),
            FadeOut(new_r_labels),
            run_time=0.8,
        )
        for seg in replan_segments:
            seg.set_stroke(width=1.5)

        # 3D labels for A, G (transparent), A', G'
        label_a_3d2 = MathTex("A", font_size=28, color=BLACK)
        label_a_3d2.set_opacity(0.15)
        label_a_3d2.move_to(np.array([*START_A[:2], 0]) + np.array([0, -0.3, 0]))
        self.add_fixed_orientation_mobjects(label_a_3d2)
        label_g_3d2 = MathTex("G", font_size=28, color=BLACK)
        label_g_3d2.set_opacity(0.15)
        label_g_3d2.move_to(np.array([*GOAL_G[:2], 0]) + np.array([-0.3, 0.2, 0]))
        self.add_fixed_orientation_mobjects(label_g_3d2)

        label_ap_3d = MathTex("A'", font_size=28, color=BLACK)
        label_ap_3d.move_to(np.array([*a_prime_pos[:2], 0]) + np.array([0.2, -0.25, 0]))
        self.add_fixed_orientation_mobjects(label_ap_3d)
        label_gp_3d = MathTex("G'", font_size=28, color=BLACK)
        label_gp_3d.move_to(np.array([*g_prime_pos[:2], 0]) + np.array([-0.25, 0.2, 0]))
        self.add_fixed_orientation_mobjects(label_gp_3d)
        self.play(
            FadeIn(label_ap_3d),
            FadeIn(label_gp_3d),
            FadeIn(label_a_3d2),
            FadeIn(label_g_3d2),
            run_time=0.3,
        )

        self.move_camera(
            phi=60 * DEGREES,
            theta=-15 * DEGREES,
            zoom=0.85,
            frame_center=np.array([0, 0, 1.5]),
            run_time=0.6,
            rate_func=smooth,
        )
        self.begin_ambient_camera_rotation(rate=0.15)

        # Separate polytopes
        replan_layer_groups = [VGroup() for _ in range(NUM_TIME_LAYERS)]
        for si in range(replan_num_segs):
            for ni in range(NUM_TIME_LAYERS):
                if replan_polytopes[si][ni] is not None:
                    replan_layer_groups[ni].add(replan_polytopes[si][ni])
        la = [
            g.animate.shift(OUT * LAYER_Z[n])
            for n, g in enumerate(replan_layer_groups)
            if len(g) > 0
        ]
        self.play(*la, run_time=2.0, rate_func=smooth)

        # Axes
        ao = np.array([3.5, -2.5, 0.0])
        al, zt = 1.2, 4.8
        za = Arrow3D(
            start=ao,
            end=ao + np.array([0, 0, zt]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )
        xa = Arrow3D(
            start=ao,
            end=ao + np.array([0, al, 0]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )
        ya = Arrow3D(
            start=ao,
            end=ao + np.array([-al, 0, 0]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )
        nl2 = MathTex(r"\text{Time Layer } n", font_size=28, color=BLACK)
        nl2.move_to(ao + np.array([0, 0, zt + 0.4]))
        self.add_fixed_orientation_mobjects(nl2)
        xl2 = MathTex("x", font_size=28, color=BLACK)
        xl2.move_to(ao + np.array([0, al + 0.25, 0]))
        self.add_fixed_orientation_mobjects(xl2)
        yl2 = MathTex("y", font_size=28, color=BLACK)
        yl2.move_to(ao + np.array([-al - 0.25, 0, 0]))
        self.add_fixed_orientation_mobjects(yl2)
        tl2, tln2 = VGroup(), VGroup()
        for ni in range(NUM_TIME_LAYERS):
            zz = LAYER_Z[ni]
            tl = MathTex(str(ni), font_size=26, color=CORRIDOR_COLORS[ni])
            tl.move_to(ao + np.array([0.3, 0, zz]))
            self.add_fixed_orientation_mobjects(tl)
            tl2.add(tl)
            tln2.add(
                Line(
                    start=ao + np.array([-0.08, 0, zz]),
                    end=ao + np.array([0.08, 0, zz]),
                    color=GREY_C,
                    stroke_width=0.8,
                )
            )
        self.play(
            FadeIn(za),
            FadeIn(xa),
            FadeIn(ya),
            FadeIn(nl2),
            FadeIn(xl2),
            FadeIn(yl2),
            FadeIn(tl2),
            FadeIn(tln2),
            run_time=0.6,
        )

        # Circles at each layer
        d3r = VGroup()
        for n in range(NUM_TIME_LAYERS):
            z = LAYER_Z[n]
            r = DYN_OBS_RADII[n]
            c = CORRIDOR_COLORS[n]
            cn = np.array([new_obs_center[0], new_obs_center[1], z])
            dc = self._make_double_circle(cn, 0.12)
            dr = DashedVMobject(
                Circle(radius=r, color=c, stroke_width=1.0).move_to(cn),
                num_dashes=max(12, int(2 * np.pi * r * 4)),
                dashed_ratio=0.5,
            )
            d3r.add(VGroup(dc, dr))
        self.play(
            LaggedStart(*[FadeIn(g, scale=0.6) for g in d3r], lag_ratio=0.2),
            run_time=1.0,
        )

        self.wait(1.0)
        self.stop_ambient_camera_rotation()
        self.play(FadeOut(d3r), run_time=0.5)
        self.wait(0.3)

        # Trajectory pieces at each level
        rtp = VGroup()
        rtl = VGroup()
        rtd = VGroup()
        rpn = [
            r"\boldsymbol{x}'_{0}",
            r"\boldsymbol{x}'_{1}",
            r"\boldsymbol{x}'_{2}",
            r"\boldsymbol{x}'_{3}",
        ]
        rwp = []
        for n in range(NUM_TIME_LAYERS):
            if len(replan_layer_groups[n]) == 0:
                rwp.append(None)
                continue
            color = CORRIDOR_COLORS[n]
            z = LAYER_Z[n]
            ts = n / (len(replan_traj_pts) - 1)
            te = (n + 1) / (len(replan_traj_pts) - 1)
            sub = replan_curve.copy().pointwise_become_partial(replan_curve, ts, te)
            sub.set_color(color).set_stroke(width=TRAJ_STROKE_WIDTH)
            sub.shift(OUT * z)
            rtp.add(sub)
            s3 = np.array([*replan_traj_pts[n][:2], z])
            e3 = np.array([*replan_traj_pts[n + 1][:2], z])
            rwp.append((s3, e3))
            for is_s, pt in [(True, s3), (False, e3)]:
                fl = BLACK if (n == 0 and is_s) or (n == 3 and not is_s) else WHITE
                rtd.add(
                    Circle(
                        radius=0.06,
                        color=BLACK,
                        fill_color=fl,
                        fill_opacity=1.0,
                        stroke_width=2.0,
                    ).move_to(pt)
                )
            mp = replan_curve.point_from_proportion((ts + te) / 2) + np.array([0, 0, z])
            # Per-layer offsets: x'3 moved left-down to avoid polytope overlap
            lo3d = [
                np.array([-0.3, 0, 0.3]),
                np.array([0, -0.3, 0.3]),
                np.array([0, -0.3, 0.3]),
                np.array([-0.3, -0.3, 0.3]),
            ]
            lb = MathTex(rpn[n], font_size=26, color=color)
            lb.move_to(mp + lo3d[n])
            rtl.add(lb)
        for i in range(len(rtp)):
            self.add_fixed_orientation_mobjects(rtl[i])
            self.play(Create(rtp[i]), FadeIn(rtl[i]), run_time=0.5)
        self.play(FadeIn(rtd), run_time=0.4)

        # Vertical lines
        rv = VGroup()
        wxz = {}
        for p in rwp:
            if p is None:
                continue
            for pt in p:
                k = (round(pt[0], 4), round(pt[1], 4))
                wxz[k] = max(wxz.get(k, 0), pt[2])
        for (x, y), zm in wxz.items():
            rv.add(
                DashedVMobject(
                    Line(
                        start=np.array([x, y, 0]),
                        end=np.array([x, y, zm]),
                        color=BLACK,
                        stroke_width=1.0,
                    ),
                    num_dashes=max(6, int(zm * 4)),
                    dashed_ratio=0.5,
                )
            )
        self.play(FadeIn(rv), run_time=0.8)
        self.wait(2.0)

        # ── Reverse to 2D ─────────────────────────────────────────────────
        self.play(
            FadeOut(rtp),
            FadeOut(rtd),
            FadeOut(rtl),
            FadeOut(rv),
            FadeOut(za),
            FadeOut(xa),
            FadeOut(ya),
            FadeOut(nl2),
            FadeOut(xl2),
            FadeOut(yl2),
            FadeOut(tl2),
            FadeOut(tln2),
            FadeOut(label_ap_3d),
            FadeOut(label_gp_3d),
            FadeOut(label_a_3d2),
            FadeOut(label_g_3d2),
            run_time=0.8,
        )
        low_a = [
            g.animate.shift(OUT * (-LAYER_Z[n]))
            for n, g in enumerate(replan_layer_groups)
            if len(g) > 0
        ]
        self.play(*low_a, run_time=1.5, rate_func=smooth)
        self.move_camera(
            phi=0 * DEGREES,
            theta=-90 * DEGREES,
            zoom=1.0,
            frame_center=np.array([-0.8, 0, 0]),
            run_time=1.5,
            rate_func=smooth,
        )

        # Re-add all labels in 2D after returning from 3D
        label_a_final = MathTex("A", font_size=30, color=BLACK)
        label_a_final.next_to(start_dot, DOWN, buff=0.12)
        label_a_final.set_opacity(0.15)
        self.add_fixed_in_frame_mobjects(label_a_final)
        label_g_final = MathTex("G", font_size=30, color=BLACK)
        label_g_final.next_to(goal_dot, UP + LEFT * 0.5, buff=0.12)
        label_g_final.set_opacity(0.15)
        self.add_fixed_in_frame_mobjects(label_g_final)

        label_ap_2d = MathTex("A'", font_size=30, color=BLACK)
        label_ap_2d.next_to(moving_dot, DOWN + RIGHT, buff=0.08)
        self.add_fixed_in_frame_mobjects(label_ap_2d)
        label_gp_2d = MathTex("G'", font_size=30, color=BLACK)
        label_gp_2d.move_to(g_prime_pos + np.array([-0.25, 0.2, 0]))
        self.add_fixed_in_frame_mobjects(label_gp_2d)

        self.wait(1.0)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _compute_polytope_verts(
        self,
        p1,
        p2,
        circle_center,
        circle_radius,
        far_width=0.4,
        extend=0.15,
        skew=0.3,
        seg_idx=0,
        layer_idx=0,
    ):
        p1, p2 = np.array(p1), np.array(p2)
        cc = np.array(circle_center)

        seg = p2 - p1
        seg_len = np.linalg.norm(seg)
        seg_hat = seg / seg_len

        n1 = np.array([-seg_hat[1], seg_hat[0]])
        n2 = -n1

        mid = (p1 + p2) / 2.0
        if np.dot(cc - mid, n1) > 0:
            n_toward = n1
            n_away = n2
        else:
            n_toward = n2
            n_away = n1

        ext_p1 = p1 - seg_hat * extend
        ext_p2 = p2 + seg_hat * extend
        seg_ext_len = np.linalg.norm(ext_p2 - ext_p1)

        seg_line_dist = np.dot(cc - p1, n_toward)
        base_tw = max(0.05, seg_line_dist - circle_radius)

        proj1 = np.dot(cc - ext_p1, seg_hat)
        proj2 = np.dot(cc - ext_p2, seg_hat)
        skew_factor1 = np.clip(proj1 / seg_ext_len, -1, 1)
        skew_factor2 = np.clip(proj2 / seg_ext_len, -1, 1)

        # For positive skew: tilt near side (toward obstacle)
        # For negative skew: tilt far side (away from obstacle) = "rotate" polytope
        if skew >= 0:
            tw1 = max(0.05, base_tw + skew * skew_factor1)
            tw2 = max(0.05, base_tw + skew * skew_factor2)
            far_tilt1 = 0.0
            far_tilt2 = 0.0
        else:
            tw1 = max(0.05, base_tw)
            tw2 = max(0.05, base_tw)
            # Negative skew tilts far side: one end gets wider, other narrower
            # Cap to prevent extreme distortion
            far_tilt1 = np.clip(skew * skew_factor1, -far_width * 0.8, far_width * 0.8)
            far_tilt2 = np.clip(skew * skew_factor2, -far_width * 0.8, far_width * 0.8)

        t_far = 0.45
        far_extra = 0.2
        v1 = ext_p1 + n_away * (far_width + far_tilt1)
        v_far_mid = (ext_p1 + seg_hat * seg_ext_len * t_far) + n_away * (
            far_width + far_extra
        )
        v2 = ext_p2 + n_away * (far_width + far_tilt2)
        v3 = ext_p2 + n_toward * tw2
        v4 = ext_p1 + n_toward * tw1

        if seg_idx == 0:
            v1 = v1 + seg_hat * (-0.4)

        if seg_idx == 2:
            x_tangent = cc[0] - circle_radius
            bot_y = -0.4
            top_y = 3.15
            far_x = cc[0] - 2.8

            v4 = np.array([x_tangent, bot_y])
            v3 = np.array([x_tangent, top_y])
            v1 = np.array([far_x, bot_y])
            v2 = np.array([far_x, top_y])
            mid_y = bot_y + (top_y - bot_y) * t_far
            v_far_mid = np.array([far_x - far_extra, mid_y])

        return [v1, v_far_mid, v2, v3, v4]

    def _make_double_circle(self, center, radius):
        center = np.array(center)
        outer = Circle(
            radius=radius, color=BLACK, stroke_width=1.5, fill_opacity=0
        ).move_to(center)
        inner = Circle(
            radius=radius * 0.6, color=BLACK, stroke_width=1.2, fill_opacity=0
        ).move_to(center)
        return VGroup(outer, inner)

    def _make_3d_slab(self, base_verts_2d, n_low, n_high, color_n):
        color = CORRIDOR_COLORS[color_n]
        z_low = n_low * TIME_LAYER_HEIGHT
        z_high = n_high * TIME_LAYER_HEIGHT

        bottom = [np.array([v[0], v[1], z_low]) for v in base_verts_2d]
        top = [np.array([v[0], v[1], z_high]) for v in base_verts_2d]

        faces = VGroup()
        faces.add(
            Polygon(
                *bottom,
                color=color,
                fill_color=color,
                fill_opacity=CORRIDOR_FILL_OPACITY,
                stroke_width=CORRIDOR_STROKE_WIDTH,
                stroke_color=color,
            )
        )
        faces.add(
            Polygon(
                *top,
                color=color,
                fill_color=color,
                fill_opacity=CORRIDOR_FILL_OPACITY,
                stroke_width=CORRIDOR_STROKE_WIDTH,
                stroke_color=color,
            )
        )
        n = len(bottom)
        for i in range(n):
            j = (i + 1) % n
            faces.add(
                Polygon(
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
            )
        return faces


class STSFCFirstHalf(SpatioTemporalSFC):
    """First half only: t=t0 planning through 3D spatio-temporal view.

    Stops after the 3D vertical dotted lines (before reversing to 2D / replanning).
    Render with:
        python3 -m manim -ql scenes/scene4_spatiotemporal_3d.py STSFCFirstHalf
    """

    STOP_AFTER_3D = True

    def construct(self):
        super().construct()


class ReplanScene(SpatioTemporalSFC):
    """Standalone scene for Part 2 (replanning at t=t1).

    Sets up the 2D state instantly (no animation) then runs the replanning
    sequence. Render with:
        python3 -m manim -ql scenes/scene4_spatiotemporal_3d.py ReplanScene
    """

    def construct(self):
        Tex.set_default(color=BLACK)
        MathTex.set_default(color=BLACK)

        self.set_camera_orientation(phi=0 * DEGREES, theta=-90 * DEGREES)
        self.camera.background_color = WHITE

        DOT_RADIUS = 0.06
        num_segs = len(CTRL_POINTS) - 1
        DASH_DENSITY = 4

        # ── Instantly set up the 2D state (as if Part 1 just finished) ──────
        # Dynamic obstacle
        dyn_obs_group = self._make_double_circle(DYN_OBS_CENTER_T0, 0.12)
        self.add(dyn_obs_group)

        # A and G dots
        start_dot = Dot(
            point=START_A,
            radius=DOT_RADIUS,
            color=BLACK,
            fill_color=BLACK,
            fill_opacity=1.0,
        )
        goal_dot = Dot(
            point=GOAL_G,
            radius=DOT_RADIUS,
            color=BLACK,
            fill_color=BLACK,
            fill_opacity=1.0,
        )
        self.add(start_dot, goal_dot)

        label_a2 = MathTex("A", font_size=32).next_to(start_dot, DOWN, buff=0.12)
        label_g2 = MathTex("G", font_size=32).next_to(
            goal_dot, UP + LEFT * 0.5, buff=0.12
        )
        self.add_fixed_in_frame_mobjects(label_a2, label_g2)

        # Global path + intermediate dots
        intermediate_dots = VGroup()
        for idx in range(1, len(CTRL_POINTS) - 1):
            dot = Dot(
                point=CTRL_POINTS[idx],
                radius=DOT_RADIUS,
                color=BLACK,
                fill_color=BLACK,
                fill_opacity=1.0,
            )
            intermediate_dots.add(dot)
        self.add(intermediate_dots)

        traj_segments_2d = VGroup()
        for i in range(num_segs):
            seg = Line(
                start=np.array(CTRL_POINTS[i]),
                end=np.array(CTRL_POINTS[i + 1]),
                color=BLACK,
                stroke_width=TRAJ_STROKE_WIDTH,
            )
            traj_segments_2d.add(seg)
        self.add(traj_segments_2d)

        # Smooth trajectory
        full_curve = VMobject()
        full_curve.set_points_smoothly([np.array(p) for p in SMOOTH_TRAJ_ALL_POINTS])

        smooth_curves = VGroup()
        for i in range(len(SMOOTH_TRAJ_COLOR_BREAKS) - 1):
            start_idx = SMOOTH_TRAJ_COLOR_BREAKS[i]
            end_idx = SMOOTH_TRAJ_COLOR_BREAKS[i + 1]
            t_start = start_idx / (len(SMOOTH_TRAJ_ALL_POINTS) - 1)
            t_end = end_idx / (len(SMOOTH_TRAJ_ALL_POINTS) - 1)
            sub = full_curve.copy().pointwise_become_partial(full_curve, t_start, t_end)
            sub.set_color(CORRIDOR_COLORS[i])
            sub.set_stroke(width=TRAJ_STROKE_WIDTH)
            smooth_curves.add(sub)
        self.add(smooth_curves)

        smooth_dots = VGroup()
        for idx in SMOOTH_TRAJ_DOT_INDICES:
            pt = SMOOTH_TRAJ_ALL_POINTS[idx]
            dot = Circle(
                radius=0.06,
                color=BLACK,
                fill_color=WHITE,
                fill_opacity=1.0,
                stroke_width=2.0,
            ).move_to(pt)
            smooth_dots.add(dot)
        self.add(smooth_dots)

        # Polytopes (all on ground)
        polytopes_2d = [[None] * NUM_TIME_LAYERS for _ in range(num_segs)]
        for seg_idx in range(num_segs):
            p1 = CTRL_POINTS[seg_idx]
            p2 = CTRL_POINTS[seg_idx + 1]
            for n in range(NUM_TIME_LAYERS):
                r = DYN_OBS_RADII[n]
                color = CORRIDOR_COLORS[n]
                verts = self._compute_polytope_verts(
                    p1[:2],
                    p2[:2],
                    DYN_OBS_CENTER_T0[:2],
                    r,
                    far_width=0.4,
                    extend=0.15,
                    seg_idx=seg_idx,
                    layer_idx=n,
                )
                poly = Polygon(
                    *[np.array([*v, 0]) for v in verts],
                    color=color,
                    fill_color=color,
                    fill_opacity=CORRIDOR_FILL_OPACITY,
                    stroke_width=CORRIDOR_STROKE_WIDTH,
                    stroke_color=color,
                )
                poly.set_z_index(seg_idx * 10 + n)
                polytopes_2d[seg_idx][n] = poly
                self.add(poly)

        # Prediction circles
        prediction_circles_2d = VGroup()
        radius_labels_2d = VGroup()
        for n, r in enumerate(DYN_OBS_RADII):
            color = CORRIDOR_COLORS[n]
            circumference = 2 * np.pi * r
            num_dashes = max(12, int(circumference * DASH_DENSITY))
            dashed = DashedVMobject(
                Circle(radius=r, color=color, stroke_width=1.0).move_to(
                    DYN_OBS_CENTER_T0
                ),
                num_dashes=num_dashes,
                dashed_ratio=0.5,
            )
            prediction_circles_2d.add(dashed)
            lbl = MathTex(f"r_{n}", font_size=22, color=color)
            lbl.move_to(DYN_OBS_CENTER_T0 + np.array([r + 0.2, 0, 0]))
            self.add_fixed_in_frame_mobjects(lbl)
            radius_labels_2d.add(lbl)
        self.add(prediction_circles_2d)

        self.wait(0.5)

        # ── Now run Step 18+ (replanning) ────────────────────────────────────
        # Add "t = t1" box
        t1_text = MathTex(r"t = t_1", font_size=30, color=BLACK)
        t1_rect = SurroundingRectangle(
            t1_text, color=BLACK, buff=0.15, stroke_width=1.5
        )
        t1_box = VGroup(t1_rect, t1_text)
        t1_box.to_edge(UP, buff=0.3)
        self.add_fixed_in_frame_mobjects(t1_text, t1_rect)
        self.play(FadeIn(t1_box), run_time=0.5)

        # Remove polytopes and global path
        self.play(
            *[
                FadeOut(polytopes_2d[s][n])
                for s in range(num_segs)
                for n in range(NUM_TIME_LAYERS)
            ],
            FadeOut(traj_segments_2d),
            FadeOut(intermediate_dots),
            run_time=0.8,
        )
        self.wait(0.3)

        # Move dot from A to A' along pink trajectory
        t_a_prime = 0.8 * (
            SMOOTH_TRAJ_COLOR_BREAKS[1] / (len(SMOOTH_TRAJ_ALL_POINTS) - 1)
        )
        pink_to_a_prime = full_curve.copy().pointwise_become_partial(
            full_curve, 0, t_a_prime
        )
        a_prime_pos = pink_to_a_prime.get_end()

        moving_dot = Dot(
            point=START_A,
            radius=DOT_RADIUS,
            color=BLACK,
            fill_color=BLACK,
            fill_opacity=1.0,
        )
        self.add(moving_dot)

        new_obs_center = DYN_OBS_CENTER_T0 + np.array([-0.3, -0.25, 0])

        # Show arrow behind obstacle as it starts moving
        obs_arrow = Arrow(
            start=DYN_OBS_CENTER_T0,
            end=new_obs_center,
            color=BLACK,
            stroke_width=8,
            max_tip_length_to_length_ratio=0.5,
            buff=0.05,
            tip_length=0.25,
        )
        self.add(obs_arrow)
        self.bring_to_front(dyn_obs_group)

        # Create O^d label that moves WITH the obstacle
        label_od_moving = MathTex(r"\mathcal{O}^d", font_size=28, color=BLACK)
        label_od_moving.move_to(DYN_OBS_CENTER_T0 + np.array([0, 0.35, 0]))
        self.add_fixed_in_frame_mobjects(label_od_moving)
        self.play(FadeIn(label_od_moving), run_time=0.3)

        # Keep moving_dot always on top with high z_index
        moving_dot.set_z_index(100)

        self.play(
            MoveAlongPath(moving_dot, pink_to_a_prime, rate_func=smooth),
            dyn_obs_group.animate.move_to(new_obs_center),
            label_od_moving.animate.move_to(new_obs_center + np.array([0, 0.35, 0])),
            run_time=2.0,
        )

        # Make old elements very transparent (stroke opacity for curves)
        old_elements = VGroup(smooth_dots, start_dot, goal_dot)
        self.play(
            smooth_curves.animate.set_stroke(opacity=0.1),
            old_elements.animate.set_opacity(0.1),
            label_a2.animate.set_opacity(0.1),
            label_g2.animate.set_opacity(0.1),
            run_time=0.4,
        )
        old_elements.add(smooth_curves)

        # A' label — set invisible before adding to avoid flicker
        label_a_prime = MathTex("A'", font_size=30, color=BLACK)
        label_a_prime.next_to(moving_dot, DOWN + RIGHT, buff=0.08)
        label_a_prime.set_opacity(0)
        self.add_fixed_in_frame_mobjects(label_a_prime)
        label_a_prime.set_opacity(1)
        self.play(FadeIn(label_a_prime), run_time=0.3)

        # O^d label already at new position (moved with obstacle)
        label_od_new = label_od_moving

        # Remove old circles, show new ones
        self.play(
            FadeOut(prediction_circles_2d), FadeOut(radius_labels_2d), run_time=0.5
        )

        new_pred_circles = VGroup()
        new_r_labels = VGroup()
        for n, r in enumerate(DYN_OBS_RADII):
            color = CORRIDOR_COLORS[n]
            circumference = 2 * np.pi * r
            num_dashes = max(12, int(circumference * DASH_DENSITY))
            dashed = DashedVMobject(
                Circle(radius=r, color=color, stroke_width=1.0).move_to(new_obs_center),
                num_dashes=num_dashes,
                dashed_ratio=0.5,
            )
            new_pred_circles.add(dashed)
            lbl = MathTex(f"r'_{n}", font_size=22, color=color)
            lbl.move_to(new_obs_center + np.array([r + 0.25, 0, 0]))
            self.add_fixed_in_frame_mobjects(lbl)
            new_r_labels.add(lbl)

        self.play(
            LaggedStart(*[FadeIn(c) for c in new_pred_circles], lag_ratio=0.15),
            FadeIn(new_r_labels),
            run_time=1.0,
        )
        self.wait(0.5)

        # G' with same displacement as A→A'
        a_to_a_prime = a_prime_pos - np.array(START_A)
        g_prime_pos = np.array(GOAL_G) + a_to_a_prime
        g_prime_dot = Dot(
            point=g_prime_pos,
            radius=DOT_RADIUS,
            color=BLACK,
            fill_color=BLACK,
            fill_opacity=1.0,
        )
        label_g_prime = MathTex("G'", font_size=30, color=BLACK)
        label_g_prime.move_to(g_prime_pos + np.array([-0.25, 0.2, 0]))
        self.add_fixed_in_frame_mobjects(label_g_prime)
        self.play(FadeIn(g_prime_dot), FadeIn(label_g_prime), run_time=0.5)

        # New global path (waypoints moved up, seg 1 extends toward G')
        replan_pts = [
            a_prime_pos,
            np.array([-2.0, -0.5, 0.0]),  # moved up from -1.3
            np.array([-3.0, 1.2, 0.0]),  # moved up from 0.5
            g_prime_pos,
        ]
        replan_num_segs = len(replan_pts) - 1

        replan_segments = VGroup()
        replan_inter_dots = VGroup()
        for i in range(replan_num_segs):
            seg = Line(
                start=np.array(replan_pts[i]),
                end=np.array(replan_pts[i + 1]),
                color=BLACK,
                stroke_width=TRAJ_STROKE_WIDTH,
            )
            replan_segments.add(seg)
        for idx in range(1, len(replan_pts) - 1):
            dot = Dot(
                point=replan_pts[idx],
                radius=DOT_RADIUS,
                color=BLACK,
                fill_color=BLACK,
                fill_opacity=1.0,
            )
            replan_inter_dots.add(dot)

        for i, seg in enumerate(replan_segments):
            if i < len(replan_inter_dots):
                self.play(Create(seg), run_time=0.5)
                self.play(FadeIn(replan_inter_dots[i]), run_time=0.2)
            else:
                self.play(Create(seg), run_time=0.5)

        # New polytopes
        replan_polytopes = [[None] * NUM_TIME_LAYERS for _ in range(replan_num_segs)]
        replan_c_labels = [[None] * NUM_TIME_LAYERS for _ in range(replan_num_segs)]
        replan_all_polys = VGroup()
        replan_all_labels = VGroup()

        for seg_idx in range(replan_num_segs):
            p1 = replan_pts[seg_idx]
            p2 = replan_pts[seg_idx + 1]
            for n in range(NUM_TIME_LAYERS):
                color = CORRIDOR_COLORS[n]
                r = DYN_OBS_RADII[n]
                seg_vec = np.array(p2[:2]) - np.array(p1[:2])
                seg_hat_r = seg_vec / np.linalg.norm(seg_vec)
                n1_r = np.array([-seg_hat_r[1], seg_hat_r[0]])
                mid_r = (np.array(p1[:2]) + np.array(p2[:2])) / 2
                if np.dot(new_obs_center[:2] - mid_r, n1_r) > 0:
                    n_toward_r = n1_r
                else:
                    n_toward_r = -n1_r
                seg_dist = np.dot(new_obs_center[:2] - np.array(p1[:2]), n_toward_r)

                if seg_dist < r + 0.02:
                    replan_polytopes[seg_idx][n] = None
                    replan_c_labels[seg_idx][n] = None
                    continue

                # Seg 1: bigger polytopes, rotated CCW
                # Seg 2: all same size as green (r=1.3)
                if seg_idx == 1:
                    fw, ext, sk = 0.55, 0.6, 0.1
                    r_use = r
                elif seg_idx == 2:
                    fw, ext, sk = 0.4, 0.15, 0.0
                    r_use = DYN_OBS_RADII[2]  # green radius for ALL layers
                else:
                    fw, ext, sk = 0.4, 0.15, 0.3
                    r_use = r

                verts = self._compute_polytope_verts(
                    p1[:2],
                    p2[:2],
                    new_obs_center[:2],
                    r_use,
                    far_width=fw,
                    extend=ext,
                    skew=sk,
                    seg_idx=seg_idx + 100,
                    layer_idx=n,
                )

                # Seg 1: make near-side edge nearly vertical (tangent to circle)
                # Override v3 and v4 to have similar x, spanning the segment's y range
                if seg_idx == 1:
                    seg_vec = np.array(p2[:2]) - np.array(p1[:2])
                    seg_h = seg_vec / np.linalg.norm(seg_vec)
                    n_tw = np.array([-seg_h[1], seg_h[0]])
                    mid_v = (np.array(p1[:2]) + np.array(p2[:2])) / 2
                    if np.dot(new_obs_center[:2] - mid_v, n_tw) < 0:
                        n_tw = -n_tw
                    # Vertical tangent: x = obs_x - r - margin (avoid violation)
                    x_tang = new_obs_center[0] - r_use - 0.05
                    slope_off = 0.1
                    bot_y = np.array(p1[:2])[1] - 0.5
                    top_y = np.array(p2[:2])[1] + 0.6
                    verts[4] = np.array([x_tang, bot_y])
                    verts[3] = np.array([x_tang + slope_off, top_y])
                    # Pull top-left vertex (v2) to the right a bit
                    verts[2] = verts[2] + np.array([0.3, 0])

                verts_3d = [np.array([*v, 0]) for v in verts]

                poly = Polygon(
                    *verts_3d,
                    color=color,
                    fill_color=color,
                    fill_opacity=CORRIDOR_FILL_OPACITY,
                    stroke_width=CORRIDOR_STROKE_WIDTH,
                    stroke_color=color,
                )
                poly.set_z_index(seg_idx * 10 + n)
                replan_polytopes[seg_idx][n] = poly
                replan_all_polys.add(poly)
                # C' labels — fixed positions decoupled from polytope vertices
                # Use same 0.25 vertical spacing as t=t0 labels
                c_lbl = MathTex(f"C'[{n}][{seg_idx}]", font_size=20, color=color)
                if seg_idx == 0:
                    c_lbl.move_to(np.array([0.5, -0.3 - n * 0.25, 0]))
                elif seg_idx == 1:
                    c_lbl.move_to(np.array([-3.5, -1.5 - n * 0.25, 0]))
                else:
                    c_lbl.move_to(np.array([-1.5, 3.5 - n * 0.25, 0]))
                c_lbl.set_opacity(0)
                self.add_fixed_in_frame_mobjects(c_lbl)
                replan_c_labels[seg_idx][n] = c_lbl
                replan_all_labels.add(c_lbl)

        # Animate polytopes
        for n in range(NUM_TIME_LAYERS):
            if replan_polytopes[0][n] is not None:
                if replan_c_labels[0][n] is not None:
                    replan_c_labels[0][n].set_opacity(1)
                self.play(
                    FadeIn(replan_polytopes[0][n]),
                    *([FadeIn(replan_c_labels[0][n])] if replan_c_labels[0][n] else []),
                    run_time=0.4,
                )
        for seg_idx in range(1, replan_num_segs):
            seg_polys = VGroup(*[p for p in replan_polytopes[seg_idx] if p is not None])
            seg_labels = VGroup()
            for n in range(NUM_TIME_LAYERS):
                if replan_c_labels[seg_idx][n] is not None:
                    replan_c_labels[seg_idx][n].set_opacity(1)
                    seg_labels.add(replan_c_labels[seg_idx][n])
            if len(seg_polys) > 0:
                self.play(FadeIn(seg_polys), FadeIn(seg_labels), run_time=0.6)

        # Fade out the obstacle arrow after polytopes are shown
        self.play(FadeOut(obs_arrow), run_time=0.4)
        self.wait(0.3)

        # ── Smooth trajectory A' → G' (4 colored pieces, close to O^d) ────
        # 5 key points: A', w1', w2', w3', G' — trajectory arcs close to obstacle
        replan_traj_pts = [
            a_prime_pos,  # A'
            np.array([-1.5, -0.6, 0.0]),  # w1' near obstacle
            np.array([-2.4, 0.2, 0.0]),  # w2' close to O^d
            np.array([-2.8, 1.5, 0.0]),  # w3'
            g_prime_pos,  # G'
        ]

        replan_curve = VMobject()
        replan_curve.set_points_smoothly([np.array(p) for p in replan_traj_pts])

        # 4 colored pieces
        replan_smooth_curves = VGroup()
        for i in range(4):
            t_s = i / (len(replan_traj_pts) - 1)
            t_e = (i + 1) / (len(replan_traj_pts) - 1)
            sub = replan_curve.copy().pointwise_become_partial(replan_curve, t_s, t_e)
            sub.set_color(CORRIDOR_COLORS[i])
            sub.set_stroke(width=TRAJ_STROKE_WIDTH)
            replan_smooth_curves.add(sub)

        # Draw one by one
        for i, curve in enumerate(replan_smooth_curves):
            self.play(Create(curve), run_time=0.5)

        # Waypoint dots at w1', w2', w3'
        replan_smooth_dots = VGroup()
        for idx in range(1, len(replan_traj_pts) - 1):
            dot = Circle(
                radius=0.06,
                color=BLACK,
                fill_color=WHITE,
                fill_opacity=1.0,
                stroke_width=2.0,
            )
            dot.move_to(replan_traj_pts[idx])
            replan_smooth_dots.add(dot)
        self.play(FadeIn(replan_smooth_dots), run_time=0.4)

        # x'_0 to x'_3 labels (4 pieces)
        replan_x_labels = VGroup()
        replan_x_names = [
            r"\boldsymbol{x}'_{0}",
            r"\boldsymbol{x}'_{1}",
            r"\boldsymbol{x}'_{2}",
            r"\boldsymbol{x}'_{3}",
        ]
        replan_x_offsets = [
            np.array([0.0, -0.3, 0]),  # x'0: below trajectory
            np.array([0.4, 0.1, 0]),  # x'1: upward
            np.array([0.35, -0.2, 0]),  # x'2: right side
            np.array([0.3, -0.2, 0]),  # x'3: right
        ]
        for i in range(4):
            color = CORRIDOR_COLORS[i]
            t_mid = (i + 0.5) / (len(replan_traj_pts) - 1)
            mid_pt = replan_curve.point_from_proportion(t_mid)
            lbl = MathTex(replan_x_names[i], font_size=30, color=color)
            lbl.move_to(mid_pt + replan_x_offsets[i])
            self.add_fixed_in_frame_mobjects(lbl)
            replan_x_labels.add(lbl)
        self.play(FadeIn(replan_x_labels), run_time=0.5)
        self.wait(0.5)

        # MIQP Assignment for replan
        miqp_title2 = Text("MIQP Assignment", font_size=26, color=BLACK, weight=BOLD)
        miqp_title2.to_edge(DOWN, buff=0.9)
        self.add_fixed_in_frame_mobjects(miqp_title2)

        miqp_entries2 = VGroup()
        replan_assignments = [
            (r"x'_0 \subseteq C'[0][0]", CORRIDOR_COLORS[0]),
            (r"x'_1 \subseteq C'[1][1]", CORRIDOR_COLORS[1]),
            (r"x'_2 \subseteq C'[2][1]", CORRIDOR_COLORS[2]),
            (r"x'_3 \subseteq C'[3][2]", CORRIDOR_COLORS[3]),
        ]
        for tex, color in replan_assignments:
            entry = MathTex(tex, font_size=26, color=color)
            miqp_entries2.add(entry)
        miqp_entries2.arrange(RIGHT, buff=0.4)
        miqp_entries2.next_to(miqp_title2, DOWN, buff=0.15)
        self.add_fixed_in_frame_mobjects(miqp_entries2)

        # Fade unassigned polytopes + their labels
        # Keep: C'[0][0], C'[1][1], C'[2][1], C'[3][2]
        to_fade_replan = VGroup()
        assigned = {(0, 0), (1, 1), (2, 1), (3, 2)}  # (n, seg_idx)
        for seg_idx in range(replan_num_segs):
            for n in range(NUM_TIME_LAYERS):
                if (n, seg_idx) not in assigned:
                    if replan_polytopes[seg_idx][n] is not None:
                        to_fade_replan.add(replan_polytopes[seg_idx][n])
                    if replan_c_labels[seg_idx][n] is not None:
                        to_fade_replan.add(replan_c_labels[seg_idx][n])

        self.play(
            FadeOut(to_fade_replan),
            FadeIn(miqp_title2),
            FadeIn(miqp_entries2),
            run_time=0.8,
        )
        self.wait(1.5)

        # Bring back all polytopes before 3D transition
        self.play(
            FadeIn(to_fade_replan),
            FadeOut(miqp_title2),
            FadeOut(miqp_entries2),
            FadeOut(replan_x_labels),
            run_time=0.6,
        )
        self.wait(0.3)

        # ── Transition to 3D (keep ground elements + A', G', t=t1) ──────
        self.play(
            FadeOut(replan_all_labels),
            FadeOut(old_elements),
            FadeOut(label_a2),
            FadeOut(label_g2),
            FadeOut(label_a_prime),
            FadeOut(label_g_prime),
            FadeOut(label_od_new),
            FadeOut(new_pred_circles),
            FadeOut(new_r_labels),
            run_time=0.8,
        )
        # Keep dyn_obs_group on ground (already at new_obs_center)

        # Add 3D-positioned A', G' labels (upright)
        label_ap_3d = MathTex("A'", font_size=28, color=BLACK)
        label_ap_3d.move_to(np.array([*a_prime_pos[:2], 0]) + np.array([0.2, -0.25, 0]))
        self.add_fixed_orientation_mobjects(label_ap_3d)
        label_gp_3d = MathTex("G'", font_size=28, color=BLACK)
        label_gp_3d.move_to(np.array([*g_prime_pos[:2], 0]) + np.array([-0.25, 0.2, 0]))
        self.add_fixed_orientation_mobjects(label_gp_3d)
        self.play(FadeIn(label_ap_3d), FadeIn(label_gp_3d), run_time=0.3)
        # Thin the global path for 3D view
        for seg in replan_segments:
            seg.set_stroke(width=1.5)

        self.move_camera(
            phi=60 * DEGREES,
            theta=-15 * DEGREES,
            zoom=0.85,
            frame_center=np.array([0, 0, 1.5]),
            run_time=0.6,
            rate_func=smooth,
        )
        self.begin_ambient_camera_rotation(rate=0.15)

        # Separate polytopes vertically
        replan_layer_groups = [VGroup() for _ in range(NUM_TIME_LAYERS)]
        for seg_idx in range(replan_num_segs):
            for n in range(NUM_TIME_LAYERS):
                if replan_polytopes[seg_idx][n] is not None:
                    replan_layer_groups[n].add(replan_polytopes[seg_idx][n])
        lift_anims = [
            g.animate.shift(OUT * LAYER_Z[n])
            for n, g in enumerate(replan_layer_groups)
            if len(g) > 0
        ]
        self.play(*lift_anims, run_time=2.0, rate_func=smooth)
        self.wait(0.3)

        # Axes
        ao = np.array([3.5, -2.5, 0.0])
        al, zt = 1.2, 4.8
        z_ax = Arrow3D(
            start=ao,
            end=ao + np.array([0, 0, zt]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )
        x_ax = Arrow3D(
            start=ao,
            end=ao + np.array([0, al, 0]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )
        y_ax = Arrow3D(
            start=ao,
            end=ao + np.array([-al, 0, 0]),
            color=GREY_C,
            thickness=0.005,
            height=0.15,
            base_radius=0.05,
        )
        nl = MathTex(r"\text{Time Layer } n", font_size=28, color=BLACK)
        nl.move_to(ao + np.array([0, 0, zt + 0.4]))
        self.add_fixed_orientation_mobjects(nl)
        xl = MathTex("x", font_size=28, color=BLACK)
        xl.move_to(ao + np.array([0, al + 0.25, 0]))
        self.add_fixed_orientation_mobjects(xl)
        yl = MathTex("y", font_size=28, color=BLACK)
        yl.move_to(ao + np.array([-al - 0.25, 0, 0]))
        self.add_fixed_orientation_mobjects(yl)
        tl2, tln2 = VGroup(), VGroup()
        for ni in range(NUM_TIME_LAYERS):
            zz = LAYER_Z[ni]
            tl = MathTex(str(ni), font_size=26, color=CORRIDOR_COLORS[ni])
            tl.move_to(ao + np.array([0.3, 0, zz]))
            self.add_fixed_orientation_mobjects(tl)
            tl2.add(tl)
            tln2.add(
                Line(
                    start=ao + np.array([-0.08, 0, zz]),
                    end=ao + np.array([0.08, 0, zz]),
                    color=GREY_C,
                    stroke_width=0.8,
                )
            )
        self.play(
            FadeIn(z_ax),
            FadeIn(x_ax),
            FadeIn(y_ax),
            FadeIn(nl),
            FadeIn(xl),
            FadeIn(yl),
            FadeIn(tl2),
            FadeIn(tln2),
            run_time=0.6,
        )

        # Prediction circles at each layer
        d3r = VGroup()
        for n in range(NUM_TIME_LAYERS):
            z = LAYER_Z[n]
            r = DYN_OBS_RADII[n]
            c = CORRIDOR_COLORS[n]
            cn = np.array([new_obs_center[0], new_obs_center[1], z])
            dc = self._make_double_circle(cn, 0.12)
            dr = DashedVMobject(
                Circle(radius=r, color=c, stroke_width=1.0).move_to(cn),
                num_dashes=max(12, int(2 * np.pi * r * 4)),
                dashed_ratio=0.5,
            )
            d3r.add(VGroup(dc, dr))
        self.play(
            LaggedStart(*[FadeIn(g, scale=0.6) for g in d3r], lag_ratio=0.2),
            run_time=1.0,
        )

        # Rotate then stop
        self.wait(1.0)
        self.stop_ambient_camera_rotation()
        self.play(FadeOut(d3r), run_time=0.5)
        self.wait(0.3)

        # ── Trajectory pieces at each n level ─────────────────────────────
        rtp = VGroup()
        rtl = VGroup()
        rtd = VGroup()
        rpn = [
            r"\boldsymbol{x}'_{0}",
            r"\boldsymbol{x}'_{1}",
            r"\boldsymbol{x}'_{2}",
            r"\boldsymbol{x}'_{3}",
        ]
        rwp = []
        for n in range(NUM_TIME_LAYERS):
            if len(replan_layer_groups[n]) == 0:
                rwp.append(None)
                continue
            color = CORRIDOR_COLORS[n]
            z = LAYER_Z[n]
            ts = n / (len(replan_traj_pts) - 1)
            te = (n + 1) / (len(replan_traj_pts) - 1)
            sub = replan_curve.copy().pointwise_become_partial(replan_curve, ts, te)
            sub.set_color(color).set_stroke(width=TRAJ_STROKE_WIDTH)
            sub.shift(OUT * z)
            rtp.add(sub)
            s3 = np.array([*replan_traj_pts[n][:2], z])
            e3 = np.array([*replan_traj_pts[n + 1][:2], z])
            rwp.append((s3, e3))
            for is_s, pt in [(True, s3), (False, e3)]:
                fl = BLACK if (n == 0 and is_s) or (n == 3 and not is_s) else WHITE
                rtd.add(
                    Circle(
                        radius=0.06,
                        color=BLACK,
                        fill_color=fl,
                        fill_opacity=1.0,
                        stroke_width=2.0,
                    ).move_to(pt)
                )
            mp = replan_curve.point_from_proportion((ts + te) / 2) + np.array([0, 0, z])
            # Per-layer offsets: x'3 moved left-down to avoid polytope overlap
            lbl_offsets_3d = [
                np.array([-0.3, 0, 0.3]),
                np.array([0, -0.3, 0.3]),
                np.array([0, -0.3, 0.3]),
                np.array([-0.3, -0.3, 0.3]),
            ]
            lb = MathTex(rpn[n], font_size=26, color=color)
            lb.move_to(mp + lbl_offsets_3d[n])
            rtl.add(lb)

        for i in range(len(rtp)):
            self.add_fixed_orientation_mobjects(rtl[i])
            self.play(Create(rtp[i]), FadeIn(rtl[i]), run_time=0.5)
        self.play(FadeIn(rtd), run_time=0.4)

        # Vertical dotted connections from ground
        rv = VGroup()
        wxz = {}
        for p in rwp:
            if p is None:
                continue
            for pt in p:
                k = (round(pt[0], 4), round(pt[1], 4))
                wxz[k] = max(wxz.get(k, 0), pt[2])
        for (x, y), zm in wxz.items():
            rv.add(
                DashedVMobject(
                    Line(
                        start=np.array([x, y, 0]),
                        end=np.array([x, y, zm]),
                        color=BLACK,
                        stroke_width=1.0,
                    ),
                    num_dashes=max(6, int(zm * 4)),
                    dashed_ratio=0.5,
                )
            )
        self.play(FadeIn(rv), run_time=0.8)
        self.wait(2.0)

        # ── Reverse to 2D view ────────────────────────────────────────────
        # Fade 3D-only elements
        self.play(
            FadeOut(rtp),
            FadeOut(rtd),
            FadeOut(rtl),
            FadeOut(rv),
            FadeOut(z_ax),
            FadeOut(x_ax),
            FadeOut(y_ax),
            FadeOut(nl),
            FadeOut(xl),
            FadeOut(yl),
            FadeOut(tl2),
            FadeOut(tln2),
            FadeOut(label_ap_3d),
            FadeOut(label_gp_3d),
            run_time=0.8,
        )

        # Lower polytopes back to ground
        lower_anims = []
        for n, g in enumerate(replan_layer_groups):
            if len(g) > 0:
                lower_anims.append(g.animate.shift(OUT * (-LAYER_Z[n])))
        self.play(*lower_anims, run_time=1.5, rate_func=smooth)

        # Rotate camera back to 2D top-down
        self.move_camera(
            phi=0 * DEGREES,
            theta=-90 * DEGREES,
            zoom=1.0,
            frame_center=np.array([-0.8, 0, 0]),
            run_time=1.5,
            rate_func=smooth,
        )
        self.wait(1.0)
