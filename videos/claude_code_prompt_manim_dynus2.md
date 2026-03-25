# SANDO2 Spatio-Temporal SFC Animation — Manim Project Prompt

## Context

I'm creating a YouTube explainer video for our paper SANDO2, a UAV trajectory planner for dynamic unknown environments. I need a Manim Community Edition animation that explains how our **Spatio-Temporal Safe Flight Corridors (STSFC)** with **heat-map-based MIQP assignment** work, and why they outperform conventional spatial-only corridor approaches when dynamic obstacles are present.

The animation should recreate and animate the concepts from our paper figure (attached as `spatioTemporalSFCWhyHeatMap.svg`). Use `manim` (Community Edition, install via `pip install manim`).

---

## Figure Description (what the SVG shows)

The figure is a **2-row × 3-column** layout comparing "Other Approaches" (left column) vs "SANDO2" (middle = 3D view, right = 2D MIQP view):

### Top Row (time t = t₀, initial plan succeeds for both approaches):

1. **Top-Left — "Other Approaches (No Heat Map & No STSFC)"**:
   - A 2D (x, y) scene with:
     - Two brown/tan **static obstacles** (O^s) — irregular blob shapes, one top-left, one bottom-left
     - One large orange **dynamic obstacle** (O^d) — a big circle in the center-right area
     - A start point **A** (bottom-right) and goal point **G** (top)
     - A blue trajectory curve from A → G passing around obstacles, with control points x₀, x₁, x₂, x₃ along it
     - **Blue parallelogram polytopes** (safe flight corridors) enclosing the trajectory — these are spatial-only, not accounting for time
   - Label: **"A → G Succeeds"**

2. **Top-Middle — "SANDO2" 3D view at t₀**:
   - The SAME 2D scene but now extruded into a **3D spatio-temporal** space (x, y on the ground plane, **time n** going vertically upward, n = 0, 1, 2, 3)
   - The safe flight corridors are now **colored 3D polytopes at different time layers**:
     - **Magenta** polytopes at n=0 (bottom time layer)
     - **Red** polytopes at n=1
     - **Green** polytopes at n=2
     - **Blue/purple** polytopes at n=3 (top time layer)
   - Each time layer has multiple corridor segments, labeled C[i][j] where i = control point interval index, j = polytope index within that interval
   - The trajectory curve goes from A upward through the 3D space to G, colored in segments matching the corridor colors
   - The dynamic obstacle O^d appears as concentric circles at different radii (r₀, r₁, r₂, r₃) — showing its predicted position/uncertainty growing over time
   - Static obstacles O^s are still present at the base
   - Label: **"A → G Succeeds"**

3. **Top-Right — "SANDO2" 2D flattened MIQP view at t₀**:
   - A 2D plot with **x-axis** = spatial progress along trajectory (showing A, x₁, x₂, x₃, G), **y-axis** = time index **n** (0, 1, 2, 3)
   - The corridors C[i][j] are drawn as colored parallelogram regions in this (progress, n) space
   - Shows the **MIQP assignment**: which control point belongs to which corridor
   - A box at the bottom listing: x₀ ⊆ C[0][0], x₁ ⊆ C[1][1], x₂ ⊆ C[2][1], x₃ ⊆ C[3][2]

### Bottom Row (time t = t₁, dynamic obstacle has MOVED — replan needed):

4. **Bottom-Left — "Other Approaches" at t₁**:
   - Same 2D scene but the dynamic obstacle O^d has **moved** (shifted position)
   - New adjusted start A' and goal G' are shown
   - The old spatial-only corridors **no longer contain a valid path** because they didn't account for the obstacle's future motion
   - Label: **"Replan A' → G' Fails"** (in red, indicating failure)

5. **Bottom-Middle — "SANDO2" 3D view at t₁**:
   - Same 3D spatio-temporal view but with **new corridors** C'[i][j] computed for the replan
   - The dynamic obstacle has moved, but the spatio-temporal corridors adapt because they encode time
   - New trajectory with control points x'₀, x'₁, x'₂, x'₃ and a waypoint W'₁
   - Label: **"Replan A' → G' Succeeds"** (showing SANDO2 handles it)

6. **Bottom-Right — "SANDO2" 2D MIQP view at t₁**:
   - Updated MIQP assignment view with new corridors C'[i][j]

### Legend:
- Colored parallelograms = Polytope (Safe Flight Corridor)
- Colored dots with curve = Trajectory with control points
- Tan blobs = Static Obst. (O^s)
- Orange circle = Dynamic Obst. (O^d)

---

## Animation Sequence

Break the video into these scenes. Each scene should be a separate Manim `Scene` (or `ThreeDScene`) class so they can be rendered individually and concatenated.

### Scene 1: "The Problem Setup" (~10-15 seconds)
1. Fade in a clean 2D coordinate plane (x, y axes)
2. Draw the two static obstacles O^s (tan/brown filled irregular rounded shapes)
3. Animate the dynamic obstacle O^d appearing — a large orange filled circle, with a small arrow showing its velocity direction
4. Place start point **A** (labeled dot, bottom-right) and goal **G** (labeled dot, top)
5. Draw a smooth blue trajectory curve from A to G with control points x₀, x₁, x₂, x₃ appearing as labeled dots
6. Text label: "Planning a safe trajectory from A to G"

### Scene 2: "Spatial-Only Corridors (Other Approaches)" (~10 seconds)
1. Starting from Scene 1's final state
2. Animate **blue parallelogram polytopes** appearing one-by-one around the trajectory — these are flat 2D spatial corridors
3. Show them overlapping slightly to form a connected corridor chain
4. Text: "Other Approaches: Spatial-only Safe Flight Corridors"
5. Green text: **"A → G Succeeds ✓"**

### Scene 3: "But What Happens When the Obstacle Moves?" (~10 seconds)
1. Animate the dynamic obstacle O^d **moving** to a new position (smooth translation)
2. The old trajectory now **intersects** the obstacle's new position — highlight collision with red flash or X
3. The spatial corridors also intersect the obstacle
4. Text: "Dynamic obstacle moves → Spatial corridors become invalid!"
5. Red text: **"Replan A' → G' Fails ✗"**

### Scene 4: "SANDO2's Solution: Spatio-Temporal SFC" (~15-20 seconds) ← MOST IMPORTANT SCENE
1. **Transition from 2D to 3D**: Start from the 2D scene and smoothly **extrude upward** along a new vertical **time axis (n)**
   - Camera rotates from top-down 2D view to angled 3D perspective
   - As the camera rotates, the time axis appears with labels n = 0, 1, 2, 3
2. Show the spatio-temporal corridors **building up layer by layer**:
   - First, magenta polytopes appear at n=0 (ground plane)
   - Then red polytopes grow/appear at n=1
   - Then green at n=2
   - Then blue at n=3
   - Each is a 3D parallelotope at its respective time layer
3. Show the dynamic obstacle's **predicted future positions** as concentric circles growing at each time layer (r₀ < r₁ < r₂ < r₃), illustrating increasing uncertainty over time
4. Draw the 3D trajectory curve threading through the corridors from bottom (A, n=0) to top (G, n=3)
5. Text: "SANDO2: Spatio-Temporal Safe Flight Corridors avoid future obstacle positions"

### Scene 5: "The 2D MIQP Assignment View" (~10 seconds)
1. Transition/split to show the flattened 2D (progress × time) view
2. Draw corridor regions C[i][j] as colored parallelograms in (progress, n) space
3. Animate control points being **assigned** to corridors — dots snapping into their respective regions
4. Show assignment text: x₀ ⊆ C[0][0], x₁ ⊆ C[1][1], etc.
5. Text: "MIQP optimally assigns control points to corridors"

### Scene 6: "Replanning Succeeds with SANDO2" (~10 seconds)
1. Back to 3D view
2. Animate dynamic obstacle moving (same motion as Scene 3)
3. Old corridors fade out, NEW spatio-temporal corridors C'[i][j] fade in
4. New trajectory threads through new corridors successfully
5. Green text: **"Replan A' → G' Succeeds ✓"**
6. Final: "Spatio-temporal corridors adapt to dynamic environments"

---

## Technical Specifications

- **Resolution**: 1920×1080 (YouTube standard HD)
- **Frame rate**: 30 fps
- **Background**: White (`#FFFFFF`)
- **Color scheme** (matching the paper figure):
  - Static obstacles: `#D4956A` (tan/brown) with slight opacity
  - Dynamic obstacle: `#F5A623` (orange) with ~0.5 opacity fill
  - Polytopes at n=0: Magenta `#FF00FF` with ~0.2 fill opacity, solid stroke
  - Polytopes at n=1: Red `#FF0000` with ~0.2 fill opacity, solid stroke
  - Polytopes at n=2: Green `#00AA00` with ~0.2 fill opacity, solid stroke
  - Polytopes at n=3: Blue `#0000FF` with ~0.2 fill opacity, solid stroke
  - Trajectory segments: colored to match the corridor they pass through
  - Control points: colored dots matching their corridor color
  - Text/labels: Black `#000000`
  - Failure text: Red `#CC0000`
  - Success text: Green `#00AA00`
- **Font**: Use `MathTex` / `Tex` for mathematical labels ($x_0$, $\mathcal{O}^s$, $\mathcal{O}^d$, $C[i][j]$)
- **Style**: Clean, academic, 3Blue1Brown-inspired but with white background. Smooth, elegant transitions. No flashy effects.

## Manim Config (put in manim.cfg or pass as CLI flags)

```
[CLI]
background_color = WHITE
pixel_height = 1080
pixel_width = 1920
frame_rate = 30
```

Also note: since background is white, all default text/axes should be black. Set `Tex.set_default(color=BLACK)` and similar at the top.

## 3D Camera Notes (for Scene 4)

- Use `ThreeDScene` with `self.set_camera_orientation(phi, theta, gamma)`
- Start with `phi=0°` (top-down, showing 2D x-y plane)
- Smoothly animate to `phi=65°`, `theta=-60°` to reveal the time axis going up
- Time axis (n) = Z direction (upward in 3D)
- Use `self.move_camera()` or `self.begin_ambient_camera_rotation()` for smooth transitions

## Code Organization

```
sando_animation/
├── scenes/
│   ├── scene1_problem_setup.py
│   ├── scene2_spatial_corridors.py
│   ├── scene3_obstacle_moves.py
│   ├── scene4_spatiotemporal_3d.py    ← start here, most important
│   ├── scene5_miqp_assignment.py
│   └── scene6_replan_success.py
├── utils/
│   ├── obstacles.py          # Helper functions for drawing obstacles
│   ├── corridors.py          # Helper functions for polytope corridors
│   ├── trajectory.py         # Helper functions for trajectory curves
│   └── config.py             # Colors, positions, shared constants
├── manim.cfg
├── render_all.sh             # Script to render all scenes
└── README.md
```

## Implementation Priority

**Start with Scene 4** (the 2D → 3D transition) since it's the most important and technically challenging. Get that working first. Then build Scenes 1-3 (the 2D setup and failure case), and finally Scenes 5-6.

## Manim Tips

- For 3D polytopes: create each face as a `Polygon3D` or use `Prism`/`Surface`. Group faces into a `VGroup` for each corridor.
- For smooth trajectory curves: use `ParametricFunction` or `CubicBezier`
- For the 2D→3D camera transition: start with `phi=0, theta=-90` (top-down), animate to `phi=65, theta=-60`
- Use `FadeIn`, `Create`, `DrawBorderThenFill`, `Transform`, `MoveToTarget`
- Use `AnimationGroup` or `LaggedStart` for staggered corridor appearances
- Use `self.wait(1)` between major steps for breathing room
- For white background, explicitly set `color=BLACK` on axes, labels, and other default-colored elements
- Keep polytope vertex coordinates as configurable constants in `config.py` so positions can be tuned

## Rendering

Individual scenes:
```bash
manim -qh scenes/scene4_spatiotemporal_3d.py SpatioTemporalSFC
```

All scenes (render_all.sh):
```bash
#!/bin/bash
for scene in scenes/scene*.py; do
    manim -qh "$scene"
done
```

Final concatenation (with ffmpeg):
```bash
ffmpeg -f concat -i filelist.txt -c copy sando_explainer.mp4
```
