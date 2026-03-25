# C++ Code Cleanup

Perform a comprehensive C++ code cleanup on the SANDO project following CLAUDE.md rules. Execute each phase sequentially, building and verifying between phases. Spawn reviewer agents (at least 2 per phase) to catch issues before moving on.

## Scope

- All `.cpp` files in `src/hgp/`, `src/sando/`, `src/sim/`, `src/tools/`
- All `.hpp` files in `include/hgp/`, `include/sando/`, `include/timer.hpp`
- **Exclude third-party:** `include/sim/exprtk.hpp`, `include/hgp/termcolor.hpp`
- **Exclude external-license:** `src/sim/gazebo_ros_imu_sensor.cpp`, `include/sim/gazebo_ros_imu_sensor.hpp` (Apache)
- **Special guards:** `include/hgp/data_type.hpp` and `include/hgp/data_utils.hpp` use intentional `#ifndef` guards shared with decomp_util — do NOT convert to `#pragma once`

## Build Command

```bash
cd ~/code/dynus_ws && colcon build --packages-select sando --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DCMAKE_C_COMPILER=/usr/bin/gcc
```

## Phase 1: Comment Cleanup

For each in-scope file:
1. Remove all commented-out code blocks (multi-line `//` or `/* */` around code statements)
2. Remove stale/resolved TODO/FIXME/HACK comments — keep TODOs that describe genuine future work
3. Keep comments that explain **why** (algorithmic decisions, non-obvious choices)
4. Do NOT change any algorithm logic

Build and verify. Spawn 2 reviewer agents to check for missed blocks and accidental logic changes.

## Phase 2: Include Hygiene & License Headers

1. Ensure all headers have `#pragma once` right after the license header (except data_type.hpp and data_utils.hpp)
2. Ensure every file has the standard license header at the top:
```
/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */
```
3. Remove duplicate header guards (files with both `#pragma once` and `#ifndef`)

Build and verify. Spawn 2 reviewer agents.

## Phase 3: Doxygen Cleanup

1. **Remove** all Doxygen `/** ... */` blocks from `.cpp` files — documentation belongs in headers only
2. **Add** Doxygen `@brief`/`@param`/`@return` to all public class methods and free functions in `.hpp` files that are missing them
3. Add `@brief` to class/struct declarations that are missing it
4. Do NOT modify function bodies or signatures

Build and verify. Spawn 2 reviewer agents.

## Phase 4: Dead Code Removal

1. For each public function declared in a `.hpp` file, grep for callers across the entire codebase
2. If a function is declared + defined but never called anywhere, remove both the declaration and definition
3. Build after each removal to verify nothing breaks
4. Report what was removed

Build and verify. Spawn 2 reviewer agents.

## Review Process

Between each phase, spawn 2-3 reviewer agents that check:
- Build succeeds
- No algorithm logic was changed (diff review)
- No public API topics/services were renamed
- Phase-specific completeness (no missed items)

Iterate on reviewer feedback before proceeding to the next phase.
