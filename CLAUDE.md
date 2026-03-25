# CLAUDE.md

## Project
SANDO - Safe Autonomous Trajectory Planning for Dynamic Unknown Environments
Language: C++17, Python 3, ROS2 Humble
Build: colcon / CMake

## Build
```bash
cd ~/code/dynus_ws
colcon build --packages-select sando --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DCMAKE_C_COMPILER=/usr/bin/gcc
```

## Style
- C++: Google style via .clang-format (already applied)
- Python: ruff-formatted, PEP8
- ROS2 naming: snake_case for packages, files, variables, functions
- CamelCase for class names, ALL_CAPS for constants
- Member variables: trailing underscore (e.g., `trajectory_`)

## Cleanup Rules
- Remove all TODO/FIXME/HACK comments that are resolved
- Remove commented-out code blocks entirely (we have git history)
- Keep comments that explain *why*, remove comments that explain *what*
- Standardize header guards to #pragma once
- Group #includes: system → ROS → project, alphabetical within groups
- Do NOT rename public API functions/topics/services (breaks users)
- Do NOT change any algorithm logic — cosmetic only
- Ensure every file has a license header

## Do Not Touch
- msg/ srv/ action/ definitions
- third_party/ or external dependencies
- include/sim/exprtk.hpp (third-party expression parser)
- include/hgp/termcolor.hpp (third-party terminal colors)
- include/hgp/data_type.hpp and data_utils.hpp use intentional `#ifndef` guards
  (shared with decomp_util to prevent redefinition — do NOT convert to `#pragma once`)