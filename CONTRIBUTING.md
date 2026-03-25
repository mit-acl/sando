# Contributing to SANDO

Thank you for your interest in contributing to SANDO. This guide covers the essentials.

## Development Environment

Set up your workspace using one of the methods described in the README:

- **Docker (recommended):** Use the provided Dockerfile for a reproducible environment.
- **Native install:** Ubuntu 22.04 with ROS 2 Humble. Requires Gurobi, Eigen3, PCL, and OpenMP.

## Code Style

- **C++:** Google style, enforced by `.clang-format`. Run `clang-format -i <file>` before committing.
- **Python:** PEP 8, enforced by `ruff`. Run `ruff format <file>` and `ruff check <file>`.
- Use `snake_case` for packages, files, variables, and functions. `CamelCase` for classes. `ALL_CAPS` for constants.
- Member variables use a trailing underscore (e.g., `trajectory_`).

## Pre-commit Hooks

A `.pre-commit-config.yaml` is provided. Install hooks with:

```bash
pip install pre-commit
pre-commit install
```

This will automatically check formatting and style on each commit.

## Building and Testing

```bash
cd ~/code/dynus_ws
colcon build --packages-select sando --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Run the planner in RViz-only mode to verify basic functionality after your changes.

## Bug Reports

Open a GitHub issue with:

1. A clear description of the problem
2. Steps to reproduce (launch commands, parameter files, environment)
3. Expected vs. actual behavior
4. ROS 2 log output and any relevant error messages
5. Your OS, ROS 2 distro, and Gurobi version

## Feature Requests

Open a GitHub issue with the `enhancement` label. Describe the use case and why existing functionality does not cover it.

## Pull Request Process

1. Fork the repository and create a feature branch from `main`.
2. Keep commits focused -- one logical change per commit.
3. Ensure `colcon build --packages-select sando` succeeds with no warnings.
4. Run pre-commit hooks and fix any issues.
5. Test your changes (simulation, benchmarks, or unit tests as appropriate).
6. Open a pull request against `main` with a clear description of what changed and why.
7. Address review feedback promptly.

Do not modify message/service definitions in `msg/`, `srv/`, or `action/` without prior discussion.

## Licensing

By contributing, you agree that your contributions will be licensed under the
[BSD 3-Clause License](LICENSE), the same license as the rest of the project.
