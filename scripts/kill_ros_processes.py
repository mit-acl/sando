# ----------------------------------------------------------------------------
# Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
import os
import subprocess

# The process to exclude — matches VS Code crashpad handler regardless of user
EXCLUDE_PROCESS = "chrome_crashpad_handler"


def get_ros_processes():
    """Get a list of ROS-related processes."""
    try:
        # Run `ps aux | grep ros`
        result = subprocess.run(["ps", "aux"], stdout=subprocess.PIPE, text=True)
        processes = result.stdout.splitlines()
        # return [line for line in processes if 'ros' in line and EXCLUDE_PROCESS not in line]
        return [
            line
            for line in processes
            if ("ros" in line and "Microsoft" not in line) or ("gazebo" in line)
        ]
    except subprocess.CalledProcessError as e:
        print(f"Error executing ps aux: {e}")
        return []


def kill_process(pid):
    """Kill a process by its PID."""
    try:
        subprocess.run(["kill", "-9", str(pid)], check=True)
        print(f"Successfully killed process with PID: {pid}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to kill process with PID {pid}: {e}")


def main():

    ros_processes = get_ros_processes()
    print(f"Found {len(ros_processes)} ROS processes")
    for process in ros_processes:
        # Extract PID (2nd column in `ps aux` output)
        parts = process.split()
        if len(parts) > 1:
            pid = parts[1]
            if "kill_ros_processes" in parts[10] or "kill_ros_processes" in parts[11]:
                continue
            print(
                f"Found ROS process to kill: PID={pid}, Command={' '.join(parts[10:])}"
            )
            kill_process(pid)


if __name__ == "__main__":
    main()
