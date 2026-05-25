#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

"""
Spawn a physics-enabled cube in Gazebo and launch it toward the SANDO drone.

The box is NOT published on /trajs, so SANDO must detect it via its onboard
sensor (depth camera / LiDAR) and react. Run after the drone is hovering.

Default mode: gravity is disabled on the box, so it moves at constant velocity
straight at the drone (guaranteed hit while the drone hovers).

With --gravity, the box is treated as a real ballistic projectile. The launch
direction is solved from the requested speed so the parabola passes through
the drone's current position; if the speed is below the minimum needed to
reach the drone, the script errors out and reports the minimum.

Usage (with a SANDO Gazebo sim already running, no goal sent):
    python3 src/sando/scripts/throw_box.py
    python3 src/sando/scripts/throw_box.py --speed 8.0 --gravity
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node

from dynus_interfaces.msg import State
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity


GRAVITY = 9.81

BOX_SDF_TEMPLATE = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <link name='link'>
      <gravity>{gravity_flag}</gravity>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry><box><size>{s} {s} {s}</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>{s} {s} {s}</size></box></geometry>
        <material>
          <ambient>0.9 0.2 0.2 1</ambient>
          <diffuse>0.9 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def solve_ballistic_velocity(spawn, target, speed, g=GRAVITY):
    """Compute (vx, vy, vz) so a projectile from spawn at given speed hits target.

    Returns (vx, vy, vz, t) for the lower-angle (faster) solution, or None if
    speed is below the minimum required.
    """
    sx_, sy_, sz_ = spawn
    tx, ty, tz = target
    dxh, dyh = tx - sx_, ty - sy_
    d_h = math.hypot(dxh, dyh)
    dz = tz - sz_

    s2 = speed * speed
    # Minimum speed: s_min^2 = g * (dz + sqrt(dz^2 + d_h^2))
    s_min_sq = g * (dz + math.hypot(dz, d_h))
    if s2 < s_min_sq - 1e-9:
        return None, math.sqrt(max(s_min_sq, 0.0))

    # Solve a*u^2 + b*u + c = 0 with u = tan(theta).
    a = g * d_h * d_h / (2.0 * s2)
    b = -d_h
    c = dz + g * d_h * d_h / (2.0 * s2)
    disc = max(b * b - 4.0 * a * c, 0.0)
    sqrt_disc = math.sqrt(disc)
    # Two solutions; lower angle => smaller tan(theta) => faster TOF.
    u_lo = (-b - sqrt_disc) / (2.0 * a) if a > 1e-12 else (-c / b if b else 0.0)
    theta = math.atan(u_lo)
    v_h = speed * math.cos(theta)
    v_v = speed * math.sin(theta)
    h_hat = (dxh / d_h, dyh / d_h) if d_h > 1e-9 else (1.0, 0.0)
    vx = v_h * h_hat[0]
    vy = v_h * h_hat[1]
    vz = v_v
    t_flight = d_h / v_h if v_h > 1e-9 else float("inf")
    return (vx, vy, vz, t_flight), math.sqrt(s_min_sq)


class BoxThrower(Node):
    def __init__(self, args):
        super().__init__("throw_box")
        self.args = args
        self.state = None
        self.create_subscription(State, args.state_topic, self._state_cb, 10)
        self.spawn_cli = self.create_client(SpawnEntity, args.spawn_service)
        self.set_state_cli = self.create_client(SetEntityState, args.set_state_service)

    def _state_cb(self, msg):
        self.state = msg

    def wait_for_state(self, timeout=10.0):
        t0 = time.time()
        while rclpy.ok() and self.state is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.state is not None

    def wait_for_services(self, timeout=10.0):
        return self.spawn_cli.wait_for_service(
            timeout_sec=timeout
        ) and self.set_state_cli.wait_for_service(timeout_sec=timeout)

    def throw(self):
        a = self.args
        drone = self.state.pos
        az = math.radians(a.azimuth_deg)
        sx = drone.x + a.offset * math.cos(az)
        sy = drone.y + a.offset * math.sin(az)
        sz = drone.z + a.height_offset

        if a.gravity:
            sol, s_min = solve_ballistic_velocity(
                (sx, sy, sz), (drone.x, drone.y, drone.z), a.speed
            )
            if sol is None:
                self.get_logger().error(
                    f"Speed {a.speed:.2f} m/s is below the minimum "
                    f"{s_min:.2f} m/s needed to reach the drone with gravity. "
                    f"Increase --speed or disable gravity."
                )
                return False
            vx, vy, vz, t_flight = sol
            self.get_logger().info(f"Ballistic flight time: {t_flight:.2f}s")
        else:
            # Constant velocity straight at the drone (gravity off).
            dx, dy, dz = drone.x - sx, drone.y - sy, drone.z - sz
            n = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
            vx, vy, vz = a.speed * dx / n, a.speed * dy / n, a.speed * dz / n

        s, m = a.size, a.mass
        # Solid cube inertia about its center: (1/6) m s^2
        ixx = m * s * s / 6.0
        sdf = BOX_SDF_TEMPLATE.format(
            name=a.name, mass=m, s=s, i=ixx, gravity_flag=1 if a.gravity else 0
        )

        req = SpawnEntity.Request()
        req.name = a.name
        req.xml = sdf
        req.initial_pose.position.x = sx
        req.initial_pose.position.y = sy
        req.initial_pose.position.z = sz
        self.get_logger().info(
            f"Spawning '{a.name}' at ({sx:.2f}, {sy:.2f}, {sz:.2f}); "
            f"drone at ({drone.x:.2f}, {drone.y:.2f}, {drone.z:.2f}); "
            f"v=({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s "
            f"gravity={'on' if a.gravity else 'off'}"
        )
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().error(f"spawn_entity failed: {res}")
            return False

        set_req = SetEntityState.Request()
        set_req.state = EntityState()
        set_req.state.name = a.name
        set_req.state.pose.position.x = sx
        set_req.state.pose.position.y = sy
        set_req.state.pose.position.z = sz
        set_req.state.pose.orientation.w = 1.0
        set_req.state.twist.linear.x = vx
        set_req.state.twist.linear.y = vy
        set_req.state.twist.linear.z = vz
        set_req.state.reference_frame = "world"

        fut2 = self.set_state_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, fut2, timeout_sec=5.0)
        res2 = fut2.result()
        if res2 is None or not res2.success:
            self.get_logger().error(f"set_entity_state failed: {res2}")
            return False

        self.get_logger().info("Box launched.")
        return True


def main():
    p = argparse.ArgumentParser(
        description="Throw a physics-enabled box at the SANDO drone in Gazebo."
    )
    p.add_argument("--name", default="thrown_box")
    p.add_argument("--mass", type=float, default=1.0, help="Box mass [kg]")
    p.add_argument("--size", type=float, default=0.3, help="Cube edge length [m]")
    p.add_argument(
        "--offset", type=float, default=5.0, help="Spawn distance from drone [m]"
    )
    p.add_argument(
        "--azimuth-deg",
        type=float,
        default=0.0,
        help="Spawn bearing from drone in xy-plane (deg, world frame). 0 = +x.",
    )
    p.add_argument(
        "--height-offset",
        type=float,
        default=0.0,
        help="Vertical offset of spawn point above drone [m]",
    )
    p.add_argument("--speed", type=float, default=1.0, help="Throw speed [m/s]")
    p.add_argument(
        "--gravity",
        action="store_true",
        help="Enable gravity on the box (ballistic). Default: gravity off, "
        "constant velocity straight at the drone.",
    )
    p.add_argument(
        "--state-topic",
        default="/NX01/state",
        help="dynus_interfaces/State topic for the drone",
    )
    p.add_argument(
        "--spawn-service",
        default="/spawn_entity",
        help="gazebo_ros SpawnEntity service name",
    )
    p.add_argument(
        "--set-state-service",
        default="/plug/set_entity_state",
        help="gazebo_ros SetEntityState service name "
        "(SANDO worlds load gazebo_ros_state under '/plug')",
    )
    args = p.parse_args()

    rclpy.init()
    node = BoxThrower(args)
    try:
        if not node.wait_for_services(10.0):
            node.get_logger().error(
                "Gazebo services /spawn_entity or /set_entity_state not available."
            )
            sys.exit(1)
        if not node.wait_for_state(10.0):
            node.get_logger().error(f"No State received on {args.state_topic}.")
            sys.exit(1)
        ok = node.throw()
        sys.exit(0 if ok else 2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
