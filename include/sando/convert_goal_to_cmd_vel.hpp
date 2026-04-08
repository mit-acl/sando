/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <sando/sando.hpp>
#include "rclcpp/rclcpp.hpp"
#include "dynus_interfaces/msg/goal.hpp"
#include "dynus_interfaces/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

/** @brief ROS 2 node that converts SANDO goal setpoints into cmd_vel Twist messages for ground
 * robots. */
class GoalToCmdVel : public rclcpp::Node {
 public:
  /** @brief Construct the node, load control gains, and set up ROS interfaces. */
  GoalToCmdVel();

 private:
  void stateCallback(const dynus_interfaces::msg::State::SharedPtr msg);
  void goalCallback(const dynus_interfaces::msg::Goal::SharedPtr msg);
  void cmdVelCallback();
  double wrapPi(double x);

  // State and Goal
  dynus_interfaces::msg::State state_;
  dynus_interfaces::msg::Goal goal_;
  double current_roll_;
  double current_pitch_;
  double current_yaw_;

  // Publishers and Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<dynus_interfaces::msg::Goal>::SharedPtr sub_goal_;
  rclcpp::Subscription<dynus_interfaces::msg::State>::SharedPtr sub_state_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  // Control parameters
  double kv_;
  double kdist_;
  double kw_;
  double kyaw_;
  double kalpha_;
  double kx_;
  double ky_;
  double eps_;

  // Flags
  bool state_initialized_;
  bool goal_initialized_;
};
