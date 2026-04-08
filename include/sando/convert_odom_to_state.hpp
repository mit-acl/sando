/* ----------------------------------------------------------------------------
 * Copyright (c) Anonymous Author
 * Anonymous Institution
 * All Rights Reserved
 * Authors: Anonymous
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <sando_interfaces/msg/state.hpp>
#include "sando/sando.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

/** @brief ROS 2 node that converts nav_msgs/Odometry messages into sando_interfaces/State messages.
 */
class OdometryToStateNode : public rclcpp::Node {
 public:
  /** @brief Construct the node and set up the odometry subscriber and state publisher. */
  OdometryToStateNode();

 private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<sando_interfaces::msg::State>::SharedPtr state_publisher_;
};