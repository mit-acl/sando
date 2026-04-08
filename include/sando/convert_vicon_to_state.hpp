/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <sando/sando.hpp>
#include "rclcpp/rclcpp.hpp"
#include "dynus_interfaces/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "std_msgs/msg/header.hpp"

// prefix
using namespace std::chrono_literals;

// Define the synchronization policy
typedef message_filters::sync_policies::
    ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped>
        MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

/** @brief ROS 2 node that fuses time-synchronized PoseStamped and TwistStamped into a
 * dynus_interfaces/State. */
class PoseTwistToStateNode : public rclcpp::Node {
 public:
  /** @brief Construct the node with synchronized pose/twist subscribers and a state publisher. */
  PoseTwistToStateNode();

 private:
  void callback(
      const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& pose_msg,
      const std::shared_ptr<const geometry_msgs::msg::TwistStamped>& twist_msg);

  // Syncronized subscription
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_pose_;
  message_filters::Subscriber<geometry_msgs::msg::TwistStamped> sub_twist_;

  // Time synchronizer
  std::shared_ptr<Sync> pose_twist_sync_;

  // Publisher
  rclcpp::Publisher<dynus_interfaces::msg::State>::SharedPtr pub_state_;
};
