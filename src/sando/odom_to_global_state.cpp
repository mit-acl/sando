/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

// Transforms DLIO local-frame odometry into global-frame State and PoseStamped
// using the TF2 lookup of world -> {veh}/init_pose.

#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dynus_interfaces/msg/state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomToGlobalState : public rclcpp::Node {
 public:
  OdomToGlobalState() : Node("odom_to_global_state") {
    // Derive vehicle name from namespace (e.g. "/PX04" -> "PX04")
    std::string ns = this->get_namespace();
    veh_name_ = ns.substr(ns.find_last_of("/") + 1);

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers
    state_pub_ = this->create_publisher<dynus_interfaces::msg::State>("state", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 10);

    // Subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&OdomToGlobalState::odomCallback, this, std::placeholders::_1));

    // Timer to poll for TF (100ms)
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&OdomToGlobalState::pollTF, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for TF: world -> %s/init_pose", veh_name_.c_str());
  }

 private:
  void pollTF() {
    std::string target_frame = "world";
    std::string source_frame = veh_name_ + "/init_pose";

    if (!tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero)) return;

    try {
      auto tf = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

      // Cache translation
      t_init_ = Eigen::Vector3d(
          tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);

      // Cache rotation
      q_init_ = Eigen::Quaterniond(
          tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y,
          tf.transform.rotation.z);
      R_init_ = q_init_.toRotationMatrix();

      // Extract yaw offset from quaternion
      Eigen::Vector3d euler = R_init_.eulerAngles(2, 1, 0);  // ZYX
      yaw_offset_ = euler[0];

      tf_acquired_ = true;
      tf_timer_->cancel();

      RCLCPP_INFO(
          this->get_logger(), "TF acquired: t=[%.3f, %.3f, %.3f], yaw=%.3f rad", t_init_.x(),
          t_init_.y(), t_init_.z(), yaw_offset_);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!tf_acquired_) return;

    // Local position
    Eigen::Vector3d local_pos(
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // Local velocity (body-frame twist from DLIO)
    Eigen::Vector3d local_vel(
        msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    // Local quaternion
    Eigen::Quaterniond local_quat(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    // Transform to global frame
    Eigen::Vector3d global_pos = R_init_ * local_pos + t_init_;
    Eigen::Vector3d global_vel = R_init_ * local_vel;
    Eigen::Quaterniond global_quat = q_init_ * local_quat;
    global_quat.normalize();

    auto stamp = msg->header.stamp;

    // Publish State
    {
      auto state_msg = dynus_interfaces::msg::State();
      state_msg.header.stamp = stamp;
      state_msg.header.frame_id = "world";
      state_msg.pos.x = global_pos.x();
      state_msg.pos.y = global_pos.y();
      state_msg.pos.z = global_pos.z();
      state_msg.vel.x = global_vel.x();
      state_msg.vel.y = global_vel.y();
      state_msg.vel.z = global_vel.z();
      state_msg.quat.x = global_quat.x();
      state_msg.quat.y = global_quat.y();
      state_msg.quat.z = global_quat.z();
      state_msg.quat.w = global_quat.w();
      state_pub_->publish(state_msg);
    }

    // Publish PoseStamped
    {
      auto pose_msg = geometry_msgs::msg::PoseStamped();
      pose_msg.header.stamp = stamp;
      pose_msg.header.frame_id = "world";
      pose_msg.pose.position.x = global_pos.x();
      pose_msg.pose.position.y = global_pos.y();
      pose_msg.pose.position.z = global_pos.z();
      pose_msg.pose.orientation.x = global_quat.x();
      pose_msg.pose.orientation.y = global_quat.y();
      pose_msg.pose.orientation.z = global_quat.z();
      pose_msg.pose.orientation.w = global_quat.w();
      pose_pub_->publish(pose_msg);
    }
  }

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  bool tf_acquired_ = false;

  // Cached transform
  Eigen::Vector3d t_init_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_init_ = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d R_init_ = Eigen::Matrix3d::Identity();
  double yaw_offset_ = 0.0;

  // Vehicle name
  std::string veh_name_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<dynus_interfaces::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToGlobalState>());
  rclcpp::shutdown();
  return 0;
}
