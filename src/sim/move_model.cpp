/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sim/exprtk.hpp>
#include <thread>

// ROS2 includes for TF broadcasting
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo {
class ModelMove : public ModelPlugin {
  typedef exprtk::symbol_table<double> symbol_table_t;
  typedef exprtk::expression<double> expression_t;
  typedef exprtk::parser<double> parser_t;

 public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
    std::cout << "Loading ModelMove plugin" << std::endl;
    model_ = _parent;
    world_ = _parent->GetWorld();

    if (!_sdf->HasElement("traj_x") || !_sdf->HasElement("traj_y") || !_sdf->HasElement("traj_z")) {
      std::cout << "Missing fields (traj_x, traj_y or traj_z), ABORTING!" << std::endl;
      abort();
    }

    // Initialize the exprtk symbol table and compile trajectory expressions.
    symbol_table_t symbol_table;
    symbol_table.add_variable("t", t_);
    symbol_table.add_constants();
    expression_t expression;
    expression.register_symbol_table(symbol_table);

    parser_t parser;
    std::string string_x = _sdf->Get<std::string>("traj_x");
    std::string string_y = _sdf->Get<std::string>("traj_y");
    std::string string_z = _sdf->Get<std::string>("traj_z");

    parser.compile(string_x, expression);
    traj_compiled_.push_back(expression);
    parser.compile(string_y, expression);
    traj_compiled_.push_back(expression);
    parser.compile(string_z, expression);
    traj_compiled_.push_back(expression);

    // Check if TF publishing is requested via SDF parameter
    if (_sdf->HasElement("publish_tf")) {
      publish_tf_ = _sdf->Get<bool>("publish_tf");
    }

    // Only create ROS2 node + TF broadcaster when needed
    if (publish_tf_) {
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      std::string nodeName = "model_move_tf_" + model_->GetName();
      rosnode_ = rclcpp::Node::make_shared(nodeName);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(rosnode_);

      spinThread_ = std::thread([this]() { rclcpp::spin(this->rosnode_); });
    }

    // Connect to the world update event.
    updateConnection =
        event::Events::ConnectWorldUpdateBegin(std::bind(&ModelMove::OnUpdate, this));
  }

  // Called at every simulation iteration.
  void OnUpdate() {
    // Use wall clock time instead of sim time.
    t_ = gazebo::common::Time::GetWallTime().Double();

    // Evaluate the trajectory expressions.
    double x_val = traj_compiled_[0].value();
    double y_val = traj_compiled_[1].value();
    double z_val = traj_compiled_[2].value();

    // Build the new pose.
    ignition::math::Pose3d pose(x_val, y_val, z_val, 0.0, 0.0, 0.0);
    model_->SetWorldPose(pose);

    // --- Publish a TF transform ---
    if (!publish_tf_) {
      // If publish_tf_ is false, skip TF broadcasting.
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    // Use the ROS2 node's clock for the header stamp.
    tf_msg.header.stamp = rosnode_->now();
    tf_msg.header.frame_id = "map";  // Or whatever the fixed frame should be.
    // Set child frame id to the model's name and link, e.g. "model_name/base_link"
    tf_msg.child_frame_id = model_->GetName();
    tf_msg.transform.translation.x = pose.Pos().X();
    tf_msg.transform.translation.y = pose.Pos().Y();
    tf_msg.transform.translation.z = pose.Pos().Z();
    ignition::math::Quaterniond quat = pose.Rot();
    tf_msg.transform.rotation.x = quat.X();
    tf_msg.transform.rotation.y = quat.Y();
    tf_msg.transform.rotation.z = quat.Z();
    tf_msg.transform.rotation.w = quat.W();

    tf_broadcaster_->sendTransform(tf_msg);
  }

 private:
  // Gazebo pointers.
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  double t_ = 0.0;
  std::vector<expression_t> traj_compiled_;
  event::ConnectionPtr updateConnection;
  bool publish_tf_ = false;

  // ROS2 node and TF broadcaster.
  rclcpp::Node::SharedPtr rosnode_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::thread spinThread_;
};

// Register the plugin with Gazebo.
GZ_REGISTER_MODEL_PLUGIN(ModelMove)
}  // namespace gazebo
