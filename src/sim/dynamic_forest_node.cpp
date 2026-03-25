/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <dynus_interfaces/msg/dyn_traj.hpp>
#include <gazebo_msgs/msg/model_state.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sim/exprtk.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using json = nlohmann::json;
using dynus_interfaces::msg::DynTraj;

struct ObstacleSpec {
  std::string model_name;
  std::string traj_x;
  std::string traj_y;
  std::string traj_z;
  std::string traj_vx;
  std::string traj_vy;
  std::string traj_vz;

  // Original spawn (center) – may be adopted
  double x0{0.0}, y0{0.0}, z0{0.0};

  // Cached evaluated parameters (if you want)
  double t_var{0.0};

  // exprtk plumbing
  exprtk::symbol_table<double> symbol_table;
  exprtk::expression<double> expr_x;
  exprtk::expression<double> expr_y;
  exprtk::expression<double> expr_z;
  bool compiled{false};

  // Optional bounding box (placeholder)
  std::array<double, 3> bbox{1.0, 1.0, 1.0};

  void compile() {
    // Ensure clean slate each time (in case of recompile)
    symbol_table.clear();
    symbol_table.add_variable("t", t_var);
    symbol_table.add_constants();

    expr_x = exprtk::expression<double>();
    expr_y = exprtk::expression<double>();
    expr_z = exprtk::expression<double>();

    expr_x.register_symbol_table(symbol_table);
    expr_y.register_symbol_table(symbol_table);
    expr_z.register_symbol_table(symbol_table);

    exprtk::parser<double> parser;

    auto compile_one = [&](const std::string& label, const std::string& src,
                           exprtk::expression<double>& expr) {
      if (!parser.compile(src, expr)) {
        std::ostringstream oss;
        oss << "Failed to compile " << label << "='" << src << "' errors:";
        for (std::size_t i = 0; i < parser.error_count(); ++i) {
          auto e = parser.get_error(i);
          oss << " [pos " << e.token.position << " type " << exprtk::parser_error::to_str(e.mode)
              << " msg '" << e.diagnostic << "']";
        }
        throw std::runtime_error(oss.str());
      }
    };

    compile_one("traj_x", traj_x, expr_x);
    compile_one("traj_y", traj_y, expr_y);
    compile_one("traj_z", traj_z, expr_z);

    // Collect variable names (to confirm 't' recognized)
    // NOTE: In standard exprtk, we can inspect via symbol_table.list_all_symbols()
    std::vector<std::string> vars;
    symbol_table.get_variable_list(vars);
    bool has_t = false;
    for (auto& v : vars)
      if (v == "t") has_t = true;

    if (!has_t) {
      throw std::runtime_error(
          "Compiled expression does not reference variable 't' (constant expression?)");
    }

    compiled = true;
  }

  inline void evaluate(double t_now, double& x, double& y, double& z) {
    if (!compiled) {
      x = y = z = std::numeric_limits<double>::quiet_NaN();
      return;
    }
    t_var = t_now;
    x = expr_x.value();
    y = expr_y.value();
    z = expr_z.value();
  }
};

class DynamicForestNode : public rclcpp::Node {
 public:
  DynamicForestNode() : Node("dynamic_forest_trajs") {
    // ---- Parameters ----
    total_num_obs_ = declare_parameter<int>("total_num_obs", 30);
    dynamic_ratio_ = declare_parameter<double>("dynamic_ratio", 0.5);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    seed_ = declare_parameter<int>("seed", 0);

    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    publish_trajs_ = declare_parameter<bool>("publish_trajs", true);
    trajs_topic_ = declare_parameter<std::string>("trajs_topic", "/trajs");
    move_models_ = declare_parameter<bool>("move_gazebo_models", false);
    use_spawn_origins_ = declare_parameter<bool>("use_spawn_origins", false);

    std::string obstacles_json_str = declare_parameter<std::string>("obstacles_json", "[]");
    bool use_external = declare_parameter<bool>("use_external_obstacles_json", true);

    if (use_external) {
      try {
        json J = json::parse(obstacles_json_str);
        obstacles_.reserve(J.size());
        for (auto& item : J) {
          ObstacleSpec o;
          o.model_name = item.value("name", "");
          o.traj_x = item.value("traj_x", "");
          o.traj_y = item.value("traj_y", "");
          o.traj_z = item.value("traj_z", "");
          o.traj_vx = item.value("traj_vx", "");
          o.traj_vy = item.value("traj_vy", "");
          o.traj_vz = item.value("traj_vz", "");
          o.x0 = item.value("x0", 0.0);
          o.y0 = item.value("y0", 0.0);
          o.z0 = item.value("z0", 0.0);
          // Support both old "size" and new "size_x/y/z" formats
          o.bbox[0] = item.value("size_x", item.value("size", 1.0));
          o.bbox[1] = item.value("size_y", item.value("size", 1.0));
          o.bbox[2] = item.value("size_z", item.value("size", 1.0));
          obstacles_.push_back(std::move(o));
        }
        for (auto& o : obstacles_) {
          o.compile();
        }
        adopted_spawn_ = true;       // Already have centers
        use_spawn_origins_ = false;  // No need to adopt
        RCLCPP_INFO(get_logger(), "Loaded & compiled %zu obstacles from JSON.", obstacles_.size());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to parse/compile obstacles_json: %s", e.what());
      }
    } else {
      RCLCPP_WARN(get_logger(), "No external obstacles_json provided; node will be idle.");
    }

    // Optionally adopt spawn origins if desired
    if (use_spawn_origins_) {
      model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
          "/gazebo/model_states", 10,
          std::bind(&DynamicForestNode::modelStatesCB, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Waiting to adopt spawn origins from /gazebo/model_states...");
    }

    if (publish_trajs_) traj_pub_ = create_publisher<DynTraj>(trajs_topic_, 10);
    if (move_models_)
      model_state_pub_ =
          create_publisher<gazebo_msgs::msg::ModelState>("/gazebo/set_model_state", 50);

    if (publish_markers_) {
      dyn_mesh_pub_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("/shapes_dynamic_mesh", 10);
      dyn_pts_pub_ = create_publisher<visualization_msgs::msg::Marker>("/shapes_dynamic", 10);
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    double period = (publish_rate_hz_ > 0.0) ? 1.0 / publish_rate_hz_ : 0.02;
    timer_ = create_wall_timer(std::chrono::duration<double>(period),
                               std::bind(&DynamicForestNode::timerCB, this));
  }

 private:
  // Adopt initial positions from Gazebo model_states if requested
  void modelStatesCB(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    if (!use_spawn_origins_ || adopted_spawn_) return;

    int matched = 0;
    for (auto& o : obstacles_) {
      for (size_t j = 0; j < msg->name.size(); ++j) {
        if (msg->name[j] == o.model_name) {
          o.x0 = msg->pose[j].position.x;
          o.y0 = msg->pose[j].position.y;
          o.z0 = msg->pose[j].position.z;
          matched++;
          break;
        }
      }
    }
    if (matched == static_cast<int>(obstacles_.size())) {
      adopted_spawn_ = true;
      RCLCPP_INFO(get_logger(), "Adopted spawn origins for all %d obstacles.", matched);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Spawn adoption incomplete (%d/%zu). Continuing...", matched,
                           obstacles_.size());
    }
  }

  visualization_msgs::msg::Marker makePointsMarker() {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "dyn_points";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.25;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
    return m;
  }

  visualization_msgs::msg::Marker makeMeshMarker(int id, const std::array<double, 3>& bbox,
                                                 double x, double y, double z) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "dyn_obs_mesh";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = bbox[0];
    m.scale.y = bbox[1];
    m.scale.z = bbox[2];
    m.color.r = 0.7f;
    m.color.g = 0.7f;
    m.color.b = 0.7f;
    m.color.a = 0.8f;
    return m;
  }

  void timerCB() {
    if (use_spawn_origins_ && !adopted_spawn_) return;

    auto stamp = now();
    double t_now = static_cast<double>(stamp.nanoseconds()) * 1e-9;

    visualization_msgs::msg::MarkerArray mesh_array;
    visualization_msgs::msg::Marker points_marker;
    if (publish_markers_) points_marker = makePointsMarker();

    int idx = 0;
    for (auto& o : obstacles_) {
      double x, y, z;
      o.evaluate(t_now, x, y, z);

      // Publish DynTraj (ground truth obstacle trajectories)
      if (publish_trajs_) {
        DynTraj traj;
        traj.header.stamp = stamp;
        traj.is_agent = false;
        traj.mode = "analytic";
        traj.id = 4000 + idx;
        traj.function = {o.traj_x, o.traj_y, o.traj_z};
        traj.velocity = {o.traj_vx, o.traj_vy, o.traj_vz};
        traj.pos.x = x;
        traj.pos.y = y;
        traj.pos.z = z;
        traj.bbox = {o.bbox[0], o.bbox[1], o.bbox[2]};
        traj_pub_->publish(traj);
      }

      if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = o.model_name;  // or dyn_traj_<id>
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;
        tf.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf);
      }

      if (move_models_) {
        gazebo_msgs::msg::ModelState ms;
        ms.model_name = o.model_name;
        ms.pose.position.x = x;
        ms.pose.position.y = y;
        ms.pose.position.z = z;
        ms.pose.orientation.w = 1.0;
        model_state_pub_->publish(ms);
      }

      if (publish_markers_) {
        auto marker = makeMeshMarker(idx, o.bbox, x, y, z);
        mesh_array.markers.push_back(marker);
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        points_marker.points.push_back(p);
      }

      ++idx;
    }

    if (publish_markers_) {
      dyn_mesh_pub_->publish(mesh_array);
      dyn_pts_pub_->publish(points_marker);
    }
  }

  // Members
  int total_num_obs_{0};
  double dynamic_ratio_{0.0};
  double publish_rate_hz_{50.0};
  int seed_{0};

  bool publish_markers_{true};
  bool publish_tf_{false};
  bool publish_trajs_{true};
  std::string trajs_topic_{"/trajs"};
  bool move_models_{false};
  bool use_spawn_origins_{false};
  bool adopted_spawn_{false};

  std::vector<ObstacleSpec> obstacles_;

  // ROS infrastructure
  rclcpp::Publisher<DynTraj>::SharedPtr traj_pub_;
  rclcpp::Publisher<gazebo_msgs::msg::ModelState>::SharedPtr model_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dyn_mesh_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dyn_pts_pub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicForestNode>());
  rclcpp::shutdown();
  return 0;
}
