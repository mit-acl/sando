/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <cstdlib>
#include <fstream>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <nlohmann/json.hpp>
#include <sim/exprtk.hpp>
#include <sstream>
#include <string>
#include <vector>
using json = nlohmann::json;

namespace gazebo {

// ------------------------------------------------------------------ obstacle
struct DynObs {
  std::string name;
  double x0{0}, y0{0}, z0{0};
  double bbox_x{0.8}, bbox_y{0.8}, bbox_z{0.8};

  std::string traj_x_str, traj_y_str, traj_z_str;

  // exprtk
  double t_var{0.0};
  exprtk::symbol_table<double> sym;
  exprtk::expression<double> ex, ey, ez;
  bool compiled{false};

  // Gazebo handle (resolved after spawn)
  physics::ModelPtr model;

  void compile() {
    sym.clear();
    sym.add_variable("t", t_var);
    sym.add_constants();

    ex = exprtk::expression<double>();
    ey = exprtk::expression<double>();
    ez = exprtk::expression<double>();
    ex.register_symbol_table(sym);
    ey.register_symbol_table(sym);
    ez.register_symbol_table(sym);

    exprtk::parser<double> p;
    if (!p.compile(traj_x_str, ex))
      gzerr << "[DynObsPlugin] compile traj_x failed: " << traj_x_str << "\n";
    if (!p.compile(traj_y_str, ey))
      gzerr << "[DynObsPlugin] compile traj_y failed: " << traj_y_str << "\n";
    if (!p.compile(traj_z_str, ez))
      gzerr << "[DynObsPlugin] compile traj_z failed: " << traj_z_str << "\n";

    compiled = true;
  }

  inline void evaluate(double t, double& x, double& y, double& z) {
    if (!compiled) {
      x = x0;
      y = y0;
      z = z0;
      return;
    }
    t_var = t;
    x = ex.value();
    y = ey.value();
    z = ez.value();
  }
};

// ------------------------------------------------------------------ plugin
class DynamicObstaclesWorldPlugin : public WorldPlugin {
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;

    if (sdf->HasElement("json_path")) json_path_ = sdf->Get<std::string>("json_path");

    gzmsg << "[DynObsPlugin] Will read obstacles from: " << json_path_ << "\n";

    // Try loading immediately (file is generated before Gazebo starts)
    if (LoadJSON()) {
      SpawnModels();
    } else {
      gzmsg << "[DynObsPlugin] JSON not found yet, will poll in OnUpdate\n";
    }

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DynamicObstaclesWorldPlugin::OnUpdate, this));
  }

 private:
  // ---- helpers ----
  bool LoadJSON() {
    std::ifstream ifs(json_path_);
    if (!ifs.is_open()) {
      gzmsg << "[DynObsPlugin] Cannot open: " << json_path_ << "\n";
      return false;
    }

    try {
      json J = json::parse(ifs);
      obstacles_.reserve(J.size());

      for (auto& item : J) {
        DynObs o;
        o.name = item.value("name", "");
        o.x0 = item.value("x0", 0.0);
        o.y0 = item.value("y0", 0.0);
        o.z0 = item.value("z0", 0.0);
        o.bbox_x = item.value("size_x", item.value("size", 0.8));
        o.bbox_y = item.value("size_y", item.value("size", 0.8));
        o.bbox_z = item.value("size_z", item.value("size", 0.8));
        o.traj_x_str = item.value("traj_x", std::to_string(o.x0));
        o.traj_y_str = item.value("traj_y", std::to_string(o.y0));
        o.traj_z_str = item.value("traj_z", std::to_string(o.z0));
        // NOTE: do NOT compile() here — exprtk expressions hold raw
        // pointers to t_var.  After std::move the pointers dangle.
        obstacles_.push_back(std::move(o));
      }

      // Compile AFTER all objects are in the vector (stable addresses).
      for (auto& o : obstacles_) o.compile();

      gzmsg << "[DynObsPlugin] Loaded " << obstacles_.size() << " obstacles from " << json_path_
            << "\n";
      json_loaded_ = true;
      return true;
    } catch (const std::exception& e) {
      gzerr << "[DynObsPlugin] JSON parse error: " << e.what() << "\n";
      return false;
    }
  }

  void SpawnModels() {
    for (const auto& o : obstacles_) {
      std::ostringstream ss;
      // No <collision> element: obstacles pass through each other and the drone.
      // The D435 depth camera ray-traces against <visual> geometry, so
      // pointcloud reflections still work without collision geometry.
      ss << "<?xml version='1.0' ?>"
         << "<sdf version='1.5'>"
         << "<model name='" << o.name << "'>"
         << "  <static>true</static>"
         << "  <pose>" << o.x0 << " " << o.y0 << " " << o.z0 << " 0 0 0</pose>"
         << "  <link name='link'>"
         << "    <gravity>0</gravity>"
         << "    <inertial>"
         << "      <mass>0.01</mass>"
         << "      <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>"
         << "        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
         << "    </inertial>"
         << "    <visual name='vis'>"
         << "      <geometry><box><size>" << o.bbox_x << " " << o.bbox_y << " " << o.bbox_z
         << "</size></box></geometry>"
         << "      <material>"
         << "        <ambient>0.5 0.5 0.5 0.8</ambient>"
         << "        <diffuse>0.5 0.5 0.5 0.8</diffuse>"
         << "      </material>"
         << "    </visual>"
         << "  </link>"
         << "</model>"
         << "</sdf>";

      // Use InsertModelString — simplest API, takes raw SDF XML
      world_->InsertModelString(ss.str());
    }

    gzmsg << "[DynObsPlugin] Queued " << obstacles_.size() << " box models for insertion\n";
  }

  bool ResolveModels() {
    int found = 0;
    for (auto& o : obstacles_) {
      if (!o.model) o.model = world_->ModelByName(o.name);
      if (o.model) ++found;
    }
    if (found < static_cast<int>(obstacles_.size())) {
      // Log progress periodically
      if (++resolve_counter_ % 1000 == 0)
        gzmsg << "[DynObsPlugin] Resolving models: " << found << "/" << obstacles_.size() << "\n";
      return false;
    }
    return true;
  }

  // ---- main loop ----
  void OnUpdate() {
    // Phase 1: wait for JSON file (if not loaded in Load())
    if (!json_loaded_) {
      if (++poll_counter_ % 1000 == 0) {
        if (LoadJSON()) SpawnModels();
      }
      return;
    }

    // Phase 2: wait for all models to be inserted by Gazebo
    if (!models_resolved_) {
      models_resolved_ = ResolveModels();
      if (!models_resolved_) return;
      gzmsg << "[DynObsPlugin] All " << obstacles_.size()
            << " model pointers resolved — moving obstacles\n";
    }

    // Phase 3: move every obstacle
    // Use Gazebo sim time — the same clock that drives sensor plugins
    // (lidar, camera, etc.).  dynamic_forest_node must run with
    // use_sim_time:=true so its now() returns the /clock published by
    // Gazebo, keeping both sides on the exact same time base.
    double t = world_->SimTime().Double();
    for (auto& o : obstacles_) {
      double x, y, z;
      o.evaluate(t, x, y, z);
      o.model->SetWorldPose(ignition::math::Pose3d(x, y, z, 0, 0, 0));
    }
  }

  // ---- state ----
  physics::WorldPtr world_;
  event::ConnectionPtr update_conn_;
  std::vector<DynObs> obstacles_;
  std::string json_path_{"/tmp/sando_obstacles.json"};
  bool json_loaded_{false};
  bool models_resolved_{false};
  int poll_counter_{0};
  int resolve_counter_{0};
};

GZ_REGISTER_WORLD_PLUGIN(DynamicObstaclesWorldPlugin)

}  // namespace gazebo
