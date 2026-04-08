/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 *
 * C++ port of analyze_dynamic_benchmark.py
 * Provides much faster rosbag-based collision and violation analysis.
 * -------------------------------------------------------------------------- */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <numeric>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

// ROS2 bag reading
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

// Message types
#include "dynus_interfaces/msg/dyn_traj.hpp"
#include "dynus_interfaces/msg/goal.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace fs = std::filesystem;

// ============================================================================
// Data structures
// ============================================================================

struct TrialData {
  int trial_id = -1;
  bool goal_reached = false;
  bool timeout_reached = false;
  bool collision = false;
  double flight_travel_time = 0.0;
  double path_length = 0.0;
  double path_efficiency = 0.0;
  double jerk_rms = 0.0;
  double jerk_integral = 0.0;
  int collision_count = 0;
  double min_distance_to_obstacles = 1e18;
  double collision_free_ratio = 1.0;
  int collision_unique_obstacles = 0;

  // Computation time
  int num_replans = 0;
  double avg_replanning_time = 0.0;
  double max_replanning_time = 0.0;
  double total_replanning_time = 0.0;
  double avg_global_planning_time = 0.0;
  double avg_sfc_corridor_time = 0.0;
  double avg_local_traj_time = 0.0;

  // Violation counts
  int vel_violation_count = 0;
  int vel_violation_total = 0;
  int acc_violation_count = 0;
  int acc_violation_total = 0;
  int jerk_violation_count = 0;
  int jerk_violation_total = 0;

  // SFC violations
  int sfc_violation_count = 0;
  int sfc_violation_total = 0;
};

struct Statistics {
  int total_trials = 0;
  int n_successful = 0;
  double success_rate = 0.0;
  double timeout_rate = 0.0;
  double collision_rate = 0.0;

  // Collision
  int collision_count_total = 0;
  double collision_count_mean = 0.0;
  double collision_free_rate = 0.0;

  // Min distance
  double min_distance_to_obstacles_min = 1e18;
  double min_distance_to_obstacles_max = -1e18;
  double min_distance_to_obstacles_mean = 0.0;
  double min_distance_to_obstacles_std = 0.0;
  bool has_min_distance = false;

  // Computation times (mean of per-trial averages)
  double avg_local_traj_time_mean = 0.0;
  double avg_replanning_time_mean = 0.0;
  double avg_global_planning_time_mean = 0.0;
  double avg_sfc_corridor_time_mean = 0.0;
  double num_replans_mean = 0.0;

  // Performance (on successful trials)
  double flight_travel_time_mean = 0.0;
  double flight_travel_time_std = 0.0;
  double path_length_mean = 0.0;
  double path_length_std = 0.0;
  double path_efficiency_mean = 0.0;
  double jerk_rms_mean = 0.0;
  double jerk_integral_mean = 0.0;

  // Violation rates
  double vel_violation_rate = 0.0;
  double acc_violation_rate = 0.0;
  double jerk_violation_rate = 0.0;
  double sfc_violation_rate = 0.0;
};

// Global flag: when true, skip cached stats and recompute from bags
static bool g_recompute = false;

static void save_statistics_json(const Statistics& s, const fs::path& path) {
  nlohmann::json j;
  j["total_trials"] = s.total_trials;
  j["n_successful"] = s.n_successful;
  j["success_rate"] = s.success_rate;
  j["timeout_rate"] = s.timeout_rate;
  j["collision_rate"] = s.collision_rate;
  j["collision_count_total"] = s.collision_count_total;
  j["collision_count_mean"] = s.collision_count_mean;
  j["collision_free_rate"] = s.collision_free_rate;
  j["min_distance_to_obstacles_min"] = s.min_distance_to_obstacles_min;
  j["min_distance_to_obstacles_max"] = s.min_distance_to_obstacles_max;
  j["min_distance_to_obstacles_mean"] = s.min_distance_to_obstacles_mean;
  j["min_distance_to_obstacles_std"] = s.min_distance_to_obstacles_std;
  j["has_min_distance"] = s.has_min_distance;
  j["avg_local_traj_time_mean"] = s.avg_local_traj_time_mean;
  j["avg_replanning_time_mean"] = s.avg_replanning_time_mean;
  j["avg_global_planning_time_mean"] = s.avg_global_planning_time_mean;
  j["avg_sfc_corridor_time_mean"] = s.avg_sfc_corridor_time_mean;
  j["num_replans_mean"] = s.num_replans_mean;
  j["flight_travel_time_mean"] = s.flight_travel_time_mean;
  j["flight_travel_time_std"] = s.flight_travel_time_std;
  j["path_length_mean"] = s.path_length_mean;
  j["path_length_std"] = s.path_length_std;
  j["path_efficiency_mean"] = s.path_efficiency_mean;
  j["jerk_rms_mean"] = s.jerk_rms_mean;
  j["jerk_integral_mean"] = s.jerk_integral_mean;
  j["vel_violation_rate"] = s.vel_violation_rate;
  j["acc_violation_rate"] = s.acc_violation_rate;
  j["jerk_violation_rate"] = s.jerk_violation_rate;
  j["sfc_violation_rate"] = s.sfc_violation_rate;
  std::ofstream f(path);
  f << j.dump(2);
}

static bool load_statistics_json(const fs::path& path, Statistics& s) {
  if (!fs::exists(path)) return false;
  try {
    std::ifstream f(path);
    nlohmann::json j = nlohmann::json::parse(f);
    s.total_trials = j.value("total_trials", 0);
    s.n_successful = j.value("n_successful", 0);
    s.success_rate = j.value("success_rate", 0.0);
    s.timeout_rate = j.value("timeout_rate", 0.0);
    s.collision_rate = j.value("collision_rate", 0.0);
    s.collision_count_total = j.value("collision_count_total", 0);
    s.collision_count_mean = j.value("collision_count_mean", 0.0);
    s.collision_free_rate = j.value("collision_free_rate", 0.0);
    s.min_distance_to_obstacles_min = j.value("min_distance_to_obstacles_min", 1e18);
    s.min_distance_to_obstacles_max = j.value("min_distance_to_obstacles_max", -1e18);
    s.min_distance_to_obstacles_mean = j.value("min_distance_to_obstacles_mean", 0.0);
    s.min_distance_to_obstacles_std = j.value("min_distance_to_obstacles_std", 0.0);
    s.has_min_distance = j.value("has_min_distance", false);
    s.avg_local_traj_time_mean = j.value("avg_local_traj_time_mean", 0.0);
    s.avg_replanning_time_mean = j.value("avg_replanning_time_mean", 0.0);
    s.avg_global_planning_time_mean = j.value("avg_global_planning_time_mean", 0.0);
    s.avg_sfc_corridor_time_mean = j.value("avg_sfc_corridor_time_mean", 0.0);
    s.num_replans_mean = j.value("num_replans_mean", 0.0);
    s.flight_travel_time_mean = j.value("flight_travel_time_mean", 0.0);
    s.flight_travel_time_std = j.value("flight_travel_time_std", 0.0);
    s.path_length_mean = j.value("path_length_mean", 0.0);
    s.path_length_std = j.value("path_length_std", 0.0);
    s.path_efficiency_mean = j.value("path_efficiency_mean", 0.0);
    s.jerk_rms_mean = j.value("jerk_rms_mean", 0.0);
    s.jerk_integral_mean = j.value("jerk_integral_mean", 0.0);
    s.vel_violation_rate = j.value("vel_violation_rate", 0.0);
    s.acc_violation_rate = j.value("acc_violation_rate", 0.0);
    s.jerk_violation_rate = j.value("jerk_violation_rate", 0.0);
    s.sfc_violation_rate = j.value("sfc_violation_rate", 0.0);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to load cached stats from " << path << ": " << e.what() << "\n";
    return false;
  }
}

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

struct ObstacleSnapshot {
  double time;
  Vec3 pos;
};

struct ObstacleTrack {
  std::vector<double> times;
  std::vector<Vec3> positions;
  double half_x = 0.4, half_y = 0.4, half_z = 0.4;
};

struct AgentSnapshot {
  double time;
  Vec3 pos;
  Vec3 vel;
  Vec3 acc;
  Vec3 jerk;
};

// ============================================================================
// CSV parsing
// ============================================================================

static std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    // Trim whitespace
    size_t start = token.find_first_not_of(" \t\r\n");
    size_t end = token.find_last_not_of(" \t\r\n");
    if (start != std::string::npos)
      tokens.push_back(token.substr(start, end - start + 1));
    else
      tokens.push_back("");
  }
  return tokens;
}

static int find_column(const std::vector<std::string>& header, const std::string& name) {
  for (size_t i = 0; i < header.size(); i++) {
    if (header[i] == name) return static_cast<int>(i);
  }
  return -1;
}

static double safe_stod(const std::string& s) {
  if (s.empty() || s == "inf" || s == "nan" || s == "N/A") return 0.0;
  try {
    return std::stod(s);
  } catch (...) {
    return 0.0;
  }
}

static int safe_stoi(const std::string& s) {
  if (s.empty()) return 0;
  try {
    return std::stoi(s);
  } catch (...) {
    return 0;
  }
}

static bool safe_stob(const std::string& s) { return s == "True" || s == "true" || s == "1"; }

static std::vector<TrialData> load_benchmark_csv(const fs::path& csv_path) {
  std::vector<TrialData> trials;
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    std::cerr << "ERROR: Cannot open CSV: " << csv_path << "\n";
    return trials;
  }

  std::string line;
  std::getline(file, line);  // header
  auto header = split_csv_line(line);

  // Find column indices
  int col_trial_id = find_column(header, "trial_id");
  int col_goal_reached = find_column(header, "goal_reached");
  int col_timeout = find_column(header, "timeout_reached");
  int col_collision = find_column(header, "collision");
  int col_travel_time = find_column(header, "flight_travel_time");
  int col_path_length = find_column(header, "path_length");
  int col_path_efficiency = find_column(header, "path_efficiency");
  int col_jerk_rms = find_column(header, "jerk_rms");
  int col_jerk_integral = find_column(header, "jerk_integral");
  int col_collision_count = find_column(header, "collision_count");
  int col_min_dist = find_column(header, "min_distance_to_obstacles");
  int col_sfc_viol_count = find_column(header, "sfc_violation_count");
  int col_sfc_viol_total = find_column(header, "sfc_violation_total");
  int col_vel_viol_count = find_column(header, "vel_violation_count");
  int col_vel_viol_total = find_column(header, "vel_violation_total");
  int col_acc_viol_count = find_column(header, "acc_violation_count");
  int col_acc_viol_total = find_column(header, "acc_violation_total");
  int col_jerk_viol_count = find_column(header, "jerk_violation_count");
  int col_jerk_viol_total = find_column(header, "jerk_violation_total");

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    auto cols = split_csv_line(line);
    TrialData t;
    if (col_trial_id >= 0 && col_trial_id < (int)cols.size())
      t.trial_id = safe_stoi(cols[col_trial_id]);
    if (col_goal_reached >= 0 && col_goal_reached < (int)cols.size())
      t.goal_reached = safe_stob(cols[col_goal_reached]);
    if (col_timeout >= 0 && col_timeout < (int)cols.size())
      t.timeout_reached = safe_stob(cols[col_timeout]);
    if (col_collision >= 0 && col_collision < (int)cols.size())
      t.collision = safe_stob(cols[col_collision]);
    if (col_travel_time >= 0 && col_travel_time < (int)cols.size())
      t.flight_travel_time = safe_stod(cols[col_travel_time]);
    if (col_path_length >= 0 && col_path_length < (int)cols.size())
      t.path_length = safe_stod(cols[col_path_length]);
    if (col_path_efficiency >= 0 && col_path_efficiency < (int)cols.size())
      t.path_efficiency = safe_stod(cols[col_path_efficiency]);
    if (col_jerk_rms >= 0 && col_jerk_rms < (int)cols.size())
      t.jerk_rms = safe_stod(cols[col_jerk_rms]);
    if (col_jerk_integral >= 0 && col_jerk_integral < (int)cols.size())
      t.jerk_integral = safe_stod(cols[col_jerk_integral]);
    if (col_collision_count >= 0 && col_collision_count < (int)cols.size())
      t.collision_count = safe_stoi(cols[col_collision_count]);
    if (col_min_dist >= 0 && col_min_dist < (int)cols.size())
      t.min_distance_to_obstacles = safe_stod(cols[col_min_dist]);
    if (col_sfc_viol_count >= 0 && col_sfc_viol_count < (int)cols.size())
      t.sfc_violation_count = safe_stoi(cols[col_sfc_viol_count]);
    if (col_sfc_viol_total >= 0 && col_sfc_viol_total < (int)cols.size())
      t.sfc_violation_total = safe_stoi(cols[col_sfc_viol_total]);
    if (col_vel_viol_count >= 0 && col_vel_viol_count < (int)cols.size())
      t.vel_violation_count = safe_stoi(cols[col_vel_viol_count]);
    if (col_vel_viol_total >= 0 && col_vel_viol_total < (int)cols.size())
      t.vel_violation_total = safe_stoi(cols[col_vel_viol_total]);
    if (col_acc_viol_count >= 0 && col_acc_viol_count < (int)cols.size())
      t.acc_violation_count = safe_stoi(cols[col_acc_viol_count]);
    if (col_acc_viol_total >= 0 && col_acc_viol_total < (int)cols.size())
      t.acc_violation_total = safe_stoi(cols[col_acc_viol_total]);
    if (col_jerk_viol_count >= 0 && col_jerk_viol_count < (int)cols.size())
      t.jerk_violation_count = safe_stoi(cols[col_jerk_viol_count]);
    if (col_jerk_viol_total >= 0 && col_jerk_viol_total < (int)cols.size())
      t.jerk_violation_total = safe_stoi(cols[col_jerk_viol_total]);
    trials.push_back(t);
  }

  return trials;
}

static fs::path find_most_recent_csv(const fs::path& dir) {
  if (!fs::exists(dir) || !fs::is_directory(dir)) return {};
  std::vector<fs::path> csv_files;
  for (auto& entry : fs::directory_iterator(dir)) {
    auto name = entry.path().filename().string();
    if (name.find("benchmark_") == 0 && name.find(".csv") != std::string::npos &&
        name.find("summary") == std::string::npos) {
      csv_files.push_back(entry.path());
    }
  }
  if (csv_files.empty()) return {};

  // Sort by modification time, pick most recent
  std::sort(csv_files.begin(), csv_files.end(), [](const fs::path& a, const fs::path& b) {
    return fs::last_write_time(a) < fs::last_write_time(b);
  });
  return csv_files.back();
}

// ============================================================================
// Computation data loading (num_*.csv)
// ============================================================================

struct ComputationStats {
  int num_replans = 0;
  double avg_replanning_time = 0.0;
  double max_replanning_time = 0.0;
  double total_replanning_time = 0.0;
  double avg_global_planning_time = 0.0;
  double avg_sfc_corridor_time = 0.0;
  double avg_local_traj_time = 0.0;
};

static std::map<int, ComputationStats> load_computation_data(const fs::path& data_dir) {
  std::map<int, ComputationStats> result;
  fs::path csv_dir = data_dir / "csv";
  if (!fs::exists(csv_dir)) return result;

  for (auto& entry : fs::directory_iterator(csv_dir)) {
    auto name = entry.path().filename().string();
    if (name.find("num_") != 0 || name.find(".csv") == std::string::npos) continue;

    // Extract trial id: num_X.csv
    int trial_id = 0;
    try {
      auto stem = entry.path().stem().string();
      trial_id = std::stoi(stem.substr(4));
    } catch (...) {
      continue;
    }

    std::ifstream file(entry.path());
    if (!file.is_open()) continue;

    std::string line;
    std::getline(file, line);  // header
    auto header = split_csv_line(line);

    int col_result = find_column(header, "Result");
    int col_total_replan = find_column(header, "Total replanning time [ms]");
    int col_global = find_column(header, "Global Planning Time [ms]");
    int col_local = find_column(header, "Local Traj Time [ms]");
    int col_cvx = find_column(header, "CVX Decomposition Time [ms]");
    if (col_cvx < 0) col_cvx = find_column(header, "SFC Corridor Time [ms]");

    int count = 0;
    double sum_replan = 0, max_replan = 0, sum_global = 0, sum_local = 0, sum_cvx = 0;

    while (std::getline(file, line)) {
      if (line.empty()) continue;
      auto cols = split_csv_line(line);
      if (col_result < 0 || col_result >= (int)cols.size()) continue;
      if (safe_stoi(cols[col_result]) != 1) continue;  // only successful replans

      count++;
      double r = (col_total_replan >= 0 && col_total_replan < (int)cols.size())
                     ? safe_stod(cols[col_total_replan])
                     : 0.0;
      sum_replan += r;
      max_replan = std::max(max_replan, r);
      if (col_global >= 0 && col_global < (int)cols.size())
        sum_global += safe_stod(cols[col_global]);
      if (col_local >= 0 && col_local < (int)cols.size()) sum_local += safe_stod(cols[col_local]);
      if (col_cvx >= 0 && col_cvx < (int)cols.size()) sum_cvx += safe_stod(cols[col_cvx]);
    }

    ComputationStats stats;
    stats.num_replans = count;
    if (count > 0) {
      stats.avg_replanning_time = sum_replan / count;
      stats.max_replanning_time = max_replan;
      stats.total_replanning_time = sum_replan;
      stats.avg_global_planning_time = sum_global / count;
      stats.avg_local_traj_time = sum_local / count;
      stats.avg_sfc_corridor_time = sum_cvx / count;
    }
    result[trial_id] = stats;
  }

  std::cout << "  Loaded computation data for " << result.size() << " trial(s)\n";
  return result;
}

// ============================================================================
// Bag reading — single-pass extraction
// ============================================================================

struct BagData {
  std::vector<AgentSnapshot> agent;        // from /NX01/goal
  std::map<int, ObstacleTrack> obstacles;  // from /trajs_ground_truth or /tf
};

static Vec3 interpolate_position(const std::vector<double>& times,
                                 const std::vector<Vec3>& positions, double t_query) {
  if (times.empty()) return {0, 0, 0};
  if (times.size() == 1) return positions[0];
  if (t_query <= times.front()) return positions.front();
  if (t_query >= times.back()) return positions.back();

  // Binary search
  auto it = std::lower_bound(times.begin(), times.end(), t_query);
  size_t idx = std::distance(times.begin(), it);
  if (idx >= times.size()) return positions.back();
  if (idx == 0) return positions.front();

  double t0 = times[idx - 1], t1 = times[idx];
  double alpha = (t1 > t0) ? (t_query - t0) / (t1 - t0) : 0.0;
  const auto& p0 = positions[idx - 1];
  const auto& p1 = positions[idx];
  return {p0.x + alpha * (p1.x - p0.x), p0.y + alpha * (p1.y - p0.y), p0.z + alpha * (p1.z - p0.z)};
}

static BagData read_bag_single_pass(const fs::path& bag_path, const std::string& trajs_topic,
                                    bool read_tf) {
  BagData data;

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path.string();
  storage_options.storage_id = "sqlite3";

  try {
    reader.open(storage_options);
  } catch (const std::exception& e) {
    std::cerr << "    Warning: Cannot open bag " << bag_path << ": " << e.what() << "\n";
    return data;
  }

  rclcpp::Serialization<dynus_interfaces::msg::Goal> goal_ser;
  rclcpp::Serialization<dynus_interfaces::msg::DynTraj> dyntraj_ser;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_ser;

  while (reader.has_next()) {
    try {
      auto msg = reader.read_next();
      double timestamp = msg->time_stamp / 1e9;

      if (msg->topic_name == "/NX01/goal") {
        dynus_interfaces::msg::Goal goal_msg;
        rclcpp::SerializedMessage serialized(*msg->serialized_data);
        goal_ser.deserialize_message(&serialized, &goal_msg);

        AgentSnapshot snap;
        snap.time = timestamp;
        snap.pos = {goal_msg.p.x, goal_msg.p.y, goal_msg.p.z};
        snap.vel = {goal_msg.v.x, goal_msg.v.y, goal_msg.v.z};
        snap.acc = {goal_msg.a.x, goal_msg.a.y, goal_msg.a.z};
        snap.jerk = {goal_msg.j.x, goal_msg.j.y, goal_msg.j.z};
        data.agent.push_back(snap);
      } else if (!trajs_topic.empty() && msg->topic_name == trajs_topic) {
        dynus_interfaces::msg::DynTraj dyntraj_msg;
        rclcpp::SerializedMessage serialized(*msg->serialized_data);
        dyntraj_ser.deserialize_message(&serialized, &dyntraj_msg);

        if (dyntraj_msg.is_agent) continue;

        int obs_id = dyntraj_msg.id;
        auto& track = data.obstacles[obs_id];
        track.times.push_back(timestamp);
        track.positions.push_back({dyntraj_msg.pos.x, dyntraj_msg.pos.y, dyntraj_msg.pos.z});

        if (dyntraj_msg.bbox.size() >= 3) {
          track.half_x = dyntraj_msg.bbox[0] / 2.0;
          track.half_y = dyntraj_msg.bbox[1] / 2.0;
          track.half_z = dyntraj_msg.bbox[2] / 2.0;
        }
      } else if (read_tf && msg->topic_name == "/tf") {
        tf2_msgs::msg::TFMessage tf_msg;
        rclcpp::SerializedMessage serialized(*msg->serialized_data);
        tf_ser.deserialize_message(&serialized, &tf_msg);

        for (auto& transform : tf_msg.transforms) {
          std::string frame = transform.child_frame_id;
          if (frame.find("obstacle") != std::string::npos || frame.find("obs_") == 0) {
            int obs_id = static_cast<int>(std::hash<std::string>{}(frame)&0x7FFFFFFF);
            auto& track = data.obstacles[obs_id];
            track.times.push_back(timestamp);
            track.positions.push_back({transform.transform.translation.x,
                                       transform.transform.translation.y,
                                       transform.transform.translation.z});
          }
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "    Warning: skipping corrupted message in bag: " << e.what() << "\n";
      continue;
    }
  }

  return data;
}

// ============================================================================
// Collision analysis
// ============================================================================

struct CollisionResult {
  int collision_count = 0;
  double min_distance = 1e18;
  double collision_free_ratio = 1.0;
  int unique_obstacles = 0;
};

static CollisionResult analyze_collisions(const BagData& data) {
  CollisionResult result;
  if (data.agent.empty() || data.obstacles.empty()) {
    result.unique_obstacles = static_cast<int>(data.obstacles.size());
    return result;
  }

  result.unique_obstacles = static_cast<int>(data.obstacles.size());
  int collision_free_segments = 0;

  for (const auto& snap : data.agent) {
    bool segment_collision_free = true;
    double px = snap.pos.x, py = snap.pos.y, pz = snap.pos.z;

    for (const auto& [obs_id, track] : data.obstacles) {
      if (track.times.empty()) continue;

      Vec3 obs_pos = interpolate_position(track.times, track.positions, snap.time);

      // Pre-filter: skip far obstacles
      double dx = px - obs_pos.x, dy = py - obs_pos.y, dz = pz - obs_pos.z;
      double center_dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (center_dist > 5.0) continue;

      double hx = track.half_x, hy = track.half_y, hz = track.half_z;

      // Point-to-AABB distance
      double cx = std::clamp(px, obs_pos.x - hx, obs_pos.x + hx);
      double cy = std::clamp(py, obs_pos.y - hy, obs_pos.y + hy);
      double cz = std::clamp(pz, obs_pos.z - hz, obs_pos.z + hz);

      double ddx = px - cx, ddy = py - cy, ddz = pz - cz;
      double distance = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
      result.min_distance = std::min(result.min_distance, distance);

      // Check collision: point inside AABB
      if (obs_pos.x - hx <= px && px <= obs_pos.x + hx && obs_pos.y - hy <= py &&
          py <= obs_pos.y + hy && obs_pos.z - hz <= pz && pz <= obs_pos.z + hz) {
        segment_collision_free = false;
        result.collision_count++;
      }
    }

    if (segment_collision_free) collision_free_segments++;
  }

  result.collision_free_ratio = static_cast<double>(collision_free_segments) / data.agent.size();
  if (result.min_distance > 1e17) result.min_distance = 0.0;
  return result;
}

// ============================================================================
// Static obstacle collision analysis (from CSV)
// ============================================================================

struct StaticObstacle {
  int id;
  double x, y, z, radius, height;
};

static std::vector<StaticObstacle> load_static_obstacles(const fs::path& csv_path) {
  std::vector<StaticObstacle> obstacles;
  std::ifstream file(csv_path);
  if (!file.is_open()) return obstacles;

  std::string line;
  std::getline(file, line);  // header
  auto header = split_csv_line(line);
  int col_id = find_column(header, "id");
  int col_x = find_column(header, "x");
  int col_y = find_column(header, "y");
  int col_z = find_column(header, "z");
  int col_radius = find_column(header, "radius");
  int col_height = find_column(header, "height");

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    auto cols = split_csv_line(line);
    StaticObstacle obs;
    obs.id = (col_id >= 0 && col_id < (int)cols.size()) ? safe_stoi(cols[col_id]) : 0;
    obs.x = (col_x >= 0 && col_x < (int)cols.size()) ? safe_stod(cols[col_x]) : 0;
    obs.y = (col_y >= 0 && col_y < (int)cols.size()) ? safe_stod(cols[col_y]) : 0;
    obs.z = (col_z >= 0 && col_z < (int)cols.size()) ? safe_stod(cols[col_z]) : 0;
    obs.radius =
        (col_radius >= 0 && col_radius < (int)cols.size()) ? safe_stod(cols[col_radius]) : 0;
    obs.height =
        (col_height >= 0 && col_height < (int)cols.size()) ? safe_stod(cols[col_height]) : 0;
    obstacles.push_back(obs);
  }
  return obstacles;
}

static CollisionResult analyze_static_collisions(const BagData& data,
                                                 const std::vector<StaticObstacle>& obstacles) {
  CollisionResult result;
  if (data.agent.empty() || obstacles.empty()) return result;

  int collision_free_segments = 0;
  std::set<int> obstacles_hit;

  for (const auto& snap : data.agent) {
    bool segment_collision_free = true;
    double px = snap.pos.x, py = snap.pos.y, pz = snap.pos.z;

    for (const auto& obs : obstacles) {
      double horiz_dist = std::sqrt((px - obs.x) * (px - obs.x) + (py - obs.y) * (py - obs.y));
      double horiz_clearance = horiz_dist - obs.radius;

      double obs_z_min = obs.z - obs.height / 2.0;
      double obs_z_max = obs.z + obs.height / 2.0;
      bool vert_inside = (pz > obs_z_min && pz < obs_z_max);

      double dist = std::max(0.0, horiz_clearance);
      result.min_distance = std::min(result.min_distance, dist);

      if (horiz_clearance < 0 && vert_inside) {
        segment_collision_free = false;
        result.collision_count++;
        obstacles_hit.insert(obs.id);
      }
    }

    if (segment_collision_free) collision_free_segments++;
  }

  result.unique_obstacles = static_cast<int>(obstacles_hit.size());
  result.collision_free_ratio = static_cast<double>(collision_free_segments) / data.agent.size();
  if (result.min_distance > 1e17) result.min_distance = 0.0;
  return result;
}

// ============================================================================
// Violation analysis (from bag data, already extracted)
// ============================================================================

struct ViolationResult {
  int vel_count = 0, vel_total = 0;
  int acc_count = 0, acc_total = 0;
  int jerk_count = 0, jerk_total = 0;
};

static ViolationResult analyze_violations(const BagData& data, double vel_limit = 5.0,
                                          double acc_limit = 20.0, double jerk_limit = 100.0,
                                          double tolerance = 1e-3) {
  ViolationResult result;
  for (const auto& snap : data.agent) {
    result.vel_total++;
    result.acc_total++;
    result.jerk_total++;

    double vmax = std::max({std::abs(snap.vel.x), std::abs(snap.vel.y), std::abs(snap.vel.z)});
    if (vmax > vel_limit + tolerance) result.vel_count++;

    double amax = std::max({std::abs(snap.acc.x), std::abs(snap.acc.y), std::abs(snap.acc.z)});
    if (amax > acc_limit + tolerance) result.acc_count++;

    double jmax = std::max({std::abs(snap.jerk.x), std::abs(snap.jerk.y), std::abs(snap.jerk.z)});
    if (jmax > jerk_limit + tolerance) result.jerk_count++;
  }
  return result;
}

// ============================================================================
// Path metrics from bag data
// ============================================================================

struct PathMetrics {
  std::optional<double> travel_time;
  std::optional<double> path_length;
};

static PathMetrics compute_path_metrics(const BagData& data, Vec3 goal_pos, Vec3 start_pos,
                                        double dist_threshold = 0.5, double speed_threshold = 0.1) {
  PathMetrics result;
  if (data.agent.empty()) return result;

  // Detect gap
  auto& first = data.agent.front();
  double gap_dist = std::sqrt((first.pos.x - start_pos.x) * (first.pos.x - start_pos.x) +
                              (first.pos.y - start_pos.y) * (first.pos.y - start_pos.y) +
                              (first.pos.z - start_pos.z) * (first.pos.z - start_pos.z));
  bool has_gap = gap_dist > 0.5;

  // Find first movement
  size_t move_start_idx = 0;
  for (size_t i = 0; i < data.agent.size(); i++) {
    auto& s = data.agent[i];
    double speed = std::sqrt(s.vel.x * s.vel.x + s.vel.y * s.vel.y + s.vel.z * s.vel.z);
    if (speed > 0.01) {
      move_start_idx = i;
      break;
    }
  }

  // Find goal arrival
  std::optional<size_t> goal_idx;
  for (size_t i = 0; i < data.agent.size(); i++) {
    auto& s = data.agent[i];
    double dist = std::sqrt((s.pos.x - goal_pos.x) * (s.pos.x - goal_pos.x) +
                            (s.pos.y - goal_pos.y) * (s.pos.y - goal_pos.y) +
                            (s.pos.z - goal_pos.z) * (s.pos.z - goal_pos.z));
    double speed = std::sqrt(s.vel.x * s.vel.x + s.vel.y * s.vel.y + s.vel.z * s.vel.z);
    if (dist < dist_threshold && speed < speed_threshold) {
      goal_idx = i;
      break;
    }
  }

  // Path length
  size_t end_idx = goal_idx.value_or(data.agent.size() - 1);
  double path_length = gap_dist;
  for (size_t i = 1; i <= end_idx; i++) {
    double dx = data.agent[i].pos.x - data.agent[i - 1].pos.x;
    double dy = data.agent[i].pos.y - data.agent[i - 1].pos.y;
    double dz = data.agent[i].pos.z - data.agent[i - 1].pos.z;
    path_length += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  result.path_length = path_length;

  // Travel time
  if (!has_gap && goal_idx.has_value()) {
    result.travel_time = data.agent[goal_idx.value()].time - data.agent[move_start_idx].time;
  }

  return result;
}

// ============================================================================
// Statistics computation
// ============================================================================

static double compute_mean(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

static double compute_std(const std::vector<double>& v, double mean) {
  if (v.size() < 2) return 0.0;
  double sum = 0.0;
  for (double x : v) sum += (x - mean) * (x - mean);
  return std::sqrt(sum / (v.size() - 1));
}

static Statistics compute_statistics(std::vector<TrialData>& trials) {
  Statistics stats;
  stats.total_trials = static_cast<int>(trials.size());
  if (trials.empty()) return stats;

  // Mark timeout
  for (auto& t : trials) {
    if (t.flight_travel_time > 100.0) {
      t.goal_reached = false;
      t.timeout_reached = true;
    }
  }

  // Success, timeout, collision rates
  int success_count = 0, timeout_count = 0, collision_count_trials = 0;
  for (auto& t : trials) {
    if (t.goal_reached && t.collision_count == 0) success_count++;
    if (t.timeout_reached) timeout_count++;
    if (t.collision) collision_count_trials++;
  }
  stats.success_rate = 100.0 * success_count / trials.size();
  stats.timeout_rate = 100.0 * timeout_count / trials.size();
  stats.collision_rate = 100.0 * collision_count_trials / trials.size();

  // Collision metrics (ALL trials)
  int total_coll = 0;
  int zero_coll = 0;
  for (auto& t : trials) {
    total_coll += t.collision_count;
    if (t.collision_count == 0) zero_coll++;
  }
  stats.collision_count_total = total_coll;
  stats.collision_count_mean = static_cast<double>(total_coll) / trials.size();
  stats.collision_free_rate = 100.0 * zero_coll / trials.size();

  // Min distance (ALL trials)
  std::vector<double> min_dists;
  for (auto& t : trials) {
    if (t.min_distance_to_obstacles < 1e17 && t.min_distance_to_obstacles > 0) {
      min_dists.push_back(t.min_distance_to_obstacles);
    }
  }
  if (!min_dists.empty()) {
    stats.has_min_distance = true;
    stats.min_distance_to_obstacles_min = *std::min_element(min_dists.begin(), min_dists.end());
    stats.min_distance_to_obstacles_max = *std::max_element(min_dists.begin(), min_dists.end());
    stats.min_distance_to_obstacles_mean = compute_mean(min_dists);
    stats.min_distance_to_obstacles_std =
        compute_std(min_dists, stats.min_distance_to_obstacles_mean);
  }

  // Successful trials
  std::vector<TrialData*> successful;
  for (auto& t : trials) {
    if (t.goal_reached && t.collision_count == 0) successful.push_back(&t);
  }
  stats.n_successful = static_cast<int>(successful.size());
  if (successful.empty()) {
    std::cout << "WARNING: No successful trials found!\n";
    return stats;
  }

  // Computation times
  std::vector<double> v_local, v_replan, v_global, v_sfc, v_nreplans;
  for (auto* t : successful) {
    if (t->avg_local_traj_time > 0) v_local.push_back(t->avg_local_traj_time);
    if (t->avg_replanning_time > 0) v_replan.push_back(t->avg_replanning_time);
    if (t->avg_global_planning_time > 0) v_global.push_back(t->avg_global_planning_time);
    if (t->avg_sfc_corridor_time > 0) v_sfc.push_back(t->avg_sfc_corridor_time);
    if (t->num_replans > 0) v_nreplans.push_back(t->num_replans);
  }
  stats.avg_local_traj_time_mean = compute_mean(v_local);
  stats.avg_replanning_time_mean = compute_mean(v_replan);
  stats.avg_global_planning_time_mean = compute_mean(v_global);
  stats.avg_sfc_corridor_time_mean = compute_mean(v_sfc);
  stats.num_replans_mean = compute_mean(v_nreplans);

  // Performance
  std::vector<double> v_tt, v_pl, v_pe, v_jrms, v_jint;
  for (auto* t : successful) {
    v_tt.push_back(t->flight_travel_time);
    v_pl.push_back(t->path_length);
    v_pe.push_back(t->path_efficiency);
    v_jrms.push_back(t->jerk_rms);
    v_jint.push_back(t->jerk_integral);
  }
  stats.flight_travel_time_mean = compute_mean(v_tt);
  stats.flight_travel_time_std = compute_std(v_tt, stats.flight_travel_time_mean);
  stats.path_length_mean = compute_mean(v_pl);
  stats.path_length_std = compute_std(v_pl, stats.path_length_mean);
  stats.path_efficiency_mean = compute_mean(v_pe);
  stats.jerk_rms_mean = compute_mean(v_jrms);
  stats.jerk_integral_mean = compute_mean(v_jint);

  // Violation rates (among successful)
  long total_vel_v = 0, total_vel_s = 0;
  long total_acc_v = 0, total_acc_s = 0;
  long total_jerk_v = 0, total_jerk_s = 0;
  long total_sfc_v = 0, total_sfc_s = 0;
  for (auto* t : successful) {
    total_vel_v += t->vel_violation_count;
    total_vel_s += t->vel_violation_total;
    total_acc_v += t->acc_violation_count;
    total_acc_s += t->acc_violation_total;
    total_jerk_v += t->jerk_violation_count;
    total_jerk_s += t->jerk_violation_total;
    total_sfc_v += t->sfc_violation_count;
    total_sfc_s += t->sfc_violation_total;
  }
  stats.vel_violation_rate = (total_vel_s > 0) ? 100.0 * total_vel_v / total_vel_s : 0.0;
  stats.acc_violation_rate = (total_acc_s > 0) ? 100.0 * total_acc_v / total_acc_s : 0.0;
  stats.jerk_violation_rate = (total_jerk_s > 0) ? 100.0 * total_jerk_v / total_jerk_s : 0.0;
  stats.sfc_violation_rate = (total_sfc_s > 0) ? 100.0 * total_sfc_v / total_sfc_s : 0.0;

  return stats;
}

// ============================================================================
// Console output
// ============================================================================

static void print_statistics(const Statistics& s) {
  std::string sep(80, '=');
  std::string dash(80, '-');

  std::cout << "\n" << sep << "\nBENCHMARK ANALYSIS RESULTS\n" << sep << "\n";

  std::cout << "\n" << std::string(30, '-') << " OVERVIEW " << std::string(30, '-') << "\n";
  std::cout << "  Total trials: " << s.total_trials << "\n";
  std::cout << "  Successful trials: " << s.n_successful << "\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "  Success rate: " << s.success_rate << "%\n";
  std::cout << "  Timeout rate: " << s.timeout_rate << "%\n";
  std::cout << "  Collision rate: " << s.collision_rate << "%\n";

  std::cout << "\n"
            << std::string(25, '-') << " COMPUTATION TIME (ms) " << std::string(25, '-') << "\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  Local Traj Time: " << s.avg_local_traj_time_mean << " ms\n";
  std::cout << "  Replanning Time: " << s.avg_replanning_time_mean << " ms\n";
  std::cout << "  Global Planning Time: " << s.avg_global_planning_time_mean << " ms\n";
  std::cout << "  SFC Corridor Time: " << s.avg_sfc_corridor_time_mean << " ms\n";

  std::cout << "\n"
            << std::string(25, '-') << " PERFORMANCE METRICS " << std::string(25, '-') << "\n";
  std::cout << "  Travel Time: " << s.flight_travel_time_mean << " +/- " << s.flight_travel_time_std
            << " s\n";
  std::cout << "  Path Length: " << s.path_length_mean << " +/- " << s.path_length_std << " m\n";
  std::cout << "  Path Efficiency: " << std::setprecision(3) << s.path_efficiency_mean << "\n";
  std::cout << "  Jerk RMS: " << std::setprecision(2) << s.jerk_rms_mean << " m/s^3\n";
  std::cout << "  Jerk Integral: " << s.jerk_integral_mean << "\n";

  std::cout << "\n"
            << std::string(25, '-') << " CONSTRAINT VIOLATIONS " << std::string(25, '-') << "\n";
  std::cout << std::setprecision(1);
  std::cout << "  VEL Violation Rate: " << s.vel_violation_rate << "%\n";
  std::cout << "  ACC Violation Rate: " << s.acc_violation_rate << "%\n";
  std::cout << "  JERK Violation Rate: " << s.jerk_violation_rate << "%\n";

  std::cout << "\n"
            << std::string(25, '-') << " COLLISION METRICS " << std::string(25, '-') << "\n";
  std::cout << "  Collision-free rate: " << s.collision_free_rate << "%\n";
  std::cout << "  Total collision events: " << s.collision_count_total << "\n";
  std::cout << std::setprecision(2);
  std::cout << "  Mean collisions per trial: " << s.collision_count_mean << "\n";

  std::cout << "\n" << std::string(25, '-') << " MIN DISTANCE " << std::string(25, '-') << "\n";
  if (s.has_min_distance) {
    std::cout << std::setprecision(3);
    std::cout << "  Min: " << s.min_distance_to_obstacles_min << " m\n";
    std::cout << "  Max: " << s.min_distance_to_obstacles_max << " m\n";
    std::cout << "  Mean: " << s.min_distance_to_obstacles_mean << " m\n";
    std::cout << "  Std: " << s.min_distance_to_obstacles_std << " m\n";
  } else {
    std::cout << "  N/A\n";
  }

  std::cout << "\n" << sep << "\n\n";
}

// ============================================================================
// CSV summary output
// ============================================================================

static void save_statistics_csv(const Statistics& s, const fs::path& output_path) {
  fs::create_directories(output_path.parent_path());
  std::ofstream f(output_path);
  if (!f.is_open()) {
    std::cerr << "ERROR: Cannot write CSV: " << output_path << "\n";
    return;
  }

  f << "total_trials,n_successful,success_rate,timeout_rate,collision_rate,"
    << "collision_count_total,collision_count_mean,collision_free_rate,"
    << "avg_local_traj_time_mean,avg_replanning_time_mean,"
    << "flight_travel_time_mean,flight_travel_time_std,"
    << "path_length_mean,path_length_std,"
    << "jerk_integral_mean,jerk_rms_mean,"
    << "min_distance_to_obstacles_mean,min_distance_to_obstacles_std,"
    << "vel_violation_rate,acc_violation_rate,jerk_violation_rate\n";

  f << std::fixed;
  f << s.total_trials << "," << s.n_successful << "," << std::setprecision(1) << s.success_rate
    << "," << s.timeout_rate << "," << s.collision_rate << "," << s.collision_count_total << ","
    << std::setprecision(2) << s.collision_count_mean << "," << std::setprecision(1)
    << s.collision_free_rate << "," << std::setprecision(2) << s.avg_local_traj_time_mean << ","
    << s.avg_replanning_time_mean << "," << std::setprecision(6) << s.flight_travel_time_mean << ","
    << s.flight_travel_time_std << "," << s.path_length_mean << "," << s.path_length_std << ","
    << s.jerk_integral_mean << "," << s.jerk_rms_mean << "," << s.min_distance_to_obstacles_mean
    << "," << s.min_distance_to_obstacles_std << "," << std::setprecision(1) << s.vel_violation_rate
    << "," << s.acc_violation_rate << "," << s.jerk_violation_rate << "\n";

  std::cout << "Statistics saved to CSV: " << output_path << "\n";
}

// ============================================================================
// LaTeX table generation
// ============================================================================

static std::string generate_sando_row(const Statistics& s, const std::string& case_name,
                                      const std::string& table_type,
                                      const std::string& algo_name = "SANDO") {
  std::ostringstream oss;
  oss << std::fixed;

  double success_rate = s.success_rate;
  double travel_time = s.flight_travel_time_mean;
  double path_length = s.path_length_mean;
  double jerk_integral = s.jerk_integral_mean;
  double vel_viol = s.vel_violation_rate;
  double acc_viol = s.acc_violation_rate;
  double jerk_viol = s.jerk_violation_rate;

  if (table_type == "static") {
    double total_opt_time = s.avg_local_traj_time_mean;
    double total_replan_time = s.avg_replanning_time_mean;
    oss << "       & " << algo_name << " & Hard & $L_\\infty$ & " << std::setprecision(1) << "{"
        << success_rate << "} & "
        << "{" << total_opt_time << "} & "
        << "{" << total_replan_time << "} & "
        << "\\best{" << travel_time << "} & "
        << "{" << path_length << "} & "
        << "\\best{" << jerk_integral << "} & "
        << "{" << vel_viol << "} & "
        << "{" << acc_viol << "} & "
        << "{" << jerk_viol << "} \\\\";
  } else if (table_type == "unknown_dynamic") {
    double per_opt_time = s.avg_local_traj_time_mean;
    double total_replan_time = s.avg_replanning_time_mean;
    double cvx_decomp_time = s.avg_sfc_corridor_time_mean;
    std::string min_dist_str;
    if (s.has_min_distance) {
      std::ostringstream md;
      md << std::fixed << std::setprecision(2) << s.min_distance_to_obstacles_mean;
      min_dist_str = md.str();
    } else {
      min_dist_str = "N/A";
    }

    oss << "      " << case_name << " & " << std::setprecision(1) << success_rate << " & "
        << per_opt_time << " & " << total_replan_time << " & " << cvx_decomp_time << " & "
        << travel_time << " & " << path_length << " & " << jerk_integral << " & " << min_dist_str
        << " & " << vel_viol << " & " << acc_viol << " & " << jerk_viol << " \\\\";
  } else {
    // dynamic
    double per_opt_time = s.avg_local_traj_time_mean;
    std::string min_dist_str;
    if (s.has_min_distance) {
      std::ostringstream md;
      md << std::fixed << std::setprecision(2) << s.min_distance_to_obstacles_mean;
      min_dist_str = md.str();
    } else {
      min_dist_str = "N/A";
    }

    oss << "      & \\multicolumn{2}{c}{" << algo_name << "} & " << std::setprecision(1)
        << success_rate << " & " << per_opt_time << " & " << travel_time << " & " << path_length
        << " & " << jerk_integral << " & " << min_dist_str << " & " << vel_viol << " & " << acc_viol
        << " & " << jerk_viol << " \\\\";
  }

  return oss.str();
}

static std::string generate_new_unknown_dynamic_table(const std::string& case_name,
                                                      const std::string& sando_row) {
  std::string dashes = "{-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\\\";
  std::vector<std::string> cases = {"Easy", "Medium", "Hard"};

  std::ostringstream oss;
  oss << "\\begin{table*}\n"
      << "  \\caption{Benchmark results in unknown dynamic environments. "
      << "SANDO navigates using only pointcloud sensing (no ground truth obstacle trajectories). "
      << "We report success rate, computation time, flight performance, smoothness, safety, "
      << "and constraint violation metrics.}\n"
      << "  \\label{tab:unknown_dynamic_benchmark}\n"
      << "  \\centering\n"
      << "  \\renewcommand{\\arraystretch}{1.2}\n"
      << "  \\resizebox{\\textwidth}{!}{\n"
      << "    \\begin{tabular}{c c c c c c c c c c c c}\n"
      << "      \\toprule\n"
      << "      \\multirow{2}{*}[-0.4em]{\\textbf{Env}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Success}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Comp. Time}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Performance}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Safety}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}\n"
      << "      \\\\\n"
      << "      \\cmidrule(lr){2-2}\n"
      << "      \\cmidrule(lr){3-5}\n"
      << "      \\cmidrule(lr){6-8}\n"
      << "      \\cmidrule(lr){9-9}\n"
      << "      \\cmidrule(lr){10-12}\n"
      << "      &\n"
      << "      $R_{\\mathrm{succ}}$ [\\%] &\n"
      << "      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &\n"
      << "      $T_{\\mathrm{replan}}$ [ms] &\n"
      << "      $T_{\\mathrm{STSFC}}$ [ms] &\n"
      << "      $T_{\\mathrm{trav}}$ [s] &\n"
      << "      $L_{\\mathrm{path}}$ [m] &\n"
      << "      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &\n"
      << "      $d_{\\mathrm{min}}$ [m] &\n"
      << "      $\\rho_{\\mathrm{vel}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{acc}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{jerk}}$ [\\%]\n"
      << "      \\\\\n"
      << "      \\midrule\n";

  for (auto& c : cases) {
    if (c == case_name) {
      oss << sando_row << "\n";
    } else {
      oss << "      " << c << " & " << dashes << "\n";
    }
  }

  oss << "      \\bottomrule\n"
      << "    \\end{tabular}\n"
      << "  }\n"
      << "  \\vspace{-1.0em}\n"
      << "\\end{table*}";

  return oss.str();
}

static std::string generate_new_dynamic_table(const std::string& sando_row) {
  std::ostringstream oss;
  oss << "\\begin{table*}\n"
      << "  \\caption{Dynamic obstacle benchmarking results: SANDO performance with moving "
         "obstacles. "
      << "We report success rate, computation time, flight performance, smoothness, safety, "
      << "and constraint violation metrics.}\n"
      << "  \\label{tab:dynamic_benchmark}\n"
      << "  \\centering\n"
      << "  \\renewcommand{\\arraystretch}{1.2}\n"
      << "  \\resizebox{\\textwidth}{!}{\n"
      << "    \\begin{tabular}{c c c c c c c c c c c c}\n"
      << "      \\toprule\n"
      << "      \\multirow{2}{*}[-0.4em]{\\textbf{Env}}\n"
      << "      & \\multicolumn{2}{c}{\\multirow{2}{*}[-0.4em]{\\textbf{Algorithm}}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Success}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Comp. Time}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Performance}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Safety}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}\n"
      << "      \\\\\n"
      << "      \\cmidrule(lr){4-4}\n"
      << "      \\cmidrule(lr){5-5}\n"
      << "      \\cmidrule(lr){6-8}\n"
      << "      \\cmidrule(lr){9-9}\n"
      << "      \\cmidrule(lr){10-12}\n"
      << "      &&&\n"
      << "      $R_{\\mathrm{succ}}$ [\\%] &\n"
      << "      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &\n"
      << "      $T_{\\mathrm{trav}}$ [s] &\n"
      << "      $L_{\\mathrm{path}}$ [m] &\n"
      << "      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &\n"
      << "      $d_{\\mathrm{min}}$ [m] &\n"
      << "      $\\rho_{\\mathrm{vel}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{acc}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{jerk}}$ [\\%]\n"
      << "      \\\\\n"
      << "      \\midrule\n"
      << sando_row << "\n"
      << "      \\bottomrule\n"
      << "    \\end{tabular}\n"
      << "  }\n"
      << "  \\vspace{-1.0em}\n"
      << "\\end{table*}";

  return oss.str();
}

static std::string generate_new_static_table(const std::string& case_name,
                                             const std::string& /*sando_row*/,
                                             const std::string& data_values) {
  std::string dashes = "{-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\\\";
  std::vector<std::string> cases = {"Easy", "Medium", "Hard"};

  std::ostringstream oss;
  oss << "\\begin{table*}\n"
      << "  \\caption{Benchmark results against state-of-the-art methods in static environments.}\n"
      << "  \\label{tab:static_benchmark}\n"
      << "  \\centering\n"
      << "  \\renewcommand{\\arraystretch}{1.2}\n"
      << "  \\resizebox{\\textwidth}{!}{\n"
      << "    \\begin{tabular}{c c c c c c c c c c c c c}\n"
      << "      \\toprule\n"
      << "      \\multirow{2}{*}[-0.4ex]{\\textbf{Env}}\n"
      << "      & \\multirow{2}{*}[-0.4ex]{\\textbf{Algorithm}}\n"
      << "      & \\multicolumn{2}{c}{\\multirow{2}{*}[-0.4ex]{\\textbf{Constr.}}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Success}}\n"
      << "      & \\multicolumn{2}{c}{\\textbf{Computation Time}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Performance}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}\n"
      << "      \\\\\n"
      << "      \\cmidrule(lr){5-5}\n"
      << "      \\cmidrule(lr){6-7}\n"
      << "      \\cmidrule(lr){8-10}\n"
      << "      \\cmidrule(lr){11-13}\n"
      << "      &&&&\n"
      << "      $R_{\\mathrm{succ}}$ [\\%]\n"
      << "      & $T^{\\mathrm{total}}_{\\mathrm{opt}}$ [ms]\n"
      << "      & $T^{\\mathrm{total}}_{\\mathrm{replan}}$ [ms]\n"
      << "      & $T_{\\mathrm{trav}}$ [s]\n"
      << "      & $L_{\\mathrm{path}}$ [m]\n"
      << "      & $S_{\\mathrm{jerk}}$ [m/s$^{2}$]\n"
      << "      & $\\rho_{\\mathrm{vel}}$ [\\%]\n"
      << "      & $\\rho_{\\mathrm{acc}}$ [\\%]\n"
      << "      & $\\rho_{\\mathrm{jerk}}$ [\\%]\n"
      << "      \\\\\n"
      << "      \\midrule\n";

  for (size_t i = 0; i < cases.size(); i++) {
    if (i > 0) oss << "\n      \\midrule\n\n";

    oss << "      \\multirow{5}{*}{" << cases[i] << "} & EGO-Swarm2 & Soft & $L_\\infty$ & "
        << dashes << "\n";
    oss << "       & \\multirow{2}{*}{SUPER} & Soft & $L_2$ & " << dashes << "\n";
    oss << "       & & Soft & $L_\\infty$ & " << dashes << "\n";
    oss << "       & FASTER & Hard & $L_\\infty$ & " << dashes << "\n";

    if (cases[i] == case_name) {
      oss << "       & SANDO & Hard & $L_\\infty$ & " << data_values << "\n";
    } else {
      oss << "       & SANDO & Hard & $L_\\infty$ & " << dashes << "\n";
    }
  }

  oss << "      \\bottomrule\n"
      << "    \\end{tabular}\n"
      << "  }\n"
      << "  \\vspace{-1.0em}\n"
      << "\\end{table*}";

  return oss.str();
}

static std::string update_existing_table(const fs::path& tex_path, const std::string& case_name,
                                         const std::string& sando_row,
                                         const std::string& table_type,
                                         const std::string& algo_name = "SANDO") {
  std::ifstream f(tex_path);
  if (!f.is_open()) return "";

  std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  f.close();

  std::istringstream iss(content);
  std::string line;
  std::vector<std::string> updated_lines;
  bool row_updated = false;
  std::string current_case;
  std::vector<std::string> valid_cases = {"Easy", "Medium", "Hard"};

  while (std::getline(iss, line)) {
    std::string stripped = line;
    // Trim
    size_t s = stripped.find_first_not_of(" \t");
    if (s != std::string::npos) stripped = stripped.substr(s);

    // Track current case from multirow
    std::regex multirow_re(R"(\\multirow\{[^}]*\}\{[^}]*\}\{(\w+)\})");
    std::sregex_iterator it(line.begin(), line.end(), multirow_re);
    for (; it != std::sregex_iterator(); ++it) {
      std::string match = (*it)[1];
      for (auto& vc : valid_cases) {
        if (match == vc) current_case = match;
      }
    }

    // Case 1: algo_name row on same line as multirow
    if (line.find(case_name) != std::string::npos && line.find(algo_name) != std::string::npos &&
        line.find("&") != std::string::npos && line.find("multirow") != std::string::npos) {
      updated_lines.push_back(sando_row);
      row_updated = true;
    }
    // Case 2: algo_name row in separate line (dynamic/static table)
    else if (current_case == case_name && line.find(algo_name) != std::string::npos &&
             line.find("&") != std::string::npos && line.find("\\\\") != std::string::npos &&
             line.find("multirow") == std::string::npos) {
      updated_lines.push_back(sando_row);
      row_updated = true;
    }
    // Case 3: unknown_dynamic format
    else if (table_type == "unknown_dynamic" && stripped.find(case_name) == 0 &&
             stripped.find("&") != std::string::npos &&
             stripped.find("\\\\") != std::string::npos) {
      updated_lines.push_back(sando_row);
      row_updated = true;
    } else {
      updated_lines.push_back(line);
    }
  }

  if (!row_updated) {
    // Insert before the next \midrule or \bottomrule after the current case block
    std::cout << "  No matching " << case_name << " + " << algo_name
              << " row found, inserting new row...\n";
    bool inserted = false;
    bool in_target_case = false;
    for (size_t i = 0; i < updated_lines.size(); i++) {
      // Detect case start via multirow
      std::regex mr_re(R"(\\multirow\{(\d+)\}\{[^}]*\}\{(\w+)\})");
      std::smatch mr_match;
      if (std::regex_search(updated_lines[i], mr_match, mr_re)) {
        std::string env = mr_match[2];
        if (env == case_name) {
          in_target_case = true;
          // Increment multirow count
          int old_count = std::stoi(mr_match[1]);
          int new_count = old_count + 1;
          std::string old_mr = "\\multirow{" + std::to_string(old_count) + "}";
          std::string new_mr = "\\multirow{" + std::to_string(new_count) + "}";
          size_t pos = updated_lines[i].find(old_mr);
          if (pos != std::string::npos) {
            updated_lines[i].replace(pos, old_mr.size(), new_mr);
          }
        } else if (in_target_case) {
          // We've entered the next case — insert before this line
          updated_lines.insert(updated_lines.begin() + static_cast<long>(i), sando_row);
          inserted = true;
          break;
        }
      }
      // If we're in target case and hit \midrule or \bottomrule, insert before it
      if (in_target_case) {
        std::string trimmed = updated_lines[i];
        size_t ts = trimmed.find_first_not_of(" \t");
        if (ts != std::string::npos) trimmed = trimmed.substr(ts);
        if (trimmed.find("\\midrule") == 0 || trimmed.find("\\bottomrule") == 0) {
          updated_lines.insert(updated_lines.begin() + static_cast<long>(i), sando_row);
          inserted = true;
          break;
        }
      }
    }
    if (!inserted) {
      // Fallback: insert before last \bottomrule
      for (int i = static_cast<int>(updated_lines.size()) - 1; i >= 0; i--) {
        if (updated_lines[i].find("\\bottomrule") != std::string::npos) {
          updated_lines.insert(updated_lines.begin() + i, sando_row);
          break;
        }
      }
    }
  } else {
    std::cout << "  Updated " << case_name << " + " << algo_name << " row\n";
  }

  std::ostringstream result;
  for (size_t i = 0; i < updated_lines.size(); i++) {
    result << updated_lines[i];
    if (i + 1 < updated_lines.size()) result << "\n";
  }
  return result.str();
}

static std::string generate_latex_table(const Statistics& s, const std::string& case_name,
                                        const fs::path& existing_file,
                                        const std::string& table_type,
                                        const std::string& algo_name = "SANDO") {
  std::string sando_row = generate_sando_row(s, case_name, table_type, algo_name);

  // Try updating existing table
  if (fs::exists(existing_file)) {
    std::cout << "  Found existing table, updating " << case_name << " + " << algo_name
              << " row...\n";
    std::string updated =
        update_existing_table(existing_file, case_name, sando_row, table_type, algo_name);
    if (!updated.empty()) return updated;
  }

  // Generate new table
  if (table_type == "static") {
    // Extract data_values from sando_row (everything after the 4th &)
    // For simplicity, just generate with full row
    std::string data_values;
    int amp_count = 0;
    for (size_t i = 0; i < sando_row.size(); i++) {
      if (sando_row[i] == '&') {
        amp_count++;
        if (amp_count == 4) {
          data_values = sando_row.substr(i + 1);
          // Trim leading whitespace
          size_t s = data_values.find_first_not_of(" \t");
          if (s != std::string::npos) data_values = data_values.substr(s);
          break;
        }
      }
    }
    return generate_new_static_table(case_name, sando_row, data_values);
  } else if (table_type == "unknown_dynamic") {
    return generate_new_unknown_dynamic_table(case_name, sando_row);
  } else {
    return generate_new_dynamic_table(sando_row);
  }
}

// ============================================================================
// Main analysis
// ============================================================================

static Statistics analyze_single_case(const fs::path& data_dir, const std::string& output_name,
                                      const fs::path& latex_output, const std::string& table_type,
                                      Vec3 goal_pos, const std::string& algo_name = "SANDO",
                                      bool skip_latex = false) {
  // Extract case name
  std::string dir_name = data_dir.filename().string();
  std::string case_name = "Unknown";
  if (dir_name == "easy" || dir_name.find("easy_") == 0)
    case_name = "Easy";
  else if (dir_name == "medium" || dir_name.find("medium_") == 0)
    case_name = "Medium";
  else if (dir_name == "hard" || dir_name.find("hard_") == 0)
    case_name = "Hard";

  std::string sep(80, '=');
  std::cout << sep << "\n";
  std::cout << "ANALYZING " << case_name << " CASE\n";
  std::cout << sep << "\n";
  std::cout << "\nLoading data from: " << data_dir << "\n\n";

  // Check for cached statistics
  fs::path cache_path = data_dir / "stats_cache.json";
  if (!g_recompute) {
    Statistics cached;
    if (load_statistics_json(cache_path, cached)) {
      std::cout << "Using cached statistics from " << cache_path.filename() << "\n";
      print_statistics(cached);

      if (!skip_latex) {
        std::cout << "\nUpdating LaTeX table...\n";
        std::string latex =
            generate_latex_table(cached, case_name, latex_output, table_type, algo_name);
        fs::create_directories(latex_output.parent_path());
        std::ofstream tex_file(latex_output);
        tex_file << latex;
        tex_file.close();
        std::cout << "LaTeX table updated for " << case_name << " case\n";
      }

      std::cout << "\n" << sep << "\n";
      std::cout << case_name << " CASE ANALYSIS COMPLETE (cached)\n";
      std::cout << sep << "\n\n";
      return cached;
    }
  }

  // Load CSV
  fs::path csv_path = find_most_recent_csv(data_dir);
  if (csv_path.empty()) {
    std::cerr << "ERROR: No benchmark CSV files found in " << data_dir << "\n";
    return Statistics{};
  }
  std::cout << "Loading: " << csv_path.filename().string() << "\n";
  auto trials = load_benchmark_csv(csv_path);
  std::cout << "Total trials loaded: " << trials.size() << "\n\n";

  // Load computation data
  std::cout << "Loading computation time data...\n";
  auto comp_stats = load_computation_data(data_dir);
  for (auto& t : trials) {
    auto it = comp_stats.find(t.trial_id);
    if (it != comp_stats.end()) {
      t.num_replans = it->second.num_replans;
      t.avg_replanning_time = it->second.avg_replanning_time;
      t.max_replanning_time = it->second.max_replanning_time;
      t.total_replanning_time = it->second.total_replanning_time / 1000.0;
      t.avg_global_planning_time = it->second.avg_global_planning_time;
      t.avg_sfc_corridor_time = it->second.avg_sfc_corridor_time;
      t.avg_local_traj_time = it->second.avg_local_traj_time;
    }
  }
  std::cout << "\n";

  // Process each trial's bag (SINGLE PASS)
  fs::path bags_dir = data_dir / "bags";
  if (fs::exists(bags_dir)) {
    // Determine which topics to read from bags
    std::string trajs_topic;
    bool read_tf = false;
    if (table_type == "unknown_dynamic") {
      trajs_topic = "/trajs_ground_truth";
    } else if (table_type == "dynamic") {
      read_tf = true;
    }
    // static: we don't need /tf or /trajs for collision (uses CSV obstacles)

    std::cout << "Processing rosbags (single-pass)...\n";
    Vec3 start_pos = {0.0, 0.0, 2.0};

    for (auto& trial : trials) {
      fs::path bag_path = bags_dir / ("trial_" + std::to_string(trial.trial_id));
      if (!fs::exists(bag_path)) {
        std::cout << "  Warning: Bag not found for trial " << trial.trial_id << "\n";
        continue;
      }

      std::cout << "  Trial " << trial.trial_id << ": reading bag... " << std::flush;

      // Single-pass read
      BagData bag_data = read_bag_single_pass(bag_path, trajs_topic, read_tf);
      std::cout << bag_data.agent.size() << " goal msgs";
      if (!bag_data.obstacles.empty())
        std::cout << ", " << bag_data.obstacles.size() << " obstacles";
      std::cout << "\n";

      if (bag_data.agent.empty()) continue;

      // 1) Path metrics
      auto path_metrics = compute_path_metrics(bag_data, goal_pos, start_pos);
      if (path_metrics.path_length.has_value()) {
        trial.path_length = path_metrics.path_length.value();
      }

      // 2) Violation analysis
      auto viol = analyze_violations(bag_data);
      trial.vel_violation_count = viol.vel_count;
      trial.vel_violation_total = viol.vel_total;
      trial.acc_violation_count = viol.acc_count;
      trial.acc_violation_total = viol.acc_total;
      trial.jerk_violation_count = viol.jerk_count;
      trial.jerk_violation_total = viol.jerk_total;

      // 3) Collision analysis
      if (table_type == "unknown_dynamic" || table_type == "dynamic") {
        auto coll = analyze_collisions(bag_data);
        trial.collision_count = coll.collision_count;
        trial.min_distance_to_obstacles = coll.min_distance;
        trial.collision_free_ratio = coll.collision_free_ratio;
        trial.collision = (coll.collision_count > 0);
        std::cout << "    Collisions: " << coll.collision_count << ", min_dist: " << std::fixed
                  << std::setprecision(3) << coll.min_distance << "m\n";
      } else if (table_type == "static") {
        // Static collisions need obstacle CSV
        std::map<std::string, std::string> case_csv_map = {
            {"Easy", "easy_forest_obstacle_parameters.csv"},
            {"Medium", "medium_forest_obstacle_parameters.csv"},
            {"Hard", "hard_forest_obstacle_parameters.csv"},
        };
        auto csv_it = case_csv_map.find(case_name);
        if (csv_it != case_csv_map.end()) {
          // Look for obstacle CSV relative to script location
          fs::path script_dir = fs::path(__FILE__).parent_path().parent_path().parent_path();
          fs::path obs_csv = script_dir / "benchmark_data" / "static" / csv_it->second;
          auto static_obs = load_static_obstacles(obs_csv);
          if (!static_obs.empty()) {
            auto coll = analyze_static_collisions(bag_data, static_obs);
            trial.collision_count = coll.collision_count;
            trial.min_distance_to_obstacles = coll.min_distance;
            trial.collision_free_ratio = coll.collision_free_ratio;
            trial.collision_unique_obstacles = coll.unique_obstacles;
            trial.collision = (coll.collision_count > 0);
          }
        }
      }

      std::cout << "    vel=" << trial.vel_violation_count << "/" << trial.vel_violation_total
                << ", acc=" << trial.acc_violation_count << "/" << trial.acc_violation_total
                << ", jerk=" << trial.jerk_violation_count << "/" << trial.jerk_violation_total
                << "\n";
    }
    std::cout << "\n";
  }

  // Compute statistics
  std::cout << "Computing statistics...\n";
  auto stats = compute_statistics(trials);
  print_statistics(stats);

  // Cache statistics as JSON for fast re-runs
  save_statistics_json(stats, cache_path);
  std::cout << "Cached statistics to " << cache_path.filename() << "\n";

  // Save CSV
  fs::path csv_output = data_dir / (output_name + ".csv");
  save_statistics_csv(stats, csv_output);

  if (!skip_latex) {
    // Generate LaTeX
    std::cout << "\nUpdating LaTeX table...\n";
    std::string latex = generate_latex_table(stats, case_name, latex_output, table_type, algo_name);
    fs::create_directories(latex_output.parent_path());
    std::ofstream tex_file(latex_output);
    tex_file << latex;
    tex_file.close();
    std::cout << "LaTeX table updated for " << case_name << " case\n";
  }

  std::cout << "\n" << sep << "\n";
  std::cout << case_name << " CASE ANALYSIS COMPLETE\n";
  std::cout << sep << "\n\n";

  return stats;
}

// ============================================================================
// Unknown-dynamic batch mode (multi-config table with best/worst highlighting)
// ============================================================================

struct UnknownDynamicConfig {
  std::string dir_name;
  int heat_weight = 0;
  int n_segments = 3;  // default N
  bool unk_infl = false;
};

struct BatchRowData {
  std::string case_name;
  int heat_weight;
  int n_segments = 3;
  bool unk_infl;
  Statistics stats;
};

static std::string extract_case_name(const fs::path& data_dir) {
  std::string dir_name = data_dir.filename().string();
  if (dir_name == "easy" || dir_name.find("easy_") == 0) return "Easy";
  if (dir_name == "medium" || dir_name.find("medium_") == 0) return "Medium";
  if (dir_name == "hard" || dir_name.find("hard_") == 0) return "Hard";
  return "Unknown";
}

static std::vector<UnknownDynamicConfig> discover_unknown_dynamic_configs(
    const fs::path& base_dir) {
  std::vector<UnknownDynamicConfig> configs;
  if (!fs::exists(base_dir)) return configs;

  // Match patterns like:
  //   inflate_unknown_voxels_heat_w_5
  //   inflate_unknown_voxels_heat_w_10
  //   inflate_unknown_voxels_heat_w_10_N_2
  //   no_inflate_unknown_voxels_heat_w_10
  std::regex config_re(R"((no_?)?inflate_unknown_voxels_heat_w_(\d+)(?:_N_(\d+))?)");

  for (auto& entry : fs::directory_iterator(base_dir)) {
    if (!entry.is_directory()) continue;
    std::string name = entry.path().filename().string();
    std::smatch match;
    if (std::regex_match(name, match, config_re)) {
      UnknownDynamicConfig cfg;
      cfg.dir_name = name;
      cfg.unk_infl = match[1].str().empty();  // no "no_" prefix means inflate=yes
      cfg.heat_weight = std::stoi(match[2]);
      cfg.n_segments = match[3].matched ? std::stoi(match[3]) : 3;  // default N=3
      configs.push_back(cfg);
    }
  }

  // Sort: by heat_weight, then n_segments, then unk_infl (Yes before No)
  std::sort(configs.begin(), configs.end(), [](const auto& a, const auto& b) {
    if (a.heat_weight != b.heat_weight) return a.heat_weight < b.heat_weight;
    if (a.n_segments != b.n_segments) return a.n_segments < b.n_segments;
    return a.unk_infl > b.unk_infl;  // true (Yes) before false (No)
  });

  return configs;
}

static std::string format_with_best_worst(double val, double best, double worst, int prec) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(prec) << val;
  std::string formatted = oss.str();

  double tol = std::pow(10.0, -prec) * 0.5;
  if (std::abs(best - worst) < tol) return "\\best{" + formatted + "}";  // all tied = all best
  if (std::abs(val - best) < tol) return "\\best{" + formatted + "}";
  if (std::abs(val - worst) < tol) return "\\worst{" + formatted + "}";
  return formatted;
}

// Parse existing unknown_dynamic .tex table to extract BatchRowData entries
static std::vector<BatchRowData> parse_unknown_dynamic_table(const std::string& tex_path) {
  std::vector<BatchRowData> rows;
  std::ifstream file(tex_path);
  if (!file.is_open()) {
    std::cerr << "WARNING: Could not open merge table: " << tex_path << "\n";
    return rows;
  }

  std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  // Helper to strip \best{...} and \worst{...} wrappers
  auto strip_highlight = [](const std::string& s) -> std::string {
    std::regex re(R"(\\(?:best|worst)\{([^}]*)\})");
    return std::regex_replace(s, re, "$1");
  };

  // Find data rows between \midrule and \bottomrule
  // Each row looks like: [multirow or empty] & w & N & v1 & v2 & ... & v9
  std::string current_env;
  std::regex row_re(R"(\\\\)");  // lines ending with backslash-backslash

  std::istringstream stream(content);
  std::string line;
  bool in_data = false;

  while (std::getline(stream, line)) {
    // Trim
    size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos) continue;
    std::string trimmed = line.substr(start);

    // Start of data after first \midrule
    if (trimmed.find("\\midrule") == 0) {
      in_data = true;
      continue;
    }
    if (trimmed.find("\\bottomrule") == 0) break;
    if (!in_data) continue;

    // Skip cmidrule lines
    if (trimmed.find("\\cmidrule") == 0) continue;

    // Must contain \\ to be a data row
    if (trimmed.find("\\\\") == std::string::npos) continue;

    // Remove trailing backslash-backslash
    size_t bs_pos = trimmed.rfind("\\\\");
    std::string row_content = trimmed.substr(0, bs_pos);

    // Split by &
    std::vector<std::string> cells;
    std::istringstream cell_stream(row_content);
    std::string cell;
    while (std::getline(cell_stream, cell, '&')) {
      // Trim cell
      size_t cs = cell.find_first_not_of(" \t");
      size_t ce = cell.find_last_not_of(" \t");
      if (cs != std::string::npos)
        cells.push_back(cell.substr(cs, ce - cs + 1));
      else
        cells.push_back("");
    }

    // Expect 12 cells (env, w, N, 9 metrics) but env may be empty or multirow
    if (cells.size() < 12) continue;

    // Extract env name from multirow or use current
    std::string env_cell = cells[0];
    std::smatch env_match;
    std::regex multirow_re(R"(\\multirow\{\d+\}\{[^}]*\}\{(\w+)\})");
    if (std::regex_search(env_cell, env_match, multirow_re)) {
      current_env = env_match[1];
    }
    // If env_cell is empty, use current_env

    BatchRowData row;
    row.case_name = current_env;
    row.heat_weight = std::stoi(strip_highlight(cells[1]));
    row.n_segments = std::stoi(strip_highlight(cells[2]));
    row.unk_infl = true;  // all current configs have inflate=true

    // Parse metrics: cells[3..11]
    auto parse_val = [&](const std::string& s) -> double {
      std::string clean = strip_highlight(s);
      if (clean == "N/A") return 0.0;
      return std::stod(clean);
    };

    row.stats.success_rate = parse_val(cells[3]);
    row.stats.avg_local_traj_time_mean = parse_val(cells[4]);
    row.stats.flight_travel_time_mean = parse_val(cells[5]);
    row.stats.path_length_mean = parse_val(cells[6]);
    row.stats.jerk_integral_mean = parse_val(cells[7]);
    double min_dist_val = parse_val(cells[8]);
    if (strip_highlight(cells[8]) == "N/A") {
      row.stats.has_min_distance = false;
    } else {
      row.stats.has_min_distance = true;
      row.stats.min_distance_to_obstacles_mean = min_dist_val;
    }
    row.stats.vel_violation_rate = parse_val(cells[9]);
    row.stats.acc_violation_rate = parse_val(cells[10]);
    row.stats.jerk_violation_rate = parse_val(cells[11]);

    rows.push_back(row);
  }

  return rows;
}

// Merge new SANDO rows into existing dynamic table, replacing old SANDO rows
// and updating multirow counts. case_rows maps case_name -> sando_row string.
static std::string merge_dynamic_table(
    const std::string& tex_path, const std::map<std::string, std::vector<std::string>>& case_rows) {
  std::ifstream file(tex_path);
  if (!file.is_open()) {
    std::cerr << "WARNING: Could not open merge table: " << tex_path << "\n";
    return "";
  }
  std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  std::istringstream iss(content);
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(iss, line)) lines.push_back(line);

  // Pass 1: Remove all old SANDO rows and track which cases lost rows
  std::map<std::string, int> rows_removed;
  std::string current_case;
  std::vector<std::string> filtered;

  for (auto& l : lines) {
    std::regex mr_re(R"(\\multirow\{\d+\}\{[^}]*\}\{(\w+)\})");
    std::smatch mr_match;
    if (std::regex_search(l, mr_match, mr_re)) {
      std::string env = mr_match[1];
      if (env == "Easy" || env == "Medium" || env == "Hard") {
        current_case = env;
      }
    }

    if (l.find("SANDO") != std::string::npos && l.find("&") != std::string::npos &&
        l.find("\\\\") != std::string::npos && l.find("caption") == std::string::npos) {
      rows_removed[current_case]++;
      continue;
    }
    filtered.push_back(l);
  }

  // Pass 2: Insert new SANDO rows before midrule/bottomrule, update multirow counts
  std::vector<std::string> result;
  current_case.clear();
  bool in_case = false;

  for (size_t i = 0; i < filtered.size(); i++) {
    auto& l = filtered[i];

    std::regex mr_re(R"(\\multirow\{(\d+)\}\{[^}]*\}\{(\w+)\})");
    std::smatch mr_match;
    if (std::regex_search(l, mr_match, mr_re)) {
      std::string env = mr_match[2];
      if (env == "Easy" || env == "Medium" || env == "Hard") {
        current_case = env;
        in_case = true;

        int old_count = std::stoi(mr_match[1]);
        int removed = rows_removed.count(env) ? rows_removed[env] : 0;
        int added = case_rows.count(env) ? (int)case_rows.at(env).size() : 0;
        int new_count = old_count - removed + added;
        if (new_count != old_count) {
          std::string old_mr = "\\multirow{" + std::to_string(old_count) + "}";
          std::string new_mr = "\\multirow{" + std::to_string(new_count) + "}";
          size_t pos = l.find(old_mr);
          if (pos != std::string::npos) {
            l.replace(pos, old_mr.size(), new_mr);
          }
        }
      }
    }

    std::string trimmed = l;
    size_t ts = trimmed.find_first_not_of(" \t");
    if (ts != std::string::npos) trimmed = trimmed.substr(ts);

    if (in_case && (trimmed.find("\\midrule") == 0 || trimmed.find("\\bottomrule") == 0)) {
      if (case_rows.count(current_case)) {
        for (auto& row : case_rows.at(current_case)) {
          result.push_back(row);
        }
      }
      in_case = false;
    }

    result.push_back(l);
  }

  std::ostringstream oss;
  for (size_t i = 0; i < result.size(); i++) {
    oss << result[i];
    if (i + 1 < result.size()) oss << "\n";
  }
  return oss.str();
}

static std::string generate_unknown_dynamic_batch_table(const std::vector<BatchRowData>& rows,
                                                        bool use_p_label = false) {
  std::vector<std::string> cases = {"Easy", "Medium", "Hard"};

  // Check if rows have multiple heat weights (ablation mode) or just N variants
  std::set<int> heat_weights;
  for (auto& r : rows) heat_weights.insert(r.heat_weight);
  bool has_heat_weight_col = (heat_weights.size() > 1);

  // Metric definitions: id, higher_is_better, precision
  // IDs: 0=success_rate, 1=per_opt_time, 2=total_replan_time, 3=cvx_decomp_time,
  //      4=travel_time, 5=path_length, 6=jerk_integral, 7=constraint_violation
  struct MetricDef {
    int id;
    bool higher_is_better;
    int precision;
  };
  std::vector<MetricDef> metrics = {{0, true, 1},  {1, false, 1}, {2, false, 1}, {3, false, 1},
                                    {4, false, 1}, {5, false, 1}, {6, false, 1}, {7, false, 1}};

  auto get_metric = [](const Statistics& s, int id) -> double {
    switch (id) {
      case 0:
        return s.success_rate;
      case 1:
        return s.avg_local_traj_time_mean;
      case 2:
        return s.avg_replanning_time_mean;
      case 3:
        return s.avg_sfc_corridor_time_mean;
      case 4:
        return s.flight_travel_time_mean;
      case 5:
        return s.path_length_mean;
      case 6:
        return s.jerk_integral_mean;
      case 7:
        return std::max({s.vel_violation_rate, s.acc_violation_rate, s.jerk_violation_rate});
      default:
        return 0.0;
    }
  };

  // Column count: Env + (optional w) + N + 8 metrics
  int n_cols = (has_heat_weight_col ? 3 : 2) + static_cast<int>(metrics.size());

  std::ostringstream oss;
  oss << "\\begin{table*}\n"
      << "  \\caption{Benchmark results in unknown dynamic environments. "
      << "SANDO navigates using only pointcloud sensing (no ground truth obstacle trajectories).";
  std::string segment_label = use_p_label ? "$P$" : "$N$";
  if (has_heat_weight_col) {
    oss << " We compare different heat map weights ($w$) and trajectory segment counts ("
        << segment_label << ").";
  }
  oss << " We highlight the \\best{best} and \\worst{worst} "
      << "value for each environment.}\n"
      << "  \\label{tab:unknown_dynamic_benchmark}\n"
      << "  \\centering\n"
      << "  \\renewcommand{\\arraystretch}{1.2}\n"
      << "  \\resizebox{\\textwidth}{!}{\n"
      << "    \\begin{tabular}{";
  for (int i = 0; i < n_cols; i++) {
    if (i > 0) oss << " ";
    oss << "c";
  }
  oss << "}\n"
      << "      \\toprule\n";

  // Two-row grouped header
  // prefix_cols = Env + (optional w) + N
  int prefix_cols = has_heat_weight_col ? 3 : 2;
  // Metric groups: Success(1), Comp. Time(3), Performance(3), Constraint Violation(1)
  // Column offsets (1-based): prefix_cols+1 .. prefix_cols+8

  oss << "      \\multirow{2}{*}[-0.4ex]{\\textbf{Env}}\n";
  if (has_heat_weight_col) {
    oss << "      & \\multirow{2}{*}[-0.4ex]{\\textbf{$w$}}\n";
  }
  oss << "      & \\multirow{2}{*}[-0.4ex]{\\textbf{" << segment_label << "}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Success}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Comp.\\ Time}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Performance}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Constr.\\ Viol.}}\n"
      << "      \\\\\n"
      << "      \\cmidrule(lr){" << prefix_cols + 1 << "-" << prefix_cols + 1 << "}\n"
      << "      \\cmidrule(lr){" << prefix_cols + 2 << "-" << prefix_cols + 4 << "}\n"
      << "      \\cmidrule(lr){" << prefix_cols + 5 << "-" << prefix_cols + 7 << "}\n"
      << "      \\cmidrule(lr){" << prefix_cols + 8 << "-" << prefix_cols + 8 << "}\n"
      << "      ";
  for (int i = 0; i < prefix_cols; i++) oss << "&";
  oss << "\n"
      << "      $R_{\\mathrm{succ}}$ [\\%]\n"
      << "      & $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms]\n"
      << "      & $T_{\\mathrm{replan}}$ [ms]\n"
      << "      & $T_{\\mathrm{STSFC}}$ [ms]\n"
      << "      & $T_{\\mathrm{trav}}$ [s]\n"
      << "      & $L_{\\mathrm{path}}$ [m]\n"
      << "      & $S_{\\mathrm{jerk}}$ [m/s$^{2}$]\n"
      << "      & $\\rho_{\\mathrm{cv}}$ [\\%]\n"
      << "      \\\\\n"
      << "      \\midrule\n";

  for (size_t ci = 0; ci < cases.size(); ci++) {
    const std::string& case_name = cases[ci];

    // Collect rows for this case (preserve config ordering)
    std::vector<const BatchRowData*> case_rows;
    for (auto& r : rows) {
      if (r.case_name == case_name) case_rows.push_back(&r);
    }
    if (case_rows.empty()) continue;

    int n_case_rows = static_cast<int>(case_rows.size());

    // Compute best/worst per metric for this case
    std::vector<double> best_vals(metrics.size()), worst_vals(metrics.size());
    for (size_t mi = 0; mi < metrics.size(); mi++) {
      std::vector<double> vals;
      for (auto* r : case_rows) {
        vals.push_back(get_metric(r->stats, metrics[mi].id));
      }
      if (vals.empty()) {
        best_vals[mi] = 0.0;
        worst_vals[mi] = 0.0;
        continue;
      }
      if (metrics[mi].higher_is_better) {
        best_vals[mi] = *std::max_element(vals.begin(), vals.end());
        worst_vals[mi] = *std::min_element(vals.begin(), vals.end());
      } else {
        best_vals[mi] = *std::min_element(vals.begin(), vals.end());
        worst_vals[mi] = *std::max_element(vals.begin(), vals.end());
      }
    }

    // Emit rows
    for (int ri = 0; ri < n_case_rows; ri++) {
      const auto* r = case_rows[ri];

      // Env cell (multirow on first row)
      std::string env_cell;
      if (ri == 0) {
        env_cell = "\\multirow{" + std::to_string(n_case_rows) + "}{*}{" + case_name + "}";
      }

      oss << "      " << env_cell;
      if (has_heat_weight_col) {
        oss << " & " << r->heat_weight;
      }
      oss << " & " << r->n_segments;

      // Data cells with best/worst highlighting
      for (size_t mi = 0; mi < metrics.size(); mi++) {
        double val = get_metric(r->stats, metrics[mi].id);
        oss << " & "
            << format_with_best_worst(val, best_vals[mi], worst_vals[mi], metrics[mi].precision);
      }

      oss << " \\\\\n";
    }

    // Midrule between cases (not after last)
    if (ci < cases.size() - 1) {
      oss << "      \\midrule\n";
    }
  }

  oss << "      \\bottomrule\n"
      << "    \\end{tabular}\n"
      << "  }\n"
      << "  \\vspace{-1.0em}\n"
      << "\\end{table*}";

  return oss.str();
}

// ============================================================================
// Temporal Ablation Table
// ============================================================================

struct TemporalAblationRow {
  std::string case_name;  // Easy, Medium, Hard
  std::string sfc_mode;   // "Worst-Case" or "SANDO2 (STSFC)"
  int n_segments = 2;     // N value
  Statistics stats;
};

static std::string generate_temporal_ablation_table(const std::vector<TemporalAblationRow>& rows) {
  // Group rows by case
  std::map<std::string, std::vector<const TemporalAblationRow*>> by_case;
  for (auto& r : rows) by_case[r.case_name].push_back(&r);

  // Metric extraction lambdas
  auto get_metrics = [](const TemporalAblationRow* r) -> std::vector<double> {
    const auto& s = r->stats;
    return {s.success_rate,
            s.avg_local_traj_time_mean,
            s.flight_travel_time_mean,
            s.path_length_mean,
            s.jerk_integral_mean,
            s.has_min_distance ? s.min_distance_to_obstacles_mean : -1e18,
            s.vel_violation_rate,
            s.acc_violation_rate,
            s.jerk_violation_rate};
  };

  // higher_is_better: success_rate (idx 0), min_dist (idx 5)
  // lower_is_better: comp_time (1), travel_time (2), path_length (3), jerk (4), vel_viol (6),
  // acc_viol (7), jerk_viol (8)
  auto is_higher_better = [](int idx) { return idx == 0 || idx == 5; };

  // Build the table
  std::ostringstream oss;
  oss << std::fixed;

  oss << "\\begin{table*}\n"
      << "  \\caption{Ablation study: temporal safe flight corridor (SFC) vs worst-case SFC. "
      << "The temporal approach uses per-layer obstacle inflation $r = v_{\\max}^{\\mathrm{obs}} "
         "\\times t_n$, "
      << "while the worst-case baseline inflates all obstacles by the maximum time horizon. "
      << "We highlight the \\best{better} and \\worst{worse} value for each case.}\n"
      << "  \\label{tab:temporal_sfc_ablation}\n"
      << "  \\centering\n"
      << "  \\renewcommand{\\arraystretch}{1.2}\n"
      << "  \\resizebox{\\textwidth}{!}{\n"
      << "    \\begin{tabular}{c c c c c c c c c c c c}\n"
      << "      \\toprule\n"
      << "      \\multirow{2}{*}[-0.4em]{\\textbf{Case}}\n"
      << "      & \\multirow{2}{*}[-0.4em]{\\textbf{SFC Mode}}\n"
      << "      & \\multirow{2}{*}[-0.4em]{\\textbf{$N$}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Success}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Comp. Time}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Performance}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Safety}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}\n"
      << "      \\\\\n"
      << "      \\cmidrule(lr){4-4}\n"
      << "      \\cmidrule(lr){5-5}\n"
      << "      \\cmidrule(lr){6-8}\n"
      << "      \\cmidrule(lr){9-9}\n"
      << "      \\cmidrule(lr){10-12}\n"
      << "      &&&\n"
      << "      $R_{\\mathrm{succ}}$ [\\%] &\n"
      << "      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &\n"
      << "      $T_{\\mathrm{trav}}$ [s] &\n"
      << "      $L_{\\mathrm{path}}$ [m] &\n"
      << "      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &\n"
      << "      $d_{\\mathrm{min}}$ [m] &\n"
      << "      $\\rho_{\\mathrm{vel}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{acc}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{jerk}}$ [\\%]\n"
      << "      \\\\\n";

  std::vector<std::string> case_order = {"Easy", "Medium", "Hard"};

  for (size_t ci = 0; ci < case_order.size(); ci++) {
    const auto& cn = case_order[ci];
    oss << "      \\midrule\n";

    auto it = by_case.find(cn);
    if (it == by_case.end()) continue;

    auto& case_rows = it->second;

    // Compute best/worst per metric for this case
    int num_metrics = 9;
    std::vector<double> best_val(num_metrics), worst_val(num_metrics);
    for (int m = 0; m < num_metrics; m++) {
      best_val[m] = is_higher_better(m) ? -1e18 : 1e18;
      worst_val[m] = is_higher_better(m) ? 1e18 : -1e18;
    }

    std::vector<std::vector<double>> all_metrics;
    for (auto* r : case_rows) {
      auto metrics = get_metrics(r);
      all_metrics.push_back(metrics);
      for (int m = 0; m < num_metrics; m++) {
        if (is_higher_better(m)) {
          best_val[m] = std::max(best_val[m], metrics[m]);
          worst_val[m] = std::min(worst_val[m], metrics[m]);
        } else {
          best_val[m] = std::min(best_val[m], metrics[m]);
          worst_val[m] = std::max(worst_val[m], metrics[m]);
        }
      }
    }

    for (size_t ri = 0; ri < case_rows.size(); ri++) {
      auto* r = case_rows[ri];
      auto& metrics = all_metrics[ri];

      if (ri == 0) {
        oss << "      \\multirow{" << case_rows.size() << "}{*}{" << cn << "}";
      } else {
        oss << "      ";
      }
      oss << " & " << r->sfc_mode << " & " << r->n_segments << " & ";

      // Format each metric with best/worst wrapping
      std::vector<int> precisions = {1, 1, 1, 1, 1, 3, 1, 1, 1};
      for (int m = 0; m < num_metrics; m++) {
        std::ostringstream val;
        val << std::fixed << std::setprecision(precisions[m]) << metrics[m];

        bool all_equal = (std::abs(best_val[m] - worst_val[m]) < 1e-9);
        if (all_equal) {
          oss << "\\best{" << val.str() << "}";
        } else if (std::abs(metrics[m] - best_val[m]) < 1e-9) {
          oss << "\\best{" << val.str() << "}";
        } else if (std::abs(metrics[m] - worst_val[m]) < 1e-9) {
          oss << "\\worst{" << val.str() << "}";
        } else {
          oss << val.str();
        }

        if (m < num_metrics - 1) oss << " & ";
      }
      oss << " \\\\\n";
    }
  }

  oss << "      \\bottomrule\n"
      << "    \\end{tabular}\n"
      << "  }\n"
      << "  \\vspace{-1.0em}\n"
      << "\\end{table*}";

  return oss.str();
}

// ============================================================================
// CLI
// ============================================================================

struct Args {
  std::string data_dir;
  std::string output_name = "benchmark_summary";
  std::string latex_name = "dynamic_benchmark.tex";
  std::string config_name = "default";
  bool all_cases = false;
  std::string table_type = "dynamic";
  Vec3 goal_pos = {105.0, 0.0, 2.0};
  std::string algo_name = "SANDO";
  std::string single_config;  // process only this config subdir
  std::string merge_table;    // path to existing .tex table to merge into
  std::string data_dir2;      // second data directory (for temporal ablation)
  std::string latex_dir;      // output directory for LaTeX tables
};

static void print_usage() {
  std::cout
      << "Usage: analyze_benchmark [options]\n"
      << "  --data-dir DIR        Path to benchmark data directory (required)\n"
      << "  --output-name NAME    Output filename prefix (default: benchmark_summary)\n"
      << "  --latex-name NAME     LaTeX table filename (default: dynamic_benchmark.tex)\n"
      << "  --config-name NAME    Configuration name (default: default)\n"
      << "  --all-cases           Analyze all cases (easy, medium, hard)\n"
      << "  --table-type TYPE     Table format: dynamic, static, unknown_dynamic, "
         "temporal_ablation (default: dynamic)\n"
      << "  --goal-pos X Y Z      Goal position (default: 105.0 0.0 2.0)\n"
      << "  --algo-name NAME      Algorithm name in LaTeX table (default: SANDO)\n"
      << "  --single-config NAME  Process only this config subdir (e.g. "
         "inflate_unknown_voxels_heat_w_5_N_2)\n"
      << "  --merge-table PATH    Merge new config into existing .tex table (recomputes "
         "best/worst)\n"
      << "  --data-dir2 DIR       Second data directory (for temporal_ablation: STSFC data)\n"
      << "  --latex-dir DIR       Output directory for LaTeX tables (required)\n"
      << "  --recompute           Force recompute from bags (ignore cached stats_cache.json)\n";
}

static Args parse_args(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--data-dir" && i + 1 < argc) {
      args.data_dir = argv[++i];
    } else if (arg == "--output-name" && i + 1 < argc) {
      args.output_name = argv[++i];
    } else if (arg == "--latex-name" && i + 1 < argc) {
      args.latex_name = argv[++i];
    } else if (arg == "--config-name" && i + 1 < argc) {
      args.config_name = argv[++i];
    } else if (arg == "--all-cases") {
      args.all_cases = true;
    } else if (arg == "--table-type" && i + 1 < argc) {
      args.table_type = argv[++i];
    } else if (arg == "--algo-name" && i + 1 < argc) {
      args.algo_name = argv[++i];
    } else if (arg == "--single-config" && i + 1 < argc) {
      args.single_config = argv[++i];
    } else if (arg == "--merge-table" && i + 1 < argc) {
      args.merge_table = argv[++i];
    } else if (arg == "--data-dir2" && i + 1 < argc) {
      args.data_dir2 = argv[++i];
    } else if (arg == "--latex-dir" && i + 1 < argc) {
      args.latex_dir = argv[++i];
    } else if (arg == "--recompute") {
      g_recompute = true;
    } else if (arg == "--goal-pos" && i + 3 < argc) {
      args.goal_pos.x = std::stod(argv[++i]);
      args.goal_pos.y = std::stod(argv[++i]);
      args.goal_pos.z = std::stod(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      exit(0);
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      print_usage();
      exit(1);
    }
  }

  if (args.data_dir.empty()) {
    std::cerr << "ERROR: --data-dir is required\n";
    print_usage();
    exit(1);
  }

  if (args.latex_dir.empty()) {
    std::cerr << "ERROR: --latex-dir is required\n";
    print_usage();
    exit(1);
  }

  // Auto-set latex-name
  if (args.table_type == "unknown_dynamic" && args.latex_name == "dynamic_benchmark.tex") {
    args.latex_name = "unknown_dynamic_sim.tex";
  } else if (args.table_type == "temporal_ablation" && args.latex_name == "dynamic_benchmark.tex") {
    args.latex_name = "temporal_sfc_ablation.tex";
  }

  return args;
}

int main(int argc, char** argv) {
  auto args = parse_args(argc, argv);

  fs::path latex_output = fs::path(args.latex_dir) / args.latex_name;

  if (args.all_cases) {
    fs::path base_dir(args.data_dir);

    if (!fs::exists(base_dir) || !fs::is_directory(base_dir)) {
      std::cerr << "ERROR: Directory does not exist: " << base_dir << "\n";
      return 1;
    }

    // Check for unknown_dynamic batch mode (data_dir contains config subdirs)
    if (args.table_type == "unknown_dynamic") {
      // Single-config + merge mode: process one config and merge into existing table
      if (!args.single_config.empty() && !args.merge_table.empty()) {
        std::string sep(80, '=');

        // Parse the single config directory name
        std::regex config_re(R"((no_?)?inflate_unknown_voxels_heat_w_(\d+)(?:_N_(\d+))?)");
        std::smatch match;
        if (!std::regex_match(args.single_config, match, config_re)) {
          std::cerr << "ERROR: --single-config name doesn't match expected pattern: "
                    << args.single_config << "\n";
          return 1;
        }
        UnknownDynamicConfig cfg;
        cfg.dir_name = args.single_config;
        cfg.unk_infl = match[1].str().empty();
        cfg.heat_weight = std::stoi(match[2]);
        cfg.n_segments = match[3].matched ? std::stoi(match[3]) : 3;

        std::cout << sep << "\n";
        std::cout << "SINGLE CONFIG MODE: " << cfg.dir_name << " (w=" << cfg.heat_weight
                  << ", N=" << cfg.n_segments << ")\n";
        std::cout << "Merging into: " << args.merge_table << "\n";
        std::cout << sep << "\n\n";

        // Process only this config
        std::vector<BatchRowData> new_rows;
        fs::path cfg_dir = base_dir / cfg.dir_name;
        if (!fs::exists(cfg_dir)) {
          std::cerr << "ERROR: Config directory does not exist: " << cfg_dir << "\n";
          return 1;
        }

        for (auto& pattern : {"easy_", "medium_", "hard_"}) {
          std::vector<fs::path> matching;
          for (auto& entry : fs::directory_iterator(cfg_dir)) {
            if (entry.is_directory() && entry.path().filename().string().find(pattern) == 0) {
              matching.push_back(entry.path());
            }
          }
          if (!matching.empty()) {
            std::sort(matching.begin(), matching.end());
            fs::path case_dir = matching.back();

            auto stats =
                analyze_single_case(case_dir, args.output_name, latex_output, "unknown_dynamic",
                                    args.goal_pos, args.algo_name, true);

            BatchRowData row;
            row.case_name = extract_case_name(case_dir);
            row.heat_weight = cfg.heat_weight;
            row.n_segments = cfg.n_segments;
            row.unk_infl = cfg.unk_infl;
            row.stats = stats;
            new_rows.push_back(row);
          }
        }

        // Parse existing table
        std::cout << "\nParsing existing table: " << args.merge_table << "\n";
        auto existing_rows = parse_unknown_dynamic_table(args.merge_table);
        std::cout << "  Found " << existing_rows.size() << " existing rows\n";

        // Remove any existing rows with same (case_name, heat_weight, n_segments) to avoid
        // duplicates
        for (auto& nr : new_rows) {
          existing_rows.erase(std::remove_if(existing_rows.begin(), existing_rows.end(),
                                             [&](const BatchRowData& er) {
                                               return er.case_name == nr.case_name &&
                                                      er.heat_weight == nr.heat_weight &&
                                                      er.n_segments == nr.n_segments;
                                             }),
                              existing_rows.end());
        }

        // Merge
        std::vector<BatchRowData> all_rows;
        all_rows.insert(all_rows.end(), existing_rows.begin(), existing_rows.end());
        all_rows.insert(all_rows.end(), new_rows.begin(), new_rows.end());

        // Sort: by case (Easy, Medium, Hard), then heat_weight, then n_segments
        auto case_order = [](const std::string& c) {
          if (c == "Easy") return 0;
          if (c == "Medium") return 1;
          if (c == "Hard") return 2;
          return 3;
        };
        std::sort(all_rows.begin(), all_rows.end(), [&](const auto& a, const auto& b) {
          int oa = case_order(a.case_name), ob = case_order(b.case_name);
          if (oa != ob) return oa < ob;
          if (a.heat_weight != b.heat_weight) return a.heat_weight < b.heat_weight;
          return a.n_segments < b.n_segments;  // N=2 before N=3
        });

        std::cout << "  Merged total: " << all_rows.size() << " rows\n";
        for (auto& r : all_rows) {
          std::cout << "    " << r.case_name << " w=" << r.heat_weight << " N=" << r.n_segments
                    << "\n";
        }

        // Generate merged table with recomputed best/worst highlighting
        std::string table = generate_unknown_dynamic_batch_table(all_rows);
        fs::create_directories(latex_output.parent_path());
        std::ofstream tex_file(latex_output);
        tex_file << table;
        tex_file.close();

        std::cout << "\n" << sep << "\n";
        std::cout << "MERGED TABLE GENERATED: " << latex_output << "\n";
        std::cout << sep << "\n\n";
        return 0;
      }

      // Check for simple N_* or P_* subdirectories (e.g., N_2, N_3, P_2, P_3)
      {
        std::vector<std::pair<int, fs::path>> n_dirs;
        std::regex np_re(R"([NP]_(\d+))");
        bool use_p_label = true;  // always use $P$ in the table header
        for (auto& entry : fs::directory_iterator(base_dir)) {
          if (!entry.is_directory()) continue;
          std::string name = entry.path().filename().string();
          std::smatch m;
          if (std::regex_match(name, m, np_re)) {
            n_dirs.push_back({std::stoi(m[1]), entry.path()});
          }
        }
        std::sort(n_dirs.begin(), n_dirs.end());

        if (!n_dirs.empty()) {
          std::string label = use_p_label ? "P" : "N";
          std::string sep(80, '=');
          std::cout << sep << "\n";
          std::cout << "UNKNOWN DYNAMIC " << label << "-COMPARISON MODE - " << n_dirs.size() << " "
                    << label << " values\n";
          std::cout << sep << "\n\n";

          for (auto& [n_val, n_dir] : n_dirs) {
            std::cout << "  " << label << "=" << n_val << " (" << n_dir << ")\n";
          }
          std::cout << "\n";

          std::vector<BatchRowData> batch_rows;

          for (auto& [n_val, n_dir] : n_dirs) {
            std::cout << sep << "\n";
            std::cout << "PROCESSING: " << label << "=" << n_val << "\n";
            std::cout << sep << "\n";

            for (auto& pattern : {"easy_", "medium_", "hard_"}) {
              std::vector<fs::path> matching;
              for (auto& entry : fs::directory_iterator(n_dir)) {
                if (entry.is_directory() && entry.path().filename().string().find(pattern) == 0) {
                  matching.push_back(entry.path());
                }
              }
              if (!matching.empty()) {
                std::sort(matching.begin(), matching.end());
                fs::path case_dir = matching.back();

                auto stats =
                    analyze_single_case(case_dir, args.output_name, latex_output, "unknown_dynamic",
                                        args.goal_pos, args.algo_name, true);

                BatchRowData row;
                row.case_name = extract_case_name(case_dir);
                row.heat_weight = 0;
                row.n_segments = n_val;
                row.unk_infl = true;
                row.stats = stats;
                batch_rows.push_back(row);
              }
            }
          }

          // Sort: by case (Easy, Medium, Hard), then N/P (descending: 3 before 2)
          auto case_order = [](const std::string& c) {
            if (c == "Easy") return 0;
            if (c == "Medium") return 1;
            if (c == "Hard") return 2;
            return 3;
          };
          std::sort(batch_rows.begin(), batch_rows.end(), [&](const auto& a, const auto& b) {
            int oa = case_order(a.case_name), ob = case_order(b.case_name);
            if (oa != ob) return oa < ob;
            return a.n_segments < b.n_segments;
          });

          std::cout << "\nGenerating batch LaTeX table...\n";
          std::string table = generate_unknown_dynamic_batch_table(batch_rows, use_p_label);
          fs::create_directories(latex_output.parent_path());
          std::ofstream tex_file(latex_output);
          tex_file << table;
          tex_file.close();

          std::cout << "\n" << sep << "\n";
          std::cout << "BATCH TABLE GENERATED: " << latex_output << "\n";
          std::cout << sep << "\n\n";
          return 0;
        }
      }

      // Check for inflate_unknown_voxels_* config subdirectories (ablation mode)
      auto ud_configs = discover_unknown_dynamic_configs(base_dir);
      if (!ud_configs.empty()) {
        std::string sep(80, '=');
        std::cout << sep << "\n";
        std::cout << "UNKNOWN DYNAMIC BATCH MODE - " << ud_configs.size() << " configurations\n";
        std::cout << sep << "\n\n";

        for (auto& cfg : ud_configs) {
          std::cout << "  " << cfg.dir_name << " (w=" << cfg.heat_weight << ", N=" << cfg.n_segments
                    << ")\n";
        }
        std::cout << "\n";

        std::vector<BatchRowData> batch_rows;

        for (auto& cfg : ud_configs) {
          fs::path cfg_dir = base_dir / cfg.dir_name;
          std::cout << sep << "\n";
          std::cout << "CONFIG: " << cfg.dir_name << "\n";
          std::cout << sep << "\n";

          for (auto& pattern : {"easy_", "medium_", "hard_"}) {
            std::vector<fs::path> matching;
            for (auto& entry : fs::directory_iterator(cfg_dir)) {
              if (entry.is_directory() && entry.path().filename().string().find(pattern) == 0) {
                matching.push_back(entry.path());
              }
            }
            if (!matching.empty()) {
              std::sort(matching.begin(), matching.end());
              fs::path case_dir = matching.back();

              auto stats =
                  analyze_single_case(case_dir, args.output_name, latex_output, "unknown_dynamic",
                                      args.goal_pos, args.algo_name, true);

              BatchRowData row;
              row.case_name = extract_case_name(case_dir);
              row.heat_weight = cfg.heat_weight;
              row.n_segments = cfg.n_segments;
              row.unk_infl = cfg.unk_infl;
              row.stats = stats;
              batch_rows.push_back(row);
            }
          }
        }

        // Generate batch table with best/worst highlighting
        std::cout << "\nGenerating batch LaTeX table with best/worst highlighting...\n";
        std::string table = generate_unknown_dynamic_batch_table(batch_rows);
        fs::create_directories(latex_output.parent_path());
        std::ofstream tex_file(latex_output);
        tex_file << table;
        tex_file.close();

        std::cout << "\n" << sep << "\n";
        std::cout << "BATCH TABLE GENERATED: " << latex_output << "\n";
        std::cout << sep << "\n\n";
        return 0;
      }
    }

    // Temporal ablation mode
    if (args.table_type == "temporal_ablation") {
      std::string sep(80, '=');
      std::cout << sep << "\n";
      std::cout << "TEMPORAL ABLATION MODE\n";
      std::cout << sep << "\n\n";

      // data_dir  = worst-case SFC data (dynamic_worst_case)
      // data_dir2 = STSFC data (dynamic)
      std::vector<std::pair<fs::path, std::string>> source_dirs = {{base_dir, "Worst-Case"}};
      if (!args.data_dir2.empty()) {
        source_dirs.push_back({fs::path(args.data_dir2), "SANDO2 (STSFC)"});
      }

      std::vector<TemporalAblationRow> all_rows;

      // Helper to process N_* subdirs or flat case dirs within a source
      auto process_source = [&](const fs::path& src_dir, const std::string& mode_label) {
        // Check for N_* subdirs
        std::vector<std::pair<int, fs::path>> n_dirs;
        for (auto& entry : fs::directory_iterator(src_dir)) {
          if (!entry.is_directory()) continue;
          std::string name = entry.path().filename().string();
          std::regex n_re(R"(N_(\d+))");
          std::smatch m;
          if (std::regex_match(name, m, n_re)) {
            n_dirs.push_back({std::stoi(m[1]), entry.path()});
          }
        }
        std::sort(n_dirs.begin(), n_dirs.end());

        if (!n_dirs.empty()) {
          for (auto& [n_val, n_dir] : n_dirs) {
            std::cout << sep << "\n";
            std::cout << "PROCESSING: " << mode_label << " N=" << n_val << " (" << n_dir << ")\n";
            std::cout << sep << "\n";

            for (auto& case_name : std::vector<std::string>{"easy", "medium", "hard"}) {
              // Try exact name first, then prefix match
              fs::path case_dir = n_dir / case_name;
              if (!fs::exists(case_dir)) {
                // Try prefix match (easy_*)
                std::vector<fs::path> matching;
                for (auto& e : fs::directory_iterator(n_dir)) {
                  if (e.is_directory() && e.path().filename().string().find(case_name + "_") == 0) {
                    matching.push_back(e.path());
                  }
                }
                if (!matching.empty()) {
                  std::sort(matching.begin(), matching.end());
                  case_dir = matching.back();
                } else {
                  std::cerr << "Warning: no " << case_name << " dir in " << n_dir << ", skipping\n";
                  continue;
                }
              }

              auto stats = analyze_single_case(case_dir, args.output_name, latex_output, "dynamic",
                                               args.goal_pos, args.algo_name, true);

              TemporalAblationRow row;
              row.case_name = extract_case_name(case_dir);
              row.sfc_mode = mode_label;
              row.n_segments = n_val;
              row.stats = stats;
              all_rows.push_back(row);
            }
          }
        } else {
          // Flat structure: temporal/ and worst_case/ subdirs (legacy)
          for (auto& case_name : std::vector<std::string>{"easy", "medium", "hard"}) {
            fs::path case_dir = src_dir / case_name;
            if (!fs::exists(case_dir)) {
              std::cerr << "Warning: " << case_dir << " not found, skipping\n";
              continue;
            }

            auto stats = analyze_single_case(case_dir, args.output_name, latex_output, "dynamic",
                                             args.goal_pos, args.algo_name, true);

            TemporalAblationRow row;
            row.case_name = extract_case_name(case_dir);
            row.sfc_mode = mode_label;
            row.n_segments = 2;
            row.stats = stats;
            all_rows.push_back(row);
          }
        }
      };

      for (auto& [src_dir, mode_label] : source_dirs) {
        process_source(src_dir, mode_label);
      }

      std::string table = generate_temporal_ablation_table(all_rows);
      fs::create_directories(latex_output.parent_path());
      std::ofstream tex_file(latex_output);
      tex_file << table;
      tex_file.close();

      std::cout << "\n" << sep << "\n";
      std::cout << "TEMPORAL ABLATION TABLE GENERATED: " << latex_output << "\n";
      std::cout << sep << "\n\n";
      return 0;
    }

    // Find case directories
    std::vector<fs::path> case_dirs;
    for (auto& pattern : {"easy_", "medium_", "hard_"}) {
      std::vector<fs::path> matching;
      for (auto& entry : fs::directory_iterator(base_dir)) {
        if (entry.is_directory() && entry.path().filename().string().find(pattern) == 0) {
          matching.push_back(entry.path());
        }
      }
      if (!matching.empty()) {
        std::sort(matching.begin(), matching.end());
        case_dirs.push_back(matching.back());  // Most recent
      }
    }

    // Dynamic merge-table mode: process cases, then merge into existing table
    if (args.table_type == "dynamic" && !args.merge_table.empty()) {
      std::string sep(80, '=');
      // Check for N_* subdirectories (multiple SANDO variants)
      std::vector<std::pair<std::string, fs::path>> n_configs;  // (label_suffix, dir)
      for (auto& entry : fs::directory_iterator(base_dir)) {
        if (!entry.is_directory()) continue;
        std::string name = entry.path().filename().string();
        std::regex n_re(R"(N_(\d+))");
        std::smatch m;
        if (std::regex_match(name, m, n_re)) {
          n_configs.push_back({name, entry.path()});
        }
      }
      std::sort(n_configs.begin(), n_configs.end());

      std::map<std::string, std::vector<std::string>> sando_rows;

      if (!n_configs.empty()) {
        // Multiple SANDO variants (N_2, N_3, etc.)
        for (auto& [n_label, n_dir] : n_configs) {
          std::string n_val = n_label.substr(2);  // "2" from "N_2"
          std::string algo = "SANDO ($N$=" + n_val + ")";

          std::cout << "\n" << sep << "\n";
          std::cout << "PROCESSING: " << n_label << " -> " << algo << "\n";
          std::cout << sep << "\n";

          // Find case dirs within this N_* subdir
          for (auto& pattern : {"easy_", "medium_", "hard_"}) {
            std::vector<fs::path> matching;
            for (auto& e : fs::directory_iterator(n_dir)) {
              if (e.is_directory() && e.path().filename().string().find(pattern) == 0) {
                matching.push_back(e.path());
              }
            }
            if (!matching.empty()) {
              std::sort(matching.begin(), matching.end());
              fs::path case_dir = matching.back();
              auto stats = analyze_single_case(case_dir, args.output_name, latex_output,
                                               args.table_type, args.goal_pos, algo, true);
              std::string case_name = extract_case_name(case_dir);
              std::string row = generate_sando_row(stats, case_name, args.table_type, algo);
              sando_rows[case_name].push_back(row);
              std::cout << "  " << case_name << " row: " << row << "\n\n";
            }
          }
        }
      } else {
        // Single SANDO variant (flat structure)
        for (auto& case_dir : case_dirs) {
          auto stats = analyze_single_case(case_dir, args.output_name, latex_output,
                                           args.table_type, args.goal_pos, args.algo_name, true);
          std::string case_name = extract_case_name(case_dir);
          std::string row = generate_sando_row(stats, case_name, args.table_type, args.algo_name);
          sando_rows[case_name].push_back(row);
          std::cout << "  " << case_name << " row: " << row << "\n\n";
        }
      }

      std::cout << "Merging into: " << args.merge_table << "\n";
      std::string merged = merge_dynamic_table(args.merge_table, sando_rows);
      if (!merged.empty()) {
        fs::create_directories(latex_output.parent_path());
        std::ofstream tex_file(latex_output);
        tex_file << merged;
        tex_file.close();
        std::cout << "\n" << sep << "\n";
        std::cout << "MERGED TABLE GENERATED: " << latex_output << "\n";
        std::cout << sep << "\n\n";
      } else {
        std::cerr << "ERROR: Failed to merge table\n";
        return 1;
      }
      return 0;
    }

    if (case_dirs.empty()) {
      std::cerr << "Error: No case directories (easy_*, medium_*, hard_*) found in " << base_dir
                << "\n";
      return 1;
    }

    std::string sep(80, '=');
    std::cout << sep << "\n";
    std::cout << "SANDO BENCHMARK ANALYZER (C++) - ALL CASES\n";
    std::cout << sep << "\n\n";
    std::cout << "Found " << case_dirs.size() << " case(s) to analyze:\n";
    for (auto& d : case_dirs) std::cout << "  - " << d.filename().string() << "\n";
    std::cout << "\n";

    for (auto& case_dir : case_dirs) {
      analyze_single_case(case_dir, args.output_name, latex_output, args.table_type, args.goal_pos,
                          args.algo_name);
    }

    std::cout << "\n" << sep << "\n";
    std::cout << "ALL CASES ANALYSIS COMPLETE\n";
    std::cout << sep << "\n";
    std::cout << "\nLaTeX table updated: " << latex_output << "\n\n";
  } else {
    fs::path single_dir(args.data_dir);
    if (!fs::exists(single_dir) || !fs::is_directory(single_dir)) {
      std::cerr << "ERROR: Directory does not exist: " << single_dir << "\n";
      return 1;
    }
    analyze_single_case(single_dir, args.output_name, latex_output, args.table_type, args.goal_pos,
                        args.algo_name);
  }

  return 0;
}
