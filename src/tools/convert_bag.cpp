/* ----------------------------------------------------------------------------
 * Copyright (c) Anonymous Author
 * Anonymous Institution
 * All Rights Reserved
 * Authors: Anonymous
 * See LICENSE file for the license information
 *
 * Fast bag converter: adjusts marker lifetimes and z-filters pointclouds.
 * -------------------------------------------------------------------------- */

#include <cstring>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <filesystem>
#include <iostream>
#include <map>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace fs = std::filesystem;

// ============================================================================

static builtin_interfaces::msg::Duration make_duration(double seconds) {
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(seconds);
  d.nanosec = static_cast<uint32_t>((seconds - d.sec) * 1e9);
  return d;
}

// Helper: copy rclcpp::SerializedMessage back into bag_msg->serialized_data
static void copy_serialized_to_bag_msg(
    const rclcpp::SerializedMessage& serialized_out,
    std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_msg) {
  const auto& rcl_out = serialized_out.get_rcl_serialized_message();
  auto* sd = bag_msg->serialized_data.get();
  // Resize the existing buffer
  auto ret = rcutils_uint8_array_resize(sd, rcl_out.buffer_length);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Failed to resize serialized data buffer");
  }
  std::memcpy(sd->buffer, rcl_out.buffer, rcl_out.buffer_length);
  sd->buffer_length = rcl_out.buffer_length;
}

// Helper: wrap bag_msg serialized data as rclcpp::SerializedMessage (no copy for read)
static rclcpp::SerializedMessage wrap_bag_msg(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_msg) {
  rclcpp::SerializedMessage serialized_in(bag_msg->serialized_data->buffer_length);
  auto& rcl_msg = serialized_in.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, bag_msg->serialized_data->buffer,
              bag_msg->serialized_data->buffer_length);
  rcl_msg.buffer_length = bag_msg->serialized_data->buffer_length;
  return serialized_in;
}

// Deserialize, modify, reserialize, and write back into bag_msg
template <typename MsgT>
void transform_in_place(std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_msg,
                        std::function<void(MsgT&)> modifier) {
  rclcpp::Serialization<MsgT> serializer;
  auto serialized_in = wrap_bag_msg(bag_msg);
  MsgT msg;
  serializer.deserialize_message(&serialized_in, &msg);
  modifier(msg);
  rclcpp::SerializedMessage serialized_out;
  serializer.serialize_message(&msg, &serialized_out);
  copy_serialized_to_bag_msg(serialized_out, bag_msg);
}

// Filter PointCloud2 by z_max (operates on deserialized message)
static void filter_pc_z(sensor_msgs::msg::PointCloud2& cloud, float z_max) {
  // Find z field offset
  int z_offset = -1;
  for (const auto& field : cloud.fields) {
    if (field.name == "z") {
      z_offset = field.offset;
      break;
    }
  }
  if (z_offset < 0) return;

  size_t point_step = cloud.point_step;
  size_t num_points = cloud.width * cloud.height;

  // Filter in-place: copy kept points to front
  size_t kept = 0;
  for (size_t i = 0; i < num_points; ++i) {
    float z;
    std::memcpy(&z, &cloud.data[i * point_step + z_offset], sizeof(float));
    if (z <= z_max) {
      if (kept != i) {
        std::memcpy(&cloud.data[kept * point_step], &cloud.data[i * point_step], point_step);
      }
      ++kept;
    }
  }

  cloud.width = static_cast<uint32_t>(kept);
  cloud.height = 1;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(kept * point_step);
}

// ============================================================================

void print_usage(const char* prog) {
  std::cout << "Usage: " << prog
            << " <input_bag> [--lifetime <sec>] [--z-max <val>] "
               "[--pc-topics <topic1> <topic2> ...]\n"
            << "\nOutput is written to {input_bag}_improved\n"
            << "\nOptions:\n"
            << "  --lifetime <sec>   Marker lifetime (default: 0.1)\n"
            << "  --z-max <val>      Max z for pointcloud filtering\n"
            << "  --pc-topics <...>  PointCloud2 topics to z-filter\n";
}

int main(int argc, char** argv) {
  if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
    print_usage(argv[0]);
    return (argc < 2) ? 1 : 0;
  }

  std::string input_path = argv[1];
  double lifetime_sec = 0.1;
  float z_max = 0.0f;
  bool use_z_filter = false;
  std::set<std::string> pc_topics;

  // Parse args
  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--lifetime" && i + 1 < argc) {
      lifetime_sec = std::stod(argv[++i]);
    } else if (arg == "--z-max" && i + 1 < argc) {
      z_max = std::stof(argv[++i]);
      use_z_filter = true;
    } else if (arg == "--pc-topics") {
      ++i;
      while (i < argc && argv[i][0] != '-') {
        pc_topics.insert(argv[i]);
        ++i;
      }
      --i;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
  }

  std::string output_path = input_path + "_improved";
  if (fs::exists(output_path)) {
    std::cerr << "Error: output path " << output_path << " already exists\n";
    return 1;
  }

  auto duration = make_duration(lifetime_sec);
  std::cout << "Marker lifetime: " << lifetime_sec << "s\n";
  if (use_z_filter) {
    std::cout << "Z-filter: z_max=" << z_max << " on " << pc_topics.size() << " topics\n";
  }

  // Open reader
  rosbag2_cpp::Reader reader;
  reader.open(input_path);

  // Collect topic types
  auto metadata = reader.get_metadata();
  std::map<std::string, std::string> topic_type_map;
  for (const auto& topic_info : metadata.topics_with_message_count) {
    topic_type_map[topic_info.topic_metadata.name] = topic_info.topic_metadata.type;
  }

  // Open writer
  rosbag2_cpp::Writer writer;
  writer.open(output_path);

  for (const auto& topic_info : metadata.topics_with_message_count) {
    writer.create_topic(topic_info.topic_metadata);
  }

  size_t count = 0;
  size_t modified = 0;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    ++count;

    const auto& topic = bag_msg->topic_name;
    const auto& type_str = topic_type_map[topic];

    bool did_modify = false;

    if (type_str == "visualization_msgs/msg/MarkerArray") {
      transform_in_place<visualization_msgs::msg::MarkerArray>(bag_msg, [&](auto& msg) {
        for (auto& m : msg.markers) m.lifetime = duration;
      });
      did_modify = true;

    } else if (type_str == "visualization_msgs/msg/Marker") {
      transform_in_place<visualization_msgs::msg::Marker>(
          bag_msg, [&](auto& msg) { msg.lifetime = duration; });
      did_modify = true;

    } else if (type_str == "decomp_ros_msgs/msg/PolyhedronArray") {
      transform_in_place<decomp_ros_msgs::msg::PolyhedronArray>(
          bag_msg, [&](auto& msg) { msg.lifetime = duration; });
      did_modify = true;

    } else if (type_str == "sensor_msgs/msg/PointCloud2" && use_z_filter &&
               pc_topics.count(topic)) {
      transform_in_place<sensor_msgs::msg::PointCloud2>(
          bag_msg, [&](auto& msg) { filter_pc_z(msg, z_max); });
      did_modify = true;
    }

    if (did_modify) ++modified;

    writer.write(bag_msg);

    if (count % 10000 == 0) {
      std::cout << "  Processed " << count << " messages (" << modified << " modified)...\n";
    }
  }

  std::cout << "Done! Processed " << count << " messages total, " << modified << " modified.\n";
  std::cout << "Output bag: " << output_path << "\n";

  return 0;
}
