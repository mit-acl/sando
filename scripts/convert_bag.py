#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
# Massachusetts Institute of Technology
# All Rights Reserved
# Authors: Kota Kondo, et al.
# See LICENSE file for the license information
# ----------------------------------------------------------------------------
"""
Convert a ROS 2 bag: adjust marker lifetimes and optionally filter pointclouds by z.

Usage:
  python3 convert_bag.py /path/to/input_bag [--z-max 2.0] [--pc-topics /PX03/occupancy_grid]

Output bag is written to {input_bag}_improved.

By default:
  - Marker/MarkerArray lifetimes are set to 0.1s (use --lifetime to change)
  - PointCloud2 topics listed in --pc-topics get z-filtered
"""

import argparse
from pathlib import Path

from rosbag2_py import (
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata,
)
from rclpy.serialization import deserialize_message, serialize_message
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from decomp_ros_msgs.msg import PolyhedronArray
from builtin_interfaces.msg import Duration


def make_duration(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def set_marker_lifetime_array(msg: MarkerArray, lifetime: Duration) -> MarkerArray:
    for m in msg.markers:
        m.lifetime = lifetime
    return msg


def set_marker_lifetime(msg: Marker, lifetime: Duration) -> Marker:
    msg.lifetime = lifetime
    return msg


def set_poly_lifetime(msg: PolyhedronArray, lifetime: Duration) -> PolyhedronArray:
    msg.lifetime = lifetime
    return msg


def filter_pc_z(msg: PointCloud2, z_max: float) -> PointCloud2:
    points = list(point_cloud2.read_points(msg, field_names=None, skip_nans=True))
    filtered = [p for p in points if p[2] <= z_max]
    return point_cloud2.create_cloud(
        msg.header, msg.fields, filtered if filtered else []
    )


TYPE_HANDLERS = {
    "visualization_msgs/msg/MarkerArray": (MarkerArray, set_marker_lifetime_array),
    "visualization_msgs/msg/Marker": (Marker, set_marker_lifetime),
    "decomp_ros_msgs/msg/PolyhedronArray": (PolyhedronArray, set_poly_lifetime),
}


def main():
    parser = argparse.ArgumentParser(
        description="Convert bag: adjust lifetimes + z-filter pointclouds"
    )
    parser.add_argument("input_bag", help="Path to input bag directory")
    parser.add_argument(
        "--lifetime",
        type=float,
        default=0.1,
        help="Marker lifetime in seconds (default: 0.1)",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=None,
        help="Max z for pointcloud filtering (disabled if not set)",
    )
    parser.add_argument(
        "--pc-topics", nargs="*", default=[], help="PointCloud2 topics to z-filter"
    )
    args = parser.parse_args()

    input_path = Path(args.input_bag)
    output_path = input_path.parent / f"{input_path.name}_improved"

    if output_path.exists():
        print(f"Error: output path {output_path} already exists")
        return

    lifetime = make_duration(args.lifetime)
    print(f"Marker lifetime: {args.lifetime}s")
    if args.z_max is not None:
        print(f"Z-filter: z_max={args.z_max} on topics {args.pc_topics}")

    # Open reader
    reader = SequentialReader()
    storage_opts = StorageOptions(uri=str(input_path), storage_id="sqlite3")
    converter_opts = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_opts, converter_opts)

    # Collect topic info
    topic_types = {}
    all_topics = reader.get_all_topics_and_types()
    for topic_info in all_topics:
        topic_types[topic_info.name] = topic_info.type

    # Open writer
    writer = SequentialWriter()
    writer_storage = StorageOptions(uri=str(output_path), storage_id="sqlite3")
    writer.open(writer_storage, converter_opts)

    for topic_info in all_topics:
        writer.create_topic(
            TopicMetadata(
                name=topic_info.name,
                type=topic_info.type,
                serialization_format="cdr",
                offered_qos_profiles=topic_info.offered_qos_profiles,
            )
        )

    pc_topics_set = set(args.pc_topics)
    count = 0
    modified = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        count += 1
        type_str = topic_types.get(topic, "")

        # Adjust marker/poly lifetimes
        if type_str in TYPE_HANDLERS:
            msg_class, fixer = TYPE_HANDLERS[type_str]
            msg = deserialize_message(data, msg_class)
            msg = fixer(msg, lifetime)
            data = serialize_message(msg)
            modified += 1

        # Z-filter pointclouds
        elif (
            type_str == "sensor_msgs/msg/PointCloud2"
            and topic in pc_topics_set
            and args.z_max is not None
        ):
            msg = deserialize_message(data, PointCloud2)
            msg = filter_pc_z(msg, args.z_max)
            data = serialize_message(msg)
            modified += 1

        writer.write(topic, data, timestamp)

        if count % 10000 == 0:
            print(f"  Processed {count} messages ({modified} modified)...")

    del writer
    del reader
    print(f"Done! Processed {count} messages total, {modified} modified.")
    print(f"Output bag: {output_path}")


if __name__ == "__main__":
    main()
