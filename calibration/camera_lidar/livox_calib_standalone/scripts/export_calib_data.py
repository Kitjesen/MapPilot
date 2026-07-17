#!/usr/bin/env python3
"""
export_calib_data.py — Export PCD + PNG from ROS2 bag for livox_calib_standalone.

Usage:
    python export_calib_data.py --bag <path_to_ros2_bag> \
        --lidar-topic /livox/lidar --image-topic /camera/image_raw \
        --output-dir ./calib_data

Requirements:
    pip install rosbag2_py rclpy sensor_msgs_pydantic numpy
    (or run on a system with ROS 2 Humble installed)

For LingTu: this script runs on the robot (S100P) or any ROS 2 workstation.
"""

import argparse
import os
import struct
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:
    print("ERROR: numpy required. pip install numpy")
    sys.exit(1)


def export_from_ros2_bag(bag_path: str, lidar_topic: str, image_topic: str, output_dir: str, max_frames: int = 1):
    """Export LiDAR PCD and camera PNG from a ROS2 bag."""
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from sensor_msgs.msg import Image, PointCloud2
    except ImportError:
        print("ERROR: ROS 2 Python packages not found.")
        print("Install with: sudo apt install ros-humble-rosbag2-py ros-humble-sensor-msgs")
        print("Or use the --pcd and --image flags to provide pre-exported files.")
        sys.exit(1)

    import rclpy

    rclpy.init()

    os.makedirs(output_dir, exist_ok=True)

    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    topic_type_map = {t.name: t.type for t in topics}

    if lidar_topic not in topic_type_map:
        print(f"WARNING: LiDAR topic '{lidar_topic}' not found in bag.")
        print(f"Available: {[t.name for t in topics]}")
    if image_topic not in topic_type_map:
        print(f"WARNING: Image topic '{image_topic}' not found in bag.")
        print(f"Available: {[t.name for t in topics]}")

    lidar_count = 0
    image_count = 0
    pcd_path = os.path.join(output_dir, "lidar.pcd")
    img_path = os.path.join(output_dir, "image.png")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == lidar_topic and lidar_count < max_frames:
            msg = deserialize_message(data, PointCloud2)
            _save_pcd(msg, pcd_path)
            lidar_count += 1
            print(f"Saved LiDAR frame {lidar_count} -> {pcd_path}")

        elif topic == image_topic and image_count < max_frames:
            msg = deserialize_message(data, Image)
            _save_png(msg, img_path)
            image_count += 1
            print(f"Saved Image frame {image_count} -> {img_path}")

        if lidar_count >= max_frames and image_count >= max_frames:
            break

    rclpy.shutdown()
    return pcd_path, img_path


def _save_pcd(msg, path: str):
    """Save PointCloud2 to ASCII PCD file."""
    # Parse point cloud data
    point_step = msg.point_step
    n_points = msg.width * msg.height

    # Find x, y, z, intensity field offsets
    offsets = {}
    for field in msg.fields:
        offsets[field.name] = field.offset

    points = []
    for i in range(n_points):
        start = i * point_step
        x = struct.unpack_from("f", msg.data, start + offsets.get("x", 0))[0]
        y = struct.unpack_from("f", msg.data, start + offsets.get("y", 4))[0]
        z = struct.unpack_from("f", msg.data, start + offsets.get("z", 8))[0]
        intensity = struct.unpack_from("f", msg.data, start + offsets.get("intensity", 16))[0]
        points.append((x, y, z, intensity))

    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z, i in points:
            f.write(f"{x} {y} {z} {i}\n")


def _save_png(msg, path: str):
    """Save sensor_msgs/Image to PNG."""
    try:
        import cv2
    except ImportError:
        print("ERROR: opencv-python required. pip install opencv-python")
        sys.exit(1)

    if msg.encoding in ("bgr8", "BGR8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif msg.encoding in ("rgb8", "RGB8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding in ("mono8", "MONO8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    else:
        print(f"WARNING: Unsupported encoding '{msg.encoding}', trying bgr8")
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

    cv2.imwrite(path, img)


def main():
    parser = argparse.ArgumentParser(description="Export calib data from ROS2 bag")
    parser.add_argument("--bag", required=True, help="Path to ROS2 bag")
    parser.add_argument("--lidar-topic", default="/livox/lidar", help="LiDAR PointCloud2 topic")
    parser.add_argument("--image-topic", default="/camera/color/image_raw", help="Camera image topic")
    parser.add_argument("--output-dir", default="./calib_data", help="Output directory for PCD + PNG")
    parser.add_argument("--max-frames", type=int, default=1, help="Max frames to export")
    args = parser.parse_args()

    pcd, img = export_from_ros2_bag(args.bag, args.lidar_topic, args.image_topic, args.output_dir, args.max_frames)

    print("\nDone! Run calibration with:")
    print(f"  livox_calib --image {img} --pcd {pcd} \\")
    print("    --camera config/camera_intrinsics.yaml \\")
    print("    --calib config/calib_config.yaml")


if __name__ == "__main__":
    main()
