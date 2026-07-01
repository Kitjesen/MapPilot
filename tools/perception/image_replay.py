#!/usr/bin/env python3
"""ROS2 图像回放工具 — 将测试图片/视频发布为相机话题, 用于测试感知管线。

用法:
  # 发布单张图片 (循环)
  python3 image_replay.py --image /tmp/test_bus.jpg

  # 发布视频
  python3 image_replay.py --video /path/to/video.mp4

  # 发布图片目录 (轮播)
  python3 image_replay.py --dir /tmp/test_images/

  # 自动下载 COCO 测试图
  python3 image_replay.py --download

会发布:
  /camera/color/image_raw  (sensor_msgs/Image, bgr8)
  /camera/depth/image_raw  (sensor_msgs/Image, 16UC1, 合成深度)
  /camera/color/camera_info (sensor_msgs/CameraInfo)
"""
import argparse
import os
import sys
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# Orbbec Gemini 335 内参
FX, FY = 693.3, 693.0
CX, CY = 640.0, 360.0
WIDTH, HEIGHT = 1280, 720

SAMPLE_URLS = [
    ("https://ultralytics.com/images/bus.jpg", "bus.jpg"),
    ("https://ultralytics.com/images/zidane.jpg", "zidane.jpg"),
]


class ImageReplayNode(Node):
    def __init__(self, fps=10.0):
        super().__init__("image_replay")
        self.bridge = CvBridge()
        qos = qos_profile_sensor_data

        self.pub_color = self.create_publisher(Image, "/camera/color/image_raw", qos)
        self.pub_depth = self.create_publisher(Image, "/camera/depth/image_raw", qos)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/color/camera_info", qos)

        self.fps = fps
        self.frames = []
        self.frame_idx = 0
        self.timer = self.create_timer(1.0 / fps, self._publish)
        self.get_logger().info(f"ImageReplay ready at {fps} Hz")

    def load_image(self, path):
        img = cv2.imread(path)
        if img is None:
            self.get_logger().error(f"Cannot read: {path}")
            return
        img = cv2.resize(img, (WIDTH, HEIGHT))
        self.frames.append(img)
        self.get_logger().info(f"Loaded: {path} -> {WIDTH}x{HEIGHT}")

    def load_video(self, path):
        cap = cv2.VideoCapture(path)
        count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.resize(frame, (WIDTH, HEIGHT))
            self.frames.append(frame)
            count += 1
        cap.release()
        self.get_logger().info(f"Loaded video: {path} ({count} frames)")

    def load_directory(self, path):
        exts = (".jpg", ".jpeg", ".png", ".bmp")
        files = sorted(
            f for f in os.listdir(path)
            if f.lower().endswith(exts)
        )
        for f in files:
            self.load_image(os.path.join(path, f))

    def _make_synthetic_depth(self, bgr):
        """生成合成深度图 — 基于亮度的伪深度, 用于测试 3D 投影。"""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
        # 亮度 → 1.0-5.0m 范围的深度
        depth_m = 1.0 + (255.0 - gray) / 255.0 * 4.0
        # 转为 uint16 毫米
        depth_mm = (depth_m * 1000).astype(np.uint16)
        return depth_mm

    def _make_camera_info(self, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_link"
        msg.width = WIDTH
        msg.height = HEIGHT
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _publish(self):
        if not self.frames:
            return
        bgr = self.frames[self.frame_idx % len(self.frames)]
        self.frame_idx += 1

        stamp = self.get_clock().now().to_msg()

        # Color
        color_msg = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = "camera_link"
        self.pub_color.publish(color_msg)

        # Depth
        depth = self._make_synthetic_depth(bgr)
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_link"
        self.pub_depth.publish(depth_msg)

        # CameraInfo
        self.pub_info.publish(self._make_camera_info(stamp))

        if self.frame_idx % (int(self.fps) * 5) == 0:
            self.get_logger().info(
                f"Published {self.frame_idx} frames "
                f"({len(self.frames)} unique, looping)"
            )


def download_samples(dest_dir="/tmp/test_images"):
    os.makedirs(dest_dir, exist_ok=True)
    paths = []
    for url, name in SAMPLE_URLS:
        path = os.path.join(dest_dir, name)
        if not os.path.exists(path):
            print(f"Downloading {url}...")
            urllib.request.urlretrieve(url, path)
        paths.append(path)
        print(f"  {path}")
    return paths


def main():
    parser = argparse.ArgumentParser(description="ROS2 Image Replay for perception testing")
    parser.add_argument("--image", help="Single image path")
    parser.add_argument("--video", help="Video file path")
    parser.add_argument("--dir", help="Directory of images")
    parser.add_argument("--download", action="store_true", help="Download COCO sample images")
    parser.add_argument("--fps", type=float, default=10.0, help="Publish rate (Hz)")
    args = parser.parse_args()

    rclpy.init()
    node = ImageReplayNode(fps=args.fps)

    if args.download:
        paths = download_samples()
        for p in paths:
            node.load_image(p)
    elif args.image:
        node.load_image(args.image)
    elif args.video:
        node.load_video(args.video)
    elif args.dir:
        node.load_directory(args.dir)
    else:
        print("No input specified. Use --image, --video, --dir, or --download")
        sys.exit(1)

    if not node.frames:
        print("No frames loaded")
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
