"""Gazebo/GZ to LingTu ROS topic bridge helpers.

This module is intentionally dependency-light. It does not import rclpy,
ros_gz, or Gazebo libraries at import time, so it can be tested on machines
that do not have ROS 2 installed.

The first Gazebo integration stage uses this file as the single source of truth
for topic and frame names. Launch files can consume the generated bridge specs,
and later a Python runtime bridge can reuse the same config for TwistStamped
conversion and frame validation.
"""

from __future__ import annotations

from dataclasses import dataclass

from runtime.runtime_interface import FRAMES, TOPICS, RuntimeFrames


FrameContract = RuntimeFrames


@dataclass(frozen=True)
class GazeboBridgeConfig:
    """Topic names for a ROS-native Gazebo/GZ simulation run."""

    world_name: str = "lingtu_world"
    robot_name: str = "thunder"
    frames: FrameContract = FRAMES

    lingtu_cmd_vel: str = TOPICS.cmd_vel
    lingtu_odometry: str = TOPICS.odometry
    lingtu_map_cloud: str = TOPICS.map_cloud
    lingtu_terrain_map: str = TOPICS.terrain_map
    lingtu_terrain_map_ext: str = TOPICS.terrain_map_ext
    lingtu_cumulative_map_cloud: str = TOPICS.cumulative_map_cloud
    lingtu_registered_cloud: str = TOPICS.registered_cloud
    lingtu_color_image: str = TOPICS.camera_color
    lingtu_depth_image: str = TOPICS.camera_depth
    lingtu_camera_info: str = TOPICS.camera_info

    gazebo_raw_odometry: str = "/lingtu/gazebo/raw/odometry"
    gazebo_raw_lidar_points: str = "/lingtu/gazebo/raw/lidar_points"
    gazebo_raw_lidar_scan: str = "/lingtu/gazebo/raw/lidar_scan"
    gazebo_raw_color_image: str = "/lingtu/gazebo/raw/color_image"
    gazebo_raw_depth_image: str = "/lingtu/gazebo/raw/depth_image"
    gazebo_raw_camera_info: str = "/lingtu/gazebo/raw/camera_info"

    gazebo_cmd_vel: str = "/lingtu/gazebo/cmd_vel"
    gazebo_cmd_vel_ros_input: str = "/lingtu/gazebo/cmd_vel"

    @property
    def gazebo_odometry(self) -> str:
        return f"/model/{self.robot_name}/odometry"

    @property
    def gazebo_lidar_points(self) -> str:
        frame = self.frames.lidar_frame
        return (
            f"/world/{self.world_name}/model/{self.robot_name}/link/{frame}"
            "/sensor/lidar/scan/points"
        )

    @property
    def gazebo_lidar_scan(self) -> str:
        frame = self.frames.lidar_frame
        return (
            f"/world/{self.world_name}/model/{self.robot_name}/link/{frame}"
            "/sensor/lidar/scan"
        )

    @property
    def gazebo_color_image(self) -> str:
        frame = self.frames.camera_frame
        return (
            f"/world/{self.world_name}/model/{self.robot_name}/link/{frame}"
            "/sensor/camera/image"
        )

    @property
    def gazebo_depth_image(self) -> str:
        frame = self.frames.camera_frame
        return (
            f"/world/{self.world_name}/model/{self.robot_name}/link/{frame}"
            "/sensor/depth_camera/image"
        )

    @property
    def gazebo_camera_info(self) -> str:
        frame = self.frames.camera_frame
        return (
            f"/world/{self.world_name}/model/{self.robot_name}/link/{frame}"
            "/sensor/camera/camera_info"
        )

    def ros_gz_bridge_specs(self) -> list[str]:
        """Return ros_gz_bridge parameter_bridge specs.

        Direction markers follow ros_gz_bridge convention:
        `[` means Gazebo to ROS, `]` means ROS to Gazebo.
        """

        return [
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"{self.gazebo_odometry}@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            (
                f"{self.gazebo_lidar_points}"
                "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            ),
            f"{self.gazebo_lidar_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{self.gazebo_color_image}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{self.gazebo_depth_image}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{self.gazebo_camera_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{self.gazebo_cmd_vel_ros_input}@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ]

    def ros_remap_args(self) -> list[str]:
        """Return ROS remapping args that normalize Gazebo topics to LingTu names."""

        remaps = {
            self.gazebo_odometry: self.gazebo_raw_odometry,
            self.gazebo_lidar_points: self.gazebo_raw_lidar_points,
            self.gazebo_lidar_scan: self.gazebo_raw_lidar_scan,
            self.gazebo_color_image: self.gazebo_raw_color_image,
            self.gazebo_depth_image: self.gazebo_raw_depth_image,
            self.gazebo_camera_info: self.gazebo_raw_camera_info,
        }
        args = ["--ros-args"]
        for source, target in remaps.items():
            args.extend(["-r", f"{source}:={target}"])
        return args

    def required_lingtu_topics(self) -> dict[str, str]:
        """Return the LingTu side of the Gazebo bridge contract."""

        return {
            "cmd_vel": self.lingtu_cmd_vel,
            "gazebo_cmd_vel_ros_input": self.gazebo_cmd_vel_ros_input,
            "odometry": self.lingtu_odometry,
            "map_cloud": self.lingtu_map_cloud,
            "terrain_map": self.lingtu_terrain_map,
            "terrain_map_ext": self.lingtu_terrain_map_ext,
            "cumulative_map_cloud": self.lingtu_cumulative_map_cloud,
            "registered_cloud": self.lingtu_registered_cloud,
            "color_image": self.lingtu_color_image,
            "depth_image": self.lingtu_depth_image,
            "camera_info": self.lingtu_camera_info,
        }

    def raw_ros_topics(self) -> dict[str, str]:
        """Return private ROS topics that carry unnormalized Gazebo output."""

        return {
            "odometry": self.gazebo_raw_odometry,
            "lidar_points": self.gazebo_raw_lidar_points,
            "lidar_scan": self.gazebo_raw_lidar_scan,
            "color_image": self.gazebo_raw_color_image,
            "depth_image": self.gazebo_raw_depth_image,
            "camera_info": self.gazebo_raw_camera_info,
        }


DEFAULT_GAZEBO_BRIDGE = GazeboBridgeConfig()
