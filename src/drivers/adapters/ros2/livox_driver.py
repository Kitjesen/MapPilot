"""ROS2 compatibility adapter for the official livox_ros_driver2 process."""

from __future__ import annotations

from typing import Any


def livox_driver_process(cfg: Any):
    """Build a NativeModule for the official livox_ros_driver2 executable."""

    from runtime.native_install import DDS_ENV, exe
    from runtime.native_module import NativeModule, NativeModuleConfig
    from runtime.runtime_interface import TOPICS
    from runtime.utils.livox_config import ensure_mid360_config_file

    publish_freq = float(getattr(cfg.lidar, "publish_freq", 10.0))
    config_path = ensure_mid360_config_file(cfg)
    binary = exe(cfg, "livox_ros_driver2", "livox_ros_driver2_node")

    return NativeModule(
        NativeModuleConfig(
            executable=binary,
            name="livox_driver",
            parameters={
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": publish_freq,
                "output_data_type": 0,
                "user_config_path": config_path,
            },
            remappings={
                "livox/lidar": TOPICS.lidar_scan,
                "livox/imu": TOPICS.imu,
            },
            env=DDS_ENV,
            auto_restart=True,
            max_restarts=3,
        )
    )


__all__ = ["livox_driver_process"]
