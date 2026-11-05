"""Native LiDAR acquisition settings; installation data stays in RobotConfig."""

import json
import math
from collections.abc import Mapping
from typing import Any

from runtime.config import RobotConfig
from runtime.tf.frames import FRAMES
from runtime.utils.livox_config import build_mid360_config_dict


def lidar_environment(settings: Mapping[str, Any], robot_config: RobotConfig) -> dict[str, str]:
    if set(settings) != {"scan_hz", "imu_hz", "sdk"}:
        raise ValueError("lidar must define scan_hz, imu_hz and sdk")
    environment = {"LINGTU_LIVOX_LIDAR_FRAME": FRAMES.lidar, "LINGTU_LIVOX_IMU_FRAME": FRAMES.imu}
    for key in ("scan_hz", "imu_hz"):
        value = settings[key]
        if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
            raise ValueError(f"lidar.{key} must be a finite number")
        if value < 0 or (key == "scan_hz" and value == 0):
            raise ValueError(f"lidar.{key} is outside its acquisition range")
        environment[f"LINGTU_LIVOX_{key.upper()}"] = format(value, ".15g")
    environment["LINGTU_LIVOX_CONFIG_JSON"] = json.dumps(
        build_mid360_config_dict(robot_config, settings=settings), separators=(",", ":")
    )
    return environment
