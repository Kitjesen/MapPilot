"""MuJoCo IMU source wrapper."""

from __future__ import annotations

import time
from typing import Any

from runtime.msgs.geometry import Quaternion, Vector3
from runtime.msgs.sensor import Imu as ImuMsg
from runtime.runtime_interface import TOPICS, topic_default_frame_id

from ...native import Sample


class Imu:
    """Read IMU samples from a MuJoCo engine."""

    def __init__(self, engine: Any | None = None) -> None:
        self._engine = engine
        self._connected = engine is not None

    def bind(self, engine: Any) -> None:
        self._engine = engine
        self._connected = True

    def connect(self) -> None:
        self._connected = self._engine is not None

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._engine is not None

    def read(self, timeout_ms: int = 0) -> Sample:
        del timeout_ms
        if not self.is_connected():
            return Sample()

        state = self._engine.get_robot_state()
        quat = getattr(state, "orientation", (0.0, 0.0, 0.0, 1.0))
        gyro = getattr(state, "imu_gyro", (0.0, 0.0, 0.0))
        accel = getattr(state, "imu_linear_acceleration", (0.0, 0.0, 0.0))
        ts = time.time()
        return Sample(
            imu=ImuMsg(
                orientation=Quaternion(
                    float(quat[0]),
                    float(quat[1]),
                    float(quat[2]),
                    float(quat[3]),
                ),
                angular_velocity=Vector3(
                    float(gyro[0]),
                    float(gyro[1]),
                    float(gyro[2]),
                ),
                linear_acceleration=Vector3(
                    float(accel[0]),
                    float(accel[1]),
                    float(accel[2]),
                ),
                ts=ts,
                frame_id=topic_default_frame_id(TOPICS.imu),
            ),
            timestamp=ts,
        )
