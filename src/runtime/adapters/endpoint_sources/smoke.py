"""No-ROS smoke source for Thunder endpoint data-flow checks."""

from __future__ import annotations

import time
from collections import Counter
from collections.abc import Mapping
from typing import Any

from runtime.msgs.geometry import Pose, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2

from runtime.adapters.endpoint_sources.types import EndpointEvent, EndpointService


class SmokeEndpointSource:
    """Publish one deterministic normalized snapshot for endpoint smoke tests.

    This is not a real hardware adapter. It proves that the endpoint process,
    source plugin contract, message schemas, and transport boundary can run on
    a machine without ROS or Thunder hardware.
    """

    name = "smoke"

    def __init__(self) -> None:
        self._started = False
        self._published = 0
        self._received: Counter[str] = Counter()
        self._last_publish_ts = 0.0
        self._last_receive_ts = 0.0

    def start(self, service: EndpointService) -> None:
        """Publish a single sensor/localization snapshot into LingTu."""

        now = time.time()
        lidar = PointCloud2(
            points=np.asarray(
                [
                    [0.0, 0.0, 0.0, 1.0],
                    [1.0, 0.0, 0.1, 0.8],
                    [0.0, 1.0, 0.1, 0.7],
                ],
                dtype=np.float32,
            ),
            ts=now,
            frame_id="lidar_link",
        )
        imu = Imu(
            linear_acceleration=Vector3(0.0, 0.0, 9.81),
            ts=now,
            frame_id="lidar_link",
        )
        odometry = Odometry(
            pose=Pose(0.0, 0.0, 0.0),
            twist=Twist(),
            ts=now,
            frame_id="odom",
            child_frame_id="body",
        )
        registered_cloud = PointCloud2(
            points=np.asarray([[0.0, 0.0, 0.0]], dtype=np.float32),
            ts=now,
            frame_id="body",
        )
        map_cloud = PointCloud2(
            points=np.asarray([[0.0, 0.0, 0.0]], dtype=np.float32),
            ts=now,
            frame_id="map",
        )

        self._published += service.publish_sensor_snapshot(
            lidar_scan=lidar,
            imu=imu,
        )
        self._published += service.publish_localization_snapshot(
            odometry=odometry,
            registered_cloud=registered_cloud,
            map_cloud=map_cloud,
            localization_health={
                "state": "SMOKE",
                "quality": 1.0,
                "source": self.name,
            },
            localization_quality=1.0,
        )
        self._last_publish_ts = now
        self._started = True

    def stop(self) -> None:
        """Stop the smoke source."""

        self._started = False

    def on_lingtu_message(self, event: EndpointEvent) -> None:
        """Record LingTu-to-endpoint events observed during smoke tests."""

        self._received[event.topic] += 1
        self._last_receive_ts = event.ts

    def health(self) -> Mapping[str, Any]:
        """Return smoke-source status."""

        return {
            "name": self.name,
            "started": self._started,
            "published": self._published,
            "received": dict(self._received),
            "last_publish_ts": self._last_publish_ts,
            "last_receive_ts": self._last_receive_ts,
            "hardware": False,
        }


def create() -> SmokeEndpointSource:
    """Factory used by endpoint runner ``--source smoke``."""

    return SmokeEndpointSource()
