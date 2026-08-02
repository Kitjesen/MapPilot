"""LidarModule - Livox MID-360 as a first-class Blueprint Module.

Decouples LiDAR from SLAM: LiDAR is an independent hardware resource
that can be started, stopped, and subscribed to without running SLAM.

Blueprint usage::

    from drivers.real.lidar import LidarModule

    bp.add(LidarModule)                             # default config
    bp.add(LidarModule, ip="192.168.1.120")         # override IP
    bp.wire("LidarModule", "scan", "nav.terrain", "cloud")

Stack factory usage (in blueprints/stacks/)::

    from drivers.real.lidar import LidarModule
    bp.add(LidarModule, ip=ip)
    # SLAM consumes raw_scan/imu regardless of the selected LiDAR source.
"""

from __future__ import annotations

import logging
from typing import Any

from runtime.module import Module
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.registry import register
from runtime.runtime_interface import real_lidar_frame_id
from runtime.stream import Out

from .native.sdk import LidarSource, LidarSourceFactory, create_lidar_source

logger = logging.getLogger(__name__)
LIDAR_RAW_FRAME_ID = real_lidar_frame_id()


@register("lidar", "mid360", description="Livox MID-360 LiDAR driver")
@register("driver", "lidar_mid360", description="Livox MID-360 LiDAR driver")
class LidarModule(Module, layer=1):
    """Livox MID-360 driver as a Module in the Blueprint system.

    Owns the selected Livox source lifecycle and bridges raw point cloud + IMU
    data into Module output ports. The default source is the ROS-free official
    SDK2 process. Device process ownership belongs to the C++ Livox service.

    Ports:
        scan (Out[PointCloud2]): Raw LiDAR point cloud per frame.
        raw_scan (Out[Any]):     Lossless Livox frame with per-point metadata.
        imu  (Out[Imu]):         IMU readings from the LiDAR unit.
        alive (Out[bool]):       Driver health status.

    Config from robot_config.yaml::

        lidar:
          lidar_ip: "192.168.1.115"
          host_ip:  "192.168.1.5"
          publish_freq: 10.0
    """

    scan: Out[PointCloud2]
    raw_scan: Out[Any]
    imu: Out[Imu]
    alive: Out[bool]

    def __init__(
        self,
        ip: str | None = None,
        source: LidarSource | None = None,
        source_factory: LidarSourceFactory | None = None,
        **kw,
    ):
        super().__init__(**kw)
        if source is not None and source_factory is not None:
            raise ValueError("Pass either source or source_factory, not both")
        factory = source_factory or create_lidar_source
        source_kwargs = {"ip": ip}
        self._lidar = source or factory(**source_kwargs)

    def setup(self) -> None:
        self._lidar.on_cloud(self._on_cloud)
        self._lidar.on_raw_cloud(self._on_raw_cloud)
        self._lidar.on_imu(self._on_imu)

    def start(self) -> None:
        super().start()
        try:
            self._lidar.connect()
            self.alive.publish(True)
        except Exception as e:
            self.alive.publish(False)
            logger.error("LidarModule start failed: %s", e)

    def stop(self) -> None:
        self._lidar.disconnect()
        self.alive.publish(False)
        super().stop()

    def _on_cloud(self, pts) -> None:
        """Forward numpy (N,4) to PointCloud2 on the scan port."""
        cloud = PointCloud2.from_numpy(pts, frame_id=LIDAR_RAW_FRAME_ID)
        self.scan.publish(cloud)

    def _on_raw_cloud(self, frame: Any) -> None:
        """Forward the lossless Livox DDS frame for LIO/deskew consumers."""
        self.raw_scan.publish(frame)

    def _on_imu(self, imu_msg: Imu) -> None:
        """Forward Imu to the imu port."""
        self.imu.publish(imu_msg)

    def health(self) -> dict[str, Any]:
        """Report driver health for monitoring."""
        base = super().port_summary()
        h = self._lidar.health
        base["lidar"] = h.to_dict()
        return base

    def __repr__(self) -> str:
        return f"LidarModule(ip={self._lidar.ip!r}, state={self._lidar.state.value})"
