"""DDS Adapter for Livox MID-360 scan and IMU topics."""

from __future__ import annotations

import logging

from . import _dds as livox_dds
from .frame_stream import LidarFrameStream

logger = logging.getLogger(__name__)


class LivoxDdsAdapter:
    """Subscribe to Livox DDS topics and feed normalized frames to a stream."""

    def __init__(
        self,
        *,
        scan_topic: str,
        imu_topic: str,
        frames: LidarFrameStream,
    ) -> None:
        self._scan_topic = scan_topic
        self._imu_topic = imu_topic
        self._frames = frames
        self._dds = None

    @property
    def dds(self):
        return self._dds

    def start(self) -> None:
        if not livox_dds.HAS_LIVOX_IDL:
            logger.warning(
                "Lidar: Python cyclonedds is not installed; Python cloud "
                "diagnostics are disabled. Robot field ingest uses the C++ "
                "Livox/CycloneDDS runtime."
            )
            return

        from runtime.dds import DDSReader

        self._dds = DDSReader()
        self._dds.subscribe(self._scan_topic, livox_dds.LivoxCustomMsg, self.on_scan)
        if livox_dds.DDS_Imu is not None:
            self._dds.subscribe(self._imu_topic, livox_dds.DDS_Imu, self.on_imu)
        self._dds.spin_background()

    def stop(self) -> None:
        if not self._dds:
            return
        self._dds.stop()
        self._dds = None

    def on_scan(self, msg) -> None:
        frame = livox_dds.livox_msg_to_frame(msg)
        self._frames.ingest_point_frame(frame)

    def on_imu(self, msg) -> None:
        try:
            imu = livox_dds.dds_imu_to_imu(msg)
        except Exception as exc:
            logger.debug("Lidar IMU conversion error: %s", exc)
            return
        self._frames.ingest_imu(imu)
