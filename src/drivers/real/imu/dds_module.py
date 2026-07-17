"""DDS-backed real IMU role."""

from __future__ import annotations

import logging
import time
from typing import Any

from message.dds import dds_topic_name, topic_spec
from message.dds_codec import from_dds_message
from runtime.contracts import IMU_BACKEND_DDS, IMU_ROLE
from runtime.module import Module
from runtime.msgs.sensor import Imu
from runtime.registry import register
from runtime.runtime_interface import TOPICS
from runtime.stream import Out

logger = logging.getLogger(__name__)


@register(IMU_ROLE, IMU_BACKEND_DDS, description="Native DDS IMU reader")
class DdsImuModule(Module, layer=1):
    """Read canonical IMU samples from DDS.

    This backend is an explicit diagnostic/profile option. Field SLAM still
    prefers the synchronized `lidar.imu` stream unless a profile wires this
    independent role on purpose.
    """

    imu: Out[Imu]
    alive: Out[bool]

    def __init__(
        self,
        *,
        domain_id: int | None = None,
        topic: str = TOPICS.raw_imu,
        dds_topic: str | None = None,
        reader_factory: Any | None = None,
        stale_after_s: float = 1.0,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._domain_id = domain_id
        self._topic = topic
        self._dds_topic = dds_topic
        self._reader_factory = reader_factory
        self._stale_after_s = float(stale_after_s)
        self._reader = None
        self._running = False
        self._last_rx_ts = 0.0
        self._samples = 0
        self._last_error = ""

    def setup(self) -> None:
        spec = topic_spec(self._topic)
        if spec is None:
            self._last_error = f"DDS topic spec not found for {self._topic}"
            logger.warning("DdsImuModule: %s", self._last_error)
            return
        try:
            if self._reader_factory is None:
                from runtime.adapters.dds.reader import DDSReader

                reader = DDSReader(domain_id=self._domain_id)
            else:
                reader = self._reader_factory(domain_id=self._domain_id)
            reader.subscribe(
                self._topic,
                spec.dds_type(),
                self._on_sample,
                dds_topic=self._dds_topic or dds_topic_name(self._topic, typed=True),
            )
            self._reader = reader
        except Exception as exc:
            self._last_error = str(exc)
            logger.warning("DdsImuModule setup failed: %s", exc)

    def start(self) -> None:
        super().start()
        if self._reader is None:
            self.alive.publish(False)
            return
        try:
            started = bool(self._reader.spin_background())
            self._running = started
            self.alive.publish(started)
        except Exception as exc:
            self._last_error = str(exc)
            self._running = False
            self.alive.publish(False)
            logger.warning("DdsImuModule start failed: %s", exc)

    def stop(self) -> None:
        self._running = False
        if self._reader is not None:
            try:
                self._reader.stop()
            except Exception:
                logger.debug("DdsImuModule reader stop failed", exc_info=True)
        self.alive.publish(False)
        super().stop()

    def _on_sample(self, sample: Any) -> None:
        try:
            imu = from_dds_message(self._topic, sample)
            if not isinstance(imu, Imu):
                raise TypeError(f"expected Imu, got {type(imu).__name__}")
            self._last_rx_ts = time.monotonic()
            self._samples += 1
            self.imu.publish(imu)
        except Exception as exc:
            self._last_error = str(exc)
            logger.debug("DdsImuModule sample rejected: %s", exc)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        age = None if self._last_rx_ts <= 0 else time.monotonic() - self._last_rx_ts
        info.update(
            {
                "role": IMU_ROLE,
                "status": "running" if self._running else "stopped",
                "backend": IMU_BACKEND_DDS,
                "source": self._dds_topic or dds_topic_name(self._topic, typed=True),
                "diagnostic_reader": True,
                "publishes_samples": True,
                "duplicates_lidar_imu_topic": self._topic == TOPICS.raw_imu,
                "note": (
                    "explicit reader for diagnostics/profile use; field SLAM "
                    "prefers lidar.imu for synchronized Livox samples"
                ),
                "samples": self._samples,
                "stale_ms": None if age is None else int(max(age, 0.0) * 1000),
                "stale": True if age is None else age > self._stale_after_s,
                "error": self._last_error,
            }
        )
        return info


ImuModuleDds = DdsImuModule
