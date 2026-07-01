"""Typed DDS map output adapter."""

from __future__ import annotations

import logging
import time
from collections import Counter
from collections.abc import Callable
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.numpy_compat import np
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In
from runtime.transport.abc import TopicConfig

logger = logging.getLogger(__name__)


@register("map", "dds_map_output", description="Typed DDS map output adapter")
class DDSMapOutModule(Module, layer=2):
    """Publish LingTu map outputs as typed DDS messages."""

    exploration_grid: In[dict]

    def __init__(
        self,
        default_frame_id: str | None = None,
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        domain_id: int = 0,
        qos_depth: int = 2,
        reliable: bool = True,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._transport = transport
        self._transport_factory = transport_factory
        self._owns_transport = transport is None
        self._domain_id = int(domain_id)
        self._qos_depth = int(qos_depth)
        self._reliable = bool(reliable)
        self._publisher = None
        self._publish_counts: Counter[str] = Counter()
        self._publish_errors: Counter[str] = Counter()
        self._last_publish_ts = 0.0
        self._frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.exploration_grid)
        )
        self._backend_status = BackendStatus.configured_as("dds_map_output")

    def setup(self) -> None:
        self._transport = self._transport or self._create_default_transport()
        self.exploration_grid.subscribe(self._on_exploration_grid)
        self.exploration_grid.set_policy("latest")

    def stop(self) -> None:
        publisher = self._publisher
        if publisher is not None:
            close = getattr(publisher, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS map publisher close failed", exc_info=True)
        self._publisher = None
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS map transport close failed", exc_info=True)
        self._transport = None
        super().stop()

    def health(self) -> dict[str, Any]:
        return {
            **self._backend_status.as_health_fields(),
            "transport": "dds",
            "published_topics": [TOPICS.exploration_grid]
            if self._publisher is not None
            else [],
            "publish_counts": dict(self._publish_counts),
            "publish_errors": dict(self._publish_errors),
            "last_publish_ts": self._last_publish_ts,
        }

    def _create_default_transport(self) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory()
        from runtime.transport.dds import DDSTransport

        return DDSTransport(domain_id=self._domain_id)

    def _on_exploration_grid(self, grid: dict[str, Any]) -> None:
        try:
            self._publisher_for_grid().publish(_to_dds_occupancy_grid(grid, self._frame_id))
        except Exception:
            self._publish_errors[TOPICS.exploration_grid] += 1
            logger.exception("Failed to publish typed DDS map payload")
            return
        self._publish_counts[TOPICS.exploration_grid] += 1
        self._last_publish_ts = time.time()

    def _publisher_for_grid(self) -> Any:
        if self._publisher is not None:
            return self._publisher
        if self._transport is None:
            raise RuntimeError("DDS map output transport is not initialized")
        self._publisher = self._transport.create_publisher(
            TopicConfig(
                name=TOPICS.exploration_grid,
                qos_depth=self._qos_depth,
                reliable=self._reliable,
            )
        )
        return self._publisher


def _dds() -> Any:
    from message import dds_types as dds_mod

    return dds_mod


def _to_dds_time(ts: float | int | None) -> Any:
    dds_mod = _dds()
    value = float(ts or time.time())
    sec = int(value)
    nanosec = int(max(0.0, value - sec) * 1_000_000_000)
    return dds_mod.DDS_Time(sec=sec, nanosec=nanosec)


def _to_dds_header(frame_id: str, ts: float | int | None) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Header(stamp=_to_dds_time(ts), frame_id=str(frame_id or ""))


def _to_dds_identity_pose(origin_xy: tuple[float, float]) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Pose(
        position=dds_mod.DDS_Point(x=origin_xy[0], y=origin_xy[1], z=0.0),
        orientation=dds_mod.DDS_Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _to_dds_occupancy_grid(grid: dict[str, Any], default_frame_id: str) -> Any:
    dds_mod = _dds()
    values = np.asarray(grid.get("grid"), dtype=np.int16)
    if values.ndim != 2:
        values = np.zeros((0, 0), dtype=np.int16)
    height, width = values.shape
    origin = grid.get("origin")
    if origin is None:
        origin_xy = (
            float(grid.get("origin_x") or 0.0),
            float(grid.get("origin_y") or 0.0),
        )
    else:
        origin_xy = (float(origin[0]), float(origin[1]))
    frame_id = str(grid.get("frame_id") or default_frame_id)
    ts = float(grid.get("ts") or time.time())
    return dds_mod.DDS_OccupancyGrid(
        header=_to_dds_header(frame_id, ts),
        info=dds_mod.DDS_MapMetaData(
            map_load_time=_to_dds_time(ts),
            resolution=float(grid.get("resolution") or 0.0),
            width=int(width),
            height=int(height),
            origin=_to_dds_identity_pose(origin_xy),
        ),
        data=np.clip(values, -1, 100).astype(np.int8).reshape(-1).tolist(),
    )
