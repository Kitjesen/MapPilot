"""Runtime bridge for live map cloud and SLAM map-save adapters."""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Any

from maps.map_save import (
    MapSaveError,
    save_slam_map_with_adapter,
    seed_default_map_save_adapter_plugins,
)
from runtime.msgs.map import MapCloudFrame
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2

logger = logging.getLogger(__name__)


class MapSaveAdapterFailed(Exception):
    """Adapter failures already shaped for maps API responses."""


class MapRuntimeBridge:
    """Owns live runtime inputs used by maps.

    This class is not a transport.  It is the ROS-free boundary that converts
    runtime map_cloud messages into a stable point array and calls the selected
    map-save adapter when a persistent source map is requested.
    """

    def __init__(
        self,
        *,
        slam_profile: str = "",
        map_save_adapter: Any = None,
        map_save_timeout_sec: float = 30.0,
    ) -> None:
        self.slam_profile = str(slam_profile or "").strip().lower()
        self.map_save_adapter = map_save_adapter
        self.map_save_timeout_sec = float(map_save_timeout_sec)
        self.map_cloud_lock = threading.Lock()
        self.latest_map_points: np.ndarray | None = None
        self.latest_map_frame_info: dict[str, Any] = {}
        self.latest_localization_status: dict[str, Any] = {}
        self.map_cloud_frame_count = 0
        self.first_map_sequence = 0
        self.last_map_sequence = 0

    def on_map_cloud(self, cloud: PointCloud2) -> None:
        """Store a legacy PointCloud2 map cloud as a full map frame."""
        self.on_map_cloud_frame(
            MapCloudFrame.from_pointcloud2(
                cloud,
                mode="FULL",
                source="map_cloud",
            )
        )

    def on_map_cloud_frame(self, frame: MapCloudFrame | dict[str, Any]) -> None:
        """Ingest a typed map-cloud frame for runtime map consumers.

        FULL replaces the stored map view. KEYFRAME and INCREMENTAL append to
        it so consumers can observe producers that publish map data in pieces.
        """
        try:
            map_frame = MapCloudFrame.from_dict(frame) if isinstance(frame, dict) else frame
            pts = map_frame.finite_xyz(max_abs=500.0)
        except Exception:
            logger.debug("failed to normalize live map_cloud_frame", exc_info=True)
            return
        with self.map_cloud_lock:
            if map_frame.mode == "FULL" or self.latest_map_points is None:
                stored = pts.copy()
                self.first_map_sequence = int(map_frame.sequence)
            else:
                stored = np.vstack([self.latest_map_points, pts]).astype(
                    np.float32,
                    copy=False,
                )
            self.latest_map_points = stored
            self.last_map_sequence = int(map_frame.sequence)
            self.map_cloud_frame_count += 1
            self.latest_map_frame_info = {
                "schema_version": map_frame.schema_version,
                "mode": map_frame.mode,
                "frame_id": map_frame.frame_id,
                "map_id": map_frame.map_id,
                "source": map_frame.source,
                "sequence": map_frame.sequence,
                "first_sequence": self.first_map_sequence,
                "last_sequence": self.last_map_sequence,
                "ts": map_frame.ts,
                "points_in_frame": int(pts.shape[0]),
                "stored_points": int(stored.shape[0]),
                "frame_count": int(self.map_cloud_frame_count),
            }

    def on_localization_status(self, status: dict[str, Any]) -> None:
        with self.map_cloud_lock:
            self.latest_localization_status = dict(status or {})

    def snapshot_health(self) -> tuple[bool, str]:
        """Return the explicit SLAM health gate for a map snapshot."""
        with self.map_cloud_lock:
            status = dict(self.latest_localization_status)
        if not status:
            return True, "SLAM health was not explicitly reported"
        if status.get("healthy") is False or status.get("ok") is False:
            return False, str(status.get("message") or "SLAM reported unhealthy")
        state = (
            str(status.get("state") or status.get("status") or status.get("localization_state") or "").strip().upper()
        )
        if state in {"ERROR", "FAILED", "LOST", "DIVERGED", "UNHEALTHY"}:
            return False, str(status.get("message") or f"SLAM state is {state}")
        return True, str(status.get("message") or state or "SLAM health accepted")

    def resolve_slam_profile(self, slam_profile: str | None = None) -> str:
        """Resolve the backend used for a map save response."""
        profile = str(slam_profile or self.slam_profile or "").strip().lower()
        if profile:
            return self.normalize_slam_profile(profile)
        status_backend = str(
            self.latest_localization_status.get("backend")
            or self.latest_localization_status.get("localization_backend")
            or ""
        ).strip().lower()
        if status_backend:
            return self.normalize_slam_profile(status_backend)
        return "unknown"

    def save_map_with_adapter(self, pcd_path: Path) -> dict[str, Any]:
        """Save ``map.pcd`` through the configured map-save adapter."""
        try:
            if self.map_save_adapter is None:
                seed_default_map_save_adapter_plugins()
            result = save_slam_map_with_adapter(
                self.map_save_adapter,
                pcd_path,
                save_patches=True,
                timeout_sec=self.map_save_timeout_sec,
            )
        except MapSaveError as exc:
            raise MapSaveAdapterFailed(str(exc)) from exc

        return result

    @staticmethod
    def map_save_capability_fields(slam_profile: str | None) -> dict[str, Any]:
        """Return the map-save capability contract shared by Gateway/MCP users."""
        profile = MapRuntimeBridge.normalize_slam_profile(slam_profile)
        if profile == "localizer":
            return {
                "map_save_supported": True,
                "map_save_source": "slam_service",
                "relocalization_supported": True,
                "saved_map_relocalization_supported": True,
                "restart_recovery_supported": True,
                "recovery_method": "relocalize_service",
            }
        if profile == "native_dds":
            return {
                "map_save_supported": True,
                "map_save_source": "native_slam_dds_control",
                "relocalization_supported": True,
                "saved_map_relocalization_supported": True,
                "restart_recovery_supported": False,
                "recovery_method": "restart_native_dds_slam",
            }
        if profile in {"fastlio2", "slam"}:
            return {
                "map_save_supported": True,
                "map_save_source": "slam_service",
                "relocalization_supported": False,
                "saved_map_relocalization_supported": False,
                "restart_recovery_supported": True,
                "recovery_method": "restart_slam",
            }
        return {
            "map_save_supported": False,
            "map_save_source": "unknown",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": False,
            "recovery_method": "none",
        }

    @staticmethod
    def normalize_slam_profile(slam_profile: str | None) -> str:
        profile = str(slam_profile or "").strip().lower()
        return profile
