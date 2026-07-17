"""Native active-map and saved-map relocalization adapter for floor changes."""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Iterable, Mapping
from pathlib import Path
from typing import Any, Protocol

from nav.building.model import ActiveFloor, PoseTarget


class NativeMapsPort(Protocol):
    """Maps capability surface required by a floor transition."""

    def get_active_map(self) -> dict[str, Any]:
        """Return the native active-map identity."""

    def get_map_bundle(self, name: str, capability: str) -> dict[str, Any]:
        """Resolve one validated map artifact by capability."""

    def set_active_map(self, name: str) -> dict[str, Any]:
        """Activate one artifact-gated map."""


class SavedMapRelocalizationPort(Protocol):
    """ROS-free native saved-map relocalization capability."""

    def relocalize_saved_map(
        self,
        pcd_path: str | Path,
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 30.0,
    ) -> Any:
        """Run a seeded saved-map relocalization and return a result with success/message."""


class NativeFloorLocalizationAdapter:
    """Switch active map, invoke native relocalization, then verify fresh tracking."""

    def __init__(
        self,
        *,
        maps: NativeMapsPort,
        relocalization: SavedMapRelocalizationPort,
        floors: Iterable[ActiveFloor],
        localization_status: Callable[[], Mapping[str, Any]],
        clock: Callable[[], float] = time.time,
        max_status_age_s: float = 1.0,
        relocalization_timeout_s: float = 30.0,
    ) -> None:
        self._maps = maps
        self._relocalization = relocalization
        self._localization_status = localization_status
        self._clock = clock
        self._max_status_age_s = max(0.05, float(max_status_age_s))
        self._relocalization_timeout_s = max(1.0, float(relocalization_timeout_s))
        self._floors_by_map: dict[str, ActiveFloor] = {}
        for floor in floors:
            if floor.map_id in self._floors_by_map:
                raise ValueError(f"duplicate floor map binding: {floor.map_id}")
            self._floors_by_map[floor.map_id] = floor
        if not self._floors_by_map:
            raise ValueError("at least one floor binding is required")
        self._last_relocalized_floor: ActiveFloor | None = None
        self._relocalization_started_s = 0.0

    def active_floor(self) -> ActiveFloor:
        """Resolve floor identity only from the native active-map source of truth."""

        response = self._maps.get_active_map()
        if not isinstance(response, Mapping) or response.get("success") is not True:
            raise RuntimeError("native active map is unavailable")
        map_id = str(response.get("active") or "").strip()
        floor = self._floors_by_map.get(map_id)
        if floor is None:
            raise RuntimeError(f"active map has no floor binding: {map_id or 'none'}")
        return floor

    def switch_and_relocalize(
        self,
        floor: ActiveFloor,
        seed: PoseTarget,
    ) -> tuple[bool, str]:
        """Artifact-gate the target before changing active map, then relocalize."""

        self._last_relocalized_floor = None
        if self._floors_by_map.get(floor.map_id) != floor:
            return False, "target_floor_binding_unknown"
        if seed.frame_id != "map":
            return False, "unsupported_navigation_frame"
        if not all(math.isfinite(value) for value in (seed.x, seed.y, seed.z, seed.yaw)):
            return False, "invalid_relocalization_seed"

        try:
            bundle = self._maps.get_map_bundle(floor.map_id, "source_pointcloud")
        except Exception:
            return False, "target_source_pointcloud_unavailable"
        pcd_path = self._bundle_path(bundle)
        if pcd_path is None or not pcd_path.is_file():
            return False, "target_source_pointcloud_unavailable"

        try:
            activation = self._maps.set_active_map(floor.map_id)
        except Exception:
            return False, "target_map_activation_error"
        if not isinstance(activation, Mapping) or activation.get("success") is not True:
            message = str(activation.get("message") or "activation rejected") if isinstance(activation, Mapping) else "activation rejected"
            return False, f"target_map_activation_failed:{message}"
        if str(activation.get("active") or "").strip() != floor.map_id:
            return False, "target_map_activation_unverified"
        if not self._active_map_matches(floor.map_id):
            return False, "target_map_activation_unverified"

        self._relocalization_started_s = float(self._clock())
        try:
            result = self._relocalization.relocalize_saved_map(
                pcd_path,
                seed.x,
                seed.y,
                seed.yaw,
                timeout_s=self._relocalization_timeout_s,
            )
        except Exception:
            return False, "native_relocalization_error"
        if getattr(result, "success", False) is not True:
            message = str(getattr(result, "message", "") or "unknown")
            return False, f"native_relocalization_failed:{message}"
        if not self._active_map_matches(floor.map_id):
            return False, "target_map_changed_during_relocalization"
        self._last_relocalized_floor = floor
        return True, "native_relocalization_completed"

    def is_localized(self, floor: ActiveFloor) -> bool:
        """Require target map identity plus fresh post-request TRACKING evidence."""

        if self._last_relocalized_floor != floor:
            return False
        if not self._active_map_matches(floor.map_id):
            return False
        try:
            status = self._localization_status()
        except Exception:
            return False
        if not isinstance(status, Mapping):
            return False
        if status.get("status_snapshot_stale") is True:
            return False
        state = str(
            status.get("state")
            or status.get("status")
            or status.get("localization_state")
            or ""
        ).strip().upper()
        if state not in {"TRACKING", "LOCALIZED", "READY"}:
            return False
        if status.get("ready") is not True:
            return False
        if status.get("pose_fresh") is not True or status.get("has_odometry") is not True:
            return False
        reported_map = str(status.get("active_map") or status.get("map_id") or "").strip()
        if reported_map and reported_map != floor.map_id:
            return False
        relocalization_state = str(status.get("relocalization_state") or "").strip().lower()
        if relocalization_state and relocalization_state not in {
            "completed",
            "succeeded",
            "tracking",
            "idle",
        }:
            return False
        stamp_s = self._status_stamp(status)
        if stamp_s is None:
            return False
        age_s = float(self._clock()) - stamp_s
        if age_s < -0.5 or age_s > self._max_status_age_s:
            return False
        if stamp_s + 1e-6 < self._relocalization_started_s:
            return False
        return True

    def _active_map_matches(self, map_id: str) -> bool:
        try:
            response = self._maps.get_active_map()
        except Exception:
            return False
        return (
            isinstance(response, Mapping)
            and response.get("success") is True
            and str(response.get("active") or "").strip() == map_id
        )

    @staticmethod
    def _bundle_path(bundle: Any) -> Path | None:
        if not isinstance(bundle, Mapping) or bundle.get("success") is not True:
            return None
        artifact = bundle.get("artifact")
        if not isinstance(artifact, Mapping):
            return None
        raw_uri = str(artifact.get("uri") or artifact.get("path") or "").strip()
        if not raw_uri:
            return None
        path = Path(raw_uri).expanduser()
        if path.is_absolute():
            return path
        base = str(bundle.get("map_dir") or "").strip()
        return Path(base).expanduser() / path if base else path

    @staticmethod
    def _status_stamp(status: Mapping[str, Any]) -> float | None:
        raw = status.get("ts", status.get("stamp_s"))
        try:
            stamp_s = float(raw)
        except (TypeError, ValueError):
            return None
        return stamp_s if math.isfinite(stamp_s) else None
