"""System state, diagnostics, emergency stop and operator control leases."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any

from .models import (
    CommandResult,
    HealthStatus,
    Position,
    RobotState,
    SessionInfo,
    _parse_navigation_status,
)

if TYPE_CHECKING:
    from .client import LingTuClient


class SystemClient:
    """System state, diagnostics, emergency stop and operator control leases."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    def state(self) -> RobotState:
        """Get full robot state: odometry, mission, safety, mode, etc."""
        raw = self._client._get("/api/v1/state")
        return self._parse_state(raw)

    def health(self) -> HealthStatus:
        """Get system health overview (modules, sensors, SLAM rate, etc.)."""
        raw = self._client._get("/api/v1/health")
        return HealthStatus(
            modules_ok=raw.get("modules_ok", 0),
            modules_fail=raw.get("modules_fail", 0),
            modules_total=raw.get("modules_ok", 0) + raw.get("modules_fail", 0),
            slam_rate=raw.get("slam_rate", 0.0),
            mode=raw.get("mode", ""),
            session=raw.get("session", ""),
            raw=raw,
        )

    def session(self) -> SessionInfo:
        """Get current session state (mode, SLAM profile, active map, etc.)."""
        raw = self._client._get("/api/v1/session")
        return SessionInfo(
            mode=raw.get("mode", "idle"),
            active_map=raw.get("active_map"),
            slam_profile=raw.get("slam_profile"),
            raw=raw,
        )

    def set_mode(self, mode: str) -> CommandResult:
        """Set robot operating mode."""
        return self._client._command("/api/v1/mode", {"mode": mode})

    def capabilities(self) -> dict[str, Any]:
        """Get the API capability manifest from the robot."""
        return self._client._get("/api/v1/app/capabilities")

    def bootstrap(self) -> dict[str, Any]:
        """Get the bootstrap snapshot (full system state for web clients)."""
        return self._client._get("/api/v1/app/bootstrap")

    def readiness(self) -> dict[str, Any]:
        """Get client readiness snapshot."""
        return self._client._get("/api/v1/readiness")

    def field_check(self) -> CommandResult:
        """Run a read-only product field readiness check."""
        return self._client._command("/api/v1/diagnostics/field-check")

    def runtime_contract(self) -> dict[str, Any]:
        """Get the canonical runtime interface contract."""
        return self._client._get("/api/v1/diagnostics/runtime-contract")

    def slam_status(self) -> dict[str, Any]:
        """Get SLAM service status."""
        return self._client._get("/api/v1/slam/status")

    @staticmethod
    def _parse_state(raw: dict[str, Any]) -> RobotState:
        localization = raw.get("localization")
        raw_odometry = localization.get("odometry") if isinstance(localization, Mapping) else None
        odometry = None
        if isinstance(raw_odometry, Mapping) and raw_odometry.get("frame_id") == "map":
            try:
                odometry = Position(
                    x=float(raw_odometry["x"]),
                    y=float(raw_odometry["y"]),
                    z=float(raw_odometry["z"]),
                    yaw=float(raw_odometry["yaw"]),
                )
            except (KeyError, TypeError, ValueError):
                odometry = None
        navigation = raw.get("navigation", {})
        navigation_payload = dict(navigation) if isinstance(navigation, Mapping) else {}
        return RobotState(
            mode=raw.get("mode", ""),
            odometry=odometry,
            mission=_parse_navigation_status(navigation_payload),
            safety_level=raw.get("safety_level", raw.get("safety", {}).get("level", "")),
            raw=raw,
        )

    def emergency_stop(self) -> CommandResult:
        """Request a software emergency stop; the receipt is not physical stop evidence."""
        return self._client._command("/api/v1/stop")

    def acquire_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Acquire the control lease."""
        return self._client._command(
            "/api/v1/lease",
            {
                "action": "acquire",
                "client_id": client_id,
                "ttl": ttl,
            },
        )

    def release_lease(self, client_id: str) -> CommandResult:
        """Release the control lease."""
        return self._client._command(
            "/api/v1/lease",
            {
                "action": "release",
                "client_id": client_id,
            },
        )

    def renew_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Renew the control lease before it expires."""
        return self._client._command(
            "/api/v1/lease",
            {
                "action": "renew",
                "client_id": client_id,
                "ttl": ttl,
            },
        )
