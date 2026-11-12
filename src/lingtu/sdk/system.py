"""System state, diagnostics, emergency stop and operator control leases."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any
from urllib.parse import quote

from ._common import _new_request_id
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
            env=raw.get("env"),
            product=raw.get("product"),
            product_session_id=raw.get("product_session_id"),
        )

    def product_status(self) -> dict[str, Any]:
        """Read the current ProductControl session and latest switch operation."""
        return self._client._get("/api/v1/product-control")

    def switch_product(
        self, product: str, *, expected_product_session_id: str,
        map_name: str | None = None, initial_pose: dict[str, float] | None = None,
        request_id: str | None = None,
    ) -> dict[str, Any]:
        """Submit one durable switch operation to ProductControl."""
        body: dict[str, Any] = {
            "product": product,
            "expected_product_session_id": expected_product_session_id,
            "request_id": request_id or _new_request_id(),
        }
        if map_name is not None:
            body["map_name"] = map_name
        if initial_pose is not None:
            body["initial_pose"] = initial_pose
        return self._client._request_json("/api/v1/product-control/switch", body)

    def product_operation(self, request_id: str) -> dict[str, Any]:
        """Read a retained switch result by its submitted request identity."""
        return self._client._get(f"/api/v1/product-control/operations/{quote(request_id, safe='')}")

    def stop_product(self, expected_product_session_id: str, *, request_id: str | None = None) -> dict[str, Any]:
        """Submit a durable stop for the selected Product session."""
        return self._client._request_json("/api/v1/product-control/stop", {
            "expected_product_session_id": expected_product_session_id,
            "request_id": request_id or _new_request_id(),
        })

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

    def stop_motion(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Request a controlled motion stop through native arbitration."""
        return self._operator_command("/api/v1/motion/stop", request_id, client_id)

    def resume_motion(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Release the native motion hold through its existing owner."""
        return self._operator_command("/api/v1/navigation/resume", request_id, client_id)

    def clear_emergency_stop(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Explicitly release the native software emergency-stop latch."""
        return self._operator_command("/api/v1/estop/reset", request_id, client_id)

    def stand_up(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Request standing through the native motion owner."""
        return self._operator_command("/api/v1/motion/stand", request_id, client_id)

    def sit_down(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Request sitting through the native motion owner."""
        return self._operator_command("/api/v1/motion/sit", request_id, client_id)

    def enable_motors(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Request motor power enable through the native motion owner."""
        return self._operator_command("/api/v1/motion/motors/enable", request_id, client_id)

    def disable_motors(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Request motor power disable through the native motion owner."""
        return self._operator_command("/api/v1/motion/motors/disable", request_id, client_id)

    def switch_profile(
        self, name: str, *, request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Switch the native motion profile while preserving its admission gates."""
        return self._operator_command("/api/v1/motion/profile", request_id, client_id, {"name": name})

    def _operator_command(
        self, path: str, request_id: str | None, client_id: str, fields: dict[str, Any] | None = None,
    ) -> CommandResult:
        resolved_request_id = request_id or _new_request_id()
        body = {"request_id": resolved_request_id, "client_id": client_id, **(fields or {})}
        result = self._client._command(path, body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

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
