"""Synchronous HTTP client for the LingTu robot Gateway.

Provides typed convenience methods for every major Gateway REST endpoint.
Uses only the Python stdlib -- no third-party dependencies.

Usage::

    from lingtu.sdk import LingTuClient

    robot = LingTuClient("192.168.66.190", 5050)
    robot.go(10.0, 5.0, yaw=1.57)
    print(robot.position())
"""

from __future__ import annotations

import json
import math
import secrets
import tempfile
import time
import urllib.error
import urllib.request
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from urllib.parse import quote

# ruff: noqa: S310  -- stdlib HTTP client; URL opening is intentional

_CONNECTION_ERROR = "robot not reachable"
_ULID_ALPHABET = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"


def _connection_error_payload(path: str, exc: BaseException) -> dict[str, Any]:
    """Return a stable SDK error payload for offline or unreachable robots."""

    return {
        "ok": False,
        "success": False,
        "error": _CONNECTION_ERROR,
        "detail": str(exc),
        "path": path,
    }


# ---------------------------------------------------------------------------
# Typed response models
# ---------------------------------------------------------------------------


@dataclass
class Position:
    """Robot position in the map frame."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Pose2D:
    """Planar pose used as a localization seed."""

    x: float
    y: float
    yaw: float


@dataclass
class HealthStatus:
    """System health overview."""

    modules_ok: int = 0
    modules_fail: int = 0
    modules_total: int = 0
    slam_rate: float = 0.0
    mode: str = ""
    session: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class NavigationStatus:
    """Navigation mission state."""

    state: str = "idle"
    distance_to_goal: float | None = None
    time_elapsed: float = 0.0
    goal: Position = field(default_factory=Position)
    raw: dict[str, Any] = field(default_factory=dict)
    request_id: str | None = None


@dataclass
class MapInfo:
    """Summary of a single saved map."""

    name: str = ""
    has_pcd: bool = False
    is_active: bool = False
    size_bytes: int = 0
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class MapList:
    """Response from the maps endpoint."""

    maps: list[MapInfo] = field(default_factory=list)
    active_map: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SessionInfo:
    """Current robot session state."""

    mode: str = "idle"
    active_map: str | None = None
    slam_profile: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class CommandResult:
    """Command admission receipt (not mission-completion evidence)."""

    ok: bool = False
    message: str = ""
    raw: dict[str, Any] = field(default_factory=dict)
    operation_id: str | None = None
    accepted: bool = False
    request_id: str | None = None
    stage: str = ""


@dataclass
class RobotState:
    """Full robot state snapshot."""

    mode: str = ""
    odometry: Position = field(default_factory=Position)
    mission: NavigationStatus = field(default_factory=NavigationStatus)
    safety_level: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


_NAVIGATION_ACTIVE_STATES = frozenset(
    {
        "PLANNING",
        "EXECUTING",
        "PAUSED",
        "RECOVERING",
    }
)
_NAVIGATION_FAILURE_STATES = frozenset({"FAILED", "CANCELLED"})


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _validate_save_request_identity(payload: Mapping[str, Any], expected: str) -> None:
    operation = _mapping(payload.get("operation"))
    for container in (payload, operation):
        returned = str(container.get("request_id") or "").strip()
        if returned and returned != expected:
            raise RuntimeError(f"Map save request identity mismatch: expected {expected}, got {returned}")


def _validate_save_operation_identity(payload: Mapping[str, Any], expected: str) -> None:
    operation = _mapping(payload.get("operation"))
    for container in (payload, operation):
        returned = str(container.get("operation_id") or "").strip()
        if returned and returned != expected:
            raise RuntimeError(f"Map save operation identity mismatch: expected {expected}, got {returned}")


def _save_operation_failure_reason(
    payload: Mapping[str, Any],
    fallback: str = "",
) -> str:
    operation = _mapping(payload.get("operation"))
    return str(
        operation.get("reason_code")
        or operation.get("message")
        or operation.get("error")
        or operation.get("detail")
        or payload.get("reason_code")
        or payload.get("message")
        or payload.get("error")
        or fallback
        or payload.get("detail")
        or "unknown reason"
    )


def _save_response_is_connection_unknown(
    payload: Mapping[str, Any],
    fallback: str = "",
) -> bool:
    operation = _mapping(payload.get("operation"))
    candidates = (
        operation.get("error"),
        operation.get("reason_code"),
        payload.get("error"),
        payload.get("reason_code"),
        fallback,
    )
    return any(str(value or "").strip().lower() == _CONNECTION_ERROR for value in candidates)


def _validated_map_save_receipt(
    receipt: CommandResult,
    expected_request_id: str,
) -> tuple[str, bool]:
    """Validate a map-save receipt and return its operation identity."""
    _validate_save_request_identity(receipt.raw, expected_request_id)
    if receipt.request_id and receipt.request_id != expected_request_id:
        raise RuntimeError(
            f"Map save request identity mismatch: expected {expected_request_id}, got {receipt.request_id}"
        )

    operation_id = str(receipt.operation_id or "").strip()
    if operation_id:
        _validate_save_operation_identity(receipt.raw, operation_id)

    submission_unknown = _save_response_is_connection_unknown(
        receipt.raw,
        receipt.message,
    )
    if (not receipt.ok or not receipt.accepted) and not submission_unknown:
        reason = _save_operation_failure_reason(receipt.raw, receipt.message)
        raise RuntimeError(f"Map save request rejected: {reason}")
    return operation_id, submission_unknown


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _new_request_id() -> str:
    """Return a canonical 26-character uppercase ULID."""
    timestamp_ms = (time.time_ns() // 1_000_000) & ((1 << 48) - 1)
    value = (timestamp_ms << 80) | secrets.randbits(80)
    encoded = ["0"] * 26
    for index in range(25, -1, -1):
        encoded[index] = _ULID_ALPHABET[value & 0x1F]
        value >>= 5
    return "".join(encoded)


def _navigation_mission_payload(raw: Mapping[str, Any]) -> Mapping[str, Any]:
    mission = _mapping(raw.get("mission"))
    mission_raw = _mapping(mission.get("raw"))
    return mission_raw or mission


def _navigation_goal_payload(raw: Mapping[str, Any]) -> Mapping[str, Any]:
    target = _mapping(raw.get("target"))
    goal = _mapping(target.get("goal"))
    if goal:
        return goal
    return _mapping(raw.get("goal"))


def _navigation_request_id(raw: Mapping[str, Any]) -> str | None:
    mission = _mapping(raw.get("mission"))
    mission_raw = _mapping(mission.get("raw"))
    diagnostics = _mapping(raw.get("diagnostics"))
    last_plan = _mapping(diagnostics.get("last_plan_report"))
    candidates = (
        raw.get("request_id"),
        mission.get("request_id"),
        mission_raw.get("request_id"),
        mission_raw.get("active_request_id"),
        mission_raw.get("planning_request_id"),
        last_plan.get("request_id"),
    )
    for candidate in candidates:
        normalized = str(candidate or "").strip()
        if normalized:
            return normalized
    return None


def _parse_navigation_status(raw: dict[str, Any]) -> NavigationStatus:
    target = _mapping(raw.get("target"))
    mission = _navigation_mission_payload(raw)
    goal_raw = _navigation_goal_payload(raw)
    distance = _optional_float(target.get("distance_to_goal_m"))
    if distance is None:
        distance = _optional_float(raw.get("distance_to_goal_m"))
    if distance is None:
        distance = _optional_float(raw.get("distance_to_goal"))
    elapsed = _optional_float(mission.get("time_elapsed"))
    if elapsed is None:
        elapsed = _optional_float(raw.get("time_elapsed"))
    return NavigationStatus(
        state=str(raw.get("state") or mission.get("state") or "idle"),
        distance_to_goal=distance,
        time_elapsed=elapsed or 0.0,
        goal=Position(
            x=_optional_float(goal_raw.get("x")) or 0.0,
            y=_optional_float(goal_raw.get("y")) or 0.0,
            z=_optional_float(goal_raw.get("z")) or 0.0,
            yaw=_optional_float(goal_raw.get("yaw")) or 0.0,
        ),
        request_id=_navigation_request_id(raw),
        raw=raw,
    )


def _parse_command_result(raw: dict[str, Any]) -> CommandResult:
    command = _mapping(raw.get("command"))
    accepted_signal = command.get("accepted", raw.get("accepted"))
    if accepted_signal is None:
        accepted = raw.get("ok") is True or raw.get("success") is True
    else:
        accepted = accepted_signal is True

    if "ok" in raw:
        response_ok = raw.get("ok") is True
    elif "success" in raw:
        response_ok = raw.get("success") is True
    else:
        response_ok = accepted

    request_id = str(command.get("request_id") or raw.get("request_id") or "").strip() or None
    stage = str(raw.get("stage") or raw.get("status") or command.get("stage") or "")
    operation_id = str(command.get("operation_id") or raw.get("operation_id") or "").strip() or None
    return CommandResult(
        ok=response_ok and accepted,
        accepted=accepted,
        message=str(raw.get("message", raw.get("reason", raw.get("error", ""))) or ""),
        request_id=request_id,
        stage=stage,
        raw=raw,
        operation_id=operation_id,
    )


def _navigation_state(status: NavigationStatus) -> str:
    return str(status.state or "").strip().upper()


def _navigation_failure_reason(status: NavigationStatus) -> str:
    raw = _mapping(status.raw)
    mission = _navigation_mission_payload(raw)
    diagnostics = _mapping(raw.get("diagnostics"))
    return str(
        raw.get("failure_reason")
        or mission.get("failure_reason")
        or diagnostics.get("failure_reason")
        or "unknown reason"
    )


def _navigation_marker(status: NavigationStatus) -> tuple[Any, ...]:
    raw = _mapping(status.raw)
    mission = _navigation_mission_payload(raw)
    return (
        status.request_id or _navigation_request_id(raw),
        mission.get("ts"),
        raw.get("ts"),
        _navigation_state(status),
        status.goal.x,
        status.goal.y,
        status.goal.z,
        status.goal.yaw,
        raw.get("wp_index"),
        raw.get("wp_total"),
        _navigation_failure_reason(status),
    )


def _navigation_goal_matches(
    status: NavigationStatus,
    expected_goal: tuple[float, float, float] | None,
) -> bool:
    if expected_goal is None:
        return True
    raw = _mapping(status.raw)
    goal_raw = _navigation_goal_payload(raw)
    if raw and not goal_raw:
        return False
    expected_x, expected_y, expected_yaw = expected_goal
    position_matches = math.isclose(
        status.goal.x,
        expected_x,
        abs_tol=1e-3,
    ) and math.isclose(
        status.goal.y,
        expected_y,
        abs_tol=1e-3,
    )
    if not position_matches:
        return False

    observed_yaw = _optional_float(goal_raw.get("yaw")) if goal_raw else status.goal.yaw
    if observed_yaw is None:
        return True
    yaw_error = math.atan2(
        math.sin(observed_yaw - expected_yaw),
        math.cos(observed_yaw - expected_yaw),
    )
    return math.isclose(yaw_error, 0.0, abs_tol=1e-3)


def _navigation_wait_outcome(
    status: NavigationStatus,
    *,
    request_id: str | None,
    expected_goal: tuple[float, float, float] | None,
    baseline: NavigationStatus | None,
    active_seen: bool,
) -> tuple[bool, str | None]:
    state = _navigation_state(status)
    observed_request_id = status.request_id or _navigation_request_id(_mapping(status.raw))
    if request_id and observed_request_id and observed_request_id != request_id:
        return active_seen, None

    request_matches = bool(request_id and observed_request_id == request_id)
    goal_matches = _navigation_goal_matches(status, expected_goal)
    advanced = baseline is None or _navigation_marker(status) != _navigation_marker(baseline)
    no_identity_constraint = request_id is None and expected_goal is None and baseline is None
    goal_transition = expected_goal is not None and advanced and goal_matches
    baseline_transition = baseline is not None and request_id is None and expected_goal is None and advanced
    relevant = request_matches or no_identity_constraint or goal_transition or baseline_transition

    if state in _NAVIGATION_ACTIVE_STATES and relevant:
        active_seen = True

    if state in _NAVIGATION_FAILURE_STATES and relevant:
        return active_seen, "failure"
    success_relevant = request_matches or active_seen or no_identity_constraint
    if state == "SUCCESS" and success_relevant:
        return active_seen, "success"
    return active_seen, None


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------


class LocalizationClient:
    """Domain facade for localization commands."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    def relocalize(
        self,
        map_name: str,
        *,
        initial_pose: Pose2D,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize on the active map from a caller-provided pose seed."""
        body: dict[str, Any] = {
            "mode": "seeded",
            "map_name": map_name,
            "initial_pose": {
                "x": initial_pose.x,
                "y": initial_pose.y,
                "yaw": initial_pose.yaw,
            },
        }
        if request_id is not None:
            body["request_id"] = request_id
        return self._client._command("/api/v1/localization/relocalizations", body)

    def global_relocalize(
        self,
        map_name: str,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize on the active map without a pose seed."""
        body: dict[str, Any] = {"mode": "global", "map_name": map_name}
        if request_id is not None:
            body["request_id"] = request_id
        return self._client._command("/api/v1/localization/relocalizations", body)

    def start_map_tracking(
        self,
        map_name: str,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Start continuous alignment against the active map."""
        body: dict[str, Any] = {"map_name": map_name}
        if request_id is not None:
            body["request_id"] = request_id
        return self._client._command("/api/v1/localization/map-tracking", body)


class LingTuClient:
    """Synchronous client for LingTu robot control and state inspection."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5050,
        api_key: str | None = None,
        timeout: float = 10.0,
    ) -> None:
        """Connect to a LingTu robot Gateway."""
        self.base_url = f"http://{host}:{port}"
        self._api_key = api_key
        self._timeout = timeout
        self._localization = LocalizationClient(self)

    @property
    def localization(self) -> LocalizationClient:
        """Access localization operations without exposing backend names."""
        return self._localization

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> LingTuClient:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def go(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Navigate to map coordinates."""
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        result = self._command(
            "/api/v1/goal",
            {"x": x, "y": y, "yaw": yaw, "request_id": resolved_request_id},
        )
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    def go_to(self, target: str) -> CommandResult:
        """Navigate by semantic label or natural-language instruction."""
        return self._command("/api/v1/instruction", {"text": target})

    def follow_person(self, target_id: str) -> CommandResult:
        """Follow the person currently published with ``target_id``."""
        return self._command(
            "/api/v1/visual_servo",
            {"mode": "follow", "target_id": target_id},
        )

    def stop_following(self) -> CommandResult:
        """Stop the active person-following task without issuing an emergency stop."""
        return self._command("/api/v1/visual_servo", {"mode": "stop"})

    def stop(self) -> CommandResult:
        """Emergency stop -- immediately halt all motion."""
        return self._command("/api/v1/stop")

    def cancel(self, reason: str = "client_cancel") -> CommandResult:
        """Gracefully cancel the current navigation mission."""
        return self._command("/api/v1/navigation/cancel", {"reason": reason})

    def navigate_click(self, x: float, y: float, yaw: float = 0.0) -> CommandResult:
        """Navigate to a map-viewer click point."""
        return self._command("/api/v1/navigate/click", {"x": x, "y": y, "yaw": yaw})

    def batch_go(self, waypoints: list[tuple[float, float, float]]) -> list[CommandResult]:
        """Navigate through a sequence of waypoints (blocking)."""
        results: list[CommandResult] = []
        for x, y, yaw in waypoints:
            baseline = self.navigation_status()
            result = self.go(x, y, yaw)
            results.append(result)
            if not result.ok:
                break
            self.wait_until_arrived(
                request_id=result.request_id,
                expected_goal=(x, y, yaw),
                baseline=baseline,
            )
        return results

    def wait_until_arrived(
        self,
        timeout: float = 120.0,
        poll_interval: float = 0.5,
        *,
        request_id: str | None = None,
        expected_goal: tuple[float, float, float] | None = None,
        baseline: NavigationStatus | None = None,
    ) -> NavigationStatus:
        """Block until the current navigation mission completes."""
        deadline = time.monotonic() + timeout
        active_seen = False
        status = baseline or NavigationStatus()
        while time.monotonic() < deadline:
            status = self.navigation_status()
            active_seen, outcome = _navigation_wait_outcome(
                status,
                request_id=request_id,
                expected_goal=expected_goal,
                baseline=baseline,
                active_seen=active_seen,
            )
            if outcome == "success":
                return status
            if outcome == "failure":
                state = _navigation_state(status)
                raise RuntimeError(f"Navigation {state}: {_navigation_failure_reason(status)}")
            time.sleep(poll_interval)

        distance = "unknown" if status.distance_to_goal is None else f"{status.distance_to_goal:.2f}m"
        raise TimeoutError(f"Navigation did not complete within {timeout}s (state={status.state}, dist={distance})")

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def state(self) -> RobotState:
        """Get full robot state: odometry, mission, safety, mode, etc."""
        raw = self._get("/api/v1/state")
        return self._parse_state(raw)

    def health(self) -> HealthStatus:
        """Get system health overview (modules, sensors, SLAM rate, etc.)."""
        raw = self._get("/api/v1/health")
        return HealthStatus(
            modules_ok=raw.get("modules_ok", 0),
            modules_fail=raw.get("modules_fail", 0),
            modules_total=raw.get("modules_ok", 0) + raw.get("modules_fail", 0),
            slam_rate=raw.get("slam_rate", 0.0),
            mode=raw.get("mode", ""),
            session=raw.get("session", ""),
            raw=raw,
        )

    def position(self) -> Position:
        """Get current position."""
        return self.state().odometry

    def session(self) -> SessionInfo:
        """Get current session state (mode, SLAM profile, active map, etc.)."""
        raw = self._get("/api/v1/session")
        return SessionInfo(
            mode=raw.get("mode", "idle"),
            active_map=raw.get("active_map"),
            slam_profile=raw.get("slam_profile"),
            raw=raw,
        )

    def navigation_status(self) -> NavigationStatus:
        """Get current navigation mission status."""
        raw = self._get("/api/v1/navigation/status")
        return _parse_navigation_status(raw)

    def localization_status(self) -> dict[str, Any]:
        """Get localization health: alignment, residual, fix quality."""
        return self._get("/api/v1/localization/status")

    def path(self) -> dict[str, Any]:
        """Get the latest planned global path as a list of poses."""
        return self._get("/api/v1/path")

    # ------------------------------------------------------------------
    # Maps
    # ------------------------------------------------------------------

    def maps(self) -> MapList:
        """List saved maps on the robot."""
        raw = self._get("/api/v1/slam/maps")
        return self._parse_map_list(raw)

    def save_map(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Queue a durable save of the current SLAM map."""
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        body: dict[str, Any] = {"request_id": resolved_request_id}
        if name:
            body["name"] = name
        result = self._command("/api/v1/map/save", body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    def save_map_and_wait(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
        timeout: float = 300.0,
        poll_interval: float = 1.0,
    ) -> dict[str, Any]:
        """Save the current map and wait for its durable terminal result."""
        deadline = time.monotonic() + timeout
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        while True:
            receipt = self.save_map(
                name,
                request_id=resolved_request_id,
            )
            operation_id, submission_unknown = _validated_map_save_receipt(
                receipt,
                resolved_request_id,
            )
            if receipt.ok and receipt.accepted and operation_id:
                break
            if not submission_unknown:
                raise RuntimeError("Map save response did not include operation_id")
            if time.monotonic() >= deadline:
                raise TimeoutError("Map save admission remained unknown until the timeout")
            time.sleep(poll_interval)
        state = ""
        while time.monotonic() < deadline:
            response = self.get_map_operation(operation_id)
            _validate_save_operation_identity(response, operation_id)
            operation = _mapping(response.get("operation"))
            state = str(operation.get("state") or "").strip().upper()
            if state == "SUCCEEDED":
                return response
            if state in {"FAILED", "CANCELLED", "CANCELED"}:
                raise RuntimeError(f"Map save {state}: {_save_operation_failure_reason(response)}")
            reason_code = str(operation.get("reason_code") or response.get("reason_code") or "").strip().lower()
            transient = reason_code == "operation_not_found" or _save_response_is_connection_unknown(response)
            explicit_error = (
                response.get("ok") is False
                or response.get("success") is False
                or response.get("error") is not None
                or response.get("detail") is not None
                or operation.get("error") is not None
                or operation.get("detail") is not None
                or state in {"ERROR", "REJECTED", "NOT_FOUND"}
            )
            if explicit_error and not transient:
                raise RuntimeError(f"Map save status query failed: {_save_operation_failure_reason(response)}")
            time.sleep(poll_interval)
        raise TimeoutError(f"Map save did not complete within {timeout}s (state={state or 'UNKNOWN'})")

    def get_map_operation(self, operation_id: str) -> dict[str, Any]:
        """Return the durable state of one map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return self._get(f"/api/v1/maps/operations/{encoded_operation_id}")

    def cancel_map_operation(self, operation_id: str) -> CommandResult:
        """Request cancellation of one map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return self._command(f"/api/v1/maps/operations/{encoded_operation_id}/cancel")

    def retry_map_operation(self, operation_id: str) -> CommandResult:
        """Retry one failed map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return self._command(f"/api/v1/maps/operations/{encoded_operation_id}/retry")

    def download_map_pcd(self, name: str, target: str | Path) -> Path:
        """Stream a saved map PCD artifact to ``target`` atomically."""
        destination = Path(target)
        encoded_name = quote(name, safe="")
        request = urllib.request.Request(self.base_url + f"/api/v1/maps/{encoded_name}/pcd")
        if self._api_key:
            request.add_header("X-API-Key", self._api_key)
        temporary_path: Path | None = None
        try:
            with urllib.request.urlopen(request, timeout=self._timeout) as response:
                with tempfile.NamedTemporaryFile(
                    mode="wb",
                    dir=destination.parent,
                    prefix=f".{destination.name}.",
                    suffix=".part",
                    delete=False,
                ) as output:
                    temporary_path = Path(output.name)
                    while True:
                        chunk = response.read(1024 * 1024)
                        if not chunk:
                            break
                        output.write(chunk)
            temporary_path.replace(destination)
        except BaseException:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
            raise
        return destination

    def rename_map(self, old_name: str, new_name: str) -> CommandResult:
        """Rename a saved map."""
        return self._command("/api/v1/map/rename", {"old_name": old_name, "new_name": new_name})

    def reset_map_cloud(self) -> CommandResult:
        """Clear the accumulated map cloud (visualisation only)."""
        return self._command("/api/v1/map_cloud/reset")

    def map_points(self) -> dict[str, Any]:
        """Get current live map point cloud as JSON."""
        return self._get("/api/v1/map/points")

    def saved_map_points(self, name: str) -> dict[str, Any]:
        """Get a saved map point cloud as JSON."""
        return self._get(f"/api/v1/maps/{name}/points")

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    def set_mode(self, mode: str) -> CommandResult:
        """Set robot operating mode."""
        return self._command("/api/v1/mode", {"mode": mode})

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    def scene(self) -> dict[str, Any]:
        """Get the current scene graph (detected objects, relations, regions)."""
        return self._get("/api/v1/scene_graph")

    def locations(self) -> dict[str, Any]:
        """List tagged navigation locations."""
        return self._get("/api/v1/locations")

    def tag_location(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        use_current_pose: bool = False,
    ) -> CommandResult:
        """Create or update a tagged navigation location."""
        body: dict[str, Any] = {"name": name, "use_current_pose": use_current_pose}
        if x is not None:
            body["x"] = x
        if y is not None:
            body["y"] = y
        if yaw is not None:
            body["yaw"] = yaw
        return self._command("/api/v1/locations", body)

    def delete_location(self, name: str) -> CommandResult:
        """Delete a tagged navigation location."""
        return self._command(f"/api/v1/locations/{name}")

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------

    def camera_snapshot(self) -> bytes:
        """Fetch a JPEG camera snapshot from the robot."""
        url = self.base_url + "/api/v1/camera/snapshot"
        req = urllib.request.Request(url)
        if self._api_key:
            req.add_header("X-API-Key", self._api_key)
        with urllib.request.urlopen(req, timeout=self._timeout) as r:
            return r.read()

    # ------------------------------------------------------------------
    # SLAM
    # ------------------------------------------------------------------

    def slam_status(self) -> dict[str, Any]:
        """Get SLAM service status."""
        return self._get("/api/v1/slam/status")

    # ------------------------------------------------------------------
    # Exploration
    # ------------------------------------------------------------------

    def explore_start(self) -> CommandResult:
        """Start autonomous frontier exploration."""
        return self._command("/api/v1/explore/start")

    def explore_stop(self) -> CommandResult:
        """Stop autonomous frontier exploration."""
        return self._command("/api/v1/explore/stop")

    def explore_status(self) -> dict[str, Any]:
        """Get exploration status."""
        return self._get("/api/v1/explore/status")

    # ------------------------------------------------------------------
    # Native MCAP recording
    # ------------------------------------------------------------------

    def recording_start(
        self,
        prefix: str | None = None,
        *,
        duration: int = 600,
        capture_profile: str | None = None,
        task_id: str | None = None,
        camera: bool | None = None,
        minimum_free_gib: int | None = None,
    ) -> CommandResult:
        """Start a native MCAP recording session with safe capture choices."""
        body: dict[str, Any] = {"duration": duration}
        if prefix:
            body["prefix"] = prefix
        if capture_profile is not None:
            body["capture_profile"] = capture_profile
        if task_id is not None:
            body["task_id"] = task_id
        if camera is not None:
            body["camera"] = camera
        if minimum_free_gib is not None:
            body["minimum_free_gib"] = minimum_free_gib
        return self._command("/api/v1/recordings/start", body)

    def recording_stop(self) -> CommandResult:
        """Stop the active native MCAP recording session."""
        return self._command("/api/v1/recordings/stop")

    def recording_status(self) -> dict[str, Any]:
        """Get native MCAP recording status."""
        return self._get("/api/v1/recordings/status")

    # ------------------------------------------------------------------
    # Temporal memory
    # ------------------------------------------------------------------

    def memory_temporal(self) -> dict[str, Any]:
        """Query temporal entity observations."""
        return self._get("/api/v1/memory/temporal")

    def memory_temporal_semantic(self, query: str) -> dict[str, Any]:
        """Semantic similarity search over temporal observations."""
        return self._post("/api/v1/memory/temporal/semantic", {"query": query})

    # ------------------------------------------------------------------
    # Lease (control ownership)
    # ------------------------------------------------------------------

    def acquire_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Acquire the control lease."""
        return self._command(
            "/api/v1/lease",
            {
                "action": "acquire",
                "client_id": client_id,
                "ttl": ttl,
            },
        )

    def release_lease(self, client_id: str) -> CommandResult:
        """Release the control lease."""
        return self._command(
            "/api/v1/lease",
            {
                "action": "release",
                "client_id": client_id,
            },
        )

    def renew_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Renew the control lease before it expires."""
        return self._command(
            "/api/v1/lease",
            {
                "action": "renew",
                "client_id": client_id,
                "ttl": ttl,
            },
        )

    # ------------------------------------------------------------------
    # App / system info
    # ------------------------------------------------------------------

    def capabilities(self) -> dict[str, Any]:
        """Get the API capability manifest from the robot."""
        return self._get("/api/v1/app/capabilities")

    def bootstrap(self) -> dict[str, Any]:
        """Get the bootstrap snapshot (full system state for web clients)."""
        return self._get("/api/v1/app/bootstrap")

    def readiness(self) -> dict[str, Any]:
        """Get client readiness snapshot."""
        return self._get("/api/v1/readiness")

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def field_check(self) -> CommandResult:
        """Run a read-only product field readiness check."""
        return self._command("/api/v1/diagnostics/field-check")

    def runtime_contract(self) -> dict[str, Any]:
        """Get the canonical runtime interface contract."""
        return self._get("/api/v1/diagnostics/runtime-contract")

    # ------------------------------------------------------------------
    # Auth
    # ------------------------------------------------------------------

    def auth_login(self, api_key: str) -> dict[str, Any]:
        """Login with an API key."""
        return self._post("/api/v1/auth/login", {"key": api_key})

    def auth_check(self) -> dict[str, Any]:
        """Check if authentication is required."""
        return self._get("/api/v1/auth/check")

    # ------------------------------------------------------------------
    # Internal HTTP helpers
    # ------------------------------------------------------------------

    def _command(self, path: str, data: dict[str, Any] | None = None) -> CommandResult:
        raw = self._post(path, data)
        return _parse_command_result(raw)

    def _get(self, path: str) -> dict[str, Any]:
        url = self.base_url + path
        req = urllib.request.Request(url)
        if self._api_key:
            req.add_header("X-API-Key", self._api_key)
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as exc:
            body = exc.read()
            try:
                detail = json.loads(body)
            except json.JSONDecodeError:
                detail = body.decode(errors="replace")
            raise RuntimeError(f"GET {path} failed with HTTP {exc.code}: {detail}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise ConnectionError(f"GET {path} failed: {exc}") from exc

    def _post(self, path: str, data: dict[str, Any] | None = None) -> dict[str, Any]:
        url = self.base_url + path
        body = json.dumps(data or {}).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        if self._api_key:
            req.add_header("X-API-Key", self._api_key)
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as exc:
            err_body = exc.read()
            try:
                return json.loads(err_body)
            except json.JSONDecodeError:
                return {"error": f"HTTP {exc.code}", "detail": err_body.decode(errors="replace")}
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            return _connection_error_payload(path, exc)

    @staticmethod
    def _parse_state(raw: dict[str, Any]) -> RobotState:
        o = raw.get("odometry", {})
        m = raw.get("navigation", raw.get("mission", {}))
        mission_payload = dict(m) if isinstance(m, Mapping) else {}
        if "state" not in mission_payload and raw.get("mission_state") is not None:
            mission_payload["state"] = raw.get("mission_state")
        return RobotState(
            mode=raw.get("mode", ""),
            odometry=Position(
                x=o.get("x", 0.0),
                y=o.get("y", 0.0),
                z=o.get("z", 0.0),
                yaw=o.get("yaw", 0.0),
            ),
            mission=_parse_navigation_status(mission_payload),
            safety_level=raw.get("safety_level", raw.get("safety", {}).get("level", "")),
            raw=raw,
        )

    @staticmethod
    def _parse_map_list(raw: dict[str, Any]) -> MapList:
        active = raw.get("active", raw.get("active_map"))
        maps_raw = raw.get("maps", [])
        maps: list[MapInfo] = []
        for m in maps_raw:
            if isinstance(m, str):
                maps.append(MapInfo(name=m, raw={"name": m}))
            elif isinstance(m, dict):
                maps.append(
                    MapInfo(
                        name=m.get("name", ""),
                        has_pcd=m.get("has_pcd", False),
                        is_active=m.get("is_active", False) or m.get("name") == active,
                        size_bytes=m.get("size_bytes", 0),
                        raw=m,
                    )
                )
        return MapList(maps=maps, active_map=active, raw=raw)

    def close(self) -> None:
        """Close any held resources (no-op for the stdlib client)."""
        pass
