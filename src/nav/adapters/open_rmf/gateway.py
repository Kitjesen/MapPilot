"""LingTu Gateway mission port for the optional Open-RMF sidecar."""

from __future__ import annotations

import json
import math
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

from .single_robot import BuildingMissionRequest, FloorBinding, RobotSnapshot

_ALLOWED_RMF_ROUTES = frozenset(
    {
        ("GET", "/api/v1/session"),
        ("GET", "/api/v1/navigation/status"),
        ("GET", "/api/v1/navigation/dds_snapshot"),
        ("POST", "/api/v1/lease"),
        ("POST", "/api/v1/goal"),
        ("POST", "/api/v1/navigation/cancel"),
    }
)


@dataclass(frozen=True)
class GatewayClientConfig:
    """Connection and safety policy for one LingTu Gateway."""

    base_url: str
    api_key: str = field(repr=False)
    client_id: str
    commands_enabled: bool = False
    lease_ttl_s: float = 30.0
    timeout_s: float = 3.0
    battery_soc_default: float = 1.0
    max_pose_age_s: float = 2.0
    max_native_status_age_s: float = 2.0
    allow_insecure_http: bool = False

    def __post_init__(self) -> None:
        if not str(self.client_id).strip():
            raise ValueError("Gateway client_id is required")
        if self.commands_enabled and not str(self.api_key).strip():
            raise ValueError("Live Open-RMF commands require a scoped API key")
        if self.lease_ttl_s <= 0.0:
            raise ValueError("Gateway lease_ttl_s must be positive")
        if self.max_pose_age_s <= 0.0 or self.max_native_status_age_s <= 0.0:
            raise ValueError("Gateway freshness limits must be positive")


class JsonTransport(Protocol):
    """External HTTP transport used by the Gateway mission port."""

    def request(
        self,
        method: str,
        path: str,
        *,
        body: dict[str, Any] | None = None,
        headers: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        """Send one JSON request and return the decoded JSON object."""


class GatewayHttpTransport:
    """Standard-library JSON transport for a remote LingTu Gateway."""

    def __init__(
        self,
        *,
        base_url: str,
        timeout_s: float = 3.0,
        allow_insecure_http: bool = False,
        opener: Any = urlopen,
    ) -> None:
        normalized = str(base_url).rstrip("/") + "/"
        parsed = urlparse(normalized)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise ValueError("Gateway base_url must be an absolute http(s) URL")
        loopback = parsed.hostname in {"localhost", "127.0.0.1", "::1"}
        if parsed.scheme == "http" and not (allow_insecure_http or loopback):
            raise ValueError("Plain HTTP requires allow_insecure_http=True outside loopback")
        self._base_url = normalized
        self._timeout_s = max(0.1, float(timeout_s))
        self._opener = opener

    def request(
        self,
        method: str,
        path: str,
        *,
        body: dict[str, Any] | None = None,
        headers: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        """Send one JSON request and decode a JSON object response."""

        method = str(method).upper()
        path = "/" + str(path).lstrip("/")
        if (method, path) not in _ALLOWED_RMF_ROUTES:
            raise PermissionError(f"Open-RMF Gateway route is not allowed: {method} {path}")
        payload = None if body is None else json.dumps(body).encode("utf-8")
        request_headers = dict(headers or {})
        if payload is not None:
            request_headers["Content-Type"] = "application/json"
        request = Request(  # noqa: S310 - base URL is prevalidated as HTTP(S).
            urljoin(self._base_url, path.lstrip("/")),
            data=payload,
            headers=request_headers,
            method=method,
        )
        try:
            with self._opener(request, timeout=self._timeout_s) as response:
                decoded = json.loads(response.read().decode("utf-8"))
        except HTTPError as exc:
            try:
                decoded = json.loads(exc.read().decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError) as parse_exc:
                raise RuntimeError(f"LingTu Gateway HTTP error {exc.code}") from parse_exc
            if isinstance(decoded, dict):
                decoded.setdefault("ok", False)
                decoded.setdefault("http_status", int(exc.code))
                return decoded
            raise RuntimeError(f"LingTu Gateway HTTP error {exc.code}") from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise RuntimeError("LingTu Gateway transport unavailable") from exc
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise RuntimeError("LingTu Gateway returned invalid JSON") from exc
        if not isinstance(decoded, dict):
            raise RuntimeError("LingTu Gateway returned a non-object JSON response")
        return decoded


class GatewayMissionPort:
    """Submit BuildingMission requests through LingTu Gateway control routes."""

    def __init__(
        self,
        *,
        config: GatewayClientConfig,
        transport: JsonTransport,
    ) -> None:
        self._config = config
        self._transport = transport

    def _headers(self) -> dict[str, str]:
        return {"X-API-Key": self._config.api_key} if self._config.api_key else {}

    @staticmethod
    def _rejection_reason(response: Mapping[str, Any], fallback: str) -> str:
        return str(response.get("error") or response.get("status") or response.get("message") or fallback)

    def _lease_command(
        self,
        *,
        action: str,
        request_id: str,
    ) -> tuple[bool, str]:
        try:
            response = self._transport.request(
                "POST",
                "/api/v1/lease",
                body={
                    "action": action,
                    "client_id": self._config.client_id,
                    "request_id": request_id,
                    "ttl": self._config.lease_ttl_s,
                },
                headers=self._headers(),
            )
        except Exception:
            return False, "gateway_transport_error"
        if response.get("ok") is not True:
            return False, self._rejection_reason(
                response,
                f"gateway_lease_{action}_rejected",
            )
        holder = str(response.get("holder") or "")
        active = response.get("active")
        if not isinstance(active, bool):
            return False, "gateway_lease_state_invalid"
        if action in {"acquire", "renew"}:
            if holder != self._config.client_id or active is not True:
                return False, "gateway_lease_not_owned"
        elif active is True and holder == self._config.client_id:
            return False, "gateway_lease_release_unconfirmed"
        success_reason = {
            "acquire": "gateway_lease_acquired",
            "renew": "gateway_lease_renewed",
            "release": "gateway_lease_released",
        }
        return True, success_reason[action]

    def submit(self, request: BuildingMissionRequest) -> tuple[bool, str]:
        """Submit a mission only when command mode is explicitly enabled."""

        if not self._config.commands_enabled:
            return False, "open_rmf_shadow_mode"
        headers = self._headers()
        try:
            session = self._transport.request(
                "GET",
                "/api/v1/session",
                headers=headers,
            )
        except Exception:
            return False, "gateway_transport_error"
        active_map = str(session.get("active_map") or session.get("saved_active_map") or "")
        if active_map != request.map_id:
            return False, "floor_transition_executor_unavailable"
        if bool(session.get("pending", False)):
            return False, "gateway_navigation_session_pending"
        if str(session.get("mode") or "") != "navigating":
            return False, "gateway_navigation_session_not_ready"

        lease_ok, lease_reason = self._lease_command(
            action="acquire",
            request_id=f"{request.request_id}:lease-acquire",
        )
        if not lease_ok:
            return False, lease_reason

        try:
            goal = self._transport.request(
                "POST",
                "/api/v1/goal",
                body={
                    "x": request.target.x,
                    "y": request.target.y,
                    "z": request.target.z,
                    "yaw": request.target.yaw,
                    "frame_id": request.target.frame_id,
                    "source": "api",
                    "target_type": "coordinate",
                    "request_id": request.request_id,
                    "client_id": self._config.client_id,
                    "metadata": {
                        "source": request.source,
                        "building_id": request.building_id,
                        "floor_id": request.floor_id,
                        "map_id": request.map_id,
                        "map_name": request.map_id,
                    },
                },
                headers=headers,
            )
        except Exception:
            cancelled, _reason = self.cancel(
                request.request_id,
                reason="open_rmf_goal_outcome_unknown",
            )
            if cancelled:
                return False, "gateway_goal_outcome_unknown_cancelled"
            return False, "gateway_goal_outcome_unknown_cancel_pending"
        if goal.get("ok") is not True:
            released, _release_reason = self.release_lease(request.request_id)
            if not released:
                return False, "gateway_goal_rejected_release_pending"
            return False, self._rejection_reason(goal, "gateway_goal_rejected")
        return True, "gateway_navigation_goal_accepted"

    def renew_lease(
        self,
        request_id: str,
        *,
        renewal_sequence: int,
    ) -> tuple[bool, str]:
        """Renew the active RMF command lease with a unique idempotency key."""

        if not self._config.commands_enabled:
            return False, "open_rmf_shadow_mode"
        return self._lease_command(
            action="renew",
            request_id=f"{request_id}:lease-renew:{int(renewal_sequence)}",
        )

    def release_lease(self, request_id: str) -> tuple[bool, str]:
        """Release the RMF command lease and verify ownership was cleared."""

        if not self._config.commands_enabled:
            return False, "open_rmf_shadow_mode"
        return self._lease_command(
            action="release",
            request_id=f"{request_id}:lease-release",
        )

    def cancel(
        self,
        request_id: str,
        *,
        reason: str = "open_rmf_cancel",
    ) -> tuple[bool, str]:
        """Cancel the active native navigation command, then release its lease."""

        if not self._config.commands_enabled:
            return False, "open_rmf_shadow_mode"
        try:
            response = self._transport.request(
                "POST",
                "/api/v1/navigation/cancel",
                body={
                    "reason": str(reason),
                    "request_id": f"{request_id}:cancel",
                    "client_id": self._config.client_id,
                },
                headers=self._headers(),
            )
        except Exception:
            return False, "gateway_transport_error"
        if response.get("ok") is not True:
            return False, self._rejection_reason(response, "gateway_cancel_rejected")
        released, release_reason = self.release_lease(request_id)
        if not released:
            return False, (f"gateway_navigation_cancelled_but_{release_reason}")
        return True, "gateway_navigation_cancelled"


class GatewayRobotStateSource:
    """Read floor-verified robot state from LingTu Gateway snapshots."""

    def __init__(
        self,
        *,
        config: GatewayClientConfig,
        transport: JsonTransport,
        floor_bindings: dict[str, FloorBinding],
        clock: Any = time.time,
    ) -> None:
        self._config = config
        self._transport = transport
        self._floor_bindings = dict(floor_bindings)
        map_ids = [item.map_id for item in self._floor_bindings.values()]
        if len(set(map_ids)) != len(map_ids):
            raise ValueError("LingTu map_id must map to exactly one Open-RMF level")
        self._clock = clock

    def snapshot(self) -> RobotSnapshot:
        """Return one robot snapshot or raise when floor/pose is unavailable."""

        headers = {"X-API-Key": self._config.api_key} if self._config.api_key else {}
        session = self._transport.request("GET", "/api/v1/session", headers=headers)
        active_map = str(session.get("active_map") or session.get("saved_active_map") or "")
        binding = next(
            (item for item in self._floor_bindings.values() if item.map_id == active_map),
            None,
        )
        if binding is None:
            raise RuntimeError(f"active map has no Open-RMF floor binding: {active_map or 'none'}")

        dds_snapshot = self._transport.request(
            "GET",
            "/api/v1/navigation/dds_snapshot",
            headers=headers,
        )
        nav_endpoint = dds_snapshot.get("nav_endpoint")
        if not isinstance(nav_endpoint, Mapping):
            raise RuntimeError("Gateway navigation snapshot has no native endpoint state")
        now = float(self._clock())
        native_stamp_s = float(nav_endpoint.get("stamp_s") or 0.0)
        if (
            not math.isfinite(native_stamp_s)
            or native_stamp_s <= 0.0
            or abs(now - native_stamp_s) > self._config.max_native_status_age_s
        ):
            raise RuntimeError("Gateway native navigation status is stale")
        path_snapshot = dds_snapshot.get("global_path") or dds_snapshot.get("local_path") or {}
        robot = path_snapshot.get("robot") if isinstance(path_snapshot, dict) else None
        if not isinstance(robot, dict):
            raise RuntimeError("Gateway navigation snapshot has no map-frame robot pose")
        if str(robot.get("frame_id") or "") != binding.frame_id:
            raise RuntimeError("Gateway robot pose frame does not match the floor binding")
        try:
            robot_pose = (
                float(robot["x"]),
                float(robot["y"]),
                float(robot.get("z", 0.0)),
                float(robot.get("yaw", 0.0)),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("Gateway robot pose is invalid") from exc
        if not all(math.isfinite(value) for value in robot_pose):
            raise RuntimeError("Gateway robot pose contains non-finite values")
        pose_stamp_s = float(robot.get("ts") or 0.0)
        if (
            not math.isfinite(pose_stamp_s)
            or pose_stamp_s <= 0.0
            or abs(now - pose_stamp_s) > self._config.max_pose_age_s
        ):
            raise RuntimeError("Gateway robot pose is stale")

        navigation = self._transport.request(
            "GET",
            "/api/v1/navigation/status",
            headers=headers,
        )
        confirmed_session = self._transport.request(
            "GET",
            "/api/v1/session",
            headers=headers,
        )
        confirmed_map = str(confirmed_session.get("active_map") or confirmed_session.get("saved_active_map") or "")
        if confirmed_map != active_map:
            raise RuntimeError("Gateway active map changed during state snapshot")
        command_boundary = nav_endpoint.get("command_boundary")
        if not isinstance(command_boundary, Mapping):
            command_boundary = {}
        control_authority = nav_endpoint.get("control_authority")
        if not isinstance(control_authority, Mapping):
            control_authority = {}
        last_plan = nav_endpoint.get("last_plan")
        if not isinstance(last_plan, Mapping):
            last_plan = {}
        last_local = nav_endpoint.get("last_local")
        if not isinstance(last_local, Mapping):
            last_local = {}
        plan_goal = last_plan.get("goal")
        native_plan_goal = None
        if isinstance(plan_goal, Mapping):
            try:
                native_plan_goal = (
                    float(plan_goal.get("x", 0.0)),
                    float(plan_goal.get("y", 0.0)),
                    float(plan_goal.get("z", 0.0)),
                )
            except (TypeError, ValueError) as exc:
                raise RuntimeError("Gateway native plan goal is invalid") from exc
            if not all(math.isfinite(value) for value in native_plan_goal):
                raise RuntimeError("Gateway native plan goal contains non-finite values")
        return RobotSnapshot(
            building_id=binding.building_id,
            floor_id=binding.floor_id,
            map_id=binding.map_id,
            x=robot_pose[0],
            y=robot_pose[1],
            z=robot_pose[2],
            yaw=robot_pose[3],
            battery_soc=max(0.0, min(1.0, float(self._config.battery_soc_default))),
            navigation_state=str(navigation.get("state") or "UNKNOWN"),
            stamp_s=pose_stamp_s,
            native_request_id=str(command_boundary.get("last_request_id") or ""),
            native_command_kind=str(command_boundary.get("last_kind") or ""),
            native_command_accepted=command_boundary.get("last_accepted") is True,
            native_status_stamp_s=native_stamp_s,
            native_control_mode=str(nav_endpoint.get("control_mode") or ""),
            native_estop_latched=bool(control_authority.get("estop_latched", False)),
            native_operator_takeover_latched=bool(control_authority.get("operator_takeover_latched", False)),
            native_plan_seen=last_plan.get("seen") is True,
            native_plan_accepted=last_plan.get("accepted") is True,
            native_plan_reason=str(last_plan.get("reason") or ""),
            native_plan_goal=native_plan_goal,
            native_local_active=bool(last_local.get("active", False)),
            native_goal_reached=last_local.get("goal_reached") is True,
        )
