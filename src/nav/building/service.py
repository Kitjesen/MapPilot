"""Fail-closed ingress for map-bound building navigation goals."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from nav.building.model import BuildingMissionRequest, PoseTarget
from runtime import In, Module, Out, rpc
from runtime.registry import register

BUILDING_GOAL_SCHEMA_VERSION = "lingtu.building_goal.v1"
BUILDING_STATUS_SCHEMA_VERSION = "lingtu.building_status.v1"
_TRAVEL_MODES = frozenset({"any", "stairs", "elevator"})
_CONNECTOR_EDGE_TYPES = {
    "stairs": frozenset({"stairs", "stair", "stairway"}),
    "elevator": frozenset({"elevator", "lift"}),
}


@dataclass(frozen=True, slots=True)
class _ValidatedGoal:
    request_id: str
    source: str
    fleet_name: str
    robot_name: str
    building_id: str
    floor_id: str
    map_id: str
    place_id: str
    frame_id: str
    x: float
    y: float
    z: float
    yaw: float
    map_version: int
    version_id: str
    map_pcd_sha256: str
    travel_mode: str
    connector_id: str


class _AdmissionError(ValueError):
    def __init__(self, reason: str, message: str) -> None:
        super().__init__(message)
        self.reason = reason
        self.message = message


@register("nav_service", "building", description="Map-bound building navigation ingress")
class BuildingService(Module, layer=5):
    """Validate floor-aware goals and delegate them to building orchestration.

    This module is admission control only. It never switches maps, drives a
    connector, publishes velocity, or starts a worker thread.
    """

    runtime_id = "nav.building"

    mission_request: In[str]
    cancel_request: In[str]
    building_status: Out[dict]

    def __init__(
        self,
        maps_module: str = "maps.service",
        mission_module: str = "nav.building.mission",
        mission_port: object | None = None,
        **config: Any,
    ) -> None:
        super().__init__(**config)
        self._maps_module = str(maps_module or "maps.service").strip()
        self._mission_module = str(mission_module or "nav.building.mission").strip()
        self._maps: object | None = None
        self._mission: object | None = mission_port

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        """Resolve only the two capabilities needed by the ingress."""

        maps_service = modules.get(self._maps_module)
        if maps_service is not None:
            api = getattr(maps_service, "api", None)
            self._maps = api if api is not None else maps_service
        if self._mission is None:
            self._mission = modules.get(self._mission_module)

    def setup(self) -> None:
        """Subscribe the JSON mission and cancellation ingress ports."""

        self.mission_request.subscribe(self._on_mission_request)
        self.cancel_request.subscribe(self._on_cancel_request)

    @rpc
    def submit(self, command: Mapping[str, Any]) -> dict[str, Any]:
        """Admit one version-bound ``building_navigate`` command."""

        request_id = self._request_id(command)
        try:
            goal = self._validate_command(command)
            active_map_id = self._validate_native_binding(goal)
            selected_connector_id = self._validate_route(
                active_map_id=active_map_id,
                target_map_id=goal.map_id,
                travel_mode=goal.travel_mode,
                connector_id=goal.connector_id,
            )
            mission = self._require_mission()
            request = BuildingMissionRequest(
                request_id=goal.request_id,
                source=goal.source,
                fleet_name=goal.fleet_name,
                robot_name=goal.robot_name,
                building_id=goal.building_id,
                floor_id=goal.floor_id,
                map_id=goal.map_id,
                target=PoseTarget(
                    frame_id=goal.frame_id,
                    x=goal.x,
                    y=goal.y,
                    z=goal.z,
                    yaw=goal.yaw,
                ),
                travel_mode=goal.travel_mode,
                connector_id=selected_connector_id,
                place_id=goal.place_id,
                map_version=goal.map_version,
                version_id=goal.version_id,
                map_pcd_sha256=goal.map_pcd_sha256,
            )
            operation = getattr(mission, "submit", None)
            if not callable(operation):
                raise _AdmissionError(
                    "mission_unavailable",
                    "building mission capability does not implement submit",
                )
            try:
                result = operation(request)
            except Exception as exc:
                raise _AdmissionError(
                    "mission_submit_failed",
                    f"building mission rejected the request: {exc}",
                ) from exc
            accepted, reason = self._mission_result(result)
            if not accepted:
                return self._publish_status(
                    action="building_navigate",
                    request_id=goal.request_id,
                    accepted=False,
                    reason=reason or "building_mission_rejected",
                    message=reason or "building mission rejected the request",
                    map_id=goal.map_id,
                    place_id=goal.place_id,
                    travel_mode=goal.travel_mode,
                )
            return self._publish_status(
                action="building_navigate",
                request_id=goal.request_id,
                accepted=True,
                reason=reason or "building_mission_accepted",
                message=reason or "building mission accepted",
                map_id=goal.map_id,
                place_id=goal.place_id,
                travel_mode=goal.travel_mode,
            )
        except _AdmissionError as exc:
            return self._publish_status(
                action="building_navigate",
                request_id=request_id,
                accepted=False,
                reason=exc.reason,
                message=exc.message,
            )
        except Exception as exc:
            return self._publish_status(
                action="building_navigate",
                request_id=request_id,
                accepted=False,
                reason="building_admission_error",
                message=f"building goal admission failed: {exc}",
            )

    @rpc
    def cancel(
        self,
        reason: str = "cancel",
        request_id: str | None = None,
    ) -> dict[str, Any]:
        """Delegate cancellation to the mission owner without issuing motion."""

        resolved_request_id = str(request_id or "").strip()
        clean_reason = str(reason or "cancel").strip() or "cancel"
        try:
            mission = self._require_mission()
            operation = getattr(mission, "cancel", None)
            if not callable(operation):
                raise _AdmissionError(
                    "mission_cancel_unavailable",
                    "building mission capability does not implement cancel",
                )
            try:
                result = operation(clean_reason)
            except Exception as exc:
                raise _AdmissionError(
                    "mission_cancel_failed",
                    f"building mission cancellation failed: {exc}",
                ) from exc
            accepted, result_reason = self._optional_mission_result(result)
            return self._publish_status(
                action="building_cancel",
                request_id=resolved_request_id,
                accepted=accepted,
                reason=(
                    result_reason or ("building_mission_cancelled" if accepted else "building_mission_cancel_rejected")
                ),
                message=(
                    result_reason
                    or ("building mission cancelled" if accepted else "building mission cancellation rejected")
                ),
            )
        except _AdmissionError as exc:
            return self._publish_status(
                action="building_cancel",
                request_id=resolved_request_id,
                accepted=False,
                reason=exc.reason,
                message=exc.message,
            )

    def _on_mission_request(self, raw: str) -> None:
        if isinstance(raw, Mapping):
            self.submit(raw)
            return
        try:
            command = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            self._publish_status(
                action="building_navigate",
                request_id="",
                accepted=False,
                reason="invalid_command",
                message="building mission request must be a JSON object",
            )
            return
        if not isinstance(command, Mapping):
            self._publish_status(
                action="building_navigate",
                request_id="",
                accepted=False,
                reason="invalid_command",
                message="building mission request must be an object",
            )
            return
        self.submit(command)

    def _on_cancel_request(self, raw: str) -> None:
        if isinstance(raw, Mapping):
            payload = raw
        else:
            try:
                decoded = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                decoded = None
            payload = decoded if isinstance(decoded, Mapping) else None
        if payload is None:
            self.cancel(str(raw or "cancel"))
            return
        self.cancel(
            str(payload.get("reason") or "cancel"),
            request_id=str(payload.get("request_id") or ""),
        )

    def _validate_command(self, command: Mapping[str, Any]) -> _ValidatedGoal:
        if not isinstance(command, Mapping):
            raise _AdmissionError("invalid_command", "building goal command must be an object")
        action = str(command.get("action") or "").strip()
        if action != "building_navigate":
            raise _AdmissionError("invalid_action", "action must be building_navigate")
        schema_version = str(command.get("schema_version") or "").strip()
        if schema_version != BUILDING_GOAL_SCHEMA_VERSION:
            raise _AdmissionError(
                "invalid_schema_version",
                f"schema_version must be {BUILDING_GOAL_SCHEMA_VERSION}",
            )
        request_id = self._required_text(command, "request_id", owner="command")
        target = command.get("target")
        if not isinstance(target, Mapping):
            raise _AdmissionError("invalid_target", "target must be an object")

        travel_mode = str(command.get("travel_mode") or "").strip()
        if travel_mode not in _TRAVEL_MODES:
            raise _AdmissionError(
                "invalid_travel_mode",
                "travel_mode must be any, stairs, or elevator",
            )
        frame_id = self._required_text(target, "frame_id", owner="target")
        if frame_id != "map":
            raise _AdmissionError("unsupported_frame", "target frame_id must be map")
        map_version = target.get("map_version")
        if isinstance(map_version, bool) or not isinstance(map_version, int) or map_version <= 0:
            raise _AdmissionError(
                "invalid_map_version",
                "target map_version must be a positive integer",
            )
        connector_id = self._optional_text(command, "connector_id", owner="command")
        return _ValidatedGoal(
            request_id=request_id,
            source=self._optional_text(command, "source", owner="command") or "semantic_navigation",
            fleet_name=self._optional_text(command, "fleet_name", owner="command"),
            robot_name=self._optional_text(command, "robot_name", owner="command"),
            building_id=self._required_text(target, "building_id", owner="target"),
            floor_id=self._required_text(target, "floor_id", owner="target"),
            map_id=self._required_text(target, "map_id", owner="target"),
            place_id=self._required_text(target, "place_id", owner="target"),
            frame_id=frame_id,
            x=self._finite_number(target, "x"),
            y=self._finite_number(target, "y"),
            z=self._finite_number(target, "z"),
            yaw=self._finite_number(target, "yaw"),
            map_version=map_version,
            version_id=self._required_text(target, "version_id", owner="target"),
            map_pcd_sha256=self._required_text(target, "map_pcd_sha256", owner="target"),
            travel_mode=travel_mode,
            connector_id=connector_id,
        )

    def _validate_native_binding(self, goal: _ValidatedGoal) -> str:
        maps = self._require_maps()
        active_response = self._maps_call(maps, "get_active_map")
        if active_response.get("success") is not True:
            raise _AdmissionError(
                "active_map_unavailable",
                str(active_response.get("message") or "native active map is unavailable"),
            )
        active_map_id = str(active_response.get("active") or active_response.get("map_id") or "").strip()
        if not active_map_id:
            raise _AdmissionError("active_map_unavailable", "native active map is empty")

        record_response = self._maps_call(maps, "get_record", goal.map_id)
        if record_response.get("success") is not True:
            raise _AdmissionError(
                "target_map_unavailable",
                str(record_response.get("message") or "target map record is unavailable"),
            )
        record = record_response.get("record")
        if not isinstance(record, Mapping):
            record = record_response

        points_response = self._maps_call(maps, "get_map_points", goal.map_id, max_points=1)
        if points_response.get("success") is not True:
            raise _AdmissionError(
                "target_map_binding_unavailable",
                str(points_response.get("message") or "target map point binding is unavailable"),
            )

        record_map_id = str(record.get("map_id") or "").strip()
        points_map_id = str(points_response.get("map_id") or "").strip()
        current_version = self._positive_int(record.get("version"))
        current_version_id = str(points_response.get("version_id") or record.get("version_id") or "").strip()
        current_hash = str(
            points_response.get("map_pcd_sha256") or record.get("map_pcd_sha256") or self._pointcloud_hash(record) or ""
        ).strip()
        scope = record.get("scope") if isinstance(record.get("scope"), Mapping) else {}
        current_frame = str(
            points_response.get("frame_id") or record.get("frame_id") or scope.get("frame_id") or ""
        ).strip()
        mismatches: list[str] = []
        if record_map_id != goal.map_id:
            mismatches.append("record.map_id")
        if points_map_id != goal.map_id:
            mismatches.append("points.map_id")
        if current_version != goal.map_version:
            mismatches.append("map_version")
        if current_version_id != goal.version_id:
            mismatches.append("version_id")
        if current_hash != goal.map_pcd_sha256:
            mismatches.append("map_pcd_sha256")
        if current_frame != goal.frame_id:
            mismatches.append("frame_id")
        if mismatches:
            raise _AdmissionError(
                "target_map_binding_mismatch",
                "target map binding is stale or incomplete: " + ", ".join(mismatches),
            )
        return active_map_id

    def _validate_route(
        self,
        *,
        active_map_id: str,
        target_map_id: str,
        travel_mode: str,
        connector_id: str,
    ) -> str:
        explicit_connector = travel_mode != "any" or bool(connector_id)
        if active_map_id == target_map_id:
            if explicit_connector:
                raise _AdmissionError(
                    "connector_transition_not_required",
                    "an explicit connector cannot be used for a same-map goal",
                )
            return ""

        maps = self._require_maps()
        route_response = self._shortest_route(maps, active_map_id, target_map_id)
        transitions = route_response.get("transitions")
        if (
            route_response.get("success") is not True
            or route_response.get("found") is not True
            or not isinstance(transitions, Sequence)
            or isinstance(transitions, (str, bytes))
        ):
            raise _AdmissionError("building_route_unavailable", "no executable map route was found")
        if len(transitions) != 1:
            raise _AdmissionError(
                "building_route_not_direct",
                "building navigation currently requires exactly one map transition",
            )
        transition = transitions[0]
        if not isinstance(transition, Mapping):
            raise _AdmissionError("building_route_invalid", "map transition is not an object")
        route_from = str(transition.get("from_map_id") or transition.get("from") or "").strip()
        route_to = str(transition.get("to_map_id") or transition.get("to") or "").strip()
        if route_from != active_map_id or route_to != target_map_id:
            raise _AdmissionError(
                "building_route_invalid",
                "map transition does not match the active and target maps",
            )
        if travel_mode == "any" and not connector_id:
            return ""

        graph_response = self._maps_call(maps, "list_map_graph")
        edges = graph_response.get("edges")
        if (
            graph_response.get("success") is not True
            or not isinstance(edges, Sequence)
            or isinstance(edges, (str, bytes))
        ):
            raise _AdmissionError(
                "map_graph_unavailable",
                "native map graph is unavailable",
            )
        allowed_types = _CONNECTOR_EDGE_TYPES.get(travel_mode)
        transition_fields = ("connector_id", "lift_id", "id", "edge_id")
        transition_ids = self._connector_ids(transition, fields=transition_fields)
        eligible_edges: list[tuple[set[str], set[str], str]] = []
        for edge in edges:
            if not isinstance(edge, Mapping):
                continue
            edge_from = str(edge.get("from") or edge.get("from_map_id") or "").strip()
            edge_to = str(edge.get("to") or edge.get("to_map_id") or "").strip()
            edge_type = str(edge.get("type") or edge.get("edge_type") or "").strip().lower()
            direct = edge_from == active_map_id and edge_to == target_map_id
            reverse_bidirectional = (
                edge_from == target_map_id and edge_to == active_map_id and edge.get("bidirectional") is True
            )
            if not (direct or reverse_bidirectional):
                continue
            if allowed_types is not None and edge_type not in allowed_types:
                continue
            correlation_ids = self._connector_ids(
                edge,
                fields=("connector_id", "lift_id", "id", "edge_id"),
            )
            connector_ids = self._connector_ids(
                edge,
                fields=("connector_id", "lift_id"),
            )
            eligible_edges.append(
                (
                    correlation_ids,
                    connector_ids,
                    self._preferred_connector_id(
                        edge,
                        fields=("connector_id", "lift_id"),
                    ),
                )
            )

        correlated_edges = [
            candidate for candidate in eligible_edges if transition_ids and candidate[0] & transition_ids
        ]
        candidate_pool = correlated_edges or eligible_edges
        if connector_id:
            selected = [
                connector_id
                for _correlation_ids, connector_ids, _preferred in candidate_pool
                if connector_id in connector_ids
            ]
        else:
            selected = [
                preferred
                for _correlation_ids, _connector_ids, preferred in candidate_pool
                if preferred
            ]
        if len(selected) == 1:
            return selected[0]
        raise _AdmissionError(
            "connector_route_unavailable",
            f"no unique direct {travel_mode} connector matches {connector_id!r}",
        )

    @staticmethod
    def _connector_ids(
        value: Mapping[str, Any],
        *,
        fields: tuple[str, ...],
    ) -> set[str]:
        return {text for field in fields if isinstance((raw := value.get(field)), str) and (text := raw.strip())}

    @staticmethod
    def _preferred_connector_id(
        value: Mapping[str, Any],
        *,
        fields: tuple[str, ...],
    ) -> str:
        for field in fields:
            raw = value.get(field)
            if isinstance(raw, str) and (text := raw.strip()):
                return text
        return ""

    def _require_maps(self) -> object:
        if self._maps is None:
            raise _AdmissionError("maps_unavailable", "native maps capability is unavailable")
        return self._maps

    def _require_mission(self) -> object:
        if self._mission is None:
            raise _AdmissionError(
                "mission_unavailable",
                "building mission capability is unavailable",
            )
        return self._mission

    @staticmethod
    def _maps_call(maps: object, method: str, *args: Any, **kwargs: Any) -> Mapping[str, Any]:
        operation = getattr(maps, method, None)
        if not callable(operation):
            raise _AdmissionError(
                "maps_unavailable",
                f"native maps capability does not implement {method}",
            )
        try:
            response = operation(*args, **kwargs)
        except Exception as exc:
            raise _AdmissionError(
                "maps_query_failed",
                f"native maps {method} failed: {exc}",
            ) from exc
        if not isinstance(response, Mapping):
            raise _AdmissionError(
                "maps_response_invalid",
                f"native maps {method} returned a non-object response",
            )
        return response

    def _shortest_route(
        self,
        maps: object,
        active_map_id: str,
        target_map_id: str,
    ) -> Mapping[str, Any]:
        operation = getattr(maps, "shortest_route", None)
        if not callable(operation):
            raise _AdmissionError(
                "building_route_unavailable",
                "native maps capability does not implement shortest_route",
            )
        command = {"from": active_map_id, "to": target_map_id}
        try:
            response = operation(command)
        except TypeError:
            try:
                response = operation(active_map_id, target_map_id)
            except Exception as exc:
                raise _AdmissionError(
                    "building_route_unavailable",
                    f"native map route query failed: {exc}",
                ) from exc
        except Exception as exc:
            raise _AdmissionError(
                "building_route_unavailable",
                f"native map route query failed: {exc}",
            ) from exc
        if not isinstance(response, Mapping):
            raise _AdmissionError(
                "building_route_invalid",
                "native map route response must be an object",
            )
        return response

    def _publish_status(
        self,
        *,
        action: str,
        request_id: str,
        accepted: bool,
        reason: str,
        message: str,
        **extra: Any,
    ) -> dict[str, Any]:
        status = {
            "schema_version": BUILDING_STATUS_SCHEMA_VERSION,
            "action": action,
            "request_id": request_id,
            "success": accepted,
            "accepted": accepted,
            "state": "accepted" if accepted else "rejected",
            "reason": reason,
            "message": message,
        }
        status.update(extra)
        self.building_status.publish(status)
        return status

    @staticmethod
    def _mission_result(result: Any) -> tuple[bool, str]:
        if not isinstance(result, tuple) or len(result) != 2 or not isinstance(result[0], bool):
            raise _AdmissionError(
                "mission_response_invalid",
                "building mission submit must return (accepted, reason)",
            )
        return result[0], str(result[1] or "").strip()

    @staticmethod
    def _optional_mission_result(result: Any) -> tuple[bool, str]:
        if isinstance(result, tuple):
            return BuildingService._mission_result(result)
        if isinstance(result, Mapping):
            accepted = result.get("accepted", result.get("success"))
            if isinstance(accepted, bool):
                return accepted, str(result.get("reason") or result.get("message") or "").strip()
        return True, str(getattr(result, "reason", "") or "").strip()

    @staticmethod
    def _request_id(command: Any) -> str:
        if not isinstance(command, Mapping):
            return ""
        value = command.get("request_id")
        return value.strip() if isinstance(value, str) else ""

    @staticmethod
    def _required_text(value: Mapping[str, Any], key: str, *, owner: str) -> str:
        text = BuildingService._optional_text(value, key, owner=owner)
        if not text:
            raise _AdmissionError(
                f"missing_{key}",
                f"{owner} {key} is required",
            )
        return text

    @staticmethod
    def _optional_text(value: Mapping[str, Any], key: str, *, owner: str) -> str:
        raw = value.get(key)
        if raw is None:
            return ""
        if not isinstance(raw, str):
            raise _AdmissionError(
                f"invalid_{key}",
                f"{owner} {key} must be a string",
            )
        text = raw.strip()
        if "\x00" in text or len(text) > 256:
            raise _AdmissionError(
                f"invalid_{key}",
                f"{owner} {key} is invalid",
            )
        return text

    @staticmethod
    def _finite_number(value: Mapping[str, Any], key: str) -> float:
        raw = value.get(key)
        if isinstance(raw, bool):
            raise _AdmissionError(f"invalid_{key}", f"target {key} must be a finite number")
        try:
            number = float(raw)
        except (TypeError, ValueError) as exc:
            raise _AdmissionError(
                f"invalid_{key}",
                f"target {key} must be a finite number",
            ) from exc
        if not math.isfinite(number):
            raise _AdmissionError(f"invalid_{key}", f"target {key} must be a finite number")
        return number

    @staticmethod
    def _positive_int(value: Any) -> int | None:
        if isinstance(value, bool):
            return None
        try:
            result = int(value)
        except (TypeError, ValueError):
            return None
        return result if result > 0 else None

    @staticmethod
    def _pointcloud_hash(record: Mapping[str, Any]) -> str:
        artifacts = record.get("artifacts")
        if not isinstance(artifacts, Sequence) or isinstance(artifacts, (str, bytes)):
            return ""
        for artifact in artifacts:
            if not isinstance(artifact, Mapping):
                continue
            artifact_type = str(artifact.get("type") or "").upper()
            name = str(artifact.get("name") or "").lower()
            if artifact_type == "POINTCLOUD" or name in {"map_pcd", "source_pointcloud"}:
                return str(artifact.get("hash") or artifact.get("sha256") or "")
        return ""


__all__ = [
    "BUILDING_GOAL_SCHEMA_VERSION",
    "BUILDING_STATUS_SCHEMA_VERSION",
    "BuildingService",
]
