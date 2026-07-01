"""Runtime dataflow inspector for Module-first Gateway clients."""

from __future__ import annotations

import os
import hashlib
import json
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any
from urllib.parse import quote

from core.runtime_interface import ARTIFACT_FORMATS
from core.runtime_interface import TOPICS
from core.runtime_interface import runtime_contract_manifest
from core.runtime_interface import runtime_data_flow_topics
from gateway.services.map_paths import active_map_link
from gateway.services.map_paths import active_map_name
from gateway.services.map_paths import map_dir_for
from gateway.services.map_paths import nav_map_root
from gateway.services.runtime_status import _runtime_boundary_status


RUNTIME_DATAFLOW_SCHEMA_VERSION = 1
LIVE_MODULE_SAMPLE_STALE_MS = 2000.0

_PRODUCT_OBSERVABILITY_TOPICS = (
    TOPICS.localization_health,
    TOPICS.mission_status,
    TOPICS.traversable_frontiers,
    TOPICS.frontier_candidate,
)

_PRODUCT_OBSERVABILITY_STAGES: tuple[dict[str, Any], ...] = (
    {
        "name": "traversable_frontier_preview",
        "owner": "TraversableFrontierModule",
        "frame_role": "map",
        "map_dependency": "live_occupancy_grid_and_traversability_layers",
        "inputs": (
            TOPICS.odometry,
            TOPICS.exploration_grid,
            "module:TraversableFrontierModule.fused_cost",
            "module:TraversableFrontierModule.slope_grid",
            "module:TraversableFrontierModule.esdf_field",
            "module:TraversableFrontierModule.elevation_map",
        ),
        "outputs": (TOPICS.traversable_frontiers, TOPICS.frontier_candidate),
    },
)


_GATEWAY_TOPIC_CHANNELS: dict[str, list[dict[str, Any]]] = {
    TOPICS.odometry: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "odometry"},
        {"transport": "gateway_rest", "path": "/api/v1/state", "field": "odometry"},
        {
            "transport": "gateway_rest",
            "path": "/api/v1/localization/status",
            "field": "has_odometry",
        },
    ],
    TOPICS.map_cloud: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "map_cloud"},
        {"transport": "gateway_ws", "path": "/ws/cloud", "payload": "binary_point_cloud"},
        {"transport": "gateway_rest", "path": "/api/v1/map/points", "payload": "point_sample"},
    ],
    TOPICS.saved_map_cloud: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "saved_map"},
        {
            "transport": "gateway_rest",
            "path": "/api/v1/maps/{name}/points",
            "payload": "point_sample",
        },
    ],
    TOPICS.localization_health: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "slam_diag"},
        {"transport": "gateway_rest", "path": "/api/v1/localization/status"},
    ],
    TOPICS.localization_quality: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "slam_diag"},
        {"transport": "gateway_rest", "path": "/api/v1/localization/status"},
    ],
    TOPICS.global_path: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "global_path"},
        {"transport": "gateway_rest", "path": "/api/v1/path", "payload": "latest_global_path"},
    ],
    TOPICS.local_path: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "local_path"},
    ],
    TOPICS.cmd_vel: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "command_ack"},
        {"transport": "gateway_rest", "path": "/api/v1/navigation/status", "field": "control"},
    ],
    TOPICS.goal_pose: [
        {"transport": "gateway_rest", "path": "/api/v1/navigation/status", "field": "target"},
    ],
    TOPICS.semantic_scene_graph: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "scene_graph"},
        {"transport": "gateway_rest", "path": "/api/v1/scene_graph"},
    ],
    TOPICS.safety_state: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "safety"},
        {"transport": "gateway_rest", "path": "/api/v1/state", "field": "safety"},
    ],
    TOPICS.mission_status: [
        {"transport": "gateway_sse", "path": "/api/v1/events", "event_type": "mission"},
        {"transport": "gateway_rest", "path": "/api/v1/navigation/status"},
    ],
    TOPICS.traversable_frontiers: [
        {
            "transport": "gateway_sse",
            "path": "/api/v1/events",
            "event_type": "traversable_frontiers",
        },
        {
            "transport": "gateway_rest",
            "path": "/api/v1/runtime/dataflow/topic",
            "field": "inspection.latest_payload",
        },
    ],
    TOPICS.frontier_candidate: [
        {
            "transport": "gateway_sse",
            "path": "/api/v1/events",
            "event_type": "frontier_candidate",
        },
        {
            "transport": "gateway_rest",
            "path": "/api/v1/runtime/dataflow/topic",
            "field": "inspection.latest_payload",
        },
    ],
    TOPICS.planner_status: [
        {"transport": "gateway_rest", "path": "/api/v1/navigation/status", "field": "diagnostics"},
    ],
}


_TOPIC_PORT_HINTS: dict[str, tuple[str, ...]] = {
    TOPICS.lidar_scan: ("lidar_scan", "scan"),
    TOPICS.imu: ("imu",),
    TOPICS.odometry: ("odometry", "odom"),
    TOPICS.registered_cloud: ("registered_cloud", "registered_scan"),
    TOPICS.map_cloud: ("map_cloud",),
    TOPICS.cumulative_map_cloud: ("cumulative_map_cloud",),
    TOPICS.saved_map_cloud: ("saved_map", "saved_map_cloud"),
    TOPICS.localization_quality: ("localization_quality", "icp_quality"),
    TOPICS.localization_health: ("localization_health", "localization_status"),
    TOPICS.global_path: ("global_path", "path"),
    TOPICS.local_path: ("local_path",),
    TOPICS.cmd_vel: ("cmd_vel", "driver_cmd_vel"),
    TOPICS.goal_pose: ("goal_pose",),
    TOPICS.cancel: ("cancel",),
    TOPICS.stop: ("stop_cmd", "stop"),
    TOPICS.safety_state: ("safety_state",),
    TOPICS.mission_status: ("mission_status",),
    TOPICS.traversable_frontiers: ("traversable_frontiers",),
    TOPICS.frontier_candidate: ("frontier_candidate",),
    TOPICS.semantic_scene_graph: ("scene_graph",),
    TOPICS.semantic_instruction: ("instruction",),
    TOPICS.added_obstacles: ("added_obstacles",),
    TOPICS.check_obstacle: ("check_obstacle",),
    TOPICS.planner_status: ("planner_status",),
}


_COMMAND_INTERFACES: tuple[dict[str, Any], ...] = (
    {
        "name": "navigation_plan_preview",
        "method": "POST",
        "path": "/api/v1/navigation/plan",
        "publishes": [],
        "read_only": True,
        "guard": "non_motion_preview",
    },
    {
        "name": "goal",
        "method": "POST",
        "path": "/api/v1/goal",
        "publishes": [TOPICS.goal_pose],
        "read_only": False,
        "guard": "planning_and_motion_readiness",
    },
    {
        "name": "navigate_click",
        "method": "POST",
        "path": "/api/v1/navigate/click",
        "publishes": [TOPICS.goal_pose],
        "read_only": False,
        "guard": "planning_and_motion_readiness",
    },
    {
        "name": "direct_cmd_vel",
        "method": "POST",
        "path": "/api/v1/cmd_vel",
        "publishes": [TOPICS.cmd_vel],
        "read_only": False,
        "guard": "motion_guard_and_lease",
    },
    {
        "name": "stop",
        "method": "POST",
        "path": "/api/v1/stop",
        "publishes": [TOPICS.stop, TOPICS.cmd_vel],
        "read_only": False,
        "guard": "always_available_safety_stop",
    },
    {
        "name": "navigation_cancel",
        "method": "POST",
        "path": "/api/v1/navigation/cancel",
        "publishes": [TOPICS.cancel],
        "read_only": False,
        "guard": "control_command_journal",
    },
    {
        "name": "instruction",
        "method": "POST",
        "path": "/api/v1/instruction",
        "publishes": [TOPICS.semantic_instruction],
        "read_only": False,
        "guard": "motion_guard_and_lease",
    },
)


_ARTIFACT_GATEWAY_CHANNELS: dict[str, list[dict[str, Any]]] = {
    "tomogram": [
        {
            "transport": "gateway_rest",
            "path": "/api/v1/diagnostics/field-check",
            "field": "map.saved_map_artifact_gate",
        },
        {
            "transport": "gateway_rest",
            "path": "/api/v1/inspection/acceptance",
            "field": "evidence.field_check.map.saved_map_artifact_gate",
        },
        {
            "transport": "gateway_cli",
            "command": "python lingtu.py saved-map-artifact-gate --require-tomogram",
        },
    ],
}

_ARTIFACT_TOKEN_REQUIRED_FORMATS: dict[str, tuple[str, ...]] = {
    "tomogram": ("map_pcd", "tomogram"),
}


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _json_payload(value: Any) -> Any:
    try:
        return json.loads(json.dumps(value, ensure_ascii=False, default=str))
    except Exception:
        return {"raw": str(value)}


def _latest_payload_for_topic(gw: Any, topic: str) -> Any | None:
    attr_by_topic = {
        TOPICS.traversable_frontiers: "_last_traversable_frontiers",
        TOPICS.frontier_candidate: "_last_frontier_candidate",
    }
    attr = attr_by_topic.get(topic)
    if not attr:
        return None
    lock = getattr(gw, "_state_lock", None)
    if lock is None:
        value = getattr(gw, attr, None)
    else:
        with lock:
            value = getattr(gw, attr, None)
    return None if value is None else _json_payload(value)


def _sha256_file(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as fh:
            for chunk in iter(lambda: fh.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def _active_map_artifact_dir() -> tuple[Path | None, str | None, Path]:
    root = nav_map_root()
    active_name = active_map_name(root)
    if active_name:
        candidate = map_dir_for(active_name, root)
        if candidate.is_dir():
            return candidate, active_name, root

    active_dir = active_map_link(root)
    if active_dir.is_dir():
        return active_dir, active_name or "active", root
    return None, active_name, root


def _artifact_metadata_path(map_dir: Path) -> Path:
    return map_dir / str(ARTIFACT_FORMATS["metadata"].path)


def _artifact_file_path(map_dir: Path, artifact_name: str, entry: Mapping[str, Any]) -> Path:
    default_path = ARTIFACT_FORMATS[artifact_name].path
    raw_path = str(entry.get("path") or default_path)
    path = Path(raw_path)
    return path if path.is_absolute() else map_dir / path


def _missing_required_fields(
    value: Mapping[str, Any],
    required_fields: tuple[str, ...],
) -> list[str]:
    return [field for field in required_fields if value.get(field) in (None, "")]


def _artifact_gate(token: str) -> dict[str, Any]:
    artifact_name = token.split(":", 1)[1]
    required_formats = _ARTIFACT_TOKEN_REQUIRED_FORMATS.get(
        artifact_name,
        (artifact_name,),
    )
    map_dir, active_name, map_root = _active_map_artifact_dir()
    gate: dict[str, Any] = {
        "schema_version": "lingtu.runtime_dataflow_artifact_gate.v1",
        "token": token,
        "artifact": artifact_name,
        "ok": False,
        "endpoint_topic_required": False,
        "ros2_topic_required": False,
        "transport": "saved_map_artifact",
        "map_root": str(map_root),
        "map_dir": str(map_dir) if map_dir is not None else None,
        "active_map": active_name,
        "checked_required_artifacts": list(required_formats),
        "metadata": {},
        "artifacts": {},
        "blockers": [],
    }
    blockers: list[str] = gate["blockers"]

    if artifact_name not in ARTIFACT_FORMATS:
        blockers.append(f"{artifact_name} artifact format not registered")
        return gate

    if map_dir is None:
        blockers.append("active map artifact directory missing")
        return gate

    metadata_path = _artifact_metadata_path(map_dir)
    gate["metadata"] = {
        "path": str(metadata_path),
        "exists": metadata_path.is_file(),
        "ok": False,
        "missing_required_fields": [],
    }
    if not metadata_path.is_file():
        blockers.append("metadata.json missing")
        return gate

    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        blockers.append(f"metadata.json unreadable: {exc}")
        return gate

    if not isinstance(metadata, Mapping):
        blockers.append("metadata.json must contain an object")
        return gate

    metadata_required = ARTIFACT_FORMATS["metadata"].required_fields
    missing_metadata_fields = _missing_required_fields(metadata, metadata_required)
    gate["metadata"].update(
        {
            "ok": not missing_metadata_fields,
            "source_profile": metadata.get("source_profile"),
            "data_source": metadata.get("data_source"),
            "slam_source": metadata.get("slam_source"),
            "localization_source": metadata.get("localization_source"),
            "mapping_source": metadata.get("mapping_source"),
            "frame_id": metadata.get("frame_id"),
            "missing_required_fields": missing_metadata_fields,
        }
    )
    if missing_metadata_fields:
        blockers.append(
            "metadata missing required fields: " + ", ".join(missing_metadata_fields)
        )

    artifact_metadata = _mapping(metadata.get("artifacts"))
    actual_shas: dict[str, str | None] = {}
    for required_name in required_formats:
        if required_name not in ARTIFACT_FORMATS:
            blockers.append(f"{required_name} artifact format not registered")
            continue

        fmt = ARTIFACT_FORMATS[required_name]
        entry = _mapping(artifact_metadata.get(required_name))
        path = _artifact_file_path(map_dir, required_name, entry)
        exists = path.is_file()
        actual_sha = _sha256_file(path) if exists else None
        actual_shas[required_name] = actual_sha
        declared_sha = entry.get("sha256")
        missing_entry_fields = _missing_required_fields(entry, fmt.required_metadata)
        info: dict[str, Any] = {
            "path": str(path),
            "exists": exists,
            "metadata_present": bool(entry),
            "declared_sha256": declared_sha,
            "actual_sha256": actual_sha,
            "sha256_ok": bool(declared_sha and actual_sha == declared_sha),
            "missing_required_metadata": missing_entry_fields,
            "frame_id": entry.get("frame_id"),
            "source_profile": entry.get("source_profile"),
            "data_source": entry.get("data_source"),
        }
        if required_name == "tomogram":
            source_map_sha = entry.get("source_map_sha256")
            info["source_map_sha256"] = source_map_sha
            info["source_map_sha256_matches_map"] = bool(
                source_map_sha and source_map_sha == actual_shas.get("map_pcd")
            )
        gate["artifacts"][required_name] = info

        if not entry:
            blockers.append(f"{required_name} metadata missing")
        if missing_entry_fields:
            blockers.append(
                f"{required_name} metadata missing required fields: "
                + ", ".join(missing_entry_fields)
            )
        if not exists:
            blockers.append(f"{required_name} file missing")
        if not declared_sha:
            blockers.append(f"{required_name} sha256 missing")
        elif actual_sha != declared_sha:
            blockers.append(f"{required_name} sha256 does not match file")
        if (
            required_name == "tomogram"
            and not info["source_map_sha256_matches_map"]
        ):
            blockers.append("tomogram source_map_sha256 does not match map_pcd")

    gate["ok"] = not blockers
    return gate


def _module_items(gw: Any) -> dict[str, Any]:
    modules = dict(getattr(gw, "_all_modules", {}) or {})
    gateway_name = type(gw).__name__
    if gateway_name not in modules:
        modules[gateway_name] = gw
    return modules


def _safe_port_summary(module: Any) -> dict[str, Any]:
    try:
        if hasattr(module, "port_summary"):
            summary = module.port_summary()
            if isinstance(summary, Mapping):
                return dict(summary)
    except Exception as exc:
        return {"module": type(module).__name__, "error": str(exc)}
    return {
        "module": type(module).__name__,
        "running": bool(getattr(module, "running", False)),
        "ports_in": {},
        "ports_out": {},
    }


def _module_port_snapshot(gw: Any) -> dict[str, Any]:
    snapshot: dict[str, Any] = {}
    for name, module in sorted(_module_items(gw).items()):
        summary = _safe_port_summary(module)
        ports_in = {
            port_name: {"direction": "in", **_mapping(port_summary)}
            for port_name, port_summary in _mapping(summary.get("ports_in")).items()
        }
        ports_out = {
            port_name: {"direction": "out", **_mapping(port_summary)}
            for port_name, port_summary in _mapping(summary.get("ports_out")).items()
        }
        snapshot[str(name)] = {
            "module": summary.get("module", type(module).__name__),
            "running": bool(summary.get("running", False)),
            "layer": summary.get("layer"),
            "ports_in": ports_in,
            "ports_out": ports_out,
        }
        if "error" in summary:
            snapshot[str(name)]["error"] = summary["error"]
    return snapshot


def _port_hint_names(topic: str) -> set[str]:
    names = set(_TOPIC_PORT_HINTS.get(topic, ()))
    leaf = topic.rsplit("/", 1)[-1]
    if leaf:
        names.add(leaf)
    return names


def _topic_module_ports(
    topic: str,
    module_ports: Mapping[str, Any],
) -> list[dict[str, Any]]:
    hints = _port_hint_names(topic)
    matches: list[dict[str, Any]] = []
    for module_name, module_summary in module_ports.items():
        for direction_key in ("ports_in", "ports_out"):
            direction = "in" if direction_key == "ports_in" else "out"
            for port_name, port_summary in _mapping(
                _mapping(module_summary).get(direction_key)
            ).items():
                if port_name not in hints:
                    continue
                stats = _mapping(port_summary)
                matches.append(
                    {
                        "module": str(module_name),
                        "port": str(port_name),
                        "direction": direction,
                        "type": stats.get("type"),
                        "msg_count": int(stats.get("msg_count", 0) or 0),
                        "rate_hz": float(stats.get("rate_hz", 0.0) or 0.0),
                        "stale_ms": stats.get("stale_ms"),
                        "connected": stats.get("connected"),
                        "callbacks": stats.get("callbacks"),
                    }
                )
    return matches


def _module_port_token_matches(
    token: str,
    module_ports: Mapping[str, Any],
) -> list[dict[str, Any]]:
    raw = token.split(":", 1)[1] if ":" in token else ""
    module_token, _, port_token = raw.partition(".")
    if not module_token or not port_token:
        return []

    matches: list[dict[str, Any]] = []
    for module_name, module_summary in module_ports.items():
        summary = _mapping(module_summary)
        declared_module = str(summary.get("module") or "")
        if module_token not in {str(module_name), declared_module}:
            continue
        for direction_key in ("ports_in", "ports_out"):
            direction = "in" if direction_key == "ports_in" else "out"
            ports = _mapping(summary.get(direction_key))
            if port_token not in ports:
                continue
            stats = _mapping(ports[port_token])
            matches.append(
                {
                    "module": str(module_name),
                    "port": str(port_token),
                    "direction": direction,
                    "type": stats.get("type"),
                    "msg_count": int(stats.get("msg_count", 0) or 0),
                    "rate_hz": float(stats.get("rate_hz", 0.0) or 0.0),
                    "stale_ms": stats.get("stale_ms"),
                    "connected": stats.get("connected"),
                    "callbacks": stats.get("callbacks"),
                }
            )
    return matches


def _topic_stages(
    manifest: Mapping[str, Any],
    topic: str,
    runtime_contract: str | None,
) -> list[dict[str, Any]]:
    stage_source = _stage_source(manifest, runtime_contract)
    stages: list[dict[str, Any]] = []
    for stage in stage_source:
        entry = _mapping(stage)
        inputs = list(entry.get("inputs") or [])
        outputs = list(entry.get("outputs") or [])
        roles: list[str] = []
        if topic in inputs:
            roles.append("input")
        if topic in outputs:
            roles.append("output")
        if roles:
            stages.append(
                {
                    "name": entry.get("name"),
                    "roles": roles,
                    "owner": entry.get("owner"),
                    "frame_role": entry.get("frame_role"),
                }
            )
    return stages


def _stage_source(
    manifest: Mapping[str, Any],
    runtime_contract: str | None,
) -> list[dict[str, Any]]:
    resolved_flow = _mapping(manifest.get("resolved_runtime_data_flow")).get(
        runtime_contract or ""
    )
    if isinstance(resolved_flow, list) and resolved_flow:
        return [dict(stage) for stage in resolved_flow if isinstance(stage, Mapping)]
    return [
        dict(stage)
        for stage in (manifest.get("runtime_data_flow") or [])
        if isinstance(stage, Mapping)
    ]


def _topic_communication(topic: str) -> dict[str, Any]:
    commands = [
        command
        for command in _COMMAND_INTERFACES
        if topic in tuple(command.get("publishes") or ())
    ]
    return {
        "allowed": bool(commands),
        "interfaces": commands,
        "arbitrary_publish_supported": False,
        "policy": (
            "command_whitelist_only"
            if commands
            else "read_only_observation_or_endpoint_adapter_owned"
        ),
    }


def _topic_inspection(
    topic_summary: Mapping[str, Any],
    *,
    latest_payload: Any | None = None,
) -> dict[str, Any]:
    topic = str(topic_summary.get("topic") or "")
    observability = _mapping(topic_summary.get("observability"))
    communication = _mapping(topic_summary.get("communication"))
    gateway_channels = [
        dict(channel)
        for channel in (observability.get("gateway_channels") or ())
        if isinstance(channel, Mapping)
    ]
    module_ports = [
        dict(port)
        for port in (observability.get("module_port_candidates") or ())
        if isinstance(port, Mapping)
    ]
    live = bool(
        observability.get("has_fresh_module_sample")
        or observability.get("live_module_samples")
    )
    payload_available = bool(gateway_channels)
    communication_allowed = bool(communication.get("allowed"))
    if live:
        observation_level = "fresh_module_sample"
    elif module_ports or gateway_channels:
        observation_level = "metadata_only"
    else:
        observation_level = "not_observable"
    stream_interfaces: list[dict[str, Any]] = []
    seen_streams: set[tuple[str, str]] = set()
    for channel in gateway_channels:
        if channel.get("transport") != "gateway_sse":
            continue
        event_type = str(channel.get("event_type") or "")
        path = str(channel.get("path") or "/api/v1/events")
        if not event_type:
            continue
        key = (path, event_type)
        if key in seen_streams:
            continue
        seen_streams.add(key)
        stream_interfaces.append(
            {
                "transport": "gateway_sse",
                "path": path,
                "query": {"topic": topic},
                "event_type": event_type,
            }
        )
    result = {
        "observable": bool(observability.get("observable")),
        "observation_level": observation_level,
        "live": live,
        "module_stats_available": bool(module_ports),
        "module_stats": module_ports,
        "payload_available": payload_available,
        "payload_interfaces": gateway_channels,
        "stream_interfaces": stream_interfaces,
        "communicate": communication_allowed,
        "write_interfaces": list(communication.get("interfaces") or []),
        "arbitrary_publish_supported": False,
        "endpoint_topic_required": False,
        "ros2_topic_required": False,
        "policy": (
            "observe via ModulePort stats plus declared Gateway REST/SSE/WS "
            "payload channels; write only through whitelisted Gateway commands"
        ),
    }
    if latest_payload is not None:
        result["latest_payload"] = latest_payload
        result["payload_sample_available"] = True
    else:
        result["payload_sample_available"] = False
    return result


def _observability_summary(
    topic: str,
    module_ports: list[dict[str, Any]],
    gateway_channels: list[dict[str, Any]],
) -> dict[str, Any]:
    observed_module_ports = [
        port
        for port in module_ports
        if _has_fresh_module_sample(port)
    ]
    via = []
    if module_ports:
        via.append("module_port_bus")
    if gateway_channels:
        via.extend(
            sorted({str(channel.get("transport")) for channel in gateway_channels})
        )
    return {
        "observable": bool(module_ports or gateway_channels),
        "observable_via": sorted(via),
        "module_port_candidates": module_ports,
        "gateway_channels": gateway_channels,
        "live_module_samples": bool(observed_module_ports),
        "has_fresh_module_sample": bool(observed_module_ports),
        "fresh_stale_ms_limit": LIVE_MODULE_SAMPLE_STALE_MS,
        "endpoint_topic_required": False,
        "ros2_topic_required": False,
    }


def _has_fresh_module_sample(port: Mapping[str, Any]) -> bool:
    try:
        msg_count = int(port.get("msg_count") or 0)
        rate_hz = float(port.get("rate_hz") or 0.0)
    except (TypeError, ValueError):
        return False
    if msg_count <= 0 and rate_hz <= 0.0:
        return False
    stale_ms = port.get("stale_ms")
    try:
        return stale_ms is None or float(stale_ms) <= LIVE_MODULE_SAMPLE_STALE_MS
    except (TypeError, ValueError):
        return True


def _token_evidence(
    token: str,
    module_ports: Mapping[str, Any],
    runtime_boundary: Mapping[str, Any],
) -> dict[str, Any]:
    token = str(token)
    if token.startswith("/"):
        port_matches = _topic_module_ports(token, module_ports)
        gateway_channels = [dict(item) for item in _GATEWAY_TOPIC_CHANNELS.get(token, [])]
        live = any(_has_fresh_module_sample(port) for port in port_matches)
        observable = bool(port_matches or gateway_channels)
        if live:
            reason = "fresh_module_sample"
        elif observable:
            reason = "metadata_only"
        else:
            reason = "not_observable"
        return {
            "token": token,
            "kind": "topic",
            "observable": observable,
            "live": live,
            "reason": reason,
            "module_ports": port_matches,
            "gateway_channels": gateway_channels,
        }

    if token.startswith("module:"):
        port_matches = _module_port_token_matches(token, module_ports)
        live = any(_has_fresh_module_sample(port) for port in port_matches)
        observable = bool(port_matches)
        if live:
            reason = "fresh_module_sample"
        elif observable:
            reason = "metadata_only"
        else:
            reason = "module_port_not_observable"
        return {
            "token": token,
            "kind": "module_port",
            "observable": observable,
            "live": live,
            "reason": reason,
            "module_ports": port_matches,
            "gateway_channels": [],
        }

    if token.startswith("artifact:"):
        artifact_name = token.split(":", 1)[1]
        artifact_gate = _artifact_gate(token)
        observable = bool(artifact_gate.get("ok"))
        return {
            "token": token,
            "kind": "artifact",
            "observable": observable,
            "live": False,
            "reason": (
                "saved_map_artifact_ok"
                if observable
                else "saved_map_artifact_missing_or_invalid"
            ),
            "module_ports": [],
            "gateway_channels": [
                dict(item)
                for item in _ARTIFACT_GATEWAY_CHANNELS.get(artifact_name, [])
            ],
            "artifact_gate": artifact_gate,
        }

    command_sink = runtime_boundary.get("command_sink")
    if token.startswith("source:"):
        kind = "source_contract"
        observable = True
        reason = "source_contract_declared"
    elif token.startswith("sink:") or token == command_sink:
        kind = "runtime_boundary"
        observable = bool(command_sink)
        reason = "runtime_boundary_declared" if observable else "runtime_boundary_missing"
    else:
        kind = "runtime_boundary"
        observable = token in {str(value) for value in runtime_boundary.values()}
        reason = "runtime_boundary_declared" if observable else "runtime_boundary_not_observable"

    return {
        "token": token,
        "kind": kind,
        "observable": observable,
        "live": False,
        "reason": reason,
        "module_ports": [],
        "gateway_channels": [],
    }


def _runtime_stage_evidence(
    manifest: Mapping[str, Any],
    module_ports: Mapping[str, Any],
    runtime_contract: str | None,
    runtime_boundary: Mapping[str, Any],
) -> list[dict[str, Any]]:
    stages: list[dict[str, Any]] = []
    for stage in (
        *_stage_source(manifest, runtime_contract),
        *_PRODUCT_OBSERVABILITY_STAGES,
    ):
        inputs = [str(item) for item in (stage.get("inputs") or [])]
        outputs = [str(item) for item in (stage.get("outputs") or [])]
        input_evidence = [
            _token_evidence(token, module_ports, runtime_boundary)
            for token in inputs
        ]
        output_evidence = [
            _token_evidence(token, module_ports, runtime_boundary)
            for token in outputs
        ]
        missing_inputs = [
            item["token"]
            for item in input_evidence
            if item["kind"] in {"topic", "artifact", "module_port"}
            and not item["observable"]
        ]
        missing_outputs = [
            item["token"]
            for item in output_evidence
            if item["kind"] in {"topic", "artifact", "module_port"}
            and not item["observable"]
        ]
        not_live_inputs = [
            item["token"]
            for item in input_evidence
            if item["kind"] in {"topic", "module_port"}
            and item["observable"]
            and not item["live"]
        ]
        not_live_outputs = [
            item["token"]
            for item in output_evidence
            if item["kind"] in {"topic", "module_port"}
            and item["observable"]
            and not item["live"]
        ]
        live = any(
            item["kind"] in {"topic", "module_port"} and item["live"]
            for item in (*input_evidence, *output_evidence)
        )
        observable = not missing_inputs and not missing_outputs
        if live:
            status = "live"
        elif observable:
            status = "metadata_only"
        else:
            status = "missing"
        stages.append(
            {
                "name": stage.get("name"),
                "owner": stage.get("owner"),
                "frame_role": stage.get("frame_role"),
                "map_dependency": stage.get("map_dependency"),
                "inputs": inputs,
                "outputs": outputs,
                "input_evidence": input_evidence,
                "output_evidence": output_evidence,
                "observable": observable,
                "live": live,
                "status": status,
                "missing_inputs": missing_inputs,
                "missing_outputs": missing_outputs,
                "not_live_inputs": not_live_inputs,
                "not_live_outputs": not_live_outputs,
            }
        )
    return stages


def _topic_summaries(
    manifest: Mapping[str, Any],
    module_ports: Mapping[str, Any],
    runtime_contract: str | None,
    gw: Any,
) -> list[dict[str, Any]]:
    topics: list[str]
    if runtime_contract:
        try:
            topics = list(runtime_data_flow_topics(runtime_contract))
        except ValueError:
            topics = []
    else:
        topics = list(manifest.get("core_required_topics") or [])

    if not topics:
        topics = list(manifest.get("core_required_topics") or [])
    command_topics = [
        topic
        for interface in _COMMAND_INTERFACES
        for topic in interface.get("publishes", [])
        if isinstance(topic, str) and topic.startswith("/")
    ]
    topics = list(
        dict.fromkeys([*topics, *_PRODUCT_OBSERVABILITY_TOPICS, *command_topics])
    )

    allowed_frames = _mapping(manifest.get("topic_allowed_frame_ids"))
    default_frames = _mapping(manifest.get("topic_default_frame_ids"))
    topic_formats = _mapping(manifest.get("topic_formats"))
    required_real_frames = set(manifest.get("real_runtime_required_topic_frame_ids") or [])

    result: list[dict[str, Any]] = []
    for topic in topics:
        port_matches = _topic_module_ports(topic, module_ports)
        gateway_channels = [dict(item) for item in _GATEWAY_TOPIC_CHANNELS.get(topic, [])]
        item = {
            "topic": topic,
            "message_formats": list(topic_formats.get(topic) or []),
            "default_frame_id": default_frames.get(topic),
            "allowed_frame_ids": list(allowed_frames.get(topic) or []),
            "required_for_real_runtime_frame_evidence": topic in required_real_frames,
            "data_flow_stages": _topic_stages(manifest, topic, runtime_contract),
            "observability": _observability_summary(
                topic,
                port_matches,
                gateway_channels,
            ),
            "communication": _topic_communication(topic),
        }
        item["inspection"] = _topic_inspection(
            item,
            latest_payload=_latest_payload_for_topic(gw, topic),
        )
        result.append(item)
    return result


def _transport_layers() -> dict[str, Any]:
    return {
        "module_port_bus": {
            "primary": True,
            "description": "In-process Module In/Out ports; independent of ROS2 topics.",
        },
        "gateway_realtime": {
            "primary": False,
            "description": "REST/SSE/WebSocket client surface over selected Module data.",
        },
        "endpoint_adapter": {
            "primary": False,
            "description": (
                "Runtime endpoint bridge normalizes external sources into "
                "canonical LingTu streams."
            ),
        },
        "ros2_adapter": {
            "primary": False,
            "description": (
                "Compatibility alias for ROS2-backed endpoint transports; "
                "not the product boundary."
            ),
            "deprecated_by": "endpoint_adapter",
        },
    }


def _control_boundary() -> dict[str, Any]:
    return {
        "arbitrary_publish_supported": False,
        "policy": "whitelisted_gateway_commands_only",
        "command_interfaces": [dict(command) for command in _COMMAND_INTERFACES],
    }


def build_runtime_dataflow_snapshot(gw: Any) -> dict[str, Any]:
    """Return the live Module-first runtime dataflow view for Gateway clients."""
    manifest = runtime_contract_manifest()
    runtime_boundary = _runtime_boundary_status()
    module_ports = _module_port_snapshot(gw)
    runtime_contract = (
        runtime_boundary.get("runtime_contract")
        or runtime_boundary.get("data_source")
        or os.environ.get("LINGTU_RUNTIME_CONTRACT")
    )
    return {
        "schema_version": RUNTIME_DATAFLOW_SCHEMA_VERSION,
        "ts": time.time(),
        "runtime_contract": runtime_contract,
        "runtime_boundary": runtime_boundary,
        "transport_layers": _transport_layers(),
        "endpoint_topic_required": False,
        "ros2_topic_required": False,
        "module_ports": module_ports,
        "topics": _topic_summaries(manifest, module_ports, runtime_contract, gw),
        "stage_evidence": _runtime_stage_evidence(
            manifest,
            module_ports,
            runtime_contract,
            runtime_boundary,
        ),
        "control_boundary": _control_boundary(),
        "links": {
            "events": "/api/v1/events",
            "cloud_ws": "/ws/cloud",
            "state": "/api/v1/state",
            "navigation_status": "/api/v1/navigation/status",
            "localization_status": "/api/v1/localization/status",
            "runtime_contract": "/api/v1/diagnostics/runtime-contract",
            "runtime_dataflow": "/api/v1/runtime/dataflow",
            "runtime_dataflow_topic": "/api/v1/runtime/dataflow/topic",
            "runtime_dataflow_subscribe": "/api/v1/runtime/dataflow/subscribe",
            "runtime_switch_plan": "/api/v1/runtime/switch-plan",
            "field_check": "/api/v1/diagnostics/field-check",
            "algorithm_benchmark_latest": "/api/v1/diagnostics/algorithm-benchmark/latest",
            "inspection_acceptance": "/api/v1/inspection/acceptance",
            "navigation_goal_candidate": "/api/v1/navigation/goal_candidate",
        },
    }


def _topic_selector_matches(selector: str, topic: str) -> bool:
    normalized = selector.strip()
    if not normalized:
        return False
    alias = normalized.strip("/")
    if normalized == topic:
        return True
    if not normalized.startswith("/") and f"/{normalized}" == topic:
        return True
    leaf = topic.rsplit("/", 1)[-1]
    if normalized == leaf or alias == leaf:
        return True
    return normalized in _port_hint_names(topic) or alias in _port_hint_names(topic)


def build_runtime_dataflow_topic_detail(gw: Any, selector: str) -> dict[str, Any]:
    """Return one operator-focused topic view from the Module-first dataflow."""
    snapshot = build_runtime_dataflow_snapshot(gw)
    topic_summary: dict[str, Any] | None = None
    matches = [
        dict(topic)
        for topic in snapshot.get("topics", [])
        if isinstance(topic, Mapping)
        and _topic_selector_matches(selector, str(topic.get("topic", "")))
    ]
    if matches:
        exact = [
            item
            for item in matches
            if str(item.get("topic", "")) == selector.strip()
        ]
        topic_summary = exact[0] if exact else matches[0]

    if topic_summary is None:
        return {
            "schema_version": RUNTIME_DATAFLOW_SCHEMA_VERSION,
            "ok": False,
            "ts": snapshot["ts"],
            "selector": selector,
            "topic": None,
            "runtime_contract": snapshot.get("runtime_contract"),
            "runtime_boundary": snapshot.get("runtime_boundary", {}),
            "inspection": {
                "observable": False,
                "observation_level": "not_observable",
                "live": False,
                "payload_available": False,
                "communicate": False,
                "arbitrary_publish_supported": False,
                "endpoint_topic_required": False,
                "ros2_topic_required": False,
            },
            "error": "runtime_topic_not_found",
            "available_topics": [
                str(item.get("topic"))
                for item in snapshot.get("topics", [])
                if isinstance(item, Mapping)
            ],
            "links": snapshot.get("links", {}),
        }

    return {
        "schema_version": RUNTIME_DATAFLOW_SCHEMA_VERSION,
        "ok": True,
        "ts": snapshot["ts"],
        "selector": selector,
        "topic": topic_summary,
        "runtime_contract": snapshot.get("runtime_contract"),
        "runtime_boundary": snapshot.get("runtime_boundary", {}),
        "inspection": dict(
            topic_summary.get("inspection")
            if isinstance(topic_summary.get("inspection"), Mapping)
            else _topic_inspection(topic_summary)
        ),
        "control_boundary": snapshot.get("control_boundary", {}),
        "links": snapshot.get("links", {}),
    }


def build_runtime_dataflow_subscription(gw: Any, request: Any) -> dict[str, Any]:
    """Return a read-only Gateway SSE subscription plan for one dataflow stream."""
    selector = str(getattr(request, "selector", "") or "").strip()
    transport = str(getattr(request, "transport", "gateway_sse") or "gateway_sse")
    detail = build_runtime_dataflow_topic_detail(gw, selector)
    inspection = (
        detail.get("inspection")
        if isinstance(detail.get("inspection"), Mapping)
        else {}
    )
    stream_interfaces = [
        dict(item)
        for item in (inspection.get("stream_interfaces") or [])
        if isinstance(item, Mapping)
        and item.get("transport") == transport
        and item.get("path")
        and item.get("event_type")
    ]
    topic_summary = (
        detail.get("topic") if isinstance(detail.get("topic"), Mapping) else {}
    )
    topic = str(topic_summary.get("topic") or "") if topic_summary else None
    event_types = sorted(
        {
            str(item.get("event_type"))
            for item in stream_interfaces
            if item.get("event_type")
        }
    )
    blockers: list[str] = []
    if detail.get("ok") is not True:
        blockers.append(str(detail.get("error") or "runtime_topic_not_found"))
    if detail.get("ok") is True and not stream_interfaces:
        blockers.append("no_gateway_sse_stream")
    stream_url = ""
    if topic and stream_interfaces:
        stream_url = (
            str(stream_interfaces[0].get("path") or "/api/v1/events")
            + "?topic="
            + quote(topic, safe="")
        )
    return {
        "schema_version": "lingtu.runtime_dataflow_subscription.v1",
        "ok": not blockers,
        "ts": time.time(),
        "read_only": True,
        "endpoint_topic_required": False,
        "ros2_topic_required": False,
        "arbitrary_publish_supported": False,
        "publishes": [],
        "selector": selector,
        "topic": topic,
        "transport": "gateway_sse",
        "stream_url": stream_url,
        "event_types": event_types,
        "stream_interfaces": stream_interfaces,
        "blockers": blockers,
        "links": detail.get("links", {}),
    }
