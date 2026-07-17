"""Gateway acceptance checks for product-facing field use.

This module consumes Gateway HTTP endpoints, validates them against product
specifications, and produces an acceptance verdict. It stays outside gateway/
to keep acceptance logic separate from Gateway implementation. Gateway imports
remain lazy to preserve module-level import purity.
"""

from __future__ import annotations

import json
import os
import time
from collections.abc import Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any
from urllib.error import URLError
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

from runtime.runtime_interface import REAL_RUNTIME_CONTRACT, TOPICS

GATEWAY_RUNTIME_ACCEPTANCE_SCHEMA_VERSION = "lingtu.gateway_runtime_acceptance.v1"
DEFAULT_GATEWAY_URL = "http://127.0.0.1:5050"
IN_PROCESS_STUB_RUNTIME_CONTRACT = "in_process_stub"
ACCEPTANCE_MODES = ("non_motion", "simulation", "field")
MAX_LIVE_STALE_MS = 2000.0
HARDWARE_COMMAND_SINK = "driver"

GATEWAY_ACCEPTANCE_ENDPOINTS: dict[str, str] = {
    "capabilities": "/api/v1/app/capabilities",
    "readiness": "/api/v1/readiness",
    "runtime_dataflow": "/api/v1/runtime/dataflow",
    "localization_status": "/api/v1/localization/status",
    "navigation_status": "/api/v1/navigation/status",
    "routecheck_latest": "/api/v1/diagnostics/routecheck/latest",
    "real_runtime_evidence": "/api/v1/diagnostics/real-runtime-evidence/latest",
    "algorithm_benchmark_latest": "/api/v1/diagnostics/algorithm-benchmark/latest",
}

REQUIRED_GATEWAY_SNAPSHOTS = (
    "capabilities",
    "readiness",
    "runtime_dataflow",
    "localization_status",
    "navigation_status",
)

PRODUCT_OBSERVABLE_TOPICS = (
    TOPICS.odometry,
    TOPICS.map_cloud,
    TOPICS.localization_health,
    TOPICS.global_path,
    TOPICS.local_path,
    TOPICS.cmd_vel,
    TOPICS.mission_status,
    TOPICS.traversable_frontiers,
    TOPICS.frontier_candidate,
)

FIELD_LIVE_TOPICS = (
    TOPICS.odometry,
    TOPICS.map_cloud,
    TOPICS.global_path,
    TOPICS.local_path,
    TOPICS.cmd_vel,
    TOPICS.traversable_frontiers,
    TOPICS.frontier_candidate,
)

TRAVERSABLE_FRONTIER_PREVIEW_STAGE = "traversable_frontier_preview"
TRAVERSABLE_FRONTIER_PREVIEW_TOPICS = (
    TOPICS.traversable_frontiers,
    TOPICS.frontier_candidate,
)

PRODUCT_REQUIRED_STAGE_NAMES = (
    "slam_or_relayed_localization_map",
    TRAVERSABLE_FRONTIER_PREVIEW_STAGE,
    "global_planning",
    "local_planning_and_following",
    "command_boundary",
)

ADVISORY_STAGE_NAMES = (
    "endpoint_adapter",
    "map_layers_and_exploration",
    "dynamic_obstacle_gate",
)

FIELD_REQUIRED_LIVE_STAGE_NAMES = (
    "slam_or_relayed_localization_map",
    TRAVERSABLE_FRONTIER_PREVIEW_STAGE,
    "global_planning",
    "local_planning_and_following",
    "command_boundary",
)

SIMULATION_REQUIRED_LIVE_STAGE_NAMES = (
    "slam_or_relayed_localization_map",
    TRAVERSABLE_FRONTIER_PREVIEW_STAGE,
    "global_planning",
    "local_planning_and_following",
)

MOTION_ACCEPTANCE_MODES = ("simulation", "field")

REQUIRED_COMMAND_INTERFACE_PATHS = (
    "/api/v1/goal",
    "/api/v1/cmd_vel",
    "/api/v1/stop",
)

ALLOWED_COMMAND_INTERFACE_PATHS = (
    "/api/v1/navigation/plan",
    *REQUIRED_COMMAND_INTERFACE_PATHS,
    "/api/v1/navigate/click",
    "/api/v1/navigation/cancel",
    "/api/v1/instruction",
)


@dataclass(frozen=True)
class GatewayFetchResult:
    name: str
    path: str
    ok: bool
    status: int | None
    payload: dict[str, Any]
    error: str | None = None


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "y"}:
            return True
        if lowered in {"0", "false", "no", "n"}:
            return False
    return None


def _as_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return int(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _snapshot_ok(snapshot: Mapping[str, Any] | None) -> bool:
    if not isinstance(snapshot, Mapping):
        return False
    if snapshot.get("_fetch_error"):
        return False
    status = snapshot.get("_http_status")
    try:
        if status is not None and int(status) >= 400:
            return False
    except (TypeError, ValueError):
        return False
    return True


def _topic_index(dataflow: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    topics: dict[str, dict[str, Any]] = {}
    for item in dataflow.get("topics") or []:
        entry = _mapping(item)
        topic = entry.get("topic")
        if topic:
            topics[str(topic)] = entry
    return topics


def _observable(topic_entry: Mapping[str, Any]) -> bool:
    observability = _mapping(topic_entry.get("observability"))
    return bool(observability.get("observable"))


def _live(topic_entry: Mapping[str, Any]) -> bool:
    observability = _mapping(topic_entry.get("observability"))
    if not observability.get("live_module_samples"):
        return False
    for candidate in observability.get("module_port_candidates") or []:
        port = _mapping(candidate)
        try:
            msg_count = int(port.get("msg_count") or 0)
            rate_hz = float(port.get("rate_hz") or 0.0)
        except (TypeError, ValueError):
            continue
        stale_ms = port.get("stale_ms")
        try:
            stale = stale_ms is not None and float(stale_ms) > MAX_LIVE_STALE_MS
        except (TypeError, ValueError):
            stale = False
        if (msg_count > 0 or rate_hz > 0.0) and not stale:
            return True
    return False


def _stage_index(dataflow: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    stages: dict[str, dict[str, Any]] = {}
    for item in dataflow.get("stage_evidence") or []:
        entry = _mapping(item)
        name = entry.get("name")
        if name:
            stages[str(name)] = entry
    return stages


def _required_section_ok(section: Any) -> bool:
    if not isinstance(section, Mapping) or not section:
        return False
    for raw_entry in section.values():
        entry = _mapping(raw_entry)
        if entry:
            if entry.get("required") is False:
                continue
            if entry.get("ok") is not True:
                return False
        elif raw_entry is not True:
            return False
    return True


def _check_gateway_contract(
    snapshots: Mapping[str, Any],
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    endpoints: dict[str, Any] = {}
    for name, path in GATEWAY_ACCEPTANCE_ENDPOINTS.items():
        snapshot = snapshots.get(name)
        ok = _snapshot_ok(snapshot if isinstance(snapshot, Mapping) else None)
        endpoints[name] = {
            "path": path,
            "ok": ok,
            "status": _mapping(snapshot).get("_http_status"),
            "error": _mapping(snapshot).get("_fetch_error"),
            "required": name in REQUIRED_GATEWAY_SNAPSHOTS,
        }
        if name in REQUIRED_GATEWAY_SNAPSHOTS and not ok:
            blockers.append(f"gateway endpoint unavailable: {name} {path}")
        if name == "routecheck_latest" and not ok:
            advisories.append("routecheck_latest unavailable; latest route preview is not visible")

    capabilities = _mapping(snapshots.get("capabilities"))
    links = _mapping(capabilities.get("links"))
    required_links = {
        "runtime_dataflow": GATEWAY_ACCEPTANCE_ENDPOINTS["runtime_dataflow"],
        "runtime_dataflow_topic": "/api/v1/runtime/dataflow/topic",
        "runtime_dataflow_subscribe": "/api/v1/runtime/dataflow/subscribe",
        "runtime_switch_plan": "/api/v1/runtime/switch-plan",
        "diagnostic_pack": "/api/v1/diagnostic_pack",
        "field_check": "/api/v1/diagnostics/field-check",
        "inspection_acceptance": "/api/v1/inspection/acceptance",
        "routecheck_latest": GATEWAY_ACCEPTANCE_ENDPOINTS["routecheck_latest"],
        "real_runtime_evidence_latest": GATEWAY_ACCEPTANCE_ENDPOINTS["real_runtime_evidence"],
        "algorithm_benchmark_latest": GATEWAY_ACCEPTANCE_ENDPOINTS["algorithm_benchmark_latest"],
        "readiness": "/api/v1/readiness",
        "navigation_status": GATEWAY_ACCEPTANCE_ENDPOINTS["navigation_status"],
        "localization_status": GATEWAY_ACCEPTANCE_ENDPOINTS["localization_status"],
        "navigation_goal_candidate": "/api/v1/navigation/goal_candidate",
    }
    missing_links = [name for name, path in required_links.items() if links.get(name) != path]
    if missing_links:
        blockers.append("capabilities missing product Gateway links: " + ", ".join(missing_links))

    return {
        "ok": all(endpoint["ok"] for endpoint in endpoints.values() if endpoint.get("required")) and not missing_links,
        "endpoints": endpoints,
        "required_links": required_links,
        "missing_links": missing_links,
    }


def _has_gateway_sse_stream(topic_entry: Mapping[str, Any]) -> bool:
    inspection = _mapping(topic_entry.get("inspection"))
    for raw_stream in inspection.get("stream_interfaces") or []:
        stream = _mapping(raw_stream)
        if stream.get("transport") == "gateway_sse" and stream.get("path") and stream.get("event_type"):
            return True

    observability = _mapping(topic_entry.get("observability"))
    for raw_channel in observability.get("gateway_channels") or []:
        channel = _mapping(raw_channel)
        if channel.get("transport") == "gateway_sse" and channel.get("path") and channel.get("event_type"):
            return True
    return False


def _check_dataflow(
    dataflow: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    topic_entries = _topic_index(dataflow)
    missing_topics = [topic for topic in PRODUCT_OBSERVABLE_TOPICS if topic not in topic_entries]
    if missing_topics:
        blockers.append("runtime dataflow missing product topics: " + ", ".join(missing_topics))

    non_observable = [
        topic for topic in PRODUCT_OBSERVABLE_TOPICS if topic in topic_entries and not _observable(topic_entries[topic])
    ]
    if non_observable:
        blockers.append(
            "runtime dataflow topics are not observable through Gateway/Module ports: " + ", ".join(non_observable)
        )

    missing_stream_interfaces = [
        topic
        for topic in PRODUCT_OBSERVABLE_TOPICS
        if topic in topic_entries and not _has_gateway_sse_stream(topic_entries[topic])
    ]
    if missing_stream_interfaces:
        blockers.append(
            "runtime dataflow topics missing Gateway SSE subscription: " + ", ".join(missing_stream_interfaces)
        )

    missing_live = [topic for topic in FIELD_LIVE_TOPICS if topic in topic_entries and not _live(topic_entries[topic])]
    if mode in MOTION_ACCEPTANCE_MODES and missing_live:
        blockers.append(f"{mode} acceptance missing live Module samples: " + ", ".join(missing_live))
    elif missing_live:
        advisories.append(
            "live field samples absent for: "
            + ", ".join(missing_live)
            + "; run field mode during an active navigation session"
        )

    endpoint_topic_required = bool(dataflow.get("endpoint_topic_required", dataflow.get("ros2_topic_required")))
    if endpoint_topic_required:
        blockers.append("Gateway acceptance must not require endpoint topic inspection")

    transport_layers = _mapping(dataflow.get("transport_layers"))
    module_bus = _mapping(transport_layers.get("module_port_bus"))
    endpoint_adapter = _mapping(transport_layers.get("endpoint_adapter") or transport_layers.get("ros2_adapter"))
    ros2_adapter = _mapping(transport_layers.get("ros2_adapter"))
    if module_bus.get("primary") is not True:
        blockers.append("module_port_bus must be the primary dataflow boundary")
    if endpoint_adapter.get("primary") is True:
        blockers.append("endpoint_adapter must not be the primary acceptance boundary")

    control_boundary = _mapping(dataflow.get("control_boundary"))
    if control_boundary.get("arbitrary_publish_supported") is not False:
        blockers.append("Gateway must not expose arbitrary publish as product control")
    command_interfaces = list(control_boundary.get("command_interfaces") or [])
    if not command_interfaces:
        blockers.append("Gateway command whitelist is empty or missing")
    command_paths = {str(_mapping(interface).get("path") or "") for interface in command_interfaces}
    missing_command_paths = [path for path in REQUIRED_COMMAND_INTERFACE_PATHS if path not in command_paths]
    if missing_command_paths:
        blockers.append("Gateway command whitelist missing required interfaces: " + ", ".join(missing_command_paths))
    unexpected_command_paths = [
        path for path in sorted(command_paths) if path and path not in ALLOWED_COMMAND_INTERFACE_PATHS
    ]
    if unexpected_command_paths:
        blockers.append(
            "Gateway command whitelist includes unexpected interfaces: " + ", ".join(unexpected_command_paths)
        )

    return {
        "ok": not missing_topics
        and not non_observable
        and not endpoint_topic_required
        and module_bus.get("primary") is True
        and endpoint_adapter.get("primary") is not True
        and control_boundary.get("arbitrary_publish_supported") is False
        and bool(command_interfaces)
        and not missing_command_paths
        and not unexpected_command_paths
        and not missing_stream_interfaces
        and (mode not in MOTION_ACCEPTANCE_MODES or not missing_live),
        "runtime_contract": dataflow.get("runtime_contract"),
        "endpoint_topic_required": endpoint_topic_required,
        "ros2_topic_required": bool(dataflow.get("ros2_topic_required")),
        "module_port_bus_primary": module_bus.get("primary"),
        "endpoint_adapter_primary": endpoint_adapter.get("primary"),
        "ros2_adapter_primary": ros2_adapter.get("primary"),
        "arbitrary_publish_supported": control_boundary.get("arbitrary_publish_supported"),
        "command_interface_count": len(command_interfaces),
        "missing_command_interfaces": missing_command_paths,
        "unexpected_command_interfaces": unexpected_command_paths,
        "observable_topics": [
            topic for topic in PRODUCT_OBSERVABLE_TOPICS if topic in topic_entries and _observable(topic_entries[topic])
        ],
        "streamable_topics": [
            topic
            for topic in PRODUCT_OBSERVABLE_TOPICS
            if topic in topic_entries and _has_gateway_sse_stream(topic_entries[topic])
        ],
        "missing_topics": missing_topics,
        "non_observable_topics": non_observable,
        "missing_stream_interfaces": missing_stream_interfaces,
        "missing_live_topics": missing_live,
    }


def _latest_payload(topic_entry: Mapping[str, Any]) -> Any:
    inspection = _mapping(topic_entry.get("inspection"))
    return inspection.get("latest_payload")


def _payload_command_published(payload: Any) -> bool:
    if isinstance(payload, Mapping):
        return _mapping(payload).get("command_published") is True
    if isinstance(payload, list):
        return any(_payload_command_published(item) for item in payload)
    return False


def _topic_is_read_only(topic_entry: Mapping[str, Any]) -> bool:
    communication = _mapping(topic_entry.get("communication"))
    inspection = _mapping(topic_entry.get("inspection"))
    return (
        communication.get("allowed") is not True
        and inspection.get("communicate") is not True
        and not communication.get("interfaces")
        and not inspection.get("write_interfaces")
        and communication.get("arbitrary_publish_supported") is not True
        and inspection.get("arbitrary_publish_supported") is not True
    )


def _check_frontier_preview(
    dataflow: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    topic_entries = _topic_index(dataflow)
    stage_entries = _stage_index(dataflow)
    check_blockers: list[str] = []
    missing_topics = [topic for topic in TRAVERSABLE_FRONTIER_PREVIEW_TOPICS if topic not in topic_entries]
    missing_stage = TRAVERSABLE_FRONTIER_PREVIEW_STAGE not in stage_entries
    read_only_topics = [
        topic
        for topic in TRAVERSABLE_FRONTIER_PREVIEW_TOPICS
        if topic in topic_entries and _topic_is_read_only(topic_entries[topic])
    ]
    non_read_only_topics = [
        topic
        for topic in TRAVERSABLE_FRONTIER_PREVIEW_TOPICS
        if topic in topic_entries and topic not in read_only_topics
    ]
    if non_read_only_topics:
        check_blockers.append("traversable frontier preview must be read-only: " + ", ".join(non_read_only_topics))

    stage = _mapping(stage_entries.get(TRAVERSABLE_FRONTIER_PREVIEW_STAGE))
    live = stage.get("live") is True
    if mode in MOTION_ACCEPTANCE_MODES and (missing_topics or missing_stage or not live):
        check_blockers.append(f"{mode} acceptance requires live traversable frontier preview")

    candidate_entry = _mapping(topic_entries.get(TOPICS.frontier_candidate))
    frontiers_entry = _mapping(topic_entries.get(TOPICS.traversable_frontiers))
    candidate_payload = _mapping(_latest_payload(candidate_entry))
    frontiers_payload = _latest_payload(frontiers_entry)
    candidate_source = str(candidate_payload.get("source") or "")
    payload_available = bool(candidate_payload) and isinstance(frontiers_payload, list)
    if mode in MOTION_ACCEPTANCE_MODES and not payload_available:
        check_blockers.append(f"{mode} acceptance requires traversable frontier preview payload")
    elif not payload_available:
        advisories.append("traversable frontier preview payload unavailable; run during active exploration")

    command_published = _payload_command_published(candidate_payload) or (_payload_command_published(frontiers_payload))
    if command_published:
        check_blockers.append("traversable frontier preview published a command")

    blockers.extend(check_blockers)
    return {
        "ok": not check_blockers,
        "required": True,
        "stage": TRAVERSABLE_FRONTIER_PREVIEW_STAGE,
        "topics": list(TRAVERSABLE_FRONTIER_PREVIEW_TOPICS),
        "missing_topics": missing_topics,
        "missing_stage": missing_stage,
        "live": live,
        "read_only": not non_read_only_topics,
        "read_only_topics": read_only_topics,
        "non_read_only_topics": non_read_only_topics,
        "payload_available": payload_available,
        "candidate_source": candidate_source or None,
        "candidate": candidate_payload,
        "frontier_count": (len(frontiers_payload) if isinstance(frontiers_payload, list) else None),
        "command_published": command_published,
        "blockers": check_blockers,
    }


def _live_stage_names_for_mode(mode: str) -> tuple[str, ...]:
    if mode == "field":
        return FIELD_REQUIRED_LIVE_STAGE_NAMES
    if mode == "simulation":
        return SIMULATION_REQUIRED_LIVE_STAGE_NAMES
    return ()


def _check_stage_evidence(
    dataflow: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    stage_entries = _stage_index(dataflow)
    available = bool(stage_entries)
    if not available:
        if mode in MOTION_ACCEPTANCE_MODES:
            blockers.append(f"{mode} acceptance requires runtime stage evidence")
        else:
            blockers.append("runtime dataflow missing stage_evidence")

    missing_stages = [stage for stage in PRODUCT_REQUIRED_STAGE_NAMES if stage not in stage_entries]
    if missing_stages:
        blockers.append("runtime dataflow missing required stages: " + ", ".join(missing_stages))

    missing_tokens: dict[str, dict[str, list[str]]] = {}
    non_observable_stages: list[str] = []
    stage_token_gaps_block = mode in MOTION_ACCEPTANCE_MODES
    for name in PRODUCT_REQUIRED_STAGE_NAMES:
        stage = stage_entries.get(name)
        if not stage:
            continue
        missing_inputs = [str(item) for item in (stage.get("missing_inputs") or []) if item]
        missing_outputs = [str(item) for item in (stage.get("missing_outputs") or []) if item]
        if missing_inputs or missing_outputs or stage.get("observable") is False:
            non_observable_stages.append(name)
        if missing_inputs or missing_outputs:
            missing_tokens[name] = {
                "inputs": missing_inputs,
                "outputs": missing_outputs,
            }
            message = f"runtime stage evidence missing tokens: {name}"
            if stage_token_gaps_block:
                blockers.append(message)
            else:
                advisories.append(message)

    live_required = _live_stage_names_for_mode(mode)
    not_live_stage_tokens: dict[str, dict[str, list[str]]] = {}
    not_live_stages: list[str] = []
    for name in live_required:
        stage = stage_entries.get(name)
        if not stage:
            continue
        not_live_inputs = [str(item) for item in (stage.get("not_live_inputs") or []) if item]
        not_live_outputs = [str(item) for item in (stage.get("not_live_outputs") or []) if item]
        if stage.get("live") is not True or not_live_inputs or not_live_outputs:
            not_live_stages.append(name)
            if not_live_inputs or not_live_outputs:
                not_live_stage_tokens[name] = {
                    "inputs": not_live_inputs,
                    "outputs": not_live_outputs,
                }
    if not_live_stages:
        blockers.append(f"{mode} acceptance requires live dataflow stages: " + ", ".join(not_live_stages))

    advisory_not_live = [
        name
        for name in PRODUCT_REQUIRED_STAGE_NAMES
        if name in stage_entries and stage_entries[name].get("live") is not True and name not in not_live_stages
    ]
    if advisory_not_live:
        advisories.append(
            "stage evidence is metadata_only for: "
            + ", ".join(advisory_not_live)
            + "; run during active navigation for motion evidence"
        )

    missing_advisory_stages = [stage for stage in ADVISORY_STAGE_NAMES if stage not in stage_entries]
    if missing_advisory_stages:
        advisories.append("diagnostic stage evidence missing: " + ", ".join(missing_advisory_stages))

    live_stages = [name for name, stage in stage_entries.items() if stage.get("live") is True]
    stage_statuses = {name: str(stage.get("status") or "unknown") for name, stage in stage_entries.items()}
    ok = (
        available
        and not missing_stages
        and (not non_observable_stages or not stage_token_gaps_block)
        and (not missing_tokens or not stage_token_gaps_block)
        and not not_live_stages
    )
    return {
        "ok": ok,
        "required": True,
        "available": available,
        "stage_count": len(stage_entries),
        "required_stages": list(PRODUCT_REQUIRED_STAGE_NAMES),
        "advisory_stages": list(ADVISORY_STAGE_NAMES),
        "live_required_stages": list(live_required),
        "live_stages": live_stages,
        "missing_stages": missing_stages,
        "non_observable_stages": non_observable_stages,
        "not_live_stages": not_live_stages,
        "not_live_stage_tokens": not_live_stage_tokens,
        "advisory_not_live_stages": advisory_not_live,
        "missing_advisory_stages": missing_advisory_stages,
        "stage_statuses": stage_statuses,
        "missing_tokens": missing_tokens,
    }


def _check_runtime_mode(
    dataflow: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    check_blockers: list[str] = []
    runtime_contract = dataflow.get("runtime_contract")
    runtime_boundary = _mapping(dataflow.get("runtime_boundary"))
    simulation_only = _as_bool(runtime_boundary.get("simulation_only"))
    data_source = runtime_boundary.get("data_source")
    endpoint = runtime_boundary.get("endpoint")
    command_sink = runtime_boundary.get("command_sink")
    boundary_blockers = [str(item) for item in runtime_boundary.get("blockers") or [] if item]
    if runtime_boundary.get("ok") is False or boundary_blockers:
        check_blockers.append("runtime boundary blockers: " + ", ".join(boundary_blockers or ["not_ok"]))

    if mode == "simulation":
        if not runtime_contract:
            check_blockers.append("simulation acceptance requires runtime_contract")
        if runtime_contract == REAL_RUNTIME_CONTRACT:
            check_blockers.append(f"simulation acceptance must not run against {REAL_RUNTIME_CONTRACT} runtime")
        if simulation_only is False:
            check_blockers.append("simulation acceptance requires simulation_only=true")
        elif simulation_only is None:
            advisories.append("simulation acceptance could not confirm simulation_only runtime boundary")
        if command_sink == HARDWARE_COMMAND_SINK:
            check_blockers.append("simulation acceptance must not use hardware command sink")
    elif mode == "field":
        if runtime_contract != REAL_RUNTIME_CONTRACT:
            check_blockers.append(f"field acceptance requires {REAL_RUNTIME_CONTRACT} runtime contract")
        if simulation_only is True:
            check_blockers.append("field acceptance must not run against simulation endpoint")
        if command_sink != HARDWARE_COMMAND_SINK:
            check_blockers.append("field acceptance requires hardware command sink after VelocityMux")
    blockers.extend(check_blockers)

    return {
        "ok": not check_blockers,
        "endpoint": endpoint,
        "data_source": data_source,
        "runtime_contract": runtime_contract,
        "simulation_only": simulation_only,
        "command_sink": command_sink,
        "blockers": check_blockers,
    }


def _check_readiness(
    readiness: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    runtime = _mapping(readiness.get("runtime"))
    summary = _mapping(runtime.get("summary"))
    data_ready = _as_bool(summary.get("data_ready", readiness.get("data_ready")))
    motion_ready = _as_bool(summary.get("motion_ready", readiness.get("motion_ready")))
    non_motion_safe = _as_bool(summary.get("non_motion_safe", readiness.get("non_motion_safe")))
    ready = _as_bool(readiness.get("ready"))

    status = str(readiness.get("status") or "unknown").lower()
    if status in {"failed", "error", "not_started"}:
        blockers.append(f"readiness status is {status}")
    if ready is False and status != "degraded":
        blockers.append("readiness ready=false")
    if non_motion_safe is False:
        blockers.append("readiness reports non_motion_safe=false")
    if data_ready is False:
        blockers.append("readiness reports data_ready=false")
    if mode in MOTION_ACCEPTANCE_MODES and motion_ready is not True:
        blockers.append(f"{mode} acceptance requires motion_ready=true")
    elif motion_ready is not True:
        advisories.append("motion_ready is not true; this is not field navigation evidence")

    return {
        "ok": status not in {"failed", "error", "not_started"}
        and (ready is not False or status == "degraded")
        and data_ready is not False
        and non_motion_safe is not False
        and (mode not in MOTION_ACCEPTANCE_MODES or motion_ready is True),
        "status": status,
        "ready": ready,
        "data_ready": data_ready,
        "motion_ready": motion_ready,
        "non_motion_safe": non_motion_safe,
    }


def _check_localization(
    localization: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    state = str(localization.get("state") or "unknown").lower()
    bad_states = {"lost", "no_odometry", "uninitialized", "uninit", "error"}
    if mode in MOTION_ACCEPTANCE_MODES and state in bad_states:
        blockers.append(f"{mode} acceptance localization state is {state}")
    elif state in bad_states:
        advisories.append(f"localization state is {state}; not field-ready")
    return {
        "ok": mode not in MOTION_ACCEPTANCE_MODES or state not in bad_states,
        "state": state,
        "has_odometry": localization.get("has_odometry"),
        "can_relocalize": localization.get("can_relocalize"),
    }


def _check_navigation(
    navigation: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    readiness = _mapping(navigation.get("readiness"))
    can_send_goal = _as_bool(readiness.get("can_send_goal", readiness.get("can_accept_goal")))
    blockers_list = list(readiness.get("blockers") or [])
    if mode in MOTION_ACCEPTANCE_MODES and can_send_goal is not True:
        blockers.append(f"{mode} acceptance requires navigation can_send_goal=true")
    elif can_send_goal is not True:
        advisories.append("navigation cannot currently accept a goal")
    if mode in MOTION_ACCEPTANCE_MODES and blockers_list:
        blockers.append("navigation readiness blockers: " + ", ".join(map(str, blockers_list)))
    return {
        "ok": mode not in MOTION_ACCEPTANCE_MODES or (can_send_goal is True and not blockers_list),
        "state": navigation.get("state"),
        "can_send_goal": can_send_goal,
        "blockers": blockers_list,
    }


def _check_real_runtime_evidence(
    evidence: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    endpoint_ok = _snapshot_ok(evidence)
    report_age_s = evidence.get("report_age_s")
    max_age_s = evidence.get("max_age_s")
    try:
        stale = report_age_s is not None and max_age_s is not None and float(report_age_s) > float(max_age_s)
    except (TypeError, ValueError):
        stale = False

    real_motion = _mapping(evidence.get("checked_real_motion_evidence"))
    hardware = _mapping(evidence.get("checked_hardware_boundary_evidence"))
    live_freshness = evidence.get("checked_live_topic_freshness")
    data_flow = evidence.get("checked_runtime_data_flow_evidence")
    field_ok = (
        endpoint_ok
        and evidence.get("ok") is True
        and evidence.get("runtime_evidence_ok") is True
        and evidence.get("runtime_contract") == REAL_RUNTIME_CONTRACT
        and evidence.get("simulation_only") is False
        and evidence.get("real_robot_motion") is True
        and evidence.get("cmd_vel_sent_to_hardware") is True
        and not stale
        and real_motion.get("ok") is True
        and hardware.get("ok") is True
        and _required_section_ok(live_freshness)
        and _required_section_ok(data_flow)
    )
    if mode == "field" and not field_ok:
        blockers.append("field acceptance requires passing real-runtime-evidence")
        for blocker in evidence.get("blockers") or []:
            blockers.append(f"real-runtime-evidence: {blocker}")
        if stale:
            blockers.append("real-runtime-evidence is stale")
    elif mode == "simulation":
        if evidence.get("simulation_only") is False and evidence.get("ok") is True:
            advisories.append("real-runtime-evidence is ignored in simulation acceptance")
    elif not field_ok:
        advisories.append("real-runtime-evidence is not currently passing; field acceptance is not proven")

    return {
        "ok": field_ok if mode == "field" else mode == "simulation",
        "required": mode == "field",
        "available": bool(evidence),
        "path": GATEWAY_ACCEPTANCE_ENDPOINTS["real_runtime_evidence"],
        "runtime_contract": evidence.get("runtime_contract"),
        "runtime_evidence_ok": evidence.get("runtime_evidence_ok"),
        "simulation_only": evidence.get("simulation_only"),
        "real_robot_motion": evidence.get("real_robot_motion"),
        "cmd_vel_sent_to_hardware": evidence.get("cmd_vel_sent_to_hardware"),
        "report_age_s": report_age_s,
        "max_age_s": max_age_s,
        "stale": stale,
        "real_motion_ok": real_motion.get("ok"),
        "hardware_boundary_ok": hardware.get("ok"),
        "live_topic_freshness_ok": _required_section_ok(live_freshness),
        "data_flow_ok": _required_section_ok(data_flow),
        "blockers": list(evidence.get("blockers") or []),
    }


def _check_routecheck_latest(
    routecheck: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    check_blockers: list[str] = []
    latest = _mapping(routecheck.get("latest"))
    published = _mapping(routecheck.get("published") or latest.get("published"))
    outcome = routecheck.get("outcome") or latest.get("outcome")
    non_motion = _as_bool(routecheck.get("non_motion", latest.get("non_motion")))
    gateway_used = _as_bool(routecheck.get("gateway_used", latest.get("gateway_used")))
    driver_used = _as_bool(routecheck.get("driver_used", latest.get("driver_used")))

    if not routecheck:
        check_blockers.append("latest routecheck is unavailable")
    elif routecheck.get("ok") is not True:
        reason = routecheck.get("reason") or routecheck.get("_fetch_error") or "not_ok"
        check_blockers.append(f"latest routecheck is not ok: {reason}")

    if routecheck:
        if outcome != "pass":
            check_blockers.append(f"latest routecheck outcome is {outcome or 'missing'}")
        if non_motion is not True:
            check_blockers.append("latest routecheck non_motion is not true")
        if gateway_used is not True:
            check_blockers.append("latest routecheck gateway_used is not true")
        if driver_used is not False:
            check_blockers.append("latest routecheck driver_used is not false")
        for topic in ("goal_pose", "cmd_vel", "stop_cmd"):
            count = _as_int(published.get(topic))
            if count is None:
                check_blockers.append(f"latest routecheck published.{topic} is missing")
            elif count != 0:
                check_blockers.append(f"latest routecheck published.{topic} is not 0")

    ok = not check_blockers
    if check_blockers:
        detail = "; ".join(check_blockers)
        if mode in MOTION_ACCEPTANCE_MODES:
            blockers.append(f"{mode} acceptance requires passing latest routecheck: {detail}")
        else:
            advisories.append(f"latest routecheck is not passing: {detail}")

    return {
        "ok": ok,
        "available": bool(routecheck),
        "path": GATEWAY_ACCEPTANCE_ENDPOINTS["routecheck_latest"],
        "outcome": outcome,
        "non_motion": non_motion,
        "gateway_used": gateway_used,
        "driver_used": driver_used,
        "published": published,
        "blockers": check_blockers,
    }


def evaluate_gateway_runtime_acceptance(
    snapshots: Mapping[str, Any],
    *,
    mode: str = "non_motion",
) -> dict[str, Any]:
    """Evaluate product acceptance from Gateway snapshots only."""

    if mode not in ACCEPTANCE_MODES:
        raise ValueError(f"unknown gateway acceptance mode {mode!r}")

    blockers: list[str] = []
    advisories: list[str] = []

    gateway_contract = _check_gateway_contract(snapshots, blockers, advisories)
    dataflow = _check_dataflow(
        _mapping(snapshots.get("runtime_dataflow")),
        mode,
        blockers,
        advisories,
    )
    stage_evidence = _check_stage_evidence(
        _mapping(snapshots.get("runtime_dataflow")),
        mode,
        blockers,
        advisories,
    )
    frontier_preview = _check_frontier_preview(
        _mapping(snapshots.get("runtime_dataflow")),
        mode,
        blockers,
        advisories,
    )
    runtime_mode = _check_runtime_mode(
        _mapping(snapshots.get("runtime_dataflow")),
        mode,
        blockers,
        advisories,
    )
    readiness = _check_readiness(
        _mapping(snapshots.get("readiness")),
        mode,
        blockers,
        advisories,
    )
    localization = _check_localization(
        _mapping(snapshots.get("localization_status")),
        mode,
        blockers,
        advisories,
    )
    navigation = _check_navigation(
        _mapping(snapshots.get("navigation_status")),
        mode,
        blockers,
        advisories,
    )
    real_runtime_evidence = _check_real_runtime_evidence(
        _mapping(snapshots.get("real_runtime_evidence")),
        mode,
        blockers,
        advisories,
    )

    routecheck_latest = _check_routecheck_latest(
        _mapping(snapshots.get("routecheck_latest")),
        mode,
        blockers,
        advisories,
    )
    algorithm_benchmark_latest = _mapping(snapshots.get("algorithm_benchmark_latest"))

    checks = {
        "gateway_contract": gateway_contract,
        "runtime_mode": runtime_mode,
        "module_first_dataflow": dataflow,
        "stage_evidence": stage_evidence,
        "frontier_preview": frontier_preview,
        "readiness": readiness,
        "localization": localization,
        "navigation": navigation,
        "real_runtime_evidence": real_runtime_evidence,
        "routecheck_latest": routecheck_latest,
        "algorithm_benchmark_latest": algorithm_benchmark_latest,
    }
    top_level_simulation_only = (
        runtime_mode.get("simulation_only") if runtime_mode.get("simulation_only") is not None else mode == "simulation"
    )
    top_level_real_robot_motion = real_runtime_evidence.get("real_robot_motion") is True if mode == "field" else False
    top_level_cmd_vel_sent_to_hardware = (
        real_runtime_evidence.get("cmd_vel_sent_to_hardware") is True if mode == "field" else False
    )
    return {
        "schema_version": GATEWAY_RUNTIME_ACCEPTANCE_SCHEMA_VERSION,
        "ok": not blockers,
        "mode": mode,
        "simulation_only": top_level_simulation_only,
        "real_robot_motion": top_level_real_robot_motion,
        "cmd_vel_sent_to_hardware": top_level_cmd_vel_sent_to_hardware,
        "target_result": ("Gateway-only product runtime acceptance; endpoint topic inspection is not required."),
        "runtime_contract": dataflow.get("runtime_contract"),
        "endpoint_topic_required": dataflow.get(
            "endpoint_topic_required",
            dataflow.get("ros2_topic_required"),
        ),
        "ros2_topic_required": dataflow.get("ros2_topic_required"),
        "blockers": blockers,
        "advisories": advisories,
        "checks": checks,
        "validated_from": {name: GATEWAY_ACCEPTANCE_ENDPOINTS[name] for name in GATEWAY_ACCEPTANCE_ENDPOINTS},
        "ts": time.time(),
    }


def _fetch_json(base_url: str, name: str, path: str, timeout_sec: float) -> GatewayFetchResult:
    url = urljoin(base_url.rstrip("/") + "/", path.lstrip("/"))
    request = Request(url, headers={"Accept": "application/json"})
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            status = getattr(response, "status", None) or response.getcode()
            raw = response.read().decode("utf-8")
    except (OSError, URLError, TimeoutError) as exc:
        return GatewayFetchResult(
            name=name,
            path=path,
            ok=False,
            status=None,
            payload={"_fetch_error": str(exc)},
            error=str(exc),
        )
    try:
        payload = json.loads(raw) if raw else {}
    except json.JSONDecodeError as exc:
        return GatewayFetchResult(
            name=name,
            path=path,
            ok=False,
            status=status,
            payload={"_fetch_error": f"invalid_json: {exc}"},
            error=f"invalid_json: {exc}",
        )
    if not isinstance(payload, dict):
        payload = {"value": payload}
    payload["_http_status"] = status
    return GatewayFetchResult(
        name=name,
        path=path,
        ok=200 <= int(status) < 400,
        status=int(status),
        payload=payload,
        error=None,
    )


def collect_gateway_runtime_snapshots(
    *,
    gateway_url: str = DEFAULT_GATEWAY_URL,
    timeout_sec: float = 2.0,
) -> dict[str, dict[str, Any]]:
    """Fetch all Gateway snapshots needed for product acceptance."""

    snapshots: dict[str, dict[str, Any]] = {}
    for name, path in GATEWAY_ACCEPTANCE_ENDPOINTS.items():
        result = _fetch_json(gateway_url, name, path, timeout_sec)
        snapshots[name] = result.payload
    return snapshots


@contextmanager
def _temporary_env(overrides: Mapping[str, str]):
    previous: dict[str, str | None] = {key: os.environ.get(key) for key in overrides}
    try:
        for key, value in overrides.items():
            os.environ[key] = value
        yield
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def _is_default_local_gateway_url(gateway_url: str) -> bool:
    parsed = urlparse(gateway_url)
    host = (parsed.hostname or "").lower()
    return parsed.scheme in {"http", "https"} and host in {"127.0.0.1", "localhost", "::1"} and parsed.port == 5050


def _all_required_gateway_fetches_failed(
    snapshots: Mapping[str, Any],
) -> bool:
    for name in REQUIRED_GATEWAY_SNAPSHOTS:
        snapshot = _mapping(snapshots.get(name))
        if not snapshot.get("_fetch_error"):
            return False
        if snapshot.get("_http_status") is not None:
            return False
    return True


def _collect_in_process_stub_gateway_snapshots() -> dict[str, dict[str, Any]]:
    """Build non-motion Gateway snapshots without starting uvicorn or drivers."""

    env = {
        "LINGTU_BLACKBOX_ENABLED": "0",
        "LINGTU_PROFILE": "stub",
        "LINGTU_ENDPOINT": "in_process_gateway_stub",
        "LINGTU_DATA_SOURCE": IN_PROCESS_STUB_RUNTIME_CONTRACT,
        "LINGTU_RUNTIME_CONTRACT": IN_PROCESS_STUB_RUNTIME_CONTRACT,
        "LINGTU_SIMULATION_ONLY": "1",
    }
    with _temporary_env(env):
        from gateway.gateway_module import GatewayModule
        from gateway.routes.diagnostics import _gateway_acceptance_snapshots

        gateway = GatewayModule(host="127.0.0.1", port=0)
        gateway.setup()
        gateway.on_system_modules({"GatewayModule": gateway})
        return _gateway_acceptance_snapshots(gateway)


def collect_gateway_runtime_acceptance(
    *,
    gateway_url: str = DEFAULT_GATEWAY_URL,
    timeout_sec: float = 2.0,
    mode: str = "non_motion",
) -> dict[str, Any]:
    """Fetch Gateway snapshots and evaluate runtime acceptance."""

    snapshots = collect_gateway_runtime_snapshots(
        gateway_url=gateway_url,
        timeout_sec=timeout_sec,
    )
    snapshot_source = "gateway_http"
    if (
        mode == "non_motion"
        and _is_default_local_gateway_url(gateway_url)
        and _all_required_gateway_fetches_failed(snapshots)
    ):
        snapshots = _collect_in_process_stub_gateway_snapshots()
        snapshot_source = "in_process_stub"
    payload = evaluate_gateway_runtime_acceptance(snapshots, mode=mode)
    payload["gateway_url"] = gateway_url
    payload["snapshot_source"] = snapshot_source
    return payload


def format_gateway_runtime_acceptance(payload: Mapping[str, Any]) -> str:
    """Operator-facing summary for the Gateway runtime acceptance report."""

    status = "PASS" if payload.get("ok") else "FAIL"
    endpoint_topic_required = payload.get(
        "endpoint_topic_required",
        payload.get("ros2_topic_required"),
    )
    lines = [
        f"Gateway runtime acceptance: {status}",
        f"Schema: {payload.get('schema_version')}",
        f"Mode: {payload.get('mode')}",
        f"Runtime contract: {payload.get('runtime_contract') or 'unknown'}",
        f"Endpoint topic required: {str(bool(endpoint_topic_required)).lower()}",
    ]
    checks = _mapping(payload.get("checks"))
    stage_evidence = _mapping(checks.get("stage_evidence"))
    if stage_evidence:
        live_stages = list(stage_evidence.get("live_stages") or [])
        missing_stages = list(stage_evidence.get("missing_stages") or [])
        not_live_stages = list(stage_evidence.get("not_live_stages") or [])
        lines.append(
            "Stage evidence: "
            f"ok={str(bool(stage_evidence.get('ok'))).lower()} "
            f"stages={stage_evidence.get('stage_count', 0)} "
            f"live={', '.join(str(item) for item in live_stages) or 'none'}"
        )
        if missing_stages:
            lines.append("Missing stages: " + ", ".join(str(item) for item in missing_stages))
        if not_live_stages:
            lines.append("Not-live stages: " + ", ".join(str(item) for item in not_live_stages))
    frontier_preview = _mapping(checks.get("frontier_preview"))
    if frontier_preview:
        lines.append(
            "Frontier preview: "
            f"ok={str(bool(frontier_preview.get('ok'))).lower()} "
            f"source={frontier_preview.get('candidate_source') or 'unknown'} "
            "command_published="
            f"{str(bool(frontier_preview.get('command_published'))).lower()}"
        )
    blockers = list(payload.get("blockers") or [])
    advisories = list(payload.get("advisories") or [])
    if blockers:
        lines.append("Blockers:")
        lines.extend(f"  - {blocker}" for blocker in blockers)
    if advisories:
        lines.append("Advisories:")
        lines.extend(f"  - {advisory}" for advisory in advisories)
    if checks:
        lines.append("Checks:")
        for name, raw_check in checks.items():
            check = _mapping(raw_check)
            lines.append(f"  {name} ok={str(bool(check.get('ok'))).lower()}")
    return "\n".join(lines)
