"""Gateway acceptance checks for product-facing field use.

This module evaluates current Gateway snapshots against one Product's resolved
topics and runtime boundary. It stays outside Gateway implementation so field
acceptance remains a read-only consumer.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from typing import Any

from runtime.runtime_interface import REAL_RUNTIME_CONTRACT, TOPICS

GATEWAY_RUNTIME_ACCEPTANCE_SCHEMA_VERSION = "lingtu.gateway_runtime_acceptance.v3"
ACCEPTANCE_MODES = ("non_motion", "simulation", "field")
MAX_LIVE_STALE_MS = 2000.0
HARDWARE_COMMAND_SINK = "driver"

GATEWAY_ACCEPTANCE_ENDPOINTS: dict[str, str] = {
    "readiness": "/api/v1/readiness",
    "runtime_dataflow": "/api/v1/runtime/dataflow",
    "localization_status": "/api/v1/localization/status",
    "navigation_status": "/api/v1/navigation/status",
    "real_runtime_evidence": "/api/v1/diagnostics/real-runtime-evidence/latest",
}

REQUIRED_GATEWAY_SNAPSHOTS = (
    "readiness",
    "runtime_dataflow",
    "localization_status",
    "navigation_status",
)

MOTION_ACCEPTANCE_MODES = ("simulation", "field")

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


def _check_gateway_contract(
    snapshots: Mapping[str, Any],
    blockers: list[str],
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

    return {
        "ok": all(endpoint["ok"] for endpoint in endpoints.values() if endpoint.get("required")),
        "endpoints": endpoints,
    }
def _check_gateway_observability(
    dataflow: Mapping[str, Any],
    mode: str,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    topic_entries = _topic_index(dataflow)
    run_plan = _mapping(dataflow.get("run_plan"))
    declared_topics = tuple(
        str(topic) for topic in run_plan.get("required_topics") or () if str(topic)
    )
    # runtime_dataflow already filters this list to the active Product and
    # attaches the Gateway port/channel contract for each topic. Native-only
    # DDS topics are valid Product topics, but are not Gateway observations.
    observable_topics = tuple(
        topic for topic, entry in topic_entries.items() if _observable(entry)
    )
    module_observable_topics = tuple(
        topic
        for topic in observable_topics
        if _mapping(topic_entries[topic].get("observability")).get("module_port_candidates")
    )

    missing_live = [
        topic
        for topic in module_observable_topics
        if not _live(topic_entries[topic])
    ]
    if mode in MOTION_ACCEPTANCE_MODES and missing_live:
        blockers.append(f"{mode} acceptance missing live Module samples: " + ", ".join(missing_live))
    elif missing_live:
        advisories.append(
            "live field samples absent for: "
            + ", ".join(missing_live)
            + "; run field mode during an active navigation session"
        )

    return {
        "ok": mode not in MOTION_ACCEPTANCE_MODES or not missing_live,
        "runtime_contract": dataflow.get("runtime_contract"),
        "required_topics": list(declared_topics or topic_entries),
        "observable_topics": list(observable_topics),
        "missing_topics": [],
        "non_observable_topics": [
            topic for topic, entry in topic_entries.items() if not _observable(entry)
        ],
        "missing_live_topics": missing_live,
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
    env = runtime_boundary.get("env")
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
            check_blockers.append("field acceptance must not run in the sim Env")
        if command_sink != HARDWARE_COMMAND_SINK:
            check_blockers.append("field acceptance requires command_sink=driver")
    blockers.extend(check_blockers)

    return {
        "ok": not check_blockers,
        "env": env,
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
    if non_motion_safe is not True:
        blockers.append("readiness requires non_motion_safe=true")
    if data_ready is not True:
        blockers.append("readiness requires data_ready=true")
    if mode in MOTION_ACCEPTANCE_MODES and motion_ready is not True:
        blockers.append(f"{mode} acceptance requires motion_ready=true")
    elif motion_ready is not True:
        advisories.append("motion_ready is not true; this is not field navigation evidence")

    return {
        "ok": status not in {"failed", "error", "not_started"}
        and (ready is not False or status == "degraded")
        and data_ready is True
        and non_motion_safe is True
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
    *,
    localization_required: bool,
    blockers: list[str],
    advisories: list[str],
) -> dict[str, Any]:
    state = str(localization.get("state") or "unknown").lower()
    has_odometry = _as_bool(localization.get("has_odometry"))
    bad_states = {"unknown", "lost", "no_odometry", "uninitialized", "uninit", "error"}
    failed = localization_required and (state in bad_states or has_odometry is not True)
    if mode in MOTION_ACCEPTANCE_MODES and failed:
        blockers.append(f"{mode} acceptance localization state is {state}")
    elif localization_required and failed:
        advisories.append(f"localization state is {state}; not field-ready")
    return {
        "ok": mode not in MOTION_ACCEPTANCE_MODES or not failed,
        "required": localization_required,
        "state": state,
        "has_odometry": has_odometry,
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
        age = float(report_age_s)
        max_age = float(max_age_s)
        age_valid = math.isfinite(age) and math.isfinite(max_age) and age >= 0.0 and max_age >= 0.0
    except (TypeError, ValueError):
        age = None
        max_age = None
        age_valid = False
    stale = bool(age_valid and age is not None and max_age is not None and age > max_age)

    field_ok = (
        endpoint_ok
        and evidence.get("ok") is True
        and evidence.get("runtime_evidence_ok") is True
        and evidence.get("runtime_contract") == REAL_RUNTIME_CONTRACT
        and evidence.get("simulation_only") is False
        and evidence.get("real_robot_motion") is True
        and evidence.get("cmd_vel_sent_to_hardware") is True
        and age_valid
        and not stale
    )
    if mode == "field" and not field_ok:
        blockers.append("field acceptance requires passing real-runtime-evidence")
        for blocker in evidence.get("blockers") or []:
            blockers.append(f"real-runtime-evidence: {blocker}")
        if stale:
            blockers.append("real-runtime-evidence is stale")
        elif not age_valid:
            blockers.append("real-runtime-evidence age is invalid")
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
        "age_valid": age_valid,
        "blockers": list(evidence.get("blockers") or []),
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

    gateway_contract = _check_gateway_contract(snapshots, blockers)
    gateway_observability = _check_gateway_observability(
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
    dataflow = _mapping(snapshots.get("runtime_dataflow"))
    run_plan = _mapping(dataflow.get("run_plan"))
    product_topics = {
        str(topic) for topic in run_plan.get("required_topics") or () if str(topic)
    } or set(_topic_index(dataflow))
    localization = _check_localization(
        _mapping(snapshots.get("localization_status")),
        mode,
        localization_required=TOPICS.odometry in product_topics,
        blockers=blockers,
        advisories=advisories,
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

    checks = {
        "gateway_contract": gateway_contract,
        "runtime_mode": runtime_mode,
        "gateway_observability": gateway_observability,
        "readiness": readiness,
        "localization": localization,
        "navigation": navigation,
        "real_runtime_evidence": real_runtime_evidence,
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
        "runtime_contract": gateway_observability.get("runtime_contract"),
        "blockers": blockers,
        "advisories": advisories,
        "checks": checks,
        "validated_from": {name: GATEWAY_ACCEPTANCE_ENDPOINTS[name] for name in GATEWAY_ACCEPTANCE_ENDPOINTS},
        "ts": time.time(),
    }
