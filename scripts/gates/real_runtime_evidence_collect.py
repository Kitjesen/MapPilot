#!/usr/bin/env python3
# ruff: noqa: S310 - Gateway URL is an operator-supplied HTTP endpoint.
"""Collect read-only real-robot runtime evidence.

This script does not publish goals, cmd_vel, or any robot-control topic. It
polls Gateway's Product runtime read-only dataflow endpoints.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]


def _ensure_import_path() -> None:
    for candidate in (ROOT / "src", ROOT):
        path = str(candidate)
        if path not in sys.path:
            sys.path.insert(0, path)


_ensure_import_path()

from diagnostics.field.evidence import (  # noqa: E402
    REAL_HARDWARE_COMMAND_SINK,
    REAL_RUNTIME_COLLECTOR_NAME,
    REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED,
    REAL_RUNTIME_EVIDENCE_REPORT_SCHEMA,
    real_runtime_evidence_payload,
    runtime_data_flow_stage_required,
    runtime_data_flow_stage_signals,
    validate_real_runtime_evidence,
)
from runtime.runtime_interface import (  # noqa: E402
    FRAME_LINKS,
    TOPICS,
    resolved_runtime_data_flow,
    runtime_data_flow_topics,
)

OBSERVED_TOPICS = runtime_data_flow_topics(REAL_RUNTIME_CONTRACT)


def _topic_entry(topic_evidence: Mapping[str, Any], topic: str) -> Mapping[str, Any]:
    entry = topic_evidence.get(topic)
    return entry if isinstance(entry, Mapping) else {}


def _positive_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0


def _topic_observed(
    topic_evidence: Mapping[str, Any],
    topic: str,
    *,
    allow_graph: bool = False,
    require_nonempty: bool = False,
    require_nonzero_cmd: bool = False,
) -> bool:
    entry = _topic_entry(topic_evidence, topic)
    if require_nonzero_cmd:
        return _positive_number(entry.get("nonzero_samples")) or _positive_number(entry.get("max_norm"))
    if require_nonempty:
        return any(_positive_number(entry.get(key)) for key in ("nonempty_samples", "max_poses", "points", "cells"))
    if _positive_number(entry.get("samples")) or entry.get("ok") is True:
        return True
    return allow_graph and entry.get("graph_exists") is True


def build_frame_evidence(
    frame_samples: Mapping[str, int],
    frame_errors: Mapping[str, str] | None = None,
) -> dict[str, dict[str, Any]]:
    errors = frame_errors or {}
    evidence: dict[str, dict[str, Any]] = {}
    for name, link in FRAME_LINKS.items():
        samples = int(frame_samples.get(name, 0))
        entry: dict[str, Any] = {
            "ok": samples > 0,
            "parent": link.parent,
            "child": link.child,
            "samples": samples,
        }
        if name in errors and samples <= 0:
            entry["error"] = errors[name]
        evidence[name] = entry
    return evidence


def build_data_flow_evidence(
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
    data_source: str = REAL_RUNTIME_CONTRACT,
) -> dict[str, dict[str, Any]]:
    evidence: dict[str, dict[str, Any]] = {}
    for stage in resolved_runtime_data_flow(data_source):
        signals = runtime_data_flow_stage_signals(
            stage.name,
            topic_evidence,
            hardware_boundary,
        )
        ok = all(signals.values())
        required = runtime_data_flow_stage_required(
            stage.name,
            topic_evidence,
            hardware_boundary,
        )
        evidence[stage.name] = {
            "ok": ok,
            "required": required,
            "observed_inputs": _observed_tokens(
                stage.inputs,
                topic_evidence,
                hardware_boundary,
            ),
            "missing_inputs": _missing_tokens(
                stage.inputs,
                topic_evidence,
                hardware_boundary,
            ),
            "observed_outputs": _observed_tokens(
                stage.outputs,
                topic_evidence,
                hardware_boundary,
            ),
            "missing_outputs": _missing_tokens(
                stage.outputs,
                topic_evidence,
                hardware_boundary,
            ),
            "observed_signals": [name for name, observed in signals.items() if observed],
            "missing_signals": [name for name, observed in signals.items() if not observed],
        }
    return evidence


def _observed_tokens(
    tokens: Sequence[str],
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
) -> list[str]:
    return [
        str(token) for token in tokens if _runtime_flow_token_observed(str(token), topic_evidence, hardware_boundary)
    ]


def _missing_tokens(
    tokens: Sequence[str],
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
) -> list[str]:
    return [
        str(token)
        for token in tokens
        if not _runtime_flow_token_observed(str(token), topic_evidence, hardware_boundary)
    ]


def _runtime_flow_token_observed(
    token: str,
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
) -> bool:
    if token.startswith("/"):
        return _topic_observed(topic_evidence, token, allow_graph=True)
    if token == REAL_HARDWARE_COMMAND_SINK:
        return hardware_boundary.get("hardware_command_route_observed") is True
    if token == hardware_boundary.get("command_sink"):
        return hardware_boundary.get("hardware_command_route_observed") is True
    return False


def _topic_summary(topic_evidence: Mapping[str, Any], topic: str) -> dict[str, Any]:
    entry = dict(_topic_entry(topic_evidence, topic))
    entry.setdefault("ok", _topic_observed(topic_evidence, topic))
    entry.setdefault("samples", 0)
    return entry


def _motion_delta_m(odom_positions: Sequence[Sequence[float]]) -> float:
    if len(odom_positions) < 2:
        return 0.0
    start = odom_positions[0]
    end = odom_positions[-1]
    return math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))


def _matches_expected_subscriber(name: str, expected: Sequence[str]) -> bool:
    normalized = name.lower().strip("/")
    for candidate in expected:
        expected_name = candidate.lower().strip("/")
        if expected_name and expected_name == normalized:
            return True
    return False


def build_hardware_boundary(
    command_subscribers: Sequence[str],
    expected_command_subscribers: Sequence[str] = (),
) -> dict[str, Any]:
    subscribers = sorted(dict.fromkeys(str(item) for item in command_subscribers if item))
    hardware_route_observed = any(
        name.lower().strip("/").split(".", 1)[0] == REAL_HARDWARE_COMMAND_SINK
        or _matches_expected_subscriber(name, expected_command_subscribers)
        for name in subscribers
    )
    return {
        "command_sink": REAL_HARDWARE_COMMAND_SINK,
        "hardware_command_route_observed": hardware_route_observed,
        "command_subscribers": subscribers,
        "expected_command_subscribers": list(expected_command_subscribers),
    }


def build_real_runtime_report(
    *,
    topic_evidence: Mapping[str, Any],
    frame_samples: Mapping[str, int],
    command_subscribers: Sequence[str],
    duration_sec: float,
    odom_positions: Sequence[Sequence[float]] = (),
    frame_errors: Mapping[str, str] | None = None,
    expected_command_subscribers: Sequence[str] = (),
    min_motion_m: float = 0.05,
) -> dict[str, Any]:
    hardware_boundary = build_hardware_boundary(
        command_subscribers,
        expected_command_subscribers=expected_command_subscribers,
    )
    frame_evidence = build_frame_evidence(frame_samples, frame_errors)
    data_flow_evidence = build_data_flow_evidence(topic_evidence, hardware_boundary)
    motion_delta = _motion_delta_m(odom_positions)
    cmd_vel = _topic_summary(topic_evidence, TOPICS.cmd_vel)
    global_path = _topic_summary(topic_evidence, TOPICS.global_path)
    local_path = _topic_summary(topic_evidence, TOPICS.local_path)
    command_nonzero = _topic_observed(
        topic_evidence,
        TOPICS.cmd_vel,
        require_nonzero_cmd=True,
    )
    cmd_sent_to_hardware = command_nonzero and hardware_boundary["hardware_command_route_observed"] is True

    return {
        "schema_version": REAL_RUNTIME_EVIDENCE_REPORT_SCHEMA,
        "collector": {
            "name": REAL_RUNTIME_COLLECTOR_NAME,
            "read_only": True,
            "duration_sec": float(duration_sec),
            "control_topics_published": list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED),
        },
        "simulation_only": False,
        "real_robot_motion": motion_delta >= float(min_motion_m),
        "cmd_vel_sent_to_hardware": cmd_sent_to_hardware,
        "motion": {
            "odom_delta_m": motion_delta,
            "min_motion_m": float(min_motion_m),
            "odom_position_samples": len(odom_positions),
        },
        "outputs": {
            "global_path_count": int(global_path.get("samples", 0)),
            "local_path_count": int(local_path.get("samples", 0)),
            "nav_cmd_vel_nonzero": int(cmd_vel.get("nonzero_samples", 0)),
        },
        "paths": {
            TOPICS.global_path: global_path,
            TOPICS.local_path: local_path,
        },
        "cmd_vel": cmd_vel,
        "hardware_boundary": hardware_boundary,
        "runtime_contract": {
            "name": REAL_RUNTIME_CONTRACT,
            "ok": True,
            "topic_evidence": {topic: _topic_summary(topic_evidence, topic) for topic in OBSERVED_TOPICS},
            "frame_evidence": frame_evidence,
            "data_flow_evidence": data_flow_evidence,
        },
    }


def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


class GatewayCollectionError(RuntimeError):
    """Raised when the read-only Gateway evidence path is unavailable."""


def _gateway_url(base_url: str, path: str, params: Mapping[str, Any] | None = None) -> str:
    base = str(base_url or "").rstrip("/")
    suffix = path if path.startswith("/") else f"/{path}"
    url = f"{base}{suffix}"
    if params:
        url = f"{url}?{urlencode(params)}"
    return url


def _fetch_gateway_json(
    base_url: str,
    path: str,
    *,
    timeout_sec: float,
    params: Mapping[str, Any] | None = None,
) -> Any:
    url = _gateway_url(base_url, path, params)
    request = Request(url, headers={"Accept": "application/json"})
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            raw = response.read().decode("utf-8")
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        raise GatewayCollectionError(f"Gateway endpoint unavailable: {url}: {exc}") from exc
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        raise GatewayCollectionError(f"Gateway endpoint returned invalid JSON: {url}: {exc}") from exc


def _mapping_payload(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _nested_mapping(value: Mapping[str, Any], *keys: str) -> Mapping[str, Any]:
    current: Any = value
    for key in keys:
        current = _mapping_payload(current).get(key)
    return current if isinstance(current, Mapping) else {}


def _numeric_payload(value: Mapping[str, Any], *keys: str) -> float | None:
    for key in keys:
        parsed = _safe_float(value.get(key))
        if parsed is not None:
            return parsed
    return None


def _count_payload(value: Any, *keys: str) -> int | None:
    payload = _mapping_payload(value)
    for key in keys:
        raw = payload.get(key)
        parsed = _safe_float(raw)
        if parsed is not None:
            return int(parsed)
        try:
            return len(raw)
        except TypeError:
            continue
    return None


def _position_payload(value: Any) -> tuple[float, float, float] | None:
    payload = _mapping_payload(value)
    x = _safe_float(payload.get("x"))
    y = _safe_float(payload.get("y"))
    z = _safe_float(payload.get("z", 0.0))
    if x is not None and y is not None:
        return (x, y, z or 0.0)
    position = _mapping_payload(payload.get("position"))
    x = _safe_float(position.get("x"))
    y = _safe_float(position.get("y"))
    z = _safe_float(position.get("z", 0.0))
    if x is not None and y is not None:
        return (x, y, z or 0.0)
    return None


def _record_gateway_sample(
    topic_evidence: dict[str, dict[str, Any]],
    odom_positions: list[tuple[float, float, float]],
    topic: str,
    *,
    sample_time_sec: float,
    min_cmd_vel_norm: float,
    frame_id: str | None = None,
    payload: Mapping[str, Any] | None = None,
    inferred_nonempty: bool = False,
    source: str = "gateway",
) -> None:
    entry = topic_evidence.setdefault(topic, {"ok": False, "samples": 0})
    entry["samples"] = int(entry.get("samples", 0)) + 1
    entry["ok"] = True
    entry["gateway_source"] = source
    first_seen = _safe_float(entry.get("first_seen_sec"))
    if first_seen is None:
        entry["first_seen_sec"] = sample_time_sec
        first_seen = sample_time_sec
    entry["last_seen_sec"] = sample_time_sec
    entry["sample_span_sec"] = max(0.0, sample_time_sec - first_seen)
    if frame_id:
        entry["frame_id"] = str(frame_id)

    data = _mapping_payload(payload)
    if data:
        if data.get("frame_id"):
            entry["frame_id"] = str(data["frame_id"])
        if data.get("data") is not None:
            entry["data"] = str(data["data"])
        if data.get("state") is not None and topic == TOPICS.localization_health:
            if data.get("fitness") is not None:
                quality = _numeric_payload(data, "fitness")
                suffix_key = "fitness"
            else:
                quality = _numeric_payload(
                    data,
                    "quality",
                    "confidence",
                    "value",
                    "icp_quality",
                    "icp_fitness",
                )
                suffix_key = "quality"
            suffix = f"|{suffix_key}={quality}" if quality is not None else ""
            entry["data"] = f"{data['state']}{suffix}"
            entry["quality_kind"] = suffix_key
        value = _numeric_payload(
            data,
            "value",
            "quality",
            "confidence",
            "fitness",
            "icp_quality",
            "icp_fitness",
        )
        if value is not None:
            entry["value"] = value
        if data.get("quality_kind"):
            entry["quality_kind"] = str(data["quality_kind"])
        points = _count_payload(data, "points", "count")
        if points is not None:
            entry["points"] = max(int(entry.get("points", 0)), points)
        cells = _count_payload(data, "cells")
        if cells is not None:
            entry["cells"] = max(int(entry.get("cells", 0)), cells)
        poses = _count_payload(data, "max_poses", "poses", "path")
        if poses is not None:
            entry["max_poses"] = max(int(entry.get("max_poses", 0)), poses)
            if poses > 0:
                entry["nonempty_samples"] = int(entry.get("nonempty_samples", 0)) + 1
        cmd_norm = _numeric_payload(data, "cmd_norm", "max_norm")
        if cmd_norm is not None:
            entry["max_norm"] = max(float(entry.get("max_norm", 0.0)), cmd_norm)
            if cmd_norm >= min_cmd_vel_norm:
                entry["nonzero_samples"] = int(entry.get("nonzero_samples", 0)) + 1
        odom_position = _position_payload(data.get("position") or data)
        if topic == TOPICS.odometry and odom_position is not None:
            odom_positions.append(odom_position)

    if inferred_nonempty:
        if topic in {TOPICS.global_path, TOPICS.local_path}:
            entry["max_poses"] = max(int(entry.get("max_poses", 0)), 1)
            entry["nonempty_samples"] = int(entry.get("nonempty_samples", 0)) + 1
            entry["nonempty_inferred_from"] = source
        elif "cloud" in topic or "scan" in topic:
            entry["points"] = max(int(entry.get("points", 0)), 1)
            entry["nonempty_samples"] = int(entry.get("nonempty_samples", 0)) + 1
            entry["nonempty_inferred_from"] = source


def _latest_summary_from_ports(topic_summary: Mapping[str, Any]) -> Mapping[str, Any]:
    candidates = (
        _mapping_payload(topic_summary.get("observability")).get("module_port_candidates")
        or _mapping_payload(topic_summary.get("inspection")).get("module_stats")
        or ()
    )
    for candidate in candidates:
        summary = _mapping_payload(_mapping_payload(candidate).get("latest_summary"))
        if summary:
            return summary
    return {}


def _topic_live(topic_summary: Mapping[str, Any]) -> bool:
    observability = _mapping_payload(topic_summary.get("observability"))
    inspection = _mapping_payload(topic_summary.get("inspection"))
    if (
        observability.get("has_fresh_module_sample") is True
        or observability.get("live_module_samples") is True
        or inspection.get("live") is True
    ):
        return True
    for port in observability.get("module_port_candidates") or ():
        stats = _mapping_payload(port)
        msg_count = _safe_float(stats.get("msg_count")) or 0.0
        rate_hz = _safe_float(stats.get("rate_hz")) or 0.0
        stale_ms = _safe_float(stats.get("stale_ms"))
        if msg_count <= 0 and rate_hz <= 0.0:
            continue
        if stale_ms is None or stale_ms <= 2000.0:
            return True
    return False


def _record_gateway_dataflow(
    topic_evidence: dict[str, dict[str, Any]],
    odom_positions: list[tuple[float, float, float]],
    dataflow: Mapping[str, Any],
    *,
    sample_time_sec: float,
    min_cmd_vel_norm: float,
    command_active: bool,
) -> None:
    topic_summaries = {
        str(item.get("topic")): item
        for item in dataflow.get("topics") or ()
        if isinstance(item, Mapping) and item.get("topic")
    }
    for topic in OBSERVED_TOPICS:
        summary = _mapping_payload(topic_summaries.get(topic))
        if not summary:
            continue
        entry = topic_evidence.setdefault(topic, {"ok": False, "samples": 0})
        observability = _mapping_payload(summary.get("observability"))
        inspection = _mapping_payload(summary.get("inspection"))
        entry["graph_exists"] = bool(
            observability.get("observable")
            or inspection.get("observable")
            or observability.get("module_port_candidates")
        )
        entry["gateway_observable"] = entry["graph_exists"]
        if summary.get("default_frame_id") and "frame_id" not in entry:
            entry["frame_id"] = str(summary["default_frame_id"])
        latest_payload = _mapping_payload(inspection.get("latest_payload"))
        latest_summary = _latest_summary_from_ports(summary)
        live = _topic_live(summary)
        if not live and not latest_payload and not latest_summary:
            continue
        payload = latest_payload or latest_summary
        if not payload and live:
            payload = {"frame_id": summary.get("default_frame_id")}
        inferred_nonempty = bool(
            live
            and topic
            in {
                TOPICS.lidar_scan,
                TOPICS.registered_cloud,
                TOPICS.map_cloud,
            }
        )
        _record_gateway_sample(
            topic_evidence,
            odom_positions,
            topic,
            sample_time_sec=sample_time_sec,
            min_cmd_vel_norm=min_cmd_vel_norm,
            frame_id=summary.get("default_frame_id"),
            payload=payload,
            inferred_nonempty=inferred_nonempty,
            source="gateway_runtime_dataflow",
        )
        if topic == TOPICS.cmd_vel and command_active:
            entry = topic_evidence[topic]
            entry["max_norm"] = max(
                float(entry.get("max_norm", 0.0)),
                float(min_cmd_vel_norm),
            )
            entry["nonzero_samples"] = int(entry.get("nonzero_samples", 0)) + 1
            entry["cmd_vel_nonzero_source"] = "gateway_navigation_status"


def _record_gateway_rest_payloads(
    topic_evidence: dict[str, dict[str, Any]],
    odom_positions: list[tuple[float, float, float]],
    payloads: Mapping[str, Any],
    *,
    sample_time_sec: float,
    min_cmd_vel_norm: float,
) -> None:
    state = _mapping_payload(payloads.get("state"))
    odometry = _mapping_payload(state.get("odometry"))
    if odometry:
        _record_gateway_sample(
            topic_evidence,
            odom_positions,
            TOPICS.odometry,
            sample_time_sec=sample_time_sec,
            min_cmd_vel_norm=min_cmd_vel_norm,
            frame_id=str(odometry.get("frame_id") or "odom"),
            payload=odometry,
            source="gateway_state",
        )

    path = _mapping_payload(payloads.get("path"))
    if path:
        _record_gateway_sample(
            topic_evidence,
            odom_positions,
            TOPICS.global_path,
            sample_time_sec=sample_time_sec,
            min_cmd_vel_norm=min_cmd_vel_norm,
            frame_id=str(path.get("frame_id") or "map"),
            payload=path,
            source="gateway_path",
        )

    map_points = _mapping_payload(payloads.get("map_points"))
    if map_points:
        _record_gateway_sample(
            topic_evidence,
            odom_positions,
            TOPICS.map_cloud,
            sample_time_sec=sample_time_sec,
            min_cmd_vel_norm=min_cmd_vel_norm,
            frame_id=str(map_points.get("frame_id") or "map"),
            payload=map_points,
            source="gateway_map_points",
        )

    localization = _mapping_payload(payloads.get("localization"))
    if localization:
        lidar_hz = _numeric_payload(localization, "lidar_input_hz")
        if lidar_hz is not None and lidar_hz > 0.0:
            _record_gateway_sample(
                topic_evidence,
                odom_positions,
                TOPICS.lidar_scan,
                sample_time_sec=sample_time_sec,
                min_cmd_vel_norm=min_cmd_vel_norm,
                frame_id="lidar_link",
                payload={"value": lidar_hz, "count": 1},
                inferred_nonempty=True,
                source="gateway_localization_status",
            )
            topic_evidence[TOPICS.lidar_scan]["rate_hz"] = lidar_hz

        imu_hz = _numeric_payload(localization, "imu_input_hz")
        if imu_hz is not None and imu_hz > 0.0:
            _record_gateway_sample(
                topic_evidence,
                odom_positions,
                TOPICS.imu,
                sample_time_sec=sample_time_sec,
                min_cmd_vel_norm=min_cmd_vel_norm,
                frame_id="lidar_link",
                payload={"value": imu_hz},
                source="gateway_localization_status",
            )
            topic_evidence[TOPICS.imu]["rate_hz"] = imu_hz

        state_value = (
            localization.get("reported_state")
            or localization.get("state")
            or localization.get("localizer_health")
            or "TRACKING"
        )
        if str(state_value).strip().lower() == "ready":
            state_value = "LOCKED"
        fitness = _numeric_payload(
            localization,
            "localizer_health_fitness",
            "icp_fitness",
        )
        confidence = _numeric_payload(
            localization,
            "confidence",
            "localization_quality",
            "icp_quality",
        )
        health_payload = {"state": str(state_value).upper()}
        if fitness is not None:
            health_payload["fitness"] = fitness
            quality_payload = {"value": fitness, "quality_kind": "fitness"}
        elif confidence is not None:
            health_payload["quality"] = confidence
            health_payload["quality_kind"] = "confidence"
            quality_payload = {"value": confidence, "quality_kind": "confidence"}
        else:
            quality_payload = None
        _record_gateway_sample(
            topic_evidence,
            odom_positions,
            TOPICS.localization_health,
            sample_time_sec=sample_time_sec,
            min_cmd_vel_norm=min_cmd_vel_norm,
            frame_id="body",
            payload=health_payload,
            source="gateway_localization_status",
        )
        if quality_payload is not None:
            _record_gateway_sample(
                topic_evidence,
                odom_positions,
                TOPICS.localization_quality,
                sample_time_sec=sample_time_sec,
                min_cmd_vel_norm=min_cmd_vel_norm,
                frame_id="body",
                payload=quality_payload,
                source="gateway_localization_status",
            )


def _gateway_command_active(navigation_status: Mapping[str, Any]) -> bool:
    control = _mapping_payload(navigation_status.get("control"))
    active_source = (
        str(control.get("active_cmd_source") or _nested_mapping(control, "active_source").get("name") or "")
        .strip()
        .lower()
    )
    if active_source and active_source != "none":
        return True
    sources = _mapping_payload(control.get("sources"))
    for source in sources.values():
        source_payload = _mapping_payload(source)
        if source_payload.get("active") is True:
            return True
    return False


def _fresh_gateway_port(stats: Mapping[str, Any]) -> bool:
    msg_count = _safe_float(stats.get("msg_count")) or 0.0
    rate_hz = _safe_float(stats.get("rate_hz")) or 0.0
    stale_ms = _safe_float(stats.get("stale_ms"))
    if msg_count <= 0.0 and rate_hz <= 0.0:
        return False
    return stale_ms is None or stale_ms <= 2000.0


def _gateway_topic_summaries(dataflow: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    return {
        str(item.get("topic")): item
        for item in dataflow.get("topics") or ()
        if isinstance(item, Mapping) and item.get("topic")
    }


def _gateway_command_subscribers(dataflow: Mapping[str, Any]) -> list[str]:
    """Return live hardware consumers of ``/nav/cmd_vel``.

    Gateway's runtime boundary and REST control interfaces are declarations:
    they prove what the graph intends to support, not that a field command sink
    is running.  The real-runtime gate must only accept dynamic module-port
    evidence for an actual hardware-looking consumer.
    """

    candidates: list[str] = []
    cmd_summary = _mapping_payload(_gateway_topic_summaries(dataflow).get(TOPICS.cmd_vel))
    observability = _mapping_payload(cmd_summary.get("observability"))
    inspection = _mapping_payload(cmd_summary.get("inspection"))
    port_candidates = list(observability.get("module_port_candidates") or ())
    port_candidates.extend(inspection.get("module_stats") or ())

    module_ports = _mapping_payload(dataflow.get("module_ports"))
    for module_name, module_payload in module_ports.items():
        ports_in = _mapping_payload(_mapping_payload(module_payload).get("ports_in"))
        cmd_port = _mapping_payload(ports_in.get("cmd_vel"))
        if cmd_port:
            port_candidates.append(
                {
                    **cmd_port,
                    "module": module_name,
                    "port": "cmd_vel",
                    "direction": "in",
                }
            )

    for candidate in port_candidates:
        stats = _mapping_payload(candidate)
        module = str(stats.get("module") or "").strip()
        port = str(stats.get("port") or "").strip()
        direction = str(stats.get("direction") or "").strip().lower()
        if not module or direction != "in":
            continue
        if not _fresh_gateway_port(stats):
            continue
        endpoint = f"{module}.{port}" if port else module
        if module.lower() == REAL_HARDWARE_COMMAND_SINK:
            candidates.append(endpoint)
    return sorted(dict.fromkeys(candidates))


def _gateway_frame_samples(dataflow: Mapping[str, Any]) -> tuple[dict[str, int], dict[str, str]]:
    frame_samples = {name: 0 for name in FRAME_LINKS}
    frame_errors: dict[str, str] = {}
    runtime_boundary = _mapping_payload(dataflow.get("runtime_boundary"))
    boundary_ok = runtime_boundary.get("ok") is True
    declared_links = _mapping_payload(runtime_boundary.get("frame_links"))
    for name, link in FRAME_LINKS.items():
        declared = _mapping_payload(declared_links.get(name))
        if boundary_ok or (declared.get("parent") == link.parent and declared.get("child") == link.child):
            frame_samples[name] = 1
        else:
            frame_errors[name] = "Gateway runtime boundary did not prove frame link"
    return frame_samples, frame_errors


def _collect_gateway_payloads(args: argparse.Namespace) -> dict[str, Any]:
    base_url = args.gateway_url
    timeout_sec = float(args.gateway_timeout_sec)
    payloads: dict[str, Any] = {
        "dataflow": _fetch_gateway_json(
            base_url,
            "/api/v1/runtime/dataflow",
            timeout_sec=timeout_sec,
        )
    }
    optional_endpoints = {
        "state": "/api/v1/state",
        "path": "/api/v1/path",
        "map_points": "/api/v1/map/points",
        "localization": "/api/v1/localization/status",
        "navigation": "/api/v1/navigation/status",
    }
    errors = []
    for name, path in optional_endpoints.items():
        try:
            payloads[name] = _fetch_gateway_json(
                base_url,
                path,
                timeout_sec=timeout_sec,
            )
        except GatewayCollectionError as exc:
            errors.append(str(exc))
    if errors:
        payloads["optional_errors"] = errors
    return payloads


def run_gateway_collect(args: argparse.Namespace) -> dict[str, Any]:
    topic_evidence: dict[str, dict[str, Any]] = {
        topic: {"ok": False, "samples": 0, "graph_exists": False} for topic in OBSERVED_TOPICS
    }
    odom_positions: list[tuple[float, float, float]] = []
    command_subscribers: list[str] = []
    frame_samples = {name: 0 for name in FRAME_LINKS}
    frame_errors: dict[str, str] = {}
    collection_errors: list[str] = []

    started_at = time.monotonic()
    duration_sec = max(float(args.duration_sec), 0.0)
    deadline = started_at + duration_sec
    poll_sec = max(float(args.gateway_poll_sec), 0.05)

    while True:
        sample_time_sec = min(
            max(0.0, time.monotonic() - started_at),
            duration_sec,
        )
        payloads = _collect_gateway_payloads(args)
        dataflow = _mapping_payload(payloads.get("dataflow"))
        navigation = _mapping_payload(payloads.get("navigation"))
        command_active = _gateway_command_active(navigation)
        _record_gateway_dataflow(
            topic_evidence,
            odom_positions,
            dataflow,
            sample_time_sec=sample_time_sec,
            min_cmd_vel_norm=args.min_cmd_vel_norm,
            command_active=command_active,
        )
        _record_gateway_rest_payloads(
            topic_evidence,
            odom_positions,
            payloads,
            sample_time_sec=sample_time_sec,
            min_cmd_vel_norm=args.min_cmd_vel_norm,
        )
        command_subscribers = _gateway_command_subscribers(dataflow)
        sample_frames, sample_frame_errors = _gateway_frame_samples(dataflow)
        for name, count in sample_frames.items():
            frame_samples[name] = frame_samples.get(name, 0) + count
        frame_errors.update(sample_frame_errors)
        collection_errors.extend(payloads.get("optional_errors") or ())

        if time.monotonic() >= deadline:
            break
        time.sleep(min(poll_sec, max(0.0, deadline - time.monotonic())))

    report = build_real_runtime_report(
        topic_evidence=topic_evidence,
        frame_samples=frame_samples,
        frame_errors=frame_errors,
        command_subscribers=command_subscribers,
        expected_command_subscribers=args.expected_command_subscriber,
        duration_sec=args.duration_sec,
        odom_positions=odom_positions,
        min_motion_m=args.min_motion_m,
    )
    report["collector"]["source"] = "gateway"
    report["collector"]["gateway_url"] = args.gateway_url
    if collection_errors:
        report["warnings"] = sorted(dict.fromkeys(collection_errors))
    return report


def run_collect(args: argparse.Namespace) -> dict[str, Any]:
    return run_gateway_collect(args)


def build_unavailable_real_runtime_report(
    *,
    duration_sec: float,
    error: str,
    expected_contract: str = REAL_RUNTIME_CONTRACT,
    expected_command_subscribers: Sequence[str] = (),
    min_motion_m: float = 0.05,
) -> dict[str, Any]:
    """Build a failed report that still exposes the full expected evidence shape."""

    report = build_real_runtime_report(
        topic_evidence={
            topic: {
                "ok": False,
                "samples": 0,
                "graph_exists": False,
                "collection_error": error,
            }
            for topic in OBSERVED_TOPICS
        },
        frame_samples={name: 0 for name in FRAME_LINKS},
        frame_errors={name: error for name in FRAME_LINKS},
        command_subscribers=(),
        expected_command_subscribers=expected_command_subscribers,
        duration_sec=duration_sec,
        odom_positions=(),
        min_motion_m=min_motion_m,
    )
    report["errors"] = [error]
    report["runtime_contract"]["name"] = expected_contract
    report["runtime_contract"]["ok"] = False
    report["runtime_contract"]["collection_available"] = False
    report["runtime_contract"]["collection_error"] = error
    return report


def _validation_payload(report: Mapping[str, Any], expected_contract: str) -> dict[str, Any]:
    result = validate_real_runtime_evidence(report, expected_contract)
    return real_runtime_evidence_payload(result, expected_contract)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect read-only real-robot runtime evidence.",
    )
    parser.add_argument("--gateway-url", default="http://127.0.0.1:5050")
    parser.add_argument("--gateway-timeout-sec", type=float, default=2.0)
    parser.add_argument("--gateway-poll-sec", type=float, default=0.25)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--min-motion-m", type=float, default=0.05)
    parser.add_argument("--min-cmd-vel-norm", type=float, default=0.01)
    parser.add_argument(
        "--expected-command-subscriber",
        action="append",
        default=[],
        help="Exact observed command subscriber accepted as the hardware route.",
    )
    parser.add_argument("--expected-contract", default=REAL_RUNTIME_CONTRACT)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--no-validate", action="store_true")
    args = parser.parse_args(argv)
    if args.expected_contract != REAL_RUNTIME_CONTRACT:
        parser.error(f"real runtime evidence only supports expected contract {REAL_RUNTIME_CONTRACT}")
    return args


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    collection_error = False
    try:
        report = run_collect(args)
    except GatewayCollectionError as exc:
        collection_error = True
        report = build_unavailable_real_runtime_report(
            duration_sec=args.duration_sec,
            error=f"Gateway collection unavailable: {exc}",
            expected_contract=args.expected_contract,
            expected_command_subscribers=args.expected_command_subscriber,
            min_motion_m=args.min_motion_m,
        )
        report["collector"]["source"] = "gateway"
        report["collector"]["gateway_url"] = args.gateway_url
    validation = None
    if not args.no_validate:
        validation = _validation_payload(report, args.expected_contract)
        report["runtime_evidence"] = validation

    if args.json_out:
        _write_json(args.json_out, report)

    if args.json or not args.json_out:
        print(json.dumps(report, indent=2))

    if validation is not None and validation["ok"] is not True:
        return 2
    if collection_error:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
