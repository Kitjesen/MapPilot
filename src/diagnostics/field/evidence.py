"""Shared runtime evidence validation for simulation, replay, and real gates."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from numbers import Real
from typing import Any, Mapping, Sequence

from runtime.runtime_interface import (
    FRAME_LINKS,
    REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
    TOPIC_ALLOWED_FRAME_IDS,
    TOPICS,
    expand_frame_id_aliases,
    normalize_algorithm_interface_contract,
    normalize_frame_id,
    resolved_runtime_data_flow,
    runtime_algorithm_interface_contract,
    runtime_data_flow_topics,
    runtime_frames_contract,
    runtime_required_topic_frame_ids,
    runtime_stage_algorithm_interface_contract,
    runtime_topic_allowed_frame_contract,
    runtime_topic_allowed_frame_ids,
    runtime_topic_default_frame_id,
    runtime_topic_default_frame_ids,
)

REAL_HARDWARE_COMMAND_SINK = "driver"
REAL_RUNTIME_EVIDENCE_REPORT_SCHEMA = "lingtu.real_runtime_evidence.report.v1"
REAL_RUNTIME_EVIDENCE_VALIDATION_SCHEMA = "lingtu.real_runtime_evidence.validation.v1"
REAL_RUNTIME_COLLECTOR_NAME = "real_runtime_evidence_collect"
REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED: tuple[str, ...] = ()
REAL_RUNTIME_LOCALIZATION_HEALTHY_STATES = ("LOCKED", "RECOVERED", "TRACKING")
REAL_RUNTIME_LOCALIZATION_QUALITY_MIN_EXCLUSIVE = 0.0
REAL_RUNTIME_LOCALIZATION_QUALITY_MAX_EXCLUSIVE = 0.5
REAL_RUNTIME_LOCALIZATION_CONFIDENCE_MIN_INCLUSIVE = 0.5
REAL_RUNTIME_LOCALIZATION_CONFIDENCE_MAX_INCLUSIVE = 1.0
REAL_RUNTIME_LIVE_TOPIC_MAX_AGE_SEC = 2.0
OPTIONAL_RUNTIME_DATA_FLOW_STAGES = frozenset({"dynamic_obstacle_gate"})
REAL_RUNTIME_LIVE_TOPIC_FRESHNESS_TOPICS = (
    TOPICS.lidar_scan,
    TOPICS.imu,
    TOPICS.odometry,
    TOPICS.registered_cloud,
    TOPICS.localization_health,
    TOPICS.localization_quality,
    TOPICS.local_path,
    TOPICS.cmd_vel,
)
REAL_RUNTIME_TEMPORAL_FLOW_STAGES = {
    "slam_or_relayed_localization_map": (
        (TOPICS.lidar_scan, TOPICS.imu),
        (TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
    ),
    "local_planning_and_following": (
        (TOPICS.global_path,),
        (TOPICS.local_path, TOPICS.cmd_vel),
    ),
}


@dataclass(frozen=True)
class RuntimeEvidenceResult:
    """Result of checking runtime evidence requirements before system start."""

    ok: bool
    blockers: tuple[str, ...]


def real_runtime_evidence_payload(
    result: RuntimeEvidenceResult,
    expected_contract: str,
    *,
    report: Mapping[str, Any] | None = None,
    paths_required: bool = True,
    command_required: bool = True,
    frame_links_required: bool = True,
    data_flow_required: bool = True,
    hardware_boundary_required: bool = True,
) -> dict[str, Any]:
    checked_runtime_topics: list[str] = []
    checked_data_flow_stages: list[str] = []
    checked_algorithm_interfaces = runtime_algorithm_interface_contract()
    checked_runtime_data_flow_stage_algorithm_interfaces = runtime_stage_algorithm_interface_contract()
    checked_runtime_data_flow_evidence: dict[str, dict[str, Any]] = {}
    checked_frame_link_evidence: dict[str, dict[str, Any]] = {}
    checked_frames: dict[str, Any] = runtime_frames_contract()
    checked_topic_default_frame_ids: dict[str, str] = {}
    checked_topic_allowed_frame_ids: dict[str, list[str]] = {}
    checked_required_topic_frame_ids: list[str] = []
    checked_required_topic_default_frame_ids: dict[str, str] = {}
    observed_required_topic_frame_ids: dict[str, str] = {}
    checked_required_topic_frame_report: dict[str, dict[str, Any]] = {}
    checked_required_topic_allowed_frame_ids: dict[str, list[str]] = {}
    checked_real_motion_evidence: dict[str, Any] = {}
    checked_hardware_boundary_evidence: dict[str, Any] = {}
    checked_required_topic_sample_windows: dict[str, dict[str, Any]] = {}
    checked_endpoint_input_sample_windows: dict[str, dict[str, Any]] = {}
    checked_localization_health_evidence: dict[str, Any] = {}
    checked_data_flow_temporal_order: dict[str, dict[str, Any]] = {}
    checked_live_topic_freshness: dict[str, dict[str, Any]] = {}
    checked_collector_contract: dict[str, Any] = {}
    try:
        checked_runtime_topics = list(runtime_data_flow_topics(expected_contract))
    except ValueError:
        checked_runtime_topics = []
    try:
        checked_data_flow_stages = [stage.name for stage in resolved_runtime_data_flow(expected_contract)]
    except ValueError:
        checked_data_flow_stages = []
    checked_required_topic_frame_ids = list(runtime_required_topic_frame_ids(expected_contract))
    try:
        default_frames = runtime_topic_default_frame_ids(expected_contract)
        checked_topic_default_frame_ids = dict(default_frames)
        checked_required_topic_default_frame_ids = {
            topic: default_frames[topic] for topic in checked_required_topic_frame_ids if topic in default_frames
        }
    except ValueError:
        checked_topic_default_frame_ids = {}
        checked_required_topic_default_frame_ids = {}
    try:
        allowed_frames = runtime_topic_allowed_frame_ids(expected_contract)
        checked_topic_allowed_frame_ids = {topic: list(frames) for topic, frames in allowed_frames.items()}
        checked_required_topic_allowed_frame_ids = {
            topic: list(allowed_frames[topic]) for topic in checked_required_topic_frame_ids if topic in allowed_frames
        }
    except ValueError:
        checked_topic_allowed_frame_ids = {}
        checked_required_topic_allowed_frame_ids = {}
    if report is not None:
        observed_required_topic_frame_ids = _observed_topic_frame_ids(
            report,
            checked_required_topic_frame_ids,
        )
        checked_real_motion_evidence = _real_motion_evidence_payload(report)
        checked_hardware_boundary_evidence = _real_hardware_boundary_evidence_payload(report)
        checked_runtime_data_flow_evidence = _runtime_data_flow_evidence_payload(
            report,
            expected_contract,
        )
        checked_frame_link_evidence = _frame_link_evidence_payload(report)
        checked_required_topic_sample_windows = _required_topic_sample_windows_payload(
            report,
            checked_required_topic_frame_ids,
        )
        checked_endpoint_input_sample_windows = _required_topic_sample_windows_payload(
            report,
            REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
        )
        checked_localization_health_evidence = _localization_health_evidence_payload(report)
        checked_data_flow_temporal_order = _data_flow_temporal_order_payload(
            report,
            None,
            expected_contract,
        )
        checked_live_topic_freshness = _live_topic_freshness_payload(
            report,
            None,
            expected_contract,
        )
        checked_collector_contract = _collector_contract_payload(report)
    checked_required_topic_frame_report = _required_topic_frame_report(
        checked_required_topic_frame_ids,
        checked_required_topic_default_frame_ids,
        observed_required_topic_frame_ids,
        checked_required_topic_allowed_frame_ids,
    )
    return {
        "schema_version": REAL_RUNTIME_EVIDENCE_VALIDATION_SCHEMA,
        "ok": result.ok,
        "expected_contract": expected_contract,
        "paths_required": paths_required,
        "command_required": command_required,
        "frame_links_required": frame_links_required,
        "data_flow_required": data_flow_required,
        "hardware_boundary_required": hardware_boundary_required,
        "checked_runtime_topics": checked_runtime_topics,
        "checked_data_flow_stages": checked_data_flow_stages,
        "checked_algorithm_interfaces": checked_algorithm_interfaces,
        "checked_runtime_data_flow_stage_algorithm_interfaces": (checked_runtime_data_flow_stage_algorithm_interfaces),
        "checked_runtime_data_flow_evidence": checked_runtime_data_flow_evidence,
        "checked_frame_links": list(FRAME_LINKS),
        "checked_frame_link_evidence": checked_frame_link_evidence,
        "checked_frames": checked_frames,
        "checked_topic_default_frame_ids": checked_topic_default_frame_ids,
        "checked_topic_allowed_frame_ids": checked_topic_allowed_frame_ids,
        "checked_required_topic_frame_ids": checked_required_topic_frame_ids,
        "checked_required_topic_default_frame_ids": (checked_required_topic_default_frame_ids),
        "checked_required_topic_allowed_frame_ids": (checked_required_topic_allowed_frame_ids),
        "observed_required_topic_frame_ids": observed_required_topic_frame_ids,
        "checked_required_topic_frame_report": checked_required_topic_frame_report,
        "checked_real_motion_evidence": checked_real_motion_evidence,
        "checked_hardware_boundary_evidence": checked_hardware_boundary_evidence,
        "checked_required_topic_sample_windows": (checked_required_topic_sample_windows),
        "checked_endpoint_input_sample_windows": (checked_endpoint_input_sample_windows),
        "checked_localization_health_evidence": (checked_localization_health_evidence),
        "checked_data_flow_temporal_order": checked_data_flow_temporal_order,
        "checked_live_topic_freshness": checked_live_topic_freshness,
        "checked_collector_contract": checked_collector_contract,
        "blockers": list(result.blockers),
    }


def validate_runtime_evidence(
    report: Mapping[str, Any],
    expected_contract: str,
    *,
    require_paths: bool = True,
    require_command: bool = True,
    require_frame_links: bool = False,
    require_data_flow: bool = False,
) -> RuntimeEvidenceResult:
    blockers: list[str] = []
    runtime_contract = _mapping(report.get("runtime_contract"))

    if runtime_contract is None:
        blockers.append("runtime_contract missing")
    else:
        if runtime_contract.get("name") != expected_contract:
            blockers.append(f"runtime_contract.name is not {expected_contract}")
        if runtime_contract.get("ok") is not True:
            blockers.append("runtime_contract.ok is not true")

    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if report.get("real_robot_motion") is not False:
        blockers.append("real_robot_motion is not false")
    if report.get("cmd_vel_sent_to_hardware") is not False:
        blockers.append("cmd_vel_sent_to_hardware is not false")

    outputs = _mapping(report.get("outputs")) or {}
    if require_paths and not _has_path_evidence(outputs, report, runtime_contract):
        blockers.append("path evidence missing")
    if require_command and not _has_command_evidence(outputs, report, runtime_contract):
        blockers.append("nav command evidence missing")

    _check_topic_frame_ids(report, runtime_contract, blockers)
    _check_hardware_safety(report, runtime_contract, blockers)
    if require_frame_links:
        _check_frame_links(report, runtime_contract, blockers)
    if require_data_flow:
        _check_data_flow(report, runtime_contract, blockers, expected_contract)

    return RuntimeEvidenceResult(ok=not blockers, blockers=tuple(blockers))


def validate_real_runtime_evidence(
    report: Mapping[str, Any],
    expected_contract: str = REAL_RUNTIME_CONTRACT,
    *,
    require_paths: bool = True,
    require_command: bool = True,
    require_frame_links: bool = True,
    require_data_flow: bool = True,
    require_hardware_boundary: bool = True,
) -> RuntimeEvidenceResult:
    """Validate an observed real robot runtime report.

    This is intentionally separate from the simulation validator. Simulation
    reports must prove that no hardware command route exists; real reports must
    prove the opposite command boundary while still proving the same frame and
    data-flow contract.
    """

    blockers: list[str] = []
    runtime_contract = _mapping(report.get("runtime_contract"))

    if expected_contract != REAL_RUNTIME_CONTRACT:
        blockers.append(f"real runtime evidence expected_contract is not {REAL_RUNTIME_CONTRACT}")

    if runtime_contract is None:
        blockers.append("runtime_contract missing")
    else:
        if runtime_contract.get("name") != expected_contract:
            blockers.append(f"runtime_contract.name is not {expected_contract}")
        if runtime_contract.get("ok") is not True:
            blockers.append("runtime_contract.ok is not true")

    if report.get("simulation_only") is not False:
        blockers.append("simulation_only is not false")
    if report.get("real_robot_motion") is not True:
        blockers.append("real_robot_motion is not true")
    if report.get("cmd_vel_sent_to_hardware") is not True:
        blockers.append("cmd_vel_sent_to_hardware is not true")
    _check_collector_contract(report, blockers)
    _check_real_motion_evidence(report, blockers)

    outputs = _mapping(report.get("outputs")) or {}
    if require_paths and not _has_path_evidence(outputs, report, runtime_contract):
        blockers.append("path evidence missing")
    if require_command and not _has_command_evidence(outputs, report, runtime_contract):
        blockers.append("nav command evidence missing")

    _check_runtime_frame_contract_payload(
        runtime_contract,
        blockers,
        expected_contract,
    )
    _check_algorithm_interface_contract(
        runtime_contract,
        blockers,
    )
    _check_runtime_stage_algorithm_interface_contract(
        runtime_contract,
        blockers,
    )
    _check_topic_frame_ids(
        report,
        runtime_contract,
        blockers,
        required_topics=runtime_required_topic_frame_ids(expected_contract),
        allowed_frame_ids=runtime_topic_allowed_frame_ids(expected_contract),
    )
    _check_required_topic_frame_contract(
        runtime_contract,
        blockers,
        expected_contract,
    )
    _check_runtime_topic_evidence_coverage(
        report,
        runtime_contract,
        blockers,
        expected_contract,
    )
    _check_required_topic_sample_windows(
        report,
        runtime_contract,
        blockers,
        expected_contract,
    )
    _check_endpoint_input_sample_windows(report, runtime_contract, blockers)
    _check_data_flow_temporal_order(
        report,
        runtime_contract,
        blockers,
        expected_contract,
    )
    _check_live_topic_freshness(report, runtime_contract, blockers, expected_contract)
    _check_localization_health_evidence(report, runtime_contract, blockers)
    if require_hardware_boundary:
        _check_real_hardware_boundary(report, runtime_contract, blockers)
    if require_frame_links:
        _check_frame_links(report, runtime_contract, blockers)
    if require_data_flow:
        _check_data_flow(
            report,
            runtime_contract,
            blockers,
            expected_contract,
            require_reasons=True,
            require_observed_stages=True,
        )

    return RuntimeEvidenceResult(ok=not blockers, blockers=tuple(blockers))


def _mapping(value: Any) -> Mapping[str, Any] | None:
    return value if isinstance(value, Mapping) else None


def _positive(value: Any) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and value > 0


def _positive_int(mapping: Mapping[str, Any], key: str) -> bool:
    return _positive(mapping.get(key))


def _finite_real(value: Any) -> float | None:
    if not isinstance(value, Real) or isinstance(value, bool):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _real_motion_evidence_payload(report: Mapping[str, Any]) -> dict[str, Any]:
    motion = _mapping(report.get("motion")) or {}
    odom_delta = _finite_real(motion.get("odom_delta_m"))
    min_motion = _finite_real(motion.get("min_motion_m"))
    odom_samples = _finite_real(motion.get("odom_position_samples"))
    return {
        "real_robot_motion": report.get("real_robot_motion") is True,
        "odom_delta_m": odom_delta,
        "min_motion_m": min_motion,
        "odom_position_samples": int(odom_samples) if odom_samples is not None else None,
        "ok": (
            report.get("real_robot_motion") is True
            and odom_delta is not None
            and min_motion is not None
            and odom_delta >= min_motion
            and odom_samples is not None
            and odom_samples >= 2
        ),
    }


def _real_hardware_boundary_evidence_payload(
    report: Mapping[str, Any],
) -> dict[str, Any]:
    runtime_contract = _mapping(report.get("runtime_contract"))
    hardware_boundary = _mapping(report.get("hardware_boundary"))
    if hardware_boundary is None:
        hardware_boundary = _mapping((runtime_contract or {}).get("hardware_boundary"))
    outputs = _mapping(report.get("outputs")) or {}
    command_observed = _has_command_evidence(outputs, report, runtime_contract)
    command_subscribers = list(_tuple_value((hardware_boundary or {}).get("command_subscribers")))
    expected_subscribers = list(_tuple_value((hardware_boundary or {}).get("expected_command_subscribers")))
    unexpected_simulation_sinks = list(_tuple_value((hardware_boundary or {}).get("unexpected_simulation_sinks")))
    route_observed = (hardware_boundary or {}).get("hardware_command_route_observed") is True
    command_sink = (hardware_boundary or {}).get("command_sink")
    return {
        "command_topic": TOPICS.cmd_vel,
        "command_sink": command_sink,
        "expected_command_sink": REAL_HARDWARE_COMMAND_SINK,
        "command_observed": command_observed,
        "cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware") is True,
        "hardware_command_route_observed": route_observed,
        "command_subscribers": command_subscribers,
        "expected_command_subscribers": expected_subscribers,
        "unexpected_simulation_sinks": unexpected_simulation_sinks,
        "ok": (
            command_sink == REAL_HARDWARE_COMMAND_SINK
            and command_observed
            and report.get("cmd_vel_sent_to_hardware") is True
            and route_observed
            and not unexpected_simulation_sinks
        ),
    }


def _runtime_data_flow_evidence_payload(
    report: Mapping[str, Any],
    expected_contract: str,
) -> dict[str, dict[str, Any]]:
    runtime_contract = _mapping(report.get("runtime_contract"))
    data_flow = _mapping((runtime_contract or {}).get("data_flow_evidence"))
    if data_flow is None:
        data_flow = _mapping(report.get("data_flow_evidence"))
    topic_evidence = _topic_evidence_mapping(report)
    hardware_boundary = _mapping(report.get("hardware_boundary"))
    if hardware_boundary is None:
        hardware_boundary = _mapping((runtime_contract or {}).get("hardware_boundary"))
    if hardware_boundary is None:
        hardware_boundary = {}

    try:
        expected_flow = resolved_runtime_data_flow(expected_contract)
    except ValueError:
        return {}

    payload: dict[str, dict[str, Any]] = {}
    for stage in expected_flow:
        entry = _mapping((data_flow or {}).get(stage.name))
        observed_stage_ok = False
        observed_reason = None
        observed_signals: dict[str, bool] = {}
        if topic_evidence is not None:
            observed_stage_ok, observed_reason = runtime_data_flow_stage_observation(
                stage.name,
                topic_evidence,
                hardware_boundary,
            )
            observed_signals = runtime_data_flow_stage_signals(
                stage.name,
                topic_evidence,
                hardware_boundary,
            )
        entry_ok = entry.get("ok") is True if entry is not None else False
        stage_required = runtime_data_flow_stage_required(
            stage.name,
            topic_evidence,
            hardware_boundary,
            (entry or {}).get("required") if entry is not None else None,
        )
        payload[stage.name] = {
            "ok": entry_ok and observed_stage_ok,
            "required": stage_required,
            "inputs": list(stage.inputs),
            "outputs": list(stage.outputs),
            "observed_inputs": list(_tuple_value((entry or {}).get("observed_inputs"))),
            "missing_inputs": (
                list(_tuple_value(entry.get("missing_inputs"))) if entry is not None else list(stage.inputs)
            ),
            "observed_outputs": list(_tuple_value((entry or {}).get("observed_outputs"))),
            "missing_outputs": (
                list(_tuple_value(entry.get("missing_outputs"))) if entry is not None else list(stage.outputs)
            ),
            "observed_signals": [name for name, observed in observed_signals.items() if observed],
            "missing_signals": [name for name, observed in observed_signals.items() if not observed],
            "owner": stage.owner,
            "frame_role": stage.frame_role,
            "map_dependency": stage.map_dependency,
            "reason": (entry.get("reason") if entry is not None and entry.get("reason") else observed_reason),
        }
    return payload


def _frame_link_evidence_payload(
    report: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    runtime_contract = _mapping(report.get("runtime_contract"))
    frame_evidence = _mapping((runtime_contract or {}).get("frame_evidence"))
    if frame_evidence is None:
        frame_evidence = _mapping(report.get("frame_evidence"))

    payload: dict[str, dict[str, Any]] = {}
    for name, expected in FRAME_LINKS.items():
        entry = _mapping((frame_evidence or {}).get(name))
        samples = _finite_real((entry or {}).get("samples"))
        observed_parent = str(entry.get("parent")) if entry and entry.get("parent") else None
        observed_child = str(entry.get("child")) if entry and entry.get("child") else None
        payload[name] = {
            "expected_parent": expected.parent,
            "expected_child": expected.child,
            "observed_parent": observed_parent,
            "observed_child": observed_child,
            "samples": int(samples) if samples is not None else None,
            "static": (entry or {}).get("static") is True,
            "published": (entry or {}).get("published") is True,
            "error": (entry or {}).get("error"),
            "ok": bool(
                entry
                and entry.get("ok") is True
                and _frame_link_observed(entry)
                and observed_parent == expected.parent
                and observed_child == expected.child
            ),
        }
    return payload


def _collector_duration_sec(report: Mapping[str, Any]) -> float | None:
    collector = _mapping(report.get("collector")) or {}
    return _finite_real(collector.get("duration_sec"))


def _collector_contract_payload(report: Mapping[str, Any]) -> dict[str, Any]:
    collector = _mapping(report.get("collector"))
    control_topics = list(_tuple_value((collector or {}).get("control_topics_published")))
    payload = {
        "name": (collector or {}).get("name"),
        "expected_name": REAL_RUNTIME_COLLECTOR_NAME,
        "read_only": (collector or {}).get("read_only"),
        "control_topics_published": control_topics,
        "expected_control_topics_published": list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED),
        "duration_sec": _collector_duration_sec(report),
    }
    payload["ok"] = (
        payload["name"] == REAL_RUNTIME_COLLECTOR_NAME
        and payload["read_only"] is True
        and tuple(control_topics) == REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED
        and payload["duration_sec"] is not None
        and payload["duration_sec"] > 0.0
    )
    return payload


def _check_collector_contract(
    report: Mapping[str, Any],
    blockers: list[str],
) -> None:
    collector = _mapping(report.get("collector"))
    if collector is None:
        blockers.append("real runtime collector contract missing")
        return
    if collector.get("name") != REAL_RUNTIME_COLLECTOR_NAME:
        blockers.append("real runtime collector name mismatch")
    if collector.get("read_only") is not True:
        blockers.append("real runtime collector is not read-only")
    if _tuple_value(collector.get("control_topics_published")) != (REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED):
        blockers.append("real runtime collector published control topics")
    duration_sec = _collector_duration_sec(report)
    if duration_sec is None or duration_sec <= 0.0:
        blockers.append("real runtime collector duration_sec missing or invalid")


def _topic_evidence_mapping(report: Mapping[str, Any]) -> Mapping[str, Any] | None:
    runtime_contract = _mapping(report.get("runtime_contract"))
    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _mapping(report.get("topic_evidence"))
    return topic_evidence


def _required_topic_sample_windows_payload(
    report: Mapping[str, Any],
    topics: Sequence[str],
) -> dict[str, dict[str, Any]]:
    topic_evidence = _topic_evidence_mapping(report)
    duration_sec = _collector_duration_sec(report)
    windows: dict[str, dict[str, Any]] = {}
    for topic in topics:
        entry = _mapping((topic_evidence or {}).get(topic))
        windows[topic] = _topic_sample_window_payload(entry, duration_sec)
    return windows


def _topic_sample_window_payload(
    entry: Mapping[str, Any] | None,
    duration_sec: float | None,
) -> dict[str, Any]:
    samples = _finite_real((entry or {}).get("samples"))
    first_seen = _finite_real((entry or {}).get("first_seen_sec"))
    last_seen = _finite_real((entry or {}).get("last_seen_sec"))
    sample_span = _finite_real((entry or {}).get("sample_span_sec"))
    return {
        "samples": int(samples) if samples is not None else None,
        "first_seen_sec": first_seen,
        "last_seen_sec": last_seen,
        "sample_span_sec": sample_span,
        "duration_sec": duration_sec,
        "ok": _topic_sample_window_ok(
            samples,
            first_seen,
            last_seen,
            sample_span,
            duration_sec,
        ),
    }


def _entry_sample_window_ok(
    entry: Mapping[str, Any] | None,
    duration_sec: float | None,
) -> bool:
    return _topic_sample_window_payload(entry, duration_sec)["ok"] is True


def _topic_sample_window_ok(
    samples: float | None,
    first_seen: float | None,
    last_seen: float | None,
    sample_span: float | None,
    duration_sec: float | None,
) -> bool:
    if (
        samples is None
        or samples <= 0
        or first_seen is None
        or last_seen is None
        or sample_span is None
        or duration_sec is None
    ):
        return False
    tolerance = 1e-6
    return (
        0.0 <= first_seen <= last_seen
        and last_seen <= duration_sec + tolerance
        and sample_span >= 0.0
        and sample_span <= duration_sec + tolerance
    )


def _check_required_topic_sample_windows(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    expected_contract: str,
) -> None:
    _check_topic_sample_windows(
        report,
        runtime_contract,
        blockers,
        runtime_required_topic_frame_ids(expected_contract),
        label="topic",
    )


def _check_endpoint_input_sample_windows(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
) -> None:
    _check_topic_sample_windows(
        report,
        runtime_contract,
        blockers,
        REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
        label="endpoint input",
    )


def _check_topic_sample_windows(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    topics: Sequence[str],
    *,
    label: str,
) -> None:
    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _topic_evidence_mapping(report)
    if topic_evidence is None:
        blockers.append(f"{label} sample window evidence missing")
        return

    duration_sec = _collector_duration_sec(report)
    if duration_sec is None:
        blockers.append("collector duration_sec missing or invalid")
        return

    for topic in topics:
        entry = _mapping(topic_evidence.get(topic))
        if entry is None:
            blockers.append(f"{label} sample window missing for {topic}")
            continue
        samples = _finite_real(entry.get("samples"))
        first_seen = _finite_real(entry.get("first_seen_sec"))
        last_seen = _finite_real(entry.get("last_seen_sec"))
        sample_span = _finite_real(entry.get("sample_span_sec"))
        if samples is None or samples <= 0 or first_seen is None or last_seen is None or sample_span is None:
            blockers.append(f"{label} sample window missing for {topic}")
            continue
        if not _topic_sample_window_ok(
            samples,
            first_seen,
            last_seen,
            sample_span,
            duration_sec,
        ):
            blockers.append(f"{label} sample window outside collection duration for {topic}")


def _data_flow_temporal_order_payload(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    expected_contract: str,
) -> dict[str, dict[str, Any]]:
    if expected_contract != REAL_RUNTIME_CONTRACT:
        return {}
    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _topic_evidence_mapping(report)
    if topic_evidence is None:
        return {
            stage: {
                "input_topics": list(input_topics),
                "output_topics": list(output_topics),
                "input_first_seen_max_sec": None,
                "output_last_seen_min_sec": None,
                "ok": False,
            }
            for stage, (input_topics, output_topics) in (REAL_RUNTIME_TEMPORAL_FLOW_STAGES.items())
        }

    duration_sec = _collector_duration_sec(report)
    payload: dict[str, dict[str, Any]] = {}
    for stage, (input_topics, output_topics) in REAL_RUNTIME_TEMPORAL_FLOW_STAGES.items():
        input_windows = {
            topic: _topic_sample_window_payload(
                _mapping(topic_evidence.get(topic)),
                duration_sec,
            )
            for topic in input_topics
        }
        output_windows = {
            topic: _topic_sample_window_payload(
                _mapping(topic_evidence.get(topic)),
                duration_sec,
            )
            for topic in output_topics
        }
        input_first_seen_values = [
            window["first_seen_sec"]
            for window in input_windows.values()
            if window["ok"] is True and window["first_seen_sec"] is not None
        ]
        output_last_seen_values = [
            window["last_seen_sec"]
            for window in output_windows.values()
            if window["ok"] is True and window["last_seen_sec"] is not None
        ]
        input_first_seen_max = (
            max(input_first_seen_values) if len(input_first_seen_values) == len(input_topics) else None
        )
        output_last_seen_min = (
            min(output_last_seen_values) if len(output_last_seen_values) == len(output_topics) else None
        )
        payload[stage] = {
            "input_topics": list(input_topics),
            "output_topics": list(output_topics),
            "input_first_seen_max_sec": input_first_seen_max,
            "output_last_seen_min_sec": output_last_seen_min,
            "ok": (
                input_first_seen_max is not None
                and output_last_seen_min is not None
                and output_last_seen_min + 1e-6 >= input_first_seen_max
            ),
        }
    return payload


def _check_data_flow_temporal_order(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    expected_contract: str,
) -> None:
    for stage, evidence in _data_flow_temporal_order_payload(
        report,
        runtime_contract,
        expected_contract,
    ).items():
        if evidence.get("ok") is not True:
            blockers.append(f"data-flow temporal order failed for {stage}")


def _live_topic_freshness_payload(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    expected_contract: str,
) -> dict[str, dict[str, Any]]:
    if expected_contract != REAL_RUNTIME_CONTRACT:
        return {}
    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _topic_evidence_mapping(report)
    duration_sec = _collector_duration_sec(report)
    payload: dict[str, dict[str, Any]] = {}
    for topic in REAL_RUNTIME_LIVE_TOPIC_FRESHNESS_TOPICS:
        entry = _mapping((topic_evidence or {}).get(topic))
        window = _topic_sample_window_payload(entry, duration_sec)
        age_sec = None
        if duration_sec is not None and window["last_seen_sec"] is not None:
            age_sec = duration_sec - window["last_seen_sec"]
        payload[topic] = {
            "samples": window["samples"],
            "last_seen_sec": window["last_seen_sec"],
            "duration_sec": duration_sec,
            "max_age_sec": REAL_RUNTIME_LIVE_TOPIC_MAX_AGE_SEC,
            "age_sec": age_sec,
            "ok": (
                window["ok"] is True and age_sec is not None and 0.0 <= age_sec <= REAL_RUNTIME_LIVE_TOPIC_MAX_AGE_SEC
            ),
        }
    return payload


def _check_live_topic_freshness(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    expected_contract: str,
) -> None:
    for topic, evidence in _live_topic_freshness_payload(
        report,
        runtime_contract,
        expected_contract,
    ).items():
        if evidence.get("ok") is not True:
            blockers.append(f"live topic stale for {topic}")


def _localization_health_evidence_payload(report: Mapping[str, Any]) -> dict[str, Any]:
    topic_evidence = _topic_evidence_mapping(report) or {}
    duration_sec = _collector_duration_sec(report)
    health_entry = _mapping(topic_evidence.get(TOPICS.localization_health))
    quality_entry = _mapping(topic_evidence.get(TOPICS.localization_quality))
    health_state, health_quality = _parse_localization_health_entry(health_entry)
    quality = _localization_quality_value(quality_entry)
    quality_source = quality_entry
    if quality is None:
        quality = health_quality
        quality_source = health_entry
    health_window_ok = _entry_sample_window_ok(health_entry, duration_sec)
    quality_window_ok = _entry_sample_window_ok(quality_entry, duration_sec)
    healthy_states = list(REAL_RUNTIME_LOCALIZATION_HEALTHY_STATES)
    return {
        "health_topic": TOPICS.localization_health,
        "health_state": health_state,
        "healthy_states": healthy_states,
        "quality_topic": TOPICS.localization_quality,
        "quality": quality,
        "quality_min_exclusive": REAL_RUNTIME_LOCALIZATION_QUALITY_MIN_EXCLUSIVE,
        "quality_max_exclusive": REAL_RUNTIME_LOCALIZATION_QUALITY_MAX_EXCLUSIVE,
        "health_window_ok": health_window_ok,
        "quality_window_ok": quality_window_ok,
        "ok": (
            health_state in REAL_RUNTIME_LOCALIZATION_HEALTHY_STATES
            and _localization_quality_healthy(quality, quality_source)
            and health_window_ok
            and quality_window_ok
        ),
    }


def _check_localization_health_evidence(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
) -> None:
    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _topic_evidence_mapping(report)
    if topic_evidence is None:
        blockers.append("localization health evidence missing")
        blockers.append("localization quality evidence missing")
        return

    duration_sec = _collector_duration_sec(report)
    health_entry = _mapping(topic_evidence.get(TOPICS.localization_health))
    quality_entry = _mapping(topic_evidence.get(TOPICS.localization_quality))
    if health_entry is None:
        blockers.append("localization health evidence missing")
    else:
        health_state, _ = _parse_localization_health_entry(health_entry)
        if not health_state:
            blockers.append("localization health state missing")
        elif health_state not in REAL_RUNTIME_LOCALIZATION_HEALTHY_STATES:
            blockers.append(f"localization health state is not healthy: {health_state}")
        if not _entry_sample_window_ok(health_entry, duration_sec):
            blockers.append("localization health sample window missing or outside collection duration")

    if quality_entry is None:
        blockers.append("localization quality evidence missing")
    else:
        quality = _localization_quality_value(quality_entry)
        if quality is None:
            blockers.append("localization quality value missing or invalid")
        elif not _localization_quality_healthy(quality, quality_entry):
            blockers.append(f"localization quality outside healthy range: {quality}")
        if not _entry_sample_window_ok(quality_entry, duration_sec):
            blockers.append("localization quality sample window missing or outside collection duration")


def _parse_localization_health_entry(
    entry: Mapping[str, Any] | None,
) -> tuple[str | None, float | None]:
    if entry is None:
        return None, None
    raw = entry.get("data")
    if raw is None:
        raw = entry.get("value")
    if raw is None:
        return None, None
    text = str(raw).strip()
    if not text:
        return None, None
    if text.startswith("{"):
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            data = {}
        if isinstance(data, Mapping):
            state = data.get("state") or data.get("status") or data.get("health")
            quality = _finite_real(data.get("fitness"))
            if quality is None:
                quality = _finite_real(data.get("quality"))
            return _normalize_health_state(state), quality
    parts = text.split("|")
    state = _normalize_health_state(parts[0] if parts else None)
    quality = None
    for item in parts[1:]:
        key, _, value = item.partition("=")
        if key.strip().lower() in {"fitness", "quality"}:
            quality = _finite_real_from_text(value)
            if quality is not None:
                break
    return state, quality


def _normalize_health_state(value: Any) -> str | None:
    if value is None:
        return None
    state = str(value).strip().upper()
    return state or None


def _finite_real_from_text(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _localization_quality_value(entry: Mapping[str, Any] | None) -> float | None:
    if entry is None:
        return None
    for key in ("value", "data", "quality", "fitness"):
        value = _finite_real(entry.get(key))
        if value is not None:
            return value
        value = _finite_real_from_text(entry.get(key))
        if value is not None:
            return value
    return None


def _localization_quality_kind(entry: Mapping[str, Any] | None) -> str:
    if entry is None:
        return "fitness"
    explicit = str(entry.get("quality_kind") or "").strip().lower()
    if explicit in {"confidence", "fitness"}:
        return explicit
    data = str(entry.get("data") or "").lower()
    if "quality=" in data or "confidence=" in data:
        return "confidence"
    return "fitness"


def _localization_quality_healthy(
    value: float | None,
    entry: Mapping[str, Any] | None = None,
) -> bool:
    if value is None:
        return False
    if _localization_quality_kind(entry) == "confidence":
        return (
            REAL_RUNTIME_LOCALIZATION_CONFIDENCE_MIN_INCLUSIVE
            <= value
            <= REAL_RUNTIME_LOCALIZATION_CONFIDENCE_MAX_INCLUSIVE
        )
    return REAL_RUNTIME_LOCALIZATION_QUALITY_MIN_EXCLUSIVE < value < REAL_RUNTIME_LOCALIZATION_QUALITY_MAX_EXCLUSIVE


def _check_real_motion_evidence(
    report: Mapping[str, Any],
    blockers: list[str],
) -> None:
    motion = _mapping(report.get("motion"))
    if motion is None:
        blockers.append("real motion evidence missing")
        return

    odom_delta = _finite_real(motion.get("odom_delta_m"))
    min_motion = _finite_real(motion.get("min_motion_m"))
    odom_samples = _finite_real(motion.get("odom_position_samples"))

    if odom_delta is None:
        blockers.append("real motion odom_delta_m missing or invalid")
    if min_motion is None:
        blockers.append("real motion min_motion_m missing or invalid")
    if odom_delta is not None and min_motion is not None and odom_delta < min_motion:
        blockers.append("real motion odom_delta_m below min_motion_m")
    if odom_samples is None or odom_samples < 2:
        blockers.append("real motion requires at least two odometry position samples")


def runtime_data_flow_stage_observation(
    stage_name: str,
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
) -> tuple[bool, str]:
    """Return whether observed runtime topics prove one declared data-flow stage."""

    signals = runtime_data_flow_stage_signals(
        stage_name,
        topic_evidence,
        hardware_boundary,
    )
    if stage_name == "endpoint_adapter":
        return all(signals.values()), "source_lidar_imu_observed_or_present"

    if stage_name == "slam_or_relayed_localization_map":
        return all(signals.values()), "slam_outputs_observed"

    if stage_name == "map_layers_and_exploration":
        return all(signals.values()), "goal_or_downstream_global_path_observed"

    if stage_name == "global_planning":
        return (
            all(signals.values()),
            "nonempty_global_path_observed",
        )

    if stage_name == "local_planning_and_following":
        return all(signals.values()), "local_planner_input_output_chain_observed"

    if stage_name == "dynamic_obstacle_gate":
        return all(signals.values()), "dynamic_obstacle_gate_observed"

    if stage_name == "command_boundary":
        return all(signals.values()), "nonzero_cmd_and_hardware_route_observed"

    return False, "unknown_stage"


def runtime_data_flow_stage_required(
    stage_name: str,
    topic_evidence: Mapping[str, Any] | None,
    hardware_boundary: Mapping[str, Any] | None,
    explicit_required: Any = None,
) -> bool:
    """Return whether a data-flow stage is required for generic runtime evidence."""

    if explicit_required is not None:
        return explicit_required is not False
    if stage_name in OPTIONAL_RUNTIME_DATA_FLOW_STAGES:
        if topic_evidence is None:
            return False
        return runtime_data_flow_stage_observation(
            stage_name,
            topic_evidence,
            hardware_boundary or {},
        )[0]
    return True


def runtime_data_flow_stage_signals(
    stage_name: str,
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
) -> dict[str, bool]:
    """Return named observations required to prove one data-flow stage."""

    if stage_name == "endpoint_adapter":
        return {
            "lidar_scan_sampled": _topic_observed(
                topic_evidence,
                TOPICS.lidar_scan,
            ),
            "imu_sampled": _topic_observed(
                topic_evidence,
                TOPICS.imu,
            ),
        }

    if stage_name == "slam_or_relayed_localization_map":
        signals = {
            "odometry_sampled": _topic_observed(topic_evidence, TOPICS.odometry),
            "registered_cloud_nonempty": _topic_observed(
                topic_evidence,
                TOPICS.registered_cloud,
                require_nonempty=True,
            ),
            "map_cloud_nonempty": _topic_observed(
                topic_evidence,
                TOPICS.map_cloud,
                require_nonempty=True,
            ),
        }
        if TOPICS.localization_health in topic_evidence or TOPICS.localization_quality in topic_evidence:
            health_state, _ = _parse_localization_health_entry(_mapping(topic_evidence.get(TOPICS.localization_health)))
            localization_quality = _localization_quality_value(
                _mapping(topic_evidence.get(TOPICS.localization_quality))
            )
            signals["localization_health_healthy"] = health_state in REAL_RUNTIME_LOCALIZATION_HEALTHY_STATES
            signals["localization_quality_sampled"] = localization_quality is not None
            signals["localization_quality_healthy_range"] = _localization_quality_healthy(
                localization_quality,
                _mapping(topic_evidence.get(TOPICS.localization_quality)),
            )
        return signals

    if stage_name == "map_layers_and_exploration":
        return {
            "goal_or_downstream_global_path_observed": (
                _any_topic_observed(
                    topic_evidence,
                    (TOPICS.exploration_way_point, TOPICS.goal_pose),
                )
                or _topic_observed(
                    topic_evidence,
                    TOPICS.global_path,
                    require_nonempty=True,
                )
            ),
        }

    if stage_name == "global_planning":
        return {
            "global_path_nonempty": _topic_observed(
                topic_evidence,
                TOPICS.global_path,
                require_nonempty=True,
            ),
        }

    if stage_name == "local_planning_and_following":
        return {
            "odometry_sampled": _topic_observed(topic_evidence, TOPICS.odometry),
            "registered_cloud_nonempty": _topic_observed(
                topic_evidence,
                TOPICS.registered_cloud,
                require_nonempty=True,
            ),
            "global_path_nonempty": _topic_observed(
                topic_evidence,
                TOPICS.global_path,
                require_nonempty=True,
            ),
            "local_path_nonempty": _topic_observed(
                topic_evidence,
                TOPICS.local_path,
                require_nonempty=True,
            ),
            "cmd_vel_nonzero": _topic_observed(
                topic_evidence,
                TOPICS.cmd_vel,
                require_nonzero_cmd=True,
            ),
        }

    if stage_name == "dynamic_obstacle_gate":
        return {
            "added_obstacles_nonempty": _topic_observed(
                topic_evidence,
                TOPICS.added_obstacles,
                require_nonempty=True,
            ),
            "local_path_nonempty": _topic_observed(
                topic_evidence,
                TOPICS.local_path,
                require_nonempty=True,
            ),
            "cmd_vel_nonzero": _topic_observed(
                topic_evidence,
                TOPICS.cmd_vel,
                require_nonzero_cmd=True,
            ),
            "obstacle_check_or_planner_status_sampled": _any_topic_observed(
                topic_evidence,
                (TOPICS.check_obstacle, TOPICS.planner_status),
            ),
        }

    if stage_name == "command_boundary":
        return {
            "cmd_vel_nonzero": _topic_observed(
                topic_evidence,
                TOPICS.cmd_vel,
                require_nonzero_cmd=True,
            ),
            "hardware_route_observed": (hardware_boundary.get("hardware_command_route_observed") is True),
        }

    return {"known_stage": False}


def _topic_entry(topic_evidence: Mapping[str, Any], topic: str) -> Mapping[str, Any]:
    entry = topic_evidence.get(topic)
    return entry if isinstance(entry, Mapping) else {}


def _observed_topic_frame_ids(
    report: Mapping[str, Any],
    topics: Sequence[str],
) -> dict[str, str]:
    runtime_contract = _mapping(report.get("runtime_contract"))
    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _mapping(report.get("topic_evidence"))
    if topic_evidence is None:
        return {}

    observed: dict[str, str] = {}
    for topic in topics:
        entry = _mapping(topic_evidence.get(topic))
        if entry is None:
            continue
        frame_id = normalize_frame_id(str(entry.get("frame_id") or ""))
        if frame_id:
            observed[topic] = frame_id
    return observed


def _required_topic_frame_report(
    topics: Sequence[str],
    default_frame_ids: Mapping[str, str],
    observed_frame_ids: Mapping[str, str],
    allowed_frame_ids: Mapping[str, Sequence[str]],
) -> dict[str, dict[str, Any]]:
    report: dict[str, dict[str, Any]] = {}
    for topic in topics:
        allowed = tuple(str(frame) for frame in allowed_frame_ids.get(topic, ()))
        observed = observed_frame_ids.get(topic)
        observed_aliases = set(expand_frame_id_aliases((observed,)))
        allowed_aliases = set(expand_frame_id_aliases(allowed))
        report[topic] = {
            "default_frame_id": default_frame_ids.get(topic),
            "observed_frame_id": observed,
            "allowed_frame_ids": list(allowed),
            "ok": bool(observed and observed_aliases & allowed_aliases),
        }
    return report


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
        return _positive(entry.get("nonzero_samples")) or _positive(entry.get("max_norm"))
    if require_nonempty:
        return any(_positive(entry.get(key)) for key in ("nonempty_samples", "max_poses", "points", "cells"))
    if _positive(entry.get("samples")) or entry.get("ok") is True:
        return True
    return allow_graph and entry.get("graph_exists") is True


def _any_topic_observed(
    topic_evidence: Mapping[str, Any],
    topics: Sequence[str],
    *,
    allow_graph: bool = False,
    require_nonempty: bool = False,
) -> bool:
    return any(
        _topic_observed(
            topic_evidence,
            topic,
            allow_graph=allow_graph,
            require_nonempty=require_nonempty,
        )
        for topic in topics
    )


def _has_path_evidence(
    outputs: Mapping[str, Any],
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
) -> bool:
    if _positive_int(outputs, "global_path_count") and _positive_int(
        outputs,
        "local_path_count",
    ):
        return True

    paths = _mapping(report.get("paths"))
    if paths and _path_topic_ok(paths, "global") and _path_topic_ok(paths, "local"):
        return True

    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if (
        topic_evidence
        and _path_topic_ok(topic_evidence, "global")
        and _path_topic_ok(
            topic_evidence,
            "local",
        )
    ):
        return True
    return False


def _path_topic_ok(entries: Mapping[str, Any], name: str) -> bool:
    return any(
        name in str(topic).lower() and "path" in str(topic).lower() and _path_entry_positive(entry)
        for topic, entry in entries.items()
    )


def _path_entry_positive(entry: Any) -> bool:
    data = _mapping(entry)
    if data is None or data.get("ok") is False:
        return False
    return any(_positive(data.get(key)) for key in ("samples", "nonempty_samples", "max_poses", "poses", "points"))


def _has_command_evidence(
    outputs: Mapping[str, Any],
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
) -> bool:
    if _positive_int(outputs, "nav_cmd_vel_nonzero"):
        return True

    cmd_vel = _mapping(report.get("cmd_vel"))
    if cmd_vel and _command_entry_positive(cmd_vel):
        return True

    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence:
        return any(
            "cmd_vel" in str(topic) and _command_entry_positive(entry) for topic, entry in topic_evidence.items()
        )
    return False


def _command_entry_positive(entry: Any) -> bool:
    data = _mapping(entry)
    if data is None or data.get("ok") is False:
        return False
    return any(_positive(data.get(key)) for key in ("nonzero_samples", "max_norm", "samples"))


def _check_hardware_safety(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
) -> None:
    hardware_safety = _mapping(report.get("hardware_safety"))
    if hardware_safety is None:
        hardware_safety = _mapping((runtime_contract or {}).get("hardware_safety"))
    if hardware_safety is None:
        blockers.append("hardware_safety missing")
        return

    if hardware_safety.get("blocked_hardware_nodes"):
        blockers.append("hardware-looking command subscriber present")
    if hardware_safety.get("unexpected_command_publishers"):
        blockers.append("unexpected command publisher present")

    for subscriber in _command_subscribers(hardware_safety):
        if _looks_like_hardware_command_subscriber(subscriber):
            blockers.append("hardware-looking command subscriber present")
            break


def _check_real_hardware_boundary(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
) -> None:
    hardware_boundary = _mapping(report.get("hardware_boundary"))
    if hardware_boundary is None:
        hardware_boundary = _mapping((runtime_contract or {}).get("hardware_boundary"))
    if hardware_boundary is None:
        blockers.append("hardware_boundary missing")
        return

    expected_sink = REAL_HARDWARE_COMMAND_SINK
    if hardware_boundary.get("command_sink") != expected_sink:
        blockers.append(f"hardware_boundary.command_sink is not {expected_sink}")

    command_subscribers = _tuple_value(hardware_boundary.get("command_subscribers"))
    route_observed = hardware_boundary.get("hardware_command_route_observed") is True
    if not route_observed and not any(_looks_like_hardware_command_subscriber(name) for name in command_subscribers):
        blockers.append("real hardware command boundary missing")

    if hardware_boundary.get("unexpected_simulation_sinks"):
        blockers.append("simulation command sink present in real runtime")


def _check_topic_frame_ids(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    *,
    required_topics: tuple[str, ...] = (),
    allowed_frame_ids: Mapping[str, tuple[str, ...]] | None = None,
) -> None:
    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _mapping(report.get("topic_evidence"))
    if topic_evidence is None:
        if required_topics:
            blockers.append("topic frame evidence missing")
        return

    required = set(required_topics)
    frame_rules = allowed_frame_ids or TOPIC_ALLOWED_FRAME_IDS
    for topic, allowed_frames in frame_rules.items():
        entry = _mapping(topic_evidence.get(topic))
        if entry is None:
            if topic in required:
                blockers.append(f"topic frame evidence missing for {topic}")
            continue
        frame_id = entry.get("frame_id")
        if not frame_id:
            if topic in required:
                blockers.append(f"topic frame_id missing for {topic}")
            continue
        observed = normalize_frame_id(str(frame_id))
        allowed = set(expand_frame_id_aliases(allowed_frames))
        if observed not in allowed:
            blockers.append(f"topic frame_id mismatch for {topic}: {observed} not in {', '.join(sorted(allowed))}")


def _check_runtime_topic_evidence_coverage(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    expected_contract: str,
) -> None:
    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _mapping(report.get("topic_evidence"))
    if topic_evidence is None:
        blockers.append("runtime topic evidence missing")
        return

    try:
        expected_topics = runtime_data_flow_topics(expected_contract)
    except ValueError as exc:
        blockers.append(f"runtime topic evidence contract unavailable: {exc}")
        return

    for topic in expected_topics:
        if not isinstance(topic_evidence.get(topic), Mapping):
            blockers.append(f"runtime topic evidence missing for {topic}")


def _check_runtime_frame_contract_payload(
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    expected_contract: str,
) -> None:
    if runtime_contract is None:
        return

    declared_frames = _mapping(runtime_contract.get("frames"))
    if declared_frames is None:
        blockers.append("runtime frames contract missing")
    else:
        expected_frames = runtime_frames_contract()
        observed_frames = _jsonable_contract_map(declared_frames)
        missing_frame_keys = sorted(set(expected_frames) - set(observed_frames))
        unknown_frame_keys = sorted(set(observed_frames) - set(expected_frames))
        if missing_frame_keys:
            blockers.append("runtime frames contract missing keys: " + ", ".join(missing_frame_keys))
        if unknown_frame_keys:
            blockers.append("runtime frames contract unknown keys: " + ", ".join(unknown_frame_keys))
        for key, expected_value in expected_frames.items():
            if observed_frames.get(key) != expected_value:
                blockers.append(f"runtime frame {key} mismatch")

    declared_defaults = _mapping(runtime_contract.get("topic_default_frame_ids"))
    if declared_defaults is None:
        blockers.append("topic default frame contract missing")
    else:
        expected_defaults = runtime_topic_default_frame_ids(expected_contract)
        observed_defaults = {str(topic): str(frame_id) for topic, frame_id in declared_defaults.items()}
        if observed_defaults != expected_defaults:
            blockers.append("topic default frame contract mismatch")

    declared_allowed = _mapping(runtime_contract.get("topic_allowed_frame_ids"))
    if declared_allowed is None:
        blockers.append("topic allowed frame contract missing")
    else:
        expected_allowed = runtime_topic_allowed_frame_contract(expected_contract)
        observed_allowed = {
            str(topic): [str(frame_id) for frame_id in _tuple_value(frame_ids)]
            for topic, frame_ids in declared_allowed.items()
        }
        if observed_allowed != expected_allowed:
            blockers.append("topic allowed frame contract mismatch")


def _check_algorithm_interface_contract(
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
) -> None:
    if runtime_contract is None:
        return

    declared = _mapping(runtime_contract.get("algorithm_interfaces"))
    if declared is None:
        blockers.append("runtime algorithm interface contract missing")
        return

    expected = runtime_algorithm_interface_contract()
    observed = normalize_algorithm_interface_contract(declared)
    if observed != expected:
        blockers.append("runtime algorithm interface contract mismatch")


def _check_runtime_stage_algorithm_interface_contract(
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
) -> None:
    if runtime_contract is None:
        return

    declared = _mapping(runtime_contract.get("runtime_data_flow_stage_algorithm_interfaces"))
    if declared is None:
        blockers.append("runtime stage algorithm interface contract missing")
        return

    expected = runtime_stage_algorithm_interface_contract()
    observed = {
        str(stage): [str(interface) for interface in _tuple_value(interfaces)] for stage, interfaces in declared.items()
    }
    if observed != expected:
        blockers.append("runtime stage algorithm interface contract mismatch")


def _jsonable_contract_map(value: Mapping[str, Any]) -> dict[str, Any]:
    return {str(key): list(item) if isinstance(item, tuple) else item for key, item in value.items()}


def _check_required_topic_frame_contract(
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    expected_contract: str,
) -> None:
    declared = _mapping((runtime_contract or {}).get("required_topic_frame_contract"))
    if declared is None:
        blockers.append("required topic frame contract missing")
        return

    allowed_frame_ids = runtime_topic_allowed_frame_ids(expected_contract)
    for topic in runtime_required_topic_frame_ids(expected_contract):
        entry = _mapping(declared.get(topic))
        if entry is None:
            blockers.append(f"required topic frame contract missing for {topic}")
            continue
        default_frame = normalize_frame_id(str(entry.get("default_frame_id") or ""))
        expected_default = runtime_topic_default_frame_id(expected_contract, topic)
        if default_frame != expected_default:
            blockers.append(f"required topic frame default mismatch for {topic}")
        declared_allowed = tuple(
            frame
            for frame in (normalize_frame_id(str(item)) for item in _tuple_value(entry.get("allowed_frame_ids")))
            if frame
        )
        if declared_allowed != allowed_frame_ids[topic]:
            blockers.append(f"required topic allowed frames mismatch for {topic}")


def _check_frame_links(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
) -> None:
    frame_evidence = _mapping((runtime_contract or {}).get("frame_evidence"))
    if frame_evidence is None:
        frame_evidence = _mapping(report.get("frame_evidence"))
    if frame_evidence is None:
        blockers.append("frame evidence missing")
        return

    for name, expected in FRAME_LINKS.items():
        entry = _mapping(frame_evidence.get(name))
        if entry is None or entry.get("ok") is not True or not _frame_link_observed(entry):
            blockers.append(f"frame evidence missing or failed for {name}")
            continue
        if entry.get("parent") != expected.parent:
            blockers.append(f"frame evidence parent mismatch for {name}")
        if entry.get("child") != expected.child:
            blockers.append(f"frame evidence child mismatch for {name}")


def _frame_link_observed(entry: Mapping[str, Any]) -> bool:
    return entry.get("static") is True or entry.get("published") is True or _positive(entry.get("samples"))


def _check_data_flow(
    report: Mapping[str, Any],
    runtime_contract: Mapping[str, Any] | None,
    blockers: list[str],
    expected_contract: str,
    *,
    require_reasons: bool = False,
    require_observed_stages: bool = False,
) -> None:
    data_flow = _mapping((runtime_contract or {}).get("data_flow_evidence"))
    if data_flow is None:
        data_flow = _mapping(report.get("data_flow_evidence"))
    if data_flow is None:
        blockers.append("data-flow evidence missing")
        return

    try:
        expected_flow = resolved_runtime_data_flow(expected_contract)
    except ValueError as exc:
        blockers.append(f"resolved data-flow contract unavailable: {exc}")
        return

    topic_evidence = _mapping((runtime_contract or {}).get("topic_evidence"))
    if topic_evidence is None:
        topic_evidence = _mapping(report.get("topic_evidence"))
    hardware_boundary = _mapping(report.get("hardware_boundary"))
    if hardware_boundary is None:
        hardware_boundary = _mapping((runtime_contract or {}).get("hardware_boundary"))
    if hardware_boundary is None:
        hardware_boundary = {}

    for stage in expected_flow:
        entry = _mapping(data_flow.get(stage.name))
        if entry is None:
            stage_required = runtime_data_flow_stage_required(
                stage.name,
                topic_evidence,
                hardware_boundary,
            )
            if stage_required:
                blockers.append(f"data-flow evidence missing or failed for {stage.name}")
            continue
        stage_required = runtime_data_flow_stage_required(
            stage.name,
            topic_evidence,
            hardware_boundary,
            entry.get("required") if "required" in entry else None,
        )
        if entry.get("ok") is not True and stage_required:
            blockers.append(f"data-flow evidence missing or failed for {stage.name}")
            continue
        if require_observed_stages and stage_required:
            if topic_evidence is None:
                blockers.append(f"data-flow observed topics missing for {stage.name}")
            elif not runtime_data_flow_stage_observation(
                stage.name,
                topic_evidence,
                hardware_boundary,
            )[0]:
                blockers.append(f"data-flow observed topics missing for {stage.name}")
        reason = entry.get("reason")
        if require_reasons and (not isinstance(reason, str) or not reason.strip()):
            blockers.append(f"data-flow evidence reason missing for {stage.name}")
        if _tuple_value(entry.get("inputs")) != stage.inputs:
            blockers.append(f"data-flow evidence inputs mismatch for {stage.name}")
        if _tuple_value(entry.get("outputs")) != stage.outputs:
            blockers.append(f"data-flow evidence outputs mismatch for {stage.name}")
        if require_observed_stages:
            expected_observed_inputs = _observed_flow_tokens(
                stage.inputs,
                topic_evidence or {},
                hardware_boundary,
            )
            expected_missing_inputs = _missing_flow_tokens(
                stage.inputs,
                topic_evidence or {},
                hardware_boundary,
            )
            expected_observed_outputs = _observed_flow_tokens(
                stage.outputs,
                topic_evidence or {},
                hardware_boundary,
            )
            expected_missing_outputs = _missing_flow_tokens(
                stage.outputs,
                topic_evidence or {},
                hardware_boundary,
            )
            if _tuple_value(entry.get("observed_inputs")) != expected_observed_inputs:
                blockers.append(f"data-flow evidence observed_inputs mismatch for {stage.name}")
            if _tuple_value(entry.get("missing_inputs")) != expected_missing_inputs:
                blockers.append(f"data-flow evidence missing_inputs mismatch for {stage.name}")
            if _tuple_value(entry.get("observed_outputs")) != expected_observed_outputs:
                blockers.append(f"data-flow evidence observed_outputs mismatch for {stage.name}")
            if _tuple_value(entry.get("missing_outputs")) != expected_missing_outputs:
                blockers.append(f"data-flow evidence missing_outputs mismatch for {stage.name}")
        if entry.get("owner") != stage.owner:
            blockers.append(f"data-flow evidence owner mismatch for {stage.name}")
        if entry.get("frame_role") != stage.frame_role:
            blockers.append(f"data-flow evidence frame_role mismatch for {stage.name}")
        if entry.get("map_dependency") != stage.map_dependency:
            blockers.append(f"data-flow evidence map_dependency mismatch for {stage.name}")


def _observed_flow_tokens(
    tokens: Sequence[str],
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
) -> tuple[str, ...]:
    return tuple(
        str(token) for token in tokens if _runtime_flow_token_observed(str(token), topic_evidence, hardware_boundary)
    )


def _missing_flow_tokens(
    tokens: Sequence[str],
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
) -> tuple[str, ...]:
    return tuple(
        str(token)
        for token in tokens
        if not _runtime_flow_token_observed(str(token), topic_evidence, hardware_boundary)
    )


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


def _tuple_value(value: Any) -> tuple[str, ...]:
    if isinstance(value, str):
        return (value,)
    if isinstance(value, Sequence):
        return tuple(str(item) for item in value)
    return ()


def _command_subscribers(hardware_safety: Mapping[str, Any]) -> tuple[str, ...]:
    topics = _mapping(hardware_safety.get("topics")) or {}
    subscribers: list[str] = []
    for topic, topic_subscribers in topics.items():
        if "cmd_vel" not in str(topic):
            continue
        if isinstance(topic_subscribers, str):
            subscribers.append(topic_subscribers)
        elif isinstance(topic_subscribers, Sequence):
            subscribers.extend(str(item) for item in topic_subscribers)
    return tuple(subscribers)


def _looks_like_hardware_command_subscriber(name: str) -> bool:
    normalized = name.lower()
    hardware_terms = (
        "hardware",
        "driver",
        "motor",
        "chassis",
        "actuator",
        "s100p",
        "thunder",
        "unitree",
    )
    return any(term in normalized for term in hardware_terms)
