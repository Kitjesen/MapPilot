#!/usr/bin/env python3
"""Collect read-only Thunder field runtime evidence.

This script does not publish goals, cmd_vel, or any robot-control topic. It
subscribes to existing ROS 2 topics, samples TF, inspects the command subscriber
graph, writes a runtime report, and optionally validates that report with the
Thunder field evidence gate.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[2]


def _ensure_import_path() -> None:
    for candidate in (ROOT / "src", ROOT):
        path = str(candidate)
        if path not in sys.path:
            sys.path.insert(0, path)


_ensure_import_path()

from core.runtime_evidence import (
    REAL_HARDWARE_COMMAND_SINK,
    REAL_RUNTIME_COLLECTOR_NAME,
    REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED,
    REAL_RUNTIME_EVIDENCE_REPORT_SCHEMA,
    real_runtime_evidence_payload,
    runtime_data_flow_stage_observation,
    runtime_data_flow_stage_required,
    runtime_data_flow_stage_signals,
    validate_real_runtime_evidence,
)
from core.runtime_interface import (
    FRAME_LINKS,
    TOPICS,
    runtime_algorithm_interface_contract,
    runtime_frames_contract,
    runtime_required_topic_frame_ids,
    runtime_stage_algorithm_interface_contract,
    runtime_topic_allowed_frame_ids,
    runtime_topic_allowed_frame_contract,
    runtime_topic_default_frame_ids,
    runtime_topic_default_frame_contract,
    resolved_runtime_data_flow,
    runtime_data_flow_topics,
)
from core.runtime_validation_gates import runtime_validation_gates


OBSERVED_TOPICS = runtime_data_flow_topics(REAL_RUNTIME_CONTRACT)
REAL_RUNTIME_VALIDATION_GATE = runtime_validation_gates()["real_runtime_evidence"]

SIMULATION_SINK_TERMS = (
    "mujoco",
    "gazebo",
    "unity",
    "simulator",
    "simulation",
    "rosbag",
    "replay",
)

HARDWARE_SINK_TERMS = (
    "hardware",
    "driver",
    "motor",
    "chassis",
    "actuator",
    "s100p",
    "thunder",
    "unitree",
)


def _jsonable_tuple(value: Sequence[str]) -> list[str]:
    return [str(item) for item in value]


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
        return _positive_number(entry.get("nonzero_samples")) or _positive_number(
            entry.get("max_norm")
        )
    if require_nonempty:
        return any(
            _positive_number(entry.get(key))
            for key in ("nonempty_samples", "max_poses", "points", "cells")
        )
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
        ok, reason = runtime_data_flow_stage_observation(
            stage.name,
            topic_evidence,
            hardware_boundary,
        )
        required = runtime_data_flow_stage_required(
            stage.name,
            topic_evidence,
            hardware_boundary,
        )
        signals = runtime_data_flow_stage_signals(
            stage.name,
            topic_evidence,
            hardware_boundary,
        )
        evidence[stage.name] = {
            "ok": ok,
            "required": required,
            "inputs": _jsonable_tuple(stage.inputs),
            "outputs": _jsonable_tuple(stage.outputs),
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
            "observed_signals": [
                name for name, observed in signals.items() if observed
            ],
            "missing_signals": [
                name for name, observed in signals.items() if not observed
            ],
            "owner": stage.owner,
            "frame_role": stage.frame_role,
            "map_dependency": stage.map_dependency,
            "reason": reason,
        }
    return evidence


def build_required_topic_frame_contract(
    data_source: str = REAL_RUNTIME_CONTRACT,
) -> dict[str, dict[str, Any]]:
    required_topics = runtime_required_topic_frame_ids(data_source)
    default_frame_ids = runtime_topic_default_frame_ids(data_source)
    allowed_frame_ids = runtime_topic_allowed_frame_ids(data_source)
    return {
        topic: {
            "default_frame_id": default_frame_ids[topic],
            "allowed_frame_ids": list(allowed_frame_ids[topic]),
        }
        for topic in required_topics
    }


def _observed_tokens(
    tokens: Sequence[str],
    topic_evidence: Mapping[str, Any],
    hardware_boundary: Mapping[str, Any],
) -> list[str]:
    return [
        str(token)
        for token in tokens
        if _runtime_flow_token_observed(str(token), topic_evidence, hardware_boundary)
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


def _looks_like_simulation_sink(name: str) -> bool:
    normalized = name.lower()
    return any(term in normalized for term in SIMULATION_SINK_TERMS)


def _looks_like_hardware_sink(name: str) -> bool:
    normalized = name.lower()
    return any(term in normalized for term in HARDWARE_SINK_TERMS)


def _matches_expected_subscriber(name: str, expected: Sequence[str]) -> bool:
    normalized = name.lower().strip("/")
    for candidate in expected:
        expected_name = candidate.lower().strip("/")
        if expected_name and (expected_name == normalized or expected_name in normalized):
            return True
    return False


def build_hardware_boundary(
    command_subscribers: Sequence[str],
    expected_command_subscribers: Sequence[str] = (),
) -> dict[str, Any]:
    subscribers = sorted(dict.fromkeys(str(item) for item in command_subscribers if item))
    unexpected_simulation_sinks = [
        name for name in subscribers if _looks_like_simulation_sink(name)
    ]
    hardware_route_observed = any(
        _looks_like_hardware_sink(name)
        or _matches_expected_subscriber(name, expected_command_subscribers)
        for name in subscribers
    )
    return {
        "command_sink": REAL_HARDWARE_COMMAND_SINK,
        "hardware_command_route_observed": hardware_route_observed,
        "command_subscribers": subscribers,
        "expected_command_subscribers": list(expected_command_subscribers),
        "unexpected_simulation_sinks": unexpected_simulation_sinks,
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
    cmd_sent_to_hardware = (
        command_nonzero
        and hardware_boundary["hardware_command_route_observed"] is True
        and not hardware_boundary["unexpected_simulation_sinks"]
    )

    return {
        "schema_version": REAL_RUNTIME_EVIDENCE_REPORT_SCHEMA,
        "collector": {
            "name": REAL_RUNTIME_COLLECTOR_NAME,
            "read_only": True,
            "duration_sec": float(duration_sec),
            "control_topics_published": list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED),
        },
        "validation_gate": REAL_RUNTIME_VALIDATION_GATE,
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
            "frames": runtime_frames_contract(),
            "topic_default_frame_ids": runtime_topic_default_frame_contract(
                REAL_RUNTIME_CONTRACT
            ),
            "topic_allowed_frame_ids": runtime_topic_allowed_frame_contract(
                REAL_RUNTIME_CONTRACT
            ),
            "algorithm_interfaces": runtime_algorithm_interface_contract(),
            "runtime_data_flow_stage_algorithm_interfaces": (
                runtime_stage_algorithm_interface_contract()
            ),
            "required_topic_frame_contract": build_required_topic_frame_contract(
                REAL_RUNTIME_CONTRACT
            ),
            "topic_evidence": {
                topic: _topic_summary(topic_evidence, topic)
                for topic in OBSERVED_TOPICS
            },
            "frame_evidence": frame_evidence,
            "data_flow_evidence": data_flow_evidence,
        },
    }


def _topic_types_by_name(node) -> dict[str, list[str]]:
    return {
        name: list(types)
        for name, types in node.get_topic_names_and_types()
    }


def _endpoint_name(endpoint: Any) -> str:
    namespace = getattr(endpoint, "node_namespace", "") or ""
    name = getattr(endpoint, "node_name", "") or ""
    namespace = namespace.rstrip("/")
    if not namespace:
        return f"/{name}" if name else ""
    return f"{namespace}/{name}".replace("//", "/")


def _command_subscribers(node) -> list[str]:
    subscribers = []
    for endpoint in node.get_subscriptions_info_by_topic(TOPICS.cmd_vel):
        name = _endpoint_name(endpoint)
        if name:
            subscribers.append(name)
    return subscribers


def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _message_frame_id(msg: Any) -> str | None:
    header = getattr(msg, "header", None)
    frame_id = getattr(header, "frame_id", None)
    return str(frame_id) if frame_id else None


def _message_points(msg: Any) -> int | None:
    width = getattr(msg, "width", None)
    height = getattr(msg, "height", None)
    if width is not None and height is not None:
        try:
            return int(width) * int(height)
        except (TypeError, ValueError):
            return None
    points = getattr(msg, "points", None)
    if points is not None:
        try:
            return len(points)
        except TypeError:
            return None
    return None


def _message_cells(msg: Any) -> int | None:
    info = getattr(msg, "info", None)
    width = getattr(info, "width", None)
    height = getattr(info, "height", None)
    if width is None or height is None:
        return None
    try:
        return int(width) * int(height)
    except (TypeError, ValueError):
        return None


def _message_path_poses(msg: Any) -> int | None:
    poses = getattr(msg, "poses", None)
    if poses is None:
        return None
    try:
        return len(poses)
    except TypeError:
        return None


def _message_cmd_norm(msg: Any) -> float | None:
    twist = getattr(msg, "twist", msg)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    if linear is None or angular is None:
        return None
    lx = _safe_float(getattr(linear, "x", 0.0)) or 0.0
    ly = _safe_float(getattr(linear, "y", 0.0)) or 0.0
    az = _safe_float(getattr(angular, "z", 0.0)) or 0.0
    return math.sqrt(lx * lx + ly * ly + az * az)


def _message_odom_position(msg: Any) -> tuple[float, float, float] | None:
    pose = getattr(msg, "pose", None)
    pose = getattr(pose, "pose", None)
    position = getattr(pose, "position", None)
    if position is None:
        return None
    x = _safe_float(getattr(position, "x", None))
    y = _safe_float(getattr(position, "y", None))
    z = _safe_float(getattr(position, "z", None))
    if x is None or y is None or z is None:
        return None
    return (x, y, z)


def _record_message(
    topic_evidence: dict[str, dict[str, Any]],
    odom_positions: list[tuple[float, float, float]],
    topic: str,
    msg: Any,
    min_cmd_vel_norm: float,
    sample_time_sec: float,
) -> None:
    entry = topic_evidence.setdefault(topic, {})
    entry["samples"] = int(entry.get("samples", 0)) + 1
    entry["ok"] = True
    first_seen = _safe_float(entry.get("first_seen_sec"))
    if first_seen is None:
        entry["first_seen_sec"] = sample_time_sec
        first_seen = sample_time_sec
    entry["last_seen_sec"] = sample_time_sec
    entry["sample_span_sec"] = max(0.0, sample_time_sec - first_seen)

    frame_id = _message_frame_id(msg)
    if frame_id:
        entry["frame_id"] = frame_id

    data = getattr(msg, "data", None)
    if isinstance(data, str):
        entry["data"] = data
    else:
        value = _safe_float(data)
        if value is not None:
            entry["value"] = value

    points = _message_points(msg)
    if points is not None:
        entry["points"] = max(int(entry.get("points", 0)), points)

    cells = _message_cells(msg)
    if cells is not None:
        entry["cells"] = max(int(entry.get("cells", 0)), cells)

    poses = _message_path_poses(msg)
    if poses is not None:
        entry["max_poses"] = max(int(entry.get("max_poses", 0)), poses)
        if poses > 0:
            entry["nonempty_samples"] = int(entry.get("nonempty_samples", 0)) + 1

    cmd_norm = _message_cmd_norm(msg)
    if cmd_norm is not None:
        entry["max_norm"] = max(float(entry.get("max_norm", 0.0)), cmd_norm)
        if cmd_norm >= min_cmd_vel_norm:
            entry["nonzero_samples"] = int(entry.get("nonzero_samples", 0)) + 1

    odom_position = _message_odom_position(msg)
    if odom_position is not None:
        odom_positions.append(odom_position)


def _transform_is_finite(transform: Any) -> bool:
    trans = transform.transform.translation
    rot = transform.transform.rotation
    values = (trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w)
    return all(
        _safe_float(value) is not None
        for value in values
    )


def _sample_frame_links(buffer, rclpy_time) -> tuple[dict[str, int], dict[str, str]]:
    from tf2_ros import TransformException

    frame_samples = {name: 0 for name in FRAME_LINKS}
    frame_errors: dict[str, str] = {}
    for name, link in FRAME_LINKS.items():
        try:
            transform = buffer.lookup_transform(link.parent, link.child, rclpy_time())
        except TransformException as exc:
            frame_errors[name] = str(exc)
            continue
        if _transform_is_finite(transform):
            frame_samples[name] += 1
        else:
            frame_errors[name] = "transform has non-finite values"
    return frame_samples, frame_errors


def run_collect(args: argparse.Namespace) -> dict[str, Any]:
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer, TransformListener

    rclpy.init(args=None)
    node = Node(f"lingtu_{REAL_RUNTIME_COLLECTOR_NAME}")
    buffer = Buffer()
    TransformListener(buffer, node)

    topic_evidence: dict[str, dict[str, Any]] = {
        topic: {"ok": False, "samples": 0}
        for topic in OBSERVED_TOPICS
    }
    odom_positions: list[tuple[float, float, float]] = []
    subscriptions = []
    subscribed_topics: set[str] = set()

    def refresh_subscriptions() -> None:
        topic_types = _topic_types_by_name(node)
        for topic in OBSERVED_TOPICS:
            types = topic_types.get(topic, [])
            entry = topic_evidence.setdefault(topic, {"ok": False, "samples": 0})
            entry["graph_exists"] = bool(types)
            entry["types"] = types
            if not types or topic in subscribed_topics:
                continue
            try:
                msg_type = get_message(types[0])
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                entry["subscription_error"] = str(exc)
                continue

            def callback(msg, topic_name=topic):
                sample_time_sec = min(
                    max(0.0, time.monotonic() - started_at),
                    float(args.duration_sec),
                )
                _record_message(
                    topic_evidence,
                    odom_positions,
                    topic_name,
                    msg,
                    args.min_cmd_vel_norm,
                    sample_time_sec,
                )

            subscriptions.append(node.create_subscription(msg_type, topic, callback, 10))
            subscribed_topics.add(topic)

    try:
        started_at = time.monotonic()
        refresh_subscriptions()
        frame_samples = {name: 0 for name in FRAME_LINKS}
        frame_errors: dict[str, str] = {}
        deadline = started_at + float(args.duration_sec)
        while time.monotonic() < deadline:
            refresh_subscriptions()
            rclpy.spin_once(node, timeout_sec=0.1)
            samples, errors = _sample_frame_links(buffer, rclpy.time.Time)
            for name, count in samples.items():
                frame_samples[name] = frame_samples.get(name, 0) + count
            frame_errors.update(errors)
        command_subscribers = _command_subscribers(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return build_real_runtime_report(
        topic_evidence=topic_evidence,
        frame_samples=frame_samples,
        frame_errors=frame_errors,
        command_subscribers=command_subscribers,
        expected_command_subscribers=args.expected_command_subscriber,
        duration_sec=args.duration_sec,
        odom_positions=odom_positions,
        min_motion_m=args.min_motion_m,
    )


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
    payload = real_runtime_evidence_payload(result, expected_contract, report=report)
    payload["validation_gate"] = REAL_RUNTIME_VALIDATION_GATE
    return payload


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect read-only Thunder field runtime evidence.",
    )
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--min-motion-m", type=float, default=0.05)
    parser.add_argument("--min-cmd-vel-norm", type=float, default=0.01)
    parser.add_argument(
        "--expected-command-subscriber",
        action="append",
        default=[],
        help="Observed command subscriber name or substring accepted as hardware route.",
    )
    parser.add_argument("--expected-contract", default=REAL_RUNTIME_CONTRACT)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--no-validate", action="store_true")
    args = parser.parse_args(argv)
    if args.expected_contract != REAL_RUNTIME_CONTRACT:
        parser.error(
            f"real runtime evidence only supports expected contract "
            f"{REAL_RUNTIME_CONTRACT}"
        )
    return args


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    collection_error = False
    try:
        report = run_collect(args)
    except ImportError as exc:
        collection_error = True
        report = build_unavailable_real_runtime_report(
            duration_sec=args.duration_sec,
            error=f"ROS 2 Python dependencies unavailable: {exc}",
            expected_contract=args.expected_contract,
            expected_command_subscribers=args.expected_command_subscriber,
            min_motion_m=args.min_motion_m,
        )

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
