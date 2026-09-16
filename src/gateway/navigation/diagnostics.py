"""Read-only navigation paths and native execution evidence."""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from typing import Any

from gateway.schemas import NavigationDdsSnapshotResponse, PathResponse
from gateway.services.native_status import read_navigation_status, read_traversability_status
from gateway.services.telemetry_normalizers import build_path_response


def _native_path_points(payload: Mapping[str, Any] | None, key: str) -> list[dict[str, Any]]:
    if not isinstance(payload, Mapping):
        return []
    raw_path = payload.get(key)
    if not isinstance(raw_path, Sequence) or isinstance(raw_path, str):
        return []

    points: list[dict[str, Any]] = []
    for item in raw_path:
        if isinstance(item, Mapping):
            point = dict(item)
        elif isinstance(item, Sequence) and not isinstance(item, str) and len(item) >= 2:
            point = {
                "x": item[0],
                "y": item[1],
                "z": item[2] if len(item) >= 3 else 0.0,
            }
        else:
            continue
        point.setdefault("frame_id", "map")
        points.append(point)
    return points


def _native_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def _native_int(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _operator_motion_twist_payload(twist: Any) -> dict[str, Any]:
    if not isinstance(twist, Mapping):
        return {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
    return {
        "linear": {
            "x": _native_float(twist.get("vx")),
            "y": _native_float(twist.get("vy")),
            "z": 0.0,
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": _native_float(twist.get("wz")),
        },
    }


def _native_operator_motion_trace(payload: Mapping[str, Any] | None) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        return {}
    operator_motion = payload.get("operator_motion")
    if not isinstance(operator_motion, Mapping):
        return {}

    operator_status = operator_motion.get("status")
    operator_status = operator_status if isinstance(operator_status, Mapping) else {}
    last_ack = operator_motion.get("last_ack")
    last_ack = last_ack if isinstance(last_ack, Mapping) else {}

    teleop_output = operator_status.get("teleop_output")
    final_cmd_vel = operator_status.get("final_cmd_vel")
    input_gate_reason = operator_status.get("input_gate_reason")
    teleop_reason = operator_status.get("authority_reason")

    return {
        "operator_motion": {
            "schema_version": _native_int(operator_motion.get("schema_version")),
            "interface_enabled": operator_motion.get("interface_enabled") is True,
            "authority_owner": operator_motion.get("authority_owner"),
            "control_mode": operator_motion.get("control_mode"),
            "allow_teleop_takeover": operator_motion.get("allow_teleop_takeover") is True,
            "teleop_output": _operator_motion_twist_payload(teleop_output),
            "final_cmd_vel": _operator_motion_twist_payload(final_cmd_vel),
            "teleop": {
                "reason": str(teleop_reason or ""),
                "output": _operator_motion_twist_payload(teleop_output),
            },
            "input_gate": {
                "reason": str(input_gate_reason or ""),
            },
            "last_ack": {
                "observed": last_ack.get("observed") is True,
                "published": last_ack.get("published") is True,
                "source_id": str(last_ack.get("source_id") or ""),
                "request_id": str(last_ack.get("request_id") or ""),
                "source_sequence": _native_int(last_ack.get("source_sequence")),
                "final_output_sequence": _native_int(last_ack.get("final_output_sequence")),
                "accepted": last_ack.get("accepted") is True,
                "reason": str(last_ack.get("reason") or ""),
            },
            "status": {
                "observed": operator_status.get("observed") is True,
                "published": operator_status.get("published") is True,
                "has_active_sample": operator_status.get("has_active_sample") is True,
                "holding": operator_status.get("holding") is True,
                "has_active_authority": operator_status.get("has_active_authority") is True,
                "last_sample_sequence": _native_int(
                    operator_status.get("last_sample_sequence")
                ),
                "admitted_sequence": _native_int(operator_status.get("admitted_sequence")),
                "final_output_sequence": _native_int(
                    operator_status.get("final_output_sequence")
                ),
            },
        }
    }


def _native_cmd_vel_payload(payload: Mapping[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(payload, Mapping):
        return None

    control_mode = str(payload.get("control_mode") or "")
    active_cmd_source = str(payload.get("active_cmd_source") or "")
    teleop_active = (
        control_mode in {"teleop", "teleop_avoid"}
        or active_cmd_source == "teleop"
    )
    final_output = payload.get("final_output")
    final_output = final_output if isinstance(final_output, Mapping) else {}
    output_sequence = _native_int(final_output.get("output_sequence"))
    final_output_published = (
        payload.get("publish_cmd_vel") is True
        and final_output.get("published") is True
        and output_sequence > 0
    )

    cmd = payload.get("final_cmd_vel")
    if isinstance(cmd, Mapping):
        active_source = "native_teleop" if teleop_active else "native_nav_endpoint"
        if not final_output_published:
            active_source = f"{active_source}_preview"
        evidence_stage = (
            "final_output_published"
            if final_output_published
            else "final_policy_output_not_published"
        )
    else:
        teleop = payload.get("teleop")
        cmd = (
            teleop.get("output")
            if teleop_active and isinstance(teleop, Mapping)
            else None
        )
        if isinstance(cmd, Mapping):
            active_source = "native_teleop_policy_preview"
            evidence_stage = "teleop_policy_output"
        else:
            last_local = payload.get("last_local")
            cmd = last_local.get("cmd_vel") if isinstance(last_local, Mapping) else None
            active_source = "native_local_planner_preview"
            evidence_stage = "local_planner_output"
    if not isinstance(cmd, Mapping):
        return None

    cmd_vel_payload = {
        "frame_id": "base_link",
        "linear": {
            "x": _native_float(cmd.get("vx")),
            "y": _native_float(cmd.get("vy")),
            "z": 0.0,
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": _native_float(cmd.get("wz")),
        },
        "active_source": active_source,
        "evidence_stage": evidence_stage,
        "final_output_confirmed": final_output_published,
        "driver_delivery_accepted": (
            final_output_published and final_output.get("driver_delivery_accepted") is True
        ),
        "output_sequence": output_sequence if final_output_published else 0,
        "ts": payload.get("stamp_s"),
    }

    operator_motion = _native_operator_motion_trace(payload)
    if operator_motion:
        cmd_vel_payload["operator_motion"] = operator_motion.get("operator_motion")
    return cmd_vel_payload


def register_navigation_diagnostic_routes(app, gw) -> None:
    @app.get(
        "/api/v1/path",
        summary="Latest planned path",
        response_model=PathResponse,
    )
    async def get_path():
        with gw._state_lock:
            path = gw._last_path
            robot = gw._odom
        return build_path_response(path, robot)

    @app.get(
        "/api/v1/navigation/dds_snapshot",
        summary="Latest navigation data for the native DDS endpoint",
        response_model=NavigationDdsSnapshotResponse,
    )
    async def get_navigation_dds_snapshot():
        with gw._state_lock:
            global_path = list(gw._last_path)
            local_path = list(gw._last_local_path)
            robot = gw._odom
            navigation_state = (
                dict(gw._navigation_state)
                if isinstance(gw._navigation_state, Mapping)
                else {}
            )
        nav_endpoint = read_navigation_status()
        traversability_endpoint = read_traversability_status()
        if not global_path:
            global_path = _native_path_points(nav_endpoint, "global_path")
        if not local_path:
            local_path = _native_path_points(nav_endpoint, "local_path")
        operator_motion = _native_operator_motion_trace(nav_endpoint)
        cmd_payload = _native_cmd_vel_payload(nav_endpoint)
        if cmd_payload is not None and operator_motion:
            cmd_payload["operator_motion"] = operator_motion.get("operator_motion")
        return {
            "schema_version": "lingtu.navigation.dds_snapshot.v1",
            "global_path": build_path_response(global_path, robot),
            "local_path": build_path_response(local_path, robot),
            "cmd_vel": cmd_payload,
            "nav_endpoint": nav_endpoint,
            "traversability_endpoint": traversability_endpoint,
            "navigation_state": navigation_state,
            "ts": time.time(),
            "source": "gateway_navigation_cache+native_status",
        }
