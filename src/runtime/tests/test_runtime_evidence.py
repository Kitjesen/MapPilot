from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import importlib.util
import json
import subprocess
import sys
from argparse import Namespace
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import yaml

from runtime.runtime_evidence import (
    REAL_HARDWARE_COMMAND_SINK,
    REAL_RUNTIME_COLLECTOR_NAME,
    REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED,
    REAL_RUNTIME_EVIDENCE_VALIDATION_SCHEMA,
    real_runtime_evidence_payload,
    validate_real_runtime_evidence,
    validate_runtime_evidence,
)
from runtime.runtime_interface import (
    FRAME_LINKS,
    LEGACY_REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
    REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS,
    RUNTIME_DATA_FLOW,
    RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
    THUNDER_FIELD_RUNTIME_CONTRACT,
    TOPICS,
    adapter_source_for_target,
    body_frame_id,
    camera_frame_id,
    canonical_data_source_name,
    lidar_frame_id,
    map_frame_id,
    normalize_algorithm_interface_contract,
    normalize_runtime_frames_contract,
    odom_frame_id,
    real_lidar_frame_id,
    resolved_runtime_data_flow,
    runtime_algorithm_interface_contract,
    runtime_contract_manifest,
    runtime_data_flow_topics,
    runtime_frames_contract,
    runtime_required_topic_frame_ids,
    runtime_stage_algorithm_interface_contract,
    runtime_topic_allowed_frame_contract,
    runtime_topic_allowed_frame_ids,
    runtime_topic_default_frame_contract,
    runtime_topic_default_frame_id,
    runtime_topic_default_frame_ids,
    simulator_world_frame_id,
    topic_default_frame_id,
)
from runtime.runtime_interface import (
    REAL_RUNTIME_CONTRACT as INTERFACE_REAL_RUNTIME_CONTRACT,
)
from runtime.runtime_validation_gates import (
    REAL_RUNTIME_EVIDENCE_COLLECTOR_COMMAND,
    REAL_RUNTIME_EVIDENCE_COMMAND,
    REAL_RUNTIME_EVIDENCE_GATE_COMMAND,
    REAL_RUNTIME_EVIDENCE_OPERATOR_SUMMARY_SECTIONS,
    REAL_RUNTIME_EVIDENCE_PROVES,
    REAL_RUNTIME_EVIDENCE_VALIDATES,
    RUNTIME_AUDIT_CHECKS,
    RUNTIME_AUDIT_OPERATOR_SUMMARY_SECTIONS,
    RUNTIME_AUDIT_PROVES,
    RUNTIME_AUDIT_VALIDATE_CHECK_COVERAGE,
    RUNTIME_AUDIT_VALIDATES,
    SAVED_MAP_ARTIFACT_GATE_COMMAND,
    SAVED_MAP_ARTIFACT_GATE_OPERATOR_SUMMARY_SECTIONS,
    SAVED_MAP_ARTIFACT_GATE_PROVES,
    SAVED_MAP_ARTIFACT_GATE_VALIDATES,
    runtime_validation_gates,
    validate_runtime_validation_gates,
)

REPO_ROOT = Path(__file__).resolve().parents[3]


def _frame_evidence() -> dict:
    return {
        "map_to_odom": {
            "ok": True,
            "parent": "map",
            "child": "odom",
            "static": True,
        },
        "odom_to_body": {
            "ok": True,
            "parent": "odom",
            "child": "body",
            "samples": 4,
        },
        "body_to_lidar": {
            "ok": True,
            "parent": "body",
            "child": "lidar_link",
            "static": True,
        },
        "body_to_camera": {
            "ok": True,
            "parent": "body",
            "child": "camera_link",
            "static": True,
        },
    }


def _flow_token_observed(
    token: str,
    topic_evidence: dict[str, dict[str, object]] | None,
    hardware_boundary: dict[str, object] | None,
) -> bool:
    if token.startswith("/") and topic_evidence is not None:
        entry = topic_evidence.get(token) or {}
        return (
            float(entry.get("samples") or 0) > 0
            or entry.get("ok") is True
            or entry.get("graph_exists") is True
        )
    if hardware_boundary is None:
        return False
    if token == REAL_HARDWARE_COMMAND_SINK:
        return hardware_boundary.get("hardware_command_route_observed") is True
    if token == hardware_boundary.get("command_sink"):
        return hardware_boundary.get("hardware_command_route_observed") is True
    return False


def _observed_flow_tokens(
    tokens: tuple[str, ...],
    topic_evidence: dict[str, dict[str, object]] | None,
    hardware_boundary: dict[str, object] | None,
) -> list[str]:
    return [
        token
        for token in tokens
        if _flow_token_observed(token, topic_evidence, hardware_boundary)
    ]


def _missing_flow_tokens(
    tokens: tuple[str, ...],
    topic_evidence: dict[str, dict[str, object]] | None,
    hardware_boundary: dict[str, object] | None,
) -> list[str]:
    return [
        token
        for token in tokens
        if not _flow_token_observed(token, topic_evidence, hardware_boundary)
    ]


def _data_flow_evidence(
    data_source: str = "cmu_unity_external",
    topic_evidence: dict[str, dict[str, object]] | None = None,
    hardware_boundary: dict[str, object] | None = None,
) -> dict:
    evidence = {}
    for stage in resolved_runtime_data_flow(data_source):
        entry = {
            "ok": True,
            "inputs": list(stage.inputs),
            "outputs": list(stage.outputs),
            "owner": stage.owner,
            "frame_role": stage.frame_role,
            "map_dependency": stage.map_dependency,
            "reason": f"{stage.name}_observed",
        }
        if topic_evidence is not None or hardware_boundary is not None:
            entry.update({
                "observed_inputs": _observed_flow_tokens(
                    stage.inputs,
                    topic_evidence,
                    hardware_boundary,
                ),
                "missing_inputs": _missing_flow_tokens(
                    stage.inputs,
                    topic_evidence,
                    hardware_boundary,
                ),
                "observed_outputs": _observed_flow_tokens(
                    stage.outputs,
                    topic_evidence,
                    hardware_boundary,
                ),
                "missing_outputs": _missing_flow_tokens(
                    stage.outputs,
                    topic_evidence,
                    hardware_boundary,
                ),
            })
        evidence[stage.name] = entry
    return evidence


def _required_topic_frame_contract(data_source: str = REAL_RUNTIME_CONTRACT) -> dict:
    default_frame_ids = runtime_topic_default_frame_ids(data_source)
    allowed_frame_ids = runtime_topic_allowed_frame_ids(data_source)
    return {
        topic: {
            "default_frame_id": default_frame_ids[topic],
            "allowed_frame_ids": list(allowed_frame_ids[topic]),
        }
        for topic in runtime_required_topic_frame_ids(data_source)
    }


def _template_data_flow_evidence() -> dict:
    return {
        stage.name: {
            "ok": True,
            "inputs": list(stage.inputs),
            "outputs": list(stage.outputs),
            "owner": stage.owner,
            "frame_role": stage.frame_role,
            "map_dependency": stage.map_dependency,
            "reason": "template_stage_observed",
        }
        for stage in RUNTIME_DATA_FLOW
    }


@pytest.mark.sim
def test_simulator_world_frame_id_is_canonical_runtime_contract() -> None:
    assert simulator_world_frame_id() == "world"


def test_named_frame_helpers_are_canonical_runtime_contract() -> None:
    assert map_frame_id() == "map"
    assert odom_frame_id() == "odom"
    assert body_frame_id() == "body"
    assert lidar_frame_id() == "lidar_link"
    assert real_lidar_frame_id() == "livox_frame"
    assert camera_frame_id() == "camera_link"


def test_core_message_defaults_use_runtime_frame_helpers() -> None:
    from runtime.msgs.geometry import PoseStamped, Transform, TwistStamped
    from runtime.msgs.gnss import GnssOdom
    from runtime.msgs.nav import OccupancyGrid, Odometry, Path
    from runtime.msgs.semantic import GoalResult, SceneGraph
    from runtime.msgs.sensor import Image, PointCloud2

    assert PoseStamped().frame_id == map_frame_id()
    assert TwistStamped().frame_id == body_frame_id()

    transform = Transform()
    assert transform.frame_id == map_frame_id()
    assert transform.child_frame_id == body_frame_id()

    odom = Odometry()
    assert odom.frame_id == odom_frame_id()
    assert odom.child_frame_id == body_frame_id()

    assert Path().frame_id == map_frame_id()
    assert OccupancyGrid().frame_id == map_frame_id()
    assert SceneGraph().frame_id == map_frame_id()
    assert GoalResult().as_pose_stamped().frame_id == map_frame_id()
    assert PointCloud2().frame_id == map_frame_id()
    assert GnssOdom().frame_id == map_frame_id()
    assert Image().frame_id == camera_frame_id()


def test_adapter_source_for_target_exposes_native_runtime_boundaries() -> None:
    assert adapter_source_for_target("fastlio2", TOPICS.odometry) == "/Odometry"
    assert adapter_source_for_target("localizer", TOPICS.saved_map_cloud) == "map_cloud"
    assert (
        adapter_source_for_target("localizer", TOPICS.global_relocalize_service)
        == "global_relocalize"
    )
    assert (
        adapter_source_for_target(
            "localizer",
            TOPICS.global_relocalize_status_service,
        )
        == "global_relocalize_status"
    )


def _safe_report() -> dict:
    return {
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "outputs": {
            "nav_odometry": 5,
            "nav_map_cloud": 4,
            "nav_cmd_vel_nonzero": 3,
        },
        "paths": {
            "/nav/global_path": {
                "samples": 1,
                "nonempty_samples": 1,
                "max_poses": 4,
            },
            "/nav/local_path": {
                "samples": 1,
                "nonempty_samples": 1,
                "max_poses": 3,
            },
        },
        "cmd_vel": {
            "samples": 5,
            "nonzero_samples": 3,
            "max_norm": 0.2,
        },
        "hardware_safety": {
            "topics": {"/cmd_vel": ["/vehicle_simulator"]},
            "blocked_hardware_nodes": [],
            "unexpected_command_publishers": [],
        },
        "runtime_contract": {
            "name": "cmu_unity_external",
            "ok": True,
            "frame_evidence": _frame_evidence(),
            "data_flow_evidence": _data_flow_evidence(),
            "topic_evidence": {
                "/nav/global_path": {"ok": True, "samples": 1, "max_poses": 4},
                "/nav/local_path": {"ok": True, "samples": 1, "max_poses": 3},
                "/nav/cmd_vel": {"ok": True, "nonzero_samples": 3},
            },
        },
    }


def _add_topic_sample_windows(
    topic_evidence: dict[str, dict[str, object]],
    topics: tuple[str, ...],
    *,
    duration_sec: float = 20.0,
) -> dict[str, dict[str, object]]:
    for index, topic in enumerate(topics):
        entry = topic_evidence[topic]
        first_seen = 0.1 + index * 0.01
        last_seen = min(duration_sec, first_seen + 1.0)
        entry["first_seen_sec"] = first_seen
        entry["last_seen_sec"] = last_seen
        entry["sample_span_sec"] = last_seen - first_seen
    return topic_evidence


def _add_valid_localization_evidence(
    topic_evidence: dict[str, dict[str, object]],
) -> dict[str, dict[str, object]]:
    topic_evidence[TOPICS.localization_health] = {
        "ok": True,
        "samples": 3,
        "data": "LOCKED|fitness=0.0234|iter=8|cov=0.12",
    }
    topic_evidence[TOPICS.localization_quality] = {
        "ok": True,
        "samples": 3,
        "value": 0.0234,
    }
    _add_topic_sample_windows(
        topic_evidence,
        (TOPICS.localization_health, TOPICS.localization_quality),
    )
    return topic_evidence


def _mark_live_topic_fresh(
    topic_evidence: dict[str, dict[str, object]],
    topics: tuple[str, ...],
    *,
    duration_sec: float = 20.0,
    age_sec: float = 0.5,
) -> dict[str, dict[str, object]]:
    fresh_seen = duration_sec - age_sec
    for topic in topics:
        entry = topic_evidence[topic]
        first_seen = float(entry["first_seen_sec"])
        entry["last_seen_sec"] = fresh_seen
        entry["sample_span_sec"] = fresh_seen - first_seen
    return topic_evidence


def _real_report() -> dict:
    topic_evidence = {
        topic: {"ok": False, "samples": 0, "graph_exists": False}
        for topic in runtime_data_flow_topics(REAL_RUNTIME_CONTRACT)
    }
    topic_evidence.update({
        TOPICS.lidar_scan: {
            "ok": True,
            "samples": 4,
            "points": 12000,
            "frame_id": "lidar_link",
        },
        TOPICS.imu: {
            "ok": True,
            "samples": 20,
            "frame_id": "lidar_link",
        },
        "/nav/odometry": {"ok": True, "samples": 6, "frame_id": "odom"},
        "/nav/registered_cloud": {
            "ok": True,
            "samples": 5,
            "points": 9000,
            "frame_id": "body",
        },
        "/nav/map_cloud": {
            "ok": True,
            "samples": 5,
            "points": 45000,
            "frame_id": "map",
        },
        "/nav/global_path": {
            "ok": True,
            "samples": 2,
            "max_poses": 12,
            "frame_id": "map",
        },
        "/nav/local_path": {
            "ok": True,
            "samples": 3,
            "max_poses": 9,
            "frame_id": "body",
        },
        "/nav/cmd_vel": {
            "ok": True,
            "samples": 5,
            "nonzero_samples": 5,
            "frame_id": "body",
        },
    })
    _add_topic_sample_windows(
        topic_evidence,
        REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS,
    )
    _add_topic_sample_windows(
        topic_evidence,
        REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
    )
    _add_valid_localization_evidence(topic_evidence)
    _mark_live_topic_fresh(
        topic_evidence,
        (
            TOPICS.lidar_scan,
            TOPICS.imu,
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.localization_health,
            TOPICS.localization_quality,
            TOPICS.local_path,
            TOPICS.cmd_vel,
        ),
    )
    hardware_boundary = {
        "command_sink": REAL_HARDWARE_COMMAND_SINK,
        "hardware_command_route_observed": True,
        "command_subscribers": ["/thunder_driver"],
        "unexpected_simulation_sinks": [],
    }
    return {
        "collector": {
            "name": REAL_RUNTIME_COLLECTOR_NAME,
            "read_only": True,
            "duration_sec": 20.0,
            "control_topics_published": list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED),
        },
        "simulation_only": False,
        "real_robot_motion": True,
        "cmd_vel_sent_to_hardware": True,
        "motion": {
            "odom_delta_m": 0.2,
            "min_motion_m": 0.05,
            "odom_position_samples": 3,
        },
        "outputs": {
            "global_path_count": 2,
            "local_path_count": 3,
            "nav_cmd_vel_nonzero": 5,
        },
        "cmd_vel": {
            "samples": 8,
            "nonzero_samples": 5,
            "max_norm": 0.3,
        },
        "paths": {
            "/nav/global_path": {
                "samples": 2,
                "nonempty_samples": 2,
                "max_poses": 12,
                "frame_id": "map",
            },
            "/nav/local_path": {
                "samples": 3,
                "nonempty_samples": 3,
                "max_poses": 9,
                "frame_id": "body",
            },
        },
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
            "required_topic_frame_contract": _required_topic_frame_contract(),
            "frame_evidence": _frame_evidence(),
            "data_flow_evidence": _data_flow_evidence(
                REAL_RUNTIME_CONTRACT,
                topic_evidence,
                hardware_boundary,
            ),
            "topic_evidence": topic_evidence,
        },
    }


def _load_real_runtime_collect_module():
    script = REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_collect.py"
    spec = importlib.util.spec_from_file_location(
        "real_runtime_evidence_collect_under_test",
        script,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_runtime_contract_audit_module():
    import cli.runtime_audit as audit

    return audit


@pytest.mark.sim
def test_safe_simulation_report_passes():
    result = validate_runtime_evidence(_safe_report(), "cmu_unity_external")

    assert result.ok is True
    assert result.blockers == ()


def test_declared_data_flow_evidence_can_be_required():
    result = validate_runtime_evidence(
        _safe_report(),
        "cmu_unity_external",
        require_data_flow=True,
    )

    assert result.ok is True
    assert result.blockers == ()


def test_template_data_flow_placeholders_fail_when_concrete_flow_is_required():
    report = _safe_report()
    report["runtime_contract"]["data_flow_evidence"] = _template_data_flow_evidence()

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_data_flow=True,
    )

    assert result.ok is False
    assert "data-flow evidence inputs mismatch for endpoint_adapter" in result.blockers
    assert "data-flow evidence outputs mismatch for command_boundary" in result.blockers


def test_missing_data_flow_stage_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["data_flow_evidence"].pop("command_boundary")

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_data_flow=True,
    )

    assert result.ok is False
    assert "data-flow evidence missing or failed for command_boundary" in result.blockers
    assert result.blockers.count(
        "data-flow evidence missing or failed for command_boundary"
    ) == 1


def test_unrequired_data_flow_stage_can_be_reported_as_not_run():
    report = _safe_report()
    stage = report["runtime_contract"]["data_flow_evidence"]["global_planning"]
    stage["required"] = False
    stage["ok"] = False
    stage["reason"] = "not_required_for_basic_slam_gate"

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_data_flow=True,
    )

    assert result.ok is True
    assert result.blockers == ()


def test_wrong_data_flow_output_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["data_flow_evidence"]["command_boundary"]["outputs"] = [
        REAL_HARDWARE_COMMAND_SINK
    ]

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_data_flow=True,
    )

    assert result.ok is False
    assert "data-flow evidence outputs mismatch for command_boundary" in result.blockers


def test_unknown_data_source_reports_explicit_data_flow_blocker():
    result = validate_runtime_evidence(
        _safe_report(),
        "unknown_runtime",
        require_data_flow=True,
    )

    assert result.ok is False
    assert any(
        blocker.startswith("resolved data-flow contract unavailable:")
        for blocker in result.blockers
    )


def test_wrong_data_flow_frame_role_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["data_flow_evidence"]["global_planning"]["frame_role"] = "body"

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_data_flow=True,
    )

    assert result.ok is False
    assert "data-flow evidence frame_role mismatch for global_planning" in result.blockers


def test_wrong_data_flow_owner_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["data_flow_evidence"]["global_planning"]["owner"] = (
        "endpoint_adapter"
    )

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_data_flow=True,
    )

    assert result.ok is False
    assert "data-flow evidence owner mismatch for global_planning" in result.blockers


def test_wrong_data_flow_map_dependency_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["data_flow_evidence"]["global_planning"][
        "map_dependency"
    ] = "none"

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_data_flow=True,
    )

    assert result.ok is False
    assert (
        "data-flow evidence map_dependency mismatch for global_planning"
        in result.blockers
    )


def test_topic_frame_id_mismatch_fails_when_evidence_reports_frame():
    report = _safe_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.registered_cloud] = {
        "ok": True,
        "samples": 1,
        "frame_id": "map",
    }

    result = validate_runtime_evidence(report, "cmu_unity_external")

    assert result.ok is False
    assert any(
        blocker.startswith(
            "topic frame_id mismatch for /nav/registered_cloud: map not in"
        )
        for blocker in result.blockers
    )


def test_body_alias_is_accepted_for_body_frame_topic_evidence():
    report = _safe_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.registered_cloud] = {
        "ok": True,
        "samples": 1,
        "frame_id": "base_link",
    }

    result = validate_runtime_evidence(report, "cmu_unity_external")

    assert result.ok is True
    assert result.blockers == ()


def test_map_cloud_accepts_map_or_odom_frame_evidence():
    for frame_id in ("map", "odom"):
        report = _safe_report()
        report["runtime_contract"]["topic_evidence"][TOPICS.map_cloud] = {
            "ok": True,
            "samples": 1,
            "frame_id": frame_id,
        }

        result = validate_runtime_evidence(report, "cmu_unity_external")

        assert result.ok is True
        assert result.blockers == ()


def test_topic_frame_id_normalization_accepts_leading_slash():
    report = _safe_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.map_cloud] = {
        "ok": True,
        "samples": 1,
        "frame_id": "/map",
    }
    report["runtime_contract"]["topic_evidence"][TOPICS.registered_cloud] = {
        "ok": True,
        "samples": 1,
        "frame_id": "/base_link",
    }

    result = validate_runtime_evidence(report, "cmu_unity_external")

    assert result.ok is True
    assert result.blockers == ()


def test_declared_frame_evidence_can_be_required():
    result = validate_runtime_evidence(
        _safe_report(),
        "cmu_unity_external",
        require_frame_links=True,
    )

    assert result.ok is True
    assert result.blockers == ()


def test_missing_frame_evidence_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["frame_evidence"].pop("body_to_lidar")

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_frame_links=True,
    )

    assert result.ok is False
    assert "frame evidence missing or failed for body_to_lidar" in result.blockers


def test_wrong_frame_child_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["frame_evidence"]["body_to_lidar"]["child"] = "map"

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_frame_links=True,
    )

    assert result.ok is False
    assert "frame evidence child mismatch for body_to_lidar" in result.blockers


def test_unobserved_frame_link_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["frame_evidence"]["odom_to_body"].pop("samples")

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_frame_links=True,
    )

    assert result.ok is False
    assert "frame evidence missing or failed for odom_to_body" in result.blockers


@pytest.mark.sim
def test_hardware_command_in_simulation_fails():
    report = _safe_report()
    report["cmd_vel_sent_to_hardware"] = True
    report["hardware_safety"]["topics"]["/cmd_vel"] = [
        "/vehicle_simulator",
        "/thunder_driver",
    ]

    result = validate_runtime_evidence(report, "cmu_unity_external")

    assert result.ok is False
    assert "cmd_vel_sent_to_hardware is not false" in result.blockers
    assert "hardware-looking command subscriber present" in result.blockers


def test_missing_paths_can_be_allowed():
    report = _safe_report()
    report.pop("paths")
    report["runtime_contract"]["topic_evidence"].pop("/nav/global_path")
    report["runtime_contract"]["topic_evidence"].pop("/nav/local_path")

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_paths=False,
    )

    assert result.ok is True


def test_missing_nav_command_can_be_allowed():
    report = _safe_report()
    report.pop("cmd_vel")
    report["runtime_contract"]["topic_evidence"].pop("/nav/cmd_vel")

    result = validate_runtime_evidence(
        report,
        "cmu_unity_external",
        require_command=False,
    )

    assert result.ok is True


def test_real_runtime_report_passes_with_hardware_boundary_frame_and_data_flow():
    result = validate_real_runtime_evidence(_real_report(), REAL_RUNTIME_CONTRACT)

    assert result.ok is True
    assert result.blockers == ()


def test_real_runtime_validator_rejects_non_real_expected_contract():
    result = validate_real_runtime_evidence(_real_report(), "cmu_unity_external")

    assert result.ok is False
    assert (
        f"real runtime evidence expected_contract is not {REAL_RUNTIME_CONTRACT}"
        in result.blockers
    )


def test_real_runtime_rejects_collector_that_publishes_control_topics():
    report = _real_report()
    report["collector"]["read_only"] = False
    report["collector"]["control_topics_published"] = [TOPICS.cmd_vel]

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "real runtime collector is not read-only" in result.blockers
    assert "real runtime collector published control topics" in result.blockers


def test_real_runtime_rejects_missing_runtime_topic_evidence_entry():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"].pop(TOPICS.goal_pose)

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert (
        "runtime topic evidence missing for /nav/goal_pose"
        in result.blockers
    )


def test_real_runtime_rejects_missing_required_topic_frame_contract():
    report = _real_report()
    report["runtime_contract"].pop("required_topic_frame_contract")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "required topic frame contract missing" in result.blockers


def test_real_runtime_rejects_missing_runtime_frames_contract():
    report = _real_report()
    report["runtime_contract"].pop("frames")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "runtime frames contract missing" in result.blockers


def test_real_runtime_rejects_runtime_frame_contract_drift():
    report = _real_report()
    report["runtime_contract"]["frames"]["axis_convention"] = "x_right_y_forward_z_up"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "runtime frame axis_convention mismatch" in result.blockers


def test_real_runtime_rejects_topic_default_frame_contract_drift():
    report = _real_report()
    report["runtime_contract"]["topic_default_frame_ids"][TOPICS.map_cloud] = "odom"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic default frame contract mismatch" in result.blockers


def test_real_runtime_rejects_topic_allowed_frame_contract_drift():
    report = _real_report()
    report["runtime_contract"]["topic_allowed_frame_ids"][TOPICS.map_cloud] = ["odom"]

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic allowed frame contract mismatch" in result.blockers


def test_real_runtime_rejects_missing_algorithm_interface_contract():
    report = _real_report()
    report["runtime_contract"].pop("algorithm_interfaces")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "runtime algorithm interface contract missing" in result.blockers


def test_real_runtime_rejects_algorithm_interface_contract_drift():
    report = _real_report()
    report["runtime_contract"]["algorithm_interfaces"]["fastlio_mapping"][
        "outputs"
    ] = [TOPICS.odometry]

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "runtime algorithm interface contract mismatch" in result.blockers


def test_real_runtime_rejects_missing_stage_algorithm_interface_contract():
    report = _real_report()
    report["runtime_contract"].pop("runtime_data_flow_stage_algorithm_interfaces")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "runtime stage algorithm interface contract missing" in result.blockers


def test_real_runtime_rejects_stage_algorithm_interface_contract_drift():
    report = _real_report()
    report["runtime_contract"]["runtime_data_flow_stage_algorithm_interfaces"][
        "global_planning"
    ] = ["fastlio_mapping"]

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "runtime stage algorithm interface contract mismatch" in result.blockers


def test_real_runtime_rejects_required_topic_frame_contract_drift():
    report = _real_report()
    report["runtime_contract"]["required_topic_frame_contract"][TOPICS.map_cloud][
        "allowed_frame_ids"
    ] = ["odom"]

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert (
        "required topic allowed frames mismatch for /nav/map_cloud"
        in result.blockers
    )


def test_real_runtime_rejects_simulation_flags():
    report = _real_report()
    report["simulation_only"] = True
    report["real_robot_motion"] = False
    report["cmd_vel_sent_to_hardware"] = False

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "simulation_only is not false" in result.blockers
    assert "real_robot_motion is not true" in result.blockers
    assert "cmd_vel_sent_to_hardware is not true" in result.blockers


def test_real_runtime_rejects_motion_flag_without_numeric_evidence():
    report = _real_report()
    report.pop("motion")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "real motion evidence missing" in result.blockers


def test_real_runtime_rejects_motion_below_declared_threshold():
    report = _real_report()
    report["motion"]["odom_delta_m"] = 0.01
    report["motion"]["min_motion_m"] = 0.05

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "real motion odom_delta_m below min_motion_m" in result.blockers


def test_real_runtime_rejects_motion_with_too_few_odom_samples():
    report = _real_report()
    report["motion"]["odom_position_samples"] = 1

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "real motion requires at least two odometry position samples" in result.blockers


def test_real_runtime_rejects_missing_required_topic_sample_window():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.map_cloud].pop("last_seen_sec")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic sample window missing for /nav/map_cloud" in result.blockers


def test_real_runtime_rejects_required_topic_sample_outside_collection_window():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.global_path][
        "last_seen_sec"
    ] = 30.0

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert (
        "topic sample window outside collection duration for /nav/global_path"
        in result.blockers
    )


def test_real_runtime_rejects_stale_local_planner_output_before_global_path():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.local_path].update({
        "first_seen_sec": 0.01,
        "last_seen_sec": 0.05,
        "sample_span_sec": 0.04,
    })

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert (
        "data-flow temporal order failed for local_planning_and_following"
        in result.blockers
    )


def test_real_runtime_rejects_stale_live_lidar_sample_at_collection_end():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.lidar_scan].update({
        "last_seen_sec": 1.0,
        "sample_span_sec": 0.9,
    })

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "live topic stale for /nav/lidar_scan" in result.blockers


def test_real_runtime_rejects_missing_localization_health_topic():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"].pop(TOPICS.localization_health)

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "localization health evidence missing" in result.blockers


def test_real_runtime_rejects_lost_localization_health():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.localization_health][
        "data"
    ] = "LOST|fitness=0.45"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "localization health state is not healthy: LOST" in result.blockers


def test_real_runtime_rejects_missing_localization_quality():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"].pop(TOPICS.localization_quality)

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "localization quality evidence missing" in result.blockers


def test_real_runtime_rejects_unhealthy_localization_quality():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.localization_health][
        "data"
    ] = "LOCKED|fitness=0.75"
    report["runtime_contract"]["topic_evidence"][TOPICS.localization_quality][
        "value"
    ] = 0.75

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "localization quality outside healthy range: 0.75" in result.blockers


def test_real_runtime_rejects_missing_hardware_command_boundary():
    report = _real_report()
    report["hardware_boundary"]["hardware_command_route_observed"] = False
    report["hardware_boundary"]["command_subscribers"] = []

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "real hardware command boundary missing" in result.blockers


def test_real_runtime_rejects_simulation_command_sink():
    report = _real_report()
    report["hardware_boundary"]["unexpected_simulation_sinks"] = [
        "mujoco_velocity_adapter"
    ]

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "simulation command sink present in real runtime" in result.blockers


def test_real_runtime_rejects_wrong_command_data_flow():
    report = _real_report()
    report["runtime_contract"]["data_flow_evidence"]["command_boundary"]["outputs"] = [
        "mujoco_velocity_adapter"
    ]

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "data-flow evidence outputs mismatch for command_boundary" in result.blockers


def test_real_runtime_rejects_mismatched_observed_data_flow_outputs():
    report = _real_report()
    report["runtime_contract"]["data_flow_evidence"]["command_boundary"][
        "observed_outputs"
    ] = []

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert (
        "data-flow evidence observed_outputs mismatch for command_boundary"
        in result.blockers
    )


def test_real_runtime_rejects_data_flow_stage_without_reason():
    report = _real_report()
    report["runtime_contract"]["data_flow_evidence"]["global_planning"].pop("reason")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "data-flow evidence reason missing for global_planning" in result.blockers


def test_real_runtime_data_flow_stage_with_blank_reason_fails():
    report = _real_report()
    report["runtime_contract"]["data_flow_evidence"]["local_planning_and_following"][
        "reason"
    ] = " "

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert (
        "data-flow evidence reason missing for local_planning_and_following"
        in result.blockers
    )


def test_real_runtime_rejects_claimed_data_flow_without_endpoint_samples():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.lidar_scan] = {
        "ok": False,
        "samples": 0,
        "graph_exists": False,
    }
    report["runtime_contract"]["topic_evidence"][TOPICS.imu] = {
        "ok": False,
        "samples": 0,
        "graph_exists": False,
    }

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert (
        "data-flow observed topics missing for endpoint_adapter"
        in result.blockers
    )


def test_real_runtime_rejects_endpoint_graph_without_sensor_samples():
    report = _real_report()
    for topic in REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS:
        report["runtime_contract"]["topic_evidence"][topic].update(
            {
                "ok": False,
                "samples": 0,
                "graph_exists": True,
            }
        )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)
    payload = real_runtime_evidence_payload(
        result,
        REAL_RUNTIME_CONTRACT,
        report=report,
    )

    endpoint_stage = payload["checked_runtime_data_flow_evidence"][
        "endpoint_adapter"
    ]
    assert result.ok is False
    assert "endpoint input sample window missing for /nav/lidar_scan" in result.blockers
    assert "endpoint input sample window missing for /nav/imu" in result.blockers
    assert "data-flow observed topics missing for endpoint_adapter" in result.blockers
    assert endpoint_stage["ok"] is False
    assert "lidar_scan_sampled" in endpoint_stage["missing_signals"]
    assert "imu_sampled" in endpoint_stage["missing_signals"]


def test_real_runtime_rejects_wrong_endpoint_input_frame_id():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.lidar_scan][
        "frame_id"
    ] = "camera_link"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert any(
        blocker.startswith(
            "topic frame_id mismatch for /nav/lidar_scan: camera_link not in"
        )
        for blocker in result.blockers
    )


def test_real_runtime_rejects_missing_key_topic_frame_id():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.registered_cloud].pop("frame_id")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic frame_id missing for /nav/registered_cloud" in result.blockers


def test_real_runtime_rejects_missing_key_topic_frame_evidence():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"].pop(TOPICS.map_cloud)

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic frame evidence missing for /nav/map_cloud" in result.blockers


def test_real_runtime_accepts_map_framed_odometry():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.odometry]["frame_id"] = "map"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is True


def test_real_runtime_rejects_non_contract_odometry_frame():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.odometry][
        "frame_id"
    ] = "camera_link"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert any(
        blocker.startswith(
            "topic frame_id mismatch for /nav/odometry: camera_link not in"
        )
        for blocker in result.blockers
    )


def test_real_runtime_rejects_odom_framed_map_cloud():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.map_cloud]["frame_id"] = "odom"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic frame_id mismatch for /nav/map_cloud: odom not in map" in result.blockers


def test_real_runtime_rejects_missing_global_path_frame_id():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.global_path].pop("frame_id")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)
    payload = real_runtime_evidence_payload(
        result,
        REAL_RUNTIME_CONTRACT,
        report=report,
    )

    assert result.ok is False
    assert "topic frame_id missing for /nav/global_path" in result.blockers
    assert payload["checked_required_topic_frame_report"][TOPICS.global_path] == {
        "default_frame_id": "map",
        "observed_frame_id": None,
        "allowed_frame_ids": ["map"],
        "ok": False,
    }


def test_real_runtime_rejects_odom_framed_global_path():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.global_path]["frame_id"] = "odom"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic frame_id mismatch for /nav/global_path: odom not in map" in result.blockers


def test_real_runtime_rejects_wrong_local_path_frame_id():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.local_path][
        "frame_id"
    ] = "camera_link"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert any(
        blocker.startswith(
            "topic frame_id mismatch for /nav/local_path: camera_link not in"
        )
        for blocker in result.blockers
    )


def test_real_runtime_evidence_gate_script_accepts_valid_report(tmp_path: Path):
    report_path = tmp_path / "real_runtime_report.json"
    report_path.write_text(json.dumps(_real_report()), encoding="utf-8")
    script = REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_gate.py"

    proc = subprocess.run(
        [sys.executable, str(script), str(report_path)],
        cwd=str(REPO_ROOT),
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["schema_version"] == REAL_RUNTIME_EVIDENCE_VALIDATION_SCHEMA
    assert payload["ok"] is True
    assert payload["expected_contract"] == REAL_RUNTIME_CONTRACT
    assert payload["validation_gate"]["expected_runtime_contract"] == (
        REAL_RUNTIME_CONTRACT
    )
    assert payload["validation_gate"]["acceptance_step"] == 3
    assert payload["validation_gate"]["requires_prior_gates"] == ["runtime_audit"]
    assert payload["validation_gate"]["proves"] == list(REAL_RUNTIME_EVIDENCE_PROVES)
    assert payload["validation_gate"]["operator_summary_sections"] == list(
        REAL_RUNTIME_EVIDENCE_OPERATOR_SUMMARY_SECTIONS
    )
    assert payload["frame_links_required"] is True
    assert payload["data_flow_required"] is True
    assert payload["hardware_boundary_required"] is True
    assert payload["checked_collector_contract"] == {
        "name": REAL_RUNTIME_COLLECTOR_NAME,
        "expected_name": REAL_RUNTIME_COLLECTOR_NAME,
        "read_only": True,
        "control_topics_published": [],
        "expected_control_topics_published": [],
        "duration_sec": 20.0,
        "ok": True,
    }
    assert payload["checked_runtime_topics"] == list(
        runtime_data_flow_topics(REAL_RUNTIME_CONTRACT)
    )
    assert payload["checked_data_flow_stages"] == [
        stage.name for stage in resolved_runtime_data_flow(REAL_RUNTIME_CONTRACT)
    ]
    assert payload["checked_algorithm_interfaces"] == (
        runtime_algorithm_interface_contract()
    )
    assert payload["checked_runtime_data_flow_stage_algorithm_interfaces"] == (
        runtime_stage_algorithm_interface_contract()
    )
    command_flow = payload["checked_runtime_data_flow_evidence"]["command_boundary"]
    assert command_flow["ok"] is True
    assert command_flow["inputs"] == [TOPICS.cmd_vel]
    assert command_flow["outputs"] == [REAL_HARDWARE_COMMAND_SINK]
    assert command_flow["missing_outputs"] == []
    assert command_flow["reason"] == "command_boundary_observed"
    assert payload["checked_frame_links"] == list(FRAME_LINKS)
    assert payload["checked_frame_link_evidence"]["body_to_lidar"] == {
        "expected_parent": "body",
        "expected_child": "lidar_link",
        "observed_parent": "body",
        "observed_child": "lidar_link",
        "samples": None,
        "static": True,
        "published": False,
        "error": None,
        "ok": True,
    }
    assert payload["checked_frames"] == runtime_frames_contract()
    assert payload["checked_topic_default_frame_ids"] == (
        runtime_topic_default_frame_contract(REAL_RUNTIME_CONTRACT)
    )
    assert payload["checked_topic_allowed_frame_ids"] == (
        runtime_topic_allowed_frame_contract(REAL_RUNTIME_CONTRACT)
    )
    assert payload["checked_required_topic_frame_ids"] == list(
        REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS
    )
    assert payload["checked_required_topic_default_frame_ids"] == {
        TOPICS.lidar_scan: "lidar_link",
        TOPICS.imu: "lidar_link",
        TOPICS.odometry: "odom",
        TOPICS.registered_cloud: "body",
        TOPICS.map_cloud: "map",
        TOPICS.global_path: "map",
        TOPICS.local_path: "map",
        TOPICS.cmd_vel: "body",
    }
    assert payload["checked_required_topic_allowed_frame_ids"] == {
        TOPICS.lidar_scan: ["lidar_link"],
        TOPICS.imu: ["lidar_link"],
        TOPICS.odometry: ["odom", "map"],
        TOPICS.registered_cloud: ["body"],
        TOPICS.map_cloud: ["map"],
        TOPICS.global_path: ["map"],
        TOPICS.local_path: ["map", "odom", "body"],
        TOPICS.cmd_vel: ["body"],
    }
    assert payload["observed_required_topic_frame_ids"] == {
        TOPICS.lidar_scan: "lidar_link",
        TOPICS.imu: "lidar_link",
        TOPICS.odometry: "odom",
        TOPICS.registered_cloud: "body",
        TOPICS.map_cloud: "map",
        TOPICS.global_path: "map",
        TOPICS.local_path: "body",
        TOPICS.cmd_vel: "body",
    }
    assert payload["checked_required_topic_frame_report"][TOPICS.map_cloud] == {
        "default_frame_id": "map",
        "observed_frame_id": "map",
        "allowed_frame_ids": ["map"],
        "ok": True,
    }
    assert payload["checked_required_topic_frame_report"][TOPICS.local_path] == {
        "default_frame_id": "map",
        "observed_frame_id": "body",
        "allowed_frame_ids": ["map", "odom", "body"],
        "ok": True,
    }
    assert payload["checked_real_motion_evidence"] == {
        "real_robot_motion": True,
        "odom_delta_m": 0.2,
        "min_motion_m": 0.05,
        "odom_position_samples": 3,
        "ok": True,
    }
    assert payload["checked_hardware_boundary_evidence"] == {
        "command_topic": TOPICS.cmd_vel,
        "command_sink": REAL_HARDWARE_COMMAND_SINK,
        "expected_command_sink": REAL_HARDWARE_COMMAND_SINK,
        "command_observed": True,
        "cmd_vel_sent_to_hardware": True,
        "hardware_command_route_observed": True,
        "command_subscribers": ["/thunder_driver"],
        "expected_command_subscribers": [],
        "unexpected_simulation_sinks": [],
        "ok": True,
    }
    assert payload["checked_required_topic_sample_windows"][TOPICS.map_cloud] == {
        "samples": 5,
        "first_seen_sec": 0.14,
        "last_seen_sec": 1.1400000000000001,
        "sample_span_sec": 1.0,
        "duration_sec": 20.0,
        "ok": True,
    }
    assert payload["checked_endpoint_input_sample_windows"][TOPICS.lidar_scan] == {
        "samples": 4,
        "first_seen_sec": 0.1,
        "last_seen_sec": 19.5,
        "sample_span_sec": 19.4,
        "duration_sec": 20.0,
        "ok": True,
    }
    assert payload["checked_endpoint_input_sample_windows"][TOPICS.imu] == {
        "samples": 20,
        "first_seen_sec": 0.11,
        "last_seen_sec": 19.5,
        "sample_span_sec": 19.39,
        "duration_sec": 20.0,
        "ok": True,
    }
    assert payload["checked_localization_health_evidence"] == {
        "health_topic": TOPICS.localization_health,
        "health_state": "LOCKED",
        "healthy_states": ["LOCKED", "RECOVERED", "TRACKING"],
        "quality_topic": TOPICS.localization_quality,
        "quality": 0.0234,
        "quality_min_exclusive": 0.0,
        "quality_max_exclusive": 0.5,
        "health_window_ok": True,
        "quality_window_ok": True,
        "ok": True,
    }
    assert payload["checked_data_flow_temporal_order"][
        "local_planning_and_following"
    ]["ok"] is True
    assert payload["checked_data_flow_temporal_order"][
        "local_planning_and_following"
    ]["input_topics"] == [TOPICS.global_path]
    assert payload["checked_data_flow_temporal_order"][
        "local_planning_and_following"
    ]["output_topics"] == [TOPICS.local_path, TOPICS.cmd_vel]
    assert payload["checked_live_topic_freshness"][TOPICS.lidar_scan] == {
        "samples": 4,
        "last_seen_sec": 19.5,
        "duration_sec": 20.0,
        "max_age_sec": 2.0,
        "age_sec": 0.5,
        "ok": True,
    }
    assert payload["checked_live_topic_freshness"][TOPICS.cmd_vel]["ok"] is True


def test_real_runtime_evidence_gate_script_rejects_missing_hardware_boundary(
    tmp_path: Path,
):
    report = _real_report()
    report.pop("hardware_boundary")
    report_path = tmp_path / "real_runtime_missing_hardware_boundary.json"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    script = REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_gate.py"

    proc = subprocess.run(
        [sys.executable, str(script), str(report_path)],
        cwd=str(REPO_ROOT),
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 2
    payload = json.loads(proc.stdout)
    assert payload["ok"] is False
    assert "hardware_boundary missing" in payload["blockers"]
    assert payload["checked_hardware_boundary_evidence"] == {
        "command_topic": TOPICS.cmd_vel,
        "command_sink": None,
        "expected_command_sink": REAL_HARDWARE_COMMAND_SINK,
        "command_observed": True,
        "cmd_vel_sent_to_hardware": True,
        "hardware_command_route_observed": False,
        "command_subscribers": [],
        "expected_command_subscribers": [],
        "unexpected_simulation_sinks": [],
        "ok": False,
    }


def test_real_runtime_payload_exposes_failed_frame_link_evidence():
    report = _real_report()
    report["runtime_contract"]["frame_evidence"].pop("body_to_lidar")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)
    payload = real_runtime_evidence_payload(
        result,
        REAL_RUNTIME_CONTRACT,
        report=report,
    )

    assert result.ok is False
    assert "frame evidence missing or failed for body_to_lidar" in result.blockers
    assert payload["checked_frame_link_evidence"]["body_to_lidar"] == {
        "expected_parent": "body",
        "expected_child": "lidar_link",
        "observed_parent": None,
        "observed_child": None,
        "samples": None,
        "static": False,
        "published": False,
        "error": None,
        "ok": False,
    }


def test_real_runtime_evidence_requires_body_to_camera_frame_link():
    report = _real_report()
    report["runtime_contract"]["frame_evidence"].pop("body_to_camera", None)

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "frame evidence missing or failed for body_to_camera" in result.blockers


def test_real_runtime_evidence_gate_script_rejects_non_real_expected_contract(
    tmp_path: Path,
):
    report_path = tmp_path / "real_runtime_report.json"
    report_path.write_text(json.dumps(_real_report()), encoding="utf-8")
    script = REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_gate.py"

    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            str(report_path),
            "--expected-contract",
            "cmu_unity_external",
        ],
        cwd=str(REPO_ROOT),
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 2
    assert f"only supports expected contract {REAL_RUNTIME_CONTRACT}" in proc.stderr


def test_real_runtime_collector_builds_valid_report_from_read_only_observations():
    collector = _load_real_runtime_collect_module()
    topic_evidence = {
        "/nav/lidar_scan": {
            "ok": True,
            "samples": 4,
            "points": 12000,
            "frame_id": "lidar_link",
        },
        "/nav/imu": {"ok": True, "samples": 20, "frame_id": "lidar_link"},
        "/nav/odometry": {"ok": True, "samples": 6, "frame_id": "odom"},
        "/nav/registered_cloud": {
            "ok": True,
            "samples": 5,
            "points": 9000,
            "frame_id": "body",
        },
        "/nav/map_cloud": {
            "ok": True,
            "samples": 5,
            "points": 45000,
            "frame_id": "map",
        },
        "/nav/global_path": {
            "ok": True,
            "samples": 2,
            "nonempty_samples": 2,
            "max_poses": 12,
            "frame_id": "map",
        },
        "/nav/local_path": {
            "ok": True,
            "samples": 3,
            "nonempty_samples": 3,
            "max_poses": 9,
            "frame_id": "body",
        },
        "/nav/cmd_vel": {
            "ok": True,
            "samples": 5,
            "nonzero_samples": 4,
            "max_norm": 0.3,
            "frame_id": "body",
        },
    }
    _add_topic_sample_windows(topic_evidence, REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS)
    _add_topic_sample_windows(
        topic_evidence,
        REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
    )
    _add_valid_localization_evidence(topic_evidence)
    _mark_live_topic_fresh(
        topic_evidence,
        (
            TOPICS.lidar_scan,
            TOPICS.imu,
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.localization_health,
            TOPICS.localization_quality,
            TOPICS.local_path,
            TOPICS.cmd_vel,
        ),
    )

    report = collector.build_real_runtime_report(
        topic_evidence=topic_evidence,
        frame_samples={
            "map_to_odom": 4,
            "odom_to_body": 4,
            "body_to_lidar": 4,
            "body_to_camera": 4,
        },
        command_subscribers=["/thunder_driver"],
        duration_sec=20.0,
        odom_positions=[(0.0, 0.0, 0.0), (0.12, 0.0, 0.0)],
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is True
    assert result.blockers == ()
    assert report["collector"]["read_only"] is True
    assert report["collector"]["control_topics_published"] == list(
        REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED
    )
    assert report["validation_gate"]["expected_runtime_contract"] == (
        REAL_RUNTIME_CONTRACT
    )
    assert report["validation_gate"]["acceptance_step"] == 3
    assert report["validation_gate"]["requires_prior_gates"] == ["runtime_audit"]
    assert report["validation_gate"]["proves"] == list(REAL_RUNTIME_EVIDENCE_PROVES)
    assert report["validation_gate"]["operator_summary_sections"] == list(
        REAL_RUNTIME_EVIDENCE_OPERATOR_SUMMARY_SECTIONS
    )
    assert report["motion"] == {
        "odom_delta_m": 0.12,
        "min_motion_m": 0.05,
        "odom_position_samples": 2,
    }
    command_boundary = report["runtime_contract"]["data_flow_evidence"][
        "command_boundary"
    ]
    assert command_boundary["observed_inputs"] == [TOPICS.cmd_vel]
    assert command_boundary["observed_outputs"] == [REAL_HARDWARE_COMMAND_SINK]
    assert command_boundary["missing_outputs"] == []
    assert command_boundary["missing_signals"] == []
    assert command_boundary["observed_signals"] == [
        "cmd_vel_nonzero",
        "hardware_route_observed",
    ]
    payload = real_runtime_evidence_payload(
        result,
        REAL_RUNTIME_CONTRACT,
        report=report,
    )
    checked_command_boundary = payload["checked_runtime_data_flow_evidence"][
        "command_boundary"
    ]
    assert checked_command_boundary["observed_inputs"] == [TOPICS.cmd_vel]
    assert checked_command_boundary["observed_outputs"] == [REAL_HARDWARE_COMMAND_SINK]
    assert checked_command_boundary["missing_outputs"] == []
    assert checked_command_boundary["missing_signals"] == []
    assert checked_command_boundary["observed_signals"] == [
        "cmd_vel_nonzero",
        "hardware_route_observed",
    ]


def test_real_runtime_collector_reports_data_flow_stage_diagnostics():
    collector = _load_real_runtime_collect_module()
    report = collector.build_real_runtime_report(
        topic_evidence={
            "/nav/lidar_scan": {"ok": True, "samples": 1},
            "/nav/imu": {"ok": True, "samples": 1},
            "/nav/odometry": {"ok": True, "samples": 1},
            "/nav/registered_cloud": {"ok": True, "samples": 1, "points": 1},
            "/nav/map_cloud": {"ok": True, "samples": 1, "points": 1},
            "/nav/global_path": {
                "ok": True,
                "samples": 1,
                "nonempty_samples": 1,
                "max_poses": 2,
            },
            "/nav/local_path": {"ok": False, "samples": 0},
            "/nav/cmd_vel": {"ok": True, "samples": 1, "max_norm": 0.0},
        },
        frame_samples={
            "map_to_odom": 1,
            "odom_to_body": 1,
            "body_to_lidar": 1,
            "body_to_camera": 1,
        },
        command_subscribers=[],
        duration_sec=5.0,
        odom_positions=[(0.0, 0.0, 0.0), (0.1, 0.0, 0.0)],
    )

    data_flow = report["runtime_contract"]["data_flow_evidence"]
    local_stage = data_flow["local_planning_and_following"]
    command_stage = data_flow["command_boundary"]

    assert local_stage["ok"] is False
    assert TOPICS.local_path in local_stage["missing_outputs"]
    assert "local_path_nonempty" in local_stage["missing_signals"]
    assert "cmd_vel_nonzero" in local_stage["missing_signals"]
    assert command_stage["ok"] is False
    assert REAL_HARDWARE_COMMAND_SINK in command_stage["missing_outputs"]
    assert "hardware_route_observed" in command_stage["missing_signals"]


def test_real_runtime_rejects_local_planner_without_registered_cloud_input():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.registered_cloud].update(
        {
            "ok": False,
            "samples": 0,
            "points": 0,
            "frame_id": "body",
        }
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)
    payload = real_runtime_evidence_payload(
        result,
        REAL_RUNTIME_CONTRACT,
        report=report,
    )

    local_stage = payload["checked_runtime_data_flow_evidence"][
        "local_planning_and_following"
    ]
    assert result.ok is False
    assert (
        "data-flow observed topics missing for local_planning_and_following"
        in result.blockers
    )
    assert local_stage["ok"] is False
    assert "registered_cloud_nonempty" in local_stage["missing_signals"]


def test_real_runtime_collector_dependency_failure_preserves_contract_shape():
    collector = _load_real_runtime_collect_module()
    report = collector.build_unavailable_real_runtime_report(
        duration_sec=0.1,
        error="ROS 2 Python dependencies unavailable: No module named 'rclpy'",
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)
    topic_evidence = report["runtime_contract"]["topic_evidence"]
    frame_evidence = report["runtime_contract"]["frame_evidence"]
    data_flow = report["runtime_contract"]["data_flow_evidence"]

    assert result.ok is False
    assert "data-flow evidence missing" not in result.blockers
    assert "frame evidence missing" not in result.blockers
    assert set(topic_evidence) == set(runtime_data_flow_topics(REAL_RUNTIME_CONTRACT))
    assert set(frame_evidence) == set(FRAME_LINKS)
    assert set(data_flow) == {stage.name for stage in resolved_runtime_data_flow(REAL_RUNTIME_CONTRACT)}
    assert data_flow["endpoint_adapter"]["missing_outputs"] == [
        TOPICS.lidar_scan,
        TOPICS.imu,
    ]
    assert "lidar_scan_sampled" in data_flow["endpoint_adapter"][
        "missing_signals"
    ]
    assert REAL_HARDWARE_COMMAND_SINK in data_flow["command_boundary"][
        "missing_outputs"
    ]
    assert report["runtime_contract"]["collection_available"] is False
    assert "No module named 'rclpy'" in report["runtime_contract"]["collection_error"]
    assert report["validation_gate"]["acceptance_step"] == 3
    assert report["validation_gate"]["operator_summary_sections"] == list(
        REAL_RUNTIME_EVIDENCE_OPERATOR_SUMMARY_SECTIONS
    )


def test_real_runtime_gateway_collector_builds_valid_report(monkeypatch):
    collector = _load_real_runtime_collect_module()
    odom_x = {"value": 0.0}

    def _latest_summary(topic: str) -> dict[str, object]:
        if topic == TOPICS.lidar_scan:
            return {"frame_id": "lidar_link", "points": 12000}
        if topic == TOPICS.imu:
            return {"frame_id": "lidar_link"}
        if topic == TOPICS.odometry:
            return {"frame_id": "odom", "position": {"x": odom_x["value"], "y": 0.0, "z": 0.0}}
        if topic == TOPICS.registered_cloud:
            return {"frame_id": "body", "points": 9000}
        if topic == TOPICS.map_cloud:
            return {"frame_id": "map", "points": 45000}
        if topic == TOPICS.global_path:
            return {"frame_id": "map", "max_poses": 12}
        if topic == TOPICS.local_path:
            return {"frame_id": "body", "max_poses": 9}
        if topic == TOPICS.cmd_vel:
            return {"frame_id": "body", "cmd_norm": 0.2}
        try:
            return {
                "frame_id": runtime_topic_default_frame_id(
                    REAL_RUNTIME_CONTRACT,
                    topic,
                )
            }
        except ValueError:
            return {}

    def _default_frame(topic: str) -> str | None:
        try:
            return runtime_topic_default_frame_id(REAL_RUNTIME_CONTRACT, topic)
        except ValueError:
            return None

    def _dataflow() -> dict[str, object]:
        return {
            "runtime_boundary": {
                "ok": True,
                "runtime_contract": REAL_RUNTIME_CONTRACT,
                "command_sink": REAL_HARDWARE_COMMAND_SINK,
                "expected_command_sink": REAL_HARDWARE_COMMAND_SINK,
                "frame_links": {
                    name: {"parent": link.parent, "child": link.child}
                    for name, link in FRAME_LINKS.items()
                },
            },
            "control_boundary": {
                "command_interfaces": [
                    {"name": "driver_cmd_vel", "publishes": [TOPICS.cmd_vel]}
                ],
            },
            "topics": [
                {
                    "topic": topic,
                    "default_frame_id": _default_frame(topic),
                    "observability": {
                        "observable": True,
                        "has_fresh_module_sample": True,
                        "live_module_samples": True,
                        "module_port_candidates": [
                            {
                                "msg_count": 3,
                                "rate_hz": 10.0,
                                "stale_ms": 0.0,
                                "latest_summary": _latest_summary(topic),
                            }
                        ],
                    },
                    "inspection": {"observable": True, "live": True},
                }
                for topic in collector.OBSERVED_TOPICS
            ],
        }

    def _fake_fetch(base_url, path, *, timeout_sec, params=None):
        if path == "/api/v1/runtime/dataflow":
            return _dataflow()
        if path == "/api/v1/state":
            odom_x["value"] += 0.06
            return {
                "odometry": {
                    "x": odom_x["value"],
                    "y": 0.0,
                    "z": 0.0,
                    "frame_id": "odom",
                }
            }
        if path == "/api/v1/path":
            return {
                "frame_id": "map",
                "count": 3,
                "path": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
            }
        if path == "/api/v1/map/points":
            return {
                "frame_id": "map",
                "count": 2,
                "points": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            }
        if path == "/api/v1/localization/status":
            return {
                "state": "ready",
                "reported_state": "LOCKED",
                "icp_quality": 0.0234,
                "icp_fitness": 0.0234,
            }
        if path == "/api/v1/navigation/status":
            return {
                "control": {
                    "active_cmd_source": "path_follower",
                    "sources": {"path_follower": {"active": True}},
                }
            }
        raise AssertionError(path)

    monkeypatch.setattr(collector, "_fetch_gateway_json", _fake_fetch)

    report = collector.run_collect(
        Namespace(
            collector="gateway",
            gateway_url="http://robot:5050",
            gateway_timeout_sec=0.1,
            gateway_poll_sec=0.01,
            duration_sec=0.03,
            min_motion_m=0.05,
            min_cmd_vel_norm=0.01,
            expected_command_subscriber=[],
        )
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is True
    assert report["collector"]["source"] == "gateway"
    assert report["collector"]["gateway_url"] == "http://robot:5050"
    assert report["real_robot_motion"] is True
    assert report["cmd_vel_sent_to_hardware"] is True
    assert report["runtime_contract"]["topic_evidence"][TOPICS.local_path][
        "max_poses"
    ] >= 9


def test_real_runtime_ros2_collector_mode_keeps_legacy_dispatch(monkeypatch):
    collector = _load_real_runtime_collect_module()
    called = {}

    def _fake_ros2_collect(args):
        called["collector"] = args.collector
        return collector.build_unavailable_real_runtime_report(
            duration_sec=args.duration_sec,
            error="test ros2 collector branch",
            expected_command_subscribers=args.expected_command_subscriber,
        )

    monkeypatch.setattr(collector, "run_ros2_collect", _fake_ros2_collect)

    report = collector.run_collect(
        Namespace(
            collector="ros2",
            duration_sec=0.01,
            expected_command_subscriber=[],
        )
    )

    assert called["collector"] == "ros2"
    assert report["collector"]["source"] == "ros2"
    assert report["runtime_contract"]["collection_available"] is False


def test_real_runtime_collector_script_rejects_non_real_expected_contract():
    script = REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_collect.py"

    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "--expected-contract",
            "cmu_unity_external",
            "--no-validate",
            "--duration-sec",
            "0.01",
        ],
        cwd=str(REPO_ROOT),
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 2
    assert f"only supports expected contract {REAL_RUNTIME_CONTRACT}" in proc.stderr


def test_real_runtime_collector_does_not_infer_hardware_route_without_observed_sink():
    collector = _load_real_runtime_collect_module()
    topic_evidence = {
        "/nav/lidar_scan": {"ok": True, "samples": 1},
        "/nav/imu": {"ok": True, "samples": 1},
        "/nav/odometry": {"ok": True, "samples": 1},
        "/nav/registered_cloud": {"ok": True, "samples": 1, "points": 1},
        "/nav/map_cloud": {"ok": True, "samples": 1, "points": 1},
        "/nav/global_path": {
            "ok": True,
            "samples": 1,
            "nonempty_samples": 1,
            "max_poses": 2,
        },
        "/nav/local_path": {
            "ok": True,
            "samples": 1,
            "nonempty_samples": 1,
            "max_poses": 2,
        },
        "/nav/cmd_vel": {
            "ok": True,
            "samples": 1,
            "nonzero_samples": 1,
            "max_norm": 0.1,
        },
    }
    _add_topic_sample_windows(topic_evidence, REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS)
    _add_topic_sample_windows(
        topic_evidence,
        REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
    )
    _add_valid_localization_evidence(topic_evidence)
    report = collector.build_real_runtime_report(
        topic_evidence=topic_evidence,
        frame_samples={
            "map_to_odom": 1,
            "odom_to_body": 1,
            "body_to_lidar": 1,
            "body_to_camera": 1,
        },
        command_subscribers=[],
        duration_sec=5.0,
        odom_positions=[(0.0, 0.0, 0.0), (0.1, 0.0, 0.0)],
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert report["cmd_vel_sent_to_hardware"] is False
    assert result.ok is False
    assert "cmd_vel_sent_to_hardware is not true" in result.blockers
    assert "real hardware command boundary missing" in result.blockers


def test_real_runtime_collector_script_does_not_publish_control_topics():
    script = REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_collect.py"
    source = script.read_text(encoding="utf-8")

    assert ".create_publisher(" not in source
    assert ".publish(" not in source


def test_robot_ops_cli_exposes_read_only_real_runtime_evidence_command():
    script = REPO_ROOT / "scripts" / "lingtu"
    source = script.read_text(encoding="utf-8", errors="ignore")
    usage_start = source.index("cmd_evidence_usage()")
    start = source.index("cmd_evidence()")
    end = source.index("cmd_health()", start)
    evidence_section = source[usage_start:end]
    evidence_block = source[start:end]

    assert "lingtu evidence --duration 20 --json-out report.json" in source
    assert "evidence|runtime-evidence|real-runtime-evidence)" in source
    assert "/nav/localization_health" in evidence_section
    assert "/nav/localization_quality" in evidence_section
    assert '"$python_bin" "$repo_root/lingtu.py" real-runtime-evidence' in evidence_block
    assert "--duration-sec" in evidence_block
    assert "--min-motion-m" in evidence_block
    assert "--min-cmd-vel-norm" in evidence_block
    assert "--json-out" in evidence_block
    assert "--expected-command-subscriber" in evidence_block
    assert "--no-validate" in evidence_block
    assert "artifacts/thunder_field_runtime/report.json" in evidence_block
    assert ">&2" in evidence_block
    assert "curl " not in evidence_block
    assert "/api/v1/goal" not in evidence_block
    assert "cmd_nav" not in evidence_block


def test_robot_ops_cli_exposes_read_only_runtime_contract_commands():
    script = REPO_ROOT / "scripts" / "lingtu"
    source = script.read_text(encoding="utf-8", errors="ignore")
    start = source.index("cmd_runtime_contract_usage()")
    end = source.index("# -- Subcommand: evidence --", start)
    runtime_block = source[start:end]

    assert "lingtu runtime-spec nav --endpoint thunder_field" in source
    assert "lingtu runtime-contract --json" in source
    assert "lingtu runtime-audit --json" in source
    assert "contract|runtime-contract)" in source
    assert "spec|runtime-spec)" in source
    assert "audit|runtime-audit)" in source
    assert '"$python_bin" "$repo_root/lingtu.py" runtime-contract "$@"' in runtime_block
    assert '"$python_bin" "$repo_root/lingtu.py" runtime-spec "$@"' in runtime_block
    assert '"$python_bin" "$repo_root/lingtu.py" runtime-audit "$@"' in runtime_block
    assert "LINGTU_PYTHON:-python3" in source
    assert "--json-out PATH" in runtime_block
    assert "data-flow and frame contract" in runtime_block
    assert "curl " not in runtime_block
    assert "/api/v1/goal" not in runtime_block
    assert "cmd_nav" not in runtime_block


def test_real_runtime_collector_observed_topics_cover_resolved_real_data_flow():
    collector = _load_real_runtime_collect_module()
    observed = set(collector.OBSERVED_TOPICS)
    required_topics = set(runtime_data_flow_topics(REAL_RUNTIME_CONTRACT))

    assert required_topics <= observed
    assert collector.OBSERVED_TOPICS == runtime_data_flow_topics(REAL_RUNTIME_CONTRACT)


def test_real_runtime_contract_constant_lives_in_runtime_interface():
    assert REAL_RUNTIME_CONTRACT == INTERFACE_REAL_RUNTIME_CONTRACT
    assert REAL_RUNTIME_CONTRACT == THUNDER_FIELD_RUNTIME_CONTRACT
    assert REAL_RUNTIME_CONTRACT == "thunder_field"
    assert canonical_data_source_name(LEGACY_REAL_RUNTIME_CONTRACT) == (
        REAL_RUNTIME_CONTRACT
    )
    assert LEGACY_REAL_RUNTIME_CONTRACT == "real_s100p"


def test_runtime_contract_audit_real_collector_uses_runtime_contract_constant(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    calls: list[str] = []

    def _record_runtime_data_flow_topics(runtime_contract: str) -> tuple[str, ...]:
        calls.append(runtime_contract)
        return ()

    monkeypatch.setattr(
        audit,
        "runtime_data_flow_topics",
        _record_runtime_data_flow_topics,
    )

    result = audit._check_real_collector(runtime_contract_manifest())

    assert audit.REAL_RUNTIME_CONTRACT == REAL_RUNTIME_CONTRACT
    assert calls == [REAL_RUNTIME_CONTRACT]
    assert result["ok"] is True


def test_runtime_contract_audit_rejects_real_collector_frame_contract_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["real_runtime_topic_allowed_frame_ids"][TOPICS.map_cloud] = ("odom",)

    result = audit._check_real_collector(manifest)

    assert result["ok"] is False
    assert (
        "real collector required topic frame contract does not match runtime manifest"
        in result["blockers"]
    )
    assert (
        "real collector topic allowed frame contract does not match runtime manifest"
        in result["blockers"]
    )


def test_runtime_contract_audit_rejects_real_collector_runtime_frames_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["frames"]["axis_convention"] = "x_right_y_forward_z_up"

    result = audit._check_real_collector(manifest)

    assert result["ok"] is False
    assert (
        "real collector runtime frames contract does not match runtime manifest"
        in result["blockers"]
    )


def test_runtime_contract_audit_rejects_real_collector_default_frame_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["real_runtime_topic_default_frame_ids"][TOPICS.map_cloud] = "odom"

    result = audit._check_real_collector(manifest)

    assert result["ok"] is False
    assert (
        "real collector topic default frame contract does not match runtime manifest"
        in result["blockers"]
    )


def test_runtime_contract_audit_rejects_real_collector_algorithm_interface_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["algorithm_interfaces"]["fastlio_mapping"]["outputs"] = (
        TOPICS.odometry,
    )

    result = audit._check_real_collector(manifest)

    assert result["ok"] is False
    assert (
        "real collector algorithm interface contract does not match runtime manifest"
        in result["blockers"]
    )


def test_runtime_contract_audit_rejects_real_collector_stage_interface_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["runtime_data_flow_stage_algorithm_interfaces"]["global_planning"] = (
        "fastlio_mapping",
    )

    result = audit._check_real_collector(manifest)

    assert result["ok"] is False
    assert (
        "real collector runtime stage algorithm interface contract "
        "does not match runtime manifest"
    ) in result["blockers"]


def test_real_runtime_required_frame_topics_are_runtime_interface_contract():
    required_topics = runtime_required_topic_frame_ids(REAL_RUNTIME_CONTRACT)
    frame_rules = runtime_topic_allowed_frame_ids(REAL_RUNTIME_CONTRACT)

    assert required_topics == REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS
    assert set(REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS) <= set(required_topics)
    assert set(required_topics) <= set(runtime_data_flow_topics(REAL_RUNTIME_CONTRACT))
    assert set(required_topics) <= set(frame_rules)


def test_topic_default_frame_id_is_first_declared_runtime_frame():
    assert topic_default_frame_id(TOPICS.exploration_grid) == "map"
    assert topic_default_frame_id(TOPICS.local_path) == "map"
    assert topic_default_frame_id(TOPICS.odometry) == "odom"
    assert runtime_topic_default_frame_id(REAL_RUNTIME_CONTRACT, TOPICS.map_cloud) == "map"
    assert (
        runtime_topic_default_frame_id(REAL_RUNTIME_CONTRACT, TOPICS.lidar_scan)
        == "lidar_link"
    )
    assert runtime_topic_default_frame_ids(None)[TOPICS.odometry] == "odom"
    assert runtime_topic_default_frame_ids(None)[TOPICS.imu] == "lidar_link"
    assert runtime_topic_default_frame_ids(REAL_RUNTIME_CONTRACT)[TOPICS.global_path] == "map"


def test_runtime_contract_manifest_exports_topic_default_frame_ids():
    manifest = runtime_contract_manifest()

    assert set(manifest["topic_default_frame_ids"]) == set(
        manifest["topic_allowed_frame_ids"]
    )
    assert set(manifest["real_runtime_topic_default_frame_ids"]) == set(
        manifest["real_runtime_topic_allowed_frame_ids"]
    )
    assert manifest["topic_default_frame_ids"][TOPICS.map_cloud] == (
        topic_default_frame_id(TOPICS.map_cloud)
    )
    assert manifest["real_runtime_topic_default_frame_ids"][TOPICS.map_cloud] == (
        runtime_topic_default_frame_id(REAL_RUNTIME_CONTRACT, TOPICS.map_cloud)
    )
    assert manifest["real_runtime_required_endpoint_input_topics"] == (
        REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS
    )
    assert normalize_runtime_frames_contract(manifest["frames"]) == (
        runtime_frames_contract()
    )
    assert normalize_algorithm_interface_contract(manifest["algorithm_interfaces"]) == (
        runtime_algorithm_interface_contract()
    )
    assert {
        stage: list(interfaces)
        for stage, interfaces in manifest[
            "runtime_data_flow_stage_algorithm_interfaces"
        ].items()
    } == runtime_stage_algorithm_interface_contract()


def test_real_runtime_collector_frame_evidence_uses_canonical_frame_links():
    collector = _load_real_runtime_collect_module()
    evidence = collector.build_frame_evidence(
        {
            "map_to_odom": 1,
            "odom_to_body": 2,
            "body_to_lidar": 3,
            "body_to_camera": 4,
        }
    )

    assert set(evidence) == set(FRAME_LINKS)
    for name, link in FRAME_LINKS.items():
        assert evidence[name]["parent"] == link.parent
        assert evidence[name]["child"] == link.child
        assert evidence[name]["ok"] is True


def test_real_runtime_collector_report_embeds_required_topic_frame_contract():
    collector = _load_real_runtime_collect_module()

    contract = collector.build_required_topic_frame_contract()

    assert contract == _required_topic_frame_contract()
    assert contract[TOPICS.lidar_scan] == {
        "default_frame_id": "lidar_link",
        "allowed_frame_ids": ["lidar_link"],
    }
    assert contract[TOPICS.map_cloud] == {
        "default_frame_id": "map",
        "allowed_frame_ids": ["map"],
    }


def test_real_runtime_collector_report_embeds_full_frame_contracts():
    collector = _load_real_runtime_collect_module()

    report = collector.build_real_runtime_report(
        topic_evidence={},
        frame_samples={},
        command_subscribers=[],
        duration_sec=0.0,
    )
    runtime_contract = report["runtime_contract"]

    assert runtime_contract["frames"] == runtime_frames_contract()
    assert (
        runtime_contract["topic_default_frame_ids"]
        == runtime_topic_default_frame_contract(REAL_RUNTIME_CONTRACT)
    )
    assert (
        runtime_contract["topic_allowed_frame_ids"]
        == runtime_topic_allowed_frame_contract(REAL_RUNTIME_CONTRACT)
    )
    assert (
        runtime_contract["algorithm_interfaces"]
        == runtime_algorithm_interface_contract()
    )
    assert (
        runtime_contract["runtime_data_flow_stage_algorithm_interfaces"]
        == runtime_stage_algorithm_interface_contract()
    )


def test_runtime_contract_audit_accepts_current_contract():
    audit = _load_runtime_contract_audit_module()

    payload = audit.build_runtime_contract_audit()

    assert payload["schema_version"] == "lingtu.runtime_contract_audit.v1"
    assert payload["ok"] is True
    assert payload["blockers"] == []
    assert payload["checks"]["yaml_manifest"]["ok"] is True
    assert payload["checks"]["profile_runtime_specs"]["ok"] is True
    profile_specs = payload["checks"]["profile_runtime_specs"]
    assert len(profile_specs["checked_profile_binding_specs"]) == (
        len(audit.PROFILE_DATA_SOURCE_BINDINGS)
        + len(profile_specs["checked_external_profile_specs"])
    )
    assert "sim_mujoco_live:external_default" in profile_specs[
        "checked_profile_binding_specs"
    ]
    assert "sim_cmu_tare:external_record" in profile_specs[
        "checked_profile_binding_specs"
    ]
    assert len(profile_specs["checked_ros2_binding_specs"]) == (
        sum(
            audit._enforces_ros2_runtime_binding_audit(profile)
            for profile in audit.PROFILES
        )
        + sum(
            sum(
                audit._enforces_ros2_runtime_binding_audit(profile, endpoint)
                for profile in endpoint.supported_profiles
            )
            for endpoint in audit.RUNTIME_ENDPOINTS.values()
        )
    )
    assert "sim:default" in profile_specs["checked_ros2_binding_specs"]
    assert "thunder_field:nav" in profile_specs["checked_ros2_binding_specs"]
    assert "tare_explore:default" in profile_specs["checked_ros2_binding_specs"]
    assert "thunder_field:tare_explore" in profile_specs["checked_ros2_binding_specs"]
    assert "sim_cmu_tare:default" in profile_specs["checked_ros2_binding_specs"]
    assert "mujoco_live:map" in profile_specs["checked_ros2_binding_specs"]
    assert "replay:nav" in profile_specs["checked_ros2_binding_specs"]
    assert "cmu_unity:sim_cmu_tare" in profile_specs["checked_ros2_binding_specs"]
    assert "gazebo:explore" in profile_specs["checked_ros2_binding_specs"]
    assert "sim_gazebo:default" in profile_specs["skipped_ros2_binding_specs"]
    assert "sim_industrial:default" in profile_specs["skipped_ros2_binding_specs"]
    assert "sim_cmu_tare:default" not in profile_specs["skipped_ros2_binding_specs"]
    assert "mujoco_live:map" not in profile_specs["skipped_ros2_binding_specs"]
    assert "replay:nav" not in profile_specs["skipped_ros2_binding_specs"]
    assert "cmu_unity:sim_cmu_tare" not in profile_specs["skipped_ros2_binding_specs"]
    assert profile_specs["skipped_ros2_binding_reasons"]["sim_gazebo:default"] == (
        "outside_no_ros_acceptance_set"
    )
    assert profile_specs["skipped_ros2_binding_reasons"]["sim_industrial:default"] == (
        "outside_no_ros_acceptance_set"
    )
    assert all(
        profile_specs["skipped_ros2_binding_reasons"][spec]
        for spec in profile_specs["skipped_ros2_binding_specs"]
    )
    assert profile_specs["skipped_ros2_binding_violations"]["sim_gazebo:default"] == []
    assert (
        profile_specs["skipped_ros2_binding_violations"]["sim_industrial:default"]
        == []
    )
    assert payload["checks"]["real_runtime_collector"]["ok"] is True
    real_collector = payload["checks"]["real_runtime_collector"]
    assert real_collector["runtime_contract"] == REAL_RUNTIME_CONTRACT
    assert real_collector["required_runtime_topics"] == sorted(
        runtime_data_flow_topics(REAL_RUNTIME_CONTRACT)
    )
    assert real_collector["required_topic_frame_contract"] == (
        _required_topic_frame_contract()
    )
    assert real_collector["frames"] == runtime_frames_contract()
    assert real_collector["topic_default_frame_ids"] == (
        runtime_topic_default_frame_contract(REAL_RUNTIME_CONTRACT)
    )
    assert real_collector["topic_allowed_frame_ids"] == (
        runtime_topic_allowed_frame_contract(REAL_RUNTIME_CONTRACT)
    )
    assert real_collector["algorithm_interfaces"] == (
        runtime_algorithm_interface_contract()
    )
    assert real_collector["runtime_data_flow_stage_algorithm_interfaces"] == (
        runtime_stage_algorithm_interface_contract()
    )
    contract_integrity = payload["checks"]["runtime_contract_integrity"]
    assert contract_integrity["ok"] is True
    assert contract_integrity["checked_runtime_stages"] == [
        stage.name for stage in RUNTIME_DATA_FLOW
    ]
    assert contract_integrity["checked_real_required_topic_frame_ids"] == list(
        REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS
    )
    assert contract_integrity["checked_real_required_endpoint_input_topics"] == list(
        REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS
    )
    assert contract_integrity["checked_topic_default_frame_ids"][TOPICS.odometry] == "odom"
    assert (
        contract_integrity["checked_real_runtime_topic_default_frame_ids"][
            TOPICS.global_path
        ]
        == "map"
    )
    assert contract_integrity["checked_algorithm_interfaces"] == [
        "exploration_strategy",
        "fastlio_mapping",
        "fastlio_raw_validation",
        "global_planning",
        "local_planning_and_following",
        "octoplanner3d_global_planning",
        "pct_global_planning",
        "tare_exploration",
        "traversable_frontier_preview",
        "wavefront_frontier_exploration",
    ]
    assert contract_integrity["checked_runtime_stage_algorithm_interfaces"] == {
        stage: list(interfaces)
        for stage, interfaces in RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items()
    }
    assert payload["checks"]["runtime_validation_gates"]["ok"] is True
    assert payload["validation_gate"]["acceptance_step"] == 1
    assert payload["validation_gate"]["requires_prior_gates"] == []
    assert payload["validation_gate"]["proves"] == list(RUNTIME_AUDIT_PROVES)
    assert payload["validation_gate"]["operator_summary_sections"] == list(
        RUNTIME_AUDIT_OPERATOR_SUMMARY_SECTIONS
    )
    assert payload["checks"]["runtime_validation_gates"]["commands"][
        "real_runtime_evidence"
    ] == REAL_RUNTIME_EVIDENCE_COMMAND
    assert payload["checks"]["runtime_validation_gates"]["commands"][
        "real_runtime_evidence_collector"
    ] == REAL_RUNTIME_EVIDENCE_COLLECTOR_COMMAND
    assert f"--expected-contract {REAL_RUNTIME_CONTRACT}" in (
        payload["checks"]["runtime_validation_gates"]["commands"][
            "real_runtime_evidence_collector"
        ]
    )
    assert payload["checks"]["runtime_validation_gates"]["commands"][
        "real_runtime_evidence_gate"
    ] == REAL_RUNTIME_EVIDENCE_GATE_COMMAND
    assert f"--expected-contract {REAL_RUNTIME_CONTRACT}" in (
        payload["checks"]["runtime_validation_gates"]["commands"][
            "real_runtime_evidence_gate"
        ]
    )
    assert payload["checks"]["runtime_validation_gates"]["validates"][
        "real_runtime_evidence"
    ] == list(REAL_RUNTIME_EVIDENCE_VALIDATES)
    assert (
        "complete_runtime_topic_evidence_coverage"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "canonical_frames_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "topic_default_frame_contract_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "topic_allowed_frame_contract_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "required_topic_frame_contract_declared"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "required_topic_default_allowed_and_observed_frame_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "required_topic_sample_window_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "data_flow_temporal_order_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "real_motion_delta_and_sample_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "localization_health_locked_or_recovered"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "localization_quality_sampled"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "localization_quality_healthy_range"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "collector_read_only_and_no_control_topics_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "raw_lidar_imu_endpoint_input_samples"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "live_topic_freshness_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "hardware_boundary_evidence_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "local_planner_input_output_chain_observed"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "frame_link_evidence_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "runtime_data_flow_stage_evidence_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "data_flow_observed_missing_token_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "runtime_stage_algorithm_interface_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert (
        "algorithm_interface_payload"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "real_runtime_evidence"
        ]
    )
    assert payload["checks"]["runtime_validation_gates"]["commands"][
        "saved_map_artifact_gate"
    ] == SAVED_MAP_ARTIFACT_GATE_COMMAND
    assert (
        "artifact_gate_payload_declares_checked_artifacts_frames_sources"
        in payload["checks"]["runtime_validation_gates"]["validates"][
            "saved_map_artifact_gate"
        ]
    )
    assert payload["checks"]["runtime_validation_gates"]["validates"][
        "runtime_audit"
    ] == list(RUNTIME_AUDIT_VALIDATES)
    assert list(payload["checks"]) == list(RUNTIME_AUDIT_CHECKS)
    assert payload["checks"]["runtime_validation_gates"]["checks"][
        "runtime_audit"
    ] == list(RUNTIME_AUDIT_CHECKS)
    assert payload["checks"]["runtime_validation_gates"]["coverage"][
        "runtime_audit"
    ] == {
        name: list(checks)
        for name, checks in RUNTIME_AUDIT_VALIDATE_CHECK_COVERAGE.items()
    }
    acceptance = payload["checks"]["runtime_validation_gates"]["acceptance"]
    assert acceptance["runtime_audit"] == {
        "acceptance_step": 1,
        "required_when": "before_any_runtime_contract_or_field_readiness_claim",
        "requires_prior_gates": [],
        "conditional_prior_gates": [],
        "proves": list(RUNTIME_AUDIT_PROVES),
        "operator_summary_sections": list(RUNTIME_AUDIT_OPERATOR_SUMMARY_SECTIONS),
    }
    assert acceptance["saved_map_artifact_gate"] == {
        "acceptance_step": 2,
        "required_when": "saved_map_tomogram_occupancy_or_pct_artifact_is_used",
        "requires_prior_gates": ["runtime_audit"],
        "conditional_prior_gates": [],
        "proves": list(SAVED_MAP_ARTIFACT_GATE_PROVES),
        "operator_summary_sections": list(
            SAVED_MAP_ARTIFACT_GATE_OPERATOR_SUMMARY_SECTIONS
        ),
    }
    assert acceptance["real_runtime_evidence"] == {
        "acceptance_step": 3,
        "required_when": "before_claiming_thunder_field_runtime_or_field_navigation",
        "requires_prior_gates": ["runtime_audit"],
        "conditional_prior_gates": [
            "saved_map_artifact_gate when saved map, tomogram, occupancy, or PCT artifact is used"
        ],
        "proves": list(REAL_RUNTIME_EVIDENCE_PROVES),
        "operator_summary_sections": list(
            REAL_RUNTIME_EVIDENCE_OPERATOR_SUMMARY_SECTIONS
        ),
    }
    assert payload["checks"]["runtime_validation_gates"]["coverage"][
        "runtime_audit"
    ]["data_source_resolved_flow_inputs_outputs"] == [
        "runtime_contract_integrity"
    ]
    assert payload["checks"]["runtime_validation_gates"]["coverage"][
        "runtime_audit"
    ]["runtime_stage_algorithm_interface_binding"] == [
        "runtime_contract_integrity"
    ]
    assert payload["checks"]["runtime_validation_gates"]["coverage"][
        "runtime_audit"
    ]["source_code_uses_canonical_runtime_topics"] == [
        "source_topic_contracts"
    ]
    assert (
        "runtime_stage_owner_frame_map_dependency_integrity"
        in payload["checks"]["runtime_validation_gates"]["validates"]["runtime_audit"]
    )
    assert (
        "runtime_stage_algorithm_interface_binding"
        in payload["checks"]["runtime_validation_gates"]["validates"]["runtime_audit"]
    )
    assert (
        "topic_default_frame_integrity"
        in payload["checks"]["runtime_validation_gates"]["validates"]["runtime_audit"]
    )
    assert (
        "ros_frame_contract_doc_mirror"
        in payload["checks"]["runtime_validation_gates"]["validates"]["runtime_audit"]
    )
    assert (
        "runtime_validation_gates_self_description"
        in payload["checks"]["runtime_validation_gates"]["validates"]["runtime_audit"]
    )
    doc_check = payload["checks"]["ros_frame_contract_doc"]
    assert doc_check["ok"] is True
    assert doc_check["checked_doc"] == "docs/architecture/ros_frame_contract.md"
    assert doc_check["checked_topics"] == list(
        runtime_topic_allowed_frame_ids(REAL_RUNTIME_CONTRACT)
    )
    assert doc_check["required_topics"] == sorted(
        REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS
    )
    assert "src/gateway/services/runtime_status.py" in payload["checks"][
        "source_frame_contracts"
    ]["checked_files"]
    assert "src/gateway/routes/diagnostics.py" in payload["checks"][
        "source_topic_contracts"
    ]["checked_files"]


def test_runtime_contract_audit_rejects_ros_frame_contract_doc_drift(tmp_path: Path):
    audit = _load_runtime_contract_audit_module()
    doc = (REPO_ROOT / "docs" / "architecture" / "ros_frame_contract.md").read_text(
        encoding="utf-8"
    )
    bad_doc = tmp_path / "ros_frame_contract.md"
    bad_doc.write_text(
        doc.replace(
            "| `/slam/map_cloud` | `map` | `map`, `odom` | yes | `map` |",
            "| `/slam/map_cloud` | `odom` | `map`, `odom` | yes | `odom` |",
        ),
        encoding="utf-8",
    )

    result = audit._check_ros_frame_contract_doc(
        runtime_contract_manifest(),
        bad_doc,
    )

    assert result["ok"] is False
    assert (
        "ros_frame_contract.md topic frame row drifted for /slam/map_cloud"
        in result["blockers"]
    )


def test_runtime_contract_audit_rejects_runtime_check_list_drift(monkeypatch):
    audit = _load_runtime_contract_audit_module()
    monkeypatch.setattr(
        audit,
        "RUNTIME_AUDIT_CHECKS",
        tuple(
            item
            for item in RUNTIME_AUDIT_CHECKS
            if item != "source_topic_contracts"
        ),
    )

    payload = audit.build_runtime_contract_audit()

    assert payload["ok"] is False
    assert any(
        "runtime_audit checks list drifted" in blocker
        for blocker in payload["blockers"]
    )


def test_runtime_contract_audit_rejects_yaml_frame_mismatch(tmp_path: Path):
    audit = _load_runtime_contract_audit_module()
    contract = yaml.safe_load(
        (REPO_ROOT / "config" / "topic_contract.yaml").read_text(encoding="utf-8")
    )
    contract["tf"]["body_frame"] = "base_link"
    bad_contract = tmp_path / "topic_contract_bad_frame.yaml"
    bad_contract.write_text(yaml.safe_dump(contract), encoding="utf-8")

    payload = audit.build_runtime_contract_audit(bad_contract)

    assert payload["ok"] is False
    assert any(
        "yaml_manifest: tf.body_frame does not mirror runtime frames" == blocker
        for blocker in payload["blockers"]
    )


def test_runtime_contract_audit_rejects_yaml_topic_format_mismatch(tmp_path: Path):
    audit = _load_runtime_contract_audit_module()
    contract = yaml.safe_load(
        (REPO_ROOT / "config" / "topic_contract.yaml").read_text(encoding="utf-8")
    )
    contract["topic_formats"][TOPICS.localization_quality] = ["std_msgs/msg/String"]
    bad_contract = tmp_path / "topic_contract_bad_topic_format.yaml"
    bad_contract.write_text(yaml.safe_dump(contract), encoding="utf-8")

    payload = audit.build_runtime_contract_audit(bad_contract)

    assert payload["ok"] is False
    assert any(
        "yaml_manifest: topic_formats does not mirror runtime manifest" == blocker
        for blocker in payload["blockers"]
    )


def test_runtime_contract_audit_rejects_yaml_topic_ros_type_mismatch(tmp_path: Path):
    audit = _load_runtime_contract_audit_module()
    contract = yaml.safe_load(
        (REPO_ROOT / "config" / "topic_contract.yaml").read_text(encoding="utf-8")
    )
    contract["topic_ros_types"][TOPICS.cmd_vel] = ["geometry_msgs/msg/Twist"]
    bad_contract = tmp_path / "topic_contract_bad_topic_ros_types.yaml"
    bad_contract.write_text(yaml.safe_dump(contract), encoding="utf-8")

    payload = audit.build_runtime_contract_audit(bad_contract)

    assert payload["ok"] is False
    assert any(
        "yaml_manifest: topic_ros_types does not mirror runtime manifest" == blocker
        for blocker in payload["blockers"]
    )


def test_runtime_contract_audit_rejects_yaml_stage_algorithm_interface_mismatch(
    tmp_path: Path,
):
    audit = _load_runtime_contract_audit_module()
    contract = yaml.safe_load(
        (REPO_ROOT / "config" / "topic_contract.yaml").read_text(encoding="utf-8")
    )
    contract["runtime_data_flow_stage_algorithm_interfaces"][
        "global_planning"
    ] = ["global_planning"]
    bad_contract = tmp_path / "topic_contract_bad_stage_algorithm_interfaces.yaml"
    bad_contract.write_text(yaml.safe_dump(contract), encoding="utf-8")

    payload = audit.build_runtime_contract_audit(bad_contract)

    assert payload["ok"] is False
    assert any(
        "yaml_manifest: runtime_data_flow_stage_algorithm_interfaces "
        "does not mirror runtime manifest"
        == blocker
        for blocker in payload["blockers"]
    )


def test_runtime_contract_audit_rejects_yaml_adapter_alias_mismatch(tmp_path: Path):
    audit = _load_runtime_contract_audit_module()
    contract = yaml.safe_load(
        (REPO_ROOT / "config" / "topic_contract.yaml").read_text(encoding="utf-8")
    )
    contract["adapter_aliases"]["localizer"][-1]["target"] = TOPICS.relocalize_service
    bad_contract = tmp_path / "topic_contract_bad_adapter_alias.yaml"
    bad_contract.write_text(yaml.safe_dump(contract), encoding="utf-8")

    payload = audit.build_runtime_contract_audit(bad_contract)

    assert payload["ok"] is False
    assert any(
        "yaml_manifest: adapter_aliases does not mirror runtime manifest" == blocker
        for blocker in payload["blockers"]
    )


def test_runtime_contract_audit_rejects_yaml_topic_default_frame_mismatch(
    tmp_path: Path,
):
    audit = _load_runtime_contract_audit_module()
    contract = yaml.safe_load(
        (REPO_ROOT / "config" / "topic_contract.yaml").read_text(encoding="utf-8")
    )
    contract.setdefault("topic_default_frame_ids", {})[TOPICS.map_cloud] = "odom"
    bad_contract = tmp_path / "topic_contract_bad_topic_default_frame.yaml"
    bad_contract.write_text(yaml.safe_dump(contract), encoding="utf-8")

    payload = audit.build_runtime_contract_audit(bad_contract)

    assert payload["ok"] is False
    assert any(
        "yaml_manifest: topic_default_frame_ids does not mirror runtime manifest"
        == blocker
        for blocker in payload["blockers"]
    )


def test_runtime_contract_integrity_rejects_unresolved_resolved_flow_placeholder():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["resolved_runtime_data_flow"][REAL_RUNTIME_CONTRACT][-1]["outputs"] = [
        "sink:data_source.command_sink"
    ]

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        f"resolved {REAL_RUNTIME_CONTRACT} command_boundary contains unresolved placeholder"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_command_sink_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["resolved_runtime_data_flow"][REAL_RUNTIME_CONTRACT][-1]["outputs"] = [
        "wrong_sink"
    ]

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        f"resolved {REAL_RUNTIME_CONTRACT} command_boundary does not match data source sink"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_undeclared_artifact_reference():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    global_planning = manifest["resolved_runtime_data_flow"][REAL_RUNTIME_CONTRACT][3]
    global_planning["inputs"] = tuple(global_planning["inputs"]) + (
        "artifact:missing_tomogram",
    )

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        (
            f"resolved {REAL_RUNTIME_CONTRACT} global_planning "
            "references undeclared artifact missing_tomogram"
        )
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_undeclared_topic_format_reference():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["topic_formats"][TOPICS.cmd_vel] = ("missing_twist_format",)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "topic format /nav/cmd_vel references undeclared format missing_twist_format"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_message_format_name_mismatch():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["message_formats"]["map_cloud"]["name"] = "wrong_map_cloud"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "message format map_cloud name field mismatch" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_artifact_format_name_mismatch():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["artifact_formats"]["tomogram"]["name"] = "wrong_tomogram"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "artifact format tomogram name field mismatch" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_non_numeric_lidar_extrinsic():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["lidar_extrinsics"]["real_mid360"]["x"] = "not-a-number"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "lidar extrinsic real_mid360 x is not numeric" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_profile_binding_name_mismatch():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["profile_data_sources"]["nav"]["profile"] = "wrong_nav"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "profile binding nav profile field mismatch" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_profile_binding_unknown_data_source():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["profile_data_sources"]["nav"]["data_source"] = "missing_source"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "profile binding nav references unknown data source missing_source"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_profile_binding_data_source_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["profile_data_sources"]["nav"]["data_source"] = "in_process_stub"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "profile binding nav data_source drifted from runtime binding"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_profile_binding_blank_mode():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["profile_data_sources"]["nav"]["mode"] = ""

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "profile binding nav mode missing" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_profile_binding_mode_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["profile_data_sources"]["nav"]["mode"] = "framework_test"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "profile binding nav mode drifted from runtime binding" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_algorithm_interface_topic_without_format():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    fastlio = manifest["algorithm_interfaces"]["fastlio_mapping"]
    fastlio["inputs"] = tuple(fastlio["inputs"]) + ("/nav/unformatted_scan",)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "algorithm interface fastlio_mapping references topic without format /nav/unformatted_scan"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_algorithm_interface_without_flow_stage():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    local_planning = manifest["algorithm_interfaces"]["local_planning_and_following"]
    local_planning["inputs"] = tuple(local_planning["inputs"]) + (TOPICS.map_cloud,)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "algorithm interface local_planning_and_following is not covered by "
        "any runtime_data_flow stage"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_algorithm_interface_without_stage_binding():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["runtime_data_flow_stage_algorithm_interfaces"][
        "local_planning_and_following"
    ] = ()

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "runtime_data_flow_stage_algorithm_interfaces "
        "local_planning_and_following must list interfaces"
        == blocker
        for blocker in result["blockers"]
    )
    assert any(
        "algorithm interfaces missing runtime data-flow stage binding: "
        "local_planning_and_following"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_stage_interface_binding_mismatch():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["runtime_data_flow_stage_algorithm_interfaces"][
        "local_planning_and_following"
    ] = ("pct_global_planning",)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "runtime_data_flow stage local_planning_and_following does not cover "
        "algorithm interface pct_global_planning"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_resolved_slam_input_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["resolved_runtime_data_flow"][REAL_RUNTIME_CONTRACT][1]["inputs"] = [
        TOPICS.raw_lidar_points,
    ]

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        f"resolved {REAL_RUNTIME_CONTRACT} slam inputs drifted" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_resolved_map_layer_input_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["resolved_runtime_data_flow"][REAL_RUNTIME_CONTRACT][2]["inputs"] = [
        TOPICS.odometry,
        TOPICS.map_cloud,
    ]

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        f"resolved {REAL_RUNTIME_CONTRACT} map layer inputs drifted" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_runtime_data_flow_topics_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["runtime_data_flow_topics"][REAL_RUNTIME_CONTRACT] = tuple(
        topic
        for topic in manifest["runtime_data_flow_topics"][REAL_RUNTIME_CONTRACT]
        if topic != TOPICS.cmd_vel
    )

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        (
            f"runtime_data_flow_topics {REAL_RUNTIME_CONTRACT} "
            "does not match resolved data flow"
        )
        == blocker
        for blocker in result["blockers"]
    )
    assert any(
        (
            "real_runtime_required_topic_frame_ids missing from "
            f"{REAL_RUNTIME_CONTRACT} data flow: /nav/cmd_vel"
        )
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_resolved_stage_metadata_drift():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    global_planning = manifest["resolved_runtime_data_flow"][REAL_RUNTIME_CONTRACT][3]
    global_planning["frame_role"] = "body"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        f"resolved {REAL_RUNTIME_CONTRACT} global_planning frame_role drifted"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_blank_stage_metadata():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["runtime_data_flow"][3]["map_dependency"] = ""

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "runtime_data_flow global_planning map_dependency missing" == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_required_real_frame_topic_without_rule():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["real_runtime_topic_allowed_frame_ids"].pop(TOPICS.cmd_vel)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "real_runtime_required_topic_frame_ids missing real frame rules: /nav/cmd_vel"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_required_real_frame_topic_without_format():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["topic_formats"].pop(TOPICS.cmd_vel)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "real_runtime_required_topic_frame_ids missing topic formats: /nav/cmd_vel"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_endpoint_input_without_frame_evidence():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["real_runtime_required_topic_frame_ids"] = tuple(
        topic
        for topic in manifest["real_runtime_required_topic_frame_ids"]
        if topic != TOPICS.lidar_scan
    )

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "real_runtime_required_endpoint_input_topics missing required frame "
        "evidence: /nav/lidar_scan"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_allowed_frame_topic_without_format():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["topic_allowed_frame_ids"]["/nav/unformatted_frame_topic"] = ("map",)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "topic_allowed_frame_ids topic /nav/unformatted_frame_topic has no topic format"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_unknown_allowed_frame():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["topic_allowed_frame_ids"][TOPICS.map_cloud] = ("world",)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "topic_allowed_frame_ids topic /nav/map_cloud allows unknown frame world"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_default_frame_not_allowed():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest.setdefault("topic_default_frame_ids", {})[TOPICS.cmd_vel] = "map"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "topic_default_frame_ids topic /nav/cmd_vel default frame map is not allowed"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_real_default_frame_not_allowed():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest.setdefault("real_runtime_topic_default_frame_ids", {})[
        TOPICS.global_path
    ] = "odom"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "real_runtime_topic_default_frame_ids topic /nav/global_path "
        "default frame odom is not allowed"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_accepts_declared_frame_alias_allowed_frame():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["topic_allowed_frame_ids"][TOPICS.cmd_vel] = ("base_link",)
    manifest["real_runtime_topic_allowed_frame_ids"][TOPICS.cmd_vel] = ("base_link",)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is True


def test_runtime_contract_integrity_rejects_real_frame_not_in_general_contract():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["real_runtime_topic_allowed_frame_ids"][TOPICS.map_cloud] = (
        "map",
        "lidar_link",
    )

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "real_runtime_topic_allowed_frame_ids topic /nav/map_cloud "
        "allows frames outside topic_allowed_frame_ids: lidar_link"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_unknown_lidar_extrinsic_child():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["lidar_extrinsics"]["real_mid360"]["child"] = "unknown_lidar_frame"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "lidar extrinsic real_mid360 child unknown_lidar_frame is not canonical lidar frame or alias"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_adapter_alias_source_without_format():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["adapter_aliases"]["fastlio2"][0]["source"] = "/legacy/missing_scan"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "adapter alias fastlio2 source /legacy/missing_scan has no topic format"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_adapter_alias_undeclared_format():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["adapter_aliases"]["fastlio2"][0]["msg_format"] = "missing_format"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "adapter alias fastlio2 references undeclared format missing_format"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_adapter_relay_target_without_format():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["adapter_relays"]["cmu_unity"][0]["target"] = "/missing/relay"

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert any(
        "adapter relay cmu_unity target /missing/relay has no topic format"
        == blocker
        for blocker in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_missing_topic_formats_section():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest.pop("topic_formats")

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert "topic_formats section missing" in result["blockers"]


def test_runtime_contract_integrity_rejects_missing_topic_ros_types_section():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest.pop("topic_ros_types")

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert "topic_ros_types section missing" in result["blockers"]


def test_runtime_contract_integrity_rejects_unresolved_topic_ros_type():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest["topic_ros_types"][TOPICS.cmd_vel] = ("cmd_vel",)

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert (
        "topic_ros_types /nav/cmd_vel references non-ROS payload type cmd_vel"
        in result["blockers"]
    )


def test_runtime_contract_integrity_rejects_missing_lidar_extrinsics_section():
    audit = _load_runtime_contract_audit_module()
    manifest = deepcopy(runtime_contract_manifest())
    manifest.pop("lidar_extrinsics")

    result = audit._check_runtime_contract_integrity(manifest)

    assert result["ok"] is False
    assert "lidar_extrinsics section missing" in result["blockers"]


def test_profile_runtime_specs_reject_endpoint_simulation_only_provider_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["thunder_field"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "thunder_field",
        replace(endpoint, simulation_only=True),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint thunder_field simulation_only does not match data source provider"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_endpoint_action_for_unsupported_profile(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(
            endpoint,
            default_actions={**endpoint.default_actions, "nav": ("gate",)},
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live default_actions references unsupported profile nav"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_product_endpoint_matrix_drift(monkeypatch):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["cmu_unity"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "cmu_unity",
        replace(endpoint, supported_profiles=(*endpoint.supported_profiles, "nav")),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "product profile nav has undeclared endpoints: cmu_unity"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_product_planner_backend_field(monkeypatch):
    audit = _load_runtime_contract_audit_module()
    monkeypatch.setitem(
        audit.PROFILES,
        "nav",
        {**audit.PROFILES["nav"], "planner_backend": "octoplanner3d"},
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "product profile nav must use planner, not planner_backend"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_product_endpoint_planner_backend(monkeypatch):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["thunder_field"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "thunder_field",
        replace(
            endpoint,
            config_overrides={
                **endpoint.config_overrides,
                "planner_backend": "octoplanner3d",
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint thunder_field config_overrides must not set planner_backend "
        "for product profiles"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_missing_endpoint_default_action_profile(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    default_actions = dict(endpoint.default_actions)
    default_actions.pop("explore")
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(endpoint, default_actions=default_actions),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live default_actions missing supported profile explore"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_missing_endpoint_record_action_profile(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    record_actions = dict(endpoint.record_actions)
    record_actions.pop("explore")
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(endpoint, record_actions=record_actions),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live record_actions missing supported profile explore"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_external_profile_missing_launcher(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    profile_data = dict(audit.PROFILES["sim_mujoco_live"])
    profile_data["_external_launcher"] = "sim/scripts/missing_launcher.sh"
    monkeypatch.setitem(audit.PROFILES, "sim_mujoco_live", profile_data)

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "profile sim_mujoco_live external launcher missing: "
        "sim/scripts/missing_launcher.sh"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_external_profile_launcher_endpoint_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    profile_data = dict(audit.PROFILES["sim_mujoco_live"])
    profile_data["_external_launcher"] = (
        "sim/scripts/launch_cmu_unity_lingtu_runtime.sh"
    )
    monkeypatch.setitem(audit.PROFILES, "sim_mujoco_live", profile_data)

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "profile sim_mujoco_live external launcher does not match "
        "endpoint mujoco_live"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_external_profile_data_source_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    profile_data = dict(audit.PROFILES["sim_mujoco_live"])
    profile_data["_endpoint_data_source"] = "thunder_field"
    monkeypatch.setitem(audit.PROFILES, "sim_mujoco_live", profile_data)

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "profile sim_mujoco_live external default runtime spec: "
        "runtime contract does not match data source"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_external_profile_binding_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    binding = audit.PROFILE_DATA_SOURCE_BINDINGS["sim_mujoco_live"]
    monkeypatch.setitem(
        audit.PROFILE_DATA_SOURCE_BINDINGS,
        "sim_mujoco_live",
        replace(binding, data_source="thunder_field"),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "profile sim_mujoco_live external default runtime spec data_source "
        "drifted from PROFILE_DATA_SOURCE_BINDINGS"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_external_profile_default_args_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    profile_data = dict(audit.PROFILES["sim_mujoco_live"])
    profile_data["_external_default_args"] = ("legacy-gate",)
    monkeypatch.setitem(audit.PROFILES, "sim_mujoco_live", profile_data)

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "profile sim_mujoco_live external default args do not match "
        "endpoint mujoco_live"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_external_profile_record_args_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    profile_data = dict(audit.PROFILES["sim_mujoco_live"])
    profile_data["_external_record_args"] = ("legacy-video",)
    monkeypatch.setitem(audit.PROFILES, "sim_mujoco_live", profile_data)

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "profile sim_mujoco_live external record args do not match "
        "endpoint mujoco_live"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_missing_endpoint_launcher(monkeypatch):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(endpoint, external_launcher="sim/scripts/missing_launcher.sh"),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live external launcher missing: sim/scripts/missing_launcher.sh"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_hardware_endpoint_launcher(monkeypatch):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["thunder_field"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "thunder_field",
        replace(
            endpoint,
            external_launcher="sim/scripts/fastlio2_rosbag_replay_gate.py",
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint thunder_field hardware endpoint must not declare external launcher"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_ros2_runtime_binding_drift(monkeypatch):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["thunder_field"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "thunder_field",
        replace(
            endpoint,
            config_overrides={
                **endpoint.config_overrides,
                "localization_adapter": "ros2_slam_bridge",
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint thunder_field profile nav runtime bindings: "
        "localization_adapter=ros2_slam_bridge selects a ROS2 localization adapter"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_endpoint_config_frame_override_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(
            endpoint,
            config_overrides={
                **endpoint.config_overrides,
                "planning_frame_id": "camera_link",
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live config_overrides planning_frame_id camera_link "
        "is not a fixed runtime frame"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_endpoint_profile_frame_override_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    tare_overrides = dict(endpoint.profile_overrides["tare_explore"])
    tare_overrides["goal_frame_id"] = "body"
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(
            endpoint,
            profile_overrides={
                **endpoint.profile_overrides,
                "tare_explore": tare_overrides,
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live profile_overrides.tare_explore goal_frame_id body "
        "is not a fixed runtime frame"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_endpoint_config_topic_override_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(
            endpoint,
            config_overrides={
                **endpoint.config_overrides,
                "cloud_topic": "/nav/missing_cloud",
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live config_overrides cloud_topic "
        "/nav/missing_cloud has no topic format"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_endpoint_config_relative_topic_override(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(
            endpoint,
            config_overrides={
                **endpoint.config_overrides,
                "cloud_topic": "nav/missing_cloud",
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live config_overrides cloud_topic "
        "nav/missing_cloud is not an absolute runtime topic"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_endpoint_profile_topic_override_drift(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    tare_overrides = dict(endpoint.profile_overrides["tare_explore"])
    tare_overrides["cloud_topic"] = "/nav/missing_cloud"
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(
            endpoint,
            profile_overrides={
                **endpoint.profile_overrides,
                "tare_explore": tare_overrides,
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live profile_overrides.tare_explore cloud_topic "
        "/nav/missing_cloud has no topic format"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_missing_endpoint_config_tomogram(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["gazebo"]
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "gazebo",
        replace(
            endpoint,
            config_overrides={
                **endpoint.config_overrides,
                "tomogram": "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/missing.pickle",
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint gazebo config_overrides tomogram path missing: "
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/missing.pickle"
        in result["blockers"]
    )


def test_profile_runtime_specs_reject_missing_endpoint_profile_tomogram(
    monkeypatch,
):
    audit = _load_runtime_contract_audit_module()
    endpoint = audit.RUNTIME_ENDPOINTS["mujoco_live"]
    explore_overrides = dict(endpoint.profile_overrides["explore"])
    explore_overrides["tomogram"] = (
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/missing.pickle"
    )
    monkeypatch.setitem(
        audit.RUNTIME_ENDPOINTS,
        "mujoco_live",
        replace(
            endpoint,
            profile_overrides={
                **endpoint.profile_overrides,
                "explore": explore_overrides,
            },
        ),
    )

    result = audit._check_profile_runtime_specs()

    assert result["ok"] is False
    assert (
        "endpoint mujoco_live profile_overrides.explore tomogram path missing: "
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/missing.pickle"
        in result["blockers"]
    )


def test_runtime_validation_gate_check_rejects_command_drift():
    current_gates = runtime_validation_gates()

    gates = deepcopy(current_gates)
    gates["real_runtime_evidence"]["command"] = "python scripts/legacy_gate.py"

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert any(
        "real_runtime_evidence validation gate command drifted" == blocker
        for blocker in result.blockers
    )


def test_runtime_validation_gate_check_rejects_collector_command_contract_drift():
    gates = runtime_validation_gates()
    gates["real_runtime_evidence"]["collector_command"] = (
        "python scripts/gates/real_runtime_evidence_collect.py --duration-sec 20"
    )

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "real_runtime_evidence validation gate collector command drifted"
        in result.blockers
    )


def test_runtime_validation_gate_check_rejects_gate_command_contract_drift():
    gates = runtime_validation_gates()
    gates["real_runtime_evidence"]["gate_command"] = (
        "python scripts/gates/real_runtime_evidence_gate.py "
        "artifacts/thunder_field_runtime/report.json"
    )

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "real_runtime_evidence validation gate gate command drifted"
        in result.blockers
    )


def test_runtime_validation_gate_check_rejects_real_runtime_evidence_validates_drift():
    gates = runtime_validation_gates()
    gates["real_runtime_evidence"]["validates"] = [
        item
        for item in REAL_RUNTIME_EVIDENCE_VALIDATES
        if item != "runtime_data_flow_stage_evidence_payload"
    ]

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "real_runtime_evidence validation gate validates list drifted"
        in result.blockers
    )
    assert (
        result.validates["real_runtime_evidence"]
        == gates["real_runtime_evidence"]["validates"]
    )


def test_runtime_validation_gate_check_rejects_acceptance_metadata_drift():
    gates = runtime_validation_gates()
    gates["real_runtime_evidence"]["operator_summary_sections"] = [
        "Blockers",
        "Checked runtime topics",
    ]

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "real_runtime_evidence validation gate acceptance metadata drifted"
        in result.blockers
    )
    assert result.acceptance["real_runtime_evidence"][
        "operator_summary_sections"
    ] == [
        "Blockers",
        "Checked runtime topics",
    ]


def test_real_runtime_evidence_operator_sections_include_stage_matrix():
    gates = runtime_validation_gates()

    sections = gates["real_runtime_evidence"]["operator_summary_sections"]

    assert "Stage evidence matrix" in sections
    assert sections.index("Stage evidence matrix") < sections.index(
        "Data-flow evidence"
    )


def test_runtime_validation_gate_check_rejects_runtime_audit_validates_drift():
    gates = runtime_validation_gates()
    gates["runtime_audit"]["validates"] = [
        item for item in RUNTIME_AUDIT_VALIDATES
        if item != "runtime_algorithm_interfaces_cover_data_flow"
    ]

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "runtime_audit validation gate validates list drifted"
        in result.blockers
    )
    assert result.validates["runtime_audit"] == gates["runtime_audit"]["validates"]


def test_runtime_validation_gate_check_rejects_runtime_audit_checks_drift():
    gates = runtime_validation_gates()
    gates["runtime_audit"]["checks"] = [
        item for item in RUNTIME_AUDIT_CHECKS if item != "source_topic_contracts"
    ]

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "runtime_audit validation gate checks list drifted"
        in result.blockers
    )
    assert result.checks["runtime_audit"] == gates["runtime_audit"]["checks"]


def test_runtime_validation_gate_check_rejects_runtime_audit_coverage_drift():
    gates = runtime_validation_gates()
    gates["runtime_audit"]["coverage"]["data_source_resolved_flow_inputs_outputs"] = [
        "source_topic_contracts"
    ]

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "runtime_audit validation gate coverage map drifted"
        in result.blockers
    )
    assert result.coverage["runtime_audit"] == gates["runtime_audit"]["coverage"]


def test_runtime_validation_gate_check_rejects_uncovered_runtime_audit_check():
    gates = runtime_validation_gates()
    for covered_checks in gates["runtime_audit"]["coverage"].values():
        while "source_topic_contracts" in covered_checks:
            covered_checks.remove("source_topic_contracts")

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "runtime_audit checks have no validation coverage: source_topic_contracts"
        in result.blockers
    )


def test_runtime_validation_gate_check_rejects_missing_saved_map_artifact_gate():
    gates = runtime_validation_gates()
    gates.pop("saved_map_artifact_gate")

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert "saved_map_artifact_gate validation gate missing" in result.blockers


def test_runtime_validation_gate_check_rejects_saved_map_artifact_command_drift():
    gates = runtime_validation_gates()
    gates["saved_map_artifact_gate"]["command"] = "python scripts/old_map_gate.py"

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "saved_map_artifact_gate validation gate command drifted"
        in result.blockers
    )


def test_runtime_validation_gate_check_rejects_saved_map_artifact_validates_drift():
    gates = runtime_validation_gates()
    gates["saved_map_artifact_gate"]["validates"] = [
        item
        for item in SAVED_MAP_ARTIFACT_GATE_VALIDATES
        if item != "artifact_gate_payload_declares_checked_artifacts_frames_sources"
    ]

    result = validate_runtime_validation_gates(gates)

    assert result.ok is False
    assert (
        "saved_map_artifact_gate validation gate validates list drifted"
        in result.blockers
    )


def test_runtime_contract_audit_script_accepts_current_contract():
    script = REPO_ROOT / "scripts" / "gates" / "runtime_contract_audit.py"

    proc = subprocess.run(
        [sys.executable, str(script), "--json"],
        cwd=str(REPO_ROOT),
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["ok"] is True
    assert payload["checks"]["real_runtime_collector"]["ok"] is True
    assert payload["checks"]["runtime_validation_gates"]["ok"] is True
