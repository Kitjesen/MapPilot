# The subprocess tests execute fixed scripts from this repository.
# ruff: noqa: S603

from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import importlib.util
import subprocess
import sys
from argparse import Namespace
from pathlib import Path

from diagnostics.field.evidence import (
    REAL_HARDWARE_COMMAND_SINK,
    REAL_RUNTIME_COLLECTOR_NAME,
    REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED,
    validate_real_runtime_evidence,
    validate_runtime_evidence,
)
from runtime.runtime_interface import (
    FRAME_LINKS,
    REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
    REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS,
    TOPICS,
    adapter_source_for_target,
    body_frame_id,
    camera_frame_id,
    lidar_frame_id,
    map_frame_id,
    odom_frame_id,
    real_lidar_frame_id,
    resolved_runtime_data_flow,
    runtime_contract_data_source,
    runtime_data_flow_topics,
    runtime_topic_default_frame_id,
    simulator_world_frame_id,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_RUNTIME_DATA_SOURCE = runtime_contract_data_source(REAL_RUNTIME_CONTRACT)


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
        "body_to_gnss": {
            "ok": True,
            "parent": "body",
            "child": "gnss_antenna",
            "static": True,
        },
    }


def _data_flow_evidence(
    data_source: str = "mujoco_module_graph",
) -> dict:
    return {stage.name: {"ok": True} for stage in resolved_runtime_data_flow(data_source)}


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


def test_fastlio2_adapter_exposes_native_odometry_source() -> None:
    assert adapter_source_for_target("fastlio2", TOPICS.odometry) == "/Odometry"


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
            "name": "mujoco_module_graph",
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
    topic_evidence.update(
        {
            TOPICS.lidar_scan: {
                "ok": True,
                "samples": 4,
                "points": 12000,
                "frame_id": "lidar_link",
            },
            TOPICS.imu: {
                "ok": True,
                "samples": 20,
                "frame_id": "imu_link",
            },
            TOPICS.odometry: {"ok": True, "samples": 6, "frame_id": "odom"},
            TOPICS.registered_cloud: {
                "ok": True,
                "samples": 5,
                "points": 9000,
                "frame_id": "body",
            },
            TOPICS.map_cloud: {
                "ok": True,
                "samples": 5,
                "points": 45000,
                "frame_id": "map",
            },
            TOPICS.global_path: {
                "ok": True,
                "samples": 2,
                "max_poses": 12,
                "frame_id": "map",
            },
            TOPICS.local_path: {
                "ok": True,
                "samples": 3,
                "max_poses": 9,
                "frame_id": "body",
            },
            TOPICS.cmd_vel: {
                "ok": True,
                "samples": 5,
                "nonzero_samples": 5,
                "frame_id": "body",
            },
        }
    )
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
        "command_subscribers": ["driver.cmd_vel"],
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
            "frame_evidence": _frame_evidence(),
            "data_flow_evidence": _data_flow_evidence(REAL_RUNTIME_CONTRACT),
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


@pytest.mark.sim
def test_safe_simulation_report_passes():
    result = validate_runtime_evidence(_safe_report(), "mujoco_module_graph")

    assert result.ok is True
    assert result.blockers == ()


def test_declared_data_flow_evidence_can_be_required():
    result = validate_runtime_evidence(
        _safe_report(),
        "mujoco_module_graph",
        require_data_flow=True,
    )

    assert result.ok is True
    assert result.blockers == ()


def test_missing_data_flow_stage_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["data_flow_evidence"].pop("command_boundary")

    result = validate_runtime_evidence(
        report,
        "mujoco_module_graph",
        require_data_flow=True,
    )

    assert result.ok is False
    assert "data-flow evidence missing or failed for command_boundary" in result.blockers
    assert result.blockers.count("data-flow evidence missing or failed for command_boundary") == 1


def test_unrequired_data_flow_stage_can_be_reported_as_not_run():
    report = _safe_report()
    stage = report["runtime_contract"]["data_flow_evidence"]["global_planning"]
    stage["required"] = False
    stage["ok"] = False
    stage["reason"] = "not_required_for_basic_slam_gate"

    result = validate_runtime_evidence(
        report,
        "mujoco_module_graph",
        require_data_flow=True,
    )

    assert result.ok is True
    assert result.blockers == ()


def test_unknown_data_source_reports_explicit_data_flow_blocker():
    result = validate_runtime_evidence(
        _safe_report(),
        "unknown_runtime",
        require_data_flow=True,
    )

    assert result.ok is False
    assert any(blocker.startswith("resolved data-flow contract unavailable:") for blocker in result.blockers)


def test_topic_frame_id_mismatch_fails_when_evidence_reports_frame():
    report = _safe_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.registered_cloud] = {
        "ok": True,
        "samples": 1,
        "frame_id": "map",
    }

    result = validate_runtime_evidence(report, "mujoco_module_graph")

    assert result.ok is False
    assert any(
        blocker.startswith("topic frame_id mismatch for /slam/registered_cloud: map not in")
        for blocker in result.blockers
    )


def test_body_alias_is_accepted_for_body_frame_topic_evidence():
    report = _safe_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.registered_cloud] = {
        "ok": True,
        "samples": 1,
        "frame_id": "base_link",
    }

    result = validate_runtime_evidence(report, "mujoco_module_graph")

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

        result = validate_runtime_evidence(report, "mujoco_module_graph")

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

    result = validate_runtime_evidence(report, "mujoco_module_graph")

    assert result.ok is True
    assert result.blockers == ()


def test_declared_frame_evidence_can_be_required():
    result = validate_runtime_evidence(
        _safe_report(),
        "mujoco_module_graph",
        require_frame_links=True,
    )

    assert result.ok is True
    assert result.blockers == ()


def test_missing_frame_evidence_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["frame_evidence"].pop("body_to_lidar")

    result = validate_runtime_evidence(
        report,
        "mujoco_module_graph",
        require_frame_links=True,
    )

    assert result.ok is False
    assert "frame evidence missing or failed for body_to_lidar" in result.blockers


def test_wrong_frame_child_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["frame_evidence"]["body_to_lidar"]["child"] = "map"

    result = validate_runtime_evidence(
        report,
        "mujoco_module_graph",
        require_frame_links=True,
    )

    assert result.ok is False
    assert "frame evidence child mismatch for body_to_lidar" in result.blockers


def test_unobserved_frame_link_fails_when_required():
    report = _safe_report()
    report["runtime_contract"]["frame_evidence"]["odom_to_body"].pop("samples")

    result = validate_runtime_evidence(
        report,
        "mujoco_module_graph",
        require_frame_links=True,
    )

    assert result.ok is False
    assert "frame evidence missing or failed for odom_to_body" in result.blockers


@pytest.mark.sim
def test_hardware_command_in_simulation_fails():
    report = _safe_report()
    report["cmd_vel_sent_to_hardware"] = True

    result = validate_runtime_evidence(report, "mujoco_module_graph")

    assert result.ok is False
    assert "cmd_vel_sent_to_hardware is not false" in result.blockers


def test_missing_paths_can_be_allowed():
    report = _safe_report()
    report.pop("paths")
    report["runtime_contract"]["topic_evidence"].pop("/nav/global_path")
    report["runtime_contract"]["topic_evidence"].pop("/nav/local_path")

    result = validate_runtime_evidence(
        report,
        "mujoco_module_graph",
        require_paths=False,
    )

    assert result.ok is True


def test_missing_nav_command_can_be_allowed():
    report = _safe_report()
    report.pop("cmd_vel")
    report["runtime_contract"]["topic_evidence"].pop("/nav/cmd_vel")

    result = validate_runtime_evidence(
        report,
        "mujoco_module_graph",
        require_command=False,
    )

    assert result.ok is True


def test_real_runtime_report_passes_with_hardware_boundary_frame_and_data_flow():
    result = validate_real_runtime_evidence(_real_report(), REAL_RUNTIME_CONTRACT)

    assert result.ok is True
    assert result.blockers == ()


def test_real_runtime_validator_rejects_non_real_expected_contract():
    result = validate_real_runtime_evidence(_real_report(), "mujoco_module_graph")

    assert result.ok is False
    assert f"real runtime evidence expected_contract is not {REAL_RUNTIME_CONTRACT}" in result.blockers


def test_real_runtime_rejects_collector_that_publishes_control_topics():
    report = _real_report()
    report["collector"]["read_only"] = False
    report["collector"]["control_topics_published"] = [TOPICS.cmd_vel]

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "real runtime collector is not read-only" in result.blockers
    assert "real runtime collector published control topics" in result.blockers


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
    assert f"topic sample window missing for {TOPICS.map_cloud}" in result.blockers


def test_real_runtime_rejects_required_topic_sample_outside_collection_window():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.global_path]["last_seen_sec"] = 30.0

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic sample window outside collection duration for /nav/global_path" in result.blockers


def test_real_runtime_rejects_stale_local_planner_output_before_global_path():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.local_path].update(
        {
            "first_seen_sec": 0.01,
            "last_seen_sec": 0.05,
            "sample_span_sec": 0.04,
        }
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "data-flow temporal order failed for local_planning_and_following" in result.blockers


def test_real_runtime_rejects_stale_live_lidar_sample_at_collection_end():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.lidar_scan].update(
        {
            "last_seen_sec": 1.0,
            "sample_span_sec": 0.9,
        }
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert f"live topic stale for {TOPICS.lidar_scan}" in result.blockers


def test_real_runtime_rejects_missing_localization_health_topic():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"].pop(TOPICS.localization_health)

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "localization health evidence missing" in result.blockers


def test_real_runtime_rejects_lost_localization_health():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.localization_health]["data"] = "LOST|fitness=0.45"

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
    report["runtime_contract"]["topic_evidence"][TOPICS.localization_health]["data"] = "LOCKED|fitness=0.75"
    report["runtime_contract"]["topic_evidence"][TOPICS.localization_quality]["value"] = 0.75

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "localization quality outside healthy range: 0.75" in result.blockers


def test_real_runtime_accepts_confidence_localization_quality():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.localization_health].update(
        {
            "data": "TRACKING|quality=1.0",
            "quality_kind": "confidence",
        }
    )
    report["runtime_contract"]["topic_evidence"][TOPICS.localization_quality].update(
        {
            "value": 1.0,
            "quality_kind": "confidence",
        }
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert "localization quality outside healthy range: 1.0" not in result.blockers


def test_real_runtime_rejects_missing_hardware_command_boundary():
    report = _real_report()
    report["hardware_boundary"]["hardware_command_route_observed"] = False
    report["hardware_boundary"]["command_subscribers"] = []

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "real hardware command boundary missing" in result.blockers


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
    assert "data-flow observed topics missing for endpoint_adapter" in result.blockers


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

    assert result.ok is False
    assert f"endpoint input sample window missing for {TOPICS.lidar_scan}" in result.blockers
    assert f"endpoint input sample window missing for {TOPICS.imu}" in result.blockers
    assert "data-flow observed topics missing for endpoint_adapter" in result.blockers


def test_real_runtime_rejects_wrong_endpoint_input_frame_id():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.lidar_scan]["frame_id"] = "camera_link"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert any(
        blocker.startswith(f"topic frame_id mismatch for {TOPICS.lidar_scan}: camera_link not in")
        for blocker in result.blockers
    )


def test_real_runtime_rejects_missing_key_topic_frame_id():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.registered_cloud].pop("frame_id")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert f"topic frame_id missing for {TOPICS.registered_cloud}" in result.blockers


def test_real_runtime_rejects_missing_key_topic_frame_evidence():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"].pop(TOPICS.map_cloud)

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert f"topic frame evidence missing for {TOPICS.map_cloud}" in result.blockers


def test_real_runtime_accepts_map_framed_odometry():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.odometry]["frame_id"] = "map"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is True


def test_real_runtime_rejects_non_contract_odometry_frame():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.odometry]["frame_id"] = "camera_link"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert any(
        blocker.startswith(f"topic frame_id mismatch for {TOPICS.odometry}: camera_link not in")
        for blocker in result.blockers
    )


def test_real_runtime_rejects_odom_framed_map_cloud():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.map_cloud]["frame_id"] = "odom"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert f"topic frame_id mismatch for {TOPICS.map_cloud}: odom not in map" in result.blockers


def test_real_runtime_rejects_missing_global_path_frame_id():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.global_path].pop("frame_id")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic frame_id missing for /nav/global_path" in result.blockers


def test_real_runtime_rejects_odom_framed_global_path():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.global_path]["frame_id"] = "odom"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "topic frame_id mismatch for /nav/global_path: odom not in map" in result.blockers


def test_real_runtime_rejects_wrong_local_path_frame_id():
    report = _real_report()
    report["runtime_contract"]["topic_evidence"][TOPICS.local_path]["frame_id"] = "camera_link"

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert any(
        blocker.startswith("topic frame_id mismatch for /nav/local_path: camera_link not in")
        for blocker in result.blockers
    )


def test_real_runtime_validator_rejects_failed_frame_link_evidence():
    report = _real_report()
    report["runtime_contract"]["frame_evidence"].pop("body_to_lidar")

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)
    assert result.ok is False
    assert "frame evidence missing or failed for body_to_lidar" in result.blockers


def test_real_runtime_evidence_requires_body_to_camera_frame_link():
    report = _real_report()
    report["runtime_contract"]["frame_evidence"].pop("body_to_camera", None)

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is False
    assert "frame evidence missing or failed for body_to_camera" in result.blockers


def test_real_runtime_collector_builds_valid_report_from_read_only_observations():
    collector = _load_real_runtime_collect_module()
    topic_evidence = {
        TOPICS.lidar_scan: {
            "ok": True,
            "samples": 4,
            "points": 12000,
            "frame_id": "lidar_link",
        },
        TOPICS.imu: {"ok": True, "samples": 20, "frame_id": "imu_link"},
        TOPICS.odometry: {"ok": True, "samples": 6, "frame_id": "odom"},
        TOPICS.registered_cloud: {
            "ok": True,
            "samples": 5,
            "points": 9000,
            "frame_id": "body",
        },
        TOPICS.map_cloud: {
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
            "body_to_gnss": 4,
        },
        command_subscribers=["driver.cmd_vel"],
        duration_sec=20.0,
        odom_positions=[(0.0, 0.0, 0.0), (0.12, 0.0, 0.0)],
    )

    result = validate_real_runtime_evidence(report, REAL_RUNTIME_CONTRACT)

    assert result.ok is True
    assert result.blockers == ()
    assert report["collector"]["read_only"] is True
    assert report["collector"]["control_topics_published"] == list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED)
    assert report["motion"] == {
        "odom_delta_m": 0.12,
        "min_motion_m": 0.05,
        "odom_position_samples": 2,
    }
    command_boundary = report["runtime_contract"]["data_flow_evidence"]["command_boundary"]
    assert command_boundary["observed_inputs"] == [TOPICS.cmd_vel]
    assert command_boundary["observed_outputs"] == [REAL_HARDWARE_COMMAND_SINK]
    assert command_boundary["missing_outputs"] == []
    assert command_boundary["missing_signals"] == []
    assert command_boundary["observed_signals"] == [
        "cmd_vel_nonzero",
        "hardware_route_observed",
    ]


def test_real_runtime_collector_reports_data_flow_stage_diagnostics():
    collector = _load_real_runtime_collect_module()
    report = collector.build_real_runtime_report(
        topic_evidence={
            TOPICS.lidar_scan: {"ok": True, "samples": 1},
            TOPICS.imu: {"ok": True, "samples": 1},
            TOPICS.odometry: {"ok": True, "samples": 1},
            TOPICS.registered_cloud: {"ok": True, "samples": 1, "points": 1},
            TOPICS.map_cloud: {"ok": True, "samples": 1, "points": 1},
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

    assert result.ok is False
    assert "data-flow observed topics missing for local_planning_and_following" in result.blockers


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
    assert "lidar_scan_sampled" in data_flow["endpoint_adapter"]["missing_signals"]
    assert REAL_HARDWARE_COMMAND_SINK in data_flow["command_boundary"]["missing_outputs"]
    assert report["runtime_contract"]["collection_available"] is False
    assert "No module named 'rclpy'" in report["runtime_contract"]["collection_error"]


def test_real_runtime_gateway_collector_builds_valid_report(monkeypatch):
    collector = _load_real_runtime_collect_module()
    odom_x = {"value": 0.0}

    def _latest_summary(topic: str) -> dict[str, object]:
        if topic == TOPICS.lidar_scan:
            return {"frame_id": "lidar_link", "points": 12000}
        if topic == TOPICS.imu:
            return {"frame_id": "imu_link"}
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
                    name: {"parent": link.parent, "child": link.child} for name, link in FRAME_LINKS.items()
                },
            },
            "control_boundary": {
                "command_interfaces": [{"name": "driver_cmd_vel", "publishes": [TOPICS.cmd_vel]}],
            },
            "module_ports": {
                "driver": {
                    "ports_in": {
                        "cmd_vel": {
                            "direction": "in",
                            "type": "Twist",
                            "msg_count": 3,
                            "rate_hz": 10.0,
                            "stale_ms": 0.0,
                        }
                    }
                }
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
    assert report["runtime_contract"]["topic_evidence"][TOPICS.local_path]["max_poses"] >= 9


def test_real_runtime_gateway_collector_does_not_treat_static_control_as_hardware_sink():
    collector = _load_real_runtime_collect_module()

    subscribers = collector._gateway_command_subscribers(
        {
            "runtime_boundary": {
                "command_sink": REAL_HARDWARE_COMMAND_SINK,
                "expected_command_sink": REAL_HARDWARE_COMMAND_SINK,
            },
            "control_boundary": {
                "command_interfaces": [
                    {"name": "direct_cmd_vel", "publishes": [TOPICS.cmd_vel]},
                ],
            },
            "topics": [
                {
                    "topic": TOPICS.cmd_vel,
                    "observability": {
                        "observable": True,
                        "module_port_candidates": [
                            {
                                "module": "nav.out",
                                "port": "cmd_vel",
                                "direction": "in",
                                "msg_count": 3,
                                "rate_hz": 10.0,
                                "stale_ms": 0.0,
                            }
                        ],
                    },
                }
            ],
        }
    )

    assert subscribers == []


def test_real_runtime_gateway_collector_accepts_live_hardware_cmd_consumer():
    collector = _load_real_runtime_collect_module()

    subscribers = collector._gateway_command_subscribers(
        {
            "topics": [
                {
                    "topic": TOPICS.cmd_vel,
                    "observability": {
                        "module_port_candidates": [
                            {
                                "module": "driver",
                                "port": "cmd_vel",
                                "direction": "in",
                                "msg_count": 3,
                                "rate_hz": 10.0,
                                "stale_ms": 0.0,
                            }
                        ],
                    },
                }
            ],
        }
    )

    assert subscribers == ["driver.cmd_vel"]


def test_real_runtime_gateway_collector_infers_raw_inputs_from_localization():
    collector = _load_real_runtime_collect_module()
    topic_evidence = {topic: {"ok": False, "samples": 0, "graph_exists": False} for topic in collector.OBSERVED_TOPICS}
    odom_positions: list[tuple[float, float, float]] = []

    collector._record_gateway_rest_payloads(
        topic_evidence,
        odom_positions,
        {
            "localization": {
                "reported_state": "TRACKING",
                "lidar_input_hz": 10.0,
                "imu_input_hz": 200.0,
                "confidence": 1.0,
            }
        },
        sample_time_sec=1.0,
        min_cmd_vel_norm=0.01,
    )

    assert topic_evidence[TOPICS.lidar_scan]["ok"] is True
    assert topic_evidence[TOPICS.lidar_scan]["rate_hz"] == 10.0
    assert topic_evidence[TOPICS.imu]["ok"] is True
    assert topic_evidence[TOPICS.imu]["rate_hz"] == 200.0
    assert topic_evidence[TOPICS.localization_quality]["quality_kind"] == "confidence"


def test_real_runtime_collector_script_rejects_non_real_expected_contract():
    script = REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_collect.py"

    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "--expected-contract",
            "mujoco_module_graph",
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
        TOPICS.lidar_scan: {"ok": True, "samples": 1, "frame_id": "lidar_link"},
        TOPICS.imu: {"ok": True, "samples": 1, "frame_id": "imu_link"},
        TOPICS.odometry: {"ok": True, "samples": 1, "frame_id": "odom"},
        TOPICS.registered_cloud: {
            "ok": True,
            "samples": 1,
            "points": 1,
            "frame_id": "body",
        },
        TOPICS.map_cloud: {"ok": True, "samples": 1, "points": 1, "frame_id": "map"},
        TOPICS.global_path: {
            "ok": True,
            "samples": 1,
            "nonempty_samples": 1,
            "max_poses": 2,
            "frame_id": "map",
        },
        TOPICS.local_path: {
            "ok": True,
            "samples": 1,
            "nonempty_samples": 1,
            "max_poses": 2,
            "frame_id": "body",
        },
        TOPICS.cmd_vel: {
            "ok": True,
            "samples": 1,
            "nonzero_samples": 1,
            "max_norm": 0.1,
            "frame_id": "body",
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
