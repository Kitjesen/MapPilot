"""Simulation readiness contract tests."""

from __future__ import annotations

import copy
import json
import os
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

from lingtu.sim.readiness import (
    EXPLORE_STATUS_SCHEMA,
    HOST_READY_SCHEMA,
    MAPD_STATUS_SCHEMA,
    NAV_STATUS_SCHEMA,
    SLAM_STATUS_SCHEMA,
    TRAVERSABILITY_STATUS_SCHEMA,
    ReadinessExpectation,
    SimReadinessError,
    SimReadinessPending,
    load_typed_readiness,
    readiness_expectation_for_process,
    validate_feeder_readiness,
)

PRODUCT_SESSION_ID = "2" * 32
STARTED_WALL_NS = 1_700_000_000_000_000_000


def _canonical(payload: dict[str, object]) -> bytes:
    return json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")


def _write_fresh(path: Path, payload: dict[str, object], *, started_wall_ns: int) -> None:
    path.write_bytes(_canonical(payload))
    os.utime(path, ns=(started_wall_ns, started_wall_ns))


def _endpoint_expectation(
    role: str = "lidar_publisher",
    protocol: str = "ltu1-v1",
) -> ReadinessExpectation:
    return ReadinessExpectation(
        adapter="local_endpoint",
        schema="lingtu.sim.local_endpoint.v1",
        role=role,
        protocol=protocol,
    )


def _feeder_expectation() -> ReadinessExpectation:
    return ReadinessExpectation(
        adapter="sim_feeder",
        schema="lingtu.sim.feeder_ready.v1",
        role="mujoco_feeder",
        protocol="mujoco-feeder-v1",
    )


def _feeder_payload() -> dict[str, object]:
    return {
        "backend": "mujoco",
        "endpoints": [
            {"protocol": "driver-v2", "role": "driver_bridge"},
            {"protocol": "ltu1-v1", "role": "lidar_publisher"},
            {"protocol": "ltu1-v1", "role": "imu_publisher"},
        ],
        "env": "sim",
        "product_session_id": PRODUCT_SESSION_ID,
        "first_physics_step_applied": True,
        "model_generation": 0,
        "process": "mujoco_feeder",
        "product": "teleop_avoid",
        "protocol": "mujoco-feeder-v1",
        "ready": True,
        "reset_generation": 0,
        "role": "mujoco_feeder",
        "schema": "lingtu.sim.feeder_ready.v1",
        "session_id": "test-session",
    }


def _endpoint_payload(
    *,
    role: str = "lidar_publisher",
    protocol: str = "ltu1-v1",
    auth_file: str = "sensor.auth",
) -> dict[str, object]:
    return {
        "auth_file": auth_file,
        "product_session_id": PRODUCT_SESSION_ID,
        "host": "127.0.0.1",
        "port": 32123,
        "protocol": protocol,
        "ready": True,
        "role": role,
        "schema": "lingtu.sim.local_endpoint.v1",
    }


def _load_endpoint(
    path: Path,
    *,
    role: str = "lidar_publisher",
    protocol: str = "ltu1-v1",
    started_wall_ns: int = STARTED_WALL_NS,
) -> object:
    return load_typed_readiness(
        path,
        expectation=_endpoint_expectation(role, protocol),
        product_session_id=PRODUCT_SESSION_ID,
        product="teleop_avoid",
        process=role,
        started_wall_ns=started_wall_ns,
    )


def _load_feeder(
    path: Path,
    *,
    lidar_required: bool = True,
    imu_required: bool = True,
    camera_required: bool = False,
) -> object:
    return load_typed_readiness(
        path,
        expectation=_feeder_expectation(),
        product_session_id=PRODUCT_SESSION_ID,
        product="teleop_avoid",
        process="mujoco_feeder",
        started_wall_ns=STARTED_WALL_NS,
        lidar_required=lidar_required,
        imu_required=imu_required,
        camera_required=camera_required,
    )


def _host_expectation() -> ReadinessExpectation:
    return ReadinessExpectation(
        adapter="host",
        schema=HOST_READY_SCHEMA,
        role="host",
        protocol="host-process-v1",
    )


def _host_payload() -> dict[str, object]:
    return {
        "env": "sim",
        "product_session_id": PRODUCT_SESSION_ID,
        "process": "host_runtime",
        "product": "teleop",
        "protocol": "host-process-v1",
        "ready": True,
        "role": "host",
        "schema": HOST_READY_SCHEMA,
    }


def _nav_expectation() -> ReadinessExpectation:
    return ReadinessExpectation(
        adapter="nav_status",
        schema=NAV_STATUS_SCHEMA,
        role="navd",
        protocol="nav-status-v1",
    )


def _slam_expectation() -> ReadinessExpectation:
    return ReadinessExpectation(
        adapter="slam_status",
        schema=SLAM_STATUS_SCHEMA,
        role="slamd",
        protocol="slam-status-v1",
    )


def _slam_payload(*, mode: str = "mapping") -> dict[str, object]:
    localization = mode == "localization"
    return {
        "schema_version": SLAM_STATUS_SCHEMA,
        "runtime_instance_id": "slam-runtime",
        "source": "cpp_cyclone_slam",
        "backend": "fastlio2",
        "mode": mode,
        "state": "TRACKING" if localization else "MAPPING",
        "reason": "healthy",
        "alive": True,
        "has_odom": True,
        "snapshot_written_at_s": STARTED_WALL_NS / 1_000_000_000,
        "imu_input": {
            "expected_frame_id": "imu_link",
            "accepted_frames": 20,
            "rejected_missing": 0,
            "rejected_mismatch": 0,
            "last_rejection_reason": "",
        },
        "imu_input_hz": 200.0,
        "lidar_input_hz": 10.0,
        "slam_tick_hz": 50.0,
        "processed_scan_hz": 10.0,
        "registered_points": 120,
        "observation_sequence": 4,
        "source_epoch": 1,
        "map_points": 300,
        "saved_map_points": 500 if localization else 0,
        "map_loaded": localization,
        "map_odom_tf": {
            "valid": True,
            "frame_id": "map",
            "child_frame_id": "odom",
            "tx": 0.0,
            "ty": 0.0,
            "tz": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "ts": STARTED_WALL_NS / 1_000_000_000,
        },
        "native_product": {
            "product": "teleop_avoid",
            "product_session_id": PRODUCT_SESSION_ID,
        },
    }


def _nav_payload() -> dict[str, object]:
    return {
        "schema_version": NAV_STATUS_SCHEMA,
        "endpoint": "navd",
        "control_mode": "teleop",
        "stamp_s": 1234.5,
        "native_product": {
            "product": "teleop",
            "product_session_id": PRODUCT_SESSION_ID,
        },
        "driver_control": {
            "received": True,
            "ready": True,
            "fresh": True,
        },
        "input_gate": {
            "ready": True,
            "driver_control_ready": True,
            "reason": "ready",
        },
        "control_loop_health": {
            "ready": True,
            "healthy": True,
        },
        "telemetry_extension": {"allowed": True},
    }


def _mapd_expectation() -> ReadinessExpectation:
    return ReadinessExpectation(
        adapter="mapd_status",
        schema=MAPD_STATUS_SCHEMA,
        role="mapd",
        protocol="maps-status-v1",
    )


def _mapd_payload() -> dict[str, object]:
    generation = 7
    return {
        "schema_version": MAPD_STATUS_SCHEMA,
        "process": "mapd",
        "native_product": {
            "product": "teleop_avoid",
            "product_session_id": PRODUCT_SESSION_ID,
        },
        "producer_boot_id": "mapd-boot",
        "status": "ready",
        "ready": True,
        "running": True,
        "live": True,
        "reset_epoch": 1,
        "observation_sequence": 4,
        "generation": generation,
        "queue_depth": 0,
        "live_points": 120,
        "voxel_points": 100,
        "voxel_cells": 80,
        "voxel_snapshot_omitted_cells": 0,
        "voxel_capacity_rejections": 0,
        "accumulated_cells": 90,
        "accumulated_snapshot_cells": 90,
        "accumulated_capacity_rejections": 0,
        "capacity_limited": False,
        "pose_quality": 1.0,
        "pose_state": "TRACKING",
        "pose_reason": "mujoco_navigation_fixture",
        "accepted_observations": 4,
        "processed_observations": 4,
        "replaced_observations": 0,
        "stale_observations": 0,
        "invalid_observations": 0,
        "dds_received": 4,
        "dds_decoded": 4,
        "dds_rejected": 0,
        "dds_write_attempts": 20,
        "dds_write_failures": 0,
        "dds_serialization_rejections": 0,
        "dds_scene_oversize_rejections": 0,
        "dds_unhealthy_writers": 0,
        "required_publications_ready": True,
        "current_generation_published": True,
        "state_published_generation": generation,
        "realtime_clouds_published_generation": generation,
        "map_layers_published_generation": generation,
        "scene_published_generation": generation,
        "engine_error": "",
        "input_error": "",
        "output_error": "",
    }


def _traversability_expectation() -> ReadinessExpectation:
    return ReadinessExpectation(
        adapter="traversability_status",
        schema=TRAVERSABILITY_STATUS_SCHEMA,
        role="lingtu_traversability_dds",
        protocol="traversability-status-v2",
    )


def _traversability_payload() -> dict[str, object]:
    return {
        "schema_version": TRAVERSABILITY_STATUS_SCHEMA,
        "endpoint": "lingtu_traversability_dds",
        "native_product": {
            "product": "teleop_avoid",
            "product_session_id": PRODUCT_SESSION_ID,
        },
        "stamp_s": 1234.5,
        "has_odom": True,
        "has_map_odom_tf": True,
        "frame_contract": {
            "odom_input_frame": "odom",
            "cloud_input_frame": "body",
            "geometry_frame": "map",
            "header_frame": "map",
            "map_odom_tf_age_s": 0.01,
            "cloud_pose_gap_s": 0.0,
            "last_error": "none",
        },
        "last_points": 120,
        "terrain_points": 100,
        "safety_grid": {"observed_before_overlays_cells": 80},
        "counters": {
            "odom": 4,
            "tf": 4,
            "registered_clouds": 4,
            "published": 4,
            "slow_published": 1,
            "map_clearing": 0,
            "cloud_clearing": 0,
            "tf_rejected": 0,
            "odom_frame_rejected": 0,
            "cloud_frame_rejected": 0,
            "map_frame_jump_clears": 0,
            "errors": 0,
        },
    }


def _explore_expectation() -> ReadinessExpectation:
    return ReadinessExpectation(
        adapter="explore_status",
        schema=EXPLORE_STATUS_SCHEMA,
        role="lingtu_explore_dds",
        protocol="explore-status-v2",
    )


def _explore_payload(*, route: str = "live") -> dict[str, object]:
    live = route == "live"
    return {
        "schema_version": EXPLORE_STATUS_SCHEMA,
        "endpoint": "lingtu_explore_dds",
        "product": "explore",
        "product_session_id": PRODUCT_SESSION_ID,
        "boot_id": "explore-boot",
        "stamp_s": 1234.5,
        "route": route,
        "state": "idle",
        "reason": "waiting_for_start",
        "ready": True,
        "map": {
            "frame_id": "map",
            "map_id": "" if live else "saved-map",
            "map_content_epoch": 0 if live else 3,
            "reset_epoch": 1,
            "generation": 7,
            "live": live,
        },
        "telemetry_extension": {"allowed": True},
    }


def _load_mapd(path: Path) -> object:
    return load_typed_readiness(
        path,
        expectation=_mapd_expectation(),
        product_session_id=PRODUCT_SESSION_ID,
        product="teleop_avoid",
        process="map_runtime",
        started_wall_ns=STARTED_WALL_NS,
    )


def _load_traversability(path: Path) -> object:
    return load_typed_readiness(
        path,
        expectation=_traversability_expectation(),
        product_session_id=PRODUCT_SESSION_ID,
        product="teleop_avoid",
        process="traversability_runtime",
        started_wall_ns=STARTED_WALL_NS,
    )


def _load_explore(path: Path, *, expected_route: str = "live") -> object:
    return load_typed_readiness(
        path,
        expectation=_explore_expectation(),
        product_session_id=PRODUCT_SESSION_ID,
        product="explore",
        process="explore_runtime",
        started_wall_ns=STARTED_WALL_NS,
        expected_explore_route=expected_route,
    )


def _load_nav(path: Path) -> object:
    return load_typed_readiness(
        path,
        expectation=_nav_expectation(),
        product_session_id=PRODUCT_SESSION_ID,
        product="teleop",
        process="nav_runtime",
        started_wall_ns=STARTED_WALL_NS,
        expected_control_mode="teleop",
    )


def _load_slam(path: Path, *, expected_mode: str = "mapping") -> object:
    return load_typed_readiness(
        path,
        expectation=_slam_expectation(),
        product_session_id=PRODUCT_SESSION_ID,
        product="teleop_avoid",
        process="slam_runtime",
        started_wall_ns=STARTED_WALL_NS,
        expected_slam_mode=expected_mode,
    )


def test_loads_mapping_slam_status_with_exact_runtime_evidence(tmp_path: Path) -> None:
    path = tmp_path / "slam.status.json"
    _write_fresh(path, _slam_payload(), started_wall_ns=STARTED_WALL_NS)

    loaded = _load_slam(path)

    assert loaded["adapter"] == "slam_status"
    assert loaded["details"] == {
        "mode": "mapping",
        "product_session_id": PRODUCT_SESSION_ID,
        "saved_map_points": 0,
        "map_loaded": False,
        "map_odom_tf_valid": True,
    }


def test_loads_localization_slam_only_after_saved_map_tracking(tmp_path: Path) -> None:
    path = tmp_path / "slam.status.json"
    _write_fresh(
        path,
        _slam_payload(mode="localization"),
        started_wall_ns=STARTED_WALL_NS,
    )

    loaded = _load_slam(path, expected_mode="localization")

    assert loaded["details"] == {
        "mode": "localization",
        "product_session_id": PRODUCT_SESSION_ID,
        "saved_map_points": 500,
        "map_loaded": True,
        "map_odom_tf_valid": True,
    }


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (("native_product", "product", "nav"), "Product identity"),
        (("source", None, "python_fixture"), "identity"),
        (("backend", None, "fixture"), "identity"),
        (("alive", None, False), "still starting"),
        (("imu_input_hz", None, 0.0), "input/output"),
        (("lidar_input_hz", None, 0.0), "input/output"),
        (("has_odom", None, False), "output"),
        (("registered_points", None, 0), "output"),
        (("observation_sequence", None, 0), "map observation"),
        (("map_points", None, 0), "map output"),
    ),
)
def test_mapping_slam_readiness_is_fail_closed(
    tmp_path: Path,
    mutation: tuple[str, str | None, object],
    message: str,
) -> None:
    payload = _slam_payload()
    field, nested, value = mutation
    if nested is None:
        payload[field] = value
    else:
        assert isinstance(payload[field], dict)
        payload[field][nested] = value
    path = tmp_path / "slam.status.json"
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises((SimReadinessError, SimReadinessPending), match=message):
        _load_slam(path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("map_loaded", False, "saved map"),
        ("state", "LOCALIZING", "tracking"),
        ("saved_map_points", 0, "saved map"),
        ("map_odom_tf", None, "map-odom"),
    ),
)
def test_localization_slam_readiness_requires_saved_map_tracking(
    tmp_path: Path,
    field: str,
    value: object,
    message: str,
) -> None:
    payload = _slam_payload(mode="localization")
    payload[field] = value
    path = tmp_path / "slam.status.json"
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessPending, match=message):
        _load_slam(path, expected_mode="localization")


def test_slam_mode_is_required_only_for_slam_adapter(tmp_path: Path) -> None:
    path = tmp_path / "slam.status.json"
    _write_fresh(path, _slam_payload(), started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="requires an exact mode"):
        load_typed_readiness(
            path,
            expectation=_slam_expectation(),
            product_session_id=PRODUCT_SESSION_ID,
            product="teleop_avoid",
            process="slam_runtime",
            started_wall_ns=STARTED_WALL_NS,
        )

    nav_path = tmp_path / "nav.status.json"
    _write_fresh(nav_path, _nav_payload(), started_wall_ns=STARTED_WALL_NS)
    with pytest.raises(SimReadinessError, match="only valid for slam"):
        load_typed_readiness(
            nav_path,
            expectation=_nav_expectation(),
            product_session_id=PRODUCT_SESSION_ID,
            product="teleop",
            process="nav_runtime",
            started_wall_ns=STARTED_WALL_NS,
            expected_control_mode="teleop",
            expected_slam_mode="mapping",
        )


def test_slam_status_rejects_missing_product_session_identity(tmp_path: Path) -> None:
    payload = _slam_payload()
    del payload["native_product"]["product_session_id"]
    path = tmp_path / "slam.status.json"
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="Product identity"):
        _load_slam(path)


def test_slam_status_rejects_foreign_product_session_identity(tmp_path: Path) -> None:
    payload = _slam_payload()
    payload["native_product"]["product_session_id"] = "3" * 32
    path = tmp_path / "slam.status.json"
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="Product identity"):
        _load_slam(path)


def _summary(
    *,
    expectation: ReadinessExpectation,
    product: str,
    process: str,
    details: dict[str, object],
) -> dict[str, object]:
    return {
        "adapter": expectation.adapter,
        "source_schema": expectation.schema,
        "role": expectation.role,
        "protocol": expectation.protocol,
        "product_session_id": PRODUCT_SESSION_ID,
        "product": product,
        "process": process,
        "ready": True,
        "details": details,
    }


def test_loads_exact_local_sensor_endpoint_readiness(tmp_path: Path) -> None:
    path = tmp_path / "sensor.ready.json"
    payload = _endpoint_payload()
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    loaded = _load_endpoint(path)

    assert loaded == _summary(
        expectation=_endpoint_expectation(),
        product="teleop_avoid",
        process="lidar_publisher",
        details={"host": "127.0.0.1", "port": 32123, "auth_file": "sensor.auth"},
    )


def test_loads_exact_mujoco_feeder_readiness(tmp_path: Path) -> None:
    path = tmp_path / "mujoco_feeder.ready.json"
    payload = _feeder_payload()
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    loaded = _load_feeder(path)

    assert loaded == _summary(
        expectation=_feeder_expectation(),
        product="teleop_avoid",
        process="mujoco_feeder",
        details={
            "endpoints": payload["endpoints"],
            "session_id": "test-session",
        },
    )


def test_feeder_camera_requirement_fails_closed(tmp_path: Path) -> None:
    path = tmp_path / "mujoco_feeder.ready.json"
    payload = _feeder_payload()
    payload["endpoints"].append(  # type: ignore[union-attr]
        {"protocol": "ltu1-v1", "role": "camera_publisher"}
    )
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    _load_feeder(path, camera_required=True)
    payload["endpoints"].pop()  # type: ignore[union-attr]
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)
    with pytest.raises(SimReadinessError, match="endpoints"):
        _load_feeder(path, camera_required=True)


def test_loads_exact_host_readiness(tmp_path: Path) -> None:
    path = tmp_path / "host.ready.json"
    payload = _host_payload()
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    loaded = load_typed_readiness(
        path,
        expectation=_host_expectation(),
        product_session_id=PRODUCT_SESSION_ID,
        product="teleop",
        process="host_runtime",
        started_wall_ns=STARTED_WALL_NS,
    )

    assert loaded == _summary(
        expectation=_host_expectation(),
        product="teleop",
        process="host_runtime",
        details={},
    )


def test_loads_fresh_pretty_printed_nav_status_with_exact_product_identity(
    tmp_path: Path,
) -> None:
    path = tmp_path / "nav.status.json"
    payload = _nav_payload()
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.utime(path, ns=(STARTED_WALL_NS, STARTED_WALL_NS))

    assert _load_nav(path) == _summary(
        expectation=_nav_expectation(),
        product="teleop",
        process="nav_runtime",
        details={"control_mode": "teleop"},
    )


def test_loads_ready_live_explore_status(tmp_path: Path) -> None:
    path = tmp_path / "explore.status.json"
    path.write_text(json.dumps(_explore_payload(), indent=2), encoding="utf-8")
    os.utime(path, ns=(STARTED_WALL_NS, STARTED_WALL_NS))

    assert _load_explore(path) == _summary(
        expectation=_explore_expectation(),
        product="explore",
        process="explore_runtime",
        details={
            "route": "live",
            "map": {
                "frame_id": "map",
                "map_id": "",
                "map_content_epoch": 0,
                "reset_epoch": 1,
                "generation": 7,
                "live": True,
            },
        },
    )


def test_loads_ready_saved_map_explore_status(tmp_path: Path) -> None:
    path = tmp_path / "explore.status.json"
    payload = _explore_payload(route="map")
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.utime(path, ns=(STARTED_WALL_NS, STARTED_WALL_NS))

    assert _load_explore(path, expected_route="map") == _summary(
        expectation=_explore_expectation(),
        product="explore",
        process="explore_runtime",
        details={
            "route": "map",
            "map": {
                "frame_id": "map",
                "map_id": "saved-map",
                "map_content_epoch": 3,
                "reset_epoch": 1,
                "generation": 7,
                "live": False,
            },
        },
    )


@pytest.mark.parametrize(
    ("field", "value"),
    (("map_id", "saved-map"), ("map_content_epoch", 1)),
)
def test_live_explore_status_rejects_saved_map_identity(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    path = tmp_path / "explore.status.json"
    payload = _explore_payload(route="live")
    payload["map"][field] = value  # type: ignore[index]
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="live map identity"):
        _load_explore(path, expected_route="live")


@pytest.mark.parametrize(
    ("status_route", "expected_route"),
    (("live", "map"), ("map", "live")),
)
def test_explore_status_rejects_route_mismatch(
    tmp_path: Path,
    status_route: str,
    expected_route: str,
) -> None:
    path = tmp_path / "explore.status.json"
    _write_fresh(
        path,
        _explore_payload(route=status_route),
        started_wall_ns=STARTED_WALL_NS,
    )

    with pytest.raises(SimReadinessError, match="identity") as caught:
        _load_explore(path, expected_route=expected_route)
    assert not isinstance(caught.value, SimReadinessPending)


@pytest.mark.parametrize("expected_route", (None, "", "external"))
def test_explore_status_requires_an_exact_expected_route(
    tmp_path: Path,
    expected_route: object,
) -> None:
    path = tmp_path / "explore.status.json"
    _write_fresh(path, _explore_payload(), started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="requires an exact route"):
        load_typed_readiness(
            path,
            expectation=_explore_expectation(),
            product_session_id=PRODUCT_SESSION_ID,
            product="explore",
            process="explore_runtime",
            started_wall_ns=STARTED_WALL_NS,
            expected_explore_route=expected_route,  # type: ignore[arg-type]
        )


def test_nav_readiness_sanitizes_native_nested_telemetry_extensions(
    tmp_path: Path,
) -> None:
    path = tmp_path / "nav.status.json"
    payload = _nav_payload()
    payload["driver_control"].update(  # type: ignore[union-attr]
        {
            "age_s": 0.01,
            "reason": "",
            "last_command_accepted": False,
        }
    )
    payload["input_gate"].update(  # type: ignore[union-attr]
        {"fresh_frames": 3, "required_frames": 3}
    )
    payload["control_loop_health"].update(  # type: ignore[union-attr]
        {"period_ms": 50.0, "deadline_misses": 0}
    )
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.utime(path, ns=(STARTED_WALL_NS, STARTED_WALL_NS))

    loaded = _load_nav(path)

    assert loaded["details"] == {"control_mode": "teleop"}


def test_feeder_without_lidar_requires_only_the_driver_endpoint(
    tmp_path: Path,
) -> None:
    path = tmp_path / "mujoco_feeder.ready.json"
    payload = _feeder_payload()
    payload["endpoints"] = payload["endpoints"][:1]  # type: ignore[index]
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    loaded = load_typed_readiness(
        path,
        expectation=_feeder_expectation(),
        product_session_id=PRODUCT_SESSION_ID,
        product="teleop_avoid",
        process="mujoco_feeder",
        started_wall_ns=STARTED_WALL_NS,
        lidar_required=False,
        imu_required=False,
    )

    assert loaded == _summary(
        expectation=_feeder_expectation(),
        product="teleop_avoid",
        process="mujoco_feeder",
        details={
            "endpoints": payload["endpoints"],
            "session_id": "test-session",
        },
    )


def test_canonical_process_readiness_registry_is_exact() -> None:
    assert readiness_expectation_for_process(
        "lidar_publisher", "lidar.ready.json"
    ) == _endpoint_expectation("lidar_publisher", "ltu1-v1")
    assert readiness_expectation_for_process(
        "imu_publisher", "imu.ready.json"
    ) == _endpoint_expectation("imu_publisher", "ltu1-v1")
    assert readiness_expectation_for_process(
        "camera_publisher", "camera.ready.json"
    ) == _endpoint_expectation("camera_publisher", "ltu1-v1")
    assert readiness_expectation_for_process(
        "driver_bridge", "driver.ready.json"
    ) == _endpoint_expectation("driver_bridge", "driver-v2")
    assert readiness_expectation_for_process(
        "mujoco_feeder", "mujoco_feeder.ready.json"
    ) == _feeder_expectation()
    assert readiness_expectation_for_process(
        "host_runtime", "host.ready.json"
    ) == _host_expectation()
    assert readiness_expectation_for_process(
        "nav_runtime", "nav.status.json"
    ) == _nav_expectation()
    assert readiness_expectation_for_process(
        "slam_runtime", "slam.status.json"
    ) == _slam_expectation()
    assert readiness_expectation_for_process(
        "map_runtime", "mapd.status.json"
    ) == _mapd_expectation()
    assert readiness_expectation_for_process(
        "traversability_runtime", "traversability.status.json"
    ) == _traversability_expectation()
    assert readiness_expectation_for_process(
        "explore_runtime", "explore.status.json"
    ) == _explore_expectation()
    assert readiness_expectation_for_process("worker", "ready.json") is None


@pytest.mark.parametrize(
    "mutate",
    (
        lambda value: value.update({"schema_version": "other"}),
        lambda value: value.update({"route": "map"}),
        lambda value: value.update({"ready": "true"}),
        lambda value: value["map"].update({"live": False}),  # type: ignore[index,union-attr]
    ),
)
def test_explore_status_readiness_rejects_invalid_evidence(
    tmp_path: Path,
    mutate: Callable[[dict[str, Any]], None],
) -> None:
    path = tmp_path / "explore.status.json"
    payload = copy.deepcopy(_explore_payload())
    mutate(payload)
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError) as caught:
        _load_explore(path)
    assert not isinstance(caught.value, SimReadinessPending)


@pytest.mark.parametrize(
    "mutate",
    (
        lambda value: value.pop("product"),
        lambda value: value.update({"product": "nav"}),
        lambda value: value.update({"product": "explore-extra"}),
        lambda value: value.update({"product_session_id": "b" * 64}),
    ),
)
def test_explore_status_readiness_requires_exact_product_identity(
    tmp_path: Path,
    mutate: Callable[[dict[str, Any]], object],
) -> None:
    path = tmp_path / "explore.status.json"
    payload = _explore_payload()
    mutate(payload)
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="Product identity") as caught:
        _load_explore(path)
    assert not isinstance(caught.value, SimReadinessPending)


def test_explore_status_readiness_treats_missing_inputs_as_pending(
    tmp_path: Path,
) -> None:
    path = tmp_path / "explore.status.json"
    payload = _explore_payload()
    payload.update(
        {
            "state": "waiting_odometry",
            "reason": "odometry_missing_or_stale",
            "ready": False,
            "map": None,
        }
    )
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessPending, match="waiting for live inputs"):
        _load_explore(path)


def test_mapd_readiness_requires_current_publication(
    tmp_path: Path,
) -> None:
    path = tmp_path / "mapd.status.json"
    payload = _mapd_payload()
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)
    assert _load_mapd(path) == _summary(
        expectation=_mapd_expectation(),
        product="teleop_avoid",
        process="map_runtime",
        details={
            "live": True,
            "reset_epoch": 1,
            "observation_sequence": 4,
            "processed_observations": 4,
        },
    )

    payload["current_generation_published"] = False
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)
    with pytest.raises(SimReadinessPending, match="current generation"):
        _load_mapd(path)


def test_rotating_status_read_race_is_pending(tmp_path: Path) -> None:
    path = tmp_path / "mapd.status.json"

    with pytest.raises(SimReadinessPending, match="temporarily unavailable"):
        _load_mapd(path)


def test_mapd_readiness_distinguishes_startup_from_broken_output(tmp_path: Path) -> None:
    path = tmp_path / "mapd.status.json"
    payload = _mapd_payload()
    payload.update(
        {
            "status": "waiting_for_observation",
            "ready": False,
            "live": False,
            "reset_epoch": 0,
            "observation_sequence": 0,
            "generation": 0,
            "accepted_observations": 0,
            "processed_observations": 0,
            "dds_received": 0,
            "dds_decoded": 0,
        }
    )
    for field in (
        "state_published_generation",
        "realtime_clouds_published_generation",
        "map_layers_published_generation",
        "scene_published_generation",
    ):
        payload[field] = 0
    payload["required_publications_ready"] = True
    payload["current_generation_published"] = True
    payload["pose_state"] = ""
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)
    with pytest.raises(SimReadinessPending, match="first observation"):
        _load_mapd(path)

    payload["output_error"] = "scene:dds_write_failed"
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)
    with pytest.raises(SimReadinessError, match="active error") as caught:
        _load_mapd(path)
    assert not isinstance(caught.value, SimReadinessPending)


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("capacity_limited", True),
        ("dds_unhealthy_writers", 1),
        ("ready", 1),
    ),
)
def test_mapd_readiness_fails_closed_on_capacity_or_type_corruption(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    path = tmp_path / "mapd.status.json"
    payload = _mapd_payload()
    payload[field] = value
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)
    with pytest.raises(SimReadinessError) as caught:
        _load_mapd(path)
    assert not isinstance(caught.value, SimReadinessPending)


def test_traversability_readiness_requires_synchronized_observed_map_input(
    tmp_path: Path,
) -> None:
    path = tmp_path / "traversability.status.json"
    payload = _traversability_payload()
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.utime(path, ns=(STARTED_WALL_NS, STARTED_WALL_NS))
    assert _load_traversability(path) == _summary(
        expectation=_traversability_expectation(),
        product="teleop_avoid",
        process="traversability_runtime",
        details={
            "has_odom": True,
            "has_map_odom_tf": True,
            "last_points": 120,
        },
    )

    payload["last_points"] = 0
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)
    with pytest.raises(SimReadinessPending, match="synchronized map-frame"):
        _load_traversability(path)


@pytest.mark.parametrize(
    ("payload_factory", "loader"),
    (
        (_mapd_payload, _load_mapd),
        (_traversability_payload, _load_traversability),
    ),
)
@pytest.mark.parametrize(
    "mutation",
    ("missing", "product", "product_session_id"),
)
def test_native_status_requires_exact_run_plan_product_identity(
    tmp_path: Path,
    payload_factory: Callable[[], dict[str, object]],
    loader: Callable[[Path], object],
    mutation: str,
) -> None:
    path = tmp_path / "native.status.json"
    payload = payload_factory()
    native_product = payload["native_product"]
    assert isinstance(native_product, dict)
    if mutation == "missing":
        del payload["native_product"]
    elif mutation == "product":
        native_product["product"] = "nav"
    elif mutation == "product_session_id":
        native_product["product_session_id"] = "e" * 64
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="Product identity"):
        loader(path)


def test_traversability_startup_sentinels_remain_pending(
    tmp_path: Path,
) -> None:
    path = tmp_path / "traversability.status.json"
    payload = _traversability_payload()
    payload.update(
        {
            "has_odom": False,
            "has_map_odom_tf": False,
            "last_points": 0,
            "terrain_points": 0,
        }
    )
    frame_contract = payload["frame_contract"]
    assert isinstance(frame_contract, dict)
    frame_contract.update(
        {
            "odom_input_frame": "",
            "cloud_input_frame": "",
            "map_odom_tf_age_s": -1.0,
            "cloud_pose_gap_s": -1.0,
        }
    )
    counters = payload["counters"]
    assert isinstance(counters, dict)
    counters.update({"odom": 0, "tf": 0, "registered_clouds": 0, "published": 0})
    safety_grid = payload["safety_grid"]
    assert isinstance(safety_grid, dict)
    safety_grid["observed_before_overlays_cells"] = 0
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessPending, match="synchronized map-frame"):
        _load_traversability(path)


def test_traversability_transient_startup_frame_error_remains_pending(
    tmp_path: Path,
) -> None:
    path = tmp_path / "traversability.status.json"
    payload = _traversability_payload()
    frame_contract = payload["frame_contract"]
    assert isinstance(frame_contract, dict)
    frame_contract["last_error"] = "map_odom_tf_stamp_invalid"
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessPending, match="recover from transient frame input"):
        _load_traversability(path)


@pytest.mark.parametrize(
    "mutate",
    (
        lambda value: value["frame_contract"].update(  # type: ignore[union-attr]
            {"map_odom_tf_age_s": -1.0}
        ),
        lambda value: value["frame_contract"].update(  # type: ignore[union-attr]
            {"cloud_pose_gap_s": -1.0}
        ),
    ),
)
def test_traversability_diagnostic_ages_do_not_control_readiness(
    tmp_path: Path,
    mutate: Callable[[dict[str, Any]], None],
) -> None:
    path = tmp_path / "traversability.status.json"
    payload = copy.deepcopy(_traversability_payload())
    mutate(payload)
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    _load_traversability(path)


@pytest.mark.parametrize(
    "mutate",
    (
        lambda value: value["frame_contract"].update(  # type: ignore[union-attr]
            {"cloud_input_frame": "lidar_link"}
        ),
        lambda value: value["frame_contract"].update(  # type: ignore[union-attr]
            {"last_error": "cloud_pose_unavailable"}
        ),
        lambda value: value.update({"has_odom": 1}),
    ),
)
def test_traversability_readiness_fails_closed_on_frame_or_type_corruption(
    tmp_path: Path,
    mutate: Callable[[dict[str, Any]], None],
) -> None:
    path = tmp_path / "traversability.status.json"
    payload = copy.deepcopy(_traversability_payload())
    mutate(payload)
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)
    with pytest.raises(SimReadinessError) as caught:
        _load_traversability(path)
    assert not isinstance(caught.value, SimReadinessPending)


def test_readiness_expectation_rejects_non_string_adapter() -> None:
    with pytest.raises(ValueError, match="approved adapter contract"):
        ReadinessExpectation(  # type: ignore[arg-type]
            adapter=[],
            schema="lingtu.sim.local_endpoint.v1",
            role="sensor_publisher",
            protocol="ltu1-v1",
        )


def test_feeder_readiness_rejects_empty_identity() -> None:
    payload = _feeder_payload()
    payload["product_session_id"] = ""

    with pytest.raises(SimReadinessError, match="identity"):
        validate_feeder_readiness(payload)


@pytest.mark.parametrize(
    ("adapter", "schema", "role", "protocol"),
    (
        ("unknown", "lingtu.sim.local_endpoint.v1", "sensor_publisher", "ltu1-v1"),
        ("local_endpoint", "other", "sensor_publisher", "ltu1-v1"),
        ("local_endpoint", "lingtu.sim.local_endpoint.v1", "sensor_publisher", "driver-v2"),
        ("local_endpoint", "lingtu.sim.local_endpoint.v1", "mujoco_feeder", "mujoco-feeder-v1"),
        ("sim_feeder", "lingtu.sim.feeder_ready.v1", "driver_bridge", "driver-v2"),
    ),
)
def test_readiness_expectation_rejects_unapproved_combinations(
    adapter: str,
    schema: str,
    role: str,
    protocol: str,
) -> None:
    with pytest.raises(ValueError, match="approved adapter contract"):
        ReadinessExpectation(adapter, schema, role, protocol)


def test_loads_exact_local_driver_endpoint_readiness(tmp_path: Path) -> None:
    path = tmp_path / "driver.ready.json"
    payload = _endpoint_payload(
        role="driver_bridge",
        protocol="driver-v2",
        auth_file="driver.auth",
    )
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    assert _load_endpoint(path, role="driver_bridge", protocol="driver-v2") == _summary(
        expectation=_endpoint_expectation("driver_bridge", "driver-v2"),
        product="teleop_avoid",
        process="driver_bridge",
        details={"host": "127.0.0.1", "port": 32123, "auth_file": "driver.auth"},
    )


@pytest.mark.parametrize(
    "mutate",
    (
        lambda value: value.update({"ready": 1}),
        lambda value: value.update({"host": "localhost"}),
        lambda value: value.update({"port": True}),
        lambda value: value.update({"port": 0}),
        lambda value: value.update({"port": 65536}),
        lambda value: value.update({"auth_file": "../sensor.auth"}),
        lambda value: value.update({"product_session_id": "b" * 64}),
        lambda value: value.update({"role": "driver_bridge"}),
        lambda value: value.update({"protocol": "driver-v2"}),
    ),
)
def test_local_endpoint_readiness_is_fail_closed(
    tmp_path: Path,
    mutate: Callable[[dict[str, object]], None],
) -> None:
    path = tmp_path / "sensor.ready.json"
    payload = _endpoint_payload()
    mutate(payload)
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError):
        _load_endpoint(path)


def test_local_endpoint_auth_file_cannot_alias_its_readiness_file(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor.ready.json"
    payload = _endpoint_payload(auth_file=path.name)
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="must differ"):
        _load_endpoint(path)


@pytest.mark.parametrize(
    "mutate",
    (
        lambda value: value.update({"schema_version": "other"}),
        lambda value: value.update({"endpoint": "other"}),
        lambda value: value.update({"control_mode": "teleop_avoid"}),
        lambda value: value.update({"stamp_s": float("nan")}),
        lambda value: value["native_product"].update(  # type: ignore[index,union-attr]
            {"product": "nav"}
        ),
        lambda value: value["native_product"].update(  # type: ignore[index,union-attr]
            {"product_session_id": "b" * 64}
        ),
        lambda value: value["driver_control"].update(  # type: ignore[index,union-attr]
            {"fresh": False}
        ),
        lambda value: value["input_gate"].update(  # type: ignore[index,union-attr]
            {"ready": False}
        ),
        lambda value: value["input_gate"].update(  # type: ignore[index,union-attr]
            {"reason": "recovering"}
        ),
        lambda value: value["control_loop_health"].update(  # type: ignore[index,union-attr]
            {"healthy": False}
        ),
    ),
)
def test_nav_status_readiness_is_fail_closed(
    tmp_path: Path,
    mutate: Callable[[dict[str, Any]], None],
) -> None:
    path = tmp_path / "nav.status.json"
    payload = copy.deepcopy(_nav_payload())
    mutate(payload)
    path.write_text(json.dumps(payload, allow_nan=True, indent=2), encoding="utf-8")
    os.utime(path, ns=(STARTED_WALL_NS, STARTED_WALL_NS))

    with pytest.raises(SimReadinessError):
        _load_nav(path)


def test_nav_status_treats_valid_startup_state_as_pending(tmp_path: Path) -> None:
    path = tmp_path / "nav.status.json"
    payload = copy.deepcopy(_nav_payload())
    payload["driver_control"].update(  # type: ignore[union-attr]
        {"received": True, "ready": False, "fresh": False}
    )
    payload["input_gate"].update(  # type: ignore[union-attr]
        {
            "ready": False,
            "driver_control_ready": False,
            "reason": "driver_control_rejected",
        }
    )
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessPending, match="waiting for fresh Driver"):
        _load_nav(path)


@pytest.mark.parametrize("value", (None, 0, 1, "true", [], {}))
def test_nav_status_rejects_non_boolean_driver_readiness(
    tmp_path: Path,
    value: object,
) -> None:
    path = tmp_path / "nav.status.json"
    payload = copy.deepcopy(_nav_payload())
    payload["driver_control"].update({"fresh": value})  # type: ignore[union-attr]
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="evidence is invalid") as caught:
        _load_nav(path)
    assert not isinstance(caught.value, SimReadinessPending)


def test_nav_status_readiness_requires_an_explicit_control_mode(
    tmp_path: Path,
) -> None:
    path = tmp_path / "nav.status.json"
    _write_fresh(path, _nav_payload(), started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError, match="exact control mode"):
        load_typed_readiness(
            path,
            expectation=_nav_expectation(),
            product_session_id=PRODUCT_SESSION_ID,
            product="teleop",
            process="nav_runtime",
            started_wall_ns=STARTED_WALL_NS,
        )


def test_readiness_loader_rejects_a_file_older_than_the_process(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor.ready.json"
    _write_fresh(path, _endpoint_payload(), started_wall_ns=STARTED_WALL_NS - 1)

    with pytest.raises(SimReadinessError, match="predates"):
        _load_endpoint(path)


@pytest.mark.parametrize(
    "mutate",
    (
        lambda value: value.update({"ready": 1}),
        lambda value: value.update({"first_physics_step_applied": False}),
        lambda value: value.update({"env": "real"}),
        lambda value: value.update({"backend": "other"}),
        lambda value: value.update({"product": "nav"}),
        lambda value: value.update({"process": "other"}),
        lambda value: value.update({"session_id": ""}),
        lambda value: value.update({"model_generation": True}),
        lambda value: value.update({"model_generation": -1}),
        lambda value: value.update({"reset_generation": 1.0}),
        lambda value: value.update({"endpoints": []}),
        lambda value: value["endpoints"].reverse(),  # type: ignore[union-attr]
        lambda value: value["endpoints"][0].update(  # type: ignore[index,union-attr]
            {"protocol": "ltu1-v1"}
        ),
    ),
)
def test_sim_feeder_readiness_is_fail_closed(
    tmp_path: Path,
    mutate: Callable[[dict[str, Any]], None],
) -> None:
    path = tmp_path / "mujoco_feeder.ready.json"
    payload = copy.deepcopy(_feeder_payload())
    mutate(payload)
    _write_fresh(path, payload, started_wall_ns=STARTED_WALL_NS)

    with pytest.raises(SimReadinessError):
        _load_feeder(path)
