"""Strict same-session evidence for the ThunderV4 navigation sensor rig."""

# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

import pytest

from sim.runtime.sensors import (
    SensorEvidenceError,
    SensorRuntime,
    build_sensor_stream_summary,
    build_thunderv4_navigation_stream_summary,
    sensor_stream_binding_identity,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SENSOR_PLAN = (
    REPO_ROOT / "build" / "session-bundles" / "thunderv4-unreal" / "sensor.plan.json"
)


def _plan() -> SensorRuntime:
    return SensorRuntime.from_plan(
        cast(dict[str, Any], json.loads(SENSOR_PLAN.read_text(encoding="utf-8")))
    )


def _active_observations(plan: SensorRuntime) -> list[dict[str, Any]]:
    runtime_sources = {
        "depth": "robotsimue-camera",
        "rgb": "robotsimue-camera",
        "imu": "mujoco-imu-dds",
        "mid360": "mujoco-mid360-dds",
        "truth_odom": "truth-odom-dds",
    }
    shm = {
        "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
        "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
    }
    observations: list[dict[str, Any]] = []
    for stream in plan.streams:
        runtime_source_id = runtime_sources[stream.stream_kind]
        observation: dict[str, Any] = {
            "sensor_id": stream.sensor_id,
            "state": "ACTIVE",
            "session_id": plan.session_id,
            "model_generation": 3,
            "reset_generation": 4,
            "owner": stream.route.owner,
            "source": stream.route.source,
            "transport": stream.route.transport,
            "message_type": stream.message_type,
            "runtime_source_id": runtime_source_id,
            "binding_identity": sensor_stream_binding_identity(
                stream,
                session_id=plan.session_id,
                model_generation=3,
                reset_generation=4,
                runtime_source_id=runtime_source_id,
            ),
            "sample_count": 2,
            "last_sample_truth_sequence": 2,
            "last_sample_sim_time_ns": 2_000_000_000,
        }
        if stream.sensor_id in shm:
            observation["shm_name"] = shm[stream.sensor_id]
        if stream.stream_kind == "mid360":
            observation["fidelity"] = {
                "reflectivity_semantics": "explicit_conservative_proxy",
                "scan_time_profile": "instantaneous_geometry/scheduled_offsets",
                "unknown_line_representation": "line_0_unknown_physical_channel",
            }
        observations.append(observation)
    return observations


def test_five_active_streams_with_samples_form_one_ready_summary() -> None:
    plan = _plan()
    summary = build_thunderv4_navigation_stream_summary(
        plan,
        _active_observations(plan),
        model_generation=3,
        reset_generation=4,
        shm_allocations={
            "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
            "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
        },
    )

    assert summary["schema"] == "lingtu.sim.sensor-stream-summary.v1"
    assert summary["is_ready"] is True
    assert summary["session_id"] == plan.session_id
    assert summary["model_generation"] == 3
    assert summary["reset_generation"] == 4
    assert summary["required_stream_ids"] == [
        "thunder_01.front_depth",
        "thunder_01.front_rgb",
        "thunder_01.imu",
        "thunder_01.mid360",
        "thunder_01.truth_odom",
    ]
    assert all(
        stream["state"] == "ACTIVE" and stream["sample_count"] > 0
        for stream in summary["streams"].values()
    )
    assert summary["streams"]["thunder_01.mid360"]["fidelity"] == {
        "qualification": "LIMITED",
        "reflectivity_semantics": "explicit_conservative_proxy",
        "scan_time_profile": "instantaneous_geometry/scheduled_offsets",
        "unknown_line_representation": "line_0_unknown_physical_channel",
        "limitations": [
            "reflectivity_is_a_conservative_proxy",
            "line_0_means_unknown_physical_channel",
            "ray_geometry_is_instantaneous_and_offsets_do_not_model_motion_distortion",
        ],
    }


def test_missing_required_stream_is_an_explicit_readiness_blocker() -> None:
    plan = _plan()
    observations = [
        item
        for item in _active_observations(plan)
        if item["sensor_id"] != "thunder_01.imu"
    ]

    summary = build_sensor_stream_summary(
        plan,
        observations,
        model_generation=3,
        reset_generation=4,
        shm_allocations={
            "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
            "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
        },
    )

    assert summary["is_ready"] is False
    assert summary["blocking_reasons"] == {
        "thunder_01.imu": "stream evidence is MISSING"
    }
    assert summary["streams"]["thunder_01.imu"]["state"] == "MISSING"


def test_active_stream_with_zero_samples_cannot_make_summary_ready() -> None:
    plan = _plan()
    observations = _active_observations(plan)
    empty_mid360 = next(
        item for item in observations if item["sensor_id"] == "thunder_01.mid360"
    )
    empty_mid360["sample_count"] = 0
    empty_mid360["last_sample_truth_sequence"] = None
    empty_mid360["last_sample_sim_time_ns"] = None

    summary = build_thunderv4_navigation_stream_summary(
        plan,
        observations,
        model_generation=3,
        reset_generation=4,
        shm_allocations={
            "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
            "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
        },
    )

    assert summary["is_ready"] is False
    assert summary["blocking_reasons"] == {
        "thunder_01.mid360": "ACTIVE stream has no published sample or frame"
    }


def test_failed_mid360_without_a_frame_keeps_honest_fidelity_limitations() -> None:
    plan = _plan()
    observations = _active_observations(plan)
    mid360 = next(
        item
        for item in observations
        if item["sensor_id"] == "thunder_01.mid360"
    )
    mid360["state"] = "FAILED"
    mid360["sample_count"] = 0
    mid360["last_sample_truth_sequence"] = None
    mid360["last_sample_sim_time_ns"] = None
    mid360["failure_reason"] = "raycast unavailable"
    del mid360["fidelity"]

    summary = build_thunderv4_navigation_stream_summary(
        plan,
        observations,
        model_generation=3,
        reset_generation=4,
        shm_allocations={
            "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
            "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
        },
    )

    assert summary["is_ready"] is False
    assert summary["blocking_reasons"] == {
        "thunder_01.mid360": "stream is FAILED"
    }
    assert summary["streams"]["thunder_01.mid360"]["fidelity"][
        "qualification"
    ] == "LIMITED"


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("session_id", "stale-session", "session_id mismatch"),
        ("model_generation", 2, "model_generation is stale"),
        ("reset_generation", 3, "reset_generation is stale"),
    ],
)
def test_stale_stream_identity_is_rejected(
    field: str,
    value: str | int,
    message: str,
) -> None:
    plan = _plan()
    observations = _active_observations(plan)
    next(
        item for item in observations if item["sensor_id"] == "thunder_01.imu"
    )[field] = value

    with pytest.raises(SensorEvidenceError, match=message):
        build_thunderv4_navigation_stream_summary(
            plan,
            observations,
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )


def test_duplicate_stream_cannot_merge_different_runtime_binding_identities() -> None:
    plan = _plan()
    observations = _active_observations(plan)
    imu = next(
        item for item in observations if item["sensor_id"] == "thunder_01.imu"
    )
    forged = dict(imu)
    forged["runtime_source_id"] = "unrelated-replay-publisher"
    stream = next(
        item for item in plan.streams if item.sensor_id == "thunder_01.imu"
    )
    forged["binding_identity"] = sensor_stream_binding_identity(
        stream,
        session_id=plan.session_id,
        model_generation=3,
        reset_generation=4,
        runtime_source_id="unrelated-replay-publisher",
    )
    observations.append(forged)

    with pytest.raises(SensorEvidenceError, match="duplicate evidence"):
        build_thunderv4_navigation_stream_summary(
            plan,
            observations,
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )


def test_unknown_extra_stream_evidence_fails_closed() -> None:
    plan = _plan()
    observations = _active_observations(plan)
    extra = dict(observations[0])
    extra["sensor_id"] = "thunder_01.extra_camera"
    observations.append(extra)

    with pytest.raises(SensorEvidenceError, match="unknown stream"):
        build_thunderv4_navigation_stream_summary(
            plan,
            observations,
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )


def test_lidar_cannot_reuse_the_imu_binding_identity() -> None:
    plan = _plan()
    observations = _active_observations(plan)
    imu = next(
        item for item in observations if item["sensor_id"] == "thunder_01.imu"
    )
    mid360 = next(
        item for item in observations if item["sensor_id"] == "thunder_01.mid360"
    )
    mid360["binding_identity"] = imu["binding_identity"]

    with pytest.raises(
        SensorEvidenceError,
        match=r"thunder_01\.mid360 binding_identity mismatch",
    ):
        build_thunderv4_navigation_stream_summary(
            plan,
            observations,
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )


def test_camera_evidence_requires_the_exact_run_allocation_shm_name() -> None:
    plan = _plan()
    observations = _active_observations(plan)
    rgb = next(
        item
        for item in observations
        if item["sensor_id"] == "thunder_01.front_rgb"
    )
    rgb["shm_name"] = "Local\\forged-camera-buffer"

    with pytest.raises(SensorEvidenceError, match="does not match run allocation"):
        build_thunderv4_navigation_stream_summary(
            plan,
            observations,
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )


def test_required_camera_without_a_run_shm_allocation_fails_closed() -> None:
    plan = _plan()

    with pytest.raises(SensorEvidenceError, match="has no run SHM allocation"):
        build_thunderv4_navigation_stream_summary(
            plan,
            _active_observations(plan),
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )


def test_camera_shm_allocation_collision_fails_closed_case_insensitively() -> None:
    plan = _plan()

    with pytest.raises(SensorEvidenceError, match="SHM allocation collision"):
        build_thunderv4_navigation_stream_summary(
            plan,
            _active_observations(plan),
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\Shared-Camera-Buffer",
                "thunder_01.front_rgb": "local\\shared-camera-buffer",
            },
        )


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("owner", "visual"),
        ("source", "replay_imu"),
        ("transport", "camera_shm"),
        ("message_type", "lingtu.dds.LivoxFrame"),
    ],
)
def test_stream_route_and_message_type_must_match_the_compiled_plan(
    field: str,
    value: str,
) -> None:
    plan = _plan()
    observations = _active_observations(plan)
    imu = next(
        item for item in observations if item["sensor_id"] == "thunder_01.imu"
    )
    imu[field] = value

    with pytest.raises(SensorEvidenceError, match=rf"imu {field} mismatch"):
        build_thunderv4_navigation_stream_summary(
            plan,
            observations,
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )


def test_mid360_cannot_claim_full_motion_distortion_fidelity() -> None:
    plan = _plan()
    observations = _active_observations(plan)
    mid360 = next(
        item for item in observations if item["sensor_id"] == "thunder_01.mid360"
    )
    mid360["fidelity"]["scan_time_profile"] = "physical_rolling/subscan"

    with pytest.raises(
        SensorEvidenceError,
        match="scan_time_profile must disclose",
    ):
        build_thunderv4_navigation_stream_summary(
            plan,
            observations,
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )


def test_thunderv4_navigation_summary_rejects_an_extra_plan_stream() -> None:
    document = cast(
        dict[str, Any], json.loads(SENSOR_PLAN.read_text(encoding="utf-8"))
    )
    extra = dict(document["streams"]["truth_odom"][0])
    extra["sensor_id"] = "thunder_01.unplanned_truth"
    document["streams"]["truth_odom"].append(extra)
    plan = SensorRuntime.from_plan(document)

    with pytest.raises(SensorEvidenceError, match="exact five-stream set"):
        build_thunderv4_navigation_stream_summary(
            plan,
            (),
            model_generation=3,
            reset_generation=4,
            shm_allocations={},
        )


def test_thunderv4_navigation_summary_rejects_a_tampered_plan_route() -> None:
    document = cast(
        dict[str, Any], json.loads(SENSOR_PLAN.read_text(encoding="utf-8"))
    )
    document["streams"]["imu"][0]["owner"] = "visual"
    plan = SensorRuntime.from_plan(document)

    with pytest.raises(SensorEvidenceError, match="route contract mismatch"):
        build_thunderv4_navigation_stream_summary(
            plan,
            _active_observations(plan),
            model_generation=3,
            reset_generation=4,
            shm_allocations={
                "thunder_01.front_depth": "Local\\lingtu-test-front-depth",
                "thunder_01.front_rgb": "Local\\lingtu-test-front-rgb",
            },
        )
