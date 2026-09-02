"""Tests for per-stream Sensor Runtime qualification."""

# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

import pytest

from sim.runtime.sensors import SensorRuntime
from sim.runtime.sensors.readiness import (
    SensorReadiness,
    SensorReadinessError,
    SensorStreamState,
)

SENSOR_PLAN = (
    Path(__file__).resolve().parents[2]
    / "build"
    / "session-bundles"
    / "thunderv4-unreal"
    / "sensor.plan.json"
)


def _sensor_plan() -> dict[str, Any]:
    return cast(dict[str, Any], json.loads(SENSOR_PLAN.read_text(encoding="utf-8")))


def test_truth_odom_alone_does_not_make_required_sensor_facet_ready() -> None:
    readiness = SensorReadiness.from_plan(
        _sensor_plan(),
        model_generation=4,
        reset_generation=2,
        optional_stream_ids={"thunder_01.front_rgb", "thunder_01.front_depth"},
    )

    assert not readiness.is_ready

    truth_only = readiness.mark_prepared(
        "thunder_01.truth_odom",
        source="mujoco_truth",
        model_generation=4,
        reset_generation=2,
    ).mark_active(
        "thunder_01.truth_odom",
        source="mujoco_truth",
        model_generation=4,
        reset_generation=2,
    )

    assert truth_only.state("thunder_01.truth_odom") is SensorStreamState.ACTIVE
    assert not truth_only.is_ready
    with pytest.raises(SensorReadinessError, match=r"thunder_01\.imu"):
        truth_only.require_ready()


def test_required_streams_must_all_be_active_and_optional_streams_never_block() -> None:
    readiness = SensorReadiness.from_plan(
        _sensor_plan(),
        required_stream_ids={"thunder_01.imu", "thunder_01.mid360"},
        model_generation=1,
        reset_generation=0,
    )
    active_imu = readiness.mark_prepared(
        "thunder_01.imu", source="mujoco_sensor"
    ).mark_active("thunder_01.imu", source="mujoco_sensor")

    assert not active_imu.is_ready

    ready = active_imu.mark_prepared(
        "thunder_01.mid360", source="mujoco_livox_model"
    ).mark_active("thunder_01.mid360", source="mujoco_livox_model")

    assert ready.is_ready
    assert ready.state("thunder_01.front_rgb") is SensorStreamState.UNBOUND
    ready.require_ready()


def test_active_stream_can_be_retracted_to_prepared_and_block_readiness() -> None:
    readiness = SensorReadiness.from_plan(
        _sensor_plan(),
        required_stream_ids={"thunder_01.imu"},
        model_generation=1,
        reset_generation=0,
    )
    active = readiness.mark_prepared(
        "thunder_01.imu", source="mujoco_sensor"
    ).mark_active("thunder_01.imu", source="mujoco_sensor")

    retracted = active.mark_prepared("thunder_01.imu", source="mujoco_sensor")

    assert retracted.state("thunder_01.imu") is SensorStreamState.PREPARED
    assert not retracted.is_ready
    with pytest.raises(SensorReadinessError, match="thunder_01\\.imu"):
        retracted.require_ready()


def test_active_requires_prepared_and_rejects_unknown_stream_source_and_generation() -> None:
    readiness = SensorReadiness.from_runtime(
        SensorRuntime.from_path(SENSOR_PLAN),
        required_stream_ids={"thunder_01.imu"},
        model_generation=3,
        reset_generation=5,
    )

    with pytest.raises(SensorReadinessError, match="PREPARED"):
        readiness.mark_active("thunder_01.imu", source="mujoco_sensor")
    with pytest.raises(SensorReadinessError, match="unknown stream"):
        readiness.mark_prepared("thunder_01.robot_package_id", source="mujoco_sensor")
    with pytest.raises(SensorReadinessError, match="source"):
        readiness.mark_prepared("thunder_01.imu", source="robot_package_id")
    with pytest.raises(SensorReadinessError, match="model_generation"):
        readiness.mark_prepared(
            "thunder_01.imu",
            source="mujoco_sensor",
            model_generation=2,
            reset_generation=5,
        )
    with pytest.raises(SensorReadinessError, match="reset_generation"):
        readiness.mark_prepared(
            "thunder_01.imu",
            source="mujoco_sensor",
            model_generation=3,
            reset_generation=4,
        )


def test_reset_preserves_failure_while_model_generation_invalidates_all_bindings() -> None:
    readiness = SensorReadiness.from_plan(
        _sensor_plan(),
        required_stream_ids={"thunder_01.imu", "thunder_01.mid360"},
        model_generation=8,
        reset_generation=1,
    )
    qualified = (
        readiness.mark_prepared("thunder_01.imu", source="mujoco_sensor")
        .mark_active("thunder_01.imu", source="mujoco_sensor")
        .mark_failed("thunder_01.mid360", "dds writer unavailable", source="mujoco_livox_model")
    )

    reset = qualified.with_generations(model_generation=8, reset_generation=2)
    assert reset.state("thunder_01.imu") is SensorStreamState.ACTIVE
    assert reset.state("thunder_01.mid360") is SensorStreamState.FAILED
    assert reset.failure_reason("thunder_01.mid360") == "dds writer unavailable"
    assert reset.streams["thunder_01.mid360"].reset_generation == 2
    assert not reset.is_ready
    with pytest.raises(SensorReadinessError, match="UNBOUND before PREPARED"):
        reset.mark_prepared("thunder_01.mid360", source="mujoco_livox_model")
    with pytest.raises(SensorReadinessError, match="PREPARED before ACTIVE"):
        reset.mark_active("thunder_01.mid360", source="mujoco_livox_model")

    rebound = reset.with_generations(model_generation=9, reset_generation=0)
    assert rebound.state("thunder_01.imu") is SensorStreamState.UNBOUND
    assert rebound.state("thunder_01.mid360") is SensorStreamState.UNBOUND
    assert rebound.failure_reason("thunder_01.mid360") is None
    assert not rebound.is_ready


def test_manifest_snapshot_is_serializable_and_uses_compiled_stream_ids() -> None:
    readiness = SensorReadiness.from_plan(
        _sensor_plan(),
        required_stream_ids={"thunder_01.truth_odom"},
        model_generation=2,
        reset_generation=7,
    ).mark_prepared("thunder_01.truth_odom", source="mujoco_truth")
    snapshot = readiness.to_manifest()

    json.dumps(snapshot)
    assert snapshot["model_generation"] == 2
    assert snapshot["reset_generation"] == 7
    assert snapshot["is_ready"] is False
    assert snapshot["streams"]["thunder_01.truth_odom"] == {
        "stream_id": "thunder_01.truth_odom",
        "stream_kind": "truth_odom",
        "required": True,
        "state": "PREPARED",
        "source": "mujoco_truth",
        "owner": "physics",
        "transport": "typed_dds",
        "model_generation": 2,
        "reset_generation": 7,
        "failure_reason": None,
    }
    assert "robot.package.yaml" not in snapshot["streams"]
