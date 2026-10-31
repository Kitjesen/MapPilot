from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest
from sim.catalog import CatalogResolver
from sim.runtime.coordinator.run_allocation import RunAllocation, load_resolved_session_bundle
from sim.runtime.sensors import (
    Mid360EndpointFactory,
    RangeNoiseConfig,
    SensorEndpoint,
    SensorRuntime,
    SessionSensorRuntime,
    build_sensor_stream_summary,
)
from sim.runtime.sensors.dds_adapter import encode_mid360_frame_sample


def _plan(noise: dict[str, Any] | None = None) -> dict[str, Any]:
    stream: dict[str, Any] = {
        "instance_id": "robot_01",
        "sensor_id": "robot_01.mid360",
        "frame_id": "robot_01/lidar",
        "raycast_frame_stable_id": "robot_01/lidar_site",
        "rate_hz": 10,
        "owner": "physics",
        "source": "mujoco_livox_model",
        "transport": "typed_dds",
        "message_type": "lingtu.dds.LivoxFrame",
    }
    if noise is not None:
        stream["range_noise"] = noise
    return {
        "schema": "lingtu.sim.sensor-plan.v1",
        "env": "sim",
        "session_id": "range-noise-test",
        "streams": {"mid360": [stream]},
    }


class RaycastProcess:
    """Boundary double: immutable collision hits supplied by the Physics Runtime."""

    def __init__(self) -> None:
        self.hits = [
            {"xyz_sensor": [3.0, 4.0, 0.0], "offset_time_ns": 0,
             "reflectivity": 23, "tag": 7, "line": 2},
            {"xyz_sensor": [0.0, 0.0, -2.0], "offset_time_ns": 5_000_000,
             "reflectivity": 24, "tag": 8, "line": 3},
        ]

    def raycast(self, **request: Any) -> dict[str, Any]:
        return {**request, "hits": self.hits}


class Cursor:
    def next_rays(self) -> tuple[Any, Any]:
        return ((0.6, 0.8, 0.0), (0.0, 0.0, -1.0)), (0, 5_000_000)


def _extractor(tmp_path: Path, runtime: SensorRuntime, process: RaycastProcess):
    allocation = RunAllocation(
        run_id="range-noise-test", run_dir=tmp_path, artifact_root=tmp_path.resolve(),
        log_dir=tmp_path / "logs", ports={}, shm={}, session_id=runtime.session_id,
        boot_id="test", dds_domain=42,
    )
    endpoint = Mid360EndpointFactory(
        tmp_path / "publisher.exe", process, cursor_factory=Cursor,
    )(runtime.streams[0], allocation)
    assert endpoint is not None
    return endpoint.extractor


def test_plan_noise_reaches_production_extractor_without_changing_physics_hits(
    tmp_path: Path,
) -> None:
    runtime = SensorRuntime.from_plan(_plan({"enabled": True, "stddev_m": 0.02, "seed": 17}))
    assert runtime.streams[0].range_noise.enabled is True
    process = RaycastProcess()
    extractor = _extractor(tmp_path, runtime, process)
    request = runtime.advance(sim_time_ns=0, reset_generation=0).samples[0]

    measured = extractor(request, {})

    assert [(point.x, point.y, point.z) for point in measured.points] != [
        (3.0, 4.0, 0.0), (0.0, 0.0, -2.0),
    ]
    assert [hit["xyz_sensor"] for hit in process.hits] == [[3.0, 4.0, 0.0], [0.0, 0.0, -2.0]]
    assert [(point.offset_time_ns, point.reflectivity, point.tag, point.line)
            for point in measured.points] == [(0, 23, 7, 2), (5_000_000, 24, 8, 3)]
    assert measured.stamp.sequence == 0
    assert measured.range_noise == runtime.streams[0].range_noise


class RecordingSink:
    def start(self) -> dict[str, bool]:
        return {"ready": True}

    def publish(self, sample: Any) -> None:
        self.sample = sample

    def close(self) -> None:
        pass


def test_noise_is_disclosed_in_session_evidence_and_strict_summary(tmp_path: Path) -> None:
    noise = {"enabled": True, "stddev_m": 0.02, "seed": 17}
    runtime = SensorRuntime.from_plan(_plan(noise))
    extractor = _extractor(tmp_path, runtime, RaycastProcess())
    sink = RecordingSink()
    session = SessionSensorRuntime(
        plan=runtime, allocation=object(),
        endpoint_factory=lambda stream, allocation: SensorEndpoint(
            source_id="mujoco-mid360-dds", sink=sink, extractor=extractor,
        ),
    )
    session.prepare()
    session.process_snapshot({"sim_time_ns": 0, "model_generation": 0, "reset_generation": 0})
    observation = session.evidence_observations()[0]
    assert observation["fidelity"]["range_noise"] == noise

    summary = build_sensor_stream_summary(
        runtime, [observation], model_generation=0, reset_generation=0,
    )
    fidelity = summary["streams"]["robot_01.mid360"]["fidelity"]
    assert fidelity["range_noise"] == noise
    assert "range_noise_is_an_uncalibrated_saturated_gaussian_model" in fidelity["limitations"]

    for field, changed in (("stddev_m", 0.03), ("seed", 18)):
        observation["fidelity"]["range_noise"] = {**noise, field: changed}
        with pytest.raises(ValueError, match="parameters do not match"):
            build_sensor_stream_summary(runtime, [observation], model_generation=0, reset_generation=0)

    extractor.set_noise_enabled(False)
    session.process_snapshot({"sim_time_ns": 100_000_000, "model_generation": 0, "reset_generation": 0})
    observation = session.evidence_observations()[0]
    summary = build_sensor_stream_summary(runtime, [observation], model_generation=0, reset_generation=0)
    assert summary["streams"]["robot_01.mid360"]["fidelity"]["range_noise"] == {**noise, "enabled": False}

    del observation["fidelity"]["range_noise"]
    with pytest.raises(ValueError, match="range_noise"):
        build_sensor_stream_summary(runtime, [observation], model_generation=0, reset_generation=0)
    session.close()


def test_switch_off_and_zero_strength_preserve_native_wire_bytes(tmp_path: Path) -> None:
    baseline = SensorRuntime.from_plan(_plan())
    configured = SensorRuntime.from_plan(_plan({"enabled": False, "stddev_m": 0.02, "seed": 17}))
    plain = _extractor(tmp_path, baseline, RaycastProcess())
    controlled = _extractor(tmp_path, configured, RaycastProcess())
    for index, enabled in enumerate((False, True, False)):
        controlled.set_noise_enabled(enabled)
        request = configured.advance(sim_time_ns=index * 100_000_000, reset_generation=0).samples[0]
        original = plain(request, {})
        measured = controlled(request, {})
        assert measured.range_noise.enabled is enabled
        assert (encode_mid360_frame_sample(original) == encode_mid360_frame_sample(measured)) is not enabled

    zero = SensorRuntime.from_plan(_plan({"enabled": True, "stddev_m": 0, "seed": 17}))
    request = zero.advance(sim_time_ns=0, reset_generation=0).samples[0]
    sample = _extractor(tmp_path, zero, RaycastProcess())(request, {})
    assert encode_mid360_frame_sample(sample) == encode_mid360_frame_sample(
        _extractor(tmp_path, baseline, RaycastProcess())(request, {}),
    )
    with pytest.raises(ValueError, match="boolean"):
        controlled.set_noise_enabled("yes")
    assert controlled.range_noise.enabled is False


def test_frame_noise_is_repeatable_and_independent_of_dropped_frames(tmp_path: Path) -> None:
    runtime = SensorRuntime.from_plan(_plan({"enabled": True, "stddev_m": 0.02, "seed": 17}))
    continuous = _extractor(tmp_path, runtime, RaycastProcess())
    dropped = _extractor(tmp_path, runtime, RaycastProcess())
    initial = runtime.advance(sim_time_ns=0, model_generation=3, reset_generation=0).samples[0]
    assert continuous(initial, {}) == dropped(initial, {})
    second = runtime.advance(sim_time_ns=100_000_000, model_generation=3, reset_generation=0).samples[0]
    continuous(second, {})
    third = runtime.advance(sim_time_ns=200_000_000, model_generation=3, reset_generation=0).samples[0]
    expected = continuous(third, {})
    assert dropped(third, {}) == expected
    assert _extractor(tmp_path, runtime, RaycastProcess())(third, {}) == expected
    other = _extractor(tmp_path, runtime, RaycastProcess())
    other.range_noise = replace(other.range_noise, seed=18)
    assert other(third, {}).points != expected.points


def test_noise_separates_streams_and_generations_and_preserves_direction(tmp_path: Path) -> None:
    runtime = SensorRuntime.from_plan(_plan({"enabled": True, "stddev_m": 0.02, "seed": 17}))
    request = runtime.advance(sim_time_ns=0, model_generation=3, reset_generation=0).samples[0]
    original = _extractor(tmp_path, runtime, RaycastProcess())(request, {})
    for changed in (
        replace(request, stream=replace(request.stream, sensor_id="robot_02.mid360", instance_id="robot_02")),
        replace(request, reset_generation=1),
        replace(request, model_generation=4),
    ):
        sample = _extractor(tmp_path, runtime, RaycastProcess())(changed, {})
        assert sample.points != original.points
        assert sample.points[0].x / sample.points[0].y == pytest.approx(3 / 4)
        assert sample.points[0].z == 0
        assert sample.points[1].x == sample.points[1].y == 0
        assert sample.points[1].z < 0


def test_extreme_noise_stays_inside_range_limits_without_manufacturing_hits(tmp_path: Path) -> None:
    runtime = SensorRuntime.from_plan(_plan({"enabled": True, "stddev_m": 1e6, "seed": 17}))
    extractor = _extractor(tmp_path, runtime, RaycastProcess())
    request = runtime.advance(sim_time_ns=0, reset_generation=0).samples[0]
    sample = extractor(request, {})
    assert len(sample.points) == 2
    for point in sample.points:
        assert extractor.range_min_m <= math.hypot(point.x, point.y, point.z) <= extractor.range_max_m


@pytest.mark.parametrize("noise", [
    {}, None, {"enabled": True},
    {"enabled": "true", "stddev_m": 0.02, "seed": 17},
    {"enabled": True, "stddev_m": True, "seed": 17},
    {"enabled": True, "stddev_m": -1, "seed": 17},
    {"enabled": True, "stddev_m": float("nan"), "seed": 17},
    {"enabled": True, "stddev_m": float("inf"), "seed": 17},
    {"enabled": True, "stddev_m": 10**1000, "seed": 17},
    {"enabled": True, "stddev_m": 0.02, "seed": True},
    {"enabled": True, "stddev_m": 0.02, "seed": -1},
    {"enabled": True, "stddev_m": 0.02, "seed": 2**64},
    {"enabled": True, "stddev_m": 0.02, "seed": 17, "stddev": 3},
])
def test_invalid_noise_is_not_silently_ignored(noise: Any) -> None:
    plan = _plan()
    plan["streams"]["mid360"][0]["range_noise"] = noise
    with pytest.raises(ValueError, match="range_noise"):
        SensorRuntime.from_plan(plan)


def test_noise_cannot_be_attached_to_truth_odometry() -> None:
    plan = _plan({"enabled": True, "stddev_m": 0.02, "seed": 17})
    stream = plan["streams"].pop("mid360")[0]
    del stream["raycast_frame_stable_id"]
    plan["streams"]["truth_odom"] = [stream]
    with pytest.raises(ValueError, match="only supported for mid360"):
        SensorRuntime.from_plan(plan)


def test_default_noise_config_is_explicitly_disabled() -> None:
    assert RangeNoiseConfig().to_dict() == {"enabled": False, "stddev_m": 0.0, "seed": 0}


def test_noise_example_compiles_rig_parameters_without_changing_default_navigation(tmp_path: Path) -> None:
    repo = Path(__file__).resolve().parents[2]
    resolver = CatalogResolver.from_repository(repo)
    example = repo / "sim/sessions/examples/thunderv4_mid360_noise/session.yaml"
    resolved = resolver.resolve(example)
    assert resolved.sensor_plan["streams"]["mid360"][0]["range_noise"] == {
        "enabled": True, "stddev_m": 0.02, "seed": 17,
    }
    runtime = SensorRuntime.from_plan(resolved.sensor_plan)
    assert runtime.streams[0].range_noise == RangeNoiseConfig(True, 0.02, 17)
    assert resolved.sensor_json == resolver.resolve(example).sensor_json
    resolved.write_bundle(tmp_path / "bundle")
    bundle = load_resolved_session_bundle(tmp_path / "bundle", repo_root=repo)
    reloaded = SensorRuntime.from_plan(bundle.plans["sensor.plan.json"])
    assert reloaded.streams[0].range_noise == runtime.streams[0].range_noise

    ordinary = resolver.resolve(repo / "sim/sessions/examples/thunderv4_unreal/session.yaml")
    mid360 = next(stream for stream in SensorRuntime.from_plan(ordinary.sensor_plan).streams
                  if stream.stream_kind == "mid360")
    assert mid360.range_noise == RangeNoiseConfig()
