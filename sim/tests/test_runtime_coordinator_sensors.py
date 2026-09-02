# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.control.fake import zero_output_components
from sim.runtime.coordinator import CoordinatorError, RuntimeCoordinator, RuntimeState
from sim.runtime.coordinator.external_evidence import ExternalEvidenceWatcher
from sim.runtime.sensors import (
    ImuSample,
    LivoxPointSample,
    Mid360FrameSample,
    SensorSampleStamp,
)
from sim.runtime.sensors.session import SensorEndpoint

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "sim" / "sessions" / "examples" / "thunderv4_unreal" / "session.yaml"
ACTUATORS = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "FR_foot_joint",
    "FL_foot_joint",
    "RR_foot_joint",
    "RL_foot_joint",
)


class PhysicsHost:
    def __init__(self) -> None:
        self.pid: int | None = 8102
        self.session_id = ""
        self.reset_generation = 0
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 0

    def prepare(self, plan: Any, allocation: Any) -> dict[str, Any]:
        del allocation
        self.session_id = plan.session_id
        return self._event("ready")

    def bind_actuators(self, **binding: Any) -> dict[str, Any]:
        return {"event": "actuator_bound", **binding}

    def apply_actuator_command(self, command: Any) -> dict[str, Any]:
        return {
            "event": "actuator_command",
            "source_id": command.controller_id,
            "sequence": command.sequence,
            "result": "applied",
        }

    def start(self) -> dict[str, Any]:
        return self._event("running")

    def advance(self, steps: int) -> dict[str, Any]:
        assert steps == 1
        self.sequence += 1
        self.physics_step += 1
        self.sim_time_ns += 2_000_000
        return self._snapshot()

    def snapshot(self) -> dict[str, Any]:
        return self._snapshot()

    def apply_kinematic_poses(self, snapshot: Any) -> dict[str, Any]:
        del snapshot
        return self._snapshot()

    def pause(self) -> dict[str, Any]:
        return self._event("paused")

    def reset(self) -> dict[str, Any]:
        self.reset_generation += 1
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 0
        return self._snapshot()

    def stop(self) -> dict[str, Any]:
        self.pid = None
        return self._event("stopped")

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "session_id": self.session_id,
            "model_generation": 0,
            "reset_generation": self.reset_generation,
            "sequence": self.sequence,
            "physics_step": self.physics_step,
            "sim_time_ns": self.sim_time_ns,
        }

    def _snapshot(self) -> dict[str, Any]:
        return {
            **self._event("snapshot"),
            "bodies": [
                {
                    "stable_id": "thunder_01/base_link",
                    "instance_id": "thunder_01",
                    "frame_id": "base_link",
                    "position_m": [0.0, 0.0, 0.45],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "linear_velocity_mps": [0.0, 0.0, 0.0],
                    "angular_velocity_rps": [0.0, 0.0, 0.0],
                }
            ],
            "joints": [
                {
                    "stable_id": f"thunder_01/{channel}",
                    "instance_id": "thunder_01",
                    "position_rad": [0.0],
                    "velocity_rps": [0.0],
                }
                for channel in ACTUATORS
            ],
            "actuators": [],
        }


class RecordingSink:
    def __init__(self) -> None:
        self.samples: list[Any] = []
        self.closed = 0

    def start(self) -> dict[str, Any]:
        return {"ready": True}

    def publish(self, sample: Any) -> None:
        self.samples.append(sample)

    def close(self) -> None:
        self.closed += 1


def _bundle(tmp_path: Path) -> Path:
    return CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(tmp_path / "bundle")


def _activate_visual(coordinator: RuntimeCoordinator, event: dict[str, Any]) -> None:
    coordinator.report_binding_prepared(
        "visual",
        source_id="robotsimue/test",
        session_id=event["session_id"],
        model_generation=event["model_generation"],
        reset_generation=event["reset_generation"],
    )
    coordinator.report_binding_active(
        "visual",
        source_id="robotsimue/test",
        session_id=event["session_id"],
        model_generation=event["model_generation"],
        reset_generation=event["reset_generation"],
    )


def _activate_external_sensor(
    coordinator: RuntimeCoordinator,
    event: dict[str, Any],
    sensor_id: str,
    source_id: str,
) -> None:
    coordinator.report_sensor_stream_prepared(
        sensor_id,
        source_id=source_id,
        session_id=event["session_id"],
        model_generation=event["model_generation"],
        reset_generation=event["reset_generation"],
    )
    coordinator.report_sensor_stream_active(
        sensor_id,
        source_id=source_id,
        session_id=event["session_id"],
        model_generation=event["model_generation"],
        reset_generation=event["reset_generation"],
    )


def _native_sensor_endpoint(stream: Any, allocation: Any) -> SensorEndpoint | None:
    del allocation
    if stream.stream_kind not in {"imu", "mid360", "truth_odom"}:
        return None
    sink = RecordingSink()
    if stream.stream_kind == "truth_odom":
        return SensorEndpoint.truth_odometry(
            source_id="truth-odom-dds/test",
            sink=sink,
        )
    if stream.stream_kind == "imu":

        def imu_sample(scheduled: Any, snapshot: Any) -> ImuSample:
            del snapshot
            return ImuSample(
                stamp=SensorSampleStamp.from_scheduled(scheduled),
                orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
                angular_velocity_rps=(0.0, 0.0, 0.0),
                linear_acceleration_mps2=(0.0, 0.0, 9.81),
            )

        return SensorEndpoint(
            source_id="mujoco-imu-dds/test",
            sink=sink,
            extractor=imu_sample,
        )

    def mid360_frame(scheduled: Any, snapshot: Any) -> Mid360FrameSample:
        del snapshot
        return Mid360FrameSample(
            stamp=SensorSampleStamp.from_scheduled(scheduled),
            points=(
                LivoxPointSample(
                    x=1.0,
                    y=0.0,
                    z=0.0,
                    reflectivity=15,
                    offset_time_ns=0,
                ),
            ),
            scan_time_profile="instantaneous_geometry/scheduled_offsets",
        )

    return SensorEndpoint(
        source_id="mujoco-mid360-dds/test",
        sink=sink,
        extractor=mid360_frame,
    )


def _write_camera_evidence(
    path: Path,
    event: dict[str, Any],
    *,
    depth_frames: int,
    rgb_frames: int,
    last_sample_truth_sequence: int | None = None,
    last_sample_sim_time_ns: int | None = None,
) -> None:
    truth_sequence = (
        event["sequence"]
        if last_sample_truth_sequence is None
        else last_sample_truth_sequence
    )
    sim_time_ns = (
        event["sim_time_ns"]
        if last_sample_sim_time_ns is None
        else last_sample_sim_time_ns
    )
    path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.sensor-readiness-evidence.v1",
                "session_id": event["session_id"],
                "model_generation": event["model_generation"],
                "reset_generation": event["reset_generation"],
                "source_id": "robotsimue-camera",
                "basis": "real_rendered_frame_to_camera_shm",
                "visual": {"state": "PREPARING"},
                "sensors": {"camera_streams": "ACTIVE", "overall": "ACTIVE"},
                "streams": [
                    {
                        "sensor_id": "thunder_01.front_depth",
                        "state": "ACTIVE",
                        "published_frames": depth_frames,
                        "last_sample_truth_sequence": truth_sequence,
                        "last_sample_sim_time_ns": sim_time_ns,
                    },
                    {
                        "sensor_id": "thunder_01.front_rgb",
                        "state": "ACTIVE",
                        "published_frames": rgb_frames,
                        "last_sample_truth_sequence": truth_sequence,
                        "last_sample_sim_time_ns": sim_time_ns,
                    },
                ],
            }
        ),
        encoding="utf-8",
    )


def _camera_watcher(
    path: Path,
    event: dict[str, Any],
) -> ExternalEvidenceWatcher:
    return ExternalEvidenceWatcher(
        path,
        session_id=event["session_id"],
        model_generation=event["model_generation"],
        reset_generation=event["reset_generation"],
        expected_source_id="robotsimue-camera",
    )


def test_one_active_stream_does_not_activate_the_sensor_facet(tmp_path: Path) -> None:
    sink = RecordingSink()

    def endpoint_factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "truth_odom":
            return None
        return SensorEndpoint.truth_odometry(source_id="truth-dds/test", sink=sink)

    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=PhysicsHost(),
        controller_factory=zero_output_components,
        sensor_endpoint_factory=endpoint_factory,
        run_id="sensor-stream-readiness",
    )
    ready = coordinator.prepare()

    assert coordinator.state is RuntimeState.PREPARING
    assert set(coordinator.allocation.shm) == {
        "thunder_01.front_depth",
        "thunder_01.front_rgb",
    }
    assert all(coordinator.allocation.run_id in name for name in coordinator.allocation.shm.values())
    assert coordinator.sensor_readiness.state("thunder_01.truth_odom").value == "ACTIVE"
    assert coordinator.sensor_readiness.state("thunder_01.imu").value == "UNBOUND"
    assert coordinator.readiness.state("sensors").value != "ACTIVE"
    assert [(sample.stamp.sequence, sample.stamp.sim_time_ns) for sample in sink.samples] == [(0, 0)]

    _activate_visual(coordinator, ready)
    for sensor_id, source_id in (
        ("thunder_01.front_depth", "robotsimue/depth"),
        ("thunder_01.front_rgb", "robotsimue/rgb"),
        ("thunder_01.imu", "mujoco/imu"),
        ("thunder_01.mid360", "mujoco/mid360"),
    ):
        _activate_external_sensor(coordinator, ready, sensor_id, source_id)

    assert coordinator.sensor_readiness.is_ready
    assert coordinator.readiness.state("sensors").value == "ACTIVE"
    assert coordinator.state is RuntimeState.READY

    coordinator.start()
    coordinator.advance(5)
    assert [(sample.stamp.sequence, sample.stamp.sim_time_ns) for sample in sink.samples] == [
        (0, 0),
        (1, 10_000_000),
    ]
    coordinator.pause()
    reset = coordinator.reset()
    assert reset["reset_generation"] == 1
    assert coordinator.sensor_readiness.streams["thunder_01.truth_odom"].reset_generation == 1
    assert coordinator.sensor_readiness.state("thunder_01.truth_odom").value == "ACTIVE"
    assert coordinator.sensor_readiness.state("thunder_01.front_rgb").value == "PREPARED"
    assert coordinator.sensor_readiness.state("thunder_01.front_depth").value == "PREPARED"
    assert coordinator.sensor_readiness.is_ready is False
    assert coordinator.readiness.state("sensors").value != "ACTIVE"
    assert coordinator.state is RuntimeState.PREPARING
    assert [(sample.stamp.reset_generation, sample.stamp.sequence) for sample in sink.samples] == [
        (0, 0),
        (0, 1),
        (1, 0),
    ]
    coordinator.stop()
    assert sink.closed == 1

    manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
    assert manifest["sensor_streams"]["is_ready"] is False
    assert manifest["sensor_streams"]["streams"]["thunder_01.truth_odom"]["state"] == "ACTIVE"
    assert manifest["sensor_streams"]["streams"]["thunder_01.front_depth"]["state"] == "PREPARED"
    assert manifest["sensor_streams"]["streams"]["thunder_01.front_rgb"]["state"] == "PREPARED"
    assert manifest["bindings"]["sensors"]["state"] == "PREPARED"


def test_manifest_commits_same_generation_five_stream_sample_summary(
    tmp_path: Path,
) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=PhysicsHost(),
        controller_factory=zero_output_components,
        sensor_endpoint_factory=_native_sensor_endpoint,
        run_id="five-stream-summary",
    )
    ready = coordinator.prepare()
    evidence_path = tmp_path / "camera-evidence.json"
    _write_camera_evidence(
        evidence_path,
        ready,
        depth_frames=2,
        rgb_frames=3,
    )
    watcher = _camera_watcher(evidence_path, ready)

    assert watcher.apply(coordinator)
    manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
    summary = manifest["sensor_streams"]["summary"]

    assert summary["schema"] == "lingtu.sim.sensor-stream-summary.v1"
    assert summary["session_id"] == manifest["session_id"]
    assert summary["model_generation"] == manifest["model_generation"]
    assert summary["reset_generation"] == manifest["reset_generation"]
    assert summary["is_ready"] is True
    assert summary["required_stream_ids"] == sorted(
        {
            "thunder_01.front_depth",
            "thunder_01.front_rgb",
            "thunder_01.imu",
            "thunder_01.mid360",
            "thunder_01.truth_odom",
        }
    )
    assert {
        stream_id: stream["sample_count"]
        for stream_id, stream in summary["streams"].items()
    } == {
        "thunder_01.front_depth": 2,
        "thunder_01.front_rgb": 3,
        "thunder_01.imu": 1,
        "thunder_01.mid360": 1,
        "thunder_01.truth_odom": 1,
    }
    assert summary["streams"]["thunder_01.front_depth"]["shm_name"] == (
        coordinator.allocation.shm["thunder_01.front_depth"]
    )
    assert summary["streams"]["thunder_01.front_rgb"]["shm_name"] == (
        coordinator.allocation.shm["thunder_01.front_rgb"]
    )
    assert summary["streams"]["thunder_01.front_depth"][
        "last_sample_truth_sequence"
    ] == ready["sequence"]
    assert summary["streams"]["thunder_01.front_depth"][
        "last_sample_sim_time_ns"
    ] == ready["sim_time_ns"]

    _write_camera_evidence(
        evidence_path,
        ready,
        depth_frames=4,
        rgb_frames=6,
        last_sample_truth_sequence=ready["sequence"] + 1,
        last_sample_sim_time_ns=ready["sim_time_ns"] + 10_000_000,
    )
    assert watcher.apply(coordinator)
    updated = json.loads(
        coordinator.manifest_path.read_text(encoding="utf-8")
    )["sensor_streams"]["summary"]
    assert updated["streams"]["thunder_01.front_depth"]["sample_count"] == 4
    assert updated["streams"]["thunder_01.front_rgb"]["sample_count"] == 6
    assert updated["streams"]["thunder_01.front_rgb"][
        "last_sample_truth_sequence"
    ] == ready["sequence"] + 1
    assert updated["streams"]["thunder_01.front_rgb"][
        "last_sample_sim_time_ns"
    ] == ready["sim_time_ns"] + 10_000_000


def test_manifest_keeps_missing_ue_camera_frames_at_zero_and_not_ready(
    tmp_path: Path,
) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=PhysicsHost(),
        controller_factory=zero_output_components,
        sensor_endpoint_factory=_native_sensor_endpoint,
        run_id="missing-camera-summary",
    )

    coordinator.prepare()
    manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
    summary = manifest["sensor_streams"]["summary"]

    assert manifest["sensor_streams"]["required_stream_ids"]
    assert summary["is_ready"] is False
    assert summary["blocking_reasons"] == {
        "thunder_01.front_depth": "stream evidence is MISSING",
        "thunder_01.front_rgb": "stream evidence is MISSING",
    }
    for sensor_id in ("thunder_01.front_depth", "thunder_01.front_rgb"):
        assert summary["streams"][sensor_id]["state"] == "MISSING"
        assert summary["streams"][sensor_id]["sample_count"] == 0
        assert summary["streams"][sensor_id]["runtime_source_id"] is None
        assert summary["streams"][sensor_id]["binding_identity"] is None


def test_reset_manifest_does_not_carry_camera_frames_across_generations(
    tmp_path: Path,
) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=PhysicsHost(),
        controller_factory=zero_output_components,
        sensor_endpoint_factory=_native_sensor_endpoint,
        run_id="reset-five-stream-summary",
    )
    ready = coordinator.prepare()
    evidence_path = tmp_path / "camera-evidence.json"
    _write_camera_evidence(
        evidence_path,
        ready,
        depth_frames=4,
        rgb_frames=5,
    )
    watcher = _camera_watcher(evidence_path, ready)
    assert watcher.apply(coordinator)
    _activate_visual(coordinator, ready)

    reset = coordinator.reset()
    manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
    summary = manifest["sensor_streams"]["summary"]

    assert reset["reset_generation"] == 1
    assert coordinator.state is RuntimeState.PREPARING
    assert coordinator.readiness.state("sensors").value == "PREPARED"
    assert coordinator.sensor_readiness.is_ready is False
    assert coordinator.sensor_readiness.state("thunder_01.front_depth").value == "PREPARED"
    assert coordinator.sensor_readiness.state("thunder_01.front_rgb").value == "PREPARED"
    assert manifest["state"] == "PREPARING"
    assert manifest["bindings"]["sensors"]["state"] == "PREPARED"
    assert manifest["sensor_streams"]["is_ready"] is False
    assert manifest["sensor_streams"]["streams"]["thunder_01.front_depth"]["state"] == "PREPARED"
    assert manifest["sensor_streams"]["streams"]["thunder_01.front_rgb"]["state"] == "PREPARED"
    assert summary["model_generation"] == manifest["model_generation"] == 0
    assert summary["reset_generation"] == manifest["reset_generation"] == 1
    assert summary["is_ready"] is False
    for sensor_id in ("thunder_01.front_depth", "thunder_01.front_rgb"):
        assert summary["streams"][sensor_id]["state"] == "MISSING"
        assert summary["streams"][sensor_id]["sample_count"] == 0
        assert summary["streams"][sensor_id]["runtime_source_id"] is None
        assert summary["streams"][sensor_id]["binding_identity"] is None
    for sensor_id in (
        "thunder_01.imu",
        "thunder_01.mid360",
        "thunder_01.truth_odom",
    ):
        assert summary["streams"][sensor_id]["sample_count"] == 1

    watcher.advance_generation(model_generation=0, reset_generation=1)
    _write_camera_evidence(
        evidence_path,
        reset,
        depth_frames=1,
        rgb_frames=2,
    )
    assert watcher.apply(coordinator)
    refreshed = json.loads(
        coordinator.manifest_path.read_text(encoding="utf-8")
    )["sensor_streams"]["summary"]
    assert refreshed["reset_generation"] == 1
    assert refreshed["is_ready"] is True
    assert refreshed["streams"]["thunder_01.front_depth"]["sample_count"] == 1
    assert refreshed["streams"]["thunder_01.front_rgb"]["sample_count"] == 2


def test_whole_sensor_facet_cannot_bypass_stream_qualification(tmp_path: Path) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=PhysicsHost(),
        controller_factory=zero_output_components,
        run_id="no-facet-bypass",
    )
    ready = coordinator.prepare()

    with pytest.raises(CoordinatorError, match="per-stream"):
        coordinator.report_binding_prepared(
            "sensors",
            source_id="sensor-runtime/test",
            session_id=ready["session_id"],
            model_generation=ready["model_generation"],
            reset_generation=ready["reset_generation"],
        )


def test_runtime_sensor_failure_marks_the_stream_and_aborts(tmp_path: Path) -> None:
    class FailingSink(RecordingSink):
        def publish(self, sample: Any) -> None:
            if self.samples:
                raise RuntimeError("DDS writer closed")
            super().publish(sample)

    sink = FailingSink()

    def endpoint_factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "truth_odom":
            return None
        return SensorEndpoint.truth_odometry(source_id="truth-dds/test", sink=sink)

    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=PhysicsHost(),
        controller_factory=zero_output_components,
        sensor_endpoint_factory=endpoint_factory,
        run_id="sensor-stream-failure",
    )
    ready = coordinator.prepare()
    _activate_visual(coordinator, ready)
    for sensor_id in (
        "thunder_01.front_depth",
        "thunder_01.front_rgb",
        "thunder_01.imu",
        "thunder_01.mid360",
    ):
        _activate_external_sensor(
            coordinator,
            ready,
            sensor_id,
            f"external/{sensor_id}",
        )

    coordinator.start()
    with pytest.raises(CoordinatorError, match="DDS writer closed"):
        coordinator.advance(5)

    assert coordinator.state is RuntimeState.FAILED
    assert coordinator.sensor_readiness.state("thunder_01.truth_odom").value == "FAILED"
    assert coordinator.readiness.state("sensors").value == "FAILED"
    assert sink.closed == 1


def test_warmup_sensor_failure_keeps_failed_terminal_and_episode(
    tmp_path: Path,
) -> None:
    class FailingSink(RecordingSink):
        def publish(self, sample: Any) -> None:
            if self.samples:
                raise RuntimeError("DDS writer closed during warmup")
            super().publish(sample)

    class DestroyedOnStopPhysicsHost(PhysicsHost):
        def __init__(self) -> None:
            super().__init__()
            self.stop_calls = 0

        def stop(self) -> dict[str, Any]:
            self.stop_calls += 1
            if self.stop_calls > 1:
                self.session_id = ""
            return super().stop()

    sink = FailingSink()
    physics = DestroyedOnStopPhysicsHost()

    def endpoint_factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "truth_odom":
            return None
        return SensorEndpoint.truth_odometry(source_id="truth-dds/warmup", sink=sink)

    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=physics,
        controller_factory=zero_output_components,
        sensor_endpoint_factory=endpoint_factory,
        run_id="sensor-warmup-failure",
    )
    coordinator.prepare()

    with pytest.raises(CoordinatorError, match="DDS writer closed during warmup") as error:
        coordinator.warmup(5)

    assert "session_id does not match" not in str(error.value)
    assert coordinator.state is RuntimeState.FAILED
    manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
    assert manifest["state"] == "FAILED"

    first_episode = (coordinator.allocation.run_dir / "episode_result.json").read_bytes()
    episode = json.loads(first_episode)
    assert episode["status"] == "FAILED"
    assert "DDS writer closed during warmup" in episode["failure_reason"]
    assert episode["model_generation"] == 0
    assert episode["reset_generation"] == 0
    assert episode["start_sim_time_ns"] == 0
    assert episode["end_sim_time_ns"] == 10_000_000

    stopped = coordinator.stop(
        failure_reason="secondary cleanup failure",
    )
    assert stopped["event"] == "snapshot"
    assert physics.stop_calls == 1
    assert (coordinator.allocation.run_dir / "episode_result.json").read_bytes() == first_episode
