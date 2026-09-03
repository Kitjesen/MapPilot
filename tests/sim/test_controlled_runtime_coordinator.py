# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.control import (
    CommandSubmitResult,
    ControllerCommand,
    DeterministicFakeAdapter,
    DeterministicFakePolicy,
    GenerationStamp,
)
from sim.runtime.coordinator import CoordinatorError, RuntimeCoordinator, RuntimeState
from sim.runtime.sensors import (
    ImuSample,
    LivoxPointSample,
    Mid360FrameSample,
    SensorEndpoint,
    SensorSampleStamp,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "sim" / "sessions" / "examples" / "thunderv4_controlled_headless" / "session.yaml"
UNREAL_SESSION = REPO_ROOT / "sim" / "sessions" / "examples" / "thunderv4_unreal" / "session.yaml"
G007_SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunderv4_open_field_pedestrian_unreal"
    / "session.yaml"
)
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


class ControlledPhysicsHost:
    def __init__(self) -> None:
        self.pid: int | None = 7001
        self.session_id = ""
        self.model_generation = 0
        self.reset_generation = 0
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 0
        self.calls: list[Any] = []
        self.applied: list[Any] = []
        self.actuator_result = "applied"
        self.kinematic_position_m = [4.0, -6.0, 0.0]
        self.kinematic_quaternion_wxyz = [1.0, 0.0, 0.0, 0.0]

    def prepare(self, plan: Any, allocation: Any) -> dict[str, Any]:
        del allocation
        self.calls.append("prepare")
        self.session_id = plan.session_id
        self.model_generation = plan.model_generation
        return self._event("ready")

    def bind_actuators(self, **binding: Any) -> dict[str, Any]:
        self.calls.append(("bind", binding))
        return {
            "event": "actuator_bound",
            **binding,
            "channel_count": len(binding["channels"]),
        }

    def start(self) -> dict[str, Any]:
        self.calls.append("start")
        return self._event("running")

    def snapshot(self) -> dict[str, Any]:
        self.calls.append("snapshot")
        return self._snapshot()

    def apply_actuator_command(self, command: Any) -> dict[str, Any]:
        self.calls.append(("actuate", command.sequence))
        self.applied.append(command)
        return {
            "event": "actuator_command",
            "source_id": command.controller_id,
            "sequence": command.sequence,
            "result": self.actuator_result,
        }

    def apply_kinematic_poses(self, snapshot: Any) -> dict[str, Any]:
        self.calls.append(("kinematic", snapshot.sequence))
        entity = snapshot.entities[0]
        self.kinematic_position_m = list(entity.transform.position_m)
        self.kinematic_quaternion_wxyz = list(entity.transform.quaternion_wxyz)
        return {
            "event": "kinematic_poses",
            "result": "applied",
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
            "entity_count": len(snapshot.entities),
        }

    def raycast(self, **kwargs: Any) -> dict[str, Any]:
        self.calls.append(("raycast", kwargs))
        return {
            "event": "raycast",
            "sensor_frame_id": kwargs["sensor_frame_id"],
            "session_id": kwargs["session_id"],
            "model_generation": kwargs["model_generation"],
            "reset_generation": kwargs["reset_generation"],
            "sequence": kwargs["sequence"],
            "sim_time_ns": kwargs["sim_time_ns"],
            "hit_count": 1,
            "hits": [
                {
                    "entity_id": "pedestrian_01",
                    "body_stable_id": "pedestrian_01/proxy_root",
                    "xyz_sensor": [4.0, -6.0, 0.0],
                    "origin_world_m": [0.0, 0.0, 0.0],
                    "direction_world": [0.5547001962252291, -0.8320502943378437, 0.0],
                    "position_world_m": list(self.kinematic_position_m),
                    "distance_m": 7.211102550927978,
                    "offset_time_ns": 0,
                    "reflectivity": 15,
                    "tag": 0,
                    "line": 0,
                }
            ],
        }

    def advance(self, steps: int) -> dict[str, Any]:
        self.calls.append(("advance", steps))
        assert steps == 1
        self.physics_step += 1
        self.sequence += 1
        self.sim_time_ns += 2_000_000
        return self._snapshot()

    def pause(self) -> dict[str, Any]:
        self.calls.append("pause")
        return self._event("paused")

    def reset(self) -> dict[str, Any]:
        self.calls.append("reset")
        self.reset_generation += 1
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 0
        return self._snapshot()

    def stop(self) -> dict[str, Any]:
        self.calls.append("stop")
        self.pid = None
        return self._event("stopped")

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "session_id": self.session_id,
            "model_generation": self.model_generation,
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
                    "position_m": [0.0, 0.0, 0.4],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "linear_velocity_mps": [0.0, 0.0, 0.0],
                    "angular_velocity_rps": [0.0, 0.0, 0.0],
                },
                {
                    "stable_id": "pedestrian_01/proxy_root",
                    "instance_id": "pedestrian_01",
                    "frame_id": "proxy_root",
                    "position_m": list(self.kinematic_position_m),
                    "quaternion_wxyz": list(self.kinematic_quaternion_wxyz),
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
            "sensors": [
                {
                    "source_stable_id": "thunder_01/imu",
                    "sensor_type": "framequat",
                    "values": [1.0, 0.0, 0.0, 0.0],
                },
                {
                    "source_stable_id": "thunder_01/imu",
                    "sensor_type": "gyro",
                    "values": [0.0, 0.0, 0.0],
                },
                {
                    "source_stable_id": "thunder_01/imu",
                    "sensor_type": "accelerometer",
                    "values": [0.0, 0.0, 9.80665],
                },
            ],
            "actuators": [],
        }


class ChunkedControlledPhysicsHost(ControlledPhysicsHost):
    def advance(self, steps: int) -> dict[str, Any]:
        self.calls.append(("advance", steps))
        self.physics_step += steps
        self.sequence += steps
        self.sim_time_ns += steps * 2_000_000
        return self._snapshot()


class SampledControlledPhysicsHost(ControlledPhysicsHost):
    def advance_sampled(self, steps: int) -> tuple[dict[str, Any], ...]:
        self.calls.append(("advance_sampled", steps))
        snapshots: list[dict[str, Any]] = []
        for index in range(steps):
            self.physics_step += 1
            self.sequence += 1
            self.sim_time_ns += 1_000_000
            if self.physics_step % 5 == 0 or index == steps - 1:
                snapshots.append(self._snapshot())
        return tuple(snapshots)


class IncompleteSampledControlledPhysicsHost(SampledControlledPhysicsHost):
    def advance_sampled(self, steps: int) -> tuple[dict[str, Any], ...]:
        snapshots = super().advance_sampled(steps)
        return snapshots[:-1]


class FusedSampledControlledPhysicsHost(SampledControlledPhysicsHost):
    def advance_sampled_with_actuator(
        self,
        command: Any,
        steps: int,
    ) -> tuple[dict[str, Any], ...]:
        self.calls.append(("actuate_advance_sampled", command.sequence, steps))
        self.applied.append(command)
        snapshots: list[dict[str, Any]] = []
        for index in range(steps):
            self.physics_step += 1
            self.sequence += 1
            self.sim_time_ns += 1_000_000
            if self.physics_step % 5 == 0 or index == steps - 1:
                snapshots.append(self._snapshot())
        return tuple(snapshots)


def _bundle(tmp_path: Path, session: Path = SESSION) -> Path:
    return Path(
        CatalogResolver.from_repository(REPO_ROOT)
        .resolve(session)
        .write_bundle(tmp_path / "bundle")
    )


def _fake_components(controller: Any, repo_root: Path) -> tuple[Any, Any]:
    del repo_root
    return (
        DeterministicFakeAdapter(),
        DeterministicFakePolicy((0.1,) * len(controller.actuators.channels)),
    )


class _TruthSink:
    def start(self) -> dict[str, bool]:
        return {"ready": True}

    def publish(self, sample: Any) -> None:
        del sample

    def close(self) -> None:
        return None


def _mid360_frame(scheduled: Any, snapshot: Any) -> Mid360FrameSample:
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


def _imu_sample(scheduled: Any, snapshot: Any) -> ImuSample:
    del snapshot
    return ImuSample(
        stamp=SensorSampleStamp.from_scheduled(scheduled),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        angular_velocity_rps=(0.0, 0.0, 0.0),
        linear_acceleration_mps2=(0.0, 0.0, 9.80665),
    )


def _camera_sample(scheduled: Any, snapshot: Any) -> SimpleNamespace:
    del snapshot
    return SimpleNamespace(stamp=SensorSampleStamp.from_scheduled(scheduled))


def _headless_sensor_endpoint(stream: Any, allocation: Any) -> SensorEndpoint | None:
    del allocation
    if stream.stream_kind == "truth_odom":
        return SensorEndpoint.truth_odometry(
            source_id="truth-odom-test",
            sink=_TruthSink(),
        )
    if stream.stream_kind == "imu":
        return SensorEndpoint(
            source_id="imu-test",
            sink=_TruthSink(),
            extractor=_imu_sample,
        )
    if stream.stream_kind == "mid360":
        return SensorEndpoint(
            source_id="mid360-test",
            sink=_TruthSink(),
            extractor=_mid360_frame,
        )
    if stream.stream_kind in {"rgb", "depth"}:
        return SensorEndpoint(
            source_id="camera-test",
            sink=_TruthSink(),
            extractor=_camera_sample,
        )
    if stream.stream_kind not in {"truth_odom", "imu", "mid360"}:
        return None
    return None


def _strict_headless_sensor_endpoint(
    stream: Any,
    allocation: Any,
) -> SensorEndpoint | None:
    del allocation
    if stream.stream_kind == "truth_odom":
        return SensorEndpoint.truth_odometry(
            source_id="truth-odom-test",
            sink=_TruthSink(),
        )
    if stream.stream_kind == "imu":
        return SensorEndpoint.imu(
            source_id="imu-test",
            sink=_TruthSink(),
        )
    if stream.stream_kind == "mid360":
        return SensorEndpoint(
            source_id="mid360-test",
            sink=_TruthSink(),
            extractor=_mid360_frame,
        )
    if stream.stream_kind in {"rgb", "depth"}:
        return SensorEndpoint(
            source_id="camera-test",
            sink=_TruthSink(),
            extractor=_camera_sample,
        )
    return None


def test_coordinator_refuses_to_ignore_a_compiled_controller(tmp_path: Path) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=ControlledPhysicsHost(),
        run_id="missing-controller-factory",
    )

    with pytest.raises(CoordinatorError, match="controller_factory"):
        coordinator.prepare()


def test_coordinator_can_bind_allocation_artifacts_to_its_owned_run_directory(
    tmp_path: Path,
) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=ControlledPhysicsHost(),
        controller_factory=_fake_components,
        sensor_endpoint_factory=_headless_sensor_endpoint,
        run_id="controlled-owned-artifacts",
        artifact_root_mode="run",
    )

    coordinator.prepare()

    assert coordinator.allocation.artifact_root == coordinator.allocation.run_dir
    persisted = json.loads(coordinator.allocation.path.read_text(encoding="utf-8"))
    assert persisted["artifact_root"] == str(coordinator.allocation.run_dir)


def test_coordinator_holds_output_between_low_level_control_ticks(
    tmp_path: Path,
) -> None:
    host = ControlledPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        controller_factory=_fake_components,
        sensor_endpoint_factory=_headless_sensor_endpoint,
        run_id="controlled-session",
    )
    ready = coordinator.prepare()
    assert coordinator.state is RuntimeState.READY
    assert host.calls[1][0] == "bind"
    assert host.calls[1][1]["channels"] == ACTUATORS

    coordinator.start()
    generation = GenerationStamp(ready["model_generation"], ready["reset_generation"])
    assert (
        coordinator.submit_controller_command(
            "thunder_01.thunderv4_locomotion",
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=generation,
                sequence=1,
                apply_time_ns=0,
                payload={"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0},
            ),
        )
        is CommandSubmitResult.ACCEPTED
    )

    snapshot = coordinator.advance(3)

    assert snapshot["physics_step"] == 3
    assert [command.sequence for command in host.applied] == [1]
    assert all(command.session_id == ready["session_id"] for command in host.applied)
    control_order = [
        call if isinstance(call, str) else call[0]
        for call in host.calls
        if call == "snapshot" or (isinstance(call, tuple) and call[0] in {"actuate", "advance"})
    ]
    assert control_order == [
        "snapshot",
        "snapshot",
        "actuate",
        "advance",
        "advance",
        "advance",
    ]

    coordinator.advance(1)
    assert [command.sequence for command in host.applied] == [1, 2]

    reset = coordinator.reset()
    assert reset["reset_generation"] == 1
    assert (
        coordinator.submit_controller_command(
            "thunder_01.thunderv4_locomotion",
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=generation,
                sequence=2,
                apply_time_ns=0,
                payload={},
            ),
        )
        is CommandSubmitResult.REJECTED_STALE_RESET_GENERATION
    )
    coordinator.stop()


def test_realtime_chunk_holds_one_control_output_across_physics_substeps(
    tmp_path: Path,
) -> None:
    host = ChunkedControlledPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        controller_factory=_fake_components,
        sensor_endpoint_factory=_headless_sensor_endpoint,
        run_id="controlled-realtime-chunk",
    )
    ready = coordinator.prepare()
    coordinator.start()
    assert (
        coordinator.submit_controller_command(
            "thunder_01.thunderv4_locomotion",
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=GenerationStamp(
                    ready["model_generation"], ready["reset_generation"]
                ),
                sequence=1,
                apply_time_ns=0,
                payload={"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0},
            ),
        )
        is CommandSubmitResult.ACCEPTED
    )

    snapshot = coordinator.advance_realtime(2)

    assert snapshot["physics_step"] == 2
    assert [command.sequence for command in host.applied] == [1]
    assert ("advance", 2) in host.calls
    coordinator.stop()


def test_realtime_sampled_chunk_preserves_every_strict_sensor_deadline(
    tmp_path: Path,
) -> None:
    host = SampledControlledPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        controller_factory=_fake_components,
        sensor_endpoint_factory=_strict_headless_sensor_endpoint,
        run_id="controlled-realtime-sampled-chunk",
    )
    ready = coordinator.prepare()
    coordinator.start()
    assert (
        coordinator.submit_controller_command(
            "thunder_01.thunderv4_locomotion",
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=GenerationStamp(
                    ready["model_generation"], ready["reset_generation"]
                ),
                sequence=1,
                apply_time_ns=0,
                payload={"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0},
            ),
        )
        is CommandSubmitResult.ACCEPTED
    )

    snapshot = coordinator.advance_realtime(7)

    assert snapshot["physics_step"] == 7
    assert [command.sequence for command in host.applied] == [1]
    assert ("advance_sampled", 7) in host.calls
    assert coordinator._sensors is not None
    observations = {
        item["sensor_id"]: item
        for item in coordinator._sensors.evidence_observations()
    }
    assert observations["thunder_01.imu"]["last_sample_sim_time_ns"] == 5_000_000
    assert observations["thunder_01.truth_odom"]["last_sample_sim_time_ns"] == 0
    coordinator.stop()


def test_realtime_sampled_chunk_fuses_one_actuator_command_with_physics_advance(
    tmp_path: Path,
) -> None:
    host = FusedSampledControlledPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        controller_factory=_fake_components,
        sensor_endpoint_factory=_strict_headless_sensor_endpoint,
        run_id="controlled-realtime-fused-chunk",
    )
    ready = coordinator.prepare()
    coordinator.start()
    assert (
        coordinator.submit_controller_command(
            "thunder_01.thunderv4_locomotion",
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=GenerationStamp(
                    ready["model_generation"], ready["reset_generation"]
                ),
                sequence=1,
                apply_time_ns=0,
                payload={"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0},
            ),
        )
        is CommandSubmitResult.ACCEPTED
    )

    snapshot = coordinator.advance_realtime(5)

    assert snapshot["physics_step"] == 5
    assert [command.sequence for command in host.applied] == [1]
    assert ("actuate_advance_sampled", 1, 5) in host.calls
    assert not any(
        isinstance(call, tuple) and call[0] in {"actuate", "advance_sampled"}
        for call in host.calls
    )
    coordinator.stop()


def test_realtime_sampled_chunk_rejects_a_batch_without_the_final_truth(
    tmp_path: Path,
) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=IncompleteSampledControlledPhysicsHost(),
        controller_factory=_fake_components,
        sensor_endpoint_factory=_strict_headless_sensor_endpoint,
        run_id="controlled-realtime-incomplete-sampled-chunk",
    )
    coordinator.prepare()
    coordinator.start()

    with pytest.raises(CoordinatorError, match="invalid snapshot batch"):
        coordinator.advance_realtime(7)

    assert coordinator.state is RuntimeState.FAILED


def test_coordinator_hold_forwards_to_prepared_control_runtime(
    tmp_path: Path,
) -> None:
    class RecordingControlRuntime:
        def __init__(self) -> None:
            self.hold_count = 0

        def hold(self) -> None:
            self.hold_count += 1

    control = RecordingControlRuntime()
    coordinator = RuntimeCoordinator(
        bundle_dir=tmp_path / "bundle",
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=ControlledPhysicsHost(),
        run_id="controlled-session-hold",
    )
    coordinator._state = RuntimeState.READY
    coordinator._control = control  # type: ignore[assignment]

    coordinator.hold_controller_commands()

    assert control.hold_count == 1


def test_coordinator_hold_is_a_noop_without_a_control_runtime(tmp_path: Path) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=tmp_path / "bundle",
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=ControlledPhysicsHost(),
        run_id="headless-session-hold",
    )
    coordinator._state = RuntimeState.READY

    coordinator.hold_controller_commands()


def test_controller_failure_stops_physics_and_marks_session_failed(
    tmp_path: Path,
) -> None:
    host = ControlledPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        controller_factory=_fake_components,
        sensor_endpoint_factory=_headless_sensor_endpoint,
        run_id="controller-failure",
    )
    ready = coordinator.prepare()
    coordinator.start()
    coordinator.submit_controller_command(
        "thunder_01.thunderv4_locomotion",
        ControllerCommand(
            channel_id="thunder_01.control.base_twist",
            instance_id="thunder_01",
            generation=GenerationStamp(ready["model_generation"], ready["reset_generation"]),
            sequence=1,
            apply_time_ns=0,
            payload={},
        ),
    )
    host.actuator_result = "rejected_out_of_range"

    with pytest.raises(CoordinatorError, match="rejected_out_of_range"):
        coordinator.advance(1)

    assert coordinator.state.value == "FAILED"
    assert host.calls[-2:] == ["pause", "stop"]


def test_warmup_control_failure_keeps_first_failed_terminal(
    tmp_path: Path,
) -> None:
    class DestroyedOnStopPhysicsHost(ControlledPhysicsHost):
        def stop(self) -> dict[str, Any]:
            if self.calls.count("stop"):
                self.session_id = ""
            return super().stop()

    host = DestroyedOnStopPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        controller_factory=_fake_components,
        sensor_endpoint_factory=_headless_sensor_endpoint,
        run_id="controller-warmup-failure",
    )
    coordinator.prepare()
    host.actuator_result = "rejected_out_of_range"

    with pytest.raises(CoordinatorError, match="rejected_out_of_range") as error:
        coordinator.warmup(1)

    assert "session_id does not match" not in str(error.value)
    assert coordinator.state is RuntimeState.FAILED
    manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
    assert manifest["state"] == "FAILED"
    episode_path = coordinator.allocation.run_dir / "episode_result.json"
    episode_bytes = episode_path.read_bytes()
    episode = json.loads(episode_bytes)
    assert episode["status"] == "FAILED"
    assert "rejected_out_of_range" in episode["failure_reason"]
    assert episode["start_sim_time_ns"] == 0
    assert episode["end_sim_time_ns"] == 0

    stopped = coordinator.stop()
    assert stopped["event"] == "snapshot"
    assert host.calls.count("stop") == 1
    assert episode_path.read_bytes() == episode_bytes


def test_required_external_bindings_need_generation_stamped_evidence(
    tmp_path: Path,
) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path, UNREAL_SESSION),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=ControlledPhysicsHost(),
        controller_factory=_fake_components,
        run_id="binding-evidence",
    )
    ready = coordinator.prepare()
    assert coordinator.state is RuntimeState.PREPARING
    with pytest.raises(CoordinatorError, match="READY"):
        coordinator.start()

    with pytest.raises(CoordinatorError, match="session_id"):
        coordinator.report_binding_prepared(
            "visual",
            source_id="robotsimue/test",
            session_id="0" * 64,
            model_generation=ready["model_generation"],
            reset_generation=ready["reset_generation"],
        )

    coordinator.report_binding_prepared(
        "visual",
        source_id="robotsimue/test",
        session_id=ready["session_id"],
        model_generation=ready["model_generation"],
        reset_generation=ready["reset_generation"],
    )
    coordinator.report_binding_active(
        "visual",
        source_id="robotsimue/test",
        session_id=ready["session_id"],
        model_generation=ready["model_generation"],
        reset_generation=ready["reset_generation"],
    )
    assert coordinator.state is RuntimeState.PREPARING

    for sensor_id in sorted(coordinator.sensor_readiness.streams):
        source_id = f"sensor-runtime/{sensor_id}"
        coordinator.report_sensor_stream_prepared(
            sensor_id,
            source_id=source_id,
            session_id=ready["session_id"],
            model_generation=ready["model_generation"],
            reset_generation=ready["reset_generation"],
        )
        coordinator.report_sensor_stream_active(
            sensor_id,
            source_id=source_id,
            session_id=ready["session_id"],
            model_generation=ready["model_generation"],
            reset_generation=ready["reset_generation"],
        )

    assert coordinator.state is RuntimeState.READY
    assert coordinator.readiness.is_ready


def test_default_coordinator_writes_scenario_physics_evidence_reference(
    tmp_path: Path,
) -> None:
    host = ControlledPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path, G007_SESSION),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        controller_factory=_fake_components,
        run_id="g007-default-physics-evidence",
    )

    coordinator.prepare()
    event = coordinator.stop()

    assert event["event"] == "stopped"
    assert any(call == ("kinematic", 0) for call in host.calls)
    raycast_calls = [
        call[1] for call in host.calls if isinstance(call, tuple) and call[0] == "raycast"
    ]
    assert raycast_calls
    assert raycast_calls[0]["session_id"] == coordinator.plan.session_id
    assert raycast_calls[0]["sequence"] == 0
    evidence_path = coordinator.allocation.run_dir / "scenario-physics-evidence.json"
    assert evidence_path.is_file()
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    assert evidence["source"] == "mujoco_applied_kinematic_proxy"
    assert evidence["session_id"] == coordinator.plan.session_id
    assert evidence["complete_proxy_set"] is True
    episode = json.loads(
        (coordinator.allocation.run_dir / "episode_result.json").read_text(
            encoding="utf-8"
        )
    )
    assert episode["artifact_references"]["scenario_physics_evidence"] == (
        "scenario-physics-evidence.json"
    )
