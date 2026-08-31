"""Lifecycle contracts for the single RobotSimUE playable runner."""

# ruff: noqa: S101

from __future__ import annotations

import copy
import ctypes
import json
import os
from collections.abc import Callable, Mapping
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

import sim.runtime.coordinator.playable_vertical_slice as playable_module
from sim.runtime.control import create_production_components
from sim.runtime.coordinator.coordinator import RuntimeState
from sim.runtime.coordinator.playable_evidence import (
    PinnedPlayableMediaToolchain,
    PlayableEvidenceError,
)
from sim.runtime.coordinator.playable_vertical_slice import (
    PLAYABLE_INPUT_SCHEDULE,
    OwnedRobotSimUEInput,
    PlayableActionContext,
    PlayableClosedRun,
    PlayableControlEvidenceWriter,
    PlayableLaunchDependencies,
    PlayableLaunchError,
    PlayableLifecycleError,
    PlayableRuntimeConfig,
    create_playable_launch,
    run_playable_launch,
    run_playable_vertical_slice,
)
from sim.runtime.coordinator.run_allocation import ResolvedSessionBundle
from sim.runtime.coordinator.unreal_process import (
    PackagedUnrealProcess,
    UnrealLaunchProfile,
    UnrealProcess,
)
from sim.runtime.process_owner import ProcessShutdownSnapshot
from sim.runtime.sensors import (
    ImuEndpointFactory,
    Mid360EndpointFactory,
    SensorEndpointRouter,
    TruthOdometryEndpointFactory,
)
from sim.runtime.windows_cpu_isolation import (
    WindowsCpuIsolationConfig,
    WindowsCpuIsolationPlan,
)

_SESSION_ID = "thunderv4_factory_park_hf"
_PORTS = {
    "visual_snapshot_udp": 25123,
    "control_intent_udp": 25124,
    "control_status_udp": 25125,
}


def _fixed_bundle(tmp_path: Path) -> ResolvedSessionBundle:
    repo_root = tmp_path / "repo"
    bundle_dir = tmp_path / "bundle"
    repo_root.mkdir()
    bundle_dir.mkdir()
    bundle = ResolvedSessionBundle(
        bundle_dir=bundle_dir,
        repo_root=repo_root,
        session_id=_SESSION_ID,
        session_spec={
            "schema": "lingtu.sim.session.v1",
            "session_id": _SESSION_ID,
            "runtime": {
                "backend": "mujoco",
                "mode": "unreal",
                "required_bindings": [
                    "physics",
                    "visual",
                    "sensors",
                    "control",
                ],
            },
        },
        plans={
            "physics.plan.json": {
                "schema": "lingtu.sim.physics-plan.v1",
                "session_id": _SESSION_ID,
                "global_policy": {"timestep_s": 0.001},
            },
            "visual.plan.json": {
                "schema": "lingtu.sim.visual-plan.v1",
                "session_id": _SESSION_ID,
                "backends": {"physics": "mujoco", "visual": "unreal"},
                "world": {
                    "package": {
                        "id": playable_module.PLAYABLE_WORLD_PACKAGE[0],
                        "version": playable_module.PLAYABLE_WORLD_PACKAGE[1],
                    },
                    "level": playable_module.PLAYABLE_LEVEL,
                },
                "robots": [
                    {
                        "instance_id": "thunder_01",
                        "package": {"id": "thunderv4", "version": "1.0.1"},
                    }
                ],
            },
            "sensor.plan.json": {
                "schema": "lingtu.sim.sensor-plan.v1",
                "session_id": _SESSION_ID,
                "backends": {"physics": "mujoco", "visual": "unreal"},
                "streams": {
                    "rgb": [
                        {
                            "sensor_id": "thunder_01.front_rgb",
                            "owner": "visual",
                            "source": "unreal_camera",
                            "transport": "camera_shm",
                            "rate_hz": 30,
                        }
                    ],
                    "depth": [
                        {
                            "sensor_id": "thunder_01.front_depth",
                            "owner": "visual",
                            "source": "unreal_camera",
                            "transport": "camera_shm",
                            "rate_hz": 30,
                        }
                    ],
                    "imu": [
                        {
                            "sensor_id": "thunder_01.imu",
                            "owner": "physics",
                            "source": "mujoco_sensor",
                            "transport": "typed_dds",
                            "rate_hz": 200,
                        }
                    ],
                    "mid360": [
                        {
                            "sensor_id": "thunder_01.mid360",
                            "owner": "physics",
                            "source": "mujoco_livox_model",
                            "transport": "typed_dds",
                            "rate_hz": 10,
                        }
                    ],
                    "truth_odom": [
                        {
                            "sensor_id": "thunder_01.truth_odom",
                            "owner": "physics",
                            "source": "mujoco_truth",
                            "transport": "typed_dds",
                            "rate_hz": 100,
                        }
                    ],
                },
            },
            "control.plan.json": {
                "schema": "lingtu.sim.control-plan.v1",
                "session_id": _SESSION_ID,
                "backends": {"physics": "mujoco", "visual": "unreal"},
                "controllers": [
                    {
                        "instance_id": "thunder_01",
                        "controller_id": "thunder_01.thunderv4_locomotion",
                        "package": {
                            "id": "thunderv4_locomotion",
                            "version": "1.0.0",
                        },
                        "command_channels": [
                            "thunder_01.control.base_twist",
                            "thunder_01.control.joint_torque",
                        ],
                    }
                ],
                "command_channels": [
                    {
                        "channel_id": "thunder_01.control.base_twist",
                        "direction": "subscribe",
                        "command_type": "base_twist",
                    },
                    {
                        "channel_id": "thunder_01.control.joint_torque",
                        "direction": "publish",
                        "command_type": "joint_torque",
                    },
                ],
            },
            "transport.intent.json": {
                "schema": "lingtu.sim.transport-intent.v1",
                "session_id": _SESSION_ID,
            },
        },
    )
    (bundle_dir / "session.yaml").write_text(
        json.dumps(bundle.session_spec),
        encoding="utf-8",
    )
    for filename, plan in bundle.plans.items():
        (bundle_dir / filename).write_text(json.dumps(plan), encoding="utf-8")
    return bundle


class _Resource:
    def __init__(self, role: str, calls: list[tuple[str, Any]]) -> None:
        self.role = role
        self.calls = calls
        self.started = False
        self.closed = False
        self.pid = 4242

    def start(self) -> Any:
        self.started = True
        self.calls.append((f"{self.role}.start", None))
        return None

    def poll(self) -> int | None:
        return 0

    def close(self) -> None:
        self.closed = True
        self.calls.append((f"{self.role}.close", None))

    def stop(self) -> Any:
        self.closed = True
        self.calls.append((f"{self.role}.stop", None))
        return None

    def register_episode_artifact(self, name: str, path: str) -> None:
        self.calls.append((f"{self.role}.register_episode_artifact", (name, path)))

    def declare_episode_artifact(self, name: str, path: str) -> None:
        self.calls.append((f"{self.role}.declare_episode_artifact", (name, path)))

    def attach_event_observer(
        self,
        _observer: Callable[[Mapping[str, Any]], object],
        *,
        replay_latest_snapshot: bool = False,
    ) -> int:
        del replay_latest_snapshot
        return 1

    def detach_event_observer(self, _token: int) -> bool:
        return True

    def capture_sensor_payloads(
        self,
        _snapshot: Mapping[str, Any],
    ) -> tuple[()]:
        return ()


class _SessionResource(_Resource):
    def __init__(self, role: str, calls: list[tuple[str, Any]]) -> None:
        super().__init__(role, calls)
        self.state = RuntimeState.NEW

    def prepare(self) -> Mapping[str, Any]:
        self.state = RuntimeState.READY
        self.calls.append(("session.prepare", None))
        return {"event": "ready"}

    def start(self) -> Mapping[str, Any]:
        self.started = True
        self.state = RuntimeState.RUNNING
        self.calls.append(("session.start", None))
        return {"event": "started"}

    def stop(self) -> Mapping[str, Any]:
        self.closed = True
        self.state = RuntimeState.STOPPED
        self.calls.append(("session.stop", None))
        return {"event": "stopped"}


class _PhysicsResource(_Resource):
    def raycast(self, **_kwargs: Any) -> object:
        return object()


class _EvidenceResource(_Resource):
    def __call__(self, _record: Mapping[str, Any]) -> None:
        self.calls.append(("evidence.write", None))


class _InputDriverAdapter:
    def __init__(
        self,
        perform: Callable[[Any], None],
        exit_request: Callable[[Any], None],
    ) -> None:
        self._perform = perform
        self._exit_request = exit_request

    def perform_actions(self, context: Any) -> None:
        self._perform(context)

    def request_exit(self, context: Any) -> None:
        self._exit_request(context)


def _factory(
    role: str,
    calls: list[tuple[str, Any]],
    resources: dict[str, _Resource],
) -> Callable[..., _Resource]:
    def create(*args: Any, **kwargs: Any) -> _Resource:
        calls.append((f"create.{role}", (args, kwargs)))
        resource = _Resource(role, calls)
        resources[role] = resource
        return resource

    return create


def _runtime(bundle: ResolvedSessionBundle, tmp_path: Path) -> PlayableRuntimeConfig:
    ffmpeg = (tmp_path / "ffmpeg.exe").resolve()
    ffprobe = (tmp_path / "ffprobe.exe").resolve()
    return PlayableRuntimeConfig(
        repo_root=bundle.repo_root or tmp_path,
        run_root=tmp_path / "runs",
        mujoco_host=tmp_path / "mujoco-host.exe",
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        media_toolchain=_fake_pinned_toolchain(ffmpeg, ffprobe),
        ports=_PORTS,
        unreal_editor_executable=tmp_path / "UnrealEditor.exe",
        unreal_project=tmp_path / "RobotSimUE.uproject",
        run_id="playable-test",
        boot_id="playable-boot-test",
    )


def test_production_playable_dependency_defaults_to_editor_game_runtime(
    tmp_path: Path,
) -> None:
    deps = PlayableLaunchDependencies()

    assert deps.editor_process_factory is UnrealProcess
    assert deps.packaged_process_factory is PackagedUnrealProcess
    process = deps.editor_process_factory(
        tmp_path / "UnrealEditor.exe",
        tmp_path / "RobotSimUE.uproject",
        playable_module.PLAYABLE_LEVEL,
    )
    assert isinstance(process, UnrealProcess)


def test_packaged_release_dependency_remains_explicit_non_default(
    tmp_path: Path,
) -> None:
    factory = PlayableLaunchDependencies().packaged_process_factory

    process = factory(
        tmp_path / "RobotSimUE-Win64-Release.exe",
        playable_module.PLAYABLE_LEVEL,
    )
    assert isinstance(process, PackagedUnrealProcess)


def _fake_pinned_toolchain(
    ffmpeg: Path,
    ffprobe: Path,
) -> PinnedPlayableMediaToolchain:
    toolchain = object.__new__(PinnedPlayableMediaToolchain)
    toolchain.paths = {
        "ffmpeg": ffmpeg.resolve(),
        "ffprobe": ffprobe.resolve(),
    }
    toolchain.descriptors = {}
    toolchain.trusted_probe = None
    toolchain.trusted_descriptor = None
    return toolchain


def test_create_playable_launch_assembles_one_inert_fixed_slice(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}

    def sensor_factory(*_args: Any, **_kwargs: Any) -> None:
        return None

    def build_sensors(physics_host: Any) -> Callable[..., None]:
        calls.append(("build.sensors", physics_host))
        return sensor_factory

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=_factory("session", calls, resources),
    )
    config = _runtime(bundle, tmp_path)

    launch = create_playable_launch(
        bundle,
        runtime=config,
        sensor_endpoint_factory_builder=build_sensors,
        dependencies=dependencies,
    )

    assert launch.run_id == "playable-test"
    assert launch.boot_id == "playable-boot-test"
    assert launch.run_dir == config.run_root / "playable-test"
    assert launch.ports == _PORTS
    assert launch.target.controller_id == "thunder_01.thunderv4_locomotion"
    assert launch.target.instance_id == "thunder_01"
    assert launch.target.channel_id == "thunder_01.control.base_twist"
    assert playable_module.PLAYABLE_ROBOT_PACKAGE == ("thunderv4", "1.0.1")

    coordinator_call = next(value for name, value in calls if name == "create.coordinator")
    _args, coordinator_kwargs = coordinator_call
    assert coordinator_kwargs["bundle_dir"] == bundle.bundle_dir
    assert coordinator_kwargs["controller_factory"] is create_production_components
    assert coordinator_kwargs["sensor_endpoint_factory"] is sensor_factory
    assert coordinator_kwargs["ports"] == _PORTS
    assert coordinator_kwargs["artifact_root_mode"] == "run"
    assert coordinator_kwargs["run_id"] == "playable-test"
    assert coordinator_kwargs["boot_id"] == "playable-boot-test"

    unreal_call = next(value for name, value in calls if name == "create.unreal")
    unreal_args, unreal_kwargs = unreal_call
    assert unreal_args == (
        config.unreal_editor_executable,
        config.unreal_project,
        playable_module.PLAYABLE_LEVEL,
    )
    assert unreal_kwargs["frame_capture_every"] == 1
    assert unreal_kwargs["frame_capture_max"] == 2000
    assert unreal_kwargs["session_camera_tag"] == "PreviewTarget:south_gate_robot_eye"
    assert unreal_kwargs["motion_camera_stable_id"] == "thunder_01/base_link"
    assert unreal_kwargs["depth_capture_in_main_renderer"] is False
    assert unreal_kwargs["shared_color_depth_capture"] is False
    assert unreal_kwargs["launch_profile"] is UnrealLaunchProfile.PLAYABLE_SDK_QUIET
    assert "affinity_mask" not in unreal_kwargs
    _physics_args, physics_kwargs = next(value for name, value in calls if name == "create.physics")
    assert "affinity_mask" not in physics_kwargs
    assert physics_kwargs["sample_stride_steps"] == 5
    assert launch.runtime_surface == playable_module.PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME
    assert launch.unreal_log_name == "Unreal.log"
    assert launch.frame_capture_wait_timeout_s == pytest.approx(900.0)

    host_call = next(value for name, value in calls if name == "create.host")
    _args, host_kwargs = host_call
    assert host_kwargs["coordinator"] is resources["coordinator"]
    assert host_kwargs["unreal_process"] is resources["unreal"]
    assert host_kwargs["publisher"] is resources["snapshot"]
    assert host_kwargs["evidence_watchers"] == (
        resources["visual_watcher"],
        resources["camera_watcher"],
    )

    receiver_call = next(value for name, value in calls if name == "create.receiver")
    receiver_args, receiver_kwargs = receiver_call
    assert receiver_args == (_PORTS["control_intent_udp"],)
    assert receiver_kwargs["expected_identity"].run_id == "playable-test"
    assert receiver_kwargs["expected_identity"].boot_id == "playable-boot-test"

    session_call = next(value for name, value in calls if name == "create.session")
    session_args, session_kwargs = session_call
    assert session_args == (resources["host"],)
    assert session_kwargs["steps_per_tick"] == 5
    assert session_kwargs["advance_wait_s"] == pytest.approx(0.005)
    assert session_kwargs["control_pump"] is resources["pump"]
    assert session_kwargs["sensor_payload_source"] is resources["camera_payload"]
    assert "owner_thread_affinity_mask" not in session_kwargs

    pump_call = next(value for name, value in calls if name == "create.pump")
    _pump_args, pump_kwargs = pump_call
    assert pump_kwargs["bundle_dir"] == bundle.bundle_dir
    assert pump_kwargs["controller_id"] == "thunder_01.thunderv4_locomotion"
    assert pump_kwargs["trace_sink"] is launch.control_evidence_writer

    assert all(not resource.started for resource in resources.values())
    assert all(not resource.closed for resource in resources.values())
    assert launch.run_dir.exists() is False


def test_create_playable_launch_routes_resolved_windows_cpu_isolation(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}
    config = WindowsCpuIsolationConfig()
    plan = WindowsCpuIsolationPlan(
        processor_group=0,
        mujoco_core_id=0,
        owner_core_id=1,
        unreal_core_ids=(2, 3),
        mujoco_affinity_mask=0b00000011,
        owner_thread_affinity_mask=0b00001100,
        unreal_affinity_mask=0b11110000,
    )

    def resolve(value: WindowsCpuIsolationConfig) -> WindowsCpuIsolationPlan:
        calls.append(("resolve.cpu", value))
        return plan

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=_factory("session", calls, resources),
        cpu_isolation_resolver=resolve,
    )
    runtime = replace(
        _runtime(bundle, tmp_path),
        windows_cpu_isolation=config,
    )

    create_playable_launch(
        bundle,
        runtime=runtime,
        sensor_endpoint_factory_builder=lambda _physics: lambda **_kwargs: None,
        dependencies=dependencies,
    )

    assert ("resolve.cpu", config) in calls
    _physics_args, physics_kwargs = next(value for name, value in calls if name == "create.physics")
    _unreal_args, unreal_kwargs = next(value for name, value in calls if name == "create.unreal")
    _session_args, session_kwargs = next(value for name, value in calls if name == "create.session")
    assert physics_kwargs["affinity_mask"] == plan.mujoco_affinity_mask
    assert unreal_kwargs["affinity_mask"] == plan.unreal_affinity_mask
    assert session_kwargs["owner_thread_affinity_mask"] == plan.owner_thread_affinity_mask


def test_create_playable_launch_uses_packaged_factory_only_for_explicit_surface(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}
    ffmpeg = (tmp_path / "ffmpeg.exe").resolve()
    ffprobe = (tmp_path / "ffprobe.exe").resolve()
    runtime = PlayableRuntimeConfig(
        repo_root=bundle.repo_root or tmp_path,
        run_root=tmp_path / "runs",
        mujoco_host=tmp_path / "mujoco-host.exe",
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        media_toolchain=_fake_pinned_toolchain(ffmpeg, ffprobe),
        ports=_PORTS,
        runtime_surface=playable_module.PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE,
        robotsimue_executable=tmp_path / "RobotSimUE-Win64-Release.exe",
        run_id="playable-test",
        boot_id="playable-boot-test",
    )

    def editor_factory(*_args: Any, **_kwargs: Any) -> object:
        raise AssertionError("editor factory must not be used for packaged_release")

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=editor_factory,
        packaged_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=_factory("session", calls, resources),
    )

    launch = create_playable_launch(
        bundle,
        runtime=runtime,
        sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
        dependencies=dependencies,
    )

    unreal_call = next(value for name, value in calls if name == "create.unreal")
    unreal_args, unreal_kwargs = unreal_call
    assert unreal_args == (
        runtime.robotsimue_executable,
        playable_module.PLAYABLE_LEVEL,
    )
    assert unreal_kwargs["shared_color_depth_capture"] is False
    assert "launch_profile" not in unreal_kwargs
    assert launch.runtime_surface == playable_module.PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE
    assert launch.unreal_log_name == "RobotSimUE.log"
    launch._closed = True
    closed = playable_module._build_closed_run(launch, exit_event_id=None)
    assert closed.unreal_log_path == launch.run_dir / "logs" / "RobotSimUE.log"


@pytest.mark.parametrize("mutation", ["missing", "extra"])
def test_create_playable_launch_rejects_any_non_exact_sensor_plan_before_construction(
    tmp_path: Path,
    mutation: str,
) -> None:
    original = _fixed_bundle(tmp_path)
    plans = copy.deepcopy(dict(original.plans))
    streams = plans["sensor.plan.json"]["streams"]
    if mutation == "missing":
        streams["mid360"] = []
    else:
        streams["rgb"].append(
            {
                "sensor_id": "thunder_01.rear_rgb",
                "owner": "visual",
                "source": "unreal_camera",
                "transport": "camera_shm",
                "rate_hz": 30,
            }
        )
    bundle = ResolvedSessionBundle(
        bundle_dir=original.bundle_dir,
        repo_root=original.repo_root,
        session_id=original.session_id,
        session_spec=original.session_spec,
        plans=plans,
    )
    constructed = False

    def physics_host_factory(*_args: Any, **_kwargs: Any) -> object:
        nonlocal constructed
        constructed = True
        return object()

    with pytest.raises(ValueError, match="exact five production streams"):
        create_playable_launch(
            bundle,
            runtime=_runtime(bundle, tmp_path),
            sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
            dependencies=PlayableLaunchDependencies(physics_host_factory=physics_host_factory),
        )

    assert constructed is False


@pytest.mark.parametrize(
    "ports",
    [
        {
            "visual_snapshot_udp": 25123,
            "control_intent_udp": 25123,
            "control_status_udp": 25125,
        },
        {
            "visual_snapshot_udp": 25123,
            "control_intent_udp": 25124,
        },
    ],
)
def test_runtime_config_requires_exactly_three_distinct_allocation_ports(
    tmp_path: Path,
    ports: Mapping[str, int],
) -> None:
    bundle = _fixed_bundle(tmp_path)
    ffmpeg = (tmp_path / "ffmpeg.exe").resolve()
    ffprobe = (tmp_path / "ffprobe.exe").resolve()

    with pytest.raises(ValueError, match=r"ports|distinct"):
        PlayableRuntimeConfig(
            repo_root=bundle.repo_root or tmp_path,
            run_root=tmp_path / "runs",
            mujoco_host=tmp_path / "mujoco-host.exe",
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            media_toolchain=_fake_pinned_toolchain(ffmpeg, ffprobe),
            ports=ports,
            unreal_editor_executable=tmp_path / "UnrealEditor.exe",
            unreal_project=tmp_path / "RobotSimUE.uproject",
        )


def test_runtime_config_rejects_invalid_editor_game_runtime_identity(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    ffmpeg = (tmp_path / "ffmpeg.exe").resolve()
    ffprobe = (tmp_path / "ffprobe.exe").resolve()

    for executable, project, expected in (
        ("RobotSimUE-Win64-Release.exe", "RobotSimUE.uproject", "UnrealEditor.exe"),
        ("UnrealEditor.exe", "Other.uproject", "RobotSimUE.uproject"),
    ):
        with pytest.raises(
            PlayableLaunchError,
            match=expected,
        ):
            PlayableRuntimeConfig(
                repo_root=bundle.repo_root or tmp_path,
                run_root=tmp_path / "runs",
                mujoco_host=tmp_path / "mujoco-host.exe",
                ffmpeg=ffmpeg,
                ffprobe=ffprobe,
                media_toolchain=_fake_pinned_toolchain(ffmpeg, ffprobe),
                ports=_PORTS,
                unreal_editor_executable=tmp_path / executable,
                unreal_project=tmp_path / project,
            )


def test_runtime_config_accepts_only_explicit_packaged_release_binary(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    ffmpeg = (tmp_path / "ffmpeg.exe").resolve()
    ffprobe = (tmp_path / "ffprobe.exe").resolve()

    for executable in ("UnrealEditor.exe", "RobotSimUE.exe", "RobotSimUE-Win64.exe"):
        with pytest.raises(
            PlayableLaunchError,
            match=r"RobotSimUE-Win64-Release\.exe",
        ):
            PlayableRuntimeConfig(
                repo_root=bundle.repo_root or tmp_path,
                run_root=tmp_path / "runs",
                mujoco_host=tmp_path / "mujoco-host.exe",
                ffmpeg=ffmpeg,
                ffprobe=ffprobe,
                media_toolchain=_fake_pinned_toolchain(ffmpeg, ffprobe),
                ports=_PORTS,
                runtime_surface=playable_module.PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE,
                robotsimue_executable=tmp_path / executable,
            )

    runtime = PlayableRuntimeConfig(
        repo_root=bundle.repo_root or tmp_path,
        run_root=tmp_path / "runs",
        mujoco_host=tmp_path / "mujoco-host.exe",
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        media_toolchain=_fake_pinned_toolchain(ffmpeg, ffprobe),
        ports=_PORTS,
        runtime_surface=playable_module.PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE,
        robotsimue_executable=tmp_path / "RobotSimUE-Win64-Release.exe",
    )
    assert runtime.runtime_surface == playable_module.PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE


def test_create_playable_launch_rejects_preexisting_run_before_construction(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    runtime = _runtime(bundle, tmp_path)
    run_dir = runtime.run_root / "playable-test"
    run_dir.mkdir(parents=True)
    (run_dir / "stale.txt").write_text("stale", encoding="utf-8")
    constructed = False

    def physics_factory(*_args: Any, **_kwargs: Any) -> object:
        nonlocal constructed
        constructed = True
        return object()

    with pytest.raises(ValueError, match="already exists"):
        create_playable_launch(
            bundle,
            runtime=runtime,
            sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
            dependencies=PlayableLaunchDependencies(physics_host_factory=physics_factory),
        )

    assert constructed is False


def test_run_rejects_bundle_disk_drift_before_starting_any_process(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}
    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=_factory("session", calls, resources),
    )
    launch = create_playable_launch(
        bundle,
        runtime=_runtime(bundle, tmp_path),
        sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
        dependencies=dependencies,
    )
    sensor_path = bundle.bundle_dir / "sensor.plan.json"
    mutated = copy.deepcopy(dict(bundle.plans["sensor.plan.json"]))
    mutated["streams"]["mid360"] = []
    sensor_path.write_text(json.dumps(mutated), encoding="utf-8")

    with pytest.raises(PlayableLifecycleError, match="changed before start"):
        run_playable_launch(launch, run_body=lambda: None)

    assert "receiver.start" not in {name for name, _value in calls}
    assert "session.prepare" not in {name for name, _value in calls}
    assert launch._closed is True


def test_run_playable_launch_starts_only_the_owned_lifecycle_and_closes_it(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}

    def session_factory(*args: Any, **kwargs: Any) -> _SessionResource:
        calls.append(("create.session", (args, kwargs)))
        session = _SessionResource("session", calls)
        resources["session"] = session
        return session

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=session_factory,
    )
    launch = create_playable_launch(
        bundle,
        runtime=_runtime(bundle, tmp_path),
        sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
        dependencies=dependencies,
    )

    def run_body() -> str:
        calls.append(("run_body", None))
        return "complete"

    result = run_playable_launch(launch, run_body=run_body)

    assert result == "complete"
    lifecycle = [
        name
        for name, _value in calls
        if name
        in {
            "receiver.start",
            "session.prepare",
            "coordinator.declare_episode_artifact",
            "session.start",
            "run_body",
            "receiver.close",
            "session.stop",
            "host.close",
            "ack.close",
            "camera_payload.close",
        }
    ]
    assert lifecycle == [
        "receiver.start",
        "session.prepare",
        "coordinator.declare_episode_artifact",
        "coordinator.declare_episode_artifact",
        "session.start",
        "run_body",
        "receiver.close",
        "session.stop",
        "host.close",
        "ack.close",
        "camera_payload.close",
    ]
    assert resources["receiver"].closed is True
    assert resources["session"].closed is True
    assert resources["host"].closed is True
    assert resources["physics"].closed is True
    assert resources["ack"].closed is True
    assert resources["camera_payload"].closed is True


def test_create_playable_launch_default_sensor_router_owns_all_three_native_streams(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}

    def physics_factory(*args: Any, **kwargs: Any) -> _PhysicsResource:
        calls.append(("create.physics", (args, kwargs)))
        physics = _PhysicsResource("physics", calls)
        resources["physics"] = physics
        return physics

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=physics_factory,
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=_factory("session", calls, resources),
    )
    runtime = replace(
        _runtime(bundle, tmp_path),
        imu_publisher=tmp_path / "lingtu_imu_publisher.exe",
        truth_odom_publisher=tmp_path / "lingtu_truth_odom_publisher.exe",
        mid360_publisher=tmp_path / "lingtu_mid360_publisher.exe",
    )

    create_playable_launch(
        bundle,
        runtime=runtime,
        dependencies=dependencies,
    )

    coordinator_call = next(value for name, value in calls if name == "create.coordinator")
    _args, coordinator_kwargs = coordinator_call
    router = coordinator_kwargs["sensor_endpoint_factory"]
    assert isinstance(router, SensorEndpointRouter)
    assert tuple(type(factory) for factory in router.factories) == (
        ImuEndpointFactory,
        TruthOdometryEndpointFactory,
        Mid360EndpointFactory,
    )
    mid360 = router.factories[2]
    assert isinstance(mid360, Mid360EndpointFactory)
    assert mid360.process is resources["physics"]

    watcher_calls = {name: value for name, value in calls if name.startswith("create.") and "watcher" in name}
    assert watcher_calls.keys() == {
        "create.visual_watcher",
        "create.camera_watcher",
    }


def test_create_playable_launch_unwinds_every_constructed_owner_on_failure(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}

    def fail_session(*_args: Any, **_kwargs: Any) -> object:
        calls.append(("create.session", None))
        raise RuntimeError("session construction failed")

    def evidence_factory(*args: Any, **kwargs: Any) -> _EvidenceResource:
        calls.append(("create.evidence", (args, kwargs)))
        evidence = _EvidenceResource("evidence", calls)
        resources["evidence"] = evidence
        return evidence

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_evidence_writer_factory=evidence_factory,
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=fail_session,
    )

    with pytest.raises(RuntimeError, match="session construction failed"):
        create_playable_launch(
            bundle,
            runtime=_runtime(bundle, tmp_path),
            sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
            dependencies=dependencies,
        )

    cleanup = [name for name, _value in calls if name.endswith(".close")]
    assert cleanup == [
        "ack.close",
        "receiver.close",
        "camera_payload.close",
        "host.close",
        "evidence.close",
    ]
    assert all(resources[name].closed for name in ("ack", "receiver", "camera_payload", "host", "evidence"))
    assert resources["physics"].closed is True


def test_run_playable_launch_preserves_body_failure_while_closing_all_owners(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}

    def session_factory(*args: Any, **kwargs: Any) -> _SessionResource:
        calls.append(("create.session", (args, kwargs)))
        session = _SessionResource("session", calls)
        resources["session"] = session
        return session

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=session_factory,
    )
    launch = create_playable_launch(
        bundle,
        runtime=_runtime(bundle, tmp_path),
        sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
        dependencies=dependencies,
    )
    failure = RuntimeError("input automation failed")

    def fail_body() -> None:
        raise failure

    with pytest.raises(RuntimeError) as caught:
        run_playable_launch(launch, run_body=fail_body)

    assert caught.value is failure
    assert resources["receiver"].closed is True
    assert resources["session"].closed is True
    assert resources["host"].closed is True
    assert resources["physics"].closed is True
    assert resources["ack"].closed is True
    assert resources["camera_payload"].closed is True


def test_control_evidence_writer_keeps_only_motion_and_exit_zero_in_qualification_log(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    writer = PlayableControlEvidenceWriter(run_dir)
    accepted = {
        "schema": "lingtu.sim.control-intent-accepted.v1",
        "event": "control_command_accepted",
        "event_id": "motion-1",
    }
    pause_zero = {
        "schema": "lingtu.sim.control-command-zero.v1",
        "event": "control_command_zero",
        "submit_result": "accepted",
        "reason": "cleared:pause",
    }
    exit_zero = {
        "schema": "lingtu.sim.control-command-zero.v1",
        "event": "control_command_zero",
        "run_id": "playable-test",
        "session_id": _SESSION_ID,
        "boot_id": "playable-boot-test",
        "model_generation": 0,
        "reset_generation": 0,
        "source_id": "robotsimue.local_player.0",
        "source_epoch": "epoch-1",
        "source_sequence": 7,
        "event_id": "exit-1",
        "source_monotonic_ns": 100,
        "arrival_monotonic_ns": 101,
        "datagram_sha256": "b" * 64,
        "controller_sequence": 8,
        "apply_time_ns": 200,
        "submit_result": "accepted",
        "admitted_twist": {
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
        },
        "reason": "cleared:exit",
    }

    writer(accepted)
    writer(pause_zero)
    writer(exit_zero)
    with pytest.raises(ValueError, match="unsupported playable control trace schema"):
        writer({"schema": "lingtu.sim.unknown.v1"})
    writer.close()

    qualification_lines = [
        json.loads(line)
        for line in (run_dir / "control-intent-accepted.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert [line["schema"] for line in qualification_lines] == [
        "lingtu.sim.control-intent-accepted.v1",
        "lingtu.sim.control-command-zero.v1",
    ]
    assert qualification_lines[-1]["reason"] == "cleared:exit"
    audit_lines = (run_dir / "control-command-zero-audit.jsonl").read_text(encoding="utf-8")
    assert json.loads(audit_lines)["reason"] == "cleared:pause"


def test_control_evidence_writer_fsyncs_at_close_not_per_sample(
    tmp_path: Path,
    monkeypatch,
) -> None:
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    sync_calls: list[int] = []
    monkeypatch.setattr(
        "sim.runtime.coordinator.playable_vertical_slice.os.fsync",
        lambda descriptor: sync_calls.append(descriptor),
    )
    writer = PlayableControlEvidenceWriter(run_dir)
    accepted = {
        "schema": "lingtu.sim.control-intent-accepted.v1",
        "event": "control_command_accepted",
        "event_id": "motion-1",
    }

    try:
        writer(accepted)
        assert (run_dir / "control-intent-accepted.jsonl").read_text(encoding="utf-8").endswith("\n")
        assert sync_calls == []
    finally:
        writer.close()

    assert len(sync_calls) == 1


def test_playable_runner_uses_fixed_input_seam_requires_exit_and_rescans_after_close(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}
    run_dir = tmp_path / "runs" / "playable-test"

    class FrameOnCloseHost(_Resource):
        def close(self) -> None:
            super().close()
            frame_dir = run_dir / "frames"
            frame_dir.mkdir(parents=True, exist_ok=True)
            (frame_dir / "frame_000001.png").write_bytes(b"post-close-frame")

    def host_factory(*args: Any, **kwargs: Any) -> FrameOnCloseHost:
        calls.append(("create.host", (args, kwargs)))
        host = FrameOnCloseHost("host", calls)
        resources["host"] = host
        return host

    def session_factory(*args: Any, **kwargs: Any) -> _SessionResource:
        calls.append(("create.session", (args, kwargs)))
        session = _SessionResource("session", calls)
        resources["session"] = session
        return session

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=host_factory,
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=session_factory,
    )
    launch = create_playable_launch(
        bundle,
        runtime=_runtime(bundle, tmp_path),
        sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
        dependencies=dependencies,
    )
    action_contexts: list[Any] = []

    def perform_actions(context: Any) -> None:
        action_contexts.append(context)
        context.run_dir.mkdir(parents=True)
        assert context.actions == PLAYABLE_INPUT_SCHEDULE
        assert [action.key for action in context.actions] == ["W", "S", "A", "D", "Q", "E"]
        assert [action.hold_s for action in context.actions[-2:]] == [5.3, 5.3]
        assert sum(action.hold_s + action.neutral_after_s for action in context.actions) >= 20.0
        assert context.deadman_key == "SHIFT"
        assert context.unreal_pid == 4242
        assert not hasattr(context, "coordinator")
        assert not hasattr(context, "submit_controller_command")
        assert set(context.hud_screenshot_paths) == {
            "drive",
            "tactical",
            "menu_recording",
        }

    def request_ue_exit(context: Any) -> None:
        calls.append(("request_ue_exit", context.unreal_pid))
        logs = context.run_dir / "logs"
        logs.mkdir()
        (logs / "ue-control-origin.jsonl").write_text(
            json.dumps(_origin_document()) + "\n",
            encoding="utf-8",
        )
        (context.run_dir / "ue-control-origin.jsonl").write_text(
            "legacy-root-origin-must-not-be-read\n",
            encoding="utf-8",
        )
        runtime_trace, zero_trace = _exit_trace_documents()
        launch.control_evidence_writer(runtime_trace)
        launch.control_evidence_writer(zero_trace)
        session = resources["session"]
        assert isinstance(session, _SessionResource)
        session.state = RuntimeState.STOPPED

    def qualify(closed_run: PlayableClosedRun) -> str:
        calls.append(("qualify", closed_run))
        assert launch._closed is True
        assert all(
            resources[name].closed for name in ("receiver", "session", "host", "physics", "ack", "camera_payload")
        )
        assert closed_run.frames == (run_dir / "frames" / "frame_000001.png",)
        assert closed_run.exit_event_id == "exit-1"
        assert closed_run.unreal_log_path == run_dir / "logs" / "Unreal.log"
        return "qualified"

    result = run_playable_vertical_slice(
        launch,
        input_driver=_InputDriverAdapter(perform_actions, request_ue_exit),
        qualify_closed_run=qualify,
    )

    assert result == "qualified"
    assert len(action_contexts) == 1
    assert [name for name, _value in calls if name in {"request_ue_exit", "qualify"}] == [
        "request_ue_exit",
        "qualify",
    ]


def test_playable_runner_rejects_python_only_exit_traces_without_ue_origin(
    tmp_path: Path,
) -> None:
    bundle = _fixed_bundle(tmp_path)
    calls: list[tuple[str, Any]] = []
    resources: dict[str, _Resource] = {}

    def session_factory(*args: Any, **kwargs: Any) -> _SessionResource:
        calls.append(("create.session", (args, kwargs)))
        session = _SessionResource("session", calls)
        resources["session"] = session
        return session

    dependencies = PlayableLaunchDependencies(
        physics_host_factory=_factory("physics", calls, resources),
        coordinator_factory=_factory("coordinator", calls, resources),
        editor_process_factory=_factory("unreal", calls, resources),
        snapshot_publisher_factory=_factory("snapshot", calls, resources),
        visual_watcher_factory=_factory("visual_watcher", calls, resources),
        camera_watcher_factory=_factory("camera_watcher", calls, resources),
        session_host_factory=_factory("host", calls, resources),
        camera_payload_source_factory=_factory("camera_payload", calls, resources),
        motion_inbox_factory=_factory("motion_inbox", calls, resources),
        request_inbox_factory=_factory("request_inbox", calls, resources),
        control_receiver_factory=_factory("receiver", calls, resources),
        control_ack_publisher_factory=_factory("ack", calls, resources),
        control_pump_factory=_factory("pump", calls, resources),
        interactive_session_factory=session_factory,
    )
    launch = create_playable_launch(
        bundle,
        runtime=_runtime(bundle, tmp_path),
        sensor_endpoint_factory_builder=lambda _physics: lambda *_args: None,
        dependencies=dependencies,
    )
    qualified: list[PlayableClosedRun] = []

    def perform_actions(context: Any) -> None:
        context.run_dir.mkdir(parents=True)

    def python_only_exit(_context: Any) -> None:
        runtime_trace, zero_trace = _exit_trace_documents()
        launch.control_evidence_writer(runtime_trace)
        launch.control_evidence_writer(zero_trace)
        session = resources["session"]
        assert isinstance(session, _SessionResource)
        session.state = RuntimeState.STOPPED

    with pytest.raises(PlayableLifecycleError, match="correlated UE-origin exit zero"):
        run_playable_vertical_slice(
            launch,
            input_driver=_InputDriverAdapter(perform_actions, python_only_exit),
            qualify_closed_run=lambda closed: qualified.append(closed),
        )

    assert len(qualified) == 1
    assert launch._closed is True


def test_playable_runner_jsonl_reader_rejects_linked_evidence(tmp_path: Path) -> None:
    source = tmp_path / "origin-source.jsonl"
    source.write_text(json.dumps(_origin_document()) + "\n", encoding="utf-8")
    linked = tmp_path / "ue-control-origin.jsonl"
    try:
        os.symlink(source, linked)
    except OSError:
        pytest.skip("symbolic-link creation is unavailable on this Windows host")

    with pytest.raises(PlayableEvidenceError, match="link/reparse"):
        playable_module._read_jsonl(linked)


def test_user32_sendinput_record_uses_the_native_windows_input_abi_size() -> None:
    if playable_module.os.name != "nt":
        pytest.skip("SendInput ABI exists only on Windows")

    backend = playable_module._User32SendInputBackend()
    expected_size = 40 if ctypes.sizeof(ctypes.c_void_p) == 8 else 28

    assert ctypes.sizeof(backend._input_type) == expected_size
    assert ctypes.sizeof(backend._user32.GetForegroundWindow.restype) == ctypes.sizeof(ctypes.c_void_p)


def test_user32_window_enumeration_preserves_pointer_width_and_owned_pid() -> None:
    large_hwnd = 0x1_0000_7001
    title = "RobotSimUE (64 bit Development PCD3D_SM5)"

    class FakeWintypes:
        HWND = ctypes.c_void_p
        DWORD = ctypes.c_ulong

    class Rect(ctypes.Structure):
        _fields_ = (
            ("left", ctypes.c_long),
            ("top", ctypes.c_long),
            ("right", ctypes.c_long),
            ("bottom", ctypes.c_long),
        )

    class FakeUser32:
        @staticmethod
        def EnumWindows(callback: Callable[[Any, Any], bool], _lparam: int) -> bool:
            return bool(callback(ctypes.c_void_p(large_hwnd), 0))

        @staticmethod
        def GetWindowThreadProcessId(_hwnd: Any, process_id: Any) -> int:
            process_id._obj.value = 57040
            return 22

        @staticmethod
        def GetWindowTextLengthW(_hwnd: Any) -> int:
            return len(title)

        @staticmethod
        def GetWindowTextW(_hwnd: Any, buffer: Any, _size: int) -> int:
            buffer.value = title
            return len(title)

        @staticmethod
        def GetWindow(_hwnd: Any, _command: int) -> int:
            return 0

        @staticmethod
        def IsWindowVisible(_hwnd: Any) -> bool:
            return True

        @staticmethod
        def IsWindowEnabled(_hwnd: Any) -> bool:
            return True

        @staticmethod
        def GetWindowRect(_hwnd: Any, rect: Any) -> bool:
            rect._obj.left = 0
            rect._obj.top = 0
            rect._obj.right = 1920
            rect._obj.bottom = 1080
            return True

        @staticmethod
        def GetClientRect(_hwnd: Any, rect: Any) -> bool:
            rect._obj.left = 0
            rect._obj.top = 0
            rect._obj.right = 1900
            rect._obj.bottom = 1000
            return True

    backend = object.__new__(playable_module._User32SendInputBackend)
    backend_fixture: Any = backend
    backend_fixture._ctypes = ctypes
    backend_fixture._wintypes = FakeWintypes
    backend_fixture._user32 = FakeUser32()
    backend_fixture._enum_callback_type = lambda callback: callback
    backend_fixture._rect_type = Rect

    windows = backend.top_level_windows(57040, title_fragment="RobotSimUE")

    assert len(windows) == 1
    assert windows[0].hwnd == large_hwnd
    assert windows[0].pid == 57040
    assert windows[0].client_area == 1_900_000


def test_user32_window_enumeration_rethrows_callback_failure_with_cause() -> None:
    class NativeCallbackAbort(BaseException):
        pass

    callback_error = NativeCallbackAbort(
        "injected GetWindowThreadProcessId failure"
    )

    class FakeWintypes:
        HWND = ctypes.c_void_p
        DWORD = ctypes.c_ulong

    class FakeUser32:
        @staticmethod
        def EnumWindows(callback: Callable[[Any, Any], bool], _lparam: int) -> bool:
            try:
                callback(ctypes.c_void_p(0x1_0000_7001), 0)
            except BaseException:
                # ctypes callbacks report the exception out-of-band and return
                # control to the native caller.  The production adapter must
                # retain it explicitly rather than turning it into no windows.
                return False
            return True

        @staticmethod
        def GetWindowThreadProcessId(_hwnd: Any, _process_id: Any) -> int:
            raise callback_error

    backend = object.__new__(playable_module._User32SendInputBackend)
    backend_fixture: Any = backend
    backend_fixture._ctypes = ctypes
    backend_fixture._wintypes = FakeWintypes
    backend_fixture._user32 = FakeUser32()
    backend_fixture._enum_callback_type = lambda callback: callback

    with pytest.raises(
        PlayableLifecycleError,
        match="RobotSimUE window enumeration callback failed",
    ) as captured:
        backend.top_level_windows(57040, title_fragment="RobotSimUE")

    assert captured.value.__cause__ is callback_error


def test_user32_focus_attaches_input_queues_and_always_detaches_on_error() -> None:
    events: list[tuple[str, Any]] = []

    class FakeWintypes:
        HWND = int
        DWORD = ctypes.c_ulong

    class FakeKernel32:
        @staticmethod
        def GetCurrentThreadId() -> int:
            events.append(("current_thread", None))
            return 10

    class FakeUser32:
        @staticmethod
        def GetForegroundWindow() -> int:
            events.append(("foreground", None))
            return 8001

        @staticmethod
        def GetWindowThreadProcessId(hwnd: int, process_id: Any) -> int:
            events.append(("window_thread", hwnd))
            process_id._obj.value = 4242
            return {8001: 20, 7001: 30}[hwnd]

        @staticmethod
        def AttachThreadInput(first: int, second: int, attach: bool) -> bool:
            events.append(("attach", (first, second, attach)))
            return True

        @staticmethod
        def ShowWindow(hwnd: int, command: int) -> bool:
            events.append(("show", (hwnd, command)))
            return True

        @staticmethod
        def BringWindowToTop(hwnd: int) -> bool:
            events.append(("bring", hwnd))
            return True

        @staticmethod
        def SetForegroundWindow(hwnd: int) -> bool:
            events.append(("set_foreground", hwnd))
            return True

        @staticmethod
        def SetFocus(hwnd: int) -> int:
            events.append(("set_focus", hwnd))
            raise OSError("injected SetFocus failure")

    backend = object.__new__(playable_module._User32SendInputBackend)
    backend_fixture: Any = backend
    backend_fixture._ctypes = ctypes
    backend_fixture._wintypes = FakeWintypes
    backend_fixture._user32 = FakeUser32()
    backend_fixture._kernel32 = FakeKernel32()

    with pytest.raises(PlayableLifecycleError, match="focus RobotSimUE window"):
        backend.focus_window(7001)

    assert events == [
        ("foreground", None),
        ("window_thread", 8001),
        ("window_thread", 7001),
        ("current_thread", None),
        ("attach", (10, 20, True)),
        ("attach", (10, 30, True)),
        ("show", (7001, 9)),
        ("bring", 7001),
        ("set_foreground", 7001),
        ("set_focus", 7001),
        ("attach", (10, 30, False)),
        ("attach", (10, 20, False)),
    ]


def test_user32_focus_reports_attach_denial_when_foreground_stays_foreign() -> None:
    events: list[tuple[str, Any]] = []

    class FakeWintypes:
        HWND = int
        DWORD = ctypes.c_ulong

    class FakeKernel32:
        @staticmethod
        def GetCurrentThreadId() -> int:
            return 10

    class FakeUser32:
        @staticmethod
        def GetForegroundWindow() -> int:
            return 8001

        @staticmethod
        def GetWindowThreadProcessId(hwnd: int, process_id: Any) -> int:
            process_id._obj.value = 4242
            return {8001: 20, 7001: 30}[hwnd]

        @staticmethod
        def AttachThreadInput(first: int, second: int, attach: bool) -> bool:
            events.append(("attach", (first, second, attach)))
            return False

        @staticmethod
        def ShowWindow(_hwnd: int, _command: int) -> bool:
            return True

        @staticmethod
        def BringWindowToTop(_hwnd: int) -> bool:
            return False

        @staticmethod
        def SetForegroundWindow(_hwnd: int) -> bool:
            return False

        @staticmethod
        def SetFocus(_hwnd: int) -> int:
            return 0

    backend = object.__new__(playable_module._User32SendInputBackend)
    backend_fixture: Any = backend
    backend_fixture._ctypes = ctypes
    backend_fixture._wintypes = FakeWintypes
    backend_fixture._user32 = FakeUser32()
    backend_fixture._kernel32 = FakeKernel32()

    with pytest.raises(
        PlayableLifecycleError,
        match=r"AttachThreadInput failed for thread IDs 20, 30; foreground=8001",
    ):
        backend.focus_window(7001)

    assert events == [
        ("attach", (10, 20, True)),
        ("attach", (10, 30, True)),
    ]


def test_user32_focus_accepts_attach_denial_only_when_final_foreground_proves_target() -> None:
    foreground = [8001]

    class FakeWintypes:
        HWND = int
        DWORD = ctypes.c_ulong

    class FakeKernel32:
        @staticmethod
        def GetCurrentThreadId() -> int:
            return 10

    class FakeUser32:
        @staticmethod
        def GetForegroundWindow() -> int:
            return foreground[0]

        @staticmethod
        def GetWindowThreadProcessId(hwnd: int, process_id: Any) -> int:
            process_id._obj.value = 4242
            return {8001: 20, 7001: 30}[hwnd]

        @staticmethod
        def AttachThreadInput(_first: int, _second: int, _attach: bool) -> bool:
            return False

        @staticmethod
        def ShowWindow(_hwnd: int, _command: int) -> bool:
            return True

        @staticmethod
        def BringWindowToTop(_hwnd: int) -> bool:
            return False

        @staticmethod
        def SetForegroundWindow(hwnd: int) -> bool:
            foreground[0] = hwnd
            return False

        @staticmethod
        def SetFocus(_hwnd: int) -> int:
            return 0

    backend = object.__new__(playable_module._User32SendInputBackend)
    backend_fixture: Any = backend
    backend_fixture._ctypes = ctypes
    backend_fixture._wintypes = FakeWintypes
    backend_fixture._user32 = FakeUser32()
    backend_fixture._kernel32 = FakeKernel32()

    backend.focus_window(7001)

    assert backend.foreground_window() == 7001


class _FakeWindowsInputBackend:
    def __init__(self, *, pid: int = 4242, hwnd: int = 7001) -> None:
        self.pid = pid
        self.hwnd = hwnd
        self.foreground: int | None = None
        self.events: list[tuple[str, Any]] = []

    def top_level_windows(self, pid: int, *, title_fragment: str) -> tuple[Any, ...]:
        self.events.append(("windows", (pid, title_fragment)))
        if pid != self.pid:
            return ()
        return (
            playable_module._TopLevelWindowSnapshot(
                hwnd=self.hwnd,
                pid=self.pid,
                title="RobotSimUE (64 bit Development PCD3D_SM5)",
                visible=True,
                enabled=True,
                owner_hwnd=None,
                window_area=2_000_000,
                client_area=1_900_000,
            ),
        )

    def focus_window(self, hwnd: int) -> None:
        self.events.append(("focus", hwnd))
        self.foreground = hwnd

    def foreground_window(self) -> int | None:
        return self.foreground

    def window_process_id(self, hwnd: int) -> int | None:
        return self.pid if hwnd == self.hwnd else None

    def send_key(self, key: str, *, pressed: bool) -> None:
        self.events.append(("key", (key, pressed)))


def _window_snapshot(
    *,
    hwnd: int,
    pid: int = 4242,
    title: str = "RobotSimUE (64 bit Development PCD3D_SM5)",
    visible: bool = True,
    enabled: bool = True,
    owner_hwnd: int | None = None,
    window_area: int = 2_000_000,
    client_area: int = 1_900_000,
) -> Any:
    return playable_module._TopLevelWindowSnapshot(
        hwnd=hwnd,
        pid=pid,
        title=title,
        visible=visible,
        enabled=enabled,
        owner_hwnd=owner_hwnd,
        window_area=window_area,
        client_area=client_area,
    )


def test_prove_owned_robotsimue_window_presence_selects_unique_largest_without_focus_or_input() -> None:
    class PresenceOnlyBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            self.events.append(("windows", (pid, title_fragment)))
            return (
                _window_snapshot(
                    hwnd=7001,
                    title="RobotSimUE (64 bit Development PCD3D_SM5)   ",
                    window_area=3_000_000,
                    client_area=2_900_000,
                ),
                _window_snapshot(hwnd=7002, client_area=1_900_000),
            )

        def focus_window(self, _hwnd: int) -> None:
            raise AssertionError("window presence must not focus a window")

        def foreground_window(self) -> int | None:
            raise AssertionError("window presence must not query foreground")

        def send_key(self, _key: str, *, pressed: bool) -> None:
            raise AssertionError(f"window presence must not send input pressed={pressed}")

    backend = PresenceOnlyBackend()

    assert "OwnedRobotSimUEWindowPresenceProof" in playable_module.__all__
    assert "prove_owned_robotsimue_window_presence" in playable_module.__all__
    proof = playable_module.prove_owned_robotsimue_window_presence(
        4242,
        backend=backend,
        monotonic=lambda: 0.0,
        sleep=lambda _seconds: None,
    )

    assert proof.to_dict() == {
        "schema": "lingtu.sim.owned-robotsimue-window-presence-proof.v1",
        "owned_unreal_pid": 4242,
        "candidates": [
            {
                "hwnd": 7001,
                "pid": 4242,
                "title": "RobotSimUE (64 bit Development PCD3D_SM5)   ",
                "title_redacted": False,
                "visible": True,
                "enabled": True,
                "owner_hwnd": None,
                "window_area": 3_000_000,
                "client_area": 2_900_000,
                "eligible": True,
            },
            {
                "hwnd": 7002,
                "pid": 4242,
                "title": "RobotSimUE (64 bit Development PCD3D_SM5)",
                "title_redacted": False,
                "visible": True,
                "enabled": True,
                "owner_hwnd": None,
                "window_area": 2_000_000,
                "client_area": 1_900_000,
                "eligible": True,
            },
        ],
        "selected_hwnd": 7001,
        "selected_pid": 4242,
        "observed_at_unix_ns": proof.observed_at_unix_ns,
        "evidence_class": "manual_diagnostic_only",
        "qualification": False,
    }
    assert proof.selected_hwnd == 7001
    assert proof.selected_pid == 4242
    assert [event[0] for event in backend.events] == ["windows"]


def test_prove_owned_robotsimue_window_presence_redacts_nonselected_sensitive_title() -> None:
    sensitive_title = r"D:\private\RobotSimUE\UnrealEditor.exe"

    class MixedTitleBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            return (
                _window_snapshot(hwnd=7001, title="RobotSimUE"),
                _window_snapshot(
                    hwnd=7002,
                    title=sensitive_title,
                    client_area=9_000_000,
                ),
            )

    proof = playable_module.prove_owned_robotsimue_window_presence(
        4242,
        backend=MixedTitleBackend(),
        monotonic=lambda: 0.0,
        sleep=lambda _seconds: None,
    ).to_dict()

    assert proof["candidates"][0]["title"] == "RobotSimUE"
    assert proof["candidates"][0]["title_redacted"] is False
    assert proof["candidates"][1]["title"] is None
    assert proof["candidates"][1]["title_redacted"] is True
    assert sensitive_title not in json.dumps(proof)


def test_prove_owned_robotsimue_window_presence_rejects_equal_largest_without_focus() -> None:
    class TiedBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            self.events.append(("windows", (pid, title_fragment)))
            return (
                _window_snapshot(hwnd=7001, client_area=1_900_000),
                _window_snapshot(hwnd=7002, client_area=1_900_000),
            )

        def focus_window(self, _hwnd: int) -> None:
            raise AssertionError("presence tie must fail before focus")

    backend = TiedBackend()
    with pytest.raises(PlayableLifecycleError, match="equally large"):
        playable_module.prove_owned_robotsimue_window_presence(
            4242,
            backend=backend,
            monotonic=lambda: 0.0,
            sleep=lambda _seconds: None,
        )

    assert [event[0] for event in backend.events] == ["windows"]


def test_prove_owned_robotsimue_window_presence_rejects_pid_readback_without_focus() -> None:
    class WrongPidBackend(_FakeWindowsInputBackend):
        def focus_window(self, _hwnd: int) -> None:
            raise AssertionError("presence PID mismatch must fail before focus")

        def window_process_id(self, _hwnd: int) -> int | None:
            return 9999

    with pytest.raises(PlayableLifecycleError, match="PID does not match owned process"):
        playable_module.prove_owned_robotsimue_window_presence(
            4242,
            backend=WrongPidBackend(),
            monotonic=lambda: 0.0,
            sleep=lambda _seconds: None,
        )


def test_prove_owned_robotsimue_window_presence_rejects_sensitive_path_title_without_leak_or_input() -> None:
    sensitive_title = r"D:\private\RobotSimUE\UnrealEditor.exe"

    class SensitiveTitleBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            self.events.append(("windows", (pid, title_fragment)))
            return (_window_snapshot(hwnd=7001, title=sensitive_title),)

        def focus_window(self, _hwnd: int) -> None:
            raise AssertionError("ineligible title must fail before focus")

        def foreground_window(self) -> int | None:
            raise AssertionError("presence must not query foreground")

        def send_key(self, _key: str, *, pressed: bool) -> None:
            raise AssertionError(f"presence must not send input pressed={pressed}")

    backend = SensitiveTitleBackend()
    clock = iter((0.0, 1.0))
    with pytest.raises(PlayableLifecycleError) as captured:
        playable_module.prove_owned_robotsimue_window_presence(
            4242,
            backend=backend,
            monotonic=lambda: next(clock),
            sleep=lambda _seconds: None,
            timeout_s=0.5,
        )

    assert "title_path_or_executable=1" in str(captured.value)
    assert sensitive_title not in str(captured.value)
    assert [event[0] for event in backend.events] == ["windows"]


def test_prove_owned_robotsimue_foreground_returns_strict_non_qualification_evidence_without_input() -> None:
    class MultipleOwnedWindowsBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            self.events.append(("windows", (pid, title_fragment)))
            return (
                _window_snapshot(
                    hwnd=7002,
                    title=r"D:\RobotSimUE\Engine\Binaries\Win64\UnrealEditor.exe",
                    window_area=3_000_000,
                    client_area=2_900_000,
                ),
                _window_snapshot(hwnd=self.hwnd),
            )

    backend = MultipleOwnedWindowsBackend()

    assert "OwnedRobotSimUEWindowProof" in playable_module.__all__
    assert "prove_owned_robotsimue_foreground" in playable_module.__all__

    proof = playable_module.prove_owned_robotsimue_foreground(
        4242,
        backend=backend,
        sleep=lambda _seconds: None,
        monotonic=lambda: 1.0,
        timeout_s=0.1,
    )

    payload = proof.to_dict()
    assert payload == {
        "schema": "lingtu.sim.owned-robotsimue-window-proof.v1",
        "owned_unreal_pid": 4242,
        "candidates": [
            {
                "hwnd": 7002,
                "pid": 4242,
                "title": r"D:\RobotSimUE\Engine\Binaries\Win64\UnrealEditor.exe",
                "visible": True,
                "enabled": True,
                "owner_hwnd": None,
                "window_area": 3_000_000,
                "client_area": 2_900_000,
                "eligible": False,
            },
            {
                "hwnd": 7001,
                "pid": 4242,
                "title": "RobotSimUE (64 bit Development PCD3D_SM5)",
                "visible": True,
                "enabled": True,
                "owner_hwnd": None,
                "window_area": 2_000_000,
                "client_area": 1_900_000,
                "eligible": True,
            },
        ],
        "selected_hwnd": 7001,
        "foreground_hwnd": 7001,
        "foreground_pid": 4242,
        "observed_at_unix_ns": payload["observed_at_unix_ns"],
        "evidence_class": "manual_diagnostic_only",
        "qualification": False,
    }
    assert isinstance(payload["observed_at_unix_ns"], int)
    assert payload["observed_at_unix_ns"] > 0
    assert proof.evidence_class == "manual_diagnostic_only"
    assert proof.qualification is False
    json.dumps(payload, allow_nan=False, sort_keys=True)
    assert ("focus", 7001) in backend.events
    assert not any(name == "key" for name, _value in backend.events)


def test_prove_owned_robotsimue_foreground_accepts_owned_title_trailing_spaces_without_input() -> None:
    class TrailingSpaceTitleBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert title_fragment == "RobotSimUE"
            return (
                _window_snapshot(
                    hwnd=self.hwnd,
                    pid=pid,
                    title="RobotSimUE (64 bit Development PCD3D_SM5)   ",
                ),
            )

    backend = TrailingSpaceTitleBackend()
    now = [0.0]

    def advance(seconds: float) -> None:
        now[0] += seconds

    proof = playable_module.prove_owned_robotsimue_foreground(
        4242,
        backend=backend,
        sleep=advance,
        monotonic=lambda: now[0],
        timeout_s=0.01,
    )

    assert proof.selected_hwnd == backend.hwnd
    assert proof.candidates[0].eligible is True
    assert proof.candidates[0].title.endswith("   ")
    assert not any(name == "key" for name, _value in backend.events)


def test_prove_owned_robotsimue_foreground_rejects_leading_or_nonspace_trailing_whitespace() -> None:
    class InvalidWhitespaceTitleBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert title_fragment == "RobotSimUE"
            return (
                _window_snapshot(
                    hwnd=self.hwnd,
                    pid=pid,
                    title=" RobotSimUE (64 bit Development PCD3D_SM5)",
                ),
                _window_snapshot(
                    hwnd=7002,
                    pid=pid,
                    title="RobotSimUE (64 bit Development PCD3D_SM5)\t",
                ),
                _window_snapshot(
                    hwnd=7003,
                    pid=pid,
                    title="RobotSimUE (64 bit Development PCD3D_SM5)\x07   ",
                ),
            )

    backend = InvalidWhitespaceTitleBackend()
    now = [0.0]

    def advance(seconds: float) -> None:
        now[0] += seconds

    with pytest.raises(PlayableLifecycleError, match="was not found") as captured:
        playable_module.prove_owned_robotsimue_foreground(
            4242,
            backend=backend,
            sleep=advance,
            monotonic=lambda: now[0],
            timeout_s=0.01,
        )

    assert "title_leading_whitespace=1" in str(captured.value)
    assert "title_control_or_nonspace_whitespace=2" in str(captured.value)
    assert not any(name in {"focus", "key"} for name, _value in backend.events)


@pytest.mark.parametrize(
    "title",
    [
        "RobotSimUE\tDevelopment",
        "RobotSimUE\nDevelopment",
        "RobotSimUE\rDevelopment",
        "RobotSimUE\x00Development",
        "RobotSimUE\x07Development",
        "RobotSimUE\u200bDevelopment",
        "RobotSimUE\u00a0Development",
    ],
)
def test_robotsimue_game_window_title_rejects_any_control_format_or_nonspace_whitespace(
    title: str,
) -> None:
    assert (
        playable_module._is_eligible_robotsimue_game_window(
            _window_snapshot(hwnd=7001, title=title),
            expected_pid=4242,
        )
        is False
    )


def test_prove_owned_robotsimue_foreground_retries_missing_window_within_bound_without_input() -> None:
    class DelayedWindowBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            self.events.append(("windows", (pid, title_fragment)))
            attempts = sum(name == "windows" for name, _value in self.events)
            if attempts == 1:
                return (
                    _window_snapshot(
                        hwnd=7002,
                        title=r"D:\RobotSimUE\Engine\Binaries\Win64\UnrealEditor.exe",
                    ),
                )
            return (_window_snapshot(hwnd=self.hwnd),)

    backend = DelayedWindowBackend()
    now = [0.0]

    def advance(seconds: float) -> None:
        now[0] += seconds

    proof = playable_module.prove_owned_robotsimue_foreground(
        4242,
        backend=backend,
        sleep=advance,
        monotonic=lambda: now[0],
        timeout_s=0.1,
    )

    assert proof.selected_hwnd == 7001
    assert [candidate.hwnd for candidate in proof.candidates] == [7002, 7001]
    assert [name for name, _value in backend.events].count("windows") == 2
    assert now[0] == pytest.approx(0.05)
    assert not any(name == "key" for name, _value in backend.events)


def test_prove_owned_robotsimue_foreground_rejects_equal_largest_candidates_without_input() -> None:
    class EqualLargestBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            self.events.append(("windows", (pid, title_fragment)))
            return (
                _window_snapshot(hwnd=self.hwnd),
                _window_snapshot(hwnd=7003),
            )

    backend = EqualLargestBackend()

    with pytest.raises(PlayableLifecycleError, match="equally large"):
        playable_module.prove_owned_robotsimue_foreground(
            4242,
            backend=backend,
            sleep=lambda _seconds: None,
            monotonic=lambda: 1.0,
            timeout_s=0.1,
        )

    assert not any(name in {"focus", "key"} for name, _value in backend.events)


def test_prove_owned_robotsimue_foreground_rejects_selected_window_pid_readback_before_focus() -> None:
    class StaleOwnedWindowBackend(_FakeWindowsInputBackend):
        def window_process_id(self, hwnd: int) -> int | None:
            assert hwnd == self.hwnd
            return 9876

    backend = StaleOwnedWindowBackend()

    with pytest.raises(
        PlayableLifecycleError,
        match="window PID does not match owned process",
    ):
        playable_module.prove_owned_robotsimue_foreground(
            4242,
            backend=backend,
            sleep=lambda _seconds: None,
            monotonic=lambda: 1.0,
            timeout_s=0.1,
        )

    assert not any(name in {"focus", "key"} for name, _value in backend.events)


@pytest.mark.parametrize(
    ("failure_mode", "message"),
    [
        ("missing", "foreground query returned no window"),
        ("foreign", "owned foreground"),
        ("wrong_pid", "foreground window PID does not match owned process"),
    ],
)
def test_prove_owned_robotsimue_foreground_fails_closed_on_foreground_readback_without_input(
    failure_mode: str,
    message: str,
) -> None:
    class BrokenForegroundBackend(_FakeWindowsInputBackend):
        def __init__(self) -> None:
            super().__init__()
            self.pid_reads = 0

        def focus_window(self, hwnd: int) -> None:
            self.events.append(("focus", hwnd))
            if failure_mode == "missing":
                self.foreground = None
            elif failure_mode == "foreign":
                self.foreground = 9999
            else:
                self.foreground = hwnd

        def window_process_id(self, hwnd: int) -> int | None:
            self.pid_reads += 1
            if failure_mode == "wrong_pid" and self.pid_reads > 1:
                return 9876
            return super().window_process_id(hwnd)

    backend = BrokenForegroundBackend()
    now = [0.0]

    def advance(seconds: float) -> None:
        now[0] += seconds

    with pytest.raises(PlayableLifecycleError, match=message):
        playable_module.prove_owned_robotsimue_foreground(
            4242,
            backend=backend,
            sleep=advance,
            monotonic=lambda: now[0],
            timeout_s=0.01,
        )

    assert now[0] == pytest.approx(0.01)
    assert len([event for event in backend.events if event[0] == "focus"]) == 2
    assert not any(name == "key" for name, _value in backend.events)


def _bind_window_until_status_wait(
    tmp_path: Path,
    backend: _FakeWindowsInputBackend,
) -> None:
    now = [0.0]

    def advance(seconds: float) -> None:
        now[0] += seconds

    driver = OwnedRobotSimUEInput(
        backend=backend,
        sleep=advance,
        monotonic=lambda: now[0],
        evidence_timeout_s=0.01,
    )
    with pytest.raises(PlayableLifecycleError, match="authoritative"):
        driver.perform_actions(_frame_wait_context(tmp_path))


def test_owned_window_sendinput_selects_game_window_not_path_title(
    tmp_path: Path,
) -> None:
    class MultipleOwnedWindowsBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert pid == self.pid
            assert title_fragment == "RobotSimUE"
            return (
                _window_snapshot(
                    hwnd=7002,
                    title=r"D:\RobotSimUE\Engine\Binaries\Win64\UnrealEditor.exe",
                    window_area=3_000_000,
                    client_area=2_900_000,
                ),
                _window_snapshot(
                    hwnd=self.hwnd,
                    window_area=2_000_000,
                    client_area=1_900_000,
                ),
            )

    backend = MultipleOwnedWindowsBackend()
    _bind_window_until_status_wait(tmp_path, backend)

    assert ("focus", backend.hwnd) in backend.events
    assert ("focus", 7002) not in backend.events


def test_owned_window_sendinput_selects_largest_interactive_game_window(
    tmp_path: Path,
) -> None:
    class CandidateBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert pid == self.pid
            assert title_fragment == "RobotSimUE"
            return (
                _window_snapshot(hwnd=7101, visible=False, window_area=9_000_000),
                _window_snapshot(hwnd=7102, enabled=False, window_area=8_000_000),
                _window_snapshot(hwnd=7103, owner_hwnd=99, window_area=7_000_000),
                _window_snapshot(hwnd=7104, window_area=0),
                _window_snapshot(hwnd=7105, client_area=0),
                _window_snapshot(hwnd=7106, pid=9876, window_area=6_000_000),
                _window_snapshot(
                    hwnd=7107,
                    title="RobotSimUE（64 位开发版 PCD3D_SM5）",
                    window_area=1_000_000,
                    client_area=900_000,
                ),
                _window_snapshot(hwnd=self.hwnd, window_area=2_000_000),
            )

    backend = CandidateBackend()
    _bind_window_until_status_wait(tmp_path, backend)

    assert ("focus", backend.hwnd) in backend.events
    assert not any(name == "focus" and value != backend.hwnd for name, value in backend.events)


def test_owned_window_sendinput_ranks_game_windows_by_client_area(
    tmp_path: Path,
) -> None:
    class ClientAreaBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert pid == self.pid
            assert title_fragment == "RobotSimUE"
            return (
                _window_snapshot(
                    hwnd=7003,
                    window_area=3_000_000,
                    client_area=1_000_000,
                ),
                _window_snapshot(
                    hwnd=self.hwnd,
                    window_area=2_000_000,
                    client_area=1_900_000,
                ),
            )

    backend = ClientAreaBackend()
    _bind_window_until_status_wait(tmp_path, backend)

    assert ("focus", backend.hwnd) in backend.events
    assert ("focus", 7003) not in backend.events


def test_owned_window_sendinput_rejects_equal_largest_game_windows(
    tmp_path: Path,
) -> None:
    class EqualLargestBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert pid == self.pid
            assert title_fragment == "RobotSimUE"
            return (
                _window_snapshot(hwnd=self.hwnd, window_area=2_000_000),
                _window_snapshot(hwnd=7003, window_area=2_000_000),
            )

    backend = EqualLargestBackend()
    driver = OwnedRobotSimUEInput(
        backend=backend,
        sleep=lambda _seconds: None,
        monotonic=lambda: 1.0,
    )

    with pytest.raises(PlayableLifecycleError, match="equally large"):
        driver.perform_actions(_frame_wait_context(tmp_path))

    assert not any(name == "focus" for name, _value in backend.events)


def test_owned_window_sendinput_accepts_localized_game_title_suffix(
    tmp_path: Path,
) -> None:
    class LocalizedGameWindowBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert pid == self.pid
            assert title_fragment == "RobotSimUE"
            return (
                _window_snapshot(
                    hwnd=self.hwnd,
                    title="RobotSimUE（64 位开发版 PCD3D_SM5）",
                ),
            )

    backend = LocalizedGameWindowBackend()
    _bind_window_until_status_wait(tmp_path, backend)

    assert ("focus", backend.hwnd) in backend.events


def test_owned_window_sendinput_never_falls_back_to_generic_or_path_window(
    tmp_path: Path,
) -> None:
    class NonGameWindowsBackend(_FakeWindowsInputBackend):
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert pid == self.pid
            assert title_fragment == "RobotSimUE"
            return (
                _window_snapshot(
                    hwnd=self.hwnd,
                    title=r"D:\RobotSimUE\Engine\Binaries\Win64\UnrealEditor.exe",
                ),
                _window_snapshot(
                    hwnd=7002,
                    title="Unreal Editor - RobotSimUE",
                ),
                _window_snapshot(
                    hwnd=7003,
                    title="RobotSimUE.exe",
                ),
                _window_snapshot(
                    hwnd=7004,
                    title="robotsimue (64 bit Development PCD3D_SM5)",
                ),
            )

    backend = NonGameWindowsBackend()
    now = [0.0]

    def advance(seconds: float) -> None:
        now[0] += seconds

    driver = OwnedRobotSimUEInput(
        backend=backend,
        sleep=advance,
        monotonic=lambda: now[0],
        focus_timeout_s=0.01,
    )

    with pytest.raises(PlayableLifecycleError, match="was not found") as captured:
        driver.perform_actions(_frame_wait_context(tmp_path))

    assert "observed_candidates=4" in str(captured.value)
    assert "title_mismatch=4" in str(captured.value)
    assert "title_path_or_executable=2" in str(captured.value)
    assert r"D:\RobotSimUE" not in str(captured.value)
    assert "Unreal Editor - RobotSimUE" not in str(captured.value)
    assert not any(name == "focus" for name, _value in backend.events)


def test_owned_window_sendinput_drives_fixed_keys_and_menu_exit_only_in_owned_pid(
    tmp_path: Path,
) -> None:
    class ImmediateEvidenceInput(OwnedRobotSimUEInput):
        def __init__(self, **kwargs: Any) -> None:
            super().__init__(**kwargs)
            self.status_sequence = 0

        def _wait_for_status(
            self,
            _context: PlayableActionContext,
            predicate: Callable[[Mapping[str, Any]], bool],
            *,
            description: str,
            after_sequence: int = 0,
        ) -> Mapping[str, Any]:
            del description
            candidates = (
                ("RUNNING", "idle", "drive", "follow"),
                ("RUNNING", "recording", "drive", "follow"),
                ("RUNNING", "recording", "drive", "inspection"),
                ("RUNNING", "recording", "tactical", "inspection"),
                ("RUNNING", "recording", "menu", "inspection"),
                ("RUNNING", "committed", "drive", "inspection"),
                ("RUNNING", "committed", "menu", "inspection"),
            )
            for sequence, (runtime_state, recording, ui_mode, camera) in enumerate(
                candidates,
                1,
            ):
                status = {
                    "server_status_sequence": sequence,
                    "sim_time_ns": sequence * 1_000_000_000,
                    "runtime": {"runtime_state": runtime_state},
                    "recording": {"state": recording},
                    "ui": {"ui_mode": ui_mode, "camera_mode": camera},
                }
                if sequence > max(after_sequence, self.status_sequence) and predicate(status):
                    self.status_sequence = sequence
                    return status
            raise AssertionError("test status schedule did not satisfy predicate")

        def _wait_for_hud_capture(
            self,
            _context: PlayableActionContext,
            _key: str,
        ) -> None:
            return

        def _wait_for_file(self, _path: Path) -> None:
            return

        def _wait_for_recording_frame_window(
            self,
            _context: PlayableActionContext,
            *,
            recording_start_sim_time_ns: int,
        ) -> None:
            assert recording_start_sim_time_ns == 2_000_000_000
            return

    backend = _FakeWindowsInputBackend()
    sleeps: list[float] = []
    driver = ImmediateEvidenceInput(
        backend=backend,
        sleep=lambda seconds: sleeps.append(seconds),
        monotonic=lambda: 1.0,
    )
    context = PlayableActionContext(
        run_id="playable-test",
        boot_id="playable-boot-test",
        session_id=_SESSION_ID,
        run_dir=tmp_path,
        unreal_pid=4242,
        deadman_key="SHIFT",
        actions=PLAYABLE_INPUT_SCHEDULE,
        hud_screenshot_paths={},
        runtime_surface=playable_module.PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME,
        unreal_log_path=tmp_path / "logs" / "Unreal.log",
        frame_capture_wait_timeout_s=900.0,
    )

    driver.perform_actions(context)
    driver.request_exit(context)

    assert driver.status_sequence == 7
    key_events = [value for name, value in backend.events if name == "key"]
    assert key_events[:2] == [("R", True), ("R", False)]
    assert key_events[2:6] == [("SHIFT", True), ("W", True), ("W", False), ("SHIFT", False)]
    assert [(key, True) for key in ("W", "S", "A", "D", "Q", "E")] == [
        event for event in key_events if event[1] is True and event[0] in {"W", "S", "A", "D", "Q", "E"}
    ]
    assert key_events[-4:] == [
        ("ESCAPE", True),
        ("ESCAPE", False),
        ("X", True),
        ("X", False),
    ]
    assert ("C", True) in key_events
    assert 5.3 in sleeps
    assert all(value == (4242, "RobotSimUE") for name, value in backend.events if name == "windows")


def test_owned_window_sendinput_rejects_foreign_foreground_before_any_key(
    tmp_path: Path,
) -> None:
    class ForeignForegroundBackend(_FakeWindowsInputBackend):
        def focus_window(self, hwnd: int) -> None:
            self.events.append(("focus", hwnd))
            self.foreground = 9999

    backend = ForeignForegroundBackend()
    now = [0.0]

    def advance(seconds: float) -> None:
        now[0] += seconds

    driver = OwnedRobotSimUEInput(
        backend=backend,
        sleep=advance,
        monotonic=lambda: now[0],
        focus_timeout_s=0.01,
    )
    context = PlayableActionContext(
        run_id="playable-test",
        boot_id="playable-boot-test",
        session_id=_SESSION_ID,
        run_dir=tmp_path,
        unreal_pid=4242,
        deadman_key="SHIFT",
        actions=PLAYABLE_INPUT_SCHEDULE,
        hud_screenshot_paths={},
        runtime_surface=playable_module.PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME,
        unreal_log_path=tmp_path / "logs" / "Unreal.log",
        frame_capture_wait_timeout_s=900.0,
    )

    with pytest.raises(PlayableLifecycleError, match="owned foreground"):
        driver.perform_actions(context)

    assert not any(name == "key" for name, _value in backend.events)


def test_owned_window_sendinput_retries_transient_foreground_denial(
    tmp_path: Path,
) -> None:
    class TransientForegroundBackend(_FakeWindowsInputBackend):
        def focus_window(self, hwnd: int) -> None:
            self.events.append(("focus", hwnd))
            focus_count = sum(name == "focus" for name, _value in self.events)
            self.foreground = None if focus_count == 1 else hwnd

    backend = TransientForegroundBackend()
    _bind_window_until_status_wait(tmp_path, backend)

    assert [value for name, value in backend.events if name == "focus"] == [
        backend.hwnd,
        backend.hwnd,
    ]


def test_owned_window_sendinput_rejects_missing_foreground_without_type_error(
    tmp_path: Path,
) -> None:
    class MissingForegroundBackend(_FakeWindowsInputBackend):
        def focus_window(self, hwnd: int) -> None:
            self.events.append(("focus", hwnd))
            self.foreground = None

    backend = MissingForegroundBackend()
    now = [0.0]

    def advance(seconds: float) -> None:
        now[0] += seconds

    driver = OwnedRobotSimUEInput(
        backend=backend,
        sleep=advance,
        monotonic=lambda: now[0],
        focus_timeout_s=0.01,
    )

    with pytest.raises(
        PlayableLifecycleError,
        match="foreground query returned no window",
    ):
        driver.perform_actions(_frame_wait_context(tmp_path))

    assert not any(name == "key" for name, _value in backend.events)


def _write_minimal_1080p_png(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n" + (13).to_bytes(4, "big") + b"IHDR" + (1920).to_bytes(4, "big") + (1080).to_bytes(4, "big")
    )


def _frame_wait_context(tmp_path: Path) -> PlayableActionContext:
    return PlayableActionContext(
        run_id="playable-test",
        boot_id="playable-boot-test",
        session_id=_SESSION_ID,
        run_dir=tmp_path,
        unreal_pid=4242,
        deadman_key="SHIFT",
        actions=PLAYABLE_INPUT_SCHEDULE,
        hud_screenshot_paths={},
        runtime_surface=playable_module.PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME,
        unreal_log_path=tmp_path / "logs" / "Unreal.log",
        frame_capture_wait_timeout_s=900.0,
    )


def test_recording_frame_counter_uses_unreal_log_after_recording_start(
    tmp_path: Path,
) -> None:
    context = _frame_wait_context(tmp_path)
    context.unreal_log_path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for index in range(601):
        frame = context.run_dir / "frames" / f"frame_{index:06d}.png"
        _write_minimal_1080p_png(frame)
        sim_time_ns = 1_000_000_000 + index * 33_333_333
        lines.append(
            "LINGTU_VISUAL_FRAME_CAPTURE_REQUESTED "
            f"capture_index={index} model_generation=0 reset_generation=0 "
            f"sequence={index + 1} sim_time_ns={sim_time_ns} path={frame.resolve()} "
            f"requested={index + 1} max=2000"
        )
    context.unreal_log_path.write_text("\n".join(lines), encoding="utf-8")

    count = playable_module._count_mapped_recording_frames(
        context,
        recording_start_sim_time_ns=1_000_000_001,
    )

    assert count == 600


def test_five_stream_status_must_be_active_current_and_unblocked() -> None:
    status = {
        "sensors": [
            {
                "stream_id": stream_id,
                "state": "ACTIVE",
                "sample_count": 1,
                "blocker": "",
            }
            for stream_id in playable_module.PLAYABLE_SENSOR_RATES
        ]
    }
    assert playable_module._status_has_current_five_streams(status)
    status["sensors"][0]["sample_count"] = 0
    assert not playable_module._status_has_current_five_streams(status)


def test_runner_waits_for_natural_zero_unreal_exit_and_rejects_timeout() -> None:
    class PollingUnreal:
        def __init__(self, values: list[int | None]) -> None:
            self.values = values

        def poll(self) -> int | None:
            return self.values.pop(0) if len(self.values) > 1 else self.values[0]

    clock_values = iter((1.0, 1.01))
    sleeps: list[float] = []
    playable_module._wait_for_owned_unreal_exit(
        PollingUnreal([None, 0]),
        timeout_s=1.0,
        poll_interval_s=0.1,
        monotonic=lambda: next(clock_values),
        sleep=lambda value: sleeps.append(value),
    )
    assert sleeps == [0.1]

    class FinalizedUnreal:
        last_shutdown = ProcessShutdownSnapshot(
            pid=4242,
            exit_code=0,
            direct_child_running_after_close=False,
            process_owner_closed=True,
            termination_mode="natural",
        )

        def poll(self) -> int | None:
            raise AssertionError("finalized process must not be polled again")

    playable_module._wait_for_owned_unreal_exit(
        FinalizedUnreal(),
        timeout_s=1.0,
        poll_interval_s=0.1,
        monotonic=lambda: 4.0,
        sleep=lambda _value: None,
    )

    class ForcedUnreal(FinalizedUnreal):
        last_shutdown = ProcessShutdownSnapshot(
            pid=4242,
            exit_code=1,
            direct_child_running_after_close=False,
            process_owner_closed=True,
            termination_mode="owned_terminate",
        )

    with pytest.raises(PlayableLifecycleError, match="not natural zero-exit"):
        playable_module._wait_for_owned_unreal_exit(
            ForcedUnreal(),
            timeout_s=1.0,
            poll_interval_s=0.1,
            monotonic=lambda: 4.0,
            sleep=lambda _value: None,
        )

    timeout_clock = iter((2.0, 2.5, 3.0))
    with pytest.raises(PlayableLifecycleError, match="did not exit naturally"):
        playable_module._wait_for_owned_unreal_exit(
            PollingUnreal([None]),
            timeout_s=1.0,
            poll_interval_s=0.5,
            monotonic=lambda: next(timeout_clock),
            sleep=lambda _value: None,
        )


def test_default_product_qualification_writes_rejection_and_cannot_return_success(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "rejected-run"
    run_dir.mkdir()
    closed = PlayableClosedRun(
        run_id="playable-test",
        boot_id="playable-boot-test",
        session_id=_SESSION_ID,
        run_dir=run_dir,
        unreal_log_path=run_dir / "logs" / "Unreal.log",
        frames=(),
        hud_screenshot_paths={},
        exit_event_id=None,
        media_toolchain=_fake_pinned_toolchain(
            tmp_path / "ffmpeg.exe",
            tmp_path / "ffprobe.exe",
        ),
    )

    with pytest.raises(
        PlayableLifecycleError,
        match="qualification rejected evidence",
    ):
        playable_module._write_default_qualification(closed)

    verdict = json.loads((run_dir / "playable-qualification.json").read_text(encoding="utf-8"))
    assert verdict["result"] == "EVIDENCE_REJECTED"
    assert verdict["qualified"] is False


def test_production_cli_double_loads_equal_bundle_before_construction_and_uses_real_defaults(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    bundle = _fixed_bundle(tmp_path)
    paths = {
        name: tmp_path / name
        for name in (
            "mujoco.exe",
            "UnrealEditor.exe",
            "RobotSimUE.uproject",
            "imu.exe",
            "truth.exe",
            "mid360.exe",
            "ffmpeg.exe",
            "ffprobe.exe",
        )
    }
    for path in paths.values():
        path.write_bytes(b"owned-cli-input")
    events: list[tuple[str, Any]] = []

    def load_bundle(path: Path, *, repo_root: Path) -> ResolvedSessionBundle:
        events.append(("load", (Path(path), repo_root)))
        return bundle

    launch = object()

    def create(
        actual: ResolvedSessionBundle,
        *,
        runtime: PlayableRuntimeConfig,
    ) -> object:
        events.append(("create", (actual, runtime)))
        return launch

    verdict_path = tmp_path / "runs" / "playable-test" / "playable-qualification.json"

    def run(actual: object, **kwargs: Any) -> Path:
        events.append(("run", (actual, kwargs)))
        return verdict_path

    monkeypatch.setattr(playable_module, "load_resolved_session_bundle", load_bundle)
    monkeypatch.setattr(playable_module, "create_playable_launch", create)
    monkeypatch.setattr(playable_module, "run_playable_vertical_slice", run)
    pinned_toolchain = _fake_pinned_toolchain(
        paths["ffmpeg.exe"],
        paths["ffprobe.exe"],
    )
    monkeypatch.setattr(
        playable_module,
        "snapshot_playable_media_toolchain",
        lambda **_kwargs: pinned_toolchain,
    )

    result = playable_module.main(
        [
            str(bundle.bundle_dir),
            "--repo-root",
            str(bundle.repo_root),
            "--run-root",
            str(tmp_path / "runs"),
            "--mujoco-host",
            str(paths["mujoco.exe"]),
            "--unreal-editor-executable",
            str(paths["UnrealEditor.exe"]),
            "--unreal-project",
            str(paths["RobotSimUE.uproject"]),
            "--imu-publisher",
            str(paths["imu.exe"]),
            "--truth-odom-publisher",
            str(paths["truth.exe"]),
            "--mid360-publisher",
            str(paths["mid360.exe"]),
            "--ffmpeg",
            str(paths["ffmpeg.exe"]),
            "--ffprobe",
            str(paths["ffprobe.exe"]),
            "--visual-snapshot-port",
            "25123",
            "--control-intent-port",
            "25124",
            "--control-status-port",
            "25125",
            "--run-id",
            "playable-test",
        ]
    )

    assert result == 0
    assert [name for name, _value in events] == ["load", "load", "create", "run"]
    _, (_, runtime) = events[2]
    assert isinstance(runtime, PlayableRuntimeConfig)
    assert runtime.ports == _PORTS
    assert runtime.runtime_surface == playable_module.PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME
    assert runtime.unreal_editor_executable == paths["UnrealEditor.exe"]
    assert runtime.unreal_project == paths["RobotSimUE.uproject"]
    assert runtime.robotsimue_executable is None
    assert runtime.media_toolchain is pinned_toolchain
    _, (actual_launch, run_kwargs) = events[3]
    assert actual_launch is launch
    assert run_kwargs == {
        "natural_exit_timeout_s": 30.0,
        "unreal_exit_timeout_s": 15.0,
    }
    output = json.loads(capsys.readouterr().out)
    assert output["result"] == "PASS"


def test_production_cli_rejects_bundle_mutation_before_any_process_constructor(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    bundle = _fixed_bundle(tmp_path)
    paths = [
        tmp_path / name
        for name in (
            "m.exe",
            "UnrealEditor.exe",
            "RobotSimUE.uproject",
            "i.exe",
            "t.exe",
            "l.exe",
            "ffmpeg.exe",
            "ffprobe.exe",
        )
    ]
    for path in paths:
        path.write_bytes(b"cli")
    loads = 0
    constructed = False

    def load_bundle(_path: Path, *, repo_root: Path) -> ResolvedSessionBundle:
        nonlocal loads
        loads += 1
        assert repo_root == bundle.repo_root
        if loads == 1:
            return bundle
        changed_session = copy.deepcopy(dict(bundle.session_spec))
        changed_session["prelaunch_mutation"] = True
        (bundle.bundle_dir / "session.yaml").write_text(
            json.dumps(changed_session),
            encoding="utf-8",
        )
        return replace(bundle, session_spec=changed_session)

    def create(*_args: Any, **_kwargs: Any) -> object:
        nonlocal constructed
        constructed = True
        return object()

    monkeypatch.setattr(playable_module, "load_resolved_session_bundle", load_bundle)
    monkeypatch.setattr(playable_module, "create_playable_launch", create)

    result = playable_module.main(
        [
            str(bundle.bundle_dir),
            "--repo-root",
            str(bundle.repo_root),
            "--mujoco-host",
            str(paths[0]),
            "--unreal-editor-executable",
            str(paths[1]),
            "--unreal-project",
            str(paths[2]),
            "--imu-publisher",
            str(paths[3]),
            "--truth-odom-publisher",
            str(paths[4]),
            "--mid360-publisher",
            str(paths[5]),
            "--ffmpeg",
            str(paths[6]),
            "--ffprobe",
            str(paths[7]),
            "--visual-snapshot-port",
            "25123",
            "--control-intent-port",
            "25124",
            "--control-status-port",
            "25125",
        ]
    )

    assert result == 1
    assert loads == 2
    assert constructed is False
    error = json.loads(capsys.readouterr().err)
    assert error["result"] == "EVIDENCE_REJECTED"
    assert "changed during pre-launch validation" in error["error"]


def _exit_trace_documents() -> tuple[dict[str, Any], dict[str, Any]]:
    common = {
        "run_id": "playable-test",
        "session_id": _SESSION_ID,
        "boot_id": "playable-boot-test",
        "model_generation": 0,
        "reset_generation": 0,
        "source_id": "robotsimue.local_player.0",
        "source_epoch": "epoch-1",
        "source_sequence": 7,
        "event_id": "exit-1",
        "source_monotonic_ns": 100,
        "arrival_monotonic_ns": 101,
        "datagram_sha256": "b" * 64,
    }
    return (
        {
            "schema": "lingtu.sim.runtime-request-trace.v1",
            "event": "runtime_request_accepted",
            **common,
            "request": "exit",
            "status": "accepted",
            "reason": "",
        },
        {
            "schema": "lingtu.sim.control-command-zero.v1",
            "event": "control_command_zero",
            **common,
            "controller_sequence": 8,
            "apply_time_ns": 200,
            "submit_result": "accepted",
            "admitted_twist": {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "angular_z": 0.0,
            },
            "reason": "cleared:exit",
        },
    )


def _origin_document() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.ue-control-origin.v1",
        "run_id": "playable-test",
        "session_id": _SESSION_ID,
        "boot_id": "playable-boot-test",
        "model_generation": 0,
        "reset_generation": 0,
        "source_id": "robotsimue.local_player.0",
        "source_epoch": "epoch-1",
        "source_sequence": 7,
        "event_id": "exit-1",
        "datagram_sha256": "b" * 64,
        "datagram_bytes": 256,
        "successful_send": True,
    }
