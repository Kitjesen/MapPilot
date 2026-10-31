"""Pure Python contracts for measurement presentation launch binding."""

from __future__ import annotations

from dataclasses import FrozenInstanceError, replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
import sim.runtime.coordinator.playable_vertical_slice as playable
from sim.runtime.coordinator.playable_evidence import PinnedPlayableMediaToolchain
from sim.runtime.coordinator.run_allocation import ResolvedSessionBundle
from sim.runtime.coordinator.unreal_process import (
    InspectionPresentationBinding,
    MeasurementPresentationBinding,
    PackagedUnrealProcess,
    UnrealLaunchProfile,
    UnrealProcess,
)


@pytest.mark.parametrize(
    "gateway_url",
    [
        "http://127.0.0.1:8000",
        "https://gateway.example.test:8443",
        "http://127.0.0.1:8000/gateway/",
        "https://gateway.example.test:8443/gateway/",
    ],
)
def test_measurement_binding_arguments_preserve_the_supplied_values(gateway_url: str) -> None:
    binding = MeasurementPresentationBinding(
        product_session_id="product-measurement-42",
        gateway_url=gateway_url,
    )

    assert binding.arguments() == [
        "-LingTuMeasurementProductSessionId=product-measurement-42",
        f"-LingTuMeasurementGatewayUrl={gateway_url}",
    ]


@pytest.mark.parametrize("suffix", ["?view=live", "?", "#live", "#"])
def test_measurement_binding_rejects_query_and_fragment_delimiters(suffix: str) -> None:
    with pytest.raises(ValueError, match="gateway_url"):
        MeasurementPresentationBinding(
            product_session_id="product-measurement-42",
            gateway_url=f"https://gateway.example.test:8443/api/{suffix}",
        )


@pytest.mark.parametrize("userinfo", ["token", "user:token", ":token", ""])
def test_measurement_binding_rejects_url_userinfo(userinfo: str) -> None:
    with pytest.raises(ValueError, match="gateway_url"):
        MeasurementPresentationBinding(
            product_session_id="product-measurement-42",
            gateway_url=f"https://{userinfo}@gateway.example.test:8443/gateway/",
        )


@pytest.mark.parametrize(
    "product_session_id",
    [None, 42, "", "   ", " product-42", "product-42 ", "product 42", "../product", "a" * 64],
)
def test_measurement_binding_rejects_noncanonical_product_identity(
    product_session_id: object,
) -> None:
    with pytest.raises(ValueError, match="product_session_id"):
        MeasurementPresentationBinding(
            product_session_id=product_session_id,  # type: ignore[arg-type]
            gateway_url="http://127.0.0.1:8000",
        )


@pytest.mark.parametrize(
    "gateway_url",
    [
        None,
        8000,
        "",
        "   ",
        "gateway.example.test",
        "ftp://gateway.example.test",
        "http:///missing-host",
        "https://:8000",
        "http://gateway.example.test:not-a-port",
        "http://gateway.example.test:65536",
        " http://127.0.0.1:8000",
        "http://gate way.example.test",
        "http://gateway.example.test\n",
        "http://[::1",
    ],
)
def test_measurement_binding_requires_an_actual_http_or_https_url(
    gateway_url: object,
) -> None:
    with pytest.raises(ValueError, match="gateway_url"):
        MeasurementPresentationBinding(
            product_session_id="product-measurement-42",
            gateway_url=gateway_url,  # type: ignore[arg-type]
        )


@pytest.mark.parametrize("field", ["product_session_id", "gateway_url"])
def test_measurement_binding_is_frozen(field: str) -> None:
    binding = MeasurementPresentationBinding("product-42", "http://127.0.0.1:8000")
    with pytest.raises(FrozenInstanceError):
        setattr(binding, field, "changed")


def _process(surface: str, **kwargs: object) -> UnrealProcess | PackagedUnrealProcess:
    if surface == "packaged":
        return PackagedUnrealProcess(
            Path("C:/RobotSimUE-Win64-Release.exe"),
            "/Game/Maps/FactoryPark_HF",
            **kwargs,  # type: ignore[arg-type]
        )
    return UnrealProcess(
        Path("C:/UnrealEditor.exe"),
        Path("C:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
        launch_profile=UnrealLaunchProfile(surface),
        **kwargs,  # type: ignore[arg-type]
    )


def _command(process: UnrealProcess | PackagedUnrealProcess) -> list[str]:
    return process.command(
        bundle_dir=Path("C:/bundles/measurement"),
        allocation=SimpleNamespace(  # type: ignore[arg-type]
            run_id="visual-run-not-product-session",
            path=Path("C:/runs/measurement/run-allocation.json"),
            artifact_root=Path("C:/artifacts/measurement"),
            log_dir=Path("C:/runs/measurement/logs"),
        ),
        snapshot_port=25123,
        model_generation=7,
        reset_generation=3,
    )


@pytest.mark.parametrize("surface", [profile.value for profile in UnrealLaunchProfile] + ["packaged"])
@pytest.mark.parametrize("with_existing_options", [False, True])
def test_launch_binding_appends_only_two_args_and_none_preserves_argv(
    surface: str,
    with_existing_options: bool,
) -> None:
    options: dict[str, object] = {}
    if with_existing_options:
        options = {
            "inspection_presentation": InspectionPresentationBinding(
                task_id="inspection-42",
                expected_map_id="factory-map",
                expected_map_content_epoch=4,
                expected_route_revision=2,
            ),
            "frame_capture_dir": Path("C:/runs/measurement/frames"),
            "frame_capture_every": 4,
            "frame_capture_max": 90,
            "session_camera_tag": "PreviewTarget:robot_eye",
            "motion_camera_stable_id": "robot/base_link",
            "depth_capture_in_main_renderer": True,
            "shared_color_depth_capture": True,
            "main_view_screen_percentage": 75,
        }
    baseline = _command(_process(surface, **options))
    assert _command(_process(surface, measurement_presentation=None, **options)) == baseline
    assert not any(argument.startswith("-LingTuMeasurement") for argument in baseline)
    binding = MeasurementPresentationBinding("product-42", "http://127.0.0.1:8000")

    assert _command(_process(surface, measurement_presentation=binding, **options)) == [
        *baseline,
        "-LingTuMeasurementProductSessionId=product-42",
        "-LingTuMeasurementGatewayUrl=http://127.0.0.1:8000",
    ]


@pytest.mark.parametrize("surface", ["unattended", "packaged"])
@pytest.mark.parametrize("binding", ["product-42", {}, object()])
def test_process_constructor_rejects_untyped_measurement_binding(
    surface: str,
    binding: object,
) -> None:
    with pytest.raises(TypeError, match="measurement_presentation must be MeasurementPresentationBinding or None"):
        _process(surface, measurement_presentation=binding)


def _runtime(tmp_path: Path, **kwargs: Any) -> playable.PlayableRuntimeConfig:
    toolchain = object.__new__(PinnedPlayableMediaToolchain)
    toolchain.paths = {
        "ffmpeg": (tmp_path / "ffmpeg.exe").resolve(),
        "ffprobe": (tmp_path / "ffprobe.exe").resolve(),
    }
    return playable.PlayableRuntimeConfig(
        repo_root=tmp_path,
        run_root=tmp_path / "runs",
        mujoco_host=tmp_path / "mujoco-host.exe",
        ffmpeg=toolchain.paths["ffmpeg"],
        ffprobe=toolchain.paths["ffprobe"],
        media_toolchain=toolchain,
        ports={"visual_snapshot_udp": 25123, "control_intent_udp": 25124, "control_status_udp": 25125},
        unreal_editor_executable=tmp_path / "UnrealEditor.exe",
        unreal_project=tmp_path / "RobotSimUE.uproject",
        robotsimue_executable=tmp_path / "RobotSimUE-Win64-Release.exe",
        run_id="visual-run-not-product-session",
        **kwargs,
    )


def _bundle(tmp_path: Path) -> ResolvedSessionBundle:
    plans: dict[str, Any] = {
        f"{name}.json": {
            "schema": f"lingtu.sim.{name.replace('.', '-')}.v1",
            "session_id": playable.PLAYABLE_SESSION_ID,
            "backends": {"physics": "mujoco", "visual": "unreal"},
        }
        for name in ("physics.plan", "visual.plan", "sensor.plan", "control.plan", "transport.intent")
    }
    plans["physics.plan.json"]["global_policy"] = {"timestep_s": 0.001}
    plans["visual.plan.json"].update(
        world={
            "level": playable.PLAYABLE_LEVEL,
            "package": dict(zip(("id", "version"), playable.PLAYABLE_WORLD_PACKAGE)),
        },
        robots=[
            {
                "instance_id": playable.PLAYABLE_INSTANCE_ID,
                "package": dict(zip(("id", "version"), playable.PLAYABLE_ROBOT_PACKAGE)),
            }
        ],
    )
    plans["control.plan.json"].update(
        controllers=[
            {
                "instance_id": playable.PLAYABLE_INSTANCE_ID,
                "controller_id": playable.PLAYABLE_CONTROLLER_ID,
                "package": dict(zip(("id", "version"), playable.PLAYABLE_CONTROLLER_PACKAGE)),
            }
        ],
        command_channels=[
            {
                "channel_id": playable.PLAYABLE_BASE_TWIST_CHANNEL,
                "command_type": "base_twist",
                "direction": "subscribe",
            }
        ],
    )
    plans["sensor.plan.json"]["streams"] = {
        group: [
            {
                "sensor_id": sensor_id,
                "owner": owner,
                "source": source,
                "transport": transport,
                "rate_hz": playable.PLAYABLE_SENSOR_RATES[sensor_id],
            }
        ]
        for sensor_id, (group, owner, source, transport) in playable.PLAYABLE_SENSOR_ROUTES.items()
    }
    return ResolvedSessionBundle(
        repo_root=tmp_path,
        bundle_dir=tmp_path / "bundle",
        session_id=playable.PLAYABLE_SESSION_ID,
        session_spec={
            "schema": "lingtu.sim.session.v1",
            "session_id": playable.PLAYABLE_SESSION_ID,
            "runtime": {
                "backend": "mujoco",
                "mode": "unreal",
                "required_bindings": list(playable.PLAYABLE_REQUIRED_BINDINGS),
            },
        },
        plans=plans,
    )


class _InertResource:
    def __call__(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def bind_session(self, _session: object) -> None:
        pass

    def close(self) -> None:
        pass


@pytest.mark.parametrize("runtime_surface", ["editor_game", "packaged_release"])
@pytest.mark.parametrize("include_binding", [False, True])
def test_create_forwards_only_explicit_measurement_binding_without_starting_resources(
    tmp_path: Path,
    runtime_surface: str,
    include_binding: bool,
) -> None:
    runtime = _runtime(tmp_path, runtime_surface=runtime_surface)
    assert runtime.measurement_presentation is None
    binding = MeasurementPresentationBinding("product-42", "https://gateway.example.test:8443")
    if include_binding:
        runtime = replace(runtime, measurement_presentation=binding)
    calls: list[dict[str, Any]] = []

    def unreal_factory(*args: Any, **kwargs: Any) -> UnrealProcess | PackagedUnrealProcess:
        calls.append(kwargs)
        constructor = UnrealProcess if runtime_surface == "editor_game" else PackagedUnrealProcess
        return constructor(*args, **kwargs)

    def inert_factory(*_args: Any, **_kwargs: Any) -> _InertResource:
        return _InertResource()

    dependencies = playable.PlayableLaunchDependencies(
        physics_host_factory=inert_factory,
        coordinator_factory=inert_factory,
        editor_process_factory=unreal_factory,
        packaged_process_factory=unreal_factory,
        snapshot_publisher_factory=inert_factory,
        visual_watcher_factory=inert_factory,
        camera_watcher_factory=inert_factory,
        session_host_factory=inert_factory,
        camera_payload_source_factory=inert_factory,
        motion_inbox_factory=inert_factory,
        request_inbox_factory=inert_factory,
        control_receiver_factory=inert_factory,
        control_ack_publisher_factory=inert_factory,
        control_evidence_writer_factory=inert_factory,
        recording_controller_factory=inert_factory,
        control_pump_factory=inert_factory,
        interactive_session_factory=inert_factory,
    )
    launch = playable.create_playable_launch(
        _bundle(tmp_path),
        runtime=runtime,
        dependencies=dependencies,
        sensor_endpoint_factory_builder=lambda _physics: lambda **_kwargs: None,
    )

    assert len(calls) == 1
    if include_binding:
        assert calls[0]["measurement_presentation"] is binding
    else:
        assert "measurement_presentation" not in calls[0]
    assert [arg for arg in _command(launch.unreal_process) if arg.startswith("-LingTuMeasurement")] == (
        binding.arguments() if include_binding else []
    )


@pytest.mark.parametrize("binding", ["product-42", {}, object()])
def test_runtime_config_rejects_untyped_measurement_binding(tmp_path: Path, binding: object) -> None:
    with pytest.raises(TypeError, match="measurement_presentation must be MeasurementPresentationBinding or None"):
        _runtime(tmp_path, measurement_presentation=binding)
