"""Inspection presentation binding at the Python-to-RobotSimUE launch seam."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
import sim.runtime.coordinator.playable_vertical_slice as playable_module
from sim.runtime.coordinator.playable_evidence import PinnedPlayableMediaToolchain
from sim.runtime.coordinator.playable_vertical_slice import (
    PlayableLaunchDependencies,
    PlayableRuntimeConfig,
    create_playable_launch,
)
from sim.runtime.coordinator.unreal_process import (
    InspectionPresentationBinding,
    PackagedUnrealProcess,
    UnrealProcess,
)


def _command(process: UnrealProcess) -> list[str]:
    allocation = SimpleNamespace(
        run_id="inspection-presentation-run",
        path=Path("C:/runs/inspection/run-allocation.json"),
        artifact_root=Path("C:/artifacts/inspection"),
        log_dir=Path("C:/runs/inspection/logs"),
    )
    return process.command(
        bundle_dir=Path("C:/bundles/inspection-session"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=7,
        reset_generation=3,
    )


def test_inspection_presentation_binding_is_default_inert_and_appends_exact_argv() -> None:
    editor = Path("D:/Epic/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe")
    project = Path("D:/lingtu/RobotSimUE.uproject")
    map_name = "/Game/RobotSim/Maps/FactoryPark_HF"
    default_process = UnrealProcess(editor, project, map_name)
    binding = InspectionPresentationBinding(
        task_id="inspection-task-42",
        expected_map_id="factory-park-map",
        expected_map_content_epoch=12,
        expected_route_revision=8,
        expected_product_session_id="product-session-inspection-7",
        gateway_url="https://gateway.example.test:8443",
    )
    bound_process = UnrealProcess(
        editor,
        project,
        map_name,
        inspection_presentation=binding,
    )

    default_argv = _command(default_process)
    expected_default = [
        str(editor.resolve()),
        str(project.resolve()),
        map_name,
        "-game",
        "-log",
        "-abslog=C:\\runs\\inspection\\logs\\Unreal.log",
        "-unattended",
        "-nop4",
        "-NoSplash",
        "-NoCompile",
        "-DisablePlugins=ModelContextProtocol,AllToolsets,Terminal,Tripo3DUEBridge,PythonScriptPlugin,EditorScriptingUtilities",
        "-DDC=InstalledNoZenLocalFallback",
        "-LocalDataCachePath=C:\\artifacts\\inspection\\build\\unreal-ddc",
        "-windowed",
        "-ResX=1920",
        "-ResY=1080",
        "-ForceRes",
        "-LingTuBundle=C:\\bundles\\inspection-session",
        "-LingTuRunAllocation=C:\\runs\\inspection\\run-allocation.json",
        "-LingTuRunId=inspection-presentation-run",
        "-LingTuArtifactRoot=C:\\artifacts\\inspection",
        "-LingTuSnapshotPort=25123",
        "-LingTuModelGeneration=7",
        "-LingTuResetGeneration=3",
        "-LingTuScreenshot=C:\\runs\\inspection\\logs\\visual-first-frame.png",
    ]
    assert default_argv == expected_default
    assert _command(bound_process) == [
        *expected_default,
        "-LingTuInspectionTaskId=inspection-task-42",
        "-LingTuInspectionExpectedMapId=factory-park-map",
        "-LingTuInspectionExpectedMapContentEpoch=12",
        "-LingTuInspectionExpectedRouteRevision=8",
        "-LingTuInspectionExpectedProductSessionId=product-session-inspection-7",
        "-LingTuInspectionGatewayUrl=https://gateway.example.test:8443",
    ]


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"task_id": "   "}, "task_id"),
        ({"expected_map_id": ""}, "expected_map_id"),
        ({"expected_map_content_epoch": 0}, "expected_map_content_epoch"),
        ({"expected_map_content_epoch": True}, "expected_map_content_epoch"),
        ({"expected_route_revision": -1}, "expected_route_revision"),
        ({"expected_product_session_id": "   "}, "expected_product_session_id"),
        ({"gateway_url": "gateway.example.test"}, "gateway_url"),
        ({"gateway_url": "ftp://gateway.example.test"}, "gateway_url"),
    ],
)
def test_inspection_presentation_binding_rejects_invalid_identity_and_revision(
    overrides: dict[str, object],
    message: str,
) -> None:
    values: dict[str, object] = {
        "task_id": "inspection-task",
        "expected_map_id": "factory-map",
        "expected_map_content_epoch": 1,
        "expected_route_revision": 1,
        "expected_product_session_id": None,
        "gateway_url": None,
    }
    values.update(overrides)

    with pytest.raises(ValueError, match=message):
        InspectionPresentationBinding(**values)  # type: ignore[arg-type]


def test_packaged_unreal_uses_the_same_binding_without_optional_gateway() -> None:
    binding = InspectionPresentationBinding(
        task_id="inspection-task-packaged",
        expected_map_id="factory-map",
        expected_map_content_epoch=5,
        expected_route_revision=2,
    )
    process = PackagedUnrealProcess(
        Path("C:/RobotSimUE-Win64-Release.exe"),
        "/Game/RobotSim/Maps/FactoryPark_HF",
        inspection_presentation=binding,
    )
    allocation = SimpleNamespace(
        run_id="packaged-inspection",
        path=Path("C:/runs/packaged/run-allocation.json"),
        artifact_root=Path("C:/artifacts/packaged"),
        log_dir=Path("C:/runs/packaged/logs"),
    )

    argv = process.command(
        bundle_dir=Path("C:/bundles/packaged"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=2,
        reset_generation=1,
    )

    assert argv[-4:] == [
        "-LingTuInspectionTaskId=inspection-task-packaged",
        "-LingTuInspectionExpectedMapId=factory-map",
        "-LingTuInspectionExpectedMapContentEpoch=5",
        "-LingTuInspectionExpectedRouteRevision=2",
    ]
    assert not any(arg.startswith("-LingTuInspectionGatewayUrl=") for arg in argv)
    assert not any(
        arg.startswith("-LingTuInspectionExpectedProductSessionId=") for arg in argv
    )


class _Resource:
    def __init__(self, role: str) -> None:
        self.role = role
        self.allocation = SimpleNamespace()

    def __call__(self, *_args: Any, **_kwargs: Any) -> None:
        return None

    def bind_session(self, _session: object) -> None:
        return None

    def close(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def terminate(self) -> None:
        return None


@pytest.mark.parametrize("include_binding", [False, True])
def test_playable_runtime_propagates_only_supplied_binding_to_injectable_unreal_factory(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    include_binding: bool,
) -> None:
    monkeypatch.setattr(playable_module, "_validate_fixed_bundle", lambda _bundle: None)
    calls: dict[str, tuple[tuple[object, ...], dict[str, object]]] = {}

    def factory(role: str):
        def create(*args: object, **kwargs: object) -> _Resource:
            calls[role] = (args, kwargs)
            return _Resource(role)

        return create

    bundle_dir = tmp_path / "bundle"
    bundle_dir.mkdir()
    bundle = SimpleNamespace(
        repo_root=tmp_path,
        bundle_dir=bundle_dir,
        session_id="thunderv4_factory_park_hf",
        plans={
            "physics.plan.json": {
                "schema": "lingtu.sim.physics-plan.v1",
                "session_id": "thunderv4_factory_park_hf",
                "global_policy": {"timestep_s": 0.001},
            },
            "sensor.plan.json": {
                "schema": "lingtu.sim.sensor-plan.v1",
                "session_id": "thunderv4_factory_park_hf",
                "streams": {
                    "imu": [{"owner": "physics", "rate_hz": 200}],
                },
            },
        },
    )
    ffmpeg = (tmp_path / "ffmpeg.exe").resolve()
    ffprobe = (tmp_path / "ffprobe.exe").resolve()
    toolchain = object.__new__(PinnedPlayableMediaToolchain)
    toolchain.paths = {"ffmpeg": ffmpeg, "ffprobe": ffprobe}
    binding = (
        InspectionPresentationBinding(
            task_id="inspection-task-factory",
            expected_map_id="factory-map",
            expected_map_content_epoch=9,
            expected_route_revision=4,
            expected_product_session_id="product-session-factory-9",
            gateway_url="http://127.0.0.1:8000",
        )
        if include_binding
        else None
    )
    runtime = PlayableRuntimeConfig(
        repo_root=tmp_path,
        run_root=tmp_path / "runs",
        mujoco_host=tmp_path / "mujoco-host.exe",
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        media_toolchain=toolchain,
        ports={
            "visual_snapshot_udp": 25123,
            "control_intent_udp": 25124,
            "control_status_udp": 25125,
        },
        unreal_editor_executable=tmp_path / "UnrealEditor.exe",
        unreal_project=tmp_path / "RobotSimUE.uproject",
        run_id="inspection-factory-run",
        boot_id="inspection-factory-boot",
        inspection_presentation=binding,
    )
    dependencies = PlayableLaunchDependencies(
        physics_host_factory=factory("physics"),
        coordinator_factory=factory("coordinator"),
        editor_process_factory=factory("unreal"),
        snapshot_publisher_factory=factory("snapshot"),
        visual_watcher_factory=factory("visual_watcher"),
        camera_watcher_factory=factory("camera_watcher"),
        session_host_factory=factory("host"),
        camera_payload_source_factory=factory("camera_payload"),
        motion_inbox_factory=factory("motion_inbox"),
        request_inbox_factory=factory("request_inbox"),
        control_receiver_factory=factory("receiver"),
        control_ack_publisher_factory=factory("ack"),
        control_evidence_writer_factory=factory("evidence"),
        recording_controller_factory=factory("recording"),
        control_pump_factory=factory("pump"),
        interactive_session_factory=factory("session"),
    )

    create_playable_launch(
        bundle,  # type: ignore[arg-type]
        runtime=runtime,
        sensor_endpoint_factory_builder=lambda _physics: lambda **_kwargs: None,
        dependencies=dependencies,
    )

    _args, kwargs = calls["unreal"]
    if binding is None:
        assert "inspection_presentation" not in kwargs
    else:
        assert kwargs["inspection_presentation"] is binding
