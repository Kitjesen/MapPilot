# ruff: noqa: S101

from __future__ import annotations

import json
import shutil
import uuid
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.coordinator import CoordinatorError, RuntimeState
from sim.runtime.coordinator.live_visual import (
    LiveVisualLaunch,
    _parser,
    create_live_visual_launch,
    default_uproject,
    run_live_visual,
)
from sim.runtime.coordinator.readiness import BindingFacet, BindingReadiness
from sim.runtime.coordinator.session_host import SessionHost
from sim.runtime.sensors import (
    ImuEndpointFactory,
    Mid360EndpointFactory,
    SensorEndpointRouter,
    TruthOdometryEndpointFactory,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunderv4_unreal"
    / "session.yaml"
)


class _VisualOnlyCoordinator:
    def __init__(self, order: list[str]) -> None:
        self.order = order
        self.state = RuntimeState.NEW
        self.bundle_dir = Path("C:/bundle")
        self.allocation = SimpleNamespace(
            ports={"visual_snapshot_udp": 25123},
            path=Path("C:/run/run-allocation.json"),
            log_dir=Path("C:/run/logs"),
        )
        self.plan = SimpleNamespace(repo_root=Path("C:/repo"), model_generation=0)
        self.readiness = BindingReadiness.for_required(
            (
                BindingFacet.PHYSICS,
                BindingFacet.VISUAL,
                BindingFacet.SENSORS,
                BindingFacet.CONTROL,
            )
        )
        self.sequence = 0

    @property
    def sensor_readiness(self) -> Any:
        return SimpleNamespace(blocking_reasons={"thunder_01.front_rgb": "stream is UNBOUND"})

    def prepare(self) -> dict[str, Any]:
        self.order.append("coordinator.prepare")
        self.state = RuntimeState.PREPARING
        self.readiness = self.readiness.mark_prepared(
            BindingFacet.PHYSICS,
            model_generation=0,
            reset_generation=0,
        ).mark_active(
            BindingFacet.PHYSICS,
            model_generation=0,
            reset_generation=0,
        )
        return self._event("ready")

    def snapshot(self) -> dict[str, Any]:
        self.order.append("coordinator.snapshot")
        return self._event("snapshot")

    def warmup(self, steps: int = 1) -> dict[str, Any]:
        self.order.append(f"coordinator.warmup:{steps}")
        assert self.state is RuntimeState.PREPARING
        self.sequence += steps
        return self._event("snapshot")

    def start(self) -> dict[str, Any]:
        self.order.append("coordinator.start")
        raise AssertionError("visual-only run must not require full session READY")

    def advance(self, steps: int = 1) -> dict[str, Any]:
        self.order.append(f"coordinator.advance:{steps}")
        raise AssertionError("visual-only PREPARING run must use warmup")

    def pause(self) -> dict[str, Any]:
        self.order.append("coordinator.pause")
        self.state = RuntimeState.PAUSED
        return self._event("paused")

    def stop(self) -> dict[str, Any]:
        self.order.append("coordinator.stop")
        self.state = RuntimeState.STOPPED
        return self._event("stopped")

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "session_id": "session-a",
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": self.sequence,
            "physics_step": self.sequence,
            "sim_time_ns": self.sequence * 2_000_000,
            "bodies": [
                {
                    "stable_id": "thunder_01/base_link",
                    "instance_id": "thunder_01",
                    "frame_id": "base_link",
                    "position_m": [0.0, 0.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                }
            ],
        }


class _Unreal:
    def __init__(self, order: list[str]) -> None:
        self.order = order

    def start(self, **_: Any) -> None:
        self.order.append("unreal.start")

    def poll(self) -> int | None:
        self.order.append("unreal.poll")
        return None

    def terminate(self) -> None:
        self.order.append("unreal.terminate")


class _Publisher:
    def __init__(self, order: list[str]) -> None:
        self.order = order
        self.snapshots: list[dict[str, Any]] = []

    def publish(self, event: dict[str, Any]) -> int:
        self.order.append("publisher.publish")
        self.snapshots.append(event)
        return 1

    def close(self) -> None:
        self.order.append("publisher.close")


class _VisualEvidence:
    def __init__(self, order: list[str]) -> None:
        self.order = order
        self.applied = False

    def apply(self, target: _VisualOnlyCoordinator) -> bool:
        self.order.append("evidence.apply:visual")
        if self.applied:
            return True
        self.applied = True
        target.readiness = target.readiness.mark_prepared(
            BindingFacet.VISUAL,
            model_generation=0,
            reset_generation=0,
        ).mark_active(
            BindingFacet.VISUAL,
            model_generation=0,
            reset_generation=0,
        )
        return True


class _RunHost:
    def __init__(self, screenshot_path: Path) -> None:
        self.screenshot_path = screenshot_path
        self.closed = False
        self.prepared = False

    def _prepare_screenshot(self) -> dict[str, str]:
        self.prepared = True
        self.screenshot_path.parent.mkdir(parents=True, exist_ok=True)
        self.screenshot_path.write_bytes(b"\x89PNG\r\n\x1a\nfirst-frame")
        return {"event": "prepared"}

    def prepare_until_visual_applied(self) -> dict[str, str]:
        return self._prepare_screenshot()

    def prepare(self) -> dict[str, str]:
        return self._prepare_screenshot()

    def run_visual_only(
        self,
        *,
        frame_limit: int,
        steps_per_frame: int,
        close_on_finish: bool = True,
    ) -> int:
        del steps_per_frame
        assert close_on_finish is False
        assert self.prepared is True
        return frame_limit

    def run(
        self,
        *,
        frame_limit: int,
        steps_per_frame: int,
        close_on_finish: bool = True,
    ) -> int:
        del steps_per_frame
        assert close_on_finish is False
        assert self.prepared is True
        return frame_limit

    def close(self) -> None:
        self.closed = True


class _NoScreenshotRunHost(_RunHost):
    def _prepare_screenshot(self) -> dict[str, str]:
        self.prepared = True
        return {"event": "prepared-without-screenshot"}


def test_visual_only_gate_streams_after_visual_active_without_session_ready() -> None:
    order: list[str] = []
    coordinator = _VisualOnlyCoordinator(order)
    publisher = _Publisher(order)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=_Unreal(order),
        publisher=publisher,
        evidence_watchers=(_VisualEvidence(order),),
        warmup_steps=3,
        sleep_s=0,
    )

    assert host.run_visual_only(frame_limit=2, steps_per_frame=5) == 2

    assert "coordinator.start" not in order
    assert "coordinator.advance:5" not in order
    assert order[:5] == [
        "coordinator.prepare",
        "unreal.start",
        "unreal.poll",
        "coordinator.snapshot",
        "publisher.publish",
    ]
    assert "evidence.apply:visual" in order
    assert order.count("coordinator.warmup:5") == 2
    assert [snapshot["physics_step"] for snapshot in publisher.snapshots[-2:]] == [5, 10]
    assert coordinator.state is RuntimeState.STOPPED


def test_live_visual_launcher_assembles_existing_runtime_seams() -> None:
    test_root = REPO_ROOT / "build" / "test-live-visual-launcher" / uuid.uuid4().hex
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        test_root / "bundle"
    )
    launch = create_live_visual_launch(
        bundle=bundle,
        repo_root=REPO_ROOT,
        run_root=test_root / "runs",
        mujoco_host=REPO_ROOT / "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
        unreal_editor=Path("C:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
        uproject=default_uproject(REPO_ROOT),
        snapshot_port=25234,
        dds_domain=37,
        run_id="live-contract",
        motion_camera_stable_id="thunder_01/base_link",
        frame_capture_dir=test_root / "frames",
        frame_capture_every=7,
        frame_capture_max=3,
        sleep_s=0,
    )

    unreal_command = launch.host._unreal.command(
        bundle_dir=bundle,
        allocation=SimpleNamespace(
            run_id="live-contract",
            path=test_root / "runs/live-contract/run-allocation.json",
            artifact_root=REPO_ROOT,
            log_dir=test_root / "runs/live-contract/logs",
        ),
        snapshot_port=25234,
        model_generation=0,
        reset_generation=0,
    )
    assert launch.run_id == "live-contract"
    assert launch.snapshot_port == 25234
    assert launch.visual_evidence_path == test_root / "runs/live-contract/logs/visual-readiness.json"
    assert launch.first_frame_screenshot_path == test_root / "runs/live-contract/logs/visual-first-frame.png"
    assert unreal_command[2] == "/Game/RobotSim/Maps/ThunderV4_RuntimePreview"
    assert f"-LingTuArtifactRoot={REPO_ROOT}" in unreal_command
    assert "-LingTuRunId=live-contract" in unreal_command
    assert "-windowed" in unreal_command
    assert unreal_command.count("-unattended") == 1
    assert unreal_command.count("-UnattendedInput") == 1
    assert unreal_command.count("-LingTuRuntimeUI") == 1
    assert unreal_command.count("-NoSound") == 1
    assert unreal_command.count("-ExecCmds=t.MaxFPS 30") == 1
    assert "-ResX=1920" in unreal_command
    assert "-ResY=1080" in unreal_command
    assert "-ForceRes" in unreal_command
    assert "-DDC=InstalledNoZenLocalFallback" in unreal_command
    assert f"-LocalDataCachePath={REPO_ROOT / 'build' / 'unreal-ddc'}" in unreal_command
    assert f"-LingTuScreenshot={launch.first_frame_screenshot_path}" in unreal_command
    assert "-LingTuMotionCameraStableId=thunder_01/base_link" in unreal_command
    assert f"-LingTuFrameCaptureDir={(test_root / 'frames').resolve()}" in unreal_command
    assert "-LingTuFrameCaptureEvery=7" in unreal_command
    assert "-LingTuFrameCaptureMax=3" in unreal_command
    assert launch.sensor_evidence_path == test_root / "runs/live-contract/logs/sensor-readiness.json"
    watchers = launch.host._evidence_watchers
    assert tuple(watcher.path for watcher in watchers) == (
        launch.visual_evidence_path,
        launch.sensor_evidence_path,
    )
    assert tuple(watcher.expected_source_id for watcher in watchers) == (
        "robotsimue-visual",
        "robotsimue-camera",
    )
    assert launch.run_allocation_path == test_root / "runs/live-contract/run-allocation.json"
    assert launch.unreal_log_path == test_root / "runs/live-contract/logs/Unreal.log"
    assert launch.coordinator._dds_domain == 37


def test_live_visual_launcher_can_use_packaged_robotsimue_without_editor() -> None:
    test_root = REPO_ROOT / "build" / "test-live-visual-packaged" / uuid.uuid4().hex
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        test_root / "bundle"
    )
    packaged_executable = (
        test_root
        / "release"
        / "Windows"
        / "RobotSimUE"
        / "Binaries"
        / "Win64"
        / "RobotSimUE-Win64-Release.exe"
    )
    launch = create_live_visual_launch(
        bundle=bundle,
        repo_root=REPO_ROOT,
        run_root=test_root / "runs",
        mujoco_host=REPO_ROOT
        / "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
        packaged_executable=packaged_executable,
        snapshot_port=25235,
        run_id="packaged-live-contract",
        motion_camera_stable_id="thunder_01/base_link",
    )

    unreal_command = launch.host._unreal.command(
        bundle_dir=bundle,
        allocation=SimpleNamespace(
            run_id="packaged-live-contract",
            path=test_root / "runs/packaged-live-contract/run-allocation.json",
            artifact_root=REPO_ROOT,
            log_dir=test_root / "runs/packaged-live-contract/logs",
        ),
        snapshot_port=25235,
        model_generation=0,
        reset_generation=0,
    )

    assert unreal_command[:2] == [
        str(packaged_executable.resolve()),
        "/Game/RobotSim/Maps/ThunderV4_RuntimePreview",
    ]
    assert not any(argument.endswith(".uproject") for argument in unreal_command)
    assert "-game" not in unreal_command
    assert "-NoCompile" not in unreal_command
    assert launch.unreal_log_path == (
        test_root / "runs/packaged-live-contract/logs/RobotSimUE.log"
    )


def test_live_visual_launcher_requires_exactly_one_unreal_runtime() -> None:
    test_root = REPO_ROOT / "build" / "test-live-visual-runtime-choice" / uuid.uuid4().hex
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        test_root / "bundle"
    )
    common = {
        "bundle": bundle,
        "repo_root": REPO_ROOT,
        "run_root": test_root / "runs",
        "mujoco_host": REPO_ROOT
        / "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
        "run_id": "runtime-choice",
    }

    with pytest.raises(ValueError, match="exactly one Unreal runtime"):
        create_live_visual_launch(**common)

    with pytest.raises(ValueError, match="exactly one Unreal runtime"):
        create_live_visual_launch(
            **common,
            unreal_editor=Path("C:/UE/UnrealEditor.exe"),
            uproject=default_uproject(REPO_ROOT),
            packaged_executable=Path(
                "C:/release/RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Release.exe"
            ),
        )


def test_run_live_visual_returns_and_validates_first_frame_screenshot() -> None:
    test_root = REPO_ROOT / "build" / "test-live-visual-screenshot" / uuid.uuid4().hex
    screenshot_path = test_root / "logs" / "visual-first-frame.png"
    try:
        host = _RunHost(screenshot_path)
        launch = LiveVisualLaunch(
            host=host,  # type: ignore[arg-type]
            coordinator=SimpleNamespace(manifest_path=test_root / "session-runtime-manifest.json"),
            run_id="live-screenshot",
            gate="visual-applied",
            snapshot_port=25123,
            visual_evidence_path=test_root / "logs" / "visual-readiness.json",
            first_frame_screenshot_path=screenshot_path,
            screenshot_timeout_s=1.0,
            sensor_evidence_path=test_root / "logs" / "sensor-readiness.json",
            run_allocation_path=test_root / "run-allocation.json",
            unreal_log_path=test_root / "logs" / "Unreal.log",
        )

        result = run_live_visual(
            launch,
            gate="visual-applied",
            frames=3,
            steps_per_frame=8,
        )

        assert result["frames"] == 3
        assert result["first_frame_screenshot"] == str(screenshot_path)
        assert screenshot_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert host.closed is True
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


def test_run_live_visual_rejects_stale_screenshot_from_reused_run_directory() -> None:
    test_root = REPO_ROOT / "build" / "test-live-visual-stale-screenshot" / uuid.uuid4().hex
    screenshot_path = test_root / "logs" / "visual-first-frame.png"
    screenshot_path.parent.mkdir(parents=True, exist_ok=True)
    screenshot_path.write_bytes(b"\x89PNG\r\n\x1a\nstale-frame")
    host = _NoScreenshotRunHost(screenshot_path)
    launch = LiveVisualLaunch(
        host=host,  # type: ignore[arg-type]
        coordinator=SimpleNamespace(manifest_path=test_root / "session-runtime-manifest.json"),
        run_id="reused-run",
        gate="visual-applied",
        snapshot_port=25123,
        visual_evidence_path=test_root / "logs" / "visual-readiness.json",
        first_frame_screenshot_path=screenshot_path,
        screenshot_timeout_s=0.05,
        sensor_evidence_path=test_root / "logs" / "sensor-readiness.json",
        run_allocation_path=test_root / "run-allocation.json",
        unreal_log_path=test_root / "logs" / "Unreal.log",
    )

    try:
        with pytest.raises(CoordinatorError, match="screenshot was not produced"):
            run_live_visual(launch, gate="visual-applied", frames=1, steps_per_frame=1)
        assert not screenshot_path.exists()
        assert host.closed is True
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


def test_live_visual_explicit_map_overrides_bundle_level() -> None:
    test_root = REPO_ROOT / "build" / "test-live-visual-override" / uuid.uuid4().hex
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        test_root / "bundle"
    )
    override = "/Game/RobotSim/Maps/ExplicitOverride"
    launch = create_live_visual_launch(
        bundle=bundle,
        repo_root=REPO_ROOT,
        run_root=test_root / "runs",
        mujoco_host=REPO_ROOT / "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
        unreal_editor=Path("C:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
        uproject=default_uproject(REPO_ROOT),
        map_name=override,
        run_id="live-override",
    )

    unreal_command = launch.host._unreal.command(
        bundle_dir=bundle,
        allocation=SimpleNamespace(
            run_id="live-override",
            path=test_root / "runs/live-override/run-allocation.json",
            artifact_root=REPO_ROOT,
            log_dir=test_root / "runs/live-override/logs",
        ),
        snapshot_port=launch.snapshot_port,
        model_generation=0,
        reset_generation=0,
    )

    assert unreal_command[2] == override
    assert "-LingTuRunId=live-override" in unreal_command


@pytest.mark.parametrize("map_name", ["", "   ", "/Engine/Maps/Forbidden", "/Game/   "])
def test_live_visual_rejects_invalid_explicit_map(map_name: str) -> None:
    test_root = REPO_ROOT / "build" / "test-live-visual-invalid-map" / uuid.uuid4().hex
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        test_root / "bundle"
    )

    with pytest.raises(ValueError, match=r"map_name must be a valid /Game/\.\.\. level"):
        create_live_visual_launch(
            bundle=bundle,
            repo_root=REPO_ROOT,
            run_root=test_root / "runs",
            mujoco_host=REPO_ROOT / "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
            unreal_editor=Path("C:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
            uproject=default_uproject(REPO_ROOT),
            map_name=map_name,
            run_id="live-invalid-map",
        )


@pytest.mark.parametrize("level", ["   ", "/Engine/Maps/Forbidden"])
def test_live_visual_rejects_invalid_bundle_level(level: str) -> None:
    test_root = REPO_ROOT / "build" / "test-live-visual-invalid-bundle" / uuid.uuid4().hex
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        test_root / "bundle"
    )
    visual_plan_path = bundle / "visual.plan.json"
    visual_plan = json.loads(visual_plan_path.read_text(encoding="utf-8"))
    visual_plan["world"]["level"] = level
    visual_plan_path.write_text(json.dumps(visual_plan), encoding="utf-8")

    with pytest.raises(
        ValueError,
        match=r"visual\.plan\.world\.level must be a valid /Game/\.\.\. level",
    ):
        create_live_visual_launch(
            bundle=bundle,
            repo_root=REPO_ROOT,
            run_root=test_root / "runs",
            mujoco_host=REPO_ROOT / "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
            unreal_editor=Path("C:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
            uproject=default_uproject(REPO_ROOT),
            run_id="live-invalid-bundle",
        )


def test_live_visual_parser_dds_domain_default_and_override() -> None:
    parser = _parser()
    required = [
        "bundle",
        "--mujoco-host",
        "mujoco.exe",
        "--unreal-editor",
        "UnrealEditor.exe",
    ]

    assert parser.parse_args(required).dds_domain == 0
    assert parser.parse_args([*required, "--dds-domain", "17"]).dds_domain == 17


def test_live_visual_parser_map_is_explicit_override_only() -> None:
    parser = _parser()
    required = [
        "bundle",
        "--mujoco-host",
        "mujoco.exe",
        "--unreal-editor",
        "UnrealEditor.exe",
    ]
    override = "/Game/RobotSim/Maps/ExplicitOverride"

    assert parser.parse_args(required).map_name is None
    assert parser.parse_args([*required, "--map", override]).map_name == override


def test_live_visual_parser_exposes_motion_camera_stable_id() -> None:
    parser = _parser()
    required = [
        "bundle",
        "--mujoco-host",
        "mujoco.exe",
        "--unreal-editor",
        "UnrealEditor.exe",
    ]

    assert parser.parse_args(required).motion_camera_stable_id is None
    assert (
        parser.parse_args(
            [*required, "--motion-camera-stable-id", "thunder_01/base_link"]
        ).motion_camera_stable_id
        == "thunder_01/base_link"
    )


def test_live_visual_parser_exposes_bounded_frame_capture() -> None:
    parser = _parser()
    required = [
        "bundle",
        "--mujoco-host",
        "mujoco.exe",
        "--unreal-editor",
        "UnrealEditor.exe",
    ]

    args = parser.parse_args(
        [
            *required,
            "--frame-capture-dir",
            "frames",
            "--frame-capture-every",
            "7",
            "--frame-capture-max",
            "3",
        ]
    )
    assert args.frame_capture_dir == Path("frames")
    assert args.frame_capture_every == 7
    assert args.frame_capture_max == 3


def _native_publisher_paths(tmp_path: Path) -> dict[str, Path]:
    return {
        "imu_publisher": tmp_path / "lingtu_imu_publisher.exe",
        "truth_odom_publisher": tmp_path / "lingtu_truth_odom_publisher.exe",
        "mid360_publisher": tmp_path / "lingtu_mid360_publisher.exe",
    }


@pytest.mark.parametrize(
    "publishers,missing",
    [
        ({"imu_publisher": "imu.exe"}, "--truth-odom-publisher"),
        ({"truth_odom_publisher": "truth.exe"}, "--imu-publisher"),
        ({"mid360_publisher": "mid360.exe"}, "--imu-publisher"),
    ],
)
def test_live_visual_rejects_partial_native_sensor_publishers(
    tmp_path: Path,
    publishers: dict[str, str],
    missing: str,
) -> None:
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        tmp_path / "bundle"
    )

    with pytest.raises(ValueError, match=missing):
        create_live_visual_launch(
            bundle=bundle,
            repo_root=REPO_ROOT,
            run_root=tmp_path / "runs",
            mujoco_host=Path("mujoco.exe"),
            unreal_editor=Path("UnrealEditor.exe"),
            uproject=default_uproject(REPO_ROOT),
            **{name: Path(value) for name, value in publishers.items()},
        )


def test_live_visual_session_ready_requires_all_native_sensor_publishers(
    tmp_path: Path,
) -> None:
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        tmp_path / "bundle"
    )

    with pytest.raises(
        ValueError,
        match=r"session-ready requires --imu-publisher, --truth-odom-publisher, and --mid360-publisher",
    ):
        create_live_visual_launch(
            bundle=bundle,
            repo_root=REPO_ROOT,
            run_root=tmp_path / "runs",
            mujoco_host=Path("mujoco.exe"),
            unreal_editor=Path("UnrealEditor.exe"),
            uproject=default_uproject(REPO_ROOT),
            gate="session-ready",
        )


def test_live_visual_native_sensor_router_reuses_the_coordinator_mujoco_process(
    tmp_path: Path,
) -> None:
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        tmp_path / "bundle"
    )
    launch = create_live_visual_launch(
        bundle=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        mujoco_host=Path("mujoco.exe"),
        unreal_editor=Path("UnrealEditor.exe"),
        uproject=default_uproject(REPO_ROOT),
        gate="session-ready",
        **_native_publisher_paths(tmp_path),
    )

    router = launch.coordinator._sensor_endpoint_factory
    assert isinstance(router, SensorEndpointRouter)
    assert tuple(type(factory) for factory in router.factories) == (
        ImuEndpointFactory,
        TruthOdometryEndpointFactory,
        Mid360EndpointFactory,
    )
    mid360_factory = router.factories[2]
    assert isinstance(mid360_factory, Mid360EndpointFactory)
    assert mid360_factory.process is launch.coordinator._host


def test_live_visual_parser_exposes_all_native_sensor_publishers() -> None:
    parser = _parser()
    required = [
        "bundle",
        "--mujoco-host",
        "mujoco.exe",
        "--unreal-editor",
        "UnrealEditor.exe",
        "--imu-publisher",
        "imu.exe",
        "--truth-odom-publisher",
        "truth.exe",
        "--mid360-publisher",
        "mid360.exe",
    ]

    args = parser.parse_args(required)

    assert args.imu_publisher == Path("imu.exe")
    assert args.truth_odom_publisher == Path("truth.exe")
    assert args.mid360_publisher == Path("mid360.exe")
