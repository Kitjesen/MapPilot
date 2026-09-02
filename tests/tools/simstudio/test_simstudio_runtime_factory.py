"""Public behavior tests for the SimStudio production runtime factory."""

# ruff: noqa: D101,D102,D103,S101

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest
import yaml
from sim.catalog import CatalogResolver
from sim.runtime.coordinator import (
    CoordinatorError,
    InteractiveSimulationSession,
    RunAllocationError,
    RuntimeState,
    create_run_allocation,
)
from sim.runtime.coordinator.session_host import SessionHost
from tools.simstudio.service.application import SimulationStudioService
from tools.simstudio.service.models import BundleRecord, StoreValidationError
from tools.simstudio.service.run_service import RunService
from tools.simstudio.service.runtime_factory import RuntimeFactory, TrustedRuntimeConfig
from tools.simstudio.service.store import StudioStore

REPO_ROOT = Path(__file__).resolve().parents[3]
SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunderv4_unreal"
    / "session.yaml"
)
HEADLESS_SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "omni_cart_controlled_headless"
    / "session.yaml"
)


def _session_id(bundle_dir: Path) -> str:
    session = yaml.safe_load((bundle_dir / "session.yaml").read_text(encoding="utf-8"))
    return str(session["session_id"])


class FakeMujocoProcess:
    def __init__(self, executable: Path) -> None:
        self.executable = Path(executable)

    @property
    def pid(self) -> int | None:
        return None


class FakeCoordinator:
    calls: list[dict[str, Any]] = []

    def __init__(self, **kwargs: Any) -> None:
        from sim.runtime.coordinator import RuntimeState

        self.kwargs = kwargs
        self.state = RuntimeState.NEW
        self.__class__.calls.append(kwargs)

    def prepare(self) -> dict[str, Any]:
        return {
            "event": "ready",
            "model_generation": 0,
            "reset_generation": 0,
            "session_id": "studio-session",
        }

    def start(self) -> dict[str, Any]:
        return {
            "event": "running",
            "model_generation": 0,
            "reset_generation": 0,
        }

    def advance(self, steps: int = 1) -> dict[str, Any]:
        return {
            "event": "snapshot",
            "model_generation": 0,
            "reset_generation": 0,
            "steps": steps,
        }

    def pause(self) -> dict[str, Any]:
        return {
            "event": "paused",
            "model_generation": 0,
            "reset_generation": 0,
        }

    def reset(self) -> dict[str, Any]:
        return {
            "event": "reset",
            "model_generation": 0,
            "reset_generation": 1,
        }

    def stop(self, *, failure_reason: str | None = None) -> dict[str, Any]:
        return {
            "event": "stopped",
            "model_generation": 0,
            "reset_generation": 0,
            "failure_reason": failure_reason,
        }


class AllocatingFakeCoordinator(FakeCoordinator):
    def prepare(self) -> dict[str, Any]:
        allocation = create_run_allocation(
            self.kwargs["bundle_dir"],
            self.kwargs["run_root"],
            run_id=self.kwargs["run_id"],
            dds_domain=self.kwargs["dds_domain"],
            ports=self.kwargs["ports"],
            shm=self.kwargs["shm"],
            repo_root=self.kwargs["repo_root"],
            adopt_existing_empty_run_dir=self.kwargs[
                "adopt_existing_empty_run_dir"
            ],
            trusted_root=self.kwargs.get("trusted_root"),
        )
        self.state = RuntimeState.READY
        return {
            "event": "ready",
            "model_generation": 0,
            "reset_generation": 0,
            "session_id": allocation.session_id,
        }


class FakeUnrealProcess:
    calls: list[dict[str, Any]] = []

    def __init__(
        self,
        editor: Path,
        uproject: Path,
        level: str,
        *,
        motion_camera_stable_id: str | None = None,
    ) -> None:
        self.editor = Path(editor)
        self.uproject = Path(uproject)
        self.level = level
        self.__class__.calls.append(
            {
                "editor": self.editor,
                "uproject": self.uproject,
                "level": self.level,
                "motion_camera_stable_id": motion_camera_stable_id,
            }
        )

    def start(self, **_: Any) -> None:
        raise AssertionError("factory construction must not launch Unreal")

    def poll(self) -> int | None:
        return None

    def terminate(self) -> None:
        return None


class FakePublisher:
    calls: list[int] = []

    def __init__(self, port: int) -> None:
        self.port = port
        self.__class__.calls.append(port)

    def publish(self, event: Any) -> int:
        del event
        return 1

    def close(self) -> None:
        return None


class FakeWatcher:
    calls: list[dict[str, Any]] = []

    def __init__(self, path: Path, **kwargs: Any) -> None:
        self.path = Path(path)
        self.kwargs = kwargs
        self.__class__.calls.append({"path": self.path, **kwargs})

    def apply(self, target: Any) -> bool:
        del target
        return False

    def advance_generation(self, *, model_generation: int, reset_generation: int) -> None:
        self.kwargs["model_generation"] = model_generation
        self.kwargs["reset_generation"] = reset_generation


def test_headless_profile_rejects_visual_required_bundle_before_process_construction(
    tmp_path: Path,
) -> None:
    studio_root = tmp_path / "studio"
    CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        studio_root / "bundles" / "visual"
    )
    process_calls: list[Path] = []

    def forbidden_process(executable: Path) -> FakeMujocoProcess:
        process_calls.append(Path(executable))
        raise AssertionError("profile validation must precede process construction")

    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=studio_root,
            mujoco_host=(
                REPO_ROOT
                / "build"
                / "simstudio-test"
                / "lingtu_mujoco_headless.exe"
            ),
        ),
        coordinator_type=FakeCoordinator,
        mujoco_process_type=forbidden_process,
    )
    store = StudioStore(studio_root)
    bundle = store.create_bundle(
        {"bundle_path": "bundles/visual"},
        status="ready",
    )
    run_service = RunService(
        store,
        factory.create_session,
        artifact_root=studio_root / "artifacts" / "runs",
    )
    run = run_service.create_run(bundle.id, "headless")

    with pytest.raises(StoreValidationError) as captured:
        run_service.prepare(
            run["id"],
            expected_revision=run["revision"],
        )

    error = captured.value
    assert getattr(error, "code", None) == "SIMSTUDIO_LAUNCH_PROFILE_MISMATCH"
    assert getattr(error, "details", None) == {
        "launch_profile": "headless",
        "required_bindings": ["control", "physics", "sensors", "visual"],
        "session_mode": "unreal",
    }
    assert error.to_dict() == {
        "schema": "lingtu.sim.studio.runtime-factory-error.v1",
        "code": "SIMSTUDIO_LAUNCH_PROFILE_MISMATCH",
        "message": (
            "launch profile 'headless' is incompatible with session mode "
            "'unreal' and required bindings"
        ),
        "details": {
            "launch_profile": "headless",
            "required_bindings": ["control", "physics", "sensors", "visual"],
            "session_mode": "unreal",
        },
    }
    assert process_calls == []
    failed = run_service.get_run(run["id"])
    assert failed["status"] == "FAILED"
    assert failed["payload"]["failure"] == {
        "type": "RuntimeFactoryError",
        "message": (
            "launch profile 'headless' is incompatible with session mode "
            "'unreal' and required bindings"
        ),
    }


def test_visual_profile_rejects_headless_bundle_before_process_construction(
    tmp_path: Path,
) -> None:
    studio_root = tmp_path / "studio"
    CatalogResolver.from_repository(REPO_ROOT).resolve(HEADLESS_SESSION).write_bundle(
        studio_root / "bundles" / "headless"
    )
    process_calls: list[Path] = []

    def forbidden_process(executable: Path) -> FakeMujocoProcess:
        process_calls.append(Path(executable))
        raise AssertionError("profile validation must precede process construction")

    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=studio_root,
            mujoco_host=(
                REPO_ROOT
                / "build"
                / "simstudio-test"
                / "lingtu_mujoco_headless.exe"
            ),
            unreal_editor=tmp_path / "UnrealEditor.exe",
        ),
        coordinator_type=FakeCoordinator,
        mujoco_process_type=forbidden_process,
        unreal_process_type=FakeUnrealProcess,
    )
    bundle = BundleRecord(
        id="a" * 32,
        revision=1,
        created_at="2026-01-01T00:00:00Z",
        updated_at="2026-01-01T00:00:00Z",
        status="ready",
        payload={"bundle_path": "bundles/headless"},
    )

    with pytest.raises(StoreValidationError) as captured:
        factory.create_session(
            bundle=bundle,
            launch_profile="visual",
            artifact_root=studio_root / "artifacts" / "runs" / "bad-visual",
            run_id="bad-visual",
        )

    error = captured.value
    assert getattr(error, "code", None) == "SIMSTUDIO_LAUNCH_PROFILE_MISMATCH"
    assert getattr(error, "details", None) == {
        "launch_profile": "visual",
        "required_bindings": ["control", "physics"],
        "session_mode": "headless",
    }
    assert process_calls == []


@pytest.mark.parametrize(
    ("runtime_value", "expected_field", "expected_mode", "expected_bindings"),
    [
        pytest.param(None, "session.runtime", None, [], id="missing-runtime"),
        pytest.param(
            {"mode": "headless"},
            "session.runtime.required_bindings",
            "headless",
            [],
            id="missing-bindings",
        ),
        pytest.param(
            {"mode": "headless", "required_bindings": "physics"},
            "session.runtime.required_bindings",
            "headless",
            [],
            id="non-list-bindings",
        ),
        pytest.param(
            {"mode": "headless", "required_bindings": ["physics", 1]},
            "session.runtime.required_bindings",
            "headless",
            ["physics"],
            id="non-string-binding",
        ),
        pytest.param(
            {"mode": "headless", "required_bindings": []},
            "session.runtime.required_bindings",
            "headless",
            [],
            id="empty-bindings",
        ),
    ],
)
def test_headless_profile_rejects_invalid_runtime_contract_before_process_construction(
    tmp_path: Path,
    runtime_value: dict[str, Any] | None,
    expected_field: str,
    expected_mode: str | None,
    expected_bindings: list[str],
) -> None:
    studio_root = tmp_path / "studio"
    bundle_dir = CatalogResolver.from_repository(REPO_ROOT).resolve(HEADLESS_SESSION).write_bundle(
        studio_root / "bundles" / "invalid-runtime"
    )
    session_path = bundle_dir / "session.yaml"
    session = yaml.safe_load(session_path.read_text(encoding="utf-8"))
    if runtime_value is None:
        del session["runtime"]
    else:
        session["runtime"] = runtime_value
    session_path.write_text(yaml.safe_dump(session, sort_keys=False), encoding="utf-8")
    process_calls: list[Path] = []

    def forbidden_process(executable: Path) -> FakeMujocoProcess:
        process_calls.append(Path(executable))
        raise AssertionError("runtime contract validation must precede process construction")

    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=studio_root,
            mujoco_host=(
                REPO_ROOT
                / "build"
                / "simstudio-test"
                / "lingtu_mujoco_headless.exe"
            ),
        ),
        coordinator_type=FakeCoordinator,
        mujoco_process_type=forbidden_process,
    )
    bundle = BundleRecord(
        id="a" * 32,
        revision=1,
        created_at="2026-01-01T00:00:00Z",
        updated_at="2026-01-01T00:00:00Z",
        status="ready",
        payload={"bundle_path": "bundles/invalid-runtime"},
    )

    with pytest.raises(StoreValidationError) as captured:
        factory.create_session(
            bundle=bundle,
            launch_profile="headless",
            artifact_root=studio_root / "artifacts" / "runs" / "invalid-runtime",
            run_id="invalid-runtime",
        )

    error = captured.value
    assert getattr(error, "code", None) == "SIMSTUDIO_RUNTIME_CONTRACT_INVALID"
    assert getattr(error, "details", None) == {
        "field": expected_field,
        "launch_profile": "headless",
        "required_bindings": expected_bindings,
        "session_mode": expected_mode,
    }
    assert process_calls == []


def test_default_service_builds_headless_session_from_server_trusted_config(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    FakeCoordinator.calls.clear()
    mujoco_host = REPO_ROOT / "build" / "simstudio-test" / "lingtu_mujoco_headless.exe"

    def provider(_root: Path, **kwargs: Any) -> RuntimeFactory:
        return RuntimeFactory(
            TrustedRuntimeConfig(
                repo_root=REPO_ROOT,
                studio_root=kwargs["studio_root"],
                mujoco_host=mujoco_host,
                dds_domain=42,
                ports={"truth": 19001},
                shm={"camera": "simstudio-camera"},
            ),
            coordinator_type=FakeCoordinator,
            mujoco_process_type=FakeMujocoProcess,
            controller_factory=lambda *_: object(),
            sensor_endpoint_factory=lambda *_: None,
        )

    monkeypatch.setattr(
        "tools.simstudio.service.application.RuntimeFactory",
        type("FactoryProvider", (), {"from_repository": staticmethod(provider)}),
    )

    service = SimulationStudioService.from_repository(REPO_ROOT, artifact_root=tmp_path)
    assert service.run_service is not None
    bundle_dir = CatalogResolver.from_repository(REPO_ROOT).resolve(HEADLESS_SESSION).write_bundle(
        tmp_path / "bundles" / "demo"
    )
    session_id = _session_id(bundle_dir)
    bundle: BundleRecord = service.store.create_bundle(  # type: ignore[union-attr]
        {
            "bundle_path": "bundles/demo",
            "session_id": session_id,
            "mujoco_host": "ignored/request/path",
            "dds_domain": 999,
            "ports": {"truth": 1},
            "visual": {"asset_path": "/Game/RobotSim/Maps/OpenField"},
        }
    )

    run = service.create_run(bundle_id=bundle.id, launch_profile="headless")
    prepared = service.run_operation("prepare", run["id"], revision=run["revision"])
    session = service.run_service._sessions[run["id"]]  # type: ignore[union-attr]

    assert prepared["status"] == "READY"
    assert isinstance(session, InteractiveSimulationSession)
    assert FakeCoordinator.calls
    coordinator_args = FakeCoordinator.calls[-1]
    assert coordinator_args["repo_root"] == REPO_ROOT.resolve()
    assert coordinator_args["physics_host"].executable == mujoco_host.resolve()
    assert coordinator_args["dds_domain"] == 42
    assert coordinator_args["ports"] == {"truth": 19001}
    assert coordinator_args["shm"] == {"camera": "simstudio-camera"}
    assert coordinator_args["run_id"] == run["id"]
    assert coordinator_args["adopt_existing_empty_run_dir"] is True
    assert "scenario_dispatcher" not in coordinator_args


def test_visual_profile_fails_closed_without_server_unreal_config(tmp_path: Path) -> None:
    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            mujoco_host=REPO_ROOT / "build" / "simstudio-test" / "lingtu_mujoco_headless.exe",
            studio_root=tmp_path,
        ),
        coordinator_type=FakeCoordinator,
        mujoco_process_type=FakeMujocoProcess,
    )
    bundle = BundleRecord(
        id="a" * 32,
        revision=1,
        created_at="2026-01-01T00:00:00Z",
        updated_at="2026-01-01T00:00:00Z",
        status="ready",
        payload={"bundle_path": "bundles/demo"},
    )

    with pytest.raises(RuntimeError, match="requires server Unreal executable config"):
        factory.create_session(
            bundle=bundle,
            launch_profile="visual",
            artifact_root=tmp_path,
            run_id="run-visual",
        )


def test_visual_profile_uses_only_server_trusted_runtime_values_and_plan_level(
    tmp_path: Path,
) -> None:
    FakeCoordinator.calls.clear()
    FakeUnrealProcess.calls.clear()
    FakePublisher.calls.clear()
    FakeWatcher.calls.clear()
    studio_root = tmp_path / "studio"
    bundle_dir = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        studio_root / "bundles" / "demo"
    )
    session_id = _session_id(bundle_dir)
    run_dir = studio_root / "artifacts" / "runs" / "visual-run"
    trusted_unreal = tmp_path / "trusted" / "UnrealEditor.exe"
    trusted_uproject = tmp_path / "trusted" / "RobotSimUE.uproject"
    malicious_port = 9
    bundle = BundleRecord(
        id="a" * 32,
        revision=1,
        created_at="2026-01-01T00:00:00Z",
        updated_at="2026-01-01T00:00:00Z",
        status="ready",
        payload={
            "bundle_path": "bundles/demo",
            "unreal_editor": "C:/attacker/UnrealEditor.exe",
            "uproject": "C:/attacker/RobotSimUE.uproject",
            "dds_domain": 999,
            "ports": {"visual_snapshot_udp": malicious_port},
        },
    )
    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=studio_root,
            mujoco_host=REPO_ROOT / "build" / "simstudio-test" / "lingtu_mujoco_headless.exe",
            dds_domain=43,
            ports={"truth": 19001},
            unreal_editor=trusted_unreal,
            uproject=trusted_uproject,
            visual_snapshot_port=25333,
            visual_ready_timeout_s=132.5,
            motion_camera_stable_id="thunder_01/base_link",
        ),
        coordinator_type=FakeCoordinator,
        mujoco_process_type=FakeMujocoProcess,
        unreal_process_type=FakeUnrealProcess,
        snapshot_publisher_type=FakePublisher,
        evidence_watcher_type=FakeWatcher,
        controller_factory=lambda *_: object(),
        sensor_endpoint_factory=lambda *_: None,
    )

    session = factory.create_session(
        bundle=bundle,
        launch_profile="visual",
        artifact_root=run_dir,
        run_id=run_dir.name,
    )

    assert isinstance(session, InteractiveSimulationSession)
    assert isinstance(session._coordinator, SessionHost)
    assert session._coordinator._ready_timeout_s == 132.5
    assert FakeUnrealProcess.calls == [
        {
            "editor": trusted_unreal.resolve(),
            "uproject": trusted_uproject.resolve(),
            "level": "/Game/RobotSim/Maps/ThunderV4_RuntimePreview",
            "motion_camera_stable_id": "thunder_01/base_link",
        }
    ]
    assert FakePublisher.calls == [25333]
    coordinator_args = FakeCoordinator.calls[-1]
    assert coordinator_args["physics_host"].executable == (
        REPO_ROOT / "build" / "simstudio-test" / "lingtu_mujoco_headless.exe"
    ).resolve()
    assert coordinator_args["dds_domain"] == 43
    assert coordinator_args["ports"] == {
        "truth": 19001,
        "visual_snapshot_udp": 25333,
    }
    assert coordinator_args["adopt_existing_empty_run_dir"] is True
    assert coordinator_args["controller_factory"] is not None
    assert len(FakeWatcher.calls) == 2
    assert [call["path"] for call in FakeWatcher.calls] == [
        run_dir / "logs" / "visual-readiness.json",
        run_dir / "logs" / "sensor-readiness.json",
    ]
    assert {call["expected_source_id"] for call in FakeWatcher.calls} == {
        "robotsimue-visual",
        "robotsimue-camera",
    }
    assert all(call["session_id"] == session_id for call in FakeWatcher.calls)
    assert bundle_dir.is_dir()


def test_visual_profile_rejects_invalid_server_compiled_plan_level(
    tmp_path: Path,
) -> None:
    studio_root = tmp_path / "studio"
    bundle_dir = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(
        studio_root / "bundles" / "demo"
    )
    visual_plan_path = bundle_dir / "visual.plan.json"
    visual_plan = json.loads(visual_plan_path.read_text(encoding="utf-8"))
    visual_plan["world"]["level"] = "/Engine/Forbidden"
    visual_plan_path.write_text(json.dumps(visual_plan), encoding="utf-8")
    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=studio_root,
            mujoco_host=REPO_ROOT / "build" / "simstudio-test" / "lingtu_mujoco_headless.exe",
            unreal_editor=tmp_path / "UnrealEditor.exe",
        ),
        coordinator_type=FakeCoordinator,
        mujoco_process_type=FakeMujocoProcess,
        unreal_process_type=FakeUnrealProcess,
        snapshot_publisher_type=FakePublisher,
        evidence_watcher_type=FakeWatcher,
    )
    bundle = BundleRecord(
        id="a" * 32,
        revision=1,
        created_at="2026-01-01T00:00:00Z",
        updated_at="2026-01-01T00:00:00Z",
        status="ready",
        payload={"bundle_path": "bundles/demo"},
    )

    with pytest.raises(RuntimeError, match=r"visual\.plan\.world\.level"):
        factory.create_session(
            bundle=bundle,
            launch_profile="visual",
            artifact_root=studio_root / "artifacts" / "runs" / "bad-level",
            run_id="bad-level",
        )


def test_visual_trusted_config_from_env_defaults_uproject_snapshot_port_and_timeout(
    tmp_path: Path,
) -> None:
    config = TrustedRuntimeConfig.from_repository(
        REPO_ROOT,
        studio_root=tmp_path,
        env={
            "LINGTU_SIMSTUDIO_MUJOCO_HOST": str(tmp_path / "mujoco.exe"),
            "LINGTU_SIMSTUDIO_UNREAL_EDITOR": str(tmp_path / "UnrealEditor.exe"),
            "LINGTU_SIMSTUDIO_VISUAL_SNAPSHOT_PORT": "25444",
            "LINGTU_SIMSTUDIO_VISUAL_READY_TIMEOUT_S": "150.5",
            "LINGTU_SIMSTUDIO_MOTION_CAMERA_STABLE_ID": "thunder_01/base_link",
        },
    )

    assert config.unreal_editor == (tmp_path / "UnrealEditor.exe").resolve()
    assert config.uproject == (
        REPO_ROOT
        / "sim"
        / "runtime"
        / "visual"
        / "RobotSimUE"
        / "RobotSimUE.uproject"
    ).resolve()
    assert config.visual_snapshot_port == 25444
    assert config.visual_ready_timeout_s == 150.5
    assert config.motion_camera_stable_id == "thunder_01/base_link"


@pytest.mark.parametrize("value", [" ", "thunder 01/base_link", "\tbad"])
def test_motion_camera_stable_id_rejects_whitespace_tokens(
    tmp_path: Path,
    value: str,
) -> None:
    with pytest.raises(ValueError, match="LINGTU_SIMSTUDIO_MOTION_CAMERA_STABLE_ID"):
        TrustedRuntimeConfig.from_repository(
            REPO_ROOT,
            studio_root=tmp_path,
            env={
                "LINGTU_SIMSTUDIO_MUJOCO_HOST": str(tmp_path / "mujoco.exe"),
                "LINGTU_SIMSTUDIO_MOTION_CAMERA_STABLE_ID": value,
            },
        )


def test_visual_ready_timeout_default_is_long_enough_for_cold_unreal_start(
    tmp_path: Path,
) -> None:
    config = TrustedRuntimeConfig.from_repository(
        REPO_ROOT,
        studio_root=tmp_path,
        env={
            "LINGTU_SIMSTUDIO_MUJOCO_HOST": str(tmp_path / "mujoco.exe"),
            "LINGTU_SIMSTUDIO_UNREAL_EDITOR": str(tmp_path / "UnrealEditor.exe"),
        },
    )

    assert config.visual_ready_timeout_s == 600.0


@pytest.mark.parametrize("value", ["0", "nan", "inf", "-inf", "not-a-number"])
def test_visual_ready_timeout_env_rejects_non_positive_non_finite_or_invalid_values(
    tmp_path: Path,
    value: str,
) -> None:
    with pytest.raises(ValueError, match="LINGTU_SIMSTUDIO_VISUAL_READY_TIMEOUT_S"):
        TrustedRuntimeConfig.from_repository(
            REPO_ROOT,
            studio_root=tmp_path,
            env={
                "LINGTU_SIMSTUDIO_MUJOCO_HOST": str(tmp_path / "mujoco.exe"),
                "LINGTU_SIMSTUDIO_UNREAL_EDITOR": str(tmp_path / "UnrealEditor.exe"),
                "LINGTU_SIMSTUDIO_VISUAL_READY_TIMEOUT_S": value,
            },
        )


@pytest.mark.parametrize("value", [0.0, math.nan, math.inf, -math.inf])
def test_visual_ready_timeout_config_rejects_non_positive_or_non_finite_values(
    tmp_path: Path,
    value: float,
) -> None:
    with pytest.raises(ValueError, match="visual_ready_timeout_s"):
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=tmp_path,
            mujoco_host=REPO_ROOT / "build" / "simstudio-test" / "lingtu_mujoco_headless.exe",
            unreal_editor=tmp_path / "UnrealEditor.exe",
            visual_ready_timeout_s=value,
        )


def test_runtime_factory_preserves_artifact_parent_for_allocation_safety(
    tmp_path: Path,
) -> None:
    FakeCoordinator.calls.clear()
    studio_root = tmp_path / "studio"
    CatalogResolver.from_repository(REPO_ROOT).resolve(HEADLESS_SESSION).write_bundle(
        studio_root / "bundles" / "demo"
    )
    run_dir = studio_root / "artifacts" / "runs" / "run-linked"
    run_dir.parent.mkdir(parents=True)
    foreign = tmp_path / "foreign-run"
    foreign.mkdir()
    try:
        run_dir.symlink_to(foreign, target_is_directory=True)
    except (NotImplementedError, OSError):
        pytest.skip("directory symbolic links are unavailable in this test environment")
    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=studio_root,
            mujoco_host=(
                REPO_ROOT
                / "build"
                / "simstudio-test"
                / "lingtu_mujoco_headless.exe"
            ),
        ),
        coordinator_type=FakeCoordinator,
        mujoco_process_type=FakeMujocoProcess,
    )
    bundle = BundleRecord(
        id="a" * 32,
        revision=1,
        created_at="2026-01-01T00:00:00Z",
        updated_at="2026-01-01T00:00:00Z",
        status="ready",
        payload={"bundle_path": "bundles/demo"},
    )

    try:
        factory.create_session(
            bundle=bundle,
            launch_profile="headless",
            artifact_root=run_dir,
            run_id=run_dir.name,
        )
    finally:
        run_dir.unlink(missing_ok=True)

    assert FakeCoordinator.calls[-1]["run_root"] == run_dir.parent


def test_prepare_rejects_runs_parent_link_swap_without_writing_foreign_target(
    tmp_path: Path,
) -> None:
    studio_root = tmp_path / "studio"
    bundle_dir = CatalogResolver.from_repository(REPO_ROOT).resolve(HEADLESS_SESSION).write_bundle(
        studio_root / "bundles" / "demo"
    )
    session_id = _session_id(bundle_dir)
    store = StudioStore(studio_root)
    bundle = store.create_bundle(
        {"bundle_path": "bundles/demo", "session_id": session_id}
    )
    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=studio_root,
            mujoco_host=(
                REPO_ROOT
                / "build"
                / "simstudio-test"
                / "lingtu_mujoco_headless.exe"
            ),
        ),
        coordinator_type=AllocatingFakeCoordinator,
        mujoco_process_type=FakeMujocoProcess,
    )
    runs_root = studio_root / "artifacts" / "runs"
    displaced = tmp_path / "displaced-runs"
    foreign = tmp_path / "foreign-runs"
    link_created = False

    def swap_then_create_session(
        *,
        bundle: BundleRecord,
        launch_profile: str,
        artifact_root: Path,
        run_id: str,
    ) -> InteractiveSimulationSession:
        nonlocal link_created
        (foreign / run_id).mkdir(parents=True)
        runs_root.rename(displaced)
        if sys.platform == "win32":
            created = subprocess.run(  # noqa: S603
                ["cmd", "/c", "mklink", "/J", str(runs_root), str(foreign)],  # noqa: S607
                capture_output=True,
                text=True,
                check=False,
            )
            if created.returncode != 0:
                displaced.rename(runs_root)
                pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")
        else:
            try:
                runs_root.symlink_to(foreign, target_is_directory=True)
            except (NotImplementedError, OSError):
                displaced.rename(runs_root)
                pytest.skip("directory symbolic links are unavailable in this test environment")
        link_created = True
        return factory.create_session(
            bundle=bundle,
            launch_profile=launch_profile,
            artifact_root=artifact_root,
            run_id=run_id,
        )

    service = RunService(store, swap_then_create_session, artifact_root=runs_root)
    run = service.create_run(bundle.id, "headless")
    foreign_run = foreign / run["id"]

    try:
        failure: StoreValidationError | None = None
        try:
            service.prepare(run["id"], expected_revision=run["revision"])
        except StoreValidationError as exc:
            failure = exc
        assert not (foreign_run / "logs").exists()
        assert not (foreign_run / "run-allocation.json").exists()
        assert failure is not None
        assert any(
            token in str(failure).lower()
            for token in ("artifact", "reparse", "link", "trusted")
        )
    finally:
        if link_created:
            if sys.platform == "win32":
                os.rmdir(runs_root)
            else:
                runs_root.unlink()
            displaced.rename(runs_root)


def test_prepare_revalidates_the_full_artifact_chain_after_factory_construction(
    tmp_path: Path,
) -> None:
    studio_root = tmp_path / "studio"
    bundle_dir = CatalogResolver.from_repository(REPO_ROOT).resolve(HEADLESS_SESSION).write_bundle(
        studio_root / "bundles" / "demo"
    )
    session_id = _session_id(bundle_dir)
    run_id = "run-ancestor-swap"
    run_dir = studio_root / "artifacts" / "runs" / run_id
    run_dir.mkdir(parents=True)
    bundle = BundleRecord(
        id="b" * 32,
        revision=1,
        created_at="2026-01-01T00:00:00Z",
        updated_at="2026-01-01T00:00:00Z",
        status="ready",
        payload={"bundle_path": "bundles/demo", "session_id": session_id},
    )
    factory = RuntimeFactory(
        TrustedRuntimeConfig(
            repo_root=REPO_ROOT,
            studio_root=studio_root,
            mujoco_host=(
                REPO_ROOT
                / "build"
                / "simstudio-test"
                / "lingtu_mujoco_headless.exe"
            ),
        ),
        coordinator_type=AllocatingFakeCoordinator,
        mujoco_process_type=FakeMujocoProcess,
    )
    session = factory.create_session(
        bundle=bundle,
        launch_profile="headless",
        artifact_root=run_dir,
        run_id=run_id,
    )

    artifacts_root = studio_root / "artifacts"
    displaced = tmp_path / "displaced-artifacts"
    foreign = tmp_path / "foreign-artifacts"
    (foreign / "runs" / run_id).mkdir(parents=True)
    artifacts_root.rename(displaced)
    link_created = False
    try:
        if sys.platform == "win32":
            created = subprocess.run(  # noqa: S603
                ["cmd", "/c", "mklink", "/J", str(artifacts_root), str(foreign)],  # noqa: S607
                capture_output=True,
                text=True,
                check=False,
            )
            if created.returncode != 0:
                displaced.rename(artifacts_root)
                pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")
        else:
            try:
                artifacts_root.symlink_to(foreign, target_is_directory=True)
            except (NotImplementedError, OSError):
                displaced.rename(artifacts_root)
                pytest.skip("directory symbolic links are unavailable in this test environment")
        link_created = True

        with pytest.raises((CoordinatorError, RunAllocationError), match=r"trusted|reparse|link|outside"):
            session.prepare()
        assert not (foreign / "runs" / run_id / "logs").exists()
        assert not (foreign / "runs" / run_id / "run-allocation.json").exists()
    finally:
        if link_created:
            if sys.platform == "win32":
                os.rmdir(artifacts_root)
            else:
                artifacts_root.unlink()
            displaced.rename(artifacts_root)
