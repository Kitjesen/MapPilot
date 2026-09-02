# ruff: noqa: S101
"""Simulation Product switch tests."""

from __future__ import annotations

import copy
import json
import os
import threading
import time
from functools import lru_cache
from pathlib import Path
from typing import Any

import pytest
from sim.catalog import CatalogResolver

import lingtu.sim.switch as sim_switch_module
from lingtu.control import ProductControl
from lingtu.product_lock import ProductControlBusy, ProductControlLock
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan
from lingtu.sim.identity import (
    ProcessIdentity,
    SimChildLedger,
    SimChildRecord,
    SimChildSnapshot,
)
from lingtu.switch_contracts import ProcessReport, SwitchFailed, SwitchRequest
from runtime.graph import (
    ProcessArtifact,
    ProcessCommand,
    ProcessReadiness,
    ProcessSpec,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATION_SESSION = "sim/sessions/examples/thunder_omni_contract/session.yaml"
OLD_PRODUCT_SESSION_ID = "1" * 32
TARGET_PRODUCT_SESSION_ID = "2" * 32


@pytest.fixture(autouse=True)
def _stable_product_session_id(monkeypatch: pytest.MonkeyPatch) -> None:
    next_id = iter(range(2, 100))

    def stable_id() -> str:
        value = next(next_id)
        return TARGET_PRODUCT_SESSION_ID if value == 2 else f"{value:032x}"

    monkeypatch.setattr(
        sim_switch_module,
        "new_product_session_id",
        stable_id,
    )


def test_sim_switch_module_does_not_expose_lifecycle_bypass() -> None:
    assert sim_switch_module.__all__ == ["SimSwitchRunner"]
    assert not hasattr(sim_switch_module, "CommittedPlan")
    assert not hasattr(sim_switch_module, "plan_switch")
    assert not hasattr(sim_switch_module, "execute_locked_switch")
    assert not hasattr(sim_switch_module, "load_committed_plan")


@lru_cache(maxsize=1)
def _resolved_simulation_snapshot() -> dict[str, Any]:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(
        REPO_ROOT / SIMULATION_SESSION
    )
    return {
        "schema": "lingtu.run_plan.simulation.v1",
        "session_source": SIMULATION_SESSION,
        "session": resolved.session,
        "physics_plan": resolved.physics_plan,
        "visual_plan": resolved.visual_plan,
        "sensor_plan": resolved.sensor_plan,
        "control_plan": resolved.control_plan,
        "transport_intent": resolved.transport_intent,
        "scenario_plan": resolved.scenario_plan,
    }


def _simulation_snapshot() -> dict[str, Any]:
    return copy.deepcopy(_resolved_simulation_snapshot())


def _plan(
    product: str,
    *,
    with_maps: bool = False,
    requires_map: bool = False,
    global_planner: str = "octoplanner3d",
    logical_roles: tuple[str, ...] = (),
    dependencies: tuple[ProcessArtifact, ...] = (),
) -> RunPlan:
    artifact = ProcessArtifact(
        path="sim/scripts/mujoco/product_acceptance.py",
    )
    process = ProcessSpec(
        name="sim_runtime",
        manager="direct",
        target=f"{product}-runtime",
        order=10,
        timeout_s=5,
        lifecycle="mode",
        command=ProcessCommand(
            argv=("python", artifact.path),
            cwd=".",
            env=(),
            artifact=artifact,
            readiness=ProcessReadiness("process"),
            dependencies=dependencies,
        ),
        provides=("sim_runtime",),
    )
    processes = [process]
    if with_maps:
        map_artifact = ProcessArtifact(
            path="sim/scripts/mujoco/product_acceptance.py",
        )
        processes.append(
            ProcessSpec(
                name="map_runtime",
                manager="direct",
                target="map_runtime",
                order=20,
                timeout_s=5,
                lifecycle="mode",
                command=ProcessCommand(
                    argv=("python", map_artifact.path),
                    cwd=".",
                    env=(("LINGTU_DDS_DOMAIN_ID", "17"),),
                    artifact=map_artifact,
                    readiness=ProcessReadiness("file", "mapd.status.json"),
                ),
                provides=("maps",),
            )
        )
    for role in logical_roles:
        processes.append(
            ProcessSpec(
                name=f"{role}_publisher",
                manager="direct",
                target=f"{role}-publisher",
                order=10,
                timeout_s=5,
                lifecycle="mode",
                command=ProcessCommand(
                    argv=("python", artifact.path),
                    cwd=".",
                    env=(),
                    artifact=artifact,
                    readiness=ProcessReadiness("process"),
                ),
                provides=(role,),
            )
        )
    return RunPlan.create(
        product=product,
        env="sim",
        robot="doso/thunder_v4",
        process_control="subprocess",
        modules=(),
        processes=tuple(processes),
        available_processes=tuple(processes),
        stop_before_start=tuple(item.target for item in processes),
        contracts=(f"lingtu.product.{product}.v1",),
        critical_modules=(),
        route_contract=None,
        host_config={},
        lifecycle={"product": product, "requires_map": requires_map},
        simulation=_simulation_snapshot(),
        native_nav={
            "global_planner": global_planner,
            "control_mode": "test",
            "publish_cmd_vel": False,
            "check_obstacle": False,
            "use_traversability_cost": False,
            "allow_teleop_takeover": False,
            "teleop_local_planner": False,
        },
    )


@pytest.mark.parametrize(
    (
        "global_planner",
        "required_artifact",
        "expected_octomap",
        "expected_occupancy",
    ),
    (
        ("octoplanner3d", "octomap", "/maps/yard/map.bt", ""),
        ("far", "occupancy", "", "/maps/yard/occupancy.yaml"),
    ),
)
def test_saved_map_environment_selects_one_backend_map(
    global_planner: str,
    required_artifact: str,
    expected_octomap: str,
    expected_occupancy: str,
) -> None:
    raw_identity = _native_identity("yard")
    raw_identity["artifacts"] = [
        artifact
        for artifact in raw_identity["artifacts"]
        if artifact["type"] in {"pointcloud", required_artifact}
    ]
    identity = sim_switch_module.map_identity_from_native(
        raw_identity,
        field_name="test map",
    )

    environment = sim_switch_module._saved_map_environment(
        _plan("nav", global_planner=global_planner),
        identity,
    )

    assert environment["LINGTU_SLAM_MAP"] == "/maps/yard/map.pcd"
    assert environment["OCTOPLANNER_MAP_PATH"] == expected_octomap
    assert environment["FAR_OCCUPANCY_PATH"] == expected_occupancy
    assert environment["EXPLORE_OCCUPANCY_PATH"] == ""
    assert not any(name.startswith("LINGTU_ACTIVE_") for name in environment)


@pytest.mark.parametrize("planner_artifact", ("octomap", "occupancy"))
def test_saved_map_record_accepts_one_planner_artifact(planner_artifact: str) -> None:
    native = _native_identity("yard")
    record = {
        "map_id": native["map_id"],
        "content_epoch": native["content_epoch"],
        "frame_id": native["frame_id"],
        "artifacts": [
            {"artifact_type": artifact["type"], "uri": artifact["uri"]}
            for artifact in native["artifacts"]
            if artifact["type"] in {"pointcloud", planner_artifact}
        ],
    }

    identity = sim_switch_module.map_identity_from_record(
        record,
        field_name="current run record map identity",
    )

    assert {artifact.artifact_type for artifact in identity.artifacts} == {
        "pointcloud",
        planner_artifact,
    }


def test_explore_saved_map_environment_exposes_coverage_occupancy() -> None:
    identity = sim_switch_module.map_identity_from_native(
        _native_identity("yard"),
        field_name="test map",
    )

    environment = sim_switch_module._saved_map_environment(
        _plan("explore"),
        identity,
    )

    assert environment["OCTOPLANNER_MAP_PATH"] == "/maps/yard/map.bt"
    assert environment["FAR_OCCUPANCY_PATH"] == ""
    assert environment["EXPLORE_OCCUPANCY_PATH"] == "/maps/yard/occupancy.yaml"


class RecordingRunner:
    def __init__(self, state_dir: Path, *, expect_current_absent: bool = True) -> None:
        self.state_dir = state_dir
        self.expect_current_absent = expect_current_absent
        self.calls: list[tuple[str, Path]] = []
        self.applied_product_session_ids: list[str | None] = []
        self.quiesced_product_session_ids: list[str] = []
        self.stopped_product_session_ids: list[str] = []

    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str | None = None,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        assert timeout_s is None
        self.applied_product_session_ids.append(product_session_id)
        assert run_plan_path.is_absolute()
        assert run_plan_path.is_file()
        if self.expect_current_absent:
            assert not (self.state_dir / "current.json").exists()
        plan = RunPlan.load(run_plan_path)
        self.calls.append(("apply", run_plan_path))
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            ok=True,
            status="active",
        )

    def quiesce(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        _ = timeout_s
        self.quiesced_product_session_ids.append(product_session_id)
        assert run_plan_path.is_absolute()
        assert run_plan_path.is_file()
        plan = RunPlan.load(run_plan_path)
        self.calls.append(("quiesce", run_plan_path))
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="quiesce",
            ok=True,
            status="stopped",
        )

    def stop(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        _ = timeout_s
        self.stopped_product_session_ids.append(product_session_id)
        assert run_plan_path.is_absolute()
        assert run_plan_path.is_file()
        plan = RunPlan.load(run_plan_path)
        self.calls.append(("stop", run_plan_path))
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="stop",
            ok=True,
            status="stopped",
        )


class SimulatedCrash(BaseException):
    """Model process death without entering the ordinary rollback handler."""


class CrashAfterPreviousQuiescedRunner(RecordingRunner):
    def __init__(self, state_dir: Path, *, target_path: Path) -> None:
        super().__init__(state_dir, expect_current_absent=False)
        self.target_path = target_path

    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str | None = None,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        self.applied_product_session_ids.append(product_session_id)
        assert timeout_s is None
        assert run_plan_path == self.target_path
        self.calls.append(("apply", run_plan_path))
        raise SimulatedCrash("power loss after previous quiesce")


class FailedRollbackRunner(RecordingRunner):
    def __init__(
        self,
        state_dir: Path,
        *,
        previous_path: Path,
        target_path: Path,
    ) -> None:
        super().__init__(state_dir, expect_current_absent=False)
        self.previous_path = previous_path
        self.target_path = target_path

    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str | None = None,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        assert timeout_s is None
        self.applied_product_session_ids.append(product_session_id)
        self.calls.append(("apply", run_plan_path))
        if run_plan_path == self.target_path:
            raise RuntimeError("target launch failed")
        assert run_plan_path == self.previous_path
        raise RuntimeError("previous restore failed")

class LockCheckingRunner(RecordingRunner):
    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str | None = None,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        outcomes: list[str] = []

        def contend_for_lock() -> None:
            try:
                with ProductControlLock(
                    self.state_dir,
                    environment={},
                    timeout_s=0,
                ):
                    outcomes.append("acquired")
            except ProductControlBusy:
                outcomes.append("busy")

        contender = threading.Thread(target=contend_for_lock)
        contender.start()
        contender.join(timeout=2)
        assert not contender.is_alive()
        assert outcomes == ["busy"]
        return super().apply(
            run_plan_path,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )


class ForbiddenSystemdRunner:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def __getattr__(self, name: str) -> Any:
        self.calls.append(name)
        raise AssertionError(f"systemd runner must not be called: {name}")


class RollbackRunner:
    def __init__(
        self,
        *,
        current_path: Path,
        expected_current: bytes,
        old_path: Path,
        target_path: Path,
    ) -> None:
        self.current_path = current_path
        self.expected_current = expected_current
        self.old_path = old_path
        self.target_path = target_path
        self.calls: list[tuple[str, Path]] = []

    def _record(self, action: str, path: Path) -> None:
        assert path.is_absolute()
        assert path.is_file()
        assert self.current_path.read_bytes() == self.expected_current
        self.calls.append((action, path))

    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str | None = None,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        assert timeout_s is None
        assert product_session_id is not None
        self._record("apply", run_plan_path)
        if run_plan_path == self.target_path:
            raise RuntimeError("target launch failed")
        assert run_plan_path == self.old_path
        plan = RunPlan.load(run_plan_path)
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            ok=True,
            status="active",
        )

    def quiesce(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        assert timeout_s is None
        assert product_session_id
        self._record("quiesce", run_plan_path)
        plan = RunPlan.load(run_plan_path)
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="quiesce",
            ok=True,
            status="stopped",
        )

    def stop(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        _ = product_session_id
        raise AssertionError(f"unexpected stop: {run_plan_path}, {timeout_s}")


def _write_current(state_dir: Path, plan: RunPlan) -> tuple[Path, Path, bytes]:
    plan_path = (state_dir / f"plan-{OLD_PRODUCT_SESSION_ID}.json").resolve()
    plan.write(plan_path)
    current_path = state_dir / "current.json"
    current_path.write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": plan.product,
                "product_variant": plan.product_variant,
                "env": plan.env,
                "run_plan_path": str(plan_path),
                "product_session_id": OLD_PRODUCT_SESSION_ID,
                "map_name": None,
                "map_identity": None,
                "committed_at": 1.0,
            },
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    os.chmod(current_path, 0o600)
    if plan.has_process("maps"):
        _write_mapd_runtime_evidence(state_dir, plan)
    return current_path, plan_path, current_path.read_bytes()


def _write_mapd_runtime_evidence(
    state_dir: Path,
    plan: RunPlan,
) -> Path:
    identity = ProcessIdentity.current(os.getpid())
    started_wall_ns = time.time_ns()
    SimChildLedger(state_dir).replace(
        SimChildSnapshot.create(
            product_session_id=OLD_PRODUCT_SESSION_ID,
            children=(
                SimChildRecord(
                    target="map_runtime",
                    process_identity=identity,
                    process_group=identity.pid,
                    started_wall_ns=started_wall_ns,
                    launch_id="9" * 64,
                ),
            ),
        )
    )
    generation = 7
    payload = {
        "schema_version": "lingtu.maps.runtime.v1",
        "process": "mapd",
        "native_product": {
            "product": plan.product,
            "product_session_id": OLD_PRODUCT_SESSION_ID,
        },
        "producer_boot_id": "mapd-boot",
        "status": "ready",
        "ready": True,
        "running": True,
        "live": True,
        "reset_epoch": 1,
        "observation_sequence": 4,
        "generation": generation,
        "queue_depth": 0,
        "live_points": 120,
        "voxel_points": 100,
        "voxel_cells": 80,
        "voxel_snapshot_omitted_cells": 0,
        "voxel_capacity_rejections": 0,
        "accumulated_cells": 90,
        "accumulated_snapshot_cells": 90,
        "accumulated_capacity_rejections": 0,
        "capacity_limited": False,
        "pose_quality": 1.0,
        "pose_state": "TRACKING",
        "pose_reason": "mujoco_navigation_fixture",
        "accepted_observations": 4,
        "processed_observations": 4,
        "replaced_observations": 0,
        "stale_observations": 0,
        "invalid_observations": 0,
        "dds_received": 4,
        "dds_decoded": 4,
        "dds_rejected": 0,
        "dds_write_attempts": 20,
        "dds_write_failures": 0,
        "dds_serialization_rejections": 0,
        "dds_scene_oversize_rejections": 0,
        "dds_unhealthy_writers": 0,
        "required_publications_ready": True,
        "current_generation_published": True,
        "state_published_generation": generation,
        "realtime_clouds_published_generation": generation,
        "map_layers_published_generation": generation,
        "scene_published_generation": generation,
        "engine_error": "",
        "input_error": "",
        "output_error": "",
    }
    path = state_dir / "mapd.status.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    assert path.stat().st_mtime_ns >= started_wall_ns
    return path


def _switch_journal_payload(
    *,
    target_path: Path,
    target: RunPlan,
    target_product_session_id: str = TARGET_PRODUCT_SESSION_ID,
) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim_switch_journal.v3",
        "state": "prepared",
        "target": {
            "run_plan_path": str(target_path),
            "product_session_id": target_product_session_id,
        },
        "previous": None,
        "map_activation": None,
    }


def _native_identity(map_id: str | None) -> dict[str, Any]:
    if map_id is None:
        return {
            "present": False,
            "map_id": "",
            "content_epoch": 0,
            "frame_id": "",
            "artifacts": [],
        }
    return {
        "present": True,
        "map_id": map_id,
        "content_epoch": 1,
        "frame_id": "map",
        "artifacts": [
            {
                "type": artifact_type,
                "uri": f"/maps/{map_id}/{filename}",
            }
            for artifact_type, filename in (
                ("pointcloud", "map.pcd"),
                ("octomap", "map.bt"),
                ("occupancy", "occupancy.yaml"),
            )
        ],
    }


def _mapctl_payload(
    operation: str,
    *,
    target: dict[str, Any],
    previous: dict[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": "lingtu.map_activation.v2",
        "request_id": "request-1",
        "operation": operation,
        "accepted": True,
        "message": "ok",
        "changed": operation in {"stage", "restore"},
        "producer_boot_id": "" if operation == "prepare" else "mapd-boot",
        "activation_token": "opaque-token" if operation in {"prepare", "stage"} else "",
        "target": copy.deepcopy(target),
        "previous": copy.deepcopy(previous),
        "active": copy.deepcopy(previous if operation in {"prepare", "restore"} else target),
    }


@pytest.mark.parametrize(
    "mutation",
    (
        lambda payload: payload.update(changed=1),
        lambda payload: payload.update(producer_boot_id=""),
        lambda payload: payload.update(activation_token=""),
        lambda payload: payload.update(operation="STAGE"),
        lambda payload: payload.update(active=_native_identity(None)),
    ),
)
def test_mapctl_stage_receipt_rejects_control_semantic_drift(
    mutation: Any,
) -> None:
    payload = _mapctl_payload(
        "stage",
        target=_native_identity("yard"),
        previous=_native_identity(None),
    )
    mutation(payload)

    with pytest.raises(RuntimeError, match="native sim map"):
        sim_switch_module._validate_mapctl_receipt(
            payload,
            operation="stage",
        )


def test_mapctl_stage_transition_rejects_prepared_token_drift() -> None:
    target = _native_identity("yard")
    previous = _native_identity(None)
    prepared = _mapctl_payload("prepare", target=target, previous=previous)
    staged = _mapctl_payload("stage", target=target, previous=previous)
    staged["activation_token"] = "foreign-token"

    with pytest.raises(RuntimeError, match="activation token drifted"):
        sim_switch_module._require_mapctl_transition(
            staged,
            prepared=prepared,
            operation="stage",
        )


@pytest.mark.parametrize("operation", ("stage", "restore", "verify"))
def test_mapctl_receipt_requires_lowercase_operation(operation: str) -> None:
    payload = _mapctl_payload(
        operation,
        target=_native_identity("yard"),
        previous=_native_identity(None),
    )
    sim_switch_module._validate_mapctl_receipt(
        payload,
        operation=operation,
    )

    payload["operation"] = operation.upper()
    with pytest.raises(RuntimeError, match="identity is invalid"):
        sim_switch_module._validate_mapctl_receipt(
            payload,
            operation=operation,
        )


@pytest.mark.parametrize(
    "operation",
    ("verify", "restore"),
)
def test_mapctl_nonstage_receipt_requires_empty_token_and_exact_semantics(
    operation: str,
) -> None:
    payload = _mapctl_payload(
        operation,
        target=_native_identity("yard"),
        previous=_native_identity(None),
    )
    payload["activation_token"] = "must-be-empty"

    with pytest.raises(RuntimeError, match="activation_token"):
        sim_switch_module._validate_mapctl_receipt(
            payload,
            operation=operation,
        )


def test_mapctl_verify_receipt_cannot_claim_changed() -> None:
    payload = _mapctl_payload(
        "verify",
        target=_native_identity("yard"),
        previous=_native_identity(None),
    )
    payload["changed"] = True

    with pytest.raises(RuntimeError, match="must not report a mutation"):
        sim_switch_module._validate_mapctl_receipt(
            payload,
            operation="verify",
        )


@pytest.mark.parametrize("operation", ("stage", "restore"))
def test_mapctl_mutations_use_mapd_dds_domain(
    operation: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _mapctl_payload(
        operation,
        target=_native_identity("yard"),
        previous=_native_identity(None),
    )
    commands: list[list[str]] = []

    def run(command: list[str], **_kwargs: Any) -> Any:
        commands.append(command)
        return type(
            "Completed",
            (),
            {"returncode": 0, "stdout": json.dumps(payload), "stderr": ""},
        )()

    monkeypatch.setattr(sim_switch_module.subprocess, "run", run)

    operand = "yard" if operation == "stage" else "opaque-token"
    sim_switch_module._mapctl(
        {
            "LINGTU_MAPCTL_BIN": "mapctl",
            "LINGTU_DDS_DOMAIN_ID": "17",
        },
        operation,
        operand,
        timeout_s=1.0,
    )

    assert "--offline" not in commands[0]
    assert commands[0][1:3] == [operation, operand]
    timeout_index = commands[0].index("--timeout-ms")
    assert commands[0][timeout_index + 1] == "1000"
    assert commands[0][-2:] == ["--domain-id", "17"]


def test_mapctl_ignores_legacy_map_dir(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _mapctl_payload(
        "stage",
        target=_native_identity("yard"),
        previous=_native_identity(None),
    )
    commands: list[list[str]] = []

    def run(command: list[str], **_kwargs: Any) -> Any:
        commands.append(command)
        return type(
            "Completed",
            (),
            {"returncode": 0, "stdout": json.dumps(payload), "stderr": ""},
        )()

    monkeypatch.setattr(sim_switch_module.subprocess, "run", run)

    sim_switch_module._mapctl(
        {
            "HOME": str(tmp_path),
            "MAP_DIR": str(tmp_path / "legacy"),
            "LINGTU_MAPCTL_BIN": "mapctl",
            "LINGTU_DDS_DOMAIN_ID": "17",
        },
        "stage",
        "yard",
        timeout_s=1.0,
    )

    map_root_index = commands[0].index("--map-root") + 1
    assert Path(commands[0][map_root_index]) == (
        tmp_path / "data" / "lingtu" / "maps"
    ).resolve()


def test_mapctl_requires_exact_sim_mapd_domain(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    called = False

    def run(*_args: Any, **_kwargs: Any) -> Any:
        nonlocal called
        called = True
        raise AssertionError("mapctl must fail before subprocess launch")

    monkeypatch.setattr(sim_switch_module.subprocess, "run", run)

    with pytest.raises(RuntimeError, match="DDS domain"):
        sim_switch_module._mapctl(
            {"LINGTU_MAPCTL_BIN": "mapctl"},
            "stage",
            "yard",
            timeout_s=1.0,
        )

    assert called is False


@pytest.mark.parametrize(
    ("field", "value"),
    (("changed", True), ("producer_boot_id", "unexpected-boot")),
)
def test_mapctl_prepare_receipt_is_strictly_read_only(
    field: str,
    value: Any,
) -> None:
    payload = _mapctl_payload(
        "prepare",
        target=_native_identity("yard"),
        previous=_native_identity(None),
    )
    payload[field] = value

    with pytest.raises(RuntimeError, match="read-only|producer_boot_id|changed"):
        sim_switch_module._validate_mapctl_receipt(
            payload,
            operation="prepare",
        )


def test_switch_journal_deserialization_rejects_missing_activation_token(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    target_path = target.write(tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json")
    payload = _switch_journal_payload(target_path=target_path, target=target)
    activation = _mapctl_payload(
        "prepare",
        target=_native_identity("yard"),
        previous=_native_identity(None),
    )
    activation["activation_token"] = ""
    payload["map_activation"] = activation
    (tmp_path / "switch.json").write_text(
        json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.chmod(tmp_path / "switch.json", 0o600)
    runner = RecordingRunner(tmp_path, expect_current_absent=False)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="map_activation"):
        control._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    assert runner.calls == []
    assert (tmp_path / "switch.json").is_file()


def test_saved_map_switch_requires_ready_committed_map_runtime_before_prepare(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop")
    _write_current(tmp_path, previous)
    target = _plan("nav", with_maps=True)
    mapctl_calls: list[str] = []
    monkeypatch.setattr(
        sim_switch_module,
        "_mapctl",
        lambda *_args, **_kwargs: mapctl_calls.append("called"),
    )
    runner = RecordingRunner(tmp_path, expect_current_absent=False)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="typed readiness"):
        control._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )

    assert mapctl_calls == []
    assert runner.calls == []


@pytest.mark.parametrize("failure", ("absent", "stale", "foreign", "old_session"))
def test_saved_map_switch_rejects_untrusted_mapd_readiness_before_prepare(
    failure: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    readiness = tmp_path / "mapd.status.json"
    if failure == "absent":
        readiness.unlink()
    elif failure == "stale":
        stale_ns = previous_path.stat().st_mtime_ns - 1
        os.utime(readiness, ns=(stale_ns, stale_ns))
    elif failure == "foreign":
        payload = json.loads(readiness.read_text(encoding="utf-8"))
        payload["native_product"]["product_session_id"] = "f" * 32
        readiness.write_text(json.dumps(payload), encoding="utf-8")
    else:
        ledger = SimChildLedger(tmp_path)
        snapshot = ledger.load()
        assert snapshot is not None
        ledger.replace(
            SimChildSnapshot.create(
                product_session_id="2" * 32,
                children=snapshot.children,
            )
        )
    target = _plan("nav", with_maps=True)
    mapctl_calls: list[str] = []
    monkeypatch.setattr(
        sim_switch_module,
        "_mapctl",
        lambda *_args, **_kwargs: mapctl_calls.append("called"),
    )
    runner = RecordingRunner(tmp_path, expect_current_absent=False)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="map runtime readiness|child ledger"):
        control._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )

    assert mapctl_calls == []
    assert runner.calls == []


def test_saved_map_readiness_uses_exact_previous_sensor_roles(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan(
        "inspection",
        with_maps=True,
        logical_roles=("lidar", "imu", "camera"),
    )
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    captured: dict[str, object] = {}

    def capture_readiness(*_args: object, **kwargs: object) -> dict[str, object]:
        captured.update(kwargs)
        return {"ready": True}

    monkeypatch.setattr(sim_switch_module, "load_typed_readiness", capture_readiness)

    sim_switch_module._require_saved_map_previous(
        sim_switch_module._CommittedPlan(
            path=previous_path,
            plan=previous,
            product_session_id=OLD_PRODUCT_SESSION_ID,
        ),
        tmp_path,
    )

    assert captured["lidar_required"] is True
    assert captured["imu_required"] is True
    assert captured["camera_required"] is True


def test_saved_map_readiness_waits_for_a_transient_map_reset(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("map", with_maps=True, logical_roles=("lidar", "imu"))
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    calls = 0
    sleeps: list[float] = []

    def pending_then_ready(*_args: object, **_kwargs: object) -> dict[str, object]:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise sim_switch_module.SimReadinessPending("waiting for observation")
        return {"ready": True}

    monkeypatch.setattr(sim_switch_module, "load_typed_readiness", pending_then_ready)
    monkeypatch.setattr(sim_switch_module.time, "sleep", sleeps.append)

    sim_switch_module._require_saved_map_previous(
        sim_switch_module._CommittedPlan(
            path=previous_path,
            plan=previous,
            product_session_id=OLD_PRODUCT_SESSION_ID,
        ),
        tmp_path,
    )

    assert calls == 2
    assert sleeps == [0.02]


@pytest.mark.parametrize("target_product", ("nav", "inspection"))
def test_saved_map_switch_journals_token_before_stage_and_binds_final_plan(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    target_product: str,
) -> None:
    previous = _plan("teleop", with_maps=True)
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    target = _plan(target_product, with_maps=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)
    mapctl_calls: list[str] = []

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert timeout_s > 0
        assert _environment["LINGTU_DDS_DOMAIN_ID"] == "17"
        mapctl_calls.append(operation)
        if operation == "prepare":
            assert operand == "yard"
        elif operation == "stage":
            assert operand == "yard"
        else:
            assert operand == "opaque-token"
        if operation == "stage":
            assert runner.calls == []
            journal = json.loads((tmp_path / "switch.json").read_text(encoding="utf-8"))
            assert journal["schema"] == "lingtu.sim_switch_journal.v3"
            assert journal["map_activation"] == {
                "activation_token": "opaque-token",
                "target": target_identity,
                "previous": previous_identity,
            }
        return _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    runner = RecordingRunner(tmp_path, expect_current_absent=False)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    report = control._switch(
        SwitchRequest(target_product=target_product, map_name="yard"),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    assert mapctl_calls == ["prepare", "stage", "verify"]
    assert runner.calls[0] == ("quiesce", previous_path)
    rebound_path = runner.calls[1][1]
    rebound = RunPlan.load(rebound_path)
    assert rebound != target
    assert rebound.native_process_environment["LINGTU_MAP_ID"] == "yard"
    assert rebound.native_process_environment["LINGTU_SLAM_MODE"] == "localization"
    assert rebound.native_process_environment["LINGTU_SLAM_MAP"] == (
        "/maps/yard/map.pcd"
    )
    assert rebound.native_process_environment["OCTOPLANNER_MAP_PATH"] == (
        "/maps/yard/map.bt"
    )
    assert rebound.native_process_environment["FAR_OCCUPANCY_PATH"] == ""
    assert rebound.native_process_environment["EXPLORE_OCCUPANCY_PATH"] == ""
    assert not any(
        name.startswith("LINGTU_ACTIVE_")
        for name in rebound.native_process_environment
    )
    current = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert current["product_session_id"] == TARGET_PRODUCT_SESSION_ID
    assert current["map_name"] == "yard"
    assert current["map_identity"]["map_id"] == "yard"
    assert not (tmp_path / "switch.json").exists()
    assert not previous_path.exists()


def test_saved_map_cold_start_uses_target_map_runtime(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("tracking", with_maps=True, requires_map=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)
    mapctl_calls: list[str] = []
    runner = RecordingRunner(tmp_path)

    def mapctl(
        environment: dict[str, str],
        operation: str,
        operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert environment["LINGTU_DDS_DOMAIN_ID"] == "17"
        assert timeout_s > 0
        mapctl_calls.append(operation)
        if operation == "prepare":
            assert operand == "yard"
            assert runner.calls == []
        elif operation == "stage":
            assert operand == "yard"
            assert len(runner.calls) == 1 and runner.calls[0][0] == "apply"
        else:
            assert operand == "opaque-token"
        return _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    report = control._switch(
        SwitchRequest(target_product="tracking", map_name="yard"),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    assert mapctl_calls == ["prepare", "stage", "verify"]
    assert [action for action, _path in runner.calls] == ["apply"]


@pytest.mark.parametrize(
    ("initial_pose", "relocalize", "expected_seed"),
    (
        (None, True, None),
        ((1.25, -2.5, 0.3), True, ("1.25", "-2.5", "0", "0.29999999999999999")),
        (None, False, None),
    ),
)
def test_saved_map_switch_binds_launch_localization_before_apply(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    initial_pose: tuple[float, float, float] | None,
    relocalize: bool,
    expected_seed: tuple[str, str, str, str] | None,
) -> None:
    previous = _plan("teleop", with_maps=True)
    _write_current(tmp_path, previous)
    target = _plan("nav", with_maps=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)

    monkeypatch.setattr(
        sim_switch_module,
        "_mapctl",
        lambda _environment, operation, _operand, *, timeout_s: _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        ),
    )

    runner = RecordingRunner(tmp_path, expect_current_absent=False)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    report = control._switch(
        SwitchRequest(
            target_product="nav",
            map_name="yard",
            relocalize=relocalize,
            initial_pose=initial_pose,
        ),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    assert report.phases[-2:] == ["localization_initialized", "committed"]
    applied_plan = RunPlan.load(runner.calls[-1][1])
    native_environment = applied_plan.native_process_environment
    seed_keys = (
        "LINGTU_SLAM_TRACK_INITIAL_X",
        "LINGTU_SLAM_TRACK_INITIAL_Y",
        "LINGTU_SLAM_TRACK_INITIAL_Z",
        "LINGTU_SLAM_TRACK_INITIAL_YAW",
    )
    if expected_seed is None:
        assert all(key not in native_environment for key in seed_keys)
    else:
        assert tuple(native_environment[key] for key in seed_keys) == expected_seed


def test_saved_map_seed_cannot_be_silently_ignored(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("nav", with_maps=True)
    runner = RecordingRunner(tmp_path)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="initial_pose requires relocalize=True"):
        control._switch(
            SwitchRequest(
                target_product="nav",
                map_name="yard",
                relocalize=False,
                initial_pose=(1.0, 2.0, 0.3),
            ),
            state_dir=tmp_path,
        )

    assert runner.calls == []
    assert list(tmp_path.glob("plan-*.json")) == []


def test_saved_map_localization_failure_rolls_back_transaction(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    current_path, previous_path, current_bytes = _write_current(tmp_path, previous)
    target = _plan("nav", with_maps=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)
    events: list[str] = []

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        _operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        events.append(f"map:{operation}")
        return _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )

    class EventRunner(RecordingRunner):
        def apply(self, run_plan_path: Path, **kwargs: Any) -> ProcessReport:
            events.append("previous:apply" if run_plan_path == previous_path else "target:apply")
            report = super().apply(run_plan_path, **kwargs)
            if run_plan_path != previous_path:
                raise RuntimeError("alignment rejected")
            return report

        def quiesce(self, run_plan_path: Path, **kwargs: Any) -> ProcessReport:
            events.append(
                "previous:quiesce" if run_plan_path == previous_path else "target:quiesce"
            )
            return super().quiesce(run_plan_path, **kwargs)

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=EventRunner(tmp_path, expect_current_absent=False),
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="alignment rejected") as failure:
        control._switch(
            SwitchRequest(
                target_product="nav",
                map_name="yard",
                initial_pose=(1.0, 2.0, 0.3),
            ),
            state_dir=tmp_path,
        )

    assert failure.value.report.status == "failed_rolled_back"
    assert events == [
        "map:prepare",
        "map:stage",
        "previous:quiesce",
        "target:apply",
        "map:restore",
        "target:quiesce",
        "previous:apply",
    ]
    assert current_path.read_bytes() == current_bytes
    assert not (tmp_path / "switch.json").exists()


def test_saved_map_failure_restores_map_before_previous_product(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    current_path, previous_path, current_bytes = _write_current(tmp_path, previous)
    target = _plan("nav", with_maps=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)
    events: list[str] = []

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        _operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert timeout_s > 0
        events.append(f"map:{operation}")
        return _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )

    class FailedTargetRunner(RecordingRunner):
        def apply(
            self,
            run_plan_path: Path,
            *,
            product_session_id: str | None = None,
            timeout_s: float | None = None,
        ) -> ProcessReport:
            assert timeout_s is None
            self.calls.append(("apply", run_plan_path))
            if run_plan_path != previous_path:
                events.append("target:apply")
                raise RuntimeError("target launch failed")
            events.append("previous:apply")
            return ProcessReport(
                product=previous.product,
                env="sim",
                action="apply",
                ok=True,
                status="active",
            )

        def quiesce(
            self,
            run_plan_path: Path,
            *,
            product_session_id: str,
            timeout_s: float | None = None,
        ) -> ProcessReport:
            events.append(
                "previous:quiesce" if run_plan_path == previous_path else "target:quiesce"
            )
            return super().quiesce(
                run_plan_path,
                product_session_id=product_session_id,
                timeout_s=timeout_s,
            )

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    runner = FailedTargetRunner(tmp_path, expect_current_absent=False)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="target launch failed") as failure:
        control._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )

    assert failure.value.report.status == "failed_rolled_back"
    assert events == [
        "map:prepare",
        "map:stage",
        "previous:quiesce",
        "target:apply",
        "map:restore",
        "target:quiesce",
        "previous:apply",
    ]
    assert current_path.read_bytes() == current_bytes
    assert not (tmp_path / "switch.json").exists()


def test_bad_verify_receipt_blocks_commit_and_restores_previous_state(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    current_path, _previous_path, current_bytes = _write_current(tmp_path, previous)
    target = _plan("nav", with_maps=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)
    operations: list[str] = []

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        _operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert timeout_s > 0
        operations.append(operation)
        payload = _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )
        if operation == "verify":
            payload["changed"] = True
        return payload

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=RecordingRunner(tmp_path, expect_current_absent=False),
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="must not report a mutation") as failure:
        control._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )

    assert failure.value.report.status == "failed_rolled_back"
    assert operations == ["prepare", "stage", "verify", "restore"]
    assert current_path.read_bytes() == current_bytes
    assert not (tmp_path / "switch.json").exists()


def test_bad_restore_receipt_retains_journal_and_fails_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    target = _plan("nav", with_maps=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        _operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert timeout_s > 0
        payload = _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )
        if operation == "restore":
            raise RuntimeError("native sim map restore receipt is invalid")
        return payload

    class FailedTargetRunner(RecordingRunner):
        def apply(
            self,
            run_plan_path: Path,
            *,
            product_session_id: str | None = None,
            timeout_s: float | None = None,
        ) -> ProcessReport:
            assert timeout_s is None
            if run_plan_path != previous_path:
                raise RuntimeError("target failed")
            return ProcessReport(
                product=previous.product,
                env="sim",
                action="apply",
                ok=True,
                status="active",
            )

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=FailedTargetRunner(tmp_path, expect_current_absent=False),
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="target failed") as failure:
        control._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )

    assert failure.value.report.status == "rollback_failed"
    assert any(item.startswith("map_failed:") for item in failure.value.report.cleanup)
    assert (tmp_path / "switch.json").is_file()


def test_failed_target_stop_restores_map_but_not_previous_product(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    target = _plan("nav", with_maps=True)
    operations: list[str] = []

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        _operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert timeout_s > 0
        operations.append(operation)
        return _mapctl_payload(
            operation,
            target=_native_identity("yard"),
            previous=_native_identity(None),
        )

    class FailedStopRunner(RecordingRunner):
        def apply(self, run_plan_path: Path, **_kwargs: Any) -> ProcessReport:
            if run_plan_path == previous_path:
                raise AssertionError("previous Product must remain stopped")
            raise RuntimeError("target launch failed")

        def quiesce(self, run_plan_path: Path, **kwargs: Any) -> ProcessReport:
            if run_plan_path != previous_path:
                raise RuntimeError("target stop failed")
            return super().quiesce(run_plan_path, **kwargs)

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=FailedStopRunner(tmp_path, expect_current_absent=False),
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="target launch failed") as failure:
        control._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )

    assert failure.value.report.status == "rollback_failed"
    assert operations == ["prepare", "stage", "restore"]
    assert failure.value.report.cleanup == [
        "map:restored",
        "target_failed:target stop failed",
    ]
    assert (tmp_path / "switch.json").is_file()


def test_saved_map_reconcile_previous_restores_exact_token_before_new_switch(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    saved_target = _plan("nav", with_maps=True)
    next_target = _plan("teleop_avoid")
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)
    events: list[str] = []

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        _operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert timeout_s > 0
        events.append(f"map:{operation}")
        return _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )

    class CrashTargetRunner(RecordingRunner):
        def apply(
            self,
            run_plan_path: Path,
            *,
            product_session_id: str | None = None,
            timeout_s: float | None = None,
        ) -> ProcessReport:
            assert timeout_s is None
            self.calls.append(("apply", run_plan_path))
            if run_plan_path != previous_path:
                raise SimulatedCrash("lost power after staged map")
            return ProcessReport(
                product=previous.product,
                env="sim",
                action="apply",
                ok=True,
                status="active",
            )

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    first_runner = CrashTargetRunner(tmp_path, expect_current_absent=False)
    first = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=first_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(first, "_resolve", lambda *_args, **_kwargs: saved_target)
    with pytest.raises(SimulatedCrash, match="staged map"):
        first._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )

    recovery = RecordingRunner(tmp_path, expect_current_absent=False)
    second = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=recovery,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(second, "_resolve", lambda *_args, **_kwargs: next_target)
    report = second._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    assert recovery.calls[:2] == [
        ("quiesce", previous_path),
        ("apply", previous_path),
    ]
    assert events == ["map:prepare", "map:stage", "map:restore"]
    assert not (tmp_path / "switch.json").exists()
    assert not first_runner.calls[-1][1].exists()


def test_saved_map_reconcile_unknown_current_keeps_journal_and_does_not_restore(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    saved_target = _plan("nav", with_maps=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)
    operations: list[str] = []

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        _operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert timeout_s > 0
        operations.append(operation)
        return _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )

    class CrashTargetRunner(RecordingRunner):
        def apply(
            self,
            run_plan_path: Path,
            *,
            product_session_id: str | None = None,
            timeout_s: float | None = None,
        ) -> ProcessReport:
            assert timeout_s is None
            self.calls.append(("apply", run_plan_path))
            if run_plan_path != previous_path:
                raise SimulatedCrash("lost power")
            return super().apply(
                run_plan_path,
                product_session_id=product_session_id,
                timeout_s=timeout_s,
            )

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    first_runner = CrashTargetRunner(tmp_path, expect_current_absent=False)
    first = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=first_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(first, "_resolve", lambda *_args, **_kwargs: saved_target)
    with pytest.raises(SimulatedCrash):
        first._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )
    current_path.unlink()

    recovery = RecordingRunner(tmp_path)
    second = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=recovery,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(second, "_resolve", lambda *_args, **_kwargs: _plan("teleop_avoid"))
    with pytest.raises(SwitchFailed, match="previous RunPlan identity"):
        second._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    assert operations == ["prepare", "stage"]
    assert len(recovery.calls) == 1 and recovery.calls[0][0] == "quiesce"
    assert (tmp_path / "switch.json").is_file()


def test_saved_map_reconcile_committed_target_verifies_token_before_clearing_journal(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop", with_maps=True)
    _write_current(tmp_path, previous)
    saved_target = _plan("nav", with_maps=True)
    target_identity = _native_identity("yard")
    previous_identity = _native_identity(None)
    operations: list[str] = []

    def mapctl(
        _environment: dict[str, str],
        operation: str,
        _operand: str,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        assert timeout_s > 0
        operations.append(operation)
        return _mapctl_payload(
            operation,
            target=target_identity,
            previous=previous_identity,
        )

    remove_journal = sim_switch_module._remove_switch_journal

    def crash_after_commit(*_args: Any, **_kwargs: Any) -> None:
        raise SimulatedCrash("lost power after saved current commit")

    monkeypatch.setattr(sim_switch_module, "_mapctl", mapctl)
    monkeypatch.setattr(sim_switch_module, "_remove_switch_journal", crash_after_commit)
    first = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=RecordingRunner(tmp_path, expect_current_absent=False),
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(first, "_resolve", lambda *_args, **_kwargs: saved_target)
    with pytest.raises(SimulatedCrash, match="saved current commit"):
        first._switch(
            SwitchRequest(target_product="nav", map_name="yard"),
            state_dir=tmp_path,
        )

    monkeypatch.setattr(sim_switch_module, "_remove_switch_journal", remove_journal)
    second_target = _plan("teleop_avoid")
    second = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=RecordingRunner(tmp_path, expect_current_absent=False),
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(second, "_resolve", lambda *_args, **_kwargs: second_target)
    report = second._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    assert operations == ["prepare", "stage", "verify", "verify"]
    assert not (tmp_path / "switch.json").exists()


def test_sim_switch_persists_exact_journal_before_target_apply(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop")
    current_path, previous_path, current_bytes = _write_current(tmp_path, previous)
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    runner = CrashAfterPreviousQuiescedRunner(
        tmp_path,
        target_path=target_path,
    )
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SimulatedCrash, match="power loss"):
        control._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    journal_path = tmp_path / "switch.json"
    payload = json.loads(journal_path.read_text(encoding="utf-8"))
    target_product_session_id = runner.applied_product_session_ids[0]
    assert target_product_session_id is not None
    assert payload == {
        "previous": {
            "product_session_id": OLD_PRODUCT_SESSION_ID,
            "run_plan_path": str(previous_path),
        },
        "schema": "lingtu.sim_switch_journal.v3",
        "map_activation": None,
        "state": "target_starting",
        "target": {
            "product_session_id": target_product_session_id,
            "run_plan_path": str(target_path),
        },
    }
    assert runner.calls == [
        ("quiesce", previous_path),
        ("apply", target_path),
    ]
    assert current_path.read_bytes() == current_bytes
    assert target_path.is_file()


def test_prepared_journal_has_no_runtime_side_effects_to_recover(
    tmp_path: Path,
) -> None:
    previous = _plan("teleop")
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    target.write(target_path)
    payload = _switch_journal_payload(target_path=target_path, target=target)
    payload["previous"] = {
        "product_session_id": OLD_PRODUCT_SESSION_ID,
        "run_plan_path": str(previous_path),
    }
    payload["map_activation"] = {
        "activation_token": "opaque-token",
        "target": _native_identity("yard"),
        "previous": _native_identity(None),
    }
    (tmp_path / "switch.json").write_text(json.dumps(payload), encoding="utf-8")
    runner = RecordingRunner(tmp_path, expect_current_absent=False)

    sim_switch_module._reconcile_incomplete_switch(tmp_path, runner, {})

    assert runner.calls == []
    assert not (tmp_path / "switch.json").exists()
    assert not target_path.exists()
    assert previous_path.is_file()


def test_reconcile_stops_restored_previous_before_reapplying_it(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop")
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    target.write(target_path)
    payload = _switch_journal_payload(target_path=target_path, target=target)
    payload["state"] = "target_starting"
    payload["previous"] = {
        "product_session_id": OLD_PRODUCT_SESSION_ID,
        "run_plan_path": str(previous_path),
    }
    (tmp_path / "switch.json").write_text(json.dumps(payload), encoding="utf-8")

    class RestoredPreviousRunner(RecordingRunner):
        def __init__(self) -> None:
            super().__init__(tmp_path, expect_current_absent=False)
            self.active_path: Path | None = previous_path

        def quiesce(
            self,
            run_plan_path: Path,
            *,
            product_session_id: str,
            timeout_s: float | None = None,
        ) -> ProcessReport:
            report = super().quiesce(
                run_plan_path,
                product_session_id=product_session_id,
                timeout_s=timeout_s,
            )
            if run_plan_path == self.active_path:
                self.active_path = None
            return report

        def apply(
            self,
            run_plan_path: Path,
            *,
            product_session_id: str | None = None,
            timeout_s: float | None = None,
        ) -> ProcessReport:
            if self.active_path is not None:
                raise RuntimeError("cannot replace a RunPlan while owned processes are active")
            report = super().apply(
                run_plan_path,
                product_session_id=product_session_id,
                timeout_s=timeout_s,
            )
            self.active_path = run_plan_path
            return report

    runner = RestoredPreviousRunner()
    monkeypatch.setattr(
        sim_switch_module,
        "_owned_child_session",
        lambda _state_root: OLD_PRODUCT_SESSION_ID,
    )

    sim_switch_module._reconcile_incomplete_switch(tmp_path, runner, {})

    assert runner.calls == [
        ("quiesce", previous_path),
        ("apply", previous_path),
    ]
    assert not (tmp_path / "switch.json").exists()
    assert not target_path.exists()


def test_reconcile_discards_interrupted_switch_after_product_stop(tmp_path: Path) -> None:
    previous = _plan("teleop")
    previous_path = (tmp_path / f"plan-{OLD_PRODUCT_SESSION_ID}.json").resolve()
    previous.write(previous_path)
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    target.write(target_path)
    payload = _switch_journal_payload(target_path=target_path, target=target)
    payload["state"] = "target_starting"
    payload["previous"] = {
        "product_session_id": OLD_PRODUCT_SESSION_ID,
        "run_plan_path": str(previous_path),
    }
    payload["map_activation"] = {
        "activation_token": "opaque-token",
        "target": _native_identity("yard"),
        "previous": _native_identity(None),
    }
    (tmp_path / "switch.json").write_text(json.dumps(payload), encoding="utf-8")
    runner = RecordingRunner(tmp_path)

    sim_switch_module._reconcile_incomplete_switch(tmp_path, runner, {})

    assert runner.calls == [("quiesce", target_path)]
    assert not (tmp_path / "switch.json").exists()
    assert not target_path.exists()
    assert not previous_path.exists()


def test_next_switch_stops_interrupted_target_and_restores_previous_first(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop")
    current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    crashing_runner = CrashAfterPreviousQuiescedRunner(
        tmp_path,
        target_path=target_path,
    )
    first = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=crashing_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(first, "_resolve", lambda *_args, **_kwargs: target)
    with pytest.raises(SimulatedCrash):
        first._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    recovery_runner = RecordingRunner(tmp_path, expect_current_absent=False)
    second = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=recovery_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(second, "_resolve", lambda *_args, **_kwargs: target)

    report = second._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    retry_path = Path(report.run_plan_path or "").resolve()
    assert retry_path != target_path
    assert recovery_runner.calls == [
        ("quiesce", target_path),
        ("quiesce", previous_path),
        ("apply", previous_path),
        ("quiesce", previous_path),
        ("apply", retry_path),
    ]
    current = json.loads(current_path.read_text(encoding="utf-8"))
    assert current["product_session_id"] == report.product_session_id
    assert not (tmp_path / "switch.json").exists()


def test_next_switch_stops_active_uncommitted_target_before_retry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    crashing_runner = RecordingRunner(tmp_path)
    first = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=crashing_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(first, "_resolve", lambda *_args, **_kwargs: target)
    commit_current = sim_switch_module._commit_current

    def crash_before_commit(*_args: Any, **_kwargs: Any) -> None:
        raise SimulatedCrash("power loss after target became active")

    monkeypatch.setattr(sim_switch_module, "_commit_current", crash_before_commit)
    with pytest.raises(SimulatedCrash, match="target became active"):
        first._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    journal = json.loads((tmp_path / "switch.json").read_text(encoding="utf-8"))
    assert journal["state"] == "target_starting"
    assert journal["previous"] is None
    assert crashing_runner.calls == [("apply", target_path)]
    assert not (tmp_path / "current.json").exists()

    monkeypatch.setattr(sim_switch_module, "_commit_current", commit_current)
    recovery_runner = RecordingRunner(tmp_path)
    second = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=recovery_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(second, "_resolve", lambda *_args, **_kwargs: target)

    report = second._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    retry_path = Path(report.run_plan_path or "").resolve()
    assert retry_path != target_path
    assert recovery_runner.calls == [
        ("quiesce", target_path),
        ("apply", retry_path),
    ]
    assert not (tmp_path / "switch.json").exists()


def test_committed_target_only_clears_leftover_journal_on_reconcile(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    crashing_runner = RecordingRunner(tmp_path)
    first = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=crashing_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(first, "_resolve", lambda *_args, **_kwargs: target)
    remove_journal = sim_switch_module._remove_switch_journal

    def crash_after_commit(*_args: Any, **_kwargs: Any) -> None:
        raise SimulatedCrash("power loss after current commit")

    monkeypatch.setattr(
        sim_switch_module,
        "_remove_switch_journal",
        crash_after_commit,
    )
    with pytest.raises(SimulatedCrash, match="current commit"):
        first._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    current_path = tmp_path / "current.json"
    journal_path = tmp_path / "switch.json"
    current = json.loads(current_path.read_text(encoding="utf-8"))
    journal = json.loads(journal_path.read_text(encoding="utf-8"))
    assert current["product_session_id"] == TARGET_PRODUCT_SESSION_ID
    assert journal["state"] == "current_committed"
    assert crashing_runner.calls == [("apply", target_path)]

    monkeypatch.setattr(
        sim_switch_module,
        "_remove_switch_journal",
        remove_journal,
    )
    recovery_runner = RecordingRunner(tmp_path, expect_current_absent=False)
    second = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=recovery_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(second, "_resolve", lambda *_args, **_kwargs: target)

    report = second._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    retry_path = Path(report.run_plan_path or "").resolve()
    assert retry_path != target_path
    assert recovery_runner.calls == [
        ("quiesce", target_path),
        ("apply", retry_path),
    ]
    assert not journal_path.exists()
    assert json.loads(current_path.read_text(encoding="utf-8"))[
        "product_session_id"
    ] == report.product_session_id


def test_committed_target_reconcile_does_not_require_obsolete_previous_plan(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop")
    _current_path, previous_path, _current_bytes = _write_current(tmp_path, previous)
    target = _plan("teleop_avoid")
    first_runner = RecordingRunner(tmp_path, expect_current_absent=False)
    first = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=first_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(first, "_resolve", lambda *_args, **_kwargs: target)
    remove_journal = sim_switch_module._remove_switch_journal

    def crash_after_commit(*_args: Any, **_kwargs: Any) -> None:
        raise SimulatedCrash("power loss after replacing previous current")

    monkeypatch.setattr(
        sim_switch_module,
        "_remove_switch_journal",
        crash_after_commit,
    )
    with pytest.raises(SimulatedCrash):
        first._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )
    previous_path.unlink()
    monkeypatch.setattr(
        sim_switch_module,
        "_remove_switch_journal",
        remove_journal,
    )
    recovery_runner = RecordingRunner(tmp_path, expect_current_absent=False)
    second = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=recovery_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(second, "_resolve", lambda *_args, **_kwargs: target)

    report = second._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=tmp_path,
    )

    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    retry_path = Path(report.run_plan_path or "").resolve()
    assert retry_path != target_path
    assert report.status == "active"
    assert recovery_runner.calls == [
        ("quiesce", target_path),
        ("apply", retry_path),
    ]
    assert not (tmp_path / "switch.json").exists()


def test_outside_switch_journal_plan_is_retained_without_runner_rpc(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    target_path = target.write(tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json")
    payload = _switch_journal_payload(target_path=target_path, target=target)
    outside_root = (tmp_path.parent / f"{tmp_path.name}-outside").resolve()
    outside_root.mkdir()
    outside_path = target.write(
        outside_root / f"plan-{TARGET_PRODUCT_SESSION_ID}.json"
    )
    payload["target"]["run_plan_path"] = str(outside_path)
    raw = json.dumps(payload, separators=(",", ":")) + "\n"
    journal_path = tmp_path / "switch.json"
    journal_path.write_text(raw, encoding="utf-8")
    os.chmod(journal_path, 0o600)
    original = journal_path.read_bytes()
    simulation_runner = RecordingRunner(tmp_path)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="journal"):
        control._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    assert simulation_runner.calls == []
    assert journal_path.read_bytes() == original


def test_failed_rollback_retains_journal_and_both_exact_plans(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop")
    current_path, previous_path, current_bytes = _write_current(tmp_path, previous)
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    runner = FailedRollbackRunner(
        tmp_path,
        previous_path=previous_path,
        target_path=target_path,
    )
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="target launch failed") as failure:
        control._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    assert failure.value.report.status == "rollback_failed"
    assert failure.value.report.cleanup == [
        "target:quiesced",
        "previous_failed:previous restore failed",
    ]
    journal = json.loads((tmp_path / "switch.json").read_text(encoding="utf-8"))
    assert journal["target"]["run_plan_path"] == str(target_path)
    assert journal["previous"]["run_plan_path"] == str(previous_path)
    assert current_path.read_bytes() == current_bytes
    assert target_path.is_file()
    assert previous_path.is_file()


def test_product_control_owns_sim_switch_under_its_mutation_lock(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    simulation_runner = LockCheckingRunner(tmp_path)
    systemd_runner = ForbiddenSystemdRunner()
    control = ProductControl(
        systemd_runner,  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )
    resolve_calls: list[tuple[str | None, str | None, str | None]] = []

    def resolve(
        product: str | None = None,
        *,
        product_variant: str | None = None,
        local_planner: str | None = None,
        **_kwargs: Any,
    ) -> RunPlan:
        resolve_calls.append((product, product_variant, local_planner))
        return target

    monkeypatch.setattr(control, "_resolve", resolve)

    report = control._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=tmp_path,
    )

    expected_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    assert resolve_calls == [("teleop_avoid", None, None)]
    assert simulation_runner.calls == [("apply", expected_path)]
    assert systemd_runner.calls == []
    assert report.status == "active"
    current_path = tmp_path / "current.json"
    current_raw = current_path.read_bytes()
    current = json.loads(current_raw)
    assert len(simulation_runner.applied_product_session_ids) == 1
    assert current["product_session_id"] == (
        simulation_runner.applied_product_session_ids[0]
    )
    assert report.product_session_id == current["product_session_id"]
    assert report.as_dict()["product_session_id"] == current["product_session_id"]
    assert current["run_plan_path"] == str(expected_path)
    assert current_raw == (
        json.dumps(
            current,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")
    assert not (tmp_path / "switch.json").exists()


def test_current_record_fsync_failure_rolls_back_and_cleans_temp(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    simulation_runner = RecordingRunner(tmp_path)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    def fail_current_fsync(_descriptor: int) -> None:
        raise OSError("simulated current fsync failure")

    monkeypatch.setattr(sim_switch_module.os, "fsync", fail_current_fsync)

    with pytest.raises(SwitchFailed, match="cannot be committed durably") as failure:
        control._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    assert failure.value.report.status == "failed_stopped"
    assert simulation_runner.calls == [
        ("apply", target_path),
        ("quiesce", target_path),
    ]
    assert not (tmp_path / "current.json").exists()
    assert not (tmp_path / "switch.json").exists()
    assert list(tmp_path.glob(".current.json.*.tmp")) == []
    assert not target_path.exists()


def test_failed_target_restores_previous_plan_without_replacing_current(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = _plan("teleop")
    current_path, previous_path, current_bytes = _write_current(tmp_path, previous)
    target = _plan("teleop_avoid")
    target_path = (tmp_path / f"plan-{TARGET_PRODUCT_SESSION_ID}.json").resolve()
    runner = RollbackRunner(
        current_path=current_path,
        expected_current=current_bytes,
        old_path=previous_path,
        target_path=target_path,
    )
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="target launch failed") as failure:
        control._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    report = failure.value.report
    assert report.current_product == "teleop"
    assert report.status == "failed_rolled_back"
    assert report.phases == ["preflight", "plan_published", "previous_quiesced"]
    assert report.cleanup == [
        "target:quiesced",
        "previous:restored",
        "plan:removed",
    ]
    assert runner.calls == [
        ("quiesce", previous_path),
        ("apply", target_path),
        ("quiesce", target_path),
        ("apply", previous_path),
    ]
    assert current_path.read_bytes() == current_bytes
    assert previous_path.is_file()
    assert not target_path.exists()
    assert not (tmp_path / "switch.json").exists()


def test_sim_switch_rejects_external_current_file_override_before_runner_rpc(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    external_root = (tmp_path.parent / f"{tmp_path.name}-current-override").resolve()
    external_current = external_root / "current.json"
    simulation_runner = RecordingRunner(tmp_path)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={"LINGTU_CURRENT_FILE": str(external_current)},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match="LINGTU_CURRENT_FILE"):
        control._switch(
            SwitchRequest(target_product="teleop_avoid"),
            state_dir=tmp_path,
        )

    assert simulation_runner.calls == []
    assert not external_root.exists()
    assert not (tmp_path / "current.json").exists()
    assert list(tmp_path.glob("plan-*.json")) == []


def test_product_control_sim_dry_run_has_no_files_or_runner_calls(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state_dir = tmp_path / "not-created"
    target = _plan("teleop_avoid")
    simulation_runner = RecordingRunner(state_dir)
    systemd_runner = ForbiddenSystemdRunner()
    control = ProductControl(
        systemd_runner,  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )
    resolve_calls: list[str | None] = []

    def resolve(product: str | None = None, **_kwargs: Any) -> RunPlan:
        resolve_calls.append(product)
        return target

    monkeypatch.setattr(control, "_resolve", resolve)

    report = control._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=state_dir,
        dry_run=True,
    )

    assert report.ok is True
    assert report.status == "planned"
    assert report.phases == ["preflight"]
    assert resolve_calls == ["teleop_avoid"]
    assert simulation_runner.calls == []
    assert systemd_runner.calls == []
    assert not state_dir.exists()


def test_product_control_sim_rejects_required_map_before_runtime_work(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = RecordingRunner(tmp_path)
    control = ProductControl(
        robot="doso/thunder_v4",
        env="sim",
        simulation_runner=runner,
        process_env={},
    )
    monkeypatch.setattr(
        control,
        "_resolve",
        lambda *_args, **_kwargs: _plan("tracking", requires_map=True),
    )

    with pytest.raises(SwitchFailed, match="Product tracking requires a map"):
        control._switch(
            SwitchRequest(target_product="tracking"),
            state_dir=tmp_path,
        )

    assert runner.calls == []
    assert [path.name for path in tmp_path.iterdir()] == [".product-control.lock"]


def test_product_control_sim_saved_map_dry_run_defers_runtime_map_checks(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state_dir = tmp_path / "not-created"
    target = _plan("nav")
    simulation_runner = RecordingRunner(state_dir)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    report = control._switch(
        SwitchRequest(target_product="nav", map_name="yard"),
        state_dir=state_dir,
        dry_run=True,
    )

    assert report.ok is True
    assert report.status == "planned"
    assert report.phases == ["preflight", "map_runtime_pending"]
    assert simulation_runner.calls == []
    assert not state_dir.exists()


@pytest.mark.parametrize(
    ("switch_request", "message"),
    (
        (
            SwitchRequest(target_product="teleop_avoid", map_name="yard"),
            "map_name",
        ),
        (
            SwitchRequest(
                target_product="teleop_avoid",
                initial_pose=(1.0, 2.0, 0.3),
            ),
            "initial_pose",
        ),
    ),
)
def test_sim_switch_rejects_unowned_request_fields_before_runner_rpc(
    switch_request: SwitchRequest,
    message: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    simulation_runner = RecordingRunner(tmp_path)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    with pytest.raises(SwitchFailed, match=message):
        control._switch(switch_request, state_dir=tmp_path)

    assert simulation_runner.calls == []
    assert not (tmp_path / "current.json").exists()
    assert list(tmp_path.glob("plan-*.json")) == []


def test_product_control_ensures_session_bound_simulation_supervisor(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = _plan("teleop_avoid")
    simulation_runner = RecordingRunner(tmp_path)
    ensured_for: list[tuple[Path, Path]] = []

    def ensure_supervisor(
        state_root: Path,
        repository_root: Path,
        *,
        timeout_s: float,
    ) -> RecordingRunner:
        assert timeout_s > 0
        assert state_root.is_absolute()
        assert state_root.is_dir()
        assert repository_root == REPO_ROOT
        ensured_for.append((state_root, repository_root))
        return simulation_runner

    monkeypatch.setattr("lingtu.control.ensure_sim_supervisor", ensure_supervisor)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        env="sim",
        process_env={},
    )
    monkeypatch.setattr(control, "_resolve", lambda *_args, **_kwargs: target)

    report = control._switch(
        SwitchRequest(target_product="teleop_avoid"),
        state_dir=tmp_path,
    )

    assert report.status == "active"
    assert ensured_for == [
        (tmp_path.resolve(), REPO_ROOT)
    ]

def test_product_control_stops_exact_committed_sim_plan_and_removes_current(
    tmp_path: Path,
) -> None:
    plan = _plan("teleop_avoid")
    _current_path, plan_path, _current_bytes = _write_current(tmp_path, plan)
    simulation_runner = RecordingRunner(tmp_path, expect_current_absent=False)
    systemd_runner = ForbiddenSystemdRunner()
    control = ProductControl(
        systemd_runner,  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )

    report = control._stop(
        expected_product="teleop_avoid",
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert simulation_runner.calls == [("stop", plan_path)]
    assert simulation_runner.stopped_product_session_ids == [OLD_PRODUCT_SESSION_ID]
    assert systemd_runner.calls == []
    assert not (tmp_path / "current.json").exists()
    assert not plan_path.exists()

def test_stop_current_rejects_field_backend_for_sim_before_supervisor_rpc(
    tmp_path: Path,
) -> None:
    plan = _plan("teleop_avoid")
    _write_current(tmp_path, plan)
    simulation_runner = RecordingRunner(tmp_path, expect_current_absent=False)
    systemd_runner = ForbiddenSystemdRunner()
    control = ProductControl(
        systemd_runner,  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )

    with pytest.raises(RuntimeError, match="does not accept a field SwitchBackend"):
        control._stop(
            backend=object(),  # type: ignore[arg-type]
            state_dir=tmp_path,
        )

    assert simulation_runner.calls == []
    assert systemd_runner.calls == []
    assert (tmp_path / "current.json").is_file()


def test_sim_stop_expected_product_is_checked_while_mutation_lock_is_held(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan = _plan("teleop_avoid")
    _write_current(tmp_path, plan)
    simulation_runner = RecordingRunner(tmp_path, expect_current_absent=False)
    lock_outcomes: list[str] = []
    product_name_calls = 0

    def checked_product_name(value: str) -> str:
        nonlocal product_name_calls
        product_name_calls += 1
        if product_name_calls == 2:
            def contend_for_lock() -> None:
                try:
                    with ProductControlLock(
                        tmp_path,
                        environment={},
                        timeout_s=0,
                    ):
                        lock_outcomes.append("acquired")
                except ProductControlBusy:
                    lock_outcomes.append("busy")

            contender = threading.Thread(target=contend_for_lock)
            contender.start()
            contender.join(timeout=2)
            assert not contender.is_alive()
        return value

    monkeypatch.setattr("lingtu.control.product_name", checked_product_name)
    control = ProductControl(
        ForbiddenSystemdRunner(),  # type: ignore[arg-type]
        simulation_runner=simulation_runner,
        env="sim",
        process_env={},
    )

    report = control._stop(
        expected_product="teleop_avoid",
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert product_name_calls == 2
    assert lock_outcomes == ["busy"]
