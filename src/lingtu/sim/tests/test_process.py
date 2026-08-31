"""Simulation direct-child lifecycle tests."""

from __future__ import annotations

import io
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from lingtu.run_plan import RunPlan
from lingtu.sim.identity import ProcessIdentity, SimChildLedger
from lingtu.sim.process import SimProcessManager
from lingtu.switch_contracts import ProcessError, ProcessFailed
from runtime.graph import (
    ProcessArtifact,
    ProcessCommand,
    ProcessReadiness,
    ProcessShutdown,
    ProcessSpec,
)

PRODUCT_SESSION_ID = "2" * 32
WINDOWS_X64_RUNTIME_DLLS = (
    "msvcp140.dll",
    "msvcp140_2.dll",
    "vcruntime140.dll",
    "vcruntime140_1.dll",
    "vcomp140.dll",
)


def test_same_stage_role_provider_starts_before_support_process() -> None:
    command = ProcessCommand(
        argv=("python", "worker.py"),
        cwd=".",
        env=(),
        artifact=ProcessArtifact("worker.py"),
        readiness=ProcessReadiness("process"),
    )
    support = ProcessSpec(
        name="mujoco_feeder",
        manager="direct",
        target="mujoco_feeder",
        order=20,
        timeout_s=5,
        lifecycle="mode",
        command=command,
        provides=(),
    )
    provider = ProcessSpec(
        name="nav_runtime",
        manager="direct",
        target="nav_runtime",
        order=20,
        timeout_s=5,
        lifecycle="mode",
        command=command,
        provides=("nav",),
    )

    ordered = SimProcessManager._ordered_processes(
        SimpleNamespace(processes=(support, provider))
    )

    assert [process.name for process in ordered] == ["nav_runtime", "mujoco_feeder"]


def test_bootstrap_environment_keeps_local_map_location(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("NAV_MAP_DIR", "C:\\maps" if os.name == "nt" else "/maps")
    if os.name == "nt":
        monkeypatch.setenv("USERPROFILE", "C:\\Users\\robot")
    else:
        monkeypatch.setenv("HOME", "/home/robot")

    environment = SimProcessManager._bootstrap_environment()

    assert environment["NAV_MAP_DIR"] == ("C:\\maps" if os.name == "nt" else "/maps")
    if os.name == "nt":
        assert environment["USERPROFILE"] == "C:\\Users\\robot"
    else:
        assert environment["HOME"] == "/home/robot"


def _minimal_simulation() -> dict[str, Any]:
    session = {
        "schema": "lingtu.sim.session.v1",
        "session_id": "test-session",
        "mujoco_version": "3.2.0",
        "seed": 0,
        "world": "test_world@1.0.0",
        "robots": [
            {
                "instance_id": "robot_01",
                "package": "test_robot@1.0.0",
                "spawn": {
                    "position_m": [0.0, 0.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
            }
        ],
        "runtime": {
            "backend": "mujoco",
            "mode": "headless",
            "required_bindings": ["physics"],
        },
    }
    world = {
        "package": {
            "id": "test_world",
            "version": "1.0.0",
            "kind": "world",
            "manifest": "sim/packages/worlds/test_world/world.package.yaml",
        },
        "mjcf": "sim/worlds/test_world/world.xml",
    }
    robots = [
        {
            "instance_id": "robot_01",
            "namespace": "robot_01",
            "package": {
                "id": "test_robot",
                "version": "1.0.0",
                "kind": "robot",
                "manifest": "sim/robots/test_robot/robot.package.yaml",
            },
            "controller": None,
            "sensor_rig": None,
            "spawn": {
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "model": {
                "mjcf": "sim/robots/test_robot/robot.xml",
                "attach_root": "base_link",
                "root_joint": "root",
                "initial_keyframe": None,
            },
            "frames": [{"name": "base_link", "role": "base"}],
            "semantic": {"class": "test_robot"},
        }
    ]
    return {
        "schema": "lingtu.run_plan.simulation.v1",
        "session_source": "test/minimal.json",
        "session": session,
        "physics_plan": {
            "schema": "lingtu.sim.physics-plan.v1",
            "session_id": session["session_id"],
            "composition": {
                "model_kind": "single_mjmodel",
                "composer": "mjs_attach_v1",
                "namespace_separator": "__",
                "state_authority": "mujoco",
            },
            "global_policy": {
                "owner": "world",
                "timestep_s": 0.002,
                "integrator": "rk4",
                "solver": "newton",
                "iterations": 100,
                "gravity_mps2": [0.0, 0.0, -9.81],
            },
            "world": world,
            "robots": robots,
        },
    }


def _native_environment(**extra: str) -> dict[str, str]:
    return {
        "LINGTU_NAV_CONTROL_MODE": "test",
        "LINGTU_NAV_PUBLISH_CMD_VEL": "0",
        "LINGTU_NAV_CHECK_OBSTACLE": "0",
        "LINGTU_NAV_USE_TRAVERSABILITY_COST": "0",
        "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER": "0",
        "LINGTU_TELEOP_LOCAL_PLANNER": "0",
        "NAV_GLOBAL_PLANNER": "octoplanner3d",
        **extra,
    }


def _plan(
    repository_root: Path,
    artifact: Path,
    *,
    process_name: str = "worker",
    process_target: str = "worker-target",
    provides: tuple[str, ...] = ("worker",),
    readiness: ProcessReadiness | None = None,
    command_env: tuple[tuple[str, str], ...] = (),
    shutdown: ProcessShutdown | None = None,
    native_process_environment: dict[str, str] | None = None,
    dependencies: tuple[ProcessArtifact, ...] = (),
    additional_processes: tuple[ProcessSpec, ...] = (),
    additional_available_processes: tuple[ProcessSpec, ...] = (),
) -> RunPlan:
    relative_artifact = artifact.relative_to(repository_root).as_posix()
    process = ProcessSpec(
        name=process_name,
        manager="direct",
        target=process_target,
        order=10,
        timeout_s=5,
        lifecycle="mode",
        command=ProcessCommand(
            argv=("python", relative_artifact),
            cwd=".",
            env=command_env,
            artifact=ProcessArtifact(relative_artifact),
            readiness=readiness or ProcessReadiness("process"),
            shutdown=shutdown,
            dependencies=dependencies,
        ),
        provides=provides,
    )
    return RunPlan.create(
        product="test_product",
        env="sim",
        robot="doso/thunder_v4",
        process_control="subprocess",
        modules=(),
        processes=(process, *additional_processes),
        available_processes=(
            process,
            *additional_processes,
            *additional_available_processes,
        ),
        stop_before_start=(),
        contracts=("lingtu.product.nav.v1",),
        critical_modules=(),
        route_contract=None,
        host_config={},
        lifecycle={},
        simulation=_minimal_simulation(),
        native_process_environment=native_process_environment,
        native_nav={
            "control_mode": "test",
            "publish_cmd_vel": False,
            "check_obstacle": False,
            "use_traversability_cost": False,
            "allow_teleop_takeover": False,
            "teleop_local_planner": False,
            "global_planner": "octoplanner3d",
        },
    )


def _role_process(
    repository_root: Path,
    artifact: Path,
    role: str,
) -> ProcessSpec:
    relative_artifact = artifact.relative_to(repository_root).as_posix()
    return ProcessSpec(
        name=f"{role}_publisher",
        manager="direct",
        target=f"{role}-publisher",
        order=10,
        timeout_s=5,
        lifecycle="mode",
        command=ProcessCommand(
            argv=("python", relative_artifact),
            cwd=".",
            env=(),
            artifact=ProcessArtifact(relative_artifact),
            readiness=ProcessReadiness("process"),
        ),
        provides=(role,),
    )


def test_readiness_retirement_retries_a_windows_sharing_violation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text("raise SystemExit(0)\n", encoding="utf-8")
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("file", "worker.ready.json"),
    )
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=_publish_plan(session_root, plan),
        product_session_id=PRODUCT_SESSION_ID,
    )
    ready_path = session_root / "worker.ready.json"
    ready_path.write_text("{}", encoding="utf-8")
    original_unlink = Path.unlink
    attempts = 0
    sleeps: list[float] = []

    def flaky_unlink(path: Path, *args: object, **kwargs: object) -> None:
        nonlocal attempts
        if path == ready_path and attempts < 2:
            attempts += 1
            raise PermissionError("sharing violation")
        original_unlink(path, *args, **kwargs)

    monkeypatch.setattr(Path, "unlink", flaky_unlink)
    monkeypatch.setattr("lingtu.sim.process.time.sleep", sleeps.append)

    manager._retire_readiness(plan.processes[0])

    assert attempts == 2
    assert sleeps == [0.05, 0.05]
    assert not ready_path.exists()


def test_run_plan_dependency_round_trip_is_strict(tmp_path: Path) -> None:
    repository_root = tmp_path / "repository"
    repository_root.mkdir()
    artifact = repository_root / "worker.py"
    dependency = repository_root / "nav_client.dll"
    artifact.write_text("raise SystemExit(0)\n", encoding="utf-8")
    dependency.write_bytes(b"native dependency")
    plan = _plan(
        repository_root,
        artifact,
        dependencies=(ProcessArtifact("nav_client.dll"),),
    )

    assert RunPlan.from_dict(plan.as_dict()) == plan
    payload = plan.as_dict()
    payload["launch"]["process_catalog"]["selected"][0]["command"][
        "dependencies"
    ][0]["extra"] = True
    with pytest.raises(ValueError, match="invalid fields"):
        RunPlan.from_dict(payload)


def test_feeder_readiness_uses_exact_bound_sensor_roles(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import os\n"
        "import time\n"
        "from pathlib import Path\n"
        "root = Path(os.environ['LINGTU_RUN_PLAN']).parent\n"
        "(root / 'mujoco_feeder.ready.json').write_text('{}', encoding='utf-8')\n"
        "while True:\n"
        "    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    sensor_processes = tuple(
        _role_process(repository_root, artifact, role)
        for role in ("lidar", "imu", "camera")
    )
    plan = _plan(
        repository_root,
        artifact,
        process_name="mujoco_feeder",
        process_target="mujoco-feeder",
        provides=("simulation_feeder",),
        readiness=ProcessReadiness("file", "mujoco_feeder.ready.json"),
        additional_processes=sensor_processes,
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=plan_path,
        product_session_id=PRODUCT_SESSION_ID,
    )
    manager.start("mujoco-feeder", timeout_s=1)
    (session_root / "mujoco_feeder.ready.json").write_text("{}", encoding="utf-8")
    captured: dict[str, object] = {}

    def capture_readiness(*_args: object, **kwargs: object) -> dict[str, object]:
        captured.update(kwargs)
        return {"ready": True}

    monkeypatch.setattr(
        "lingtu.sim.process.load_typed_readiness",
        capture_readiness,
    )
    try:
        manager.wait(plan.process("simulation_feeder"), timeout_s=1)
    finally:
        if manager.active("mujoco-feeder"):
            try:
                manager.stop_process(plan.process("simulation_feeder"), timeout_s=1)
            except ProcessError as exc:
                assert "required force stop" in str(exc)

    assert captured["lidar_required"] is True
    assert captured["imu_required"] is True
    assert captured["camera_required"] is True


@pytest.mark.parametrize(
    ("roles", "endpoint_roles"),
    (
        (("lidar", "imu"), ("driver_bridge", "lidar_publisher")),
        (
            ("lidar", "imu", "camera"),
            ("driver_bridge", "lidar_publisher", "imu_publisher"),
        ),
    ),
)
def test_feeder_readiness_fails_closed_when_a_required_sensor_is_missing(
    tmp_path: Path,
    roles: tuple[str, ...],
    endpoint_roles: tuple[str, ...],
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text("import time\nwhile True: time.sleep(0.05)\n", encoding="utf-8")
    sensor_processes = tuple(
        _role_process(repository_root, artifact, role) for role in roles
    )
    plan = _plan(
        repository_root,
        artifact,
        process_name="mujoco_feeder",
        process_target="mujoco-feeder",
        provides=("simulation_feeder",),
        readiness=ProcessReadiness("file", "mujoco_feeder.ready.json"),
        additional_processes=sensor_processes,
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=plan_path,
        product_session_id=PRODUCT_SESSION_ID,
    )
    manager.start("mujoco-feeder", timeout_s=1)
    protocol = {
        "driver_bridge": "driver-v2",
        "lidar_publisher": "ltu1-v1",
        "imu_publisher": "ltu1-v1",
        "camera_publisher": "ltu1-v1",
    }
    payload = {
        "backend": "mujoco",
        "endpoints": [
            {"protocol": protocol[role], "role": role} for role in endpoint_roles
        ],
        "env": "sim",
        "product_session_id": PRODUCT_SESSION_ID,
        "first_physics_step_applied": True,
        "model_generation": 0,
        "process": "mujoco_feeder",
        "product": plan.product,
        "protocol": "mujoco-feeder-v1",
        "ready": True,
        "reset_generation": 0,
        "role": "mujoco_feeder",
        "schema": "lingtu.sim.feeder_ready.v1",
        "session_id": plan.simulation["session"]["session_id"],
    }
    (session_root / "mujoco_feeder.ready.json").write_text(
        json.dumps(payload, separators=(",", ":"), sort_keys=True),
        encoding="ascii",
    )
    try:
        with pytest.raises(ProcessError, match="readiness evidence is invalid"):
            manager.wait(plan.process("simulation_feeder"), timeout_s=1)
    finally:
        if manager.active("mujoco-feeder"):
            try:
                manager.stop_process(plan.process("simulation_feeder"), timeout_s=1)
            except ProcessError as exc:
                assert "required force stop" in str(exc)


def test_slam_readiness_uses_exact_bound_run_plan_mode(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import os\n"
        "import time\n"
        "from pathlib import Path\n"
        "root = Path(os.environ['LINGTU_RUN_PLAN']).parent\n"
        "(root / 'slam.status.json').write_text('{}', encoding='utf-8')\n"
        "while True:\n"
        "    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(
        repository_root,
        artifact,
        process_name="slam_runtime",
        process_target="slam-runtime",
        provides=("slam",),
        readiness=ProcessReadiness("file", "slam.status.json"),
        native_process_environment=_native_environment(
            LINGTU_SLAM_MODE="localization"
        ),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=plan_path,
        product_session_id=PRODUCT_SESSION_ID,
    )
    manager.start("slam-runtime", timeout_s=5)
    captured: dict[str, object] = {}

    def capture_readiness(*_args: object, **kwargs: object) -> dict[str, object]:
        captured.update(kwargs)
        return {"ready": True}

    monkeypatch.setattr(
        "lingtu.sim.process.load_typed_readiness",
        capture_readiness,
    )
    try:
        manager.wait(plan.process("slam"), timeout_s=5)
    finally:
        if manager.active("slam-runtime"):
            manager.stop_process(plan.process("slam"), timeout_s=5)

    assert captured["expected_slam_mode"] == "localization"
    assert captured["product_session_id"] == PRODUCT_SESSION_ID
    assert captured["expected_control_mode"] is None
    assert captured["expected_explore_route"] is None


def test_slam_readiness_rejects_invalid_bound_run_plan_mode(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import os\n"
        "import time\n"
        "from pathlib import Path\n"
        "root = Path(os.environ['LINGTU_RUN_PLAN']).parent\n"
        "(root / 'slam.status.json').write_text('{}', encoding='utf-8')\n"
        "while True:\n"
        "    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(
        repository_root,
        artifact,
        process_name="slam_runtime",
        process_target="slam-runtime",
        provides=("slam",),
        readiness=ProcessReadiness("file", "slam.status.json"),
        native_process_environment=_native_environment(LINGTU_SLAM_MODE="fixture"),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=plan_path,
        product_session_id=PRODUCT_SESSION_ID,
    )
    manager.start("slam-runtime", timeout_s=1)
    try:
        with pytest.raises(ProcessError, match="readiness evidence is invalid"):
            manager.wait(plan.process("slam"), timeout_s=2)
    finally:
        if manager.active("slam-runtime"):
            manager.stop_process(plan.process("slam"), timeout_s=1)


def test_direct_process_output_is_bounded_and_exposed_as_session_evidence(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import os\n"
        "import sys\n"
        "import time\n"
        "from pathlib import Path\n"
        "sys.stdout.buffer.write(b'x' * (512 * 1024) + b'STDOUT_TAIL\\n')\n"
        "sys.stdout.buffer.flush()\n"
        "sys.stderr.buffer.write(b'y' * (512 * 1024) + b'STDERR_TAIL\\n')\n"
        "sys.stderr.buffer.flush()\n"
        "root = Path(os.environ['LINGTU_RUN_PLAN']).parent\n"
        "(root / 'ready.json').write_text('{}', encoding='utf-8')\n"
        "while True:\n"
        "    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("process"),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    manager.start("worker-target", timeout_s=1)
    try:
        evidence = manager.wait(plan.process("worker"), timeout_s=3)
        stdout_log = session_root / "logs" / "worker.stdout.log"
        stderr_log = session_root / "logs" / "worker.stderr.log"
        deadline = time.monotonic() + 3.0
        while (
            not stdout_log.exists()
            or not stderr_log.exists()
            or not stdout_log.read_bytes().endswith(b"STDOUT_TAIL\n")
            or not stderr_log.read_bytes().endswith(b"STDERR_TAIL\n")
        ) and time.monotonic() < deadline:
            time.sleep(0.02)
    finally:
        manager.stop_process(plan.processes[0], timeout_s=1)

    assert evidence == {
        "kind": "process",
        "target": "worker-target",
        "active": True,
    }
    assert stdout_log.parent == session_root / "logs"
    assert stderr_log.parent == session_root / "logs"
    assert stdout_log.stat().st_size <= 128 * 1024
    assert stderr_log.stat().st_size <= 128 * 1024
    assert stdout_log.read_bytes().endswith(b"STDOUT_TAIL\n")
    assert stderr_log.read_bytes().endswith(b"STDERR_TAIL\n")
    assert not any(
        thread.name.startswith("lingtu-log-worker-")
        for thread in threading.enumerate()
    )


def test_process_exit_before_readiness_reports_exit_code_and_log_paths(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import sys\n"
        "print('startup failed for test', file=sys.stderr, flush=True)\n"
        "raise SystemExit(7)\n",
        encoding="utf-8",
    )
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("file", "never.ready"),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)

    with pytest.raises(ProcessFailed, match="exit_code=7") as captured:
        manager.apply(plan)

    message = str(captured.value)
    assert captured.value.report.started == ["worker-target"]
    assert captured.value.report.rolled_back == []
    stdout_log = session_root / "logs" / "worker.stdout.log"
    stderr_log = session_root / "logs" / "worker.stderr.log"
    assert f"stdout_log={stdout_log}" in message
    assert f"stderr_log={stderr_log}" in message
    assert stderr_log.read_text(encoding="utf-8") == "startup failed for test\n"
    assert manager.owns("worker-target") is False

def test_preexisting_readiness_is_retired_before_unowned_start(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\nwhile True:\n    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(
        repository_root,
        artifact,
        process_name="mujoco_feeder",
        process_target="mujoco-feeder",
        provides=("simulation_feeder",),
        readiness=ProcessReadiness("file", "mujoco_feeder.ready.json"),
    )
    plan_path = _publish_plan(session_root, plan)
    (session_root / "mujoco_feeder.ready.json").write_text(
        "foreign",
        encoding="utf-8",
    )
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=plan_path,
        product_session_id=PRODUCT_SESSION_ID,
    )

    manager.start("mujoco-feeder", timeout_s=1)
    try:
        assert (session_root / "mujoco_feeder.ready.json").exists() is False
        assert manager.owns("mujoco-feeder") is True
    finally:
        if manager.active("mujoco-feeder"):
            try:
                manager.stop_process(plan.process("simulation_feeder"), timeout_s=1)
            except ProcessError as exc:
                assert "required force stop" in str(exc)


def _publish_plan(session_root: Path, plan: RunPlan) -> Path:
    return plan.write(session_root / f"plan-{PRODUCT_SESSION_ID}.json")


def _write_child_ledger(
    session_root: Path,
    plan_path: Path,
    plan: RunPlan,
    *,
    identity: ProcessIdentity,
    target: str = "worker-target",
    process_group: int | None = None,
    launch_id: str = "d" * 64,
    product_session_id: str = PRODUCT_SESSION_ID,
) -> Path:
    supervisor = session_root / "supervisor"
    supervisor.mkdir(mode=0o700, exist_ok=True)
    if os.name != "nt":
        supervisor.chmod(0o700)
    ledger = supervisor / "children.json"
    payload = {
        "schema_version": "lingtu.sim_children.v3",
        "run_plan_path": str(plan_path.resolve()),
        "product": plan.product,
        "product_session_id": product_session_id,
        "children": [
            {
                "target": target,
                "process_identity": identity.as_dict(),
                "process_group": identity.pid if process_group is None else process_group,
                "started_wall_ns": time.time_ns(),
                "launch_id": launch_id,
            }
        ],
    }
    ledger.write_bytes(
        (
            json.dumps(
                payload,
                allow_nan=False,
                ensure_ascii=True,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
    )
    if os.name != "nt":
        ledger.chmod(0o600)
    return ledger


def _terminate_test_process(child: subprocess.Popen[bytes]) -> None:
    if child.poll() is not None:
        return
    if os.name == "nt":
        child.terminate()
    else:
        os.killpg(child.pid, signal.SIGKILL)
    child.wait(timeout=5)


def _write_natural_motion_worker(artifact: Path) -> None:
    src_root = Path(__file__).resolve().parents[3]
    artifact.write_text(
        "import os\n"
        "import sys\n"
        "import time\n"
        "from pathlib import Path\n"
        f"sys.path.insert(0, {str(src_root)!r})\n"
        "from lingtu.sim.stop import publish_motion_stop_evidence\n"
        "root = Path(os.environ['LINGTU_RUN_PLAN']).parent\n"
        "payload = {\n"
        "    'schema': 'lingtu.sim.motion_stop.v2',\n"
        "    'product_session_id': os.environ['LINGTU_PRODUCT_SESSION_ID'],\n"
        "    'product': os.environ['LINGTU_PRODUCT'],\n"
        "    'process': 'worker',\n"
        "    'outcome': 'zero_applied',\n"
        "    'bridge_boot_id': 'b' * 32,\n"
        "    'controller_boot_id': 'c' * 32,\n"
        "    'bridge_command_seq': 1,\n"
        "    'applied_step_seq': 2,\n"
        "    'command_kind': 'deactivate_zero',\n"
        "    'walk_x': 0.0,\n"
        "    'walk_y': 0.0,\n"
        "    'walk_z': 0.0,\n"
        "    'terminal_ack': True,\n"
        "}\n"
        "publish_motion_stop_evidence(\n"
        "    session_root=root, target='driver_bridge.stop.json', payload=payload\n"
        ")\n"
        "(root / 'ready.json').write_text('ready', encoding='utf-8')\n"
        "time.sleep(0.1)\n",
        encoding="utf-8",
    )


def _wait_for_inactive(manager: SimProcessManager, target: str) -> None:
    deadline = time.monotonic() + 10.0
    while manager.active(target) and time.monotonic() < deadline:
        time.sleep(0.02)
    assert manager.active(target) is False


def test_new_manager_adopts_exact_live_child_and_stops_it(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\nwhile True:\n    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(repository_root, artifact)
    plan_path = _publish_plan(session_root, plan)
    child = subprocess.Popen(  # noqa: S603
        [sys.executable, str(artifact)],
        cwd=repository_root,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=os.name != "nt",
        creationflags=(subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0),
    )
    try:
        identity = ProcessIdentity.current(child.pid)
        ledger = _write_child_ledger(session_root, plan_path, plan, identity=identity)

        manager = SimProcessManager(repository_root)
        if os.name == "nt":
            with pytest.raises(ProcessError, match="cannot adopt live Windows"):
                manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
            assert child.poll() is None
            assert ledger.exists()
            return

        manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)

        assert manager.active("worker-target") is True
        manager.stop_process(plan.processes[0], timeout_s=2)
        child.wait(timeout=5)
        assert manager.active("worker-target") is False
    finally:
        _terminate_test_process(child)


def test_reused_pid_identity_is_cleared_without_signalling_foreign_process(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\nwhile True:\n    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(repository_root, artifact)
    plan_path = _publish_plan(session_root, plan)
    child = subprocess.Popen(  # noqa: S603
        [sys.executable, str(artifact)],
        cwd=repository_root,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=os.name != "nt",
        creationflags=(subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0),
    )
    try:
        actual = ProcessIdentity.current(child.pid)
        if actual.platform == "windows":
            foreign_start = str(int(actual.start_identity) + 1)
        else:
            boot_id, ticks = actual.start_identity.split(":", 1)
            foreign_start = f"{boot_id}:{int(ticks) + 1}"
        foreign = ProcessIdentity(actual.pid, actual.platform, foreign_start)
        ledger = _write_child_ledger(
            session_root,
            plan_path,
            plan,
            identity=foreign,
        )

        manager = SimProcessManager(repository_root)
        manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)

        assert manager.active("worker-target") is False
        assert child.poll() is None
        assert json.loads(ledger.read_text(encoding="utf-8"))["children"] == []
    finally:
        _terminate_test_process(child)

def test_bind_rejects_duplicate_live_child_target_without_signal(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\nwhile True:\n    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(repository_root, artifact)
    plan_path = _publish_plan(session_root, plan)
    child = subprocess.Popen(  # noqa: S603
        [sys.executable, str(artifact)],
        cwd=repository_root,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=os.name != "nt",
        creationflags=(subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0),
    )
    try:
        identity = ProcessIdentity.current(child.pid)
        ledger = _write_child_ledger(
            session_root,
            plan_path,
            plan,
            identity=identity,
        )
        payload = json.loads(ledger.read_bytes())
        payload["children"].append(dict(payload["children"][0]))
        ledger.write_text(json.dumps(payload), encoding="utf-8")

        with pytest.raises(ProcessError, match="ledger is invalid"):
            SimProcessManager(repository_root).bind(
                plan,
                run_plan_path=plan_path,
                product_session_id=PRODUCT_SESSION_ID,
            )

        assert child.poll() is None
    finally:
        _terminate_test_process(child)


def test_ledger_publish_failure_reclaims_just_spawned_child(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\nwhile True:\n    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(repository_root, artifact)
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    real_popen = subprocess.Popen
    spawned: list[subprocess.Popen[bytes]] = []

    def tracking_popen(*args: Any, **kwargs: Any) -> subprocess.Popen[bytes]:
        child = real_popen(*args, **kwargs)
        spawned.append(child)
        return child

    def fail_replace(_source: object, _target: object) -> None:
        raise OSError("simulated atomic publish failure")

    monkeypatch.setattr("lingtu.sim.process.subprocess.Popen", tracking_popen)
    monkeypatch.setattr("lingtu.sim.identity.os.replace", fail_replace)
    launch_id = "launch-ledger-failure"
    monkeypatch.setattr(
        "lingtu.sim.process.secrets.token_hex",
        lambda _size: launch_id,
    )

    with pytest.raises(ProcessError, match="ledger cannot be updated") as failure:
        manager.start("worker-target", timeout_s=1)

    assert len(spawned) == 1
    spawned[0].wait(timeout=5)
    assert manager.active("worker-target") is False
    assert launch_id not in str(failure.value)
    assert launch_id not in repr(failure.value)


def test_naturally_exited_child_is_removed_from_ledger(tmp_path: Path) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text("import time\ntime.sleep(0.1)\n", encoding="utf-8")
    plan = _plan(repository_root, artifact)
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)

    manager.start("worker-target", timeout_s=1)
    deadline = time.monotonic() + 5
    while manager.active("worker-target") and time.monotonic() < deadline:
        time.sleep(0.02)

    assert manager.active("worker-target") is False
    ledger = session_root / "supervisor" / "children.json"
    assert not ledger.exists()


def test_start_wraps_popen_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text("raise SystemExit(0)\n", encoding="utf-8")
    plan = _plan(repository_root, artifact)
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=_publish_plan(session_root, plan),
        product_session_id=PRODUCT_SESSION_ID,
    )

    def fail_start(*_args: Any, **_kwargs: Any) -> subprocess.Popen[bytes]:
        raise OSError("unsupported executable")

    monkeypatch.setattr(subprocess, "Popen", fail_start)
    with pytest.raises(ProcessError, match="failed to start direct process"):
        manager.start("worker-target", timeout_s=1)


def test_apply_reports_only_successfully_started_processes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text("pass\n", encoding="utf-8")
    plan = _plan(repository_root, artifact)
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=_publish_plan(session_root, plan),
        product_session_id=PRODUCT_SESSION_ID,
    )

    def fail_start(_target: str, _timeout_s: float) -> None:
        raise ProcessError("spawn failed")

    monkeypatch.setattr(manager, "start", fail_start)
    with pytest.raises(ProcessFailed, match="spawn failed") as captured:
        manager.apply(plan)

    assert captured.value.report.started == []
    assert captured.value.report.rolled_back == []


@pytest.mark.skipif(os.name == "nt", reason="POSIX process-group cleanup")
def test_start_failure_kills_process_group_and_closes_pipes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text("pass\n", encoding="utf-8")
    plan = _plan(repository_root, artifact)
    manager = SimProcessManager(repository_root)
    manager.bind(
        plan,
        run_plan_path=_publish_plan(session_root, plan),
        product_session_id=PRODUCT_SESSION_ID,
    )

    class FakeChild:
        pid = 12345

        def __init__(self) -> None:
            self.stdout = io.BytesIO()
            self.stderr = io.BytesIO()
            self.killed = False

        def poll(self) -> None:
            return None

        def kill(self) -> None:
            self.killed = True

        def wait(self, *, timeout: float) -> int:
            del timeout
            return 0

    child = FakeChild()
    killed_groups: list[tuple[int, int]] = []

    def fail_log_capture(*_args: Any, **_kwargs: Any) -> None:
        raise ProcessError("log init failed")

    monkeypatch.setattr(subprocess, "Popen", lambda *_args, **_kwargs: child)
    monkeypatch.setattr(manager, "_start_log_capture", fail_log_capture)
    monkeypatch.setattr(
        os,
        "killpg",
        lambda pid, sig: killed_groups.append((pid, sig)),
    )

    with pytest.raises(ProcessError, match="log init failed"):
        manager.start("worker-target", timeout_s=1)

    assert killed_groups == [(child.pid, signal.SIGKILL)]
    assert child.killed is False
    assert child.stdout.closed
    assert child.stderr.closed


def test_subprocess_runner_starts_waits_and_stops_owned_child(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import json\n"
        "import os\n"
        "import sys\n"
        "import time\n"
        "from pathlib import Path\n"
        "ready = Path(os.environ['LINGTU_RUN_PLAN']).parent / 'ready.json'\n"
        "values = {key: value for key, value in os.environ.items() "
        "if (key.startswith('LINGTU_') or key == 'NAV_GLOBAL_PLANNER') and "
        "key != 'LINGTU_PROCESS_LAUNCH_ID'}\n"
        "values['launch_id'] = os.environ['LINGTU_PROCESS_LAUNCH_ID']\n"
        "values['python_executable'] = sys.executable\n"
        "values['path'] = os.environ.get('PATH')\n"
        "values['blocked_ambient'] = {key: os.environ.get(key) for key in "
        "('NON_LINGTU_BOOTSTRAP', 'LD_PRELOAD', 'PYTHONPATH', 'SSL_CERT_FILE')}\n"
        "ready.write_text(json.dumps(values), "
        "encoding='utf-8')\n"
        "while True:\n"
        "    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    native_environment = _native_environment(LINGTU_NATIVE_SETTING="native-value")
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("process"),
        command_env=(("LINGTU_COMMAND_SETTING", "command-value"),),
        native_process_environment=native_environment,
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    runner = manager

    monkeypatch.setenv("LINGTU_AMBIENT_POLLUTION", "must-not-leak")
    monkeypatch.setenv("LINGTU_HOST_BOOT_ID", "stale-parent-boot")
    monkeypatch.setenv("NON_LINGTU_BOOTSTRAP", "bootstrap-value")
    monkeypatch.setenv("LD_PRELOAD", "must-not-leak")
    monkeypatch.setenv("PYTHONPATH", "must-not-leak")
    monkeypatch.setenv("SSL_CERT_FILE", "must-not-leak")
    monkeypatch.setenv("PATH", "must-not-leak")
    launch_id = "launch-runtime-identity"
    monkeypatch.setattr(
        "lingtu.sim.process.secrets.token_hex",
        lambda _size: launch_id,
    )
    manager.bind(
        plan,
        run_plan_path=plan_path,
        product_session_id=PRODUCT_SESSION_ID,
    )
    stopped = None
    try:
        report = runner.apply(plan)
        assert report.status == "active"
        assert report.ready["worker"] == {
            "kind": "process",
            "target": "worker-target",
            "active": True,
        }
        assert manager.active("worker-target") is True
        ready_path = session_root / "ready.json"
        deadline = time.monotonic() + 3.0
        while not ready_path.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        identity = json.loads(ready_path.read_text(encoding="utf-8"))
        assert identity == {
            **native_environment,
            "LINGTU_COMMAND_SETTING": "command-value",
            "LINGTU_ENV": "sim",
            "LINGTU_RUN_PLAN": str(plan_path.resolve()),
            "LINGTU_PRODUCT": plan.product,
            "LINGTU_PRODUCT_SESSION_ID": PRODUCT_SESSION_ID,
            "LINGTU_HOST_BOOT_ID": PRODUCT_SESSION_ID,
            "LINGTU_SESSION_ROOT": str(session_root.resolve()),
            "launch_id": launch_id,
            "path": None if os.name == "nt" else os.defpath,
            "python_executable": sys.executable,
            "blocked_ambient": {
                "LD_PRELOAD": None,
                "NON_LINGTU_BOOTSTRAP": None,
                "PYTHONPATH": None,
                "SSL_CERT_FILE": None,
            },
        }
        raw_ledger = (session_root / "supervisor" / "children.json").read_text(
            encoding="utf-8"
        )
        assert "LINGTU_PROCESS_LAUNCH_ID" not in raw_ledger
        assert identity["launch_id"] in raw_ledger
    finally:
        if manager.active("worker-target"):
            stopped = runner.stop_plan(plan)

    assert stopped is not None
    assert stopped.status == "stopped"
    assert manager.active("worker-target") is False
    assert os.path.exists(session_root)


def test_foreign_start_identity_is_never_signalled(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\nwhile True:\n    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(repository_root, artifact)
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    manager.start("worker-target", timeout_s=1)
    assert manager.active("worker-target") is True

    with monkeypatch.context() as patch:
        patch.setattr(
            ProcessIdentity,
            "matches",
            lambda _identity: False,
        )
        assert manager.active("worker-target") is False
        with pytest.raises(ProcessError, match="cannot replace"):
            manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
        with pytest.raises(ProcessError, match="identity is foreign"):
            manager.stop_process(plan.processes[0], timeout_s=0.2)

    assert manager.active("worker-target") is True
    if os.name == "nt":
        try:
            manager.stop_process(plan.processes[0], timeout_s=1)
        except ProcessError as exc:
            assert "required force stop" in str(exc)
    else:
        manager.stop_process(plan.processes[0], timeout_s=1)
    assert manager.active("worker-target") is False


def test_inactive_motion_process_without_stop_evidence_releases_ownership(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\ntime.sleep(0.1)\n",
        encoding="utf-8",
    )
    plan = _plan(
        repository_root,
        artifact,
        shutdown=ProcessShutdown(
            "file",
            target="driver_bridge.stop.json",
            schema="lingtu.sim.motion_stop.v2",
        ),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    runner = manager

    runner.apply(plan)
    time.sleep(0.2)
    _wait_for_inactive(manager, "worker-target")
    report = runner.stop_plan(plan)

    assert manager.active("worker-target") is False
    assert manager.owns("worker-target") is False
    assert report.ok is True
    assert report.status == "stopped"
    assert report.stop_evidence["worker"] == {
        "schema": "lingtu.sim.process_stop.v1",
        "process": "worker",
        "target": "worker-target",
        "outcome": "inactive_without_motion_evidence",
        "process_identity": report.stop_evidence["worker"]["process_identity"],
        "graceful": False,
        "forced": False,
        "inactive": True,
        "motion_stop_confirmed": False,
    }
    assert SimChildLedger(session_root).load() is None


@pytest.mark.skipif(
    os.name == "nt",
    reason="Windows CTRL_BREAK delivery is not a reliable child signal-handler seam",
)
def test_motion_stop_file_shutdown_accepts_child_written_zero_ack(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    src_root = Path(__file__).resolve().parents[3]
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import os\n"
        "import signal\n"
        "import sys\n"
        "import time\n"
        "from pathlib import Path\n"
        f"sys.path.insert(0, {str(src_root)!r})\n"
        "from lingtu.sim.stop import publish_motion_stop_evidence\n"
        "root = Path(os.environ['LINGTU_RUN_PLAN']).parent\n"
        "def shutdown(_signum, _frame):\n"
        "    payload = {\n"
        "        'schema': 'lingtu.sim.motion_stop.v2',\n"
        "        'product_session_id': os.environ['LINGTU_PRODUCT_SESSION_ID'],\n"
        "        'product': os.environ['LINGTU_PRODUCT'],\n"
        "        'process': 'worker',\n"
        "        'outcome': 'zero_applied',\n"
        "        'bridge_boot_id': 'b' * 32,\n"
        "        'controller_boot_id': 'c' * 32,\n"
        "        'bridge_command_seq': 1,\n"
        "        'applied_step_seq': 2,\n"
        "        'command_kind': 'deactivate_zero',\n"
        "        'walk_x': 0.0,\n"
        "        'walk_y': 0.0,\n"
        "        'walk_z': 0.0,\n"
        "        'terminal_ack': True,\n"
        "    }\n"
        "    publish_motion_stop_evidence(\n"
        "        session_root=root, target='driver_bridge.stop.json', payload=payload\n"
        "    )\n"
        "    raise SystemExit(0)\n"
        "signal.signal(signal.SIGTERM, shutdown)\n"
        "if hasattr(signal, 'SIGBREAK'):\n"
        "    signal.signal(signal.SIGBREAK, shutdown)\n"
        "(root / 'ready.json').write_text('ready', encoding='utf-8')\n"
        "while True:\n"
        "    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("process"),
        shutdown=ProcessShutdown(
            "file",
            target="driver_bridge.stop.json",
            schema="lingtu.sim.motion_stop.v2",
        ),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    runner = manager

    runner.apply(plan)
    ready_path = session_root / "ready.json"
    deadline = time.monotonic() + 3.0
    while not ready_path.exists() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert ready_path.is_file()
    stopped = runner.stop_plan(plan)

    assert stopped.ok is True
    assert stopped.stop_evidence["worker"]["schema"] == "lingtu.sim.motion_stop.v2"
    assert stopped.stop_evidence["worker"]["outcome"] == "zero_applied"
    assert manager.active("worker-target") is False


def test_naturally_exited_motion_child_still_requires_and_returns_its_zero_ack(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    _write_natural_motion_worker(artifact)
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("process"),
        shutdown=ProcessShutdown(
            "file",
            target="driver_bridge.stop.json",
            schema="lingtu.sim.motion_stop.v2",
        ),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    runner = manager

    runner.apply(plan)
    time.sleep(0.3)
    _wait_for_inactive(manager, "worker-target")

    stopped = runner.stop_plan(plan)

    assert stopped.stop_evidence["worker"]["outcome"] == "zero_applied"
    assert manager.active("worker-target") is False


def test_naturally_exited_motion_child_validates_ack_when_popen_poll_is_stale(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    _write_natural_motion_worker(artifact)
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("process"),
        shutdown=ProcessShutdown(
            "file",
            target="driver_bridge.stop.json",
            schema="lingtu.sim.motion_stop.v2",
        ),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    runner = manager
    runner.apply(plan)
    time.sleep(0.3)
    _wait_for_inactive(manager, "worker-target")
    owned = manager._children["worker-target"]
    assert owned.child is not None
    deadline = time.monotonic() + 3.0
    while owned.child.poll() is None and time.monotonic() < deadline:
        time.sleep(0.02)
    assert owned.child.poll() is not None
    monkeypatch.setattr(owned.child, "poll", lambda: None)

    stopped = runner.stop_plan(plan)

    assert stopped.stop_evidence["worker"]["outcome"] == "zero_applied"
    assert manager.active("worker-target") is False


def test_dead_motion_tombstone_is_adopted_and_released_after_exact_evidence(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    _write_natural_motion_worker(artifact)
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("process"),
        shutdown=ProcessShutdown(
            "file",
            target="driver_bridge.stop.json",
            schema="lingtu.sim.motion_stop.v2",
        ),
    )
    plan_path = _publish_plan(session_root, plan)
    first = SimProcessManager(repository_root)
    first.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    first.apply(plan)
    time.sleep(0.3)
    _wait_for_inactive(first, "worker-target")

    snapshot = SimChildLedger(session_root).load()
    assert snapshot is not None
    assert [child.target for child in snapshot.children] == ["worker-target"]

    successor = SimProcessManager(repository_root)
    successor.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    stopped = successor.stop_plan(plan)

    assert stopped.stop_evidence["worker"]["outcome"] == "zero_applied"
    released = SimChildLedger(session_root).load()
    assert released is None

def test_invalid_motion_evidence_releases_dead_owner_with_unconfirmed_stop(
    tmp_path: Path,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    _write_natural_motion_worker(artifact)
    plan = _plan(
        repository_root,
        artifact,
        readiness=ProcessReadiness("process"),
        shutdown=ProcessShutdown(
            "file",
            target="driver_bridge.stop.json",
            schema="lingtu.sim.motion_stop.v2",
        ),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    runner = manager
    runner.apply(plan)
    time.sleep(0.3)
    _wait_for_inactive(manager, "worker-target")

    evidence_path = session_root / "driver_bridge.stop.json"
    canonical = evidence_path.read_bytes()
    payload = json.loads(canonical)
    payload["launch_id"] = "e" * 64
    evidence_path.write_text(
        json.dumps(
            payload,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    stopped = runner.stop_plan(plan)
    assert stopped.stop_evidence["worker"]["outcome"] == (
        "inactive_without_motion_evidence"
    )
    assert stopped.stop_evidence["worker"]["motion_stop_confirmed"] is False
    released = SimChildLedger(session_root).load()
    assert released is None


def test_forced_cleanup_reclaims_child_but_fails_the_stop(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\nwhile True:\n    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(repository_root, artifact)
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    manager.start("worker-target", timeout_s=1)
    monkeypatch.setattr(
        SimProcessManager,
        "_graceful_stop_group",
        classmethod(lambda _cls, _owned: None),
    )

    with pytest.raises(ProcessError, match="required force stop"):
        manager.stop_process(plan.processes[0], timeout_s=0.2)

    assert manager.active("worker-target") is False


def test_forced_motion_cleanup_releases_dead_owner_on_retry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    artifact = repository_root / "worker.py"
    artifact.write_text(
        "import time\nwhile True:\n    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    plan = _plan(
        repository_root,
        artifact,
        shutdown=ProcessShutdown(
            "file",
            target="driver_bridge.stop.json",
            schema="lingtu.sim.motion_stop.v2",
        ),
    )
    plan_path = _publish_plan(session_root, plan)
    manager = SimProcessManager(repository_root)
    manager.bind(plan, run_plan_path=plan_path, product_session_id=PRODUCT_SESSION_ID)
    manager.start("worker-target", timeout_s=1)
    runner = manager
    monkeypatch.setattr(
        SimProcessManager,
        "_graceful_stop_group",
        classmethod(lambda _cls, _owned: None),
    )

    with pytest.raises(ProcessFailed, match="required force stop"):
        runner.stop_plan(plan)

    assert manager.active("worker-target") is False
    retained = SimChildLedger(session_root).load()
    assert retained is not None
    assert [child.target for child in retained.children] == ["worker-target"]

    retry = runner.stop_plan(plan)
    assert retry.stop_evidence["worker"]["outcome"] == (
        "inactive_without_motion_evidence"
    )
    assert retry.stop_evidence["worker"]["motion_stop_confirmed"] is False
    assert SimChildLedger(session_root).load() is None
