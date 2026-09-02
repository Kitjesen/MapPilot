"""Simulation supervisor tests."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import pytest

import lingtu.sim.supervisor as runtime_module
from lingtu.run_plan import RunPlan
from lingtu.sim.daemon import _SimulationRequestHandler
from lingtu.sim.process import SimProcessManager
from lingtu.sim.rpc import (
    REQUEST_SCHEMA,
    SupervisorRequest,
    SupervisorResponse,
)
from lingtu.sim.supervisor import (
    SimulationSupervisorClient,
)
from lingtu.switch_contracts import ProcessFailed, ProcessReport
from runtime.graph import (
    ProcessArtifact,
    ProcessCommand,
    ProcessReadiness,
    ProcessShutdown,
    ProcessSpec,
)

PRODUCT_SESSION_ID = "1" * 32


def _generic_stop_evidence(
    *,
    process: str = "sim_runtime",
    target: str = "nav-runtime",
) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.process_stop.v1",
        "process": process,
        "target": target,
        "outcome": "graceful_exit",
        "graceful": True,
        "forced": False,
        "inactive": True,
    }


def _motion_stop_evidence(plan: RunPlan) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.motion_stop.v2",
        "product_session_id": PRODUCT_SESSION_ID,
        "product": plan.product,
        "process": "sim_runtime",
        "outcome": "zero_applied",
        "launch_id": "d" * 64,
        "bridge_boot_id": "b" * 32,
        "controller_boot_id": "c" * 32,
        "bridge_command_seq": 1,
        "applied_step_seq": 2,
        "command_kind": "deactivate_zero",
        "walk_x": 0.0,
        "walk_y": 0.0,
        "walk_z": 0.0,
        "terminal_ack": True,
    }


def _simulation_snapshot() -> dict[str, Any]:
    session = {
        "schema": "lingtu.sim.session.v1",
        "session_id": "test-session",
        "mujoco_version": "3.2.0",
        "seed": 0,
        "world": "test_world@1.0.0",
        "robots": ["test_robot@1.0.0"],
        "runtime": {
            "backend": "mujoco",
            "mode": "headless",
            "required_bindings": ["physics"],
        },
    }
    global_policy = {
        "owner": "world",
        "timestep_s": 0.002,
        "integrator": "rk4",
        "solver": "newton",
        "iterations": 100,
        "gravity_mps2": [0.0, 0.0, -9.81],
    }
    world_package = {
        "id": "test_world",
        "version": "1.0.0",
        "kind": "world",
        "manifest": "sim/packages/worlds/test_world/world.package.yaml",
    }
    robot_package = {
        "id": "test_robot",
        "version": "1.0.0",
        "kind": "robot",
        "manifest": "sim/packages/robots/test_robot/robot.package.yaml",
    }
    world = {
        "package": world_package,
        "mjcf": "sim/packages/worlds/test_world/world.xml",
    }
    robots = [
        {
            "instance_id": "robot_01",
            "namespace": "robot_01",
            "package": robot_package,
            "controller": None,
            "sensor_rig": None,
            "spawn": {
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "model": {
                "mjcf": "sim/packages/robots/test_robot/robot.xml",
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
        "session_source": "tests/sim/session.json",
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
            "global_policy": global_policy,
            "world": world,
            "robots": robots,
        },
    }


def _published_plan(
    tmp_path: Path,
    *,
    motion_shutdown: bool = False,
) -> tuple[RunPlan, Path]:
    artifact = ProcessArtifact("src/lingtu/run_plan.py")
    process = ProcessSpec(
        name="sim_runtime",
        manager="direct",
        target="nav-runtime",
        order=10,
        timeout_s=5,
        lifecycle="mode",
        command=ProcessCommand(
            argv=("python", artifact.path),
            cwd=".",
            env=(),
            artifact=artifact,
            readiness=ProcessReadiness("process"),
            shutdown=(
                ProcessShutdown(
                    "file",
                    "driver_bridge.stop.json",
                    "lingtu.sim.motion_stop.v2",
                )
                if motion_shutdown
                else None
            ),
        ),
        provides=("sim_runtime",),
    )
    plan = RunPlan.create(
        product="nav",
        env="sim",
        robot="doso/thunder_v4",
        process_control="subprocess",
        modules=(),
        processes=(process,),
        available_processes=(process,),
        stop_before_start=() if motion_shutdown else (process.target,),
        contracts=("lingtu.product.nav.v1",),
        critical_modules=(),
        route_contract=None,
        host_config={},
        lifecycle={"product": "nav"},
        simulation=_simulation_snapshot(),
        native_nav={
            "control_mode": "test",
            "global_planner": "octoplanner3d",
            "publish_cmd_vel": False,
            "check_obstacle": False,
            "use_traversability_cost": False,
            "allow_teleop_takeover": False,
            "teleop_local_planner": False,
        },
    )
    path = (tmp_path / f"plan-{PRODUCT_SESSION_ID}.json").resolve()
    plan.write(path)
    return plan, path


def _owned_process_plan(
    repository_root: Path,
    session_root: Path,
    *,
    motion_shutdown: bool = False,
) -> tuple[RunPlan, Path]:
    artifact_path = repository_root / "worker.py"
    if motion_shutdown:
        src_root = Path(__file__).resolve().parents[3] / "src"
        artifact_path.write_text(
            "import os\n"
            "import sys\n"
            "import time\n"
            "from pathlib import Path\n"
            f"sys.path.insert(0, {str(src_root)!r})\n"
            "from lingtu.sim.stop import publish_motion_stop_evidence\n"
            "root = Path(os.environ['LINGTU_SESSION_ROOT'])\n"
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
    else:
        artifact_path.write_text(
            "import os\n"
            "import signal\n"
            "import time\n"
            "from pathlib import Path\n"
            "root = Path(os.environ['LINGTU_SESSION_ROOT'])\n"
            "def shutdown(_signum, _frame):\n"
            "    raise SystemExit(0)\n"
            "signal.signal(signal.SIGTERM, shutdown)\n"
            "if hasattr(signal, 'SIGBREAK'):\n"
            "    signal.signal(signal.SIGBREAK, shutdown)\n"
            "(root / 'ready.json').write_text('ready', encoding='utf-8')\n"
            "try:\n"
            "    while True:\n"
            "        time.sleep(0.05)\n"
            "finally:\n"
            "    (root / 'stopped.txt').write_text('stopped', encoding='utf-8')\n",
            encoding="utf-8",
        )
    artifact = ProcessArtifact(
        "worker.py",
    )
    process = ProcessSpec(
        name="worker",
        manager="direct",
        target="worker-target",
        order=10,
        timeout_s=10,
        lifecycle="mode",
        command=ProcessCommand(
            argv=("python", artifact.path),
            cwd=".",
            env=(),
            artifact=artifact,
            readiness=ProcessReadiness("process"),
            shutdown=(
                ProcessShutdown(
                    "file",
                    "driver_bridge.stop.json",
                    "lingtu.sim.motion_stop.v2",
                )
                if motion_shutdown
                else None
            ),
        ),
        provides=("worker",),
    )
    plan = RunPlan.create(
        product="nav",
        env="sim",
        robot="doso/thunder_v4",
        process_control="subprocess",
        modules=(),
        processes=(process,),
        available_processes=(process,),
        stop_before_start=() if motion_shutdown else (process.target,),
        contracts=("lingtu.product.nav.v1",),
        critical_modules=(),
        route_contract=None,
        host_config={},
        lifecycle={"product": "nav"},
        simulation=_simulation_snapshot(),
        native_nav={
            "control_mode": "test",
            "global_planner": "octoplanner3d",
            "publish_cmd_vel": False,
            "check_obstacle": False,
            "use_traversability_cost": False,
            "allow_teleop_takeover": False,
            "teleop_local_planner": False,
        },
    )
    plan_path = plan.write(session_root / f"plan-{PRODUCT_SESSION_ID}.json")
    return plan, plan_path


def _request(
    plan: RunPlan,
    path: Path,
    *,
    action: str = "apply",
    product_session_id: str = PRODUCT_SESSION_ID,
) -> SupervisorRequest:
    return SupervisorRequest(
        schema_version=REQUEST_SCHEMA,
        action=action,
        run_plan_path=str(path),
        product_session_id=product_session_id,
    )


class FakeOwner:
    def __init__(self, reports: dict[str, ProcessReport]) -> None:
        self.reports = reports
        self.events: list[tuple[str, str]] = []

    def bind(
        self,
        plan: RunPlan,
        *,
        run_plan_path: str,
        product_session_id: str,
    ) -> None:
        self.events.append(("bind", run_plan_path))
        self.events.append(("product_session_id", product_session_id))

    def apply(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        self.events.append(("apply", plan.product))
        return self.reports["apply"]

    def assert_bound(self, _plan: RunPlan) -> None:
        pass

    def quiesce(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        self.events.append(("quiesce", plan.product))
        return self.reports["quiesce"]

    def stop_plan(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        self.events.append(("stop", plan.product))
        return self.reports["stop"]


def _wait_for_file(path: Path, *, timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not path.exists() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert path.exists()


def test_daemon_handler_owns_apply_then_stop(tmp_path: Path) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    plan, plan_path = _owned_process_plan(repository_root, session_root)
    handler = _SimulationRequestHandler(SimProcessManager(repository_root))

    try:
        applied = handler.handle(_request(plan, plan_path))
        _wait_for_file(session_root / "ready.json")
        stopped = handler.handle(_request(plan, plan_path, action="stop"))
    finally:
        handler.close()

    assert applied.success is True
    assert applied.result is not None
    assert applied.result["status"] == "active"
    assert stopped.success is True
    assert stopped.result is not None
    assert stopped.result["status"] == "stopped"


def test_daemon_handler_close_reclaims_active_process(tmp_path: Path) -> None:
    repository_root = tmp_path / "repository"
    session_root = tmp_path / "session"
    repository_root.mkdir()
    session_root.mkdir()
    plan, plan_path = _owned_process_plan(repository_root, session_root)

    handler = _SimulationRequestHandler(SimProcessManager(repository_root))
    try:
        applied = handler.handle(_request(plan, plan_path))
        assert applied.success is True
        _wait_for_file(session_root / "ready.json")
    finally:
        handler.close()

    assert (session_root / "stopped.txt").read_text(encoding="utf-8") == "stopped"


def test_supervisor_runtime_never_reloads_runtime_graph() -> None:
    source = Path(runtime_module.__file__).read_text(encoding="utf-8")

    assert "load_runtime_graph" not in source
    assert "runtime.graph" not in source
    assert ".assert_compatible(" not in source


def test_handler_binds_exact_plan_before_apply_and_returns_strict_report(
    tmp_path: Path,
) -> None:
    plan, path = _published_plan(tmp_path)
    report = ProcessReport(
        product=plan.product,
        env="sim",
        action="apply",
        ok=True,
        status="active",
        planned=["nav-runtime"],
        started=["nav-runtime"],
        ready={
            "sim_runtime": {
                "kind": "process",
                "target": "nav-runtime",
                "active": True,
            }
        },
    )
    owner = FakeOwner({"apply": report})
    handler = _SimulationRequestHandler(owner)

    response = handler.handle(_request(plan, path))

    assert response.success is True
    assert response.result == report.as_dict()
    assert owner.events == [
        ("bind", str(path)),
        ("product_session_id", "11111111111111111111111111111111"),
        ("apply", plan.product),
    ]


def test_process_report_accepts_child_that_exited_before_rollback(
    tmp_path: Path,
) -> None:
    plan, _path = _published_plan(tmp_path)
    payload = ProcessReport(
        product=plan.product,
        env=plan.env,
        action="apply",
        ok=False,
        status="failed",
        planned=["nav-runtime"],
        started=["nav-runtime"],
        error="direct process exited before readiness: exit_code=7",
    ).as_dict()

    report = runtime_module._process_report_from_payload(
        payload,
        plan=plan,
        action="apply",
    )

    assert report.started == ["nav-runtime"]
    assert report.rolled_back == []


def test_process_report_preserves_manager_readiness_evidence(tmp_path: Path) -> None:
    plan, _path = _published_plan(tmp_path)
    ready = {
        "sim_runtime": {
            "kind": "process",
            "target": "nav-runtime",
            "active": True,
            "details": {"producer_owned": True},
        }
    }
    payload = ProcessReport(
        product=plan.product,
        env=plan.env,
        action="apply",
        ok=True,
        status="active",
        planned=["nav-runtime"],
        started=["nav-runtime"],
        ready=ready,
    ).as_dict()

    decoded = runtime_module._process_report_from_payload(
        payload,
        plan=plan,
        action="apply",
    )

    assert decoded.ready == ready


def test_process_report_preserves_motion_stop_evidence(
    tmp_path: Path,
) -> None:
    plan, _path = _published_plan(tmp_path, motion_shutdown=True)
    report = ProcessReport(
        product=plan.product,
        env=plan.env,
        action="stop",
        ok=True,
        status="stopped",
        planned=["nav-runtime"],
        stopped=["nav-runtime"],
        stop_evidence={"sim_runtime": _motion_stop_evidence(plan)},
    )

    decoded = runtime_module._process_report_from_payload(
        report.as_dict(),
        plan=plan,
        action="stop",
    )

    assert decoded == report


def test_handler_reuses_one_owner_for_quiesce_and_stop(tmp_path: Path) -> None:
    plan, path = _published_plan(tmp_path)
    reports = {
        action: ProcessReport(
            product=plan.product,
            env="sim",
            action=action,
            ok=True,
            status="stopped",
            planned=["nav-runtime"],
            stopped=["nav-runtime"],
            stop_evidence={"sim_runtime": _generic_stop_evidence()},
        )
        for action in ("quiesce", "stop")
    }
    owner = FakeOwner(reports)
    handler = _SimulationRequestHandler(owner)

    quiesce = handler.handle(_request(plan, path, action="quiesce"))
    stop = handler.handle(_request(plan, path, action="stop"))

    assert quiesce.success is True
    assert stop.success is True
    assert owner.events == [
        ("bind", str(path)),
        ("product_session_id", "11111111111111111111111111111111"),
        ("quiesce", plan.product),
        ("stop", plan.product),
    ]


def test_handler_returns_runner_failure_message(tmp_path: Path) -> None:
    plan, path = _published_plan(tmp_path)

    class RaisingOwner(FakeOwner):
        def apply(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
            raise RuntimeError("private runner detail")

    handler = _SimulationRequestHandler(RaisingOwner({}))

    response = handler.handle(_request(plan, path))

    assert response.success is False
    assert response.error == {
        "code": "operation_failed",
        "message": "private runner detail",
    }


def test_handler_preserves_strict_partial_readiness_from_process_failure(
    tmp_path: Path,
) -> None:
    plan, path = _published_plan(tmp_path)
    failed = ProcessReport(
        product=plan.product,
        env="sim",
        action="apply",
        ok=False,
        status="failed",
        planned=["nav-runtime"],
        started=["nav-runtime"],
        ready={},
        rolled_back=["nav-runtime"],
        stop_evidence={"sim_runtime": _generic_stop_evidence()},
        error="direct process readiness timed out: sim_runtime",
    )

    class FailingOwner(FakeOwner):
        def apply(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
            raise ProcessFailed(failed)

    handler = _SimulationRequestHandler(FailingOwner({}))

    response = handler.handle(_request(plan, path))

    assert response.success is True
    assert response.result == failed.as_dict()


def test_handler_retains_monitored_product_failure_truth(tmp_path: Path) -> None:
    plan, path = _published_plan(tmp_path)
    active = ProcessReport(
        product=plan.product,
        env=plan.env,
        action="apply",
        ok=True,
        status="active",
    )
    failed = ProcessReport(
        product=plan.product,
        env=plan.env,
        action="apply",
        status="failed",
        error="critical simulation process exited: driver_bridge",
    )

    class MonitoringOwner(FakeOwner):
        def monitor(self, checked: RunPlan) -> None:
            self.events.append(("monitor", checked.product))
            raise ProcessFailed(failed)

    owner = MonitoringOwner({"apply": active})
    handler = _SimulationRequestHandler(owner)

    assert handler.handle(_request(plan, path)).result == active.as_dict()
    assert handler.handle(_request(plan, path)).result == active.as_dict()
    assert owner.events.count(("apply", plan.product)) == 1
    assert handler.poll() == failed
    assert handler.handle(_request(plan, path, action="status")).result == failed.as_dict()
    assert owner.events.count(("apply", plan.product)) == 1


def test_handler_limits_operation_failure_message(tmp_path: Path) -> None:
    plan, path = _published_plan(tmp_path)

    class RaisingOwner(FakeOwner):
        def apply(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
            raise RuntimeError("x" * 600)

    response = _SimulationRequestHandler(RaisingOwner({})).handle(_request(plan, path))

    assert response.error == {"code": "operation_failed", "message": "x" * 512}


def test_handler_names_exception_without_a_message(tmp_path: Path) -> None:
    plan, path = _published_plan(tmp_path)

    class RaisingOwner(FakeOwner):
        def apply(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
            raise RuntimeError

    response = _SimulationRequestHandler(RaisingOwner({})).handle(_request(plan, path))

    assert response.error == {"code": "operation_failed", "message": "RuntimeError"}


def test_client_raises_supervisor_error_message(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _plan, path = _published_plan(tmp_path)

    def fake_round_trip(
        _session_root: Path,
        request: SupervisorRequest,
        _timeout_s: float,
    ) -> SupervisorResponse:
        return SupervisorResponse.failed(
            code="operation_failed",
            message="sim runtime executable is missing",
        )

    monkeypatch.setattr(runtime_module, "round_trip", fake_round_trip)

    with pytest.raises(
        runtime_module.SimulationSupervisorError,
        match="sim runtime executable is missing",
    ):
        SimulationSupervisorClient(tmp_path).apply(
            path,
            product_session_id="11111111111111111111111111111111",
            timeout_s=1,
        )


def test_client_apply_sends_exact_published_plan_and_decodes_report(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan, path = _published_plan(tmp_path)
    captured: list[tuple[Path, SupervisorRequest, float]] = []
    expected = ProcessReport(
        product=plan.product,
        env="sim",
        action="apply",
        ok=True,
        status="active",
        planned=["sim_runtime"],
        started=["sim_runtime"],
        ready={
            "sim_runtime": {
                "kind": "process",
                "target": "nav-runtime",
                "active": True,
            }
        },
    )

    def fake_round_trip(
        session_root: Path,
        request: SupervisorRequest,
        timeout_s: float,
    ) -> SupervisorResponse:
        captured.append((session_root, request, timeout_s))
        return SupervisorResponse.ok(expected.as_dict())

    monkeypatch.setattr(runtime_module, "round_trip", fake_round_trip)

    report = SimulationSupervisorClient(tmp_path).apply(
        path,
        product_session_id="11111111111111111111111111111111",
        timeout_s=3.5,
    )

    assert report == expected
    assert len(captured) == 1
    root, request, timeout_s = captured[0]
    assert root == tmp_path
    assert request.run_plan_path == str(path)
    assert request.product_session_id == "11111111111111111111111111111111"
    assert request.action == "apply"
    assert timeout_s == 3.5


def test_client_default_apply_timeout_covers_sequential_plan_deadlines(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan, path = _published_plan(tmp_path)
    observed_timeouts: list[float] = []
    expected = ProcessReport(
        product=plan.product,
        env="sim",
        action="apply",
        ok=True,
        status="active",
        ready={
            "sim_runtime": {
                "kind": "process",
                "target": "nav-runtime",
                "active": True,
            }
        },
    )
    def fake_round_trip(
        _session_root: Path,
        request: SupervisorRequest,
        timeout_s: float,
    ) -> SupervisorResponse:
        observed_timeouts.append(timeout_s)
        return SupervisorResponse.ok(expected.as_dict())

    monkeypatch.setattr(runtime_module, "round_trip", fake_round_trip)

    SimulationSupervisorClient(tmp_path).apply(
        path,
        product_session_id="11111111111111111111111111111111",
    )

    # One 5-second process may consume 5 seconds stopping a conflict, two
    # 5-second start/readiness intervals, and another 5-second rollback stop.
    # The RPC deadline also carries a fixed 5-second transport margin.
    assert observed_timeouts == [25.0]


def test_client_quiesce_uses_the_same_published_plan_contract(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan, path = _published_plan(tmp_path)
    actions: list[str] = []
    expected = ProcessReport(
        product=plan.product,
        env="sim",
        action="quiesce",
        ok=True,
        status="stopped",
        planned=["nav-runtime"],
        stopped=["nav-runtime"],
        stop_evidence={"sim_runtime": _generic_stop_evidence()},
    )
    def fake_round_trip(
        _session_root: Path,
        request: SupervisorRequest,
        _timeout_s: float,
    ) -> SupervisorResponse:
        actions.append(request.action)
        return SupervisorResponse.ok(expected.as_dict())

    monkeypatch.setattr(runtime_module, "round_trip", fake_round_trip)

    report = SimulationSupervisorClient(tmp_path).quiesce(path, product_session_id="11111111111111111111111111111111")

    assert report == expected
    assert actions == ["quiesce"]


def test_client_stop_uses_the_same_published_plan_contract(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan, path = _published_plan(tmp_path)
    actions: list[str] = []
    expected = ProcessReport(
        product=plan.product,
        env="sim",
        action="stop",
        ok=True,
        status="stopped",
        planned=["nav-runtime"],
        stopped=["nav-runtime"],
        stop_evidence={"sim_runtime": _generic_stop_evidence()},
    )
    def fake_round_trip(
        _session_root: Path,
        request: SupervisorRequest,
        _timeout_s: float,
    ) -> SupervisorResponse:
        actions.append(request.action)
        return SupervisorResponse.ok(expected.as_dict())

    monkeypatch.setattr(runtime_module, "round_trip", fake_round_trip)

    report = SimulationSupervisorClient(tmp_path).stop(path, product_session_id="11111111111111111111111111111111")

    assert report == expected
    assert actions == ["stop"]
